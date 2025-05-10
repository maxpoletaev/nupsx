const std = @import("std");
const mem = @import("mem.zig");

const CPU = @import("cpu.zig").CPU;
const BIOS = @import("bios.zig").BIOS;
const Disasm = @import("disasm.zig").Disasm;
const UI = @import("ui.zig").UI;

const Args = struct {
    bios_path: []const u8,
    step_execute: bool = false,
    no_ui: bool = false,
    disasm: bool = false,
    breakpoint: u32 = 0,
    iter: *std.process.ArgIterator,

    fn parse(allocator: std.mem.Allocator) !Args {
        var iter = try std.process.argsWithAllocator(allocator);
        var args = std.mem.zeroInit(Args, .{ .iter = &iter });

        while (iter.next()) |arg| {
            if (std.mem.eql(u8, arg, "--bios")) {
                const bios_path = iter.next() orelse return error.InvalidArgument;
                args.bios_path = bios_path;
            }
            if (std.mem.eql(u8, arg, "--step")) {
                args.step_execute = true;
            }
            if (std.mem.eql(u8, arg, "--no-ui")) {
                args.no_ui = true;
            }
            if (std.mem.eql(u8, arg, "--disasm")) {
                args.disasm = true;
            }
            if (std.mem.eql(u8, arg, "--breakpoint")) {
                const addr_str = iter.next() orelse return error.InvalidArgument;
                args.breakpoint = try std.fmt.parseUnsigned(u32, addr_str, 16);
            }
        }

        return args;
    }

    pub fn deinit(self: *Args) void {
        self.iter.deinit();
    }
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    defer if (gpa.deinit() == .leak) {
        std.log.warn("leak detected", .{});
    };

    var args = try Args.parse(allocator);
    defer args.deinit();

    const bios = BIOS.loadFromFile(allocator, args.bios_path) catch |err| {
        std.log.err("failed to load BIOS: {}", .{err});
        std.process.exit(1);
    };
    defer bios.deinit();

    const bus = try mem.Bus.init(allocator, bios);
    defer bus.deinit();

    const cpu = try CPU.init(allocator, bus);
    defer cpu.deinit();

    if (args.breakpoint != 0) {
        cpu.setBreakpoint(args.breakpoint);
        std.log.info("breakpoint is set to {x}", .{args.breakpoint});
    }

    const stdin = std.io.getStdIn();
    var buf: [1]u8 = undefined;

    const disasm = try Disasm.init(allocator);
    defer disasm.deinit();

    if (args.no_ui) {
        while (true) {
            cpu.execute();

            if (args.disasm) {
                const decoded = disasm.disassemble(cpu.instr);
                std.log.debug("{x}\t {x}\t {s}", .{ cpu.instr_addr, cpu.instr.code, decoded });
            }

            if (cpu.stall) {
                _ = try stdin.read(&buf);
                args.disasm = true;
                cpu.step();
            }
        }

        return;
    }

    if (args.step_execute) {
        cpu.stall = true;
    }

    const ui = try UI.init(allocator, cpu, bus, disasm);
    defer ui.deinit();

    while (!ui.shouldClose()) {
        cpu.execute();
        ui.update();
    }
}
