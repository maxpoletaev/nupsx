const std = @import("std");
const mem = @import("mem.zig");

const CPU = @import("cpu.zig").CPU;
const BIOS = @import("bios.zig").BIOS;
const Disasm = @import("disasm.zig").Disasm;
const GPU = @import("gpu.zig").GPU;
const UI = @import("ui.zig").UI;
const exe = @import("exe.zig");

const Args = struct {
    bios_path: []const u8,
    exe_path: ?[]const u8,
    step_execute: bool = false,
    no_ui: bool = false,
    disasm: bool = false,
    breakpoint: u32 = 0,
    iter: *std.process.ArgIterator,

    fn parse(allocator: std.mem.Allocator) !Args {
        var iter = try std.process.argsWithAllocator(allocator);
        var args = std.mem.zeroInit(Args, .{ .iter = &iter });

        while (iter.next()) |arg| {
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
            if (std.mem.eql(u8, arg, "--bios")) {
                args.bios_path = iter.next() orelse return error.InvalidArgument;
            }
            if (std.mem.eql(u8, arg, "--exe")) {
                args.exe_path = iter.next() orelse return error.InvalidArgument;
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
    defer if (gpa.deinit() == .leak) {
        std.log.warn("leak detected", .{});
    };

    const allocator = gpa.allocator();

    var args = try Args.parse(allocator);
    defer args.deinit();

    const bios = try BIOS.loadFromFile(allocator, args.bios_path);
    defer bios.deinit();

    const gpu = try GPU.init(allocator);
    defer gpu.deinit();

    const bus = try mem.Bus.init(allocator, bios, gpu);
    defer bus.deinit();

    const cpu = try CPU.init(allocator, bus);
    defer cpu.deinit();

    if (args.breakpoint != 0) {
        cpu.setBreakpoint(args.breakpoint);
        std.log.info("breakpoint is set to {x}", .{args.breakpoint});
    }

    const stdin = std.io.getStdIn();
    var stdin_buf: [1]u8 = undefined;

    const disasm = try Disasm.init(allocator);
    defer disasm.deinit();

    if (args.step_execute) {
        cpu.stall = true;
    }

    if (args.exe_path) |path| {
        while (cpu.pc != 0x80030000) {
            cpu.execute();
        }

        std.log.info("loading exe file: {s}", .{path});
        try exe.loadExe(allocator, path, cpu, bus);
    }

    if (args.no_ui) {
        while (true) {
            cpu.execute();

            if (args.disasm) {
                const decoded = disasm.disassemble(cpu.instr);
                std.log.debug("{x}\t {x}\t {s}", .{ cpu.instr_addr, cpu.instr.code, decoded });
            }

            if (cpu.stall) {
                _ = try stdin.read(&stdin_buf);
                args.disasm = true;
                cpu.step();
            }
        }

        return;
    }

    const ui = try UI.init(allocator, cpu, bus, disasm);
    defer ui.deinit();

    while (!ui.shouldClose()) {
        cpu.execute();
        ui.update();

        if (captureTtyOutput(cpu)) |ch| {
            ui.tty_view.writeChar(ch) catch unreachable;
        }
    }
}

fn captureTtyOutput(cpu: *const CPU) ?u8 {
    const pc = cpu.pc & 0x1fffffff;
    if ((pc == 0xa0 and cpu.gpr[9] == 0x3c) or (pc == 0xb0 and cpu.gpr[9] == 0x3d)) {
        return @as(u8, @truncate(cpu.gpr[4]));
    }
    return null;
}
