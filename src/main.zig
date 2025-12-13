const std = @import("std");
const mem = @import("mem.zig");

const Args = @import("Args.zig");
const CPU = @import("cpu.zig").CPU;
const BIOS = @import("bios.zig").BIOS;
const Disasm = @import("Disasm.zig");
const GPU = @import("gpu.zig").GPU;
const DMA = @import("dma.zig").DMA;
const UI = @import("ui.zig").UI;
const exe = @import("exe.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer if (gpa.deinit() == .leak) {
        std.log.warn("leak detected", .{});
    };

    const allocator = gpa.allocator();

    var args = try Args.parse(allocator);
    defer args.deinit();

    const bus = try mem.Bus.init(allocator);
    defer bus.deinit();

    if (args.bios_path.len == 0) {
        std.log.err("--bios argument is required", .{});
        std.process.exit(1);
    }
    const bios = try BIOS.loadFromFile(allocator, args.bios_path);
    defer bios.deinit();

    const gpu = try GPU.init(allocator);
    defer gpu.deinit();

    const cpu = try CPU.init(allocator, bus);
    defer cpu.deinit();

    const dma = try DMA.init(allocator, bus);
    defer dma.deinit();

    bus.initDevices(.{
        .gpu = gpu,
        .dma = dma,
        .bios = bios,
    });

    if (args.breakpoint != 0) {
        cpu.setBreakpoint(args.breakpoint);
        std.log.info("breakpoint is set to {x}", .{args.breakpoint});
    }

    const stdin = std.fs.File.stdin();
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

    while (ui.is_running) {
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
