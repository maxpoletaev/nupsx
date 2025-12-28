const std = @import("std");
const mem = @import("mem.zig");

const timer_mod = @import("timer.zig");
const disasm_mod = @import("disasm.zig");

const Args = @import("args.zig").Args;
const CPU = @import("cpu.zig").CPU;
const BIOS = @import("bios.zig").BIOS;
const GPU = @import("gpu.zig").GPU;
const DMA = @import("dma.zig").DMA;
const DebugUI = @import("debug_ui/DebugUI.zig");
const UI = @import("ui.zig").UI;
const exe = @import("exe.zig");
const Disasm = disasm_mod.Disasm;
const Timers = timer_mod.Timers;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer if (gpa.deinit() == .leak) {
        std.log.warn("leak detected", .{});
    };

    const allocator = gpa.allocator();

    var args = Args.parse(allocator) catch |err| {
        switch (err) {
            error.InvalidArgument => std.process.exit(1),
            else => return err,
        }
    };
    defer args.deinit();

    const bus = try mem.Bus.init(allocator);
    defer bus.deinit();

    const ram = try mem.RAM.init(allocator);
    defer ram.deinit(allocator);

    const scratchpad = try mem.Scratchpad.init(allocator);
    defer scratchpad.deinit(allocator);

    const bios = try BIOS.loadFromFile(allocator, args.bios_path);
    defer bios.deinit();

    const gpu = try GPU.init(allocator);
    defer gpu.deinit();

    const cpu = try CPU.init(allocator, bus);
    defer cpu.deinit();

    const dma = try DMA.init(allocator, bus);
    defer dma.deinit();

    const timers = Timers.init(allocator, bus);
    defer timers.deinit();

    bus.initDevices(.{
        .cpu = cpu,
        .bios = bios,
        .ram = ram,
        .scratchpad = scratchpad,
        .gpu = gpu,
        .dma = dma,
        .timers = timers,
    });

    if (args.breakpoint != 0) {
        cpu.setBreakpoint(args.breakpoint);
        std.log.info("breakpoint is set to {x}", .{args.breakpoint});
    }

    const stdin = std.fs.File.stdin();
    var stdin_buf: [1]u8 = undefined;

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
        const disasm = try Disasm.init(allocator);
        defer disasm.deinit();

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
    } else if (args.debug_ui) {
        const debug_ui = try DebugUI.init(allocator, cpu, bus);
        defer debug_ui.deinit();

        while (debug_ui.is_running) {
            cpu.execute();
            bus.tickSystemClock();

            if (bus.debug_pause or gpu.debug_pause) {
                cpu.stall = true;
            }

            debug_ui.update();
            if (captureTtyOutput(cpu)) |ch| {
                try debug_ui.tty_view.writeChar(ch);
            }
        }
    } else {
        const display_ui = try UI.init(allocator, gpu);
        defer display_ui.deinit();

        while (display_ui.is_running) {
            cpu.execute();
            display_ui.update();
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
