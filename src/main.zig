const std = @import("std");
const mem = @import("mem.zig");

const timer_mod = @import("timer.zig");
const disasm_mod = @import("disasm.zig");
const gpu_mod = @import("gpu.zig");
const cdrom_mod = @import("cdrom.zig");

const Args = @import("args.zig").Args;
const CPU = @import("cpu.zig").CPU;
const BIOS = @import("bios.zig").BIOS;
const DMA = @import("dma.zig").DMA;
const DebugUI = @import("debug_ui/DebugUI.zig");
const UI = @import("ui.zig").UI;
const exe = @import("exe.zig");
const GPU = gpu_mod.GPU;
const GPUEvent = gpu_mod.GPUEvent;
const Disasm = disasm_mod.Disasm;
const Timers = timer_mod.Timers;
const CDROM = cdrom_mod.CDROM;

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

    const timers = Timers.init(allocator);
    defer timers.deinit();

    const cdrom = CDROM.init(allocator);
    defer cdrom.deinit();

    bus.connect(.{
        .cpu = cpu,
        .bios = bios,
        .ram = ram,
        .gpu = gpu,
        .dma = dma,
        .cdrom = cdrom,
        .timers = timers,
        .scratchpad = scratchpad,
    });

    // const stdin = std.fs.File.stdin();
    // var stdin_buf: [1]u8 = undefined;

    if (args.exe_path) |path| {
        while (cpu.pc != 0x80030000) {
            bus.tick();
        }

        std.log.info("loading exe file: {s}", .{path});
        try exe.loadExe(allocator, path, cpu, bus);
    }

    if (args.no_ui) {
        const disasm = try Disasm.init(allocator);
        defer disasm.deinit();

        var tty_buf = try std.array_list.Aligned(u8, null).initCapacity(allocator, 1024);
        defer tty_buf.deinit(allocator);

        var stdout = std.fs.File.stdout();

        while (true) {
            bus.tick();

            if (captureTtyOutput(cpu)) |ch| {
                try tty_buf.append(allocator, ch);
                if (ch == '\n' or tty_buf.items.len >= 1024) {
                    try stdout.writeAll(tty_buf.items);
                    tty_buf.clearRetainingCapacity();
                }
            }

            if (args.disasm) {
                const decoded = disasm.disassemble(cpu.instr);
                std.log.debug("{x}\t {x}\t {s}", .{ cpu.instr_addr, cpu.instr.code, decoded });
            }
        }
    } else if (args.debug_ui) {
        const debug_ui = try DebugUI.init(allocator, cpu, bus);
        defer debug_ui.deinit();

        while (debug_ui.is_running) {
            if (debug_ui.cpu_view.paused and !debug_ui.cpu_view.step_requested) {
                debug_ui.updatePaused();
            } else {
                if (debug_ui.cpu_view.step_requested) {
                    debug_ui.cpu_view.step_requested = false;
                    debug_ui.cpu_view.paused = true;
                }

                bus.tick();

                if (captureTtyOutput(cpu)) |ch| {
                    try debug_ui.tty_view.writeChar(ch);
                }

                if (gpu.consumeFrameReady()) {
                    debug_ui.update();
                }
            }
        }
    } else {
        const display_ui = try UI.init(allocator, gpu);
        defer display_ui.deinit();

        var tty_buf = try std.array_list.Aligned(u8, null).initCapacity(allocator, 1024);
        defer tty_buf.deinit(allocator);

        var stdout = std.fs.File.stdout();

        while (true) {
            bus.tick();

            if (captureTtyOutput(cpu)) |ch| {
                try tty_buf.append(allocator, ch);
                if (ch == '\n' or tty_buf.items.len >= 1024) {
                    try stdout.writeAll(tty_buf.items);
                    tty_buf.clearRetainingCapacity();
                }
            }

            if (gpu.consumeFrameReady()) {
                if (!display_ui.is_running) {
                    break;
                }
                display_ui.update();
            }
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
