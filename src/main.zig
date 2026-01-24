const std = @import("std");
const zaudio = @import("zaudio");

const args_mod = @import("args.zig");
const mem_mod = @import("mem.zig");
const timer_mod = @import("timer.zig");
const disasm_mod = @import("disasm.zig");
const gpu_mod = @import("gpu.zig");
const cdrom_mod = @import("cdrom.zig");
const spu_mod = @import("spu.zig");
const joy_mod = @import("joy.zig");

const CPU = @import("cpu.zig").CPU;
const DMA = @import("dma.zig").DMA;
const MDEC = @import("mdec.zig").MDEC;
const DebugUI = @import("debug_ui/DebugUI.zig");
const Args = @import("args.zig").Args;
const UI = @import("ui.zig").UI;
const exe = @import("exe.zig");

const Bus = mem_mod.Bus;
const BIOS = mem_mod.BIOS;
const RAM = mem_mod.RAM;
const Scratchpad = mem_mod.Scratchpad;
const GPU = gpu_mod.GPU;
const Disasm = disasm_mod.Disasm;
const GPUEvent = gpu_mod.GPUEvent;
const Timers = timer_mod.Timers;
const CDROM = cdrom_mod.CDROM;
const Disc = cdrom_mod.Disc;
const SPU = spu_mod.SPU;
const Joypad = joy_mod.Joypad;

pub const std_options = std.Options{
    .log_level = .info,
    .log_scope_levels = &[_]std.log.ScopeLevel{
        // .{ .scope = .spu, .level = .debug },
        // .{ .scope = .cdrom, .level = .debug },
        // .{ .scope = .cue, .level = .debug },
        // .{ .scope = .mem, .level = .debug },
        // .{ .scope = .timer, .level = .debug },
        // .{ .scope = .gte, .level = .debug },
        // .{ .scope = .gpu, .level = .debug },
        // .{ .scope = .dma, .level = .debug },
        // .{ .scope = .joy, .level = .debug },
    },
};

const logo_text =
    \\
    \\              ____  ______  __
    \\  _ __  _   _|  _ \/ ___\ \/ /
    \\ | '_ \| | | | |_) \___ \\  / 
    \\ | | | | |_| |  __/ ___) /  \ 
    \\ |_| |_|\__,_|_|   |____/_/\_\   PlayStation Emulator
    \\
    \\
;

fn printLogo() void {
    var buf: [1024]u8 = undefined;
    var writer = std.fs.File.stdout().writer(&buf);
    _ = writer.interface.write(logo_text) catch 0;
    writer.interface.flush() catch {};
}

const Audio = struct {
    allocator: std.mem.Allocator,
    device: *zaudio.Device,
    bus: *Bus,

    pub fn init(allocator: std.mem.Allocator, bus: *Bus) *@This() {
        zaudio.init(allocator);

        const self = allocator.create(Audio) catch unreachable;

        var config = zaudio.Device.Config.init(.playback);
        config.playback.format = .signed16;
        config.playback.channels = 2;
        config.data_callback = callback;
        config.sample_rate = 44100;
        config.user_data = self;

        var device = zaudio.Device.create(null, config) catch |err| {
            std.debug.panic("failed to create audio device: {}", .{err});
        };

        device.start() catch |err| {
            std.debug.panic("failed to start audio device: {}", .{err});
        };

        self.* = .{
            .allocator = allocator,
            .device = device,
            .bus = bus,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.device.stop() catch {};
        self.device.destroy();
        self.allocator.destroy(self);
        zaudio.deinit();
    }

    fn callback(
        device: *zaudio.Device,
        output: ?*anyopaque,
        input: ?*const anyopaque,
        frame_count: u32,
    ) callconv(.c) void {
        _ = input;

        const self: *@This() = @ptrCast(@alignCast(device.getUserData()));
        const buf: [*]i16 = @ptrCast(@alignCast(output.?));

        for (0..frame_count) |i| {
            const sample = self.bus.dev.spu.consumeAudioSample();
            const cd_sample = self.bus.dev.cdrom.consumeAudioSample();
            buf[i * 2 + 0] = sample[0] +| cd_sample[0]; // left
            buf[i * 2 + 1] = sample[1] +| cd_sample[1]; // right
        }
    }
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer if (gpa.deinit() == .leak) {
        std.log.warn("leak detected", .{});
    };

    const allocator = gpa.allocator();

    var args = Args.parse(allocator) catch |err| {
        switch (err) {
            error.InvalidArgument => {
                Args.printHelp();
                std.process.exit(1);
            },
            else => return err,
        }
    };
    defer args.deinit();

    printLogo();

    const bios = try BIOS.loadFromFile(allocator, args.bios_path);
    defer bios.deinit();

    const bus = try Bus.init(allocator);
    defer bus.deinit();

    const ram = try RAM.init(allocator);
    defer ram.deinit(allocator);

    const scratchpad = try Scratchpad.init(allocator);
    defer scratchpad.deinit(allocator);

    const gpu = try GPU.init(allocator);
    defer gpu.deinit();

    const cpu = try CPU.init(allocator, bus);
    defer cpu.deinit();

    const dma = try DMA.init(allocator, bus);
    defer dma.deinit();

    const mdec = MDEC.init(allocator);
    defer mdec.deinit();

    const timers = Timers.init(allocator);
    defer timers.deinit();

    const spu = SPU.init(allocator);
    defer spu.deinit();

    const joy = Joypad.init(allocator);
    defer joy.deinit();

    var disc: ?Disc = null;
    defer if (disc) |*d| d.deinit();

    const cdrom = CDROM.init(allocator);
    defer cdrom.deinit();

    if (args.cd_image_path) |path| {
        if (std.mem.endsWith(u8, path, ".cue")) {
            disc = try Disc.loadCue(allocator, path);
        } else {
            std.log.err("unsupported cd image format: {s}", .{path});
            std.process.exit(1);
        }
        cdrom.insertDisc(disc.?);
    }

    bus.connect(.{
        .cpu = cpu,
        .bios = bios,
        .ram = ram,
        .gpu = gpu,
        .dma = dma,
        .mdec = mdec,
        .spu = spu,
        .joy = joy,
        .cdrom = cdrom,
        .timers = timers,
        .scratchpad = scratchpad,
    });

    const audio = Audio.init(allocator, bus);
    defer audio.deinit();

    // const stdin = std.fs.File.stdin();
    // var stdin_buf: [1]u8 = undefined;

    if (args.exe_path) |path| {
        while (cpu.pc != 0x80030000) {
            cpu.tickOnce();
        }
        std.log.info("loading exe file: {s}", .{path});
        try exe.loadExe(allocator, path, cpu, bus);
    }

    if (args.debug) {
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

                if (bus.breakpoint) {
                    std.log.info("breakpoint hit at PC={x}", .{cpu.pc});
                    debug_ui.cpu_view.paused = true;
                    bus.breakpoint = false;
                }

                // if (cpu.next_pc == 0x8004e740) {
                //     debug_ui.cpu_view.paused = true;
                // }

                if (gpu.consumeFrameReady()) {
                    debug_ui.update();
                }
            }
        }
    } else {
        const ui = try UI.init(allocator, gpu, joy);
        defer ui.deinit();

        var buf: [1024]u8 = undefined;
        var stdout = std.fs.File.stdout().writer(&buf);

        cpu.tty = &stdout.interface;

        if (args.uncapped) {
            std.log.warn("running in uncapped mode", .{});
            ui.uncapped = true;
        }

        while (true) {
            bus.tick();

            if (gpu.consumeFrameReady()) {
                if (!ui.is_running) {
                    break;
                }
                ui.update();
            }
        }
    }
}
