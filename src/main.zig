const std = @import("std");
const zaudio = @import("zaudio");

const args_mod = @import("args.zig");
const mem_mod = @import("mem.zig");
const timer_mod = @import("timer.zig");
const disasm_mod = @import("disasm.zig");
const gpu_mod = @import("gpu.zig");
const cdrom_mod = @import("cdrom.zig");
const spu_mod = @import("spu.zig");
const sio0_mod = @import("sio0.zig");
const sio1_mod = @import("sio1.zig");
const memcard_mod = @import("memcard.zig");

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
const SIO0 = sio0_mod.SIO0;
const SIO1 = sio1_mod.SIO1;
const MemoryCard = memcard_mod.MemoryCard;

const memcard_default_path = "memcard.mcd";

pub fn logFn(
    comptime message_level: std.log.Level,
    comptime scope: @EnumLiteral(),
    comptime format: []const u8,
    args: anytype,
) void {
    const level_str = switch (message_level) {
        .err => "ERROR",
        .warn => "WARN",
        .info => "INFO",
        .debug => "DEBUG",
    };

    var buf: [4096]u8 = undefined;
    const stderr = std.debug.lockStderr(&buf);
    defer std.debug.unlockStderr();

    const prefix = std.fmt.comptimePrint("{s}({s}): ", .{ level_str, @tagName(scope) });
    stderr.file_writer.interface.print(prefix ++ format ++ "\n", args) catch {};
}

pub const std_options = std.Options{
    .logFn = logFn,
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
        // .{ .scope = .mdec, .level = .debug },
        // .{ .scope = .joy, .level = .debug },
    },
};

const banner_text =
    \\
    \\              ____  ______  __
    \\  _ __  _   _|  _ \/ ___\ \/ /
    \\ | '_ \| | | | |_) \___ \\  / 
    \\ | | | | |_| |  __/ ___) /  \ 
    \\ |_| |_|\__,_|_|   |____/_/\_\   PlayStation Emulator
    \\
    \\
;

fn printBanner(io: std.Io) void {
    var buf: [1024]u8 = undefined;
    var writer = std.Io.File.stdout().writer(io, &buf);
    _ = writer.interface.write(banner_text) catch 0;
    writer.interface.flush() catch {};
}

const Audio = struct {
    allocator: std.mem.Allocator,
    device: *zaudio.Device,
    bus: *Bus,
    muted: bool = false,

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

    pub fn setMuted(self: *@This(), muted: bool) void {
        self.device.setMasterVolume(if (muted) 0.0 else 1.0) catch |err| {
            std.log.err("failed to set audio master volume: {}", .{err});
            return;
        };
        self.muted = muted;
    }

    pub fn toggleMuted(self: *@This()) void {
        self.setMuted(!self.muted);
    }

    pub fn muteToggleCallback(user_data: *anyopaque) void {
        const self: *@This() = @ptrCast(@alignCast(user_data));
        self.toggleMuted();
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
            const sample = self.bus.audio_stream.pop();
            buf[i * 2 + 0] = sample[0];
            buf[i * 2 + 1] = sample[1];
        }
    }
};

pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const io = init.io;

    var args = Args.parse(allocator, io, init.minimal.args) catch {
        Args.printHelp(io);
        std.process.exit(1);
    };
    defer args.deinit();

    printBanner(io);

    const bios = try BIOS.loadFromFile(allocator, io, args.bios_path);
    defer bios.deinit();

    const bus = Bus.init(allocator);
    defer bus.deinit();

    const ram = RAM.init(allocator);
    defer ram.deinit();

    const scratchpad = Scratchpad.init(allocator);
    defer scratchpad.deinit();

    const gpu = GPU.init(allocator, io, bus);
    defer gpu.deinit();

    const cpu = CPU.init(allocator, bus);
    defer cpu.deinit();

    const dma = DMA.init(allocator, bus);
    defer dma.deinit();

    const mdec = MDEC.init(allocator);
    defer mdec.deinit();

    const timers = Timers.init(allocator, bus);
    defer timers.deinit();

    const spu = SPU.init(allocator, bus);
    defer spu.deinit();

    const memcard_path = if (args.memcard_path.len != 0) args.memcard_path else memcard_default_path;
    const memcard = MemoryCard.loadOrCreate(allocator, io, memcard_path) catch |err| {
        std.log.err("failed to load or create memory card: {}", .{err});
        return err;
    };
    std.log.info("using memory card file: {s}", .{memcard_path});
    defer memcard.deinit();

    const sio0 = SIO0.init(allocator, bus, memcard);
    defer sio0.deinit();

    const sio1 = SIO1.init(allocator);
    defer sio1.deinit();

    var disc: ?*Disc = null;
    defer if (disc) |d| d.deinit();

    const cdrom = CDROM.init(allocator, bus);
    defer cdrom.deinit();

    if (args.cd_image_path.len != 0) {
        const path = args.cd_image_path;
        if (std.mem.endsWith(u8, path, ".cue")) {
            disc = try Disc.loadCue(allocator, io, path);
        } else if (std.mem.endsWith(u8, path, ".bin")) {
            disc = try Disc.loadBin(allocator, io, path);
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
        .sio0 = sio0,
        .sio1 = sio1,
        .cdrom = cdrom,
        .timers = timers,
        .scratchpad = scratchpad,
    });

    const audio = Audio.init(allocator, bus);
    defer audio.deinit();

    if (args.exe_path.len != 0) {
        const path = args.exe_path;
        while (cpu.pc != exe.exe_start_addr) {
            cpu.tickOnce();
        }
        std.log.info("loading exe file: {s}", .{path});
        try exe.loadExe(allocator, io, path, cpu, bus);
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

                if (gpu.consumeFrameReady()) {
                    debug_ui.update(io);
                }
            }
        }
    } else {
        const ui = try UI.init(allocator, io, gpu, sio0);
        defer ui.deinit();

        ui.setMuteCallback(Audio.muteToggleCallback, audio);
        if (args.uncapped) ui.setUncapped(true);

        if (args.cd_image_path.len != 0) {
            try ui.setFilename(args.cd_image_path);
        } else if (args.exe_path.len != 0) {
            try ui.setFilename(args.exe_path);
        }

        var buf: [1024]u8 = undefined;
        var stdout = std.Io.File.stdout().writer(io, &buf);
        cpu.tty = &stdout.interface;

        while (ui.is_running) {
            bus.tick();

            if (gpu.consumeFrameReady()) {
                ui.update();
            }
        }
    }
}
