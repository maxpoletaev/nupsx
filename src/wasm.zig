const std = @import("std");

const mem_mod = @import("mem.zig");
const timer_mod = @import("timer.zig");
const gpu_mod = @import("gpu.zig");
const cdrom_mod = @import("cdrom.zig");
const spu_mod = @import("spu.zig");
const joy_mod = @import("joy.zig");
const CPU = @import("cpu.zig").CPU;
const DMA = @import("dma.zig").DMA;
const MDEC = @import("mdec.zig").MDEC;
const exe_mod = @import("exe.zig");

const Bus = mem_mod.Bus;
const BIOS = mem_mod.BIOS;
const RAM = mem_mod.RAM;
const Scratchpad = mem_mod.Scratchpad;
const GPU = gpu_mod.GPU;
const Timers = timer_mod.Timers;
const CDROM = cdrom_mod.CDROM;
const SPU = spu_mod.SPU;
const Joypad = joy_mod.Joypad;

var bus: *Bus = undefined;
var cpu: *CPU = undefined;
var gpu: *GPU = undefined;
var ram: *RAM = undefined;
var scratchpad: *Scratchpad = undefined;
var bios: *BIOS = undefined;
var dma: *DMA = undefined;
var mdec: *MDEC = undefined;
var timers: *Timers = undefined;
var spu: *SPU = undefined;
var joy: *Joypad = undefined;
var cdrom: *CDROM = undefined;

extern fn js_console_log(ptr: [*]const u8, len: usize) void;

pub fn logFn(
    comptime message_level: std.log.Level,
    comptime scope: @Type(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    var buf: [4096]u8 = undefined;
    const level_str = switch (message_level) {
        .err => "ERROR",
        .warn => "WARN",
        .info => "INFO",
        .debug => "DEBUG",
    };

    const prefix = std.fmt.bufPrint(&buf, "[{s}] ({s}): ", .{ level_str, @tagName(scope) }) catch return;
    const message = std.fmt.bufPrint(buf[prefix.len..], format, args) catch return;
    const full_len = prefix.len + message.len;

    js_console_log(&buf, full_len);
}

pub const std_options: std.Options = .{
    .logFn = logFn,
};

// 1GB of heap for WASM (CPU needs ~128MB for block cache + disk image)
var heap_memory: [1 << 30]u8 = undefined;
var fba = std.heap.FixedBufferAllocator.init(&heap_memory);

export fn alloc(size: usize) [*]u8 {
    const allocator = fba.allocator();
    const buf = allocator.alloc(u8, size) catch @panic("OOM");
    return buf.ptr;
}

export fn allocAligned(size: usize, alignment: usize) [*]u8 {
    const allocator = fba.allocator();
    const buf = switch (alignment) {
        1 => allocator.alignedAlloc(u8, .@"1", size) catch @panic("OOM"),
        2 => allocator.alignedAlloc(u8, .@"2", size) catch @panic("OOM"),
        4 => allocator.alignedAlloc(u8, .@"4", size) catch @panic("OOM"),
        8 => allocator.alignedAlloc(u8, .@"8", size) catch @panic("OOM"),
        16 => allocator.alignedAlloc(u8, .@"16", size) catch @panic("OOM"),
        32 => allocator.alignedAlloc(u8, .@"32", size) catch @panic("OOM"),
        else => @panic("unsupported alignment"),
    };
    return buf.ptr;
}

export fn init(bios_ptr: [*]const u8, bios_len: usize) void {
    const allocator = fba.allocator();

    bios = BIOS.loadFromBuffer(allocator, bios_ptr[0..bios_len]) catch |err| {
        std.debug.panic("invalid bios: {}", .{err});
    };

    bus = Bus.init(allocator);
    ram = RAM.init(allocator);
    scratchpad = Scratchpad.init(allocator);
    gpu = GPU.init(allocator, bus);
    cpu = CPU.init(allocator, bus);
    dma = DMA.init(allocator, bus);
    mdec = MDEC.init(allocator);
    timers = Timers.init(allocator, bus);
    spu = SPU.init(allocator, bus);
    joy = Joypad.init(allocator, bus);
    cdrom = CDROM.init(allocator, bus);

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

    std.log.info("nuPSX initialized", .{});
}

export fn loadExe(exe_ptr: [*]const u8, exe_len: usize) void {
    while (cpu.pc != 0x80030000) {
        cpu.tickOnce();
    }
    exe_mod.loadExeFromBuffer(exe_ptr[0..exe_len], cpu, bus) catch |err| {
        std.debug.panic("failed to load exe: {}", .{err});
    };
}

export fn loadBin(bin_ptr: [*]const u8, bin_len: usize) void {
    const buf: []align(2) const u8 = @alignCast(bin_ptr[0..bin_len]);
    const disc = cdrom_mod.Disc.loadBinFromOwnedBuffer(fba.allocator(), buf);
    cdrom.insertDisc(disc);
}

export fn runFrame() void {
    while (!gpu.consumeFrameReady()) {
        bus.tick();
    }
}

export fn getVRAMPtr() [*]u16 {
    return @ptrCast(gpu.vram);
}

const DisplayInfo = extern struct {
    offset_x: u16,
    offset_y: u16,
    width: u16,
    height: u16,
};

var display_info: DisplayInfo = .{
    .offset_x = 0,
    .offset_y = 0,
    .width = 320,
    .height = 240,
};

export fn getDisplayInfoPtr() *DisplayInfo {
    const res = gpu.getDisplayRes();
    display_info = .{
        .offset_x = @intCast(gpu.gp1_display_area_start.x),
        .offset_y = @intCast(gpu.gp1_display_area_start.y),
        .width = res[0],
        .height = res[1],
    };
    return &display_info;
}

export fn setButtonState(state: u16) void {
    joy.buttons[0] = @bitCast(state);
}
