const std = @import("std");
const builtin = @import("builtin");

const GPU = @import("gpu.zig").GPU;
const DMA = @import("dma.zig").DMA;
const CPU = @import("cpu.zig").CPU;
const SPU = @import("spu.zig").SPU;
const CDROM = @import("cdrom.zig").CDROM;
const Timers = @import("timer.zig").Timers;
const Joypad = @import("joy.zig").Joypad;

const expectEqual = std.testing.expectEqual;
const log = std.log.scoped(.mem);

const addr_mask_table = [_]u32{
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0x7fffffff, 0x1fffffff, 0xffffffff, 0xffffffff,
};

inline fn maskAddr(addr: u32) u32 {
    const mask = addr_mask_table[addr >> 29];
    return addr & mask;
}

test "maskAddr" {
    try expectEqual(0x00000000, maskAddr(0x00000000));
    try expectEqual(0x00000000, maskAddr(0x80000000));
    try expectEqual(0x00000000, maskAddr(0xa0000000));
}

/// BIOS ROM (512KB)
pub const BIOS = struct {
    pub const addr_start: u32 = 0x1fc00000;
    pub const addr_end: u32 = 0x1fc7ffff;

    allocator: std.mem.Allocator,
    rom: []u8,

    pub fn loadFromFile(allocator: std.mem.Allocator, path: []const u8) !*@This() {
        const file = try std.fs.cwd().openFile(path, .{});
        defer file.close();

        const file_size = try file.getEndPos();
        const rom = try allocator.alloc(u8, file_size);

        const bytes_read = try file.readAll(rom);
        if (bytes_read != file_size) {
            return error.FileReadError;
        }

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .rom = rom,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        const allocator = self.allocator;
        allocator.free(self.rom);
        allocator.destroy(self);
    }

    pub inline fn read(self: *@This(), comptime T: type, addr: u32) T {
        return readBuf(T, self.rom, addr - addr_start);
    }

    pub inline fn write(_: *@This(), comptime T: type, _: u32, _: T) void {
        // BIOS is read-only
    }
};

/// Main RAM (2MB)
pub const RAM = struct {
    pub const addr_start: u32 = 0x00000000;
    pub const addr_end: u32 = 0x001fffff;

    data: [0x200000]u8,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{ .data = undefined };
        @memset(&self.data, 0);
        return self;
    }

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        allocator.destroy(self);
    }

    pub inline fn read(self: *@This(), comptime T: type, addr: u32) T {
        return readBuf(T, &self.data, addr - addr_start);
    }

    pub inline fn write(self: *@This(), comptime T: type, addr: u32, value: T) void {
        return writeBuf(T, &self.data, addr - addr_start, value);
    }
};

/// Scratchpad fast RAM (1KB)
pub const Scratchpad = struct {
    pub const addr_start: u32 = 0x1f800000;
    pub const addr_end: u32 = 0x1f8003ff;

    data: [0x400]u8,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{ .data = undefined };
        @memset(&self.data, 0);
        return self;
    }

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        allocator.destroy(self);
    }

    pub inline fn read(self: *@This(), comptime T: type, addr: u32) T {
        return readBuf(T, &self.data, addr - addr_start);
    }

    pub inline fn write(self: *@This(), comptime T: type, addr: u32, value: T) void {
        return writeBuf(T, &self.data, addr - addr_start, value);
    }
};

/// Connetced deivces set for 2-step initialization.
pub const Devices = struct {
    cpu: *CPU,
    bios: *BIOS,
    ram: *RAM,
    gpu: *GPU,
    dma: *DMA,
    spu: *SPU,
    joy: *Joypad,
    cdrom: *CDROM,
    timers: *Timers,
    scratchpad: *Scratchpad,
};

pub const Interrupt = opaque {
    pub const vblank: u32 = 1 << 0;
    pub const gpu: u32 = 1 << 1;
    pub const cdrom: u32 = 1 << 2;
    pub const dma: u32 = 1 << 3;
    pub const tmr0: u32 = 1 << 4;
    pub const tmr1: u32 = 1 << 5;
    pub const tmr2: u32 = 1 << 6;
    pub const joy_mc_byte: u32 = 1 << 7;
    pub const sio: u32 = 1 << 8;
    pub const spu: u32 = 1 << 9;
    pub const lightpen: u32 = 1 << 10;
};

const addr_irq_stat: u32 = 0x1f801070;
const addr_irq_mask: u32 = 0x1f801074;

/// The main interconnect bus for all the devices within the console.
pub const Bus = struct {
    allocator: std.mem.Allocator,
    dev: Devices,

    irq_mask: u32 = 0,
    irq_stat: u32 = 0,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .dev = undefined,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn connect(self: *@This(), dev: Devices) void {
        self.dev = dev;
    }

    inline fn updateInterruptPending(self: *@This()) void {
        const pending = (self.irq_stat & self.irq_mask) != 0;
        self.dev.cpu.requestInterrupt(pending);
    }

    pub fn setInterrupt(self: *@This(), v: u32) void {
        self.irq_stat |= v;
        self.updateInterruptPending();
    }

    pub fn tick(self: *@This()) void {
        self.dev.cpu.tick();
        inline for (0..5) |_| self.dev.gpu.tick();
        self.dev.timers.tick();
        self.dev.cdrom.tick();
        self.dev.joy.tick();

        // GPU events dispatch
        const gpu_events = self.dev.gpu.consumeEvents();
        if (gpu_events.hblank_start) self.dev.timers.hblankStart();
        if (gpu_events.hblank_end) self.dev.timers.hblankEnd();
        if (gpu_events.vblank_start) {
            self.dev.timers.vblankStart();
            self.setInterrupt(Interrupt.vblank);
        }
        if (gpu_events.vblank_end) self.dev.timers.vblankEnd();

        // Timer events dispatch
        const timer_events = self.dev.timers.consumeEvents();
        if (timer_events.t0_fired) self.setInterrupt(Interrupt.tmr0);
        if (timer_events.t1_fired) self.setInterrupt(Interrupt.tmr1);
        if (timer_events.t2_fired) self.setInterrupt(Interrupt.tmr2);

        const cdrom_events = self.dev.cdrom.consumeEvents();
        if (cdrom_events.interrupt) self.setInterrupt(Interrupt.cdrom);

        const dma_irq = self.dev.dma.consumeIrq();
        if (dma_irq) self.setInterrupt(Interrupt.dma);

        const joy_irq = self.dev.joy.consumeIrq();
        if (joy_irq) {
            self.setInterrupt(Interrupt.joy_mc_byte);
            // log.debug("Joypad IRQ set", .{});
        }
    }

    inline fn setIrqStat(self: *@This(), v: u32) void {
        self.irq_stat &= v; // (0=acknowledge, 1=no change)`
        self.updateInterruptPending();
        // log.debug("irq_stat write: {x}", .{v});
    }

    inline fn setIrqMask(self: *@This(), v: u32) void {
        self.irq_mask = v;
        self.updateInterruptPending();
        // log.debug("irq_mask write: {x}", .{v});
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const masked_addr = maskAddr(addr);

        return switch (masked_addr) {
            addr_irq_stat => @as(T, @truncate(self.irq_stat)),
            addr_irq_mask => @as(T, @truncate(self.irq_mask)),

            RAM.addr_start...RAM.addr_end => self.dev.ram.read(T, masked_addr),
            BIOS.addr_start...BIOS.addr_end => self.dev.bios.read(T, masked_addr),
            GPU.addr_start...GPU.addr_end => self.dev.gpu.read(T, masked_addr),
            DMA.addr_start...DMA.addr_end => self.dev.dma.read(T, masked_addr),
            Timers.addr_start...Timers.addr_end => self.dev.timers.read(T, masked_addr),
            CDROM.addr_start...CDROM.addr_end => self.dev.cdrom.read(T, masked_addr),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.read(T, masked_addr),
            SPU.addr_start...SPU.addr_end => self.dev.spu.read(T, masked_addr),
            Joypad.addr_start...Joypad.addr_end => self.dev.joy.read(T, masked_addr),

            0x1f801000...0x1f801023 => 0, // memctl
            0x1f801060...0x1f801063 => 0, // ramsize
            0xfffe0130...0xfffe0133 => 0, // cachectl
            0x1f000000...0x1f0000ff => 0, // expansion 1
            0x1f802000...0x1f802041 => 0, // expansion 2

            else => blk: {
                const v = switch (T) {
                    u8 => 0xac,
                    u16 => 0xacab,
                    u32 => 0xacabacab,
                    else => @compileError("unsupported type"),
                };
                std.debug.panic("unhandled read ({s}) at {x}", .{ @typeName(T), addr });
                break :blk v;
            },
        };
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const masked_addr = maskAddr(addr);

        switch (masked_addr) {
            addr_irq_stat => self.setIrqStat(v),
            addr_irq_mask => self.setIrqMask(v),

            RAM.addr_start...RAM.addr_end => self.dev.ram.write(T, masked_addr, v),
            BIOS.addr_start...BIOS.addr_end => self.dev.bios.write(T, masked_addr, v),
            GPU.addr_start...GPU.addr_end => self.dev.gpu.write(T, masked_addr, v),
            DMA.addr_start...DMA.addr_end => self.dev.dma.write(T, masked_addr, v),
            Timers.addr_start...Timers.addr_end => self.dev.timers.write(T, masked_addr, v),
            CDROM.addr_start...CDROM.addr_end => self.dev.cdrom.write(T, masked_addr, v),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.write(T, masked_addr, v),
            SPU.addr_start...SPU.addr_end => self.dev.spu.write(T, masked_addr, v),
            Joypad.addr_start...Joypad.addr_end => self.dev.joy.write(T, masked_addr, v),

            0x1f801000...0x1f801023 => {}, // memctl
            0x1f801060...0x1f801063 => {}, // ramsize
            0xfffe0130...0xfffe0133 => {}, // cachectl
            0x1f000000...0x1f0000ff => {}, // expansion 1
            0x1f802000...0x1f802041 => {}, // expansion 2

            else => std.debug.panic("unhandled write ({s}) at {x} = {x}", .{ @typeName(T), addr, v }),
        }
    }
};

pub inline fn readBuf(comptime T: type, buf: []const u8, offset: u32) T {
    const t_size = @sizeOf(T);

    if (comptime builtin.mode == .Debug) {
        if (offset % t_size != 0) {
            @branchHint(.unlikely);
            log.warn("unaligned read: {x}", .{offset});
        }
    }

    const sl = buf[offset .. offset + t_size][0..t_size];
    return std.mem.readInt(T, sl, .little);
}

pub inline fn writeBuf(comptime T: type, buf: []u8, offset: u32, v: T) void {
    const t_size = @sizeOf(T);

    if (comptime builtin.mode == .Debug) {
        if (offset % t_size != 0) {
            @branchHint(.unlikely);
            log.warn("unaligned write: {x}", .{offset});
        }
    }

    const sl = buf[offset .. offset + t_size][0..t_size];
    std.mem.writeInt(T, sl, v, .little);
}
