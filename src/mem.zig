const std = @import("std");
const builtin = @import("builtin");

const BIOS = @import("bios.zig").BIOS;
const GPU = @import("gpu.zig").GPU;
const DMA = @import("dma.zig").DMA;
const CPU = @import("cpu.zig").CPU;
const Timers = @import("timer.zig").Timers;

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

    pub inline fn readByte(self: *@This(), addr: u32) u8 {
        return read(u8, &self.data, addr - addr_start);
    }
    pub inline fn readHalf(self: *@This(), addr: u32) u16 {
        return read(u16, &self.data, addr - addr_start);
    }
    pub inline fn readWord(self: *@This(), addr: u32) u32 {
        return read(u32, &self.data, addr - addr_start);
    }

    pub inline fn writeByte(self: *@This(), addr: u32, value: u8) void {
        write(u8, &self.data, addr - addr_start, value);
    }
    pub inline fn writeHalf(self: *@This(), addr: u32, value: u16) void {
        write(u16, &self.data, addr - addr_start, value);
    }
    pub inline fn writeWord(self: *@This(), addr: u32, value: u32) void {
        write(u32, &self.data, addr - addr_start, value);
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

    pub inline fn readByte(self: *@This(), addr: u32) u8 {
        return read(u8, &self.data, addr - addr_start);
    }
    pub inline fn readHalf(self: *@This(), addr: u32) u16 {
        return read(u16, &self.data, addr - addr_start);
    }
    pub inline fn readWord(self: *@This(), addr: u32) u32 {
        return read(u32, &self.data, addr - addr_start);
    }

    pub inline fn writeByte(self: *@This(), addr: u32, value: u8) void {
        write(u8, &self.data, addr - addr_start, value);
    }
    pub inline fn writeHalf(self: *@This(), addr: u32, value: u16) void {
        write(u16, &self.data, addr - addr_start, value);
    }
    pub inline fn writeWord(self: *@This(), addr: u32, value: u32) void {
        write(u32, &self.data, addr - addr_start, value);
    }
};

/// Connetced deivces set for 2-step initialization.
pub const Devices = struct {
    cpu: *CPU,
    bios: *BIOS,
    ram: *RAM,
    scratchpad: *Scratchpad,
    gpu: *GPU,
    dma: *DMA,
    timers: *Timers,
};

pub const Interrupt = opaque {
    pub const vblank: u32 = 1 << 0;
    pub const gpu: u32 = 1 << 1;
    pub const cdrom: u32 = 1 << 2;
    pub const dma: u32 = 1 << 3;
    pub const tmr0: u32 = 1 << 4;
    pub const tmr1: u32 = 1 << 5;
    pub const tmr2: u32 = 1 << 6;
    pub const ctrl_mc_byte: u32 = 1 << 7;
    pub const sio: u32 = 1 << 8;
    pub const spu: u32 = 1 << 9;
    pub const lightpen: u32 = 1 << 10;
};

pub fn unhandledRead(comptime T: anytype, addr: u32) T {
    const v = switch (T) {
        u8 => 0xac,
        u16 => 0xacab,
        u32 => 0xacabacab,
        else => @compileError("unsupported type"),
    };
    log.warn("unhandled read at address: {x}", .{addr});
    return v;
}

pub fn unhandledWrite(comptime T: anytype, addr: u32, value: T) void {
    log.warn("unhandled write at address: {x}={x}", .{ addr, value });
}

const addr_istat: u32 = 0x1f801070;
const addr_imask: u32 = 0x1f801074;
const addr_spu_start: u32 = 0x1f801d80;
const addr_spu_end: u32 = 0x1f801dbf;
const addr_spu_stat: u32 = 0x1f801dae;

/// The main interconnect bus for all the devices within the console.
pub const Bus = struct {
    allocator: std.mem.Allocator,
    dev: Devices,

    irq_mask: u32 = 0,
    irq_status: u32 = 0,

    debug_pause: bool = false,

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

    pub fn initDevices(self: *@This(), dev: Devices) void {
        self.dev = dev;
    }

    pub fn setInterrupt(self: *@This(), v: u32) void {
        self.irq_status |= v;
        self.dev.cpu.setInterruptPending(@truncate(self.irq_status));
    }

    pub fn tickSystemClock(self: *@This()) void {
        self.dev.timers.tickSystemClock();
    }

    pub fn readByte(self: *@This(), addr: u32) u8 {
        const masked_addr = maskAddr(addr);

        return switch (masked_addr) {
            RAM.addr_start...RAM.addr_end => self.dev.ram.readByte(masked_addr),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.readByte(masked_addr),
            BIOS.addr_start...BIOS.addr_end => self.dev.bios.readByte(masked_addr),
            Timers.addr_start...Timers.addr_end => self.dev.timers.readByte(masked_addr),
            else => unhandledRead(u8, addr),
        };
    }

    pub fn readHalf(self: *@This(), addr: u32) u16 {
        const masked_addr = maskAddr(addr);

        return switch (masked_addr) {
            addr_spu_stat => return 0x2007,
            RAM.addr_start...RAM.addr_end => self.dev.ram.readHalf(masked_addr),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.readHalf(masked_addr),
            BIOS.addr_start...BIOS.addr_end => self.dev.bios.readHalf(masked_addr),
            Timers.addr_start...Timers.addr_end => self.dev.timers.readHalf(masked_addr),
            else => unhandledRead(u16, masked_addr),
        };
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        const masked_addr = maskAddr(addr);

        return switch (masked_addr) {
            addr_istat => self.irq_status,
            addr_imask => self.irq_mask,
            RAM.addr_start...RAM.addr_end => self.dev.ram.readWord(masked_addr),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.readWord(masked_addr),
            BIOS.addr_start...BIOS.addr_end => self.dev.bios.readWord(masked_addr),
            GPU.addr_start...GPU.addr_end => self.dev.gpu.readWord(masked_addr),
            DMA.addr_start...DMA.addr_end => self.dev.dma.readWord(masked_addr),
            Timers.addr_start...Timers.addr_end => self.dev.timers.readWord(masked_addr),
            else => unhandledRead(u32, masked_addr),
        };
    }

    pub fn writeByte(self: *@This(), addr: u32, value: u8) void {
        const masked_addr = maskAddr(addr);
        switch (masked_addr) {
            RAM.addr_start...RAM.addr_end => self.dev.ram.writeByte(masked_addr, value),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.writeByte(masked_addr, value),
            Timers.addr_start...Timers.addr_end => self.dev.timers.writeByte(masked_addr, value),
            else => unhandledWrite(u8, addr, value),
        }
    }

    pub fn writeHalf(self: *@This(), addr: u32, value: u16) void {
        const masked_addr = maskAddr(addr);

        switch (masked_addr) {
            RAM.addr_start...RAM.addr_end => self.dev.ram.writeHalf(masked_addr, value),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.writeHalf(masked_addr, value),
            Timers.addr_start...Timers.addr_end => self.dev.timers.writeHalf(masked_addr, value),
            else => unhandledWrite(u16, addr, value),
        }
    }

    pub fn writeWord(self: *@This(), addr: u32, v: u32) void {
        const masked_addr = maskAddr(addr);

        switch (masked_addr) {
            addr_istat => {
                self.irq_status &= v;
                self.dev.cpu.setInterruptPending(@truncate(self.irq_status));
            },
            addr_imask => self.irq_mask = v,
            RAM.addr_start...RAM.addr_end => self.dev.ram.writeWord(masked_addr, v),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.writeWord(masked_addr, v),
            GPU.addr_start...GPU.addr_end => self.dev.gpu.writeWord(masked_addr, v),
            DMA.addr_start...DMA.addr_end => self.dev.dma.writeWord(masked_addr, v),
            Timers.addr_start...Timers.addr_end => self.dev.timers.writeWord(masked_addr, v),
            else => unhandledWrite(u32, addr, v),
        }
    }
};

pub inline fn read(comptime T: type, buf: []u8, offset: u32) T {
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

pub inline fn write(comptime T: type, buf: []u8, offset: u32, v: T) void {
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
