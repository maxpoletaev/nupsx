const std = @import("std");
const builtin = @import("builtin");

const BIOS = @import("bios.zig").BIOS;
const GPU = @import("gpu.zig").GPU;
const DMA = @import("dma.zig").DMA;

const expectEqual = std.testing.expectEqual;
const log = std.log.scoped(.mem);

const DeviceId = enum { none, ram, bios, scratchpad, gpu, dma, spu_ctrl };

pub const Addr = opaque {
    pub const addr_gpu_gp0: u32 = 0x1f801810;
    pub const addr_gpu_gp1: u32 = 0x1f801814;
    pub const addr_gpu_gpuread: u32 = 0x1f801810;
    pub const addr_gpu_gpustat: u32 = 0x1f801814;
};

pub const AddrRange = struct {
    device: DeviceId,
    start: u32,
    end: u32,
    size: u32,

    pub fn init(device: DeviceId, start: u32, size: u32) AddrRange {
        return AddrRange{
            .device = device,
            .start = start,
            .end = start + size,
            .size = size,
        };
    }

    pub inline fn match(self: AddrRange, addr: u32) bool {
        return (addr >= self.start and addr <= self.end);
    }

    pub inline fn matchWithOffset(self: AddrRange, addr: u32) ?u32 {
        if (addr >= self.start and addr <= self.end) {
            return addr - self.start;
        }
        return null;
    }
};

const mem_region_mirror_table = [_]u32{
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0x7fffffff, 0x1fffffff, 0xffffffff, 0xffffffff,
};

inline fn resolveAddr(addr: u32) u32 {
    const mask = mem_region_mirror_table[addr >> 29];
    return addr & mask;
}

const addr_ranges = [_]AddrRange{
    .init(.ram, 0x00000000, 0x200000),
    .init(.scratchpad, 0x1f800000, 0x400),
    .init(.bios, 0x1fc00000, 0x80000),
    .init(.gpu, 0x1f801810, 0x8),
    .init(.dma, 0x1f801080, 0x80),
    .init(.spu_ctrl, 0x1f801d80, 0x40),
};

/// Lookup device by its address range.
inline fn lookupDevice(addr: u32) struct { DeviceId, u32 } {
    inline for (addr_ranges) |r| {
        if (r.matchWithOffset(addr)) |offset| {
            return .{ r.device, offset };
        }
    }
    return .{ .none, 0 };
}

test "lookup device" {
    const device, const offset = lookupDevice(0x1f800000);
    try expectEqual(.scratchpad, device);
    try expectEqual(0, offset);
}

/// Connetced deivces set for 2-step initialization.
pub const Devices = struct {
    bios: *BIOS,
    gpu: *GPU,
    dma: *DMA,
};

/// The main interconnect bus for all the devices within the console.
pub const Bus = struct {
    allocator: std.mem.Allocator,
    scratchpad: [0x400]u8,
    ram: [0x200000]u8,
    dev: Devices,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .scratchpad = undefined,
            .ram = undefined,
            .dev = undefined,
        };

        @memset(&self.ram, 0);
        @memset(&self.scratchpad, 0);

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn initDevices(self: *@This(), dev: Devices) void {
        self.dev = dev;
    }

    pub fn readByte(self: *@This(), addr: u32) u8 {
        const resolved_addr = resolveAddr(addr);
        const device, const offset = lookupDevice(resolved_addr);

        switch (device) {
            .scratchpad => return read(u8, &self.scratchpad, offset),
            .bios => return read(u8, self.dev.bios.rom, offset),
            .ram => return read(u8, &self.ram, offset),
            else => {},
        }

        // log.debug("readByte: out of bounds: {x}", .{addr});
        return 0xac;
    }

    pub fn readHalf(self: *@This(), addr: u32) u16 {
        const resolved_addr = resolveAddr(addr);
        const device, const offset = lookupDevice(resolved_addr);

        switch (device) {
            .ram => return read(u16, &self.ram, offset),
            .scratchpad => return read(u16, &self.scratchpad, offset),
            .bios => return read(u16, self.dev.bios.rom, offset),
            else => {},
        }

        // log.debug("readHalf: out of bounds: {x}", .{addr});
        return 0xacab;
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        const resolved_addr = resolveAddr(addr);
        const device, const offset = lookupDevice(resolved_addr);

        switch (device) {
            .ram => return read(u32, &self.ram, offset),
            .scratchpad => return read(u32, &self.scratchpad, offset),
            .bios => return read(u32, self.dev.bios.rom, offset),
            .gpu => return self.dev.gpu.readWord(offset),
            else => {},
        }

        // log.debug("readWord: out of bounds: {x}", .{addr});
        return 0xacabacab;
    }

    pub fn writeByte(self: *@This(), addr: u32, value: u8) void {
        const resolved_addr = resolveAddr(addr);
        const device, const offset = lookupDevice(resolved_addr);

        switch (device) {
            .ram => return writeToBuf(u8, &self.ram, offset, value),
            .scratchpad => return writeToBuf(u8, &self.scratchpad, offset, value),
            else => {},
        }

        // log.warn("writeByte: out of bounds: {x}", .{addr})
    }

    pub fn writeHalf(self: *@This(), addr: u32, value: u16) void {
        const resolved_addr = resolveAddr(addr);
        const device, const offset = lookupDevice(resolved_addr);

        switch (device) {
            .ram => return writeToBuf(u16, &self.ram, offset, value),
            .scratchpad => return writeToBuf(u16, &self.scratchpad, offset, value),
            else => {},
        }

        // log.warn("writeHalf: out of bounds: {x}", .{addr});
    }

    pub fn writeWord(self: *@This(), addr: u32, v: u32) void {
        const resolved_addr = resolveAddr(addr);
        const device, const offset = lookupDevice(resolved_addr);

        switch (device) {
            .ram => return writeToBuf(u32, &self.ram, offset, v),
            .scratchpad => return writeToBuf(u32, &self.scratchpad, offset, v),
            .gpu => return self.dev.gpu.writeWord(offset, v),
            .dma => return self.dev.dma.writeWord(offset, v),
            else => {},
        }

        // log.warn("writeWord: out of bounds: {x}", .{addr});
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

pub inline fn writeToBuf(comptime T: type, buf: []u8, offset: u32, v: T) void {
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
