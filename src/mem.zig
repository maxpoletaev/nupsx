const std = @import("std");
const builtin = @import("builtin");

const bios = @import("bios.zig");
const GPU = @import("gpu.zig").GPU;

const log = std.log.scoped(.mem);

pub const AddrRange = struct {
    start: u32,
    end: u32,
    size: u32,

    pub fn init(start: u32, size: u32) AddrRange {
        return AddrRange{
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

pub const ram_range = AddrRange.init(0x00000000, 0x200000); // 2048K
pub const exp1_range = AddrRange.init(0x1f000000, 0x800000); // 8192K
pub const scratchpad_range = AddrRange.init(0x1f800000, 0x400); // 1K
pub const exp2_range = AddrRange.init(0x1f802000, 0x2000); // 8K
pub const bios_range = AddrRange.init(0x1fc00000, 0x80000); // 512K

const RAM = struct {
    const ram_size = 0x200000;

    allocator: std.mem.Allocator,
    buf: [0x200000]u8,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .buf = undefined,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        const allocator = self.allocator;
        allocator.destroy(self);
    }

    pub fn read(self: *const @This(), comptime T: type, addr: u32) T {
        const size = @sizeOf(T);
        if (addr % size != 0) {
            std.debug.panic("unaligned address: {x}", .{addr});
        }

        const idx = addr % ram_size;
        const buf = self.buf[idx .. idx + size][0..size];
        return std.mem.readInt(T, buf, .little);
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const size = @sizeOf(T);
        if (addr % size != 0) {
            std.debug.panic("unaligned address: {x}", .{addr});
        }

        const idx = addr % ram_size;
        const buf = self.buf[idx .. idx + size][0..size];
        return std.mem.writeInt(T, buf, v, .little);
    }
};

const mem_region_mask_table = [_]u32{
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, // KUSEG (2048MB)
    0x7fffffff, // KSEG0 (512MB)
    0x1fffffff, // KSEG1 (512MB)
    0xffffffff, 0xffffffff, // KSEG2 (1024MB)
};

inline fn resolveAddr(addr: u32) u32 {
    const mask = mem_region_mask_table[addr >> 29];
    return addr & mask;
}

pub const Bus = struct {
    ram: [0x200000]u8,
    scratchpad: [0x400]u8,

    _gpu: *GPU,
    _bios: *bios.BIOS,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, bios_: *bios.BIOS, gpu: *GPU) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .ram = undefined,
            .scratchpad = undefined,
            ._allocator = allocator,
            ._bios = bios_,
            ._gpu = gpu,
        };

        @memset(&self.ram, 0);
        @memset(&self.scratchpad, 0);

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.destroy(self);
    }

    pub fn readByte(self: *@This(), addr: u32) u8 {
        const real_addr = resolveAddr(addr);

        if (ram_range.matchWithOffset(real_addr)) |offset| {
            return read(u8, &self.ram, offset);
        }

        if (scratchpad_range.matchWithOffset(real_addr)) |offset| {
            return read(u8, &self.scratchpad, offset);
        }

        if (bios_range.match(real_addr)) {
            return self._bios.readByte(real_addr);
        }

        if (exp1_range.match(addr)) {
            return 0xff;
        }

        log.warn("readByte: out of bounds: {x}", .{addr});

        return 0xAC;
    }

    pub fn readHalf(self: *@This(), addr: u32) u16 {
        const real_addr = resolveAddr(addr);

        if (ram_range.matchWithOffset(real_addr)) |offset| {
            return read(u16, &self.ram, offset);
        }

        if (scratchpad_range.matchWithOffset(real_addr)) |offset| {
            return read(u16, &self.scratchpad, offset);
        }

        if (bios_range.match(real_addr)) {
            return self._bios.readHalf(real_addr);
        }

        if (exp1_range.match(addr)) {
            return 0xffff;
        }

        log.warn("readHalf: out of bounds: {x}", .{addr});

        return 0xACAB;
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        const real_addr = resolveAddr(addr);

        switch (real_addr) {
            0x1f801810, 0x1f801814 => return self._gpu.readWord(real_addr),
            else => {
                if (ram_range.matchWithOffset(real_addr)) |offset| {
                    return read(u32, &self.ram, offset);
                }
                if (scratchpad_range.matchWithOffset(real_addr)) |offset| {
                    return read(u32, &self.scratchpad, offset);
                }
                if (bios_range.match(real_addr)) {
                    return self._bios.readWord(real_addr);
                }
                if (exp1_range.match(addr)) {
                    return 0xffffffff;
                }
            },
        }

        log.warn("readWord: out of bounds: {x}", .{addr});

        return 0xACABACAB;
    }

    pub fn writeByte(self: *@This(), addr: u32, value: u8) void {
        const real_addr = resolveAddr(addr);

        if (ram_range.matchWithOffset(real_addr)) |offset| {
            write(u8, &self.ram, offset, value);
            return;
        }

        if (scratchpad_range.matchWithOffset(real_addr)) |offset| {
            write(u8, &self.scratchpad, offset, value);
            return;
        }

        log.warn("writeByte: out of bounds: {x}", .{addr});
    }

    pub fn writeHalf(self: *@This(), addr: u32, value: u16) void {
        const real_addr = resolveAddr(addr);

        if (ram_range.matchWithOffset(real_addr)) |offset| {
            write(u16, &self.ram, offset, value);
            return;
        }

        if (scratchpad_range.matchWithOffset(real_addr)) |offset| {
            write(u16, &self.scratchpad, offset, value);
            return;
        }

        log.warn("writeHalf: out of bounds: {x}", .{addr});
    }

    pub fn writeWord(self: *@This(), addr: u32, value: u32) void {
        const real_addr = resolveAddr(addr);

        switch (addr) {
            0x1f801810 => self._gpu.gp0write(value), // GP0
            0x1f801814 => self._gpu.gp1write(value), // GP1
            else => {
                if (ram_range.matchWithOffset(real_addr)) |offset| {
                    write(u32, &self.ram, offset, value);
                    return;
                }
                if (scratchpad_range.matchWithOffset(real_addr)) |offset| {
                    write(u32, &self.scratchpad, offset, value);
                    return;
                }
            },
        }

        log.warn("writeWord: out of bounds: {x}", .{addr});
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
