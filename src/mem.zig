const std = @import("std");
const bios = @import("bios.zig");

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
};

pub const ram_size = 0x200000;
pub const ram_range = AddrRange.init(0x00000000, 0x200000); // 2048K
pub const exp1_range = AddrRange.init(0x1F000000, 0x800000); // 8192K
pub const exp2_range = AddrRange.init(0x1F802000, 0x2000); // 8K
pub const bios_range = AddrRange.init(0x1FC00000, 0x80000); // 512K

const RAM = struct {
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
    // KUSEG (2048MB)
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    // KSEG0 (512MB)
    0x7FFFFFFF,
    // KSEG1 (512MB)
    0x1FFFFFFF,
    // KSEG2 (1024MB)
    0xFFFFFFFF, 0xFFFFFFFF,
};

inline fn resolveAddr(addr: u32) u32 {
    const mask = mem_region_mask_table[addr >> 29];
    return addr & mask;
}

pub const Bus = struct {
    allocator: std.mem.Allocator,
    bios: *bios.BIOS,
    ram: *RAM,

    pub fn init(allocator: std.mem.Allocator, bios_: *bios.BIOS) !*@This() {
        const ram = try RAM.init(allocator);

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .bios = bios_,
            .ram = ram,
        };

        return self;
    }

    pub fn deinit(self: *const @This()) void {
        self.ram.deinit();

        const allocator = self.allocator;
        allocator.destroy(self);
    }

    pub fn readByte(self: *const @This(), addr: u32) u8 {
        const real_addr = resolveAddr(addr);

        if (ram_range.match(real_addr)) {
            return self.ram.read(u8, real_addr);
        } else if (bios_range.match(real_addr)) {
            return self.bios.readByte(real_addr);
        } else if (exp1_range.match(addr)) {
            return 0xFF;
        }

        // logger.warn("readByte: out of bounds: {x}", .{addr});
        return 0;
    }

    pub fn readHalf(self: *const @This(), addr: u32) u16 {
        if (addr % 2 != 0) log.warn("readHalf: unaligned read: {x}", .{addr});
        const real_addr = resolveAddr(addr);

        if (ram_range.match(real_addr)) {
            return self.ram.read(u16, real_addr);
        } else if (bios_range.match(real_addr)) {
            return self.bios.readHalf(real_addr);
        } else if (exp1_range.match(addr)) {
            return 0xFFFF;
        }

        //logger.warn("readHalf: out of bounds: {x}", .{addr});
        return 0;
    }

    pub fn readWord(self: *const @This(), addr: u32) u32 {
        if (addr % 4 != 0) log.warn("readWord: unaligned read: {x}", .{addr});
        const real_addr = resolveAddr(addr);

        if (ram_range.match(real_addr)) {
            return self.ram.read(u32, real_addr);
        } else if (bios_range.match(real_addr)) {
            return self.bios.readWord(real_addr);
        } else if (exp1_range.match(addr)) {
            return 0xFFFFFFFF;
        }

        //logger.warn("readWord: out of bounds: {x}", .{addr});
        return 0;
    }

    pub fn writeByte(self: *@This(), addr: u32, value: u8) void {
        const real_addr = resolveAddr(addr);

        if (ram_range.match(real_addr)) {
            self.ram.write(u8, real_addr, value);
            return;
        }

        //logger.warn("writeByte out of bounds: {x}", .{addr});
    }

    pub fn writeHalf(self: *@This(), addr: u32, value: u16) void {
        if (addr % 2 != 0) log.warn("writeHalf: unaligned write: {x}", .{addr});
        const real_addr = resolveAddr(addr);

        if (ram_range.match(real_addr)) {
            self.ram.write(u16, real_addr, value);
            return;
        }

        //logger.warn("writeHalf: out of bounds: {x}", .{addr});
    }

    pub fn writeWord(self: *@This(), addr: u32, value: u32) void {
        if (addr % 4 != 0) log.warn("writeWord: unaligned write: {x}", .{addr});
        const real_addr = resolveAddr(addr);

        if (ram_range.match(real_addr)) {
            self.ram.write(u32, real_addr, value);
            return;
        }

        //logger.warn("writeWord: out of bounds: {x}", .{addr});
    }
};
