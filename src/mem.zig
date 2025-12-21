const std = @import("std");
const builtin = @import("builtin");

const BIOS = @import("bios.zig").BIOS;
const GPU = @import("gpu.zig").GPU;
const DMA = @import("dma.zig").DMA;

const expectEqual = std.testing.expectEqual;
const log = std.log.scoped(.mem);

pub const Addr = opaque {
    // Main RAM: 2MB
    pub const ram_start: u32 = 0x00000000;
    pub const ram_end: u32 = 0x001fffff;

    // Scratchpad: 1KB fast RAM
    pub const scratchpad_start: u32 = 0x1f800000;
    pub const scratchpad_end: u32 = 0x1f8003ff;

    // Interrupt Control
    pub const i_stat: u32 = 0x1f801070;
    pub const i_mask: u32 = 0x1f801074;

    // DMA
    pub const dma_start: u32 = 0x1f801080;
    pub const dma_end: u32 = 0x1f8010ff;
    pub const dma_dpcr: u32 = 0x1f8010f0;
    pub const dma_dicr: u32 = 0x1f8010f4;

    // GPU
    pub const gpu_start: u32 = 0x1f801810;
    pub const gpu_end: u32 = 0x1f801817;
    pub const gpu_gp0: u32 = 0x1f801810;
    pub const gpu_gp1: u32 = 0x1f801814;

    // SPU
    pub const spu_start: u32 = 0x1f801d80;
    pub const spu_end: u32 = 0x1f801dbf;
    pub const spu_stat: u32 = 0x1f801dae;

    // BIOS: 512KB ROM
    pub const bios_start: u32 = 0x1fc00000;
    pub const bios_end: u32 = 0x1fc7ffff;
};

const mem_region_mirror_table = [_]u32{
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0x7fffffff, 0x1fffffff, 0xffffffff, 0xffffffff,
};

inline fn resolveAddr(addr: u32) u32 {
    const mask = mem_region_mirror_table[addr >> 29];
    return addr & mask;
}

test "resolveAddr" {
    try expectEqual(0x00000000, resolveAddr(0x00000000));
    try expectEqual(0x00000000, resolveAddr(0x80000000));
    try expectEqual(0x00000000, resolveAddr(0xa0000000));
}

/// Connetced deivces set for 2-step initialization.
pub const Devices = struct {
    bios: *BIOS,
    gpu: *GPU,
    dma: *DMA,
};

pub const Interupt = enum(u32) {
    vblank = 0,
    gpu = 1,
    cdrom = 2,
    dma = 3,
    tmr0 = 4,
    tmr1 = 5,
    tmr2 = 6,
    ctrl_mc_byte = 7,
    sio = 8,
    spu = 9,
    lightpen = 10,
};

/// The main interconnect bus for all the devices within the console.
pub const Bus = struct {
    allocator: std.mem.Allocator,
    interrupt_mask: u32 = 0,
    interrupt_status: u32 = 0,
    scratchpad: [0x400]u8,
    ram: [0x200000]u8,
    dev: Devices,

    debug_pause: bool = false,

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

        switch (resolved_addr) {
            Addr.ram_start...Addr.ram_end => {
                return read(u8, &self.ram, resolved_addr);
            },
            Addr.scratchpad_start...Addr.scratchpad_end => {
                const offset = resolved_addr - Addr.scratchpad_start;
                return read(u8, &self.scratchpad, offset);
            },
            Addr.bios_start...Addr.bios_end => {
                const offset = resolved_addr - Addr.bios_start;
                return read(u8, self.dev.bios.rom, offset);
            },
            else => {},
        }

        // log.debug("readByte: out of bounds: {x}", .{addr});
        // self.debug_pause = true;
        return 0xac;
    }

    pub fn readHalf(self: *@This(), addr: u32) u16 {
        switch (addr) {
            Addr.spu_stat => return 0x2007,
            else => {},
        }

        const resolved_addr = resolveAddr(addr);

        switch (resolved_addr) {
            Addr.ram_start...Addr.ram_end => {
                return read(u16, &self.ram, resolved_addr);
            },
            Addr.scratchpad_start...Addr.scratchpad_end => {
                const offset = resolved_addr - Addr.scratchpad_start;
                return read(u16, &self.scratchpad, offset);
            },
            Addr.bios_start...Addr.bios_end => {
                const offset = resolved_addr - Addr.bios_start;
                return read(u16, self.dev.bios.rom, offset);
            },
            else => {},
        }

        log.debug("readHalf: out of bounds: {x}", .{addr});
        return 0xacab;
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        switch (addr) {
            Addr.i_stat => return self.interrupt_status,
            Addr.i_mask => return self.interrupt_mask,
            else => {},
        }

        const resolved_addr = resolveAddr(addr);

        switch (resolved_addr) {
            Addr.ram_start...Addr.ram_end => {
                return read(u32, &self.ram, resolved_addr);
            },
            Addr.scratchpad_start...Addr.scratchpad_end => {
                const offset = resolved_addr - Addr.scratchpad_start;
                return read(u32, &self.scratchpad, offset);
            },
            Addr.bios_start...Addr.bios_end => {
                const offset = resolved_addr - Addr.bios_start;
                return read(u32, self.dev.bios.rom, offset);
            },
            Addr.gpu_start...Addr.gpu_end => {
                return self.dev.gpu.readWord(resolved_addr);
            },
            Addr.dma_start...Addr.dma_end => {
                return self.dev.dma.readWord(resolved_addr);
            },
            else => {},
        }

        // log.debug("readWord: out of bounds: {x}", .{addr});
        // self.debug_pause = true;

        return 0xacabacab;
    }

    pub fn writeByte(self: *@This(), addr: u32, value: u8) void {
        const resolved_addr = resolveAddr(addr);

        switch (resolved_addr) {
            Addr.ram_start...Addr.ram_end => {
                return writeToBuf(u8, &self.ram, resolved_addr, value);
            },
            Addr.scratchpad_start...Addr.scratchpad_end => {
                const offset = resolved_addr - Addr.scratchpad_start;
                return writeToBuf(u8, &self.scratchpad, offset, value);
            },
            else => {},
        }

        // log.warn("writeByte: out of bounds: {x}", .{addr})
    }

    pub fn writeHalf(self: *@This(), addr: u32, value: u16) void {
        const resolved_addr = resolveAddr(addr);

        switch (resolved_addr) {
            Addr.ram_start...Addr.ram_end => {
                return writeToBuf(u16, &self.ram, resolved_addr, value);
            },
            Addr.scratchpad_start...Addr.scratchpad_end => {
                const offset = resolved_addr - Addr.scratchpad_start;
                return writeToBuf(u16, &self.scratchpad, offset, value);
            },
            else => {},
        }

        // log.warn("writeHalf: out of bounds: {x}", .{addr});
    }

    pub fn writeWord(self: *@This(), addr: u32, v: u32) void {
        switch (addr) {
            Addr.i_stat => {
                self.interrupt_status = v & self.interrupt_mask;
                return;
            },
            Addr.i_mask => {
                self.interrupt_mask = v;
                return;
            },
            else => {},
        }

        const resolved_addr = resolveAddr(addr);

        switch (resolved_addr) {
            Addr.ram_start...Addr.ram_end => {
                return writeToBuf(u32, &self.ram, resolved_addr, v);
            },
            Addr.scratchpad_start...Addr.scratchpad_end => {
                const offset = resolved_addr - Addr.scratchpad_start;
                return writeToBuf(u32, &self.scratchpad, offset, v);
            },
            Addr.gpu_start...Addr.gpu_end => return self.dev.gpu.writeWord(resolved_addr, v),
            Addr.dma_start...Addr.dma_end => return self.dev.dma.writeWord(resolved_addr, v),
            else => {},
        }

        // self.debug_pause = true;
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
