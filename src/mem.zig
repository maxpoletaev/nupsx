const std = @import("std");
const builtin = @import("builtin");
const cpu_mod = @import("cpu.zig");
const gpu_mod = @import("gpu.zig");
const sched = @import("sched.zig");

const CPU = cpu_mod.CPU;
const GPU = gpu_mod.GPU;
const DMA = @import("dma.zig").DMA;
const SPU = @import("spu.zig").SPU;
const CDROM = @import("cdrom.zig").CDROM;
const MDEC = @import("mdec.zig").MDEC;
const Timers = @import("timer.zig").Timers;
const Joypad = @import("joy.zig").Joypad;

const expectEqual = std.testing.expectEqual;
const log = std.log.scoped(.mem);

const addr_mask_table = [_]u32{
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, // KUSEG
    0x7fffffff, // KSEG0
    0x1fffffff, // KSEG1
    0xffffffff, 0xffffffff, // KSEG2
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

const Error = error{
    InvalidBiosSize,
    FileReadError,
};

/// BIOS ROM (512KB)
pub const BIOS = struct {
    pub const addr_start: u32 = 0x1fc00000;
    pub const addr_end: u32 = 0x1fc7ffff;
    pub const bios_rom_size = 0x80000;

    allocator: std.mem.Allocator,
    rom: []u8,

    pub fn loadFromBuffer(allocator: std.mem.Allocator, buf: []const u8) Error!*@This() {
        if (buf.len != bios_rom_size) {
            log.err("invalid BIOS size: expected 512KB, got {d} bytes", .{buf.len});
            return Error.InvalidBiosSize;
        }

        const rom = allocator.alloc(u8, buf.len) catch @panic("OOM");
        @memcpy(rom, buf);

        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .rom = rom,
        };

        return self;
    }

    pub fn loadFromFile(allocator: std.mem.Allocator, path: []const u8) Error!*@This() {
        const file = std.fs.cwd().openFile(path, .{}) catch |err| {
            log.err("failed to open BIOS file: {}", .{err});
            return Error.FileReadError;
        };
        defer file.close();

        const file_size = file.getEndPos() catch |err| {
            log.err("failed to read BIOS file size: {}", .{err});
            return Error.FileReadError;
        };

        if (file_size != bios_rom_size) {
            log.err("invalid BIOS file size: expected 512KB, got {d} bytes", .{file_size});
            return Error.InvalidBiosSize;
        }

        const rom = allocator.alloc(u8, file_size) catch @panic("OOM");
        errdefer allocator.free(rom);

        const bytes_read = file.readAll(rom) catch |err| {
            log.err("failed to read BIOS file: {}", .{err});
            return Error.FileReadError;
        };

        if (bytes_read != file_size) {
            log.err("incomplete read of BIOS file: expected {d} bytes, got {d}", .{ file_size, bytes_read });
            return Error.FileReadError;
        }

        const self = allocator.create(@This()) catch @panic("OOM");
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

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
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

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
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
    mdec: *MDEC,
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

const AudioStreamWasm = struct {
    pub fn push(_: *@This(), _: [2]i16) void {}
    pub fn pop(_: *@This()) [2]i16 {
        return .{ 0, 0 };
    }
    pub fn signal(_: *@This()) void {}
};

const AudioStreamNative = struct {
    const capacity = 1024;

    buf: [capacity][2]i16 = undefined,
    head: std.atomic.Value(usize) = .init(0),
    tail: std.atomic.Value(usize) = .init(0),
    sem: std.Thread.Semaphore = .{},

    inline fn isFull(self: *@This()) bool {
        const tail = self.tail.load(.monotonic);
        const next_tail = (tail + 1) % capacity;
        return next_tail == self.head.load(.acquire);
    }

    pub fn push(self: *@This(), sample: [2]i16) void {
        while (self.isFull()) {
            self.sem.wait();
        }

        const tail = self.tail.load(.monotonic);
        const next_tail = (tail + 1) % capacity;

        self.buf[tail] = sample;
        self.tail.store(next_tail, .release);
    }

    pub fn pop(self: *@This()) [2]i16 {
        const head = self.head.load(.monotonic);
        if (head == self.tail.load(.acquire)) {
            return .{ 0, 0 };
        }

        const sample = self.buf[head];
        self.head.store((head + 1) % capacity, .release);
        return sample;
    }

    pub fn signal(self: *@This()) void {
        self.sem.post();
    }
};

pub const AudioStream = switch (builtin.target.cpu.arch) {
    .wasm32, .wasm64 => AudioStreamWasm,
    else => AudioStreamNative,
};

/// The main interconnect bus for all the devices within the console.
pub const Bus = struct {
    const page_size = 0x10000;
    const num_pages = 0x100000000 / page_size;

    const MemPage = [page_size]u8;

    allocator: std.mem.Allocator,
    dev: Devices,

    irq_mask: u32 = 0,
    irq_stat: u32 = 0,

    breakpoint: bool = false,

    read_pages: [num_pages]?*MemPage,
    write_pages: [num_pages]?*MemPage,

    audio_stream: AudioStream = .{},
    audio_counter: u32 = 0,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .dev = undefined,
            .read_pages = undefined,
            .write_pages = undefined,
        };

        @memset(&self.read_pages, null);
        @memset(&self.write_pages, null);

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn connect(self: *@This(), dev: Devices) void {
        self.dev = dev;
        self.initFastMem();
    }

    fn initFastMem(self: *@This()) void {
        // RAM: 2MB (0x200000 bytes) = 32 pages of 64KB each
        // 0x00000000-0x001fffff (pages 0x0000-0x001f)
        for (0..128) |page_idx| {
            const ram_offset = (page_idx & 0x1f) * page_size; // wraps every 32 pages
            const page_ptr: *MemPage = @ptrCast(&self.dev.ram.data[ram_offset]);
            self.read_pages[page_idx] = page_ptr;
            self.write_pages[page_idx] = page_ptr;
        }

        // BIOS: 512KB = 8 pages, read-only
        // Physical range: 0x1fc00000-0x1fc7ffff (pages 0x1fc0-0x1fc7)
        for (0..8) |page_idx| {
            const offset = page_idx * page_size;
            const page_ptr: *MemPage = @ptrCast(&self.dev.bios.rom[offset]);
            self.read_pages[0x1fc0 + page_idx] = page_ptr;
            // write_pages left as null for BIOS (read-only)
        }
    }

    pub inline fn updateCpuIrq(self: *@This()) void {
        const pending = (self.irq_stat & self.irq_mask) != 0;
        self.dev.cpu.requestInterrupt(pending);
    }

    pub fn setInterrupt(self: *@This(), v: u32) void {
        // if (v & Interrupt.tmr2 != 0) {
        //     log.debug("CPU interrupt set {x}", .{v});
        // }
        self.irq_stat |= v;
        self.updateCpuIrq();
    }

    pub fn tick(self: *@This()) void {
        const cyc = self.dev.cpu.tick();
        self.dev.gpu.tick(cyc);
        self.dev.cdrom.tick(cyc);
        self.dev.joy.tick(cyc);
        self.dev.timers.tick(cyc);

        self.audio_counter += cyc;
        if (self.audio_counter >= 768) {
            self.audio_counter -= 768;
            const sample = self.dev.spu.consumeAudioSample();
            const cd_sample = self.dev.cdrom.consumeAudioSample();
            self.audio_stream.push(.{
                sample[0] +| cd_sample[0],
                sample[1] +| cd_sample[1],
            });
        }
    }

    inline fn setIrqStat(self: *@This(), v: u32) void {
        self.irq_stat &= v; // (0=acknowledge, 1=no change)`
        self.updateCpuIrq();
    }

    inline fn setIrqMask(self: *@This(), v: u32) void {
        self.irq_mask = v;
        self.updateCpuIrq();
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const masked_addr = maskAddr(addr);

        // Fast path: check fastmem page table first
        const page_idx = masked_addr >> 16;
        const offset = masked_addr & 0xffff;
        if (self.read_pages[page_idx]) |page| {
            @branchHint(.likely);
            return readBuf(T, page, offset);
        }

        // Slow path: I/O and special regions
        return self.readSlow(T, masked_addr);
    }

    inline fn readSlow(self: *@This(), comptime T: type, masked_addr: u32) T {
        return switch (masked_addr) {
            addr_irq_stat => @as(T, @truncate(self.irq_stat)),
            addr_irq_mask => @as(T, @truncate(self.irq_mask)),

            GPU.addr_start...GPU.addr_end => self.dev.gpu.read(T, masked_addr),
            DMA.addr_start...DMA.addr_end => self.dev.dma.read(T, masked_addr),
            MDEC.addr_start...MDEC.addr_end => self.dev.mdec.read(T, masked_addr),
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

            0x1f801054 => 0x05, // sio1 status (padtest.exe stalls without this)

            else => blk: {
                // std.debug.panic("unhandled read ({s}) at {x}", .{ @typeName(T), masked_addr });
                log.err("unhandled read ({s}) at {x}", .{ @typeName(T), masked_addr });
                break :blk 0;
            },
        };
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const masked_addr = maskAddr(addr);

        // Fast path: check fastmem page table first
        const page_idx = masked_addr >> 16;
        const offset = masked_addr & 0xffff;

        if (self.write_pages[page_idx]) |page| {
            @branchHint(.likely);
            writeBuf(T, page, offset, v);
            self.dev.cpu.invalidateBlock(masked_addr); // sadly need that for self-modifying code
            return;
        }

        // Slow path: I/O and special regions
        self.writeSlow(T, addr, masked_addr, v);
    }

    inline fn writeSlow(self: *@This(), comptime T: type, addr: u32, masked_addr: u32, v: T) void {
        switch (masked_addr) {
            addr_irq_stat => self.setIrqStat(v),
            addr_irq_mask => self.setIrqMask(v),

            GPU.addr_start...GPU.addr_end => self.dev.gpu.write(T, masked_addr, v),
            DMA.addr_start...DMA.addr_end => self.dev.dma.write(T, masked_addr, v),
            MDEC.addr_start...MDEC.addr_end => self.dev.mdec.write(T, masked_addr, v),
            Timers.addr_start...Timers.addr_end => self.dev.timers.write(T, masked_addr, v),
            CDROM.addr_start...CDROM.addr_end => self.dev.cdrom.write(T, masked_addr, v),
            Scratchpad.addr_start...Scratchpad.addr_end => self.dev.scratchpad.write(T, masked_addr, v),
            SPU.addr_start...SPU.addr_end => self.dev.spu.write(T, masked_addr, v),
            Joypad.addr_start...Joypad.addr_end => self.dev.joy.write(T, masked_addr, v),

            0x1f801000...0x1f801023 => {}, // memctl
            0x1f801050...0x1f80105f => {}, // sio1
            0x1f801060...0x1f801063 => {}, // ramsize
            0xfffe0130...0xfffe0133 => {}, // cachectl
            0x1f000000...0x1f0000ff => {}, // expansion 1
            0x1f802000...0x1f802041 => {}, // expansion 2

            else => {
                // std.debug.panic("unhandled write ({s}) at {x} = {x}", .{ @typeName(T), masked_addr, v });
                log.warn("unhandled write ({s}) at {x} ({x}) = {x}", .{ @typeName(T), masked_addr, addr, v });
            },
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
