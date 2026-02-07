const std = @import("std");
const mem = @import("mem.zig");
const bits = @import("bits.zig");

const Bus = mem.Bus;
const Interrupt = mem.Interrupt;

const log = std.log.scoped(.dma);

const SyncMode = enum(u2) { burst = 0, slice = 1, linked_list = 2 };

const ChanCtrl = packed struct(u32) {
    direction: enum(u1) { to_ram = 0, to_device = 1 }, // 0
    addr_inc: enum(u1) { incr = 0, decr = 1 }, // 1
    _pad0: u6, // 2-7
    something: u1, // 8
    sync_mode: SyncMode, // 9-10
    _pad1: u5, // 11-15
    chopping_dma: u3, // 16-18
    _pad2: u1, // 19
    chopping_cpu: u3, // 20-22
    _pad3: u1, // 23
    start: bool, // 24
    _pad4: u3, // 25-27
    force_start: bool, // 28
    pause: bool, // 29
    snooping: bool, // 30
    _pad5: u1, // 31

    pub fn isActive(self: *ChanCtrl) bool {
        return (self.start or self.force_start) and !self.pause;
    }

    pub fn resetActive(self: *ChanCtrl) void {
        self.start = false;
        self.force_start = false;
    }
};

const BlockCtrl = packed struct(u32) {
    size: u16,
    count: u16,
};

const Channel = struct {
    maddr: u32,
    ctrl: ChanCtrl,
    block: BlockCtrl,
};

const IntterruptReg = packed struct(u32) {
    irq_mode: u7, // 0-6 (0=after full transfer, 1=after each block)
    _pad0: u8, // 7-14
    bus_error: bool, // 15
    irq_mask: u7, // 16-22
    master_enable: bool, // 23
    irq_flags: u7, // 24-30
    master_irq: bool, // 31

    inline fn updateMasterIrq(self: *IntterruptReg) void {
        const irq_pending = (self.irq_flags & self.irq_mask) != 0;
        self.master_irq = self.bus_error or (self.master_enable and irq_pending);
    }
};

const ControlReg = packed struct(u32) {
    chan_data: u28, // 0-27 (4 bits per channel: u3 priority + u1 master enable)
    cpu_mem_priority: u3, // 28-30
    _pad0: u1, // 31

    pub inline fn getChanPriority(self: *ControlReg, chan_id: u3) u3 {
        const off = @as(u8, chan_id) * 4;
        return bits.field(self.chan_data, off, u3);
    }

    pub inline fn getChanMasterEnable(self: *ControlReg, chan_id: u3) bool {
        const off = @as(u8, chan_id) * 4 + 3;
        return bits.field(self.chan_data, off, u1) == 1;
    }
};

const ChanId = opaque {
    pub const mdec_in: u3 = 0;
    pub const mdec_out: u3 = 1;
    pub const gpu: u3 = 2;
    pub const cdrom: u3 = 3;
    pub const spu: u3 = 4;
    pub const pio: u3 = 5;
    pub const otc: u3 = 6;
};

pub const DMA = struct {
    pub const addr_start: u32 = 0x1f801080;
    pub const addr_end: u32 = 0x1f8010ff;
    pub const addr_dpcr: u32 = 0x1f8010f0;
    pub const addr_dicr: u32 = 0x1f8010f4;

    allocator: std.mem.Allocator,
    bus: *Bus,
    dpcr: ControlReg,
    dicr: IntterruptReg,
    channels: [7]Channel,

    pub fn init(allocator: std.mem.Allocator, bus: *Bus) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .bus = bus,
            .dpcr = @bitCast(@as(u32, 0x07654321)),
            .dicr = std.mem.zeroes(IntterruptReg),
            .channels = std.mem.zeroes([7]Channel),
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const aligned_addr = addr & ~@as(u32, 3);
        const v = switch (aligned_addr) {
            addr_dpcr => @as(u32, @bitCast(self.dpcr)),
            addr_dicr => @as(u32, @bitCast(self.dicr)),

            else => blk: {
                const offset = aligned_addr - addr_start;
                const reg_id = bits.field(offset, 2, u2);
                const chan_id = bits.field(offset, 4, u3);
                break :blk self.readChannelReg(chan_id, reg_id);
            },
        };

        return partalRead(T, v, addr);
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const aligned_addr = addr & ~@as(u32, 3);
        switch (aligned_addr) {
            addr_dpcr => {
                self.dpcr = @bitCast(partalWrite(T, @bitCast(self.dpcr), addr, v));
            },
            addr_dicr => {
                var new_dicr: IntterruptReg = @bitCast(partalWrite(T, @bitCast(self.dicr), addr, v));
                new_dicr.irq_flags = self.dicr.irq_flags & ~new_dicr.irq_flags; // write 1 to clear
                new_dicr.master_irq = self.dicr.master_irq; // bit 31 is read-only
                self.dicr = @bitCast(new_dicr);
                self.dicr.updateMasterIrq();
            },
            else => {
                const offset = aligned_addr - addr_start;
                const reg_id = bits.field(offset, 2, u2);
                const chan_id = bits.field(offset, 4, u3);
                const old_val = self.readChannelReg(chan_id, reg_id);
                const new_val = partalWrite(T, old_val, addr, v);
                self.writeChannelReg(chan_id, reg_id, new_val);
            },
        }
    }

    fn setChannelIrq(self: *@This(), chan_id: u3, comptime mode: enum { block, full_transfer }) void {
        const chan_mask = @as(u7, 1) << chan_id;

        if (self.dicr.irq_mask & chan_mask == 0) {
            return;
        }

        const mode_bit = switch (mode) {
            .block => self.dicr.irq_mode & chan_mask != 0,
            .full_transfer => self.dicr.irq_mode & chan_mask == 0,
        };

        if (mode_bit) {
            self.dicr.irq_flags |= chan_mask;
            self.dicr.updateMasterIrq();

            if (self.dicr.master_irq) {
                self.bus.setInterrupt(Interrupt.dma);
            }
        }
    }

    fn readChannelReg(self: *@This(), chan_id: u8, reg_id: u8) u32 {
        if (chan_id >= 7) {
            log.warn("read from invalid dma channel: {d}", .{chan_id});
            return 0;
        }

        const chan = &self.channels[chan_id];

        return switch (reg_id) {
            0 => chan.maddr,
            1 => @bitCast(chan.block),
            2 => @bitCast(chan.ctrl),
            else => unreachable,
        };
    }

    fn writeChannelReg(self: *@This(), chan_id: u8, reg_id: u8, v: u32) void {
        if (chan_id >= 7) {
            log.warn("write to invalid dma channel: {d}", .{chan_id});
            return;
        }

        const chan = &self.channels[chan_id];

        switch (reg_id) {
            0 => chan.maddr = v,
            1 => chan.block = @bitCast(v),
            2 => chan.ctrl = @bitCast(v),
            else => unreachable,
        }

        if (reg_id == 2) {
            switch (chan_id) {
                2 => self.doGpu(),
                3 => self.doCdrom(),
                4 => self.doSpu(),
                6 => self.doOtc(),
                else => {
                    // std.debug.panic("DMA channel {d} start not implemented", .{chan_id});
                    log.warn("DMA channel {d} start not implemented", .{chan_id});
                },
            }
        }
    }

    fn doGpu(self: *@This()) void {
        const chan = &self.channels[ChanId.gpu];

        if (!chan.ctrl.isActive()) return;

        switch (chan.ctrl.sync_mode) {
            .slice => self.doGpuSyncModeSlice(),
            .linked_list => self.doGpuSyncModeLinkedList(),
            else => std.debug.panic("not implemented gpu sync mode: {d}", .{chan.ctrl.sync_mode}),
        }
    }

    fn doGpuSyncModeSlice(self: *@This()) void {
        const chan = &self.channels[ChanId.gpu];

        const transfer_len = @as(u32, chan.block.size) * @as(u32, chan.block.count);

        log.debug("DMA GPU transfer slice, size={x}", .{transfer_len});

        const addr_inc = @as(u32, @bitCast(switch (chan.ctrl.addr_inc) {
            .incr => @as(i32, 4),
            .decr => @as(i32, -4),
        }));

        switch (chan.ctrl.direction) {
            .to_device => for (0..transfer_len) |i| {
                const v = self.bus.read(u32, chan.maddr);
                self.bus.dev.gpu.gp0write(v);
                chan.maddr +%= addr_inc;

                if ((i + 1) % chan.block.size == 0) {
                    self.setChannelIrq(ChanId.gpu, .block);
                }
            },
            .to_ram => for (0..transfer_len) |i| {
                const v = self.bus.dev.gpu.readGpuread();
                self.bus.write(u32, chan.maddr, v);
                chan.maddr +%= addr_inc;

                if ((i + 1) % chan.block.size == 0) {
                    self.setChannelIrq(ChanId.gpu, .block);
                }
            },
        }

        chan.ctrl.resetActive();

        self.setChannelIrq(ChanId.gpu, .full_transfer);
    }

    fn doGpuSyncModeLinkedList(self: *@This()) void {
        const chan = &self.channels[ChanId.gpu];

        var addr = chan.maddr & 0x1ffffc;

        log.debug("DMA GPU transfer linked list, start_addr={x}", .{addr});

        while (addr != 0xffffff) {
            const hdr = self.bus.read(u32, addr);
            const word_count = hdr >> 24;

            for (0..word_count) |i| {
                const data_addr = @as(u32, @intCast(addr + 4 * (i + 1)));
                const v = self.bus.read(u32, data_addr);
                self.bus.dev.gpu.gp0write(v);
            }

            self.setChannelIrq(ChanId.gpu, .block);

            addr = hdr & 0xffffff;
        }

        chan.ctrl.resetActive();

        self.setChannelIrq(ChanId.gpu, .full_transfer);
    }

    fn doSpu(self: *@This()) void {
        const chan = &self.channels[ChanId.spu];

        if (!chan.ctrl.isActive()) return;

        const addr_inc = @as(u32, @bitCast(switch (chan.ctrl.addr_inc) {
            .incr => @as(i32, 4),
            .decr => @as(i32, -4),
        }));

        const transfer_len = @as(u32, chan.block.size) * @as(u32, chan.block.count);

        log.debug("DMA SPU transfer, size={x}", .{transfer_len});

        switch (chan.ctrl.direction) {
            .to_device => for (0..transfer_len) |i| {
                const v = self.bus.read(u32, chan.maddr);
                self.bus.dev.spu.writeData(@truncate(v >> 0));
                self.bus.dev.spu.writeData(@truncate(v >> 16));
                chan.maddr +%= addr_inc;

                if ((i + 1) % chan.block.size == 0) {
                    self.setChannelIrq(ChanId.spu, .block);
                }
            },
            .to_ram => for (0..transfer_len) |i| {
                const v = self.bus.dev.spu.readData();
                self.bus.write(u32, chan.maddr, v);
                chan.maddr +%= addr_inc;

                if ((i + 1) % chan.block.size == 0) {
                    self.setChannelIrq(ChanId.spu, .block);
                }
            },
        }

        self.setChannelIrq(ChanId.spu, .full_transfer);

        chan.ctrl.resetActive();
    }

    fn doCdrom(self: *@This()) void {
        const chan = &self.channels[ChanId.cdrom];

        if (!chan.ctrl.isActive()) return;

        const addr_inc = @as(u32, @bitCast(switch (chan.ctrl.addr_inc) {
            .incr => @as(i32, 4),
            .decr => @as(i32, -4),
        }));

        const transfer_len = @as(u32, chan.block.size) * @as(u32, chan.block.count);

        log.debug("DMA CDROM transfer, size={x}", .{transfer_len});

        switch (chan.ctrl.direction) {
            .to_ram => for (0..transfer_len) |i| {
                const v = self.bus.dev.cdrom.consumeData(u32);
                self.bus.write(u32, chan.maddr, v);
                chan.maddr +%= addr_inc;

                if ((i + 1) % chan.block.size == 0) {
                    self.setChannelIrq(ChanId.cdrom, .block);
                }
            },
            .to_device => @panic("not implemented"),
        }

        chan.ctrl.resetActive();

        self.setChannelIrq(ChanId.cdrom, .full_transfer);
    }

    fn doOtc(self: *@This()) void {
        const chan = &self.channels[ChanId.otc];

        if (!chan.ctrl.isActive()) return;

        const transfer_len = chan.block.size;

        log.debug("DMA OTC transfer, size={x}", .{transfer_len});

        var addr = chan.maddr;

        for (0..transfer_len) |i| {
            const next_addr = (addr - 4) & 0xffffff;
            const hdr: u32 = if (i == transfer_len - 1) 0xffffff else next_addr;
            self.bus.write(u32, addr, hdr);
            addr = next_addr;
        }

        chan.ctrl.resetActive();

        self.setChannelIrq(ChanId.otc, .full_transfer);
    }
};

inline fn partalRead(comptime T: type, base: u32, addr: u32) T {
    return switch (T) {
        u8 => blk: {
            const shift: u5 = @intCast((addr & 3) * 8);
            break :blk @truncate(base >> shift);
        },
        u16 => blk: {
            const shift: u5 = @intCast((addr & 2) * 8);
            break :blk @truncate(base >> shift);
        },
        u32 => base,
        else => unreachable,
    };
}

inline fn partalWrite(comptime T: type, base: u32, addr: u32, value: T) u32 {
    return switch (T) {
        u8 => blk: {
            const shift: u5 = @intCast((addr & 3) * 8);
            const mask: u32 = @as(u32, 0xff) << shift;
            break :blk (base & ~mask) | (@as(u32, value) << shift);
        },
        u16 => blk: {
            const shift: u5 = @intCast((addr & 2) * 8);
            const mask: u32 = @as(u32, 0xffff) << shift;
            break :blk (base & ~mask) | (@as(u32, value) << shift);
        },
        u32 => value,
        else => unreachable,
    };
}
