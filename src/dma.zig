const std = @import("std");
const mem_mod = @import("mem.zig");
const bits = @import("bits.zig");

const Bus = mem_mod.Bus;
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

    pub fn init() Channel {
        return .{
            .maddr = 0,
            .ctrl = std.mem.zeroes(ChanCtrl),
            .block = std.mem.zeroes(BlockCtrl),
        };
    }
};

const IntterruptReg = packed struct(u32) {
    irq_mode: u7, // 0-6 (0=after full transfer, 1=after each block)
    _pad0: u8, // 7-14
    bus_error: bool, // 15
    irq_mask: u7, // 16-22
    master_enable: bool, // 23
    irq_flags: u7, // 24-30
    master_irq: bool, // 31

    inline fn updateMaster(self: *IntterruptReg) void {
        const irq_pending = (self.irq_flags & self.irq_mask) != 0;
        self.master_irq = self.bus_error or (self.master_enable and irq_pending);
    }
};

const ControlReg = packed struct(u32) {
    chan_data: u28, // 0-27 (4 bits per channel: u3 priority + u1 master enable)
    cpu_mem_priority: u3, // 28-30
    _pad0: u1, // 31

    inline fn getChanPriority(self: *ControlReg, chan_id: u3) u3 {
        const off = @as(u8, chan_id) * 4;
        return bits.field(self.chan_data, off, u3);
    }

    inline fn getChanMasterEnable(self: *ControlReg, chan_id: u3) bool {
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
    irq: bool,

    pub fn init(allocator: std.mem.Allocator, bus: *Bus) !*@This() {
        const self = try allocator.create(@This());

        self.* = .{
            .allocator = allocator,
            .bus = bus,
            .dpcr = @bitCast(@as(u32, 0x07654321)),
            .dicr = std.mem.zeroes(IntterruptReg),
            .channels = [_]Channel{
                Channel.init(),
                Channel.init(),
                Channel.init(),
                Channel.init(),
                Channel.init(),
                Channel.init(),
                Channel.init(),
            },
            .irq = false,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn consumeIrq(self: *@This()) bool {
        const v = self.irq;
        self.irq = false;
        return v;
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const v = switch (addr) {
            addr_dpcr => @as(u32, @bitCast(self.dpcr)),
            addr_dicr => @as(u32, @bitCast(self.dicr)),
            else => blk: {
                const offset = addr - addr_start;
                const reg_id = bits.field(offset, 2, u2);
                const chan_id = bits.field(offset, 4, u3);
                break :blk self.readChannelReg(chan_id, reg_id);
            },
        };
        return @truncate(v);
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        switch (addr) {
            addr_dpcr => {
                self.dpcr = @bitCast(@as(u32, v));
            },
            addr_dicr => {
                var new_dicr: IntterruptReg = @bitCast(@as(u32, v));
                new_dicr.irq_flags = self.dicr.irq_flags & ~new_dicr.irq_flags; // write 1 to clear
                new_dicr.master_enable = self.dicr.master_enable; // bit 31 is read-only
                self.dicr = new_dicr;
                self.dicr.updateMaster();
            },
            else => {
                const offset = addr - addr_start;
                const reg_id = bits.field(offset, 2, u2);
                const chan_id = bits.field(offset, 4, u3);
                self.writeChannelReg(chan_id, reg_id, v);
            },
        }
    }

    fn setIrqOnCompletion(self: *@This(), chan_id: u3) void {
        const chan_mask = @as(u7, 1) << chan_id;
        const pending = self.dicr.irq_mask & chan_mask != 0 and
            self.dicr.irq_mode & chan_mask == 0;
        if (pending) {
            self.dicr.irq_flags |= chan_mask;
            self.dicr.updateMaster();

            if (self.dicr.master_irq) {
                self.irq = true;
            }
        }
    }

    fn setIrqOnBlockReady(self: *@This(), chan_id: u3) void {
        const chan_mask = @as(u7, 1) << chan_id;
        const pending = self.dicr.irq_mask & chan_mask != 0 and
            (self.dicr.irq_mode & chan_mask != 0);
        if (pending) {
            self.dicr.irq_flags |= chan_mask;
            self.dicr.updateMaster();

            if (self.dicr.master_irq) {
                self.irq = true;
            }
        }
    }

    fn readChannelReg(self: *@This(), chan_id: u8, reg_id: u8) u32 {
        const chan = &self.channels[chan_id];

        return switch (reg_id) {
            0 => chan.maddr,
            1 => @bitCast(chan.block),
            2 => @bitCast(chan.ctrl),
            else => unreachable,
        };
    }

    fn writeChannelReg(self: *@This(), chan_id: u8, reg_id: u8, v: u32) void {
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
                6 => self.doOtc(),
                else => std.debug.panic("DMA channel {d} start not implemented", .{chan_id}),
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

        chan.ctrl.resetActive();

        self.setIrqOnCompletion(ChanId.gpu);
    }

    fn doGpuSyncModeSlice(self: *@This()) void {
        const chan = &self.channels[ChanId.gpu];
        const transfer_len = chan.block.size * chan.block.count;

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
                    self.setIrqOnBlockReady(ChanId.gpu);
                }
            },
            .to_ram => for (0..transfer_len) |i| {
                const v = self.bus.dev.gpu.readGpuread();
                self.bus.write(u32, chan.maddr, v);
                chan.maddr +%= addr_inc;

                if ((i + 1) % chan.block.size == 0) {
                    self.setIrqOnBlockReady(ChanId.gpu);
                }
            },
        }
    }

    fn doGpuSyncModeLinkedList(self: *@This()) void {
        const chan = &self.channels[ChanId.gpu];

        var addr = chan.maddr;

        while (addr != 0xffffff) {
            const hdr = self.bus.read(u32, addr);
            const word_count = hdr >> 24;

            for (0..word_count) |i| {
                const data_addr = @as(u32, @intCast(addr + 4 * (i + 1)));
                const v = self.bus.read(u32, data_addr);
                self.bus.dev.gpu.gp0write(v);
            }

            self.setIrqOnBlockReady(ChanId.gpu);

            addr = hdr & 0xffffff;
        }
    }

    fn doCdrom(self: *@This()) void {
        const chan = &self.channels[ChanId.cdrom];

        if (!chan.ctrl.isActive()) return;

        const addr_inc = @as(u32, @bitCast(switch (chan.ctrl.addr_inc) {
            .incr => @as(i32, 4),
            .decr => @as(i32, -4),
        }));

        const transfer_len = chan.block.size * chan.block.count;

        switch (chan.ctrl.direction) {
            .to_ram => for (0..transfer_len) |i| {
                const v = self.bus.dev.cdrom.consumeSectorData(u32);
                self.bus.write(u32, chan.maddr, v);
                chan.maddr +%= addr_inc;

                if ((i + 1) % chan.block.size == 0) {
                    self.setIrqOnBlockReady(ChanId.cdrom);
                }
            },
            .to_device => @panic("not implemented"),
        }

        chan.ctrl.resetActive();

        self.setIrqOnCompletion(ChanId.cdrom);
    }

    fn doOtc(self: *@This()) void {
        const chan = &self.channels[ChanId.otc];

        if (!chan.ctrl.isActive()) return;

        var addr = chan.maddr;
        const len = chan.block.size;

        for (0..len) |i| {
            const next_addr = (addr - 4) & 0xffffff;
            const hdr: u32 = if (i == len - 1) 0xffffff else next_addr;
            self.bus.write(u32, addr, hdr);
            addr = next_addr;
        }

        chan.ctrl.resetActive();
    }
};
