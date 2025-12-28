const std = @import("std");
const mem_mod = @import("mem.zig");
const bits = @import("bits.zig");

const Bus = mem_mod.Bus;
const log = std.log.scoped(.dma);

const ChanCtrl = packed struct(u32) {
    direction: enum(u1) { to_ram = 0, to_device = 1 },
    addr_inc: enum(u1) { @"+4" = 0, @"-4" = 1 },
    _pad0: u6,
    something: u1,
    sync_mode: enum(u2) { burst = 0, slice = 1, linked_list = 2, reserved = 3 },
    _pad1: u5,
    chopping_dma: u3,
    _pad2: u1,
    chopping_cpu: u3,
    _pad3: u1,
    start: u1,
    _pad4: u3,
    force_start: bool,
    pause: bool,
    snooping: bool,
    _pad5: u1,
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

pub const DMA = struct {
    pub const addr_start: u32 = 0x1f801080;
    pub const addr_end: u32 = 0x1f8010ff;
    pub const addr_dpcr: u32 = 0x1f8010f0;
    pub const addr_dicr: u32 = 0x1f8010f4;

    allocator: std.mem.Allocator,
    bus: *Bus,
    dpcr: u32,
    dicr: u32,

    mdec_in: Channel,
    mdec_out: Channel,
    gpu: Channel,
    cdrom: Channel,
    spu: Channel,
    pio: Channel,
    otc: Channel,

    pub fn init(allocator: std.mem.Allocator, bus: *Bus) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .bus = bus,
            .dpcr = 0,
            .dicr = 0,
            .mdec_in = .init(),
            .mdec_out = .init(),
            .gpu = .init(),
            .cdrom = .init(),
            .spu = .init(),
            .pio = .init(),
            .otc = .init(),
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    fn getChannel(self: *@This(), chan_id: u8) *Channel {
        return switch (chan_id) {
            0 => &self.mdec_in,
            1 => &self.mdec_out,
            2 => &self.gpu,
            3 => &self.cdrom,
            4 => &self.spu,
            5 => &self.pio,
            6 => &self.otc,
            else => unreachable,
        };
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        switch (addr) {
            addr_dpcr => return self.dpcr,
            addr_dicr => return self.dicr,

            else => {
                const offset = addr - addr_start;
                const reg_id = bits.field(offset, 2, u2);
                const chan_id = bits.field(offset, 4, u3);
                return self.readChannelReg(chan_id, reg_id);
            },
        }
    }

    pub fn writeWord(self: *@This(), addr: u32, v: u32) void {
        switch (addr) {
            addr_dpcr => self.dpcr = v,
            addr_dicr => self.dicr = v,

            else => {
                const offset = addr - addr_start;
                const reg_id = bits.field(offset, 2, u2);
                const chan_id = bits.field(offset, 4, u3);
                self.writeChannelReg(chan_id, reg_id, v);
            },
        }
    }

    fn readChannelReg(self: *@This(), chan_id: u8, reg_id: u8) u32 {
        const chan = self.getChannel(chan_id);

        return switch (reg_id) {
            0 => chan.maddr,
            1 => @bitCast(chan.block),
            2 => @bitCast(chan.ctrl),
            else => unreachable,
        };
    }

    fn writeChannelReg(self: *@This(), chan_id: u8, reg_id: u8, v: u32) void {
        const chan = self.getChannel(chan_id);

        switch (reg_id) {
            0 => chan.maddr = v,
            1 => chan.block = @bitCast(v),
            2 => chan.ctrl = @bitCast(v),
            else => unreachable,
        }

        if (reg_id == 2) {
            switch (chan_id) {
                2 => self.doGpu(),
                6 => self.doOtc(),
                else => {},
            }
        }
    }

    fn doGpu(self: *@This()) void {
        const ctrl = &self.gpu.ctrl;

        if (ctrl.start == 1) {
            switch (ctrl.sync_mode) {
                .slice => self.doGpuSyncModeSlice(),
                .linked_list => self.doGpuSyncModeLinkedList(),
                else => std.debug.panic("sync_mode={d}", .{ctrl.sync_mode}),
            }
            ctrl.start = 0;
        }
    }

    fn doGpuSyncModeSlice(self: *@This()) void {
        const ctrl = &self.gpu.ctrl;
        const block = &self.gpu.block;
        const len = block.size * block.count;

        const addr_inc = @as(u32, @bitCast(switch (ctrl.addr_inc) {
            .@"+4" => @as(i32, 4),
            .@"-4" => @as(i32, -4),
        }));

        switch (ctrl.direction) {
            .to_device => for (0..len) |_| {
                const v = self.bus.readWord(self.gpu.maddr);
                self.bus.dev.gpu.gp0write(v);
                self.gpu.maddr, _ = @addWithOverflow(self.gpu.maddr, addr_inc);
            },
            .to_ram => for (0..len) |_| {
                const v = self.bus.dev.gpu.readGpuread();
                self.bus.writeWord(self.gpu.maddr, v);
                self.gpu.maddr, _ = @addWithOverflow(self.gpu.maddr, addr_inc);
            },
        }
    }

    fn doGpuSyncModeLinkedList(self: *@This()) void {
        var addr = self.gpu.maddr;

        while (addr != 0xffffff) {
            const hdr = self.bus.readWord(addr);
            const word_count = hdr >> 24;

            for (0..word_count) |i| {
                const data_addr = @as(u32, @intCast(addr + 4 * (i + 1)));
                const v = self.bus.readWord(data_addr);
                self.bus.dev.gpu.gp0write(v);
            }

            addr = hdr & 0xffffff;
        }
    }

    fn doOtc(self: *@This()) void {
        const ctrl = &self.otc.ctrl;
        const block = &self.otc.block;

        if (ctrl.start == 1) {
            var addr = self.otc.maddr;
            const len = block.size;

            for (0..len) |i| {
                const next_addr = (addr - 4) & 0xffffff;
                const hdr: u32 = if (i == len - 1) 0xffffff else next_addr;
                self.bus.writeWord(addr, hdr);
                addr = next_addr;
            }

            ctrl.start = 0;
        }
    }
};
