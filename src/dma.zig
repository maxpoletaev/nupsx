const std = @import("std");
const Bus = @import("mem.zig").Bus;

const log = std.log.scoped(.dma);

const gpu_gp0_addr = 0x1f801810;

const ChanCtrl = packed struct(u32) {
    to_device: u1, // transfer direction: 0=to ram, 1=to device
    addr_inc: u1, // maddr increment: 0=+4, 1=-4
    _pad0: u6,
    something: u1,
    sync_mode: u2, // 0=burst, 1=slice, 2=linked-list, 3=reserved
    _pad1: u5,
    chopping_dma: u3,
    _pad2: u1,
    chopping_cpu: u3,
    _pad3: u1,
    start: u1,
    _pad4: u3,
    force_start: u1,
    pause: u1,
    snooping: u1,
    _pad5: u1,
};

const BlockCtrl = packed struct(u32) {
    block_size: u16,
    block_count: u16,
};

const Channel = struct {
    maddr: u32,
    chan_ctrl: ChanCtrl,
    block_ctrl: BlockCtrl,

    pub fn init() Channel {
        return .{
            .maddr = 0,
            .chan_ctrl = std.mem.zeroes(ChanCtrl),
            .block_ctrl = std.mem.zeroes(BlockCtrl),
        };
    }
};

pub const DMA = struct {
    allocator: std.mem.Allocator,
    bus: *Bus,
    dma_ctrl: u32,
    int_ctrl: u32,

    ch_mdec_in: Channel,
    ch_mdec_out: Channel,
    ch_gpu: Channel,
    ch_cdrom: Channel,
    ch_spu: Channel,
    ch_pio: Channel,
    ch_otc: Channel,

    pub fn init(allocator: std.mem.Allocator, bus: *Bus) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .bus = bus,
            .dma_ctrl = 0,
            .int_ctrl = 0,
            .ch_mdec_in = .init(),
            .ch_mdec_out = .init(),
            .ch_gpu = .init(),
            .ch_cdrom = .init(),
            .ch_spu = .init(),
            .ch_pio = .init(),
            .ch_otc = .init(),
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    fn channelFromIndex(self: *@This(), idx: u8) *Channel {
        return switch (idx) {
            0 => &self.ch_mdec_in,
            1 => &self.ch_mdec_out,
            2 => &self.ch_gpu,
            3 => &self.ch_cdrom,
            4 => &self.ch_spu,
            5 => &self.ch_pio,
            6 => &self.ch_otc,
            else => unreachable,
        };
    }

    pub fn writeWord(self: *@This(), offset: u32, v: u32) void {
        switch (offset) {
            0x70 => self.dma_ctrl = v,
            0x74 => self.int_ctrl = v,
            else => {
                const reg_id = @as(u8, @truncate((offset >> 2) & 0x3));
                const chan_id = @as(u8, @truncate((offset >> 4) & 0x7));
                self.updateChannel(chan_id, reg_id, v);
            },
        }
    }

    fn updateChannel(self: *@This(), chan_id: u8, reg_id: u8, v: u32) void {
        const chan = self.channelFromIndex(chan_id);

        switch (reg_id) {
            0 => chan.maddr = v,
            1 => chan.block_ctrl = @bitCast(v),
            2 => chan.chan_ctrl = @bitCast(v),
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
        const ctrl = &self.ch_gpu.chan_ctrl;

        if (ctrl.start == 1) {
            switch (ctrl.sync_mode) {
                1 => self.doGpuSyncModeSlice(),
                2 => self.doGpuSyncModeLinkedList(),
                else => std.debug.panic("sync_mode={d}", .{ctrl.sync_mode}),
            }
            ctrl.start = 0;
        }
    }

    fn doGpuSyncModeSlice(self: *@This()) void {
        const ctrl = &self.ch_gpu.chan_ctrl;
        const blk = &self.ch_gpu.block_ctrl;
        const len = blk.block_size * blk.block_count;
        const addr_inc = @as(u32, @bitCast(@as(i32, if (ctrl.addr_inc == 1) -4 else 4)));

        if (ctrl.to_device == 1) {
            for (0..len) |_| {
                const v = self.bus.readWord(self.ch_gpu.maddr);
                self.bus.dev.gpu.gp0write(v);
                self.ch_gpu.maddr, _ = @addWithOverflow(self.ch_gpu.maddr, addr_inc);
            }
        } else {
            for (0..len) |_| {
                const v = self.bus.dev.gpu.readGpuread();
                self.bus.writeWord(self.ch_gpu.maddr, v);
                self.ch_gpu.maddr, _ = @addWithOverflow(self.ch_gpu.maddr, addr_inc);
            }
        }
    }

    fn doGpuSyncModeLinkedList(self: *@This()) void {
        var addr = self.ch_gpu.maddr;

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
        const ctrl = &self.ch_otc.chan_ctrl;
        const blk = &self.ch_otc.block_ctrl;

        if (ctrl.start == 1) {
            var addr = self.ch_otc.maddr;
            const len = blk.block_size;

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
