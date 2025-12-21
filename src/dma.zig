const std = @import("std");
const mem = @import("mem.zig");
const Bus = mem.Bus;
const bits = @import("bits.zig");

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
    dpcr: u32,
    dicr: u32,

    chan_mdec_in: Channel,
    chan_mdec_out: Channel,
    chan_gpu: Channel,
    chan_cdrom: Channel,
    chan_spu: Channel,
    chan_pio: Channel,
    chan_otc: Channel,

    pub fn init(allocator: std.mem.Allocator, bus: *Bus) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .bus = bus,
            .dpcr = 0,
            .dicr = 0,
            .chan_mdec_in = .init(),
            .chan_mdec_out = .init(),
            .chan_gpu = .init(),
            .chan_cdrom = .init(),
            .chan_spu = .init(),
            .chan_pio = .init(),
            .chan_otc = .init(),
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    fn channelFromIndex(self: *@This(), idx: u8) *Channel {
        return switch (idx) {
            0 => &self.chan_mdec_in,
            1 => &self.chan_mdec_out,
            2 => &self.chan_gpu,
            3 => &self.chan_cdrom,
            4 => &self.chan_spu,
            5 => &self.chan_pio,
            6 => &self.chan_otc,
            else => unreachable,
        };
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        switch (addr) {
            mem.Addr.dma_dpcr => return self.dpcr,
            mem.Addr.dma_dicr => return self.dicr,
            else => {
                log.debug("readWord: dma channel register read: {x}", .{addr});
                return 0;
            },
        }
    }

    pub fn writeWord(self: *@This(), addr: u32, v: u32) void {
        switch (addr) {
            mem.Addr.dma_dpcr => self.dpcr = v,
            mem.Addr.dma_dicr => self.dicr = v,
            else => {
                const offset = addr - mem.Addr.dma_start;
                const reg_id = bits.field(offset, 2, u2);
                const chan_id = bits.field(offset, 4, u3);
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
        const ctrl = &self.chan_gpu.chan_ctrl;

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
        const ctrl = &self.chan_gpu.chan_ctrl;
        const block = &self.chan_gpu.block_ctrl;
        const len = block.block_size * block.block_count;

        const addr_inc = @as(u32, @bitCast(switch (ctrl.addr_inc) {
            .@"+4" => @as(i32, 4),
            .@"-4" => @as(i32, -4),
        }));

        switch (ctrl.direction) {
            .to_device => for (0..len) |_| {
                const v = self.bus.readWord(self.chan_gpu.maddr);
                self.bus.dev.gpu.gp0write(v);
                self.chan_gpu.maddr, _ = @addWithOverflow(self.chan_gpu.maddr, addr_inc);
            },
            .to_ram => for (0..len) |_| {
                const v = self.bus.dev.gpu.readGpuread();
                self.bus.writeWord(self.chan_gpu.maddr, v);
                self.chan_gpu.maddr, _ = @addWithOverflow(self.chan_gpu.maddr, addr_inc);
            },
        }
    }

    fn doGpuSyncModeLinkedList(self: *@This()) void {
        var addr = self.chan_gpu.maddr;

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
        const ctrl = &self.chan_otc.chan_ctrl;
        const block = &self.chan_otc.block_ctrl;

        if (ctrl.start == 1) {
            var addr = self.chan_otc.maddr;
            const len = block.block_size;

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
