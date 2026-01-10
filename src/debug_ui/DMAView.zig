const std = @import("std");
const zgui = @import("zgui");
const DMA = @import("../dma.zig").DMA;

allocator: std.mem.Allocator,
dma: *DMA,

const channel_names = [7][:0]const u8{
    "MDEC IN",
    "MDEC OUT",
    "GPU",
    "CDROM",
    "SPU",
    "PIO",
    "OTC",
};

pub fn init(allocator: std.mem.Allocator, dma: *DMA) !*@This() {
    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .dma = dma,
    };
    return self;
}

pub fn deinit(self: *@This()) void {
    const allocator = self.allocator;
    allocator.destroy(self);
}

pub fn update(self: *@This()) void {
    if (zgui.begin("DMA", .{})) {
        // Global registers
        const dpcr_val: u32 = @bitCast(self.dma.dpcr);
        const dicr_val: u32 = @bitCast(self.dma.dicr);

        zgui.text("DPCR: 0x{x:0>8}  DICR: 0x{x:0>8}  IRQ: {s}", .{
            dpcr_val,
            dicr_val,
            if (self.dma.irq_pending) "yes" else "no",
        });

        zgui.text("Master Enable: {s}  Master IRQ: {s}  Bus Error: {s}", .{
            if (self.dma.dicr.master_enable) "yes" else "no",
            if (self.dma.dicr.master_irq) "yes" else "no",
            if (self.dma.dicr.bus_error) "yes" else "no",
        });

        zgui.separator();

        // Channel IRQ table header
        zgui.text("Channel:    ", .{});
        for (channel_names) |name| {
            zgui.sameLine(.{});
            zgui.text("{s: <9}", .{name});
        }

        // IRQ Mask row
        zgui.text("IRQ Mask:   ", .{});
        for (0..7) |j| {
            const mask = (@as(u7, 1) << @intCast(j));
            const enabled = (self.dma.dicr.irq_mask & mask) != 0;
            zgui.sameLine(.{});
            if (enabled) {
                zgui.textColored(.{ 0.0, 1.0, 0.0, 1.0 }, "[x]      ", .{});
            } else {
                zgui.text("[ ]      ", .{});
            }
        }

        // IRQ Flags row
        zgui.text("IRQ Flag:   ", .{});
        for (0..7) |j| {
            const mask = (@as(u7, 1) << @intCast(j));
            const flag = (self.dma.dicr.irq_flags & mask) != 0;
            zgui.sameLine(.{});
            if (flag) {
                zgui.textColored(.{ 1.0, 1.0, 0.0, 1.0 }, "[x]      ", .{});
            } else {
                zgui.text("[ ]      ", .{});
            }
        }

        // IRQ Mode row
        zgui.text("IRQ Mode:   ", .{});
        for (0..7) |j| {
            const mask = (@as(u7, 1) << @intCast(j));
            const mode = (self.dma.dicr.irq_mode & mask) != 0;
            zgui.sameLine(.{});
            zgui.text("{s: <9}", .{if (mode) "block" else "complete"});
        }

        zgui.separator();

        if (zgui.beginTabBar("##dma_tabs", .{})) {
            // Channel tabs
            for (0..7) |i| {
                const chan = &self.dma.channels[i];
                const is_active = chan.ctrl.isActive();

                zgui.pushIntId(@intCast(i));
                if (zgui.beginTabItem(channel_names[i], .{})) {
                    self.drawChannel(@intCast(i));
                    zgui.endTabItem();
                } else if (is_active) {
                    // Show indicator on inactive tabs if channel is active
                    // (tab item returns false when not selected)
                }
                zgui.popId();
            }

            zgui.endTabBar();
        }
    }
    zgui.end();
}

fn drawChannel(self: *@This(), i: u3) void {
    const chan = &self.dma.channels[i];
    const ctrl_val: u32 = @bitCast(chan.ctrl);
    const block_val: u32 = @bitCast(chan.block);

    const is_active = chan.ctrl.isActive();
    const master_enable = self.dma.dpcr.getChanMasterEnable(i);
    const priority = self.dma.dpcr.getChanPriority(i);

    // Status summary
    if (is_active) {
        zgui.textColored(.{ 0.0, 1.0, 0.0, 1.0 }, "ACTIVE", .{});
    } else {
        zgui.textColored(.{ 0.5, 0.5, 0.5, 1.0 }, "Inactive", .{});
    }
    zgui.sameLine(.{});
    zgui.text("| Master: {s} | Priority: {d}", .{
        if (master_enable) "ON" else "OFF",
        priority,
    });

    zgui.separator();

    // Base address
    zgui.text("MADR: 0x{x:0>8}", .{chan.maddr});

    // Block control
    zgui.text("BCR:  0x{x:0>8}", .{block_val});
    zgui.text("  Block Size:  0x{x:0>4} ({d})", .{ chan.block.size, chan.block.size });
    zgui.text("  Block Count: 0x{x:0>4} ({d})", .{ chan.block.count, chan.block.count });

    // Channel control
    zgui.text("CHCR: 0x{x:0>8}", .{ctrl_val});

    const dir_str = switch (chan.ctrl.direction) {
        .to_ram => "To RAM",
        .to_device => "To Device",
    };
    const addr_inc_str = switch (chan.ctrl.addr_inc) {
        .incr => "+4",
        .decr => "-4",
    };
    const sync_str = switch (chan.ctrl.sync_mode) {
        .burst => "Burst",
        .slice => "Slice",
        .linked_list => "Linked List",
    };

    zgui.text("  Direction: {s}", .{dir_str});
    zgui.text("  Addr Step: {s}", .{addr_inc_str});
    zgui.text("  Sync Mode: {s}", .{sync_str});
    zgui.text("  Start: {s}  Force: {s}  Pause: {s}", .{
        if (chan.ctrl.start) "yes" else "no",
        if (chan.ctrl.force_start) "yes" else "no",
        if (chan.ctrl.pause) "yes" else "no",
    });
    zgui.text("  Chopping DMA: {d}  CPU: {d}", .{ chan.ctrl.chopping_dma, chan.ctrl.chopping_cpu });
}
