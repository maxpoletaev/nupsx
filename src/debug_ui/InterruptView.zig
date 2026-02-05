const std = @import("std");
const zgui = @import("zgui");
const mem = @import("../mem.zig");
const cpu_mod = @import("../cpu.zig");

const Bus = mem.Bus;
const CPU = cpu_mod.CPU;
const Interrupt = mem.Interrupt;

const InterruptInfo = struct {
    name: [:0]const u8,
    bit: u32,
    short: [:0]const u8,
};

const interrupts = [_]InterruptInfo{
    .{ .name = "VBLANK", .bit = Interrupt.vblank, .short = "VBL" },
    .{ .name = "GPU", .bit = Interrupt.gpu, .short = "GPU" },
    .{ .name = "CDROM", .bit = Interrupt.cdrom, .short = "CD" },
    .{ .name = "DMA", .bit = Interrupt.dma, .short = "DMA" },
    .{ .name = "TMR0", .bit = Interrupt.tmr0, .short = "T0" },
    .{ .name = "TMR1", .bit = Interrupt.tmr1, .short = "T1" },
    .{ .name = "TMR2", .bit = Interrupt.tmr2, .short = "T2" },
    .{ .name = "JOY/MC", .bit = Interrupt.joy_mc_byte, .short = "JOY" },
    .{ .name = "SIO", .bit = Interrupt.sio, .short = "SIO" },
    .{ .name = "SPU", .bit = Interrupt.spu, .short = "SPU" },
    .{ .name = "LIGHTPEN", .bit = Interrupt.lightpen, .short = "LGT" },
};

allocator: std.mem.Allocator,
bus: *Bus,
cpu: *CPU,

pub fn init(allocator: std.mem.Allocator, cpu: *CPU, bus: *Bus) !*@This() {
    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .bus = bus,
        .cpu = cpu,
    };
    return self;
}

pub fn deinit(self: *@This()) void {
    const allocator = self.allocator;
    allocator.destroy(self);
}

pub fn update(self: *@This()) void {
    if (zgui.begin("Interrupts", .{})) {
        self.drawStatusHeader();
        zgui.separator();
        self.drawInterruptRows();
        zgui.separator();
        self.drawQuickActions();
    }
    zgui.end();
}

fn drawStatusHeader(self: *@This()) void {
    // COP0 register 12 = Status, register 13 = Cause
    const status_reg = self.cpu.cop0.getReg(12);
    const cause_reg = self.cpu.cop0.getReg(13);

    const pending = (self.bus.irq_stat & self.bus.irq_mask) != 0;
    const cpu_irq_enabled = (status_reg & 1) != 0; // Bit 0: current interrupt enable
    const ip2_set = (cause_reg & (1 << 10)) != 0; // Bit 10: IP2 (hardware IRQ)

    // Overall IRQ status
    zgui.text("I_STAT: 0x{X:0>4}  I_MASK: 0x{X:0>4}", .{ self.bus.irq_stat, self.bus.irq_mask });

    // CPU status
    const irq_will_fire = pending and cpu_irq_enabled and ip2_set;
    if (irq_will_fire) {
        zgui.textColored(.{ 1.0, 0.3, 0.3, 1.0 }, "IRQ ACTIVE", .{});
    } else if (pending) {
        zgui.textColored(.{ 1.0, 0.8, 0.2, 1.0 }, "IRQ PENDING (CPU disabled)", .{});
    } else {
        zgui.textColored(.{ 0.5, 0.5, 0.5, 1.0 }, "No pending IRQs", .{});
    }

    zgui.sameLine(.{});
    zgui.text("  |  CPU IE: {s}  IP2: {s}", .{
        if (cpu_irq_enabled) "ON" else "OFF",
        if (ip2_set) "SET" else "CLR",
    });
}

fn drawInterruptRows(self: *@This()) void {
    // Header row
    zgui.text("IRQ:        ", .{});
    for (interrupts) |irq| {
        zgui.sameLine(.{});
        zgui.text("{s: <6}", .{irq.short});
    }

    // Mask row with checkboxes
    zgui.text("Mask:       ", .{});
    for (interrupts, 0..) |irq, i| {
        zgui.sameLine(.{});
        var mask_enabled = (self.bus.irq_mask & irq.bit) != 0;
        zgui.pushIntId(@intCast(i));
        if (zgui.checkbox("##m", .{ .v = &mask_enabled })) {
            if (mask_enabled) {
                self.bus.irq_mask |= irq.bit;
            } else {
                self.bus.irq_mask &= ~irq.bit;
            }
            self.bus.updateCpuIrq();
        }
        zgui.popId();
    }

    // Pending row with checkboxes
    zgui.text("Pending:    ", .{});
    for (interrupts, 0..) |irq, i| {
        zgui.sameLine(.{});
        var pend = (self.bus.irq_stat & irq.bit) != 0;
        zgui.pushIntId(@intCast(i + 100));
        if (zgui.checkbox("##p", .{ .v = &pend })) {
            if (pend) {
                self.bus.irq_stat |= irq.bit;
            } else {
                self.bus.irq_stat &= ~irq.bit;
            }
            self.bus.updateCpuIrq();
        }
        zgui.popId();
    }

    // Active indicator row (mask AND pending)
    zgui.text("Active:     ", .{});
    for (interrupts) |irq| {
        zgui.sameLine(.{});
        const is_active = (self.bus.irq_stat & irq.bit) != 0 and (self.bus.irq_mask & irq.bit) != 0;
        if (is_active) {
            zgui.textColored(.{ 1.0, 0.3, 0.3, 1.0 }, "[*]   ", .{});
        } else {
            zgui.textColored(.{ 0.4, 0.4, 0.4, 1.0 }, "[ ]   ", .{});
        }
    }
}

fn drawQuickActions(self: *@This()) void {
    zgui.text("Quick Actions:", .{});

    if (zgui.smallButton("Clear All")) {
        self.bus.irq_stat = 0;
        self.bus.updateCpuIrq();
    }

    zgui.sameLine(.{});
    if (zgui.smallButton("Mask All")) {
        self.bus.irq_mask = 0x7FF;
        self.bus.updateCpuIrq();
    }

    zgui.sameLine(.{});
    if (zgui.smallButton("Unmask All")) {
        self.bus.irq_mask = 0;
        self.bus.updateCpuIrq();
    }

    zgui.sameLine(.{});
    if (zgui.smallButton("Fire VBLANK")) {
        self.bus.setInterrupt(Interrupt.vblank);
    }

    // Trigger buttons for each interrupt
    zgui.text("Trigger:", .{});
    for (interrupts, 0..) |irq, i| {
        if (i > 0 and i % 6 == 0) {
            // New row after 6 buttons
        } else if (i > 0) {
            zgui.sameLine(.{});
        }
        zgui.pushIntId(@intCast(i + 200));
        if (zgui.smallButton(irq.name)) {
            self.bus.setInterrupt(irq.bit);
        }
        zgui.popId();
    }
}
