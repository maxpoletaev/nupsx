const std = @import("std");
const zgui = @import("zgui");
const Timers = @import("../timer.zig").Timers;

allocator: std.mem.Allocator,
timers: *Timers,

pub fn init(allocator: std.mem.Allocator, timers: *Timers) !*@This() {
    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .timers = timers,
    };
    return self;
}

pub fn deinit(self: *@This()) void {
    const allocator = self.allocator;
    allocator.destroy(self);
}

pub fn update(self: *@This()) void {
    if (zgui.begin("Timers", .{})) {
        for (0..3) |i| {
            const timer = &self.timers.timers[i];
            const mode_val: u16 = @bitCast(timer.mode);
            const clock_source = timer.getClockSource();
            const sync_mode = timer.getSyncMode();

            const header_label = switch (i) {
                0 => "Timer 0",
                1 => "Timer 1",
                2 => "Timer 2",
                else => unreachable,
            };
            if (zgui.collapsingHeader(header_label, .{ .default_open = true })) {
                const indent_w = 20.0;
                zgui.indent(.{ .indent_w = indent_w });

                zgui.text("Current: 0x{x:0>4} ({d})", .{ timer.current, timer.current });
                zgui.text("Target: 0x{x:0>4} ({d})", .{ timer.target, timer.target });
                zgui.text("Paused: {s}", .{if (timer.paused) "yes" else "no"});

                zgui.separator();

                zgui.text("Clock Source: {s}", .{@tagName(clock_source)});
                zgui.text("Sync Mode: {s}", .{@tagName(sync_mode)});

                zgui.pushIntId(@intCast(i));
                if (zgui.collapsingHeader("Mode Register", .{})) {
                    zgui.text("Raw Value: 0x{x:0>4}", .{mode_val});
                    zgui.text("Sync Enable: {s}", .{if (timer.mode.sync_enable) "yes" else "no"});
                    zgui.text("Sync Mode: {d}", .{timer.mode.sync_mode});
                    zgui.text("Reset on Target: {s}", .{if (timer.mode.reset_on_target) "yes" else "no"});
                    zgui.text("IRQ on Target: {s}", .{if (timer.mode.irq_on_target) "yes" else "no"});
                    zgui.text("IRQ on 0xFFFF: {s}", .{if (timer.mode.irq_on_ffff) "yes" else "no"});
                    zgui.text("IRQ Repeat: {s}", .{if (timer.mode.irq_repeat) "yes" else "no"});
                    zgui.text("IRQ Toggle: {s}", .{if (timer.mode.irq_toggle) "yes" else "no"});
                    zgui.text("Clock Source Bits: {d}", .{timer.mode.clock_source});
                    zgui.text("IRQ Disabled: {s}", .{if (timer.mode.irq_disabled) "yes" else "no"});
                    zgui.text("Reached Target: {s}", .{if (timer.mode.reached_target) "yes" else "no"});
                    zgui.text("Reached 0xFFFF: {s}", .{if (timer.mode.reached_ffff) "yes" else "no"});
                }
                zgui.popId();

                zgui.unindent(.{ .indent_w = indent_w });
            }
        }
    }
    zgui.end();
}
