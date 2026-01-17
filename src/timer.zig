const std = @import("std");
const bits = @import("bits.zig");

const log = std.log.scoped(.timer);

const SyncMode = enum {
    unset,

    pause_during_hblank,
    reset_at_hblank,
    reset_and_pause_outside_hblank,
    pause_until_hblank,

    pause_during_vblank,
    reset_at_vblank,
    reset_and_pause_outside_vblank,
    pause_until_vblank,

    stop_at_current,
    free_run,
};

const ClockSource = enum {
    system_clock,
    system_clock_div8,
    dotclock,
    hblank,
};

const clock_source_table = [3][4]ClockSource{
    .{ .system_clock, .dotclock, .system_clock, .dotclock },
    .{ .system_clock, .hblank, .system_clock, .hblank },
    .{ .system_clock, .system_clock, .system_clock_div8, .system_clock_div8 },
};

const sync_mode_table = [3][4]SyncMode{
    .{ .pause_during_hblank, .reset_at_hblank, .reset_and_pause_outside_hblank, .pause_until_hblank },
    .{ .pause_during_vblank, .reset_at_vblank, .reset_and_pause_outside_vblank, .pause_until_vblank },
    .{ .stop_at_current, .free_run, .free_run, .stop_at_current },
};

const TimerEvent = struct {
    t0_fired: bool = false,
    t1_fired: bool = false,
    t2_fired: bool = false,
};

const Mode = packed struct(u16) {
    sync_enable: bool, // 0
    sync_mode: u2, // 1-2
    reset_on_target: bool, // 3
    irq_on_target: bool, // 4
    irq_on_ffff: bool, // 5
    irq_repeat: bool, // 6
    irq_mode: enum(u1) { pulse = 0, toggle = 1 }, // 7
    clock_source: u2, // 8-9
    irq_disabled: bool, // 10
    reached_target: bool, // 11
    reached_ffff: bool, // 12
    pad: u3, // 13-15
};

const Timer = struct {
    timer_idx: u2,

    mode: Mode = std.mem.zeroes(Mode),
    current: u16 = 0,
    target: u16 = 0,

    paused: bool = false,
    supress_irq: bool = false,

    fn init(timer_id: u2) @This() {
        return .{
            .timer_idx = timer_id,
        };
    }

    fn increment(self: *@This(), v: u16) bool {
        var hit_ffff = false;
        var hit_target = false;

        for (0..v) |_| {
            if (self.mode.reset_on_target and self.target != 0 and self.current == self.target) {
                self.current = 0;
                continue;
            }
            self.current +%= 1;
            if (self.target != 0 and self.current == self.target) {
                self.mode.reached_target = true;
                hit_target = true;
            }
            if (self.current == 0xffff) {
                self.mode.reached_ffff = true;
                hit_ffff = true;
            }
        }

        if (self.supress_irq) {
            return false;
        }

        const hit_target_or_ffff = ((self.mode.irq_on_ffff and hit_ffff) or
            (self.mode.irq_on_target and hit_target));

        switch (self.mode.irq_mode) {
            .toggle => if (hit_target_or_ffff) {
                // In toggle mode, irq_disabled is flipped on IRQ.
                // The interrupt is only triggered on transition 1 -> 0.
                if (!self.mode.irq_repeat) self.supress_irq = true;
                self.mode.irq_disabled = !self.mode.irq_disabled;
                return self.mode.irq_disabled == false;
            },
            .pulse => if (hit_target_or_ffff) {
                // In pulse mode, irq_disabled is permanenty set, except *for a
                // few clock cycles when the interrupt is triggered*. But no
                // games seem to rely on that behavior, so we just keep it off.
                if (!self.mode.irq_repeat) self.supress_irq = true;
                self.mode.irq_disabled = true;
                return hit_target_or_ffff;
            },
        }

        return false;
    }

    inline fn getMode(self: *@This()) u16 {
        const v: u16 = @bitCast(self.mode);
        // Reached bits are reset on read
        self.mode.reached_target = false;
        self.mode.reached_ffff = false;
        return v;
    }

    inline fn setMode(self: *@This(), v: u16, in_hblank: bool, in_vblank: bool) void {
        const prev = self.mode;
        self.mode = @bitCast(v);

        if (self.getClockSource() == .dotclock) {
            log.warn("timer set to dotclock source; unimplemented behavior", .{});
        }

        // Reached bits are preserved during mode write
        self.mode.reached_target = prev.reached_target;
        self.mode.reached_ffff = prev.reached_ffff;

        // IRQ and counters are reset on mode write
        self.mode.irq_disabled = true;
        self.supress_irq = false;
        self.paused = false;
        self.current = 0;

        // Depending on the sync mode, the timer may start paused
        if (self.mode.sync_enable) {
            switch (self.getSyncMode()) {
                .pause_during_hblank => self.paused = in_hblank,
                .pause_during_vblank => self.paused = in_vblank,
                .pause_until_hblank => self.paused = !in_hblank,
                .pause_until_vblank => self.paused = !in_vblank,
                .stop_at_current => self.paused = true,
                else => {},
            }
        }
    }

    pub inline fn getSyncMode(self: *@This()) SyncMode {
        return sync_mode_table[self.timer_idx][self.mode.sync_mode];
    }

    pub inline fn getClockSource(self: *@This()) ClockSource {
        return clock_source_table[self.timer_idx][self.mode.clock_source];
    }
};

const timer_reg_current = 0;
const timer_reg_mode = 4;
const timer_reg_target = 8;

pub const Timers = struct {
    pub const addr_start: u32 = 0x1f801100;
    pub const addr_end: u32 = 0x1f80112f;

    allocator: std.mem.Allocator,
    timers: [3]Timer,
    div8_counter: u8,
    in_hblank: bool = false,
    in_vblank: bool = false,
    pending_events: TimerEvent = .{},

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch unreachable;
        self.* = .{
            .div8_counter = 0,
            .allocator = allocator,
            .timers = .{ .init(0), .init(1), .init(2) },
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub inline fn consumeEvents(self: *@This()) TimerEvent {
        const events = self.pending_events;
        self.pending_events = .{};
        return events;
    }

    inline fn setTimerFired(self: *@This(), idx: u2) void {
        switch (idx) {
            0 => self.pending_events.t0_fired = true,
            1 => self.pending_events.t1_fired = true,
            2 => self.pending_events.t2_fired = true,
            else => unreachable,
        }
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const offset = addr - addr_start;
        const regid = bits.field(offset, 0, u4);
        const tmrid = bits.field(offset, 4, u2);

        const v = switch (regid) {
            timer_reg_current => self.timers[tmrid].current,
            timer_reg_mode => self.timers[tmrid].getMode(),
            timer_reg_target => self.timers[tmrid].target,
            else => std.debug.panic("timer: unhandled read ({s}) at {x}", .{ @typeName(T), addr }),
        };

        return switch (T) {
            u16, u32 => v,
            u8 => @truncate(v),
            else => @compileError("timers.read: unsupported type"),
        };
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const offset = addr - addr_start;
        const regid = bits.field(offset, 0, u4);
        const tmrid = bits.field(offset, 4, u2);

        const val: u16 = switch (T) {
            u8, u16 => v,
            u32 => @truncate(v),
            else => @compileError("Timers.Write: Unsupported type"),
        };

        switch (regid) {
            timer_reg_current => self.timers[tmrid].current = val,
            timer_reg_mode => self.timers[tmrid].setMode(val, self.in_hblank, self.in_vblank),
            timer_reg_target => self.timers[tmrid].target = val,
            else => std.debug.panic("timer: unhandled write at {x}", .{addr}),
        }
    }

    pub fn tick(self: *@This(), cyc: u8) void {
        self.tickTimer0(cyc);
        self.tickTimer1(cyc);
        self.tickTimer2(cyc);
    }

    inline fn tickTimer0(self: *@This(), cyc: u8) void {
        const timer = &self.timers[0];
        if (timer.paused) return;

        switch (timer.getClockSource()) {
            .system_clock => {
                const fired = timer.increment(cyc);
                if (fired) self.setTimerFired(0);
            },
            .dotclock => {
                // Dotclock ticks depend on the current gpu mode.
                // For now just increment on every system clock tick.
                const fired = timer.increment(cyc);
                if (fired) self.setTimerFired(0);
            },
            else => {},
        }
    }

    inline fn tickTimer1(self: *@This(), cyc: u8) void {
        const timer = &self.timers[1];
        if (timer.paused) return;

        if (timer.getClockSource() == .system_clock) {
            const fired = timer.increment(cyc);
            if (fired) self.setTimerFired(1);
        }
    }

    inline fn tickTimer2(self: *@This(), cyc: u8) void {
        const timer = &self.timers[2];
        if (timer.paused) return;

        switch (timer.getClockSource()) {
            .system_clock => {
                const fired = timer.increment(cyc);
                if (fired) self.setTimerFired(2);
            },
            .system_clock_div8 => {
                self.div8_counter += cyc;
                if (self.div8_counter >= 8) {
                    const fired = timer.increment(self.div8_counter / 8);
                    if (fired) self.setTimerFired(2);
                    self.div8_counter %= 8;
                }
            },
            else => {},
        }
    }

    pub fn hblankStart(self: *@This()) void {
        self.in_hblank = true;

        const t0 = &self.timers[0];
        const t1 = &self.timers[1];

        // Timer 0 hblank sync modes
        if (t0.mode.sync_enable) {
            switch (t0.getSyncMode()) {
                .pause_during_hblank => t0.paused = true,
                .pause_until_hblank => t0.paused = false,
                .reset_at_hblank => t0.current = 0,
                .reset_and_pause_outside_hblank => {
                    t0.current = 0;
                    t0.paused = false;
                },
                else => unreachable,
            }
        }

        // Timer 1 in hblank clock mode increments on hblank
        if (t1.getClockSource() == .hblank and !t1.paused) {
            const fired = t1.increment(1);
            if (fired) self.setTimerFired(1);
        }
    }

    pub fn hblankEnd(self: *@This()) void {
        self.in_hblank = false;

        const t0 = &self.timers[0];
        if (!t0.mode.sync_enable) return;

        switch (t0.getSyncMode()) {
            .pause_during_hblank => t0.paused = false,
            .reset_and_pause_outside_hblank => t0.paused = true,
            else => {},
        }
    }

    pub fn vblankStart(self: *@This()) void {
        self.in_vblank = true;

        // Timer 1 vblank sync modes
        const t1 = &self.timers[1];
        if (!t1.mode.sync_enable) return;

        switch (t1.getSyncMode()) {
            .pause_during_vblank => t1.paused = true,
            .pause_until_vblank => t1.paused = false,
            .reset_at_vblank => t1.current = 0,
            .reset_and_pause_outside_vblank => {
                t1.current = 0;
                t1.paused = false;
            },
            else => {},
        }
    }

    pub fn vblankEnd(self: *@This()) void {
        self.in_vblank = false;

        const t1 = &self.timers[1];
        if (!t1.mode.sync_enable) return;

        switch (t1.getSyncMode()) {
            .pause_during_vblank => t1.paused = false,
            .reset_and_pause_outside_vblank => t1.paused = true,
            else => {},
        }
    }
};
