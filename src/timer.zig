const std = @import("std");
const bits = @import("bits.zig");

const log = std.log.scoped(.timers);

const SyncMode = enum {
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
    sync_enable: bool,
    sync_mode: u2,
    reset_on_target: bool,
    irq_on_target: bool,
    irq_on_ffff: bool,
    irq_repeat: bool,
    irq_toggle: bool,
    clock_source: u2,
    irq_disabled: bool,
    reached_target: bool,
    reached_ffff: bool,
    pad: u3,
};

const Timer = struct {
    mode: Mode = std.mem.zeroes(Mode),
    current: u16 = 0,
    target: u16 = 0,

    paused: bool = false,
    once: bool = false,

    clock_source_table: [4]ClockSource,
    sync_mode_table: [4]SyncMode,

    fn init(idx: u2) @This() {
        return .{
            .clock_source_table = clock_source_table[idx],
            .sync_mode_table = sync_mode_table[idx],
        };
    }

    inline fn increment(self: *@This()) void {
        if (self.mode.reset_on_target and self.target != 0 and self.current == self.target) {
            self.current = 0; // reset happens on the next tick after reaching target
            return;
        }
        self.current +%= 1;
    }

    inline fn getMode(self: *@This()) u16 {
        const v: u16 = @bitCast(self.mode);
        // reached bits are reset on read
        self.mode.reached_target = false;
        self.mode.reached_ffff = false;
        return v;
    }

    inline fn setMode(self: *@This(), v: u16, in_hblank: bool, in_vblank: bool) void {
        const prev = self.mode;
        self.mode = @bitCast(v);

        // reached bits are preserved during mode write
        self.mode.reached_target = prev.reached_target;
        self.mode.reached_ffff = prev.reached_ffff;

        // irq and counters are reset on mode write
        self.mode.irq_disabled = true;
        self.paused = false;
        self.once = false;
        self.current = 0;

        // depending on the sync mode, the timer may start paused
        if (self.mode.sync_enable) {
            switch (self.getSyncMode()) {
                .reset_and_pause_outside_vblank => self.paused = in_vblank,
                .reset_and_pause_outside_hblank => self.paused = in_hblank,
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
        return self.sync_mode_table[self.mode.sync_mode];
    }

    pub inline fn getClockSource(self: *@This()) ClockSource {
        return self.clock_source_table[self.mode.clock_source];
    }
};

pub const Timers = struct {
    pub const addr_start: u32 = 0x1f801100;
    pub const addr_end: u32 = 0x1f80112f;

    const reg_current = 0;
    const reg_mode = 4;
    const reg_target = 8;

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
        log.debug("timer {d} fired", .{idx});

        switch (idx) {
            0 => self.pending_events.t0_fired = true,
            1 => self.pending_events.t1_fired = true,
            2 => self.pending_events.t2_fired = true,
            else => unreachable,
        }
    }

    pub fn readByte(_: *@This(), addr: u32) u8 {
        std.debug.panic("timers: unhandled readByte at {x}", .{addr});
    }

    pub fn readHalf(self: *@This(), addr: u32) u16 {
        const offset = addr - addr_start;
        const regid = bits.field(offset, 0, u4);
        const tmrid = bits.field(offset, 4, u2);

        switch (regid) {
            reg_current => return self.timers[tmrid].current,
            reg_mode => return self.timers[tmrid].getMode(),
            reg_target => return self.timers[tmrid].target,
            else => {
                std.debug.panic("timers: unhandled readHalf at {x}", .{addr});
            },
        }
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        const offset = addr - addr_start;
        const regid = bits.field(offset, 0, u4);
        const tmrid = bits.field(offset, 4, u2);

        switch (regid) {
            reg_current => return self.timers[tmrid].current,
            reg_mode => return self.timers[tmrid].getMode(),
            reg_target => return self.timers[tmrid].target,
            else => {
                std.debug.panic("timers: unhandled readHalf at {x}", .{addr});
            },
        }
    }

    pub fn writeByte(_: *@This(), addr: u32, v: u8) void {
        std.debug.panic("unhandled writeByte at {x} with value {x}", .{ addr, v });
    }

    pub fn writeHalf(self: *@This(), addr: u32, v: u16) void {
        // log.debug("writeHalf {x} = {x}", .{ addr, v });

        const offset = addr - addr_start;
        const regid = bits.field(offset, 0, u4);
        const tmrid = bits.field(offset, 4, u2);

        switch (regid) {
            reg_current => self.timers[tmrid].current = v,
            reg_mode => self.timers[tmrid].setMode(v, self.in_hblank, self.in_vblank),
            reg_target => self.timers[tmrid].target = v,
            else => {
                log.warn("unhandled writeHalf at {x}", .{addr});
            },
        }
    }

    pub fn writeWord(self: *@This(), addr: u32, v: u32) void {
        // log.debug("writeHalf {x} = {x}", .{ addr, v });

        const offset = addr - addr_start;
        const regid = bits.field(offset, 0, u4);
        const tmrid = bits.field(offset, 4, u2);

        switch (regid) {
            reg_current => self.timers[tmrid].current = @truncate(v),
            reg_mode => self.timers[tmrid].setMode(@truncate(v), self.in_hblank, self.in_vblank),
            reg_target => self.timers[tmrid].target = @truncate(v),
            else => {
                log.warn("unhandled writeHalf at {x}", .{addr});
            },
        }
    }

    // -------------------------
    // System Clock Tick
    // -------------------------

    pub fn tick(self: *@This()) void {
        self.tickTimer0();
        self.tickTimer1();
        self.tickTimer2();
    }

    inline fn tickTimer0(self: *@This()) void {
        const timer = &self.timers[0];
        if (timer.paused) return;

        switch (timer.getClockSource()) {
            .system_clock => {
                timer.increment();
                self.checkTimerIRQ(0);
            },
            .dotclock => {
                // Dotclock ticks depend on the current gpu mode.
                // For now just increment on every system clock tick.
                timer.increment();
                self.checkTimerIRQ(0);
            },
            else => {},
        }
    }

    inline fn tickTimer1(self: *@This()) void {
        const timer = &self.timers[1];
        if (timer.paused) return;

        if (timer.getClockSource() == .system_clock) {
            timer.increment();
            self.checkTimerIRQ(1);
        }
    }

    inline fn tickTimer2(self: *@This()) void {
        const timer = &self.timers[2];
        if (timer.paused) return;

        switch (timer.getClockSource()) {
            .system_clock => {
                timer.increment();
                self.checkTimerIRQ(2);
            },
            .system_clock_div8 => {
                self.div8_counter +%= 1;
                if (self.div8_counter % 8 == 0) {
                    timer.increment();
                    self.checkTimerIRQ(2);
                }
            },
            else => {},
        }
    }

    fn checkTimerIRQ(self: *@This(), idx: u2) void {
        const timer = &self.timers[idx];

        if (timer.target != 0 and timer.current == timer.target) {
            timer.mode.reached_target = true;
            if (timer.mode.irq_on_target) self.setTimerFired(idx);
        }

        if (timer.current == 0xffff) {
            timer.mode.reached_ffff = true;
            if (timer.mode.irq_on_ffff) self.setTimerFired(idx);
        }
    }

    // -------------------------
    // GPU Event Handlers
    // -------------------------

    pub fn hblankStart(self: *@This()) void {
        self.in_hblank = true;

        const t0 = &self.timers[0];
        const t1 = &self.timers[1];

        // Timer 0 hblank sync modes
        if (t0.mode.sync_enable) {
            switch (t0.getSyncMode()) {
                .pause_during_hblank => t0.paused = true,
                .reset_at_hblank => t0.current = 0,
                .reset_and_pause_outside_hblank => {
                    t0.current = 0;
                    t0.paused = false;
                },
                .pause_until_hblank => {
                    if (!t0.once) {
                        t0.once = true;
                        t0.paused = false;
                        t0.mode.sync_enable = false;
                    }
                },
                else => {},
            }
        }

        // Timer 1 in hblank clock mode increments on hblank
        if (t1.getClockSource() == .hblank and !t1.paused) {
            t1.increment();
            self.checkTimerIRQ(1);
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
            .reset_at_vblank => t1.current = 0,
            .reset_and_pause_outside_vblank => {
                t1.current = 0;
                t1.paused = false;
            },
            .pause_until_vblank => {
                if (!t1.once) {
                    t1.once = true;
                    t1.paused = false;
                    t1.mode.sync_enable = false;
                }
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
