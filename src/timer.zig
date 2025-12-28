const std = @import("std");
const bits = @import("bits.zig");
const mem_mod = @import("mem.zig");

const Bus = mem_mod.Bus;
const Interrupt = mem_mod.Interrupt;

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
    paused: bool = false,
    current: u16 = 0,
    target: u16 = 0,
    idx: u2,

    fn init(idx: u2) @This() {
        return .{ .idx = idx };
    }

    inline fn getMode(self: *@This()) u16 {
        const v: u16 = @bitCast(self.mode);
        self.mode.reached_target = false;
        self.mode.reached_ffff = false;
        return v;
    }

    inline fn setMode(self: *@This(), v: u16) void {
        const prev = self.mode;
        self.mode = @bitCast(v);
        self.mode.irq_disabled = true;
        self.mode.reached_target = prev.reached_target;
        self.mode.reached_ffff = prev.reached_ffff;
    }

    pub inline fn getSyncMode(self: *@This()) SyncMode {
        return sync_mode_table[self.idx][self.mode.sync_mode];
    }

    pub inline fn getClockSource(self: *@This()) ClockSource {
        return clock_source_table[self.idx][self.mode.clock_source];
    }
};

pub const Timers = struct {
    pub const addr_start: u32 = 0x1f801100;
    pub const addr_end: u32 = 0x1f80112f;

    const reg_current = 0;
    const reg_mode = 4;
    const reg_target = 8;

    allocator: std.mem.Allocator,
    bus: *mem_mod.Bus,
    t: [3]Timer,
    div8_counter: u8,

    pub fn init(allocator: std.mem.Allocator, bus: *Bus) *@This() {
        const self = allocator.create(@This()) catch unreachable;
        self.* = .{
            .allocator = allocator,
            .bus = bus,
            .t = .{
                .init(0),
                .init(1),
                .init(2),
            },
            .div8_counter = 0,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn tickSystemClock(self: *@This()) void {
        for (0..3) |i| {
            const timer = &self.t[i];
            if (timer.paused or timer.getClockSource() != .system_clock) {
                continue;
            }

            const flags = switch (i) {
                0 => Interrupt.tmr0,
                1 => Interrupt.tmr1,
                2 => Interrupt.tmr2,
                else => unreachable,
            };

            timer.current, _ = @addWithOverflow(timer.current, 1);

            if (timer.current == timer.target) {
                timer.mode.reached_target = true;
                if (timer.mode.reset_on_target) timer.current = 0;
                if (timer.mode.irq_on_target) self.bus.setInterrupt(flags);
            } else if (timer.current == 0xFFFF) {
                timer.mode.reached_ffff = true;
                if (timer.mode.irq_on_ffff) self.bus.setInterrupt(flags);
            }
        }
    }

    pub fn readByte(_: *@This(), addr: u32) u8 {
        std.debug.panic("timers: unhandled readByte at {x}", .{addr});
    }

    pub fn readHalf(self: *@This(), addr: u32) u16 {
        const offset = addr - addr_start;
        const reg = bits.field(offset, 0, u4);
        const timer = bits.field(offset, 4, u2);

        switch (reg) {
            reg_current => return self.t[timer].current,
            reg_mode => return self.t[timer].getMode(),
            reg_target => return self.t[timer].target,
            else => {
                std.debug.panic("timers: unhandled readHalf at {x}", .{addr});
            },
        }
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        const offset = addr - addr_start;
        const reg = bits.field(offset, 0, u4);
        const timer = bits.field(offset, 4, u2);

        switch (reg) {
            reg_current => return self.t[timer].current,
            reg_mode => return self.t[timer].getMode(),
            reg_target => return self.t[timer].target,
            else => {
                std.debug.panic("timers: unhandled readHalf at {x}", .{addr});
            },
        }
    }

    pub fn writeByte(_: *@This(), addr: u32, v: u8) void {
        std.debug.panic("unhandled writeByte at {x} with value {x}", .{ addr, v });
    }

    pub fn writeHalf(self: *@This(), addr: u32, v: u16) void {
        const offset = addr - addr_start;
        const reg = bits.field(offset, 0, u4);
        const timer = bits.field(offset, 4, u2);

        switch (reg) {
            reg_current => self.t[timer].current = v,
            reg_mode => self.t[timer].setMode(v),
            reg_target => self.t[timer].target = v,
            else => {
                log.warn("unhandled writeHalf at {x}", .{addr});
            },
        }
    }

    pub fn writeWord(self: *@This(), addr: u32, v: u32) void {
        const offset = addr - addr_start;
        const reg = bits.field(offset, 0, u4);
        const timer = bits.field(offset, 4, u2);

        switch (reg) {
            reg_current => self.t[timer].current = @truncate(v),
            reg_mode => self.t[timer].setMode(@truncate(v)),
            reg_target => self.t[timer].target = @truncate(v),
            else => {
                log.warn("unhandled writeHalf at {x}", .{addr});
            },
        }
    }
};
