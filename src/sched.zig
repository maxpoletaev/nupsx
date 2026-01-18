const std = @import("std");

pub const Event = enum {
    hblank_start,
    hblank_end,
    vblank_start,
    vblank_end,
};

const ScheduledEvent = struct {
    event: Event,
    cycle: u64,
    repeat_interval: ?u64,

    pub fn compare(_: void, a: ScheduledEvent, b: ScheduledEvent) std.math.Order {
        if (a.cycle < b.cycle) return .lt;
        if (a.cycle > b.cycle) return .gt;
        return .eq;
    }
};

pub const Scheduler = struct {
    allocator: std.mem.Allocator,
    queue: std.PriorityQueue(ScheduledEvent, void, ScheduledEvent.compare),
    cycle: u64,

    pub fn init(allocator: std.mem.Allocator) *Scheduler {
        const self = allocator.create(Scheduler) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .queue = .init(allocator, {}),
            .cycle = 0,
        };
        return self;
    }

    pub fn deinit(self: *Scheduler) void {
        self.queue.deinit();
        self.allocator.destroy(self);
    }

    pub fn scheduleAt(self: *Scheduler, event: Event, cycle: u64, repeat_interval: ?u64) void {
        self.queue.add(.{
            .event = event,
            .cycle = cycle,
            .repeat_interval = repeat_interval,
        }) catch @panic("OOM");
    }

    pub fn scheduleIn(self: *Scheduler, event: Event, cycles_from_now: u64, repeat_interval: ?u64) void {
        self.scheduleAt(event, self.cycle + cycles_from_now, repeat_interval);
    }

    pub fn peek(self: *Scheduler) ?ScheduledEvent {
        return self.queue.peek();
    }

    pub fn cyclesUntilNext(self: *Scheduler) ?u64 {
        if (self.peek()) |event| {
            if (event.cycle > self.cycle) {
                return event.cycle - self.cycle;
            }
            return 0;
        }
        return null;
    }

    pub fn advance(self: *Scheduler, cycles: u64) void {
        self.cycle += cycles;
    }

    pub fn popNextDue(self: *Scheduler) ?Event {
        if (self.queue.peek()) |next_event| {
            if (next_event.cycle <= self.cycle) {
                const event = self.queue.remove();

                // Reschedule repeating events
                if (event.repeat_interval) |interval| {
                    self.queue.add(.{
                        .event = event.event,
                        .cycle = event.cycle + interval,
                        .repeat_interval = event.repeat_interval,
                    }) catch @panic("OOM");
                }

                return event.event;
            }
        }
        return null;
    }
};
