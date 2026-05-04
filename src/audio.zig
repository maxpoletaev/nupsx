const std = @import("std");

const log = std.log.scoped(.audio);

pub const AudioStream = struct {
    const capacity = 8192;

    buf: [capacity][2]i16 = undefined,
    head: std.atomic.Value(usize) = .init(0),
    tail: std.atomic.Value(usize) = .init(0),
    last_sample: [2]i16 = .{ 0, 0 },

    inline fn isFull(self: *@This()) bool {
        const tail = self.tail.load(.monotonic);
        const next_tail = (tail + 1) % capacity;
        return next_tail == self.head.load(.acquire);
    }

    pub fn push(self: *@This(), sample: [2]i16) void {
        if (self.isFull()) {
            log.debug("audio buffer overrun", .{});
            return;
        }

        const tail = self.tail.load(.monotonic);
        const next_tail = (tail + 1) % capacity;

        self.buf[tail] = sample;
        self.tail.store(next_tail, .release);
    }

    pub fn pop(self: *@This()) [2]i16 {
        const head = self.head.load(.monotonic);
        if (head == self.tail.load(.acquire)) {
            log.debug("audio buffer underrun", .{});
            return self.last_sample;
        }

        const sample = self.buf[head];
        self.head.store((head + 1) % capacity, .release);
        self.last_sample = sample;
        return sample;
    }

    pub fn drain(self: *@This(), ptr: [*]f32, max_frames: u32) u32 {
        var i: u32 = 0;
        while (i < max_frames) : (i += 1) {
            const head = self.head.load(.monotonic);
            if (head == self.tail.load(.acquire)) break;

            const sample = self.buf[head];
            self.head.store((head + 1) % capacity, .release);

            ptr[i] = @as(f32, @floatFromInt(sample[0])) / 32768.0;
            ptr[max_frames + i] = @as(f32, @floatFromInt(sample[1])) / 32768.0;
        }
        return i;
    }
};
