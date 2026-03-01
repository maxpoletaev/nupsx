const std = @import("std");
const builtin = @import("builtin");

const AudioStreamDummy = struct {
    pub fn push(_: *@This(), _: [2]i16) void {}
    pub fn pop(_: *@This()) [2]i16 {
        return .{ 0, 0 };
    }
    pub fn signal(_: *@This()) void {}
};

const AudioStreamNative = struct {
    const capacity = 1024;

    buf: [capacity][2]i16 = undefined,
    head: std.atomic.Value(usize) = .init(0),
    tail: std.atomic.Value(usize) = .init(0),
    sem: std.Thread.Semaphore = .{},

    inline fn isFull(self: *@This()) bool {
        const tail = self.tail.load(.monotonic);
        const next_tail = (tail + 1) % capacity;
        return next_tail == self.head.load(.acquire);
    }

    pub fn push(self: *@This(), sample: [2]i16) void {
        while (self.isFull()) {
            self.sem.wait();
        }

        const tail = self.tail.load(.monotonic);
        const next_tail = (tail + 1) % capacity;

        self.buf[tail] = sample;
        self.tail.store(next_tail, .release);
    }

    pub fn pop(self: *@This()) [2]i16 {
        const head = self.head.load(.monotonic);
        if (head == self.tail.load(.acquire)) {
            return .{ 0, 0 };
        }

        const sample = self.buf[head];
        self.head.store((head + 1) % capacity, .release);
        return sample;
    }

    pub fn signal(self: *@This()) void {
        self.sem.post();
    }
};

fn audioStreamType() type {
    if (builtin.mode == .Debug) {
        return AudioStreamDummy; // Debug is very slow and crackly
    }
    return switch (builtin.target.cpu.arch) {
        .wasm32, .wasm64 => AudioStreamDummy, // TODO: Needs separate implementation
        else => AudioStreamNative,
    };
}

pub const AudioStream = audioStreamType();
