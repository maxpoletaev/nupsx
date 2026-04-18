const std = @import("std");
const builtin = @import("builtin");
const options = @import("build_options");

const AudioStreamDummy = struct {
    pub fn push(_: *@This(), _: [2]i16) void {}
    pub fn pop(_: *@This()) [2]i16 {
        return .{ 0, 0 };
    }
    pub fn signal(_: *@This(), _: std.Io) void {}
};

const AudioStreamNative = struct {
    const capacity = 1024;

    buf: [capacity][2]i16 = undefined,
    head: std.atomic.Value(usize) = .init(0),
    tail: std.atomic.Value(usize) = .init(0),
    sem: std.Io.Semaphore = .{},

    inline fn isFull(self: *@This()) bool {
        const tail = self.tail.load(.monotonic);
        const next_tail = (tail + 1) % capacity;
        return next_tail == self.head.load(.acquire);
    }

    pub fn push(self: *@This(), sample: [2]i16) void {
        while (self.isFull()) {
            self.sem.waitUncancelable(std.Options.debug_io);
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

    pub fn signal(self: *@This(), io: std.Io) void {
        self.sem.post(io);
    }
};

const AudioStreamWasm = struct {
    const capacity = 8192;

    buf: [capacity][2]i16 = undefined,
    head: usize = 0,
    tail: usize = 0,

    pub fn push(self: *@This(), sample: [2]i16) void {
        const next_tail = (self.tail + 1) % capacity;
        if (next_tail == self.head) return; // drop if full
        self.buf[self.tail] = sample;
        self.tail = next_tail;
    }

    pub fn pop(self: *@This()) [2]i16 {
        if (self.head == self.tail) return .{ 0, 0 };
        const sample = self.buf[self.head];
        self.head = (self.head + 1) % capacity;
        return sample;
    }

    pub fn drain(self: *@This(), ptr: [*]f32, max_frames: u32) u32 {
        var i: u32 = 0;
        while (i < max_frames) : (i += 1) {
            if (self.head == self.tail) break;
            const sample = self.buf[self.head];
            self.head = (self.head + 1) % capacity;
            ptr[i] = @as(f32, @floatFromInt(sample[0])) / 32768.0;
            ptr[max_frames + i] = @as(f32, @floatFromInt(sample[1])) / 32768.0;
        }
        return i;
    }

    pub fn signal(_: *@This(), _: std.Io) void {}
};

fn audioStreamType() type {
    if (options.uncapped) return AudioStreamDummy;
    if (builtin.mode == .Debug) return AudioStreamDummy; // Debug is very slow and crackly

    return switch (builtin.target.cpu.arch) {
        .wasm32, .wasm64 => AudioStreamWasm,
        else => AudioStreamNative,
    };
}

pub const AudioStream = audioStreamType();
