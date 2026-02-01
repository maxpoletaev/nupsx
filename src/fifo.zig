const std = @import("std");

const Error = error{
    MaxCapacityReached,
    OutOfMemory,
};

pub fn StaticFifo(comptime T: type, comptime capacity: usize) type {
    return struct {
        const Self = @This();

        pub const empty: Self = .{
            .buf = undefined,
            .head = 0,
            .tail = 0,
            .len = 0,
        };

        buf: [capacity]T,
        head: usize,
        tail: usize,
        len: usize,

        pub inline fn isEmpty(self: *@This()) bool {
            return self.len == 0;
        }

        pub inline fn isFull(self: *@This()) bool {
            return self.len == capacity;
        }

        pub fn push(self: *@This(), item: T) void {
            if (self.isFull()) {
                @panic("StaticFifo overflow");
            }
            self.buf[self.tail] = item;
            self.tail = (self.tail + 1) % capacity;
            self.len += 1;
        }

        pub fn pop(self: *@This()) ?T {
            if (self.isEmpty()) {
                return null;
            }
            const item = self.buf[self.head];
            self.head = (self.head + 1) % capacity;
            self.len -= 1;
            return item;
        }

        pub fn peek(self: *@This()) ?T {
            if (self.isEmpty()) {
                return null;
            }
            return self.buf[self.head];
        }

        pub fn clear(self: *@This()) void {
            self.head = 0;
            self.tail = 0;
            self.len = 0;
        }
    };
}

pub fn DynamicFifo(comptime T: type) type {
    return struct {
        const Self = @This();

        buf: []T,
        head: usize,
        tail: usize,
        len: usize,
        max_capacity: ?usize,

        pub fn init() Self {
            return .{
                .buf = &[_]T{},
                .head = 0,
                .tail = 0,
                .len = 0,
                .max_capacity = null,
            };
        }

        pub fn initCapacity(allocator: std.mem.Allocator, size: usize) Error!Self {
            var fifo = Self.init();
            try fifo.ensureCapacity(allocator, size);
            return fifo;
        }

        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            if (self.buf.len != 0) {
                allocator.free(self.buf);
            }
            self.* = Self.init();
        }

        pub fn setMaxCapacity(self: *Self, max_capacity: usize) void {
            self.max_capacity = max_capacity;
        }

        pub fn isEmpty(self: *Self) bool {
            return self.len == 0;
        }

        pub fn capacity(self: *Self) usize {
            return self.buf.len;
        }

        pub fn push(self: *Self, allocator: std.mem.Allocator, item: T) Error!void {
            try self.ensureCapacity(allocator, self.len + 1);
            self.buf[self.tail] = item;
            self.tail = (self.tail + 1) % self.buf.len;
            self.len += 1;
        }

        pub fn pop(self: *Self) ?T {
            if (self.isEmpty()) {
                return null;
            }
            const item = self.buf[self.head];
            self.head = (self.head + 1) % self.buf.len;
            self.len -= 1;
            return item;
        }

        pub fn peek(self: *Self) ?T {
            if (self.isEmpty()) {
                return null;
            }
            return self.buf[self.head];
        }

        pub fn clear(self: *Self) void {
            self.head = 0;
            self.tail = 0;
            self.len = 0;
        }

        fn ensureCapacity(self: *Self, allocator: std.mem.Allocator, min_capacity: usize) Error!void {
            if (self.buf.len >= min_capacity) return;

            var new_capacity: usize = if (self.buf.len == 0) 4 else self.buf.len * 2;

            while (new_capacity < min_capacity) : (new_capacity *= 2) {}

            if (self.max_capacity) |max_cap| {
                if (new_capacity > max_cap) {
                    return Error.MaxCapacityReached;
                }
            }

            var new_buf = allocator.alloc(T, new_capacity) catch {
                return Error.OutOfMemory;
            };

            if (self.len != 0) {
                if (self.head < self.tail) {
                    std.mem.copyForwards(T, new_buf[0..self.len], self.buf[self.head..self.tail]);
                } else {
                    const first_len = self.buf.len - self.head;
                    std.mem.copyForwards(T, new_buf[0..first_len], self.buf[self.head..self.buf.len]);
                    std.mem.copyForwards(T, new_buf[first_len..self.len], self.buf[0..self.tail]);
                }
            }

            if (self.buf.len != 0) {
                allocator.free(self.buf);
            }

            self.buf = new_buf;
            self.head = 0;
            self.tail = self.len;
        }
    };
}
