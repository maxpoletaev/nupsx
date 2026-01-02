const std = @import("std");

pub fn StaticFifo(comptime T: type, comptime capacity: usize) type {
    return struct {
        const Self = @This();

        buffer: [capacity]T = undefined,
        head: usize = 0,
        tail: usize = 0,
        len: usize = 0,

        pub fn init() Self {
            return Self{};
        }

        pub fn isEmpty(self: *@This()) bool {
            return self.len == 0;
        }

        pub fn isFull(self: *@This()) bool {
            return self.len == capacity;
        }

        pub fn push(self: *@This(), item: T) !void {
            if (self.isFull()) {
                return error.FifoFull;
            }
            self.buffer[self.tail] = item;
            self.tail = (self.tail + 1) % capacity;
            self.len += 1;
        }

        pub fn pop(self: *@This()) ?T {
            if (self.isEmpty()) {
                return null;
            }
            const item = self.buffer[self.head];
            self.head = (self.head + 1) % capacity;
            self.len -= 1;
            return item;
        }
    };
}
