const std = @import("std");
const bits = @import("bits.zig");

const log = std.log.scoped(.spu);

/// All we know for now is that there are 24 voices and each
/// is represented by 8 16-bit registers.
const Voice = struct {
    r: [8]u16,
};

pub const SPU = struct {
    pub const addr_start: u32 = 0x1f801c00;
    pub const addr_end: u32 = 0x1f801e7f;

    allocator: std.mem.Allocator,
    voices: [24]Voice,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(SPU) catch unreachable;
        self.* = .{
            .allocator = allocator,
            .voices = std.mem.zeroes([24]Voice),
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        // log.debug("SPU read at {x}", .{addr});
        const offset = addr - addr_start;

        if (offset < 0x180) {
            const reg_id = bits.field(offset, 1, u3); // register 0-7
            const voice = bits.field(offset, 4, u5); // voice 0-23
            const v = self.voices[voice].r[reg_id];
            switch (T) {
                u16, u32 => return v,
                else => std.debug.panic("SPU read: unsupported type {s}", .{@typeName(T)}),
            }
        }

        return 0x4;
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const offset = addr - addr_start;

        if (offset < 0x180) {
            const reg_id = bits.field(offset, 1, u3); // register 0-7
            const voice = bits.field(offset, 4, u5); // voice 0-23
            switch (T) {
                u16, u32 => self.voices[voice].r[reg_id] = @truncate(v),
                else => std.debug.panic("SPU read: unsupported type {s}", .{@typeName(T)}),
            }
        }

        // log.warn("unhandled SPU write at {x} = {x}", .{ addr, v });
    }
};
