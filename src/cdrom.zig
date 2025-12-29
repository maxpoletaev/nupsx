const std = @import("std");

const log = std.log.scoped(.cdrom);

pub const CDROM = struct {
    pub const addr_start: u32 = 0x1f801800;
    pub const addr_end: u32 = 0x1f801803;

    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch unreachable;
        self.* = .{
            .allocator = allocator,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn read(_: @This(), comptime T: type, addr: u32) T {
        log.warn("unhandled read ({s}) at {x}", .{ @typeName(T), addr });
        return 0;
    }

    pub fn write(_: @This(), comptime T: type, addr: u32, v: T) void {
        log.warn("unhandled write ({s}) at {x} = {x}", .{ @typeName(T), addr, @as(u32, v) });
    }
};
