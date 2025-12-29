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

    pub fn readByte(_: @This(), addr: u32) u8 {
        log.warn("unhandled readByte at {x}", .{addr});
        return 0;
    }

    pub fn readHalf(_: @This(), addr: u32) u16 {
        log.warn("unhandled readHalf at {x}", .{addr});
        return 0;
    }

    pub fn readWord(_: @This(), addr: u32) u32 {
        log.warn("unhandled readWord at {x}", .{addr});
        return 0;
    }

    pub fn writeByte(_: @This(), addr: u32, v: u8) void {
        log.warn("unhandled writeByte at {x} = {x}", .{ addr, v });
    }

    pub fn writeHalf(_: @This(), addr: u32, v: u16) void {
        log.warn("unhandled writeHalf at {x} = {x}", .{ addr, v });
    }

    pub fn writeWord(_: @This(), addr: u32, v: u32) void {
        log.warn("unhandled writeWord at {x} = {x}", .{ addr, v });
    }
};
