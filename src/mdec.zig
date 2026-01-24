const std = @import("std");

const log = std.log.scoped(.mdec);

pub const MDEC = struct {
    pub const addr_start = 0x1f801820;
    pub const addr_end = 0x1f801828;

    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(MDEC) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        _ = self;
        // std.debug.panic("mdec: read not implemented: addr={x}", .{addr});
        log.warn("read not implemented: addr={x}", .{addr});
        return 0;
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        _ = self;
        // std.debug.panic("mdec: write not implemented: addr={x}, value={x}", .{ addr, v });
        log.warn("write not implemented: addr={x}, value={x}", .{ addr, v });
    }
};
