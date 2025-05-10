const std = @import("std");
const mem = @import("mem.zig");

pub const base_addr = 0x1FC00000;

const log = std.log.scoped(.bios);

pub const BIOS = struct {
    allocator: std.mem.Allocator,
    buf: []u8,

    pub fn loadFromFile(allocator: std.mem.Allocator, path: []const u8) !*@This() {
        const file = try std.fs.cwd().openFile(path, .{});
        defer file.close();

        const file_size = try file.getEndPos();
        const buf = try allocator.alloc(u8, file_size);
        const bytes_read = try file.readAll(buf);
        if (bytes_read != file_size) {
            return error.FileReadError;
        }

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .buf = buf,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        const allocator = self.allocator;
        allocator.free(self.buf);
        allocator.destroy(self);
    }

    pub fn readByte(self: *@This(), addr: u32) u8 {
        const idx = addr - base_addr;
        return self.buf[idx];
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        const idx = addr - base_addr;
        const buf = self.buf[idx .. idx + 4];
        return std.mem.readInt(u32, buf[0..4], .little);
    }

    pub fn readHalf(self: *@This(), addr: u32) u16 {
        const idx = addr - base_addr;
        const buf = self.buf[idx .. idx + 2];
        return std.mem.readInt(u16, buf[0..2], .little);
    }
};
