const std = @import("std");
const mem = @import("mem.zig");

pub const base_addr = 0x1fc00000;

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
        const offset = addr - base_addr;
        return mem.read(u8, self.buf, offset);
    }

    pub fn readHalf(self: *@This(), addr: u32) u16 {
        const offset = addr - base_addr;
        return mem.read(u16, self.buf, offset);
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        const offset = addr - base_addr;
        return mem.read(u32, self.buf, offset);
    }
};
