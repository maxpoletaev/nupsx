const std = @import("std");
const mem = @import("mem.zig");

const log = std.log.scoped(.bios);

pub const BIOS = struct {
    allocator: std.mem.Allocator,
    rom: []u8,

    pub fn loadFromFile(allocator: std.mem.Allocator, path: []const u8) !*@This() {
        const file = try std.fs.cwd().openFile(path, .{});
        defer file.close();

        const file_size = try file.getEndPos();
        const rom = try allocator.alloc(u8, file_size);

        const bytes_read = try file.readAll(rom);
        if (bytes_read != file_size) {
            return error.FileReadError;
        }

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .rom = rom,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        const allocator = self.allocator;
        allocator.free(self.rom);
        allocator.destroy(self);
    }
};
