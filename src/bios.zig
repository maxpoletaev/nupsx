const std = @import("std");
const mem_mod = @import("mem.zig");

const log = std.log.scoped(.bios);

pub const BIOS = struct {
    pub const addr_start: u32 = 0x1fc00000;
    pub const addr_end: u32 = 0x1fc7ffff;

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

    pub inline fn readByte(self: *@This(), addr: u32) u8 {
        return mem_mod.read(u8, self.rom, addr - addr_start);
    }
    pub inline fn readHalf(self: *@This(), addr: u32) u16 {
        return mem_mod.read(u16, self.rom, addr - addr_start);
    }
    pub inline fn readWord(self: *@This(), addr: u32) u32 {
        return mem_mod.read(u32, self.rom, addr - addr_start);
    }

    pub inline fn writeByte(_: *@This(), _: u32, _: u8) void {
        // BIOS is read-only
    }
    pub inline fn writeHalf(_: *@This(), _: u32, _: u16) void {
        // BIOS is read-only
    }
    pub inline fn writeWord(_: *@This(), _: u32, _: u32) void {
        // BIOS is read-only
    }
};
