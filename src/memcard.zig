const std = @import("std");

const log = std.log.scoped(.memcard);

pub const sector_size = 128;
pub const sector_count = 1024;
pub const image_size = sector_size * sector_count;

pub const Error = error{
    InvalidImageSize,
    MissingPath,
};

pub const MemoryCard = struct {
    allocator: std.mem.Allocator,
    io: std.Io,
    path: ?[]const u8 = null,
    data: [image_size]u8,
    flag: u8 = 0x08,
    dirty: bool = false,

    pub fn initBlank(allocator: std.mem.Allocator, io: std.Io, path: ?[]const u8) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .io = io,
            .path = if (path) |p| allocator.dupe(u8, p) catch @panic("OOM") else null,
            .data = undefined,
        };
        self.format();
        self.dirty = path != null;
        return self;
    }

    pub fn loadOrCreate(allocator: std.mem.Allocator, io: std.Io, path: []const u8) !*@This() {
        const self = initBlank(allocator, io, path);
        errdefer self.deinit();

        const file = std.Io.Dir.openFile(.cwd(), io, path, .{}) catch |err| switch (err) {
            error.FileNotFound => return self,
            else => return err,
        };
        defer file.close(io);

        const file_size = try file.length(io);
        if (file_size != image_size) {
            log.err("invalid memory card image size: expected {d}, got {d}", .{ image_size, file_size });
            return Error.InvalidImageSize;
        }

        var read_buf: [4096]u8 = undefined;
        var reader = file.reader(io, &read_buf);
        try reader.interface.readSliceAll(&self.data);
        self.dirty = false;

        return self;
    }

    pub fn deinit(self: *@This()) void {
        if (self.path) |path| self.allocator.free(path);
        self.allocator.destroy(self);
    }

    pub fn save(self: *@This()) !void {
        if (!self.dirty) return; // nothing has changed
        const path = self.path orelse return Error.MissingPath;

        if (std.fs.path.dirname(path)) |dir| {
            try std.Io.Dir.createDirPath(.cwd(), self.io, dir);
        }

        const file = try std.Io.Dir.createFile(.cwd(), self.io, path, .{ .truncate = true });
        defer file.close(self.io);

        var write_buf: [4096]u8 = undefined;
        var writer = file.writer(self.io, &write_buf);
        try writer.interface.writeAll(&self.data);
        try writer.interface.flush();
        self.dirty = false;
    }

    pub fn readSector(self: *@This(), sector: u16) *const [sector_size]u8 {
        const offset = @as(usize, sector) * sector_size;
        return @ptrCast(&self.data[offset]);
    }

    pub fn writeSector(self: *@This(), sector: u16, buf: *const [sector_size]u8) void {
        const offset = @as(usize, sector) * sector_size;
        @memcpy(self.data[offset .. offset + sector_size], buf);
        self.flag &= ~@as(u8, 0x08);
        self.dirty = true;
    }

    pub fn flushWrite(self: *@This()) void {
        if (self.path == null) return;
        self.save() catch |err| {
            log.err("failed to flush memory card write: {}", .{err});
        };
    }

    pub fn format(self: *@This()) void {
        @memset(&self.data, 0);

        var frame = self.getSectorMut(0);
        frame[0] = 'M';
        frame[1] = 'C';
        frame[127] = calcChecksum(frame[0..127]);

        for (1..16) |sector| {
            frame = self.getSectorMut(@intCast(sector));
            @memset(frame, 0);
            std.mem.writeInt(u32, frame[0..4], 0x000000a0, .little);
            std.mem.writeInt(u16, frame[8..10], 0xffff, .little);
            frame[127] = calcChecksum(frame[0..127]);
        }

        for (16..36) |sector| {
            frame = self.getSectorMut(@intCast(sector));
            @memset(frame, 0);
            std.mem.writeInt(u32, frame[0..4], 0xffffffff, .little);
            frame[127] = calcChecksum(frame[0..127]);
        }

        for (36..56) |sector| {
            frame = self.getSectorMut(@intCast(sector));
            @memset(frame, 0xff);
        }

        for (56..63) |sector| {
            frame = self.getSectorMut(@intCast(sector));
            @memset(frame, 0xff);
        }

        frame = self.getSectorMut(63);
        @memset(frame, 0);
        frame[0] = 'M';
        frame[1] = 'C';
        frame[127] = calcChecksum(frame[0..127]);

        self.flag = 0x08;
        self.dirty = true;
    }

    pub fn calcSectorChecksum(sector: u16) u8 {
        return @truncate((sector >> 8) ^ sector);
    }

    fn getSectorMut(self: *@This(), sector: u16) []u8 {
        const offset = @as(usize, sector) * sector_size;
        return self.data[offset .. offset + sector_size];
    }
};

pub fn calcChecksum(data: []const u8) u8 {
    var checksum: u8 = 0;
    for (data) |byte| {
        checksum ^= byte;
    }
    return checksum;
}

test "blank card layout contains boot header and empty directory" {
    const allocator = std.testing.allocator;
    const card = MemoryCard.initBlank(allocator, std.testing.io, null);
    defer card.deinit();

    try std.testing.expectEqualSlices(u8, "MC", card.readSector(0)[0..2]);
    try std.testing.expectEqual(@as(u8, 0x0e), card.readSector(0)[127]);
    try std.testing.expectEqual(@as(u32, 0x000000a0), std.mem.readInt(u32, card.readSector(1)[0..4], .little));
    try std.testing.expectEqual(@as(u16, 0xffff), std.mem.readInt(u16, card.readSector(1)[8..10], .little));
}

test "sector checksum includes address and payload" {
    var payload = [_]u8{0} ** sector_size;
    payload[0] = 0x12;
    payload[1] = 0x34;

    const checksum = MemoryCard.calcSectorChecksum(0x0123) ^ calcChecksum(&payload);
    try std.testing.expectEqual(@as(u8, 0x04), checksum);
}
