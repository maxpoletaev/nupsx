const std = @import("std");

const log = std.log.scoped(.config);
const config_max_size = 16 * 1024;

const Entry = struct {
    key: []const u8,
    value: []const u8,
};

pub const Config = struct {
    allocator: std.mem.Allocator,
    io: std.Io,

    path: []const u8,
    entries: std.ArrayList(Entry) = .empty,

    pub fn init(allocator: std.mem.Allocator, io: std.Io, path: []const u8) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
        const path_copy = allocator.dupe(u8, path) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .io = io,
            .path = path_copy,
        };
        self.loadConfig();
        return self;
    }

    pub fn deinit(self: *@This()) void {
        for (self.entries.items) |entry| {
            self.allocator.free(entry.key);
            self.allocator.free(entry.value);
        }
        self.entries.deinit(self.allocator);
        self.allocator.free(self.path);
        self.allocator.destroy(self);
    }

    pub fn get(self: *const @This(), key: []const u8) ?[]const u8 {
        for (self.entries.items) |entry| {
            if (std.mem.eql(u8, entry.key, key)) return entry.value;
        }
        return null;
    }

    pub fn getBool(self: *const @This(), key: []const u8) ?bool {
        const value = self.get(key) orelse return null;
        if (std.mem.eql(u8, value, "true") or std.mem.eql(u8, value, "1")) return true;
        if (std.mem.eql(u8, value, "false") or std.mem.eql(u8, value, "0")) return false;
        return null;
    }

    pub fn setBool(self: *@This(), key: []const u8, value: bool) void {
        self.set(key, if (value) "true" else "false");
    }

    pub fn set(self: *@This(), key: []const u8, value: []const u8) void {
        for (self.entries.items) |*entry| {
            if (std.mem.eql(u8, entry.key, key)) {
                self.allocator.free(entry.value);
                entry.value = self.allocator.dupe(u8, value) catch @panic("OOM");
                return;
            }
        }

        const entry: Entry = .{
            .key = self.allocator.dupe(u8, key) catch @panic("OOM"),
            .value = self.allocator.dupe(u8, value) catch @panic("OOM"),
        };
        self.entries.append(self.allocator, entry) catch @panic("OOM");
    }

    pub fn saveConfig(self: *const @This()) void {
        const file = std.Io.Dir.createFile(.cwd(), self.io, self.path, .{
            .truncate = true,
        }) catch |err| {
            log.warn("failed to save config {s}: {}", .{ self.path, err });
            return;
        };
        defer file.close(self.io);

        var write_buf: [4096]u8 = undefined;
        var writer = file.writer(self.io, &write_buf);

        for (self.entries.items) |entry| {
            writer.interface.print("{s}={s}\n", .{ entry.key, entry.value }) catch |err| {
                log.warn("failed to write config {s}: {}", .{ self.path, err });
                return;
            };
        }

        writer.interface.flush() catch |err| {
            log.warn("failed to flush config {s}: {}", .{ self.path, err });
            return;
        };

        log.info("saved config: {s}", .{self.path});
    }

    fn loadConfig(self: *@This()) void {
        const file = std.Io.Dir.openFile(.cwd(), self.io, self.path, .{}) catch |err| {
            if (err != error.FileNotFound) {
                log.warn("failed to read config {s}: {}", .{ self.path, err });
            }
            return;
        };
        defer file.close(self.io);

        const file_size = file.length(self.io) catch return;
        if (file_size > config_max_size) return;

        var read_buf: [4096]u8 = undefined;
        var reader = file.reader(self.io, &read_buf);
        const contents = reader.interface.readAlloc(self.allocator, @intCast(file_size)) catch return;
        defer self.allocator.free(contents);

        var lines = std.mem.splitScalar(u8, contents, '\n');
        while (lines.next()) |line| {
            const separator = std.mem.indexOfScalar(u8, line, '=') orelse continue;
            if (separator == 0) continue;
            self.set(line[0..separator], std.mem.trimEnd(u8, line[separator + 1 ..], "\r"));
        }

        log.info("loaded config: {s}", .{self.path});
    }
};
