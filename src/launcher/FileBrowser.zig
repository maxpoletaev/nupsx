const std = @import("std");
const zgui = @import("zgui");

pub const max_path_len = 1024;

const title_color: [4]f32 = .{ 0.3, 0.7, 1.0, 1.0 };

pub const FileKind = enum {
    bios,
    game,
    memcard,

    fn extensions(self: FileKind) []const []const u8 {
        return switch (self) {
            .bios => &.{ ".bin", ".rom" },
            .game => &.{ ".cue", ".bin", ".exe" },
            .memcard => &.{ ".mcd", ".bin" },
        };
    }

    fn prompt(self: FileKind) [:0]const u8 {
        return switch (self) {
            .bios => "Select BIOS (.bin, .rom)",
            .game => "Select Game Image (.cue, .bin, .exe)",
            .memcard => "Select Memory Card (.mcd)",
        };
    }

    pub fn matches(self: FileKind, name: []const u8) bool {
        for (self.extensions()) |ext| {
            if (name.len < ext.len) continue;
            if (std.ascii.eqlIgnoreCase(name[name.len - ext.len ..], ext)) return true;
        }
        return false;
    }
};

pub const PathInput = struct {
    buf: [max_path_len:0]u8 = @splat(0),

    pub fn set(self: *PathInput, new_path: []const u8) void {
        const len = @min(new_path.len, self.buf.len - 1);
        @memcpy(self.buf[0..len], new_path[0..len]);
        self.buf[len] = 0;
    }

    pub fn path(self: *const PathInput) [:0]const u8 {
        return std.mem.sliceTo(&self.buf, 0);
    }
};

const Entry = struct {
    name: [:0]u8,
    is_dir: bool,

    fn lessThan(_: void, a: Entry, b: Entry) bool {
        if (a.is_dir != b.is_dir) return a.is_dir;
        return std.ascii.lessThanIgnoreCase(a.name, b.name);
    }
};

allocator: std.mem.Allocator,
io: std.Io,

is_open: bool = false,
kind: FileKind = .bios,
target: *PathInput = undefined,

current_dir: std.ArrayList(u8) = .empty,
entries: std.ArrayList(Entry) = .empty,

pub fn init(allocator: std.mem.Allocator, io: std.Io, start_dir: []const u8) *@This() {
    const self = allocator.create(@This()) catch @panic("OOM");
    self.* = .{ .allocator = allocator, .io = io };
    self.setCurrentDir(start_dir);
    return self;
}

pub fn deinit(self: *@This()) void {
    self.clearEntries();
    self.entries.deinit(self.allocator);
    self.current_dir.deinit(self.allocator);
    self.allocator.destroy(self);
}

pub fn open(self: *@This(), kind: FileKind, target: *PathInput) void {
    self.kind = kind;
    self.target = target;
    self.is_open = true;
    self.refreshEntries();
}

pub fn currentDir(self: *const @This()) []const u8 {
    return self.current_dir.items;
}

pub fn update(self: *@This(), parent_w: f32, parent_h: f32) bool {
    if (!self.is_open) return false;

    zgui.openPopup("Select File", .{});

    const modal_w = parent_w - 40;
    const modal_h = parent_h - 40;
    zgui.setNextWindowSize(.{ .w = modal_w, .h = modal_h, .cond = .always });
    zgui.setNextWindowPos(.{ .x = 20, .y = 20 });

    var picked = false;

    if (zgui.beginPopupModal("Select File", .{ .popen = &self.is_open })) {
        zgui.textColored(title_color, "{s}", .{self.kind.prompt()});
        zgui.separator();

        // navigation bar: up button + current path
        if (zgui.button("Up [..]", .{ .w = 70, .h = 0 })) {
            self.navigateUp();
        }
        zgui.sameLine(.{ .spacing = 10 });
        zgui.textWrapped("{s}", .{self.current_dir.items});
        zgui.separator();

        // file and directory list
        if (zgui.beginChild("FileList", .{ .w = 0, .h = modal_h - 130 })) {
            for (self.entries.items) |entry| {
                var label_buf: [max_path_len + 16:0]u8 = undefined;
                const prefix = if (entry.is_dir) "[DIR] " else "      ";
                const label = std.fmt.bufPrintZ(&label_buf, "{s}{s}", .{ prefix, entry.name }) catch continue;

                if (zgui.selectable(label, .{})) {
                    if (entry.is_dir) {
                        self.navigateInto(entry.name);
                    } else {
                        self.selectFile(entry.name);
                        picked = true;
                    }
                    break;
                }
            }
        }
        zgui.endChild();

        // footer buttons
        zgui.separator();
        if (zgui.button("Cancel", .{ .w = 100, .h = 0 })) {
            self.is_open = false;
            zgui.closeCurrentPopup();
        }

        zgui.endPopup();
    }

    return picked;
}

fn clearEntries(self: *@This()) void {
    for (self.entries.items) |entry| {
        self.allocator.free(entry.name);
    }
    self.entries.clearRetainingCapacity();
}

fn openDir(self: *@This(), path: []const u8) !std.Io.Dir {
    if (std.fs.path.isAbsolute(path)) {
        return std.Io.Dir.openDirAbsolute(self.io, path, .{ .iterate = true });
    } else {
        return std.Io.Dir.openDir(.cwd(), self.io, path, .{ .iterate = true });
    }
}

fn refreshEntries(self: *@This()) void {
    self.clearEntries();

    const path = self.current_dir.items;
    var dir = openDir(self, path) catch return;
    defer dir.close(self.io);

    var iter = dir.iterate();
    while (iter.next(self.io) catch null) |entry| {
        if (std.mem.startsWith(u8, entry.name, ".")) continue;

        const is_dir = entry.kind == .directory;
        if (!is_dir and !self.kind.matches(entry.name)) continue;

        self.entries.append(self.allocator, .{
            .name = self.allocator.dupeZ(u8, entry.name) catch @panic("OOM"),
            .is_dir = is_dir,
        }) catch @panic("OOM");
    }

    std.mem.sort(Entry, self.entries.items, {}, Entry.lessThan);
}

pub fn setCurrentDir(self: *@This(), path: []const u8) void {
    self.current_dir.clearRetainingCapacity();
    self.current_dir.appendSlice(self.allocator, path) catch @panic("OOM");
    self.refreshEntries();
}

fn navigateUp(self: *@This()) void {
    const parent = std.fs.path.dirname(self.current_dir.items) orelse return;
    if (parent.len == 0) return;

    self.current_dir.shrinkRetainingCapacity(parent.len);
    self.refreshEntries();
}

fn navigateInto(self: *@This(), sub_dir: []const u8) void {
    const new_path = std.fs.path.join(self.allocator, &.{ self.current_dir.items, sub_dir }) catch @panic("OOM");
    defer self.allocator.free(new_path);
    self.setCurrentDir(new_path);
}

fn selectFile(self: *@This(), filename: []const u8) void {
    const full_path = std.fs.path.join(self.allocator, &.{ self.current_dir.items, filename }) catch @panic("OOM");
    defer self.allocator.free(full_path);

    self.target.set(full_path);
    self.is_open = false;
}
