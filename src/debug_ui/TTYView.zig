const std = @import("std");
const zgui = @import("zgui");

allocator: std.mem.Allocator,
lines: std.array_list.Aligned([]u8, null),
line_buf: std.array_list.Aligned(u8, null),
new_line_added: bool,

pub fn init(allocator: std.mem.Allocator) !*@This() {
    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .lines = .empty,
        .line_buf = .empty,
        .new_line_added = false,
    };
    return self;
}

pub fn deinit(self: *@This()) void {
    for (self.lines.items) |line| {
        self.allocator.free(line);
    }

    self.line_buf.deinit(self.allocator);
    self.lines.deinit(self.allocator);
    self.allocator.destroy(self);
}

pub fn writeLine(self: *@This(), line: []const u8) !void {
    const buf = try self.allocator.alloc(u8, line.len);
    @memcpy(buf, line);

    try self.lines.append(self.allocator, buf);
    self.new_line_added = true;
}

pub fn writeChar(self: *@This(), char: u8) !void {
    try self.line_buf.append(self.allocator, char);
    if (char == '\n') {
        try self.writeLine(self.line_buf.items);
        self.line_buf.clearRetainingCapacity();
    }
}

pub fn update(self: *@This()) void {
    if (zgui.begin("TTY", .{})) {
        for (self.lines.items) |line| {
            zgui.textUnformatted(line);
        }
    }

    if (self.new_line_added) {
        zgui.setScrollHereY(.{});
        self.new_line_added = false;
    }

    zgui.end();
}
