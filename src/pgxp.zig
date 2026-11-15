const std = @import("std");
const Vertex = @import("renderer.zig").Vertex;

pub const VertexCache = struct {
    const dim = 2048; // SX/SY are saturated to 11 bits (-1024..1023)
    const empty = [2]f32{ std.math.nan(f32), std.math.nan(f32) };

    allocator: std.mem.Allocator,
    entries: *[dim * dim][2]f32,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
        const entries = allocator.create([dim * dim][2]f32) catch @panic("OOM");
        @memset(entries, empty);
        self.* = .{ .allocator = allocator, .entries = entries };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self.entries);
        self.allocator.destroy(self);
    }

    inline fn index(x: i32, y: i32) usize {
        const ix: u11 = @truncate(@as(u32, @bitCast(x +% 1024)));
        const iy: u11 = @truncate(@as(u32, @bitCast(y +% 1024)));
        return @as(usize, iy) * dim + ix;
    }

    pub fn store(self: *@This(), sx: i16, sy: i16, x: f32, y: f32) void {
        // H=0 and SZ3=0 gives 0/0 - clamp would turn NaN into a bogus value
        if (!std.math.isFinite(x) or !std.math.isFinite(y)) {
            self.entries[index(sx, sy)] = empty;
            return;
        }
        self.entries[index(sx, sy)] = .{
            std.math.clamp(x, -1024.0, 1023.0),
            std.math.clamp(y, -1024.0, 1023.0),
        };
    }

    pub fn apply(self: *const @This(), v: *Vertex) void {
        const p = self.entries[index(v.x, v.y)];
        const dx = p[0] - @as(f32, @floatFromInt(v.x));
        const dy = p[1] - @as(f32, @floatFromInt(v.y));
        // NaN (empty entry) fails the comparison
        if (@abs(dx) <= 1.0 and @abs(dy) <= 1.0) {
            v.pgxp_dx = dx;
            v.pgxp_dy = dy;
        }
    }
};
