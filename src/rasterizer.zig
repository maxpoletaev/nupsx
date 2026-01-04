const std = @import("std");

const log = std.log.scoped(.rasterizer);

pub const Color24 = packed struct {
    r: u8,
    g: u8,
    b: u8,

    pub inline fn init(r: u8, g: u8, b: u8) @This() {
        return .{ .r = r, .g = g, .b = b };
    }

    pub inline fn from15(v: Color15) @This() {
        return .{
            .r = @as(u8, v.r) << 3,
            .g = @as(u8, v.g) << 3,
            .b = @as(u8, v.b) << 3,
        };
    }
};

pub const Color15 = packed struct {
    r: u5,
    g: u5,
    b: u5,
    a: u1 = 1,

    pub fn init(r: u5, g: u5, b: u5) @This() {
        return .{ .r = r, .g = g, .b = b };
    }

    pub inline fn from24(v: Color24) @This() {
        return .{
            .r = @truncate(v.r >> 3),
            .g = @truncate(v.g >> 3),
            .b = @truncate(v.b >> 3),
        };
    }
};

pub const ColorDepth = enum {
    bit4,
    bit8,
    bit15,
};

pub const Vertex = struct {
    x: i32,
    y: i32,
    u: u32 = 0,
    v: u32 = 0,
    color: Color24 = .init(0, 0, 0),
};

pub const Rasterizer = struct {
    const vram_res_x = 1024;
    const vram_res_y = 512;

    vram: *align(16) [vram_res_x * vram_res_y]u16,
    draw_area_start: [2]i32,
    draw_area_end: [2]i32,
    draw_offset: [2]i32,
    texwin_mask: [2]u16,
    texwin_offset: [2]u16,

    pub fn init(vram: *align(16) [vram_res_x * vram_res_y]u16) @This() {
        return .{
            .vram = vram,
            .texwin_mask = .{ 0, 0 },
            .texwin_offset = .{ 0, 0 },
            .draw_offset = .{ 0, 0 },
            .draw_area_start = .{ 0, 0 },
            .draw_area_end = .{ vram_res_x - 1, vram_res_y - 1 },
        };
    }

    pub inline fn setTextureWindow(self: *@This(), mask_x: u16, mask_y: u16, offset_x: u16, offset_y: u16) void {
        self.texwin_mask = .{ mask_x, mask_y };
        self.texwin_offset = .{ offset_x, offset_y };
    }

    pub inline fn setDrawAreaStart(self: *@This(), x: i32, y: i32) void {
        self.draw_area_start = .{ x, y };
    }

    pub inline fn setDrawOffset(self: *@This(), x: i32, y: i32) void {
        self.draw_offset = .{ x, y };
    }

    pub inline fn setDrawAreaEnd(self: *@This(), x: i32, y: i32) void {
        self.draw_area_end = .{ x, y };
    }

    inline fn applyOffset(self: *@This(), v: Vertex) Vertex {
        return .{
            .x = v.x + self.draw_offset[0],
            .y = v.y + self.draw_offset[1],
            .u = v.u,
            .v = v.v,
            .color = v.color,
        };
    }

    inline fn isWithinDrawArea(self: *@This(), x: i32, y: i32) bool {
        return x >= self.draw_area_start[0] and x <= self.draw_area_end[0] and
            y >= self.draw_area_start[1] and y <= self.draw_area_end[1];
    }

    pub inline fn fill(self: *@This(), c: Color24) void {
        @memset(self.vram, @bitCast(Color15.from24(c)));
    }

    inline fn toVramAddr(x: i32, y: i32) usize {
        const xx = @as(u32, @bitCast(x)) & 0x3ff; // 0..1023
        const yy = @as(u32, @bitCast(y)) & 0x1ff; // 0..511
        return xx + yy * vram_res_x;
    }

    inline fn edgeFunc(a: Vertex, b: Vertex, c: Vertex) i32 {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    pub fn drawTriangleFlat(
        self: *@This(),
        v0_orig: Vertex,
        v1_orig: Vertex,
        v2_orig: Vertex,
        c: Color24,
    ) void {
        const v0 = self.applyOffset(v0_orig);
        const v1 = self.applyOffset(v1_orig);
        const v2 = self.applyOffset(v2_orig);
        const c15 = Color15.from24(c);

        var abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return;

        const flipped = abc < 0;
        if (flipped) abc = -abc;

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], vram_res_y - 1);

        var y = y_min;

        while (y <= y_max) : (y += 1) {
            var x = x_min;

            while (x <= x_max) : (x += 1) {
                const p = Vertex{ .x = x, .y = y };
                var abp = edgeFunc(v0, v1, p);
                var bcp = edgeFunc(v1, v2, p);
                var cap = edgeFunc(v2, v0, p);

                if (flipped) {
                    abp = -abp;
                    bcp = -bcp;
                    cap = -cap;
                }

                if (abp >= 0 and bcp >= 0 and cap >= 0) {
                    self.vram[toVramAddr(x, y)] = @bitCast(c15);
                }
            }
        }
    }

    pub fn drawTriangleShaded(
        self: *@This(),
        v0_orig: Vertex,
        v1_orig: Vertex,
        v2_orig: Vertex,
    ) void {
        const v0 = self.applyOffset(v0_orig);
        const v1 = self.applyOffset(v1_orig);
        const v2 = self.applyOffset(v2_orig);

        var abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return;

        const flipped = abc < 0;
        if (flipped) abc = -abc;

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], vram_res_y - 1);

        const fixed_bits = 8;
        const fixed_one = 1 << fixed_bits;

        var y = y_min;

        while (y <= y_max) : (y += 1) {
            var x = x_min;

            while (x <= x_max) : (x += 1) {
                const p = Vertex{ .x = x, .y = y };

                var abp = edgeFunc(v0, v1, p);
                var bcp = edgeFunc(v1, v2, p);
                var cap = edgeFunc(v2, v0, p);

                if (flipped) {
                    abp = -abp;
                    bcp = -bcp;
                    cap = -cap;
                }

                if (abp >= 0 and bcp >= 0 and cap >= 0) {
                    const w0: u32 = @intCast(@divTrunc(bcp * fixed_one, abc));
                    const w1: u32 = @intCast(@divTrunc(cap * fixed_one, abc));
                    const w2: u32 = @intCast(@divTrunc(abp * fixed_one, abc));

                    const r = (v0.color.r * w0 + v1.color.r * w1 + v2.color.r * w2);
                    const g = (v0.color.g * w0 + v1.color.g * w1 + v2.color.g * w2);
                    const b = (v0.color.b * w0 + v1.color.b * w1 + v2.color.b * w2);

                    self.vram[toVramAddr(x, y)] = @bitCast(Color15.from24(.{
                        .r = @truncate(r >> fixed_bits),
                        .g = @truncate(g >> fixed_bits),
                        .b = @truncate(b >> fixed_bits),
                    }));
                }
            }
        }
    }

    fn getTexel(
        self: *@This(),
        u_orig: u16,
        v_orig: u16,
        texp_x: u16,
        texp_y: u16,
        clutx: u16,
        clut_y: u16,
        depth: ColorDepth,
    ) u16 {
        const u: u8 = @truncate((u_orig & ~self.texwin_mask[0]) | (self.texwin_offset[0] & self.texwin_mask[0]));
        const v: u8 = @truncate((v_orig & ~self.texwin_mask[1]) | (self.texwin_offset[1] & self.texwin_mask[1]));

        switch (depth) {
            .bit4 => {
                const texel = self.vram[toVramAddr(texp_x + u / 4, texp_y + v)];
                const shift = @as(u4, @truncate((u % 4) * 4));
                const offset = (texel >> shift) & 0xf;
                return self.vram[toVramAddr(clutx + offset, clut_y)];
            },
            .bit8 => {
                const texel = self.vram[toVramAddr(texp_x + u / 2, texp_y + v)];
                const shift = @as(u4, @truncate((u % 2) * 8));
                const offset = (texel >> shift) & 0xff;
                return self.vram[toVramAddr(clutx + offset, clut_y)];
            },
            .bit15 => {
                return self.vram[toVramAddr(texp_x + u, texp_y + v)];
            },
        }
    }

    pub fn drawTriangleTextured(
        self: *@This(),
        v0_orig: Vertex,
        v1_orig: Vertex,
        v2_orig: Vertex,
        clutx: u16,
        clut_y: u16,
        tpx: u16,
        tpy: u16,
        depth: ColorDepth,
    ) void {
        const v0 = self.applyOffset(v0_orig);
        const v1 = self.applyOffset(v1_orig);
        const v2 = self.applyOffset(v2_orig);

        const abc = edgeFunc(v0, v1, v2);
        if (abc < 0) return;

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], vram_res_y - 1);

        const fixed_bits = 8;
        const fixed_one = 1 << fixed_bits;

        var y = y_min;

        while (y <= y_max) : (y += 1) {
            var x = x_min;

            while (x <= x_max) : (x += 1) {
                const p = Vertex{ .x = x, .y = y };

                const abp = edgeFunc(v0, v1, p);
                const bcp = edgeFunc(v1, v2, p);
                const cap = edgeFunc(v2, v0, p);

                if (abp > 0 and bcp > 0 and cap > 0) {
                    const w0: u32 = @intCast(@divTrunc(bcp * fixed_one, abc));
                    const w1: u32 = @intCast(@divTrunc(cap * fixed_one, abc));
                    const w2: u32 = @intCast(@divTrunc(abp * fixed_one, abc));

                    const tx = v0.u * w0 + v1.u * w1 + v2.u * w2;
                    const ty = v0.v * w0 + v1.v * w1 + v2.v * w2;

                    const texel = self.getTexel(
                        @truncate(tx >> fixed_bits),
                        @truncate(ty >> fixed_bits),
                        tpx,
                        tpy,
                        clutx,
                        clut_y,
                        depth,
                    );

                    if (texel == 0) continue;
                    self.vram[toVramAddr(x, y)] = texel;
                }
            }
        }
    }

    pub fn drawRectFlat(
        self: *@This(),
        x_orig: i32,
        y_orig: i32,
        w: i32,
        h: i32,
        c: Color24,
    ) void {
        const x = x_orig + self.draw_offset[0];
        const y = y_orig + self.draw_offset[1];
        const c15 = Color15.from24(c);

        const x_min = @max(x, self.draw_area_start[0], 0);
        const y_min = @max(y, self.draw_area_start[1], 0);
        const x_max = @min(x + w - 1, self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(y + h - 1, self.draw_area_end[1], vram_res_y - 1);

        var yy = y_min;
        while (yy <= y_max) : (yy += 1) {
            var xx = x_min;
            while (xx <= x_max) : (xx += 1) {
                self.vram[toVramAddr(xx, yy)] = @bitCast(c15);
            }
        }
    }

    pub fn drawRectTextured(
        self: *@This(),
        x_orig: i32,
        y_orig: i32,
        w: i32,
        h: i32,
        u: u16,
        v: u16,
        clut_x: u16,
        clut_y: u16,
        texp_x: u16,
        texp_y: u16,
        depth: ColorDepth,
    ) void {
        const x = x_orig + self.draw_offset[0];
        const y = y_orig + self.draw_offset[1];

        const x_min = @max(x, self.draw_area_start[0], 0);
        const y_min = @max(y, self.draw_area_start[1], 0);
        const x_max = @min(x + w - 1, self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(y + h - 1, self.draw_area_end[1], vram_res_y - 1);

        const u_start = u;
        const v_start = v;

        var v_curr = v_start;
        var py = y_min;

        while (py <= y_max) : (py += 1) {
            var u_curr = u_start;
            var px = x_min;

            while (px <= x_max) : (px += 1) {
                const texel = self.getTexel(u_curr, v_curr, texp_x, texp_y, clut_x, clut_y, depth);
                if (texel != 0) self.vram[toVramAddr(px, py)] = texel;
                u_curr += 1;
            }

            v_curr += 1;
        }
    }

    pub fn drawLineFlat(
        self: *@This(),
        x0_orig: i32,
        y0_orig: i32,
        x1_orig: i32,
        y1_orig: i32,
        c: Color24,
    ) void {
        const x0 = x0_orig + self.draw_offset[0];
        const y0 = y0_orig + self.draw_offset[1];
        const x1 = x1_orig + self.draw_offset[0];
        const y1 = y1_orig + self.draw_offset[1];

        const dx: i32 = @intCast(@abs(x1 - x0));
        const dy: i32 = @intCast(@abs(y1 - y0));
        const sx: i32 = if (x0 < x1) 1 else -1;
        const sy: i32 = if (y0 < y1) 1 else -1;
        var err = dx - dy;

        var x = x0;
        var y = y0;

        const c15 = Color15.from24(c);

        while (true) {
            if (self.isWithinDrawArea(x, y)) {
                self.vram[toVramAddr(x, y)] = @bitCast(c15);
            }
            if (x == x1 and y == y1) break;
            const err2 = err * 2;
            if (err2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (err2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
};
