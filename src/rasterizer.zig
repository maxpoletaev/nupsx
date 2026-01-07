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
    a: u1 = 0,

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

const fp_bits = 12;
const fp_one: i32 = 1 << fp_bits;

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

    fn sampleTexture(
        self: *@This(),
        u_orig: u16,
        v_orig: u16,
        texp_x: u16,
        texp_y: u16,
        clut_x: u16,
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
                return self.vram[toVramAddr(clut_x + offset, clut_y)];
            },
            .bit8 => {
                const texel = self.vram[toVramAddr(texp_x + u / 2, texp_y + v)];
                const shift = @as(u4, @truncate((u % 2) * 8));
                const offset = (texel >> shift) & 0xff;
                return self.vram[toVramAddr(clut_x + offset, clut_y)];
            },
            .bit15 => {
                return self.vram[toVramAddr(texp_x + u, texp_y + v)];
            },
        }
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

        const abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return;

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], vram_res_y - 1);

        const abp_dx = v0.y - v1.y;
        const abp_dy = v1.x - v0.x;
        const bcp_dx = v1.y - v2.y;
        const bcp_dy = v2.x - v1.x;
        const cap_dx = v2.y - v0.y;
        const cap_dy = v0.x - v2.x;

        const p = Vertex{ .x = x_min, .y = y_min };
        var abp_row = edgeFunc(v0, v1, p);
        var bcp_row = edgeFunc(v1, v2, p);
        var cap_row = edgeFunc(v2, v0, p);

        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var abp = abp_row;
            var bcp = bcp_row;
            var cap = cap_row;

            var x = x_min;
            while (x <= x_max) : (x += 1) {
                const inside = if (abc > 0) (abp >= 0 and bcp >= 0 and cap >= 0) else (abp <= 0 and bcp <= 0 and cap <= 0);

                if (inside) self.vram[toVramAddr(x, y)] = @bitCast(c15);

                abp += abp_dx;
                bcp += bcp_dx;
                cap += cap_dx;
            }

            abp_row += abp_dy;
            bcp_row += bcp_dy;
            cap_row += cap_dy;
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

        const abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return;

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], vram_res_y - 1);

        const abp_dx = v0.y - v1.y;
        const abp_dy = v1.x - v0.x;
        const bcp_dx = v1.y - v2.y;
        const bcp_dy = v2.x - v1.x;
        const cap_dx = v2.y - v0.y;
        const cap_dy = v0.x - v2.x;

        const p = Vertex{ .x = x_min, .y = y_min };
        const abp_row_start = edgeFunc(v0, v1, p);
        const bcp_row_start = edgeFunc(v1, v2, p);
        const cap_row_start = edgeFunc(v2, v0, p);

        const r0: i32 = v0.color.r;
        const r1: i32 = v1.color.r;
        const r2: i32 = v2.color.r;
        const g0: i32 = v0.color.g;
        const g1: i32 = v1.color.g;
        const g2: i32 = v2.color.g;
        const b0: i32 = v0.color.b;
        const b1: i32 = v1.color.b;
        const b2: i32 = v2.color.b;

        const r_dx = @divTrunc(((r1 - r0) * (v2.y - v0.y) - (r2 - r0) * (v1.y - v0.y)) * fp_one, abc);
        const r_dy = @divTrunc(((r2 - r0) * (v1.x - v0.x) - (r1 - r0) * (v2.x - v0.x)) * fp_one, abc);
        const g_dx = @divTrunc(((g1 - g0) * (v2.y - v0.y) - (g2 - g0) * (v1.y - v0.y)) * fp_one, abc);
        const g_dy = @divTrunc(((g2 - g0) * (v1.x - v0.x) - (g1 - g0) * (v2.x - v0.x)) * fp_one, abc);
        const b_dx = @divTrunc(((b1 - b0) * (v2.y - v0.y) - (b2 - b0) * (v1.y - v0.y)) * fp_one, abc);
        const b_dy = @divTrunc(((b2 - b0) * (v1.x - v0.x) - (b1 - b0) * (v2.x - v0.x)) * fp_one, abc);

        var r_row = r0 * fp_one + (x_min - v0.x) * r_dx + (y_min - v0.y) * r_dy;
        var g_row = g0 * fp_one + (x_min - v0.x) * g_dx + (y_min - v0.y) * g_dy;
        var b_row = b0 * fp_one + (x_min - v0.x) * b_dx + (y_min - v0.y) * b_dy;

        var abp_row = abp_row_start;
        var bcp_row = bcp_row_start;
        var cap_row = cap_row_start;

        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var abp = abp_row;
            var bcp = bcp_row;
            var cap = cap_row;
            var r = r_row;
            var g = g_row;
            var b = b_row;

            var x = x_min;
            while (x <= x_max) : (x += 1) {
                const inside = if (abc > 0) (abp >= 0 and bcp >= 0 and cap >= 0) else (abp <= 0 and bcp <= 0 and cap <= 0);

                if (inside) {
                    self.vram[toVramAddr(x, y)] = @bitCast(Color15.from24(.{
                        .r = @truncate(@as(u32, @bitCast(r >> fp_bits))),
                        .g = @truncate(@as(u32, @bitCast(g >> fp_bits))),
                        .b = @truncate(@as(u32, @bitCast(b >> fp_bits))),
                    }));
                }

                abp += abp_dx;
                bcp += bcp_dx;
                cap += cap_dx;
                r += r_dx;
                g += g_dx;
                b += b_dx;
            }

            abp_row += abp_dy;
            bcp_row += bcp_dy;
            cap_row += cap_dy;
            r_row += r_dy;
            g_row += g_dy;
            b_row += b_dy;
        }
    }

    pub fn drawTriangleTextured(
        self: *@This(),
        v0_orig: Vertex,
        v1_orig: Vertex,
        v2_orig: Vertex,
        clut_x: u16,
        clut_y: u16,
        texp_x: u16,
        texp_y: u16,
        depth: ColorDepth,
    ) void {
        const v0 = self.applyOffset(v0_orig);
        const v1 = self.applyOffset(v1_orig);
        const v2 = self.applyOffset(v2_orig);

        const abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return;

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], vram_res_y - 1);

        const abp_dx = v0.y - v1.y;
        const abp_dy = v1.x - v0.x;
        const bcp_dx = v1.y - v2.y;
        const bcp_dy = v2.x - v1.x;
        const cap_dx = v2.y - v0.y;
        const cap_dy = v0.x - v2.x;

        const p = Vertex{ .x = x_min, .y = y_min };
        const abp_row_start = edgeFunc(v0, v1, p);
        const bcp_row_start = edgeFunc(v1, v2, p);
        const cap_row_start = edgeFunc(v2, v0, p);

        const tex_u0: i32 = @intCast(v0.u);
        const tex_u1: i32 = @intCast(v1.u);
        const tex_u2: i32 = @intCast(v2.u);
        const tex_v0: i32 = @intCast(v0.v);
        const tex_v1: i32 = @intCast(v1.v);
        const tex_v2: i32 = @intCast(v2.v);

        const u_dx = @divTrunc(((tex_u1 - tex_u0) * (v2.y - v0.y) - (tex_u2 - tex_u0) * (v1.y - v0.y)) * fp_one, abc);
        const u_dy = @divTrunc(((tex_u2 - tex_u0) * (v1.x - v0.x) - (tex_u1 - tex_u0) * (v2.x - v0.x)) * fp_one, abc);
        const v_dx = @divTrunc(((tex_v1 - tex_v0) * (v2.y - v0.y) - (tex_v2 - tex_v0) * (v1.y - v0.y)) * fp_one, abc);
        const v_dy = @divTrunc(((tex_v2 - tex_v0) * (v1.x - v0.x) - (tex_v1 - tex_v0) * (v2.x - v0.x)) * fp_one, abc);

        var u_row = tex_u0 * fp_one + (x_min - v0.x) * u_dx + (y_min - v0.y) * u_dy;
        var v_row = tex_v0 * fp_one + (x_min - v0.x) * v_dx + (y_min - v0.y) * v_dy;

        var abp_row = abp_row_start;
        var bcp_row = bcp_row_start;
        var cap_row = cap_row_start;

        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var abp = abp_row;
            var bcp = bcp_row;
            var cap = cap_row;
            var u = u_row;
            var v = v_row;

            var x = x_min;
            while (x <= x_max) : (x += 1) {
                const inside = if (abc > 0) (abp >= 0 and bcp >= 0 and cap >= 0) else (abp <= 0 and bcp <= 0 and cap <= 0);

                if (inside) {
                    const texel = self.sampleTexture(
                        @truncate(@as(u32, @bitCast(u >> fp_bits))),
                        @truncate(@as(u32, @bitCast(v >> fp_bits))),
                        texp_x,
                        texp_y,
                        clut_x,
                        clut_y,
                        depth,
                    );

                    if (texel != 0) {
                        self.vram[toVramAddr(x, y)] = texel;
                    }
                }

                abp += abp_dx;
                bcp += bcp_dx;
                cap += cap_dx;
                u += u_dx;
                v += v_dx;
            }

            abp_row += abp_dy;
            bcp_row += bcp_dy;
            cap_row += cap_dy;
            u_row += u_dy;
            v_row += v_dy;
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
                const texel = self.sampleTexture(u_curr, v_curr, texp_x, texp_y, clut_x, clut_y, depth);
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

        if (@max(x0, x1) < self.draw_area_start[0] or @min(x0, x1) > self.draw_area_end[0]) return;
        if (@max(y0, y1) < self.draw_area_start[1] or @min(y0, y1) > self.draw_area_end[1]) return;

        const dx = x1 - x0;
        const dy = y1 - y0;

        const c15 = Color15.from24(c);

        const steps: i32 = @intCast(@max(@abs(dx), @abs(dy)));

        if (steps == 0) {
            self.vram[toVramAddr(x0, y0)] = @bitCast(c15);
            return;
        }

        const x_dx = @divTrunc(dx * fp_one, steps);
        const y_dx = @divTrunc(dy * fp_one, steps);

        var x_fp = x0 * fp_one;
        var y_fp = y0 * fp_one;

        var i: i32 = 0;
        while (i <= steps) : (i += 1) {
            const x = x_fp >> fp_bits;
            const y = y_fp >> fp_bits;

            self.vram[toVramAddr(x, y)] = @bitCast(c15);

            x_fp += x_dx;
            y_fp += y_dx;
        }
    }
};
