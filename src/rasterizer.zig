const std = @import("std");

const log = std.log.scoped(.rasterizer);

pub const RGB8 = packed struct {
    pub const black: RGB8 = .{ .r = 0, .g = 0, .b = 0 };

    r: u8,
    g: u8,
    b: u8,

    pub inline fn init(r: u8, g: u8, b: u8) @This() {
        return .{ .r = r, .g = g, .b = b };
    }

    pub inline fn from15(v: RGB5) @This() {
        return .{
            .r = @as(u8, v.r) << 3,
            .g = @as(u8, v.g) << 3,
            .b = @as(u8, v.b) << 3,
        };
    }
};

pub const RGB5 = packed struct {
    r: u5,
    g: u5,
    b: u5,
    mask_bit: bool,

    pub fn init(r: u5, g: u5, b: u5, mask_bit: bool) @This() {
        return .{ .r = r, .g = g, .b = b, .mask_bit = mask_bit };
    }

    inline fn isZero(self: RGB5) bool {
        return @as(u16, @bitCast(self)) == 0;
    }
};

fn toRGB5(c: RGB8, mask_bit: bool) RGB5 {
    return .{
        .r = @truncate(c.r >> 3),
        .g = @truncate(c.g >> 3),
        .b = @truncate(c.b >> 3),
        .mask_bit = mask_bit,
    };
}

pub const ColorDepth = enum {
    bit4,
    bit8,
    bit15,
};

pub const TransparencyMode = enum {
    @"B/2+F/2",
    @"B+F",
    @"B-F",
    @"B+F/4",
};

pub const Vertex = struct {
    x: i32,
    y: i32,
    u: u32 = 0,
    v: u32 = 0,
    color: RGB8 = .init(0, 0, 0),
};

const fp_bits = 12;
const fp_one: i32 = 1 << fp_bits;

pub const Rasterizer = struct {
    const vram_res_x = 1024;
    const vram_res_y = 512;

    vram: *align(16) [vram_res_x * vram_res_y]u16,
    transparency_mode: TransparencyMode,
    draw_area_start: [2]i32,
    draw_area_end: [2]i32,
    draw_offset: [2]i32,
    texwin_mask: [2]u16,
    texwin_offset: [2]u16,
    force_mask_bit: bool = false,
    check_mask_bit: bool = false,
    enable_dithering: bool = false,

    pub fn init(vram: *align(16) [vram_res_x * vram_res_y]u16) @This() {
        return .{
            .vram = vram,
            .texwin_mask = .{ 0, 0 },
            .texwin_offset = .{ 0, 0 },
            .draw_offset = .{ 0, 0 },
            .draw_area_start = .{ 0, 0 },
            .transparency_mode = .@"B+F",
            .draw_area_end = .{ vram_res_x - 1, vram_res_y - 1 },
        };
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    pub fn setTransparencyMode(self: *@This(), mode: TransparencyMode) void {
        self.transparency_mode = mode;
    }

    pub fn setTextureWindow(self: *@This(), mask_x: u16, mask_y: u16, offset_x: u16, offset_y: u16) void {
        self.texwin_mask = .{ mask_x, mask_y };
        self.texwin_offset = .{ offset_x, offset_y };
    }

    pub fn setDrawAreaStart(self: *@This(), x: i32, y: i32) void {
        self.draw_area_start = .{ x, y };
    }

    pub fn setDrawOffset(self: *@This(), x: i32, y: i32) void {
        self.draw_offset = .{ x, y };
    }

    pub fn setDrawAreaEnd(self: *@This(), x: i32, y: i32) void {
        self.draw_area_end = .{ x, y };
    }

    pub fn setMaskBitSetting(self: *@This(), force_mask_bit: bool, check_mask_bit: bool) void {
        self.force_mask_bit = force_mask_bit;
        self.check_mask_bit = check_mask_bit;
    }

    pub fn setDithering(self: *@This(), enable: bool) void {
        self.enable_dithering = enable;
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    inline fn applyOffset(self: *@This(), v: Vertex) Vertex {
        return .{
            .x = v.x + self.draw_offset[0],
            .y = v.y + self.draw_offset[1],
            .u = v.u,
            .v = v.v,
            .color = v.color,
        };
    }

    pub inline fn fill(self: *@This(), c: RGB8) void {
        @memset(self.vram, @bitCast(toRGB5(c, false)));
    }

    inline fn toVramAddr(x: i32, y: i32) usize {
        const xx = @as(u32, @bitCast(x)) & 0x3ff; // 0..1023
        const yy = @as(u32, @bitCast(y)) & 0x1ff; // 0..511
        return xx + yy * vram_res_x;
    }

    const dithering_table: [4][4]i8 = .{
        .{ -4, 0, -3, 1 },
        .{ 2, -2, 3, -1 },
        .{ -3, 1, -4, 0 },
        .{ 3, -1, 2, -2 },
    };

    inline fn applyDithering(color: RGB8, x: i32, y: i32) RGB8 {
        const dither_x: u8 = @intCast(y & 0x3);
        const dither_y: u8 = @intCast(x & 0x3);

        const v = dithering_table[dither_y][dither_x];
        const r = @as(i16, color.r) + v;
        const g = @as(i16, color.g) + v;
        const b = @as(i16, color.b) + v;

        return .{
            .r = @intCast(std.math.clamp(r, 0, 255)),
            .g = @intCast(std.math.clamp(g, 0, 255)),
            .b = @intCast(std.math.clamp(b, 0, 255)),
        };
    }

    inline fn applyTransparency(front: RGB5, back: RGB5, mode: TransparencyMode) RGB5 {
        if (!front.mask_bit) {
            return front; // not semi-transparent pixel
        }

        const out: RGB5 = switch (mode) {
            .@"B/2+F/2" => .{
                .r = back.r / 2 +| front.r / 2,
                .g = back.g / 2 +| front.g / 2,
                .b = back.b / 2 +| front.b / 2,
                .mask_bit = front.mask_bit,
            },
            .@"B+F" => .{
                .r = back.r +| front.r,
                .g = back.g +| front.g,
                .b = back.b +| front.b,
                .mask_bit = front.mask_bit,
            },
            .@"B-F" => .{
                .r = back.r -| front.r,
                .g = back.g -| front.g,
                .b = back.b -| front.b,
                .mask_bit = front.mask_bit,
            },
            .@"B+F/4" => .{
                .r = back.r +| (front.r / 4),
                .g = back.g +| (front.g / 4),
                .b = back.b +| (front.b / 4),
                .mask_bit = front.mask_bit,
            },
        };
        return @bitCast(out);
    }

    fn applyBlending(texel: RGB5, color: RGB8) RGB5 {
        const r = (@as(u32, texel.r)) * @as(u32, color.r) / 128;
        const g = (@as(u32, texel.g)) * @as(u32, color.g) / 128;
        const b = (@as(u32, texel.b)) * @as(u32, color.b) / 128;

        return .{
            .r = @intCast(@min(r, 31)),
            .g = @intCast(@min(g, 31)),
            .b = @intCast(@min(b, 31)),
            .mask_bit = texel.mask_bit,
        };
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
    ) RGB5 {
        // const u: u8 = @truncate((u_orig & ~self.texwin_mask[0]) | (self.texwin_offset[0] & self.texwin_mask[0]));
        // const v: u8 = @truncate((v_orig & ~self.texwin_mask[1]) | (self.texwin_offset[1] & self.texwin_mask[1]));
        const u = u_orig;
        const v = v_orig;

        switch (depth) {
            .bit4 => {
                const texel = self.vram[toVramAddr(texp_x + u / 4, texp_y +% v)];
                const shift = @as(u4, @truncate((u % 4) * 4));
                const offset = (texel >> shift) & 0xf;
                return @bitCast(self.vram[toVramAddr(clut_x + offset, clut_y)]);
            },
            .bit8 => {
                const texel = self.vram[toVramAddr(texp_x + u / 2, texp_y +% v)];
                const shift = @as(u4, @truncate((u % 2) * 8));
                const offset = (texel >> shift) & 0xff;
                return @bitCast(self.vram[toVramAddr(clut_x + offset, clut_y)]);
            },
            .bit15 => {
                return @bitCast(self.vram[toVramAddr(texp_x + u, texp_y + v)]);
            },
        }
    }

    pub inline fn setPixelRaw(self: *@This(), x: i32, y: i32, color: u16) void {
        const addr = toVramAddr(x, y);
        if (self.check_mask_bit) {
            const curr = self.vram[addr];
            if (curr & (1 << 15) != 0) return; // write-protected
        }
        var out = color;
        if (self.force_mask_bit) out |= (1 << 15);
        self.vram[addr] = out;
    }

    inline fn setPixelFlat(
        self: *@This(),
        x: i32,
        y: i32,
        color: RGB8,
        comptime mode: struct {
            dither: bool = false,
            semi_trans: bool = false,
        },
    ) void {
        const addr = toVramAddr(x, y);
        const back: RGB5 = @bitCast(self.vram[addr]);

        if (self.check_mask_bit and back.mask_bit) {
            // write-protected
        } else {
            var dithered = color;
            if (mode.dither and self.enable_dithering) dithered = applyDithering(dithered, x, y);

            var out = toRGB5(dithered, mode.semi_trans);
            if (mode.semi_trans) out = applyTransparency(out, back, self.transparency_mode);
            if (self.force_mask_bit) out.mask_bit = true;

            self.vram[addr] = @bitCast(out);
        }
    }

    inline fn setPixelTextured(
        self: *@This(),
        x: i32,
        y: i32,
        texel: RGB5,
        blend_color: RGB8,
        comptime mode: struct {
            semi_trans: bool = false,
            dither: bool = false,
            blend: bool = false,
        },
    ) void {
        const addr = toVramAddr(x, y);
        const back: RGB5 = @bitCast(self.vram[addr]);

        if (self.check_mask_bit and back.mask_bit) {
            // write-protected
        } else {
            var dithered = blend_color;
            if (mode.dither and self.enable_dithering) dithered = applyDithering(dithered, x, y);

            var out = texel;
            if (mode.blend) out = applyBlending(out, dithered);
            if (mode.semi_trans) out = applyTransparency(out, back, self.transparency_mode);
            if (self.force_mask_bit) out.mask_bit = true;

            self.vram[addr] = @bitCast(out);
        }
    }

    // =========================================================================
    // Triangle Primitives
    // =========================================================================

    inline fn edgeFunc(a: Vertex, b: Vertex, c: Vertex) i32 {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    inline fn isTopLeft(v0: Vertex, v1: Vertex) bool {
        const dx = v1.x - v0.x;
        const dy = v1.y - v0.y;
        return (dy < 0) or (dy == 0 and dx < 0);
    }

    pub fn drawTriangleFlat(
        self: *@This(),
        v0_orig: Vertex,
        v1_orig: Vertex,
        v2_orig: Vertex,
        color: RGB8,
        comptime semi_trans: bool,
    ) void {
        const v0 = self.applyOffset(v0_orig);
        var v1 = self.applyOffset(v1_orig);
        var v2 = self.applyOffset(v2_orig);

        var abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return; // degenerate triangle
        if (abc < 0) {
            // ensure clockwise winding
            std.mem.swap(Vertex, &v1, &v2);
            abc = -abc;
        }

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

        const bias0: i32 = if (isTopLeft(v0, v1)) 0 else -1;
        const bias1: i32 = if (isTopLeft(v1, v2)) 0 else -1;
        const bias2: i32 = if (isTopLeft(v2, v0)) 0 else -1;

        const p = Vertex{ .x = x_min, .y = y_min };
        var abp_row = edgeFunc(v0, v1, p) + bias0;
        var bcp_row = edgeFunc(v1, v2, p) + bias1;
        var cap_row = edgeFunc(v2, v0, p) + bias2;

        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var abp = abp_row;
            var bcp = bcp_row;
            var cap = cap_row;

            var x = x_min;
            while (x <= x_max) : (x += 1) {
                const inside = (abp >= 0 and bcp >= 0 and cap >= 0);

                if (inside) {
                    self.setPixelFlat(x, y, color, .{
                        .semi_trans = semi_trans,
                    });
                }

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
        comptime semi_trans: bool,
    ) void {
        const v0 = self.applyOffset(v0_orig);
        var v1 = self.applyOffset(v1_orig);
        var v2 = self.applyOffset(v2_orig);

        var abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return; // degenerate triangle
        if (abc < 0) {
            // ensure clockwise winding
            std.mem.swap(Vertex, &v1, &v2);
            abc = -abc;
        }

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

        const bias0: i32 = if (isTopLeft(v0, v1)) 0 else -1;
        const bias1: i32 = if (isTopLeft(v1, v2)) 0 else -1;
        const bias2: i32 = if (isTopLeft(v2, v0)) 0 else -1;

        const p = Vertex{ .x = x_min, .y = y_min };
        const abp_row_start = edgeFunc(v0, v1, p) + bias0;
        const bcp_row_start = edgeFunc(v1, v2, p) + bias1;
        const cap_row_start = edgeFunc(v2, v0, p) + bias2;

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
                const inside = (abp >= 0 and bcp >= 0 and cap >= 0);

                if (inside) {
                    const color: RGB8 = .{
                        .r = @truncate(@as(u32, @bitCast(r >> fp_bits))),
                        .g = @truncate(@as(u32, @bitCast(g >> fp_bits))),
                        .b = @truncate(@as(u32, @bitCast(b >> fp_bits))),
                    };
                    self.setPixelFlat(x, y, color, .{
                        .semi_trans = semi_trans,
                        .dither = true,
                    });
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
        blend_color: RGB8,
        comptime semi_trans: bool,
        comptime tex_blend: bool,
    ) void {
        const v0 = self.applyOffset(v0_orig);
        var v1 = self.applyOffset(v1_orig);
        var v2 = self.applyOffset(v2_orig);

        var abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return; // degenerate triangle
        if (abc < 0) {
            // ensure clockwise winding
            std.mem.swap(Vertex, &v1, &v2);
            abc = -abc;
        }

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

        const bias0: i32 = if (isTopLeft(v0, v1)) 0 else -1;
        const bias1: i32 = if (isTopLeft(v1, v2)) 0 else -1;
        const bias2: i32 = if (isTopLeft(v2, v0)) 0 else -1;

        const p = Vertex{ .x = x_min, .y = y_min };
        const abp_row_start = edgeFunc(v0, v1, p) + bias0;
        const bcp_row_start = edgeFunc(v1, v2, p) + bias1;
        const cap_row_start = edgeFunc(v2, v0, p) + bias2;

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
                const inside = (abp >= 0 and bcp >= 0 and cap >= 0);

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

                    if (!texel.isZero()) {
                        self.setPixelTextured(x, y, texel, blend_color, .{
                            .semi_trans = semi_trans,
                            .blend = tex_blend,
                        });
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

    pub fn drawTriangleShadedTextured(
        self: *@This(),
        v0_orig: Vertex,
        v1_orig: Vertex,
        v2_orig: Vertex,
        clut_x: u16,
        clut_y: u16,
        texp_x: u16,
        texp_y: u16,
        depth: ColorDepth,
        comptime semi_trans: bool,
    ) void {
        const v0 = self.applyOffset(v0_orig);
        var v1 = self.applyOffset(v1_orig);
        var v2 = self.applyOffset(v2_orig);

        var abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return; // degenerate triangle
        if (abc < 0) {
            // ensure clockwise winding
            std.mem.swap(Vertex, &v1, &v2);
            abc = -abc;
        }

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

        const bias0: i32 = if (isTopLeft(v0, v1)) 0 else -1;
        const bias1: i32 = if (isTopLeft(v1, v2)) 0 else -1;
        const bias2: i32 = if (isTopLeft(v2, v0)) 0 else -1;

        const p = Vertex{ .x = x_min, .y = y_min };
        const abp_row_start = edgeFunc(v0, v1, p) + bias0;
        const bcp_row_start = edgeFunc(v1, v2, p) + bias1;
        const cap_row_start = edgeFunc(v2, v0, p) + bias2;

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

        var u_row = tex_u0 * fp_one + (x_min - v0.x) * u_dx + (y_min - v0.y) * u_dy;
        var v_row = tex_v0 * fp_one + (x_min - v0.x) * v_dx + (y_min - v0.y) * v_dy;
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
            var u = u_row;
            var v = v_row;
            var r = r_row;
            var g = g_row;
            var b = b_row;

            var x = x_min;
            while (x <= x_max) : (x += 1) {
                const inside = (abp >= 0 and bcp >= 0 and cap >= 0);

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

                    if (!texel.isZero()) {
                        const blend_color: RGB8 = .{
                            .r = @truncate(@as(u32, @bitCast(r >> fp_bits))),
                            .g = @truncate(@as(u32, @bitCast(g >> fp_bits))),
                            .b = @truncate(@as(u32, @bitCast(b >> fp_bits))),
                        };
                        self.setPixelTextured(x, y, texel, blend_color, .{
                            .semi_trans = semi_trans,
                            .dither = true,
                            .blend = true,
                        });
                    }
                }

                abp += abp_dx;
                bcp += bcp_dx;
                cap += cap_dx;
                u += u_dx;
                v += v_dx;
                r += r_dx;
                g += g_dx;
                b += b_dx;
            }

            abp_row += abp_dy;
            bcp_row += bcp_dy;
            cap_row += cap_dy;
            u_row += u_dy;
            v_row += v_dy;
            r_row += r_dy;
            g_row += g_dy;
            b_row += b_dy;
        }
    }

    // =========================================================================
    // Rectangle Primitives
    // =========================================================================

    pub fn drawRectFlat(
        self: *@This(),
        x_orig: i32,
        y_orig: i32,
        w: i32,
        h: i32,
        c: RGB8,
        comptime semi_trans: bool,
    ) void {
        const x = x_orig + self.draw_offset[0];
        const y = y_orig + self.draw_offset[1];

        const x_min = @max(x, self.draw_area_start[0], 0);
        const y_min = @max(y, self.draw_area_start[1], 0);
        const x_max = @min(x + w - 1, self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(y + h - 1, self.draw_area_end[1], vram_res_y - 1);

        var yy = y_min;
        while (yy <= y_max) : (yy += 1) {
            var xx = x_min;
            while (xx <= x_max) : (xx += 1) {
                self.setPixelFlat(xx, yy, c, .{ .semi_trans = semi_trans });
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
        blend_color: RGB8,
        comptime semi_trans: bool,
        comptime tex_bleld: bool,
    ) void {
        const x = x_orig + self.draw_offset[0];
        const y = y_orig + self.draw_offset[1];

        const x_min = @max(x, self.draw_area_start[0], 0);
        const y_min = @max(y, self.draw_area_start[1], 0);
        const x_max = @min(x + w - 1, self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(y + h - 1, self.draw_area_end[1], vram_res_y - 1);

        const u_offset: u16 = @intCast(x_min - x);
        const v_offset: u16 = @intCast(y_min - y);

        const u_start = u +% u_offset;
        const v_start = v +% v_offset;

        var v_curr = v_start;
        var py = y_min;

        while (py <= y_max) : (py += 1) {
            var u_curr = u_start;
            var px = x_min;

            while (px <= x_max) : (px += 1) {
                const texel = self.sampleTexture(u_curr, v_curr, texp_x, texp_y, clut_x, clut_y, depth);

                if (!texel.isZero()) {
                    self.setPixelTextured(px, py, texel, blend_color, .{
                        .semi_trans = semi_trans,
                        .blend = tex_bleld,
                    });
                }

                u_curr +%= 1;
            }

            v_curr +%= 1;
        }
    }

    pub fn fillRectUnmasked(
        self: *@This(),
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        c: RGB8,
    ) void {
        const color: u16 = @bitCast(toRGB5(c, false));

        const x_min = @max(x, self.draw_area_start[0], 0);
        const y_min = @max(y, self.draw_area_start[1], 0);
        const x_max = @min(x + w - 1, self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(y + h - 1, self.draw_area_end[1], vram_res_y - 1);

        var yy = y_min;
        while (yy <= y_max) : (yy += 1) {
            var xx = x_min;
            while (xx <= x_max) : (xx += 1) {
                self.vram[toVramAddr(xx, yy)] = color;
            }
        }
    }

    pub fn copyRect(
        self: *@This(),
        src_x: i32,
        src_y: i32,
        dest_x: i32,
        dest_y: i32,
        w: i32,
        h: i32,
    ) void {
        var yy: i32 = 0;
        while (yy < h) : (yy += 1) {
            var xx: i32 = 0;
            while (xx < w) : (xx += 1) {
                const pixel = self.vram[toVramAddr(src_x + xx, src_y + yy)];
                self.setPixelRaw(dest_x + xx, dest_y + yy, pixel);
            }
        }
    }

    // =========================================================================
    // Line Primitives
    // =========================================================================

    pub fn drawLineFlat(
        self: *@This(),
        x0_orig: i32,
        y0_orig: i32,
        x1_orig: i32,
        y1_orig: i32,
        color: RGB8,
        comptime semi_trans: bool,
    ) void {
        const x0 = x0_orig + self.draw_offset[0];
        const y0 = y0_orig + self.draw_offset[1];
        const x1 = x1_orig + self.draw_offset[0];
        const y1 = y1_orig + self.draw_offset[1];

        if (@max(x0, x1) < self.draw_area_start[0] or @min(x0, x1) > self.draw_area_end[0]) return;
        if (@max(y0, y1) < self.draw_area_start[1] or @min(y0, y1) > self.draw_area_end[1]) return;

        const dx = x1 - x0;
        const dy = y1 - y0;

        const steps: i32 = @intCast(@max(@abs(dx), @abs(dy)));
        if (steps == 0) return;

        const x_dx = @divTrunc(dx * fp_one, steps);
        const y_dx = @divTrunc(dy * fp_one, steps);

        var x_fp = x0 * fp_one;
        var y_fp = y0 * fp_one;

        var i: i32 = 0;
        while (i <= steps) : (i += 1) {
            const x = x_fp >> fp_bits;
            const y = y_fp >> fp_bits;

            self.setPixelFlat(x, y, color, .{ .semi_trans = semi_trans, .dither = true });

            x_fp += x_dx;
            y_fp += y_dx;
        }
    }

    pub fn drawLineShaded(
        self: *@This(),
        x0_orig: i32,
        y0_orig: i32,
        c0_orig: RGB8,
        x1_orig: i32,
        y1_orig: i32,
        c1_orig: RGB8,
        comptime semi_trans: bool,
    ) void {
        const x0 = x0_orig + self.draw_offset[0];
        const y0 = y0_orig + self.draw_offset[1];
        const x1 = x1_orig + self.draw_offset[0];
        const y1 = y1_orig + self.draw_offset[1];

        if (@max(x0, x1) < self.draw_area_start[0] or @min(x0, x1) > self.draw_area_end[0]) return;
        if (@max(y0, y1) < self.draw_area_start[1] or @min(y0, y1) > self.draw_area_end[1]) return;

        const dx = x1 - x0;
        const dy = y1 - y0;

        const steps: i32 = @intCast(@max(@abs(dx), @abs(dy)));
        if (steps == 0) return;

        const x_dx = @divTrunc(dx * fp_one, steps);
        const y_dx = @divTrunc(dy * fp_one, steps);

        const r0: i32 = c0_orig.r;
        const g0: i32 = c0_orig.g;
        const b0: i32 = c0_orig.b;
        const r1: i32 = c1_orig.r;
        const g1: i32 = c1_orig.g;
        const b1: i32 = c1_orig.b;

        const r_dx = @divTrunc((r1 - r0) * fp_one, steps);
        const g_dx = @divTrunc((g1 - g0) * fp_one, steps);
        const b_dx = @divTrunc((b1 - b0) * fp_one, steps);

        var x_fp = x0 * fp_one;
        var y_fp = y0 * fp_one;
        var r_fp = r0 * fp_one;
        var g_fp = g0 * fp_one;
        var b_fp = b0 * fp_one;

        var i: i32 = 0;
        while (i <= steps) : (i += 1) {
            const x = x_fp >> fp_bits;
            const y = y_fp >> fp_bits;

            const color: RGB8 = .{
                .r = @truncate(@as(u32, @bitCast(r_fp >> fp_bits))),
                .g = @truncate(@as(u32, @bitCast(g_fp >> fp_bits))),
                .b = @truncate(@as(u32, @bitCast(b_fp >> fp_bits))),
            };

            self.setPixelFlat(x, y, color, .{
                .semi_trans = semi_trans,
                .dither = true,
            });

            x_fp += x_dx;
            y_fp += y_dx;
            r_fp += r_dx;
            g_fp += g_dx;
            b_fp += b_dx;
        }
    }
};
