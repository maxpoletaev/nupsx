const std = @import("std");
const fifo = @import("fifo.zig");

const clamp = std.math.clamp;
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
        // return @as(u16, @bitCast(self)) & 0x7fff == 0;
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

pub const RasterCommand = union(enum) {
    fill_cmd: RGB8,
    set_dithering: bool,
    set_transparency_mode: TransparencyMode,
    set_draw_offset: struct { x: i32, y: i32 },
    set_draw_area_end: struct { x: i32, y: i32 },
    set_draw_area_start: struct { x: i32, y: i32 },
    set_mask_bit_setting: struct { force_mask_bit: bool, check_mask_bit: bool },
    set_texture_window: struct { mask_x: u16, mask_y: u16, offset_x: u16, offset_y: u16 },
    fill_rect_unmasked: struct { x: i32, y: i32, w: i32, h: i32, color: RGB8 },
    copy_rect: struct { src_x: i32, src_y: i32, dest_x: i32, dest_y: i32, w: i32, h: i32 },
    draw_line_flat: struct { x0: i32, y0: i32, x1: i32, y1: i32, color: RGB8, semi_trans: bool },
    draw_line_shaded: struct { x0: i32, y0: i32, c0: RGB8, x1: i32, y1: i32, c1: RGB8, semi_trans: bool },
    draw_rect_flat: struct { x: i32, y: i32, w: i32, h: i32, color: RGB8, semi_trans: bool },
    draw_rect_textured: struct { x: i32, y: i32, w: i32, h: i32, u: u16, v: u16, clut_x: u16, clut_y: u16, texp_x: u16, texp_y: u16, depth: ColorDepth, blend_color: RGB8, semi_trans: bool, tex_blend: bool },
    draw_triangle_flat: struct { v0: Vertex, v1: Vertex, v2: Vertex, color: RGB8, semi_trans: bool },
    draw_triangle_shaded: struct { v0: Vertex, v1: Vertex, v2: Vertex, semi_trans: bool },
    draw_triangle_textured: struct { v0: Vertex, v1: Vertex, v2: Vertex, clut_x: u16, clut_y: u16, texp_x: u16, texp_y: u16, depth: ColorDepth, blend_color: RGB8, semi_trans: bool, tex_blend: bool },
    draw_triangle_shaded_textured: struct { v0: Vertex, v1: Vertex, v2: Vertex, clut_x: u16, clut_y: u16, texp_x: u16, texp_y: u16, depth: ColorDepth, semi_trans: bool },

    pub fn fill(c: RGB8) RasterCommand {
        return .{ .fill_cmd = c };
    }

    pub fn setTransparencyMode(mode: TransparencyMode) RasterCommand {
        return .{ .set_transparency_mode = mode };
    }

    pub fn setTextureWindow(mask_x: u16, mask_y: u16, offset_x: u16, offset_y: u16) RasterCommand {
        return .{ .set_texture_window = .{ .mask_x = mask_x, .mask_y = mask_y, .offset_x = offset_x, .offset_y = offset_y } };
    }

    pub fn setDrawAreaStart(x: i32, y: i32) RasterCommand {
        return .{ .set_draw_area_start = .{ .x = x, .y = y } };
    }

    pub fn setDrawAreaEnd(x: i32, y: i32) RasterCommand {
        return .{ .set_draw_area_end = .{ .x = x, .y = y } };
    }

    pub fn setDrawOffset(x: i32, y: i32) RasterCommand {
        return .{ .set_draw_offset = .{ .x = x, .y = y } };
    }

    pub fn setMaskBitSetting(force_mask_bit: bool, check_mask_bit: bool) RasterCommand {
        return .{ .set_mask_bit_setting = .{ .force_mask_bit = force_mask_bit, .check_mask_bit = check_mask_bit } };
    }

    pub fn setDithering(enable: bool) RasterCommand {
        return .{ .set_dithering = enable };
    }

    pub fn fillRectUnmasked(x: i32, y: i32, w: i32, h: i32, color: RGB8) RasterCommand {
        return .{ .fill_rect_unmasked = .{ .x = x, .y = y, .w = w, .h = h, .color = color } };
    }

    pub fn copyRect(src_x: i32, src_y: i32, dest_x: i32, dest_y: i32, w: i32, h: i32) RasterCommand {
        return .{ .copy_rect = .{ .src_x = src_x, .src_y = src_y, .dest_x = dest_x, .dest_y = dest_y, .w = w, .h = h } };
    }

    pub fn drawLineFlat(x0: i32, y0: i32, x1: i32, y1: i32, color: RGB8, semi_trans: bool) RasterCommand {
        return .{ .draw_line_flat = .{ .x0 = x0, .y0 = y0, .x1 = x1, .y1 = y1, .color = color, .semi_trans = semi_trans } };
    }

    pub fn drawLineShaded(x0: i32, y0: i32, c0: RGB8, x1: i32, y1: i32, c1: RGB8, semi_trans: bool) RasterCommand {
        return .{ .draw_line_shaded = .{ .x0 = x0, .y0 = y0, .c0 = c0, .x1 = x1, .y1 = y1, .c1 = c1, .semi_trans = semi_trans } };
    }

    pub fn drawRectFlat(x: i32, y: i32, w: i32, h: i32, color: RGB8, semi_trans: bool) RasterCommand {
        return .{ .draw_rect_flat = .{ .x = x, .y = y, .w = w, .h = h, .color = color, .semi_trans = semi_trans } };
    }

    pub fn drawRectTextured(
        x: i32,
        y: i32,
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
        semi_trans: bool,
        tex_blend: bool,
    ) RasterCommand {
        return .{ .draw_rect_textured = .{
            .x = x,
            .y = y,
            .w = w,
            .h = h,
            .u = u,
            .v = v,
            .clut_x = clut_x,
            .clut_y = clut_y,
            .texp_x = texp_x,
            .texp_y = texp_y,
            .depth = depth,
            .blend_color = blend_color,
            .semi_trans = semi_trans,
            .tex_blend = tex_blend,
        } };
    }

    pub fn drawTriangleFlat(v0: Vertex, v1: Vertex, v2: Vertex, color: RGB8, semi_trans: bool) RasterCommand {
        return .{ .draw_triangle_flat = .{ .v0 = v0, .v1 = v1, .v2 = v2, .color = color, .semi_trans = semi_trans } };
    }

    pub fn drawTriangleShaded(v0: Vertex, v1: Vertex, v2: Vertex, semi_trans: bool) RasterCommand {
        return .{ .draw_triangle_shaded = .{ .v0 = v0, .v1 = v1, .v2 = v2, .semi_trans = semi_trans } };
    }

    pub fn drawTriangleTextured(
        v0: Vertex,
        v1: Vertex,
        v2: Vertex,
        clut_x: u16,
        clut_y: u16,
        texp_x: u16,
        texp_y: u16,
        depth: ColorDepth,
        blend_color: RGB8,
        semi_trans: bool,
        tex_blend: bool,
    ) RasterCommand {
        return .{ .draw_triangle_textured = .{
            .v0 = v0,
            .v1 = v1,
            .v2 = v2,
            .clut_x = clut_x,
            .clut_y = clut_y,
            .texp_x = texp_x,
            .texp_y = texp_y,
            .depth = depth,
            .blend_color = blend_color,
            .semi_trans = semi_trans,
            .tex_blend = tex_blend,
        } };
    }

    pub fn drawTriangleShadedTextured(
        v0: Vertex,
        v1: Vertex,
        v2: Vertex,
        clut_x: u16,
        clut_y: u16,
        texp_x: u16,
        texp_y: u16,
        depth: ColorDepth,
        semi_trans: bool,
    ) RasterCommand {
        return .{ .draw_triangle_shaded_textured = .{
            .v0 = v0,
            .v1 = v1,
            .v2 = v2,
            .clut_x = clut_x,
            .clut_y = clut_y,
            .texp_x = texp_x,
            .texp_y = texp_y,
            .depth = depth,
            .semi_trans = semi_trans,
        } };
    }
};

const fp_bits = 12;
const fp_one: i32 = 1 << fp_bits;
const Vec4i = @Vector(4, i32);
const vec4_offsets = Vec4i{ 0, 1, 2, 3 };

inline fn vec4Init(base: i32, dx: i32) Vec4i {
    return @as(Vec4i, @splat(base)) +% vec4_offsets *% @as(Vec4i, @splat(dx));
}

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

    pub fn deinit(_: *@This()) void {}
    pub fn start(_: *@This()) void {}
    pub fn flush(_: *@This()) void {}

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
    // Command dispatching
    // =========================================================================

    pub fn execute(self: *@This(), cmd: RasterCommand) void {
        switch (cmd) {
            .fill_cmd => |c| self.fill(c),
            .set_transparency_mode => |mode| self.setTransparencyMode(mode),
            .set_draw_area_start => |args| self.setDrawAreaStart(args.x, args.y),
            .set_draw_area_end => |args| self.setDrawAreaEnd(args.x, args.y),
            .set_draw_offset => |args| self.setDrawOffset(args.x, args.y),
            .set_dithering => |enable| self.setDithering(enable),
            .fill_rect_unmasked => |args| self.fillRectUnmasked(args.x, args.y, args.w, args.h, args.color),
            .set_mask_bit_setting => |args| self.setMaskBitSetting(args.force_mask_bit, args.check_mask_bit),
            .set_texture_window => |args| self.setTextureWindow(args.mask_x, args.mask_y, args.offset_x, args.offset_y),
            .copy_rect => |args| self.copyRect(args.src_x, args.src_y, args.dest_x, args.dest_y, args.w, args.h),
            .draw_line_flat => |args| self.execDrawLineFlat(args),
            .draw_line_shaded => |args| self.execDrawLineShaded(args),
            .draw_rect_flat => |args| self.execDrawRectFlat(args),
            .draw_rect_textured => |args| self.execDrawRectTextured(args),
            .draw_triangle_flat => |args| self.execDrawTriangleFlat(args),
            .draw_triangle_shaded => |args| self.execDrawTriangleShaded(args),
            .draw_triangle_textured => |args| self.execDrawTriangleTextured(args),
            .draw_triangle_shaded_textured => |args| self.execDrawTriangleShadedTextured(args),
        }
    }

    inline fn execDrawLineFlat(self: *@This(), args: @FieldType(RasterCommand, "draw_line_flat")) void {
        if (args.semi_trans) {
            self.drawLineFlat(args.x0, args.y0, args.x1, args.y1, args.color, true);
        } else {
            self.drawLineFlat(args.x0, args.y0, args.x1, args.y1, args.color, false);
        }
    }

    inline fn execDrawLineShaded(self: *@This(), args: @FieldType(RasterCommand, "draw_line_shaded")) void {
        if (args.semi_trans) {
            self.drawLineShaded(args.x0, args.y0, args.c0, args.x1, args.y1, args.c1, true);
        } else {
            self.drawLineShaded(args.x0, args.y0, args.c0, args.x1, args.y1, args.c1, false);
        }
    }

    inline fn execDrawRectFlat(self: *@This(), args: @FieldType(RasterCommand, "draw_rect_flat")) void {
        if (args.semi_trans) {
            self.drawRectFlat(args.x, args.y, args.w, args.h, args.color, true);
        } else {
            self.drawRectFlat(args.x, args.y, args.w, args.h, args.color, false);
        }
    }

    inline fn execDrawRectTextured(self: *@This(), args: @FieldType(RasterCommand, "draw_rect_textured")) void {
        switch (@as(u2, @intFromBool(args.semi_trans)) << 1 | @intFromBool(args.tex_blend)) {
            0 => self.drawRectTextured(args.x, args.y, args.w, args.h, args.u, args.v, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, args.blend_color, false, false),
            1 => self.drawRectTextured(args.x, args.y, args.w, args.h, args.u, args.v, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, args.blend_color, false, true),
            2 => self.drawRectTextured(args.x, args.y, args.w, args.h, args.u, args.v, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, args.blend_color, true, false),
            3 => self.drawRectTextured(args.x, args.y, args.w, args.h, args.u, args.v, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, args.blend_color, true, true),
        }
    }

    inline fn execDrawTriangleFlat(self: *@This(), args: @FieldType(RasterCommand, "draw_triangle_flat")) void {
        if (args.semi_trans) {
            self.drawTriangleFlat(args.v0, args.v1, args.v2, args.color, true);
        } else {
            self.drawTriangleFlat(args.v0, args.v1, args.v2, args.color, false);
        }
    }

    inline fn execDrawTriangleShaded(self: *@This(), args: @FieldType(RasterCommand, "draw_triangle_shaded")) void {
        if (args.semi_trans) {
            self.drawTriangleShaded(args.v0, args.v1, args.v2, true);
        } else {
            self.drawTriangleShaded(args.v0, args.v1, args.v2, false);
        }
    }

    inline fn execDrawTriangleTextured(self: *@This(), args: @FieldType(RasterCommand, "draw_triangle_textured")) void {
        switch (@as(u2, @intFromBool(args.semi_trans)) << 1 | @intFromBool(args.tex_blend)) {
            0 => self.drawTriangleTextured(args.v0, args.v1, args.v2, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, args.blend_color, false, false),
            1 => self.drawTriangleTextured(args.v0, args.v1, args.v2, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, args.blend_color, false, true),
            2 => self.drawTriangleTextured(args.v0, args.v1, args.v2, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, args.blend_color, true, false),
            3 => self.drawTriangleTextured(args.v0, args.v1, args.v2, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, args.blend_color, true, true),
        }
    }

    inline fn execDrawTriangleShadedTextured(self: *@This(), args: @FieldType(RasterCommand, "draw_triangle_shaded_textured")) void {
        if (args.semi_trans) {
            self.drawTriangleShadedTextured(args.v0, args.v1, args.v2, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, true);
        } else {
            self.drawTriangleShadedTextured(args.v0, args.v1, args.v2, args.clut_x, args.clut_y, args.texp_x, args.texp_y, args.depth, false);
        }
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    inline fn applyOffset(self: *@This(), v: Vertex) Vertex {
        return .{
            .x = v.x +% self.draw_offset[0],
            .y = v.y +% self.draw_offset[1],
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
            .r = @intCast(clamp(r, 0, 255)),
            .g = @intCast(clamp(g, 0, 255)),
            .b = @intCast(clamp(b, 0, 255)),
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
        const u: u8 = @truncate((u_orig & ~self.texwin_mask[0]) | (self.texwin_offset[0] & self.texwin_mask[0]));
        const v: u8 = @truncate((v_orig & ~self.texwin_mask[1]) | (self.texwin_offset[1] & self.texwin_mask[1]));

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

    fn setPixelFlat(
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
            if (comptime mode.dither) {
                if (self.enable_dithering) dithered = applyDithering(dithered, x, y);
            }

            var out = toRGB5(dithered, mode.semi_trans);
            if (comptime mode.semi_trans) out = applyTransparency(out, back, self.transparency_mode);
            if (self.force_mask_bit) out.mask_bit = true;

            self.vram[addr] = @bitCast(out);
        }
    }

    fn setPixelTextured(
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
            if (comptime mode.dither) {
                if (self.enable_dithering) dithered = applyDithering(dithered, x, y);
            }

            var front = texel;
            if (comptime mode.blend) front = applyBlending(front, dithered);
            if (comptime mode.semi_trans) front = applyTransparency(front, back, self.transparency_mode);
            if (self.force_mask_bit) front.mask_bit = true;

            self.vram[addr] = @bitCast(front);
        }
    }

    // =========================================================================
    // Triangle Primitives
    // =========================================================================

    inline fn edgeFunc(a: Vertex, b: Vertex, c: Vertex) i32 {
        return (b.x -% a.x) *% (c.y -% a.y) -% (b.y -% a.y) *% (c.x -% a.x);
    }

    inline fn isTopLeft(v0: Vertex, v1: Vertex) bool {
        const dx = v1.x - v0.x;
        const dy = v1.y - v0.y;
        return (dy < 0) or (dy == 0 and dx > 0);
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

        const zero_v: Vec4i = @splat(0);
        const abp_step4: Vec4i = @splat(abp_dx *% 4);
        const bcp_step4: Vec4i = @splat(bcp_dx *% 4);
        const cap_step4: Vec4i = @splat(cap_dx *% 4);

        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var abp_v = vec4Init(abp_row, abp_dx);
            var bcp_v = vec4Init(bcp_row, bcp_dx);
            var cap_v = vec4Init(cap_row, cap_dx);

            var x = x_min;
            while (x + 3 <= x_max) : (x += 4) {
                const inside = (abp_v >= zero_v) & (bcp_v >= zero_v) & (cap_v >= zero_v);
                if (@reduce(.Or, inside)) {
                    inline for (0..4) |lane| {
                        if (inside[lane]) {
                            self.setPixelFlat(x + @as(i32, @intCast(lane)), y, color, .{
                                .semi_trans = semi_trans,
                            });
                        }
                    }
                }
                abp_v +%= abp_step4;
                bcp_v +%= bcp_step4;
                cap_v +%= cap_step4;
            }

            // Scalar remainder
            var abp = abp_v[0];
            var bcp = bcp_v[0];
            var cap = cap_v[0];

            while (x <= x_max) : (x += 1) {
                if (abp >= 0 and bcp >= 0 and cap >= 0) {
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

        const r_dx = @divTrunc(((r1 -% r0) *% (v2.y -% v0.y) -% (r2 -% r0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const r_dy = @divTrunc(((r2 -% r0) *% (v1.x -% v0.x) -% (r1 -% r0) *% (v2.x -% v0.x)) *% fp_one, abc);
        const g_dx = @divTrunc(((g1 -% g0) *% (v2.y -% v0.y) -% (g2 -% g0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const g_dy = @divTrunc(((g2 -% g0) *% (v1.x -% v0.x) -% (g1 -% g0) *% (v2.x -% v0.x)) *% fp_one, abc);
        const b_dx = @divTrunc(((b1 -% b0) *% (v2.y -% v0.y) -% (b2 -% b0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const b_dy = @divTrunc(((b2 -% b0) *% (v1.x -% v0.x) -% (b1 -% b0) *% (v2.x -% v0.x)) *% fp_one, abc);

        var r_row = r0 *% fp_one +% (x_min -% v0.x) *% r_dx +% (y_min -% v0.y) *% r_dy;
        var g_row = g0 *% fp_one +% (x_min -% v0.x) *% g_dx +% (y_min -% v0.y) *% g_dy;
        var b_row = b0 *% fp_one +% (x_min -% v0.x) *% b_dx +% (y_min -% v0.y) *% b_dy;

        const zero_v: Vec4i = @splat(0);
        const abp_step4: Vec4i = @splat(abp_dx *% 4);
        const bcp_step4: Vec4i = @splat(bcp_dx *% 4);
        const cap_step4: Vec4i = @splat(cap_dx *% 4);
        const r_step4: Vec4i = @splat(r_dx *% 4);
        const g_step4: Vec4i = @splat(g_dx *% 4);
        const b_step4: Vec4i = @splat(b_dx *% 4);

        var abp_row = abp_row_start;
        var bcp_row = bcp_row_start;
        var cap_row = cap_row_start;

        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var abp_v = vec4Init(abp_row, abp_dx);
            var bcp_v = vec4Init(bcp_row, bcp_dx);
            var cap_v = vec4Init(cap_row, cap_dx);
            var r_v = vec4Init(r_row, r_dx);
            var g_v = vec4Init(g_row, g_dx);
            var b_v = vec4Init(b_row, b_dx);

            var x = x_min;
            while (x + 3 <= x_max) : (x += 4) {
                const inside = (abp_v >= zero_v) & (bcp_v >= zero_v) & (cap_v >= zero_v);
                if (@reduce(.Or, inside)) {
                    inline for (0..4) |lane| {
                        if (inside[lane]) {
                            const color: RGB8 = .{
                                .r = @truncate(@as(u32, @bitCast(r_v[lane] >> fp_bits))),
                                .g = @truncate(@as(u32, @bitCast(g_v[lane] >> fp_bits))),
                                .b = @truncate(@as(u32, @bitCast(b_v[lane] >> fp_bits))),
                            };
                            self.setPixelFlat(x + @as(i32, @intCast(lane)), y, color, .{
                                .semi_trans = semi_trans,
                                .dither = true,
                            });
                        }
                    }
                }
                abp_v +%= abp_step4;
                bcp_v +%= bcp_step4;
                cap_v +%= cap_step4;
                r_v +%= r_step4;
                g_v +%= g_step4;
                b_v +%= b_step4;
            }

            // Scalar remainder
            var abp = abp_v[0];
            var bcp = bcp_v[0];
            var cap = cap_v[0];
            var r = r_v[0];
            var g = g_v[0];
            var b = b_v[0];

            while (x <= x_max) : (x += 1) {
                if (abp >= 0 and bcp >= 0 and cap >= 0) {
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
                r +%= r_dx;
                g +%= g_dx;
                b +%= b_dx;
            }

            abp_row += abp_dy;
            bcp_row += bcp_dy;
            cap_row += cap_dy;
            r_row +%= r_dy;
            g_row +%= g_dy;
            b_row +%= b_dy;
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

        const u_dx = @divTrunc(((tex_u1 -% tex_u0) *% (v2.y -% v0.y) -% (tex_u2 -% tex_u0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const u_dy = @divTrunc(((tex_u2 -% tex_u0) *% (v1.x -% v0.x) -% (tex_u1 -% tex_u0) *% (v2.x -% v0.x)) *% fp_one, abc);
        const v_dx = @divTrunc(((tex_v1 -% tex_v0) *% (v2.y -% v0.y) -% (tex_v2 -% tex_v0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const v_dy = @divTrunc(((tex_v2 -% tex_v0) *% (v1.x -% v0.x) -% (tex_v1 -% tex_v0) *% (v2.x -% v0.x)) *% fp_one, abc);

        var u_row = tex_u0 *% fp_one +% (x_min -% v0.x) *% u_dx +% (y_min -% v0.y) *% u_dy;
        var v_row = tex_v0 *% fp_one +% (x_min -% v0.x) *% v_dx +% (y_min -% v0.y) *% v_dy;

        const zero_v: Vec4i = @splat(0);
        const abp_step4: Vec4i = @splat(abp_dx *% 4);
        const bcp_step4: Vec4i = @splat(bcp_dx *% 4);
        const cap_step4: Vec4i = @splat(cap_dx *% 4);
        const u_step4: Vec4i = @splat(u_dx *% 4);
        const v_step4: Vec4i = @splat(v_dx *% 4);

        var abp_row = abp_row_start;
        var bcp_row = bcp_row_start;
        var cap_row = cap_row_start;

        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var abp_v = vec4Init(abp_row, abp_dx);
            var bcp_v = vec4Init(bcp_row, bcp_dx);
            var cap_v = vec4Init(cap_row, cap_dx);
            var u_v = vec4Init(u_row, u_dx);
            var v_v = vec4Init(v_row, v_dx);

            var x = x_min;
            while (x + 3 <= x_max) : (x += 4) {
                const inside = (abp_v >= zero_v) & (bcp_v >= zero_v) & (cap_v >= zero_v);
                if (@reduce(.Or, inside)) {
                    inline for (0..4) |lane| {
                        if (inside[lane]) {
                            const texel = self.sampleTexture(
                                @truncate(@as(u32, @bitCast(u_v[lane] >> fp_bits))),
                                @truncate(@as(u32, @bitCast(v_v[lane] >> fp_bits))),
                                texp_x,
                                texp_y,
                                clut_x,
                                clut_y,
                                depth,
                            );
                            if (!texel.isZero()) {
                                self.setPixelTextured(x + @as(i32, @intCast(lane)), y, texel, blend_color, .{
                                    .semi_trans = semi_trans,
                                    .blend = tex_blend,
                                });
                            }
                        }
                    }
                }
                abp_v +%= abp_step4;
                bcp_v +%= bcp_step4;
                cap_v +%= cap_step4;
                u_v +%= u_step4;
                v_v +%= v_step4;
            }

            // Scalar remainder
            var abp = abp_v[0];
            var bcp = bcp_v[0];
            var cap = cap_v[0];
            var u = u_v[0];
            var v = v_v[0];

            while (x <= x_max) : (x += 1) {
                if (abp >= 0 and bcp >= 0 and cap >= 0) {
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
                u +%= u_dx;
                v +%= v_dx;
            }

            abp_row += abp_dy;
            bcp_row += bcp_dy;
            cap_row += cap_dy;
            u_row +%= u_dy;
            v_row +%= v_dy;
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

        const u_dx = @divTrunc(((tex_u1 -% tex_u0) *% (v2.y -% v0.y) -% (tex_u2 -% tex_u0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const u_dy = @divTrunc(((tex_u2 -% tex_u0) *% (v1.x -% v0.x) -% (tex_u1 -% tex_u0) *% (v2.x -% v0.x)) *% fp_one, abc);
        const v_dx = @divTrunc(((tex_v1 -% tex_v0) *% (v2.y -% v0.y) -% (tex_v2 -% tex_v0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const v_dy = @divTrunc(((tex_v2 -% tex_v0) *% (v1.x -% v0.x) -% (tex_v1 -% tex_v0) *% (v2.x -% v0.x)) *% fp_one, abc);

        const r0: i32 = v0.color.r;
        const r1: i32 = v1.color.r;
        const r2: i32 = v2.color.r;
        const g0: i32 = v0.color.g;
        const g1: i32 = v1.color.g;
        const g2: i32 = v2.color.g;
        const b0: i32 = v0.color.b;
        const b1: i32 = v1.color.b;
        const b2: i32 = v2.color.b;

        const r_dx = @divTrunc(((r1 -% r0) *% (v2.y -% v0.y) -% (r2 -% r0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const r_dy = @divTrunc(((r2 -% r0) *% (v1.x -% v0.x) -% (r1 -% r0) *% (v2.x -% v0.x)) *% fp_one, abc);
        const g_dx = @divTrunc(((g1 -% g0) *% (v2.y -% v0.y) -% (g2 -% g0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const g_dy = @divTrunc(((g2 -% g0) *% (v1.x -% v0.x) -% (g1 -% g0) *% (v2.x -% v0.x)) *% fp_one, abc);
        const b_dx = @divTrunc(((b1 -% b0) *% (v2.y -% v0.y) -% (b2 -% b0) *% (v1.y -% v0.y)) *% fp_one, abc);
        const b_dy = @divTrunc(((b2 -% b0) *% (v1.x -% v0.x) -% (b1 -% b0) *% (v2.x -% v0.x)) *% fp_one, abc);

        var u_row = tex_u0 *% fp_one +% (x_min -% v0.x) *% u_dx +% (y_min -% v0.y) *% u_dy;
        var v_row = tex_v0 *% fp_one +% (x_min -% v0.x) *% v_dx +% (y_min -% v0.y) *% v_dy;
        var r_row = r0 *% fp_one +% (x_min -% v0.x) *% r_dx +% (y_min -% v0.y) *% r_dy;
        var g_row = g0 *% fp_one +% (x_min -% v0.x) *% g_dx +% (y_min -% v0.y) *% g_dy;
        var b_row = b0 *% fp_one +% (x_min -% v0.x) *% b_dx +% (y_min -% v0.y) *% b_dy;

        const zero_v: Vec4i = @splat(0);
        const abp_step4: Vec4i = @splat(abp_dx *% 4);
        const bcp_step4: Vec4i = @splat(bcp_dx *% 4);
        const cap_step4: Vec4i = @splat(cap_dx *% 4);
        const u_step4: Vec4i = @splat(u_dx *% 4);
        const v_step4: Vec4i = @splat(v_dx *% 4);
        const r_step4: Vec4i = @splat(r_dx *% 4);
        const g_step4: Vec4i = @splat(g_dx *% 4);
        const b_step4: Vec4i = @splat(b_dx *% 4);

        var abp_row = abp_row_start;
        var bcp_row = bcp_row_start;
        var cap_row = cap_row_start;

        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var abp_v = vec4Init(abp_row, abp_dx);
            var bcp_v = vec4Init(bcp_row, bcp_dx);
            var cap_v = vec4Init(cap_row, cap_dx);
            var u_v = vec4Init(u_row, u_dx);
            var v_v = vec4Init(v_row, v_dx);
            var r_v = vec4Init(r_row, r_dx);
            var g_v = vec4Init(g_row, g_dx);
            var b_v = vec4Init(b_row, b_dx);

            var x = x_min;
            while (x + 3 <= x_max) : (x += 4) {
                const inside = (abp_v >= zero_v) & (bcp_v >= zero_v) & (cap_v >= zero_v);
                if (@reduce(.Or, inside)) {
                    inline for (0..4) |lane| {
                        if (inside[lane]) {
                            const texel = self.sampleTexture(
                                @truncate(@as(u32, @bitCast(u_v[lane] >> fp_bits))),
                                @truncate(@as(u32, @bitCast(v_v[lane] >> fp_bits))),
                                texp_x,
                                texp_y,
                                clut_x,
                                clut_y,
                                depth,
                            );
                            if (!texel.isZero()) {
                                const blend_color: RGB8 = .{
                                    .r = @truncate(@as(u32, @bitCast(r_v[lane] >> fp_bits))),
                                    .g = @truncate(@as(u32, @bitCast(g_v[lane] >> fp_bits))),
                                    .b = @truncate(@as(u32, @bitCast(b_v[lane] >> fp_bits))),
                                };
                                self.setPixelTextured(x + @as(i32, @intCast(lane)), y, texel, blend_color, .{
                                    .semi_trans = semi_trans,
                                    .dither = true,
                                    .blend = true,
                                });
                            }
                        }
                    }
                }
                abp_v +%= abp_step4;
                bcp_v +%= bcp_step4;
                cap_v +%= cap_step4;
                u_v +%= u_step4;
                v_v +%= v_step4;
                r_v +%= r_step4;
                g_v +%= g_step4;
                b_v +%= b_step4;
            }

            // Scalar remainder
            var abp = abp_v[0];
            var bcp = bcp_v[0];
            var cap = cap_v[0];
            var u = u_v[0];
            var v = v_v[0];
            var r = r_v[0];
            var g = g_v[0];
            var b = b_v[0];

            while (x <= x_max) : (x += 1) {
                if (abp >= 0 and bcp >= 0 and cap >= 0) {
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
                u +%= u_dx;
                v +%= v_dx;
                r +%= r_dx;
                g +%= g_dx;
                b +%= b_dx;
            }

            abp_row += abp_dy;
            bcp_row += bcp_dy;
            cap_row += cap_dy;
            u_row +%= u_dy;
            v_row +%= v_dy;
            r_row +%= r_dy;
            g_row +%= g_dy;
            b_row +%= b_dy;
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

        const x_min = @max(self.draw_area_start[0], 0);
        const y_min = @max(self.draw_area_start[1], 0);
        const x_max = @min(self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(self.draw_area_end[1], vram_res_y - 1);

        if (x_min > x_max or y_min > y_max) return;

        if (@max(x0, x1) < x_min or @min(x0, x1) > x_max) return;
        if (@max(y0, y1) < y_min or @min(y0, y1) > y_max) return;

        const line_inside =
            x0 >= x_min and x0 <= x_max and
            y0 >= y_min and y0 <= y_max and
            x1 >= x_min and x1 <= x_max and
            y1 >= y_min and y1 <= y_max;

        const dx = x1 - x0;
        const dy = y1 - y0;

        const steps: i32 = @intCast(@max(@abs(dx), @abs(dy)));
        if (steps == 0) return;

        const x_dx = @divTrunc(dx * fp_one, steps);
        const y_dx = @divTrunc(dy * fp_one, steps);

        var x_fp = x0 * fp_one;
        var y_fp = y0 * fp_one;

        var i: i32 = 0;
        if (line_inside) { // fast path
            while (i <= steps) : (i += 1) {
                const x = x_fp >> fp_bits;
                const y = y_fp >> fp_bits;

                self.setPixelFlat(x, y, color, .{ .semi_trans = semi_trans, .dither = true });

                x_fp += x_dx;
                y_fp += y_dx;
            }
        } else {
            while (i <= steps) : (i += 1) {
                const x = x_fp >> fp_bits;
                const y = y_fp >> fp_bits;

                if (x >= x_min and x <= x_max and y >= y_min and y <= y_max) {
                    self.setPixelFlat(x, y, color, .{ .semi_trans = semi_trans, .dither = true });
                }

                x_fp += x_dx;
                y_fp += y_dx;
            }
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

        const x_min = @max(self.draw_area_start[0], 0);
        const y_min = @max(self.draw_area_start[1], 0);
        const x_max = @min(self.draw_area_end[0], vram_res_x - 1);
        const y_max = @min(self.draw_area_end[1], vram_res_y - 1);

        if (x_min > x_max or y_min > y_max) return;

        if (@max(x0, x1) < x_min or @min(x0, x1) > x_max) return;
        if (@max(y0, y1) < y_min or @min(y0, y1) > y_max) return;

        const line_inside =
            x0 >= x_min and x0 <= x_max and
            y0 >= y_min and y0 <= y_max and
            x1 >= x_min and x1 <= x_max and
            y1 >= y_min and y1 <= y_max;

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
        if (line_inside) {
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
        } else {
            while (i <= steps) : (i += 1) {
                const x = x_fp >> fp_bits;
                const y = y_fp >> fp_bits;

                if (x >= x_min and x <= x_max and y >= y_min and y <= y_max) {
                    const color: RGB8 = .{
                        .r = @truncate(@as(u32, @bitCast(r_fp >> fp_bits))),
                        .g = @truncate(@as(u32, @bitCast(g_fp >> fp_bits))),
                        .b = @truncate(@as(u32, @bitCast(b_fp >> fp_bits))),
                    };

                    self.setPixelFlat(x, y, color, .{
                        .semi_trans = semi_trans,
                        .dither = true,
                    });
                }

                x_fp += x_dx;
                y_fp += y_dx;
                r_fp += r_dx;
                g_fp += g_dx;
                b_fp += b_dx;
            }
        }
    }
};

pub const ThreadedRasterizer = struct {
    const Queue = std.ArrayListUnmanaged(RasterCommand);

    allocator: std.mem.Allocator,
    io: std.Io,

    rasterizer: Rasterizer,
    mutex: std.Io.Mutex = .init,
    cond: std.Io.Condition = .init,
    next_seq: u64 = 0,
    completed_seq: u64 = 0,
    stopping: bool = false,
    worker: ?std.Thread = null,
    pending: Queue = .empty,
    active: Queue = .empty,

    pub fn init(allocator: std.mem.Allocator, io: std.Io, vram: *align(16) [1024 * 512]u16) @This() {
        return .{
            .io = io,
            .allocator = allocator,
            .rasterizer = Rasterizer.init(vram),
        };
    }

    pub fn start(self: *@This()) void {
        self.worker = std.Thread.spawn(.{}, workerMain, .{self}) catch @panic("spawn rasterizer worker");
    }

    pub fn deinit(self: *@This()) void {
        if (self.worker) |worker| {
            self.mutex.lockUncancelable(self.io);
            self.stopping = true;
            self.cond.broadcast(self.io);
            self.mutex.unlock(self.io);
            worker.join();
            self.worker = null;
        }

        self.pending.deinit(self.allocator);
        self.active.deinit(self.allocator);
    }

    pub fn flush(self: *@This()) void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        const target_seq = self.next_seq;
        while (self.completed_seq < target_seq) {
            self.cond.waitUncancelable(self.io, &self.mutex);
        }
    }

    fn enqueue(self: *@This(), payload: RasterCommand) void {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);

        self.next_seq += 1;
        const was_empty = self.pending.items.len == 0;
        self.pending.append(self.allocator, payload) catch @panic("OOM");
        if (was_empty) self.cond.signal(self.io);
    }

    pub fn execute(self: *@This(), cmd: RasterCommand) void {
        @call(.always_inline, enqueue, .{ self, cmd });
    }

    pub inline fn setPixelRaw(self: *@This(), x: i32, y: i32, color: u16) void {
        self.rasterizer.setPixelRaw(x, y, color);
    }

    fn swapQueues(self: *@This(), batch_seq: u64) u64 {
        self.mutex.lockUncancelable(self.io);
        defer self.mutex.unlock(self.io);
        if (batch_seq != 0) {
            self.completed_seq = batch_seq;
            self.cond.broadcast(self.io);
        }
        while (self.pending.items.len == 0 and !self.stopping) {
            self.cond.waitUncancelable(self.io, &self.mutex);
        }
        if (self.pending.items.len == 0) return 0;
        std.mem.swap(Queue, &self.pending, &self.active);
        return self.next_seq;
    }

    fn workerMain(self: *@This()) void {
        var batch_seq: u64 = 0;
        while (true) {
            batch_seq = self.swapQueues(batch_seq);
            if (batch_seq == 0) return;
            for (self.active.items) |cmd| {
                self.rasterizer.execute(cmd);
            }
            self.active.clearRetainingCapacity();
        }
    }
};
