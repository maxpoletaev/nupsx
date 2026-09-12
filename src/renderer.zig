const std = @import("std");

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

    pub inline fn isZero(self: RGB5) bool {
        // return @as(u16, @bitCast(self)) & 0x7fff == 0;
        return @as(u16, @bitCast(self)) == 0;
    }
};

pub fn toRGB5(c: RGB8, mask_bit: bool) RGB5 {
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

pub const Framebuffer = struct {
    pixels: []u16,
    width: i32,
    height: i32,
    upscale: i32,
};

pub const Renderer = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        start: *const fn (ptr: *anyopaque) void,
        deinit: *const fn (ptr: *anyopaque) void,
        execute: *const fn (ptr: *anyopaque, cmd: RasterCommand) void,
        setPixelRaw: *const fn (ptr: *anyopaque, x: i32, y: i32, color: u16) void,
        framebuffer: *const fn (ptr: *anyopaque) Framebuffer,
        flush: *const fn (ptr: *anyopaque) void,
    };

    pub inline fn start(self: Renderer) void {
        self.vtable.start(self.ptr);
    }

    pub inline fn deinit(self: Renderer) void {
        self.vtable.deinit(self.ptr);
    }

    pub inline fn execute(self: Renderer, cmd: RasterCommand) void {
        self.vtable.execute(self.ptr, cmd);
    }

    pub inline fn flush(self: Renderer) void {
        self.vtable.flush(self.ptr);
    }

    pub inline fn setPixelRaw(self: Renderer, x: i32, y: i32, color: u16) void {
        self.vtable.setPixelRaw(self.ptr, x, y, color);
    }

    pub inline fn framebuffer(self: Renderer) Framebuffer {
        return self.vtable.framebuffer(self.ptr);
    }

    pub fn from(comptime T: type, impl: *T) Renderer {
        const gen = struct {
            fn startImpl(ptr: *anyopaque) void {
                T.start(@ptrCast(@alignCast(ptr)));
            }
            fn deinitImpl(ptr: *anyopaque) void {
                T.deinit(@ptrCast(@alignCast(ptr)));
            }
            fn executeImpl(ptr: *anyopaque, cmd: RasterCommand) void {
                T.execute(@ptrCast(@alignCast(ptr)), cmd);
            }
            fn setPixelRawImpl(ptr: *anyopaque, x: i32, y: i32, color: u16) void {
                T.setPixelRaw(@ptrCast(@alignCast(ptr)), x, y, color);
            }
            fn framebufferImpl(ptr: *anyopaque) Framebuffer {
                return T.framebuffer(@ptrCast(@alignCast(ptr)));
            }
            fn flushImpl(ptr: *anyopaque) void {
                T.flush(@ptrCast(@alignCast(ptr)));
            }
            const vtable: VTable = .{
                .start = startImpl,
                .deinit = deinitImpl,
                .execute = executeImpl,
                .setPixelRaw = setPixelRawImpl,
                .framebuffer = framebufferImpl,
                .flush = flushImpl,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }
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
