const std = @import("std");
const zopengl = @import("zopengl");
const renderer = @import("renderer.zig");
const consts = @import("consts.zig");

const gl = zopengl.bindings;
const log = std.log.scoped(.renderer_gl);

const RasterCommand = renderer.RasterCommand;
const Framebuffer = renderer.Framebuffer;
const TransparencyMode = renderer.TransparencyMode;
const Renderer = renderer.Renderer;
const Vram = [consts.vram_res_x * consts.vram_res_y]u16;

pub const OpenGlRenderer = struct {
    allocator: std.mem.Allocator,
    vram: *align(16) Vram,
    upscale: i32,

    transparency_mode: TransparencyMode = .@"B+F",
    draw_area_start: [2]i32 = .{ 0, 0 },
    draw_area_end: [2]i32 = .{
        consts.vram_res_x - 1,
        consts.vram_res_y - 1,
    },
    draw_offset: [2]i32 = .{ 0, 0 },
    texwin_mask: [2]u16 = .{ 0, 0 },
    texwin_offset: [2]u16 = .{ 0, 0 },
    force_mask_bit: bool = false,
    check_mask_bit: bool = false,
    enable_dithering: bool = false,

    pub fn init(allocator: std.mem.Allocator, vram: *align(16) Vram, upscale: u32) *@This() {
        std.debug.assert(upscale >= 1);
        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .upscale = @intCast(upscale),
            .vram = vram,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn renderer(self: *@This()) Renderer {
        return .from(@This(), self);
    }

    pub fn framebuffer(self: *@This()) Framebuffer {
        return .{
            .pixels = self.vram,
            .width = consts.vram_res_x,
            .height = consts.vram_res_y,
            .upscale = 1,
        };
    }

    pub fn start(_: *@This()) void {}

    pub fn setPixelRaw(self: *@This(), x: i32, y: i32, color: u16) void {
        _ = self;
        _ = x;
        _ = y;
        _ = color;
    }

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
            // .fill_cmd => |c| self.fill(c),
            .set_transparency_mode => |mode| self.setTransparencyMode(mode),
            .set_draw_area_start => |args| self.setDrawAreaStart(args.x, args.y),
            .set_draw_area_end => |args| self.setDrawAreaEnd(args.x, args.y),
            .set_draw_offset => |args| self.setDrawOffset(args.x, args.y),
            .set_dithering => |enable| self.setDithering(enable),
            // .fill_rect_unmasked => |args| self.fillRectUnmasked(args.x, args.y, args.w, args.h, args.color),
            .set_mask_bit_setting => |args| self.setMaskBitSetting(args.force_mask_bit, args.check_mask_bit),
            .set_texture_window => |args| self.setTextureWindow(args.mask_x, args.mask_y, args.offset_x, args.offset_y),
            // .copy_rect => |args| self.copyRect(args.src_x, args.src_y, args.dest_x, args.dest_y, args.w, args.h),
            // .draw_line_flat => |args| self.execDrawLineFlat(args),
            // .draw_line_shaded => |args| self.execDrawLineShaded(args),
            // .draw_rect_flat => |args| self.execDrawRectFlat(args),
            // .draw_rect_textured => |args| self.execDrawRectTextured(args),
            // .draw_triangle_flat => |args| self.execDrawTriangleFlat(args),
            // .draw_triangle_shaded => |args| self.execDrawTriangleShaded(args),
            // .draw_triangle_textured => |args| self.execDrawTriangleTextured(args),
            // .draw_triangle_shaded_textured => |args| self.execDrawTriangleShadedTextured(args),
            else => log.warn("unhandled rendering command: {s}", .{@tagName(cmd)}),
        }
    }
};
