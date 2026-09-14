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
const Vertex = renderer.Vertex;
const ColorDepth = renderer.ColorDepth;
const RGB8 = renderer.RGB8;
const Vram = [consts.vram_size]u16;

const vertex_shader_source = @embedFile("shaders/raster_vertex.glsl");
const fragment_shader_source = @embedFile("shaders/raster_fragment.glsl");

const Rect = struct {
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,

    const empty: Rect = .{
        .x0 = std.math.maxInt(i32),
        .y0 = std.math.maxInt(i32),
        .x1 = std.math.minInt(i32),
        .y1 = std.math.minInt(i32),
    };

    inline fn init(x0: i32, y0: i32, x1: i32, y1: i32) Rect {
        return .{
            .x0 = x0,
            .y0 = y0,
            .x1 = x1,
            .y1 = y1,
        };
    }

    inline fn isEmpty(self: Rect) bool {
        return self.x1 < self.x0 or self.y1 < self.y0;
    }

    inline fn intersects(a: Rect, b: Rect) bool {
        return a.x0 <= b.x1 and b.x0 <= a.x1 and a.y0 <= b.y1 and b.y0 <= a.y1;
    }

    inline fn add(self: *Rect, x0: i32, y0: i32, x1: i32, y1: i32) void {
        self.x0 = @min(self.x0, x0);
        self.y0 = @min(self.y0, y0);
        self.x1 = @max(self.x1, x1);
        self.y1 = @max(self.y1, y1);
    }

    inline fn addRect(self: *Rect, r: Rect) void {
        self.add(r.x0, r.y0, r.x1, r.y1);
    }
};

const BatchState = struct {
    semi_trans: bool = false,
    textured: bool = false,
    depth: ColorDepth = .bit4,
};

const TexInfo = struct {
    texp: [2]u16,
    clut: [2]u16,
    depth: ColorDepth,
    tex_blend: bool,

    fn pageRect(self: TexInfo) Rect {
        const w: i32 = switch (self.depth) {
            .bit4 => 64,
            .bit8 => 128,
            .bit15 => 256,
        };
        return .{
            .x0 = self.texp[0],
            .y0 = self.texp[1],
            .x1 = self.texp[0] + w - 1,
            .y1 = self.texp[1] + 255,
        };
    }

    fn clutRect(self: TexInfo) Rect {
        const w: i32 = if (self.depth == .bit4) 16 else 256;
        return .{
            .x0 = self.clut[0],
            .y0 = self.clut[1],
            .x1 = self.clut[0] + w - 1,
            .y1 = self.clut[1],
        };
    }
};

const Uniforms = struct {
    vram: gl.Int,
    upscale: gl.Int,
    textured: gl.Int,
    depth: gl.Int,
    mask_select: gl.Int,
    texwin_mask: gl.Int,
    texwin_offset: gl.Int,
};

const GlVertex = extern struct {
    x: f32,
    y: f32,
    u: f32,
    v: f32,
    r: u8,
    g: u8,
    b: u8,
    a: u8 = 0,
    texp_x: u16 = 0,
    texp_y: u16 = 0,
    clut_x: u16 = 0,
    clut_y: u16 = 0,
    tex_blend: u8 = 0,
    dither: u8 = 0,
};

pub const GLRenderer = struct {
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

    queue: std.ArrayListUnmanaged(RasterCommand) = .empty,
    batch: std.ArrayListUnmanaged(GlVertex) = .empty,
    batch_state: BatchState = .{},
    batch_bounds: Rect = .empty,
    batch_sampled: Rect = .empty,
    vram_dirty: Rect = .empty,
    read_dirty: Rect = .empty,

    gl_ready: bool = false,
    program: gl.Uint = 0,
    u: Uniforms = undefined,
    read_tex: gl.Uint = 0,
    native_tex: gl.Uint = 0,
    native_fbo: gl.Uint = 0,
    vao: gl.Uint = 0,
    vbo: gl.Uint = 0,
    fbo: gl.Uint = 0,
    tex: gl.Uint = 0,

    pub fn init(allocator: std.mem.Allocator, vram: *align(16) Vram, upscale: u32) *@This() {
        std.debug.assert(upscale >= 1 and upscale <= consts.max_upscale);
        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .upscale = @intCast(upscale),
            .vram = vram,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        if (self.gl_ready) {
            gl.deleteProgram(self.program);
            gl.deleteVertexArrays(1, &self.vao);
            gl.deleteBuffers(1, &self.vbo);
            gl.deleteFramebuffers(1, &self.fbo);
            gl.deleteTextures(1, &self.tex);
            gl.deleteTextures(1, &self.read_tex);
            gl.deleteTextures(1, &self.native_tex);
            gl.deleteFramebuffers(1, &self.native_fbo);
        }
        self.queue.deinit(self.allocator);
        self.batch.deinit(self.allocator);
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
            .upscale = self.upscale,
        };
    }

    pub fn start(_: *@This()) void {}

    pub fn texture(self: *@This()) ?u32 {
        return self.tex;
    }

    pub fn initBackend(self: *@This()) void {
        std.debug.assert(!self.gl_ready);
        self.gl_ready = true;

        self.program = glCreateProgram(vertex_shader_source, fragment_shader_source);
        self.u = .{
            .vram = gl.getUniformLocation(self.program, "uVram"),
            .upscale = gl.getUniformLocation(self.program, "uUpscale"),
            .textured = gl.getUniformLocation(self.program, "uTextured"),
            .depth = gl.getUniformLocation(self.program, "uDepth"),
            .mask_select = gl.getUniformLocation(self.program, "uMaskSelect"),
            .texwin_mask = gl.getUniformLocation(self.program, "uTexWinMask"),
            .texwin_offset = gl.getUniformLocation(self.program, "uTexWinOffset"),
        };

        gl.disable(gl.DITHER);
        gl.genVertexArrays(1, &self.vao);
        gl.genBuffers(1, &self.vbo);
        gl.bindVertexArray(self.vao);
        gl.bindBuffer(gl.ARRAY_BUFFER, self.vbo);
        gl.vertexAttribPointer(0, 2, gl.FLOAT, gl.FALSE, @sizeOf(GlVertex), @ptrFromInt(@offsetOf(GlVertex, "x")));
        gl.enableVertexAttribArray(0);
        gl.vertexAttribPointer(1, 2, gl.FLOAT, gl.FALSE, @sizeOf(GlVertex), @ptrFromInt(@offsetOf(GlVertex, "u")));
        gl.enableVertexAttribArray(1);
        gl.vertexAttribPointer(2, 4, gl.UNSIGNED_BYTE, gl.TRUE, @sizeOf(GlVertex), @ptrFromInt(@offsetOf(GlVertex, "r")));
        gl.enableVertexAttribArray(2);
        gl.vertexAttribIPointer(3, 4, gl.UNSIGNED_SHORT, @sizeOf(GlVertex), @ptrFromInt(@offsetOf(GlVertex, "texp_x")));
        gl.enableVertexAttribArray(3);
        gl.vertexAttribIPointer(4, 2, gl.UNSIGNED_BYTE, @sizeOf(GlVertex), @ptrFromInt(@offsetOf(GlVertex, "tex_blend")));
        gl.enableVertexAttribArray(4);
        gl.bindVertexArray(0);

        self.tex = createVramTexture(self.upscale);
        self.read_tex = createVramTexture(self.upscale);
        self.native_tex = createVramTexture(1);
        self.fbo = createFramebuffer(self.tex);
        self.native_fbo = createFramebuffer(self.native_tex);
    }

    pub fn execute(self: *@This(), cmd: RasterCommand) void {
        self.queue.append(self.allocator, cmd) catch @panic("OOM");
    }

    pub fn setPixelRaw(self: *@This(), x: i32, y: i32, color: u16) void {
        const xx = x & (consts.vram_res_x - 1);
        const yy = y & (consts.vram_res_y - 1);
        self.vram[@intCast(yy * consts.vram_res_x + xx)] = color;
        self.vram_dirty.add(xx, yy, xx, yy);
    }

    pub fn flush(self: *@This()) void {
        std.debug.assert(self.gl_ready);
        self.uploadVram();

        gl.bindFramebuffer(gl.FRAMEBUFFER, self.fbo);
        gl.viewport(0, 0, consts.vram_res_x * self.upscale, consts.vram_res_y * self.upscale);
        gl.disable(gl.BLEND);

        for (self.queue.items) |cmd| {
            self.exec(cmd);
        }

        self.queue.clearRetainingCapacity();
        self.drawBatch();

        gl.disable(gl.SCISSOR_TEST);
        gl.disable(gl.BLEND);
        gl.bindFramebuffer(gl.FRAMEBUFFER, 0);
        gl.bindVertexArray(0);
    }

    pub fn downloadVram(self: *@This(), x: i32, y: i32, w: i32, h: i32) void {
        self.flush();

        var x0 = x & (consts.vram_res_x - 1);
        var y0 = y & (consts.vram_res_y - 1);
        var x1 = x0 + w - 1;
        if (x1 >= consts.vram_res_x) {
            x0 = 0;
            x1 = consts.vram_res_x - 1;
        }
        var y1 = y0 + h - 1;
        if (y1 >= consts.vram_res_y) {
            y0 = 0;
            y1 = consts.vram_res_y - 1;
        }

        if (self.upscale != 1) {
            const s = self.upscale;
            gl.disable(gl.SCISSOR_TEST);
            gl.bindFramebuffer(gl.READ_FRAMEBUFFER, self.fbo);
            gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, self.native_fbo);
            gl.blitFramebuffer(
                x0 * s,
                y0 * s,
                (x1 + 1) * s,
                (y1 + 1) * s,
                x0,
                y0,
                x1 + 1,
                y1 + 1,
                gl.COLOR_BUFFER_BIT,
                gl.NEAREST,
            );
        }

        gl.bindFramebuffer(gl.READ_FRAMEBUFFER, if (self.upscale != 1) self.native_fbo else self.fbo);
        gl.pixelStorei(gl.PACK_ROW_LENGTH, consts.vram_res_x);
        gl.readPixels(
            x0,
            y0,
            x1 - x0 + 1,
            y1 - y0 + 1,
            gl.RGBA,
            gl.UNSIGNED_SHORT_1_5_5_5_REV,
            self.vram[@intCast(y0 * consts.vram_res_x + x0)..].ptr,
        );
        gl.pixelStorei(gl.PACK_ROW_LENGTH, 0);
        gl.bindFramebuffer(gl.FRAMEBUFFER, 0);
    }

    fn uploadVram(self: *@This()) void {
        if (self.vram_dirty.isEmpty()) return;
        const d = self.vram_dirty;
        self.vram_dirty = .empty;
        self.read_dirty.addRect(d);
        gl.bindTexture(gl.TEXTURE_2D, if (self.upscale != 1) self.native_tex else self.tex);
        gl.pixelStorei(gl.UNPACK_ROW_LENGTH, consts.vram_res_x);
        gl.texSubImage2D(
            gl.TEXTURE_2D,
            0,
            d.x0,
            d.y0,
            d.x1 - d.x0 + 1,
            d.y1 - d.y0 + 1,
            gl.RGBA,
            gl.UNSIGNED_SHORT_1_5_5_5_REV,
            self.vram[@intCast(d.y0 * consts.vram_res_x + d.x0)..].ptr,
        );
        gl.pixelStorei(gl.UNPACK_ROW_LENGTH, 0);

        if (self.upscale != 1) {
            const s = self.upscale;
            gl.disable(gl.SCISSOR_TEST);
            gl.bindFramebuffer(gl.READ_FRAMEBUFFER, self.native_fbo);
            gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, self.fbo);
            gl.blitFramebuffer(
                d.x0,
                d.y0,
                d.x1 + 1,
                d.y1 + 1,
                d.x0 * s,
                d.y0 * s,
                (d.x1 + 1) * s,
                (d.y1 + 1) * s,
                gl.COLOR_BUFFER_BIT,
                gl.NEAREST,
            );
        }
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    pub fn setTransparencyMode(self: *@This(), mode: TransparencyMode) void {
        if (self.transparency_mode != mode) {
            if (self.batch_state.semi_trans) self.drawBatch();
            self.transparency_mode = mode;
        }
    }

    pub fn setTextureWindow(self: *@This(), mask_x: u16, mask_y: u16, offset_x: u16, offset_y: u16) void {
        const mask: [2]u16 = .{ mask_x, mask_y };
        const offset: [2]u16 = .{ offset_x, offset_y };

        if (!std.meta.eql(self.texwin_mask, mask) or
            !std.meta.eql(self.texwin_offset, offset))
        {
            if (self.batch_state.textured) self.drawBatch();
            self.texwin_mask = mask;
            self.texwin_offset = offset;
        }
    }

    pub fn setDrawAreaStart(self: *@This(), x: i32, y: i32) void {
        if (!std.meta.eql(self.draw_area_start, .{ x, y })) {
            self.drawBatch();
            self.draw_area_start = .{ x, y };
        }
    }

    pub fn setDrawAreaEnd(self: *@This(), x: i32, y: i32) void {
        if (!std.meta.eql(self.draw_area_end, .{ x, y })) {
            self.drawBatch();
            self.draw_area_end = .{ x, y };
        }
    }

    pub fn setDrawOffset(self: *@This(), x: i32, y: i32) void {
        self.draw_offset = .{ x, y };
    }

    pub fn setMaskBitSetting(self: *@This(), force_mask_bit: bool, check_mask_bit: bool) void {
        if (self.force_mask_bit != force_mask_bit or
            self.check_mask_bit != check_mask_bit)
        {
            self.drawBatch();
            self.force_mask_bit = force_mask_bit;
            self.check_mask_bit = check_mask_bit;
        }
    }

    pub fn setDithering(self: *@This(), enable: bool) void {
        self.enable_dithering = enable;
    }

    // =========================================================================
    // Command dispatching
    // =========================================================================

    fn exec(self: *@This(), cmd: RasterCommand) void {
        switch (cmd) {
            .fill_cmd => |c| self.clearRect(0, 0, consts.vram_res_x, consts.vram_res_y, c, false),
            .set_transparency_mode => |mode| self.setTransparencyMode(mode),
            .set_draw_area_start => |args| self.setDrawAreaStart(args.x, args.y),
            .set_draw_area_end => |args| self.setDrawAreaEnd(args.x, args.y),
            .set_draw_offset => |args| self.setDrawOffset(args.x, args.y),
            .set_dithering => |enable| self.setDithering(enable),
            .fill_rect_unmasked => |args| self.clearRect(args.x, args.y, args.w, args.h, args.color, true),
            .set_mask_bit_setting => |args| self.setMaskBitSetting(args.force_mask_bit, args.check_mask_bit),
            .set_texture_window => |args| self.setTextureWindow(args.mask_x, args.mask_y, args.offset_x, args.offset_y),
            .copy_rect => |args| self.copyRect(args.src_x, args.src_y, args.dest_x, args.dest_y, args.w, args.h),
            .draw_line_flat => |args| self.drawLineFlat(args),
            .draw_line_shaded => |args| self.drawLineShaded(args),
            .draw_rect_flat => |args| self.drawRectFlat(args),
            .draw_rect_textured => |args| self.drawRectTextured(args),
            .draw_triangle_flat => |args| self.drawTriangleFlat(args),
            .draw_triangle_shaded => |args| self.drawTriangleShaded(args),
            .draw_triangle_textured => |args| self.drawTriangleTextured(args),
            .draw_triangle_shaded_textured => |args| self.drawTriangleShadedTextured(args),
        }
    }

    fn drawRectFlat(self: *@This(), args: @FieldType(RasterCommand, "draw_rect_flat")) void {
        self.setBatchState(.{ .semi_trans = args.semi_trans });
        self.pushRect(args.x, args.y, args.w, args.h, args.color);
    }

    fn drawRectTextured(self: *@This(), args: @FieldType(RasterCommand, "draw_rect_textured")) void {
        const tex = self.setTexState(args.semi_trans, .{
            .texp = .{ args.texp_x, args.texp_y },
            .clut = .{ args.clut_x, args.clut_y },
            .depth = args.depth,
            .tex_blend = args.tex_blend,
        });
        self.pushRectUv(args.x, args.y, args.w, args.h, args.u, args.v, args.blend_color, tex);
    }

    fn drawLineFlat(self: *@This(), args: @FieldType(RasterCommand, "draw_line_flat")) void {
        self.setBatchState(.{ .semi_trans = args.semi_trans });
        self.pushLine(args.x0, args.y0, args.color, args.x1, args.y1, args.color, false);
    }

    fn drawLineShaded(self: *@This(), args: @FieldType(RasterCommand, "draw_line_shaded")) void {
        self.setBatchState(.{ .semi_trans = args.semi_trans });
        self.pushLine(args.x0, args.y0, args.c0, args.x1, args.y1, args.c1, self.enable_dithering);
    }

    fn drawTriangleFlat(self: *@This(), args: @FieldType(RasterCommand, "draw_triangle_flat")) void {
        self.setBatchState(.{ .semi_trans = args.semi_trans });
        self.pushTriangle(args.v0, args.v1, args.v2, args.color);
    }

    fn drawTriangleShaded(self: *@This(), args: @FieldType(RasterCommand, "draw_triangle_shaded")) void {
        self.setBatchState(.{ .semi_trans = args.semi_trans });
        self.pushTriangle(args.v0, args.v1, args.v2, null);
    }

    fn drawTriangleTextured(self: *@This(), args: @FieldType(RasterCommand, "draw_triangle_textured")) void {
        const tex = self.setTexState(args.semi_trans, .{
            .texp = .{ args.texp_x, args.texp_y },
            .clut = .{ args.clut_x, args.clut_y },
            .depth = args.depth,
            .tex_blend = args.tex_blend,
        });
        self.pushTriangleTex(args.v0, args.v1, args.v2, args.blend_color, tex);
    }

    fn drawTriangleShadedTextured(self: *@This(), args: @FieldType(RasterCommand, "draw_triangle_shaded_textured")) void {
        const tex = self.setTexState(args.semi_trans, .{
            .texp = .{ args.texp_x, args.texp_y },
            .clut = .{ args.clut_x, args.clut_y },
            .depth = args.depth,
            .tex_blend = true,
        });
        self.pushTriangleTex(args.v0, args.v1, args.v2, null, tex);
    }

    fn clearRect(self: *@This(), x: i32, y: i32, w: i32, h: i32, c: RGB8, clip_to_draw_area: bool) void {
        self.drawBatch();
        var x_min = @max(x, 0);
        var y_min = @max(y, 0);
        var x_max = @min(x + w - 1, consts.vram_res_x - 1);
        var y_max = @min(y + h - 1, consts.vram_res_y - 1);
        if (clip_to_draw_area) {
            x_min = @max(x_min, self.draw_area_start[0]);
            y_min = @max(y_min, self.draw_area_start[1]);
            x_max = @min(x_max, self.draw_area_end[0]);
            y_max = @min(y_max, self.draw_area_end[1]);
        }
        if (x_max < x_min or y_max < y_min) return;

        gl.enable(gl.SCISSOR_TEST);
        const s = self.upscale;
        gl.scissor(x_min * s, y_min * s, (x_max - x_min + 1) * s, (y_max - y_min + 1) * s);
        gl.clearColor(
            @as(f32, @floatFromInt(c.r >> 3)) / 31.0,
            @as(f32, @floatFromInt(c.g >> 3)) / 31.0,
            @as(f32, @floatFromInt(c.b >> 3)) / 31.0,
            0.0,
        );
        gl.clear(gl.COLOR_BUFFER_BIT);
        self.read_dirty.add(x_min, y_min, x_max, y_max);
    }

    fn copyRect(self: *@This(), src_x: i32, src_y: i32, dest_x: i32, dest_y: i32, w: i32, h: i32) void {
        self.drawBatch();
        gl.disable(gl.SCISSOR_TEST);
        gl.bindFramebuffer(gl.READ_FRAMEBUFFER, self.fbo);
        gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, self.fbo);
        const s = self.upscale;
        gl.blitFramebuffer(
            src_x * s,
            src_y * s,
            (src_x + w) * s,
            (src_y + h) * s,
            dest_x * s,
            dest_y * s,
            (dest_x + w) * s,
            (dest_y + h) * s,
            gl.COLOR_BUFFER_BIT,
            gl.NEAREST,
        );
        self.read_dirty.add(dest_x, dest_y, dest_x + w - 1, dest_y + h - 1);
    }

    // =========================================================================
    // Drawing calls
    // =========================================================================

    fn setBatchState(self: *@This(), state: BatchState) void {
        if (!std.meta.eql(self.batch_state, state)) {
            self.drawBatch();
        }
        self.batch_state = state;
    }

    fn setTexState(self: *@This(), semi_trans: bool, info: TexInfo) TexInfo {
        self.setBatchState(.{
            .semi_trans = semi_trans,
            .textured = true,
            .depth = info.depth,
        });
        self.batch_sampled.addRect(info.pageRect());
        if (info.depth != .bit15) self.batch_sampled.addRect(info.clutRect());
        return info;
    }

    inline fn pushVertex(self: *@This(), x: i32, y: i32, u: f32, v: f32, c: RGB8, tex: ?TexInfo, dither: bool) void {
        const px = x + self.draw_offset[0];
        const py = y + self.draw_offset[1];
        self.pushVertexF(@floatFromInt(px), @floatFromInt(py), u, v, c, tex, dither);
    }

    inline fn pushVertexF(self: *@This(), x: f32, y: f32, u: f32, v: f32, c: RGB8, tex: ?TexInfo, dither: bool) void {
        const px: i32 = @intFromFloat(@floor(x));
        const py: i32 = @intFromFloat(@floor(y));
        self.batch_bounds.add(px, py, px, py);
        var vertex: GlVertex = .{
            .x = x,
            .y = y,
            .u = u,
            .v = v,
            .r = c.r,
            .g = c.g,
            .b = c.b,
            .dither = @intFromBool(dither),
        };
        if (tex) |t| {
            vertex.texp_x = t.texp[0];
            vertex.texp_y = t.texp[1];
            vertex.clut_x = t.clut[0];
            vertex.clut_y = t.clut[1];
            vertex.tex_blend = @intFromBool(t.tex_blend);
        }
        self.batch.append(self.allocator, vertex) catch @panic("OOM");
    }

    fn pushTriangle(self: *@This(), v0: Vertex, v1: Vertex, v2: Vertex, flat: ?RGB8) void {
        self.pushTriangleTex(v0, v1, v2, flat, null);
    }

    fn pushTriangleTex(self: *@This(), v0: Vertex, v1: Vertex, v2: Vertex, flat: ?RGB8, tex: ?TexInfo) void {
        const dither = flat == null and self.enable_dithering; // gouraud only
        inline for (.{ v0, v1, v2 }) |v| {
            self.pushVertex(v.x, v.y, @floatFromInt(v.u), @floatFromInt(v.v), flat orelse v.color, tex, dither);
        }
    }

    fn pushRect(self: *@This(), x: i32, y: i32, w: i32, h: i32, c: RGB8) void {
        self.pushRectUv(x, y, w, h, 0, 0, c, null);
    }

    fn pushRectUv(self: *@This(), x: i32, y: i32, w: i32, h: i32, u: u16, v: u16, c: RGB8, tex: ?TexInfo) void {
        const tu0: f32 = @floatFromInt(u);
        const tv0: f32 = @floatFromInt(v);
        const tu1 = tu0 + @as(f32, @floatFromInt(w));
        const tv1 = tv0 + @as(f32, @floatFromInt(h));
        self.pushVertex(x, y, tu0, tv0, c, tex, false);
        self.pushVertex(x + w, y, tu1, tv0, c, tex, false);
        self.pushVertex(x, y + h, tu0, tv1, c, tex, false);
        self.pushVertex(x + w, y, tu1, tv0, c, tex, false);
        self.pushVertex(x + w, y + h, tu1, tv1, c, tex, false);
        self.pushVertex(x, y + h, tu0, tv1, c, tex, false);
    }

    fn pushLine(self: *@This(), x0: i32, y0: i32, c0: RGB8, x1: i32, y1: i32, c1: RGB8, dither: bool) void {
        const ox: f32 = @floatFromInt(self.draw_offset[0]);
        const oy: f32 = @floatFromInt(self.draw_offset[1]);
        const ax = @as(f32, @floatFromInt(x0)) + ox + 0.5;
        const ay = @as(f32, @floatFromInt(y0)) + oy + 0.5;
        const bx = @as(f32, @floatFromInt(x1)) + ox + 0.5;
        const by = @as(f32, @floatFromInt(y1)) + oy + 0.5;

        var dx = bx - ax;
        var dy = by - ay;
        const len = @sqrt(dx * dx + dy * dy);
        if (len == 0) {
            self.pushRect(x0, y0, 1, 1, c0);
            return;
        }
        dx /= len;
        dy /= len;

        const sx = ax - dx * 0.5;
        const sy = ay - dy * 0.5;
        const ex = bx + dx * 0.5;
        const ey = by + dy * 0.5;
        const nx = -dy * 0.5;
        const ny = dx * 0.5;

        self.pushVertexF(sx + nx, sy + ny, 0, 0, c0, null, dither);
        self.pushVertexF(sx - nx, sy - ny, 0, 0, c0, null, dither);
        self.pushVertexF(ex + nx, ey + ny, 0, 0, c1, null, dither);
        self.pushVertexF(sx - nx, sy - ny, 0, 0, c0, null, dither);
        self.pushVertexF(ex - nx, ey - ny, 0, 0, c1, null, dither);
        self.pushVertexF(ex + nx, ey + ny, 0, 0, c1, null, dither);
    }

    fn drawBatch(self: *@This()) void {
        if (self.batch.items.len == 0) return;
        defer {
            self.batch.clearRetainingCapacity();
            self.batch_bounds = .empty;
            self.batch_sampled = .empty;
        }

        const x0 = @max(self.draw_area_start[0], 0);
        const y0 = @max(self.draw_area_start[1], 0);
        const x1 = @min(self.draw_area_end[0], consts.vram_res_x - 1);
        const y1 = @min(self.draw_area_end[1], consts.vram_res_y - 1);
        if (x1 < x0 or y1 < y0) return;

        gl.enable(gl.SCISSOR_TEST);
        const s = self.upscale;
        gl.scissor(x0 * s, y0 * s, (x1 - x0 + 1) * s, (y1 - y0 + 1) * s);

        const st = self.batch_state;
        if (st.textured) self.updateReadTexture(self.batch_sampled);

        gl.useProgram(self.program);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, self.read_tex);
        gl.uniform1i(self.u.vram, 0);
        gl.uniform1i(self.u.upscale, s);
        gl.uniform1i(self.u.textured, @intFromBool(st.textured));
        gl.uniform1i(self.u.depth, @intFromEnum(st.depth));
        gl.uniform2ui(self.u.texwin_mask, self.texwin_mask[0], self.texwin_mask[1]);
        gl.uniform2ui(self.u.texwin_offset, self.texwin_offset[0], self.texwin_offset[1]);
        gl.bindVertexArray(self.vao);
        gl.bindBuffer(gl.ARRAY_BUFFER, self.vbo);
        gl.bufferData(
            gl.ARRAY_BUFFER,
            @intCast(self.batch.items.len * @sizeOf(GlVertex)),
            self.batch.items.ptr,
            gl.STREAM_DRAW,
        );

        const count: gl.Sizei = @intCast(self.batch.items.len);
        if (st.textured and st.semi_trans) {
            gl.uniform1i(self.u.mask_select, 0);
            self.setBlend(false);
            gl.drawArrays(gl.TRIANGLES, 0, count);
            gl.uniform1i(self.u.mask_select, 1);
            self.setBlend(true);
            gl.drawArrays(gl.TRIANGLES, 0, count);
        } else {
            gl.uniform1i(self.u.mask_select, -1);
            self.setBlend(st.semi_trans);
            gl.drawArrays(gl.TRIANGLES, 0, count);
        }

        const b = self.batch_bounds;
        if (b.intersects(.init(x0, y0, x1, y1))) {
            self.read_dirty.add(@max(b.x0, x0), @max(b.y0, y0), @min(b.x1, x1), @min(b.y1, y1));
        }
    }

    fn updateReadTexture(self: *@This(), sampled: Rect) void {
        if (!self.read_dirty.intersects(sampled)) return;
        const d = self.read_dirty;
        self.read_dirty = .empty;
        gl.bindFramebuffer(gl.READ_FRAMEBUFFER, self.fbo);
        gl.bindTexture(gl.TEXTURE_2D, self.read_tex);
        const s = self.upscale;
        gl.copyTexSubImage2D(
            gl.TEXTURE_2D,
            0,
            d.x0 * s,
            d.y0 * s,
            d.x0 * s,
            d.y0 * s,
            (d.x1 - d.x0 + 1) * s,
            (d.y1 - d.y0 + 1) * s,
        );
    }

    fn setBlend(self: *@This(), enable: bool) void {
        if (!enable) {
            gl.disable(gl.BLEND);
            return;
        }

        gl.enable(gl.BLEND);
        gl.blendEquationSeparate(gl.FUNC_ADD, gl.FUNC_ADD);

        switch (self.transparency_mode) {
            .@"B/2+F/2" => {
                gl.blendColor(0, 0, 0, 0.5);
                gl.blendFuncSeparate(gl.CONSTANT_ALPHA, gl.CONSTANT_ALPHA, gl.ONE, gl.ZERO);
            },
            .@"B+F" => {
                gl.blendFuncSeparate(gl.ONE, gl.ONE, gl.ONE, gl.ZERO);
            },
            .@"B-F" => {
                gl.blendEquationSeparate(gl.FUNC_REVERSE_SUBTRACT, gl.FUNC_ADD);
                gl.blendFuncSeparate(gl.ONE, gl.ONE, gl.ONE, gl.ZERO);
            },
            .@"B+F/4" => {
                gl.blendColor(0, 0, 0, 0.25);
                gl.blendFuncSeparate(gl.CONSTANT_ALPHA, gl.ONE, gl.ONE, gl.ZERO);
            },
        }
    }
};

fn createFramebuffer(tex: gl.Uint) gl.Uint {
    var fbo: gl.Uint = undefined;
    gl.genFramebuffers(1, &fbo);
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
    if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) != gl.FRAMEBUFFER_COMPLETE) @panic("vram framebuffer incomplete");
    gl.bindFramebuffer(gl.FRAMEBUFFER, 0);
    return fbo;
}

fn createVramTexture(upscale: i32) gl.Uint {
    var tex: gl.Uint = undefined;
    gl.genTextures(1, &tex);
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texImage2D(
        gl.TEXTURE_2D,
        0,
        gl.RGB5_A1,
        consts.vram_res_x * upscale,
        consts.vram_res_y * upscale,
        0,
        gl.RGBA,
        gl.UNSIGNED_SHORT_1_5_5_5_REV,
        null,
    );
    return tex;
}

fn glCreateProgram(vertex: []const u8, fragment: []const u8) gl.Uint {
    const vs = glCompileShader(gl.VERTEX_SHADER, vertex);
    defer gl.deleteShader(vs);
    const fs = glCompileShader(gl.FRAGMENT_SHADER, fragment);
    defer gl.deleteShader(fs);

    const program = gl.createProgram();
    gl.attachShader(program, vs);
    gl.attachShader(program, fs);
    gl.linkProgram(program);

    var success: gl.Int = undefined;
    gl.getProgramiv(program, gl.LINK_STATUS, &success);
    if (success == 0) {
        var info_log: [512]u8 = undefined;
        gl.getProgramInfoLog(program, 512, null, &info_log);
        log.err("shader program link failed: {s}", .{info_log});
        @panic("shader program link failed");
    }

    return program;
}

fn glCompileShader(kind: gl.Enum, source: []const u8) gl.Uint {
    const shader = gl.createShader(kind);
    gl.shaderSource(shader, 1, &source.ptr, &(@as(c_int, @intCast(source.len))));
    gl.compileShader(shader);

    var success: gl.Int = undefined;
    gl.getShaderiv(shader, gl.COMPILE_STATUS, &success);
    if (success == 0) {
        var info_log: [512]u8 = undefined;
        gl.getShaderInfoLog(shader, 512, null, &info_log);
        log.err("shader compilation failed: {s}", .{info_log});
        @panic("shader compilation failed");
    }
    return shader;
}
