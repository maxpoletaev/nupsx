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
const RGB8 = renderer.RGB8;
const Vram = [consts.vram_size]u16;

const vertex_shader_source = @embedFile("shaders/raster_vertex.glsl");
const fragment_shader_source = @embedFile("shaders/raster_fragment.glsl");

const empty_color: RGB8 = .init(255, 0, 255);

const Rect = struct {
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,

    fn add(self: *?Rect, x0: i32, y0: i32, x1: i32, y1: i32) void {
        if (self.*) |*r| {
            r.x0 = @min(r.x0, x0);
            r.y0 = @min(r.y0, y0);
            r.x1 = @max(r.x1, x1);
            r.y1 = @max(r.y1, y1);
        } else {
            self.* = .{ .x0 = x0, .y0 = y0, .x1 = x1, .y1 = y1 };
        }
    }
};

const GlVertex = extern struct {
    x: f32,
    y: f32,
    r: u8,
    g: u8,
    b: u8,
    a: u8 = 0,
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
    vram_dirty: ?Rect = null,

    gl_ready: bool = false,
    program: gl.Uint = 0,
    vao: gl.Uint = 0,
    vbo: gl.Uint = 0,
    fbo: gl.Uint = 0,
    tex: gl.Uint = 0,

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
        if (self.gl_ready) {
            gl.deleteProgram(self.program);
            gl.deleteVertexArrays(1, &self.vao);
            gl.deleteBuffers(1, &self.vbo);
            gl.deleteFramebuffers(1, &self.fbo);
            gl.deleteTextures(1, &self.tex);
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
            .upscale = 1,
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

        gl.disable(gl.DITHER);
        gl.genVertexArrays(1, &self.vao);
        gl.genBuffers(1, &self.vbo);
        gl.bindVertexArray(self.vao);
        gl.bindBuffer(gl.ARRAY_BUFFER, self.vbo);
        gl.vertexAttribPointer(0, 2, gl.FLOAT, gl.FALSE, @sizeOf(GlVertex), @ptrFromInt(@offsetOf(GlVertex, "x")));
        gl.enableVertexAttribArray(0);
        gl.vertexAttribPointer(1, 4, gl.UNSIGNED_BYTE, gl.TRUE, @sizeOf(GlVertex), @ptrFromInt(@offsetOf(GlVertex, "r")));
        gl.enableVertexAttribArray(1);
        gl.bindVertexArray(0);

        gl.genTextures(1, &self.tex);
        gl.bindTexture(gl.TEXTURE_2D, self.tex);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.texImage2D(
            gl.TEXTURE_2D,
            0,
            gl.RGB5_A1,
            consts.vram_res_x,
            consts.vram_res_y,
            0,
            gl.RGBA,
            gl.UNSIGNED_SHORT_1_5_5_5_REV,
            null,
        );

        gl.genFramebuffers(1, &self.fbo);
        gl.bindFramebuffer(gl.FRAMEBUFFER, self.fbo);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, self.tex, 0);
        if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) != gl.FRAMEBUFFER_COMPLETE) @panic("vram framebuffer incomplete");
        gl.bindFramebuffer(gl.FRAMEBUFFER, 0);
    }

    pub fn execute(self: *@This(), cmd: RasterCommand) void {
        self.queue.append(self.allocator, cmd) catch @panic("OOM");
    }

    pub fn setPixelRaw(self: *@This(), x: i32, y: i32, color: u16) void {
        const xx = x & (consts.vram_res_x - 1);
        const yy = y & (consts.vram_res_y - 1);
        self.vram[@intCast(yy * consts.vram_res_x + xx)] = color;
        Rect.add(&self.vram_dirty, xx, yy, xx, yy);
    }

    pub fn flush(self: *@This()) void {
        std.debug.assert(self.gl_ready);
        self.uploadVram();

        gl.bindFramebuffer(gl.FRAMEBUFFER, self.fbo);
        gl.viewport(0, 0, consts.vram_res_x, consts.vram_res_y);
        gl.disable(gl.BLEND);
        for (self.queue.items) |cmd| {
            self.exec(cmd);
        }
        self.queue.clearRetainingCapacity();
        self.drawBatch();

        gl.disable(gl.SCISSOR_TEST);
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

        gl.bindFramebuffer(gl.READ_FRAMEBUFFER, self.fbo);
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
        const d = self.vram_dirty orelse return;
        self.vram_dirty = null;
        gl.bindTexture(gl.TEXTURE_2D, self.tex);
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
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    pub fn setTransparencyMode(self: *@This(), mode: TransparencyMode) void {
        self.drawBatch();
        self.transparency_mode = mode;
    }

    pub fn setTextureWindow(self: *@This(), mask_x: u16, mask_y: u16, offset_x: u16, offset_y: u16) void {
        self.drawBatch();
        self.texwin_mask = .{ mask_x, mask_y };
        self.texwin_offset = .{ offset_x, offset_y };
    }

    pub fn setDrawAreaStart(self: *@This(), x: i32, y: i32) void {
        self.drawBatch();
        self.draw_area_start = .{ x, y };
    }

    pub fn setDrawOffset(self: *@This(), x: i32, y: i32) void {
        self.draw_offset = .{ x, y };
    }

    pub fn setDrawAreaEnd(self: *@This(), x: i32, y: i32) void {
        self.drawBatch();
        self.draw_area_end = .{ x, y };
    }

    pub fn setMaskBitSetting(self: *@This(), force_mask_bit: bool, check_mask_bit: bool) void {
        self.drawBatch();
        self.force_mask_bit = force_mask_bit;
        self.check_mask_bit = check_mask_bit;
    }

    pub fn setDithering(self: *@This(), enable: bool) void {
        self.drawBatch();
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
            // .draw_line_flat => |args| self.execDrawLineFlat(args),
            // .draw_line_shaded => |args| self.execDrawLineShaded(args),
            .draw_rect_flat => |args| self.pushRect(args.x, args.y, args.w, args.h, args.color),
            .draw_rect_textured => |args| self.pushRect(args.x, args.y, args.w, args.h, empty_color),
            .draw_triangle_flat => |args| self.pushTriangle(args.v0, args.v1, args.v2, args.color),
            .draw_triangle_shaded => |args| self.pushTriangle(args.v0, args.v1, args.v2, null),
            .draw_triangle_textured => |args| self.pushTriangle(args.v0, args.v1, args.v2, empty_color),
            .draw_triangle_shaded_textured => |args| self.pushTriangle(args.v0, args.v1, args.v2, empty_color),
            else => log.warn("unhandled rendering command: {s}", .{@tagName(cmd)}),
        }
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
        gl.scissor(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
        gl.clearColor(
            @as(f32, @floatFromInt(c.r >> 3)) / 31.0,
            @as(f32, @floatFromInt(c.g >> 3)) / 31.0,
            @as(f32, @floatFromInt(c.b >> 3)) / 31.0,
            0.0,
        );
        gl.clear(gl.COLOR_BUFFER_BIT);
    }

    fn copyRect(self: *@This(), src_x: i32, src_y: i32, dest_x: i32, dest_y: i32, w: i32, h: i32) void {
        self.drawBatch();
        gl.disable(gl.SCISSOR_TEST);
        gl.bindFramebuffer(gl.READ_FRAMEBUFFER, self.fbo);
        gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, self.fbo);
        gl.blitFramebuffer(
            src_x,
            src_y,
            src_x + w,
            src_y + h,
            dest_x,
            dest_y,
            dest_x + w,
            dest_y + h,
            gl.COLOR_BUFFER_BIT,
            gl.NEAREST,
        );
    }

    inline fn pushVertex(self: *@This(), x: i32, y: i32, c: RGB8) void {
        self.batch.append(self.allocator, .{
            .x = @floatFromInt(x + self.draw_offset[0]),
            .y = @floatFromInt(y + self.draw_offset[1]),
            .r = c.r,
            .g = c.g,
            .b = c.b,
        }) catch @panic("OOM");
    }

    fn pushTriangle(self: *@This(), v0: Vertex, v1: Vertex, v2: Vertex, flat: ?RGB8) void {
        inline for (.{ v0, v1, v2 }) |v| {
            self.pushVertex(v.x, v.y, flat orelse v.color);
        }
    }

    fn pushRect(self: *@This(), x: i32, y: i32, w: i32, h: i32, c: RGB8) void {
        self.pushVertex(x, y, c);
        self.pushVertex(x + w, y, c);
        self.pushVertex(x, y + h, c);
        self.pushVertex(x + w, y, c);
        self.pushVertex(x + w, y + h, c);
        self.pushVertex(x, y + h, c);
    }

    fn drawBatch(self: *@This()) void {
        if (self.batch.items.len == 0) return;
        defer self.batch.clearRetainingCapacity();

        const x0 = @max(self.draw_area_start[0], 0);
        const y0 = @max(self.draw_area_start[1], 0);
        const x1 = @min(self.draw_area_end[0], consts.vram_res_x - 1);
        const y1 = @min(self.draw_area_end[1], consts.vram_res_y - 1);
        if (x1 < x0 or y1 < y0) return;

        gl.enable(gl.SCISSOR_TEST);
        gl.scissor(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

        gl.useProgram(self.program);
        gl.bindVertexArray(self.vao);
        gl.bindBuffer(gl.ARRAY_BUFFER, self.vbo);
        gl.bufferData(gl.ARRAY_BUFFER, @intCast(self.batch.items.len * @sizeOf(GlVertex)), self.batch.items.ptr, gl.STREAM_DRAW);
        gl.drawArrays(gl.TRIANGLES, 0, @intCast(self.batch.items.len));
    }
};

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
