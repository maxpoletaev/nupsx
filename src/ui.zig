const std = @import("std");
const glfw = @import("zglfw");
const zopengl = @import("zopengl");

const gpu_mod = @import("gpu.zig");
const joy_mod = @import("joy.zig");
const GPU = gpu_mod.GPU;
const Joypad = joy_mod.Joypad;

const gl = zopengl.bindings;
const log = std.log.scoped(.ui);

const gl_version = .{ 4, 1 };
const window_title = "nuPSX";
const window_width = 320 * 3;
const window_height = 240 * 3;

const vertex_shader_source = @embedFile("shaders/vertex.glsl");
const ntsc_encoder_source = @embedFile("shaders/ntsc_encoder.glsl");
const ntsc_decoder_source = @embedFile("shaders/ntsc_decoder.glsl");

fn createShaderProgram(vertex: []const u8, fragment: []const u8) !gl.Uint {
    const vertex_shader = gl.createShader(gl.VERTEX_SHADER);
    defer gl.deleteShader(vertex_shader);

    gl.shaderSource(vertex_shader, 1, &vertex.ptr, &(@as(c_int, @intCast(vertex.len))));
    gl.compileShader(vertex_shader);

    var success: gl.Int = undefined;
    gl.getShaderiv(vertex_shader, gl.COMPILE_STATUS, &success);
    if (success == 0) {
        var info_log: [512]u8 = undefined;
        gl.getShaderInfoLog(vertex_shader, 512, null, &info_log);
        log.err("vertex shader compilation failed: {s}", .{info_log});
        return error.ShaderCompilationFailed;
    }

    const fragment_shader = gl.createShader(gl.FRAGMENT_SHADER);
    defer gl.deleteShader(fragment_shader);

    gl.shaderSource(fragment_shader, 1, &fragment.ptr, &(@as(c_int, @intCast(fragment.len))));
    gl.compileShader(fragment_shader);

    gl.getShaderiv(fragment_shader, gl.COMPILE_STATUS, &success);
    if (success == 0) {
        var info_log: [512]u8 = undefined;
        gl.getShaderInfoLog(fragment_shader, 512, null, &info_log);
        log.err("fragment shader compilation failed: {s}", .{info_log});
        return error.ShaderCompilationFailed;
    }

    const shader_program = gl.createProgram();
    gl.attachShader(shader_program, vertex_shader);
    gl.attachShader(shader_program, fragment_shader);
    gl.linkProgram(shader_program);

    gl.getProgramiv(shader_program, gl.LINK_STATUS, &success);
    if (success == 0) {
        var info_log: [512]u8 = undefined;
        gl.getProgramInfoLog(shader_program, 512, null, &info_log);
        log.err("shader program linking failed: {s}", .{info_log});
        return error.ShaderLinkingFailed;
    }

    return shader_program;
}

const NtscEncoder = struct {
    program: gl.Uint,
    u_display_offset: gl.Int,
    u_display_size: gl.Int,
    u_display_range_y: gl.Int,
    u_vram_size: gl.Int,
    u_resolution: gl.Int,
    u_frame: gl.Int,
    u_noise: gl.Int,

    fn init() @This() {
        const program = createShaderProgram(vertex_shader_source, ntsc_encoder_source) catch {
            @panic("ntsc encoder compilation failed");
        };
        return .{
            .program = program,
            .u_display_offset = gl.getUniformLocation(program, "uDisplayOffset"),
            .u_display_size = gl.getUniformLocation(program, "uDisplaySize"),
            .u_display_range_y = gl.getUniformLocation(program, "uDisplayRangeY"),
            .u_vram_size = gl.getUniformLocation(program, "uVramSize"),
            .u_resolution = gl.getUniformLocation(program, "uResolution"),
            .u_frame = gl.getUniformLocation(program, "uFrame"),
            .u_noise = gl.getUniformLocation(program, "uNoise"),
        };
    }

    fn draw(self: *@This(), fbo: gl.Uint, vram_tex: gl.Uint, fw: gl.Sizei, fh: gl.Sizei, gpu: *GPU, frame: gl.Int) void {
        const color_depth = gpu.getColorDepth();
        const display_res = gpu.getDisplayRes();
        const start_x: f32 = @floatFromInt(gpu.gp1_display_area_start.x);
        var start_y: f32 = @floatFromInt(gpu.gp1_display_area_start.y);
        if (start_y == 2) start_y = 0; // HACK: old bioses set this to 2, resulting in cluts being displayed in the viewport

        const offset_x: f32 = switch (color_depth) {
            .bit15 => start_x,
            .bit24 => start_x * (2.0 / 3.0),
        };
        const vram_size_x: f32 = switch (color_depth) {
            .bit15 => 1024.0,
            .bit24 => 682.0,
        };

        gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
        gl.viewport(0, 0, fw, fh);
        gl.clear(gl.COLOR_BUFFER_BIT);
        gl.useProgram(self.program);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, vram_tex);
        gl.uniform2f(self.u_display_offset, offset_x, start_y);
        gl.uniform2f(self.u_display_size, @as(f32, @floatFromInt(display_res[0])), @as(f32, @floatFromInt(display_res[1])));
        gl.uniform2f(self.u_display_range_y, @floatFromInt(gpu.gp1_display_range_y.y1), @floatFromInt(gpu.gp1_display_range_y.y2));
        gl.uniform2f(self.u_vram_size, vram_size_x, 512.0);
        gl.uniform2f(self.u_resolution, @floatFromInt(fw), @floatFromInt(fh));
        gl.uniform1i(self.u_frame, frame);
        gl.uniform1f(self.u_noise, 0.03);
        gl.drawArrays(gl.TRIANGLES, 0, 6);
    }

    fn deinit(self: *@This()) void {
        gl.deleteProgram(self.program);
    }
};

const NtscDecoder = struct {
    program: gl.Uint,
    u_resolution: gl.Int,
    u_frame: gl.Int,

    fn init() @This() {
        const program = createShaderProgram(vertex_shader_source, ntsc_decoder_source) catch {
            @panic("ntsc decoder compilation failed");
        };
        return .{
            .program = program,
            .u_resolution = gl.getUniformLocation(program, "uResolution"),
            .u_frame = gl.getUniformLocation(program, "uFrame"),
        };
    }

    fn draw(self: *@This(), composite_tex: gl.Uint, fw: gl.Sizei, fh: gl.Sizei, frame: gl.Int) void {
        gl.bindFramebuffer(gl.FRAMEBUFFER, 0);
        gl.viewport(0, 0, fw, fh);
        gl.useProgram(self.program);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, composite_tex);
        gl.uniform2f(self.u_resolution, @floatFromInt(fw), @floatFromInt(fh));
        gl.uniform1i(self.u_frame, frame);
        gl.drawArrays(gl.TRIANGLES, 0, 6);
    }

    fn deinit(self: *@This()) void {
        gl.deleteProgram(self.program);
    }
};

pub const UI = struct {
    allocator: std.mem.Allocator,
    window: *glfw.Window,
    gpu: *GPU,
    joy: *Joypad,
    vram_tex: gl.Uint,
    vao: gl.Uint,
    vbo: gl.Uint,
    encoder: NtscEncoder,
    decoder: NtscDecoder,
    composite_fbo: gl.Uint,
    composite_tex: gl.Uint,
    composite_fbo_w: gl.Sizei,
    composite_fbo_h: gl.Sizei,
    ntsc_frame: gl.Int,
    last_fps_update_time: f64 = 0,
    frame_count: u64 = 0,
    is_running: bool = true,
    filename: ?[]const u8 = null,

    const vertices = [_]f32{ // [x, y, u, v]
        -1.0, 1.0, 0.0, 0.0, // top left
        -1.0, -1.0, 0.0, 1.0, // bottom left
        1.0, -1.0, 1.0, 1.0, // bottom right
        -1.0, 1.0, 0.0, 0.0, // top left
        1.0, -1.0, 1.0, 1.0, // bottom right
        1.0, 1.0, 1.0, 0.0, // top right
    };

    pub fn init(allocator: std.mem.Allocator, gpu: *GPU, joy: *Joypad) !*@This() {
        try glfw.init();
        glfw.windowHint(.context_version_major, gl_version[0]);
        glfw.windowHint(.context_version_minor, gl_version[1]);
        glfw.windowHint(.opengl_profile, .opengl_core_profile);
        glfw.windowHint(.opengl_forward_compat, true);
        glfw.windowHint(.cocoa_retina_framebuffer, true);
        glfw.windowHint(.client_api, .opengl_api);
        glfw.windowHint(.doublebuffer, true);

        const window = try glfw.createWindow(window_width, window_height, window_title, null, null);
        window.setAspectRatio(4, 3);

        glfw.makeContextCurrent(window);
        glfw.swapInterval(0);

        try zopengl.loadCoreProfile(glfw.getProcAddress, gl_version[0], gl_version[1]);

        // VAO and VBO for fullscreen quad
        var vao: gl.Uint = undefined;
        var vbo: gl.Uint = undefined;
        gl.genVertexArrays(1, &vao);
        gl.genBuffers(1, &vbo);
        gl.bindVertexArray(vao);
        gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
        gl.bufferData(gl.ARRAY_BUFFER, @intCast(vertices.len * @sizeOf(f32)), &vertices, gl.STATIC_DRAW);
        gl.vertexAttribPointer(0, 2, gl.FLOAT, gl.FALSE, 4 * @sizeOf(f32), null);
        gl.enableVertexAttribArray(0);
        gl.vertexAttribPointer(1, 2, gl.FLOAT, gl.FALSE, 4 * @sizeOf(f32), @ptrFromInt(2 * @sizeOf(f32)));
        gl.enableVertexAttribArray(1);

        // VRAM texture
        var vram_tex: gl.Uint = undefined;
        gl.genTextures(1, &vram_tex);
        gl.bindTexture(gl.TEXTURE_2D, vram_tex);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

        // Composite signal FBO (R16F to preserve full dynamic range)
        var composite_fbo: gl.Uint = undefined;
        var composite_tex: gl.Uint = undefined;
        gl.genFramebuffers(1, &composite_fbo);
        gl.genTextures(1, &composite_tex);
        gl.bindTexture(gl.TEXTURE_2D, composite_tex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.R16F, window_width, window_height, 0, gl.RED, gl.FLOAT, null);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.bindFramebuffer(gl.FRAMEBUFFER, composite_fbo);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, composite_tex, 0);
        gl.bindFramebuffer(gl.FRAMEBUFFER, 0);

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .window = window,
            .gpu = gpu,
            .joy = joy,
            .vram_tex = vram_tex,
            .vao = vao,
            .vbo = vbo,
            .encoder = NtscEncoder.init(),
            .decoder = NtscDecoder.init(),
            .composite_fbo = composite_fbo,
            .composite_tex = composite_tex,
            .composite_fbo_w = window_width,
            .composite_fbo_h = window_height,
            .ntsc_frame = 0,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        if (self.filename) |f| self.allocator.free(f);
        gl.deleteTextures(1, &self.vram_tex);
        gl.deleteTextures(1, &self.composite_tex);
        gl.deleteFramebuffers(1, &self.composite_fbo);
        gl.deleteBuffers(1, &self.vbo);
        gl.deleteVertexArrays(1, &self.vao);
        self.encoder.deinit();
        self.decoder.deinit();
        self.window.destroy();
        glfw.terminate();

        const allocator = self.allocator;
        allocator.destroy(self);
    }

    pub fn setFilename(self: *@This(), path: []const u8) !void {
        if (self.filename) |old| self.allocator.free(old);
        const basename = std.fs.path.basename(path);
        self.filename = try self.allocator.dupe(u8, basename);
    }

    pub fn update(self: *@This()) void {
        self.handleInput();
        self.updateInternal(glfw.getTime());
    }

    const KeyMapping = struct { glfw.Key, joy_mod.Button };
    const key_mappings = [_]KeyMapping{
        .{ glfw.Key.w, .up },
        .{ glfw.Key.a, .left },
        .{ glfw.Key.s, .down },
        .{ glfw.Key.d, .right },
        .{ glfw.Key.k, .cross },
        .{ glfw.Key.l, .circle },
        .{ glfw.Key.j, .square },
        .{ glfw.Key.i, .triangle },
        .{ glfw.Key.e, .l1 },
        .{ glfw.Key.q, .l2 },
        .{ glfw.Key.u, .r1 },
        .{ glfw.Key.o, .r2 },
        .{ glfw.Key.enter, .start },
        .{ glfw.Key.right_shift, .select },
    };

    const GamepadMapping = struct { u8, joy_mod.Button };
    const gamepad_mappings = [_]GamepadMapping{
        .{ @intFromEnum(glfw.Gamepad.Button.dpad_up), .up },
        .{ @intFromEnum(glfw.Gamepad.Button.dpad_down), .down },
        .{ @intFromEnum(glfw.Gamepad.Button.dpad_left), .left },
        .{ @intFromEnum(glfw.Gamepad.Button.dpad_right), .right },
        .{ @intFromEnum(glfw.Gamepad.Button.cross), .cross },
        .{ @intFromEnum(glfw.Gamepad.Button.circle), .circle },
        .{ @intFromEnum(glfw.Gamepad.Button.square), .square },
        .{ @intFromEnum(glfw.Gamepad.Button.triangle), .triangle },
        .{ @intFromEnum(glfw.Gamepad.Button.left_bumper), .l1 },
        .{ @intFromEnum(glfw.Gamepad.Button.left_thumb), .l2 },
        .{ @intFromEnum(glfw.Gamepad.Button.right_bumper), .r1 },
        .{ @intFromEnum(glfw.Gamepad.Button.right_thumb), .r2 },
        .{ @intFromEnum(glfw.Gamepad.Button.start), .start },
        .{ @intFromEnum(glfw.Gamepad.Button.back), .select },
    };

    fn handleInput(self: *@This()) void {
        if (glfw.getKey(self.window, glfw.Key.escape) == .press) {
            glfw.setWindowShouldClose(self.window, true);
        }

        if (self.window.shouldClose()) {
            self.is_running = false;
        }

        inline for (key_mappings) |mapping| {
            const key_state = glfw.getKey(self.window, mapping[0]);
            const pressed = key_state == .press or key_state == .repeat;
            self.joy.setButtonState(mapping[1], pressed);
        }

        const gamepad_id = 0;

        if (glfw.joystickIsGamepad(@enumFromInt(gamepad_id))) {
            const gp_state = glfw.Gamepad.getState(@enumFromInt(gamepad_id)) catch |err| {
                log.err("failed to get gamepad state: {}", .{err});
                return;
            };
            inline for (gamepad_mappings) |mapping| {
                const pressed = gp_state.buttons[mapping[0]] == .press;
                self.joy.setButtonState(mapping[1], pressed);
            }
        }
    }

    fn updateTitle(self: *@This(), now: f64) void {
        self.frame_count += 1;
        const fps_elapsed = now - self.last_fps_update_time;
        if (fps_elapsed < 1.0) return;

        const fps = @as(f64, @floatFromInt(self.frame_count)) / fps_elapsed;
        var title_buf: [256]u8 = undefined;

        if (self.filename) |filename| {
            const title = std.fmt.bufPrintZ(
                &title_buf,
                "{s} - {s} - {d:.1} FPS",
                .{ window_title, filename, fps },
            ) catch unreachable;
            self.window.setTitle(title);
        } else {
            const title = std.fmt.bufPrintZ(
                &title_buf,
                "{s} - NO DISK - {d:.1} FPS",
                .{ window_title, fps },
            ) catch unreachable;
            self.window.setTitle(title);
        }

        self.last_fps_update_time = now;
        self.frame_count = 0;
    }

    fn uploadVram(self: *@This()) void {
        gl.bindTexture(gl.TEXTURE_2D, self.vram_tex);
        gl.pixelStorei(gl.UNPACK_ROW_LENGTH, 0);
        switch (self.gpu.getColorDepth()) {
            .bit15 => gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB5, 1024, 512, 0, gl.RGBA, gl.UNSIGNED_SHORT_1_5_5_5_REV, self.gpu.vram),
            .bit24 => gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB8, 682, 512, 0, gl.RGB, gl.UNSIGNED_BYTE, self.gpu.vram),
        }
    }

    fn resizeCompositeFbo(self: *@This(), fw: gl.Sizei, fh: gl.Sizei) void {
        if (fw == self.composite_fbo_w and fh == self.composite_fbo_h) return;
        self.composite_fbo_w = fw;
        self.composite_fbo_h = fh;
        gl.bindTexture(gl.TEXTURE_2D, self.composite_tex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.R16F, fw, fh, 0, gl.RED, gl.FLOAT, null);
    }

    fn updateInternal(self: *@This(), now: f64) void {
        glfw.pollEvents();
        self.updateTitle(now);

        gl.clearColor(0.0, 0.0, 0.0, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT);

        if (self.gpu.gp1_display_enable != .on) {
            self.window.swapBuffers();
            return;
        }

        const fb_size = self.window.getFramebufferSize();
        const fw: gl.Sizei = @intCast(fb_size[0]);
        const fh: gl.Sizei = @intCast(fb_size[1]);

        self.uploadVram();
        self.resizeCompositeFbo(fw, fh);

        gl.bindVertexArray(self.vao);
        self.encoder.draw(self.composite_fbo, self.vram_tex, fw, fh, self.gpu, self.ntsc_frame);
        self.decoder.draw(self.composite_tex, fw, fh, self.ntsc_frame);

        self.ntsc_frame +%= 1;
        self.window.swapBuffers();
    }
};
