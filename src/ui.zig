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
        log.err("Vertex shader compilation failed: {s}", .{info_log});
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
        log.err("Fragment shader compilation failed: {s}", .{info_log});
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
        log.err("Shader program linking failed: {s}", .{info_log});
        return error.ShaderLinkingFailed;
    }

    return shader_program;
}

pub const UI = struct {
    allocator: std.mem.Allocator,
    window: *glfw.Window,
    gpu: *GPU,
    joy: *Joypad,
    vram_tex: gl.Uint,
    vao: gl.Uint,
    vbo: gl.Uint,
    // NTSC encode pass
    encoder_program: gl.Uint,
    composite_fbo: gl.Uint,
    composite_tex: gl.Uint,
    composite_fbo_w: gl.Sizei,
    composite_fbo_h: gl.Sizei,
    enc_u_display_offset: gl.Int,
    enc_u_display_size: gl.Int,
    enc_u_display_range_y: gl.Int,
    enc_u_vram_size: gl.Int,
    enc_u_resolution: gl.Int,
    enc_u_frame: gl.Int,
    enc_u_noise: gl.Int,
    // NTSC decode pass
    decoder_program: gl.Uint,
    dec_u_resolution: gl.Int,
    dec_u_frame: gl.Int,
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

        const window = try glfw.createWindow(640, 480, window_title, null, null);
        window.setAspectRatio(4, 3);

        glfw.makeContextCurrent(window);
        glfw.swapInterval(0);

        try zopengl.loadCoreProfile(glfw.getProcAddress, gl_version[0], gl_version[1]);

        const encoder_program = try createShaderProgram(vertex_shader_source, ntsc_encoder_source);
        const decoder_program = try createShaderProgram(vertex_shader_source, ntsc_decoder_source);

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
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.R16F, 640, 480, 0, gl.RED, gl.FLOAT, null);
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
            .encoder_program = encoder_program,
            .composite_fbo = composite_fbo,
            .composite_tex = composite_tex,
            .composite_fbo_w = 640,
            .composite_fbo_h = 480,
            .enc_u_display_offset = gl.getUniformLocation(encoder_program, "uDisplayOffset"),
            .enc_u_display_size = gl.getUniformLocation(encoder_program, "uDisplaySize"),
            .enc_u_display_range_y = gl.getUniformLocation(encoder_program, "uDisplayRangeY"),
            .enc_u_vram_size = gl.getUniformLocation(encoder_program, "uVramSize"),
            .enc_u_resolution = gl.getUniformLocation(encoder_program, "uResolution"),
            .enc_u_frame = gl.getUniformLocation(encoder_program, "uFrame"),
            .enc_u_noise = gl.getUniformLocation(encoder_program, "uNoise"),
            .decoder_program = decoder_program,
            .dec_u_resolution = gl.getUniformLocation(decoder_program, "uResolution"),
            .dec_u_frame = gl.getUniformLocation(decoder_program, "uFrame"),
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
        gl.deleteProgram(self.encoder_program);
        gl.deleteProgram(self.decoder_program);
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

    fn updateInternal(self: *@This(), now: f64) void {
        glfw.pollEvents();

        self.frame_count += 1;
        const fps_elapsed = now - self.last_fps_update_time;

        if (fps_elapsed >= 1.0) {
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

        gl.clearColor(0.0, 0.0, 0.0, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT);

        if (self.gpu.gp1_display_enable != .on) {
            self.window.swapBuffers();
            return;
        }

        const color_depth = self.gpu.getColorDepth();

        // Upload VRAM texture
        gl.bindTexture(gl.TEXTURE_2D, self.vram_tex);
        gl.pixelStorei(gl.UNPACK_ROW_LENGTH, 0);
        switch (color_depth) {
            .bit15 => gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB5, 1024, 512, 0, gl.RGBA, gl.UNSIGNED_SHORT_1_5_5_5_REV, self.gpu.vram),
            .bit24 => gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB8, 682, 512, 0, gl.RGB, gl.UNSIGNED_BYTE, self.gpu.vram),
        }

        const display_res = self.gpu.getDisplayRes();
        const start_x: f32 = @floatFromInt(self.gpu.gp1_display_area_start.x);
        var start_y: f32 = @floatFromInt(self.gpu.gp1_display_area_start.y);
        if (start_y == 2) start_y = 0; // HACK: old bioses set this to 2, resulting in cluts being displayed in the viewport

        const display_range_y1: f32 = @floatFromInt(self.gpu.gp1_display_range_y.y1);
        const display_range_y2: f32 = @floatFromInt(self.gpu.gp1_display_range_y.y2);

        const offset_x: f32 = switch (color_depth) {
            .bit15 => start_x,
            .bit24 => start_x * (2.0 / 3.0),
        };
        const vram_size_x: f32 = switch (color_depth) {
            .bit15 => 1024.0,
            .bit24 => 682.0,
        };

        const fb_size = self.window.getFramebufferSize();
        const fw: gl.Sizei = @intCast(fb_size[0]);
        const fh: gl.Sizei = @intCast(fb_size[1]);

        if (fw != self.composite_fbo_w or fh != self.composite_fbo_h) {
            self.composite_fbo_w = fw;
            self.composite_fbo_h = fh;
            gl.bindTexture(gl.TEXTURE_2D, self.composite_tex);
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.R16F, fw, fh, 0, gl.RED, gl.FLOAT, null);
        }

        gl.bindVertexArray(self.vao);

        // Pass 1: encode VRAM → composite signal
        gl.bindFramebuffer(gl.FRAMEBUFFER, self.composite_fbo);
        gl.viewport(0, 0, fw, fh);
        gl.clear(gl.COLOR_BUFFER_BIT);

        gl.useProgram(self.encoder_program);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, self.vram_tex);
        gl.uniform2f(self.enc_u_display_offset, offset_x, start_y);
        gl.uniform2f(self.enc_u_display_size, @as(f32, @floatFromInt(display_res[0])), @as(f32, @floatFromInt(display_res[1])));
        gl.uniform2f(self.enc_u_display_range_y, display_range_y1, display_range_y2);
        gl.uniform2f(self.enc_u_vram_size, vram_size_x, 512.0);
        gl.uniform2f(self.enc_u_resolution, @floatFromInt(fw), @floatFromInt(fh));
        gl.uniform1i(self.enc_u_frame, self.ntsc_frame);
        gl.uniform1f(self.enc_u_noise, 0.03);
        gl.drawArrays(gl.TRIANGLES, 0, 6);

        // Pass 2: decode composite → screen
        gl.bindFramebuffer(gl.FRAMEBUFFER, 0);
        gl.viewport(0, 0, fw, fh);

        gl.useProgram(self.decoder_program);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, self.composite_tex);
        gl.uniform2f(self.dec_u_resolution, @floatFromInt(fw), @floatFromInt(fh));
        gl.uniform1i(self.dec_u_frame, self.ntsc_frame);
        gl.drawArrays(gl.TRIANGLES, 0, 6);

        self.ntsc_frame +%= 1;

        self.window.swapBuffers();
    }
};
