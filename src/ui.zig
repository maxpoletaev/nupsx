const std = @import("std");
const glfw = @import("zglfw");
const zopengl = @import("zopengl");
const options = @import("build_options");

const gpu_mod = @import("gpu.zig");
const sio0_mod = @import("sio0.zig");
const GPU = gpu_mod.GPU;
const SIO0 = sio0_mod.SIO0;

const gl = zopengl.bindings;
const log = std.log.scoped(.ui);

const gl_version = .{ 4, 1 };
const window_title = "nuPSX";
const scale = 3;
const window_width = 320 * scale;
const window_height = 240 * scale;
const ntsc_width = 960;
const ntsc_height = 720;

const vertex_shader_source = @embedFile("shaders/vertex.glsl");
const fragment_shader_source = @embedFile("shaders/fragment.glsl");
const ntsc_encoder_source = @embedFile("shaders/ntsc_encoder.glsl");
const ntsc_decoder_source = @embedFile("shaders/ntsc_decoder.glsl");

const Callback = struct {
    func: *const fn (*anyopaque) void,
    user_data: *anyopaque,

    fn call(self: @This()) void {
        self.func(self.user_data);
    }
};

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

const DisplayPass = struct {
    program: gl.Uint,
    u_display_offset: gl.Int,
    u_display_size: gl.Int,
    u_video_mode: gl.Int,
    u_display_range_y: gl.Int,
    u_vram_size: gl.Int,

    fn init() @This() {
        const program = createShaderProgram(vertex_shader_source, fragment_shader_source) catch {
            @panic("display shader compilation failed");
        };
        return .{
            .program = program,
            .u_display_offset = gl.getUniformLocation(program, "uDisplayOffset"),
            .u_display_size = gl.getUniformLocation(program, "uDisplaySize"),
            .u_video_mode = gl.getUniformLocation(program, "uVideoMode"),
            .u_display_range_y = gl.getUniformLocation(program, "uDisplayRangeY"),
            .u_vram_size = gl.getUniformLocation(program, "uVramSize"),
        };
    }

    fn draw(self: *@This(), fbo: gl.Uint, vram_tex: gl.Uint, fw: gl.Sizei, fh: gl.Sizei, gpu: *GPU) void {
        const color_depth = gpu.getColorDepth();
        const display_res = gpu.getDisplayRes();
        const start_x: f32 = @floatFromInt(gpu.gp1_display_area_start.x);
        var start_y: f32 = @floatFromInt(gpu.gp1_display_area_start.y);

        const res_scale: f32 = switch (color_depth) {
            .bit15 => @floatFromInt(gpu.rasterizer.framebuffer().upscale),
            .bit24 => 1.0, // always native since this is mostly mdec
        };
        const offset_x: f32 = switch (color_depth) {
            .bit15 => start_x * res_scale,
            .bit24 => start_x * (2.0 / 3.0),
        };
        start_y *= res_scale;
        const vram_size_x: f32 = switch (color_depth) {
            .bit15 => 1024.0 * res_scale,
            .bit24 => 682.0,
        };

        gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
        gl.viewport(0, 0, fw, fh);
        gl.clear(gl.COLOR_BUFFER_BIT);
        gl.useProgram(self.program);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, vram_tex);
        gl.uniform2f(self.u_display_offset, offset_x, start_y);
        gl.uniform2f(self.u_display_size, @as(f32, @floatFromInt(display_res[0])) * res_scale, @as(f32, @floatFromInt(display_res[1])) * res_scale);
        gl.uniform1i(self.u_video_mode, @intCast(gpu.getVideoMode()));
        gl.uniform2f(self.u_display_range_y, @floatFromInt(gpu.gp1_display_range_y.y1), @floatFromInt(gpu.gp1_display_range_y.y2));
        gl.uniform2f(self.u_vram_size, vram_size_x, 512.0 * res_scale);
        gl.drawArrays(gl.TRIANGLES, 0, 6);
    }

    fn deinit(self: *@This()) void {
        gl.deleteProgram(self.program);
    }
};

const NtscEncoder = struct {
    program: gl.Uint,
    u_resolution: gl.Int,
    u_frame: gl.Int,
    u_noise: gl.Int,

    fn init() @This() {
        const program = createShaderProgram(vertex_shader_source, ntsc_encoder_source) catch {
            @panic("ntsc encoder compilation failed");
        };
        return .{
            .program = program,
            .u_resolution = gl.getUniformLocation(program, "uResolution"),
            .u_frame = gl.getUniformLocation(program, "uFrame"),
            .u_noise = gl.getUniformLocation(program, "uNoise"),
        };
    }

    fn draw(self: *@This(), fbo: gl.Uint, rgb_tex: gl.Uint, fw: gl.Sizei, fh: gl.Sizei, frame: gl.Int) void {
        gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
        gl.viewport(0, 0, fw, fh);
        gl.clear(gl.COLOR_BUFFER_BIT);
        gl.useProgram(self.program);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, rgb_tex);
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

    fn draw(self: *@This(), output_fbo: gl.Uint, composite_tex: gl.Uint, w: gl.Sizei, h: gl.Sizei, frame: gl.Int) void {
        gl.bindFramebuffer(gl.FRAMEBUFFER, output_fbo);
        gl.viewport(0, 0, w, h);
        gl.useProgram(self.program);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, composite_tex);
        gl.uniform2f(self.u_resolution, @floatFromInt(w), @floatFromInt(h));
        gl.uniform1i(self.u_frame, frame);
        gl.drawArrays(gl.TRIANGLES, 0, 6);
    }

    fn deinit(self: *@This()) void {
        gl.deleteProgram(self.program);
    }
};

const KeyMapping = struct { glfw.Key, sio0_mod.Button };
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

const GamepadMapping = struct { u8, sio0_mod.Button };
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

const HotkeyAction = enum {
    close,
    mute_toggle,
};

const HotkeyMapping = struct { glfw.Key, HotkeyAction };
const hotkey_mappings = [_]HotkeyMapping{
    .{ glfw.Key.escape, .close },
    .{ glfw.Key.m, .mute_toggle },
};

pub const UI = struct {
    allocator: std.mem.Allocator,
    io: std.Io,
    window: *glfw.Window,
    gpu: *GPU,
    joy: *SIO0,
    vram_tex: gl.Uint,
    vao: gl.Uint,
    vbo: gl.Uint,
    display: DisplayPass,
    encoder: NtscEncoder,
    decoder: NtscDecoder,
    rgb_fbo: gl.Uint,
    rgb_tex: gl.Uint,
    composite_fbo: gl.Uint,
    composite_tex: gl.Uint,
    output_fbo: gl.Uint,
    output_tex: gl.Uint,
    ntsc_frame: gl.Int,
    ntsc_shader_enabled: bool = true,
    last_fps_update_time: f64 = 0,
    frame_count: u64 = 0,
    is_running: bool = true,
    next_frame_time: f64 = 0,
    uncapped: bool = false,
    game_name: ?[]const u8 = null,
    mute_toggle_callback: ?Callback = null,
    hotkey_down: std.enums.EnumArray(HotkeyAction, bool) = .initFill(false),

    const vertices = [_]f32{ // [x, y, u, v]
        -1.0, 1.0, 0.0, 0.0, // top left
        -1.0, -1.0, 0.0, 1.0, // bottom left
        1.0, -1.0, 1.0, 1.0, // bottom right
        -1.0, 1.0, 0.0, 0.0, // top left
        1.0, -1.0, 1.0, 1.0, // bottom right
        1.0, 1.0, 1.0, 0.0, // top right
    };

    pub fn init(allocator: std.mem.Allocator, io: std.Io, gpu: *GPU, joy: *SIO0) *@This() {
        glfw.init() catch @panic("GLFW");
        glfw.windowHint(.context_version_major, gl_version[0]);
        glfw.windowHint(.context_version_minor, gl_version[1]);
        glfw.windowHint(.opengl_profile, .opengl_core_profile);
        glfw.windowHint(.opengl_forward_compat, true);
        glfw.windowHint(.cocoa_retina_framebuffer, true);
        glfw.windowHint(.client_api, .opengl_api);
        glfw.windowHint(.doublebuffer, true);

        const window = glfw.createWindow(window_width, window_height, window_title, null, null) catch @panic("GLFW");
        window.setAspectRatio(4, 3);

        glfw.makeContextCurrent(window);
        glfw.swapInterval(0);

        zopengl.loadCoreProfile(glfw.getProcAddress, gl_version[0], gl_version[1]) catch @panic("OpenGL");

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

        // RGB FBO (display pass output, input to NTSC encoder)
        var rgb_fbo: gl.Uint = undefined;
        var rgb_tex: gl.Uint = undefined;
        gl.genFramebuffers(1, &rgb_fbo);
        gl.genTextures(1, &rgb_tex);
        gl.bindTexture(gl.TEXTURE_2D, rgb_tex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB8, window_width, window_height, 0, gl.RGB, gl.UNSIGNED_BYTE, null);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.bindFramebuffer(gl.FRAMEBUFFER, rgb_fbo);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, rgb_tex, 0);

        // Composite signal FBO (R16F to preserve full dynamic range)
        var composite_fbo: gl.Uint = undefined;
        var composite_tex: gl.Uint = undefined;
        gl.genFramebuffers(1, &composite_fbo);
        gl.genTextures(1, &composite_tex);
        gl.bindTexture(gl.TEXTURE_2D, composite_tex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.R16F, ntsc_width, ntsc_height, 0, gl.RED, gl.FLOAT, null);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.bindFramebuffer(gl.FRAMEBUFFER, composite_fbo);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, composite_tex, 0);

        // Output FBO: decoder renders here at fixed ntsc resolution, then blitted to window
        var output_fbo: gl.Uint = undefined;
        var output_tex: gl.Uint = undefined;
        gl.genFramebuffers(1, &output_fbo);
        gl.genTextures(1, &output_tex);
        gl.bindTexture(gl.TEXTURE_2D, output_tex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB8, ntsc_width, ntsc_height, 0, gl.RGB, gl.UNSIGNED_BYTE, null);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.bindFramebuffer(gl.FRAMEBUFFER, output_fbo);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, output_tex, 0);
        gl.bindFramebuffer(gl.FRAMEBUFFER, 0);

        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .io = io,
            .window = window,
            .gpu = gpu,
            .joy = joy,
            .vram_tex = vram_tex,
            .vao = vao,
            .vbo = vbo,
            .display = DisplayPass.init(),
            .encoder = NtscEncoder.init(),
            .decoder = NtscDecoder.init(),
            .rgb_fbo = rgb_fbo,
            .rgb_tex = rgb_tex,
            .composite_fbo = composite_fbo,
            .composite_tex = composite_tex,
            .output_fbo = output_fbo,
            .output_tex = output_tex,
            .ntsc_frame = 0,
            .uncapped = options.uncapped,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        if (self.game_name) |f| self.allocator.free(f);
        gl.deleteTextures(1, &self.vram_tex);
        gl.deleteTextures(1, &self.rgb_tex);
        gl.deleteFramebuffers(1, &self.rgb_fbo);
        gl.deleteTextures(1, &self.composite_tex);
        gl.deleteFramebuffers(1, &self.composite_fbo);
        gl.deleteTextures(1, &self.output_tex);
        gl.deleteFramebuffers(1, &self.output_fbo);
        gl.deleteBuffers(1, &self.vbo);
        gl.deleteVertexArrays(1, &self.vao);
        self.display.deinit();
        self.encoder.deinit();
        self.decoder.deinit();
        self.window.destroy();
        glfw.terminate();

        const allocator = self.allocator;
        allocator.destroy(self);
    }

    pub fn setGameNameFromPath(self: *@This(), path: []const u8) void {
        const basename = std.fs.path.basename(path);

        var game_name = basename;
        if (std.mem.lastIndexOfScalar(u8, basename, '.')) |dot| {
            if (dot > 0) game_name = basename[0..dot];
        }

        if (self.game_name) |old| self.allocator.free(old);
        self.game_name = self.allocator.dupe(u8, game_name) catch @panic("OOM");
    }

    pub fn setUncapped(self: *@This(), uncapped: bool) void {
        self.uncapped = uncapped;
    }

    pub fn setNtscShaderEnabled(self: *@This(), enabled: bool) void {
        self.ntsc_shader_enabled = enabled;
    }

    pub fn setMuteCallback(
        self: *@This(),
        func: *const fn (*anyopaque) void,
        user_data: *anyopaque,
    ) void {
        self.mute_toggle_callback = .{
            .func = func,
            .user_data = user_data,
        };
    }

    pub fn update(self: *@This()) void {
        const now = glfw.getTime();
        if (!self.uncapped and self.next_frame_time > now) {
            const sleep_seconds = self.next_frame_time - now;
            const ns: i96 = @intFromFloat(sleep_seconds * std.time.ns_per_s);
            self.io.sleep(.{ .nanoseconds = ns }, .awake) catch {};
        }

        self.handleInput();
        self.updateInternal(glfw.getTime());

        const after = glfw.getTime();
        self.next_frame_time = @max(self.next_frame_time + self.gpu.targetFrameTime(), after);
    }

    fn triggerHotkey(self: *@This(), action: HotkeyAction) void {
        switch (action) {
            .mute_toggle => if (self.mute_toggle_callback) |cb| cb.call(),
            .close => glfw.setWindowShouldClose(self.window, true),
        }
    }

    fn handleInput(self: *@This()) void {
        if (glfw.getKey(self.window, glfw.Key.escape) == .press) {
            glfw.setWindowShouldClose(self.window, true);
        }

        if (self.window.shouldClose()) {
            self.is_running = false;
        }

        for (hotkey_mappings) |mapping| {
            const pressed = glfw.getKey(self.window, mapping[0]) == .press;
            const was_pressed = self.hotkey_down.get(mapping[1]);
            if (pressed and !was_pressed) self.triggerHotkey(mapping[1]);
            self.hotkey_down.set(mapping[1], pressed);
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

        if (self.game_name) |filename| {
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
        const fb = self.gpu.rasterizer.framebuffer();
        gl.bindTexture(gl.TEXTURE_2D, self.vram_tex);
        gl.pixelStorei(gl.UNPACK_ROW_LENGTH, 0);
        switch (self.gpu.getColorDepth()) {
            .bit15 => gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB5, fb.width, fb.height, 0, gl.RGBA, gl.UNSIGNED_SHORT_1_5_5_5_REV, fb.pixels.ptr),
            .bit24 => gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB8, 682, 512, 0, gl.RGB, gl.UNSIGNED_BYTE, self.gpu.vram),
        }
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
        const win_w: gl.Sizei = @intCast(fb_size[0]);
        const win_h: gl.Sizei = @intCast(fb_size[1]);

        self.uploadVram();
        gl.bindVertexArray(self.vao);

        if (self.ntsc_shader_enabled) {
            self.display.draw(self.rgb_fbo, self.vram_tex, window_width, window_height, self.gpu);
            self.encoder.draw(self.composite_fbo, self.rgb_tex, ntsc_width, ntsc_height, self.ntsc_frame);
            self.decoder.draw(self.output_fbo, self.composite_tex, ntsc_width, ntsc_height, self.ntsc_frame);
            self.ntsc_frame +%= 1;

            // Blit output FBO to window, stretching to fit
            gl.bindFramebuffer(gl.READ_FRAMEBUFFER, self.output_fbo);
            gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, 0);
            gl.blitFramebuffer(0, 0, ntsc_width, ntsc_height, 0, 0, win_w, win_h, gl.COLOR_BUFFER_BIT, gl.LINEAR);
        } else {
            self.display.draw(0, self.vram_tex, win_w, win_h, self.gpu);
        }

        self.window.swapBuffers();
    }
};
