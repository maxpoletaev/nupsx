const std = @import("std");
const zgui = @import("zgui");
const glfw = @import("zglfw");
const zopengl = @import("zopengl");

const mem = @import("../mem.zig");
const cpu_mod = @import("../cpu.zig");
const Disasm = @import("../Disasm.zig");

const Bus = mem.Bus;
const CPU = cpu_mod.CPU;

const VramView = @import("VramView.zig");
const TTYView = @import("TTYView.zig");
const AssemblyView = @import("AssemblyView.zig");
const CPUView = @import("CPUView.zig");
const GPUView = @import("GPUView.zig");
const TimerView = @import("TimerView.zig");

const default_font = @embedFile("../assets/freepixel.ttf");
const default_font_size = 16.0;
const target_frame_time: f64 = 1.0 / 60.0;
const window_title = "nuPSX (Debug)";
const gl_version = .{ 4, 1 };
const gl = zopengl.bindings;

const logger = std.log.scoped(.debug);

allocator: std.mem.Allocator,
window: *glfw.Window,
bus: *Bus,
cpu_view: *CPUView,
gpu_view: *GPUView,
timer_view: *TimerView,
assembly_view: *AssemblyView,
vram_view: *VramView,
tty_view: *TTYView,
last_frame_time: f64 = 0,
next_frame_time: f64 = 0,
is_running: bool = true,

pub fn init(allocator: std.mem.Allocator, cpu: *CPU, bus: *Bus) !*@This() {
    try glfw.init();
    glfw.windowHint(.context_version_major, gl_version[0]);
    glfw.windowHint(.context_version_minor, gl_version[1]);
    glfw.windowHint(.opengl_profile, .opengl_core_profile);
    glfw.windowHint(.opengl_forward_compat, true);
    glfw.windowHint(.cocoa_retina_framebuffer, true);
    glfw.windowHint(.scale_framebuffer, false);
    glfw.windowHint(.client_api, .opengl_api);
    glfw.windowHint(.doublebuffer, true);

    const monitor = glfw.getPrimaryMonitor();
    const video_mode = try glfw.getVideoMode(monitor.?);

    const window = try glfw.Window.create(video_mode.width, video_mode.height, window_title, null);
    glfw.makeContextCurrent(window);
    glfw.swapInterval(1); // vsync

    try zopengl.loadCoreProfile(glfw.getProcAddress, gl_version[0], gl_version[1]);

    zgui.init(allocator);
    zgui.backend.init(window);

    const style = zgui.getStyle();
    style.window_rounding = 6.0;
    style.frame_rounding = 4.0;
    zgui.styleColorsDark(style);

    _ = zgui.io.addFontFromMemory(default_font, default_font_size);

    const tty_view = try TTYView.init(allocator);
    const cpu_view = try CPUView.init(allocator, cpu, bus);
    const gpu_view = try GPUView.init(allocator, bus.dev.gpu);
    const timer_view = try TimerView.init(allocator, bus.dev.timers);
    const assembly_view = try AssemblyView.init(allocator, cpu, bus);
    const vram_view = try VramView.init(allocator, bus.dev.gpu);

    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .bus = bus,
        .cpu_view = cpu_view,
        .gpu_view = gpu_view,
        .timer_view = timer_view,
        .assembly_view = assembly_view,
        .tty_view = tty_view,
        .vram_view = vram_view,
        .window = window,
    };

    return self;
}

pub fn deinit(self: *@This()) void {
    self.assembly_view.deinit();
    self.cpu_view.deinit();
    self.gpu_view.deinit();
    self.timer_view.deinit();
    self.tty_view.deinit();
    self.vram_view.deinit();

    zgui.backend.deinit();
    zgui.deinit();
    self.window.destroy();
    glfw.terminate();

    const allocator = self.allocator;
    allocator.destroy(self);
}

pub fn updatePaused(self: *@This()) void {
    const now = glfw.getTime();
    const elapsed = now - self.last_frame_time;

    if (elapsed < target_frame_time) {
        glfw.waitEventsTimeout(target_frame_time - elapsed);
        return;
    }

    self.updateInternal(now);
    self.handleInput();
}

pub fn update(self: *@This()) void {
    const now = glfw.getTime();

    if (self.next_frame_time > now) {
        const sleep_seconds = self.next_frame_time - now;
        std.Thread.sleep(@intFromFloat(sleep_seconds * std.time.ns_per_s));
    }

    self.updateInternal(now);
    self.handleInput();

    const after = glfw.getTime();
    self.next_frame_time = @max(self.next_frame_time + target_frame_time, after);
}

inline fn handleInput(self: *@This()) void {
    if (glfw.getKey(self.window, glfw.Key.escape) == .press) {
        glfw.setWindowShouldClose(self.window, true);
    }
    if (self.window.shouldClose()) {
        self.is_running = false;
    }
}

inline fn updateInternal(self: *@This(), now: f64) void {
    const fb_size = self.window.getFramebufferSize();
    zgui.backend.newFrame(@intCast(fb_size[0]), @intCast(fb_size[1]));
    glfw.pollEvents();

    gl.clearColor(0.1, 0.1, 0.1, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.flush();

    self.updateFrameRate(now);
    self.cpu_view.update();
    self.gpu_view.update();
    self.timer_view.update();
    self.assembly_view.update();
    self.tty_view.update();
    self.vram_view.update();

    zgui.backend.draw();
    self.window.swapBuffers();
    self.last_frame_time = now;
}

fn updateFrameRate(self: *@This(), now: f64) void {
    const delta = now - self.last_frame_time;
    const fps = 1.0 / delta;
    const cur_frame_time = delta * 1000.0;

    const flags = zgui.WindowFlags{
        .no_title_bar = true,
        .no_resize = true,
        .no_move = true,
        .no_scrollbar = true,
        .no_collapse = true,
        .no_background = true,
        .always_auto_resize = true,
    };

    zgui.setNextWindowPos(.{ .x = 10, .y = 10 });
    if (zgui.begin("Frame Rate", .{ .flags = flags })) {
        zgui.text("FPS: {:.2}", .{fps});
        zgui.text("Frame Time: {:.4} ms", .{cur_frame_time});
    }
    zgui.end();
}
