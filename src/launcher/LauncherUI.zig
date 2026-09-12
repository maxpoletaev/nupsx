const std = @import("std");
const zgui = @import("zgui");
const glfw = @import("zglfw");
const zopengl = @import("zopengl");

const imgui_fix = @import("../imgui_fix.zig");
const assets = @import("../assets/embed.zig");
const args_mod = @import("../args.zig");
const Config = @import("../config.zig").Config;
const host_paths = @import("../host_paths.zig");
const FileBrowser = @import("FileBrowser.zig");
const PathInput = FileBrowser.PathInput;

const Args = args_mod.Args;
const RendererBackend = args_mod.RendererBackend;

const default_font = assets.firacode_ttf;
const default_font_size = 18.0;
const window_title = "nuPSX";
const window_width = 600;
const window_height = 760;
const gl_version = .{ 4, 1 };
const gl = zopengl.bindings;

const content_padding = 24.0;
const hint_font_size = 14.0;
const browse_button_width = 80;
const browse_button_spacing = 10;

const bios_size = 512 * 1024;
const max_upscale = 4;
const upscale_combo_width = 80;
const renderer_combo_width = 120;

const header_height = 90.0;
const header_title_prefix = "nu";
const header_title_suffix = "PSX";
const header_subtitle = "PLAYSTATION EMULATOR";

const header_bg_color: [4]f32 = .{ 0.08, 0.13, 0.20, 1.0 };
const title_color: [4]f32 = .{ 0.94, 0.97, 1.0, 1.0 };
const accent_color: [4]f32 = .{ 0.35, 0.75, 1.0, 1.0 };
const subtitle_color: [4]f32 = .{ 0.65, 0.65, 0.7, 1.0 };
const error_color: [4]f32 = .{ 1.0, 0.3, 0.3, 1.0 };

allocator: std.mem.Allocator,
io: std.Io,
window: *glfw.Window,
browser: *FileBrowser,
config: *Config,

bios: PathInput = .{},
game: PathInput = .{},
memcard: PathInput = .{},

shader_enabled: bool = true,
debug: bool = false,
upscale: u32 = 1,
renderer: RendererBackend = .threaded,

error_message: ?[:0]const u8 = null,

var active_instance: ?*@This() = null;

pub fn init(allocator: std.mem.Allocator, io: std.Io) *@This() {
    glfw.init() catch @panic("GLFW");
    glfw.windowHint(.context_version_major, gl_version[0]);
    glfw.windowHint(.context_version_minor, gl_version[1]);
    glfw.windowHint(.opengl_profile, .opengl_core_profile);
    glfw.windowHint(.opengl_forward_compat, true);
    glfw.windowHint(.cocoa_retina_framebuffer, true);
    glfw.windowHint(.client_api, .opengl_api);
    glfw.windowHint(.doublebuffer, true);
    glfw.windowHint(.resizable, false);

    const window = glfw.createWindow(window_width, window_height, window_title, null, null) catch @panic("GLFW");
    glfw.makeContextCurrent(window);
    glfw.swapInterval(1);

    zopengl.loadCoreProfile(glfw.getProcAddress, gl_version[0], gl_version[1]) catch @panic("OpenGL");

    zgui.init(allocator);
    zgui.backend.init(window);

    const style = zgui.getStyle();
    style.window_rounding = 6.0;
    style.frame_rounding = 4.0;
    zgui.styleColorsDark(style);

    _ = zgui.io.addFontFromMemory(default_font, default_font_size);

    const browser = FileBrowser.init(allocator, io, host_paths.home_path);
    const config = Config.init(allocator, io, host_paths.config_path);

    const self = allocator.create(@This()) catch @panic("OOM");
    self.* = .{
        .allocator = allocator,
        .io = io,
        .window = window,
        .browser = browser,
        .config = config,
    };

    self.memcard.set(host_paths.default_memcard_path);
    self.loadConfig();

    std.debug.assert(active_instance == null);
    active_instance = self;

    _ = glfw.setDropCallback(window, dropCallback);

    return self;
}

pub fn deinit(self: *@This()) void {
    self.saveConfig();

    active_instance = null;
    self.browser.deinit();
    self.config.deinit();

    zgui.backend.deinit();
    zgui.deinit();
    self.window.destroy();
    glfw.terminate();

    self.allocator.destroy(self);
}

pub fn run(self: *@This()) ?Args {
    while (!self.window.shouldClose()) {
        glfw.pollEvents();

        if (!self.browser.is_open and glfw.getKey(self.window, glfw.Key.escape) == .press) {
            glfw.setWindowShouldClose(self.window, true);
        }

        imgui_fix.newFrame();

        gl.clearColor(0.12, 0.12, 0.14, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT);

        const start_pressed = self.update();

        zgui.backend.draw();
        self.window.swapBuffers();

        if (start_pressed) {
            const game = self.game.path();
            const is_exe = std.mem.endsWith(u8, game, ".exe");

            return .{
                .allocator = self.allocator,
                .bios_path = self.dupePath(self.bios.path()),
                .exe_path = self.dupePath(if (is_exe) game else ""),
                .cd_image_path = self.dupePath(if (is_exe) "" else game),
                .memcard_path = self.dupePath(self.memcard.path()),
                .no_shader = !self.shader_enabled,
                .debug = self.debug,
                .upscale = self.upscale,
                .renderer = self.renderer,
            };
        }
    }

    return null;
}

fn dupePath(self: *@This(), path: []const u8) []const u8 {
    return self.allocator.dupe(u8, path) catch @panic("OOM");
}

fn loadConfig(self: *@This()) void {
    if (self.config.get("bios")) |path| self.bios.set(path);
    if (self.config.get("game")) |path| self.game.set(path);
    if (self.config.get("memcard")) |path| self.memcard.set(path);
    if (self.config.get("current_dir")) |path| self.browser.setCurrentDir(path);
    if (self.config.getBool("shader_enabled")) |enabled| self.shader_enabled = enabled;
    if (self.config.getBool("debug")) |enabled| self.debug = enabled;
    if (self.config.get("upscale")) |value| {
        const scale = std.fmt.parseUnsigned(u32, value, 10) catch 1;
        if (scale >= 1 and scale <= max_upscale) self.upscale = scale;
    }
    if (self.config.get("renderer")) |value| {
        if (std.meta.stringToEnum(RendererBackend, value)) |backend| self.renderer = backend;
    }
}

fn saveConfig(self: *@This()) void {
    self.config.set("bios", self.bios.path());
    self.config.set("game", self.game.path());
    self.config.set("memcard", self.memcard.path());
    self.config.set("current_dir", self.browser.currentDir());
    self.config.setBool("shader_enabled", self.shader_enabled);
    self.config.setBool("debug", self.debug);
    var buf: [8]u8 = undefined;
    self.config.set("upscale", std.fmt.bufPrint(&buf, "{d}", .{self.upscale}) catch unreachable);
    self.config.set("renderer", @tagName(self.renderer));
    self.config.saveConfig();
}

fn isValidBios(self: *@This(), path: []const u8) bool {
    const file = std.Io.Dir.openFile(.cwd(), self.io, path, .{}) catch return false;
    defer file.close(self.io);
    const len = file.length(self.io) catch return false;
    return len == bios_size;
}

fn dropCallback(_: *glfw.Window, count: i32, paths: [*][*:0]const u8) callconv(.c) void {
    const self = active_instance orelse return;
    if (count <= 0) return;

    const path_z = paths[0];
    const path = std.mem.sliceTo(path_z, 0);

    // disk image or ps-exe
    if (std.mem.endsWith(u8, path, ".cue") or
        std.mem.endsWith(u8, path, ".exe"))
    {
        self.game.set(path);
        return;
    }

    // memory card
    if (std.mem.endsWith(u8, path, ".mcd")) {
        self.memcard.set(path);
        return;
    }

    // bios rom or possibly raw disc image
    if (std.mem.endsWith(u8, path, ".bin") or
        std.mem.endsWith(u8, path, ".rom"))
    {
        if (self.isValidBios(path)) {
            self.bios.set(path);
        } else {
            self.game.set(path);
        }
    }

    self.error_message = null;
}

fn update(self: *@This()) bool {
    const win_size = self.window.getSize();
    const win_w: f32 = @floatFromInt(win_size[0]);
    const win_h: f32 = @floatFromInt(win_size[1]);

    zgui.setNextWindowPos(.{ .x = 0, .y = 0 });
    zgui.setNextWindowSize(.{ .w = win_w, .h = win_h });

    const flags = zgui.WindowFlags{
        .no_title_bar = true,
        .no_resize = true,
        .no_move = true,
        .no_collapse = true,
        .no_bring_to_front_on_focus = true,
    };

    zgui.pushStyleVar1f(.{ .idx = .window_rounding, .v = 0 });
    zgui.pushStyleVar1f(.{ .idx = .window_border_size, .v = 0 });
    defer zgui.popStyleVar(.{ .count = 2 });

    var start_pressed = false;

    zgui.pushStyleVar2f(.{ .idx = .window_padding, .v = .{ content_padding, 12.0 } });
    const window_open = zgui.begin("Launcher", .{ .flags = flags });
    zgui.popStyleVar(.{ .count = 1 });

    if (window_open) {
        const content_w = zgui.getContentRegionAvail()[0];
        const input_w = content_w - browse_button_width - browse_button_spacing;

        drawHeader(win_w);

        drawHint("You can drag-and-drop ROMs directly onto this window");
        zgui.dummy(.{ .w = 0, .h = 4 });

        zgui.separator();
        zgui.dummy(.{ .w = 0, .h = 10 });

        // bios selection
        {
            zgui.text("BIOS File (Required):", .{});
            zgui.pushItemWidth(input_w);
            _ = zgui.inputText("##bios", .{ .buf = &self.bios.buf });
            zgui.popItemWidth();
            zgui.sameLine(.{ .spacing = browse_button_spacing });
            if (zgui.button("Browse##bios", .{ .w = browse_button_width, .h = 0 })) {
                self.browser.open(.bios, &self.bios);
            }
            drawHint("Select a 512 KB PS1 BIOS dump (e.g. SCPH1001.bin)");
            zgui.dummy(.{ .w = 0, .h = 12 });
        }

        // disk image selection
        {
            zgui.text("Disk Image or Executable:", .{});
            zgui.pushItemWidth(input_w);
            _ = zgui.inputText("##game", .{ .buf = &self.game.buf });
            zgui.popItemWidth();
            zgui.sameLine(.{ .spacing = browse_button_spacing });
            if (zgui.button("Browse##game", .{ .w = browse_button_width, .h = 0 })) {
                self.browser.open(.game, &self.game);
            }
            drawHint("Accepts .cue, .bin, or .exe (leave empty to boot into BIOS menu)");
            zgui.dummy(.{ .w = 0, .h = 12 });
        }

        // memory card
        {
            zgui.text("Memory Card Path:", .{});
            zgui.pushItemWidth(input_w);
            _ = zgui.inputText("##memcard", .{ .buf = &self.memcard.buf });
            zgui.popItemWidth();
            zgui.sameLine(.{ .spacing = browse_button_spacing });
            if (zgui.button("Browse##memcard", .{ .w = browse_button_width, .h = 0 })) {
                self.browser.open(.memcard, &self.memcard);
            }
            drawHint("Auto-created if not found");
            zgui.dummy(.{ .w = 0, .h = 15 });
        }

        zgui.separator();
        zgui.dummy(.{ .w = 0, .h = 10 });

        // internal resolution
        {
            zgui.alignTextToFramePadding();
            zgui.text("Internal Resolution:", .{});
            zgui.sameLine(.{ .spacing = 10 });
            var upscale_idx: i32 = @intCast(self.upscale - 1);
            zgui.pushItemWidth(upscale_combo_width);
            if (zgui.combo("##upscale", .{
                .current_item = &upscale_idx,
                .items_separated_by_zeros = "1x\x002x\x003x\x004x\x00",
            })) self.upscale = @intCast(upscale_idx + 1);
            zgui.popItemWidth();
            drawHint("Higher rendering quality at the cost of performance");
            zgui.dummy(.{ .w = 0, .h = 4 });
        }

        // renderer backend
        {
            zgui.alignTextToFramePadding();
            zgui.text("Renderer Backend:", .{});
            zgui.sameLine(.{ .spacing = 10 });
            var renderer_idx: i32 = @intFromEnum(self.renderer);
            zgui.pushItemWidth(renderer_combo_width);
            if (zgui.combo("##renderer", .{
                .current_item = &renderer_idx,
                .items_separated_by_zeros = "Software\x00Threaded\x00OpenGL\x00",
            })) self.renderer = @enumFromInt(renderer_idx);
            zgui.popItemWidth();
            drawHint("Threaded runs the software rasterizer on a separate thread");
            drawHint("OpenGL should be faster for upscaled res, but incomplete");
            zgui.dummy(.{ .w = 0, .h = 4 });
        }

        zgui.separator();
        zgui.dummy(.{ .w = 0, .h = 10 });

        // other options
        {
            _ = zgui.checkbox("Enable NTSC Shader Filter", .{ .v = &self.shader_enabled });
            drawHint("Simulates composite video artifacts");
            zgui.dummy(.{ .w = 0, .h = 4 });

            _ = zgui.checkbox("Launch with Debugger", .{ .v = &self.debug });
            drawHint("Opens the disassembly, CPU, VRAM, and register inspector views");
            zgui.dummy(.{ .w = 0, .h = 20 });
        }

        // error display
        if (self.error_message) |msg| {
            zgui.textColored(error_color, "{s}", .{msg});
            zgui.dummy(.{ .w = 0, .h = 5 });
        }

        // launch button
        if (zgui.button("Start Emulator", .{ .w = content_w, .h = 42 })) {
            start_pressed = self.validate();
        }

        // file browser modal (if open)
        if (self.browser.update(win_w, win_h)) {
            self.error_message = null;
        }
    }
    zgui.end();

    return start_pressed;
}

fn validate(self: *@This()) bool {
    const bios = self.bios.path();
    if (bios.len == 0) {
        self.error_message = "Error: BIOS file is required to start emulation";
        return false;
    } else if (!self.isValidBios(bios)) {
        self.error_message = "Error: Selected BIOS file is not found or not 512 KB";
        return false;
    }

    const game = self.game.path();
    if (game.len > 0) {
        const file = std.Io.Dir.openFile(.cwd(), self.io, game, .{}) catch {
            self.error_message = "Error: Game file could not be found";
            return false;
        };
        file.close(self.io);
    }

    return true;
}

fn rgba(c: [4]f32, alpha: f32) u32 {
    return zgui.colorConvertFloat4ToU32(.{ c[0], c[1], c[2], c[3] * alpha });
}

fn drawHeader(win_w: f32) void {
    const dl = zgui.getWindowDrawList();
    const origin = zgui.getWindowPos();
    const x0 = origin[0];
    const y0 = origin[1];
    const x1 = x0 + win_w;
    const y1 = y0 + header_height;

    dl.pushClipRect(.{ .pmin = .{ x0, y0 }, .pmax = .{ x1, y1 } });
    defer dl.popClipRect();

    dl.addRectFilled(.{
        .pmin = .{ x0, y0 },
        .pmax = .{ x1, y1 },
        .col = rgba(header_bg_color, 1.0),
    });

    zgui.pushFont(null, 48.0);
    const prefix_dim = zgui.calcTextSize(header_title_prefix, .{});
    const title_x = x0 + content_padding;
    const title_y = y0 + 18.0;
    dl.addTextUnformatted(.{ title_x, title_y }, rgba(title_color, 1.0), header_title_prefix);
    dl.addTextUnformatted(.{ title_x + prefix_dim[0], title_y }, rgba(accent_color, 1.0), header_title_suffix);
    zgui.popFont();

    zgui.pushFont(null, 13.0);
    dl.addTextUnformatted(
        .{ title_x + 3.0, title_y + prefix_dim[1] + 2.0 },
        rgba(subtitle_color, 0.9),
        header_subtitle,
    );
    zgui.popFont();
    zgui.setCursorPosY(header_height + 14.0);
}

fn drawHint(comptime txt: []const u8) void {
    zgui.pushFont(null, hint_font_size);
    zgui.textDisabled(txt, .{});
    zgui.popFont();
}
