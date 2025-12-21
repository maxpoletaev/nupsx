const std = @import("std");
const zgui = @import("zgui");
const glfw = @import("zglfw");
const zopengl = @import("zopengl");

const mem = @import("mem.zig");
const gpu_mod = @import("gpu.zig");
const cpu_mod = @import("cpu.zig");
const Disasm = @import("Disasm.zig");

const Bus = mem.Bus;
const CPU = cpu_mod.CPU;
const GPU = gpu_mod.GPU;

const default_font = @embedFile("assets/freepixel.ttf");
const default_font_size = 16.0;

const frame_time: f64 = 1.0 / 60.0;
const window_title = "nuPSX";
const gl_version = .{ 4, 1 };
const gl = zopengl.bindings;

const logger = std.log.scoped(.nupsx);

const addr_space_000 = mem.AddrRange.init(.none, 0x00000000, 0x800000);
const addr_space_800 = mem.AddrRange.init(.none, 0x80000000, 0x800000);
const addr_space_bfc = mem.AddrRange.init(.none, 0xbfc00000, 0x800000);

pub const VramView = struct {
    pub const HighlightRect = struct {
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        color: u32,
        label: []const u8,
        enabled: bool,
    };

    allocator: std.mem.Allocator,
    texture_id: gl.Uint,
    gpu: *GPU,
    show_draw_area: bool,
    show_display_area: bool,

    pub fn init(allocator: std.mem.Allocator, gpu: *GPU) !*@This() {
        var texture_id: gl.Uint = undefined;
        gl.genTextures(1, &texture_id);

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .texture_id = texture_id,
            .gpu = gpu,
            .show_draw_area = false,
            .show_display_area = false,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        gl.deleteTextures(1, &self.texture_id);
        self.allocator.destroy(self);
    }

    pub fn update(self: *@This()) void {
        if (zgui.begin("VRAM", .{})) {
            _ = zgui.checkbox("Show Draw Area", .{ .v = &self.show_draw_area });
            zgui.sameLine(.{});
            _ = zgui.checkbox("Show Display Area", .{ .v = &self.show_display_area });

            gl.bindTexture(gl.TEXTURE_2D, self.texture_id);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.pixelStorei(gl.UNPACK_ROW_LENGTH, 0);
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB5, 1024, 512, 0, gl.RGBA, gl.UNSIGNED_SHORT_1_5_5_5_REV, self.gpu.vram);

            const aspect_ratio = 1024.0 / 512.0; // 2:1 aspect ratio
            const available_width = zgui.getContentRegionAvail()[0];
            const available_height = zgui.getContentRegionAvail()[1];

            var display_width = available_width;
            var display_height = display_width / aspect_ratio;

            if (display_height > available_height) {
                display_height = available_height;
                display_width = display_height * aspect_ratio;
            }

            const cursor_pos = zgui.getCursorScreenPos();
            const dl = zgui.getWindowDrawList();

            const textureRef = zgui.TextureRef{
                .tex_id = @enumFromInt(self.texture_id),
                .tex_data = null,
            };

            zgui.image(textureRef, .{
                .w = display_width,
                .h = display_height,
            });

            const draw_w = self.gpu.gp0_draw_area_end.x -| self.gpu.gp0_draw_area_start.x;
            const draw_h = self.gpu.gp0_draw_area_end.y -| self.gpu.gp0_draw_area_start.y;
            const gpu_display_res = self.gpu.getDisplayRes();

            const highlights = [_]HighlightRect{
                .{
                    .x = self.gpu.gp0_draw_area_start.x,
                    .y = self.gpu.gp0_draw_area_start.y,
                    .w = draw_w,
                    .h = draw_h,
                    .color = 0xff00ff00,
                    .label = "Draw",
                    .enabled = self.show_draw_area,
                },
                .{
                    .x = self.gpu.gp1_display_area_start.x,
                    .y = self.gpu.gp1_display_area_start.y,
                    .w = gpu_display_res[0],
                    .h = gpu_display_res[1],
                    .color = 0xff0000ff,
                    .label = "Display",
                    .enabled = self.show_display_area,
                },
            };

            const scale_x = display_width / 1024.0;
            const scale_y = display_height / 512.0;

            for (highlights) |rect| {
                if (rect.enabled and rect.w > 0 and rect.h > 0) {
                    const x1 = cursor_pos[0] + @as(f32, @floatFromInt(rect.x)) * scale_x;
                    const y1 = cursor_pos[1] + @as(f32, @floatFromInt(rect.y)) * scale_y;
                    const x2 = x1 + @as(f32, @floatFromInt(rect.w)) * scale_x;
                    const y2 = y1 + @as(f32, @floatFromInt(rect.h)) * scale_y;

                    dl.addRect(.{ .pmin = .{ x1, y1 }, .pmax = .{ x2, y2 }, .col = rect.color, .thickness = 2.0 });
                    dl.addText(.{ x1 + 4, y1 + 4 }, rect.color, "{s}", .{rect.label});
                }
            }
        }
        zgui.end();
    }
};

pub const TTYView = struct {
    allocator: std.mem.Allocator,
    lines: std.array_list.Aligned([]u8, null),
    line_buf: std.array_list.Aligned(u8, null),
    new_line_added: bool,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .lines = .empty,
            .line_buf = .empty,
            .new_line_added = false,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        for (self.lines.items) |line| {
            self.allocator.free(line);
        }

        self.line_buf.deinit(self.allocator);
        self.lines.deinit(self.allocator);
        self.allocator.destroy(self);
    }

    pub fn writeLine(self: *@This(), line: []const u8) !void {
        const buf = try self.allocator.alloc(u8, line.len);
        @memcpy(buf, line);

        try self.lines.append(self.allocator, buf);
        self.new_line_added = true;
    }

    pub fn writeChar(self: *@This(), char: u8) !void {
        try self.line_buf.append(self.allocator, char);
        if (char == '\n') {
            try self.writeLine(self.line_buf.items);
            self.line_buf.clearRetainingCapacity();
        }
    }

    pub fn update(self: *@This()) void {
        if (zgui.begin("TTY", .{})) {
            for (self.lines.items) |line| {
                zgui.textUnformatted(line);
            }
        }

        if (self.new_line_added) {
            zgui.setScrollHereY(.{});
            self.new_line_added = false;
        }

        zgui.end();
    }
};

const AssemblyView = struct {
    allocator: std.mem.Allocator,
    tmp_buf: std.Io.Writer.Allocating,
    lc: zgui.ListClipper,
    dasm: *Disasm,
    bus: *Bus,
    cpu: *CPU,

    follow_pc: bool,
    custom_addr: u32,

    pub fn init(allocator: std.mem.Allocator, cpu_: *CPU, bus: *Bus, dasm: *Disasm) !*@This() {
        const list_clipper = zgui.ListClipper.init();

        const self = try allocator.create(@This());
        self.* = .{
            .tmp_buf = .init(allocator),
            .allocator = allocator,
            .lc = list_clipper,
            .dasm = dasm,
            .cpu = cpu_,
            .bus = bus,
            .follow_pc = false,
            .custom_addr = 0,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.tmp_buf.deinit();
        self.allocator.destroy(self);
    }

    pub fn update(self: *@This()) void {
        const pc_addr = self.cpu.instr_addr;
        if (self.follow_pc) self.custom_addr = pc_addr;

        const addr_range = if (addr_space_000.match(pc_addr)) addr_space_000 //
            else if (addr_space_800.match(pc_addr)) addr_space_800 //
            else addr_space_bfc;

        const col1_offset = 150;
        const col2_offset = col1_offset + 150;

        const items_count = @as(i32, @intCast(addr_range.size / 4));

        if (zgui.begin("Assembly", .{})) {
            _ = zgui.checkbox("[F]ollow PC", .{ .v = &self.follow_pc });
            zgui.sameLine(.{});

            _ = zgui.setNextItemWidth(300.0);
            _ = zgui.inputInt("##", .{
                .v = @ptrCast(&self.custom_addr),
                .step = 0,
                .flags = .{
                    .read_only = self.follow_pc,
                    .chars_hexadecimal = true,
                },
            });

            // Hotkeys
            if (zgui.isWindowFocused(.{})) {
                if (zgui.isKeyPressed(zgui.Key.f, false)) {
                    self.follow_pc = !self.follow_pc;
                }

                if (zgui.isKeyPressed(zgui.Key.space, false)) {
                    if (self.cpu.stall) self.cpu.step();
                }
            }

            // Scrollable area
            if (zgui.beginChild("scroll", .{})) {
                defer zgui.endChild();

                self.lc.begin(items_count, zgui.getTextLineHeightWithSpacing());
                defer self.lc.end();

                while (self.lc.step()) {
                    const start = @as(usize, @intCast(self.lc.DisplayStart));
                    const end = @as(usize, @intCast(self.lc.DisplayEnd));

                    for (start..end) |i| {
                        const addr = @as(u32, @truncate(addr_range.start + i * 4));
                        const instr = cpu_mod.Instr{ .code = self.bus.readWord(addr) };

                        self.tmp_buf.clearRetainingCapacity();
                        self.dasm.disassembleToBuffer(instr, &self.tmp_buf.writer) catch unreachable;
                        const instr_text = self.tmp_buf.written();

                        {
                            if (pc_addr == addr) {
                                const dl = zgui.getWindowDrawList();

                                const cursor_screen_pos = zgui.getCursorScreenPos();
                                const fill_width = zgui.getContentRegionAvail()[0];

                                dl.addRectFilled(.{
                                    .col = 0x888888ff,
                                    .pmin = cursor_screen_pos,
                                    .pmax = .{ cursor_screen_pos[0] + fill_width, cursor_screen_pos[1] + self.lc.ItemsHeight },
                                });
                            }

                            const cursor = zgui.getCursorPos();

                            zgui.text("{x}", .{addr});
                            zgui.sameLine(.{});

                            zgui.setCursorPosX(cursor[0] + col1_offset);
                            zgui.text("{x}", .{instr.code});
                            zgui.sameLine(.{});

                            zgui.setCursorPosX(cursor[0] + col2_offset);
                            zgui.textUnformatted(instr_text);
                        }
                    }
                }

                if (self.follow_pc and addr_range.match(self.custom_addr)) {
                    const item_idx = @divTrunc((self.custom_addr - addr_range.start), 4);
                    const item_pos_y = @as(f32, @floatCast(self.lc.StartPosY)) + self.lc.ItemsHeight * @as(f32, @floatFromInt(item_idx));
                    zgui.setScrollFromPosY(.{ .local_y = item_pos_y - zgui.getWindowPos()[1] });
                }
            }
        }
        zgui.end();
    }
};

const CPUView = struct {
    const hex_field_flags = zgui.InputTextFlags{
        .chars_hexadecimal = true,
        .chars_uppercase = true,
    };

    const hex_field_flags_ro = zgui.InputTextFlags{
        .chars_hexadecimal = true,
        .chars_uppercase = true,
        .read_only = true,
    };

    allocator: std.mem.Allocator,
    cpu: *CPU,

    pub fn init(allocator: std.mem.Allocator, cpu_: *CPU) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .cpu = cpu_,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        const allocator = self.allocator;
        allocator.destroy(self);
    }

    fn drawCop0Tab(self: *@This()) void {
        if (zgui.beginTabItem("COP0", .{})) {
            defer zgui.endTabItem();

            const exc_code = @as(cpu_mod.Cop0.ExcCode, self.cpu.cop0.cause().exc_code);
            const exc_tag = std.enums.tagName(cpu_mod.Cop0.ExcCode, exc_code);

            _ = zgui.text("in exception: {d}", .{@intFromBool(self.cpu.in_exception)});
            _ = zgui.text("exc code: {s}", .{if (exc_tag) |t| t else "-"});
        }
    }

    fn drawGprTab(self: *@This()) void {
        if (zgui.beginTabItem("GPR", .{})) {
            defer zgui.endTabItem();

            zgui.pushItemWidth(300.0);

            for (self.cpu.gpr, 0..) |_, i| {
                _ = zgui.inputInt(cpu_mod.RegName[i], .{
                    .step = 0,
                    .flags = hex_field_flags,
                    .v = @ptrCast(&self.cpu.gpr[i]),
                });
            }

            zgui.popItemWidth();
        }
    }

    pub fn update(self: *@This()) void {
        if (zgui.begin("CPU", .{})) {
            _ = zgui.checkbox("Stall", .{
                .v = &self.cpu.stall,
            });

            if (self.cpu.stall) {
                zgui.sameLine(.{});
                if (zgui.button("Step", .{})) {
                    self.cpu.step();
                }
            }

            _ = zgui.inputInt("PC", .{
                .step = 4,
                .flags = hex_field_flags,
                .v = @ptrCast(&self.cpu.pc),
            });

            _ = zgui.inputInt("Next PC", .{
                .step = 4,
                .flags = hex_field_flags,
                .v = @ptrCast(&self.cpu.next_pc),
            });

            _ = zgui.inputInt("Instr", .{
                .step = 0,
                .flags = .{
                    .chars_hexadecimal = true,
                    .chars_uppercase = true,
                    .read_only = true,
                },
                .v = @ptrCast(&self.cpu.instr.code),
            });

            if (zgui.beginTabBar("##", .{})) {
                self.drawGprTab();
                self.drawCop0Tab();
            }
            zgui.endTabBar();
        }

        zgui.end();
    }
};

const GPUView = struct {
    allocator: std.mem.Allocator,
    gpu: *GPU,

    pub fn init(allocator: std.mem.Allocator, gpu: *GPU) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .gpu = gpu,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        const allocator = self.allocator;
        allocator.destroy(self);
    }

    pub fn update(self: *@This()) void {
        if (zgui.begin("GPU", .{})) {
            // FIFO State
            if (zgui.collapsingHeader("FIFO", .{})) {
                zgui.text("State: {s}", .{@tagName(self.gpu.gp0_state)});
                zgui.text("Command: 0x{x:0>2}", .{self.gpu.gp0_cmd});
                zgui.text("FIFO Length: {d}", .{self.gpu.gp0_fifo.len});

                if (self.gpu.gp0_fifo.len > 0) {
                    zgui.text("FIFO Contents:", .{});
                    for (0..self.gpu.gp0_fifo.len) |i| {
                        zgui.text("  [{d}] 0x{x:0>8}", .{ i, self.gpu.gp0_fifo.buf[i] });
                    }
                }
            }

            // Draw Area
            if (zgui.collapsingHeader("Draw Area", .{ .default_open = true })) {
                const draw_w = self.gpu.gp0_draw_area_end.x -| self.gpu.gp0_draw_area_start.x;
                const draw_h = self.gpu.gp0_draw_area_end.y -| self.gpu.gp0_draw_area_start.y;

                zgui.text("Start: ({d}, {d})", .{ self.gpu.gp0_draw_area_start.x, self.gpu.gp0_draw_area_start.y });
                zgui.text("End: ({d}, {d})", .{ self.gpu.gp0_draw_area_end.x, self.gpu.gp0_draw_area_end.y });
                zgui.text("Offset: ({d}, {d})", .{ self.gpu.gp0_draw_offset.x, self.gpu.gp0_draw_offset.y });
                zgui.text("Size: {d}x{d}", .{ draw_w, draw_h });
            }

            // Display Area
            if (zgui.collapsingHeader("Display Area", .{ .default_open = true })) {
                const display_res = self.gpu.getDisplayRes();
                zgui.text("Display Enabled: {s}", .{if (self.gpu.gp1_display_enable) "yes" else "no"});
                zgui.text("Start: ({d}, {d})", .{ self.gpu.gp1_display_area_start.x, self.gpu.gp1_display_area_start.y });
                zgui.text("Resolution: {d}x{d}", .{ display_res[0], display_res[1] });
            }

            // Texture Window
            if (zgui.collapsingHeader("Texture Window", .{})) {
                zgui.text("Mask: ({d}, {d})", .{ self.gpu.gp0_texwin_mask[0], self.gpu.gp0_texwin_mask[1] });
                zgui.text("Offset: ({d}, {d})", .{ self.gpu.gp0_texwin_offset[0], self.gpu.gp0_texwin_offset[1] });
            }

            // Status
            if (zgui.collapsingHeader("Status", .{})) {
                zgui.text("DMA Direction: {s}", .{@tagName(self.gpu.gp1_dma_direction)});
                zgui.text("GPUSTAT: 0x{x:0>8}", .{self.gpu.readGpustat()});
                zgui.text("GPUREAD: 0x{x:0>8}", .{self.gpu.gpuread});
            }
        }
        zgui.end();
    }
};

pub const DebugUI = struct {
    allocator: std.mem.Allocator,
    window: *glfw.Window,
    cpu_view: *CPUView,
    gpu_view: *GPUView,
    assembly_view: *AssemblyView,
    vram_view: *VramView,
    tty_view: *TTYView,
    last_update_time: f64 = 0,
    is_running: bool = true,

    pub fn init(allocator: std.mem.Allocator, cpu: *CPU, bus: *Bus, dasm: *Disasm) !*@This() {
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
        glfw.swapInterval(0);

        try zopengl.loadCoreProfile(glfw.getProcAddress, gl_version[0], gl_version[1]);

        zgui.init(allocator);
        zgui.backend.init(window);

        const style = zgui.getStyle();
        style.window_rounding = 6.0;
        style.frame_rounding = 4.0;
        zgui.styleColorsDark(style);

        _ = zgui.io.addFontFromMemory(default_font, default_font_size);

        const tty_view = try TTYView.init(allocator);
        const cpu_view = try CPUView.init(allocator, cpu);
        const gpu_view = try GPUView.init(allocator, bus.dev.gpu);
        const assembly_view = try AssemblyView.init(allocator, cpu, bus, dasm);
        const vram_view = try VramView.init(allocator, bus.dev.gpu);

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .cpu_view = cpu_view,
            .gpu_view = gpu_view,
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
        self.tty_view.deinit();
        self.vram_view.deinit();

        zgui.backend.deinit();
        zgui.deinit();
        self.window.destroy();
        glfw.terminate();

        const allocator = self.allocator;
        allocator.destroy(self);
    }

    pub fn update(self: *@This()) void {
        const now = glfw.getTime();
        if ((now - self.last_update_time) < frame_time) {
            return;
        }
        self.updateInternal(now);
        self.handleInput();
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
        self.assembly_view.update();
        self.tty_view.update();
        self.vram_view.update();

        zgui.backend.draw();
        self.window.swapBuffers();
        self.last_update_time = now;
    }

    fn updateFrameRate(self: *@This(), now: f64) void {
        const delta = now - self.last_update_time;
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
};
