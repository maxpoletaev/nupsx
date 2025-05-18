const std = @import("std");
const zgui = @import("zgui");
const glfw = @import("zglfw");
const zopengl = @import("zopengl");

const gpu = @import("gpu.zig");
const cpu = @import("cpu.zig");
const mem = @import("mem.zig");
const disasm = @import("disasm.zig");
const RegName = cpu.RegName;

const default_font = @embedFile("assets/freepixel.ttf");
const default_font_size = 16.0;

const frame_time: f64 = 1.0 / 60.0;
const window_title = "nuPSX";
const gl_version = .{ 4, 1 };
const gl = zopengl.bindings;

const logger = std.log.scoped(.nupsx);

const addr_space_000 = mem.AddrRange.init(0x00000000, 0x800000);
const addr_space_800 = mem.AddrRange.init(0x80000000, 0x800000);
const addr_space_bfc = mem.AddrRange.init(0xbfc00000, 0x800000);

pub const VramView = struct {
    texture_id: gl.Uint,
    _vram: *gpu.Vram,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, vram: *gpu.Vram) !*@This() {
        var texture_id: gl.Uint = undefined;
        gl.genTextures(1, &texture_id);

        const self = try allocator.create(@This());
        self.* = .{
            .texture_id = texture_id,
            ._vram = vram,
            ._allocator = allocator,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        gl.deleteTextures(1, &self.texture_id);
        self._allocator.destroy(self);
    }

    pub fn update(self: *@This()) void {
        if (zgui.begin("VRAM", .{})) {
            gl.bindTexture(gl.TEXTURE_2D, self.texture_id);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
            gl.pixelStorei(gl.UNPACK_ROW_LENGTH, 0);

            gl.texImage2D(
                gl.TEXTURE_2D,
                0,
                gl.RGB5_A1,
                1024,
                512,
                0,
                gl.RGBA,
                gl.UNSIGNED_SHORT_1_5_5_5_REV,
                self._vram.buf,
            );

            const aspect_ratio = 1024.0 / 512.0; // 2:1 aspect ratio
            const available_width = zgui.getContentRegionAvail()[0];
            const available_height = zgui.getContentRegionAvail()[1];

            var display_width = available_width;
            var display_height = display_width / aspect_ratio;

            if (display_height > available_height) {
                display_height = available_height;
                display_width = display_height * aspect_ratio;
            }

            zgui.image(@ptrFromInt(self.texture_id), .{
                .w = display_width,
                .h = display_height,
            });
        }
        zgui.end();
    }
};

pub const TTYView = struct {
    lines: std.ArrayList([]u8),
    allocator: std.mem.Allocator,
    line_buf: std.ArrayList(u8),
    new_line_added: bool,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .lines = .init(allocator),
            .line_buf = .init(allocator),
            .new_line_added = false,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        for (self.lines.items) |line| {
            self.allocator.free(line);
        }

        self.lines.deinit();
        self.line_buf.deinit();
        self.allocator.destroy(self);
    }

    pub fn writeLine(self: *@This(), line: []const u8) !void {
        const buf = try self.allocator.alloc(u8, line.len);
        @memcpy(buf, line);

        try self.lines.append(buf);
        self.new_line_added = true;
    }

    pub fn writeChar(self: *@This(), char: u8) !void {
        try self.line_buf.append(char);
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
    tmp_buf: std.ArrayList(u8),
    lc: zgui.ListClipper,
    dasm: *disasm.Disasm,
    bus: *mem.Bus,
    cpu: *cpu.CPU,

    follow_pc: bool,
    custom_addr: u32,

    pub fn init(allocator: std.mem.Allocator, cpu_: *cpu.CPU, bus: *mem.Bus, dasm: *disasm.Disasm) !*@This() {
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

        const col1_offset = 300;
        const col2_offset = col1_offset + 300;

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
                        const instr = cpu.Instr{ .code = self.bus.readWord(addr) };

                        self.tmp_buf.clearRetainingCapacity();
                        self.dasm.disassembleToBuffer(instr, &self.tmp_buf) catch unreachable;
                        const instr_text = self.tmp_buf.items;

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
                    const item_pos_y = self.lc.StartPosY + self.lc.ItemsHeight * @as(f32, @floatFromInt(item_idx));
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
    cpu: *cpu.CPU,

    pub fn init(allocator: std.mem.Allocator, cpu_: *cpu.CPU) !*@This() {
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

            const exc_code = @as(cpu.ExcCode, @enumFromInt(self.cpu.cop0.cause().exc_code));
            const exc_tag = std.enums.tagName(cpu.ExcCode, exc_code);

            _ = zgui.text("in exception: {d}", .{@intFromBool(self.cpu.in_exception)});
            _ = zgui.text("exc code: {s}", .{if (exc_tag) |t| t else "-"});
        }
    }

    fn drawGprTab(self: *@This()) void {
        if (zgui.beginTabItem("GPR", .{})) {
            defer zgui.endTabItem();

            zgui.pushItemWidth(300.0);

            for (self.cpu.gpr, 0..) |_, i| {
                const reg_name = cpu.RegName[i];
                _ = zgui.inputInt(reg_name, .{
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

pub const UI = struct {
    allocator: std.mem.Allocator,
    window: *glfw.Window,
    cpu_view: *CPUView,
    assembly_view: *AssemblyView,
    vram_view: *VramView,
    tty_view: *TTYView,
    last_update_time: f64 = 0,

    pub fn init(allocator: std.mem.Allocator, cpu_: *cpu.CPU, bus: *mem.Bus, dasm: *disasm.Disasm) !*@This() {
        try glfw.init();
        glfw.windowHint(.context_version_major, gl_version[0]);
        glfw.windowHint(.context_version_minor, gl_version[1]);
        glfw.windowHint(.opengl_profile, .opengl_core_profile);
        glfw.windowHint(.opengl_forward_compat, true);
        glfw.windowHint(.client_api, .opengl_api);
        glfw.windowHint(.doublebuffer, true);

        const monitor = glfw.getPrimaryMonitor();
        const video_mode = try glfw.getVideoMode(monitor.?);

        const window = try glfw.Window.create(
            video_mode.width,
            video_mode.height,
            window_title,
            null,
        );
        glfw.makeContextCurrent(window);
        glfw.swapInterval(0);

        try zopengl.loadCoreProfile(
            glfw.getProcAddress,
            gl_version[0],
            gl_version[1],
        );

        zgui.init(allocator);
        const scale_factor = window.getContentScale()[0];

        const font_size = std.math.floor(default_font_size * scale_factor);
        _ = zgui.io.addFontFromMemory(default_font, font_size);
        zgui.getStyle().scaleAllSizes(scale_factor);
        zgui.backend.init(window);

        const tty_view = try TTYView.init(allocator);
        const cpu_view = try CPUView.init(allocator, cpu_);
        const assembly_view = try AssemblyView.init(allocator, cpu_, bus, dasm);
        const vram_view = try VramView.init(allocator, &bus._gpu.vram);

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .cpu_view = cpu_view,
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
        self.tty_view.deinit();
        self.vram_view.deinit();

        zgui.backend.deinit();
        zgui.deinit();
        self.window.destroy();
        glfw.terminate();

        const allocator = self.allocator;
        allocator.destroy(self);
    }

    pub fn shouldClose(self: *@This()) bool {
        return self.window.shouldClose();
    }

    pub inline fn update(self: *@This()) void {
        const now = glfw.getTime();
        if ((now - self.last_update_time) < frame_time) {
            return;
        }

        self.updateInternal(now);
    }

    fn updateInternal(self: *@This(), now: f64) void {
        const fb_size = self.window.getFramebufferSize();
        zgui.backend.newFrame(@intCast(fb_size[0]), @intCast(fb_size[1]));
        glfw.pollEvents();

        gl.clearColor(0.1, 0.1, 0.1, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT);
        gl.flush();

        self.cpu_view.update();
        self.assembly_view.update();
        self.tty_view.update();
        self.vram_view.update();

        zgui.backend.draw();
        self.window.swapBuffers();
        self.last_update_time = now;

        // Close on ESC
        if (glfw.getKey(self.window, glfw.Key.escape) == .press) {
            glfw.setWindowShouldClose(self.window, true);
        }
    }
};
