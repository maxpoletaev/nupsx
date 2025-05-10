const std = @import("std");
const zgui = @import("zgui");
const glfw = @import("zglfw");
const zopengl = @import("zopengl");

const cpu = @import("cpu.zig");
const mem = @import("mem.zig");
const disasm = @import("disasm.zig");
const RegName = cpu.RegName;

const frame_time: f64 = 1.0 / 60.0;
const default_font = @embedFile("assets/freepixel.ttf");
const window_title = "nuPSX";
const gl_version = .{ 4, 1 };
const gl = zopengl.bindings;

const logger = std.log.scoped(.nupsx);

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
        const addr_range = mem.AddrRange.init(0xBFC00000, 0xFFFFF);
        const items_count = @as(i32, @intCast(addr_range.size / 4));

        const col1_offset = 300;
        const col2_offset = col1_offset + 300;

        const pc_addr = self.cpu.instr_addr;
        if (self.follow_pc) self.custom_addr = pc_addr;

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
                                    .col = 0x888888FF,
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

    fn drawGPRTab(self: *@This()) void {
        if (zgui.beginTabItem("GPR", .{})) {
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

            zgui.separatorText("GPR");
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
            zgui.endTabItem();
        }
    }

    pub fn update(self: *@This()) void {
        if (zgui.begin("CPU", .{})) {
            if (zgui.beginTabBar("", .{})) {
                self.drawGPRTab();
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
        glfw.swapInterval(1);

        try zopengl.loadCoreProfile(
            glfw.getProcAddress,
            gl_version[0],
            gl_version[1],
        );

        zgui.init(allocator);
        const scale_factor = window.getContentScale()[0];

        _ = zgui.io.addFontFromMemory(default_font, std.math.floor(16.0 * scale_factor));
        zgui.getStyle().scaleAllSizes(scale_factor);
        zgui.backend.init(window);

        const cpu_view = try CPUView.init(allocator, cpu_);
        const assembly_view = try AssemblyView.init(allocator, cpu_, bus, dasm);

        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .cpu_view = cpu_view,
            .assembly_view = assembly_view,
            .window = window,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.cpu_view.deinit();
        self.assembly_view.deinit();

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

    pub fn update(self: *@This()) void {
        const now = glfw.getTime();
        if ((now - self.last_update_time) < frame_time) {
            return;
        }

        const fb_size = self.window.getFramebufferSize();
        zgui.backend.newFrame(@intCast(fb_size[0]), @intCast(fb_size[1]));
        glfw.pollEvents();

        gl.clearColor(0.1, 0.1, 0.1, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT);
        gl.flush();

        self.cpu_view.update();
        self.assembly_view.update();

        zgui.backend.draw();
        self.window.swapBuffers();
        self.last_update_time = now;

        // Close on ESC
        if (glfw.getKey(self.window, glfw.Key.escape) == .press) {
            glfw.setWindowShouldClose(self.window, true);
        }
    }
};
