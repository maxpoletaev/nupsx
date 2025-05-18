const std = @import("std");
const mem = @import("mem.zig");

const log = std.log.scoped(.gpu);

pub const GP0Command = enum {
    noop,
    cpu_to_vram,
    vram_to_cpu,
    draw_area_top_left,
    draw_area_bottom_right,
    draw_offset,
    draw_mode,
    texture_window,
    bit_mask,
    rectangle_1x1,

    pub fn decode(v: u32) GP0Command {
        const cmd = @as(u8, @truncate(v >> 24));
        return switch (cmd) {
            0xa0 => .cpu_to_vram,
            0xc0 => .vram_to_cpu,
            0xe3 => .draw_area_top_left,
            0xe4 => .draw_area_bottom_right,
            0xe5 => .draw_offset,
            0xe1 => .draw_mode,
            0xe2 => .texture_window,
            0xe6 => .bit_mask,
            0x68 => .rectangle_1x1,
            else => .noop,
        };
    }
};

const GP1Command = enum {
    noop,
    reset,
    clear_fifo,
    register_read,

    pub fn decode(v: u32) GP1Command {
        const cmd = @as(u8, @truncate(v >> 24));
        return switch (cmd) {
            0x00 => .reset,
            0x01 => .clear_fifo,
            else => .noop,
        };
    }
};

const Color = packed struct {
    r: u5,
    g: u5,
    b: u5,
    a: u1,

    pub inline fn fromBGR24(v: u32) Color {
        return .{
            .r = @truncate(((v >> 0) & 0xff) >> 3),
            .g = @truncate(((v >> 8) & 0xff) >> 3),
            .b = @truncate(((v >> 16) & 0xff) >> 3),
            .a = 1,
        };
    }
};

pub const Vram = struct {
    const yres = 512;
    const xres = 1024;
    const size = 0x100000;

    buf: *align(16) [size]u8,
    pixels: *align(16) [xres * yres]Color,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        const buf = try allocator.alignedAlloc(u8, 16, size);
        const pixels = std.mem.bytesAsSlice(Color, buf);
        return .{
            .buf = buf[0..size],
            .pixels = pixels[0 .. xres * yres],
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.destroy(self.buf);
    }

    pub inline fn drawPixel(self: *@This(), x: u32, y: u32, c: Color) void {
        self.pixels[y * xres + x] = c;
    }
};

const CommandBuffer = struct {
    buf: [16]u32,
    len: u32,

    fn add(self: *CommandBuffer, v: u32) void {
        self.buf[self.len] = v;
        self.len += 1;
    }
};

pub const GPU = struct {
    const State = enum {
        recv_command,
        recv_args,
        recv_data,
        send_data,
    };

    vram: Vram,
    gp0_state: State,
    gp0_fifo: CommandBuffer,
    gp0_command: ?GP0Command,
    gp0_xcur: u16,
    gp0_ycur: u16,
    gpuread_v: u32,
    draw_area_start: [2]u16,
    draw_area_end: [2]u16,
    draw_area_offset: [2]u16,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .vram = try .init(allocator),
            .gp0_fifo = std.mem.zeroes(CommandBuffer),
            .gp0_state = .recv_command,
            .gp0_command = null,
            .gp0_xcur = 0,
            .gp0_ycur = 0,
            .gpuread_v = 0,
            .draw_area_start = .{ 0, 0 },
            .draw_area_end = .{ 0, 0 },
            .draw_area_offset = .{ 0, 0 },
            ._allocator = allocator,
        };

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.vram.deinit();
        self._allocator.destroy(self);
    }

    pub fn readWord(self: *@This(), addr: u32) u32 {
        switch (addr) {
            0x1f801810 => { // GPUREAD
                if (self.gp0_state == .send_data) {
                    self.stepCommandState(0);
                }
                const v = self.gpuread_v;
                log.debug("gpuread: {x}", .{v});
                self.gpuread_v = 0;
                return v;
            },
            0x1f801814 => return 0xffffffff, // GPUSTAT
            else => return 0,
        }
        return 0;
    }

    // -------------------------
    // GP0 Commands
    // -------------------------

    pub fn gp0write(self: *@This(), v: u32) void {
        log.debug("gp0 - write: {x}", .{v});

        if (self.gp0_state == .recv_command) {
            self.gp0_command = GP0Command.decode(v);
            self.gp0_fifo.len = 0;
        }

        self.stepCommandState(v);
    }

    fn stepCommandState(self: *@This(), v: u32) void {
        switch (self.gp0_command.?) {
            .noop => {},
            .cpu_to_vram => self.cpuToVram(v),
            .vram_to_cpu => self.vramToCpu(v),
            .draw_area_top_left => {},
            .draw_area_bottom_right => {},
            .draw_offset => {},
            .draw_mode => {},
            .texture_window => {},
            .bit_mask => {},
            .rectangle_1x1 => self.drawRectangle1x1(v),
        }
    }

    fn cpuToVram(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 3) {
                    self.gp0_state = .recv_data;
                    self.gp0_ycur = 0;
                    self.gp0_xcur = 0;
                }
            },
            .recv_data => {
                const xpos = @as(u16, @truncate(self.gp0_fifo.buf[1] >> 0));
                const ypos = @as(u16, @truncate(self.gp0_fifo.buf[1] >> 16));
                const xsiz = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 0));
                const ysiz = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 16));

                inline for (0..2) |i| {
                    const shift = @as(u5, @truncate(i * 16));
                    const hw = @as(u16, @truncate(v >> shift));
                    const x = @as(u32, (xpos + self.gp0_xcur) & 0x1FF);
                    const y = @as(u32, (ypos + self.gp0_ycur) & 0x3FF);

                    const addr = 2 * (1024 * y + x);
                    mem.write(u16, self.vram.buf, addr, hw);

                    self.gp0_xcur += 1;
                    if (self.gp0_xcur == xsiz) {
                        self.gp0_xcur = 0;
                        self.gp0_ycur += 1;
                        if (self.gp0_ycur == ysiz) {
                            self.gp0_state = .recv_command;
                            self.gp0_command = null;
                        }
                    }
                }
            },
            else => unreachable,
        }
    }

    fn vramToCpu(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 3) {
                    self.gp0_state = .send_data;
                    self.gp0_ycur = 0;
                    self.gp0_xcur = 0;
                }
            },
            .send_data => {
                self.gpuread_v = 0;

                const xpos = @as(u16, @truncate(self.gp0_fifo.buf[1] >> 0));
                const ypos = @as(u16, @truncate(self.gp0_fifo.buf[1] >> 16));
                const xsiz = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 0));
                const ysiz = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 16));

                inline for (0..2) |i| {
                    const shift = @as(u5, @truncate(i * 16));
                    const x = @as(u32, (xpos + self.gp0_xcur) & 0x1FF);
                    const y = @as(u32, (ypos + self.gp0_ycur) & 0x3FF);

                    const addr = 2 * (1024 * y + x);
                    const hw = mem.read(u16, self.vram.buf, addr);
                    self.gpuread_v |= @as(u32, hw) << shift;

                    self.gp0_xcur += 1;
                    if (self.gp0_xcur == xsiz) {
                        self.gp0_xcur = 0;
                        self.gp0_ycur += 1;
                        if (self.gp0_ycur == ysiz) {
                            self.gp0_state = .recv_command;
                            self.gp0_command = null;
                        }
                    }
                }
            },
            else => unreachable,
        }
    }

    fn drawRectangle1x1(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 2) {
                    const color = self.gp0_fifo.buf[0] & 0xffffff;
                    const x = ((self.gp0_fifo.buf[1] >> 0) & 0xffff) + self.draw_area_offset[0];
                    const y = ((self.gp0_fifo.buf[1] >> 16) & 0xffff) + self.draw_area_offset[1];
                    self.vram.drawPixel(x, y, Color.fromBGR24(color));
                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    // -------------------------
    // GP1 Commands
    // -------------------------

    pub fn gp1write(self: *@This(), v: u32) void {
        log.debug("gp1 - write: {x}", .{v});
        switch (GP1Command.decode(v)) {
            .noop => {},
            .reset => {},
            .clear_fifo => self.clearFifo(v),
            .register_read => self.registerRead(v),
        }
    }

    fn clearFifo(self: *@This(), _: u32) void {
        self.gp0_state = .recv_command;
        self.gp0_fifo.len = 0;
    }

    fn registerRead(self: *@This(), v: u32) void {
        const reg = @as(u8, @truncate(v));
        self.gpuread_v = @as(u32, switch (reg) {
            7 => 0x00000000,
            else => 0,
        });
    }
};
