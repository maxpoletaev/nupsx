const std = @import("std");
const mem = @import("mem.zig");
const builtin = @import("builtin");
const rasterizer = @import("rasterizer.zig");
const bits = @import("bits.zig");

const Rasterizer = rasterizer.Rasterizer;
const Color15 = rasterizer.Color15;
const Color24 = rasterizer.Color24;
const Vertex = rasterizer.Vertex;
const ColorDepth = rasterizer.ColorDepth;

const log = std.log.scoped(.gpu);

const Fifo = struct {
    buf: [16]u32,
    len: u32,

    inline fn add(self: *@This(), v: u32) void {
        self.buf[self.len] = v;
        self.len += 1;
    }

    inline fn reset(self: *@This()) void {
        self.len = 0;
    }
};

const DisplayMode = packed struct(u32) {
    hres: enum(u2) { r256 = 0, r320 = 1, r512 = 2, r640 = 3 },
    _pad0: u30,
};

pub const GPU = struct {
    const State = enum { recv_command, recv_args, recv_data, send_data };
    pub const vram_size = 0x100000; // 1MB
    pub const VramArray = [vram_size]u8;

    allocator: std.mem.Allocator,
    rasterizer: Rasterizer,

    vram: *align(16) VramArray,
    gpuread: u32,
    gpustat: u32,

    gp0_state: State,
    gp0_fifo: Fifo,
    gp0_cmd: u8,
    gp0_xcur: u16,
    gp0_ycur: u16,
    gp0_draw_area_start: [2]u16,
    gp0_draw_area_end: [2]u16,
    gp0_draw_offset: [2]i16,
    gp0_texture_window_mask: [2]u16,
    gp0_texture_window_offset: [2]u16,

    gp1_display_area_start: [2]u16,
    gp1_display_area_end: [2]u16,
    gp1_display_offset: [2]u16,
    gp1_display_range_x: [2]u16,
    gp1_display_range_y: [2]u16,
    gp1_display_mode: DisplayMode,
    gp1_display_enable: bool,
    gp1_dma_direction: enum(u2) { off = 0, fifo = 1, cpu_to_gp0 = 2, gpuread_to_cpu = 3 },

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        const vram = try allocator.alignedAlloc(u8, std.mem.Alignment.@"16", vram_size);

        self.* = .{
            .allocator = allocator,
            .rasterizer = Rasterizer.init(vram[0..vram_size]),
            .vram = vram[0..vram_size],

            .gpuread = 0,
            .gpustat = 0x14802000,

            .gp0_fifo = std.mem.zeroes(Fifo),
            .gp0_state = .recv_command,
            .gp0_cmd = 0,
            .gp0_xcur = 0,
            .gp0_ycur = 0,
            .gp0_draw_area_start = .{ 0, 0 },
            .gp0_draw_area_end = .{ 0, 0 },
            .gp0_draw_offset = .{ 0, 0 },
            .gp0_texture_window_mask = .{ 0, 0 },
            .gp0_texture_window_offset = .{ 0, 0 },

            .gp1_display_area_start = .{ 0, 0 },
            .gp1_display_area_end = .{ 0, 0 },
            .gp1_display_offset = .{ 0, 0 },
            .gp1_display_range_x = .{ 0, 0 },
            .gp1_display_range_y = .{ 0, 0 },
            .gp1_display_enable = false,
            .gp1_display_mode = std.mem.zeroes(DisplayMode),
            .gp1_dma_direction = .off,
        };
        self.rasterizer.fill(.{ .r = 8, .g = 8, .b = 8 });
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.free(self.vram);
        self.allocator.destroy(self);
    }

    pub fn readWord(self: *@This(), offset: u32) u32 {
        switch (offset) {
            0 => return self.readGpuread(),
            4 => return self.readGpustat(),
            else => std.debug.panic("invalid offset: {x}", .{offset}),
        }
    }

    pub fn writeWord(self: *@This(), offset: u32, v: u32) void {
        switch (offset) {
            0 => self.gp0write(v),
            4 => self.gp1write(v),
            else => std.debug.panic("invalid offset: {x}", .{offset}),
        }
    }

    pub fn readGpuread(self: *@This()) u32 {
        if (self.gp0_state == .send_data) {
            self.stepCommandState(0);
        }
        const v = self.gpuread;
        self.gpuread = 0;
        return v;
    }

    pub fn readGpustat(self: *@This()) u32 {
        return self.gpustat | 0x1c000000;
    }

    // -------------------------
    // GP0 Commands
    // -------------------------

    pub fn gp0write(self: *@This(), v: u32) void {
        // log.debug("gp0 - write: {x}", .{v});
        if (self.gp0_state == .recv_command) {
            self.gp0_cmd = @as(u8, @truncate(v >> 24));
            self.gp0_fifo.reset();
        }
        self.stepCommandState(v);
    }

    fn stepCommandState(self: *@This(), v: u32) void {
        switch (self.gp0_cmd) {
            0x00 => {},
            0x01 => {}, // self.clearCache(v),
            0x28 => self.drawPoly4Mono(v),
            0x2c => self.drawPoly4Textured(v),
            0x30 => self.drawPoly3Shaded(v),
            0x38 => self.drawPoly4Shaded(v),
            0x68 => self.drawRect1x1(v),
            0xa0 => self.cpuToVram(v),
            0xc0 => self.vramToCpu(v),
            0xe1 => {}, // self.setDrawMode(v),
            0xe2 => self.setTextureWindow(v),
            0xe3 => { // self.setDrawAreaTopLeft(v),
                const x = @as(u16, @truncate(v >> 0)) & 0x3ff;
                const y = @as(u16, @truncate(v >> 16)) & 0x1ff;
                self.gp0_draw_area_start = .{ x, y };
            },
            0xe4 => { // self.setDrawAreaBottomRight(v),
                const x = @as(u16, @truncate(v >> 0)) & 0x3ff;
                const y = @as(u16, @truncate(v >> 16)) & 0x1ff;
                self.gp0_draw_area_end = .{ x, y };
            },
            0xe5 => { // self.setDrawOffset(v),
                const x = @as(i16, @bitCast(@as(u16, @truncate(v >> 0)) & 0x3ff));
                const y = @as(i16, @bitCast(@as(u16, @truncate(v >> 16)) & 0x1ff));
                self.gp0_draw_offset = .{ x, y };
            },
            0xe6 => {}, // self.setBitMask(v),
            0x04...0x1e, 0xe0, 0xe7...0xef => {}, // nop
            else => std.debug.panic("unknown gp0 command: 0x{x}", .{self.gp0_cmd}),
        }
    }

    inline fn argColor24(v: u32) Color24 {
        return @bitCast(@as(u24, @truncate(v)));
    }

    inline fn argVertex(v: u32) [2]i16 {
        const x = @as(i16, @intCast(@as(u16, @truncate(v >> 0))));
        const y = @as(i16, @intCast(@as(u16, @truncate(v >> 16))));
        return .{ x, y };
    }

    inline fn argTexcoord(v: u32) [2]u8 {
        const tx = @as(u8, @truncate(v >> 0));
        const ty = @as(u8, @truncate(v >> 8));
        return .{ tx, ty };
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
                const arg_xpos = @as(u16, @truncate(self.gp0_fifo.buf[1] >> 0)) & 0x3ff;
                const arg_ypos = @as(u16, @truncate(self.gp0_fifo.buf[1] >> 16)) & 0x1ff;
                var arg_xsiz = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 0)) & 0x3ff;
                var arg_ysiz = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 16)) & 0x1ff;

                // 0 is treated as max possible value
                if (arg_xsiz == 0) arg_xsiz = 1024;
                if (arg_ysiz == 0) arg_ysiz = 512;

                for (0..2) |i| {
                    const shift = @as(u5, @truncate(i * 16));
                    const halfword = @as(u16, @truncate(v >> shift));

                    const x = @as(u32, (arg_xpos + self.gp0_xcur) & 0x3ff);
                    const y = @as(u32, (arg_ypos + self.gp0_ycur) & 0x1ff);

                    const addr = 2 * (1024 * y + x);
                    mem.writeToBuf(u16, self.vram, addr, halfword);

                    self.gp0_xcur += 1;

                    if (self.gp0_xcur == arg_xsiz) {
                        self.gp0_xcur = 0;
                        self.gp0_ycur += 1;

                        if (self.gp0_ycur == arg_ysiz) {
                            self.gp0_state = .recv_command;
                            break;
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
                self.gpuread = 0;

                const arg_xpos = @as(u16, @truncate(self.gp0_fifo.buf[1] >> 0)) & 0x3ff;
                const arg_ypos = @as(u16, @truncate(self.gp0_fifo.buf[1] >> 16)) & 0x1ff;
                var arg_xsiz = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 0)) & 0x3ff;
                var arg_ysiz = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 16)) & 0x1ff;

                // 0 is treated as max possible value
                if (arg_xsiz == 0) arg_xsiz = 1024;
                if (arg_ysiz == 0) arg_ysiz = 512;

                for (0..2) |i| {
                    const shift = @as(u5, @truncate(i * 16));

                    const x = @as(u32, (arg_xpos + self.gp0_xcur) & 0x3ff);
                    const y = @as(u32, (arg_ypos + self.gp0_ycur) & 0x1ff);

                    const addr = 2 * (1024 * y + x);
                    const halfword = mem.read(u16, self.vram, addr);

                    self.gpuread |= @as(u32, halfword) << shift;
                    self.gp0_xcur += 1;

                    if (self.gp0_xcur == arg_xsiz) {
                        self.gp0_xcur = 0;
                        self.gp0_ycur += 1;

                        if (self.gp0_ycur == arg_ysiz) {
                            self.gp0_state = .recv_command;
                            break;
                        }
                    }
                }
            },
            else => unreachable,
        }
    }

    fn setTextureWindow(self: *@This(), v: u32) void {
        const mask_x = (@as(u16, @truncate(v >> 0)) & 0x1f) << 3;
        const mask_y = (@as(u16, @truncate(v >> 5)) & 0x1f) << 3;
        const offset_x = (@as(u16, @truncate(v >> 10)) & 0x1f) << 3;
        const offset_y = (@as(u16, @truncate(v >> 15)) & 0x1f) << 3;
        self.gp0_texture_window_mask = .{ mask_x, mask_y };
        self.gp0_texture_window_offset = .{ offset_x, offset_y };
        self.rasterizer.setTextureWindow(self.gp0_texture_window_mask, self.gp0_texture_window_offset);
    }

    fn drawRect1x1(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 2) {
                    self.gp0_state = .recv_command;
                    const color = argColor24(self.gp0_fifo.buf[0]);
                    var x, var y = argVertex(self.gp0_fifo.buf[1]);
                    x += self.gp0_draw_offset[0];
                    y += self.gp0_draw_offset[1];
                    self.rasterizer.drawPixel24(x, y, color);
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly3Shaded(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 6) {
                    self.gp0_state = .recv_command;
                    const v0_color = argColor24(self.gp0_fifo.buf[0]);
                    const v0_x, const v0_y = argVertex(self.gp0_fifo.buf[1]);
                    const v1_color = argColor24(self.gp0_fifo.buf[2]);
                    const v1_x, const v1_y = argVertex(self.gp0_fifo.buf[3]);
                    const v2_color = argColor24(self.gp0_fifo.buf[4]);
                    const v2_x, const v2_y = argVertex(self.gp0_fifo.buf[5]);
                    self.rasterizer.drawTriangleShaded(
                        .{ .x = v0_x, .y = v0_y, .color = v0_color },
                        .{ .x = v1_x, .y = v1_y, .color = v1_color },
                        .{ .x = v2_x, .y = v2_y, .color = v2_color },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4Mono(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 5) {
                    self.gp0_state = .recv_command;
                    const color = argColor24(self.gp0_fifo.buf[0]);
                    const v0_x, const v0_y = argVertex(self.gp0_fifo.buf[1]);
                    const v1_x, const v1_y = argVertex(self.gp0_fifo.buf[2]);
                    const v2_x, const v2_y = argVertex(self.gp0_fifo.buf[3]);
                    const v3_x, const v3_y = argVertex(self.gp0_fifo.buf[4]);
                    self.rasterizer.drawTriangleFlat(
                        .{ .x = v0_x, .y = v0_y },
                        .{ .x = v1_x, .y = v1_y },
                        .{ .x = v2_x, .y = v2_y },
                        color,
                    );
                    self.rasterizer.drawTriangleFlat(
                        .{ .x = v1_x, .y = v1_y },
                        .{ .x = v3_x, .y = v3_y },
                        .{ .x = v2_x, .y = v2_y },
                        color,
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4Shaded(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 8) {
                    self.gp0_state = .recv_command;
                    const v0_color = argColor24(self.gp0_fifo.buf[0]);
                    const v0_x, const v0_y = argVertex(self.gp0_fifo.buf[1]);
                    const v1_color = argColor24(self.gp0_fifo.buf[2]);
                    const v1_x, const v1_y = argVertex(self.gp0_fifo.buf[3]);
                    const v2_color = argColor24(self.gp0_fifo.buf[4]);
                    const v2_x, const v2_y = argVertex(self.gp0_fifo.buf[5]);
                    const v3_color = argColor24(self.gp0_fifo.buf[6]);
                    const v3_x, const v3_y = argVertex(self.gp0_fifo.buf[7]);
                    self.rasterizer.drawTriangleShaded(
                        .{ .x = v0_x, .y = v0_y, .color = v0_color },
                        .{ .x = v1_x, .y = v1_y, .color = v1_color },
                        .{ .x = v2_x, .y = v2_y, .color = v2_color },
                    );
                    self.rasterizer.drawTriangleShaded(
                        .{ .x = v1_x, .y = v1_y, .color = v1_color },
                        .{ .x = v3_x, .y = v3_y, .color = v3_color },
                        .{ .x = v2_x, .y = v2_y, .color = v2_color },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4Textured(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 9) {
                    self.gp0_state = .recv_command;

                    const v0_x, const v0_y = argVertex(self.gp0_fifo.buf[1]);
                    const v0_tx, const v0_ty = argTexcoord(self.gp0_fifo.buf[2]);
                    const v1_x, const v1_y = argVertex(self.gp0_fifo.buf[3]);
                    const v1_tx, const v1_ty = argTexcoord(self.gp0_fifo.buf[4]);
                    const v2_x, const v2_y = argVertex(self.gp0_fifo.buf[5]);
                    const v2_tx, const v2_ty = argTexcoord(self.gp0_fifo.buf[6]);
                    const v3_x, const v3_y = argVertex(self.gp0_fifo.buf[7]);
                    const v3_tx, const v3_ty = argTexcoord(self.gp0_fifo.buf[8]);

                    const palette = @as(u16, @truncate(self.gp0_fifo.buf[2] >> 16));
                    const clutx = bits.field(palette, 0, u6);
                    const cluty = bits.field(palette, 6, u9);

                    const texpage = @as(u16, @truncate(self.gp0_fifo.buf[4] >> 16));
                    const tpx = @as(u16, bits.field(texpage, 0, u4)) * 64;
                    const tpy = @as(u16, bits.field(texpage, 4, u1)) * 256;
                    const depth = switch (bits.field(texpage, 7, u2)) {
                        0 => ColorDepth.bit4,
                        1 => ColorDepth.bit8,
                        else => ColorDepth.bit15,
                    };

                    const v0 = Vertex{ .x = v0_x, .y = v0_y, .tx = v0_tx, .ty = v0_ty };
                    const v1 = Vertex{ .x = v1_x, .y = v1_y, .tx = v1_tx, .ty = v1_ty };
                    const v2 = Vertex{ .x = v2_x, .y = v2_y, .tx = v2_tx, .ty = v2_ty };
                    const v3 = Vertex{ .x = v3_x, .y = v3_y, .tx = v3_tx, .ty = v3_ty };

                    self.rasterizer.drawTriangleTextured(v0, v1, v2, clutx, cluty, tpx, tpy, depth);
                    self.rasterizer.drawTriangleTextured(v1, v3, v2, clutx, cluty, tpx, tpy, depth);
                }
            },
            else => unreachable,
        }
    }

    // -------------------------
    // GP1 Commands
    // -------------------------

    pub fn gp1write(self: *@This(), v: u32) void {
        // log.debug("gp1 - write: {x}", .{v});
        const cmd = @as(u8, @truncate(v >> 24)) & 0x3f;
        switch (cmd) {
            0x00 => {}, // self.reset(v),
            0x01 => self.clearFifo(v),
            0x03 => self.gp1_display_enable = (v & 1 != 0),
            0x04 => self.gp1_dma_direction = @enumFromInt(@as(u2, @truncate(v))),
            0x05 => self.setDisplayAreaStart(v),
            0x06 => self.setDisplayRangeX(v),
            0x07 => self.setDisplayRangeY(v),
            0x08 => self.gp1_display_mode = @bitCast(v),
            else => std.debug.panic("unknown gp1 command: 0x{x}", .{cmd}),
        }
    }

    fn clearFifo(self: *@This(), _: u32) void {
        self.gp0_state = .recv_command;
        self.gp0_fifo.reset();
    }

    fn setDisplayAreaStart(self: *@This(), v: u32) void {
        const x = @as(u16, @truncate(v >> 0)) & 0x3ff;
        const y = @as(u16, @truncate(v >> 10)) & 0x1ff;
        self.gp1_display_area_start = .{ x, y };
    }

    fn setDisplayRangeX(self: *@This(), v: u32) void {
        const start_x = @as(u16, @truncate(v >> 0)) & 0xfff;
        const end_x = @as(u16, @truncate(v >> 12)) & 0xfff;
        self.gp1_display_range_x = .{ start_x, end_x };
    }

    fn setDisplayRangeY(self: *@This(), v: u32) void {
        const start_y = @as(u16, @truncate(v >> 0)) & 0x3ff;
        const end_y = @as(u16, @truncate(v >> 10)) & 0x3ff;
        self.gp1_display_range_y = .{ start_y, end_y };
    }
};
