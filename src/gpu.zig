const std = @import("std");
const mem = @import("mem.zig");
const builtin = @import("builtin");

const log = std.log.scoped(.gpu);

pub const GP0Command = enum {
    nop,
    cpu_to_vram,
    vram_to_cpu,
    draw_area_top_left,
    draw_area_bottom_right,
    draw_offset,
    draw_mode,
    texture_window,
    clear_cache,
    bit_mask,
    rect_1x1,

    poly4_mono_opaque,
    poly4_shaded_opaque,
    poly3_shaded_opaque,

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
            0x68 => .rect_1x1,
            0x01 => .clear_cache,
            0x28 => .poly4_mono_opaque,
            0x30 => .poly3_shaded_opaque,
            0x38 => .poly4_shaded_opaque,
            0x00, 0x04...0x1E, 0xE0, 0xE7...0xEF => .nop,
            else => std.debug.panic("unknown gp0 command: {x}", .{cmd}),
        };
    }
};

const GP1Command = enum {
    nop,
    reset,
    clear_fifo,
    register_read,

    pub fn decode(v: u32) GP1Command {
        const cmd = @as(u8, @truncate(v >> 24));
        return switch (cmd) {
            0x00 => .reset,
            0x01 => .clear_fifo,
            else => .nop,
        };
    }
};

const Color24 = packed struct {
    r: u8,
    g: u8,
    b: u8,

    pub inline fn init(r: u8, g: u8, b: u8) Color24 {
        return .{ .r = r, .g = g, .b = b };
    }

    pub inline fn fromU32(v: u32) Color24 {
        return @bitCast(@as(u24, @truncate(v)));
    }
};

const Color15 = packed struct {
    r: u5,
    g: u5,
    b: u5,
    a: u1 = 1,

    pub fn init(r: u5, g: u5, b: u5) Color24 {
        return .{ .r = r, .g = g, .b = b };
    }

    pub inline fn from24(v: Color24) Color15 {
        return .{
            .r = @truncate(v.r >> 3),
            .g = @truncate(v.g >> 3),
            .b = @truncate(v.b >> 3),
        };
    }
};

const Vertex = struct {
    x: i32,
    y: i32,
    color: Color24 = .init(0, 0, 0),
};

pub const Rasterizer = struct {
    const y_res = 512;
    const x_res = 1024;

    pixels: *align(16) [x_res * y_res]Color15,

    pub fn init(vram: *align(16) GPU.VramArray) @This() {
        const pixels = std.mem.bytesAsSlice(Color15, vram);
        return .{ .pixels = pixels[0 .. x_res * y_res] };
    }

    pub inline fn fill(self: *@This(), c: Color24) void {
        @memset(self.pixels, Color15.from24(c));
    }

    pub inline fn drawPixel(self: *@This(), x: i32, y: i32, c: Color24) void {
        const idx: usize = @intCast(x + (y * x_res));
        if (idx >= self.pixels.len) return;

        self.pixels[idx] = Color15.from24(c);
    }

    inline fn edgeFunc(a: Vertex, b: Vertex, c: Vertex) i32 {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    pub fn drawTriangleFlat(
        self: *@This(),
        v0: Vertex,
        v1: Vertex,
        v2: Vertex,
        c: Color24,
    ) void {
        const abc = edgeFunc(v0, v1, v2);
        if (abc < 0) return;

        const x_min = @max(@min(v0.x, v1.x, v2.x), 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), x_res);
        const y_max = @min(@max(v0.y, v1.y, v2.y), y_res);

        var y = y_min;

        while (y <= y_max) : (y += 1) {
            var x = x_min;

            while (x <= x_max) : (x += 1) {
                const p = Vertex{ .x = x, .y = y };
                const abp = edgeFunc(v0, v1, p);
                const bcp = edgeFunc(v1, v2, p);
                const cap = edgeFunc(v2, v0, p);

                if (abp >= 0 and bcp >= 0 and cap >= 0) {
                    self.drawPixel(x, y, c);
                }
            }
        }
    }

    pub fn drawTriangleShaded(
        self: *@This(),
        v0: Vertex,
        v1: Vertex,
        v2: Vertex,
    ) void {
        const abc = edgeFunc(v0, v1, v2);
        if (abc < 0) return;

        const x_min = @max(@min(v0.x, v1.x, v2.x), 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), x_res);
        const y_max = @min(@max(v0.y, v1.y, v2.y), y_res);

        const fixed_bits = 8;
        const fixed_one = 1 << fixed_bits;

        var y = y_min;

        while (y <= y_max) : (y += 1) {
            var x = x_min;

            while (x <= x_max) : (x += 1) {
                const p = Vertex{ .x = x, .y = y };

                const abp = edgeFunc(v0, v1, p);
                const bcp = edgeFunc(v1, v2, p);
                const cap = edgeFunc(v2, v0, p);

                if (abp >= 0 and bcp >= 0 and cap >= 0) {
                    const w0: u32 = @intCast(@divTrunc(bcp * fixed_one, abc));
                    const w1: u32 = @intCast(@divTrunc(cap * fixed_one, abc));
                    const w2: u32 = @intCast(@divTrunc(abp * fixed_one, abc));

                    const r = (v0.color.r * w0 + v1.color.r * w1 + v2.color.r * w2);
                    const g = (v0.color.g * w0 + v1.color.g * w1 + v2.color.g * w2);
                    const b = (v0.color.b * w0 + v1.color.b * w1 + v2.color.b * w2);

                    self.drawPixel(x, y, .{
                        .r = @truncate(r >> fixed_bits),
                        .g = @truncate(g >> fixed_bits),
                        .b = @truncate(b >> fixed_bits),
                    });
                }
            }
        }
    }
};

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

pub const GPU = struct {
    const State = enum { recv_command, recv_args, recv_data, send_data };
    pub const vram_size = 0x100000; // 1MB
    pub const VramArray = [vram_size]u8;

    vram: *align(16) VramArray,
    rasterizer: Rasterizer,
    gp0_state: State,
    gp0_fifo: Fifo,
    gp0_command: ?GP0Command,
    gp0_xcur: u16,
    gp0_ycur: u16,
    gpuread: u32,
    gpustat: u32,
    draw_area_start: [2]u16,
    draw_area_end: [2]u16,
    draw_area_offset: [2]u16,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        const vram = try allocator.alignedAlloc(u8, std.mem.Alignment.@"16", vram_size);

        self.* = .{
            .vram = vram[0..vram_size],
            .rasterizer = Rasterizer.init(vram[0..vram_size]),
            .gp0_fifo = std.mem.zeroes(Fifo),
            .gp0_state = .recv_command,
            .gp0_command = null,
            .gp0_xcur = 0,
            .gp0_ycur = 0,
            .gpuread = 0,
            .gpustat = 0x14802000,
            .draw_area_start = .{ 0, 0 },
            .draw_area_end = .{ 0, 0 },
            .draw_area_offset = .{ 0, 0 },
            .allocator = allocator,
        };

        self.rasterizer.fill(.{ .r = 8, .g = 8, .b = 8 });

        // self.rasterizer.drawTriangleFlat(
        //     .{ .x = 0, .y = 20 },
        //     .{ .x = 100, .y = 20 },
        //     .{ .x = 50, .y = 120 },
        //     Color24.init(255, 0, 0),
        // );

        // self.rasterizer.drawTriangleShaded(
        //     .{ .x = 0, .y = 0, .color = Color24.init(255, 0, 0) },
        //     .{ .x = 480, .y = 0, .color = Color24.init(0, 255, 0) },
        //     .{ .x = 0, .y = 480, .color = Color24.init(0, 0, 255) },
        // );

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
            self.gp0_command = GP0Command.decode(v);
            self.gp0_fifo.len = 0;
        }

        self.stepCommandState(v);
    }

    fn stepCommandState(self: *@This(), v: u32) void {
        switch (self.gp0_command.?) {
            .nop => {},
            .cpu_to_vram => self.cpuToVram(v),
            .vram_to_cpu => self.vramToCpu(v),
            .draw_area_top_left => {},
            .draw_area_bottom_right => {},
            .draw_offset => {},
            .draw_mode => {},
            .texture_window => {},
            .bit_mask => {},
            .clear_cache => {},
            .rect_1x1 => self.drawRect(v),
            .poly3_shaded_opaque => self.drawPoly3Shaded(v),
            .poly4_mono_opaque => self.drawPoly4(v),
            .poly4_shaded_opaque => self.drawPoly4Shaded(v),
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

    fn drawRect(self: *@This(), v: u32) void {
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
                    self.rasterizer.drawPixel(@intCast(x), @intCast(y), Color24.fromU32(color));
                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    inline fn argColor24(v: u32) Color24 {
        return @bitCast(@as(u24, @truncate(v)));
    }

    inline fn argCoords(v: u32) [2]i16 {
        const x = @as(i16, @bitCast(@as(u16, @truncate(v >> 0))));
        const y = @as(i16, @bitCast(@as(u16, @truncate(v >> 16))));
        return .{ x, y };
    }

    fn drawPoly4(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 5) {
                    const color = argColor24(self.gp0_fifo.buf[0]);
                    const v0_x, const v0_y = argCoords(self.gp0_fifo.buf[1]);
                    const v1_x, const v1_y = argCoords(self.gp0_fifo.buf[2]);
                    const v2_x, const v2_y = argCoords(self.gp0_fifo.buf[3]);
                    const v3_x, const v3_y = argCoords(self.gp0_fifo.buf[4]);
                    self.rasterizer.drawTriangleFlat(
                        .{ .x = @intCast(v0_x), .y = @intCast(v0_y) },
                        .{ .x = @intCast(v1_x), .y = @intCast(v1_y) },
                        .{ .x = @intCast(v2_x), .y = @intCast(v2_y) },
                        color,
                    );
                    self.rasterizer.drawTriangleFlat(
                        .{ .x = @intCast(v1_x), .y = @intCast(v1_y) },
                        .{ .x = @intCast(v3_x), .y = @intCast(v3_y) },
                        .{ .x = @intCast(v2_x), .y = @intCast(v2_y) },
                        color,
                    );
                    self.gp0_state = .recv_command;
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
                    const v0_color = argColor24(self.gp0_fifo.buf[0]);
                    const v0_x, const v0_y = argCoords(self.gp0_fifo.buf[1]);
                    const v1_color = argColor24(self.gp0_fifo.buf[2]);
                    const v1_x, const v1_y = argCoords(self.gp0_fifo.buf[3]);
                    const v2_color = argColor24(self.gp0_fifo.buf[4]);
                    const v2_x, const v2_y = argCoords(self.gp0_fifo.buf[5]);
                    const v3_color = argColor24(self.gp0_fifo.buf[6]);
                    const v3_x, const v3_y = argCoords(self.gp0_fifo.buf[7]);
                    self.rasterizer.drawTriangleShaded(
                        .{ .x = @intCast(v0_x), .y = @intCast(v0_y), .color = v0_color },
                        .{ .x = @intCast(v1_x), .y = @intCast(v1_y), .color = v1_color },
                        .{ .x = @intCast(v2_x), .y = @intCast(v2_y), .color = v2_color },
                    );
                    self.rasterizer.drawTriangleShaded(
                        .{ .x = @intCast(v1_x), .y = @intCast(v1_y), .color = v1_color },
                        .{ .x = @intCast(v3_x), .y = @intCast(v3_y), .color = v3_color },
                        .{ .x = @intCast(v2_x), .y = @intCast(v2_y), .color = v2_color },
                    );
                    self.gp0_state = .recv_command;
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
                    const v0_color = argColor24(self.gp0_fifo.buf[0]);
                    const v0_x, const v0_y = argCoords(self.gp0_fifo.buf[1]);
                    const v1_color = argColor24(self.gp0_fifo.buf[2]);
                    const v1_x, const v1_y = argCoords(self.gp0_fifo.buf[3]);
                    const v2_color = argColor24(self.gp0_fifo.buf[4]);
                    const v2_x, const v2_y = argCoords(self.gp0_fifo.buf[5]);
                    self.rasterizer.drawTriangleShaded(
                        .{ .x = @intCast(v0_x), .y = @intCast(v0_y), .color = v0_color },
                        .{ .x = @intCast(v1_x), .y = @intCast(v1_y), .color = v1_color },
                        .{ .x = @intCast(v2_x), .y = @intCast(v2_y), .color = v2_color },
                    );
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
        // log.debug("gp1 - write: {x}", .{v});

        switch (GP1Command.decode(v)) {
            .nop => {},
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
        self.gpuread = @as(u32, switch (reg) {
            7 => 0x00000000,
            else => 0,
        });
    }
};
