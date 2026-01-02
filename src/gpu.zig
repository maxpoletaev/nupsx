const std = @import("std");
const mem = @import("mem.zig");
const builtin = @import("builtin");
const rasterizer = @import("rasterizer.zig");
const bits = @import("bits.zig");

const Rasterizer = rasterizer.Rasterizer;
const Color15 = rasterizer.Color15;
const Color24 = rasterizer.Color24;
const Vertex = rasterizer.Vertex;
const RasterDepth = rasterizer.ColorDepth;

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

const Hres1 = enum(u2) { @"256" = 0, @"320" = 1, @"512" = 2, @"640" = 3 };
const Hres2 = enum(u1) { @"256/320/512/640" = 0, @"368" = 1 };
const Vres = enum(u1) { @"240" = 0, @"480" = 1 };
const VideoMode = enum(u1) { ntsc = 0, pal = 1 };
const ColorDepth = enum(u1) { bit15 = 0, bit24 = 1 };
const TexPageColors = enum(u2) { bit4 = 0, bit8 = 1, bit15 = 2 };
const DmaDirection = enum(u2) { off = 0, fifo = 1, cpu_to_gp0 = 2, gpuread_to_cpu = 3 };

const DisplayMode = packed struct(u32) {
    hres: Hres1,
    vres: Vres,
    video_mode: VideoMode,
    color_depth: ColorDepth,
    interlace: bool,
    _pad0: u26,
};

const DrawMode = packed struct(u32) {
    texture_page_x: u4,
    texture_page_y: u1,
    semi_transparency: u2,
    texture_page_colors: TexPageColors,
    dithering: bool,
    drawing_to_display_area: bool,
    texture_disable: bool,
    texrect_xflip: bool,
    texrect_yflip: bool,
    _pad0: u18,
};

const GpuStat = packed struct(u32) {
    texture_page_x: u4,
    texture_page_y: u1,
    semi_transparency: u2,
    texture_page_colors: TexPageColors,
    dithering: bool,
    drawing_to_display_area: bool,

    set_mask_bit: bool,
    draw_pixels_masked: bool,

    interlace_field: bool,
    reverseflag: bool,
    texture_disable: bool,
    hres2: Hres2,
    hres1: Hres1,
    vres: Vres,
    video_mode: VideoMode,
    color_depth: ColorDepth,
    vertical_interlace: bool,

    display_enable: bool,
    interrupt_request: bool,
    dma_data_request: bool,
    ready_receive_cmd: bool,
    ready_send_vram_to_cpu: bool,
    ready_receive_dma_block: bool,
    dma_direction: DmaDirection,
    interlace_odd_line: bool,
};

const CmdState = enum {
    recv_command,
    recv_args,
    recv_data,
    send_data,
};

const gpu_cycles_hblank_start_ntsc: u32 = 2560;
const gpu_cycles_hblank_end_ntsc: u32 = 3413;
const gpu_scans_vblank_start_ntsc: u32 = 240;
const gpu_scans_vblank_end_ntsc: u32 = 263;

inline fn argColor(v: u32) Color24 {
    return @bitCast(@as(u24, @truncate(v)));
}

inline fn argVertex(v: u32) struct { x: i16, y: i16 } {
    const x = @as(i16, @bitCast(bits.field(v, 0, u16)));
    const y = @as(i16, @bitCast(bits.field(v, 16, u16)));
    return .{ .x = x, .y = y };
}

inline fn argTexcoord(v: u32) struct { x: u8, y: u8 } {
    const x = @as(u8, @truncate(v >> 0));
    const y = @as(u8, @truncate(v >> 8));
    return .{ .x = x, .y = y };
}

inline fn argClut(v: u32) struct { x: u16, y: u16 } {
    const clut = v >> 16;
    const x = @as(u16, bits.field(clut, 0, u6)) * 16;
    const y = bits.field(clut, 6, u9);
    return .{ .x = x, .y = y };
}

const Textpage = struct {
    x: u16,
    y: u16,
    depth: RasterDepth,
};

inline fn argTextpage(v: u32) Textpage {
    const texpage = v >> 16;
    const tpx = @as(u16, bits.field(texpage, 0, u4)) * 64;
    const tpy = @as(u16, bits.field(texpage, 4, u1)) * 256;
    const depth = switch (bits.field(texpage, 7, u2)) {
        0 => RasterDepth.bit4,
        1 => RasterDepth.bit8,
        else => RasterDepth.bit15,
    };
    return .{ .x = tpx, .y = tpy, .depth = depth };
}

pub const GPUEvents = packed struct(u4) {
    hblank_start: bool = false,
    hblank_end: bool = false,
    vblank_start: bool = false,
    vblank_end: bool = false,
};

pub const GPU = struct {
    pub const vram_size = 0x100000;
    pub const addr_gp0: u32 = 0x1f801810;
    pub const addr_gp1: u32 = 0x1f801814;
    pub const addr_start: u32 = 0x1f801810;
    pub const addr_end: u32 = 0x1f801817;

    allocator: std.mem.Allocator,
    rasterizer: Rasterizer,

    vram: *align(16) [vram_size]u8,
    gpuread: u32,

    gp0_state: CmdState,
    gp0_fifo: Fifo,
    gp0_cmd: u8,
    gp0_xcur: u16,
    gp0_ycur: u16,
    gp0_draw_mode: DrawMode,
    gp0_draw_area_start: packed struct(u32) { x: u10, y: u9, _pad: u13 },
    gp0_draw_area_end: packed struct(u32) { x: u10, y: u9, _pad: u13 },
    gp0_draw_offset: packed struct(u32) { x: i11, y: i11, _pad: u10 },
    gp0_textwin: packed struct(u32) { mask_x: u5, mask_y: u5, offset_x: u5, offset_y: u5, _pad: u12 },
    gp0_texwin_mask: [2]u16, // todo: use gp0_textwin
    gp0_texwin_offset: [2]u16, // todo: use gp0_textwin

    gp1_display_area_start: packed struct(u32) { x: u10, y: u9, _pad: u13 },
    gp1_display_range_x: packed struct(u32) { x1: u12, x2: u12, _pad: u8 },
    gp1_display_range_y: packed struct(u32) { y1: u11, y2: u11, _pad: u10 },
    gp1_dma_direction: DmaDirection,
    gp1_display_mode: DisplayMode,
    gp1_display_enable: bool,
    interrupt_request: bool,

    cycle: u32 = 0,
    scanline: u32 = 0,
    in_hblank: bool = false,
    in_vblank: bool = false,

    events: GPUEvents = .{},
    frame_ready: bool = false,
    debug_pause: bool = false,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());

        const vram_buf = try allocator.alignedAlloc(u8, .@"16", vram_size);
        const vram = vram_buf[0..vram_size];

        self.* = std.mem.zeroInit(@This(), .{
            .allocator = allocator,
            .rasterizer = Rasterizer.init(vram),
            .gp0_state = .recv_command,
            .gp1_dma_direction = .off,
            .vram = vram,
        });
        self.rasterizer.fill(.{ .r = 8, .g = 8, .b = 8 });
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.free(self.vram);
        self.allocator.destroy(self);
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const v = switch (addr) {
            addr_gp0 => self.readGpuread(),
            addr_gp1 => self.readGpustat(),
            else => std.debug.panic("unhandled GPU read at {x}", .{addr}),
        };
        return @truncate(v);
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        switch (addr) {
            addr_gp0 => self.gp0write(v),
            addr_gp1 => self.gp1write(v),
            else => std.debug.panic("unhandled GPU write at {x}", .{addr}),
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
        var gpustat = std.mem.zeroes(GpuStat);

        gpustat.texture_page_x = self.gp0_draw_mode.texture_page_x;
        gpustat.texture_page_y = self.gp0_draw_mode.texture_page_y;
        gpustat.semi_transparency = self.gp0_draw_mode.semi_transparency;
        gpustat.texture_page_colors = self.gp0_draw_mode.texture_page_colors;
        gpustat.dithering = self.gp0_draw_mode.dithering;
        gpustat.drawing_to_display_area = self.gp0_draw_mode.drawing_to_display_area;
        gpustat.texture_disable = self.gp0_draw_mode.texture_disable;

        gpustat.display_enable = self.gp1_display_enable;
        gpustat.interrupt_request = self.interrupt_request;
        gpustat.dma_direction = self.gp1_dma_direction;
        gpustat.hres1 = self.gp1_display_mode.hres;
        gpustat.vres = .@"240"; // TODO: self.gp1_display_mode.vres does not work, why?
        gpustat.video_mode = .ntsc;
        gpustat.color_depth = self.gp1_display_mode.color_depth;
        gpustat.ready_send_vram_to_cpu = true;
        gpustat.ready_receive_dma_block = true;
        gpustat.ready_receive_cmd = true;

        return @as(u32, @bitCast(gpustat));
    }

    // -------------------------
    // Debugger Helpers
    // -------------------------

    pub fn getDisplayRes(self: *@This()) [2]u16 {
        const w: u16 = switch (self.gp1_display_mode.hres) {
            .@"256" => 256,
            .@"320" => 320,
            .@"512" => 512,
            .@"640" => 640,
        };
        const h: u16 = switch (self.gp1_display_mode.vres) {
            .@"240" => 240,
            .@"480" => 480,
        };
        return .{ w, h };
    }

    // -------------------------
    // GP0 Commands
    // -------------------------

    pub fn gp0write(self: *@This(), v: u32) void {
        if (self.gp0_state == .recv_command) {
            self.gp0_cmd = @as(u8, @truncate(v >> 24));
            self.gp0_fifo.reset();
        }

        // switch (self.gp0_state) {
        //     .recv_command => log.debug("gp0 - cmd: {x}", .{self.gp0_cmd}),
        //     .recv_args => log.debug("gp0 - arg: {x}", .{v}),
        //     else => {},
        // }

        self.stepCommandState(v);
    }

    fn stepCommandState(self: *@This(), v: u32) void {
        switch (self.gp0_cmd) {
            0x00 => {},
            0x01 => {}, // self.clearCache(v),
            0x02 => self.drawRectFlat(v),
            0x1f => self.interrupt_request = true,

            0x20 => self.drawPoly3Flat(v),
            0x22 => self.drawPoly3Flat(v),
            0x28 => self.drawPoly4Flat(v),
            0x2a => self.drawPoly4Flat(v),

            0x24 => self.drawPoly3Textured(v),
            0x25 => self.drawPoly3Textured(v),
            0x26 => self.drawPoly3Textured(v),
            0x27 => self.drawPoly3Textured(v),
            0x2c => self.drawPoly4Textured(v),
            0x2d => self.drawPoly4Textured(v),
            0x2e => self.drawPoly4Textured(v),
            0x2f => self.drawPoly4Textured(v),

            0x34 => self.drawPoly3ShadedTextured(v),
            0x36 => self.drawPoly3ShadedTextured(v),
            0x3c => self.drawPoly4ShadedTextured(v),
            0x3e => self.drawPoly4ShadedTextured(v),

            0x30 => self.drawPoly3Shaded(v),
            0x38 => self.drawPoly4Shaded(v),

            0x40 => self.drawLineFlat(v),
            0x42 => self.drawLineFlat(v),
            // 0x48 => {}, // self.drawPolyLineFlat(v),
            // 0x4a => {}, // self.drawPolyLineFlat(v),
            // 0x50 => {}, // self.drawLineShaded(v),
            // 0x52 => {}, // self.drawLineShaded(v),
            // 0x58 => {}, // self.drawPolyLineShaded(v),
            // 0x5a => {}, // self.drawPolyLineShaded(v),

            0x60 => self.drawRectFlat(v),
            0x62 => self.drawRectFlat(v),
            0x64 => self.drawRectTextured(v, null),
            0x65 => self.drawRectTextured(v, null),
            0x66 => self.drawRectTextured(v, null),
            0x67 => self.drawRectTextured(v, null),
            0x6a => self.drawRect1x1(v),
            0x68 => self.drawRect1x1(v),
            0x6c => self.drawRectTextured(v, 1),
            0x6d => self.drawRectTextured(v, 1),
            0x6e => self.drawRectTextured(v, 1),
            0x6f => self.drawRectTextured(v, 1),
            0x70 => self.drawRectFixedFlat(v, 8),
            0x72 => self.drawRectFixedFlat(v, 8),
            0x74 => self.drawRectTextured(v, 8),
            0x75 => self.drawRectTextured(v, 8),
            0x76 => self.drawRectTextured(v, 8),
            0x77 => self.drawRectTextured(v, 8),
            0x78 => self.drawRectFixedFlat(v, 16),
            0x7a => self.drawRectFixedFlat(v, 16),
            0x7c => self.drawRectTextured(v, 16),
            0x7d => self.drawRectTextured(v, 16),
            0x7e => self.drawRectTextured(v, 16),
            0x7f => self.drawRectTextured(v, 16),

            0x80 => self.vramToVram(v),
            0xa0 => self.cpuToVram(v),
            0xc0 => self.vramToCpu(v),
            0xe1 => self.gp0_draw_mode = @bitCast(v),
            0xe2 => self.setTextureWindow(v),
            0xe3 => self.setDrawAreaStart(v),
            0xe4 => self.setDrawAreaEnd(v),
            0xe5 => self.gp0_draw_offset = @bitCast(v),

            0xe6 => {}, // self.setBitMask(v),
            0x04...0x1e, 0xe0, 0xe7...0xef => {}, // nop
            else => std.debug.panic("unknown gp0 command: 0x{x}", .{self.gp0_cmd}),
        }
    }

    fn vramToVram(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 4) {
                    const src = argVertex(self.gp0_fifo.buf[0]);
                    const dest = argVertex(self.gp0_fifo.buf[1]);
                    const size = argVertex(self.gp0_fifo.buf[2]);

                    self.rasterizer.copyRect(src.x, src.y, dest.x, dest.y, size.x, size.y);
                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
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
                    mem.writeBuf(u16, self.vram, addr, halfword);

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
                    const halfword = mem.readBuf(u16, self.vram, addr);

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
        const mask_x = bits.field(v, 0, u5) *% 8;
        const mask_y = bits.field(v, 5, u5) *% 8;
        const offset_x = bits.field(v, 10, u5) *% 8;
        const offset_y = bits.field(v, 15, u5) *% 8;
        self.gp0_textwin = @bitCast(v);
        self.gp0_texwin_mask = .{ mask_x, mask_y };
        self.gp0_texwin_offset = .{ offset_x, offset_y };
        self.rasterizer.setTextureWindow(self.gp0_texwin_mask, self.gp0_texwin_offset);
    }

    fn setDrawAreaStart(self: *@This(), v: u32) void {
        self.gp0_draw_area_start = @bitCast(v);
        self.rasterizer.setDrawAreaStart(self.gp0_draw_area_start.x, self.gp0_draw_area_start.y);
    }

    fn setDrawAreaEnd(self: *@This(), v: u32) void {
        self.gp0_draw_area_end = @bitCast(v);
        self.rasterizer.setDrawAreaEnd(self.gp0_draw_area_end.x, self.gp0_draw_area_end.y);
    }

    fn setDrawOffset(self: *@This(), v: u32) void {
        self.gp0_draw_offset = @bitCast(v);
        self.rasterizer.setDrawOffset(self.gp0_draw_offset.x, self.gp0_draw_offset.y);
    }

    fn getTextpageFromDrawMode(self: *@This()) Textpage {
        return .{
            .x = @as(u16, self.gp0_draw_mode.texture_page_x) * 64,
            .y = @as(u16, self.gp0_draw_mode.texture_page_y) * 256,
            .depth = switch (self.gp0_draw_mode.texture_page_colors) {
                .bit4 => RasterDepth.bit4,
                .bit8 => RasterDepth.bit8,
                else => RasterDepth.bit15,
            },
        };
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
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos = argVertex(self.gp0_fifo.buf[1]);
                    const x = pos.x + self.gp0_draw_offset.x;
                    const y = pos.y + self.gp0_draw_offset.y;
                    self.rasterizer.drawPixel24(x, y, color);
                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawLineFlat(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 3) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const pos1 = argVertex(self.gp0_fifo.buf[2]);
                    self.rasterizer.drawLineFlat(pos0.x, pos0.y, pos1.x, pos1.y, color);
                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawRectFlat(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 3) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos = argVertex(self.gp0_fifo.buf[1]);
                    const size = argVertex(self.gp0_fifo.buf[2]);
                    self.rasterizer.drawRectFlat(pos.x, pos.y, size.x, size.y, color);
                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawRectFixedFlat(self: *@This(), v: u32, wh: u16) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 3) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos = argVertex(self.gp0_fifo.buf[1]);
                    self.rasterizer.drawRectFlat(pos.x, pos.y, wh, wh, color);
                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawRectTextured(self: *@This(), v: u32, comptime hw: ?i32) void {
        const need_args = if (hw != null) 3 else 4;

        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == need_args) {
                    const pos = argVertex(self.gp0_fifo.buf[1]);
                    const clut = argClut(self.gp0_fifo.buf[2]);

                    const tp = self.getTextpageFromDrawMode();
                    const size = if (hw) |vv| .{ .x = vv, .y = vv } else argVertex(self.gp0_fifo.buf[3]);

                    self.rasterizer.drawRectTextured(pos.x, pos.y, size.x, size.y, clut.x, clut.y, tp.x, tp.y, tp.depth);

                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly3Flat(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 4) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const pos1 = argVertex(self.gp0_fifo.buf[2]);
                    const pos2 = argVertex(self.gp0_fifo.buf[3]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y };

                    self.rasterizer.drawTriangleFlat(v0, v1, v2, color);

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
                    const c0 = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const c1 = argColor(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);
                    const c2 = argColor(self.gp0_fifo.buf[4]);
                    const pos2 = argVertex(self.gp0_fifo.buf[5]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .color = c0 };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .color = c1 };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .color = c2 };

                    self.rasterizer.drawTriangleShaded(v0, v1, v2);

                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4Flat(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 5) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const pos1 = argVertex(self.gp0_fifo.buf[2]);
                    const pos2 = argVertex(self.gp0_fifo.buf[3]);
                    const pos3 = argVertex(self.gp0_fifo.buf[4]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y };
                    const v3 = Vertex{ .x = pos3.x, .y = pos3.y };

                    self.rasterizer.drawTriangleFlat(v0, v1, v2, color);
                    self.rasterizer.drawTriangleFlat(v1, v3, v2, color);

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
                    const c0 = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const c1 = argColor(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);
                    const c2 = argColor(self.gp0_fifo.buf[4]);
                    const pos2 = argVertex(self.gp0_fifo.buf[5]);
                    const c3 = argColor(self.gp0_fifo.buf[6]);
                    const pos3 = argVertex(self.gp0_fifo.buf[7]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .color = c0 };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .color = c1 };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .color = c2 };
                    const v3 = Vertex{ .x = pos3.x, .y = pos3.y, .color = c3 };

                    self.rasterizer.drawTriangleShaded(v0, v1, v2);
                    self.rasterizer.drawTriangleShaded(v1, v3, v2);

                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly3Textured(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 7) {
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const tx0 = argTexcoord(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);
                    const tx1 = argTexcoord(self.gp0_fifo.buf[4]);
                    const pos2 = argVertex(self.gp0_fifo.buf[5]);
                    const tx2 = argTexcoord(self.gp0_fifo.buf[6]);

                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const tp = argTextpage(self.gp0_fifo.buf[4]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .tx = tx0.x, .ty = tx0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .tx = tx1.x, .ty = tx1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .tx = tx2.x, .ty = tx2.y };

                    self.rasterizer.drawTriangleTextured(v0, v1, v2, clut.x, clut.y, tp.x, tp.y, tp.depth);

                    self.gp0_state = .recv_command;
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
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const tx0 = argTexcoord(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);
                    const tx1 = argTexcoord(self.gp0_fifo.buf[4]);
                    const pos2 = argVertex(self.gp0_fifo.buf[5]);
                    const tx2 = argTexcoord(self.gp0_fifo.buf[6]);
                    const pos3 = argVertex(self.gp0_fifo.buf[7]);
                    const tx3 = argTexcoord(self.gp0_fifo.buf[8]);

                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const tp = argTextpage(self.gp0_fifo.buf[4]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .tx = tx0.x, .ty = tx0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .tx = tx1.x, .ty = tx1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .tx = tx2.x, .ty = tx2.y };
                    const v3 = Vertex{ .x = pos3.x, .y = pos3.y, .tx = tx3.x, .ty = tx3.y };

                    self.rasterizer.drawTriangleTextured(v0, v1, v2, clut.x, clut.y, tp.x, tp.y, tp.depth);
                    self.rasterizer.drawTriangleTextured(v1, v3, v2, clut.x, clut.y, tp.x, tp.y, tp.depth);

                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly3ShadedTextured(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 8) {
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const tx0 = argTexcoord(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[4]);
                    const tx1 = argTexcoord(self.gp0_fifo.buf[5]);
                    const pos2 = argVertex(self.gp0_fifo.buf[7]);
                    const tx2 = argTexcoord(self.gp0_fifo.buf[8]);

                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const tp = argTextpage(self.gp0_fifo.buf[5]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .tx = tx0.x, .ty = tx0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .tx = tx1.x, .ty = tx1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .tx = tx2.x, .ty = tx2.y };

                    self.rasterizer.drawTriangleTextured(v0, v1, v2, clut.x, clut.y, tp.x, tp.y, tp.depth);

                    self.gp0_state = .recv_command;
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4ShadedTextured(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.add(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.add(v);
                if (self.gp0_fifo.len == 12) {
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const tx0 = argTexcoord(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[4]);
                    const tx1 = argTexcoord(self.gp0_fifo.buf[5]);
                    const pos2 = argVertex(self.gp0_fifo.buf[7]);
                    const tx2 = argTexcoord(self.gp0_fifo.buf[8]);
                    const pos3 = argVertex(self.gp0_fifo.buf[10]);
                    const tx3 = argTexcoord(self.gp0_fifo.buf[11]);

                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const tp = argTextpage(self.gp0_fifo.buf[5]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .tx = tx0.x, .ty = tx0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .tx = tx1.x, .ty = tx1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .tx = tx2.x, .ty = tx2.y };
                    const v3 = Vertex{ .x = pos3.x, .y = pos3.y, .tx = tx3.x, .ty = tx3.y };

                    self.rasterizer.drawTriangleTextured(v0, v1, v2, clut.x, clut.y, tp.x, tp.y, tp.depth);
                    self.rasterizer.drawTriangleTextured(v1, v3, v2, clut.x, clut.y, tp.x, tp.y, tp.depth);

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
        const cmd = @as(u8, @truncate(v >> 24)) & 0x3f;
        switch (cmd) {
            0x00 => {}, // self.reset(v),
            0x01 => self.clearFifo(v),
            0x02 => self.interrupt_request = false,
            0x03 => self.gp1_display_enable = (v & 1 != 0),
            0x04 => self.gp1_dma_direction = @enumFromInt(@as(u2, @truncate(v))),
            0x05 => self.gp1_display_area_start = @bitCast(v),
            0x06 => self.gp1_display_range_x = @bitCast(v),
            0x07 => self.gp1_display_range_y = @bitCast(v),
            0x08 => self.gp1_display_mode = @bitCast(v),
            0x10 => self.readRegister(v),
            else => std.debug.panic("unknown gp1 command: 0x{x}", .{cmd}),
        }
    }

    fn clearFifo(self: *@This(), _: u32) void {
        self.gp0_state = .recv_command;
        self.gp0_fifo.reset();
    }

    fn readRegister(self: *@This(), v: u32) void {
        const reg = @as(u8, @truncate(v));
        self.gpuread = switch (reg) {
            0, 1, 6, 7 => self.gpuread, // noop (remains unchanged)
            2 => @bitCast(self.gp0_textwin),
            3 => @bitCast(self.gp0_draw_area_start),
            4 => @bitCast(self.gp0_draw_area_end),
            5 => @bitCast(self.gp0_draw_offset),
            8 => @bitCast(self.gp1_display_mode),
            else => std.debug.panic("unknown gp1 read register: 0x{x}", .{reg}),
        };
    }

    // -------------------------
    // GPU Timing
    // -------------------------

    pub inline fn consumeEvents(self: *@This()) GPUEvents {
        const events = self.events;
        self.events = .{};
        return events;
    }

    pub inline fn consumeFrameReady(self: *@This()) bool {
        const ready = self.frame_ready;
        self.frame_ready = false;
        return ready;
    }

    pub fn tick(self: *@This()) void {
        self.cycle += 1;

        switch (self.cycle) {
            gpu_cycles_hblank_start_ntsc => {
                self.in_hblank = true;
                self.events.hblank_start = true;
            },
            gpu_cycles_hblank_end_ntsc => {
                self.in_hblank = false;
                self.events.hblank_end = true;
                self.scanline += 1;
                self.cycle = 0;

                switch (self.scanline) {
                    gpu_scans_vblank_start_ntsc => {
                        self.in_vblank = true;
                        self.frame_ready = true;
                        self.events.vblank_start = true;
                    },
                    gpu_scans_vblank_end_ntsc => {
                        self.scanline = 0;
                        self.in_vblank = false;
                        self.events.vblank_end = true;
                    },
                    else => {},
                }
            },
            else => {},
        }
    }
};
