const std = @import("std");
const mem = @import("mem.zig");
const consts = @import("consts.zig");
const renderer = @import("renderer.zig");
const bits = @import("bits.zig");
const fifo = @import("fifo.zig");

const log = std.log.scoped(.gpu);

const Interrupt = mem.Interrupt;
const RasterCommand = renderer.RasterCommand;
const Transparency = renderer.TransparencyMode;
const RasterDepth = renderer.ColorDepth;
const Vertex = renderer.Vertex;
const Color = renderer.RGB8;

const Hres1 = enum(u2) { @"256" = 0, @"320" = 1, @"512" = 2, @"640" = 3 };
const Hres2 = enum(u1) { @"256/320/512/640" = 0, @"368" = 1 };
const Vres = enum(u1) { @"240" = 0, @"480" = 1 };
const VideoMode = enum(u1) { ntsc = 0, pal = 1 };
const TexpageColorMode = enum(u2) { bit4 = 0, bit8 = 1, bit15 = 2 };
const DmaDirection = enum(u2) { off = 0, fifo = 1, cpu_to_gp0 = 2, gpuread_to_cpu = 3 };
pub const ColorDepth = enum(u1) { bit15 = 0, bit24 = 1 };

const DisplayMode = packed struct(u32) {
    hres: Hres1, // 0-1
    vres: Vres, // 2
    video_mode: VideoMode, // 3
    color_depth: ColorDepth, // 4
    interlace: bool, // 5
    hres2: Hres2, // 6
    _pad0: u25, // 7-31
};

const DrawMode = packed struct(u32) {
    texpage_x: u4, // 0-3
    texpage_y: u1, // 4
    semi_transparency: u2, // 5-6
    texpage_color_mode: TexpageColorMode, // 7-8
    dithering: bool, // 9
    draw_to_display_area: bool, // 10
    texture_disable: bool, // 11
    texrect_xflip: bool, // 12
    texrect_yflip: bool, // 13
    _pad0: u18,
};

const MaskBitSetting = packed struct(u32) {
    force_mask_bit: bool, // 0
    check_mask_bit: bool, // 1
    _pad: u30,
};

const GpuStat = packed struct(u32) {
    texpage_x: u4, // 0-3
    texpage_y: u1, // 4
    semi_transparency: u2, // 5-6
    texpage_color_mode: TexpageColorMode, // 7-8
    dithering: bool, // 9
    drawing_to_display_area: bool, // 10

    force_mask_bit: bool, // 11
    check_mask_bit: bool, // 12

    interlace_field: bool, // 13
    reverseflag: bool, // 14
    texture_disable: bool, // 15
    hres2: Hres2, // 16
    hres1: Hres1, // 17-18
    vres: Vres, // 19
    video_mode: VideoMode, // 20
    color_depth: ColorDepth, // 21
    vertical_interlace: bool, // 22

    display_enable: u1, // 23
    interrupt_request: bool, // 24
    dma_data_request: bool, // 25
    ready_receive_cmd: bool, // 26
    ready_send_vram_to_cpu: bool, // 27
    ready_receive_dma_block: bool, // 28
    dma_direction: DmaDirection, // 29-30
    interlace_odd_line: bool, // 31
};

const CmdState = enum {
    recv_command,
    recv_args,
    recv_data,
    send_data,
};

inline fn argColor(v: u32) Color {
    return @bitCast(@as(u24, @truncate(v)));
}

inline fn signExtend11(v: u16) i16 {
    return @intCast(@as(i11, @bitCast(@as(u11, @truncate(v)))));
}

inline fn argVertex(v: u32) struct { x: i16, y: i16 } {
    const x = signExtend11(bits.field(v, 0, u16));
    const y = signExtend11(bits.field(v, 16, u16));
    return .{ .x = x, .y = y };
}

inline fn argVertexU(v: u32) struct { x: u16, y: u16 } {
    const x = bits.field(v, 0, u16) & 0x3ff;
    const y = bits.field(v, 16, u16) & 0x1ff;
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
    const base_x = @as(u16, bits.field(texpage, 0, u4)) * 64;
    const base_y = @as(u16, bits.field(texpage, 4, u1)) * 256;
    const depth = switch (bits.field(texpage, 7, u2)) {
        0 => RasterDepth.bit4,
        1 => RasterDepth.bit8,
        else => RasterDepth.bit15,
    };
    return .{ .x = base_x, .y = base_y, .depth = depth };
}

const Timing = struct {
    hblank_start: u32,
    hblank_end: u32,
    vblank_start: u32,
    vblank_end: u32,
    frame_time: f64,
};

const ntsc_timing = Timing{
    .hblank_start = consts.gpu_cycles_hblank_start_ntsc,
    .hblank_end = consts.gpu_cycles_hblank_end_ntsc,
    .vblank_start = consts.gpu_scans_vblank_start_ntsc,
    .vblank_end = consts.gpu_scans_vblank_end_ntsc,
    .frame_time = consts.gpu_target_frame_time_ntsc,
};

const pal_timing = Timing{
    .hblank_start = consts.gpu_cycles_hblank_start_pal,
    .hblank_end = consts.gpu_cycles_hblank_end_pal,
    .vblank_start = consts.gpu_scans_vblank_start_pal,
    .vblank_end = consts.gpu_scans_vblank_end_pal,
    .frame_time = consts.gpu_target_frame_time_pal,
};

pub const GPU = struct {
    pub const addr_gp0: u32 = 0x1f801810;
    pub const addr_gp1: u32 = 0x1f801814;
    pub const addr_start: u32 = 0x1f801810;
    pub const addr_end: u32 = 0x1f801817;

    allocator: std.mem.Allocator,
    renderer: renderer.Renderer,

    vram: *align(16) [consts.vram_size]u16,
    gpuread: u32,

    gp0_state: CmdState,
    gp0_fifo: fifo.StaticFifo(u32, 16),
    gp0_cmd: u8,
    gp0_prev_cmd: u8,
    gp0_blit_x: u16,
    gp0_blit_y: u16,
    gp0_draw_mode: DrawMode,
    gp0_mask_bit: MaskBitSetting,
    gp0_draw_area_start: packed struct(u32) { x: u10, y: u9, _pad: u13 },
    gp0_draw_area_end: packed struct(u32) { x: u10, y: u9, _pad: u13 },
    gp0_draw_offset: packed struct(u32) { x: i11, y: i11, _pad: u10 },
    gp0_textwin: packed struct(u32) { mask_x: u5, mask_y: u5, offset_x: u5, offset_y: u5, _pad: u12 },

    gp1_display_area_start: packed struct(u32) { x: u10, y: u9, _pad: u13 },
    gp1_display_range_x: packed struct(u32) { x1: u12, x2: u12, _pad: u8 },
    gp1_display_range_y: packed struct(u32) { y1: u10, y2: u10, _pad: u12 },
    gp1_display_enable: enum(u1) { on = 0, off = 1 },
    gp1_dma_direction: DmaDirection,
    gp1_display_mode: DisplayMode,
    interrupt_request: bool,

    cycle_f: f32 = 0.0,
    scanline: u32 = 0,
    in_hblank: bool = false,
    in_vblank: bool = false,
    timing: Timing = ntsc_timing,

    bus: *mem.Bus,
    frame_ready: bool = false,
    debug_pause: bool = false,

    pub fn init(allocator: std.mem.Allocator, bus: *mem.Bus, vram: *align(16) [consts.vram_size]u16, backend: renderer.Renderer) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");

        self.* = std.mem.zeroInit(@This(), .{
            .allocator = allocator,
            .renderer = backend,
            .gp0_state = .recv_command,
            .gp1_dma_direction = .off,
            .vram = vram,
            .bus = bus,
        });

        self.renderer.start();
        self.renderer.execute(.fill(.{ .r = 8, .g = 8, .b = 8 }));
        self.reset();

        return self;
    }

    pub fn deinit(self: *@This()) void {
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

        gpustat.texpage_x = self.gp0_draw_mode.texpage_x;
        gpustat.texpage_y = self.gp0_draw_mode.texpage_y;
        gpustat.semi_transparency = self.gp0_draw_mode.semi_transparency;
        gpustat.texpage_color_mode = self.gp0_draw_mode.texpage_color_mode;
        gpustat.dithering = self.gp0_draw_mode.dithering;
        gpustat.drawing_to_display_area = self.gp0_draw_mode.draw_to_display_area;
        gpustat.texture_disable = self.gp0_draw_mode.texture_disable;

        gpustat.force_mask_bit = self.gp0_mask_bit.force_mask_bit;
        gpustat.check_mask_bit = self.gp0_mask_bit.check_mask_bit;

        gpustat.display_enable = @intFromEnum(self.gp1_display_enable);
        gpustat.interrupt_request = self.interrupt_request;
        gpustat.dma_direction = self.gp1_dma_direction;
        gpustat.hres1 = self.gp1_display_mode.hres;
        gpustat.video_mode = self.gp1_display_mode.video_mode;
        gpustat.color_depth = self.gp1_display_mode.color_depth;
        gpustat.hres2 = self.gp1_display_mode.hres2;
        gpustat.ready_send_vram_to_cpu = true;
        gpustat.ready_receive_dma_block = true;
        gpustat.ready_receive_cmd = true;
        gpustat.interlace_odd_line = !self.in_vblank and (self.scanline & 1) != 0;

        // The following fields should be taken from self.gp1_display_mode, but setting
        // them to anything other than the hardcoded values seems to break everything.
        gpustat.vres = .@"240";
        gpustat.vertical_interlace = false;

        return @as(u32, @bitCast(gpustat));
    }

    pub inline fn getDisplayRes(self: *@This()) [2]u16 {
        const w: u16 = switch (self.gp1_display_mode.hres2) {
            .@"256/320/512/640" => switch (self.gp1_display_mode.hres) {
                .@"256" => 256,
                .@"320" => 320,
                .@"512" => 512,
                .@"640" => 640,
            },
            .@"368" => 368,
        };
        const base_h: u16 = switch (self.gp1_display_mode.video_mode) {
            .ntsc => 240,
            .pal => 288,
        };
        const h: u16 = switch (self.gp1_display_mode.vres) {
            .@"240" => base_h,
            .@"480" => base_h * 2,
        };
        return .{ w, h };
    }

    pub inline fn getVideoMode(self: *@This()) u32 {
        return @intFromEnum(self.gp1_display_mode.video_mode);
    }

    pub inline fn getColorDepth(self: *@This()) ColorDepth {
        return self.gp1_display_mode.color_depth;
    }

    pub inline fn targetFrameTime(self: *@This()) f64 {
        return self.timing.frame_time;
    }

    // =========================================================================
    // GP0 Commands
    // =========================================================================

    pub fn gp0write(self: *@This(), v: u32) void {
        if (self.gp0_state == .recv_command) {
            self.gp0_prev_cmd = self.gp0_cmd;
            self.gp0_cmd = @as(u8, @truncate(v >> 24));
            self.gp0_fifo.clear();
        }

        switch (self.gp0_state) {
            .recv_command => log.debug("gp0: cmd={x}", .{self.gp0_cmd}),
            // .recv_args => log.debug("gp0 - arg: {x}", .{v}),
            else => {},
        }

        self.stepCommandState(v);
    }

    fn stepCommandState(self: *@This(), v: u32) void {
        const Opaque = false;
        const SemiTrans = true;
        const Blend = true;
        const Raw = false;

        switch (self.gp0_cmd) {
            0x00 => {},
            0x01 => {}, // self.clearCache(v),
            0x02 => self.fillVram(v),
            0x1f => self.interrupt_request = true,

            0x20, 0x21 => self.drawPoly3Flat(v, Opaque),
            0x22, 0x23 => self.drawPoly3Flat(v, SemiTrans),
            0x28, 0x29 => self.drawPoly4Flat(v, Opaque),
            0x2a, 0x2b => self.drawPoly4Flat(v, SemiTrans),

            0x24 => self.drawPoly3Textured(v, Opaque, Blend),
            0x25 => self.drawPoly3Textured(v, Opaque, Raw),
            0x26 => self.drawPoly3Textured(v, SemiTrans, Blend),
            0x27 => self.drawPoly3Textured(v, SemiTrans, Raw),
            0x2c => self.drawPoly4Textured(v, Opaque, Blend),
            0x2d => self.drawPoly4Textured(v, Opaque, Raw),
            0x2e => self.drawPoly4Textured(v, SemiTrans, Blend),
            0x2f => self.drawPoly4Textured(v, SemiTrans, Raw),

            0x34, 0x35 => self.drawPoly3ShadedTextured(v, Opaque),
            0x36, 0x37 => self.drawPoly3ShadedTextured(v, SemiTrans),
            0x3c, 0x3d => self.drawPoly4ShadedTextured(v, Opaque),
            0x3e, 0x3f => self.drawPoly4ShadedTextured(v, SemiTrans),

            0x30, 0x31 => self.drawPoly3Shaded(v, Opaque),
            0x32, 0x33 => self.drawPoly3Shaded(v, SemiTrans),
            0x38, 0x39 => self.drawPoly4Shaded(v, Opaque),
            0x3a, 0x3b => self.drawPoly4Shaded(v, SemiTrans),

            0x40, 0x41, 0x44, 0x45 => self.drawLineFlat(v, Opaque),
            0x42, 0x43, 0x46, 0x47 => self.drawLineFlat(v, SemiTrans),
            0x48, 0x49, 0x4c, 0x4d => self.drawPolyLineFlat(v, Opaque),
            0x4a, 0x4b, 0x4e, 0x4f => self.drawPolyLineFlat(v, SemiTrans),
            0x50, 0x51, 0x54, 0x55 => self.drawLineShaded(v, Opaque),
            0x52, 0x53, 0x56, 0x57 => self.drawLineShaded(v, SemiTrans),
            0x58, 0x59, 0x5c, 0x5d => self.drawPolyLineShaded(v, Opaque),
            0x5a, 0x5b, 0x5e, 0x5f => self.drawPolyLineShaded(v, SemiTrans),

            0x60, 0x61 => self.drawRectFlat(v, null, Opaque),
            0x62, 0x63 => self.drawRectFlat(v, null, SemiTrans),
            0x64 => self.drawRectTextured(v, null, Opaque, Blend),
            0x65 => self.drawRectTextured(v, null, Opaque, Raw),
            0x66 => self.drawRectTextured(v, null, SemiTrans, Blend),
            0x67 => self.drawRectTextured(v, null, SemiTrans, Raw),

            0x68, 0x69 => self.drawRectFlat(v, 1, Opaque),
            0x6a, 0x6b => self.drawRectFlat(v, 1, SemiTrans),
            0x6c => self.drawRectTextured(v, 1, Opaque, Blend),
            0x6d => self.drawRectTextured(v, 1, Opaque, Raw),
            0x6e => self.drawRectTextured(v, 1, SemiTrans, Blend),
            0x6f => self.drawRectTextured(v, 1, SemiTrans, Raw),

            0x70, 0x71 => self.drawRectFlat(v, 8, Opaque),
            0x72, 0x73 => self.drawRectFlat(v, 8, SemiTrans),
            0x74 => self.drawRectTextured(v, 8, Opaque, Blend),
            0x75 => self.drawRectTextured(v, 8, Opaque, Raw),
            0x76 => self.drawRectTextured(v, 8, SemiTrans, Blend),
            0x77 => self.drawRectTextured(v, 8, SemiTrans, Raw),

            0x78, 0x79 => self.drawRectFlat(v, 16, Opaque),
            0x7a, 0x7b => self.drawRectFlat(v, 16, SemiTrans),
            0x7c => self.drawRectTextured(v, 16, Opaque, Blend),
            0x7d => self.drawRectTextured(v, 16, Opaque, Raw),
            0x7e => self.drawRectTextured(v, 16, SemiTrans, Blend),
            0x7f => self.drawRectTextured(v, 16, SemiTrans, Raw),

            0x80 => self.vramToVram(v),
            0xa0 => self.cpuToVram(v),
            0xc0 => self.vramToCpu(v),

            0xe1 => self.setDrawMode(v),
            0xe2 => self.setTextureWindow(v),
            0xe3 => self.setDrawAreaStart(v),
            0xe4 => self.setDrawAreaEnd(v),
            0xe5 => self.setDrawOffset(v),
            0xe6 => self.setMaskBitSetting(v),

            0x03, 0x04...0x1e, 0xe0, 0xe7...0xef => {}, // nop

            else => {
                log.warn("unknown gp0 command: {x} (prev: {x}) ", .{ self.gp0_cmd, self.gp0_prev_cmd });
                // std.debug.panic("unknown gp0 command: {x} (prev: {x}) ", .{ self.gp0_cmd, self.gp0_prev_cmd });
            },
        }
    }

    fn setTextureWindow(self: *@This(), v: u32) void {
        self.gp0_textwin = @bitCast(v);
        const mask_x = self.gp0_textwin.mask_x *% 8;
        const mask_y = self.gp0_textwin.mask_y *% 8;
        const offset_x = self.gp0_textwin.offset_x *% 8;
        const offset_y = self.gp0_textwin.offset_y *% 8;
        self.renderer.execute(.setTextureWindow(mask_x, mask_y, offset_x, offset_y));
    }

    inline fn transparencyModeFromInt(v: u2) Transparency {
        return switch (v) {
            0 => .@"B/2+F/2",
            1 => .@"B+F",
            2 => .@"B-F",
            3 => .@"B+F/4",
        };
    }

    fn setDrawMode(self: *@This(), v: u32) void {
        self.gp0_draw_mode = @bitCast(v);
        self.renderer.execute(.setDithering(self.gp0_draw_mode.dithering));
        self.renderer.execute(.setTransparencyMode(transparencyModeFromInt(self.gp0_draw_mode.semi_transparency)));
        log.debug("setDrawMode: mode={any}", .{self.gp0_draw_mode});
    }

    fn setDrawModeFromArg(self: *@This(), v: u32) void {
        const mode: DrawMode = @bitCast(v >> 16);
        self.gp0_draw_mode.texpage_x = mode.texpage_x;
        self.gp0_draw_mode.texpage_y = mode.texpage_y;
        self.gp0_draw_mode.semi_transparency = mode.semi_transparency;
        self.gp0_draw_mode.texpage_color_mode = mode.texpage_color_mode;
        self.renderer.execute(.setTransparencyMode(transparencyModeFromInt(self.gp0_draw_mode.semi_transparency)));
    }

    fn setDrawAreaStart(self: *@This(), v: u32) void {
        self.gp0_draw_area_start = @bitCast(v);
        self.renderer.execute(.setDrawAreaStart(self.gp0_draw_area_start.x, self.gp0_draw_area_start.y));
        log.debug("setDrawAreaStart: x={} y={}", .{ self.gp0_draw_area_start.x, self.gp0_draw_area_start.y });
    }

    fn setDrawAreaEnd(self: *@This(), v: u32) void {
        self.gp0_draw_area_end = @bitCast(v);
        self.renderer.execute(.setDrawAreaEnd(self.gp0_draw_area_end.x, self.gp0_draw_area_end.y));
        log.debug("setDrawAreaEnd: x={} y={}", .{ self.gp0_draw_area_end.x, self.gp0_draw_area_end.y });
    }

    fn setDrawOffset(self: *@This(), v: u32) void {
        self.gp0_draw_offset = @bitCast(v);
        self.renderer.execute(.setDrawOffset(self.gp0_draw_offset.x, self.gp0_draw_offset.y));
        log.debug("setDrawOffset: x={} y={}", .{ self.gp0_draw_offset.x, self.gp0_draw_offset.y });
    }

    fn setMaskBitSetting(self: *@This(), v: u32) void {
        self.gp0_mask_bit = @bitCast(v);
        self.renderer.execute(.setMaskBitSetting(self.gp0_mask_bit.force_mask_bit, self.gp0_mask_bit.check_mask_bit));
        log.debug("setMaskBitSetting: setting={any}", .{self.gp0_mask_bit});
    }

    fn getTexpage(self: *@This()) Textpage {
        return .{
            .x = @as(u16, self.gp0_draw_mode.texpage_x) * 64,
            .y = @as(u16, self.gp0_draw_mode.texpage_y) * 256,
            .depth = switch (self.gp0_draw_mode.texpage_color_mode) {
                .bit4 => RasterDepth.bit4,
                .bit8 => RasterDepth.bit8,
                else => RasterDepth.bit15,
            },
        };
    }

    // =========================================================================
    // GP0 Memory Transfers
    // =========================================================================

    inline fn readVram(self: *@This(), x: u16, y: u16) u16 {
        const xx = @as(u32, x) & 0x3ff; // 0..1023
        const yy = @as(u32, y) & 0x1ff; // 0..511
        return self.vram[yy * 1024 + xx];
    }

    fn fillVram(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 3) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos = argVertexU(self.gp0_fifo.buf[1]);
                    const size = argVertexU(self.gp0_fifo.buf[2]);

                    self.renderer.execute(.fillRectUnmasked(pos.x, pos.y, size.x, size.y, color));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "fillVram: color={x} pos=({},{}) size=({},{})",
                        .{ @as(u24, @bitCast(color)), pos.x, pos.y, size.x, size.y },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn vramToVram(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 4) {
                    const src = argVertexU(self.gp0_fifo.buf[1]);
                    const dest = argVertexU(self.gp0_fifo.buf[2]);
                    const size = argVertexU(self.gp0_fifo.buf[3]);

                    const width: u16 = if (size.x == 0) 1024 else size.x;
                    const height: u16 = if (size.y == 0) 512 else size.y;

                    self.renderer.execute(.copyRect(src.x, src.y, dest.x, dest.y, width, height));
                    self.gp0_state = .recv_command;

                    log.debug("vramToVram: src=({},{}) dest=({},{}) size=({},{})", .{ src.x, src.y, dest.x, dest.y, width, height });
                }
            },
            else => unreachable,
        }
    }

    fn cpuToVram(self: *@This(), v: u32) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 3) {
                    self.renderer.flush();
                    self.gp0_state = .recv_data;
                    self.gp0_blit_y = 0;
                    self.gp0_blit_x = 0;
                }
            },
            .recv_data => {
                const pos = argVertexU(self.gp0_fifo.buf[1]);
                const size = argVertexU(self.gp0_fifo.buf[2]);

                // 0 is treated as max possible value
                const size_x: u16 = if (size.x == 0) 1024 else size.x;
                const size_y: u16 = if (size.y == 0) 512 else size.y;

                for (0..2) |i| {
                    const shift = @as(u5, @truncate(i * 16));
                    const color = @as(u16, @truncate(v >> shift));

                    self.renderer.setPixelRaw(
                        pos.x + self.gp0_blit_x,
                        pos.y + self.gp0_blit_y,
                        color,
                    );

                    self.gp0_blit_x += 1;

                    if (self.gp0_blit_x == size_x) {
                        self.gp0_blit_x = 0;
                        self.gp0_blit_y += 1;

                        if (self.gp0_blit_y == size_y) {
                            self.gp0_state = .recv_command;
                            log.debug("cpuToVram: pos=({},{}) size=({},{})", .{ pos.x, pos.y, size_x, size_y });
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
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 3) {
                    self.renderer.flush();
                    self.gp0_state = .send_data;
                    self.gp0_blit_y = 0;
                    self.gp0_blit_x = 0;
                }
            },
            .send_data => {
                self.gpuread = 0;

                const pos = argVertexU(self.gp0_fifo.buf[1]);
                const size = argVertexU(self.gp0_fifo.buf[2]);

                // 0 is treated as max possible value
                const size_x: u16 = if (size.x == 0) 1024 else size.x;
                const size_y: u16 = if (size.y == 0) 512 else size.y;

                for (0..2) |i| {
                    const shift = @as(u5, @truncate(i * 16));
                    const x = (pos.x + self.gp0_blit_x) & 0x3ff;
                    const y = (pos.y + self.gp0_blit_y) & 0x1ff;
                    const hw = self.readVram(x, y);

                    self.gpuread |= @as(u32, hw) << shift;
                    self.gp0_blit_x += 1;

                    if (self.gp0_blit_x == size_x) {
                        self.gp0_blit_x = 0;
                        self.gp0_blit_y += 1;

                        if (self.gp0_blit_y == size_y) {
                            self.gp0_state = .recv_command;
                            log.debug("vramToCpu: pos=({},{}) size=({},{})", .{ pos.x, pos.y, size_x, size_y });
                            break;
                        }
                    }
                }
            },
            else => unreachable,
        }
    }

    // =========================================================================
    // GP0 Rendering Commands
    // =========================================================================

    inline fn isLineTerminator(v: u32) bool {
        return v & 0xf000f000 == 0x50005000;
    }

    fn drawLineFlat(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 3) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const pos1 = argVertex(self.gp0_fifo.buf[2]);

                    self.renderer.execute(.drawLineFlat(pos0.x, pos0.y, pos1.x, pos1.y, color, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "lineFlat: color={x} pos0=({},{}) pos1=({},{}) semi_trans={}",
                        .{ @as(u24, @bitCast(color)), pos0.x, pos0.y, pos1.x, pos1.y, semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPolyLineFlat(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                if (!isLineTerminator(v)) {
                    self.gp0_fifo.push(v);
                    return;
                }

                const color = argColor(self.gp0_fifo.pop().?);
                var v0 = argVertex(self.gp0_fifo.pop().?);
                var seg_count: u32 = 0;

                while (!self.gp0_fifo.isEmpty()) {
                    const v1 = argVertex(self.gp0_fifo.pop().?);
                    self.renderer.execute(.drawLineFlat(v0.x, v0.y, v1.x, v1.y, color, semi_trans));
                    v0 = v1;
                    seg_count += 1;
                }

                self.gp0_state = .recv_command;
                log.debug("polyLineFlat: color={x} segments={} semi_trans={}", .{ @as(u24, @bitCast(color)), seg_count, semi_trans });
            },
            else => unreachable,
        }
    }

    fn drawLineShaded(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 4) {
                    const color0 = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const color1 = argColor(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);

                    self.renderer.execute(.drawLineShaded(pos0.x, pos0.y, color0, pos1.x, pos1.y, color1, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "lineShaded: color0={x} pos0=({},{}) color1={x} pos1=({},{}) semi_trans={}",
                        .{ @as(u24, @bitCast(color0)), pos0.x, pos0.y, @as(u24, @bitCast(color1)), pos1.x, pos1.y, semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPolyLineShaded(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                if (!isLineTerminator(v)) {
                    self.gp0_fifo.push(v);
                    return;
                }

                var c0 = argColor(self.gp0_fifo.pop().?);
                var v0 = argVertex(self.gp0_fifo.pop().?);
                var seg_count: u32 = 0;

                while (!self.gp0_fifo.isEmpty()) {
                    const c1 = argColor(self.gp0_fifo.pop().?);
                    const v1 = argVertex(self.gp0_fifo.pop().?);
                    self.renderer.execute(.drawLineShaded(v0.x, v0.y, c0, v1.x, v1.y, c1, semi_trans));
                    c0 = c1;
                    v0 = v1;
                    seg_count += 1;
                }

                self.gp0_state = .recv_command;
                log.debug("polyLineShaded: segments={} semi_trans={}", .{ seg_count, semi_trans });
            },
            else => unreachable,
        }
    }

    fn drawRectFlat(self: *@This(), v: u32, comptime fix_size: ?u16, semi_trans: bool) void {
        const need_args = if (fix_size != null) 2 else 3;

        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == need_args) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos = argVertex(self.gp0_fifo.buf[1]);
                    const size = if (fix_size) |wh| .{ .x = wh, .y = wh } else argVertex(self.gp0_fifo.buf[2]);

                    self.renderer.execute(.drawRectFlat(pos.x, pos.y, size.x, size.y, color, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "rectFlat: color={x} pos=({},{}) size=({},{}) semi_trans={}",
                        .{ @as(u24, @bitCast(color)), pos.x, pos.y, size.x, size.y, semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawRectTextured(self: *@This(), v: u32, comptime fix_size: ?u16, semi_trans: bool, tex_blend: bool) void {
        const need_args = if (fix_size != null) 3 else 4;

        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == need_args) {
                    const texp = self.getTexpage();
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos = argVertex(self.gp0_fifo.buf[1]);
                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const uv = argTexcoord(self.gp0_fifo.buf[2]);
                    const size = if (fix_size) |wh| .{ .x = wh, .y = wh } else argVertex(self.gp0_fifo.buf[3]);

                    self.renderer.execute(.drawRectTextured(pos.x, pos.y, size.x, size.y, uv.x, uv.y, clut.x, clut.y, texp.x, texp.y, texp.depth, color, semi_trans, tex_blend));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "rectTextured: color={x} pos=({},{}) size=({},{}) uv=({},{}) clut=({},{}) texpage=({},{},{s}) semi_trans={} blend={}",
                        .{ @as(u24, @bitCast(color)), pos.x, pos.y, size.x, size.y, uv.x, uv.y, clut.x, clut.y, texp.x, texp.y, @tagName(texp.depth), semi_trans, tex_blend },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly3Flat(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 4) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const pos1 = argVertex(self.gp0_fifo.buf[2]);
                    const pos2 = argVertex(self.gp0_fifo.buf[3]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y };

                    self.renderer.execute(.drawTriangleFlat(v0, v1, v2, color, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "poly3Flat: color={x} pos0=({},{}) pos1=({},{}) pos2=({},{}) semi_trans={}",
                        .{ @as(u24, @bitCast(color)), pos0.x, pos0.y, pos1.x, pos1.y, pos2.x, pos2.y, semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly3Shaded(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 6) {
                    const color0 = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const color1 = argColor(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);
                    const color2 = argColor(self.gp0_fifo.buf[4]);
                    const pos2 = argVertex(self.gp0_fifo.buf[5]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .color = color0 };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .color = color1 };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .color = color2 };

                    self.renderer.execute(.drawTriangleShaded(v0, v1, v2, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "poly3Shaded: color0={x} pos0=({},{}) color1={x} pos1=({},{}) color2={x} pos2=({},{}) semi_trans={}",
                        .{ @as(u24, @bitCast(color0)), pos0.x, pos0.y, @as(u24, @bitCast(color1)), pos1.x, pos1.y, @as(u24, @bitCast(color2)), pos2.x, pos2.y, semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4Flat(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
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

                    self.renderer.execute(.drawTriangleFlat(v0, v1, v2, color, semi_trans));
                    self.renderer.execute(.drawTriangleFlat(v1, v3, v2, color, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "poly4Flat: color={x} pos0=({},{}) pos1=({},{}) pos2=({},{}) pos3=({},{}) semi_trans={}",
                        .{ @as(u24, @bitCast(color)), pos0.x, pos0.y, pos1.x, pos1.y, pos2.x, pos2.y, pos3.x, pos3.y, semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4Shaded(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 8) {
                    const color0 = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const color1 = argColor(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);
                    const color2 = argColor(self.gp0_fifo.buf[4]);
                    const pos2 = argVertex(self.gp0_fifo.buf[5]);
                    const color3 = argColor(self.gp0_fifo.buf[6]);
                    const pos3 = argVertex(self.gp0_fifo.buf[7]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .color = color0 };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .color = color1 };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .color = color2 };
                    const v3 = Vertex{ .x = pos3.x, .y = pos3.y, .color = color3 };

                    self.renderer.execute(.drawTriangleShaded(v0, v1, v2, semi_trans));
                    self.renderer.execute(.drawTriangleShaded(v1, v3, v2, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "poly4Shaded: color0={x} pos0=({},{}) color1={x} pos1=({},{}) color2={x} pos2=({},{}) color3={x} pos3=({},{}) semi_trans={}",
                        .{ @as(u24, @bitCast(color0)), pos0.x, pos0.y, @as(u24, @bitCast(color1)), pos1.x, pos1.y, @as(u24, @bitCast(color2)), pos2.x, pos2.y, @as(u24, @bitCast(color3)), pos3.x, pos3.y, semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly3Textured(self: *@This(), v: u32, semi_trans: bool, tex_blend: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 7) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const uv0 = argTexcoord(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);
                    const uv1 = argTexcoord(self.gp0_fifo.buf[4]);
                    const pos2 = argVertex(self.gp0_fifo.buf[5]);
                    const uv2 = argTexcoord(self.gp0_fifo.buf[6]);
                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const texp = argTextpage(self.gp0_fifo.buf[4]);

                    self.setDrawModeFromArg(self.gp0_fifo.buf[4]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .u = uv0.x, .v = uv0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .u = uv1.x, .v = uv1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .u = uv2.x, .v = uv2.y };

                    self.renderer.execute(.drawTriangleTextured(v0, v1, v2, clut.x, clut.y, texp.x, texp.y, texp.depth, color, semi_trans, tex_blend));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "poly3Textured: color={x} pos0=({},{}) uv0=({},{}) pos1=({},{}) uv1=({},{}) pos2=({},{}) uv2=({},{}) clut=({},{}) texpage=({},{},{s}) semi_trans={} blend={}",
                        .{ @as(u24, @bitCast(color)), pos0.x, pos0.y, uv0.x, uv0.y, pos1.x, pos1.y, uv1.x, uv1.y, pos2.x, pos2.y, uv2.x, uv2.y, clut.x, clut.y, texp.x, texp.y, @tagName(texp.depth), semi_trans, tex_blend },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4Textured(self: *@This(), v: u32, semi_trans: bool, tex_blend: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 9) {
                    const color = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const uv0 = argTexcoord(self.gp0_fifo.buf[2]);
                    const pos1 = argVertex(self.gp0_fifo.buf[3]);
                    const uv1 = argTexcoord(self.gp0_fifo.buf[4]);
                    const pos2 = argVertex(self.gp0_fifo.buf[5]);
                    const uv2 = argTexcoord(self.gp0_fifo.buf[6]);
                    const pos3 = argVertex(self.gp0_fifo.buf[7]);
                    const uv3 = argTexcoord(self.gp0_fifo.buf[8]);
                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const texp = argTextpage(self.gp0_fifo.buf[4]);

                    self.setDrawModeFromArg(self.gp0_fifo.buf[4]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .u = uv0.x, .v = uv0.y };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .u = uv1.x, .v = uv1.y };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .u = uv2.x, .v = uv2.y };
                    const v3 = Vertex{ .x = pos3.x, .y = pos3.y, .u = uv3.x, .v = uv3.y };

                    self.renderer.execute(.drawTriangleTextured(v0, v1, v2, clut.x, clut.y, texp.x, texp.y, texp.depth, color, semi_trans, tex_blend));
                    self.renderer.execute(.drawTriangleTextured(v1, v3, v2, clut.x, clut.y, texp.x, texp.y, texp.depth, color, semi_trans, tex_blend));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "poly4Textured: color={x} pos0=({},{}) uv0=({},{}) pos1=({},{}) uv1=({},{}) pos2=({},{}) uv2=({},{}) pos3=({},{}) uv3=({},{}) clut=({},{}) texpage=({},{},{s}) semi_trans={} blend={}",
                        .{ @as(u24, @bitCast(color)), pos0.x, pos0.y, uv0.x, uv0.y, pos1.x, pos1.y, uv1.x, uv1.y, pos2.x, pos2.y, uv2.x, uv2.y, pos3.x, pos3.y, uv3.x, uv3.y, clut.x, clut.y, texp.x, texp.y, @tagName(texp.depth), semi_trans, tex_blend },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly3ShadedTextured(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 9) {
                    const color0 = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const uv0 = argTexcoord(self.gp0_fifo.buf[2]);
                    const color1 = argColor(self.gp0_fifo.buf[3]);
                    const pos1 = argVertex(self.gp0_fifo.buf[4]);
                    const uv1 = argTexcoord(self.gp0_fifo.buf[5]);
                    const color2 = argColor(self.gp0_fifo.buf[6]);
                    const pos2 = argVertex(self.gp0_fifo.buf[7]);
                    const uv2 = argTexcoord(self.gp0_fifo.buf[8]);
                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const texp = argTextpage(self.gp0_fifo.buf[5]);

                    self.setDrawModeFromArg(self.gp0_fifo.buf[5]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .u = uv0.x, .v = uv0.y, .color = color0 };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .u = uv1.x, .v = uv1.y, .color = color1 };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .u = uv2.x, .v = uv2.y, .color = color2 };

                    self.renderer.execute(.drawTriangleShadedTextured(v0, v1, v2, clut.x, clut.y, texp.x, texp.y, texp.depth, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "poly3ShadedTextured: color0={x} pos0=({},{}) uv0=({},{}) color1={x} pos1=({},{}) uv1=({},{}) color2={x} pos2=({},{}) uv2=({},{}) clut=({},{}) texpage=({},{},{s}) semi_trans={}",
                        .{ @as(u24, @bitCast(color0)), pos0.x, pos0.y, uv0.x, uv0.y, @as(u24, @bitCast(color1)), pos1.x, pos1.y, uv1.x, uv1.y, @as(u24, @bitCast(color2)), pos2.x, pos2.y, uv2.x, uv2.y, clut.x, clut.y, texp.x, texp.y, @tagName(texp.depth), semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    fn drawPoly4ShadedTextured(self: *@This(), v: u32, semi_trans: bool) void {
        switch (self.gp0_state) {
            .recv_command => {
                self.gp0_fifo.push(v);
                self.gp0_state = .recv_args;
            },
            .recv_args => {
                self.gp0_fifo.push(v);
                if (self.gp0_fifo.len == 12) {
                    const color0 = argColor(self.gp0_fifo.buf[0]);
                    const pos0 = argVertex(self.gp0_fifo.buf[1]);
                    const uv0 = argTexcoord(self.gp0_fifo.buf[2]);
                    const color1 = argColor(self.gp0_fifo.buf[3]);
                    const pos1 = argVertex(self.gp0_fifo.buf[4]);
                    const uv1 = argTexcoord(self.gp0_fifo.buf[5]);
                    const color2 = argColor(self.gp0_fifo.buf[6]);
                    const pos2 = argVertex(self.gp0_fifo.buf[7]);
                    const uv2 = argTexcoord(self.gp0_fifo.buf[8]);
                    const color3 = argColor(self.gp0_fifo.buf[9]);
                    const pos3 = argVertex(self.gp0_fifo.buf[10]);
                    const uv3 = argTexcoord(self.gp0_fifo.buf[11]);
                    const clut = argClut(self.gp0_fifo.buf[2]);
                    const texp = argTextpage(self.gp0_fifo.buf[5]);

                    self.setDrawModeFromArg(self.gp0_fifo.buf[5]);

                    const v0 = Vertex{ .x = pos0.x, .y = pos0.y, .u = uv0.x, .v = uv0.y, .color = color0 };
                    const v1 = Vertex{ .x = pos1.x, .y = pos1.y, .u = uv1.x, .v = uv1.y, .color = color1 };
                    const v2 = Vertex{ .x = pos2.x, .y = pos2.y, .u = uv2.x, .v = uv2.y, .color = color2 };
                    const v3 = Vertex{ .x = pos3.x, .y = pos3.y, .u = uv3.x, .v = uv3.y, .color = color3 };

                    self.renderer.execute(.drawTriangleShadedTextured(v0, v1, v2, clut.x, clut.y, texp.x, texp.y, texp.depth, semi_trans));
                    self.renderer.execute(.drawTriangleShadedTextured(v1, v3, v2, clut.x, clut.y, texp.x, texp.y, texp.depth, semi_trans));
                    self.gp0_state = .recv_command;

                    log.debug(
                        "poly4ShadedTextured: color0={x} pos0=({},{}) uv0=({},{}) color1={x} pos1=({},{}) uv1=({},{}) color2={x} pos2=({},{}) uv2=({},{}) color3={x} pos3=({},{}) uv3=({},{}) clut=({},{}) texpage=({},{},{s}) semi_trans={}",
                        .{ @as(u24, @bitCast(color0)), pos0.x, pos0.y, uv0.x, uv0.y, @as(u24, @bitCast(color1)), pos1.x, pos1.y, uv1.x, uv1.y, @as(u24, @bitCast(color2)), pos2.x, pos2.y, uv2.x, uv2.y, @as(u24, @bitCast(color3)), pos3.x, pos3.y, uv3.x, uv3.y, clut.x, clut.y, texp.x, texp.y, @tagName(texp.depth), semi_trans },
                    );
                }
            },
            else => unreachable,
        }
    }

    // =========================================================================
    // GP1 Commands
    // =========================================================================

    pub fn gp1write(self: *@This(), v: u32) void {
        const cmd = @as(u8, @truncate(v >> 24)) & 0x3f;

        switch (cmd) {
            0x00 => {
                self.reset();
                log.debug("gp1 reset gpu", .{});
            },
            0x01 => {
                self.resetCommand();
                log.debug("gp1 reset fifo", .{});
            },
            0x02 => {
                self.interrupt_request = false;
                log.debug("gp1 clear irq", .{});
            },
            0x03 => {
                const enable: u1 = @truncate(v);
                self.gp1_display_enable = @enumFromInt(enable);
                log.debug("gp1 set display enable: enable={d}", .{enable});
            },
            0x04 => {
                self.gp1_dma_direction = @enumFromInt(@as(u2, @truncate(v)));
                log.debug("gp1 set dma direction: dir={s}", .{@tagName(self.gp1_dma_direction)});
            },
            0x05 => {
                self.gp1_display_area_start = @bitCast(v);
                log.debug("gp1 set display start: x={d} y={d}", .{ self.gp1_display_area_start.x, self.gp1_display_area_start.y });
            },
            0x06 => {
                self.gp1_display_range_x = @bitCast(v);
                log.debug("gp1 set horizontal range: x1={d} x2={d}", .{ self.gp1_display_range_x.x1, self.gp1_display_range_x.x2 });
            },
            0x07 => {
                self.gp1_display_range_y = @bitCast(v);
                log.debug("gp1 set vertical range: y1={d} y2={d}", .{ self.gp1_display_range_y.y1, self.gp1_display_range_y.y2 });
            },
            0x08 => {
                self.gp1_display_mode = @bitCast(v);
                log.debug("gp1 set display mode", .{});
                switch (self.gp1_display_mode.video_mode) {
                    .ntsc => self.timing = ntsc_timing,
                    .pal => self.timing = pal_timing,
                }
            },
            0x10 => self.registerToGpuread(v),
            else => std.debug.panic("unknown gp1 command: 0x{x}", .{cmd}),
        }
    }

    fn reset(self: *@This()) void {
        self.gp1_display_range_y.y1 = 0x10;
        self.gp1_display_range_y.y2 = 0x10 + 240;
    }

    fn resetCommand(self: *@This()) void {
        self.gp0_state = .recv_command;
        self.gp0_fifo.clear();
    }

    fn registerToGpuread(self: *@This(), v: u32) void {
        const reg_id = @as(u8, @truncate(v));
        self.gpuread = switch (reg_id) {
            0, 1, 6, 7 => self.gpuread, // noop (remains unchanged)
            2 => @bitCast(self.gp0_textwin),
            3 => @bitCast(self.gp0_draw_area_start),
            4 => @bitCast(self.gp0_draw_area_end),
            5 => @bitCast(self.gp0_draw_offset),
            8 => @bitCast(self.gp1_display_mode),
            else => std.debug.panic("unknown gp1 read register: 0x{x}", .{reg_id}),
        };
    }

    // =========================================================================
    // GPU Timing
    // =========================================================================

    pub inline fn consumeFrameReady(self: *@This()) bool {
        const ready = self.frame_ready;
        if (ready) {
            @branchHint(.unlikely);
            self.renderer.flush();
        }
        self.frame_ready = false;
        return ready;
    }

    pub fn tick(self: *@This(), cyc: u32) void {
        const t = self.timing;
        self.cycle_f += @as(f32, @floatFromInt(cyc)) * @as(f32, @floatCast(consts.gpu_cycles_per_cpu_cycle));

        if (!self.in_hblank and self.cycle_f >= @as(f32, @floatFromInt(t.hblank_start))) {
            self.in_hblank = true;
            self.bus.dev.timers.hblankStart();
        } else if (self.in_hblank and self.cycle_f >= @as(f32, @floatFromInt(t.hblank_end))) {
            self.scanline += 1;
            self.in_hblank = false;
            self.cycle_f -= @as(f32, @floatFromInt(t.hblank_end));
            self.bus.dev.timers.hblankEnd();

            if (self.scanline == t.vblank_start) {
                self.in_vblank = true;
                self.frame_ready = true;
                self.bus.dev.timers.vblankStart();
                self.bus.setInterrupt(Interrupt.vblank);
            } else if (self.scanline == t.vblank_end) {
                self.scanline = 0;
                self.in_vblank = false;
                self.bus.dev.timers.vblankEnd();
            }
        }
    }
};
