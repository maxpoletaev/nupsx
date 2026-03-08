const std = @import("std");

const log = std.log.scoped(.mdec);

const CmdState = enum { idle, set_quant, set_scale, decode };

const OutputDepth = enum(u2) { bit4 = 0, bit8 = 1, bit24 = 2, bit15 = 3 };

const Command = packed struct(u32) {
    param_words: u16, // 0-15
    _pad: u9, // 16-24
    output_bit15: bool, // 25
    output_signed: bool, // 26
    output_depth: OutputDepth, // 27-28
    cmd: u3, // 29-31
};

const Status = packed struct(u32) {
    params_remaining: u16, // 0-15: words remaining minus 1 (FFFFh=None)
    current_block: u3, // 16-18: 0..3=Y1..Y4, 4=Cr, 5=Cb
    _pad0: u4, // 19-22
    output_bit15: bool, // 23
    output_signed: bool, // 24
    output_depth: OutputDepth, // 25-26
    dma_out_request: bool, // 27
    dma_in_request: bool, // 28
    cmd_busy: bool, // 29
    data_in_full: bool, // 30
    data_out_empty: bool, // 31
};

const zigzag = [64]u6{
    0,  1,  5,  6,  14, 15, 27, 28,
    2,  4,  7,  13, 16, 26, 29, 42,
    3,  8,  12, 17, 25, 30, 41, 43,
    9,  11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54,
    20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61,
    35, 36, 48, 49, 57, 58, 62, 63,
};

const zagzig: [64]u6 = blk: {
    var t: [64]u6 = undefined;
    for (0..64) |i| t[zigzag[i]] = @intCast(i);
    break :blk t;
};

inline fn signed10(v: u16) i32 {
    return @as(i32, @as(i10, @bitCast(@as(u10, @truncate(v)))));
}

inline fn clamp8(v: i32) u8 {
    return @intCast(std.math.clamp(v, 0, 255));
}

pub const MDEC = struct {
    pub const addr_start = 0x1f801820;
    pub const addr_end = 0x1f801828;

    const reset_status: u32 = 0x80040000;

    allocator: std.mem.Allocator,
    status: Status,
    dma_in_enable: bool,
    dma_out_enable: bool,

    cmd_state: CmdState,
    param_word_idx: u16,
    quant_color: bool,

    iq_y: [64]u8,
    iq_uv: [64]u8,
    scale_table: [64]i16,

    in_buf: std.ArrayList(u16),
    src_pos: u32,

    crblk: [64]i16,
    cbblk: [64]i16,
    yblk: [4][64]i16,
    out_buf: std.ArrayList(u32),
    out_pos: u32,
    read_part: u2, // which packed word within a 4-pixel RGB24 group (0, 1, or 2)

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(MDEC) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .status = @bitCast(reset_status),
            .dma_in_enable = false,
            .dma_out_enable = false,
            .cmd_state = .idle,
            .param_word_idx = 0,
            .quant_color = false,
            .iq_y = std.mem.zeroes([64]u8),
            .iq_uv = std.mem.zeroes([64]u8),
            .scale_table = std.mem.zeroes([64]i16),
            .in_buf = .empty,
            .src_pos = 0,
            .crblk = std.mem.zeroes([64]i16),
            .cbblk = std.mem.zeroes([64]i16),
            .yblk = std.mem.zeroes([4][64]i16),
            .out_buf = .empty,
            .out_pos = 0,
            .read_part = 0,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.in_buf.deinit(self.allocator);
        self.out_buf.deinit(self.allocator);
        self.allocator.destroy(self);
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const v: u32 = switch (addr) {
            0x1f801820 => self.dmaReadWord(),
            0x1f801824 => self.readStatus(),
            else => 0,
        };
        return @truncate(v >> @as(u5, @intCast((addr & 3) * 8)));
    }

    fn readStatus(self: *@This()) u32 {
        var status = self.status;
        status.data_out_empty = self.out_buf.items.len == 0;
        status.data_in_full = false; // always ready for new input
        return @bitCast(status);
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        switch (addr) {
            0x1f801820 => self.writeParam(@as(u32, v)),
            0x1f801824 => self.writeControl(@as(u32, v)),
            else => {},
        }
    }

    fn writeParam(self: *@This(), v: u32) void {
        if (self.cmd_state == .idle) {
            self.writeCommand(v);
            return;
        }

        const was_decode = self.cmd_state == .decode;
        const is_last = self.status.params_remaining == 0;

        switch (self.cmd_state) {
            .set_quant => self.receiveQuantParam(v),
            .set_scale => self.receiveScaleParam(v),
            .decode => {
                self.receiveDecodeParam(v);
                if (is_last) self.decodeMacroblocks();
            },
            .idle => unreachable,
        }

        self.status.params_remaining -%= 1;
        self.param_word_idx +%= 1;

        if (is_last) {
            self.cmd_state = .idle;
            self.param_word_idx = 0;
            self.status.cmd_busy = was_decode and self.out_buf.items.len > 0;
        }
    }

    fn writeCommand(self: *@This(), v: u32) void {
        const cmd: Command = @bitCast(v);
        self.status.output_bit15 = cmd.output_bit15;
        self.status.output_signed = cmd.output_signed;
        self.status.output_depth = cmd.output_depth;

        switch (cmd.cmd) {
            1 => self.decodeMacroblock(cmd),
            2 => self.setQuantTable(cmd),
            3 => self.setScaleTable(),
            else => self.status.params_remaining = cmd.param_words,
        }

        self.updateDmaRequests();
    }

    fn decodeMacroblock(self: *@This(), cmd: Command) void {
        self.status.params_remaining = cmd.param_words -% 1;
        self.status.cmd_busy = true;
        self.status.current_block = 4; // start at Cr
        self.cmd_state = .decode;
        self.param_word_idx = 0;
        self.in_buf.clearRetainingCapacity();
        self.in_buf.ensureTotalCapacity(self.allocator, @as(u32, cmd.param_words) * 2) catch @panic("OOM");
        self.src_pos = 0;
        self.out_buf.clearRetainingCapacity();
        self.out_pos = 0;
        self.read_part = 0;
        log.debug("decodeMacroblock: words={d} depth={s} signed={}", .{
            cmd.param_words, @tagName(self.status.output_depth), self.status.output_signed,
        });
    }

    fn setQuantTable(self: *@This(), cmd: Command) void {
        self.quant_color = cmd.param_words & 1 != 0;
        self.status.params_remaining = if (self.quant_color) 31 else 15;
        self.status.cmd_busy = true;
        self.cmd_state = .set_quant;
        self.param_word_idx = 0;
        log.debug("setQuantTable: color={}", .{self.quant_color});
    }

    fn setScaleTable(self: *@This()) void {
        self.status.params_remaining = 31;
        self.status.cmd_busy = true;
        self.cmd_state = .set_scale;
        self.param_word_idx = 0;
        log.debug("setScaleTable", .{});
    }

    fn receiveQuantParam(self: *@This(), v: u32) void {
        const byte_idx = @as(u32, self.param_word_idx) * 4;
        const bytes: [4]u8 = @bitCast(v);
        if (byte_idx < 64) {
            for (bytes, 0..) |b, i| {
                if (byte_idx + i < 64) self.iq_y[byte_idx + i] = b;
            }
        } else {
            const uv_idx = byte_idx - 64;
            for (bytes, 0..) |b, i| {
                if (uv_idx + i < 64) self.iq_uv[uv_idx + i] = b;
            }
        }
    }

    fn receiveDecodeParam(self: *@This(), v: u32) void {
        self.in_buf.appendSlice(self.allocator, &.{
            @as(u16, @truncate(v)),
            @as(u16, @truncate(v >> 16)),
        }) catch @panic("OOM");
    }

    fn receiveScaleParam(self: *@This(), v: u32) void {
        const hw_idx = @as(u32, self.param_word_idx) * 2;
        if (hw_idx + 1 < 64) {
            self.scale_table[hw_idx] = @bitCast(@as(u16, @truncate(v)));
            self.scale_table[hw_idx + 1] = @bitCast(@as(u16, @truncate(v >> 16)));
        }
    }

    fn nextHalfword(self: *@This()) u16 {
        if (self.src_pos >= self.in_buf.items.len) return 0xfe00;
        const hw = self.in_buf.items[self.src_pos];
        self.src_pos += 1;
        return hw;
    }

    fn writeControl(self: *@This(), v: u32) void {
        const value: packed struct(u32) {
            _pad: u29,
            dma_out_enable: bool,
            dma_in_enable: bool,
            reset: bool,
        } = @bitCast(v);

        log.debug(
            "writeControl: dma_in={} dma_out={} reset={}",
            .{ value.dma_in_enable, value.dma_out_enable, value.reset },
        );

        if (value.reset) {
            self.status = @bitCast(reset_status);
            self.dma_in_enable = false;
            self.dma_out_enable = false;
            self.cmd_state = .idle;
            self.in_buf.clearRetainingCapacity();
            self.src_pos = 0;
            self.out_buf.clearRetainingCapacity();
            self.out_pos = 0;
            self.read_part = 0;
            return;
        }
        self.dma_in_enable = value.dma_in_enable;
        self.dma_out_enable = value.dma_out_enable;
        self.updateDmaRequests();
    }

    inline fn updateDmaRequests(self: *@This()) void {
        self.status.dma_in_request = self.dma_in_enable and self.out_buf.items.len == 0;
        self.status.dma_out_request = self.dma_out_enable and self.out_buf.items.len > 0;
    }

    // =========================================================================
    // Decoding
    // =========================================================================

    /// Decode one 8x8 block using run-length encoding (spec: rl_decode_block).
    fn rlDecodeBlock(self: *@This(), blk: *[64]i16, qt: *const [64]u8) bool {
        @memset(blk, 0);

        // Skip FE00h padding
        var hw = self.nextHalfword();
        while (hw == 0xfe00) {
            if (self.src_pos >= self.in_buf.items.len) return false; // end of input
            hw = self.nextHalfword();
        }

        // DCT entry: bits 15-10 = q_scale, bits 9-0 = signed DC value
        const q_scale: u32 = (hw >> 10) & 0x3f;
        var current: i32 = signed10(hw);
        var val: i32 = current * @as(i32, qt[0]);

        var k: u32 = 0;
        while (k < 64) {
            if (q_scale == 0) val = current * 2;
            val = std.math.clamp(val, -0x400, 0x3ff);

            if (q_scale > 0) {
                blk[zagzig[k]] = @intCast(val);
            } else {
                blk[k] = @intCast(val);
            }

            const rle = self.nextHalfword();
            current = signed10(rle);
            k += ((rle >> 10) & 0x3f) + 1;
            if (k >= 64) break;

            val = @divTrunc(current * @as(i32, qt[k]) * @as(i32, @intCast(q_scale)) + 4, 8);
        }

        idct(&self.scale_table, blk);
        return true;
    }

    fn decodeMacroblocks(self: *@This()) void {
        self.src_pos = 0;
        self.out_buf.clearRetainingCapacity();

        const is_color = self.status.output_depth == .bit15 or
            self.status.output_depth == .bit24;

        while (self.src_pos < self.in_buf.items.len) {
            if (is_color) {
                if (!self.rlDecodeBlock(&self.crblk, &self.iq_uv)) break;
                if (!self.rlDecodeBlock(&self.cbblk, &self.iq_uv)) break;
                for (&self.yblk) |*yb| {
                    if (!self.rlDecodeBlock(yb, &self.iq_y)) break;
                }
                var pixels: [256]u32 = undefined;
                self.yuvToRgb(&pixels, 0, 0);
                self.yuvToRgb(&pixels, 8, 0);
                self.yuvToRgb(&pixels, 0, 8);
                self.yuvToRgb(&pixels, 8, 8);
                self.out_buf.appendSlice(self.allocator, &pixels) catch @panic("OOM");
            } else {
                if (!self.rlDecodeBlock(&self.yblk[0], &self.iq_y)) break;
                var mono: [64]u8 align(@alignOf(u32)) = undefined;
                self.yToMono(&mono);
                const words = std.mem.bytesAsSlice(u32, &mono);
                self.out_buf.appendSlice(self.allocator, words) catch @panic("OOM");
            }
        }

        self.out_pos = 0;
        self.read_part = 0;
        self.updateDmaRequests();
        log.debug("decoded: words={}", .{self.out_buf.items.len});
    }

    fn yuvToRgb(self: *@This(), out: *[256]u32, bx: u32, by: u32) void {
        const yb = &self.yblk[(by / 8) * 2 + (bx / 8)];
        for (0..8) |y| {
            for (0..8) |x| {
                const luma: i32 = yb[y * 8 + x];
                const cb: i32 = self.cbblk[((y + by) / 2) * 8 + ((x + bx) / 2)];
                const cr: i32 = self.crblk[((y + by) / 2) * 8 + ((x + bx) / 2)];

                const cr_f: f64 = @floatFromInt(cr);
                const cb_f: f64 = @floatFromInt(cb);

                // Spec: R=(1.402*Cr), G=(-0.3437*Cb)+(-0.7143*Cr), B=(1.772*Cb)
                var r: i32 = @as(i32, @intFromFloat(1.402 * cr_f));
                var g: i32 = @as(i32, @intFromFloat(-0.3437 * cb_f - 0.7143 * cr_f));
                var b: i32 = @as(i32, @intFromFloat(1.772 * cb_f));

                // Clamp to signed 8-bit after adding luma
                r = std.math.clamp(luma + r, -128, 127);
                g = std.math.clamp(luma + g, -128, 127);
                b = std.math.clamp(luma + b, -128, 127);

                // If unsigned output, add 128 (xor 0x80)
                if (!self.status.output_signed) {
                    r ^= 0x80;
                    g ^= 0x80;
                    b ^= 0x80;
                }

                const ru: u32 = @intCast(r & 0xff);
                const gu: u32 = @intCast(g & 0xff);
                const bu: u32 = @intCast(b & 0xff);
                out[(by + y) * 16 + (bx + x)] = (bu << 16) | (gu << 8) | ru;
            }
        }
    }

    fn yToMono(self: *@This(), out: *[64]u8) void {
        for (0..64) |i| {
            const raw: i32 = self.yblk[0][i];
            // Clip to signed 9-bit range, then saturate to signed 8-bit
            const clipped: i32 = @as(i32, @as(i9, @truncate(raw)));
            var v: i32 = std.math.clamp(clipped, -128, 127);
            if (!self.status.output_signed) v ^= 0x80;
            out[i] = @intCast(v & 0xff);
        }
    }

    // =========================================================================
    // DMA read interface
    // =========================================================================

    pub fn dataInRequest(self: *@This()) bool {
        return self.status.dma_in_request and self.out_buf.items.len == 0;
    }

    pub fn dataOutRequest(self: *@This()) bool {
        return self.status.dma_out_request and self.out_buf.items.len > 0;
    }

    pub fn dmaReadWord(self: *@This()) u32 {
        if (self.out_pos >= self.out_buf.items.len) return 0;

        const out = self.out_buf.items;
        const data: u32 = switch (self.status.output_depth) {
            .bit15 => blk: {
                if (self.out_pos + 1 >= out.len) break :blk 0;
                const hw0 = rgb24ToBgr15(out[self.out_pos], self.status.output_bit15);
                const hw1 = rgb24ToBgr15(out[self.out_pos + 1], self.status.output_bit15);
                self.out_pos += 2;
                break :blk hw0 | (hw1 << 16);
            },
            .bit24 => self.packRgb24(),
            .bit8, .bit4 => blk: {
                const word = out[self.out_pos];
                self.out_pos += 1;
                break :blk word;
            },
        };

        if (self.out_pos >= out.len) {
            self.out_buf.clearRetainingCapacity();
            self.out_pos = 0;
            self.status.cmd_busy = false;
            self.updateDmaRequests();
        }

        return data;
    }

    /// Serialize 4 RGB24 pixels (stored as RGB32) into a 24bpp DMA word stream.
    /// Called up to 3 times per 4-pixel group, each call returns the next 4 packed bytes.
    fn packRgb24(self: *@This()) u32 {
        const base = self.out_pos;
        const out = self.out_buf.items;
        if (base + 3 >= out.len) {
            self.out_pos += 1;
            return 0;
        }

        const px = [4]u32{ out[base], out[base + 1], out[base + 2], out[base + 3] };
        var rgb: [12]u8 = undefined;
        for (0..4) |i| {
            rgb[i * 3 + 0] = @truncate(px[i]);
            rgb[i * 3 + 1] = @truncate(px[i] >> 8);
            rgb[i * 3 + 2] = @truncate(px[i] >> 16);
        }

        const part = self.read_part;
        const off: usize = @as(usize, part) * 4;
        const data = @as(u32, rgb[off]) |
            @as(u32, rgb[off + 1]) << 8 |
            @as(u32, rgb[off + 2]) << 16 |
            @as(u32, rgb[off + 3]) << 24;

        if (part == 2) {
            self.read_part = 0;
            self.out_pos += 4;
        } else {
            self.read_part += 1;
        }
        return data;
    }
};

inline fn rgb24ToBgr15(rgb: u32, bit15: bool) u32 {
    const r: u32 = (rgb >> 3) & 0x1f;
    const g: u32 = (rgb >> 11) & 0x1f;
    const b: u32 = (rgb >> 19) & 0x1f;
    return (@as(u32, @intFromBool(bit15)) << 15) | (b << 10) | (g << 5) | r;
}

fn idct(scale_table: *const [64]i16, blk: *[64]i16) void {
    var tmp: [64]i64 = undefined;

    for (0..8) |x| {
        for (0..8) |y| {
            var sum: i64 = 0;
            for (0..8) |i| sum += @as(i64, scale_table[i * 8 + y]) * @as(i64, blk[x + i * 8]);
            tmp[x + y * 8] = sum;
        }
    }

    for (0..8) |x| {
        for (0..8) |y| {
            var sum: i64 = 0;
            for (0..8) |i| sum += tmp[i + y * 8] * @as(i64, scale_table[x + i * 8]);
            const round: i64 = (sum >> 31) & 1;
            blk[x + y * 8] = @truncate((sum >> 32) + round);
        }
    }
}
