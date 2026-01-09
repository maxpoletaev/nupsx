const std = @import("std");
const bits = @import("bits.zig");

const log = std.log.scoped(.gte);

const Vertex = struct {
    x: i16,
    y: i16,
    z: i16,

    fn toArray(self: Vertex) [3]i16 {
        return .{ self.x, self.y, self.z };
    }

    fn getWord(self: Vertex, comptime word_i: u8) u32 {
        return switch (word_i) {
            0 => @as(u32, @as(u16, @bitCast(self.x))) | (@as(u32, @as(u16, @bitCast(self.y))) << 16),
            1 => @bitCast(@as(i32, self.z)),
            else => @compileError("invalid word index"),
        };
    }

    fn setWord(self: *Vertex, comptime word_i: u8, v: u32) void {
        switch (word_i) {
            0 => {
                self.x = @bitCast(bits.field(v, 0, u16));
                self.y = @bitCast(bits.field(v, 16, u16));
            },
            1 => self.z = @bitCast(bits.field(v, 0, u16)),
            else => @compileError("invalid word index"),
        }
    }
};

const Matrix = struct {
    m: [3][3]i16,

    fn getWord(self: Matrix, comptime word_i: u8) u32 {
        const flat = @as(*const [9]i16, @ptrCast(&self.m));
        return switch (word_i) {
            0 => pack16(flat[0], flat[1]),
            1 => pack16(flat[2], flat[3]),
            2 => pack16(flat[4], flat[5]),
            3 => pack16(flat[6], flat[7]),
            4 => @bitCast(@as(i32, flat[8])),
            else => @compileError("invalid word index"),
        };
    }

    fn setWord(self: *Matrix, comptime word_i: u8, v: u32) void {
        const flat = @as(*[9]i16, @ptrCast(&self.m));
        switch (word_i) {
            0 => {
                flat[0] = @bitCast(bits.field(v, 0, u16));
                flat[1] = @bitCast(bits.field(v, 16, u16));
            },
            1 => {
                flat[2] = @bitCast(bits.field(v, 0, u16));
                flat[3] = @bitCast(bits.field(v, 16, u16));
            },
            2 => {
                flat[4] = @bitCast(bits.field(v, 0, u16));
                flat[5] = @bitCast(bits.field(v, 16, u16));
            },
            3 => {
                flat[6] = @bitCast(bits.field(v, 0, u16));
                flat[7] = @bitCast(bits.field(v, 16, u16));
            },
            4 => flat[8] = @bitCast(bits.field(v, 0, u16)),
            else => @compileError("invalid word index"),
        }
    }

    fn pack16(a: i16, b: i16) u32 {
        return @as(u32, @as(u16, @bitCast(a))) | (@as(u32, @as(u16, @bitCast(b))) << 16);
    }
};

const Command = packed struct(u32) {
    opcode: u6, // 0-5
    _pad0: u4, // 6-9
    lm: bool, // 10
    _pad1: u2, // 11-12
    mvmva_tv: u2, // 13-14
    mvmva_mv: u2, // 15-16
    mvmva_mm: u2, // 17-18
    sf: u1, // 19
    _pad2: u12, // 20-31
};

const Flags = packed struct(u32) {
    _pad0: u12, // 0-11
    ir0_saturated: bool, // 12
    sy2_saturated: bool, // 13
    sx2_saturated: bool, // 14
    mac0_neg_overflow: bool, // 15
    mac0_pos_overflow: bool, // 16
    divide_overflow: bool, // 17
    sz3_otz_saturated: bool, // 18
    color_b_saturated: bool, // 19
    color_g_saturated: bool, // 20
    color_r_saturated: bool, // 21
    ir3_saturated: bool, // 22
    ir2_saturated: bool, // 23
    ir1_saturated: bool, // 24
    mac3_neg_overflow: bool, // 25
    mac2_neg_overflow: bool, // 26
    mac1_neg_overflow: bool, // 27
    mac3_pos_overflow: bool, // 28
    mac2_pos_overflow: bool, // 29
    mac1_pos_overflow: bool, // 30
    error_flag: bool, // 31

    fn reset(self: *Flags) void {
        self.* = @bitCast(@as(u32, 0));
    }
};

// =============================================================================
// GTE - Geometry Transformation Engine
// =============================================================================

pub const GTE = struct {
    // Data registers
    v: [3]Vertex, // 0-5
    rgbc: [4]u8, // 6
    otz: u16, // 7
    ir: [4]i16, // 8-11
    sxy: [3][2]i16, // 12-15
    sz: [4]u16, // 16-19
    rgb_fifo: [3][4]u8, // 20-22
    res1: u32, // 23 (prohibited)
    mac: [4]i32, // 24-27
    // irgb: u16, // 28 (constructed from ir1/ir2/ir3)
    // orgb: u16, // 29 (constructed from ir1/ir2/ir3)
    lzcs: i32, // 30
    // lzcr: u32, // 31 (computed from lzcs)

    // Control registers
    rotation: Matrix, // 32-36
    translation: [3]i32, // 37-39
    light: Matrix, // 40-44
    background: [3]i32, // 45-47
    color: Matrix, // 48-52
    far_color: [3]i32, // 53-55
    screen_offset: [2]i32, // 56-57
    proj_dist: u16, // 58
    dqa: i16, // 59
    dqb: i32, // 60
    zsf3: i16, // 61
    zsf4: i16, // 62
    flag: Flags, // 63

    pub fn init() GTE {
        return std.mem.zeroes(GTE);
    }

    // =========================================================================
    // Register I/O
    // =========================================================================

    pub fn readReg(self: *GTE, reg: u8) u32 {
        return switch (reg) {
            // Data registers
            0 => self.v[0].getWord(0),
            1 => self.v[0].getWord(1),
            2 => self.v[1].getWord(0),
            3 => self.v[1].getWord(1),
            4 => self.v[2].getWord(0),
            5 => self.v[2].getWord(1),
            6 => @bitCast(self.rgbc),
            7 => self.otz,
            8 => @bitCast(@as(i32, self.ir[0])),
            9 => @bitCast(@as(i32, self.ir[1])),
            10 => @bitCast(@as(i32, self.ir[2])),
            11 => @bitCast(@as(i32, self.ir[3])),
            12 => @bitCast(self.sxy[0]),
            13 => @bitCast(self.sxy[1]),
            14, 15 => @bitCast(self.sxy[2]),
            16 => self.sz[0],
            17 => self.sz[1],
            18 => self.sz[2],
            19 => self.sz[3],
            20 => @bitCast(self.rgb_fifo[0]),
            21 => @bitCast(self.rgb_fifo[1]),
            22 => @bitCast(self.rgb_fifo[2]),
            23 => self.res1,
            24 => @bitCast(self.mac[0]),
            25 => @bitCast(self.mac[1]),
            26 => @bitCast(self.mac[2]),
            27 => @bitCast(self.mac[3]),
            28, 29 => self.readIRGB(),
            30 => @bitCast(self.lzcs),
            31 => self.readLZCR(),

            // Control registers
            32 => self.rotation.getWord(0),
            33 => self.rotation.getWord(1),
            34 => self.rotation.getWord(2),
            35 => self.rotation.getWord(3),
            36 => self.rotation.getWord(4),
            37 => @bitCast(self.translation[0]),
            38 => @bitCast(self.translation[1]),
            39 => @bitCast(self.translation[2]),
            40 => self.light.getWord(0),
            41 => self.light.getWord(1),
            42 => self.light.getWord(2),
            43 => self.light.getWord(3),
            44 => self.light.getWord(4),
            45 => @bitCast(self.background[0]),
            46 => @bitCast(self.background[1]),
            47 => @bitCast(self.background[2]),
            48 => self.color.getWord(0),
            49 => self.color.getWord(1),
            50 => self.color.getWord(2),
            51 => self.color.getWord(3),
            52 => self.color.getWord(4),
            53 => @bitCast(self.far_color[0]),
            54 => @bitCast(self.far_color[1]),
            55 => @bitCast(self.far_color[2]),
            56 => @bitCast(self.screen_offset[0]),
            57 => @bitCast(self.screen_offset[1]),
            58 => @bitCast(@as(i32, @as(i16, @bitCast(self.proj_dist)))),
            59 => @bitCast(@as(i32, self.dqa)),
            60 => @bitCast(self.dqb),
            61 => @bitCast(@as(i32, self.zsf3)),
            62 => @bitCast(@as(i32, self.zsf4)),
            63 => self.readFLAG(),
            else => std.debug.panic("unhandled GTE read from reg {d}", .{reg}),
        };
    }

    pub fn writeReg(self: *GTE, reg: u8, val: u32) void {
        switch (reg) {
            // Data registers
            0 => self.v[0].setWord(0, val),
            1 => self.v[0].setWord(1, val),
            2 => self.v[1].setWord(0, val),
            3 => self.v[1].setWord(1, val),
            4 => self.v[2].setWord(0, val),
            5 => self.v[2].setWord(1, val),
            6 => self.rgbc = @bitCast(val),
            7 => self.otz = @truncate(val),
            8 => self.ir[0] = @truncate(@as(i32, @bitCast(val))),
            9 => self.ir[1] = @truncate(@as(i32, @bitCast(val))),
            10 => self.ir[2] = @truncate(@as(i32, @bitCast(val))),
            11 => self.ir[3] = @truncate(@as(i32, @bitCast(val))),
            12 => self.sxy[0] = @bitCast(val),
            13 => self.sxy[1] = @bitCast(val),
            14 => self.sxy[2] = @bitCast(val),
            15 => {
                self.sxy[0] = self.sxy[1];
                self.sxy[1] = self.sxy[2];
                self.sxy[2] = @bitCast(val);
            },
            16 => self.sz[0] = @truncate(val),
            17 => self.sz[1] = @truncate(val),
            18 => self.sz[2] = @truncate(val),
            19 => self.sz[3] = @truncate(val),
            20 => self.rgb_fifo[0] = @bitCast(val),
            21 => self.rgb_fifo[1] = @bitCast(val),
            22 => self.rgb_fifo[2] = @bitCast(val),
            23 => self.res1 = val,
            24 => self.mac[0] = @bitCast(val),
            25 => self.mac[1] = @bitCast(val),
            26 => self.mac[2] = @bitCast(val),
            27 => self.mac[3] = @bitCast(val),
            28 => self.writeIRGB(val),
            29 => {},
            30 => self.lzcs = @bitCast(val),
            31 => {},

            // Control registers
            32 => self.rotation.setWord(0, val),
            33 => self.rotation.setWord(1, val),
            34 => self.rotation.setWord(2, val),
            35 => self.rotation.setWord(3, val),
            36 => self.rotation.setWord(4, val),
            37 => self.translation[0] = @bitCast(val),
            38 => self.translation[1] = @bitCast(val),
            39 => self.translation[2] = @bitCast(val),
            40 => self.light.setWord(0, val),
            41 => self.light.setWord(1, val),
            42 => self.light.setWord(2, val),
            43 => self.light.setWord(3, val),
            44 => self.light.setWord(4, val),
            45 => self.background[0] = @bitCast(val),
            46 => self.background[1] = @bitCast(val),
            47 => self.background[2] = @bitCast(val),
            48 => self.color.setWord(0, val),
            49 => self.color.setWord(1, val),
            50 => self.color.setWord(2, val),
            51 => self.color.setWord(3, val),
            52 => self.color.setWord(4, val),
            53 => self.far_color[0] = @bitCast(val),
            54 => self.far_color[1] = @bitCast(val),
            55 => self.far_color[2] = @bitCast(val),
            56 => self.screen_offset[0] = @bitCast(val),
            57 => self.screen_offset[1] = @bitCast(val),
            58 => self.proj_dist = @truncate(val),
            59 => self.dqa = bits.field(val, 0, i16),
            60 => self.dqb = @bitCast(val),
            61 => self.zsf3 = bits.field(val, 0, i16),
            62 => self.zsf4 = bits.field(val, 0, i16),
            63 => self.flag = @bitCast(val & 0x7ffff000),
            else => std.debug.panic("unhandled GTE write to reg {d} = {x}", .{ reg, val }),
        }
    }

    fn readLZCR(self: *GTE) u32 {
        var v: u32 = @bitCast(self.lzcs);
        if (self.lzcs < 0) v = ~v;
        return @clz(v);
    }

    fn readIRGB(self: *GTE) u32 {
        const r: u32 = @intCast(std.math.clamp(self.ir[1] >> 7, 0, 0x1f));
        const g: u32 = @intCast(std.math.clamp(self.ir[2] >> 7, 0, 0x1f));
        const b: u32 = @intCast(std.math.clamp(self.ir[3] >> 7, 0, 0x1f));
        return r | (g << 5) | (b << 10);
    }

    fn writeIRGB(self: *GTE, val: u32) void {
        self.ir[1] = @intCast(((val >> 0) & 0x1f) * 0x80);
        self.ir[2] = @intCast(((val >> 5) & 0x1f) * 0x80);
        self.ir[3] = @intCast(((val >> 10) & 0x1f) * 0x80);
    }

    fn readFLAG(self: *GTE) u32 {
        var flags = self.flag;
        const raw: u32 = @bitCast(self.flag);
        flags.error_flag = (raw & 0x7f87e000) != 0;
        return @bitCast(flags);
    }

    // =========================================================================
    // Command Execution
    // =========================================================================

    pub fn exec(self: *GTE, val: u32) void {
        const cmd: Command = @bitCast(val);
        self.flag.reset();

        switch (cmd.opcode) {
            0x00 => {},
            0x01 => self.rtps(cmd, 0),
            0x06 => self.nclip(),
            0x0c => self.op(cmd),
            0x10 => self.dpcs(cmd, false),
            0x11 => self.intpl(cmd),
            0x12 => self.mvmva(cmd),
            0x13 => self.ncds(cmd, 0),
            0x14 => self.cdp(cmd),
            0x16 => {
                self.ncds(cmd, 0);
                self.ncds(cmd, 1);
                self.ncds(cmd, 2);
            },
            0x1b => self.nccs(cmd, 0),
            0x1c => self.cc(cmd),
            0x1e => self.ncs(cmd, 0),
            0x20 => {
                self.ncs(cmd, 0);
                self.ncs(cmd, 1);
                self.ncs(cmd, 2);
            },
            0x28 => self.sqr(cmd),
            0x29 => self.dcpl(cmd),
            0x2a => {
                self.dpcs(cmd, true);
                self.dpcs(cmd, true);
                self.dpcs(cmd, true);
            },
            0x2d => self.avsz3(),
            0x2e => self.avsz4(),
            0x30 => {
                self.rtps(cmd, 0);
                self.rtps(cmd, 1);
                self.rtps(cmd, 2);
            },
            0x3d => self.gpf(cmd),
            0x3e => self.gpl(cmd),
            0x3f => {
                self.nccs(cmd, 0);
                self.nccs(cmd, 1);
                self.nccs(cmd, 2);
            },
            else => log.warn("unhandled GTE command {x}", .{cmd.opcode}),
        }
    }

    /// NCLIP - Normal clipping
    fn nclip(self: *GTE) void {
        const s = self.sxy;
        const result = @as(i64, s[0][0]) *% (s[1][1] -% s[2][1]) +%
            @as(i64, s[1][0]) *% (s[2][1] -% s[0][1]) +%
            @as(i64, s[2][0]) *% (s[0][1] -% s[1][1]);
        self.mac[0] = self.saturateMAC0(result);
    }

    /// RTPS - Perspective transformation
    fn rtps(self: *GTE, cmd: Command, n: u2) void {
        const result_z = self.transformVertex(n, cmd);

        self.pushSZ(result_z >> 12);

        const quotient: i64 = self.unrDivide(self.proj_dist, self.sz[3]);
        const sx = self.setMAC0(quotient * self.ir[1] + self.screen_offset[0]) >> 16;
        const sy = self.setMAC0(quotient * self.ir[2] + self.screen_offset[1]) >> 16;
        self.pushSXY(sx, sy);

        const depth = self.setMAC0(quotient * self.dqa + self.dqb);
        self.ir[0] = self.saturateIR(0, depth >> 12, false);
    }

    /// AVSZ3 - Average of 3 Z values
    fn avsz3(self: *GTE) void {
        const sum: i64 = @as(i64, self.sz[1]) + self.sz[2] + self.sz[3];
        const result = @as(i64, self.zsf3) * sum;
        self.mac[0] = self.saturateMAC0(result);
        self.otz = self.saturateOTZ(result >> 12);
    }

    /// AVSZ4 - Average of 4 Z values
    fn avsz4(self: *GTE) void {
        const sum: i64 = @as(i64, self.sz[0]) + self.sz[1] + self.sz[2] + self.sz[3];
        const result = @as(i64, self.zsf4) * sum;
        self.mac[0] = self.saturateMAC0(result);
        self.otz = self.saturateOTZ(result >> 12);
    }

    /// NCS - Normal color (single)
    fn ncs(self: *GTE, cmd: Command, n: u2) void {
        self.transformLight(n, cmd);
        self.transformColor(cmd);
        self.pushColor();
    }

    /// NCCS - Normal color color (single)
    fn nccs(self: *GTE, cmd: Command, n: u2) void {
        self.transformLight(n, cmd);
        self.transformColor(cmd);
        self.interpolateRGB(cmd);
        self.pushColor();
    }

    /// NCDS - Normal color depth cue (single)
    fn ncds(self: *GTE, cmd: Command, n: u2) void {
        self.transformLight(n, cmd);
        self.transformColor(cmd);
        self.depthCue(cmd);
        self.pushColor();
    }

    /// CC - Color color
    fn cc(self: *GTE, cmd: Command) void {
        self.transformColor(cmd);
        self.interpolateRGB(cmd);
        self.pushColor();
    }

    /// CDP - Color depth cue
    fn cdp(self: *GTE, cmd: Command) void {
        self.transformColor(cmd);
        self.depthCue(cmd);
        self.pushColor();
    }

    /// DPCS - Depth cue (single)
    fn dpcs(self: *GTE, cmd: Command, use_fifo: bool) void {
        var rgb: [3]i64 = undefined;
        if (use_fifo) {
            rgb[0] = @as(i64, self.rgb_fifo[0][0]) << 4;
            rgb[1] = @as(i64, self.rgb_fifo[0][1]) << 4;
            rgb[2] = @as(i64, self.rgb_fifo[0][2]) << 4;
        } else {
            rgb[0] = self.rgbScaled(0);
            rgb[1] = self.rgbScaled(1);
            rgb[2] = self.rgbScaled(2);
        }

        inline for (1..4) |i| {
            self.setMACtoIR(i, (self.far_color[i - 1] << 12) - (rgb[i - 1] << 12), false, cmd.sf);
        }

        self.interpolateWithIR0(.{ @intCast(rgb[0]), @intCast(rgb[1]), @intCast(rgb[2]) }, cmd);
        self.pushColor();
    }

    /// DCPL - Depth cue color light
    fn dcpl(self: *GTE, cmd: Command) void {
        const prev = [3]i16{ self.ir[1], self.ir[2], self.ir[3] };

        inline for (1..4) |i| {
            self.setMACtoIR(i, (self.far_color[i - 1] << 12) - self.rgbScaled(i - 1) * prev[i - 1], false, cmd.sf);
        }

        inline for (1..4) |i| {
            self.setMACtoIR(i, self.rgbScaled(i - 1) * prev[i - 1] + @as(i64, self.ir[0]) * self.ir[i], cmd.lm, cmd.sf);
        }

        self.pushColor();
    }

    /// INTPL - Interpolation
    fn intpl(self: *GTE, cmd: Command) void {
        const prev = [3]i16{ self.ir[1], self.ir[2], self.ir[3] };

        inline for (1..4) |i| {
            self.setMACtoIR(i, (self.far_color[i - 1] << 12) - (@as(i64, prev[i - 1]) << 12), false, cmd.sf);
        }

        self.interpolateWithIR0(prev, cmd);
        self.pushColor();
    }

    /// GPF - General purpose interpolation
    fn gpf(self: *GTE, cmd: Command) void {
        self.interpolateWithIR0(.{ 0, 0, 0 }, cmd);
        self.pushColor();
    }

    /// GPL - General purpose interpolation with base
    fn gpl(self: *GTE, cmd: Command) void {
        const shift: u6 = @as(u6, cmd.sf) * 12;
        inline for (1..4) |i| {
            self.setMACtoIR(i, (@as(i64, self.mac[i]) << shift) + @as(i64, self.ir[0]) * self.ir[i], cmd.lm, cmd.sf);
        }
        self.pushColor();
    }

    /// SQR - Square vector
    fn sqr(self: *GTE, cmd: Command) void {
        inline for (1..4) |i| {
            self.setMACtoIR(i, @as(i64, self.ir[i]) * self.ir[i], cmd.lm, cmd.sf);
        }
    }

    /// OP - Outer product
    fn op(self: *GTE, cmd: Command) void {
        const d = [3]i64{ self.rotation.m[0][0], self.rotation.m[1][1], self.rotation.m[2][2] };
        self.setMACtoIR(1, d[2] * self.ir[3] - d[1] * self.ir[2], cmd.lm, cmd.sf);
        self.setMACtoIR(2, d[0] * self.ir[1] - d[2] * self.ir[3], cmd.lm, cmd.sf);
        self.setMACtoIR(3, d[1] * self.ir[2] - d[0] * self.ir[1], cmd.lm, cmd.sf);
    }

    /// MVMVA - Matrix-vector multiply and add
    fn mvmva(self: *GTE, cmd: Command) void {
        const matrix = self.selectMatrix(cmd.mvmva_mm);
        const vec = self.selectVector(cmd.mvmva_mv);

        if (cmd.mvmva_tv == 2) {
            self.mvmvaBuggyFarColor(matrix, vec, cmd);
            return;
        }

        const tr: [3]i32 = switch (cmd.mvmva_tv) {
            0 => self.translation,
            1 => self.background,
            3 => .{ 0, 0, 0 },
            else => unreachable,
        };

        self.matrixMultiply(matrix, vec, tr, cmd);
    }

    // =========================================================================
    // Transformation Helpers
    // =========================================================================

    fn transformVertex(self: *GTE, n: u2, cmd: Command) i64 {
        const vec = self.v[n].toArray();
        const tr = self.translation;
        const m = self.rotation.m;

        const rx = self.accumulate(1, tr[0], m[0], vec);
        const ry = self.accumulate(2, tr[1], m[1], vec);
        const rz = self.accumulate(3, tr[2], m[2], vec);

        self.setMACtoIR(1, rx, cmd.lm, cmd.sf);
        self.setMACtoIR(2, ry, cmd.lm, cmd.sf);

        // RTP handles IR3 specially
        const shift: u6 = @as(u6, cmd.sf) * 12;
        self.mac[3] = @truncate(rz >> shift);
        _ = self.saturateIR(3, rz >> 12, false); // Set flag only
        self.ir[3] = saturateNoFlag(self.mac[3], cmd.lm);

        return rz;
    }

    fn transformLight(self: *GTE, n: u2, cmd: Command) void {
        const vec = self.v[n].toArray();
        self.matrixMultiply(self.light.m, vec, .{ 0, 0, 0 }, cmd);
    }

    fn transformColor(self: *GTE, cmd: Command) void {
        const vec = [3]i16{ self.ir[1], self.ir[2], self.ir[3] };
        self.matrixMultiply(self.color.m, vec, self.background, cmd);
    }

    fn depthCue(self: *GTE, cmd: Command) void {
        const prev = [3]i16{ self.ir[1], self.ir[2], self.ir[3] };

        inline for (1..4) |i| {
            self.setMACtoIR(i, (self.far_color[i - 1] << 12) - self.rgbScaled(i - 1) * self.ir[i], false, cmd.sf);
        }

        inline for (1..4) |i| {
            self.setMACtoIR(i, self.rgbScaled(i - 1) * prev[i - 1] + @as(i64, self.ir[0]) * self.ir[i], cmd.lm, cmd.sf);
        }
    }

    fn interpolateRGB(self: *GTE, cmd: Command) void {
        inline for (1..4) |i| {
            self.setMACtoIR(i, self.rgbScaled(i - 1) * self.ir[i], cmd.lm, cmd.sf);
        }
    }

    fn interpolateWithIR0(self: *GTE, base: [3]i16, cmd: Command) void {
        inline for (1..4) |i| {
            self.setMACtoIR(i, (@as(i64, base[i - 1]) << 12) + @as(i64, self.ir[0]) * self.ir[i], cmd.lm, cmd.sf);
        }
    }

    fn matrixMultiply(self: *GTE, m: [3][3]i16, vec: [3]i16, tr: [3]i32, cmd: Command) void {
        inline for (1..4) |i| {
            const result = self.accumulate(i, tr[i - 1], m[i - 1], vec);
            self.setMACtoIR(i, result, cmd.lm, cmd.sf);
        }
    }

    fn accumulate(self: *GTE, comptime i: u2, tr: i32, row: [3]i16, vec: [3]i16) i64 {
        var acc = self.saturateMAC(i, (@as(i64, tr) << 12) + @as(i64, row[0]) * vec[0]);
        acc = self.saturateMAC(i, acc + @as(i64, row[1]) * vec[1]);
        acc = self.saturateMAC(i, acc + @as(i64, row[2]) * vec[2]);
        return acc;
    }

    fn mvmvaBuggyFarColor(self: *GTE, matrix: [3][3]i16, vec: [3]i16, cmd: Command) void {
        const shift: u6 = @as(u6, cmd.sf) * 12;

        // Flags from first component, but IR gets intermediate result
        inline for (1..4) |i| {
            const val = self.saturateMAC(i, (self.far_color[i - 1] << 12) + @as(i64, matrix[i - 1][0]) * vec[0]);
            self.ir[i] = self.saturateIR(i, val >> shift, false);
        }

        // Result from components 2 and 3
        inline for (1..4) |i| {
            var acc = self.saturateMAC(i, @as(i64, matrix[i - 1][1]) * vec[1]);
            acc = self.saturateMAC(i, acc + @as(i64, matrix[i - 1][2]) * vec[2]);
            self.setMACtoIR(i, acc, cmd.lm, cmd.sf);
        }
    }

    fn selectMatrix(self: *GTE, sel: u2) [3][3]i16 {
        return switch (sel) {
            0 => self.rotation.m,
            1 => self.light.m,
            2 => self.color.m,
            3 => blk: {
                // Buggy garbage matrix
                const r: i16 = @intCast(self.rgbScaled(0));
                break :blk .{
                    .{ -r, r, self.ir[0] },
                    .{ self.rotation.m[0][2], self.rotation.m[0][2], self.rotation.m[0][2] },
                    .{ self.rotation.m[1][1], self.rotation.m[1][1], self.rotation.m[1][1] },
                };
            },
        };
    }

    fn selectVector(self: *GTE, sel: u2) [3]i16 {
        return switch (sel) {
            0 => self.v[0].toArray(),
            1 => self.v[1].toArray(),
            2 => self.v[2].toArray(),
            3 => .{ self.ir[1], self.ir[2], self.ir[3] },
        };
    }

    // =========================================================================
    // FIFO Operations
    // =========================================================================

    fn pushSXY(self: *GTE, sx: i64, sy: i64) void {
        self.sxy[0] = self.sxy[1];
        self.sxy[1] = self.sxy[2];
        self.sxy[2] = .{ self.saturateSX(sx), self.saturateSY(sy) };
    }

    fn pushSZ(self: *GTE, sz: i64) void {
        self.sz[0] = self.sz[1];
        self.sz[1] = self.sz[2];
        self.sz[2] = self.sz[3];
        self.sz[3] = self.saturateSZ(sz);
    }

    fn pushColor(self: *GTE) void {
        self.rgb_fifo[0] = self.rgb_fifo[1];
        self.rgb_fifo[1] = self.rgb_fifo[2];
        self.rgb_fifo[2] = .{
            self.saturateRGB(0, @divTrunc(self.mac[1], 16)),
            self.saturateRGB(1, @divTrunc(self.mac[2], 16)),
            self.saturateRGB(2, @divTrunc(self.mac[3], 16)),
            self.rgbc[3],
        };
    }

    // =========================================================================
    // Saturation helpers
    // =========================================================================

    fn rgbScaled(self: *GTE, comptime i: usize) i64 {
        return @as(i64, self.rgbc[i]) << 4;
    }

    fn setMACtoIR(self: *GTE, comptime i: u2, value: i64, lm: bool, sf: u1) void {
        const shift: u6 = @as(u6, sf) * 12;
        const extended = self.saturateMAC(i, value);
        const shifted = extended >> shift;
        self.mac[i] = @truncate(shifted);
        self.ir[i] = self.saturateIR(i, self.mac[i], lm);
    }

    fn setMAC0(self: *GTE, value: i64) i64 {
        self.mac[0] = self.saturateMAC0(value);
        return value;
    }

    fn saturateMAC0(self: *GTE, value: i64) i32 {
        if (value < -0x80000000) self.flag.mac0_neg_overflow = true;
        if (value > 0x7fffffff) self.flag.mac0_pos_overflow = true;
        return @truncate(value);
    }

    fn saturateMAC(self: *GTE, comptime i: u2, value: i64) i64 {
        const min = -0x80000000000;
        const max = 0x7ffffffffff;

        if (value > max) {
            switch (i) {
                1 => self.flag.mac1_pos_overflow = true,
                2 => self.flag.mac2_pos_overflow = true,
                3 => self.flag.mac3_pos_overflow = true,
                else => unreachable,
            }
        }
        if (value < min) {
            switch (i) {
                1 => self.flag.mac1_neg_overflow = true,
                2 => self.flag.mac2_neg_overflow = true,
                3 => self.flag.mac3_neg_overflow = true,
                else => unreachable,
            }
        }

        const masked: i64 = value & 0xfffffffffff;
        return (masked << 20) >> 20;
    }

    fn saturateIR(self: *GTE, comptime i: u2, value: i64, lm: bool) i16 {
        const min: i64, const max: i64 = switch (i) {
            0 => .{ 0, 0x1000 },
            1, 2, 3 => if (lm) .{ 0, 0x7FFF } else .{ -0x8000, 0x7fff },
        };

        if (value < min) {
            switch (i) {
                0 => self.flag.ir0_saturated = true,
                1 => self.flag.ir1_saturated = true,
                2 => self.flag.ir2_saturated = true,
                3 => self.flag.ir3_saturated = true,
            }
            return @intCast(min);
        }
        if (value > max) {
            switch (i) {
                0 => self.flag.ir0_saturated = true,
                1 => self.flag.ir1_saturated = true,
                2 => self.flag.ir2_saturated = true,
                3 => self.flag.ir3_saturated = true,
            }
            return @intCast(max);
        }
        return @intCast(value);
    }

    fn saturateNoFlag(value: i32, lm: bool) i16 {
        const min: i32 = if (lm) 0 else -0x8000;
        if (value < min) return @intCast(min);
        if (value > 0x7fff) return 0x7fff;
        return @truncate(value);
    }

    fn saturateSX(self: *GTE, value: i64) i16 {
        if (value < -0x400) {
            self.flag.sx2_saturated = true;
            return -0x400;
        }
        if (value > 0x3ff) {
            self.flag.sx2_saturated = true;
            return 0x3ff;
        }
        return @intCast(value);
    }

    fn saturateSY(self: *GTE, value: i64) i16 {
        if (value < -0x400) {
            self.flag.sy2_saturated = true;
            return -0x400;
        }
        if (value > 0x3ff) {
            self.flag.sy2_saturated = true;
            return 0x3ff;
        }
        return @intCast(value);
    }

    fn saturateSZ(self: *GTE, value: i64) u16 {
        if (value < 0) {
            self.flag.sz3_otz_saturated = true;
            return 0;
        }
        if (value > 0xffff) {
            self.flag.sz3_otz_saturated = true;
            return 0xffff;
        }
        return @intCast(value);
    }

    fn saturateOTZ(self: *GTE, value: i64) u16 {
        if (value < 0) {
            self.flag.sz3_otz_saturated = true;
            return 0;
        }
        if (value > 0xffff) {
            self.flag.sz3_otz_saturated = true;
            return 0xffff;
        }
        return @intCast(value);
    }

    fn saturateRGB(self: *GTE, comptime i: u2, value: i32) u8 {
        if (value < 0) {
            switch (i) {
                0 => self.flag.color_r_saturated = true,
                1 => self.flag.color_g_saturated = true,
                2 => self.flag.color_b_saturated = true,
                else => {},
            }
            return 0;
        }
        if (value > 0xff) {
            switch (i) {
                0 => self.flag.color_r_saturated = true,
                1 => self.flag.color_g_saturated = true,
                2 => self.flag.color_b_saturated = true,
                else => {},
            }
            return 0xff;
        }
        return @intCast(value);
    }

    // =========================================================================
    // Division (Newton-Raphson)
    // =========================================================================

    const unr_table = [_]u8{
        0xff, 0xfd, 0xfb, 0xf9, 0xf7, 0xf5, 0xf3, 0xf1, 0xef, 0xee, 0xec, 0xea, 0xe8, 0xe6, 0xe4, 0xe3,
        0xe1, 0xdf, 0xdd, 0xdc, 0xda, 0xd8, 0xd6, 0xd5, 0xd3, 0xd1, 0xd0, 0xce, 0xcd, 0xcb, 0xc9, 0xc8,
        0xc6, 0xc5, 0xc3, 0xc1, 0xc0, 0xbe, 0xbd, 0xbb, 0xba, 0xb8, 0xb7, 0xb5, 0xb4, 0xb2, 0xb1, 0xb0,
        0xae, 0xad, 0xab, 0xaa, 0xa9, 0xa7, 0xa6, 0xa4, 0xa3, 0xa2, 0xa0, 0x9f, 0x9e, 0x9c, 0x9b, 0x9a,
        0x99, 0x97, 0x96, 0x95, 0x94, 0x92, 0x91, 0x90, 0x8f, 0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x87, 0x86,
        0x85, 0x84, 0x83, 0x82, 0x81, 0x7f, 0x7e, 0x7d, 0x7c, 0x7b, 0x7a, 0x79, 0x78, 0x77, 0x75, 0x74,
        0x73, 0x72, 0x71, 0x70, 0x6f, 0x6e, 0x6d, 0x6c, 0x6b, 0x6a, 0x69, 0x68, 0x67, 0x66, 0x65, 0x64,
        0x63, 0x62, 0x61, 0x60, 0x5f, 0x5e, 0x5d, 0x5d, 0x5c, 0x5b, 0x5a, 0x59, 0x58, 0x57, 0x56, 0x55,
        0x54, 0x53, 0x53, 0x52, 0x51, 0x50, 0x4f, 0x4e, 0x4d, 0x4d, 0x4c, 0x4b, 0x4a, 0x49, 0x48, 0x48,
        0x47, 0x46, 0x45, 0x44, 0x43, 0x43, 0x42, 0x41, 0x40, 0x3f, 0x3f, 0x3e, 0x3d, 0x3c, 0x3c, 0x3b,
        0x3a, 0x39, 0x39, 0x38, 0x37, 0x36, 0x36, 0x35, 0x34, 0x33, 0x33, 0x32, 0x31, 0x31, 0x30, 0x2f,
        0x2e, 0x2e, 0x2d, 0x2c, 0x2c, 0x2b, 0x2a, 0x2a, 0x29, 0x28, 0x28, 0x27, 0x26, 0x26, 0x25, 0x24,
        0x24, 0x23, 0x22, 0x22, 0x21, 0x20, 0x20, 0x1f, 0x1e, 0x1e, 0x1d, 0x1d, 0x1c, 0x1b, 0x1b, 0x1a,
        0x19, 0x19, 0x18, 0x18, 0x17, 0x16, 0x16, 0x15, 0x15, 0x14, 0x14, 0x13, 0x12, 0x12, 0x11, 0x11,
        0x10, 0x0f, 0x0f, 0x0e, 0x0e, 0x0d, 0x0d, 0x0c, 0x0c, 0x0b, 0x0a, 0x0a, 0x09, 0x09, 0x08, 0x08,
        0x07, 0x07, 0x06, 0x06, 0x05, 0x05, 0x04, 0x04, 0x03, 0x03, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00,
        0x00, // extra entry for index 0x100
    };

    fn unrDivide(self: *GTE, h: u16, sz3: u16) u32 {
        if (sz3 == 0 or h >= @as(u32, sz3) * 2) {
            self.flag.divide_overflow = true;
            return 0x1ffff;
        }

        const z: u5 = @clz(sz3);
        const n: u64 = @as(u64, h) << z;
        var d: u32 = @as(u32, sz3) << z;

        const table_idx: usize = (d - 0x7fc0) >> 7;
        const u: u32 = @as(u32, unr_table[table_idx]) + 0x101;

        d = (0x2000080 -% (d *% u)) >> 8;
        d = (0x80 +% (d *% u)) >> 8;

        var result: u32 = @truncate((n * d + 0x8000) >> 16);
        if (result > 0x1ffff) result = 0x1ffff;

        return result;
    }
};
