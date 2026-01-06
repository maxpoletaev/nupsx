const std = @import("std");
const bits = @import("bits.zig");

const log = std.log.scoped(.gte);

const Vertex = struct {
    x: i16,
    y: i16,
    z: i16,

    pub fn init(x: i16, y: i16, z: i16) @This() {
        return .{ .x = x, .y = y, .z = z };
    }

    pub fn getWord(self: @This(), comptime word_i: u8) u32 {
        switch (word_i) {
            0 => {
                const x_bits: u16 = @bitCast(self.x);
                const y_bits: u16 = @bitCast(self.y);
                return @as(u32, x_bits) | (@as(u32, y_bits) << 16);
            },
            1 => return @bitCast(@as(i32, self.z)),
            else => @compileError("invalid word index"),
        }
    }

    pub fn setWord(self: *@This(), comptime word_i: u8, v: u32) void {
        switch (word_i) {
            0 => {
                self.x = @bitCast(bits.field(v, 0, u16));
                self.y = @bitCast(bits.field(v, 16, u16));
            },
            1 => {
                self.z = @bitCast(bits.field(v, 0, u16));
            },
            else => @compileError("invalid word index"),
        }
    }
};

const Matrix = struct {
    m: [3][3]i16,

    fn getWord(self: @This(), comptime word_i: u8) u32 {
        switch (word_i) {
            0 => {
                const v0: u16 = @bitCast(self.m[0][0]);
                const v1: u16 = @bitCast(self.m[0][1]);
                return @as(u32, v0) | (@as(u32, v1) << 16);
            },
            1 => {
                const v0: u16 = @bitCast(self.m[0][2]);
                const v1: u16 = @bitCast(self.m[1][0]);
                return @as(u32, v0) | (@as(u32, v1) << 16);
            },
            2 => {
                const v0: u16 = @bitCast(self.m[1][1]);
                const v1: u16 = @bitCast(self.m[1][2]);
                return @as(u32, v0) | (@as(u32, v1) << 16);
            },
            3 => {
                const v0: u16 = @bitCast(self.m[2][0]);
                const v1: u16 = @bitCast(self.m[2][1]);
                return @as(u32, v0) | (@as(u32, v1) << 16);
            },
            4 => {
                const v0: i16 = @bitCast(self.m[2][2]);
                return @bitCast(@as(i32, v0));
            },
            else => @compileError("invalid word index"),
        }
    }

    fn setWord(self: *@This(), comptime word_i: u8, v: u32) void {
        switch (word_i) {
            0 => {
                self.m[0][0] = @bitCast(bits.field(v, 0, u16));
                self.m[0][1] = @bitCast(bits.field(v, 16, u16));
            },
            1 => {
                self.m[0][2] = @bitCast(bits.field(v, 0, u16));
                self.m[1][0] = @bitCast(bits.field(v, 16, u16));
            },
            2 => {
                self.m[1][1] = @bitCast(bits.field(v, 0, u16));
                self.m[1][2] = @bitCast(bits.field(v, 16, u16));
            },
            3 => {
                self.m[2][0] = @bitCast(bits.field(v, 0, u16));
                self.m[2][1] = @bitCast(bits.field(v, 16, u16));
            },
            4 => {
                self.m[2][2] = @bitCast(bits.field(v, 0, u16));
            },
            else => @compileError("invalid word index"),
        }
    }
};

const Command = packed struct(u32) {
    opcode: u6, // 0-5
    _pad0: u4, // 6-9
    lm: bool, // 10
    _pad1: u2, // 11-12
    mvmva_tv: enum(u2) { tr = 1, bk = 2, fc = 3 }, // 13-14
    mvmva_mv: enum(u2) { v0 = 0, v1 = 1, v2 = 2, ir = 3 }, // 15-16
    mvmva_mm: enum(u2) { rot = 0, light = 1, color = 2 }, // 17-18
    sf: u1, // 19
    _pad2: u12, // 20-24
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

pub const GTE = struct {
    // Data registers
    V0: Vertex, // 0-1
    V1: Vertex, // 2-3
    V2: Vertex, // 4-5
    RGBC: [4]u8, // 6
    OTZ: u16, // 7
    IR0: i16, // 8
    IR1: i16, // 9
    IR2: i16, // 10
    IR3: i16, // 11
    SXY0: [2]i16, // 12
    SXY1: [2]i16, // 13
    SXY2: [2]i16, // 14
    // SXYP: 15 (mirrors SXY2)
    SZ0: u16, // 16
    SZ1: u16, // 17
    SZ2: u16, // 18
    SZ3: u16, // 19
    RGB0: [4]u8, // 20
    RGB1: [4]u8, // 21
    RGB2: [4]u8, // 22
    RES1: u32, // 23 (prohibited register)
    MAC0: i32, // 24
    MAC1: i32, // 25
    MAC2: i32, // 26
    MAC3: i32, // 27
    // IRGB: u16, // 28 (constructed from IR1/IR2/IR3)
    // ORGB: u16, // 29 (constructed from IR1/IR2/IR3)
    LZCS: i32, // 30
    // LZCR: i32, // 31 (read-only)

    // Control registers
    RT: Matrix, // 32-36
    TRX: i32, // 37
    TRY: i32, // 38
    TRZ: i32, // 39
    LLM: Matrix, // 40-44
    BK: [3]u32, // 45-47
    LCM: Matrix, // 48-52
    RFC: u32, // 53
    GFC: u32, // 54
    BFC: u32, // 55
    OFX: i32, // 56
    OFY: i32, // 57
    H: u16, // 58
    DQA: i16, // 59
    DQB: u32, // 60
    ZSF3: i16, // 61
    ZSF4: i16, // 62
    FLAG: Flags, // 63

    pub fn init() @This() {
        return std.mem.zeroInit(@This(), .{});
    }

    // ------------------------
    // Register reads
    // ------------------------

    pub fn readReg(self: *@This(), reg: u8) u32 {
        const v: u32 = switch (reg) {
            // Data registers (0-31)
            0 => self.V0.getWord(0),
            1 => self.V0.getWord(1),
            2 => self.V1.getWord(0),
            3 => self.V1.getWord(1),
            4 => self.V2.getWord(0),
            5 => self.V2.getWord(1),
            6 => @bitCast(self.RGBC),
            7 => self.OTZ,
            8 => @bitCast(@as(i32, self.IR0)),
            9 => @bitCast(@as(i32, self.IR1)),
            10 => @bitCast(@as(i32, self.IR2)),
            11 => @bitCast(@as(i32, self.IR3)),
            12 => @bitCast(self.SXY0),
            13 => @bitCast(self.SXY1),
            14 => @bitCast(self.SXY2),
            15 => @bitCast(self.SXY2), // SXYP same as SXY2
            16 => @as(u32, self.SZ0),
            17 => @as(u32, self.SZ1),
            18 => @as(u32, self.SZ2),
            19 => @as(u32, self.SZ3),
            20 => @bitCast(self.RGB0),
            21 => @bitCast(self.RGB1),
            22 => @bitCast(self.RGB2),
            23 => self.RES1, // Prohibited register?
            24 => @bitCast(self.MAC0),
            25 => @bitCast(self.MAC1),
            26 => @bitCast(self.MAC2),
            27 => @bitCast(self.MAC3),
            28, 29 => self.readORGB(), // IRGB/ORGB computed from IR1/IR2/IR3
            30 => @bitCast(self.LZCS),
            31 => self.readLZCS(),

            // Control registers (32-63)
            32 => self.RT.getWord(0),
            33 => self.RT.getWord(1),
            34 => self.RT.getWord(2),
            35 => self.RT.getWord(3),
            36 => self.RT.getWord(4),
            37 => @bitCast(self.TRX),
            38 => @bitCast(self.TRY),
            39 => @bitCast(self.TRZ),
            40 => self.LLM.getWord(0),
            41 => self.LLM.getWord(1),
            42 => self.LLM.getWord(2),
            43 => self.LLM.getWord(3),
            44 => self.LLM.getWord(4),
            45 => self.BK[0],
            46 => self.BK[1],
            47 => self.BK[2],
            48 => self.LCM.getWord(0),
            49 => self.LCM.getWord(1),
            50 => self.LCM.getWord(2),
            51 => self.LCM.getWord(3),
            52 => self.LCM.getWord(4),
            53 => self.RFC,
            54 => self.GFC,
            55 => self.BFC,
            56 => @bitCast(self.OFX),
            57 => @bitCast(self.OFY),
            58 => @bitCast(@as(i32, @as(i16, @bitCast(self.H)))), // unsigned but sign-extended?
            59 => @bitCast(@as(i32, self.DQA)),
            60 => self.DQB,
            61 => @bitCast(@as(i32, self.ZSF3)),
            62 => @bitCast(@as(i32, self.ZSF4)),
            63 => self.readFLAG(),

            else => std.debug.panic("unhandled GTE read from reg {d}", .{reg}),
        };
        // log.debug("GTE read reg {d} = {x}", .{ reg, v });
        return v;
    }

    fn readLZCS(self: *@This()) u32 {
        var v = @as(u32, @bitCast(self.LZCS));
        if (self.LZCS < 0) v = ~v;
        return @clz(v);
    }

    fn readORGB(self: *@This()) u32 {
        const r = clampRGB5(self.IR1 >> 7);
        const g = clampRGB5(self.IR2 >> 7);
        const b = clampRGB5(self.IR3 >> 7);
        return @as(u32, r) | (@as(u32, g) << 5) | (@as(u32, b) << 10);
    }

    fn readFLAG(self: *@This()) u32 {
        var flags = self.FLAG;
        const v: u32 = @bitCast(self.FLAG);
        flags.error_flag = (v & 0x7f87e000) != 0; // bits 13-18 and 23-30
        return @bitCast(flags);
    }

    // ------------------------
    // Register writes
    // ------------------------

    pub fn writeReg(self: *@This(), reg: u8, v: u32) void {
        // log.debug("GTE write reg {d} = {x}", .{ reg, v });
        switch (reg) {
            // Data registers (0-31)
            0 => self.V0.setWord(0, v),
            1 => self.V0.setWord(1, v),
            2 => self.V1.setWord(0, v),
            3 => self.V1.setWord(1, v),
            4 => self.V2.setWord(0, v),
            5 => self.V2.setWord(1, v),
            6 => self.RGBC = @bitCast(v),
            7 => self.OTZ = @truncate(v),
            8 => self.IR0 = @truncate(@as(i32, @bitCast(v))),
            9 => self.IR1 = @truncate(@as(i32, @bitCast(v))),
            10 => self.IR2 = @truncate(@as(i32, @bitCast(v))),
            11 => self.IR3 = @truncate(@as(i32, @bitCast(v))),
            12 => self.SXY0 = @bitCast(v),
            13 => self.SXY1 = @bitCast(v),
            14 => self.SXY2 = @bitCast(v),
            15 => {
                // SXYP - pushes to FIFO
                self.SXY0 = self.SXY1;
                self.SXY1 = self.SXY2;
                self.SXY2 = @bitCast(v);
            },
            16 => self.SZ0 = @truncate(v),
            17 => self.SZ1 = @truncate(v),
            18 => self.SZ2 = @truncate(v),
            19 => self.SZ3 = @truncate(v),
            20 => self.RGB0 = @bitCast(v),
            21 => self.RGB1 = @bitCast(v),
            22 => self.RGB2 = @bitCast(v),
            23 => self.RES1 = v, // Prohibited register?
            24 => self.MAC0 = @bitCast(v),
            25 => self.MAC1 = @bitCast(v),
            26 => self.MAC2 = @bitCast(v),
            27 => self.MAC3 = @bitCast(v),
            28 => self.writeIRGB(v),
            29 => {}, // ORGB is read-only copy of IRGB
            30 => self.LZCS = @bitCast(v),
            31 => {}, // LZCR is read-only

            // Control registers (32-63)
            32 => self.RT.setWord(0, v),
            33 => self.RT.setWord(1, v),
            34 => self.RT.setWord(2, v),
            35 => self.RT.setWord(3, v),
            36 => self.RT.setWord(4, v),
            37 => self.TRX = @bitCast(v),
            38 => self.TRY = @bitCast(v),
            39 => self.TRZ = @bitCast(v),
            40 => self.LLM.setWord(0, v),
            41 => self.LLM.setWord(1, v),
            42 => self.LLM.setWord(2, v),
            43 => self.LLM.setWord(3, v),
            44 => self.LLM.setWord(4, v),
            45 => self.BK[0] = v,
            46 => self.BK[1] = v,
            47 => self.BK[2] = v,
            48 => self.LCM.setWord(0, v),
            49 => self.LCM.setWord(1, v),
            50 => self.LCM.setWord(2, v),
            51 => self.LCM.setWord(3, v),
            52 => self.LCM.setWord(4, v),
            53 => self.RFC = v,
            54 => self.GFC = v,
            55 => self.BFC = v,
            56 => self.OFX = @bitCast(v),
            57 => self.OFY = @bitCast(v),
            58 => self.H = @truncate(v),
            59 => self.DQA = bits.field(v, 0, i16),
            60 => self.DQB = v,
            61 => self.ZSF3 = bits.field(v, 0, i16),
            62 => self.ZSF4 = bits.field(v, 0, i16),
            63 => self.FLAG = @bitCast(v & ~@as(u32, 0xfff)), // upper 12 bits are read-only

            else => std.debug.panic("unhandled GTE write to reg {d} value {x}", .{ reg, v }),
        }
    }

    fn writeIRGB(self: *@This(), v: u32) void {
        self.IR1 = bits.field(((v >> 0) & 0x1F) * 0x80, 0, i16);
        self.IR2 = bits.field(((v >> 5) & 0x1F) * 0x80, 0, i16);
        self.IR3 = bits.field(((v >> 10) & 0x1F) * 0x80, 0, i16);
    }

    // ------------------------
    // Setters and helpers
    // ------------------------

    fn clampRGB5(v: i16) u5 {
        if (v < 0) return 0;
        if (v > 0x1f) return 0x1f;
        return @intCast(v);
    }

    fn clampOTZ(self: *@This(), v: i64) u16 {
        if (v < 0) {
            return 0;
        } else if (v > 0xffff) {
            self.FLAG.sz3_otz_saturated = true;
            return 0xffff;
        } else {
            return @truncate(@as(u64, @bitCast(v)));
        }
    }

    fn setOTZ(self: *@This(), v: i64) u16 {
        self.OTZ = self.clampOTZ(v);
        return self.OTZ;
    }

    fn clampMAC(self: *@This(), comptime mac_i: u2, v: i64) i32 {
        const min = if (mac_i == 0) -0x80000000 else -0x80000000000;
        const max = if (mac_i == 0) 0x7fffffff else 0x7ffffffffff;

        if (v < min) {
            switch (mac_i) {
                0 => self.FLAG.mac0_neg_overflow = true,
                1 => self.FLAG.mac1_neg_overflow = true,
                2 => self.FLAG.mac2_neg_overflow = true,
                3 => self.FLAG.mac3_neg_overflow = true,
            }
        } else if (v > max) {
            switch (mac_i) {
                0 => self.FLAG.mac0_pos_overflow = true,
                1 => self.FLAG.mac1_pos_overflow = true,
                2 => self.FLAG.mac2_pos_overflow = true,
                3 => self.FLAG.mac3_pos_overflow = true,
            }
        }

        const v_u64: u64 = @as(u64, @bitCast(v));
        const res: i32 = @bitCast(@as(u32, @truncate(v_u64)));
        return res;
    }

    fn setMAC(self: *@This(), comptime mac_i: u2, v: i64) i32 {
        const res: i32 = self.clampMAC(mac_i, v);
        switch (mac_i) {
            0 => self.MAC0 = res,
            1 => self.MAC1 = res,
            2 => self.MAC2 = res,
            3 => self.MAC3 = res,
        }
        return res;
    }

    fn clampIR(self: *@This(), comptime ir_i: u2, v: i64, lm: bool) i16 {
        const min: i64 = if (lm) 0x0000 else -0x8000;
        const max: i64 = 0x7fff;
        var res: i16 = undefined;

        if (v < min) {
            res = @intCast(min);
            switch (ir_i) {
                0 => self.FLAG.ir0_saturated = true,
                1 => self.FLAG.ir1_saturated = true,
                2 => self.FLAG.ir2_saturated = true,
                3 => self.FLAG.ir3_saturated = true,
            }
        } else if (v > max) {
            res = @intCast(max);
            switch (ir_i) {
                0 => self.FLAG.ir0_saturated = true,
                1 => self.FLAG.ir1_saturated = true,
                2 => self.FLAG.ir2_saturated = true,
                3 => self.FLAG.ir3_saturated = true,
            }
        } else {
            res = @intCast(v);
        }
        return res;
    }

    fn setIR(self: *@This(), comptime ir_i: u2, v: i64, lm: bool) i16 {
        const res: i16 = self.clampIR(ir_i, v, lm);
        switch (ir_i) {
            0 => self.IR0 = res,
            1 => self.IR1 = res,
            2 => self.IR2 = res,
            3 => self.IR3 = res,
        }
        return res;
    }

    fn pushSXY(self: *@This(), sx: i64, sy: i64) void {
        self.SXY0 = self.SXY1;
        self.SXY1 = self.SXY2;

        if (sx < -0x400) {
            self.SXY2[0] = -0x400;
            self.FLAG.sx2_saturated = true;
        } else if (sx > 0x3ff) {
            self.SXY2[0] = 0x3ff;
            self.FLAG.sx2_saturated = true;
        } else {
            self.SXY2[0] = @truncate(sx);
        }

        if (sy < -0x400) {
            self.SXY2[1] = -0x400;
            self.FLAG.sy2_saturated = true;
        } else if (sy > 0x3ff) {
            self.SXY2[1] = 0x3ff;
            self.FLAG.sy2_saturated = true;
        } else {
            self.SXY2[1] = @truncate(sy);
        }
    }

    fn pushSZ(self: *@This(), sz: i64) void {
        self.SZ0 = self.SZ1;
        self.SZ1 = self.SZ2;
        self.SZ2 = self.SZ3;

        if (sz < 0) {
            self.SZ3 = 0;
            self.FLAG.sz3_otz_saturated = true;
        } else if (sz > 0xffff) {
            self.SZ3 = 0xffff;
            self.FLAG.sz3_otz_saturated = true;
        } else {
            self.SZ3 = @truncate(@as(u64, @bitCast(sz)));
        }
    }

    // ------------------------
    // GTE Commands
    // ------------------------

    pub fn exec(self: *@This(), v: u32) void {
        const cmd = @as(Command, @bitCast(v));
        self.FLAG.reset();

        switch (cmd.opcode) {
            0x00 => {}, // NOP
            0x01 => self.rtps(cmd, 0),
            0x06 => self.nclip(cmd),
            0x13 => self.ncds(cmd),
            0x2d => self.avsz3(cmd),
            0x2e => self.avsz4(cmd),
            0x30 => self.rtpt(cmd),
            // else => {},
            else => log.warn("unhandled GTE command {x}", .{cmd.opcode}),
            // else => std.debug.panic("unhandled GTE command opcode 0x{x:0>2}", .{cmd.opcode}),
        }
    }

    /// Normal clip
    /// MAC0 = SX0*(SY1-SY2) + SX1*(SY2-SY0) + SX2*(SY0-SY1)
    fn nclip(self: *@This(), _: Command) void {
        const sx0: i64 = self.SXY0[0];
        const sy0: i64 = self.SXY0[1];
        const sx1: i64 = self.SXY1[0];
        const sy1: i64 = self.SXY1[1];
        const sx2: i64 = self.SXY2[0];
        const sy2: i64 = self.SXY2[1];

        const mac0 = self.setMAC(0, sx0 * (sy1 - sy2) +
            sx1 * (sy2 - sy0) +
            sx2 * (sy0 - sy1));

        _ = self.setMAC(0, mac0);
    }

    /// Rotate, Translate, and Perspective Transformation of a Single Vertex
    /// IR1 = MAC1 = (TRX*1000h + RT11*VX0 + RT12*VY0 + RT13*VZ0) SAR (sf*12)
    //  IR2 = MAC2 = (TRY*1000h + RT21*VX0 + RT22*VY0 + RT23*VZ0) SAR (sf*12)
    //  IR3 = MAC3 = (TRZ*1000h + RT31*VX0 + RT32*VY0 + RT33*VZ0) SAR (sf*12)
    //  SZ3 = MAC3 SAR ((1-sf)*12)
    //  MAC0=(((H*20000h/SZ3)+1)/2)*IR1+OFX, SX2=MAC0/10000h
    //  MAC0=(((H*20000h/SZ3)+1)/2)*IR2+OFY, SY2=MAC0/10000h
    //  MAC0=(((H*20000h/SZ3)+1)/2)*DQA+DQB, IR0=MAC0/1000h
    fn rtps(self: *@This(), cmd: Command, v_i: u2) void {
        const v = switch (v_i) {
            0 => self.V0,
            1 => self.V1,
            2 => self.V2,
            else => unreachable,
        };

        const vx0: i64 = v.x;
        const vy0: i64 = v.y;
        const vz0: i64 = v.z;

        const rt11: i64 = self.RT.m[0][0];
        const rt12: i64 = self.RT.m[0][1];
        const rt13: i64 = self.RT.m[0][2];
        const rt21: i64 = self.RT.m[1][0];
        const rt22: i64 = self.RT.m[1][1];
        const rt23: i64 = self.RT.m[1][2];
        const rt31: i64 = self.RT.m[2][0];
        const rt32: i64 = self.RT.m[2][1];
        const rt33: i64 = self.RT.m[2][2];

        const mac_shift = @as(u6, cmd.sf) * 12;
        const mac1 = (self.TRX *% 0x1000 +% rt11 *% vx0 + rt12 *% vy0 +% rt13 *% vz0) >> mac_shift;
        const mac2 = (self.TRY *% 0x1000 +% rt21 *% vx0 + rt22 *% vy0 +% rt23 *% vz0) >> mac_shift;
        const mac3 = (self.TRZ *% 0x1000 +% rt31 *% vx0 + rt32 *% vy0 +% rt33 *% vz0) >> mac_shift;

        self.MAC1 = self.clampMAC(1, mac1);
        self.MAC2 = self.clampMAC(2, mac2);
        self.MAC3 = self.clampMAC(3, mac3);

        self.IR1 = self.clampIR(1, mac1, cmd.lm);
        self.IR2 = self.clampIR(2, mac2, cmd.lm);
        self.IR3 = self.clampIR(3, mac3, cmd.lm);

        const cz_shift = @as(u5, 1 - cmd.sf) * 12;
        const sz3 = self.MAC3 >> cz_shift;
        self.pushSZ(sz3);

        const ofx: i64 = self.OFX;
        const ofy: i64 = self.OFY;
        const dqa: i64 = self.DQA;
        const dqb: i64 = @as(i64, @as(i32, @bitCast(self.DQB)));

        const div: i64 = self.unrDivide(self.H, self.SZ3);

        var mac0 = div * self.IR1 + ofx;
        const sx2 = @divTrunc(mac0, 0x10000);
        mac0 = div * self.IR2 + ofy;
        const sy2 = @divTrunc(mac0, 0x10000);
        mac0 = div * dqa + dqb;
        const ir0 = @divTrunc(mac0, 0x1000);

        self.MAC0 = self.clampMAC(0, mac0);
        self.IR0 = self.clampIR(0, ir0, cmd.lm);
        self.pushSXY(sx2, sy2);
    }

    fn rtpt(self: *@This(), cmd: Command) void {
        self.rtps(cmd, 0);
        self.rtps(cmd, 1);
        self.rtps(cmd, 2);
    }

    /// Average Z of 3 vertices
    /// MAC0 = ZSF3*(SZ1+SZ2+SZ3)
    /// OTZ = MAC0/1000h
    fn avsz3(self: *@This(), _: Command) void {
        const sz0: i64 = self.SZ0;
        const sz1: i64 = self.SZ1;
        const sz2: i64 = self.SZ2;
        const zsf3: i64 = self.ZSF3;

        const mac0 = zsf3 * (sz0 + sz1 + sz2);
        const otz = @divTrunc(mac0, 0x1000);

        _ = self.setMAC(0, mac0);
        _ = self.setOTZ(otz);
    }

    /// Average Z of 4 vertices
    /// MAC0 = ZSF4*(SZ0+SZ1+SZ2+SZ3)
    /// OTZ  = MAC0/1000h
    fn avsz4(self: *@This(), _: Command) void {
        const sz0: i64 = self.SZ0;
        const sz1: i64 = self.SZ1;
        const sz2: i64 = self.SZ2;
        const sz3: i64 = self.SZ3;
        const zsf4: i64 = self.ZSF4;

        const mac0 = zsf4 * (sz0 + sz1 + sz2 + sz3);
        const otz = @divTrunc(mac0, 0x1000);

        _ = self.setMAC(0, mac0);
        _ = self.setOTZ(otz);
    }

    // ------------------------
    // Helper functions
    // ------------------------

    fn matrixMultiply(self: *@This(), matrix: Matrix, vec: Vertex, lm: bool, shift: u6) void {
        const vx: i64 = vec.x;
        const vy: i64 = vec.y;
        const vz: i64 = vec.z;

        const m11: i64 = matrix.m[0][0];
        const m12: i64 = matrix.m[0][1];
        const m13: i64 = matrix.m[0][2];
        const m21: i64 = matrix.m[1][0];
        const m22: i64 = matrix.m[1][1];
        const m23: i64 = matrix.m[1][2];
        const m31: i64 = matrix.m[2][0];
        const m32: i64 = matrix.m[2][1];
        const m33: i64 = matrix.m[2][2];

        const mac1 = (m11 * vx + m12 * vy + m13 * vz) >> shift;
        const mac2 = (m21 * vx + m22 * vy + m23 * vz) >> shift;
        const mac3 = (m31 * vx + m32 * vy + m33 * vz) >> shift;

        self.MAC1 = self.clampMAC(1, mac1);
        self.MAC2 = self.clampMAC(2, mac2);
        self.MAC3 = self.clampMAC(3, mac3);

        self.IR1 = self.clampIR(1, mac1, lm);
        self.IR2 = self.clampIR(2, mac2, lm);
        self.IR3 = self.clampIR(3, mac3, lm);
    }

    fn clampRGB8(self: *@This(), comptime color_i: u2, v: i32) u8 {
        if (v < 0) {
            return 0;
        } else if (v > 0xff) {
            switch (color_i) {
                0 => self.FLAG.color_r_saturated = true,
                1 => self.FLAG.color_g_saturated = true,
                2 => self.FLAG.color_b_saturated = true,
                3 => {},
            }
            return 0xff;
        } else {
            return @intCast(v);
        }
    }

    fn pushColorFIFO(self: *@This()) void {
        self.RGB0 = self.RGB1;
        self.RGB1 = self.RGB2;

        // Divide MAC by 16 (arithmetic right shift handles sign correctly)
        const r = self.clampRGB8(0, @divTrunc(self.MAC1, 16));
        const g = self.clampRGB8(1, @divTrunc(self.MAC2, 16));
        const b = self.clampRGB8(2, @divTrunc(self.MAC3, 16));
        const code = self.RGBC[3];

        self.RGB2 = .{ r, g, b, code };
    }

    fn ncds(self: *@This(), cmd: Command) void {
        const v0 = self.V0;

        // [IR1,IR2,IR3] = [MAC1,MAC2,MAC3] = (LLM*V0) SAR (sf*12)
        const shift: u6 = @as(u6, cmd.sf) * 12;
        self.matrixMultiply(self.LLM, v0, cmd.lm, shift);

        // [IR1,IR2,IR3] = [MAC1,MAC2,MAC3] = (BK*1000h + LCM*IR) SAR (sf*12)
        const bk1: i64 = @as(i64, @as(i32, @bitCast(self.BK[0])));
        const bk2: i64 = @as(i64, @as(i32, @bitCast(self.BK[1])));
        const bk3: i64 = @as(i64, @as(i32, @bitCast(self.BK[2])));

        const ir_vec = Vertex.init(self.IR1, self.IR2, self.IR3);
        const lcm_ir1: i64 = self.LCM.m[0][0] *% ir_vec.x +% self.LCM.m[0][1] *% ir_vec.y +% self.LCM.m[0][2] *% ir_vec.z;
        const lcm_ir2: i64 = self.LCM.m[1][0] *% ir_vec.x +% self.LCM.m[1][1] *% ir_vec.y +% self.LCM.m[1][2] *% ir_vec.z;
        const lcm_ir3: i64 = self.LCM.m[2][0] *% ir_vec.x +% self.LCM.m[2][1] *% ir_vec.y +% self.LCM.m[2][2] *% ir_vec.z;

        var mac1 = (bk1 *% 0x1000 +% lcm_ir1) >> shift;
        var mac2 = (bk2 *% 0x1000 +% lcm_ir2) >> shift;
        var mac3 = (bk3 *% 0x1000 +% lcm_ir3) >> shift;

        self.MAC1 = self.clampMAC(1, mac1);
        self.MAC2 = self.clampMAC(2, mac2);
        self.MAC3 = self.clampMAC(3, mac3);

        self.IR1 = self.clampIR(1, mac1, cmd.lm);
        self.IR2 = self.clampIR(2, mac2, cmd.lm);
        self.IR3 = self.clampIR(3, mac3, cmd.lm);

        // [MAC1,MAC2,MAC3] = [R*IR1,G*IR2,B*IR3] SHL 4
        const r: i64 = self.RGBC[0];
        const g: i64 = self.RGBC[1];
        const b: i64 = self.RGBC[2];

        mac1 = (r * self.IR1) << 4;
        mac2 = (g * self.IR2) << 4;
        mac3 = (b * self.IR3) << 4;

        self.MAC1 = self.clampMAC(1, mac1);
        self.MAC2 = self.clampMAC(2, mac2);
        self.MAC3 = self.clampMAC(3, mac3);

        // [MAC1,MAC2,MAC3] = MAC+(FC-MAC)*IR0
        const rfc: i64 = @as(i64, @as(i32, @bitCast(self.RFC)));
        const gfc: i64 = @as(i64, @as(i32, @bitCast(self.GFC)));
        const bfc: i64 = @as(i64, @as(i32, @bitCast(self.BFC)));

        const fc_mac1 = ((rfc << 12) - mac1) >> shift;
        const fc_mac2 = ((gfc << 12) - mac2) >> shift;
        const fc_mac3 = ((bfc << 12) - mac3) >> shift;

        self.IR1 = self.clampIR(1, fc_mac1, false);
        self.IR2 = self.clampIR(2, fc_mac2, false);
        self.IR3 = self.clampIR(3, fc_mac3, false);

        const ir0: i64 = self.IR0;
        self.MAC1 = self.clampMAC(1, @as(i64, self.IR1) * ir0 + mac1);
        self.MAC2 = self.clampMAC(2, @as(i64, self.IR2) * ir0 + mac2);
        self.MAC3 = self.clampMAC(3, @as(i64, self.IR3) * ir0 + mac3);

        // [MAC1,MAC2,MAC3] = [MAC1,MAC2,MAC3] SAR (sf*12)
        self.MAC1 = self.clampMAC(1, @as(i64, self.MAC1) >> shift);
        self.MAC2 = self.clampMAC(2, @as(i64, self.MAC2) >> shift);
        self.MAC3 = self.clampMAC(3, @as(i64, self.MAC3) >> shift);

        // [IR1,IR2,IR3] = [MAC1,MAC2,MAC3]
        self.IR1 = self.clampIR(1, self.MAC1, cmd.lm);
        self.IR2 = self.clampIR(2, self.MAC2, cmd.lm);
        self.IR3 = self.clampIR(3, self.MAC3, cmd.lm);

        self.pushColorFIFO();
    }

    // ------------------------
    // Newton-Raphson Division
    // ------------------------

    const unr_table = [_]u8{
        0xFF, 0xFD, 0xFB, 0xF9, 0xF7, 0xF5, 0xF3, 0xF1, 0xEF, 0xEE, 0xEC, 0xEA, 0xE8, 0xE6, 0xE4, 0xE3,
        0xE1, 0xDF, 0xDD, 0xDC, 0xDA, 0xD8, 0xD6, 0xD5, 0xD3, 0xD1, 0xD0, 0xCE, 0xCD, 0xCB, 0xC9, 0xC8,
        0xC6, 0xC5, 0xC3, 0xC1, 0xC0, 0xBE, 0xBD, 0xBB, 0xBA, 0xB8, 0xB7, 0xB5, 0xB4, 0xB2, 0xB1, 0xB0,
        0xAE, 0xAD, 0xAB, 0xAA, 0xA9, 0xA7, 0xA6, 0xA4, 0xA3, 0xA2, 0xA0, 0x9F, 0x9E, 0x9C, 0x9B, 0x9A,
        0x99, 0x97, 0x96, 0x95, 0x94, 0x92, 0x91, 0x90, 0x8F, 0x8D, 0x8C, 0x8B, 0x8A, 0x89, 0x87, 0x86,
        0x85, 0x84, 0x83, 0x82, 0x81, 0x7F, 0x7E, 0x7D, 0x7C, 0x7B, 0x7A, 0x79, 0x78, 0x77, 0x75, 0x74,
        0x73, 0x72, 0x71, 0x70, 0x6F, 0x6E, 0x6D, 0x6C, 0x6B, 0x6A, 0x69, 0x68, 0x67, 0x66, 0x65, 0x64,
        0x63, 0x62, 0x61, 0x60, 0x5F, 0x5E, 0x5D, 0x5D, 0x5C, 0x5B, 0x5A, 0x59, 0x58, 0x57, 0x56, 0x55,
        0x54, 0x53, 0x53, 0x52, 0x51, 0x50, 0x4F, 0x4E, 0x4D, 0x4D, 0x4C, 0x4B, 0x4A, 0x49, 0x48, 0x48,
        0x47, 0x46, 0x45, 0x44, 0x43, 0x43, 0x42, 0x41, 0x40, 0x3F, 0x3F, 0x3E, 0x3D, 0x3C, 0x3C, 0x3B,
        0x3A, 0x39, 0x39, 0x38, 0x37, 0x36, 0x36, 0x35, 0x34, 0x33, 0x33, 0x32, 0x31, 0x31, 0x30, 0x2F,
        0x2E, 0x2E, 0x2D, 0x2C, 0x2C, 0x2B, 0x2A, 0x2A, 0x29, 0x28, 0x28, 0x27, 0x26, 0x26, 0x25, 0x24,
        0x24, 0x23, 0x22, 0x22, 0x21, 0x20, 0x20, 0x1F, 0x1E, 0x1E, 0x1D, 0x1D, 0x1C, 0x1B, 0x1B, 0x1A,
        0x19, 0x19, 0x18, 0x18, 0x17, 0x16, 0x16, 0x15, 0x15, 0x14, 0x14, 0x13, 0x12, 0x12, 0x11, 0x11,
        0x10, 0x0F, 0x0F, 0x0E, 0x0E, 0x0D, 0x0D, 0x0C, 0x0C, 0x0B, 0x0A, 0x0A, 0x09, 0x09, 0x08, 0x08,
        0x07, 0x07, 0x06, 0x06, 0x05, 0x05, 0x04, 0x04, 0x03, 0x03, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00,
        0x00, // extra entry for index 0x100
    };

    fn unrDivide(self: *@This(), h: u16, sz3: u16) u32 {
        if (h >= @as(u32, sz3) * 2) {
            self.FLAG.divide_overflow = true;
            return 0x1ffff;
        }
        if (sz3 == 0) {
            self.FLAG.divide_overflow = true;
            return 0x1ffff;
        }

        const z: u5 = @clz(sz3); // z = count_leading_zeroes(SZ3)
        const n: u64 = @as(u64, h) << z; // n = (H SHL z)
        var d: u32 = @as(u32, sz3) << z; // d = (SZ3 SHL z)

        const table_idx: usize = (d - 0x7fc0) >> 7;
        const u: u32 = @as(u32, unr_table[table_idx]) + 0x101; // u = unr_table[(d-7FC0h) SHR 7] + 101h

        d = (0x2000080 -% (d *% u)) >> 8; // d = ((2000080h - (d * u)) SHR 8)
        d = (0x80 +% (d *% u)) >> 8; // d = ((0000080h + (d * u)) SHR 8)

        var v: u32 = @truncate((n * d + 0x8000) >> 16);
        if (v > 0x1ffff) v = 0x1ffff;

        return v;
    }
};
