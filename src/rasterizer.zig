const std = @import("std");

pub const Color24 = packed struct {
    r: u8,
    g: u8,
    b: u8,

    pub inline fn init(r: u8, g: u8, b: u8) @This() {
        return .{ .r = r, .g = g, .b = b };
    }

    pub inline fn from15(v: Color15) @This() {
        return .{
            .r = @as(u8, v.r) << 3,
            .g = @as(u8, v.g) << 3,
            .b = @as(u8, v.b) << 3,
        };
    }
};

pub const Color15 = packed struct {
    r: u5,
    g: u5,
    b: u5,
    a: u1 = 1,

    pub fn init(r: u5, g: u5, b: u5) @This() {
        return .{ .r = r, .g = g, .b = b };
    }

    pub inline fn from24(v: Color24) @This() {
        return .{
            .r = @truncate(v.r >> 3),
            .g = @truncate(v.g >> 3),
            .b = @truncate(v.b >> 3),
        };
    }
};

pub const ColorDepth = enum {
    bit4,
    bit8,
    bit15,
};

pub const Vertex = struct {
    x: i32,
    y: i32,
    tx: u32 = 0,
    ty: u32 = 0,
    color: Color24 = .init(0, 0, 0),
};

pub const Rasterizer = struct {
    const y_res = 512;
    const x_res = 1024;

    pixels: *align(16) [x_res * y_res]Color15,
    texw_mask: [2]u16,
    texw_offset: [2]u16,

    pub fn init(vram: *align(16) [0x100000]u8) @This() {
        const pixels = std.mem.bytesAsSlice(Color15, vram);
        return .{
            .pixels = pixels[0 .. x_res * y_res],
            .texw_mask = .{ 0, 0 },
            .texw_offset = .{ 0, 0 },
        };
    }

    pub inline fn fill(self: *@This(), c: Color24) void {
        @memset(self.pixels, Color15.from24(c));
    }

    pub inline fn drawPixel24(self: *@This(), x: i32, y: i32, c: Color24) void {
        const idx: usize = @intCast(x + (y * x_res));
        if (idx >= self.pixels.len) return;
        self.pixels[idx] = Color15.from24(c);
    }

    pub inline fn drawPixel15(self: *@This(), x: i32, y: i32, c: Color15) void {
        const idx: usize = @intCast(x + (y * x_res));
        if (idx >= self.pixels.len) return;
        self.pixels[idx] = c;
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
                    self.drawPixel24(x, y, c);
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

                    self.drawPixel24(x, y, .{
                        .r = @truncate(r >> fixed_bits),
                        .g = @truncate(g >> fixed_bits),
                        .b = @truncate(b >> fixed_bits),
                    });
                }
            }
        }
    }

    pub fn setTextureWindow(self: *@This(), mask: [2]u16, offset: [2]u16) void {
        self.texw_mask = mask;
        self.texw_offset = offset;
    }

    fn fetchTexturePixel(
        self: *@This(),
        tx_orig: u16,
        ty_orig: u16,
        tpx: u16,
        tpy: u16,
        clutx: u16,
        cluty: u16,
        depth: ColorDepth,
    ) Color15 {
        const tx: u16 = @truncate((tx_orig & ~self.texw_mask[0]) | (self.texw_offset[0] & self.texw_mask[0]));
        const ty: u16 = @truncate((ty_orig & ~self.texw_mask[1]) | (self.texw_offset[1] & self.texw_mask[1]));

        switch (depth) {
            .bit4 => { // 4-bit
                // uint16_t texel = gpu->vram[(tpx + (tx >> 2)) + ((tpy + ty) * 1024)];
                // int index = (texel >> ((tx & 0x3) << 2)) & 0xf;
                // return gpu->vram[(clutx + index) + (cluty * 1024)];
                const texel_index = (tpx + (tx >> 2)) + ((tpy + ty) * 1024);
                if (texel_index >= self.pixels.len) {
                    return .init(0, 0x1f, 0x1f);
                }

                const texel: u16 = @bitCast(self.pixels[texel_index]);
                const index = (texel >> @intCast((tx & 0x3) << 2)) & 0xf;

                const pixel_index = (clutx + index) + @mulWithOverflow(cluty, 1024)[0];
                if (pixel_index >= self.pixels.len) {
                    return Color15.init(0, 0x1f, 0x1f);
                }

                return self.pixels[pixel_index];
            },
            .bit8 => { // 8-bit
                // uint16_t texel = gpu->vram[(tpx + (tx >> 1)) + ((tpy + ty) * 1024)];
                // int index = (texel >> ((tx & 0x1) << 3)) & 0xff;
                // return gpu->vram[(clutx + index) + (cluty * 1024)];
                return Color15.init(0, 0x1f, 0); // TODO
            },
            .bit15 => { // 15-bit
                const index = ((tpx + tx) + (tpy + ty) * 1024);
                if (index >= self.pixels.len) {
                    return Color15.init(0, 0x1f, 0x1f);
                }
                return self.pixels[index];
            },
        }
    }

    pub fn drawTriangleTextured(
        self: *@This(),
        v0: Vertex,
        v1: Vertex,
        v2: Vertex,
        clutx: u16,
        cluty: u16,
        tpx: u16,
        tpy: u16,
        depth: ColorDepth,
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

                    const tx = v0.tx * w0 + v1.tx * w1 + v2.tx * w2;
                    const ty = v0.ty * w0 + v1.ty * w1 + v2.ty * w2;

                    const texel = self.fetchTexturePixel(
                        @truncate(tx >> fixed_bits),
                        @truncate(ty >> fixed_bits),
                        tpx,
                        tpy,
                        clutx,
                        cluty,
                        depth,
                    );

                    self.drawPixel15(x, y, texel);
                }
            }
        }
    }
};
