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
    const xres = 1024;
    const yres = 512;

    pixels: *align(16) [xres * yres]Color15,
    texwin_mask: [2]u16,
    texwin_offset: [2]u16,

    pub fn init(vram: *align(16) [0x100000]u8) @This() {
        const pixels = std.mem.bytesAsSlice(Color15, vram);
        std.debug.assert(xres * yres == pixels.len);

        return .{
            .pixels = pixels[0 .. xres * yres],
            .texwin_mask = .{ 0, 0 },
            .texwin_offset = .{ 0, 0 },
        };
    }

    pub inline fn fill(self: *@This(), c: Color24) void {
        @memset(self.pixels, Color15.from24(c));
    }

    pub inline fn drawPixel24(self: *@This(), x: i32, y: i32, c: Color24) void {
        if (x < 0 or y < 0) return;
        const idx: usize = @intCast(x + (y * xres));
        self.pixels[idx & 0xfffff] = Color15.from24(c);
    }

    pub inline fn drawPixel15(self: *@This(), x: i32, y: i32, c: Color15) void {
        const idx: u32 = @intCast(x + (y * xres));
        self.pixels[idx & 0xfffff] = c;
    }

    inline fn getPixel(self: *@This(), x: i32, y: i32) Color15 {
        const idx: usize = @intCast(x + (y * xres));
        return self.pixels[idx & 0xfffff];
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
        const x_max = @min(@max(v0.x, v1.x, v2.x), xres);
        const y_max = @min(@max(v0.y, v1.y, v2.y), yres);

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
        const x_max = @min(@max(v0.x, v1.x, v2.x), xres);
        const y_max = @min(@max(v0.y, v1.y, v2.y), yres);

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
        self.texwin_mask = mask;
        self.texwin_offset = offset;
    }

    fn getTexel(
        self: *@This(),
        tx_orig: u16,
        ty_orig: u16,
        tpx: u16,
        tpy: u16,
        clutx: u16,
        cluty: u16,
        depth: ColorDepth,
    ) Color15 {
        // const tx: u8 = @truncate((tx_orig & ~self.texwin_mask[0]) | (self.texwin_offset[0] & self.texwin_mask[0]));
        // const ty: u8 = @truncate((ty_orig & ~self.texwin_mask[1]) | (self.texwin_offset[1] & self.texwin_mask[1]));
        const tx: u16 = tx_orig;
        const ty: u16 = ty_orig;

        switch (depth) {
            .bit4 => { // 4-bit
                _ = clutx;
                _ = cluty;
                return Color15.init(0, 0x1f, 0); // TODO
                // const texel: u16 = @bitCast(self.getPixel(tpx + tx / 4, tpy + ty));
                // const index = (texel >> @as(u4, @truncate((tx % 4) * 4))) & 0x0f;

                // std.log.debug("tx={}, ty={}, tpx={}, tpy={}, clutx={}, cluty={}", .{ tx, ty, tpx, tpy, clutx, cluty });
                // std.log.debug("texel={x}, index={}", .{ texel, index });
                // const pixel: u16 = @bitCast(self.getPixel(clutx + index, cluty));
                // std.log.debug("pixel={:04x}", .{pixel});

                // return self.getPixel(clutx + index, cluty);
            },
            .bit8 => { // 8-bit
                // uint16_t texel = gpu->vram[(tpx + (tx >> 1)) + ((tpy + ty) * 1024)];
                // int index = (texel >> ((tx & 0x1) << 3)) & 0xff;
                // return gpu->vram[(clutx + index) + (cluty * 1024)];
                return Color15.init(0, 0x1f, 0); // TODO
            },
            .bit15 => { // 15-bit
                return self.getPixel(tpx + tx, tpy + ty);
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
        const x_max = @min(@max(v0.x, v1.x, v2.x), xres);
        const y_max = @min(@max(v0.y, v1.y, v2.y), yres);

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

                    const texel = self.getTexel(
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

    pub fn drawRectFlat(
        self: *@This(),
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        c: Color24,
    ) void {
        const x_end = @min(x + w, xres);
        const y_end = @min(y + h, yres);

        var yy = y;
        while (yy < y_end) : (yy += 1) {
            var xx = x;
            while (xx < x_end) : (xx += 1) {
                self.drawPixel24(xx, yy, c);
            }
        }
    }

    pub fn drawLineFlat(
        self: *@This(),
        x0: i32,
        y0: i32,
        x1: i32,
        y1: i32,
        c: Color24,
    ) void {
        const dx: i32 = @intCast(@abs(x1 - x0));
        const dy: i32 = @intCast(@abs(y1 - y0));
        const sx: i32 = if (x0 < x1) 1 else -1;
        const sy: i32 = if (y0 < y1) 1 else -1;
        var err = dx - dy;

        var x = x0;
        var y = y0;

        while (true) {
            self.drawPixel24(x, y, c);
            if (x == x1 and y == y1) break;
            const err2 = err * 2;
            if (err2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (err2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
};
