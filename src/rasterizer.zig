const std = @import("std");

const log = std.log.scoped(.rasterizer);

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
    draw_area_start: [2]i32,
    draw_area_end: [2]i32,
    draw_offset: [2]i32,
    texwin_mask: [2]u16,
    texwin_offset: [2]u16,

    pub fn init(vram: *align(16) [0x100000]u8) @This() {
        const pixels = std.mem.bytesAsSlice(Color15, vram);
        std.debug.assert(xres * yres == pixels.len);

        return .{
            .pixels = pixels[0 .. xres * yres],
            .texwin_mask = .{ 0, 0 },
            .texwin_offset = .{ 0, 0 },
            .draw_offset = .{ 0, 0 },
            .draw_area_start = .{ 0, 0 },
            .draw_area_end = .{ xres - 1, yres - 1 },
        };
    }

    pub inline fn setTextureWindow(self: *@This(), mask: [2]u16, offset: [2]u16) void {
        self.texwin_mask = mask;
        self.texwin_offset = offset;
    }

    pub inline fn setDrawAreaStart(self: *@This(), x: i32, y: i32) void {
        self.draw_area_start = .{ x, y };
    }

    pub inline fn setDrawOffset(self: *@This(), x: i32, y: i32) void {
        self.draw_offset = .{ x, y };
    }

    pub inline fn setDrawAreaEnd(self: *@This(), x: i32, y: i32) void {
        self.draw_area_end = .{ x, y };
    }

    inline fn inBounds(self: *@This(), x: i32, y: i32) bool {
        return x >= self.draw_area_start[0] and x <= self.draw_area_end[0] and
            y >= self.draw_area_start[1] and y <= self.draw_area_end[1] and
            x >= 0 and x < xres and y >= 0 and y < yres;
    }

    pub inline fn fill(self: *@This(), c: Color24) void {
        @memset(self.pixels, Color15.from24(c));
    }

    pub inline fn drawPixel24(self: *@This(), x: i32, y: i32, c: Color24) void {
        const xx = @as(u32, @bitCast(x)) & 0x3ff; // 0..1023
        const yy = @as(u32, @bitCast(y)) & 0x1ff; // 0..511
        self.pixels[xx + yy * xres] = Color15.from24(c);
    }

    pub inline fn drawPixel15(self: *@This(), x: i32, y: i32, c: Color15) void {
        const xx = @as(u32, @bitCast(x)) & 0x3ff; // 0..1023
        const yy = @as(u32, @bitCast(y)) & 0x1ff; // 0..511
        self.pixels[xx + yy * xres] = c;
    }

    inline fn getPixel(self: *@This(), x: i32, y: i32) Color15 {
        const xx = @as(u32, @bitCast(x)) & 0x3ff; // 0..1023
        const yy = @as(u32, @bitCast(y)) & 0x1ff; // 0..511
        return self.pixels[xx + yy * xres];
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
        var abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return;

        const flipped = abc < 0;
        if (flipped) abc = -abc;

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], xres - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], yres - 1);

        var y = y_min;

        while (y <= y_max) : (y += 1) {
            var x = x_min;

            while (x <= x_max) : (x += 1) {
                const p = Vertex{ .x = x, .y = y };
                var abp = edgeFunc(v0, v1, p);
                var bcp = edgeFunc(v1, v2, p);
                var cap = edgeFunc(v2, v0, p);

                if (flipped) {
                    abp = -abp;
                    bcp = -bcp;
                    cap = -cap;
                }

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
        var abc = edgeFunc(v0, v1, v2);
        if (abc == 0) return;

        const flipped = abc < 0;
        if (flipped) abc = -abc;

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], xres - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], yres - 1);

        const fixed_bits = 8;
        const fixed_one = 1 << fixed_bits;

        var y = y_min;

        while (y <= y_max) : (y += 1) {
            var x = x_min;

            while (x <= x_max) : (x += 1) {
                const p = Vertex{ .x = x, .y = y };

                var abp = edgeFunc(v0, v1, p);
                var bcp = edgeFunc(v1, v2, p);
                var cap = edgeFunc(v2, v0, p);

                if (flipped) {
                    abp = -abp;
                    bcp = -bcp;
                    cap = -cap;
                }

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
        const tx: u8 = @truncate((tx_orig & ~self.texwin_mask[0]) | (self.texwin_offset[0] & self.texwin_mask[0]));
        const ty: u8 = @truncate((ty_orig & ~self.texwin_mask[1]) | (self.texwin_offset[1] & self.texwin_mask[1]));
        // const tx: u16 = tx_orig;
        // const ty: u16 = ty_orig;

        switch (depth) {
            .bit4 => { // 4-bit
                const texel: u16 = @bitCast(self.getPixel(tpx + tx / 4, tpy + ty));
                const shift = @as(u4, @truncate((tx % 4) * 4));
                const idx = (texel >> shift) & 0xf;
                return self.getPixel(clutx + idx, cluty);
            },
            .bit8 => { // 8-bit
                const texel: u16 = @bitCast(self.getPixel(tpx + tx / 2, tpy + ty));
                const shift = @as(u4, @truncate((tx % 2) * 8));
                const idx = (texel >> shift) & 0xff;
                return self.getPixel(clutx + idx, cluty);
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

        const x_min = @max(@min(v0.x, v1.x, v2.x), self.draw_area_start[0], 0);
        const y_min = @max(@min(v0.y, v1.y, v2.y), self.draw_area_start[1], 0);
        const x_max = @min(@max(v0.x, v1.x, v2.x), self.draw_area_end[0], xres - 1);
        const y_max = @min(@max(v0.y, v1.y, v2.y), self.draw_area_end[1], yres - 1);

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

                if (abp > 0 and bcp > 0 and cap > 0) {
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

                    if (@as(u16, @bitCast(texel)) == 0) continue;
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

    pub fn drawRectTextured(
        self: *@This(),
        x_orig: i32,
        y_orig: i32,
        w: i32,
        h: i32,
        clutx: u16,
        cluty: u16,
        tpx: u16,
        tpy: u16,
        depth: ColorDepth,
    ) void {
        const x = x_orig + self.draw_offset[0];
        const y = y_orig + self.draw_offset[1];
        const x_end = @min(x + w, xres);
        const y_end = @min(y + h, yres);

        var tx: u16 = 0;
        var ty: u16 = 0;

        var py = y;
        while (py < y_end) : (py += 1) {
            var px = x;
            while (px < x_end) : (px += 1) {
                const texel = self.getTexel(tx, ty, tpx, tpy, clutx, cluty, depth);
                if (@as(u16, @bitCast(texel)) != 0) {
                    self.drawPixel15(px, py, texel);
                }

                tx += 1;
            }

            tx = 0;
            ty += 1;
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
        if (!self.inBounds(x0, y0) and !self.inBounds(x1, y1)) {
            return;
        }

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

    pub fn copyRect(
        self: *@This(),
        src_x: i32,
        src_y: i32,
        dest_x: i32,
        dest_y: i32,
        w: i32,
        h: i32,
    ) void {
        var y: i32 = 0;
        while (y < h) : (y += 1) {
            var x: i32 = 0;
            while (x < w) : (x += 1) {
                const pixel = self.getPixel(src_x + x, src_y + y);
                self.drawPixel15(dest_x + x, dest_y + y, pixel);
            }
        }
    }
};
