const std = @import("std");

const adpcm_filter_table_pos = [5]i32{ 0, 60, 115, 98, 122 };
const adpcm_filter_table_neg = [5]i32{ 0, 0, -52, -55, -60 };

fn decodeParams(header: u8, max_filter: u8) struct { shift: u8, filter_pos: i32, filter_neg: i32 } {
    var shift = header & 0x0f;
    var filter = (header & 0x70) >> 4;

    if (shift > 12) shift = 9;
    if (filter > max_filter) filter = max_filter;

    return .{
        .shift = shift,
        .filter_pos = adpcm_filter_table_pos[filter],
        .filter_neg = adpcm_filter_table_neg[filter],
    };
}

pub fn decodeBlock(block: *const [16]u8, prev_samples: *[2]i32, out: *[28]i16) void {
    const params = decodeParams(block[0], 4);

    for (0..28) |n| {
        const byte = block[2 + n / 2];

        const raw_nibble: u4 = if (n % 2 == 0) @truncate(byte & 0x0f) else @truncate((byte >> 4) & 0x0f);
        const nibble: i16 = @as(i4, @bitCast(raw_nibble)); // sign-extend

        var sample: i32 = @as(i32, nibble) << 12;
        sample >>= @intCast(params.shift);

        sample += @divTrunc(prev_samples[0] * params.filter_pos + prev_samples[1] * params.filter_neg + 32, 64);

        out[n] = @intCast(std.math.clamp(sample, -0x8000, 0x7fff));

        prev_samples[1] = prev_samples[0];
        prev_samples[0] = sample;
    }
}

pub fn decodeXaBlock(header: u8, data: []const u8, nibble_shift: u3, prev_samples: *[2]i32, out: *[28]i16) void {
    std.debug.assert(data.len >= 28);

    const params = decodeParams(header, 3);

    for (0..28) |n| {
        const raw_nibble: u4 = @truncate((data[n] >> nibble_shift) & 0x0f);
        const nibble: i16 = @as(i4, @bitCast(raw_nibble));

        var sample: i32 = @as(i32, nibble) << 12;
        sample >>= @intCast(params.shift);
        sample += @divTrunc(prev_samples[0] * params.filter_pos + prev_samples[1] * params.filter_neg + 32, 64);

        out[n] = @intCast(std.math.clamp(sample, -0x8000, 0x7fff));
        prev_samples[1] = prev_samples[0];
        prev_samples[0] = sample;
    }
}
