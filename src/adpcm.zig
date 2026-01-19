const std = @import("std");

const adpcm_filter_table_pos = [5]i32{ 0, 60, 115, 98, 122 };
const adpcm_filter_table_neg = [5]i32{ 0, 0, -52, -55, -60 };

pub fn decodeBlock(block: *const [16]u8, prev_samples: *[2]i32, out: *[28]i16) void {
    var shift = block[0] & 0x0f;
    var filter = (block[0] & 0x70) >> 4;

    if (shift > 12) shift = 9;
    if (filter > 4) filter = 4;

    const filter_pos = adpcm_filter_table_pos[filter];
    const filter_neg = adpcm_filter_table_neg[filter];

    for (0..28) |n| {
        const byte = block[2 + n / 2];

        const raw_nibble: u4 = if (n % 2 == 0) @truncate(byte & 0x0f) else @truncate((byte >> 4) & 0x0f);
        const nibble: i16 = @as(i4, @bitCast(raw_nibble)); // sign-extend

        var sample: i32 = @as(i32, nibble) << 12;
        sample >>= @intCast(shift);

        sample += @divTrunc(prev_samples[0] * filter_pos + prev_samples[1] * filter_neg + 32, 64);

        out[n] = @intCast(std.math.clamp(sample, -0x8000, 0x7fff));

        prev_samples[1] = prev_samples[0];
        prev_samples[0] = sample;
    }
}
