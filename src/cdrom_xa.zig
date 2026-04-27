const std = @import("std");
const adpcm = @import("adpcm.zig");
const Disc = @import("cdrom.zig").Disc;

const xa_stereo_samples = 2016;
const xa_mono_samples = 4032;
const xa_phase_unit = 44_100;

pub const Submode = packed struct(u8) {
    eor: bool = false, // 0
    video: bool = false, // 1
    audio: bool = false, // 2
    data: bool = false, // 3
    trigger: bool = false, // 4
    form2: bool = false, // 5
    realtime: bool = false, // 6
    eof: bool = false, // 7
};

pub const CodingInfo = packed struct(u8) {
    stereo: u2 = 0, // 0-1
    sample_rate: u1 = 0, // 2
    _pad0: u1 = 0, // 3
    bits_per_sample: u2 = 0, // 4-5
    emphasis: bool = false, // 6
    _pad1: u1 = 0, // 7
};

pub const Subheader = packed struct {
    file: u8,
    channel: u8,
    submode: Submode,
    codinginfo: CodingInfo,
};

pub fn parseSubheader(sector: []const u8) Subheader {
    return .{
        .file = sector[16],
        .channel = sector[17],
        .submode = @bitCast(sector[18]),
        .codinginfo = @bitCast(sector[19]),
    };
}

pub const XaState = struct {
    active: bool = false,
    lba: u32 = 0,
    xa_adpcm_enabled: bool = false,
    filter_enabled: bool = false,
    filter_file: u8 = 0,
    filter_channel: u8 = 0,
    file: u8 = 0,
    channel: u8 = 0,
    sample_rate: u32 = 18_900,
    stereo: bool = false,
    bits_per_sample: u2 = 0,
    emphasis: bool = false,
    decoded_samples: usize = 0,
    sample_index: usize = 0,
    phase: u32 = 0,
    prev_left: [2]i32 = .{ 0, 0 },
    prev_right: [2]i32 = .{ 0, 0 },
    left_buf: [xa_stereo_samples]i16 = std.mem.zeroes([xa_stereo_samples]i16),
    right_buf: [xa_stereo_samples]i16 = std.mem.zeroes([xa_stereo_samples]i16),
    mono_buf: [xa_mono_samples]i16 = std.mem.zeroes([xa_mono_samples]i16),

    pub fn reset(self: *@This()) void {
        self.active = false;
        self.lba = 0;
        self.xa_adpcm_enabled = false;
        self.filter_enabled = false;
        self.filter_file = 0;
        self.filter_channel = 0;
        self.file = 0;
        self.channel = 0;
        self.sample_rate = 18_900;
        self.stereo = false;
        self.bits_per_sample = 0;
        self.emphasis = false;
        self.decoded_samples = 0;
        self.sample_index = 0;
        self.phase = 0;
        self.prev_left = .{ 0, 0 };
        self.prev_right = .{ 0, 0 };
    }

    pub fn beginAt(self: *@This(), lba: u32) void {
        const xa_adpcm_enabled = self.xa_adpcm_enabled;
        const filter_enabled = self.filter_enabled;
        const filter_file = self.filter_file;
        const filter_channel = self.filter_channel;
        self.reset();
        self.xa_adpcm_enabled = xa_adpcm_enabled;
        self.filter_enabled = filter_enabled;
        self.filter_file = filter_file;
        self.filter_channel = filter_channel;
        self.active = true;
        self.lba = lba;
    }

    pub fn setMode(self: *@This(), xa_enabled: bool, filter_enabled: bool) void {
        self.xa_adpcm_enabled = xa_enabled;
        self.filter_enabled = filter_enabled;
    }

    pub fn setFilter(self: *@This(), file: u8, channel: u8) void {
        self.filter_file = file;
        self.filter_channel = channel;
    }

    pub fn isXaAudioSector(self: *const @This(), sector: []const u8) bool {
        if (!self.xa_adpcm_enabled) return false;
        if (sector.len != 2352) return false;
        if (sector[15] != 2) return false;

        const subheader = parseSubheader(sector);
        if (!subheader.submode.form2) return false;
        if (!subheader.submode.audio or !subheader.submode.realtime) return false;

        if (self.filter_enabled) {
            if (subheader.file != self.filter_file or subheader.channel != self.filter_channel) {
                return false;
            }
        }

        return true;
    }

    pub fn consumeSample(self: *@This(), disc: *Disc) [2]i16 {
        if (!self.active) {
            return .{ 0, 0 };
        }

        while (self.sample_index >= self.decoded_samples) {
            if (!self.fetchNextSector(disc)) {
                self.active = false;
                return .{ 0, 0 };
            }
        }

        const sample = switch (self.stereo) {
            true => [2]i16{ self.left_buf[self.sample_index], self.right_buf[self.sample_index] },
            false => [2]i16{ self.mono_buf[self.sample_index], self.mono_buf[self.sample_index] },
        };

        self.phase +%= self.sample_rate;
        while (self.phase >= xa_phase_unit) {
            self.phase -= xa_phase_unit;
            self.sample_index += 1;
        }

        return sample;
    }

    fn fetchNextSector(self: *@This(), disc: *Disc) bool {
        if (!self.active) return false;

        while (true) {
            const sector = disc.readSectorRawAt(self.lba) orelse return false;
            self.lba += 1;

            if (sector[15] != 2) continue;

            const subheader = parseSubheader(sector);
            if (subheader.submode.eor) return false;
            if (!subheader.submode.form2) continue;
            if (!subheader.submode.audio or !subheader.submode.realtime) continue;

            if (self.filter_enabled and (subheader.file != self.filter_file or subheader.channel != self.filter_channel)) {
                continue;
            }

            return self.decodeSector(sector[24 .. 24 + 0x900], subheader);
        }
    }

    fn decodeSector(self: *@This(), data: []const u8, subheader: Subheader) bool {
        if (subheader.codinginfo.stereo > 1 or subheader.codinginfo.bits_per_sample > 0) {
            return false;
        }

        const stereo = subheader.codinginfo.stereo == 1;
        const sample_rate: u32 = if (subheader.codinginfo.sample_rate == 0) 37_800 else 18_900;

        if (self.file != subheader.file or
            self.channel != subheader.channel or
            self.stereo != stereo or
            self.sample_rate != sample_rate)
        {
            self.prev_left = .{ 0, 0 };
            self.prev_right = .{ 0, 0 };
            self.sample_index = 0;
            self.phase = 0;
        }

        self.file = subheader.file;
        self.channel = subheader.channel;
        self.stereo = stereo;
        self.sample_rate = sample_rate;
        self.bits_per_sample = subheader.codinginfo.bits_per_sample;
        self.emphasis = subheader.codinginfo.emphasis;

        var left_block: [28]i16 = undefined;
        var right_block: [28]i16 = undefined;
        var block_data: [28]u8 = undefined;

        var left_pos: usize = 0;
        var right_pos: usize = 0;
        var mono_pos: usize = 0;

        for (0..18) |group_i| {
            const group = data[group_i * 128 ..][0..128];

            for (0..4) |blk| {
                const left_header = group[4 + blk * 2 + 0];
                const right_header = group[4 + blk * 2 + 1];

                for (0..28) |sample_i| {
                    block_data[sample_i] = group[16 + blk + sample_i * 4];
                }

                if (stereo) {
                    adpcm.decodeXaBlock(left_header, &block_data, 0, &self.prev_left, &left_block);
                    adpcm.decodeXaBlock(right_header, &block_data, 4, &self.prev_right, &right_block);
                    @memcpy(self.left_buf[left_pos .. left_pos + 28], left_block[0..]);
                    @memcpy(self.right_buf[right_pos .. right_pos + 28], right_block[0..]);
                    left_pos += 28;
                    right_pos += 28;
                } else {
                    adpcm.decodeXaBlock(left_header, &block_data, 0, &self.prev_left, &left_block);
                    @memcpy(self.mono_buf[mono_pos .. mono_pos + 28], left_block[0..]);
                    mono_pos += 28;
                    adpcm.decodeXaBlock(right_header, &block_data, 4, &self.prev_left, &left_block);
                    @memcpy(self.mono_buf[mono_pos .. mono_pos + 28], left_block[0..]);
                    mono_pos += 28;
                }
            }
        }

        self.decoded_samples = if (stereo) left_pos else mono_pos;
        self.sample_index = 0;
        return true;
    }
};
