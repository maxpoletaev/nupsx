const std = @import("std");
const bits = @import("bits.zig");
const fifo = @import("fifo.zig");
const adpcm = @import("adpcm.zig");

const log = std.log.scoped(.spu);
const clamp = std.math.clamp;

const fp_bits = 12;

const CntReg = packed struct(u16) {
    cd_audio_enable: bool = false, // 0
    external_audio_enable: bool = false, // 1
    cd_audio_reverb: bool = false, // 2
    external_audio_reverb: bool = false, // 3
    ram_transfer_mode: enum(u2) { stop = 0, manual_write = 1, dma_write = 2, dma_read = 3 } = .stop, // 4-5
    irq9_enable: bool = false, // 6
    reverb_master_enable: bool = false, // 7
    noise_frequency_step: enum(u2) { step4 = 0, step5 = 1, step6 = 2, step7 = 3 } = .step4, // 8-9
    noise_frequency_shift: u4 = 0, // 10-13
    spu_unmute: bool = false, // 14
    spu_enable: bool = false, // 15
};

const StatReg = packed struct(u16) {
    current_spu_mode: u6 = 0, // 0-5
    irq9_flag: bool = false, // 6
    dma_read_write_request: bool = false, // 7
    dma_write_request: bool = false, // 8
    dma_read_request: bool = false, // 9
    data_transfer_busy_flag: bool = false, // 10
    writing_capture_buffer_half: bool = false, // 11
    _unused: u4 = 0, // 12-15
};

const AdsrReg = packed struct(u32) {
    sustain_level: u4 = 0, // 0-3
    decay_shift: u4 = 0, // 4-7
    attack_step: u2 = 0, // 8-9
    attack_shift: u5 = 0, // 10-14
    attack_exponential: bool = false, // 15
    release_shift: u5 = 0, // 16-20
    release_exponential: bool = false, // 21
    sustain_step: u2 = 0, // 22-23
    sustain_shift: u5 = 0, // 24-28
    _pad0: u1 = 0, // 29
    sustain_decrease: bool = false, // 30
    sustain_exponential: bool = false, // 31
};

const Phase = enum {
    attack,
    decay,
    sustain,
    release,
};

const Envelope = struct {
    step: i32 = 0,
    cycles: u32 = 0,
    exponential: bool = false,
    decrease: bool = false,
};

const Voice = struct {
    volume_left: u16, // +0x00
    volume_right: u16, // +0x02
    sample_rate: u16, // +0x04
    start_addr: u16, // +0x06
    adsr: AdsrReg, // +0x08 +0x0A
    repeat_addr: u16, // +0x0E
    noise_mode: bool, // 0x1f801d94

    // Flags
    key_on: bool = false,
    key_off: bool = false,
    reached_end: bool = false,

    // Envelope state
    adsr_volume: i16, // +0x0C
    adsr_phase: Phase = .release,
    adsr_env: Envelope = .{},
    adsr_cycles: u32 = 0,

    curr_addr: u16,
    samples: [28]i16,
    pitch_counter: u32,
    current_sample: i16,
    decoder_prev: [2]i32, // adpcm decoder window previous samples
    interpolation_prev: [4]i16, // last 4 samples from previous block for gauss interpolation

    inline fn setAdsrLo(self: *Voice, v: u16) void {
        const hi = @as(u32, @bitCast(self.adsr)) & 0xffff0000;
        self.adsr = @bitCast(hi | @as(u32, v));
    }

    inline fn setAdsrHi(self: *Voice, v: u16) void {
        const lo = @as(u32, @bitCast(self.adsr)) & 0x0000ffff;
        self.adsr = @bitCast(lo | (@as(u32, v) << 16));
    }

    fn initEnvelope(self: *Voice) void {
        switch (self.adsr_phase) {
            .attack => {
                const shift = self.adsr.attack_shift;
                const step = 7 - @as(u4, self.adsr.attack_step);

                self.adsr_env = .{
                    .exponential = self.adsr.attack_exponential,
                    .decrease = false,
                    .cycles = @as(u32, 1) << (shift -| 11),
                    .step = @as(i32, step) << (11 -| shift),
                };
            },
            .decay => {
                const shift = self.adsr.decay_shift;
                const step: u4 = 8;

                self.adsr_env = .{
                    .exponential = true,
                    .decrease = true,
                    .cycles = @as(u32, 1) << (shift -| 11),
                    .step = @as(i32, step) << (11 -| shift),
                };
            },
            .sustain => {
                const shift = self.adsr.sustain_shift;
                const step = 7 - @as(u4, self.adsr.sustain_step);

                self.adsr_env = .{
                    .exponential = self.adsr.sustain_exponential,
                    .decrease = self.adsr.sustain_decrease,
                    .cycles = @as(u32, 1) << (shift -| 11),
                    .step = @as(i32, step) << (11 -| shift),
                };
            },
            .release => {
                const shift = self.adsr.release_shift;
                const step: u4 = 8;

                self.adsr_env = .{
                    .exponential = self.adsr.release_exponential,
                    .decrease = true,
                    .cycles = @as(u32, 1) << (shift -| 11),
                    .step = @as(i32, step) << (11 -| shift),
                };
            },
        }
    }

    fn keyOn(self: *Voice) void {
        self.key_on = true;
        self.key_off = false;
        self.reached_end = false;
        self.curr_addr = self.start_addr;
        self.repeat_addr = self.start_addr;
        self.pitch_counter = 0;
        self.current_sample = 0;
        self.decoder_prev = .{ 0, 0 };
        self.interpolation_prev = .{ 0, 0, 0, 0 };
        self.samples = std.mem.zeroes([28]i16);
        self.adsr_phase = .attack;
        self.adsr_volume = 0;
        self.adsr_cycles = 0;
        self.initEnvelope();
    }

    fn keyOff(self: *Voice) void {
        self.key_off = true;
    }

    inline fn advanceAdsrPhase(self: *Voice) void {
        switch (self.adsr_phase) {
            .attack => if (self.adsr_volume >= 0x7fff) {
                self.adsr_phase = .decay;
                self.adsr_cycles = 0;
                self.initEnvelope();
            },
            .decay => {
                const sustain_level = (@as(i32, self.adsr.sustain_level) + 1) * 0x800;
                if (self.adsr_volume <= sustain_level) {
                    self.adsr_phase = .sustain;
                    self.adsr_cycles = 0;
                    self.initEnvelope();
                }
            },
            .sustain => {
                // Sustain continues until key_off
            },
            .release => if (self.adsr_volume == 0) {
                // Release continues until volume reaches 0
                self.key_on = false;
                self.adsr_cycles = 0;
            },
        }
    }

    fn stepAdsrEnvelope(self: *Voice) void {
        if (self.key_off and self.adsr_phase != .release) {
            self.adsr_phase = .release;
            self.adsr_cycles = 0;
            self.initEnvelope();
        }

        if (self.adsr_cycles > 0) {
            self.adsr_cycles -= 1;
            return;
        }

        var cycles = self.adsr_env.cycles;

        var step = self.adsr_env.step;

        if (self.adsr_env.exponential) {
            if (!self.adsr_env.decrease and self.adsr_volume > 0x6000) cycles *= 4;
            if (self.adsr_env.decrease) step = (step * self.adsr_volume) >> 15; // div by 0x8000
        }

        if (self.adsr_env.decrease) step = -step;

        const new_volume: i32 = self.adsr_volume + step;

        self.adsr_volume = @intCast(clamp(new_volume, 0, 0x7fff));

        self.adsr_cycles = cycles;

        self.advanceAdsrPhase();
    }

    fn stepPitchCounter(self: *Voice) void {
        if (self.noise_mode) {
            log.warn("voice noise mode not implemented", .{});
            return;
        }

        const sample_i = self.pitch_counter >> fp_bits;
        std.debug.assert(sample_i < 28);

        self.current_sample = interpolate(&self.samples, &self.interpolation_prev, sample_i);
        self.pitch_counter += clamp(self.sample_rate, 0, 0x4000);
    }
};

inline fn applyVolume(sample: i32, volume: i32) i32 {
    return (sample *| volume) >> 15;
}

pub const SPU = struct {
    pub const addr_start: u32 = 0x1f801c00;
    pub const addr_end: u32 = 0x1f801fff;

    allocator: std.mem.Allocator,

    ram: [0x80000]u8,
    voices: [24]Voice,

    data_addr: u16,
    data_ctrl: u16,
    data_addr_internal: u32,
    irq_addr: u16,

    spucnt: CntReg,
    spustat: StatReg,

    stub_data: [0x400]u16,

    irq_pending: bool,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(SPU) catch unreachable;
        self.* = .{
            .allocator = allocator,
            .ram = std.mem.zeroes([0x80000]u8),
            .voices = std.mem.zeroes([24]Voice),
            .data_addr = 0,
            .data_ctrl = 0,
            .data_addr_internal = 0,
            .irq_addr = 0,
            .spucnt = .{},
            .spustat = .{},
            .stub_data = std.mem.zeroes([0x400]u16),
            .irq_pending = false,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    fn loadAdpcmBlock(self: *@This(), voice_i: u8) void {
        const voice = &self.voices[voice_i];

        if (self.spucnt.irq9_enable and voice.curr_addr == self.irq_addr) {
            self.spustat.irq9_flag = true;
            self.irq_pending = true;
        }

        const block_addr = @as(u32, voice.curr_addr) * 8;
        const block: *const [16]u8 = self.ram[block_addr..][0..16];

        const flags: packed struct(u8) {
            loop_end: bool,
            loop_repeat: bool,
            loop_start: bool,
            _pad: u5,
        } = @bitCast(block[1]);

        if (flags.loop_start) {
            voice.repeat_addr = voice.curr_addr;
        }

        voice.interpolation_prev = voice.samples[24..28].*;
        adpcm.decodeBlock(block, &voice.decoder_prev, &voice.samples);

        voice.curr_addr +%= 2;

        if (flags.loop_end) {
            voice.reached_end = true;
            voice.curr_addr = voice.repeat_addr;
            if (!flags.loop_repeat) voice.key_on = false;
        }
    }

    pub fn consumeIrq(self: *@This()) bool {
        const was_pending = self.irq_pending;
        self.irq_pending = false;
        return was_pending;
    }

    inline fn getVoiceSample(self: *@This(), voice_i: u8) [2]i32 {
        const voice = &self.voices[voice_i];

        if (!voice.key_on) return .{ 0, 0 };

        voice.stepPitchCounter();

        if (voice.pitch_counter >> fp_bits >= 28) {
            self.loadAdpcmBlock(@intCast(voice_i));
            voice.pitch_counter -= @as(u32, 28) << fp_bits;
        }

        voice.stepAdsrEnvelope();

        const vol_left: i16 = @bitCast(voice.volume_left);
        const vol_right: i16 = @bitCast(voice.volume_right);

        var left: i32 = voice.current_sample;
        left = applyVolume(left, vol_left);
        left = applyVolume(left, voice.adsr_volume);

        var right: i32 = voice.current_sample;
        right = applyVolume(right, vol_right);
        right = applyVolume(right, voice.adsr_volume);

        return .{ @intCast(left), @intCast(right) };
    }

    pub fn consumeAudioSample(self: *@This()) [2]i16 {
        if (!self.spucnt.spu_enable) return .{ 0, 0 };

        var mix: [2]i32 = .{ 0, 0 };

        for (0..24) |i| {
            const sample = self.getVoiceSample(@intCast(i));
            mix[0] +|= @as(i32, sample[0]);
            mix[1] +|= @as(i32, sample[1]);
        }

        return .{
            @intCast(clamp(mix[0], -0x8000, 0x7fff)),
            @intCast(clamp(mix[1], -0x8000, 0x7fff)),
        };
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        switch (T) {
            u8 => {
                // non 16-bit aligned reads are possible
                const value = self.read16(addr & ~@as(u32, 1));
                const shift: u4 = @truncate((addr & 1) * 8);
                return @truncate((value >> shift) & 0xff);
            },
            u32 => {
                const low = self.read16(addr);
                const high = self.read16(addr + 2);
                return @as(T, ((@as(u32, high) << 16) | @as(u32, low)));
            },
            u16 => return self.read16(addr),
            else => std.debug.panic("unsupported type {s}", .{@typeName(T)}),
        }
    }

    fn read16(self: *@This(), addr: u32) u16 {
        // Voice registers
        if (addr >= 0x1f801c00 and addr < 0x1f801d80) {
            const voice_idx = ((addr - 0x1f801c00) >> 4) & 0x1f;
            const reg_offset = (addr - 0x1f801c00) & 0xf;

            if (voice_idx >= 24) {
                log.warn("invalid voice index {d}", .{voice_idx});
                return 0;
            }

            const voice = &self.voices[voice_idx];
            const value: u16 = switch (reg_offset) {
                0x0 => voice.volume_left,
                0x2 => voice.volume_right,
                0x4 => voice.sample_rate,
                0x6 => voice.start_addr,
                0x8 => @truncate(@as(u32, @bitCast(voice.adsr)) >> 0),
                0xa => @truncate(@as(u32, @bitCast(voice.adsr)) >> 16),
                0xc => @bitCast(voice.adsr_volume),
                0xe => voice.repeat_addr,
                else => 0,
            };

            return value;
        }

        // Control registers
        return switch (addr) {
            0x1f801d9c, 0x1f801d9e => self.readEndx(addr),
            0x1f801da4 => self.irq_addr,
            0x1f801da6 => self.data_addr,
            0x1f801daa => @bitCast(self.spucnt),
            0x1f801dac => self.data_ctrl,
            0x1f801dae => blk: { // SPUSTAT
                const cnt: u16 = @bitCast(self.spucnt);
                const stat: u16 = @bitCast(self.spustat);
                break :blk (stat & ~@as(u16, 0x1f)) | (cnt & 0x1f); // lower 5 bits are from SPUCNT
            },
            else => blk: {
                // log.warn("unhandled SPU read at {x}", .{addr});
                const offset = addr - addr_start;
                break :blk self.stub_data[offset];
            },
        };
    }

    fn readEndx(self: *@This(), addr: u32) u16 {
        const shift: u5 = if ((addr & 0x2) == 0) 0 else 16;
        var endx: u16 = 0;
        for (0..16) |i| {
            const voice_idx = i + shift;
            if (voice_idx >= 24) break;
            if (self.voices[voice_idx].reached_end) {
                endx |= @as(u16, 1) << @intCast(i);
            }
        }
        return endx;
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        switch (T) {
            u8 => {
                if ((addr & 1) != 0) return;
                const existing = self.read16(addr);
                const new_value = (existing & 0xff00) | @as(u16, v);
                self.write16(addr, new_value);
            },
            u32 => {
                self.write16(addr + 0, @truncate((@as(u32, v) >> 0)));
                self.write16(addr + 2, @truncate((@as(u32, v) >> 16)));
            },
            u16 => self.write16(addr, @as(u16, v)),
            else => unreachable,
        }
    }

    fn forEachVoiceInMask(self: *@This(), addr: u32, v: u16, comptime operation: anytype) void {
        const shift: u5 = if ((addr & 0x2) == 0) 0 else 16;

        var mask_bits = v;
        while (mask_bits != 0) {
            defer mask_bits &= mask_bits - 1;
            const voice_idx = @ctz(mask_bits) + shift;

            if (voice_idx >= 24) continue;

            const voice = &self.voices[voice_idx];
            const bit_value: u1 = if ((v & (@as(u16, 1) << @intCast(voice_idx - shift))) != 0) 1 else 0;

            @call(.always_inline, operation, .{ self, voice, voice_idx, bit_value });
        }
    }

    fn setVoiceKeyOn(self: *@This(), addr: u32, v: u16) void {
        self.forEachVoiceInMask(addr, v, struct {
            fn callback(spu: *SPU, voice: *Voice, voice_idx: u8, _: u1) void {
                voice.keyOn();
                spu.loadAdpcmBlock(voice_idx);
            }
        }.callback);
    }

    fn setVoiceKeyOff(self: *@This(), addr: u32, v: u16) void {
        self.forEachVoiceInMask(addr, v, struct {
            fn callback(_: *SPU, voice: *Voice, _: u8, _: u1) void {
                voice.keyOff();
            }
        }.callback);
    }

    fn setVoiceNoiseMode(self: *@This(), addr: u32, v: u16) void {
        // Unlike Key On/Off, noise mode is a state register - must set ALL voices
        const shift: u5 = if ((addr & 0x2) == 0) 0 else 16;
        for (0..16) |i| {
            const voice_idx = i + shift;
            if (voice_idx >= 24) break;
            self.voices[voice_idx].noise_mode = (v & (@as(u16, 1) << @intCast(i))) != 0;
        }
    }

    fn write16(self: *@This(), addr: u32, v: u16) void {
        // Voice registers
        if (addr >= 0x1f801c00 and addr < 0x1f801d80) {
            const offset = addr - 0x1f801c00;
            const voice_idx = (offset >> 4) & 0x1f;
            const reg_offset = offset & 0xf;

            if (voice_idx >= 24) {
                log.warn("invalid voice index {d}", .{voice_idx});
                return;
            }

            const voice = &self.voices[voice_idx];
            switch (reg_offset) {
                0x0 => voice.volume_left = v,
                0x2 => voice.volume_right = v,
                0x4 => voice.sample_rate = v,
                0x6 => voice.start_addr = v,
                0x8 => voice.setAdsrLo(v),
                0xa => voice.setAdsrHi(v),
                0xc => voice.adsr_volume = @bitCast(v),
                0xe => voice.repeat_addr = v,
                else => {},
            }

            return;
        }

        // Control registers
        switch (addr) {
            0x1f801d88, 0x1f801d8a => self.setVoiceKeyOn(addr, v),
            0x1f801d8c, 0x1f801d8e => self.setVoiceKeyOff(addr, v),
            0x1f801d94, 0x1f801d96 => self.setVoiceNoiseMode(addr, v), // NON register
            0x1f801da4 => self.irq_addr = v,
            0x1f801da6 => {
                self.data_addr = v;
                self.data_addr_internal = @as(u32, v) * 8;
            },
            0x1f801da8 => self.writeData(v),
            0x1f801daa => self.spucnt = @bitCast(v),
            0x1f801dac => self.data_ctrl = v,

            else => {
                // log.warn("unhandled SPU write at {x} = {x}", .{ addr, v });
                const offset = addr - addr_start;
                self.stub_data[offset] = v;
            },
        }
    }

    pub fn writeData(self: *@This(), v: u16) void {
        if (self.data_addr_internal > self.ram.len - 1) {
            log.warn("SPU data write out of bounds at addr {x}", .{self.data_addr_internal});
            return;
        }

        self.ram[self.data_addr_internal + 0] = @truncate((v >> 0) & 0xff);
        self.ram[self.data_addr_internal + 1] = @truncate((v >> 8) & 0xff);
        self.data_addr_internal += 2;

        if (self.spucnt.irq9_enable and self.data_addr_internal == self.irq_addr * 8) {
            self.spustat.irq9_flag = true;
            self.irq_pending = true;
        }
    }
};

fn interpolate(samples: *[28]i16, prev_samples: *[4]i16, sample_i: u32) i16 {
    const oldest: i32 = if (sample_i >= 3) samples[sample_i - 3] else prev_samples[1 + sample_i];
    const older: i32 = if (sample_i >= 2) samples[sample_i - 2] else prev_samples[2 + sample_i];
    const old: i32 = if (sample_i >= 1) samples[sample_i - 1] else prev_samples[3 + sample_i];
    const new_: i32 = samples[sample_i];

    var out: i32 = 0;
    out += (@as(i32, spu_gauss_table[0x0ff - sample_i]) * oldest) >> 15;
    out += (@as(i32, spu_gauss_table[0x1ff - sample_i]) * older) >> 15;
    out += (@as(i32, spu_gauss_table[0x100 + sample_i]) * old) >> 15;
    out += (@as(i32, spu_gauss_table[0x000 + sample_i]) * new_) >> 15;

    return @intCast(out);
}

// zig fmt: off
const spu_gauss_table = [_]i16{
    -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001,
    -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001,
    0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0002, 0x0003, 0x0003,
    0x0003, 0x0004, 0x0004, 0x0005, 0x0005, 0x0006, 0x0007, 0x0007,
    0x0008, 0x0009, 0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x000e,
    0x000f, 0x0010, 0x0011, 0x0012, 0x0013, 0x0015, 0x0016, 0x0018,
    0x0019, 0x001b, 0x001c, 0x001e, 0x0020, 0x0021, 0x0023, 0x0025,
    0x0027, 0x0029, 0x002c, 0x002e, 0x0030, 0x0033, 0x0035, 0x0038,
    0x003a, 0x003d, 0x0040, 0x0043, 0x0046, 0x0049, 0x004d, 0x0050,
    0x0054, 0x0057, 0x005b, 0x005f, 0x0063, 0x0067, 0x006b, 0x006f,
    0x0074, 0x0078, 0x007d, 0x0082, 0x0087, 0x008c, 0x0091, 0x0096,
    0x009c, 0x00a1, 0x00a7, 0x00ad, 0x00b3, 0x00ba, 0x00c0, 0x00c7,
    0x00cd, 0x00d4, 0x00db, 0x00e3, 0x00ea, 0x00f2, 0x00fa, 0x0101,
    0x010a, 0x0112, 0x011b, 0x0123, 0x012c, 0x0135, 0x013f, 0x0148,
    0x0152, 0x015c, 0x0166, 0x0171, 0x017b, 0x0186, 0x0191, 0x019c,
    0x01a8, 0x01b4, 0x01c0, 0x01cc, 0x01d9, 0x01e5, 0x01f2, 0x0200,
    0x020d, 0x021b, 0x0229, 0x0237, 0x0246, 0x0255, 0x0264, 0x0273,
    0x0283, 0x0293, 0x02a3, 0x02b4, 0x02c4, 0x02d6, 0x02e7, 0x02f9,
    0x030b, 0x031d, 0x0330, 0x0343, 0x0356, 0x036a, 0x037e, 0x0392,
    0x03a7, 0x03bc, 0x03d1, 0x03e7, 0x03fc, 0x0413, 0x042a, 0x0441,
    0x0458, 0x0470, 0x0488, 0x04a0, 0x04b9, 0x04d2, 0x04ec, 0x0506,
    0x0520, 0x053b, 0x0556, 0x0572, 0x058e, 0x05aa, 0x05c7, 0x05e4,
    0x0601, 0x061f, 0x063e, 0x065c, 0x067c, 0x069b, 0x06bb, 0x06dc,
    0x06fd, 0x071e, 0x0740, 0x0762, 0x0784, 0x07a7, 0x07cb, 0x07ef,
    0x0813, 0x0838, 0x085d, 0x0883, 0x08a9, 0x08d0, 0x08f7, 0x091e,
    0x0946, 0x096f, 0x0998, 0x09c1, 0x09eb, 0x0a16, 0x0a40, 0x0a6c,
    0x0a98, 0x0ac4, 0x0af1, 0x0b1e, 0x0b4c, 0x0b7a, 0x0ba9, 0x0bd8,
    0x0c07, 0x0c38, 0x0c68, 0x0c99, 0x0ccb, 0x0cfd, 0x0d30, 0x0d63,
    0x0d97, 0x0dcb, 0x0e00, 0x0e35, 0x0e6b, 0x0ea1, 0x0ed7, 0x0f0f,
    0x0f46, 0x0f7f, 0x0fb7, 0x0ff1, 0x102a, 0x1065, 0x109f, 0x10db,
    0x1116, 0x1153, 0x118f, 0x11cd, 0x120b, 0x1249, 0x1288, 0x12c7,
    0x1307, 0x1347, 0x1388, 0x13c9, 0x140b, 0x144d, 0x1490, 0x14d4,
    0x1517, 0x155c, 0x15a0, 0x15e6, 0x162c, 0x1672, 0x16b9, 0x1700,
    0x1747, 0x1790, 0x17d8, 0x1821, 0x186b, 0x18b5, 0x1900, 0x194b,
    0x1996, 0x19e2, 0x1a2e, 0x1a7b, 0x1ac8, 0x1b16, 0x1b64, 0x1bb3,
    0x1c02, 0x1c51, 0x1ca1, 0x1cf1, 0x1d42, 0x1d93, 0x1de5, 0x1e37,
    0x1e89, 0x1edc, 0x1f2f, 0x1f82, 0x1fd6, 0x202a, 0x207f, 0x20d4,
    0x2129, 0x217f, 0x21d5, 0x222c, 0x2282, 0x22da, 0x2331, 0x2389,
    0x23e1, 0x2439, 0x2492, 0x24eb, 0x2545, 0x259e, 0x25f8, 0x2653,
    0x26ad, 0x2708, 0x2763, 0x27be, 0x281a, 0x2876, 0x28d2, 0x292e,
    0x298b, 0x29e7, 0x2a44, 0x2aa1, 0x2aff, 0x2b5c, 0x2bba, 0x2c18,
    0x2c76, 0x2cd4, 0x2d33, 0x2d91, 0x2df0, 0x2e4f, 0x2eae, 0x2f0d,
    0x2f6c, 0x2fcc, 0x302b, 0x308b, 0x30ea, 0x314a, 0x31aa, 0x3209,
    0x3269, 0x32c9, 0x3329, 0x3389, 0x33e9, 0x3449, 0x34a9, 0x3509,
    0x3569, 0x35c9, 0x3629, 0x3689, 0x36e8, 0x3748, 0x37a8, 0x3807,
    0x3867, 0x38c6, 0x3926, 0x3985, 0x39e4, 0x3a43, 0x3aa2, 0x3b00,
    0x3b5f, 0x3bbd, 0x3c1b, 0x3c79, 0x3cd7, 0x3d35, 0x3d92, 0x3def,
    0x3e4c, 0x3ea9, 0x3f05, 0x3f62, 0x3fbd, 0x4019, 0x4074, 0x40d0,
    0x412a, 0x4185, 0x41df, 0x4239, 0x4292, 0x42eb, 0x4344, 0x439c,
    0x43f4, 0x444c, 0x44a3, 0x44fa, 0x4550, 0x45a6, 0x45fc, 0x4651,
    0x46a6, 0x46fa, 0x474e, 0x47a1, 0x47f4, 0x4846, 0x4898, 0x48e9,
    0x493a, 0x498a, 0x49d9, 0x4a29, 0x4a77, 0x4ac5, 0x4b13, 0x4b5f,
    0x4bac, 0x4bf7, 0x4c42, 0x4c8d, 0x4cd7, 0x4d20, 0x4d68, 0x4db0,
    0x4df7, 0x4e3e, 0x4e84, 0x4ec9, 0x4f0e, 0x4f52, 0x4f95, 0x4fd7,
    0x5019, 0x505a, 0x509a, 0x50da, 0x5118, 0x5156, 0x5194, 0x51d0,
    0x520c, 0x5247, 0x5281, 0x52ba, 0x52f3, 0x532a, 0x5361, 0x5397,
    0x53cc, 0x5401, 0x5434, 0x5467, 0x5499, 0x54ca, 0x54fa, 0x5529,
    0x5558, 0x5585, 0x55b2, 0x55de, 0x5609, 0x5632, 0x565b, 0x5684,
    0x56ab, 0x56d1, 0x56f6, 0x571b, 0x573e, 0x5761, 0x5782, 0x57a3,
    0x57c3, 0x57e2, 0x57ff, 0x581c, 0x5838, 0x5853, 0x586d, 0x5886,
    0x589e, 0x58b5, 0x58cb, 0x58e0, 0x58f4, 0x5907, 0x5919, 0x592a,
    0x593a, 0x5949, 0x5958, 0x5965, 0x5971, 0x597c, 0x5986, 0x598f,
    0x5997, 0x599e, 0x59a4, 0x59a9, 0x59ad, 0x59b0, 0x59b2, 0x59b3,
};
// zig fmt: on
