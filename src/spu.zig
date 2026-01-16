const std = @import("std");
const bits = @import("bits.zig");
const fifo = @import("fifo.zig");
const adpcm = @import("adpcm.zig");

const log = std.log.scoped(.spu);

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

const Voice = struct {
    volume_left: u16, // +0x00
    volume_right: u16, // +0x02
    sample_rate: u16, // +0x04
    start_addr: u16, // +0x06
    adsr_lo: u16, // +0x08
    adsr_hi: u16, // +0x0A
    adsr_volume: u16, // +0x0C
    repeat_addr: u16, // +0x0E

    // 0x1f801d94
    mode: enum(u1) { adpcm = 0, noise = 1 },

    // Flags
    key_on: bool = false,
    key_off: bool = false,
    reached_end: bool = false,

    samples: [28]i16,
    pitch_counter: u32,
    prev_samples: [2]i32,
    curr_addr: u16,
};

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

    stub_data: [0x200]u16,

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
            .stub_data = std.mem.zeroes([0x200]u16),
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

        adpcm.decodeBlock(block, &voice.prev_samples, &voice.samples);
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

    inline fn getVoiceSample(self: *@This(), voice_i: u8) struct { left: i32, right: i32 } {
        const voice = &self.voices[voice_i];

        if (!voice.key_on) return .{ .left = 0, .right = 0 };

        if (voice.mode == .noise) {
            log.warn("voice noise mode not implemented", .{});
            return .{ .left = 0, .right = 0 };
        }

        const sample_index = voice.pitch_counter >> 12;
        if (sample_index >= 28) {
            self.loadAdpcmBlock(@intCast(voice_i));
            voice.pitch_counter -= 28 << 12;
        }

        const curr_sample_index = voice.pitch_counter >> 12;
        const sample = voice.samples[curr_sample_index];

        var step: u32 = voice.sample_rate;
        if (step > 0x3fff) step = 0x4000;
        voice.pitch_counter += step;

        const vol_left: i16 = @bitCast(voice.volume_left);
        const vol_right: i16 = @bitCast(voice.volume_right);

        const left = (@as(i32, sample) * @as(i32, vol_left)) >> 15;
        const right = (@as(i32, sample) * @as(i32, vol_right)) >> 15;

        return .{ .left = left, .right = right };
    }

    pub fn getAudioSample(self: *@This()) struct { left: i16, right: i16 } {
        var mix_left: i32 = 0;
        var mix_right: i32 = 0;

        if (self.spucnt.spu_enable) {
            for (0..24) |i| {
                const s = self.getVoiceSample(@intCast(i));
                mix_left += @as(i32, s.left);
                mix_right += @as(i32, s.right);
            }
        }

        return .{
            .left = @intCast(std.math.clamp(mix_left, -0x8000, 0x7fff)),
            .right = @intCast(std.math.clamp(mix_right, -0x8000, 0x7fff)),
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
                0x8 => voice.adsr_lo,
                0xA => voice.adsr_hi,
                0xC => voice.adsr_volume,
                0xE => voice.repeat_addr,
                else => 0,
            };

            return value;
        }

        // Control registers
        return switch (addr) {
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
        const KeyOnOp = opaque {
            fn op(spu: *SPU, voice: *Voice, voice_idx: u8, _: u1) void {
                voice.key_on = true;
                voice.key_off = false;
                voice.reached_end = false;
                voice.curr_addr = voice.start_addr;
                voice.pitch_counter = 0;
                voice.prev_samples = .{ 0, 0 };

                spu.loadAdpcmBlock(voice_idx);
                log.debug("voice {d} KEY ON start_addr={x}", .{ voice_idx, voice.start_addr });
            }
        };

        self.forEachVoiceInMask(addr, v, KeyOnOp.op);
    }

    fn setVoiceKeyOff(self: *@This(), addr: u32, v: u16) void {
        const KeyOffOp = struct {
            fn op(_: *SPU, voice: *Voice, voice_idx: u8, _: u1) void {
                voice.key_off = true;
                voice.key_on = false;
                log.debug("voice {d} KEY OFF", .{voice_idx});
            }
        };

        self.forEachVoiceInMask(addr, v, KeyOffOp.op);
    }

    fn setVoiceNoiseMode(self: *@This(), addr: u32, v: u16) void {
        const NoiseModeOp = struct {
            fn op(_: *SPU, voice: *Voice, voice_idx: u8, bit_value: u1) void {
                voice.mode = @enumFromInt(bit_value);
                log.debug("voice {d} NOISE MODE set to {any}", .{ voice_idx, voice.mode });
            }
        };

        self.forEachVoiceInMask(addr, v, NoiseModeOp.op);
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

            log.debug("voice {d} register write: {x} = {x}", .{ voice_idx, reg_offset, v });
            const voice = &self.voices[voice_idx];
            switch (reg_offset) {
                0x0 => voice.volume_left = v,
                0x2 => voice.volume_right = v,
                0x4 => voice.sample_rate = v,
                0x6 => voice.start_addr = v,
                0x8 => voice.adsr_lo = v,
                0xa => voice.adsr_hi = v,
                0xc => voice.adsr_volume = v,
                0xe => voice.repeat_addr = v,
                else => {},
            }

            return;
        }

        // Control registers
        switch (addr) {
            0x1f801d88, 0x1f801d8a => self.setVoiceKeyOn(addr, v),
            0x1f801d8c, 0x1f801d8e => self.setVoiceKeyOff(addr, v),
            0x1f801d90 => self.setVoiceNoiseMode(addr, v),
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
        self.ram[self.data_addr_internal + 0] = @truncate((v >> 0) & 0xff);
        self.ram[self.data_addr_internal + 1] = @truncate((v >> 8) & 0xff);
        self.data_addr_internal += 2;

        if (self.spucnt.irq9_enable and self.data_addr_internal == self.irq_addr * 8) {
            self.spustat.irq9_flag = true;
            self.irq_pending = true;
        }
    }
};
