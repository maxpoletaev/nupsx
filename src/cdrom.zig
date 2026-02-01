const std = @import("std");
const bits = @import("bits.zig");
const mem = @import("mem.zig");
const cue = @import("cue.zig");
const fifo = @import("fifo.zig");

const Interrupt = mem.Interrupt;

const log = std.log.scoped(.cdrom);

const cdrom_sector_size_cue = 2352;
const cdrom_file_offset_sectors_cue = 150;
const cdrom_file_offset_bytes_cue = cdrom_file_offset_sectors_cue * cdrom_sector_size_cue;

const cdrom_avg_delay_cycles = 50_401;
const cdrom_init_delay_cycles = 81_102;
const cdrom_seekl_delay_cycles = 450_000;
const cdrom_read_delay_cycles = (33_868_800 / 75);
const cdrom_read_2x_delay_cycles = (33_868_800 / 150);
const cdrom_pause_delay_cycles = 2_168_860;
const cdrom_pause_2x_delay_cycles = 1_097_107;
const cdrom_spinup_delay_cycles = 33_868_800; // 1 second

const cdrom_getid_nodisk = [_]u8{ 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const cdrom_getid_region = 'A'; // A=America, I=Japan, E=Europe
const cdrom_getid_licensed = [_]u8{ 0x02, 0x00, 0x20, 0x00, 'S', 'C', 'E', cdrom_getid_region };
const cdrom_date_version = [_]u8{ 0x94, 0x09, 0x19, 0xC0 };

pub const Error = error{
    FileIoError,
    CueParseError,
    BadCueSheet,
};

const AddressReg = packed struct(u8) {
    bank_index: u2 = 0, // 0-1 RA
    adpcm_busy: bool = false, // 2 ADPBUSY
    param_not_full: bool = true, // 3 PRMWRDY (set on read)
    param_empty: bool = true, // 4 PRMEMPT (set on read)
    result_ready: bool = false, // 5 RSLRRDY
    data_ready: bool = false, // 6 DRQSTS
    busy_status: bool = false, // 7 BUSYSTS
};

const StatusReg = packed struct(u8) {
    err: bool = false, // 0
    motor_on: bool = false, // 1
    seekerr: bool = false, // 2
    iderr: bool = false, // 3
    shellopen: bool = false, // 4
    read: bool = false, // 5
    seek: bool = false, // 6
    play: bool = false, // 7
};

const SectorSize = enum(u1) {
    data_only = 0,
    whole_sector = 1,
};

const ModeReg = packed struct(u8) {
    cdda: bool = false, // 0
    auto_pause: bool = false,
    play_report: bool = false, // 2
    ignore_bit: bool = false, // 3
    xa_filter: bool = false, // 4
    sector_size: SectorSize = .data_only,
    xa_adpcm: bool = false, // 6
    speed: enum(u1) { normal = 0, double = 1 } = .normal, // 7
};

const RequestReg = packed struct(u8) {
    unused0: u5 = 0, // 0-4 (always 0)
    unused1: u2 = 0, // 5-6 (smen and bfwr)
    want_data: bool = false, // 7 (BFRD)
};

const Position = struct {
    minute: u8,
    second: u8,
    sector: u8,

    pub fn init(m: u8, s: u8, f: u8) Position {
        return .{
            .minute = m,
            .second = s,
            .sector = f,
        };
    }

    pub fn fromSectors(sectors: u32) Position {
        const m = sectors / 75 / 60;
        const s = (sectors / 75) % 60;
        const f = sectors % 75;

        return .{
            .minute = @intCast(m),
            .second = @intCast(s),
            .sector = @intCast(f),
        };
    }

    pub fn toSectors(self: Position) u32 {
        return @as(u32, self.minute) * 60 * 75 +
            @as(u32, self.second) * 75 +
            @as(u32, self.sector);
    }
};

const Track = struct {
    number: u8,
    start_sector: u32,
    end_sector: u32,
};

pub const Disc = struct {
    allocator: std.mem.Allocator,
    cue: cue.CueSheet,
    data: []align(2) const u8,
    tracks: []Track,
    track_count: u8,
    pos: u32 = 0,

    pub fn loadCue(allocator: std.mem.Allocator, cue_path: []const u8) Error!@This() {
        var cue_sheet = cue.parseFile(allocator, cue_path) catch {
            log.err("failed to parse cue sheet: {s}", .{cue_path});
            return Error.FileIoError;
        };
        defer cue.free(allocator, &cue_sheet);

        const cue_root = std.fs.path.dirname(cue_path) orelse ".";
        const cue_dir = std.fs.cwd().openDir(cue_root, .{}) catch {
            log.err("failed to open cue sheet directory: {s}", .{cue_root});
            return Error.FileIoError;
        };

        var tracks: std.ArrayList(Track) = .empty;
        var absolute_sector: u32 = cdrom_file_offset_sectors_cue;
        var total_size: usize = 0;

        for (cue_sheet.files) |file| {
            const stat = cue_dir.statFile(file.filename) catch |err| {
                log.err("failed to stat file {s}: {}", .{ file.filename, err });
                return Error.FileIoError;
            };

            const file_sector_count: u32 = @intCast(@divExact(stat.size, cdrom_sector_size_cue));

            for (file.tracks) |track| {
                const index_01: ?cue.IndexNode = for (track.indices) |idx| {
                    if (idx.number == 1) break idx;
                } else null;

                if (index_01 == null) {
                    log.err("cue sheet track {d} missing INDEX 01", .{track.number});
                    return Error.BadCueSheet;
                }

                const track_start = absolute_sector + index_01.?.position.toSectors();
                const track_end = track_start + file_sector_count;

                tracks.append(allocator, Track{
                    .number = track.number,
                    .start_sector = track_start,
                    .end_sector = track_end,
                }) catch @panic("OOM");
            }

            total_size += stat.size;
            absolute_sector += file_sector_count;
        }

        for (tracks.items) |track| {
            const start_pos = Position.fromSectors(track.start_sector);
            log.info("track {d}: {d:02}:{d:02}:{d:02} lba={d}", .{
                track.number,
                start_pos.minute,
                start_pos.second,
                start_pos.sector,
                track.start_sector,
            });
        }

        var disc_data = allocator.alignedAlloc(u8, .@"2", total_size) catch @panic("OOM");
        errdefer allocator.free(disc_data);

        var reader_buf: [8192]u8 = undefined;
        var offset: usize = 0;

        for (cue_sheet.files) |file| {
            const bin_file = cue_dir.openFile(file.filename, .{ .mode = .read_only }) catch |err| {
                log.err("failed to open binary file {s}: {}", .{ file.filename, err });
                return Error.FileIoError;
            };
            defer bin_file.close();

            const file_size = bin_file.getEndPos() catch |err| {
                log.err("failed to stat file {s}: {}", .{ file.filename, err });
                return Error.FileIoError;
            };

            var reader = bin_file.reader(&reader_buf);
            reader.interface.readSliceAll(disc_data[offset .. offset + file_size]) catch |err| {
                log.err("failed to read file {s}: {}", .{ file.filename, err });
                return Error.FileIoError;
            };

            log.info("loaded bin file: {s} ({d} bytes)", .{ file.filename, file_size });
            offset += file_size;
        }

        if (tracks.items.len == 0) {
            log.err("cue sheet contains no tracks", .{});
            return Error.BadCueSheet;
        }

        const tracks_slice = tracks.toOwnedSlice(allocator) catch @panic("OOM");

        return .{
            .allocator = allocator,
            .tracks = tracks_slice,
            .track_count = @intCast(tracks_slice.len),
            .data = disc_data,
            .cue = cue_sheet,
        };
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.free(self.tracks);
        self.allocator.free(self.data);
    }

    pub fn seek(self: *@This(), sector: u32) void {
        self.pos = sector * cdrom_sector_size_cue;
        self.pos -= cdrom_file_offset_bytes_cue;
    }

    pub fn readSector(self: *@This(), sector_size: SectorSize) []align(2) const u8 {
        if (self.pos + cdrom_sector_size_cue > self.data.len) {
            std.debug.panic("disc read out of bounds at {d}", .{self.pos});
        }

        const sector = self.data[self.pos .. self.pos + cdrom_sector_size_cue];
        const v = switch (sector_size) {
            .data_only => sector[24 .. 24 + 2048],
            .whole_sector => sector[12..sector.len],
        };
        self.pos += cdrom_sector_size_cue;
        return @alignCast(v);
    }

    fn readSectorRaw(self: *@This()) []align(2) const u8 {
        if (self.pos + cdrom_sector_size_cue > self.data.len) {
            std.debug.panic("disc read out of bounds at {d}", .{self.pos});
        }

        const sector = self.data[self.pos .. self.pos + cdrom_sector_size_cue];
        self.pos += cdrom_sector_size_cue;
        return @alignCast(sector);
    }

    pub fn getTrackByNumber(self: *@This(), number: u8) ?*const Track {
        if (number == 0) {
            std.debug.assert(self.tracks.len > 0);
            return &self.tracks[self.tracks.len - 1];
        }
        for (0.., self.tracks) |i, track| {
            if (track.number == number) {
                return &self.tracks[i];
            }
        }
        return null;
    }
};

const CmdState = enum {
    recv_cmd,
    resp1,
    resp2,
};

const ReadState = enum {
    idle,
    reading,
    playing,
};

const AudioBuffer = fifo.StaticFifo([2]i16, cdrom_sector_size_cue * 10 / 4);

pub const CDROM = struct {
    pub const addr_start: u32 = 0x1f801800;
    pub const addr_end: u32 = 0x1f801803;

    allocator: std.mem.Allocator,
    bus: *mem.Bus,

    addr: AddressReg,
    stat: StatusReg,
    mode: ModeReg,
    req: RequestReg,

    mute: bool,

    cmd: ?u8,
    cmd_state: CmdState,
    cmd_queue: fifo.StaticFifo(u8, 16),
    cmd_delay: u32,
    params: fifo.StaticFifo(u8, 16),
    results: fifo.StaticFifo(u8, 16),

    disc: ?Disc,
    seekloc: ?u32,
    audio_buffer: *AudioBuffer,
    data_buffer: ?[]const u8,
    data_pos: u32,
    read_state: ReadState,
    read_delay: u32,

    irq_mask: packed struct(u8) { int_enable: u3 = 0, _pad: u5 = 0 },
    irq_pending: packed struct(u8) { ints: u3 = 0, _pad: u5 = 0 },

    pub fn init(allocator: std.mem.Allocator, bus: *mem.Bus) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");

        const audio_buffer = allocator.create(AudioBuffer) catch @panic("OOM");
        audio_buffer.* = .empty;

        self.* = .{
            .allocator = allocator,
            .params = .empty,
            .results = .empty,
            .irq_mask = std.mem.zeroes(@TypeOf(self.irq_mask)),
            .irq_pending = std.mem.zeroes(@TypeOf(self.irq_pending)),
            .audio_buffer = audio_buffer,
            .data_buffer = null,
            .data_pos = 0,
            .read_state = .idle,
            .read_delay = 0,
            .cmd = null,
            .cmd_state = .recv_cmd,
            .cmd_queue = .empty,
            .cmd_delay = 0,
            .disc = null,
            .mode = .{},
            .mute = false,
            .seekloc = null,
            .bus = bus,
            .stat = .{},
            .addr = .{},
            .req = .{},
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self.audio_buffer);
        self.allocator.destroy(self);
    }

    pub fn insertDisc(self: *@This(), disc: Disc) void {
        self.disc = disc;
    }

    // =========================================================================
    // Buffer management
    // =========================================================================

    inline fn fillAudioBuffer(self: *@This(), samples: []const i16) void {
        var offset: usize = 0;
        while (offset < samples.len) : (offset += 2) {
            const sample = [2]i16{
                samples[offset + 0],
                samples[offset + 1],
            };
            self.audio_buffer.push(sample);
        }
    }

    inline fn setDataBuffer(self: *@This(), data: []const u8) void {
        self.data_buffer = data;
        self.data_pos = 0;
    }

    pub fn consumeAudioSample(self: *@This()) [2]i16 {
        const is_playing = !self.mute and
            self.read_state == .playing and
            self.mode.cdda;
        if (is_playing) {
            return self.audio_buffer.pop() orelse blk: {
                log.warn("audio buffer underrun", .{});
                break :blk [2]i16{ 0, 0 };
            };
        }
        return [2]i16{ 0, 0 };
    }

    pub fn consumeData(self: *@This(), comptime T: type) T {
        if (self.data_buffer) |buf| {
            const v = mem.readBuf(T, buf, self.data_pos);
            self.data_pos += @sizeOf(T);

            if (self.data_pos >= buf.len) {
                self.data_buffer = null;
                self.data_pos = 0;
            }

            return v;
        } else {
            log.warn("read from empty data buffer", .{});
            return 0;
        }
    }

    fn consumeResultByte(self: *@This()) u8 {
        const v = self.results.pop() orelse blk: {
            log.warn("read from empty result fifo", .{});
            break :blk 0;
        };
        return v;
    }

    // =========================================================================
    // State machine
    // =========================================================================

    pub fn tick(self: *@This(), cyc: u32) void {
        if (self.irq_pending.ints != 0) {
            return;
        }
        if (self.cmd_delay > 0) {
            self.cmd_delay -|= cyc;
            if (self.cmd_delay == 0) {
                @branchHint(.unlikely);
                self.stepCommand();
            }
        } else if (self.read_delay > 0) {
            self.read_delay -|= cyc;
            if (self.read_delay == 0) {
                @branchHint(.unlikely);
                switch (self.read_state) {
                    .idle => unreachable,
                    .reading => self.stepReading(),
                    .playing => self.stepPlaying(),
                }
            }
        }
    }

    fn beginPlaying(self: *@This()) void {
        self.read_state = .playing;
        if (self.seekloc != null) {
            self.stat.seek = true;
            self.read_delay = cdrom_seekl_delay_cycles;
        } else {
            self.stat.play = true;
            self.read_delay = cdrom_read_delay_cycles;
        }
    }

    fn stepPlaying(self: *@This()) void {
        if (self.seekloc) |loc| {
            self.disc.?.seek(loc);
            self.seekloc = null;
            self.stat.seek = false;
            self.read_delay = cdrom_read_delay_cycles;
        } else {
            self.stat.play = true;
            const sector = self.disc.?.readSectorRaw();
            const samples = std.mem.bytesAsSlice(i16, sector);
            self.read_delay = cdrom_read_delay_cycles;
            self.fillAudioBuffer(samples);
        }
    }

    fn beginReading(self: *@This()) void {
        self.read_state = .reading;
        if (self.seekloc != null) {
            self.stat.seek = true;
            self.read_delay = cdrom_seekl_delay_cycles;
        } else {
            self.stat.read = true;
            self.read_delay = switch (self.mode.speed) {
                .normal => cdrom_read_delay_cycles,
                .double => cdrom_read_2x_delay_cycles,
            };
        }
    }

    fn stepReading(self: *@This()) void {
        if (self.seekloc) |loc| {
            self.disc.?.seek(loc);
            self.seekloc = null;
            self.stat.seek = false;
            self.read_delay = switch (self.mode.speed) {
                .normal => cdrom_read_delay_cycles,
                .double => cdrom_read_2x_delay_cycles,
            };
        } else {
            self.stat.read = true;
            self.results.clear();

            const sector = self.disc.?.readSector(self.mode.sector_size);
            self.read_delay = switch (self.mode.speed) {
                .normal => cdrom_read_delay_cycles,
                .double => cdrom_read_2x_delay_cycles,
            };

            self.pushResultByte(self.readStat());
            self.setDataBuffer(sector);
            self.setInterrupt(1);
        }
    }

    fn resetReadState(self: *@This()) void {
        self.read_state = .idle;
        self.read_delay = 0;
        self.audio_buffer.clear();
        self.data_buffer = null;
        self.data_pos = 0;
        self.stat.seek = false;
        self.stat.read = false;
        self.stat.play = false;
    }

    fn stepCommand(self: *@This()) void {
        if (self.cmd == null) return;
        const cmd = self.cmd.?;

        // This fixes some games but should we actually do this?
        self.irq_pending.ints = 0;
        self.results.clear();

        switch (cmd) {
            0x01 => commands.getStat(self),
            0x02 => commands.setLoc(self),
            0x03 => commands.play(self),
            0x06 => commands.readN(self),
            0x08 => commands.stop(self),
            0x09 => commands.pause(self),
            0x0a => commands.initCmd(self),
            0x0c => commands.demute(self),
            0x0d => commands.setFilter(self),
            0x0e => commands.setMode(self),
            0x11 => commands.getLocP(self),
            0x1b => commands.readN(self), // readS
            0x13 => commands.getTn(self),
            0x14 => commands.getTd(self),
            0x15 => commands.seekL(self),
            0x16 => commands.seekL(self), // seekP
            0x1a => commands.getId(self),
            0x19 => commands.testCmd(self),
            else => {
                // std.debug.panic("unhandled CDROM command: {x}", .{cmd});
                log.warn("unhandled CDROM command: {x}", .{cmd});
            },
        }
    }

    fn finishCommand(self: *@This()) void {
        self.params.clear();
        self.cmd_state = .recv_cmd;
        self.stat.err = false;
        self.cmd = null;
    }

    fn pushResultByte(self: *@This(), byte: u8) void {
        self.results.push(byte);
    }

    fn pushResultSlice(self: *@This(), bytes: []const u8) void {
        for (bytes) |b| self.pushResultByte(b);
    }

    fn pushError(self: *@This(), err_code: u8) void {
        var stat = self.stat;
        stat.err = true;
        self.pushResultByte(@bitCast(stat));
        self.pushResultByte(err_code);
    }

    // =========================================================================
    // Register I/O
    // =========================================================================

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const reg_id = bits.field(addr, 0, u2);
        const bank_index = self.addr.bank_index;

        const v: T = switch (reg_id) {
            0 => self.readAddr(),
            1 => self.consumeResultByte(),
            2 => self.consumeData(T),
            3 => switch (bank_index) {
                0, 2 => @as(u8, @bitCast(self.irq_mask)),
                1, 3 => @as(u8, @bitCast(self.irq_pending)) | 0xe0, // bits 5-7 always read as 1
            },
        };

        // log.debug("read: {x} = {x}", .{ addr, v });
        return v;
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        // log.debug("write: {x} = {x}", .{ addr, @as(u32, v) });

        const reg_id = bits.field(addr, 0, u2);
        const bank_index = self.addr.bank_index;

        const val: u8 = switch (T) {
            u8 => v,
            u16, u32 => @truncate(v),
            else => @compileError("CDROM.Write: Unsupported type"),
        };

        switch (reg_id) {
            0 => self.addr.bank_index = bits.field(val, 0, u2),
            1 => switch (bank_index) {
                0 => self.writeCommand(val),
                1 => log.warn("WRDATA: {x}", .{v}),
                2 => log.warn("CI: {x}", .{v}),
                3 => log.warn("AVT2: {x}", .{v}),
            },
            2 => switch (bank_index) {
                0 => self.writePram(val),
                1 => self.irq_mask = @bitCast(val),
                2 => log.warn("ATV0: {x}", .{v}),
                3 => log.warn("ATV3: {x}", .{v}),
            },
            3 => switch (bank_index) {
                0 => self.writeRequest(val),
                1 => self.ackInterrupt(val),
                2 => log.warn("ATV1: {x}", .{v}),
                3 => log.warn("ADPCTL: {x}", .{v}),
            },
        }
    }

    fn readAddr(self: *@This()) u8 {
        var addr = self.addr;
        addr.param_empty = self.params.isEmpty();
        addr.param_not_full = !self.params.isFull();
        addr.result_ready = !self.results.isEmpty();
        addr.data_ready = self.req.want_data and self.data_buffer != null;
        addr.busy_status = self.cmd_state != .recv_cmd;
        return @bitCast(addr);
    }

    fn readStat(self: *@This()) u8 {
        return @bitCast(self.stat);
    }

    fn writeRequest(self: *@This(), v: u8) void {
        const req: RequestReg = @bitCast(v);
        self.req = req;

        if (!req.want_data) {
            // self.data_buffer = null;
            self.data_pos = 0;
        }
    }

    fn writePram(self: *@This(), v: u8) void {
        self.params.push(v);
    }

    fn writeCommand(self: *@This(), opcode: u8) void {
        if (self.cmd != null and opcode != 0x09) {
            log.warn("new command ({x}) while another command in progress ({x})", .{ opcode, self.cmd.? });
        } else if (!self.results.isEmpty()) {
            log.warn("new command ({x}) while having unread results", .{opcode});
        }

        if (opcode == 0x09) { // pause
            self.cmd = opcode;
            self.cmd_state = .recv_cmd;
            self.stepCommand();
        } else {
            self.cmd_queue.push(opcode);
            if (self.cmd == null) {
                self.cmd = self.cmd_queue.pop().?;
                self.cmd_state = .recv_cmd;
                self.stepCommand();
            }
        }
    }

    // =========================================================================
    // Interrupts
    // =========================================================================

    fn setInterrupt(self: *@This(), it: u3) void {
        if (self.irq_pending.ints != 0) {
            log.warn("unacknowledged interrupt: {x} -> {x}", .{ self.irq_pending.ints, it });
        }

        self.irq_pending.ints = it;

        if (@as(u8, @bitCast(self.irq_pending)) & @as(u8, @bitCast(self.irq_mask)) != 0) {
            self.bus.setInterrupt(Interrupt.cdrom);
        }
    }

    fn ackInterrupt(self: *@This(), v: u8) void {
        const ack: packed struct(u8) {
            ack_int: u3,
            _pad: u3,
            clear_params: bool,
            reset_chip: bool,
        } = @bitCast(v);

        self.irq_pending.ints &= ~ack.ack_int;

        if (ack.clear_params) self.params.clear();

        if (self.cmd == null and !self.cmd_queue.isEmpty()) {
            self.cmd = self.cmd_queue.pop().?;
            self.results.clear();
            self.stepCommand();
        }

        // psx-spx states that the results queue is drained. Seems like it does not happen in reality.
        // pcsx-redux tests try to read the response AFTER acknowledging. Duckstation actually clears it
        // before writing a command response, so we will do that as well.
        // self.results.clear();
    }
};

const commands = opaque {
    fn getId(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                log.debug("CDROM GETID", .{});
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                self.pushResultByte(self.readStat());
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp2;
                self.setInterrupt(3);
            },
            .resp2 => {
                if (self.disc != null) {
                    self.pushResultSlice(&cdrom_getid_licensed);
                    self.setInterrupt(2);
                } else {
                    self.pushResultSlice(&cdrom_getid_nodisk);
                    self.setInterrupt(5);
                }
                self.finishCommand();
            },
        }
    }

    fn pause(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM PAUSE", .{});
                self.pushResultByte(self.readStat());
                self.cmd_delay = switch (self.mode.speed) {
                    .normal => cdrom_pause_delay_cycles,
                    .double => cdrom_pause_2x_delay_cycles,
                };
                self.cmd_state = .resp2;
                self.setInterrupt(3);
            },
            .resp2 => {
                self.pushResultByte(self.readStat());
                self.resetReadState();
                self.setInterrupt(2);
                self.finishCommand();
            },
        }
    }

    fn getStat(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                const stat = self.stat;
                self.stat.shellopen = false; // reading stat resets shellopen bit
                self.pushResultByte(@bitCast(stat));
                log.debug("CDROM GETSTAT: {any}", .{stat});
                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn setLoc(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                std.debug.assert(self.params.len == 3);
                const m = fromBCD(self.params.pop().?);
                const s = fromBCD(self.params.pop().?);
                const f = fromBCD(self.params.pop().?);

                self.seekloc = Position.init(m, s, f).toSectors();

                log.debug("CDROM SETLOC to {d}:{d}:{d}", .{ m, s, f });

                self.pushResultByte(self.readStat());
                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn setMode(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                std.debug.assert(self.params.len == 1);
                const mode_byte = self.params.pop().?;

                self.mode = @bitCast(mode_byte);
                self.pushResultByte(self.readStat());
                log.debug("CDROM SETMODE: {x}", .{mode_byte});

                if (self.mode.auto_pause) log.warn("auto-pause not implemented", .{});
                if (self.mode.play_report) log.warn("play-report not implemented", .{});

                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn seekL(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
                self.stat.seek = true;
                self.stat.motor_on = true;
            },
            .resp1 => {
                if (self.seekloc == null) {
                    std.debug.panic("CDROM SEEKL: no seek location set", .{});
                }
                const loc = self.seekloc.?;
                log.debug("CDROM SEEKL to {d}", .{loc});
                self.pushResultByte(self.readStat());
                self.cmd_delay = cdrom_seekl_delay_cycles;
                self.setInterrupt(3);
                self.cmd_state = .resp2;
            },
            .resp2 => {
                self.disc.?.seek(self.seekloc.?);
                self.seekloc = null; // acknowledge seek
                self.pushResultByte(self.readStat());
                self.stat.seek = false;
                self.setInterrupt(2);
                self.finishCommand();
            },
        }
    }

    fn readN(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
                self.stat.motor_on = true;
            },
            .resp1 => {
                log.debug("CDROM READN", .{});
                self.beginReading();
                self.pushResultByte(self.readStat());
                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn play(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
                self.stat.motor_on = true;
            },
            .resp1 => {
                const track_id = self.params.pop();
                if (track_id) |id| {
                    const track = self.disc.?.getTrackByNumber(id) orelse {
                        std.debug.panic("cdrom - play: invalid track id {d}", .{id});
                    };
                    self.seekloc = track.start_sector;
                    log.debug("seek to track {d}", .{id});
                }

                self.beginPlaying();
                self.pushResultByte(self.readStat());
                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn initCmd(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_init_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM INIT", .{});
                self.pushResultByte(self.readStat());
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp2;
                self.setInterrupt(3);
            },
            .resp2 => {
                self.stat.motor_on = true;
                self.mode = @bitCast(@as(u8, 0x20));
                self.resetReadState();
                self.pushResultByte(self.readStat());
                self.setInterrupt(2);
                self.finishCommand();
            },
        }
    }

    fn testCmd(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                std.debug.assert(self.params.len == 1);
                const sub_cmd = self.params.pop() orelse unreachable;
                switch (sub_cmd) {
                    0x20 => self.pushResultSlice(&cdrom_date_version),
                    else => log.warn("unhandled CDROM TEST sub-command: {x}", .{sub_cmd}),
                }
                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn demute(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM DEMUTE (stubbed)", .{});
                self.mute = false;
                self.pushResultByte(self.readStat());
                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn getTn(self: *CDROM) void {
        // std.debug.assert(self.params.len == 0);

        self.pushResultByte(self.readStat());
        self.pushResultByte(toBCD(1)); // first track number
        self.pushResultByte(toBCD(self.disc.?.track_count)); // last track number

        log.debug("GetTN -> INT3(stat,{d},{d})", .{ 1, self.disc.?.track_count });
        self.setInterrupt(3);
        self.finishCommand();
    }

    fn getTd(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                // std.debug.assert(self.params.len == 1);

                const track_num = fromBCD(self.params.pop() orelse 0);

                const track = self.disc.?.getTrackByNumber(track_num) orelse {
                    self.pushError(0x10);
                    self.setInterrupt(3);
                    self.finishCommand();
                    return;
                };

                if (track_num == 0) {
                    const end_pos = Position.fromSectors(track.end_sector);
                    self.pushResultByte(self.readStat());
                    self.pushResultByte(toBCD(end_pos.minute));
                    self.pushResultByte(toBCD(end_pos.second));
                    log.debug("GetTD({d}) -> INT3(stat, {d}, {d})", .{ track_num, end_pos.minute, end_pos.second });
                } else {
                    const start_pos = Position.fromSectors(track.start_sector);
                    self.pushResultByte(self.readStat());
                    self.pushResultByte(toBCD(start_pos.minute));
                    self.pushResultByte(toBCD(start_pos.second));
                    log.debug("GetTD({d}) -> INT3(stat, {d}, {d})", .{ track_num, start_pos.minute, start_pos.second });
                }

                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn getLocP(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM GETLOCP (stubbed)", .{});
                self.pushResultByte(self.readStat());

                self.pushResultByte(toBCD(1)); // track number
                self.pushResultByte(toBCD(1)); // track index

                self.pushResultByte(toBCD(0)); // track minute
                self.pushResultByte(toBCD(2)); // track second
                self.pushResultByte(toBCD(0)); // track sect

                self.pushResultByte(toBCD(0)); // absolute minute
                self.pushResultByte(toBCD(2)); // absolute second
                self.pushResultByte(toBCD(0)); // absolute sect

                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn setFilter(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                std.debug.assert(self.params.len == 2);

                const file = self.params.pop() orelse unreachable;
                const channel = self.params.pop() orelse unreachable;

                log.debug("CDROM SETFILTER (stubbed), file={x}, channel={x}", .{ file, channel });
                self.pushResultByte(self.readStat());
                self.setInterrupt(3);
                self.finishCommand();
            },
            else => unreachable,
        }
    }

    fn stop(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM STOP", .{});
                self.pushResultByte(self.readStat());
                self.stat.read = false;
                self.stat.play = false;
                self.audio_buffer.clear();
                self.setInterrupt(3);
                self.cmd_delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp2;
            },
            .resp2 => {
                self.stat.motor_on = false;
                self.pushResultByte(self.readStat());
                self.setInterrupt(2);
                self.finishCommand();
            },
        }
    }
};

inline fn fromBCD(bcd: u8) u8 {
    return (bcd >> 4) * 10 + (bcd & 0x0f);
}

inline fn toBCD(dec: u8) u8 {
    return ((dec / 10) << 4) | (dec % 10);
}

test "toBCD/fromBCD" {
    for (0..100) |i| {
        const v: u8 = @intCast(i);
        const bcd = toBCD(v);
        const dec = fromBCD(bcd);
        try std.testing.expectEqual(v, dec);
    }
}
