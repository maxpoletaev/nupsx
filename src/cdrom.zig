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
const cdrom_read_delay_cycles = (33868800 / 75);
const cdrom_read_2x_delay_cycles = (33868800 / (75 * 2));

const cdrom_getid_nodisk = [_]u8{ 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const cdrom_getid_region = 'A'; // A=America, I=Japan, E=Europe
const cdrom_getid_licensed = [_]u8{ 0x02, 0x00, 0x20, 0x00, 'S', 'C', 'E', cdrom_getid_region };
const cdrom_date_version = [_]u8{ 0x94, 0x09, 0x19, 0xC0 };

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
    _pad: u5 = 0, // 0-4 (always 0)
    _unused: u2 = 0, // 5-6 (smen and bfwr)
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

    pub fn loadCue(allocator: std.mem.Allocator, cue_path: []const u8) !@This() {
        var cue_sheet = try cue.parseFile(allocator, cue_path);
        defer cue.free(allocator, &cue_sheet);

        const cue_root = std.fs.path.dirname(cue_path) orelse ".";
        const cue_dir = try std.fs.cwd().openDir(cue_root, .{});

        var tracks: std.ArrayList(Track) = .empty;
        var absolute_sector: u32 = cdrom_file_offset_sectors_cue;
        var total_size: usize = 0;

        for (cue_sheet.files) |file| {
            const stat = try cue_dir.statFile(file.filename);
            const file_sector_count: u32 = @intCast(@divExact(stat.size, cdrom_sector_size_cue));

            for (file.tracks) |track| {
                const index_01: ?cue.IndexNode = for (track.indices) |idx| {
                    if (idx.number == 1) break idx;
                } else null;

                if (index_01 == null) std.debug.panic("track {d} does not have index 01", .{track.number});
                const track_start = absolute_sector + index_01.?.position.toSectors();
                const track_end = track_start + file_sector_count;

                try tracks.append(allocator, Track{
                    .number = track.number,
                    .start_sector = track_start,
                    .end_sector = track_end,
                });
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

        var disc_data = try allocator.alignedAlloc(u8, .@"2", total_size);
        var reader_buf: [8192]u8 = undefined;
        var offset: usize = 0;

        for (cue_sheet.files) |file| {
            const bin_file = try cue_dir.openFile(file.filename, .{ .mode = .read_only });
            defer bin_file.close();

            const file_size = try bin_file.getEndPos();
            var reader = bin_file.reader(&reader_buf);
            try reader.interface.readSliceAll(disc_data[offset .. offset + file_size]);
            log.info("loaded bin file: {s} ({d} bytes)", .{ file.filename, file_size });

            offset += file_size;
        }

        std.debug.assert(tracks.items.len > 0);
        const track_count = tracks.items.len;

        return .{
            .allocator = allocator,
            .tracks = tracks.toOwnedSlice(allocator) catch @panic("OOM"),
            .track_count = @intCast(track_count),
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
            std.debug.panic(
                "disc read out of bounds at {d} (sector {d})",
                .{ self.pos, self.pos / cdrom_sector_size_cue },
            );
        }

        const sector = self.data[self.pos .. self.pos + cdrom_sector_size_cue];

        const v = switch (sector_size) {
            .data_only => sector[24 .. 24 + 2048],
            .whole_sector => sector[12..sector.len],
        };

        self.pos += cdrom_sector_size_cue;
        return @alignCast(v);
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
    read,
};

pub const CDROM = struct {
    pub const addr_start: u32 = 0x1f801800;
    pub const addr_end: u32 = 0x1f801803;

    allocator: std.mem.Allocator,
    addr: AddressReg,
    stat: StatusReg,
    mode: ModeReg,
    req: RequestReg,
    params: fifo.StaticFifo(u8, 16),
    results: fifo.StaticFifo(u8, 16),
    delay: u32 = 0,
    cmd: ?u8,
    cmd_state: CmdState,
    cmd_queue: fifo.StaticFifo(u8, 16),
    bus: *mem.Bus,
    mute: bool = false,

    disc: ?Disc,
    seekloc: ?u32,
    sect_buf: ?[]const u8,
    sect_pos: u32 = 0,
    cdda_buf: ?[]const i16,
    cdda_pos: u32 = 0,

    irq_mask: packed struct(u8) { int_enable: u3 = 0, _pad: u5 = 0 },
    irq_pending: packed struct(u8) { ints: u3 = 0, _pad: u5 = 0 },

    pub fn init(allocator: std.mem.Allocator, bus: *mem.Bus) *@This() {
        const self = allocator.create(@This()) catch unreachable;

        self.* = .{
            .allocator = allocator,
            .params = .empty,
            .results = .empty,
            .irq_mask = std.mem.zeroes(@TypeOf(self.irq_mask)),
            .irq_pending = std.mem.zeroes(@TypeOf(self.irq_pending)),
            .cmd = null,
            .cmd_state = .recv_cmd,
            .cmd_queue = .empty,
            .disc = null,
            .sect_buf = null,
            .sect_pos = 0,
            .cdda_buf = null,
            .cdda_pos = 0,
            .mode = .{},
            .seekloc = null,
            .bus = bus,
            .stat = .{},
            .addr = .{},
            .req = .{},
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn insertDisc(self: *@This(), disc: Disc) void {
        self.disc = disc;
    }

    pub fn consumeAudioSample(self: *@This()) [2]i16 {
        if (self.mute) {
            if (self.cdda_buf != null) self.cdda_pos += 2; // advance even when muted
            return .{ 0, 0 };
        }

        if (self.cdda_buf) |buf| {
            const left = buf[self.cdda_pos + 0];
            const right = buf[self.cdda_pos + 1];
            self.cdda_pos += 2;

            if (self.cdda_pos >= buf.len) {
                self.cdda_buf = null;
                self.cdda_pos = 0;
            }

            return .{ left, right };
        }

        return .{ 0, 0 };
    }

    fn setAudioBuffer(self: *@This(), data_ref: []const i16) void {
        self.cdda_buf = data_ref;
        self.cdda_pos = 0;
    }

    fn clearAudioBuffer(self: *@This()) void {
        self.cdda_buf = null;
        self.cdda_pos = 0;
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const reg_id = bits.field(addr, 0, u2);
        const bank_index = self.addr.bank_index;

        const v: T = switch (reg_id) {
            0 => self.readAddressReg(),
            1 => self.consumeResultByte(),
            2 => self.consumeSectorData(T),
            3 => switch (bank_index) {
                0, 2 => @as(u8, @bitCast(self.irq_mask)),
                1, 3 => @as(u8, @bitCast(self.irq_pending)) | 0xe0, // bits 5-7 always read as 1
            },
        };

        // log.debug("read: {x} = {x}", .{ addr, v });
        return v;
    }

    pub fn consumeSectorData(self: *@This(), comptime T: type) T {
        if (self.sect_buf == null) {
            std.debug.panic("cdrom: no sector data available", .{});
        }

        const buf = self.sect_buf.?;
        const v = mem.readBuf(T, buf, self.sect_pos);
        self.sect_pos += @sizeOf(T);

        if (self.sect_pos >= buf.len) {
            self.addr.data_ready = false;
            self.sect_buf = null;
            self.sect_pos = 0;
        }

        return v;
    }

    fn setSectorData(self: *@This(), data_ref: []const u8) void {
        self.sect_buf = data_ref;
        self.sect_pos = 0;
    }

    fn readAddressReg(self: *@This()) u8 {
        var addr = self.addr;
        addr.param_empty = self.params.isEmpty();
        addr.param_not_full = !self.params.isFull();
        addr.result_ready = !self.results.isEmpty();
        addr.data_ready = self.req.want_data and (self.sect_buf != null);
        addr.busy_status = self.cmd_state != .recv_cmd;
        return @bitCast(addr);
    }

    fn consumeResultByte(self: *@This()) u8 {
        if (self.results.isEmpty()) {
            log.warn("cdrom: read from empty result fifo", .{});
            // log.debug("<< result byte: {x} (empty)", .{0xbe});
            return 0;
        }
        const v = self.results.pop().?;
        // log.debug("<< result byte: {x}", .{v});
        return v;
    }

    pub fn tick(self: *@This(), cyc: u32) void {
        if (self.delay > 0) {
            self.delay -|= cyc;
            if (self.delay > 0) return;
        }

        if (self.cmd) |cmd| {
            self.results.clear();
            self.stepCommand(cmd);

            if (self.cmd_state == .recv_cmd) {
                if (self.cmd_queue.isEmpty()) {
                    self.cmd = null;
                } else {
                    self.cmd = self.cmd_queue.pop() orelse unreachable;
                    self.stepCommand(self.cmd.?); // schedule next command
                }
            }
        }
    }

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

        // psx-spx states that the results queue is drained. Seems like it does not happen in reality.
        // pcsx-redux tests try to read the response AFTER acknowledging. Duckstation actually clears it
        // before writing a command response, so we will do that as well.
        // self.results.clear();
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

    fn writeRequest(self: *@This(), v: u8) void {
        const req: RequestReg = @bitCast(v);
        self.req = req;

        if (!req.want_data) {
            // self.sect_buf = null;
            self.sect_pos = 0;
        }
    }

    fn writePram(self: *@This(), v: u8) void {
        self.params.push(v);
    }

    fn writeCommand(self: *@This(), opcode: u8) void {
        // if (self.cmd != null and opcode != 0x09) {
        //     std.debug.panic("cdrom: new command ({x}) while another command in progress ({x})", .{ opcode, self.cmd.? });
        // } else if (!self.results.isEmpty()) {
        //     std.debug.panic("cdrom: new command ({x}) while having unread results", .{opcode});
        // }

        // if (opcode == 0x03) {
        //     std.debug.panic("cdrom: play command received", .{});
        // }

        if (opcode == 0x09) {
            self.cmd = opcode;
            self.cmd_state = .recv_cmd;
            self.stepCommand(opcode);
            return;
        }

        self.cmd_queue.push(opcode);

        if (self.cmd_state == .recv_cmd) {
            self.cmd = self.cmd_queue.pop().?;
            self.stepCommand(opcode);
        }
    }

    fn stepCommand(self: *@This(), cmd: u8) void {
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

    fn resetCommand(self: *@This()) void {
        self.cmd = null;
        self.params.clear();
        self.cmd_state = .recv_cmd;
        self.stat.read = false;
        self.stat.play = false;
        self.stat.seek = false;
        self.stat.err = false;
    }

    fn pushResultByte(self: *@This(), byte: u8) void {
        self.results.push(byte);
    }

    fn pushResultSlice(self: *@This(), bytes: []const u8) void {
        for (bytes) |b| {
            self.pushResultByte(b);
        }
    }

    fn pushError(self: *@This(), err_code: u8) void {
        var stat = self.stat;
        stat.err = true;
        self.pushResultByte(@bitCast(stat));
        self.pushResultByte(err_code);
    }
};

const commands = opaque {
    fn getId(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                log.debug("CDROM GETID", .{});
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                self.pushResultByte(@bitCast(self.stat));
                self.delay = cdrom_avg_delay_cycles;
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
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn pause(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM PAUSE", .{});
                self.pushResultByte(@bitCast(self.stat));
                self.stat.read = false;
                self.setInterrupt(3);
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp2;
            },
            .resp2 => {
                self.pushResultByte(@bitCast(self.stat));
                self.clearAudioBuffer();
                self.setInterrupt(2);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn play(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;

                const track_id = self.params.pop();
                if (track_id) |id| {
                    const track = self.disc.?.getTrackByNumber(id) orelse {
                        std.debug.panic("cdrom - play: invalid track id {d}", .{id});
                    };
                    self.disc.?.seek(track.start_sector);
                    log.debug("seek to track {d}", .{id});
                } else if (self.seekloc) |loc| {
                    log.debug("seek to loc {d}", .{loc});
                    self.disc.?.seek(loc);
                    self.seekloc = null;
                    self.stat.seek = true;
                }
            },
            .resp1 => {
                const sector = self.disc.?.readSector(.data_only);
                const buf = std.mem.bytesAsSlice(i16, sector);
                self.setAudioBuffer(buf);

                self.pushResultByte(@bitCast(self.stat));
                self.stat.play = true;
                self.setInterrupt(3);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn getStat(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                const stat = self.stat;
                self.stat.shellopen = false; // reading stat resets shellopen bit
                self.pushResultByte(@bitCast(stat));
                log.debug("CDROM GETSTAT: {any}", .{stat});
                self.setInterrupt(3);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn setLoc(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                std.debug.assert(self.params.len == 3);
                const m = fromBCD(self.params.pop().?);
                const s = fromBCD(self.params.pop().?);
                const f = fromBCD(self.params.pop().?);

                self.seekloc = Position.init(m, s, f).toSectors();

                log.debug("CDROM SETLOC to {d}:{d}:{d}", .{ m, s, f });

                self.pushResultByte(@bitCast(self.stat));
                self.setInterrupt(3);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn setMode(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                std.debug.assert(self.params.len == 1);
                const mode_byte = self.params.pop().?;

                self.mode = @bitCast(mode_byte);
                self.pushResultByte(@bitCast(self.stat));
                log.debug("CDROM SETMODE: {x}", .{mode_byte});

                if (self.mode.auto_pause) log.warn("auto-pause not implemented", .{});
                if (self.mode.play_report) log.warn("play-report not implemented", .{});

                self.setInterrupt(3);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn seekL(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
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
                self.pushResultByte(@bitCast(self.stat));
                self.delay = cdrom_seekl_delay_cycles;
                self.setInterrupt(3);
                self.cmd_state = .resp2;
            },
            .resp2 => {
                self.disc.?.seek(self.seekloc.?);
                self.seekloc = null; // acknowledge seek
                self.pushResultByte(@bitCast(self.stat));
                self.stat.seek = false;
                self.setInterrupt(2);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn readN(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
                self.stat.motor_on = true;

                if (self.seekloc) |loc| { // unacknowledged seek
                    log.debug("CDROM READN seek to {d}", .{loc});
                    self.delay += cdrom_seekl_delay_cycles;
                    self.disc.?.seek(loc);
                    self.seekloc = null;
                    self.stat.seek = true;
                }
            },
            .resp1 => {
                log.debug("CDROM READN", .{});
                self.pushResultByte(@bitCast(self.stat));
                self.delay = switch (self.mode.speed) {
                    .normal => cdrom_read_delay_cycles,
                    .double => cdrom_read_2x_delay_cycles,
                };
                self.cmd_state = .read;
                self.stat.seek = false;
                self.stat.read = true;
                self.setInterrupt(3);
            },
            .read => {
                self.pushResultByte(@bitCast(self.stat));
                const sect_buf = self.disc.?.readSector(self.mode.sector_size);
                self.setSectorData(sect_buf);
                self.delay = switch (self.mode.speed) {
                    .normal => cdrom_read_delay_cycles,
                    .double => cdrom_read_2x_delay_cycles,
                };
                self.addr.data_ready = true;
                self.setInterrupt(1);
            },
            else => unreachable,
        }
    }

    fn initCmd(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_init_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM INIT", .{});
                self.cmd_state = .resp2;
                self.mode = @bitCast(@as(u8, 0x20));
                self.stat.motor_on = true;
                self.pushResultByte(@bitCast(self.stat));
                self.delay = cdrom_avg_delay_cycles;
                self.setInterrupt(3);
            },
            .resp2 => {
                self.pushResultByte(@bitCast(self.stat));
                self.setInterrupt(2);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn testCmd(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
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
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn demute(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM DEMUTE (stubbed)", .{});
                self.mute = false;
                self.pushResultByte(@bitCast(self.stat));
                self.setInterrupt(3);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn getTn(self: *CDROM) void {
        // std.debug.assert(self.params.len == 0);

        self.pushResultByte(@bitCast(self.stat));
        self.pushResultByte(toBCD(1)); // first track number
        self.pushResultByte(toBCD(self.disc.?.track_count)); // last track number

        log.debug("GetTN -> INT3(stat,{d},{d})", .{ 1, self.disc.?.track_count });
        self.setInterrupt(3);
        self.resetCommand();
    }

    fn getTd(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                // std.debug.assert(self.params.len == 1);

                const track_num = fromBCD(self.params.pop() orelse 0);

                const track = self.disc.?.getTrackByNumber(track_num) orelse {
                    self.pushError(0x10);
                    self.setInterrupt(3);
                    self.resetCommand();
                    return;
                };

                if (track_num == 0) {
                    const end_pos = Position.fromSectors(track.end_sector);
                    self.pushResultByte(@bitCast(self.stat));
                    self.pushResultByte(toBCD(end_pos.minute));
                    self.pushResultByte(toBCD(end_pos.second));
                    log.debug("GetTD({d}) -> INT3(stat, {d}, {d})", .{ track_num, end_pos.minute, end_pos.second });
                } else {
                    const start_pos = Position.fromSectors(track.start_sector);
                    self.pushResultByte(@bitCast(self.stat));
                    self.pushResultByte(toBCD(start_pos.minute));
                    self.pushResultByte(toBCD(start_pos.second));
                    log.debug("GetTD({d}) -> INT3(stat, {d}, {d})", .{ track_num, start_pos.minute, start_pos.second });
                }

                self.setInterrupt(3);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn getLocP(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM GETLOCP (stubbed)", .{});
                self.pushResultByte(@bitCast(self.stat));

                self.pushResultByte(toBCD(1)); // track number
                self.pushResultByte(toBCD(1)); // track index

                self.pushResultByte(toBCD(0)); // track minute
                self.pushResultByte(toBCD(2)); // track second
                self.pushResultByte(toBCD(0)); // track sect

                self.pushResultByte(toBCD(0)); // absolute minute
                self.pushResultByte(toBCD(2)); // absolute second
                self.pushResultByte(toBCD(0)); // absolute sect

                self.setInterrupt(3);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn setFilter(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                std.debug.assert(self.params.len == 2);

                const file = self.params.pop() orelse unreachable;
                const channel = self.params.pop() orelse unreachable;

                log.debug("CDROM SETFILTER (stubbed), file={x}, channel={x}", .{ file, channel });
                self.pushResultByte(@bitCast(self.stat));
                self.setInterrupt(3);
                self.resetCommand();
            },
            else => unreachable,
        }
    }

    fn stop(self: *CDROM) void {
        switch (self.cmd_state) {
            .recv_cmd => {
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp1;
            },
            .resp1 => {
                log.debug("CDROM STOP", .{});
                self.pushResultByte(@bitCast(self.stat));
                self.stat.read = false;
                self.stat.play = false;
                self.clearAudioBuffer();
                self.setInterrupt(3);
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp2;
            },
            .resp2 => {
                self.stat.motor_on = false;
                self.pushResultByte(@bitCast(self.stat));
                self.setInterrupt(2);
                self.resetCommand();
            },
            else => unreachable,
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
