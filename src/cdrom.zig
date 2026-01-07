const std = @import("std");
const bits = @import("bits.zig");
const mem_mod = @import("mem.zig");
const cue_mod = @import("cue.zig");
const fifo = @import("fifo.zig");

const log = std.log.scoped(.cdrom);

const cdrom_sector_size_cue = 2352;
const cdrom_file_offset_sectors_cue = 150;
const cdrom_file_offset_bytes_cue = cdrom_file_offset_sectors_cue * cdrom_sector_size_cue;
const cdrom_avg_delay_cycles = 50_000;
const cdrom_seekl_delay_cycles = 450_000;
const cdrom_read_delay_cycles = (33868800 / 75);
const cdrom_read_2x_delay_cycles = (33868800 / (75 * 2));
const cdrom_getid_nodisk = [_]u8{ 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const cdrom_getid_region = 'A'; // A=America, I=Japan, E=Europe
const cdrom_getid_licensed = [_]u8{ 0x02, 0x00, 0x20, 0x00, 'S', 'C', 'E', cdrom_getid_region };
const cdrom_date_version = [_]u8{ 0x94, 0x09, 0x19, 0xC0 };

const AddressReg = packed struct(u8) {
    bank_index: u2 = 0, // RA
    adpcm_busy: bool = false, // ADPBUSY
    param_not_full: bool = true, // PRMWRDY (set on read)
    param_empty: bool = true, // PRMEMPT (set on read)
    result_ready: bool = false, // RSLRRDY
    data_ready: bool = false, // DRQSTS
    busy_status: bool = false, // BUSYSTS
};

const StatusReg = packed struct(u8) {
    err: bool = false,
    motor_on: bool = false,
    seekerr: bool = false,
    iderr: bool = false,
    shellopen: bool = false,
    read: bool = false,
    seek: bool = false,
    play: bool = false,
};

const SectorSize = enum(u1) {
    data_only = 0,
    whole_sector = 1,
};

const ModeReg = packed struct(u8) {
    cdda: bool = false,
    auto_pause: bool = false,
    report: bool = false,
    ignore_bit: bool = false,
    xa_filter: bool = false,
    sector_size: SectorSize = .data_only,
    xa_adpcm: bool = false,
    speed: enum(u1) { normal = 0, double = 1 } = .normal,
};

const RequestReg = packed struct(u8) {
    _pad: u5 = 0, // 0-4 (always 0)
    _unused: u2 = 0, // 5-6 (smen and bfwr)
    want_data: bool = false, // 7
};

const CdromEvents = packed struct(u8) {
    interrupt: bool = false,
    _pad: u7 = 0,
};

const SeekLocation = struct {
    minute: u8 = 0,
    second: u8 = 0,
    sector: u8 = 0,

    pub fn lba(self: SeekLocation) u32 {
        const m = @as(u32, self.minute);
        const s = @as(u32, self.second);
        const f = @as(u32, self.sector);
        return (m * 60 + s) * 75 + f;
    }
};

pub const Disc = struct {
    allocator: std.mem.Allocator,
    data: []const u8,
    pos: u32 = 0,

    pub fn fromBinFile(allocator: std.mem.Allocator, path: []const u8) !@This() {
        const file = try std.fs.cwd().openFile(path, .{ .mode = .read_only });
        defer file.close();

        const file_size = try file.getEndPos();
        const buffer = try allocator.alloc(u8, file_size);
        _ = try file.readAll(buffer[0..file_size]);

        log.info("loaded disc image: {s} ({d} bytes)", .{ path, file_size });

        return .{
            .allocator = allocator,
            .data = buffer,
        };
    }

    pub fn fromCueFile(allocator: std.mem.Allocator, cue_path: []const u8) !@This() {
        var cue_sheet = try cue_mod.parse(allocator, cue_path);
        defer cue_sheet.deinit();

        const bin_path = cue_sheet.files[0].path;
        return try fromBinFile(allocator, bin_path);
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.free(self.data);
    }

    pub fn seek(self: *@This(), loc: SeekLocation) void {
        self.pos = loc.lba() * cdrom_sector_size_cue;
        self.pos -= cdrom_file_offset_bytes_cue;
    }

    pub fn readSector(self: *@This(), sector_size: SectorSize) []const u8 {
        if (self.pos + cdrom_sector_size_cue > self.data.len) {
            std.debug.panic(
                "disc read out of bounds at {d} (sector {d})",
                .{ self.pos, self.pos / cdrom_sector_size_cue },
            );
        }

        const sector = self.data[self.pos .. self.pos + cdrom_sector_size_cue];

        const v = switch (sector_size) {
            .data_only => sector[24 .. 24 + 2048],
            .whole_sector => @panic("whole_sector not implemented"),
        };

        self.pos += cdrom_sector_size_cue;
        return v;
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
    cmd_state: CmdState = .recv_cmd,
    events: CdromEvents,
    mute: bool = false,

    disc: ?Disc,
    seekloc: ?SeekLocation,
    sect_buf: ?[]const u8,
    sect_pos: u32 = 0,

    irq_mask: packed struct(u8) { int_enable: u3 = 0, _pad: u5 = 0 },
    irq_pending: packed struct(u8) { ints: u3 = 0, _pad: u5 = 0 },

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch unreachable;

        self.* = .{
            .allocator = allocator,
            .params = .init(),
            .results = .init(),
            .irq_mask = std.mem.zeroes(@TypeOf(self.irq_mask)),
            .irq_pending = std.mem.zeroes(@TypeOf(self.irq_pending)),
            .cmd = null,
            .disc = null,
            .sect_buf = null,
            .mode = .{},
            .seekloc = .{},
            .events = .{},
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

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        // log.debug("CDROM read: {x}", .{addr});

        const reg_id = bits.field(addr, 0, u2);
        const bank_index = self.addr.bank_index;

        return switch (reg_id) {
            0 => @as(u8, @bitCast(self.addr)),
            1 => self.consumeResultByte(),
            2 => self.consumeSectorData(T),
            3 => switch (bank_index) {
                0, 2 => @as(u8, @bitCast(self.irq_mask)),
                1, 3 => @as(u8, @bitCast(self.irq_pending)) | 0xe0, // bits 5-7 always read as 1
            },
        };
    }

    pub fn consumeSectorData(self: *@This(), comptime T: type) T {
        if (self.sect_buf == null) {
            std.debug.panic("cdrom: no sector data available", .{});
        }

        const buf = self.sect_buf.?;
        const v = mem_mod.readBuf(T, buf, self.sect_pos);
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
        const addr = self.addr;
        addr.param_empty = self.params.isEmpty();
        addr.param_not_full = !self.params.isFull();
        addr.result_ready = !self.results.isEmpty();
        addr.data_ready = self.req.want_data and (self.sect_buf != null);
        return @bitCast(addr);
    }

    fn consumeResultByte(self: *@This()) u8 {
        if (self.results.isEmpty()) {
            log.warn("cdrom: read from empty result fifo", .{});
            return 0;
        }
        const v = self.results.pop() orelse 0;
        log.debug("CDROM result byte: {x}", .{v});
        return v;
    }

    pub fn tick(self: *@This(), cyc: u32) void {
        if (self.delay > 0) {
            self.delay -|= cyc;
            if (self.delay > 0) return;
        }
        if (self.cmd) |cmd| {
            self.stepCommand(cmd);
        }
    }

    pub fn consumeEvents(self: *@This()) CdromEvents {
        const ev = self.events;
        self.events = .{};
        return ev;
    }

    fn setInterrupt(self: *@This(), it: u3) void {
        if (self.irq_pending.ints != 0) {
            std.debug.panic("unacknowledged interrupt: {x} -> {x}", .{ self.irq_pending.ints, it });
        }
        self.irq_pending.ints = it;

        if (@as(u8, @bitCast(self.irq_pending)) & @as(u8, @bitCast(self.irq_mask)) != 0) {
            self.events.interrupt = true;
        }

        log.debug("CDROM interrupt set {x}", .{it});
    }

    fn ackInterrupt(self: *@This(), v: u8) void {
        const ack: packed struct(u8) {
            ack_int: u3,
            _pad: u3,
            clear_params: bool,
            reset_chip: bool,
        } = @bitCast(v);

        self.irq_pending.ints &= ~ack.ack_int;

        log.debug("CDROM interrupt ack {x}", .{ack.ack_int});

        if (ack.clear_params) self.params.clear();

        self.results.clear();
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
            self.sect_pos = 0;
        }
    }

    fn writePram(self: *@This(), v: u8) void {
        self.params.push(v) catch {
            std.debug.panic("cdrom: parameter fifo overflow", .{});
        };
    }

    fn writeCommand(self: *@This(), opcode: u8) void {
        if (self.cmd != null and opcode != commands.PAUSE) {
            std.debug.panic("cdrom: new command ({x}) while another command in progress ({x})", .{ opcode, self.cmd.? });
        } else if (!self.results.isEmpty()) {
            std.debug.panic("cdrom: new command ({x}) while having unread results", .{opcode});
        }
        self.cmd = opcode;
        self.cmd_state = .recv_cmd;
        self.stepCommand(opcode);
    }

    fn stepCommand(self: *@This(), cmd: u8) void {
        if (commands.table[cmd]) |f| {
            f(self);
            return;
        }
        std.debug.panic("unhandled CDROM command: {x}", .{cmd});
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
        self.results.push(byte) catch {
            std.debug.panic("cdrom: result fifo overflow", .{});
        };
    }

    fn pushResultSlice(self: *@This(), bytes: []const u8) void {
        for (bytes) |b| {
            self.pushResultByte(b);
        }
    }
};

const commands = opaque {
    const GET_STAT: u8 = 0x01;
    const SET_LOC: u8 = 0x02;
    const READ_N: u8 = 0x06;
    const PAUSE: u8 = 0x09;
    const INIT: u8 = 0x0a;
    const DEMUTE: u8 = 0x0c;
    const SET_MODE: u8 = 0x0e;
    const GET_ID: u8 = 0x1a;
    const SEEK_L: u8 = 0x15;
    const TEST: u8 = 0x19;

    const table = init: {
        const CmdPtr = *const fn (self: *CDROM) void;
        var cmds = std.mem.zeroes([256]?CmdPtr);

        cmds[GET_STAT] = getStat;
        cmds[SET_LOC] = setLoc;
        cmds[READ_N] = readN;
        cmds[PAUSE] = pause;
        cmds[INIT] = initCmd;
        cmds[SET_MODE] = setMode;
        cmds[GET_ID] = getId;
        cmds[SEEK_L] = seekL;
        cmds[TEST] = testCmd;
        cmds[DEMUTE] = demute;

        break :init cmds;
    };

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
                self.setInterrupt(2);
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
                log.debug("CDROM GETSTAT: {x}", .{@as(u8, @bitCast(stat))});
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
                const minute = bcdToDec(self.params.pop() orelse unreachable);
                const second = bcdToDec(self.params.pop() orelse unreachable);
                const sector = bcdToDec(self.params.pop() orelse unreachable);

                self.seekloc = SeekLocation{ .minute = minute, .second = second, .sector = sector };

                log.debug("CDROM SETLOC to {d}:{d}:{d}", .{ minute, second, sector });

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
                const mode_byte = self.params.pop() orelse unreachable;
                self.mode = @bitCast(mode_byte);
                log.debug("CDROM SETMODE: {x}", .{mode_byte});
                self.pushResultByte(@bitCast(self.stat));
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
                log.debug(
                    "CDROM SEEKL to {d}:{d}:{d} (lba={d})",
                    .{ loc.minute, loc.second, loc.sector, loc.lba() },
                );
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
                    log.debug("CDROM READN seek to {d}:{d}:{d} (lba={d})", .{ loc.minute, loc.second, loc.sector, loc.lba() });
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
                self.delay = cdrom_avg_delay_cycles;
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
};

inline fn bcdToDec(bcd: u8) u8 {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}
