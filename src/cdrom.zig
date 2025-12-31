const std = @import("std");
const bits = @import("bits.zig");
const mem_mod = @import("mem.zig");

const log = std.log.scoped(.cdrom);

const cdrom_date_version = [_]u8{ 0x94, 0x09, 0x19, 0xC0 };
const cdrom_sector_size_cue = 2352;
const cdrom_avg_delay_cycles = 50_000;
const cdrom_seekl_delay_cycles = 450_000;
const cdrom_read_normal_delay_cycles = (33868800 / 75);
const cdrom_read_double_delay_cycles = (33868800 / (75));
const cdrom_getid_nodisk = [_]u8{ 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const cdrom_getid_licensed = [_]u8{ 0x02, 0x00, 0x20, 0x00, 'S', 'C', 'E', 'A' };

const AddressReg = packed struct(u8) {
    bank_index: u2 = 0, // RA
    adpcm_busy: bool = false, // ADPBUSY
    param_empty: bool = true, // PRMEMPT
    param_ready: bool = true, // PRMWRDY
    result_ready: bool = false, // RSLRRDY
    data_request: bool = false, // DRQSTS
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
    smen: bool = false, // 5
    bfwr: bool = false, // 6
    bfrd: bool = false, // 7
};

const CdromEvents = packed struct(u8) {
    interrupt: bool = false,
    _pad: u7 = 0,
};

const SeekLocation = struct {
    minute: u8 = 0,
    second: u8 = 0,
    sector: u8 = 0,

    inline fn lba(self: SeekLocation) u32 {
        const m = @as(u32, self.minute);
        const s = @as(u32, self.second);
        const f = @as(u32, self.sector);
        return (m * 60 + s) * 75 + f;
    }
};

const Fifo = struct {
    data: [16]u8 = std.mem.zeroes([16]u8),
    read_pos: u4 = 0,
    write_pos: u4 = 0,

    pub fn init() Fifo {
        return Fifo{};
    }

    fn clear(self: *Fifo) void {
        self.read_pos = 0;
        self.write_pos = 0;
    }

    fn isEmpty(self: *Fifo) bool {
        return self.read_pos == self.write_pos;
    }

    fn writeByte(self: *Fifo, byte: u8) void {
        self.data[self.write_pos] = byte;
        self.write_pos +%= 1;
    }

    fn writeSlice(self: *Fifo, bytes: []const u8) void {
        for (bytes) |b| {
            self.writeByte(b);
        }
    }

    fn readByte(self: *Fifo) u8 {
        const byte = self.data[self.read_pos];
        self.read_pos +%= 1;
        return byte;
    }
};

pub const Disc = struct {
    allocator: std.mem.Allocator,
    data: []const u8,
    pos: u32 = 0,

    pub fn fromFile(allocator: std.mem.Allocator, path: []const u8) !@This() {
        const file = try std.fs.cwd().openFile(path, .{ .mode = .read_only });
        defer file.close();

        const file_size = try file.getEndPos();
        const pad_size = 150 * cdrom_sector_size_cue; // 150 sectors gap (missing in images)
        const buffer = try allocator.alloc(u8, file_size + pad_size);
        _ = try file.readAll(buffer[pad_size .. pad_size + file_size]);

        log.info("loaded disc image: {s} ({d} bytes)", .{ path, file_size });

        return .{
            .allocator = allocator,
            .data = buffer,
        };
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.free(self.data);
    }

    pub fn seek(self: *@This(), loc: SeekLocation) void {
        self.pos = loc.lba() * cdrom_sector_size_cue;
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

        // log.debug("disc sector read at {x} ({x}), mode={s} - {x}", .{
        //     self.pos,
        //     self.pos - (150 * cdrom_sector_size_cue),
        //     @tagName(sector_size),
        //     v[0..16],
        // });

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

const Opcode = opaque {
    pub const GET_STAT: u8 = 0x01;
    pub const SET_LOC: u8 = 0x02;
    pub const READ_N: u8 = 0x06;
    pub const PAUSE: u8 = 0x09;
    pub const INIT: u8 = 0x0a;
    pub const SET_MODE: u8 = 0x0e;
    pub const GET_ID: u8 = 0x1a;
    pub const SEEK_L: u8 = 0x15;
    pub const TEST: u8 = 0x19;
};

pub const CDROM = struct {
    pub const addr_start: u32 = 0x1f801800;
    pub const addr_end: u32 = 0x1f801803;

    allocator: std.mem.Allocator,
    addr: AddressReg,
    stat: StatusReg,
    mode: ModeReg,
    params: Fifo,
    results: Fifo,
    delay: u32 = 0,
    cmd: ?u8,
    cmd_state: CmdState = .recv_cmd,
    events: CdromEvents,

    disc: ?Disc,
    seekloc: ?SeekLocation,
    sect_buf: ?[]const u8,
    sect_pos: u32 = 0,

    hintmsk: packed struct(u8) { enint: u3 = 0, enbfempt: u1 = 0, enbfwrdy: u1 = 0, _pad: u3 = 0 },
    hintsts: packed struct(u8) { intsts: u3 = 0, bffempt: u1 = 0, bffwrdy: u1 = 0, _pad: u3 = 0 },

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch unreachable;

        self.* = .{
            .allocator = allocator,
            .params = .init(),
            .results = .init(),
            .hintmsk = std.mem.zeroes(@TypeOf(self.hintmsk)),
            .hintsts = std.mem.zeroes(@TypeOf(self.hintsts)),
            .cmd = null,
            .disc = null,
            .sect_buf = null,
            .mode = .{},
            .seekloc = .{},
            .events = .{},
            .stat = .{},
            .addr = .{},
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
        // log.debug("cdrom read: {x}", .{addr});

        const reg_id = bits.field(addr, 0, u2);
        const bank_index = self.addr.bank_index;

        return switch (reg_id) {
            0 => @as(u8, @bitCast(self.addr)),
            1 => self.consumeResultByte(),
            2 => self.consumeSectorData(T),
            3 => switch (bank_index) {
                0, 2 => @as(u8, @bitCast(self.hintmsk)),
                1, 3 => @as(u8, @bitCast(self.hintsts)) | 0xe0, // bits 5-7 always read as 1
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
            self.addr.data_request = false;
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
        addr.result_ready = !self.results.isEmpty();
        addr.param_ready = true; // always ready for params
        return @bitCast(addr);
    }

    fn consumeResultByte(self: *@This()) u8 {
        const v = self.results.readByte();
        if (self.results.isEmpty()) {
            self.addr.result_ready = false;
        }
        return v;
    }

    pub fn tick(self: *@This()) void {
        if (self.delay > 0) {
            self.delay -= 1;
            return;
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
        // log.debug("set interrupt: {x}", .{it});
        if (self.hintsts.intsts != 0) {
            std.debug.panic("overwriting unacknowledged interrupt: {x} -> {x}", .{ self.hintsts.intsts, it });
        }

        self.hintsts.intsts = it;
        if (@as(u8, @bitCast(self.hintsts)) & @as(u8, @bitCast(self.hintmsk)) != 0) {
            self.events.interrupt = true;
        }
    }

    fn clearInterrupt(self: *@This(), v: u8) void {
        const clr: packed struct(u8) {
            clrint: u3,
            clrbffempt: u1,
            clrbffwrdy: u1,
            smadpclr: u1,
            clrprm: u1,
            chpclr: u1,
        } = @bitCast(v);

        const before = self.hintsts.intsts;
        self.hintsts.intsts &= ~clr.clrint;
        self.hintsts.bffempt &= ~clr.clrbffempt;
        self.hintsts.bffwrdy &= ~clr.clrbffwrdy;
        if (clr.clrprm == 1) self.params.clear();

        if (before != self.hintsts.intsts) {
            // log.debug("interrupt ack: {x} -> {x}", .{ before, self.hintsts.intsts });
        }
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
                1 => self.hintmsk = @bitCast(val),
                2 => log.warn("ATV0: {x}", .{v}),
                3 => log.warn("ATV3: {x}", .{v}),
            },
            3 => switch (bank_index) {
                0 => {
                    const req: packed struct(u8) {
                        _pad: u5, // 0-4 (always 0)
                        smen: u1, // 5
                        bfwr: u1, // 6
                        bfrd: u1, // 7
                    } = @bitCast(val);
                    self.hintsts.bffempt = req.bfrd;
                    self.hintsts.bffwrdy = req.bfwr;
                },
                1 => self.clearInterrupt(val),
                2 => log.warn("ATV1: {x}", .{v}),
                3 => log.warn("ADPCTL: {x}", .{v}),
            },
        }
    }

    fn writePram(self: *@This(), v: u8) void {
        self.params.writeByte(v);
    }

    fn writeCommand(self: *@This(), v: u8) void {
        // log.debug("cdrom command: {x}", .{v});

        if (self.cmd != null and v != Opcode.PAUSE) {
            log.warn("another command in progress ({x}) while writing {x}", .{ self.cmd.?, v });
        }

        self.cmd = v;
        self.cmd_state = .recv_cmd;
        self.stepCommand(v);
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
};

const commands = opaque {
    const table = init: {
        const CmdPtr = *const fn (self: *CDROM) void;
        var cmds: [256]?CmdPtr = undefined;

        cmds[Opcode.GET_STAT] = getStat;
        cmds[Opcode.SET_LOC] = setLoc;
        cmds[Opcode.READ_N] = readN;
        cmds[Opcode.PAUSE] = pause;
        cmds[Opcode.INIT] = initCmd;
        cmds[Opcode.SET_MODE] = setMode;
        cmds[Opcode.GET_ID] = getId;
        cmds[Opcode.SEEK_L] = seekL;
        cmds[Opcode.TEST] = testCmd;

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
                self.results.writeByte(@bitCast(self.stat));
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp2;
                self.setInterrupt(3);
            },
            .resp2 => {
                if (self.disc) |_| {
                    self.results.writeSlice(&cdrom_getid_licensed);
                    self.setInterrupt(2);
                } else {
                    self.results.writeSlice(&cdrom_getid_nodisk);
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
                self.results.writeByte(@bitCast(self.stat));
                self.stat.read = false;
                self.setInterrupt(3);
                self.delay = cdrom_avg_delay_cycles;
                self.cmd_state = .resp2;
            },
            .resp2 => {
                self.results.writeByte(@bitCast(self.stat));
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
                self.results.writeByte(@bitCast(stat));
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
                const minute = bcdToDec(self.params.readByte());
                const second = bcdToDec(self.params.readByte());
                const sector = bcdToDec(self.params.readByte());
                log.debug("CDROM SETLOC to {d}:{d}:{d}", .{ minute, second, sector });
                self.seekloc = SeekLocation{ .minute = minute, .second = second, .sector = sector };
                self.results.writeByte(@bitCast(self.stat));
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
                const mode_byte = self.params.readByte();
                self.mode = @bitCast(mode_byte);
                log.debug("CDROM SETMODE: {x}", .{mode_byte});
                self.results.writeByte(@bitCast(self.stat));
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
                self.results.writeByte(@bitCast(self.stat));
                self.delay = cdrom_seekl_delay_cycles;
                self.setInterrupt(3);
                self.cmd_state = .resp2;
            },
            .resp2 => {
                self.disc.?.seek(self.seekloc.?);
                self.seekloc = null; // acknowledge seek
                self.results.writeByte(@bitCast(self.stat));
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
                self.stat.read = true;
                self.stat.motor_on = true;

                if (self.seekloc) |loc| { // unacknowledged seek
                    log.debug(
                        "CDROM READN seek to {d}:{d}:{d} (lba={d})",
                        .{ loc.minute, loc.second, loc.sector, loc.lba() },
                    );
                    self.disc.?.seek(loc);
                    self.seekloc = null;
                }
            },
            .resp1 => {
                log.debug("CDROM READN", .{});
                self.results.writeByte(@bitCast(self.stat));
                self.delay = switch (self.mode.speed) {
                    .normal => cdrom_read_normal_delay_cycles,
                    .double => cdrom_read_double_delay_cycles,
                };
                self.cmd_state = .read;
                self.setInterrupt(3);
            },
            .read => {
                self.results.writeByte(@bitCast(self.stat));
                const sect_buf = self.disc.?.readSector(self.mode.sector_size);
                self.setSectorData(sect_buf);
                self.delay = switch (self.mode.speed) {
                    .normal => cdrom_read_normal_delay_cycles,
                    .double => cdrom_read_double_delay_cycles,
                };
                self.addr.data_request = true;
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
                self.results.writeByte(@bitCast(self.stat));
                self.delay = cdrom_avg_delay_cycles;
                self.setInterrupt(3);
            },
            .resp2 => {
                self.results.writeByte(@bitCast(self.stat));
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
                const sub_cmd = self.params.readByte();
                switch (sub_cmd) {
                    0x20 => self.results.writeSlice(&cdrom_date_version),
                    else => log.warn("unhandled CDROM TEST sub-command: {x}", .{sub_cmd}),
                }
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
