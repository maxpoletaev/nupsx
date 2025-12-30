const std = @import("std");
const bits = @import("bits.zig");

const log = std.log.scoped(.cdrom);

const cdrom_date_version = [_]u8{ 0x94, 0x09, 0x19, 0xC0 };

const StatusReg = packed struct(u8) {
    bank_index: u2 = 0,
    adpbusy: bool = false,
    prmempt: bool = true,
    prmwrdy: bool = true,
    rslrrdy: bool = false,
    drqsts: bool = false,
    busysts: bool = false,
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

pub const CDROM = struct {
    pub const addr_start: u32 = 0x1f801800;
    pub const addr_end: u32 = 0x1f801803;

    allocator: std.mem.Allocator,
    status: StatusReg,
    params: Fifo,
    results: Fifo,

    hintmsk: packed struct(u8) { enint: u3 = 0, enbfempt: u1 = 0, enbfwrdy: u1 = 0, _pad: u3 = 0 },
    hintsts: packed struct(u8) { intsts: u3 = 0, bffempt: u1 = 0, bffwrdy: u1 = 0, _pad: u3 = 0 },

    interrupt: bool = false,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch unreachable;
        self.* = .{
            .allocator = allocator,
            .params = .init(),
            .results = .init(),
            .hintmsk = std.mem.zeroes(@TypeOf(self.hintmsk)),
            .hintsts = std.mem.zeroes(@TypeOf(self.hintsts)),
            .status = .{},
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        log.debug("cdrom read: {x}", .{addr});

        const reg_id = bits.field(addr, 0, u2);
        const bank_index = self.status.bank_index;

        const v: u8 = switch (reg_id) {
            0 => @bitCast(self.status),
            1 => self.consumeResultByte(),
            3 => switch (bank_index) {
                0, 2 => @bitCast(self.hintmsk),
                1, 3 => @as(u8, @bitCast(self.hintsts)) | 0xe0, // bits 5-7 always read as 1
            },
            else => blk: {
                log.warn("unhandled read ({s}) at {x}", .{ @typeName(T), addr });
                break :blk 0;
            },
        };

        return v;
    }

    fn readStatus(self: *@This()) u8 {
        const status = self.status;
        status.prmempt = self.params.isEmpty();
        status.rslrrdy = !self.results.isEmpty();
        status.prmwrdy = true; // always ready for params
        return @bitCast(status);
    }

    fn consumeResultByte(self: *@This()) u8 {
        const v = self.results.readByte();
        if (self.results.isEmpty()) {
            self.status.rslrrdy = false;
        }
        return v;
    }

    pub fn tick(self: *@This()) void {
        const hintsts = @as(u8, @bitCast(self.hintsts));
        const hintmsk = @as(u8, @bitCast(self.hintmsk));
        if ((hintsts & hintmsk) != 0) {
            self.interrupt = true;
        }
    }

    pub fn consumeInterrupt(self: *@This()) bool {
        const v = self.interrupt;
        self.interrupt = false;
        return v;
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        log.debug("cdrom write: {x} = {x}", .{ addr, @as(u32, v) });

        const reg_id = bits.field(addr, 0, u2);
        const bank_index = self.status.bank_index;

        const val: u8 = switch (T) {
            u8 => v,
            u16, u32 => @truncate(v),
            else => @compileError("CDROM.Write: Unsupported type"),
        };

        switch (reg_id) {
            0 => self.status.bank_index = bits.field(val, 0, u2),
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
                0 => log.warn("HCHPCTL: {x}", .{v}),
                1 => self.hintsts.intsts &= ~bits.field(val, 0, u3), // 1 to clear
                2 => log.warn("ATV1: {x}", .{v}),
                3 => log.warn("ADPCTL: {x}", .{v}),
            },
        }
    }

    fn writePram(self: *@This(), v: u8) void {
        self.params.writeByte(v);
        self.status.prmempt = false;
    }

    fn writeCommand(self: *@This(), v: u8) void {
        switch (v) {
            0x19 => self.cmdTest(v),
            else => log.warn("unhandled CDROM command: {x}", .{v}),
        }
    }

    fn cmdTest(self: *@This(), _: u8) void {
        const sub_cmd = self.params.readByte();
        switch (sub_cmd) {
            0x20 => self.results.writeSlice(&cdrom_date_version),
            else => log.warn("unhandled CDROM TEST sub-command: {x}", .{sub_cmd}),
        }
        self.hintsts.intsts = 3;
        self.status.rslrrdy = true;
        self.status.prmempt = true;
        self.params.clear();
    }
};
