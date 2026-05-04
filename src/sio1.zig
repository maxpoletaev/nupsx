const std = @import("std");

const RegId = opaque {
    const data = 0x0;
    const stat = 0x4;
    const mode = 0x8;
    const ctrl = 0xa;
    const misc = 0xc;
    const baud = 0xe;
};

pub const SIO1 = struct {
    pub const addr_start: u32 = 0x1f801050;
    pub const addr_end: u32 = 0x1f80105f;

    allocator: std.mem.Allocator,

    mode: u16 = 0,
    ctrl: u16 = 0,
    misc: u16 = 0,
    baud: u16 = 0,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const reg_id = addr & 0xf;
        const v: u32 = switch (reg_id) {
            RegId.data => 0xff, // disconnected RX line idles high
            RegId.stat => 0x05, // TX FIFO not full, TX idle, RX empty, no IRQ
            RegId.mode => self.mode,
            RegId.ctrl => self.ctrl,
            RegId.misc => self.misc,
            RegId.baud => self.baud,
            else => 0,
        };
        return @as(T, @truncate(v));
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const reg_id = addr & 0xf;
        const val: u16 = @truncate(v);
        switch (reg_id) {
            RegId.data => {}, // no serial peer attached
            RegId.mode => self.mode = val & 0x01ff,
            RegId.ctrl => self.writeCtrl(val),
            RegId.misc => self.misc = val,
            RegId.baud => self.baud = val,
            else => {},
        }
    }

    fn writeCtrl(self: *@This(), val: u16) void {
        if ((val & (1 << 6)) != 0) {
            self.mode = 0;
            self.ctrl = 0;
            self.misc = 0;
            self.baud = 0;
            return;
        }

        self.ctrl = val & 0x3fff;
        self.ctrl &= ~@as(u16, (1 << 4)); // acknowledge is write-only
    }
};
