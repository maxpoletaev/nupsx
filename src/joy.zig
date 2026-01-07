const std = @import("std");
const bits = @import("bits.zig");
const fifo = @import("fifo.zig");

const log = std.log.scoped(.joy);

const joy_id_digital_lo: u8 = 0x41;
const joy_id_digital_hi: u8 = 0x5A;
const joy_irq_delay_cycles: u32 = 1088;

const ModeReg = packed struct(u16) {
    baudrate_factor: enum(u2) { mul1 = 0, mul16 = 1, mul64 = 2 } = .mul1, // 0-1
    char_len: enum(u2) { bits5 = 0, bits6 = 1, bits7 = 2, bits8 = 3 } = .bits5, // 2-3
    parity_enable: bool = false, // 4
    parity_type: enum(u1) { even = 0, odd = 1 } = .even, // 5
    _pad0: u2 = 0, // 6-7
    polarity_inverse: bool = false, // 8
    _pad1: u7 = 0, // 9-15
};

const StatusReg = packed struct(u32) {
    tx_ready: bool = true, // 0
    rx_fifo_not_empty: bool = false, // 1
    tx_done: bool = true, // 2
    rx_parity_err: bool = false, // 3
    _pad0: u3 = 0, // 4-6
    ack_input: bool = false, // 7
    _pad1: u1 = 0, // 8
    irq_request: bool = false, // 9
    _pad2: u1 = 0, // 10
    baudrate_timer: u21 = 0x88, // 11-31
};

const CotrolReg = packed struct(u16) {
    tx_enable: bool = false, // 0
    joy_select_enable: bool = false, // 1
    rx_enable: bool = false, // 2
    _pad0: u1 = 0, // 3
    ack: bool = false, // 4
    _unknown2: bool = false, // 5
    reset: bool = false, // 6
    _pad1: u1 = 0, // 7
    rx_irq_mode: enum(u2) { irq_1byte = 0, irq_2bytes = 1, irq_4bytes = 2, irq_8bytes = 3 } = .irq_1byte, // 8-9
    tx_irq_enable: bool = false, // 10
    rx_irq_enable: bool = false, // 11
    ack_irq_enable: bool = false, // 12
    joy_select: u1 = 0, // 13
    _pad2: u2 = 0, // 14-15
};

const RegId = opaque {
    const JOY_DATA = 0x0;
    const JOY_STAT = 0x4;
    const JOY_MODE = 0x8;
    const JOY_CTRL = 0xA;
    const JOY_BAUD = 0xE;
};

pub const ButtonId = enum { select, l3, r3, start, up, right, down, left, l2, r2, l1, r1, triangle, circle, cross, square };

pub const ButtonState = packed struct(u16) {
    select: bool = true,
    l3: bool = true,
    r3: bool = true,
    start: bool = true,
    up: bool = true,
    right: bool = true,
    down: bool = true,
    left: bool = true,
    l2: bool = true,
    r2: bool = true,
    l1: bool = true,
    r1: bool = true,
    triangle: bool = true,
    circle: bool = true,
    cross: bool = true,
    square: bool = true,
};

pub const Joypad = struct {
    pub const addr_start = 0x1f801040;
    pub const addr_end = 0x1f80104e;

    allocator: std.mem.Allocator,
    baudrate_reload: u16 = 0,
    mode: ModeReg,
    stat: StatusReg,
    ctrl: CotrolReg,

    state: enum { idle, id_lo, id_hi, btn_lo, btn_hi } = .idle,
    buttons: [2]ButtonState = .{ .{}, .{} },
    tx_data: fifo.StaticFifo(u8, 16),
    rx_data: fifo.StaticFifo(u8, 16),

    irq_pending: bool = false,
    irq_delay: u32 = 0,

    pub fn init(allocator: std.mem.Allocator) *@This() {
        const self = allocator.create(@This()) catch unreachable;
        self.* = .{
            .allocator = allocator,
            .mode = .{},
            .stat = .{},
            .ctrl = .{},
            .tx_data = .init(),
            .rx_data = .init(),
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.destroy(self);
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const offset = addr - addr_start;
        const reg_id = bits.field(offset, 0, u4);

        const v: u32 = switch (reg_id) {
            RegId.JOY_DATA => self.readData(T),
            RegId.JOY_STAT => @as(u32, @bitCast(self.stat)),
            RegId.JOY_MODE => @as(u16, @bitCast(self.mode)),
            RegId.JOY_CTRL => @as(u16, @bitCast(self.ctrl)),
            RegId.JOY_BAUD => self.baudrate_reload,
            else => std.debug.panic("unhandled register read: {x}", .{reg_id}),
        };
        return @truncate(v);
    }

    fn writeControlReg(self: *@This(), v: u16) void {
        const ctrl = @as(CotrolReg, @bitCast(v));
        if (ctrl.ack) {
            self.stat.rx_parity_err = false;
            self.stat.irq_request = false;
        }
        if (ctrl.reset) {
            self.mode = .{};
            self.stat = .{};
            self.irq_pending = false;
            self.irq_delay = 0;
            self.state = .idle;
        }
        if (!ctrl.joy_select_enable) {
            self.stat.ack_input = false;
            self.state = .idle;
        }
        self.ctrl = ctrl;
        self.ctrl.ack = false; // read-only (always read as 0)
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        // log.debug("JOY write addr={x} value={x}", .{ addr, @as(u32, v) });

        const offset = addr - addr_start;
        const reg_id = bits.field(offset, 0, u4);

        switch (reg_id) {
            RegId.JOY_DATA => {
                self.writeData(T, v);
                self.stepSerialState();
            },
            RegId.JOY_MODE => self.mode = @bitCast(@as(u16, @intCast(v))),
            RegId.JOY_BAUD => self.baudrate_reload = @intCast(v),
            RegId.JOY_CTRL => self.writeControlReg(@as(u16, @intCast(v))),
            else => std.debug.panic("unhandled register write: {x}", .{reg_id}),
        }
    }

    pub fn consumeIrq(self: *@This()) bool {
        const was_pending = self.irq_pending;
        self.irq_pending = false;
        return was_pending;
    }

    pub fn tick(self: *@This(), cpu_cyc: u32) void {
        if (self.irq_delay > 0) {
            self.irq_delay -|= cpu_cyc;

            if (self.irq_delay == 0) {
                self.irq_pending = true;
                self.stat.irq_request = true;
            }
        }
    }

    pub fn stepSerialState(self: *@This()) void {
        if (!self.ctrl.tx_enable) return;

        const tx_byte = self.tx_data.pop() orelse return;

        const btns = self.buttons[self.ctrl.joy_select];

        const rx_byte: u8 = switch (self.state) {
            .idle => 0xff,
            .id_lo => joy_id_digital_lo,
            .id_hi => joy_id_digital_hi,
            .btn_lo => @truncate(@as(u16, @bitCast(btns)) >> 0),
            .btn_hi => @truncate(@as(u16, @bitCast(btns)) >> 8),
        };

        self.state = switch (self.state) {
            .idle => if (tx_byte == 0x01) .id_lo else .idle,
            .id_lo => if (tx_byte == 0x42) .id_hi else .idle,
            .id_hi => .btn_lo,
            .btn_lo => .btn_hi,
            .btn_hi => .idle,
        };

        const ack = self.state != .idle;
        self.stat.ack_input = ack;

        self.rx_data.push(rx_byte);
        self.stat.rx_fifo_not_empty = true;

        if (self.ctrl.ack_irq_enable and ack) {
            self.irq_delay = joy_irq_delay_cycles;
        }
    }

    fn writeData(self: *@This(), comptime T: type, v: T) void {
        switch (T) {
            u8 => {
                self.tx_data.push(@as(u8, v));
            },
            u16 => {
                const low = bits.field(@as(u16, v), 0, u8);
                const high = bits.field(@as(u16, v), 8, u8);
                self.tx_data.push(low);
                self.tx_data.push(high);
            },
            else => std.debug.panic("unsupported writeData type: {s}", .{@typeName(T)}),
        }
    }

    fn readData(self: *@This(), comptime T: type) T {
        switch (T) {
            u8 => {
                const v = self.rx_data.pop() orelse 0xff;
                self.stat.rx_fifo_not_empty = !self.rx_data.isEmpty();
                return @as(T, v);
            },
            u16 => {
                const low = self.rx_data.pop() orelse 0xff;
                const high = self.rx_data.pop() orelse 0xff;
                const v = @as(u16, low) | (@as(u16, high) << 8);
                return @as(T, v);
            },
            else => std.debug.panic("unsupported readData type: {s}", .{@typeName(T)}),
        }
    }

    pub inline fn setButtonState(self: *@This(), button: ButtonId, pressed: bool) void {
        @field(self.buttons[0], @tagName(button)) = !pressed; // 1=not pressed
    }
};
