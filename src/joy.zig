const std = @import("std");
const bits = @import("bits.zig");
const fifo = @import("fifo.zig");
const mem = @import("mem.zig");

const Interrupt = mem.Interrupt;

const log = std.log.scoped(.joy);

const joy_id_digital = [2]u8{ 0x5a, 0x41 };
const joy_irq_delay_cycles: u32 = 544;

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
    ack_signal_level: enum(u1) { high = 0, low = 1 } = .low, // 7
    _pad1: u1 = 0, // 8
    irq_pending: bool = false, // 9
    _pad2: u1 = 0, // 10
    baudrate_timer: u21 = 0x88, // 11-31
};

const RxIrqMode = enum(u2) {
    irq_1byte = 0,
    irq_2bytes = 1,
    irq_4bytes = 2,
    irq_8bytes = 3,
};

const CotrolReg = packed struct(u16) {
    tx_enable: bool = false, // 0
    joy_select_enable: bool = false, // 1
    rx_enable: bool = false, // 2
    _pad0: u1 = 0, // 3
    clear_irq: bool = false, // 4
    _unknown2: bool = false, // 5
    reset: bool = false, // 6
    _pad1: u1 = 0, // 7
    rx_irq_mode: RxIrqMode = .irq_1byte, // 8-9
    tx_irq_enable: bool = false, // 10
    rx_irq_enable: bool = false, // 11
    ack_irq_enable: bool = false, // 12
    joy_select: u1 = 0, // 13
    _pad2: u2 = 0, // 14-15
};

pub const Button = enum {
    select,
    l3,
    r3,
    start,
    up,
    right,
    down,
    left,
    l2,
    r2,
    l1,
    r1,
    triangle,
    circle,
    cross,
    square,
};

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

const RegId = opaque {
    const joy_data = 0x0;
    const joy_stat = 0x4;
    const joy_mode = 0x8;
    const joy_ctrl = 0xa;
    const joy_baud = 0xe;
};

pub const Joypad = struct {
    pub const addr_start = 0x1f801040;
    pub const addr_end = 0x1f80104e;

    allocator: std.mem.Allocator,
    baudrate_reload: u16 = 0,
    mode: ModeReg,
    stat: StatusReg,
    ctrl: CotrolReg,

    state: enum { idle, id_lo, id_hi, swlo, swhi } = .idle,
    buttons: [2]ButtonState = .{ .{}, .{} },
    tx_data: fifo.StaticFifo(u8, 16),
    rx_data: fifo.StaticFifo(u8, 16),

    bus: *mem.Bus,
    irq_delay: u32 = 0,

    pub fn init(allocator: std.mem.Allocator, bus: *mem.Bus) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .mode = .{},
            .stat = .{},
            .ctrl = .{},
            .tx_data = .empty,
            .rx_data = .empty,
            .bus = bus,
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
            RegId.joy_data => self.readData(T),
            RegId.joy_stat => @as(u32, @bitCast(self.stat)),
            RegId.joy_mode => @as(u16, @bitCast(self.mode)),
            RegId.joy_ctrl => @as(u16, @bitCast(self.ctrl)),
            RegId.joy_baud => self.baudrate_reload,
            else => std.debug.panic("unhandled register read: {x}", .{reg_id}),
        };

        return @truncate(v);
    }

    fn writeControlReg(self: *@This(), v: u16) void {
        const ctrl = @as(CotrolReg, @bitCast(v));
        if (ctrl.clear_irq) {
            self.stat.rx_parity_err = false;
            self.stat.irq_pending = false;
        }
        if (ctrl.reset) {
            self.mode = .{};
            self.stat = .{};
            self.ctrl = .{};
            self.irq_delay = 0;
            self.state = .idle;
            self.tx_data.clear();
            self.rx_data.clear();
        }
        if (!ctrl.joy_select_enable) {
            self.stat.ack_signal_level = .low;
            self.state = .idle;
        }
        self.ctrl = ctrl;
        self.ctrl.clear_irq = false; // read-only (always read as 0)
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const offset = addr - addr_start;
        const reg_id = bits.field(offset, 0, u4);

        switch (reg_id) {
            RegId.joy_data => {
                self.writeData(T, v);
                self.advanceState();
            },
            RegId.joy_mode => self.mode = @bitCast(@as(u16, @intCast(v))),
            RegId.joy_baud => self.baudrate_reload = @intCast(v),
            RegId.joy_ctrl => self.writeControlReg(@as(u16, @intCast(v))),
            else => std.debug.panic("unhandled register write: {x}", .{reg_id}),
        }
    }

    pub fn tick(self: *@This(), cpu_cyc: u32) void {
        if (self.irq_delay > 0) {
            self.irq_delay -|= cpu_cyc;

            if (self.irq_delay == 0) {
                self.stat.irq_pending = true;
                self.stat.ack_signal_level = .low;
                self.bus.setInterrupt(Interrupt.joy_mc_byte);
            }
        }
    }

    pub fn advanceState(self: *@This()) void {
        // if (!self.ctrl.tx_enable) return;
        // if (!self.ctrl.joy_select_enable) return;

        const btns: u16 = @bitCast(self.buttons[self.ctrl.joy_select]);

        const tx_byte = self.tx_data.pop() orelse return;

        const rx_byte: u8, self.state = switch (self.state) {
            .idle => if (tx_byte == 0x01) .{ 0xff, .id_lo } else .{ 0xff, .idle },
            .id_lo => if (tx_byte == 0x42) .{ joy_id_digital[1], .id_hi } else .{ 0xff, .idle },
            .id_hi => .{ joy_id_digital[0], .swlo },
            .swlo => .{ @truncate(btns >> 0), .swhi },
            .swhi => .{ @truncate(btns >> 8), .idle },
        };

        log.debug("IN:{x} OUT:{x}", .{ tx_byte, rx_byte });

        self.rx_data.push(rx_byte);

        self.stat.rx_fifo_not_empty = true;

        // Weird stuff: the interrupts are triggered when there is
        // MORE data to send. Silence means end of transmission.

        const last_byte = self.state == .idle;

        if (self.ctrl.ack_irq_enable and !last_byte) {
            self.irq_delay = joy_irq_delay_cycles;
            self.stat.ack_signal_level = .high;
        }
    }

    fn writeData(self: *@This(), comptime T: type, v: T) void {
        switch (T) {
            u8 => {
                self.tx_data.push(v);
            },
            u16 => {
                self.tx_data.push(bits.field(v, 0, u8));
                self.tx_data.push(bits.field(v, 8, u8));
            },
            else => std.debug.panic("unsupported writeData type: {s}", .{@typeName(T)}),
        }
    }

    fn readData(self: *@This(), comptime T: type) T {
        switch (T) {
            u8 => {
                const v = self.rx_data.pop() orelse 0xff;
                self.stat.rx_fifo_not_empty = !self.rx_data.isEmpty();
                return v;
            },
            u16 => {
                const low = self.rx_data.pop() orelse 0xff;
                const high = self.rx_data.pop() orelse 0xff;
                const v = @as(u16, low) | (@as(u16, high) << 8);
                self.stat.rx_fifo_not_empty = !self.rx_data.isEmpty();
                return v;
            },
            else => std.debug.panic("unsupported readData type: {s}", .{@typeName(T)}),
        }
    }

    pub inline fn setButtonState(self: *@This(), button: Button, pressed: bool) void {
        @field(self.buttons[0], @tagName(button)) = !pressed; // 1=not pressed
    }
};
