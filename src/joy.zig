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
    unused0: u2 = 0, // 6-7
    polarity_inverse: bool = false, // 8
    unused1: u7 = 0, // 9-15
};

const StatusReg = packed struct(u32) {
    tx_fifo_not_full: bool = true, // 0
    rx_fifo_not_empty: bool = false, // 1
    tx_done: bool = true, // 2
    rx_parity_err: bool = false, // 3
    unused0: u3 = 0, // 4-6
    ack_line: enum(u1) { high = 0, low = 1 } = .high, // 7
    unused1: u1 = 0, // 8
    irq_pending: bool = false, // 9
    unused2: u1 = 0, // 10
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
    unused0: u1 = 0, // 3
    clear_irq: bool = false, // 4
    unused1: bool = false, // 5
    reset: bool = false, // 6
    unused2: u1 = 0, // 7
    rx_irq_mode: RxIrqMode = .irq_1byte, // 8-9
    tx_irq_enable: bool = false, // 10
    rx_irq_enable: bool = false, // 11
    ack_irq_enable: bool = false, // 12
    joy_select: u1 = 0, // 13
    unused3: u2 = 0, // 14-15
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
    select: bool = false,
    l3: bool = false,
    r3: bool = false,
    start: bool = false,
    up: bool = false,
    right: bool = false,
    down: bool = false,
    left: bool = false,
    l2: bool = false,
    r2: bool = false,
    l1: bool = false,
    r1: bool = false,
    triangle: bool = false,
    circle: bool = false,
    cross: bool = false,
    square: bool = false,
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

    pub fn tick(self: *@This(), cpu_cyc: u32) void {
        if (self.irq_delay > 0) {
            self.irq_delay -|= cpu_cyc;
            if (self.irq_delay == 0) {
                self.stat.irq_pending = true;
                self.bus.setInterrupt(Interrupt.joy_mc_byte);
            }
        }
    }

    inline fn triggerIrqWithDelay(self: *@This()) void {
        self.stat.ack_line = .low; // /ACK is pulled low immediately
        self.irq_delay = joy_irq_delay_cycles;
    }

    pub fn read(self: *@This(), comptime T: type, addr: u32) T {
        const offset = addr - addr_start;
        const reg_id = bits.field(offset, 0, u4);

        const v: u32 = switch (reg_id) {
            RegId.joy_data => self.readData(T),
            RegId.joy_stat => self.readStat(),
            RegId.joy_mode => @as(u16, @bitCast(self.mode)),
            RegId.joy_ctrl => @as(u16, @bitCast(self.ctrl)),
            RegId.joy_baud => self.baudrate_reload,
            else => std.debug.panic("unhandled register read: {x}", .{reg_id}),
        };

        return @truncate(v);
    }

    pub fn write(self: *@This(), comptime T: type, addr: u32, v: T) void {
        const offset = addr - addr_start;
        const reg_id = bits.field(offset, 0, u4);

        switch (reg_id) {
            RegId.joy_data => self.writeData(T, v),
            RegId.joy_mode => self.mode = @bitCast(@as(u16, @intCast(v))),
            RegId.joy_baud => self.baudrate_reload = @intCast(v),
            RegId.joy_ctrl => self.writeControl(@as(u16, @intCast(v))),
            else => std.debug.panic("unhandled register write: {x}", .{reg_id}),
        }
    }

    inline fn readStat(self: *@This()) u32 {
        var stat = self.stat;
        stat.tx_fifo_not_full = true; // always ready to accept data
        stat.rx_fifo_not_empty = !self.rx_data.isEmpty();
        self.stat.ack_line = .high;
        return @bitCast(stat);
    }

    inline fn readData(self: *@This(), comptime T: type) T {
        switch (T) {
            u8 => {
                return self.rx_data.pop() orelse 0xff;
            },
            u16 => {
                const low = self.rx_data.pop() orelse 0xff;
                const high = self.rx_data.peek() orelse 0xff; // NOTE: second byte is not consumed
                const v = (@as(u16, high) << 8) | @as(u16, low);
                return v;
            },
            else => std.debug.panic("unsupported readData type: {s}", .{@typeName(T)}),
        }
    }

    inline fn writeData(self: *@This(), comptime T: type, v: T) void {
        switch (T) {
            u8 => {
                self.advanceState(v);
            },
            u16 => {
                self.advanceState(@truncate(v >> 0));
                self.advanceState(@truncate(v >> 8));
            },
            u32 => {
                self.advanceState(@truncate(v >> 0));
                self.advanceState(@truncate(v >> 8));
                self.advanceState(@truncate(v >> 16));
                self.advanceState(@truncate(v >> 24));
            },
            else => @compileError("unhandled type " ++ @typeName(T)),
        }
    }

    inline fn writeControl(self: *@This(), v: u16) void {
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
            self.state = .idle;
        }
        self.ctrl = ctrl;
        self.ctrl.clear_irq = false; // read-only (always read as 0)
    }

    fn advanceState(self: *@This(), tx_byte: u8) void {
        if (!self.ctrl.tx_enable) @panic("tx_enable=false unhandled");
        if (!self.ctrl.joy_select_enable) @panic("joy_select_enable=false unhandled");

        switch (self.state) {
            .idle => {
                if (tx_byte != 0x01) {
                    self.rx_data.push(0xff);
                    self.state = .idle;
                    return;
                }
                self.rx_data.push(0xff);
                self.state = .id_lo;
            },
            .id_lo => {
                if (tx_byte != 0x42) {
                    self.rx_data.push(0xff);
                    self.state = .idle;
                    return;
                }
                self.rx_data.push(joy_id_digital[1]);
                self.state = .id_hi;
            },
            .id_hi => {
                self.rx_data.push(joy_id_digital[0]);
                self.state = .swlo;
            },
            .swlo => {
                const btns = ~@as(u16, @bitCast(self.buttons[self.ctrl.joy_select])); // 0=pressed
                self.rx_data.push(@truncate(btns >> 0));
                self.state = .swhi;
            },
            .swhi => {
                const btns = ~@as(u16, @bitCast(self.buttons[self.ctrl.joy_select])); // 0=pressed
                self.rx_data.push(@truncate(btns >> 8));
                self.state = .idle;
            },
        }

        // NOTE: interrupt is triggered when there is MORE data to send.
        // Silence means end of transmission.

        if (self.ctrl.ack_irq_enable and self.state != .idle) {
            self.triggerIrqWithDelay();
        }
    }

    pub inline fn setButtonState(self: *@This(), button: Button, pressed: bool) void {
        @field(self.buttons[0], @tagName(button)) = pressed; // 1=not pressed
    }
};
