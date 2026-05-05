const std = @import("std");
const bits = @import("bits.zig");
const fifo = @import("fifo.zig");
const mem = @import("mem.zig");
const memcard_mod = @import("memcard.zig");

const Interrupt = mem.Interrupt;
const MemoryCard = memcard_mod.MemoryCard;

const log = std.log.scoped(.joy);

const joy_id_digital = [2]u8{ 0x5a, 0x41 };
const joy_irq_delay_cycles: u32 = 544;
const joy_mc_read_delay_cycles: u32 = 31000;

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

const ControllerState = struct {
    port: usize,
    phase: enum { await_command, id_hi, buttons_lo, buttons_hi },
};

const MemcardCommand = enum {
    read,
    write,
    get_id,
};

const MemcardPhase = enum {
    await_command,
    id1,
    id2,
    read_addr_hi,
    read_addr_lo,
    read_ack1,
    read_ack2,
    read_addr_echo_hi,
    read_addr_echo_lo,
    read_data,
    read_checksum,
    read_end,
    write_addr_hi,
    write_addr_lo,
    write_data,
    write_checksum,
    write_ack1,
    write_ack2,
    write_end,
    get_id_ack1,
    get_id_ack2,
    get_id_tail,
};

const MemcardState = struct {
    port: usize,
    command: MemcardCommand,
    phase: MemcardPhase,
    addr: u16 = 0,
    data_idx: usize = 0,
    checksum: u8 = 0,
    data: [memcard_mod.sector_size]u8 = undefined,
    tail_idx: u8 = 0,
    invalid_sector: bool = false,
};

const TransactionState = union(enum) {
    idle,
    memcard: MemcardState,
    controller: ControllerState,
};

const Exchange = struct {
    rx: u8,
    ack: bool,
    irq_delay: u32 = joy_irq_delay_cycles,
};

pub const SIO0 = struct {
    pub const addr_start = 0x1f801040;
    pub const addr_end = 0x1f80104e;

    allocator: std.mem.Allocator,
    baudrate_reload: u16 = 0,
    mode: ModeReg,
    stat: StatusReg,
    ctrl: CotrolReg,

    buttons: ButtonState = .{},
    rx_data: fifo.StaticFifo(u8, 256),
    memcard: ?*MemoryCard = null,

    bus: *mem.Bus,
    irq_delay: u32 = 0,
    state: TransactionState = .idle,

    pub fn init(allocator: std.mem.Allocator, bus: *mem.Bus, memcard: ?*MemoryCard) *@This() {
        const self = allocator.create(@This()) catch @panic("OOM");
        self.* = .{
            .allocator = allocator,
            .mode = .{},
            .stat = .{},
            .ctrl = .{},
            .rx_data = .empty,
            .memcard = memcard,
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
                log.debug("irq", .{});
                self.stat.irq_pending = true;
                self.bus.setInterrupt(Interrupt.joy_mc_byte);
            }
        }
    }

    inline fn triggerIrqWithDelay(self: *@This(), delay_cycles: u32) void {
        self.stat.ack_line = .low; // /ACK is pulled low immediately
        self.irq_delay = delay_cycles;
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
            self.rx_data.clear();
        }
        if (!ctrl.joy_select_enable) {
            self.state = .idle;
        }
        self.ctrl = ctrl;
        self.ctrl.clear_irq = false; // read-only (always read as 0)
    }

    fn advanceState(self: *@This(), tx_byte: u8) void {
        if (!self.ctrl.tx_enable) return;
        if (!self.ctrl.joy_select_enable) return;

        const exchange = switch (self.state) {
            .idle => self.handleIdle(tx_byte),
            .memcard => |card| self.handleMemcard(card, tx_byte),
            .controller => |controller| self.handleController(controller, tx_byte),
        };

        self.rx_data.push(exchange.rx);

        // Interrupt is triggered only when there is more data to send.
        // Silence means end of transmission.
        if (self.ctrl.ack_irq_enable and exchange.ack) {
            self.triggerIrqWithDelay(exchange.irq_delay);
        }
    }

    fn handleIdle(self: *@This(), tx_byte: u8) Exchange {
        const port = self.ctrl.joy_select;
        if (port != 0) {
            self.state = .idle;
            return .{ .rx = 0xff, .ack = false };
        }

        return switch (tx_byte) {
            0x01 => blk: {
                self.state = .{ .controller = .{ .port = port, .phase = .await_command } };
                break :blk .{ .rx = 0xff, .ack = true };
            },
            0x81 => blk: {
                if (self.memcard == null) {
                    self.state = .idle;
                    break :blk .{ .rx = 0xff, .ack = false };
                }
                self.state = .{ .memcard = .{ .port = port, .command = .read, .phase = .await_command } };
                break :blk .{ .rx = 0xff, .ack = true };
            },
            else => .{ .rx = 0xff, .ack = false },
        };
    }

    fn handleController(self: *@This(), controller: ControllerState, tx_byte: u8) Exchange {
        switch (controller.phase) {
            .await_command => {
                if (tx_byte != 0x42) {
                    self.state = .idle;
                    return .{ .rx = 0xff, .ack = false };
                }
                self.state = .{ .controller = .{ .port = controller.port, .phase = .id_hi } };
                return .{ .rx = joy_id_digital[1], .ack = true };
            },
            .id_hi => {
                self.state = .{ .controller = .{ .port = controller.port, .phase = .buttons_lo } };
                return .{ .rx = joy_id_digital[0], .ack = true };
            },
            .buttons_lo => {
                const btns = ~@as(u16, @bitCast(self.buttons)); // 0=pressed
                self.state = .{ .controller = .{ .port = controller.port, .phase = .buttons_hi } };
                return .{ .rx = @truncate(btns >> 0), .ack = true };
            },
            .buttons_hi => {
                const btns = ~@as(u16, @bitCast(self.buttons)); // 0=pressed
                self.state = .idle;
                return .{ .rx = @truncate(btns >> 8), .ack = false };
            },
        }
    }

    fn handleMemcard(self: *@This(), card_state: MemcardState, tx_byte: u8) Exchange {
        var state = card_state;
        const card = self.memcard.?;

        switch (state.phase) {
            .await_command => {
                state.command = switch (tx_byte) {
                    'R' => .read,
                    'W' => .write,
                    'S' => .get_id,
                    else => {
                        self.state = .idle;
                        return .{ .rx = card.flag, .ack = false };
                    },
                };
                state.phase = .id1;
                self.state = .{ .memcard = state };
                return .{ .rx = card.flag, .ack = true };
            },
            .id1 => {
                state.phase = .id2;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x5a, .ack = true };
            },
            .id2 => {
                state.phase = switch (state.command) {
                    .read => .read_addr_hi,
                    .write => .write_addr_hi,
                    .get_id => .get_id_ack1,
                };
                self.state = .{ .memcard = state };
                return .{ .rx = 0x5d, .ack = true };
            },
            .read_addr_hi => {
                state.addr = @as(u16, tx_byte) << 8;
                state.phase = .read_addr_lo;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x00, .ack = true };
            },
            .read_addr_lo => {
                state.addr |= tx_byte;
                state.invalid_sector = state.addr >= memcard_mod.sector_count;
                state.checksum = MemoryCard.calcSectorChecksum(state.addr);
                state.phase = .read_ack1;
                self.state = .{ .memcard = state };
                return .{ .rx = tx_byte, .ack = true, .irq_delay = joy_mc_read_delay_cycles };
            },
            .read_ack1 => {
                state.phase = .read_ack2;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x5c, .ack = true };
            },
            .read_ack2 => {
                state.phase = .read_addr_echo_hi;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x5d, .ack = true };
            },
            .read_addr_echo_hi => {
                state.phase = .read_addr_echo_lo;
                self.state = .{ .memcard = state };
                return .{ .rx = if (state.invalid_sector) 0xff else @truncate(state.addr >> 8), .ack = true };
            },
            .read_addr_echo_lo => {
                if (state.invalid_sector) {
                    self.state = .idle;
                    return .{ .rx = 0xff, .ack = false };
                }
                state.phase = .read_data;
                state.data_idx = 0;
                self.state = .{ .memcard = state };
                return .{ .rx = @truncate(state.addr), .ack = true };
            },
            .read_data => {
                const data = card.readSector(state.addr);
                const value = data[state.data_idx];
                state.checksum ^= value;
                state.data_idx += 1;
                state.phase = if (state.data_idx == memcard_mod.sector_size) .read_checksum else .read_data;
                self.state = .{ .memcard = state };
                return .{ .rx = value, .ack = true };
            },
            .read_checksum => {
                state.phase = .read_end;
                self.state = .{ .memcard = state };
                return .{ .rx = state.checksum, .ack = true };
            },
            .read_end => {
                self.state = .idle;
                return .{ .rx = 0x47, .ack = false };
            },
            .write_addr_hi => {
                state.addr = @as(u16, tx_byte) << 8;
                state.phase = .write_addr_lo;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x00, .ack = true };
            },
            .write_addr_lo => {
                state.addr |= tx_byte;
                state.invalid_sector = state.addr >= memcard_mod.sector_count;
                state.data_idx = 0;
                state.checksum = MemoryCard.calcSectorChecksum(state.addr);
                state.phase = .write_data;
                self.state = .{ .memcard = state };
                return .{ .rx = tx_byte, .ack = true };
            },
            .write_data => {
                state.data[state.data_idx] = tx_byte;
                state.checksum ^= tx_byte;
                state.data_idx += 1;
                state.phase = if (state.data_idx == memcard_mod.sector_size) .write_checksum else .write_data;
                self.state = .{ .memcard = state };
                return .{ .rx = tx_byte, .ack = true };
            },
            .write_checksum => {
                const checksum_ok = state.checksum == tx_byte;
                if (!state.invalid_sector and checksum_ok) {
                    card.writeSector(state.addr, &state.data);
                }
                state.phase = .write_ack1;
                // 0x47='G' success, 0x4e='N' checksum error, 0xff invalid sector.
                state.checksum = if (state.invalid_sector) 0xff else if (checksum_ok) 0x47 else 0x4e;
                self.state = .{ .memcard = state };
                return .{ .rx = tx_byte, .ack = true };
            },
            .write_ack1 => {
                state.phase = .write_ack2;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x5c, .ack = true };
            },
            .write_ack2 => {
                state.phase = .write_end;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x5d, .ack = true };
            },
            .write_end => {
                if (state.checksum == 0x47) {
                    card.flushWrite();
                }
                self.state = .idle;
                return .{ .rx = @truncate(state.checksum), .ack = false };
            },
            .get_id_ack1 => {
                state.phase = .get_id_ack2;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x5c, .ack = true };
            },
            .get_id_ack2 => {
                state.phase = .get_id_tail;
                state.tail_idx = 0;
                self.state = .{ .memcard = state };
                return .{ .rx = 0x5d, .ack = true };
            },
            .get_id_tail => {
                const payload = [_]u8{ 0x04, 0x00, 0x00, 0x80 };
                const value = payload[state.tail_idx];
                state.tail_idx += 1;
                if (state.tail_idx == payload.len) {
                    self.state = .idle;
                    return .{ .rx = value, .ack = false };
                }
                self.state = .{ .memcard = state };
                return .{ .rx = value, .ack = true };
            },
        }
    }

    pub inline fn setButtonState(self: *@This(), button: Button, pressed: bool) void {
        @field(self.buttons, @tagName(button)) = pressed; // 1=not pressed
    }
};
