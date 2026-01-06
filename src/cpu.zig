const std = @import("std");
const mem = @import("mem.zig");
const bits = @import("bits.zig");
const GTE = @import("gte.zig").GTE;

const log = std.log.scoped(.cpu);
const reset_addr = 0xbfc00000; // start of the BIOS

// zig fmt: off
pub const Reg = enum(u8) {
    zero, at, v0, v1, a0, a1, a2, a3,
    t0, t1, t2, t3, t4, t5, t6, t7,
    s0, s1, s2, s3, s4, s5, s6, s7,
    t8, t9, k0, k1, gp, sp, fp, ra,
};

pub const RegName = [_][:0]const u8{
    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra",
};
// zig fmt: on

pub const Opcode = enum(u8) {
    special = 0x00,
    bcondz = 0x01,
    j = 0x02,
    jal = 0x03,
    beq = 0x04,
    bne = 0x05,
    blez = 0x06,
    bgtz = 0x07,
    addi = 0x08,
    addiu = 0x09,
    slti = 0x0a,
    sltiu = 0x0b,
    andi = 0x0c,
    ori = 0x0d,
    xori = 0x0e,
    lui = 0x0f,
    cop0 = 0x10,
    cop1 = 0x11,
    cop2 = 0x12,
    cop3 = 0x13,
    lb = 0x20,
    lh = 0x21,
    lwl = 0x22,
    lw = 0x23,
    lbu = 0x24,
    lhu = 0x25,
    lwr = 0x26,
    sb = 0x28,
    sh = 0x29,
    swl = 0x2a,
    sw = 0x2b,
    swr = 0x2e,
    lwc0 = 0x30,
    lwc1 = 0x31,
    lwc2 = 0x32,
    lwc3 = 0x33,
    swc0 = 0x38,
    swc1 = 0x39,
    swc2 = 0x3a,
    swc3 = 0x3b,
    _,
};

pub const SpecialOpcode = enum(u8) {
    sll = 0x00,
    srl = 0x02,
    sra = 0x03,
    sllv = 0x04,
    srlv = 0x06,
    srav = 0x07,
    jr = 0x08,
    jalr = 0x09,
    syscall = 0x0c,
    break_ = 0x0d,
    mfhi = 0x10,
    mthi = 0x11,
    mflo = 0x12,
    mtlo = 0x13,
    mult = 0x18,
    multu = 0x19,
    div = 0x1a,
    divu = 0x1b,
    add = 0x20,
    addu = 0x21,
    sub = 0x22,
    subu = 0x23,
    and_ = 0x24,
    or_ = 0x25,
    xor = 0x26,
    nor = 0x27,
    slt = 0x2a,
    sltu = 0x2b,
    _,
};

pub const CopOpcode = enum(u8) {
    mfc = 0x00,
    cfc = 0x02,
    mtc = 0x04,
    ctc = 0x06,
    bc = 0x08,
    rfe = 0x10,
    _,
};

pub const BranchCond = enum(u8) {
    bltz = 0b00000,
    bltzal = 0b10000,
    bltzl = 0b00010,
    bgez = 0b00001,
    bgezal = 0b10001,
    bgezl = 0b00011,
    _,
};

pub const Instr = struct {
    code: u32,

    pub inline fn opcode(self: Instr) Opcode {
        return @enumFromInt(bits.field(self.code, 26, u6));
    }

    pub inline fn copOpcode(self: Instr) CopOpcode {
        return @enumFromInt(bits.field(self.code, 21, u5));
    }

    pub inline fn rs(self: Instr) u5 {
        return bits.field(self.code, 21, u5);
    }

    pub inline fn rt(self: Instr) u5 {
        return bits.field(self.code, 16, u5);
    }

    pub inline fn rd(self: Instr) u5 {
        return bits.field(self.code, 11, u5);
    }

    pub inline fn shift(self: Instr) u5 {
        return bits.field(self.code, 6, u5);
    }

    pub inline fn imm(self: Instr) u32 {
        return bits.field(self.code, 0, u16);
    }

    pub inline fn imm_s(self: Instr) i32 {
        const v = bits.field(self.code, 0, u16);
        return @as(i32, @as(i16, @bitCast(v)));
    }

    pub inline fn addr(self: Instr) u32 {
        return bits.field(self.code, 0, u26);
    }

    pub inline fn special(self: Instr) SpecialOpcode {
        return @enumFromInt(bits.field(self.code, 0, u6));
    }

    pub inline fn bcond(self: Instr) BranchCond {
        return @enumFromInt(bits.field(self.code, 16, u5));
    }
};

pub const Cop0 = struct {
    const reg_write_mask = [16]u32{
        0x00000000,
        0x00000000,
        0x00000000,
        0xffffffff,
        0x00000000,
        0xffffffff,
        0x00000000,
        0xffc0f03f,
        0x00000000,
        0xffffffff,
        0x00000000,
        0xffffffff,
        0xffffffff,
        0x00000300,
        0x00000000,
        0x00000000,
    };

    const Status = packed struct(u32) {
        curr_int_enable: bool, // 0
        curr_user_mode: bool, // 1
        prev_int_enable: bool, // 2
        prev_user_mode: bool, // 3
        old_int_enable: bool, // 4
        old_user_mode: bool, // 5
        _pad0: u2, // 6-7
        interrupt_mask: u8, // 8-15
        isolate_cache: bool, // 16
        swap_caches: bool, // 17
        force_parity_zero: bool, // 18
        cache_miss: bool, // 19
        parity_error: bool, // 20
        tlb_shutdown: bool, // 21
        boot_vectors: u1, // 22
        _pad1: u2, // 23-24
        reverse_endianness: bool, // 25
        _pad2: u2, // 26-27
        cop0_enable: bool, // 28
        cop1_enable: bool, // 29
        cop2_enable: bool, // 30
        cop3_enable: bool, // 31
    };

    const Cause = packed struct(u32) {
        _pad0: u2, // 0-1
        exc_code: ExcCode, // 2-6
        _pad1: u1, // 7
        interrupt_pending: u8, // 8-15
        _pad2: u12, // 16-27
        cop_number: u2, // 28-29
        branch_taken: bool, // 30
        epc_at_branch: bool, // 31
    };

    pub const ExcCode = enum(u5) {
        interrupt = 0x0,
        tlb_mod = 0x1,
        tlb_load = 0x2,
        tlb_store = 0x3,
        addr_load = 0x4,
        addr_store = 0x5,
        bus_fetch = 0x6,
        bus_data = 0x7,
        syscall = 0x8,
        breakpoint = 0x9,
        reserved_instr = 0xA,
        cop_unusable = 0xB,
        overflow = 0xC,
        _,
    };

    const reg_status: u8 = 12;
    const reg_cause: u8 = 13;
    const reg_epc: u8 = 14;

    r: [16]u32,
    depth: u8 = 0,

    pub inline fn getReg(self: *@This(), reg: u8) u32 {
        return self.r[reg];
    }

    pub inline fn setReg(self: *@This(), reg: u8, v: u32) void {
        self.r[reg] = v & reg_write_mask[reg];
    }

    pub inline fn status(self: *@This()) *Status {
        return @ptrCast(&self.r[reg_status]);
    }

    pub inline fn cause(self: *@This()) *Cause {
        return @ptrCast(&self.r[reg_cause]);
    }

    pub fn pushException(self: *@This()) void {
        const mode = self.r[reg_status] & 0x3f; // extract mode stack (6 bits)
        self.r[reg_status] &= ~@as(u32, 0x3f); // clear mode bits on the SR register
        self.r[reg_status] |= (mode << 2) & 0x3f; // shift mode stack 2 bits to the left
        self.depth, const ov = @addWithOverflow(self.depth, 1);
        if (ov != 0) log.warn("cop0 exception stack overflow", .{});
    }

    pub fn popException(self: *@This()) void {
        const mode = self.r[reg_status] & 0x3f; // extract mode stack (6 bits)
        self.r[reg_status] &= ~@as(u32, 0x0f); // clear mode bits on the SR register
        self.r[reg_status] |= (mode >> 2); // shift mode stack 2 bits to the right
        self.depth, const ov = @subWithOverflow(self.depth, 1);
        if (ov != 0) log.warn("cop0 exception stack underflow", .{});
    }
};

const LoadSlot = struct {
    r: u8 = 0,
    v: u32 = 0,

    pub fn clear(self: *LoadSlot) void {
        self.* = .{ .r = 0, .v = 0 };
    }
};

pub const CPU = struct {
    allocator: std.mem.Allocator,
    mem: *mem.Bus,
    gte: GTE,
    pc: u32,
    next_pc: u32,
    gpr: [32]u32,
    cop0: Cop0,
    instr: Instr,
    instr_addr: u32,
    delay_load: LoadSlot,
    delay_load_next: LoadSlot,
    branch_taken: bool,
    in_delay_slot: bool,
    in_exception: bool,
    in_branch: bool,
    cycles: u8,
    lo: u32,
    hi: u32,

    pub fn init(allocator: std.mem.Allocator, memory: *mem.Bus) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .mem = memory,
            .gte = .init(),
            .allocator = allocator,
            .instr = Instr{ .code = 0 },
            .instr_addr = 0,
            .gpr = undefined,
            .cop0 = undefined,
            .next_pc = 0,
            .pc = 0,
            .delay_load = .{},
            .delay_load_next = .{},
            .in_branch = false,
            .in_delay_slot = false,
            .in_exception = false,
            .branch_taken = false,
            .cycles = 0,
            .lo = 0,
            .hi = 0,
        };

        @memset(&self.gpr, 0);
        @memset(&self.cop0.r, 0);

        self.reset();
        return self;
    }

    pub fn deinit(self: *@This()) void {
        const allocator = self.allocator;
        allocator.destroy(self);
    }

    pub fn reset(self: *@This()) void {
        self.pc = reset_addr;
        self.next_pc = self.pc + 4;
    }

    fn unhandled(self: *@This(), instr: Instr) void {
        if (instr.code <= 1) return; // NOP

        std.debug.panic(
            "unhandled instruction at {x}: opcode={x} cop_opcode={x}",
            .{ self.instr_addr, instr.opcode(), instr.copOpcode() },
        );
        // log.err("unhandled instruction {x} at {x}", .{ code, self.instr_addr });
        // _ = self;
    }

    fn exception(self: *@This(), exc_code: Cop0.ExcCode) void {
        self.cop0.cause().exc_code = exc_code;
        self.cop0.pushException();

        if (self.in_delay_slot) {
            const branch_addr, _ = @subWithOverflow(self.instr_addr, 4);
            self.cop0.r[Cop0.reg_epc] = branch_addr;
            self.cop0.cause().epc_at_branch = true;
        } else {
            self.cop0.r[Cop0.reg_epc] = self.instr_addr;
            self.cop0.cause().epc_at_branch = false;
        }

        // Jump to exception handler (no delay slot)
        self.pc = if (self.cop0.status().boot_vectors != 0) 0xbfc00180 else 0x80000080;
        self.next_pc, _ = @addWithOverflow(self.pc, 4);
        self.in_exception = true;
    }

    inline fn jump(self: *@This(), addr: u32) void {
        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        self.in_branch = true;
        self.next_pc = addr;
    }

    inline fn branchIf(self: *@This(), cond: bool, offset: i32) void {
        self.in_branch = true;
        self.branch_taken = false;

        if (cond) {
            const addr, _ = @addWithOverflow(self.pc, @as(u32, @bitCast(offset << 2)));
            if (addr % 4 != 0) {
                @branchHint(.unlikely);
                self.exception(.addr_load);
                return;
            }
            self.next_pc = addr;
            self.branch_taken = true;
        }
    }

    inline fn writeGpr(self: *@This(), r: u8, v: u32) void {
        if (self.delay_load.r == r) {
            self.delay_load.clear();
        }

        self.gpr[r] = v;
    }

    inline fn writeGprDelay(self: *@This(), r: u8, v: u32) void {
        if (self.delay_load.r == r) {
            @branchHint(.unlikely);
            self.delay_load.clear();
        }

        self.delay_load_next = .{
            .r = r,
            .v = v,
        };
    }

    pub fn resetPC(self: *@This(), pc: u32) void {
        self.pc = pc;
        self.next_pc = pc +% 4;
    }

    pub fn requestInterrupt(self: *@This(), v: bool) void {
        if (v) {
            self.cop0.r[Cop0.reg_cause] |= @as(u32, 1 << 10);
        } else {
            self.cop0.r[Cop0.reg_cause] &= ~@as(u32, 1 << 10);
        }
    }

    fn checkInterrupt(self: *@This()) bool {
        const status = self.cop0.status();
        if (!status.curr_int_enable) return false;

        if ((self.cop0.cause().interrupt_pending & status.interrupt_mask) != 0) {
            self.exception(.interrupt);
            return true;
        }

        return false;
    }

    pub fn tick(self: *@This()) u8 {
        self.gpr[0] = 0;

        const v = self.memRead(u32, self.pc);
        const instr = Instr{ .code = v };
        self.instr_addr = self.pc;
        self.instr = instr;

        self.pc = self.next_pc;
        self.next_pc = self.pc +% 4;

        // Each instruction is 2 cycles on average
        self.cycles = 2;

        // If the last instruction was a branch, then we are in delay slot
        self.in_delay_slot = self.in_branch;
        self.in_branch = false;

        // Break if there is an interrupt
        if (self.checkInterrupt()) {
            return self.cycles;
        }

        // Execute the instruction
        switch (instr.opcode()) {
            .special => switch (instr.special()) {
                .sll => self.sll(instr),
                .or_ => self.or_(instr),
                .sltu => self.sltu(instr),
                .addu => self.addu(instr),
                .jr => self.jr(instr),
                .and_ => self.and_(instr),
                .add => self.add(instr),
                .jalr => self.jalr(instr),
                .subu => self.subu(instr),
                .sra => self.sra(instr),
                .div => self.div(instr),
                .mflo => self.mflo(instr),
                .mfhi => self.mfhi(instr),
                .srl => self.srl(instr),
                .divu => self.divu(instr),
                .slt => self.slt(instr),
                .syscall => self.syscall(instr),
                .mtlo => self.mtlo(instr),
                .mthi => self.mthi(instr),
                .sllv => self.sllv(instr),
                .nor => self.nor(instr),
                .srav => self.srav(instr),
                .srlv => self.srlv(instr),
                .multu => self.multu(instr),
                .xor => self.xor(instr),
                .break_ => self.break_(instr),
                .mult => self.mult(instr),
                .sub => self.sub(instr),
                else => self.unhandled(instr),
            },
            .cop0 => switch (instr.copOpcode()) {
                .mtc => self.mtc0(instr),
                .mfc => self.mfc0(instr),
                .rfe => self.rfe(instr),
                else => self.unhandled(instr),
            },
            .cop2 => switch (instr.copOpcode()) {
                .mfc => self.mfc2(instr),
                .mtc => self.mtc2(instr),
                .cfc => self.cfc2(instr),
                .ctc => self.ctc2(instr),
                else => self.cop2cmd(instr),
            },
            .bcondz => switch (instr.bcond()) {
                .bltz => self.bltz(instr),
                .bgez => self.bgez(instr),
                .bltzal => self.bltzal(instr),
                .bgezal => self.bgezal(instr),
                else => switch ((instr.code >> 16) & 0x1) {
                    0 => self.bltz(instr),
                    1 => self.bgez(instr),
                    else => unreachable,
                },
            },
            .lui => self.lui(instr),
            .ori => self.ori(instr),
            .sw => self.sw(instr),
            .addi => self.addi(instr),
            .addiu => self.addiu(instr),
            .j => self.j(instr),
            .bne => self.bne(instr),
            .lw => self.lw(instr),
            .sh => self.sh(instr),
            .jal => self.jal(instr),
            .andi => self.andi(instr),
            .sb => self.sb(instr),
            .lb => self.lb(instr),
            .beq => self.beq(instr),
            .bgtz => self.bgtz(instr),
            .blez => self.blez(instr),
            .lbu => self.lbu(instr),
            .slti => self.slti(instr),
            .sltiu => self.sltiu(instr),
            .lhu => self.lhu(instr),
            .lh => self.lh(instr),
            .xori => self.xori(instr),
            .lwl => self.lwl(instr),
            .lwr => self.lwr(instr),
            .swl => self.swl(instr),
            .swr => self.swr(instr),
            .cop1 => self.exception(.cop_unusable),
            .cop3 => self.exception(.cop_unusable),
            .lwc2 => self.lwc2(instr),
            .swc2 => self.swc2(instr),
            else => self.unhandled(instr),
        }

        // Handle delayed loads
        self.gpr[self.delay_load.r] = self.delay_load.v;
        self.delay_load = self.delay_load_next;
        self.delay_load_next.clear();

        return self.cycles;
    }

    inline fn memRead(self: *@This(), comptime T: type, addr: u32) T {
        return self.mem.read(T, addr);
    }

    inline fn memWrite(self: *@This(), comptime T: type, addr: u32, v: T) void {
        self.mem.write(T, addr, v);
    }

    // --------------------------------------
    // Coprocessor instructions
    // --------------------------------------

    fn lwc2(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        const v = self.memRead(u32, addr);
        self.gte.writeReg(instr.rt(), v);
    }

    fn swc2(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_store);
            return;
        }

        const v = self.gte.readReg(instr.rt());
        self.memWrite(u32, addr, v);
    }

    fn mfc2(self: *@This(), instr: Instr) void {
        const gte_reg: u8 = @intCast(instr.rd());
        const v = self.gte.readReg(gte_reg);
        self.writeGpr(instr.rt(), v);
    }

    fn mtc2(self: *@This(), instr: Instr) void {
        const gte_reg: u8 = @intCast(instr.rd());
        self.gte.writeReg(gte_reg, self.gpr[instr.rt()]);
    }

    fn cfc2(self: *@This(), instr: Instr) void {
        const gte_reg: u8 = 32 + @as(u8, instr.rd());
        const v = self.gte.readReg(gte_reg);
        self.writeGpr(instr.rt(), v);
    }

    fn ctc2(self: *@This(), instr: Instr) void {
        const gte_reg: u8 = 32 + @as(u8, instr.rd());
        self.gte.writeReg(gte_reg, self.gpr[instr.rt()]);
    }

    fn cop2cmd(self: *@This(), instr: Instr) void {
        self.gte.exec(instr.code);
    }

    // --------------------------------------
    // Instructions
    // --------------------------------------

    fn lui(self: *@This(), instr: Instr) void {
        self.writeGpr(instr.rt(), instr.imm() << 16);
    }

    fn ori(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()] | instr.imm();
        self.writeGpr(instr.rt(), v);
    }

    fn andi(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()] & instr.imm();
        self.writeGpr(instr.rt(), v);
    }

    fn sw(self: *@This(), instr: Instr) void {
        if (self.cop0.status().isolate_cache) {
            // log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_store);
            return;
        }

        self.memWrite(u32, addr, self.gpr[instr.rt()]);
    }

    fn addiu(self: *@This(), instr: Instr) void {
        const sum, _ = add32s(self.gpr[instr.rs()], instr.imm_s());
        self.writeGpr(instr.rt(), sum);
    }

    fn addi(self: *@This(), instr: Instr) void {
        const sum, const ov = add32s(self.gpr[instr.rs()], instr.imm_s());

        if (ov != 0) {
            self.exception(.overflow);
            return;
        }

        self.writeGpr(instr.rt(), sum);
    }

    fn add(self: *@This(), instr: Instr) void {
        const sum, const ov = addAsSigned(
            self.gpr[instr.rs()],
            self.gpr[instr.rt()],
        );

        if (ov != 0) {
            self.exception(.overflow);
            return;
        }

        self.writeGpr(instr.rd(), sum);
    }

    fn addu(self: *@This(), instr: Instr) void {
        const sum, _ = @addWithOverflow(
            self.gpr[instr.rs()],
            self.gpr[instr.rt()],
        );
        self.writeGpr(instr.rd(), sum);
    }

    fn j(self: *@This(), instr: Instr) void {
        self.jump((self.pc & 0xF0000000) | (instr.addr() << 2));
    }

    fn bne(self: *@This(), instr: Instr) void {
        self.branchIf(
            self.gpr[instr.rs()] != self.gpr[instr.rt()],
            instr.imm_s(),
        );
    }

    fn beq(self: *@This(), instr: Instr) void {
        self.branchIf(
            self.gpr[instr.rs()] == self.gpr[instr.rt()],
            instr.imm_s(),
        );
    }

    fn lw(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        self.writeGprDelay(instr.rt(), self.memRead(u32, addr));
    }

    fn lb(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());
        const v = signExtend(u8, self.memRead(u8, addr));

        self.writeGprDelay(instr.rt(), v);
    }

    fn lbu(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());
        const v = self.memRead(u8, addr);

        self.writeGprDelay(instr.rt(), v);
    }

    fn or_(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()] | self.gpr[instr.rt()];
        self.writeGpr(instr.rd(), v);
    }

    fn and_(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()] & self.gpr[instr.rt()];
        self.writeGpr(instr.rd(), v);
    }

    fn sll(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rt()] << instr.shift();
        self.writeGpr(instr.rd(), v);
    }

    fn sltu(self: *@This(), instr: Instr) void {
        const less = self.gpr[instr.rs()] < self.gpr[instr.rt()];
        self.writeGpr(instr.rd(), @intFromBool(less));
    }

    fn sh(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_store);
            return;
        }

        const v = self.gpr[instr.rt()];
        self.memWrite(u16, addr, @truncate(v));
    }

    fn sb(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());
        const v = self.gpr[instr.rt()];

        self.memWrite(u8, addr, @truncate(v));
    }

    fn jal(self: *@This(), instr: Instr) void {
        const ra = @intFromEnum(Reg.ra);
        self.writeGpr(ra, self.next_pc);
        self.jump((self.pc & 0xF0000000) | (instr.addr() << 2));
    }

    fn jr(self: *@This(), instr: Instr) void {
        self.jump(self.gpr[instr.rs()]);
    }

    fn bgtz(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()];
        self.branchIf(
            @as(i32, @bitCast(v)) > 0,
            instr.imm_s(),
        );
    }

    fn blez(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()];
        self.branchIf(
            @as(i32, @bitCast(v)) <= 0,
            instr.imm_s(),
        );
    }

    fn jalr(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()];
        self.writeGpr(instr.rd(), self.next_pc);
        self.jump(v);
    }

    fn bltz(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()];
        self.branchIf(
            @as(i32, @bitCast(v)) < 0,
            instr.imm_s(),
        );
    }

    fn bgez(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()];
        self.branchIf(
            @as(i32, @bitCast(v)) >= 0,
            instr.imm_s(),
        );
    }

    fn slti(self: *@This(), instr: Instr) void {
        const rs = @as(i32, @bitCast(self.gpr[instr.rs()]));
        const imm = @as(i32, @bitCast(instr.imm_s()));
        const less = @intFromBool(rs < imm);
        self.writeGpr(instr.rt(), less);
    }

    fn subu(self: *@This(), instr: Instr) void {
        const v, _ = @subWithOverflow(self.gpr[instr.rs()], self.gpr[instr.rt()]);
        self.writeGpr(instr.rd(), v);
    }

    fn sra(self: *@This(), instr: Instr) void {
        const v = @as(i32, @bitCast(self.gpr[instr.rt()])) >> instr.shift();
        self.writeGpr(instr.rd(), @as(u32, @bitCast(v)));
    }

    fn div(self: *@This(), instr: Instr) void {
        const num = @as(i32, @bitCast(self.gpr[instr.rs()]));
        const denom = @as(i32, @bitCast(self.gpr[instr.rt()]));

        if (std.math.divTrunc(i32, num, denom)) |v| {
            self.lo = @bitCast(v);
            self.hi = @bitCast(@rem(num, denom));
        } else |err| switch (err) {
            error.DivisionByZero => {
                self.lo = if (num >= 0) 0xffffffff else 0x00000001;
                self.hi = @bitCast(num);
            },
            error.Overflow => {
                self.lo = 0x80000000;
                self.hi = 0;
            },
        }
    }

    fn divu(self: *@This(), instr: Instr) void {
        const num = self.gpr[instr.rs()];
        const denom = self.gpr[instr.rt()];

        if (denom == 0) {
            @branchHint(.unlikely);
            self.lo = 0xffffffff;
            self.hi = num;
            return;
        }

        self.lo = @divTrunc(num, denom);
        self.hi = @mod(num, denom);
    }

    fn mflo(self: *@This(), instr: Instr) void {
        self.writeGpr(instr.rd(), self.lo);
    }

    fn mfhi(self: *@This(), instr: Instr) void {
        self.writeGpr(instr.rd(), self.hi);
    }

    fn srl(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rt()] >> instr.shift();
        self.writeGpr(instr.rd(), v);
    }

    fn sltiu(self: *@This(), instr: Instr) void {
        const less = self.gpr[instr.rs()] < @as(u32, @bitCast(instr.imm_s()));
        self.writeGpr(instr.rt(), @intFromBool(less));
    }

    fn slt(self: *@This(), instr: Instr) void {
        const a = @as(i32, @bitCast(self.gpr[instr.rs()]));
        const b = @as(i32, @bitCast(self.gpr[instr.rt()]));
        self.writeGpr(instr.rd(), @intFromBool(a < b));
    }

    fn syscall(self: *@This(), _: Instr) void {
        self.exception(.syscall);
    }

    fn mtlo(self: *@This(), instr: Instr) void {
        self.lo = self.gpr[instr.rs()];
    }

    fn mthi(self: *@This(), instr: Instr) void {
        self.hi = self.gpr[instr.rs()];
    }

    fn mfc0(self: *@This(), instr: Instr) void {
        const v = self.cop0.getReg(instr.rd());
        self.writeGprDelay(instr.rt(), v);
    }

    fn mtc0(self: *@This(), instr: Instr) void {
        self.cop0.setReg(instr.rd(), self.gpr[instr.rt()]);
    }

    fn rfe(self: *@This(), _: Instr) void {
        self.cop0.popException();

        if (self.cop0.depth == 0) {
            self.in_exception = false;
        }
    }

    fn lhu(self: *@This(), instr: Instr) void {
        if (self.cop0.status().isolate_cache) {
            // log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        const v = self.memRead(u16, addr);
        self.writeGprDelay(instr.rt(), v);
    }

    fn sllv(self: *@This(), instr: Instr) void {
        const shift = @as(u5, @truncate(self.gpr[instr.rs()] & 0x1F)); // only 5 bits are used
        const v = self.gpr[instr.rt()] << shift;
        self.writeGpr(instr.rd(), v);
    }

    fn lh(self: *@This(), instr: Instr) void {
        if (self.cop0.status().isolate_cache) {
            // log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        const v = signExtend(u16, self.memRead(u16, addr));
        self.writeGprDelay(instr.rt(), v);
    }

    fn nor(self: *@This(), instr: Instr) void {
        const v = ~(self.gpr[instr.rs()] | self.gpr[instr.rt()]);
        self.writeGpr(instr.rd(), v);
    }

    fn srav(self: *@This(), instr: Instr) void {
        const shift = @as(u5, @truncate(self.gpr[instr.rs()] & 0x1F)); // 5 bits
        const v = @as(i32, @bitCast(self.gpr[instr.rt()])) >> shift;
        self.writeGpr(instr.rd(), @as(u32, @bitCast(v)));
    }

    fn srlv(self: *@This(), instr: Instr) void {
        const shift = @as(u5, @truncate(self.gpr[instr.rs()] & 0x1F)); // 5 bits
        const v = self.gpr[instr.rt()] >> shift;
        self.writeGpr(instr.rd(), v);
    }

    fn multu(self: *@This(), instr: Instr) void {
        const a = @as(u64, self.gpr[instr.rs()]);
        const b = @as(u64, self.gpr[instr.rt()]);
        const v, _ = @mulWithOverflow(a, b);

        self.lo = @truncate(v);
        self.hi = @truncate(v >> 32);
    }

    fn xor(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()] ^ self.gpr[instr.rt()];
        self.writeGpr(instr.rd(), v);
    }

    fn xori(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()] ^ instr.imm();
        self.writeGpr(instr.rt(), v);
    }

    fn break_(self: *@This(), _: Instr) void {
        self.exception(.breakpoint);
    }

    fn mult(self: *@This(), instr: Instr) void {
        const a = @as(i64, @as(i32, @bitCast(self.gpr[instr.rs()])));
        const b = @as(i64, @as(i32, @bitCast(self.gpr[instr.rt()])));
        const m, _ = @mulWithOverflow(a, b);
        const v = @as(u64, @bitCast(m));

        self.lo = @truncate(v);
        self.hi = @truncate(v >> 32);
    }

    fn sub(self: *@This(), instr: Instr) void {
        const a = @as(i32, @bitCast(self.gpr[instr.rs()]));
        const b = @as(i32, @bitCast(self.gpr[instr.rt()]));

        const v, const ov = @subWithOverflow(a, b);
        if (ov != 0) {
            self.exception(.overflow);
            return;
        }

        self.writeGpr(instr.rd(), @bitCast(v));
    }

    fn bltzal(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()];
        self.writeGpr(@intFromEnum(Reg.ra), self.next_pc);
        self.branchIf(@as(i32, @bitCast(v)) < 0, instr.imm_s());
    }

    fn bgezal(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()];
        self.writeGpr(@intFromEnum(Reg.ra), self.next_pc);
        self.branchIf(@as(i32, @bitCast(v)) >= 0, instr.imm_s());
    }

    fn bltzl(self: *@This(), instr: Instr) void {
        const v = self.gpr[instr.rs()];

        self.branchIf(
            @as(i32, @bitCast(v)) < 0,
            instr.imm_s(),
        );

        if (!self.branch_taken) {
            self.pc = self.next_pc;
            self.next_pc += 4;
        }
    }

    fn lwl(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());
        const aligned_addr = addr & ~@as(u32, 0x3);

        const load_v = self.memRead(u32, aligned_addr);
        var curr_v = self.gpr[instr.rt()];

        if (self.delay_load.r == instr.rt()) {
            curr_v = self.delay_load.v;
        }

        const v = switch (addr & 0x3) {
            0 => (curr_v & 0x00ffffff) | (load_v << 24),
            1 => (curr_v & 0x0000ffff) | (load_v << 16),
            2 => (curr_v & 0x000000ff) | (load_v << 8),
            3 => (curr_v & 0x00000000) | (load_v << 0),
            else => undefined,
        };

        self.writeGprDelay(instr.rt(), v);
    }

    fn lwr(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());
        const aligned_addr = addr & ~@as(u32, 0x3);

        const load_v = self.memRead(u32, aligned_addr);
        var curr_v = self.gpr[instr.rt()];

        if (self.delay_load.r == instr.rt()) {
            curr_v = self.delay_load.v;
        }

        const v = switch (addr & 0x3) {
            0 => (curr_v & 0x00000000) | (load_v >> 0),
            1 => (curr_v & 0xff000000) | (load_v >> 8),
            2 => (curr_v & 0xffff0000) | (load_v >> 16),
            3 => (curr_v & 0xffffff00) | (load_v >> 24),
            else => undefined,
        };

        self.writeGprDelay(instr.rt(), v);
    }

    fn swl(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());
        const aligned_addr = addr & ~@as(u32, 0x3);

        const curr_v = self.memRead(u32, aligned_addr);
        const store_v = self.gpr[instr.rt()];

        const v = switch (addr & 0x3) {
            0 => (curr_v & 0xffffff00) | (store_v >> 24),
            1 => (curr_v & 0xffff0000) | (store_v >> 16),
            2 => (curr_v & 0xff000000) | (store_v >> 8),
            3 => (curr_v & 0x00000000) | (store_v >> 0),
            else => unreachable,
        };

        self.memWrite(u32, aligned_addr, v);
    }

    fn swr(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = add32s(base, instr.imm_s());
        const aligned_addr = addr & ~@as(u32, 0x3);

        const curr_v = self.memRead(u32, aligned_addr);
        const store_v = self.gpr[instr.rt()];

        const v = switch (addr & 0x03) {
            0 => (curr_v & 0x00000000) | (store_v << 0),
            1 => (curr_v & 0x000000ff) | (store_v << 8),
            2 => (curr_v & 0x0000ffff) | (store_v << 16),
            3 => (curr_v & 0x00ffffff) | (store_v << 24),
            else => unreachable,
        };

        self.memWrite(u32, aligned_addr, v);
    }
};

inline fn signExtend(comptime T: type, v: T) u32 {
    return switch (T) {
        u8 => @bitCast(@as(i32, @as(i8, @bitCast(v)))),
        u16 => @bitCast(@as(i32, @as(i16, @bitCast(v)))),
        else => undefined,
    };
}

inline fn add32s(a: u32, b: i32) struct { u32, u1 } {
    const v, const ov = @addWithOverflow(@as(i32, @bitCast(a)), b);
    return .{ @bitCast(v), ov };
}

inline fn addAsSigned(a: u32, b: u32) struct { u32, u1 } {
    const v, const ov = @addWithOverflow(
        @as(i32, @bitCast(a)),
        @as(i32, @bitCast(b)),
    );
    return .{ @bitCast(v), ov };
}
