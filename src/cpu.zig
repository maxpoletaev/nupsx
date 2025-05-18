const std = @import("std");
const mem = @import("mem.zig");

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
        return @enumFromInt(@as(u8, @truncate(self.code >> 26))); // 6 bits
    }

    pub inline fn copOpcode(self: Instr) CopOpcode {
        return @enumFromInt(@as(u8, @truncate(self.code >> 21))); // 5 bits
    }

    pub inline fn rs(self: Instr) u5 {
        return @truncate(self.code >> 21); // 5 bits
    }

    pub inline fn rt(self: Instr) u5 {
        return @truncate(self.code >> 16); // 5 bits
    }

    pub inline fn rd(self: Instr) u5 {
        return @truncate(self.code >> 11); // 5 bits
    }

    pub inline fn shift(self: Instr) u5 {
        return @truncate(self.code >> 6); // 5 bits
    }

    pub inline fn imm(self: Instr) u32 {
        return @truncate(self.code & 0xFFFF); // 16 bits
    }

    pub inline fn imm_s(self: Instr) u32 {
        return signExtend(u16, @as(u16, @truncate(self.code))); // 16 bits
    }

    pub inline fn addr(self: Instr) u32 {
        return @truncate(self.code & 0x03ffffff); // 26 bits
    }

    pub inline fn special(self: Instr) SpecialOpcode {
        const v: u8 = @truncate(self.code & 0x3f); // 6 bits
        return @enumFromInt(v);
    }

    pub inline fn bcond(self: Instr) BranchCond {
        return @enumFromInt((self.code >> 16) & 0x1f); // 5 bits
    }
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

const COP0 = struct {
    const write_mask_table = [16]u32{
        0x00000000, // cop0r0   - N/A
        0x00000000, // cop0r1   - N/A
        0x00000000, // cop0r2   - N/A
        0xffffffff, // BPC      - Breakpoint on execute (R/W)
        0x00000000, // cop0r4   - N/A
        0xffffffff, // BDA      - Breakpoint on data access (R/W)
        0x00000000, // JUMPDEST - Randomly memorized jump address (R)
        0xffc0f03f, // DCIC     - Breakpoint control (R/W)
        0x00000000, // BadVaddr - Bad Virtual Address (R)
        0xffffffff, // BDAM     - Data Access breakpoint mask (R/W)
        0x00000000, // cop0r10  - N/A
        0xffffffff, // BPCM     - Execute breakpoint mask (R/W)
        0xffffffff, // SR       - System status register (R/W)
        0x00000300, // CAUSE    - Describes the most recently recognised exception (R)
        0x00000000, // EPC      - Return Address from Trap (R)
        0x00000000, // PRID     - Processor ID (R)
    };

    const Status = packed struct(u32) {
        iec: u1, // Current Interrupt Enable (0=Disable, 1=Enable)
        kuc: u1, // Current Kernel/User Mode (0=Kernel, 1=User)
        iep: u1, // Previous Interrupt Enable
        kup: u1, // Previous Kernel/User Mode
        ieo: u1, // Old Interrupt Enable
        kuo: u1, // Old Kernel/User Mode
        _pad0: u2, // Not used (zero)
        im: u8, // 8 bit interrupt mask fields
        isc: u1, // Isolate Cache (0=No, 1=Isolate)
        swc: u1, // Swapped cache mode (0=Normal, 1=Swapped)
        pz: u1, // When set cache parity bits are written as 0
        cm: u1, // Result of last load with D-cache isolated
        pe: u1, // Cache parity error
        ts: u1, // TLB shutdown
        bev: u1, // Boot exception vectors (0=RAM/KSEG0, 1=ROM/KSEG1)
        _pad1: u2, // Not used (zero)
        re: u1, // Reverse endianness
        _pad2: u2, // Not used (zero)
        cu0: u1, // COP0 Enable
        cu1: u1, // COP1 Enable (none in PSX)
        cu2: u1, // COP2 Enable (GTE in PSX)
        cu3: u1, // COP3 Enable (none in PSX)
    };

    const Cause = packed struct(u32) {
        _pad0: u2, // Not used (zero)
        exc_code: u5, // Exception code
        _pad1: u1, // Not used (zero)
        sw: u2, // Software interrupt
        ip: u6, // Interrupt pending
        _pad2: u12, // Not used (zero)
        ce: u2, // Coprocessor number
        bt: u1, // Branch taken
        bd: u1, // EPC points to the branch instuction instead of the instruction in the branch delay slot
    };

    const reg_sr: u8 = 12;
    const reg_cause: u8 = 13;
    const reg_epc: u8 = 14;

    r: [16]u32,
    depth: u8 = 0,

    pub inline fn read(self: *@This(), reg: u8, v: u32) void {
        const mask = write_mask_table[reg];
        self.r[reg] = v & mask;
    }

    pub inline fn write(self: *@This(), reg: u8) u32 {
        return self.r[reg];
    }

    pub inline fn status(self: *@This()) *Status {
        return @ptrCast(&self.r[reg_sr]);
    }

    pub inline fn cause(self: *@This()) *Cause {
        return @ptrCast(&self.r[reg_cause]);
    }

    pub fn push(self: *@This()) void {
        var sr = self.r[reg_sr];
        const mode = sr & 0x3F; // extract mode stack (6 bits)
        sr &= ~@as(u32, 0x3F); // clear mode bits on the SR register
        sr |= (mode << 2) & 0x3F; // shift mode stack 2 bits to the left
        self.r[reg_sr] = sr; // store it back
        self.depth += 1;
    }

    pub fn pop(self: *@This()) void {
        var sr = self.r[reg_sr];
        const mode = sr & 0x3F; // extract mode stack (6 bits)
        sr &= ~@as(u32, 0x3F); // clear mode bits on the SR register
        sr |= (mode >> 2) & 0x3F; // shift mode stack 2 bits to the right
        self.r[reg_sr] = sr; // store it back
        self.depth -= 1;
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
    pc: u32,
    next_pc: u32,
    gpr: [32]u32,
    cop0: COP0,
    instr: Instr,
    instr_addr: u32,
    stall: bool,
    breakpoint: u32,
    delay_load: LoadSlot,
    delay_load_next: LoadSlot,
    branch_taken: bool,
    in_delay_slot: bool,
    in_exception: bool,
    in_branch: bool,
    lo: u32,
    hi: u32,

    pub fn init(allocator: std.mem.Allocator, memory: *mem.Bus) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .mem = memory,
            .allocator = allocator,
            .instr = Instr{ .code = 0 },
            .instr_addr = 0,
            .gpr = undefined,
            .cop0 = undefined,
            .next_pc = 0,
            .stall = false,
            .pc = 0,
            .delay_load = .{},
            .delay_load_next = .{},
            .breakpoint = 0,
            .in_branch = false,
            .in_delay_slot = false,
            .in_exception = false,
            .branch_taken = false,
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
        self.stall = false;
        self.pc = reset_addr;
        self.next_pc = self.pc + 4;
    }

    fn unhandled(self: *@This(), code: u32) void {
        if (code == 0 or code == 1) {
            return;
        }
        log.err("unhandled instruction: {x}", .{code});
        self.stall = true;
    }

    pub fn setBreakpoint(self: *@This(), addr: u32) void {
        self.breakpoint = addr;
    }

    fn exception(self: *@This(), exc_code: ExcCode) void {
        self.cop0.cause().exc_code = @intFromEnum(exc_code);
        self.cop0.r[COP0.reg_epc] = self.instr_addr;
        self.cop0.push();

        // If we are in delay slot, EPC should point to the branch instruction
        if (self.in_delay_slot) {
            const branch_addr, _ = @subWithOverflow(self.instr_addr, 4);
            self.cop0.r[COP0.reg_epc] = branch_addr;
            self.cop0.cause().bd = 1;
        }

        // Jump to exception handler (no delay slot)
        self.pc = if (self.cop0.status().bev != 0) 0xBFC00180 else 0x80000080;
        self.next_pc, _ = @addWithOverflow(self.pc, 4);

        if (exc_code != .syscall) {
            const exc_name = std.enums.tagName(ExcCode, exc_code).?;
            log.debug("cpu exception: {s}", .{exc_name});
        }

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

    inline fn branchIf(self: *@This(), cond: bool, offset: u32) void {
        self.in_branch = true;
        self.branch_taken = false;

        if (cond) {
            const addr, _ = @addWithOverflow(self.pc, offset << 2);
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

    pub fn forceSetPC(self: *@This(), pc: u32) void {
        self.pc = pc;
        self.next_pc, _ = @addWithOverflow(pc, 4);
    }

    pub fn step(self: *@This()) void {
        self.gpr[0] = 0;

        // Fetch the instruction
        const instr_code = self.mem.readWord(self.pc);
        const instr = Instr{ .code = instr_code };
        self.instr_addr = self.pc;
        self.instr = instr;

        // Increment the PC
        self.pc = self.next_pc;
        self.next_pc, _ = @addWithOverflow(self.pc, 4);

        // If the last instruction was a branch, then we are in delay slot
        self.in_delay_slot = self.in_branch;
        self.in_branch = false;

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
                else => self.unhandled(instr_code),
            },
            .cop0 => switch (instr.copOpcode()) {
                .mtc => self.mtc0(instr),
                .mfc => self.mfc0(instr),
                .rfe => self.rfe(instr),
                else => self.unhandled(instr_code),
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
            else => self.unhandled(instr_code),
        }

        // Handle delayed loads
        self.gpr[self.delay_load.r] = self.delay_load.v;
        self.delay_load = self.delay_load_next;
        self.delay_load_next.clear();
    }

    pub fn execute(self: *@This()) void {
        if (self.stall) {
            return;
        }

        if (self.breakpoint != 0 and self.pc == self.breakpoint) {
            @branchHint(.unlikely);
            log.debug("reached breakpoint: {x}", .{self.breakpoint});
            self.stall = true;
            return;
        }

        self.step();
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
        if (self.cop0.status().isc == 1) {
            log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs()];
        const addr, _ = @addWithOverflow(base, instr.imm_s());

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_store);
            return;
        }

        self.mem.writeWord(addr, self.gpr[instr.rt()]);
    }

    fn addiu(self: *@This(), instr: Instr) void {
        const sum, _ = addAsSigned(self.gpr[instr.rs()], instr.imm_s());
        self.writeGpr(instr.rt(), sum);
    }

    fn addi(self: *@This(), instr: Instr) void {
        const sum, const ov = addAsSigned(
            self.gpr[instr.rs()],
            instr.imm_s(),
        );

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
        const addr, _ = @addWithOverflow(base, instr.imm_s());

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        self.writeGprDelay(instr.rt(), self.mem.readWord(addr));
    }

    fn lb(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = @addWithOverflow(base, instr.imm_s());
        const v = signExtend(u8, self.mem.readByte(addr));

        self.writeGprDelay(instr.rt(), v);
    }

    fn lbu(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = @addWithOverflow(base, instr.imm_s());
        const v = self.mem.readByte(addr);

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
        const addr, _ = @addWithOverflow(base, instr.imm_s());

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_store);
            return;
        }

        const v = self.gpr[instr.rt()];
        self.mem.writeHalf(addr, @truncate(v));
    }

    fn sb(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = @addWithOverflow(base, instr.imm_s());
        const v = self.gpr[instr.rt()];

        self.mem.writeByte(addr, @truncate(v));
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
        self.writeGpr(instr.rd(), self.next_pc);
        self.jump(self.gpr[instr.rs()]);
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
            self.hi = @bitCast(@mod(num, denom));
        } else |err| switch (err) {
            error.DivisionByZero => {
                self.lo = if (num >= 0) 0xffffffff else 0x00000001;
                self.hi = @bitCast(denom);
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
        const less = self.gpr[instr.rs()] < instr.imm_s();
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
        const v = self.cop0.write(instr.rd());
        self.writeGprDelay(instr.rt(), v);
    }

    fn mtc0(self: *@This(), instr: Instr) void {
        self.cop0.read(instr.rd(), self.gpr[instr.rt()]);
    }

    fn rfe(self: *@This(), _: Instr) void {
        self.cop0.pop();

        if (self.cop0.depth == 0) {
            self.in_exception = false;
        }
    }

    fn lhu(self: *@This(), instr: Instr) void {
        if (self.cop0.status().isc == 1) {
            log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs()];
        const addr, _ = @addWithOverflow(base, instr.imm_s());

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        const v = self.mem.readHalf(addr);
        self.writeGprDelay(instr.rt(), v);
    }

    fn sllv(self: *@This(), instr: Instr) void {
        const shift = @as(u5, @truncate(self.gpr[instr.rs()] & 0x1F)); // only 5 bits are used
        const v = self.gpr[instr.rt()] << shift;
        self.writeGpr(instr.rd(), v);
    }

    fn lh(self: *@This(), instr: Instr) void {
        if (self.cop0.status().isc == 1) {
            log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs()];
        const addr, _ = @addWithOverflow(base, instr.imm_s());

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        const v = signExtend(u16, self.mem.readHalf(addr));
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
        self.writeGpr(@intFromEnum(Reg.ra), self.next_pc);
        const v = self.gpr[instr.rs()];

        self.branchIf(
            @as(i32, @bitCast(v)) < 0,
            instr.imm_s(),
        );
    }

    fn bgezal(self: *@This(), instr: Instr) void {
        self.writeGpr(@intFromEnum(Reg.ra), self.next_pc);
        const v = self.gpr[instr.rs()];

        self.branchIf(
            @as(i32, @bitCast(v)) >= 0,
            instr.imm_s(),
        );
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
        const addr, _ = @addWithOverflow(base, instr.imm_s());
        const aligned_addr = addr & ~@as(u32, 0x3);

        const load_v = self.mem.readWord(aligned_addr);
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
        const addr, _ = @addWithOverflow(base, instr.imm_s());
        const aligned_addr = addr & ~@as(u32, 0x3);

        const load_v = self.mem.readWord(aligned_addr);
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
        const addr, _ = @addWithOverflow(base, instr.imm_s());
        const aligned_addr = addr & ~@as(u32, 0x3);

        const curr_v = self.mem.readWord(aligned_addr);
        const store_v = self.gpr[instr.rt()];

        const v = switch (addr & 0x3) {
            0 => (curr_v & 0xffffff00) | (store_v >> 24),
            1 => (curr_v & 0xffff0000) | (store_v >> 16),
            2 => (curr_v & 0xff000000) | (store_v >> 8),
            3 => (curr_v & 0x00000000) | (store_v >> 0),
            else => unreachable,
        };

        self.mem.writeWord(aligned_addr, v);
    }

    fn swr(self: *@This(), instr: Instr) void {
        const base = self.gpr[instr.rs()];
        const addr, _ = @addWithOverflow(base, instr.imm_s());
        const aligned_addr = addr & ~@as(u32, 0x3);

        const curr_v = self.mem.readWord(aligned_addr);
        const store_v = self.gpr[instr.rt()];

        const v = switch (addr & 0x03) {
            0 => (curr_v & 0x00000000) | (store_v << 0),
            1 => (curr_v & 0x000000ff) | (store_v << 8),
            2 => (curr_v & 0x0000ffff) | (store_v << 16),
            3 => (curr_v & 0x00ffffff) | (store_v << 24),
            else => unreachable,
        };

        self.mem.writeWord(aligned_addr, v);
    }
};

inline fn signExtend(comptime T: type, v: T) u32 {
    return switch (T) {
        u8 => @bitCast(@as(i32, @as(i8, @bitCast(v)))),
        u16 => @bitCast(@as(i32, @as(i16, @bitCast(v)))),
        else => undefined,
    };
}

inline fn addAsSigned(a: u32, b: u32) struct { u32, u1 } {
    const v, const ov = @addWithOverflow(
        @as(i32, @bitCast(a)),
        @as(i32, @bitCast(b)),
    );
    return .{ @bitCast(v), ov };
}
