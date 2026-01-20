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

pub const RawInstr = struct {
    raw: u32,

    pub inline fn opcode(self: RawInstr) Opcode {
        return @enumFromInt(bits.field(self.raw, 26, u6));
    }

    pub inline fn isJumpOrBranch(self: RawInstr) bool {
        const op = self.opcode();
        return switch (op) {
            .j, .jal => true,
            .beq, .bne, .blez, .bgtz, .bcondz => true,
            .special => switch (self.special()) {
                .jr, .jalr, .syscall, .break_ => true,
                else => false,
            },
            else => false,
        };
    }

    pub inline fn copOpcode(self: RawInstr) CopOpcode {
        return @enumFromInt(bits.field(self.raw, 21, u5));
    }

    pub inline fn rs(self: RawInstr) u5 {
        return bits.field(self.raw, 21, u5);
    }

    pub inline fn rt(self: RawInstr) u5 {
        return bits.field(self.raw, 16, u5);
    }

    pub inline fn rd(self: RawInstr) u5 {
        return bits.field(self.raw, 11, u5);
    }

    pub inline fn shift(self: RawInstr) u5 {
        return bits.field(self.raw, 6, u5);
    }

    pub inline fn imm(self: RawInstr) u32 {
        return bits.field(self.raw, 0, u16);
    }

    pub inline fn imm_s(self: RawInstr) i32 {
        const v = bits.field(self.raw, 0, u16);
        return @as(i32, @as(i16, @bitCast(v)));
    }

    pub inline fn addr(self: RawInstr) u32 {
        return bits.field(self.raw, 0, u26);
    }

    pub inline fn special(self: RawInstr) SpecialOpcode {
        return @enumFromInt(bits.field(self.raw, 0, u6));
    }

    pub inline fn bcond(self: RawInstr) BranchCond {
        return @enumFromInt(bits.field(self.raw, 16, u5));
    }
};

const DecodedInstr = struct {
    addr: u32,
    imm: u32,
    imm_s: i32,
    opcode: Opcode,
    shift: u5,
    rs: u5,
    rt: u5,
    rd: u5,
};

const CachedInstr = struct {
    handler: ?*const fn (cpu: *CPU, instr: *const DecodedInstr) void,
    decoded: DecodedInstr,
};

/// Cached instruction block (64 instructions)
const CachedBlock = struct {
    instrs: [64]CachedInstr,
};

/// Bump allocator for instruction cache blocks
const BlockAllocator = struct {
    blocks: []CachedBlock,
    pos: usize = 0,

    pub fn init(allocator: std.mem.Allocator, block_count: usize) @This() {
        const blocks = allocator.alloc(CachedBlock, block_count) catch @panic("OOM");
        @memset(blocks, std.mem.zeroes(CachedBlock));
        return .{ .blocks = blocks };
    }

    pub fn deinit(self: @This(), allocator: std.mem.Allocator) void {
        allocator.free(self.blocks);
    }

    pub fn alloc(self: *@This()) ?*CachedBlock {
        if (self.pos >= self.blocks.len) {
            return null;
        }

        const block = &self.blocks[self.pos];
        self.pos += 1;
        return block;
    }

    pub fn reset(self: *@This()) void {
        @memset(self.blocks, std.mem.zeroes(CachedBlock));
        self.pos = 0;
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

    const cop0_reg_status: u8 = 12;
    const cop0_reg_cause: u8 = 13;
    const cop0_reg_epc: u8 = 14;

    r: [16]u32,
    depth: u8 = 0,

    pub inline fn getReg(self: *@This(), reg: u8) u32 {
        return self.r[reg];
    }

    pub inline fn setReg(self: *@This(), reg: u8, v: u32) void {
        self.r[reg] = v & reg_write_mask[reg];
    }

    pub inline fn status(self: *@This()) *Status {
        return @ptrCast(&self.r[cop0_reg_status]);
    }

    pub inline fn cause(self: *@This()) *Cause {
        return @ptrCast(&self.r[cop0_reg_cause]);
    }

    pub fn pushException(self: *@This()) void {
        const mode = self.r[cop0_reg_status] & 0x3f; // extract mode stack (6 bits)
        self.r[cop0_reg_status] &= ~@as(u32, 0x3f); // clear mode bits on the SR register
        self.r[cop0_reg_status] |= (mode << 2) & 0x3f; // shift mode stack 2 bits to the left
        self.depth, const ov = @addWithOverflow(self.depth, 1);
        if (ov != 0) log.warn("cop0 exception stack overflow", .{});
    }

    pub fn popException(self: *@This()) void {
        const mode = self.r[cop0_reg_status] & 0x3f; // extract mode stack (6 bits)
        self.r[cop0_reg_status] &= ~@as(u32, 0x0f); // clear mode bits on the SR register
        self.r[cop0_reg_status] |= (mode >> 2); // shift mode stack 2 bits to the right
        self.depth, const ov = @subWithOverflow(self.depth, 1);
        if (ov != 0) log.warn("cop0 exception stack underflow", .{});
    }

    inline fn checkInterrupt(self: *@This()) bool {
        const stat: Status = @bitCast(self.r[cop0_reg_status]);
        const caus: Cause = @bitCast(self.r[cop0_reg_cause]);
        return stat.curr_int_enable and (caus.interrupt_pending & stat.interrupt_mask) != 0;
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
    instr_addr: u32,
    cached_blocks: *[1 << 21]?*CachedBlock,
    block_allocator: BlockAllocator,
    delay_load: LoadSlot,
    delay_load_next: LoadSlot,
    branch_taken: bool,
    in_delay_slot: bool,
    in_exception: bool,
    in_branch: bool,
    cycles: u8,
    lo: u32,
    hi: u32,
    tty: ?*std.io.Writer = null,

    pub fn init(allocator: std.mem.Allocator, memory: *mem.Bus) !*@This() {
        const self = try allocator.create(@This());

        const block_allocator = BlockAllocator.init(allocator, (1 << 27) / @sizeOf(CachedBlock)); // 128 MB worth of blocks
        const cached_blocks = try allocator.create([1 << 21]?*CachedBlock);
        @memset(cached_blocks, null);

        self.* = .{
            .mem = memory,
            .gte = .init(),
            .allocator = allocator,
            .cached_blocks = cached_blocks,
            .block_allocator = block_allocator,
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
        self.block_allocator.deinit(self.allocator);
        self.allocator.free(self.cached_blocks);
        self.allocator.destroy(self);
    }

    pub fn reset(self: *@This()) void {
        self.pc = reset_addr;
        self.next_pc = self.pc + 4;
    }

    fn unhandled(self: *@This(), instr: *const DecodedInstr) void {
        std.debug.panic(
            "unhandled instruction at {x}: opcode={x}",
            .{ self.instr_addr, instr.opcode },
        );

        // _ = self;
        // log.err("unhandled instruction {x} at {x}", .{ code, self.instr_addr });
    }

    fn exception(self: *@This(), exc_code: Cop0.ExcCode) void {
        self.cop0.cause().exc_code = exc_code;
        self.cop0.pushException();

        if (self.in_delay_slot) {
            const branch_addr, _ = @subWithOverflow(self.instr_addr, 4);
            self.cop0.r[Cop0.cop0_reg_epc] = branch_addr;
            self.cop0.cause().epc_at_branch = true;
        } else {
            self.cop0.r[Cop0.cop0_reg_epc] = self.instr_addr;
            self.cop0.cause().epc_at_branch = false;
        }

        // Jump to exception handler (no delay slot)
        self.pc = if (self.cop0.status().boot_vectors != 0) 0xbfc00180 else 0x80000080;
        self.next_pc, _ = @addWithOverflow(self.pc, 4);
        self.in_exception = true;
        self.in_branch = true;
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
            const addr = self.pc +% @as(u32, @bitCast(offset << 2));
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
            self.cop0.r[Cop0.cop0_reg_cause] |= @as(u32, 1 << 10);
        } else {
            self.cop0.r[Cop0.cop0_reg_cause] &= ~@as(u32, 1 << 10);
        }
    }

    // =========================================================================
    // Instrunction decoding & caching
    // =========================================================================

    fn decode(instr: RawInstr) CachedInstr {
        const opcode = instr.opcode();

        const decoded = DecodedInstr{
            .opcode = opcode,
            .addr = instr.addr(),
            .imm = instr.imm(),
            .imm_s = instr.imm_s(),
            .shift = instr.shift(),
            .rs = instr.rs(),
            .rt = instr.rt(),
            .rd = instr.rd(),
        };

        const handler = switch (opcode) {
            .special => switch (instr.special()) {
                .sll => &CPU.sll,
                .or_ => &CPU.or_,
                .sltu => &CPU.sltu,
                .addu => &CPU.addu,
                .jr => &CPU.jr,
                .and_ => &CPU.and_,
                .add => &CPU.add,
                .jalr => &CPU.jalr,
                .subu => &CPU.subu,
                .sra => &CPU.sra,
                .div => &CPU.div,
                .mflo => &CPU.mflo,
                .mfhi => &CPU.mfhi,
                .srl => &CPU.srl,
                .divu => &CPU.divu,
                .slt => &CPU.slt,
                .syscall => &CPU.syscall,
                .mtlo => &CPU.mtlo,
                .mthi => &CPU.mthi,
                .sllv => &CPU.sllv,
                .nor => &CPU.nor,
                .srav => &CPU.srav,
                .srlv => &CPU.srlv,
                .multu => &CPU.multu,
                .xor => &CPU.xor,
                .break_ => &CPU.break_,
                .mult => &CPU.mult,
                .sub => &CPU.sub,
                else => &CPU.unhandled,
            },
            .cop0 => switch (instr.copOpcode()) {
                .mtc => &CPU.mtc0,
                .mfc => &CPU.mfc0,
                .rfe => &CPU.rfe,
                else => &CPU.unhandled,
            },
            .bcondz => switch (instr.bcond()) {
                .bltz => &CPU.bltz,
                .bgez => &CPU.bgez,
                .bltzal => &CPU.bltzal,
                .bgezal => &CPU.bgezal,
                else => switch ((instr.raw >> 16) & 1) {
                    0 => &CPU.bltz,
                    1 => &CPU.bgez,
                    else => unreachable,
                },
            },
            .lui => &CPU.lui,
            .ori => &CPU.ori,
            .sw => &CPU.sw,
            .addi => &CPU.addi,
            .addiu => &CPU.addiu,
            .j => &CPU.j,
            .bne => &CPU.bne,
            .lw => &CPU.lw,
            .sh => &CPU.sh,
            .jal => &CPU.jal,
            .andi => &CPU.andi,
            .sb => &CPU.sb,
            .lb => &CPU.lb,
            .beq => &CPU.beq,
            .bgtz => &CPU.bgtz,
            .blez => &CPU.blez,
            .lbu => &CPU.lbu,
            .slti => &CPU.slti,
            .sltiu => &CPU.sltiu,
            .lhu => &CPU.lhu,
            .lh => &CPU.lh,
            .xori => &CPU.xori,
            .lwl => &CPU.lwl,
            .lwr => &CPU.lwr,
            .swl => &CPU.swl,
            .swr => &CPU.swr,
            .cop1 => &CPU.cop1,
            .cop2 => switch (instr.copOpcode()) {
                .mfc => &CPU.mfc2,
                .mtc => &CPU.mtc2,
                .cfc => &CPU.cfc2,
                .ctc => &CPU.ctc2,
                else => &CPU.cop2,
            },
            .cop3 => &CPU.cop3,
            .lwc2 => &CPU.lwc2,
            .swc2 => &CPU.swc2,
            else => if (instr.raw <= 1) &CPU.nop else &CPU.unhandled,
        };

        return CachedInstr{
            .decoded = decoded,
            .handler = handler,
        };
    }

    fn prepareBlock(self: *@This(), addr: u32) *CachedBlock {
        // Allocate new block
        const block = self.block_allocator.alloc() orelse done: {
            self.block_allocator.pos = 0;
            @memset(self.cached_blocks, null);
            log.warn("interpreter cache is full, resetting", .{});
            break :done self.block_allocator.alloc().?;
        };

        // Decode entire instrunction block (64 instructions = 256 bytes)
        const base_addr = addr & 0xffff_ff00; // align to 256 bytes
        for (0..64) |i| {
            const instr_addr = base_addr + (@as(u32, @intCast(i)) * 4);
            const word = self.memRead(u32, instr_addr);
            block.instrs[i] = decode(RawInstr{ .raw = word });
        }

        return block;
    }

    pub inline fn invalidateBlock(self: *@This(), addr: u32) void {
        const masked = addr & 0x1fffffff;
        const block_i = masked >> 8;
        self.cached_blocks[block_i] = null;
    }

    inline fn fetch(self: *@This(), pc: u32) *CachedInstr {
        const addr = pc & 0x1fffffff; // drop segment selector (0x80000000/0xa0000000)

        const block_i = addr >> 8; // upper 21 bits
        const instr_i = (addr >> 2) & 63; // lower 6 bits

        if (self.cached_blocks[block_i]) |block| {
            @branchHint(.likely);
            return &block.instrs[instr_i];
        } else {
            const block = self.prepareBlock(addr);
            self.cached_blocks[block_i] = block;
            return &block.instrs[instr_i];
        }
    }

    inline fn tickInternal(self: *@This()) bool {
        self.gpr[0] = 0;

        // Handle TTY output
        if ((self.pc == 0xa0 and self.gpr[9] == 0x3c) or
            (self.pc == 0xb0 and self.gpr[9] == 0x3d))
        {
            @branchHint(.unlikely);
            if (self.tty) |tty| {
                const char: u8 = @truncate(self.gpr[4]);
                tty.writeByte(char) catch @panic("TTY write");
                if (char == '\n') tty.flush() catch @panic("TTY flush");
            }
        }

        const instr = self.fetch(self.pc);
        self.instr_addr = self.pc;

        self.pc = self.next_pc;
        self.next_pc = self.pc +% 4;

        // Each instruction is 2 cycles on average
        self.cycles = 2;

        // If the last instruction was a branch, then we are in delay slot
        self.in_delay_slot = self.in_branch;
        self.in_branch = false;

        // Check for interrupts (but GTE has priority)
        if (instr.decoded.opcode != .cop2 and self.cop0.checkInterrupt()) {
            self.exception(.interrupt);
            return false;
        }

        // Execute the instruction
        std.debug.assert(instr.handler != null);
        instr.handler.?(self, &instr.decoded);

        // Handle delayed loads
        self.gpr[self.delay_load.r] = self.delay_load.v;
        self.delay_load = self.delay_load_next;
        self.delay_load_next.clear();

        if (self.in_branch) return false;

        return true;
    }

    pub fn tickOnce(self: *@This()) void {
        _ = self.tickInternal();
    }

    pub fn tick(self: *@This()) u8 {
        var total_cycles: u8 = 0;

        // During the execution:
        //  * pc - incremented after fetch but before execute, so it points to the NEXT instruction to execute
        //  * next_pc - address that will be loaded into pc after the current instruction (for branches/jumps)
        //  * instr_addr - points to the current instruction being executed

        for (0..64) |_| {
            const continue_exec = self.tickInternal();
            total_cycles += self.cycles;
            if (!continue_exec) break;
        }

        return total_cycles;
    }

    inline fn memRead(self: *@This(), comptime T: type, addr: u32) T {
        return self.mem.read(T, addr);
    }

    inline fn memWrite(self: *@This(), comptime T: type, addr: u32, v: T) void {
        self.mem.write(T, addr, v);
    }

    // =========================================================================
    // Coprocessor instructions
    // =========================================================================

    fn lwc2(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        const v = self.memRead(u32, addr);
        self.gte.writeReg(instr.rt, v);
    }

    fn swc2(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_store);
            return;
        }

        const v = self.gte.readReg(instr.rt);
        self.memWrite(u32, addr, v);
    }

    fn mfc2(self: *@This(), instr: *const DecodedInstr) void {
        const gte_reg: u8 = @intCast(instr.rd);
        const v = self.gte.readReg(gte_reg);
        self.writeGpr(instr.rt, v);
    }

    fn mtc2(self: *@This(), instr: *const DecodedInstr) void {
        const gte_reg: u8 = @intCast(instr.rd);
        self.gte.writeReg(gte_reg, self.gpr[instr.rt]);
    }

    fn cfc2(self: *@This(), instr: *const DecodedInstr) void {
        const gte_reg: u8 = 32 + @as(u8, instr.rd);
        const v = self.gte.readReg(gte_reg);
        self.writeGpr(instr.rt, v);
    }

    fn ctc2(self: *@This(), instr: *const DecodedInstr) void {
        const gte_reg: u8 = 32 + @as(u8, instr.rd);
        self.gte.writeReg(gte_reg, self.gpr[instr.rt]);
    }

    fn cop1(self: *@This(), _: *const DecodedInstr) void {
        self.exception(.cop_unusable);
    }

    fn cop2(self: *@This(), _: *const DecodedInstr) void {
        const raw_cmd = self.memRead(u32, self.instr_addr);
        self.gte.exec(raw_cmd);
    }

    fn cop3(self: *@This(), _: *const DecodedInstr) void {
        self.exception(.cop_unusable);
    }

    // =========================================================================
    // Instructions
    // =========================================================================

    fn nop(_: *@This(), _: *const DecodedInstr) void {
        // do nothing
    }

    fn lui(self: *@This(), instr: *const DecodedInstr) void {
        self.writeGpr(instr.rt, instr.imm << 16);
    }

    fn ori(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs] | instr.imm;
        self.writeGpr(instr.rt, v);
    }

    fn andi(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs] & instr.imm;
        self.writeGpr(instr.rt, v);
    }

    fn sw(self: *@This(), instr: *const DecodedInstr) void {
        if (self.cop0.status().isolate_cache) {
            // log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_store);
            return;
        }

        self.memWrite(u32, addr, self.gpr[instr.rt]);
    }

    fn addiu(self: *@This(), instr: *const DecodedInstr) void {
        const sum, _ = addSignedToUnsigned(self.gpr[instr.rs], instr.imm_s);
        self.writeGpr(instr.rt, sum);
    }

    fn addi(self: *@This(), instr: *const DecodedInstr) void {
        const sum, const ov = addSignedToUnsigned(self.gpr[instr.rs], instr.imm_s);

        if (ov != 0) {
            self.exception(.overflow);
            return;
        }

        self.writeGpr(instr.rt, sum);
    }

    fn add(self: *@This(), instr: *const DecodedInstr) void {
        const sum, const ov = addAsSigned(
            self.gpr[instr.rs],
            self.gpr[instr.rt],
        );

        if (ov != 0) {
            self.exception(.overflow);
            return;
        }

        self.writeGpr(instr.rd, sum);
    }

    fn addu(self: *@This(), instr: *const DecodedInstr) void {
        const sum, _ = @addWithOverflow(
            self.gpr[instr.rs],
            self.gpr[instr.rt],
        );
        self.writeGpr(instr.rd, sum);
    }

    fn j(self: *@This(), instr: *const DecodedInstr) void {
        self.jump((self.pc & 0xF0000000) | (instr.addr << 2));
    }

    fn bne(self: *@This(), instr: *const DecodedInstr) void {
        self.branchIf(
            self.gpr[instr.rs] != self.gpr[instr.rt],
            instr.imm_s,
        );
    }

    fn beq(self: *@This(), instr: *const DecodedInstr) void {
        self.branchIf(
            self.gpr[instr.rs] == self.gpr[instr.rt],
            instr.imm_s,
        );
    }

    fn lw(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);

        if (addr % 4 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        self.writeGprDelay(instr.rt, self.memRead(u32, addr));
    }

    fn lb(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);
        const v = signExtend(u8, self.memRead(u8, addr));

        self.writeGprDelay(instr.rt, v);
    }

    fn lbu(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);
        const v = self.memRead(u8, addr);

        self.writeGprDelay(instr.rt, v);
    }

    fn or_(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs] | self.gpr[instr.rt];
        self.writeGpr(instr.rd, v);
    }

    fn and_(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs] & self.gpr[instr.rt];
        self.writeGpr(instr.rd, v);
    }

    fn sll(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rt] << instr.shift;
        self.writeGpr(instr.rd, v);
    }

    fn sltu(self: *@This(), instr: *const DecodedInstr) void {
        const less = self.gpr[instr.rs] < self.gpr[instr.rt];
        self.writeGpr(instr.rd, @intFromBool(less));
    }

    fn sh(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_store);
            return;
        }

        const v = self.gpr[instr.rt];
        self.memWrite(u16, addr, @truncate(v));
    }

    fn sb(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);
        const v = self.gpr[instr.rt];

        self.memWrite(u8, addr, @truncate(v));
    }

    fn jal(self: *@This(), instr: *const DecodedInstr) void {
        const ra = @intFromEnum(Reg.ra);
        self.writeGpr(ra, self.next_pc);
        self.jump((self.pc & 0xF0000000) | (instr.addr << 2));
    }

    fn jr(self: *@This(), instr: *const DecodedInstr) void {
        self.jump(self.gpr[instr.rs]);
    }

    fn bgtz(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs];
        self.branchIf(
            @as(i32, @bitCast(v)) > 0,
            instr.imm_s,
        );
    }

    fn blez(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs];
        self.branchIf(
            @as(i32, @bitCast(v)) <= 0,
            instr.imm_s,
        );
    }

    fn jalr(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs];
        self.writeGpr(instr.rd, self.next_pc);
        self.jump(v);
    }

    fn bltz(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs];
        self.branchIf(
            @as(i32, @bitCast(v)) < 0,
            instr.imm_s,
        );
    }

    fn bgez(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs];
        self.branchIf(
            @as(i32, @bitCast(v)) >= 0,
            instr.imm_s,
        );
    }

    fn slti(self: *@This(), instr: *const DecodedInstr) void {
        const rs = @as(i32, @bitCast(self.gpr[instr.rs]));
        const imm = @as(i32, @bitCast(instr.imm_s));
        const less = @intFromBool(rs < imm);
        self.writeGpr(instr.rt, less);
    }

    fn subu(self: *@This(), instr: *const DecodedInstr) void {
        const v, _ = @subWithOverflow(self.gpr[instr.rs], self.gpr[instr.rt]);
        self.writeGpr(instr.rd, v);
    }

    fn sra(self: *@This(), instr: *const DecodedInstr) void {
        const v = @as(i32, @bitCast(self.gpr[instr.rt])) >> instr.shift;
        self.writeGpr(instr.rd, @as(u32, @bitCast(v)));
    }

    fn div(self: *@This(), instr: *const DecodedInstr) void {
        const num = @as(i32, @bitCast(self.gpr[instr.rs]));
        const denom = @as(i32, @bitCast(self.gpr[instr.rt]));

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

    fn divu(self: *@This(), instr: *const DecodedInstr) void {
        const num = self.gpr[instr.rs];
        const denom = self.gpr[instr.rt];

        if (denom == 0) {
            @branchHint(.unlikely);
            self.lo = 0xffffffff;
            self.hi = num;
            return;
        }

        self.lo = @divTrunc(num, denom);
        self.hi = @mod(num, denom);
    }

    fn mflo(self: *@This(), instr: *const DecodedInstr) void {
        self.writeGpr(instr.rd, self.lo);
    }

    fn mfhi(self: *@This(), instr: *const DecodedInstr) void {
        self.writeGpr(instr.rd, self.hi);
    }

    fn srl(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rt] >> instr.shift;
        self.writeGpr(instr.rd, v);
    }

    fn sltiu(self: *@This(), instr: *const DecodedInstr) void {
        const less = self.gpr[instr.rs] < @as(u32, @bitCast(instr.imm_s));
        self.writeGpr(instr.rt, @intFromBool(less));
    }

    fn slt(self: *@This(), instr: *const DecodedInstr) void {
        const a = @as(i32, @bitCast(self.gpr[instr.rs]));
        const b = @as(i32, @bitCast(self.gpr[instr.rt]));
        self.writeGpr(instr.rd, @intFromBool(a < b));
    }

    fn syscall(self: *@This(), _: *const DecodedInstr) void {
        self.exception(.syscall);
    }

    fn mtlo(self: *@This(), instr: *const DecodedInstr) void {
        self.lo = self.gpr[instr.rs];
    }

    fn mthi(self: *@This(), instr: *const DecodedInstr) void {
        self.hi = self.gpr[instr.rs];
    }

    fn mfc0(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.cop0.getReg(instr.rd);
        self.writeGprDelay(instr.rt, v);
    }

    fn mtc0(self: *@This(), instr: *const DecodedInstr) void {
        self.cop0.setReg(instr.rd, self.gpr[instr.rt]);
    }

    fn rfe(self: *@This(), _: *const DecodedInstr) void {
        self.cop0.popException();

        if (self.cop0.depth == 0) {
            self.in_exception = false;
        }
    }

    fn lhu(self: *@This(), instr: *const DecodedInstr) void {
        if (self.cop0.status().isolate_cache) {
            // log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        const v = self.memRead(u16, addr);
        self.writeGprDelay(instr.rt, v);
    }

    fn sllv(self: *@This(), instr: *const DecodedInstr) void {
        const shift = @as(u5, @truncate(self.gpr[instr.rs] & 0x1f));
        const v = self.gpr[instr.rt] << shift;
        self.writeGpr(instr.rd, v);
    }

    fn lh(self: *@This(), instr: *const DecodedInstr) void {
        if (self.cop0.status().isolate_cache) {
            // log.debug("ignoring write while cache is isolated", .{});
            return;
        }

        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);

        if (addr % 2 != 0) {
            @branchHint(.unlikely);
            self.exception(.addr_load);
            return;
        }

        const v = signExtend(u16, self.memRead(u16, addr));
        self.writeGprDelay(instr.rt, v);
    }

    fn nor(self: *@This(), instr: *const DecodedInstr) void {
        const v = ~(self.gpr[instr.rs] | self.gpr[instr.rt]);
        self.writeGpr(instr.rd, v);
    }

    fn srav(self: *@This(), instr: *const DecodedInstr) void {
        const shift = @as(u5, @truncate(self.gpr[instr.rs] & 0x1f));
        const v = @as(i32, @bitCast(self.gpr[instr.rt])) >> shift;
        self.writeGpr(instr.rd, @as(u32, @bitCast(v)));
    }

    fn srlv(self: *@This(), instr: *const DecodedInstr) void {
        const shift = @as(u5, @truncate(self.gpr[instr.rs] & 0x1f));
        const v = self.gpr[instr.rt] >> shift;
        self.writeGpr(instr.rd, v);
    }

    fn multu(self: *@This(), instr: *const DecodedInstr) void {
        const a = @as(u64, self.gpr[instr.rs]);
        const b = @as(u64, self.gpr[instr.rt]);
        const v, _ = @mulWithOverflow(a, b);

        self.lo = @truncate(v);
        self.hi = @truncate(v >> 32);
    }

    fn xor(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs] ^ self.gpr[instr.rt];
        self.writeGpr(instr.rd, v);
    }

    fn xori(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs] ^ instr.imm;
        self.writeGpr(instr.rt, v);
    }

    fn break_(self: *@This(), _: *const DecodedInstr) void {
        self.exception(.breakpoint);
    }

    fn mult(self: *@This(), instr: *const DecodedInstr) void {
        const a = @as(i64, @as(i32, @bitCast(self.gpr[instr.rs])));
        const b = @as(i64, @as(i32, @bitCast(self.gpr[instr.rt])));
        const m, _ = @mulWithOverflow(a, b);
        const v = @as(u64, @bitCast(m));

        self.lo = @truncate(v);
        self.hi = @truncate(v >> 32);
    }

    fn sub(self: *@This(), instr: *const DecodedInstr) void {
        const a = @as(i32, @bitCast(self.gpr[instr.rs]));
        const b = @as(i32, @bitCast(self.gpr[instr.rt]));

        const v, const ov = @subWithOverflow(a, b);
        if (ov != 0) {
            self.exception(.overflow);
            return;
        }

        self.writeGpr(instr.rd, @bitCast(v));
    }

    fn bltzal(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs];
        self.writeGpr(@intFromEnum(Reg.ra), self.next_pc);
        self.branchIf(@as(i32, @bitCast(v)) < 0, instr.imm_s);
    }

    fn bgezal(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs];
        self.writeGpr(@intFromEnum(Reg.ra), self.next_pc);
        self.branchIf(@as(i32, @bitCast(v)) >= 0, instr.imm_s);
    }

    fn bltzl(self: *@This(), instr: *const DecodedInstr) void {
        const v = self.gpr[instr.rs];

        self.branchIf(
            @as(i32, @bitCast(v)) < 0,
            instr.imm_s,
        );

        if (!self.branch_taken) {
            self.pc = self.next_pc;
            self.next_pc += 4;
        }
    }

    fn lwl(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);
        const aligned_addr = addr & ~@as(u32, 0x3);

        const load_v = self.memRead(u32, aligned_addr);
        var curr_v = self.gpr[instr.rt];

        if (self.delay_load.r == instr.rt) {
            curr_v = self.delay_load.v;
        }

        const v = switch (addr & 0x3) {
            0 => (curr_v & 0x00ffffff) | (load_v << 24),
            1 => (curr_v & 0x0000ffff) | (load_v << 16),
            2 => (curr_v & 0x000000ff) | (load_v << 8),
            3 => (curr_v & 0x00000000) | (load_v << 0),
            else => undefined,
        };

        self.writeGprDelay(instr.rt, v);
    }

    fn lwr(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);
        const aligned_addr = addr & ~@as(u32, 0x3);

        const load_v = self.memRead(u32, aligned_addr);
        var curr_v = self.gpr[instr.rt];

        if (self.delay_load.r == instr.rt) {
            curr_v = self.delay_load.v;
        }

        const v = switch (addr & 0x3) {
            0 => (curr_v & 0x00000000) | (load_v >> 0),
            1 => (curr_v & 0xff000000) | (load_v >> 8),
            2 => (curr_v & 0xffff0000) | (load_v >> 16),
            3 => (curr_v & 0xffffff00) | (load_v >> 24),
            else => undefined,
        };

        self.writeGprDelay(instr.rt, v);
    }

    fn swl(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);
        const aligned_addr = addr & ~@as(u32, 0x3);

        const curr_v = self.memRead(u32, aligned_addr);
        const store_v = self.gpr[instr.rt];

        const v = switch (addr & 0x3) {
            0 => (curr_v & 0xffffff00) | (store_v >> 24),
            1 => (curr_v & 0xffff0000) | (store_v >> 16),
            2 => (curr_v & 0xff000000) | (store_v >> 8),
            3 => (curr_v & 0x00000000) | (store_v >> 0),
            else => unreachable,
        };

        self.memWrite(u32, aligned_addr, v);
    }

    fn swr(self: *@This(), instr: *const DecodedInstr) void {
        const base = self.gpr[instr.rs];
        const addr, _ = addSignedToUnsigned(base, instr.imm_s);
        const aligned_addr = addr & ~@as(u32, 0x3);

        const curr_v = self.memRead(u32, aligned_addr);
        const store_v = self.gpr[instr.rt];

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

inline fn addSignedToUnsigned(a: u32, b: i32) struct { u32, u1 } {
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
