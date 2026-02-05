const std = @import("std");
const cpu_mod = @import("cpu.zig");

const Instr = cpu_mod.RawInstr;
const Opcode = cpu_mod.Opcode;
const RegName = cpu_mod.RegName;
const CopOpCode = cpu_mod.CopOpcode;
const BranchCond = cpu_mod.BranchCond;
const SpecialOpcode = cpu_mod.SpecialOpcode;

const DisasmBuf = std.array_list.Aligned(u8, null);
const Writer = DisasmBuf.Writer;

pub const Disasm = struct {
    allocator: std.mem.Allocator,
    buf: std.Io.Writer.Allocating,

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .allocator = allocator,
            .buf = .init(allocator),
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.buf.deinit();
        self.allocator.destroy(self);
    }

    pub fn disassemble(self: *@This(), instr: Instr) []const u8 {
        self.buf.clearRetainingCapacity();
        toWriter(instr, &self.buf.writer) catch unreachable;
        return self.buf.written();
    }
};

fn fallback(comptime T: type, v: T, w: *std.Io.Writer, base: []const u8) !void {
    if (std.enums.tagName(T, v)) |tag| {
        try w.print("{s} ???", .{tag});
    } else {
        try w.writeAll(base);
    }
}

pub fn toWriter(instr: Instr, w: *std.Io.Writer) !void {
    if (instr.raw == 0) {
        try w.writeAll("nop");
        return;
    }

    const op = instr.opcode();
    const special = instr.special();
    const cop_opcode = instr.copOpcode();
    const bcond = instr.bcond();

    const rs = instr.rs();
    const rt = instr.rt();
    const rd = instr.rd();
    const shift = instr.shift();
    const imm = instr.imm();
    const addr = instr.addr();
    const imm_s = @as(i32, @bitCast(instr.imm_s()));

    try switch (op) {
        .special => switch (special) {
            .sll => w.print("sll ${s}, ${s}, {d}", .{ RegName[rd], RegName[rt], shift }),
            .srl => w.print("srl ${s}, ${s}, {d}", .{ RegName[rd], RegName[rt], shift }),
            .sra => w.print("sra ${s}, ${s}, {d}", .{ RegName[rd], RegName[rt], shift }),
            .sllv => w.print("sllv ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rt], RegName[rs] }),
            .srlv => w.print("srlv ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rt], RegName[rs] }),
            .srav => w.print("srav ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rt], RegName[rs] }),
            .jr => w.print("jr ${s}", .{RegName[rs]}),
            .jalr => w.print("jalr ${s}, ${s}", .{ RegName[rd], RegName[rs] }),
            .syscall => w.writeAll("syscall"),
            .break_ => w.writeAll("break"),
            .mfhi => w.print("mfhi ${s}", .{RegName[rd]}),
            .mthi => w.print("mthi ${s}", .{RegName[rs]}),
            .mflo => w.print("mflo ${s}", .{RegName[rd]}),
            .mtlo => w.print("mtlo ${s}", .{RegName[rs]}),
            .mult => w.print("mult ${s}, ${s}", .{ RegName[rs], RegName[rt] }),
            .multu => w.print("multu ${s}, ${s}", .{ RegName[rs], RegName[rt] }),
            .div => w.print("div ${s}, ${s}", .{ RegName[rs], RegName[rt] }),
            .divu => w.print("divu ${s}, ${s}", .{ RegName[rs], RegName[rt] }),
            .add => w.print("add ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .addu => w.print("addu ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .sub => w.print("sub ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .subu => w.print("subu ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .and_ => w.print("and ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .or_ => w.print("or ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .xor => w.print("xor ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .nor => w.print("nor ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .slt => w.print("slt ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            .sltu => w.print("sltu ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
            else => fallback(SpecialOpcode, special, w, "special - ???"),
        },
        .cop0 => switch (cop_opcode) {
            .mfc => w.print("mfc0 ${s}, ${d}", .{ RegName[rt], rd }),
            .mtc => w.print("mtc0 ${s}, ${d}", .{ RegName[rt], rd }),
            .rfe => w.writeAll("rfe"),
            else => fallback(CopOpCode, cop_opcode, w, "cop0 - ???"),
        },
        .cop1 => w.writeAll("cop1"),
        .cop2 => switch (cop_opcode) {
            .mfc => w.print("mfc2 ${s}, ${d}", .{ RegName[rt], rd }),
            .cfc => w.print("cfc2 ${s}, ${d}", .{ RegName[rt], rd }),
            .mtc => w.print("mtc2 ${s}, ${d}", .{ RegName[rt], rd }),
            .ctc => w.print("ctc2 ${s}, ${d}", .{ RegName[rt], rd }),
            else => w.print("cop2 {x}", .{instr.raw & 0x1ffffff}),
        },
        .cop3 => w.writeAll("cop3"),
        .bcondz => switch (bcond) {
            .bltz => w.print("bltz ${s}, {x}", .{ RegName[rs], imm_s }),
            .bgez => w.print("bgez ${s}, {x}", .{ RegName[rs], imm_s }),
            .bltzal => w.print("bltzal ${s}, {x}", .{ RegName[rs], imm_s }),
            .bgezal => w.print("bgezal ${s}, {x}", .{ RegName[rs], imm_s }),
            .bltzl => w.print("bltzl ${s}, {x}", .{ RegName[rs], imm_s }),
            .bgezl => w.print("bgezl ${s}, {x}", .{ RegName[rs], imm_s }),
            else => fallback(BranchCond, bcond, w, "bcondz - ???"),
        },
        .j => w.print("j {x}", .{addr}),
        .jal => w.print("jal {x}", .{addr}),
        .beq => w.print("beq ${s}, ${s}, {x}", .{ RegName[rs], RegName[rt], imm_s }),
        .bne => w.print("bne ${s}, ${s}, {x}", .{ RegName[rs], RegName[rt], imm_s }),
        .blez => w.print("blez ${s}, {x}", .{ RegName[rs], imm_s }),
        .bgtz => w.print("bgtz ${s}, {x}", .{ RegName[rs], imm_s }),
        .addi => w.print("addi ${s}, ${s}, {x}", .{ RegName[rt], RegName[rs], imm_s }),
        .addiu => w.print("addiu ${s}, ${s}, {x}", .{ RegName[rt], RegName[rs], imm_s }),
        .slti => w.print("slti ${s}, ${s}, {x}", .{ RegName[rt], RegName[rs], imm_s }),
        .sltiu => w.print("sltiu ${s}, ${s}, {x}", .{ RegName[rt], RegName[rs], imm }),
        .andi => w.print("andi ${s}, ${s}, {x}", .{ RegName[rt], RegName[rs], imm }),
        .ori => w.print("ori ${s}, ${s}, {x}", .{ RegName[rt], RegName[rs], imm }),
        .xori => w.print("xori ${s}, ${s}, {x}", .{ RegName[rt], RegName[rs], imm }),
        .lui => w.print("lui ${s}, {x}", .{ RegName[rt], imm }),
        .lb => w.print("lb ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .lh => w.print("lh ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .lwl => w.print("lwl ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .lw => w.print("lw ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .lbu => w.print("lbu ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .lhu => w.print("lhu ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .lwr => w.print("lwr ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .sb => w.print("sb ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .sh => w.print("sh ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .swl => w.print("swl ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .sw => w.print("sw ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .swr => w.print("swr ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
        .lwc2 => w.print("lwc2 ${d}, {x}(${s})", .{ rt, imm_s, RegName[rs] }),
        .swc2 => w.print("swc2 ${d}, {x}(${s})", .{ rt, imm_s, RegName[rs] }),
        else => fallback(Opcode, op, w, "???"),
    };
}
