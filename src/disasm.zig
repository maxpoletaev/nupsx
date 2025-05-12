const std = @import("std");
const cpu = @import("cpu.zig");

const Instr = cpu.Instr;
const Opcode = cpu.Opcode;
const CopOpCode = cpu.CopOpcode;
const RegName = cpu.RegName;
const BranchCond = cpu.BranchCond;
const SpecialOpcode = cpu.SpecialOpcode;

const Writer = std.ArrayList(u8).Writer;

pub const Disasm = struct {
    allocator: std.mem.Allocator,
    buf: std.ArrayList(u8),

    pub fn init(allocator: std.mem.Allocator) !*@This() {
        const self = try allocator.create(@This());
        self.* = .{
            .buf = .init(allocator),
            .allocator = allocator,
        };
        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.buf.deinit();
        self.allocator.destroy(self);
    }

    pub fn disassemble(self: *@This(), instr: Instr) []const u8 {
        self.buf.clearRetainingCapacity();
        self.disassembleToBuffer(instr, &self.buf) catch unreachable;
        return self.buf.items;
    }

    fn fallback(_: *@This(), comptime T: type, v: T, w: Writer, base: []const u8) !void {
        if (std.enums.tagName(T, v)) |tag| {
            try w.print("{s} ???", .{tag});
        } else {
            try w.writeAll(base);
        }
    }

    pub fn disassembleToBuffer(self: *@This(), instr: Instr, buf: *std.ArrayList(u8)) !void {
        const w = buf.writer();

        if (instr.code == 0) {
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
        const imm = instr.imm();
        const addr = instr.addr();
        const imm_s = @as(i32, @bitCast(instr.imm_s()));

        try switch (op) {
            .special => switch (special) {
                .syscall => w.writeAll("syscall"),
                .jr => w.print("jr ${s}", .{RegName[rs]}),
                .jalr => w.print("jalr ${s}, ${s}", .{ RegName[rs], RegName[rd] }),
                .addu => w.print("addu ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
                .and_ => w.print("and ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
                .xor => w.print("xor ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
                .sub => w.print("sub ${s}, ${s}, ${s}", .{ RegName[rd], RegName[rs], RegName[rt] }),
                .break_ => w.writeAll("break"),
                else => self.fallback(SpecialOpcode, special, w, "special - ???"),
            },
            .cop0 => switch (cop_opcode) {
                .rfe => w.writeAll("rfe"),
                .mfc => w.print("mfc0 ???", .{}),
                .mtc => w.print("mtc0 ???", .{}),
                else => self.fallback(CopOpCode, cop_opcode, w, "cop0 - ???"),
            },
            .bcondz => switch (bcond) {
                else => self.fallback(BranchCond, bcond, w, "bcondz - ???"),
            },
            .j => w.print("j {x}", .{addr}),
            .lui => w.print("lui ${s}, {x}", .{ RegName[rt], imm }),
            .lb => w.print("lb ${s}, {x}", .{ RegName[rs], imm_s }),
            .addi => w.print("addi ${s}, {x}", .{ RegName[rs], imm_s }),
            .beq => w.print("beq ${s}, ${s}, {x}", .{ RegName[rs], RegName[rt], imm_s }),
            .lw => w.print("lw ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
            .sb => w.print("sb ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
            .lbu => w.print("lbu ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
            .bgtz => w.print("bgtz ${s}, {x}", .{ RegName[rs], imm_s }),
            .addiu => w.print("addiu ${s}, ${s}, {x}", .{ RegName[rt], RegName[rs], imm }),
            .blez => w.print("blez ${s}, {x}", .{ RegName[rs], imm_s }),
            .bne => w.print("bne ${s}, ${s}, {x}", .{ RegName[rs], RegName[rt], imm_s }),
            .sw => w.print("sw ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
            .lwl => w.print("lwl ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
            .lwr => w.print("lwr ${s}, {x}(${s})", .{ RegName[rt], imm_s, RegName[rs] }),
            .jal => w.print("jal {x}", .{addr}),
            else => self.fallback(Opcode, op, w, "???"),
        };
    }
};
