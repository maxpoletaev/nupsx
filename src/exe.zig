const std = @import("std");

const Bus = @import("mem.zig").Bus;
const CPU = @import("cpu.zig").CPU;

const log = std.log.scoped(.exe);

pub const Error = error{
    InvalidExeFormat,
    FileReadError,
};

const Header = struct {
    magic: [8]u8,
    initial_pc: u32,
    initial_gp: u32,
    ram_addr: u32,
    exe_size: u32,
    initial_sp: u32,
    sp_offset: u32,
};

pub fn loadExeFromBuffer(buf: []const u8, cpu: *CPU, bus: *Bus) Error!void {
    if (buf.len < 0x800) {
        log.err("unexpected EXE buffer size: {}", .{buf.len});
        return Error.InvalidExeFormat;
    }

    const header: Header = .{
        .magic = buf[0x00..0x08].*,
        .initial_pc = std.mem.readInt(u32, buf[0x10..0x14], .little),
        .initial_gp = std.mem.readInt(u32, buf[0x14..0x18], .little),
        .ram_addr = std.mem.readInt(u32, buf[0x18..0x1c], .little),
        .exe_size = std.mem.readInt(u32, buf[0x1c..0x20], .little),
        .initial_sp = std.mem.readInt(u32, buf[0x30..0x34], .little),
        .sp_offset = std.mem.readInt(u32, buf[0x34..0x38], .little),
    };

    if (!std.mem.eql(u8, &header.magic, "PS-X EXE")) {
        log.err("invalid EXE magic number", .{});
        return Error.InvalidExeFormat;
    }

    for (0..header.exe_size) |i| {
        const addr = header.ram_addr + i;
        bus.write(u8, @intCast(addr), buf[0x800 + i]);
    }

    cpu.resetPC(header.initial_pc);
    cpu.gpr[28] = header.initial_gp;

    if (header.initial_sp != 0) {
        cpu.gpr[29] = header.initial_sp + header.sp_offset;
        cpu.gpr[30] = cpu.gpr[29];
    }

    log.info("exe loaded: pc=0x{x:0>8}", .{header.initial_pc});
}

pub fn loadExe(allocator: std.mem.Allocator, path: []const u8, cpu: *CPU, bus: *Bus) Error!void {
    const file = std.fs.cwd().openFile(path, .{}) catch |err| {
        log.err("failed to open EXE file: {}", .{err});
        return Error.FileReadError;
    };
    defer file.close();

    var reader_buf: [1024]u8 = undefined;
    var reader = file.reader(&reader_buf);

    const file_size = file.getEndPos() catch |err| {
        log.err("failed to get EXE file size: {}", .{err});
        return Error.FileReadError;
    };

    const buf = reader.interface.readAlloc(allocator, file_size) catch |err| {
        log.err("failed to read exe file: {}", .{err});
        return Error.FileReadError;
    };
    defer allocator.free(buf);

    try loadExeFromBuffer(buf, cpu, bus);
}
