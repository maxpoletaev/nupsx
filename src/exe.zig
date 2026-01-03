const std = @import("std");

const Bus = @import("mem.zig").Bus;
const CPU = @import("cpu.zig").CPU;

const Header = struct {
    magic: [8]u8,
    initial_pc: u32,
    initial_gp: u32,
    ram_addr: u32,
    exe_size: u32,
    initial_sp: u32,
    sp_offset: u32,

    pub fn read(buf: []u8) Header {
        return .{
            .magic = buf[0x00..0x08].*,
            .initial_pc = std.mem.readInt(u32, buf[0x10..0x14], .little),
            .initial_gp = std.mem.readInt(u32, buf[0x14..0x18], .little),
            .ram_addr = std.mem.readInt(u32, buf[0x18..0x1c], .little),
            .exe_size = std.mem.readInt(u32, buf[0x1c..0x20], .little),
            .initial_sp = std.mem.readInt(u32, buf[0x30..0x34], .little),
            .sp_offset = std.mem.readInt(u32, buf[0x34..0x38], .little),
        };
    }
};

pub fn loadExe(allocator: std.mem.Allocator, path: []const u8, cpu: *CPU, bus: *Bus) !void {
    const file = try std.fs.cwd().openFile(path, .{});
    defer file.close();

    const file_size = try file.getEndPos();
    if (file_size < 0x800) {
        return error.InvalidValue;
    }

    const buf = try allocator.alloc(u8, file_size);
    defer allocator.free(buf);

    const bytes_read = try file.readAll(buf);
    if (bytes_read != file_size) {
        return error.FileReadError;
    }

    const hdr = Header.read(buf);

    if (!std.mem.eql(u8, &hdr.magic, "PS-X EXE")) {
        return error.InvalidFileFormat;
    }

    // Load EXE payload to RAM
    for (0..hdr.exe_size) |i| {
        const addr = hdr.ram_addr + i;
        bus.write(u8, @intCast(addr), buf[0x800 + i]);
    }

    // Set initial register values
    cpu.resetPC(hdr.initial_pc);
    cpu.gpr[28] = hdr.initial_gp;
    if (hdr.initial_sp != 0) {
        cpu.gpr[29] = hdr.initial_sp + hdr.sp_offset;
        cpu.gpr[30] = cpu.gpr[29];
    }
}
