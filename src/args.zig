const std = @import("std");

const log = std.log.scoped(.args);

const help_text = (
    \\nuPSX - A PlayStation emulator written in Zig
    \\
    \\Usage: nupsx --bios <path> [options]
    \\
    \\Options:
    \\  --bios <path>        Path to BIOS file (required)
    \\  --exe <path>         Path to executable file to run
    \\  --cdrom <path>       Path to CD-ROM image file (.cue)
    \\  --debug              Enable debug user interface
    \\  --disasm             Enable disassembly output
    \\  --breakpoint <addr>  Set a breakpoint at the specified address (hexadecimal)
    \\  --uncapped           Run the emulator without frame rate limiting
    \\  -h, --help           Show this help message
);

pub const Args = struct {
    iter: *std.process.ArgIterator,

    bios_path: []const u8,
    exe_path: ?[]const u8,
    cd_image_path: ?[]const u8,

    debug: bool = false,
    disasm: bool = false,
    breakpoint: u32 = 0,
    uncapped: bool = false,

    pub fn printHelp() void {
        std.fs.File.stdout().writeAll(help_text) catch {};
    }

    pub fn parse(allocator: std.mem.Allocator) !@This() {
        var iter = try std.process.argsWithAllocator(allocator);
        var args = std.mem.zeroInit(Args, .{ .iter = &iter });

        while (iter.next()) |arg| {
            if (std.mem.eql(u8, arg, "--help") or std.mem.eql(u8, arg, "-h")) {
                printHelp();
                std.process.exit(0);
            }

            if (std.mem.eql(u8, arg, "--debug")) {
                args.debug = true;
            }
            if (std.mem.eql(u8, arg, "--disasm")) {
                args.disasm = true;
            }
            if (std.mem.eql(u8, arg, "--breakpoint")) {
                const addr_str = iter.next() orelse return error.InvalidArgument;
                args.breakpoint = try std.fmt.parseUnsigned(u32, addr_str, 16);
            }
            if (std.mem.eql(u8, arg, "--bios")) {
                args.bios_path = iter.next() orelse return error.InvalidArgument;
            }
            if (std.mem.eql(u8, arg, "--exe")) {
                args.exe_path = iter.next() orelse return error.InvalidArgument;
            }
            if (std.mem.eql(u8, arg, "--cdrom")) {
                args.cd_image_path = iter.next() orelse return error.InvalidArgument;
            }
            if (std.mem.eql(u8, arg, "--uncapped")) {
                args.uncapped = true;
            }
        }

        try args.validate();
        return args;
    }

    pub fn deinit(self: *Args) void {
        self.iter.deinit();
    }

    fn validate(self: *Args) !void {
        if (self.bios_path.len == 0) {
            log.err("--bios path is required", .{});
            return error.InvalidArgument;
        }
    }
};
