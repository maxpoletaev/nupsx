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
    \\  --memcard <path>     Path to memory card image (default: ~/.nupsx/memcard.mcd)
    \\  --debug              Enable debug user interface
    \\  --disasm             Enable disassembly output
    \\  --breakpoint <addr>  Set a breakpoint at the specified address (hexadecimal)
    \\  --uncapped           Run the emulator without frame rate limiting
    \\  --no-shader          Disable the CRT/NTSC shader
    \\  --upscale <n>        Internal resolution scale, 1-4 (default: 1)
    \\  -h, --help           Show this help message
);

pub const Error = error{
    InvalidArgument,
};

pub const Args = struct {
    allocator: std.mem.Allocator,

    bios_path: []const u8,
    exe_path: []const u8,
    cd_image_path: []const u8,
    memcard_path: []const u8,

    debug: bool = false,
    disasm: bool = false,
    breakpoint: u32 = 0,
    uncapped: bool = false,

    no_shader: bool = false,
    upscale: u32 = 1,

    pub fn printHelp(io: std.Io) void {
        std.Io.File.stdout().writeStreamingAll(io, help_text) catch {};
    }

    pub fn hasExplicitArgs(allocator: std.mem.Allocator, proc_args: std.process.Args) bool {
        var args_iter = proc_args.iterateAllocator(allocator) catch @panic("OOM");
        defer args_iter.deinit();

        _ = args_iter.next(); // skip binary path
        return args_iter.next() != null;
    }

    pub fn parse(allocator: std.mem.Allocator, io: std.Io, proc_args: std.process.Args) Error!@This() {
        var args_iter = proc_args.iterateAllocator(allocator) catch @panic("OOM");
        defer args_iter.deinit();

        var args: Args = .{
            .allocator = allocator,
            .bios_path = &.{},
            .exe_path = &.{},
            .cd_image_path = &.{},
            .memcard_path = &.{},
            .debug = false,
            .disasm = false,
            .breakpoint = 0,
            .uncapped = false,
            .no_shader = false,
            .upscale = 1,
        };

        while (args_iter.next()) |arg| {
            if (std.mem.eql(u8, arg, "--help") or std.mem.eql(u8, arg, "-h")) {
                printHelp(io);
                std.process.exit(0);
            }

            if (std.mem.eql(u8, arg, "--debug")) {
                args.debug = true;
            }

            if (std.mem.eql(u8, arg, "--disasm")) {
                args.disasm = true;
            }

            if (std.mem.eql(u8, arg, "--breakpoint")) {
                const addr_str = args_iter.next() orelse {
                    log.err("missing address after --breakpoint", .{});
                    return Error.InvalidArgument;
                };
                args.breakpoint = std.fmt.parseUnsigned(u32, addr_str, 16) catch {
                    log.err("failed to parse breakpoint address: {s}", .{addr_str});
                    return Error.InvalidArgument;
                };
            }

            if (std.mem.eql(u8, arg, "--bios")) {
                const bios_path = args_iter.next() orelse {
                    log.err("missing path after --bios", .{});
                    return Error.InvalidArgument;
                };
                args.bios_path = allocator.dupe(u8, bios_path) catch @panic("OOM");
            }

            if (std.mem.eql(u8, arg, "--exe")) {
                const exe_path = args_iter.next() orelse {
                    log.err("missing path after --exe", .{});
                    return Error.InvalidArgument;
                };
                args.exe_path = allocator.dupe(u8, exe_path) catch @panic("OOM");
            }

            if (std.mem.eql(u8, arg, "--cdrom")) {
                const cd_image_path = args_iter.next() orelse {
                    log.err("missing path after --cdrom", .{});
                    return Error.InvalidArgument;
                };
                args.cd_image_path = allocator.dupe(u8, cd_image_path) catch @panic("OOM");
            }

            if (std.mem.eql(u8, arg, "--memcard")) {
                const memcard_path = args_iter.next() orelse {
                    log.err("missing path after --memcard", .{});
                    return Error.InvalidArgument;
                };
                args.memcard_path = allocator.dupe(u8, memcard_path) catch @panic("OOM");
            }

            if (std.mem.eql(u8, arg, "--uncapped")) {
                args.uncapped = true;
            }

            if (std.mem.eql(u8, arg, "--no-shader")) {
                args.no_shader = true;
            }

            if (std.mem.eql(u8, arg, "--upscale")) {
                const scale_str = args_iter.next() orelse {
                    log.err("missing value after --upscale", .{});
                    return Error.InvalidArgument;
                };
                args.upscale = std.fmt.parseUnsigned(u32, scale_str, 10) catch 0;
                if (args.upscale < 1 or args.upscale > 4) {
                    log.err("invalid --upscale value: {s} (expected 1-4)", .{scale_str});
                    return Error.InvalidArgument;
                }
            }
        }

        try args.validate();
        return args;
    }

    pub fn deinit(self: *@This()) void {
        self.allocator.free(self.bios_path);
        self.allocator.free(self.exe_path);
        self.allocator.free(self.cd_image_path);
        self.allocator.free(self.memcard_path);
    }

    fn validate(self: *Args) !void {
        if (self.bios_path.len == 0) {
            log.err("--bios path is required", .{});
            return Error.InvalidArgument;
        }
    }
};
