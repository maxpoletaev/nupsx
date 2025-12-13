const std = @import("std");

iter: *std.process.ArgIterator,

bios_path: []const u8,
exe_path: ?[]const u8,

step_execute: bool = false,
no_ui: bool = false,
disasm: bool = false,
breakpoint: u32 = 0,

const Args = @This();

pub fn parse(allocator: std.mem.Allocator) !Args {
    var iter = try std.process.argsWithAllocator(allocator);
    var args = std.mem.zeroInit(Args, .{ .iter = &iter });

    while (iter.next()) |arg| {
        if (std.mem.eql(u8, arg, "--step")) {
            args.step_execute = true;
        }
        if (std.mem.eql(u8, arg, "--no-ui")) {
            args.no_ui = true;
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
    }

    return args;
}

pub fn deinit(self: *Args) void {
    self.iter.deinit();
}
