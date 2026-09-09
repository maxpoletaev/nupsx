const std = @import("std");
const builtin = @import("builtin");

const log = std.log.scoped(.paths);

const app_dir_name = ".nupsx";
const config_file_name = "nupsx.conf";
const memcard_file_name = "memcard.mcd";

pub var home_path: []const u8 = undefined;
pub var app_root: []const u8 = undefined;
pub var config_path: []const u8 = undefined;
pub var default_memcard_path: []const u8 = undefined;

var context: ?struct {
    allocator: std.mem.Allocator,
    root_dir: std.Io.Dir,
} = null;

pub fn init(allocator: std.mem.Allocator, io: std.Io, environ: *const std.process.Environ.Map) void {
    std.debug.assert(context == null);

    const home_env = homeDirFromEnv(environ) orelse ".";
    home_path = allocator.dupe(u8, home_env) catch @panic("OOM");

    app_root = join(allocator, home_path, app_dir_name);
    config_path = join(allocator, app_root, config_file_name);
    default_memcard_path = join(allocator, app_root, memcard_file_name);

    const root_dir = openOrCreateDir(io, app_root) catch |err| dir: {
        log.warn("failed to open {s}: {}, falling back to the working directory", .{ app_root, err });
        break :dir std.Io.Dir.cwd();
    };

    context = .{
        .allocator = allocator,
        .root_dir = root_dir,
    };

    log.info("using directory: {s}", .{app_root});
}

pub fn deinit(io: std.Io) void {
    const ctx = context.?;
    ctx.root_dir.close(io);
    ctx.allocator.free(default_memcard_path);
    ctx.allocator.free(config_path);
    ctx.allocator.free(app_root);
    ctx.allocator.free(home_path);
    context = null;
}

fn join(allocator: std.mem.Allocator, dir: []const u8, name: []const u8) []const u8 {
    return std.fs.path.join(allocator, &.{ dir, name }) catch @panic("OOM");
}

fn homeDirFromEnv(environ: *const std.process.Environ.Map) ?[]const u8 {
    const key = if (builtin.os.tag == .windows) "USERPROFILE" else "HOME";
    const home = environ.get(key) orelse return null;
    return if (home.len != 0) home else null;
}

fn openOrCreateDir(io: std.Io, path: []const u8) !std.Io.Dir {
    std.Io.Dir.createDirAbsolute(io, path, .default_dir) catch |err| switch (err) {
        error.PathAlreadyExists => {},
        else => return err,
    };
    return std.Io.Dir.openDirAbsolute(io, path, .{});
}
