const std = @import("std");

const log = std.log.scoped(.cue);

pub const CueError = error{
    InvalidFormat,
    MissingDataTrack,
    UnknownMode,
    UnexpectedEof,
};

pub const TrackMode = enum {
    mode1_2352,
    mode2_2352,
    mode2_2336,
    audio,
};

pub const CueTrack = struct {
    number: u8,
    mode: TrackMode,
};

pub const CueFile = struct {
    path: []const u8,
    tracks: []CueTrack,
};

pub const CueSheet = struct {
    files: []CueFile,
    allocator: std.mem.Allocator,

    pub fn deinit(self: *@This()) void {
        for (self.files) |*file| {
            self.allocator.free(file.path);
            self.allocator.free(file.tracks);
        }

        self.allocator.free(self.files);
    }
};

pub fn parse(
    allocator: std.mem.Allocator,
    cue_path: []const u8,
) !CueSheet {
    const file = try std.fs.cwd().openFile(cue_path, .{});
    defer file.close();

    var rbuf: [4096]u8 = undefined;
    var reader = file.reader(&rbuf);
    const content = try reader.interface.allocRemaining(allocator, .unlimited);
    defer allocator.free(content);

    const cue_root = std.fs.path.dirname(cue_path) orelse ".";
    var parser = Parser.init(allocator, cue_root, content);
    defer parser.deinit();

    while (!parser.eof()) {
        parser.skipWhitespace();
        if (parser.eof()) break;

        const keyword = try parser.parseKeyword();

        switch (keyword) {
            .file => try parser.handleFile(),
            .track => try parser.handleTrack(),
            .index => parser.handleIndex(),
            .rem, .pregap, .flags, .postgap => parser.skipLine(),
            else => {},
        }
    }

    return parser.finish();
}

const Keyword = enum {
    file,
    track,
    index,
    rem,
    pregap,
    postgap,
    flags,
    unknown,
};

const Parser = struct {
    allocator: std.mem.Allocator,
    cue_root: []const u8,
    content: []const u8,
    pos: usize = 0,

    active_file: ?CueFile = null,
    pending_files: std.array_list.Aligned(CueFile, null) = .empty,
    pending_tracks: std.array_list.Aligned(CueTrack, null) = .empty,

    fn deinit(self: *@This()) void {
        if (self.active_file) |*file| {
            self.allocator.free(file.path);
            self.allocator.free(file.tracks);
        }
        for (self.pending_files.items) |*file| {
            self.allocator.free(file.path);
            self.allocator.free(file.tracks);
        }
        self.pending_files.deinit(self.allocator);
        self.pending_tracks.deinit(self.allocator);
    }

    fn init(
        allocator: std.mem.Allocator,
        cue_root: []const u8,
        content: []const u8,
    ) Parser {
        return .{
            .allocator = allocator,
            .cue_root = cue_root,
            .content = content,
        };
    }

    fn eof(self: *const @This()) bool {
        return self.pos >= self.content.len;
    }

    fn peek(self: *const @This()) ?u8 {
        if (self.eof()) return null;
        return self.content[self.pos];
    }

    fn advance(self: *@This()) ?u8 {
        if (self.eof()) return null;
        const c = self.content[self.pos];
        self.pos += 1;
        return c;
    }

    fn skipWhitespace(self: *@This()) void {
        while (self.peek()) |c| {
            if (!std.ascii.isWhitespace(c)) break;
            _ = self.advance();
        }
    }

    fn skipLine(self: *@This()) void {
        while (self.peek()) |c| {
            _ = self.advance();
            if (c == '\n') break;
        }
    }

    fn parseKeyword(self: *@This()) !Keyword {
        var buf: [32]u8 = undefined;
        var len: usize = 0;

        while (self.peek()) |c| {
            if (!std.ascii.isAlphabetic(c) and c != '/' and !std.ascii.isDigit(c)) break;
            if (len >= buf.len) return CueError.InvalidFormat;
            buf[len] = c;
            len += 1;
            _ = self.advance();
        }

        const word = buf[0..len];
        if (std.mem.eql(u8, word, "FILE")) return .file;
        if (std.mem.eql(u8, word, "TRACK")) return .track;
        if (std.mem.eql(u8, word, "INDEX")) return .index;
        if (std.mem.eql(u8, word, "REM")) return .rem;
        if (std.mem.eql(u8, word, "PREGAP")) return .pregap;
        if (std.mem.eql(u8, word, "POSTGAP")) return .postgap;
        if (std.mem.eql(u8, word, "FLAGS")) return .flags;

        log.warn("unknown keyword: {s}", .{word});

        return .unknown;
    }

    fn parseQuotedString(self: *@This()) ![]const u8 {
        self.skipWhitespace();

        if (self.peek() != '"') return CueError.InvalidFormat;
        _ = self.advance(); // Skip opening quote

        const start = self.pos;
        while (self.peek()) |c| {
            if (c == '"') break;
            _ = self.advance();
        }

        const end = self.pos;
        if (self.peek() != '"') return CueError.UnexpectedEof;
        _ = self.advance(); // Skip closing quote

        return self.content[start..end];
    }

    fn parseNumber(self: *@This()) !u8 {
        self.skipWhitespace();

        var num: u8 = 0;
        var found_digit = false;

        while (self.peek()) |c| {
            if (!std.ascii.isDigit(c)) break;
            found_digit = true;
            num = num * 10 + (c - '0');
            _ = self.advance();
        }

        if (!found_digit) return CueError.InvalidFormat;
        return num;
    }

    fn parseMode(self: *@This()) !TrackMode {
        self.skipWhitespace();

        var buf: [16]u8 = undefined;
        var len: usize = 0;

        while (self.peek()) |c| {
            if (!std.ascii.isAlphabetic(c) and c != '/' and !std.ascii.isDigit(c)) break;
            if (len >= buf.len) return CueError.InvalidFormat;
            buf[len] = c;
            len += 1;
            _ = self.advance();
        }

        const mode = buf[0..len];
        if (std.mem.eql(u8, mode, "MODE1/2352")) return .mode1_2352;
        if (std.mem.eql(u8, mode, "MODE2/2352")) return .mode2_2352;
        if (std.mem.eql(u8, mode, "MODE2/2336")) return .mode2_2336;
        if (std.mem.eql(u8, mode, "AUDIO")) return .audio;

        return CueError.UnknownMode;
    }

    fn handleFile(self: *@This()) !void {
        if (self.active_file) |*file| {
            file.tracks = try self.pending_tracks.toOwnedSlice(self.allocator);
            try self.pending_files.append(self.allocator, file.*);
            self.pending_tracks.clearRetainingCapacity();
            self.active_file = null;
        }

        const filename = try self.parseQuotedString();
        const full_path = try std.fs.path.join(self.allocator, &[_][]const u8{ self.cue_root, filename });
        errdefer self.allocator.free(full_path);

        log.debug("found FILE: {s}", .{filename});

        self.active_file = CueFile{ .path = full_path, .tracks = &[_]CueTrack{} };

        self.skipLine(); // Skip file type (BINARY, WAVE, etc.)
    }

    fn handleTrack(self: *@This()) !void {
        if (self.active_file == null) {
            log.err("TRACK found before FILE", .{});
            return CueError.InvalidFormat;
        }

        const track_num = try self.parseNumber();
        const mode = try self.parseMode();

        log.debug("found TRACK: number={d}, mode={s}", .{ track_num, @tagName(mode) });

        const track = CueTrack{
            .number = track_num,
            .mode = mode,
        };

        try self.pending_tracks.append(self.allocator, track);
    }

    fn handleIndex(self: *@This()) void {
        self.skipLine(); // Skip INDEX parsing for now
    }

    fn finish(self: *@This()) !CueSheet {
        if (self.active_file) |*file| {
            file.tracks = try self.pending_tracks.toOwnedSlice(self.allocator);
            try self.pending_files.append(self.allocator, file.*);
            self.active_file = null;
        }

        const files = try self.pending_files.toOwnedSlice(self.allocator);
        self.pending_files.clearRetainingCapacity();

        return CueSheet{
            .files = files,
            .allocator = self.allocator,
        };
    }
};
