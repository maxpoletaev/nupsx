const std = @import("std");

const log = std.log.scoped(.cue);

pub const CueError = error{
    InvalidFormat,
    UnknownMode,
    UnknownFileType,
    UnexpectedEof,
};

pub const Position = struct {
    minute: u8,
    second: u8,
    frame: u8,

    pub fn toSectors(self: Position) u32 {
        return @as(u32, self.minute) * 60 * 75 +
            @as(u32, self.second) * 75 +
            @as(u32, self.frame);
    }
};

pub const TrackMode = enum {
    mode1_2352,
    mode2_2352,
    mode2_2336,
    audio,
};

pub const FileType = enum {
    binary,
    wave,
    mp3,
    aiff,
};

pub const IndexNode = struct {
    number: u8,
    position: Position,
};

pub const TrackNode = struct {
    number: u8,
    mode: TrackMode,
    indices: []IndexNode,
};

pub const FileNode = struct {
    filename: []const u8,
    file_type: FileType,
    tracks: []TrackNode,
};

pub const CueSheet = struct {
    files: []FileNode,
};

pub fn parse(allocator: std.mem.Allocator, content: []const u8) !CueSheet {
    var parser = Parser.init(allocator, content);
    defer parser.deinit();

    parser.skipBom();

    while (!parser.eof()) {
        parser.skipWhitespace();
        if (parser.eof()) break;

        const keyword = try parser.parseKeyword();

        switch (keyword) {
            .file => try parser.handleFile(),
            .track => try parser.handleTrack(),
            .index => try parser.handleIndex(),
            .rem, .pregap, .flags, .postgap => parser.skipLine(),
            else => {},
        }
    }

    return parser.finish();
}

pub fn parseFile(allocator: std.mem.Allocator, cue_path: []const u8) !CueSheet {
    const file = try std.fs.cwd().openFile(cue_path, .{});
    defer file.close();

    var buf: [4096]u8 = undefined;
    var reader = file.reader(&buf);
    const content = try reader.interface.allocRemaining(allocator, .unlimited);
    defer allocator.free(content);

    return parse(allocator, content);
}

pub fn free(allocator: std.mem.Allocator, sheet: *CueSheet) void {
    for (sheet.files) |*file| {
        allocator.free(file.filename);
        for (file.tracks) |*track| {
            allocator.free(track.indices);
        }
        allocator.free(file.tracks);
    }
    allocator.free(sheet.files);
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
    content: []const u8,
    pos: usize = 0,

    files: std.ArrayListUnmanaged(FileNode) = .{},
    tracks: std.ArrayListUnmanaged(TrackNode) = .{},
    indices: std.ArrayListUnmanaged(IndexNode) = .{},

    tmp: struct {
        file_name: ?[]const u8 = null,
        file_type: ?FileType = null,
        track_number: ?u8 = null,
        track_mode: ?TrackMode = null,
    } = .{},

    fn deinit(self: *@This()) void {
        for (self.files.items) |*file| {
            self.allocator.free(file.filename);
            for (file.tracks) |*track| {
                self.allocator.free(track.indices);
            }
        }

        for (self.tracks.items) |*track| {
            self.allocator.free(track.indices);
        }

        self.indices.deinit(self.allocator);
    }

    fn init(allocator: std.mem.Allocator, content: []const u8) Parser {
        return .{
            .allocator = allocator,
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

    fn skipBom(self: *@This()) void {
        const bom: [3]u8 = .{ 0xEF, 0xBB, 0xBF };
        for (bom) |b| {
            if (self.peek() == b) {
                _ = self.advance();
            } else {
                break;
            }
        }
    }

    fn skipLine(self: *@This()) void {
        while (self.peek()) |c| {
            _ = self.advance();
            if (c == '\n') break;
        }
    }

    fn parseWord(self: *@This()) []const u8 {
        const start = self.pos;
        while (self.peek()) |c| {
            if (!std.ascii.isAlphabetic(c) and c != '/' and !std.ascii.isDigit(c)) break;
            _ = self.advance();
        }
        return self.content[start..self.pos];
    }

    fn parseKeyword(self: *@This()) !Keyword {
        const word = self.parseWord();
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

        const mode = self.parseWord();
        if (std.mem.eql(u8, mode, "MODE1/2352")) return .mode1_2352;
        if (std.mem.eql(u8, mode, "MODE2/2352")) return .mode2_2352;
        if (std.mem.eql(u8, mode, "MODE2/2336")) return .mode2_2336;
        if (std.mem.eql(u8, mode, "AUDIO")) return .audio;

        return CueError.UnknownMode;
    }

    fn parseFileType(self: *@This()) !FileType {
        self.skipWhitespace();

        const file_type = self.parseWord();
        if (std.mem.eql(u8, file_type, "BINARY")) return .binary;
        if (std.mem.eql(u8, file_type, "WAVE")) return .wave;
        if (std.mem.eql(u8, file_type, "MP3")) return .mp3;
        if (std.mem.eql(u8, file_type, "AIFF")) return .aiff;

        return CueError.UnknownFileType;
    }

    fn parsePosition(self: *@This()) !Position {
        self.skipWhitespace();

        const start = self.pos;
        while (self.peek()) |c| {
            if (std.ascii.isWhitespace(c)) break;
            _ = self.advance();
        }

        const str = self.content[start..self.pos];
        var iter = std.mem.splitScalar(u8, str, ':');

        const minute = try std.fmt.parseInt(u8, iter.next() orelse return CueError.InvalidFormat, 10);
        const second = try std.fmt.parseInt(u8, iter.next() orelse return CueError.InvalidFormat, 10);
        const frame = try std.fmt.parseInt(u8, iter.next() orelse return CueError.InvalidFormat, 10);

        if (iter.next() != null) return CueError.InvalidFormat; // Too many parts

        return Position{
            .minute = minute,
            .second = second,
            .frame = frame,
        };
    }

    fn handleFile(self: *@This()) !void {
        self.finalizeTrack();
        self.finalizeFile();

        const filename = try self.parseQuotedString();
        const path = self.allocator.dupe(u8, filename) catch @panic("OOM");

        const file_type = try self.parseFileType();

        log.debug("found FILE: {s} type={s}", .{ filename, @tagName(file_type) });

        self.tmp.file_name = path;
        self.tmp.file_type = file_type;

        self.skipLine();
    }

    fn handleTrack(self: *@This()) !void {
        if (self.tmp.file_name == null) {
            log.err("TRACK found before FILE", .{});
            return CueError.InvalidFormat;
        }

        self.finalizeTrack();

        const track_num = try self.parseNumber();
        const mode = try self.parseMode();

        log.debug("found TRACK: number={d}, mode={s}", .{ track_num, @tagName(mode) });

        self.tmp.track_number = track_num;
        self.tmp.track_mode = mode;

        self.skipLine();
    }

    fn handleIndex(self: *@This()) !void {
        if (self.tmp.file_name == null) {
            log.err("INDEX found before FILE", .{});
            return CueError.InvalidFormat;
        }

        if (self.tmp.track_number == null) {
            log.err("INDEX found before TRACK", .{});
            return CueError.InvalidFormat;
        }

        const index_num = try self.parseNumber();
        const pos = try self.parsePosition();

        log.debug(
            "found INDEX: number={d}, position={d:0>2}:{d:0>2}:{d:0>2}",
            .{ index_num, pos.minute, pos.second, pos.frame },
        );

        const index = IndexNode{
            .number = index_num,
            .position = pos,
        };

        try self.indices.append(self.allocator, index);

        self.skipLine();
    }

    fn finalizeTrack(self: *@This()) void {
        if (self.tmp.track_number) |track_num| {
            const track = TrackNode{
                .number = track_num,
                .mode = self.tmp.track_mode.?,
                .indices = self.indices.toOwnedSlice(self.allocator) catch @panic("OOM"),
            };
            self.tracks.append(self.allocator, track) catch @panic("OOM");
            self.tmp.track_number = null;
            self.tmp.track_mode = null;
        }
    }

    fn finalizeFile(self: *@This()) void {
        if (self.tmp.file_name) |path| {
            const file = FileNode{
                .filename = path,
                .file_type = self.tmp.file_type.?,
                .tracks = self.tracks.toOwnedSlice(self.allocator) catch @panic("OOM"),
            };
            self.files.append(self.allocator, file) catch @panic("OOM");
            self.tmp.file_name = null;
            self.tmp.file_type = null;
        }
    }

    fn finish(self: *@This()) CueSheet {
        self.finalizeTrack();
        self.finalizeFile();

        return CueSheet{
            .files = self.files.toOwnedSlice(self.allocator) catch @panic("OOM"),
        };
    }
};

test "parse multi-track multi-file CUE" {
    const content =
        \\REM Generated by some tool
        \\FILE "track01.bin" BINARY
        \\  TRACK 01 MODE1/2352
        \\    INDEX 00 00:00:00
        \\    INDEX 01 00:02:00
        \\  TRACK 02 MODE2/2352
        \\    PREGAP 00:02:00
        \\    INDEX 01 05:30:45
        \\FILE "track03.bin" BINARY
        \\  TRACK 03 AUDIO
        \\    FLAGS DCP
        \\    INDEX 00 00:00:00
        \\    INDEX 01 00:02:33
        \\FILE "track04.wav" WAVE
        \\  TRACK 04 AUDIO
        \\    INDEX 01 00:00:00
        \\REM End of cue sheet
    ;

    var sheet = try parse(std.testing.allocator, content);
    defer free(std.testing.allocator, &sheet);

    try std.testing.expectEqual(@as(usize, 3), sheet.files.len);

    try std.testing.expectEqualStrings("track01.bin", sheet.files[0].filename);
    try std.testing.expectEqual(FileType.binary, sheet.files[0].file_type);
    try std.testing.expectEqual(@as(usize, 2), sheet.files[0].tracks.len);

    const track1 = sheet.files[0].tracks[0];
    try std.testing.expectEqual(@as(u8, 1), track1.number);
    try std.testing.expectEqual(TrackMode.mode1_2352, track1.mode);
    try std.testing.expectEqual(@as(usize, 2), track1.indices.len);
    try std.testing.expectEqual(@as(u8, 0), track1.indices[0].number);
    try std.testing.expectEqual(@as(u8, 1), track1.indices[1].number);
    try std.testing.expectEqual(@as(u8, 0), track1.indices[1].position.minute);
    try std.testing.expectEqual(@as(u8, 2), track1.indices[1].position.second);
    try std.testing.expectEqual(@as(u8, 0), track1.indices[1].position.frame);

    const track2 = sheet.files[0].tracks[1];
    try std.testing.expectEqual(@as(u8, 2), track2.number);
    try std.testing.expectEqual(TrackMode.mode2_2352, track2.mode);
    try std.testing.expectEqual(@as(usize, 1), track2.indices.len);
    try std.testing.expectEqual(@as(u8, 5), track2.indices[0].position.minute);
    try std.testing.expectEqual(@as(u8, 30), track2.indices[0].position.second);
    try std.testing.expectEqual(@as(u8, 45), track2.indices[0].position.frame);

    try std.testing.expectEqualStrings("track03.bin", sheet.files[1].filename);
    try std.testing.expectEqual(FileType.binary, sheet.files[1].file_type);
    try std.testing.expectEqual(@as(usize, 1), sheet.files[1].tracks.len);

    const track3 = sheet.files[1].tracks[0];
    try std.testing.expectEqual(@as(u8, 3), track3.number);
    try std.testing.expectEqual(TrackMode.audio, track3.mode);
    try std.testing.expectEqual(@as(usize, 2), track3.indices.len);

    try std.testing.expectEqualStrings("track04.wav", sheet.files[2].filename);
    try std.testing.expectEqual(FileType.wave, sheet.files[2].file_type);
}
