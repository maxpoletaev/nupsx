const std = @import("std");

pub inline fn field(value: anytype, offset: u8, comptime ReturnType: type) ReturnType {
    const T = @TypeOf(value);
    const type_info = @typeInfo(T);
    if (type_info != .int) {
        @compileError("field requires an integer type");
    }
    const return_info = @typeInfo(ReturnType);
    if (return_info != .int) {
        @compileError("return type must be an integer type");
    }
    const size = @bitSizeOf(ReturnType);
    const mask: T = if (size == @bitSizeOf(T)) ~@as(T, 0) else (@as(T, 1) << size) - 1;
    return @intCast((value >> @intCast(offset)) & mask);
}

test "field - extract single bit" {
    const value: u8 = 0b10101010;
    try std.testing.expectEqual(@as(u1, 0), field(value, 0, u1));
    try std.testing.expectEqual(@as(u1, 1), field(value, 1, u1));
    try std.testing.expectEqual(@as(u1, 0), field(value, 2, u1));
    try std.testing.expectEqual(@as(u1, 1), field(value, 7, u1));
}

test "field - extract multiple bits" {
    const value: u16 = 0b11010110_10101100;
    try std.testing.expectEqual(@as(u4, 0b1100), field(value, 0, u4));
    try std.testing.expectEqual(@as(u4, 0b1011), field(value, 2, u4));
    try std.testing.expectEqual(@as(u8, 0b10101100), field(value, 0, u8));
    try std.testing.expectEqual(@as(u8, 0b11010110), field(value, 8, u8));
}

test "field - extract from u32" {
    const value: u32 = 0xABCD1234;
    try std.testing.expectEqual(@as(u4, 0x4), field(value, 0, u4));
    try std.testing.expectEqual(@as(u8, 0x34), field(value, 0, u8));
    try std.testing.expectEqual(@as(u8, 0x12), field(value, 8, u8));
    try std.testing.expectEqual(@as(u16, 0x1234), field(value, 0, u16));
    try std.testing.expectEqual(@as(u16, 0xABCD), field(value, 16, u16));
}

test "field - extract arbitrary bit widths" {
    const value: u32 = 0b11111111_00000000_10101010_01010101;
    try std.testing.expectEqual(@as(u3, 0b101), field(value, 0, u3));
    try std.testing.expectEqual(@as(u5, 0b10101), field(value, 2, u5));
    // 2725 = 0b101010100101
    try std.testing.expectEqual(@as(u12, 0b101010100101), field(value, 4, u12));
}

test "field - all zeros and all ones" {
    const zeros: u16 = 0x0000;
    const ones: u16 = 0xFFFF;
    try std.testing.expectEqual(@as(u8, 0), field(zeros, 0, u8));
    try std.testing.expectEqual(@as(u8, 0xFF), field(ones, 0, u8));
    try std.testing.expectEqual(@as(u16, 0), field(zeros, 0, u16));
    try std.testing.expectEqual(@as(u16, 0xFFFF), field(ones, 0, u16));
}
