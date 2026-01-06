const std = @import("std");

pub inline fn field(value: anytype, offset: u8, comptime ReturnType: type) ReturnType {
    const T = @TypeOf(value);
    const type_info = @typeInfo(T);
    if (type_info != .int and type_info.int.signedness != .unsigned) {
        @compileError("field requires an unsigned integer type");
    }

    const return_info = @typeInfo(ReturnType);
    if (return_info != .int) {
        @compileError("return type must be an integer type");
    }

    const size = @bitSizeOf(ReturnType);
    const mask: T = if (size == @bitSizeOf(T)) ~@as(T, 0) else (@as(T, 1) << size) - 1;
    const extracted = (value >> @intCast(offset)) & mask;

    switch (return_info.int.signedness) {
        .signed => {
            const UnsignedType = std.meta.Int(.unsigned, size);
            return @bitCast(@as(UnsignedType, @truncate(extracted)));
        },
        .unsigned => return @truncate(extracted),
    }
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

test "field - signed integers with sign extension" {
    const value1: u32 = 0b00000000_00000000_00000000_01111111;
    try std.testing.expectEqual(@as(i8, 127), field(value1, 0, i8));
    try std.testing.expectEqual(@as(i16, 127), field(value1, 0, i16));
    const value2: u32 = 0b00000000_00000000_00000000_10000000;
    try std.testing.expectEqual(@as(i8, -128), field(value2, 0, i8));
    const value3: u32 = 0b00000000_00000000_00000000_11111111;
    try std.testing.expectEqual(@as(i8, -1), field(value3, 0, i8));
    const value4: u32 = 0b00000000_11111111_00000000_00000000;
    try std.testing.expectEqual(@as(i8, -1), field(value4, 16, i8));
    const value5: u32 = 0b00000000_00000000_10000000_00000000;
    try std.testing.expectEqual(@as(i16, -32768), field(value5, 0, i16));
    const value6: u32 = 0b00000000_00000000_11111111_11111111;
    try std.testing.expectEqual(@as(i16, -1), field(value6, 0, i16));
    const value7: u32 = 0xFFFF_8000; // Bits 0-15 = 0x8000
    try std.testing.expectEqual(@as(i16, -32768), field(value7, 0, i16));
}

test "field - signed integers from different offsets" {
    const value: u32 = 0b11111111_10000001_01111110_10101010;
    try std.testing.expectEqual(@as(i8, -86), field(value, 0, i8));
    try std.testing.expectEqual(@as(i8, 126), field(value, 8, i8));
    try std.testing.expectEqual(@as(i8, -127), field(value, 16, i8));
    try std.testing.expectEqual(@as(i8, -1), field(value, 24, i8));
}
