const zgui = @import("zgui");

const hint_font_size = 14.0;

const grey_button_color: [4]f32 = .{ 0.30, 0.30, 0.33, 1.0 };
const grey_button_hovered_color: [4]f32 = .{ 0.38, 0.38, 0.42, 1.0 };
const grey_button_active_color: [4]f32 = .{ 0.25, 0.25, 0.28, 1.0 };

pub fn hint(comptime txt: []const u8) void {
    zgui.pushFont(null, hint_font_size);
    zgui.textDisabled(txt, .{});
    zgui.popFont();
}

pub fn greyButton(label: [:0]const u8, w: f32, h: f32) bool {
    zgui.pushStyleColor4f(.{ .idx = .button, .c = grey_button_color });
    zgui.pushStyleColor4f(.{ .idx = .button_hovered, .c = grey_button_hovered_color });
    zgui.pushStyleColor4f(.{ .idx = .button_active, .c = grey_button_active_color });
    defer zgui.popStyleColor(.{ .count = 3 });
    return zgui.button(label, .{ .w = w, .h = h });
}
