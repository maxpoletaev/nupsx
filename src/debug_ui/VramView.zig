const std = @import("std");
const zgui = @import("zgui");
const zopengl = @import("zopengl");
const GPU = @import("../gpu.zig").GPU;

const gl = zopengl.bindings;

pub const HighlightRect = struct {
    x: u16,
    y: u16,
    w: u16,
    h: u16,
    color: u32,
    label: []const u8,
    enabled: bool,
};

allocator: std.mem.Allocator,
texture_id: gl.Uint,
gpu: *GPU,
show_draw_area: bool,
show_display_area: bool,

pub fn init(allocator: std.mem.Allocator, gpu: *GPU) !*@This() {
    var texture_id: gl.Uint = undefined;
    gl.genTextures(1, &texture_id);

    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .texture_id = texture_id,
        .gpu = gpu,
        .show_draw_area = false,
        .show_display_area = false,
    };

    return self;
}

pub fn deinit(self: *@This()) void {
    gl.deleteTextures(1, &self.texture_id);
    self.allocator.destroy(self);
}

pub fn update(self: *@This()) void {
    if (zgui.begin("VRAM", .{})) {
        _ = zgui.checkbox("Show Draw Area", .{ .v = &self.show_draw_area });
        zgui.sameLine(.{});
        _ = zgui.checkbox("Show Display Area", .{ .v = &self.show_display_area });

        gl.bindTexture(gl.TEXTURE_2D, self.texture_id);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.pixelStorei(gl.UNPACK_ROW_LENGTH, 0);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB5, 1024, 512, 0, gl.RGBA, gl.UNSIGNED_SHORT_1_5_5_5_REV, self.gpu.vram);

        const aspect_ratio = 1024.0 / 512.0; // 2:1 aspect ratio
        const available_width = zgui.getContentRegionAvail()[0];
        const available_height = zgui.getContentRegionAvail()[1];

        var display_width = available_width;
        var display_height = display_width / aspect_ratio;

        if (display_height > available_height) {
            display_height = available_height;
            display_width = display_height * aspect_ratio;
        }

        const cursor_pos = zgui.getCursorScreenPos();
        const dl = zgui.getWindowDrawList();

        const textureRef = zgui.TextureRef{
            .tex_id = @enumFromInt(self.texture_id),
            .tex_data = null,
        };

        zgui.image(textureRef, .{
            .w = display_width,
            .h = display_height,
        });

        const draw_w = self.gpu.gp0_draw_area_end.x -| self.gpu.gp0_draw_area_start.x;
        const draw_h = self.gpu.gp0_draw_area_end.y -| self.gpu.gp0_draw_area_start.y;
        const gpu_display_res = self.gpu.getDisplayRes();

        const highlights = [_]HighlightRect{
            .{
                .x = self.gpu.gp0_draw_area_start.x,
                .y = self.gpu.gp0_draw_area_start.y,
                .w = draw_w,
                .h = draw_h,
                .color = 0xff00ff00,
                .label = "Draw",
                .enabled = self.show_draw_area,
            },
            .{
                .x = self.gpu.gp1_display_area_start.x,
                .y = self.gpu.gp1_display_area_start.y,
                .w = gpu_display_res[0],
                .h = gpu_display_res[1],
                .color = 0xff0000ff,
                .label = "Display",
                .enabled = self.show_display_area,
            },
        };

        const scale_x = display_width / 1024.0;
        const scale_y = display_height / 512.0;

        for (highlights) |rect| {
            if (rect.enabled and rect.w > 0 and rect.h > 0) {
                const x1 = cursor_pos[0] + @as(f32, @floatFromInt(rect.x)) * scale_x;
                const y1 = cursor_pos[1] + @as(f32, @floatFromInt(rect.y)) * scale_y;
                const x2 = x1 + @as(f32, @floatFromInt(rect.w)) * scale_x;
                const y2 = y1 + @as(f32, @floatFromInt(rect.h)) * scale_y;

                dl.addRect(.{ .pmin = .{ x1, y1 }, .pmax = .{ x2, y2 }, .col = rect.color, .thickness = 2.0 });
                dl.addText(.{ x1 + 4, y1 + 4 }, rect.color, "{s}", .{rect.label});
            }
        }
    }
    zgui.end();
}
