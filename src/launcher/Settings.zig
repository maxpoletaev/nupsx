const std = @import("std");
const zgui = @import("zgui");

const consts = @import("../consts.zig");
const Config = @import("../config.zig").Config;
const RendererBackend = @import("../args.zig").RendererBackend;
const widgets = @import("widgets.zig");

const modal_width = 480;
const upscale_combo_width = 140;
const renderer_combo_width = 120;
const window_padding = 20.0;
const section_spacing = 12.0;

const button_width = 100;

pub const Values = struct {
    shader_enabled: bool = true,
    upscale: u32 = 1,
    renderer: RendererBackend = .threaded,
};

config: *Config,
values: Values = .{},
draft: Values = .{}, // edited in the modal, committed on Save
is_open: bool = false,

pub fn open(self: *@This()) void {
    self.draft = self.values;
    self.is_open = true;
}

pub fn load(self: *@This()) void {
    if (self.config.getBool("shader_enabled")) |enabled| {
        self.values.shader_enabled = enabled;
    }
    if (self.config.getInt(u32, "upscale")) |scale| {
        if (scale >= 1 and scale <= consts.max_upscale) self.values.upscale = scale;
    }
    if (self.config.getEnum(RendererBackend, "renderer")) |backend| {
        self.values.renderer = backend;
    }
}

pub fn apply(self: *const @This()) void {
    self.config.setBool("shader_enabled", self.values.shader_enabled);
    self.config.setInt("upscale", self.values.upscale);
    self.config.setEnum("renderer", self.values.renderer);
}

fn commit(self: *@This()) void {
    self.values = self.draft;
    self.apply();
    self.config.saveConfig();
}

pub fn update(self: *@This(), parent_w: f32, parent_h: f32) void {
    if (!self.is_open) return;
    zgui.openPopup("Settings", .{});

    zgui.setNextWindowSize(.{ .w = modal_width, .h = 0, .cond = .always });
    zgui.setNextWindowPos(.{ .x = parent_w / 2, .y = parent_h / 2, .pivot_x = 0.5, .pivot_y = 0.5 });

    zgui.pushStyleVar2f(.{ .idx = .window_padding, .v = .{ window_padding, window_padding } });
    const visible = zgui.beginPopupModal("Settings", .{
        .popen = &self.is_open,
        .flags = .{ .no_resize = true },
    });
    zgui.popStyleVar(.{ .count = 1 });
    if (!visible) return;
    defer zgui.endPopup();

    zgui.dummy(.{ .w = 0, .h = 4 });

    // internal resolution
    {
        zgui.alignTextToFramePadding();
        zgui.text("Internal Resolution:", .{});
        zgui.sameLine(.{ .spacing = 10 });
        var upscale_idx: i32 = @intCast(self.draft.upscale - 1);
        zgui.pushItemWidth(upscale_combo_width);
        if (zgui.combo("##upscale", .{
            .current_item = &upscale_idx,
            .items_separated_by_zeros = "1x (~240p)\x00" ++
                "2x (~480p)\x00" ++
                "3x (~720p)\x00" ++
                "4x (~960p)\x00" ++
                "5x (~1080p)\x00" ++
                "6x (~1440p)\x00" ++
                "7x (~1680p)\x00" ++
                "8x (~4K)\x00",
        })) self.draft.upscale = @intCast(upscale_idx + 1);
        zgui.popItemWidth();
        widgets.hint("Higher rendering quality at the cost of performance");
        zgui.dummy(.{ .w = 0, .h = section_spacing });
    }

    // renderer backend
    {
        zgui.alignTextToFramePadding();
        zgui.text("Renderer Backend:", .{});
        zgui.sameLine(.{ .spacing = 10 });
        var renderer_idx: i32 = @intFromEnum(self.draft.renderer);
        zgui.pushItemWidth(renderer_combo_width);
        if (zgui.combo("##renderer", .{
            .current_item = &renderer_idx,
            .items_separated_by_zeros = "Software\x00Threaded\x00OpenGL\x00",
        })) self.draft.renderer = @enumFromInt(renderer_idx);
        zgui.popItemWidth();
        widgets.hint("Threaded runs the software rasterizer on a separate thread");
        widgets.hint("OpenGL is faster (epsecially for upscaled res), but incomplete");
        zgui.dummy(.{ .w = 0, .h = section_spacing });
    }

    // shader
    {
        _ = zgui.checkbox("Enable NTSC Shader", .{ .v = &self.draft.shader_enabled });
        widgets.hint("Simulates composite video artifacts");
        zgui.dummy(.{ .w = 0, .h = section_spacing });
    }

    zgui.separator();
    zgui.dummy(.{ .w = 0, .h = section_spacing });

    if (zgui.button("Save", .{ .w = button_width, .h = 0 })) {
        self.commit();
        self.is_open = false;
        zgui.closeCurrentPopup();
    }
    zgui.sameLine(.{ .spacing = 10 });
    if (widgets.greyButton("Cancel", button_width, 0)) {
        self.is_open = false;
        zgui.closeCurrentPopup();
    }
}
