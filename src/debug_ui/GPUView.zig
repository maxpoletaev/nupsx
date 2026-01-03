const std = @import("std");
const zgui = @import("zgui");
const GPU = @import("../gpu.zig").GPU;

allocator: std.mem.Allocator,
gpu: *GPU,

pub fn init(allocator: std.mem.Allocator, gpu: *GPU) !*@This() {
    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .gpu = gpu,
    };
    return self;
}

pub fn deinit(self: *@This()) void {
    const allocator = self.allocator;
    allocator.destroy(self);
}

pub fn update(self: *@This()) void {
    if (zgui.begin("GPU", .{})) {
        // FIFO State
        if (zgui.collapsingHeader("FIFO", .{})) {
            zgui.text("State: {s}", .{@tagName(self.gpu.gp0_state)});
            zgui.text("Command: 0x{x:0>2}", .{self.gpu.gp0_cmd});
            zgui.text("FIFO Length: {d}", .{self.gpu.gp0_fifo.len});

            if (self.gpu.gp0_fifo.len > 0) {
                zgui.text("FIFO Contents:", .{});
                for (0..self.gpu.gp0_fifo.len) |i| {
                    zgui.text("  [{d}] 0x{x:0>8}", .{ i, self.gpu.gp0_fifo.buf[i] });
                }
            }
        }

        // Draw Area
        if (zgui.collapsingHeader("Draw Area", .{ .default_open = true })) {
            const draw_w = self.gpu.gp0_draw_area_end.x -| self.gpu.gp0_draw_area_start.x;
            const draw_h = self.gpu.gp0_draw_area_end.y -| self.gpu.gp0_draw_area_start.y;

            zgui.text("Start: ({d}, {d})", .{ self.gpu.gp0_draw_area_start.x, self.gpu.gp0_draw_area_start.y });
            zgui.text("End: ({d}, {d})", .{ self.gpu.gp0_draw_area_end.x, self.gpu.gp0_draw_area_end.y });
            zgui.text("Offset: ({d}, {d})", .{ self.gpu.gp0_draw_offset.x, self.gpu.gp0_draw_offset.y });
            zgui.text("Size: {d}x{d}", .{ draw_w, draw_h });
        }

        // Display Area
        if (zgui.collapsingHeader("Display Area", .{ .default_open = true })) {
            const display_res = self.gpu.getDisplayRes();
            zgui.text("Display Enabled: {s}", .{if (self.gpu.gp1_display_enable) "yes" else "no"});
            zgui.text("Start: ({d}, {d})", .{ self.gpu.gp1_display_area_start.x, self.gpu.gp1_display_area_start.y });
            zgui.text("Resolution: {d}x{d}", .{ display_res[0], display_res[1] });
        }

        // Status
        if (zgui.collapsingHeader("Status", .{})) {
            zgui.text("DMA Direction: {s}", .{@tagName(self.gpu.gp1_dma_direction)});
            zgui.text("GPUSTAT: 0x{x:0>8}", .{self.gpu.readGpustat()});
            zgui.text("GPUREAD: 0x{x:0>8}", .{self.gpu.gpuread});
        }
    }
    zgui.end();
}
