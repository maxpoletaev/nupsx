const std = @import("std");
const zgui = @import("zgui");
const cpu_mod = @import("../cpu.zig");
const mem = @import("../mem.zig");

const CPU = cpu_mod.CPU;
const Bus = mem.Bus;

const hex_field_flags = zgui.InputTextFlags{
    .chars_hexadecimal = true,
    .chars_uppercase = true,
};

const hex_field_flags_ro = zgui.InputTextFlags{
    .chars_hexadecimal = true,
    .chars_uppercase = true,
    .read_only = true,
};

allocator: std.mem.Allocator,
cpu: *CPU,
bus: *Bus,

paused: bool = false,
step_requested: bool = false,

pub fn init(allocator: std.mem.Allocator, cpu: *CPU, bus: *Bus) !*@This() {
    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .cpu = cpu,
        .bus = bus,
    };
    return self;
}

pub fn deinit(self: *@This()) void {
    const allocator = self.allocator;
    allocator.destroy(self);
}

fn drawCop0Tab(self: *@This()) void {
    if (zgui.beginTabItem("COP0", .{})) {
        defer zgui.endTabItem();

        const status = self.cpu.cop0.status();
        const cause = self.cpu.cop0.cause();

        const exc_code = @as(cpu_mod.Cop0.ExcCode, cause.exc_code);
        const exc_tag = std.enums.tagName(cpu_mod.Cop0.ExcCode, exc_code);

        // Exception state
        zgui.separatorText("Exception State");
        zgui.text("In Exception: {s}", .{if (self.cpu.in_exception) "Yes" else "No"});
        zgui.text("Exception Depth: {d}", .{self.cpu.cop0.depth});
        zgui.text("Exc Code: {s} ({d})", .{ if (exc_tag) |t| t else "unknown", @intFromEnum(exc_code) });

        var interrupt_mask: u32 = status.interrupt_mask;
        var interrupt_pending: u32 = cause.interrupt_pending;

        zgui.separatorText("Interrupts");
        _ = zgui.inputInt("Mask", .{
            .step = 0,
            .flags = hex_field_flags_ro,
            .v = @ptrCast(&interrupt_mask),
        });
        _ = zgui.inputInt("Pending", .{
            .step = 0,
            .flags = hex_field_flags_ro,
            .v = @ptrCast(&interrupt_pending),
        });
    }
}

fn drawGprTab(self: *@This()) void {
    if (zgui.beginTabItem("GPR", .{})) {
        defer zgui.endTabItem();

        zgui.pushItemWidth(300.0);

        for (self.cpu.gpr, 0..) |_, i| {
            _ = zgui.inputInt(cpu_mod.RegName[i], .{
                .step = 0,
                .flags = hex_field_flags,
                .v = @ptrCast(&self.cpu.gpr[i]),
            });
        }

        zgui.popItemWidth();
    }
}

pub fn update(self: *@This()) void {
    if (zgui.begin("CPU", .{})) {
        _ = zgui.checkbox("Pause", .{
            .v = &self.paused,
        });

        if (self.paused) {
            zgui.sameLine(.{});
            if (zgui.button("Step", .{})) {
                self.step_requested = true;
            }
        }

        _ = zgui.inputInt("PC", .{
            .step = 4,
            .flags = hex_field_flags,
            .v = @ptrCast(&self.cpu.pc),
        });

        _ = zgui.inputInt("Next PC", .{
            .step = 4,
            .flags = hex_field_flags,
            .v = @ptrCast(&self.cpu.next_pc),
        });

        _ = zgui.inputInt("Instr", .{
            .step = 0,
            .flags = .{
                .chars_hexadecimal = true,
                .chars_uppercase = true,
                .read_only = true,
            },
            .v = @ptrCast(&self.cpu.instr.code),
        });

        if (zgui.beginTabBar("##", .{})) {
            self.drawGprTab();
            self.drawCop0Tab();
        }
        zgui.endTabBar();
    }

    zgui.end();
}
