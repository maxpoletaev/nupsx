const std = @import("std");
const zgui = @import("zgui");
const cpu_mod = @import("../cpu.zig");

const CPU = cpu_mod.CPU;

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

pub fn init(allocator: std.mem.Allocator, cpu_: *CPU) !*@This() {
    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .cpu = cpu_,
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

        const exc_code = @as(cpu_mod.Cop0.ExcCode, self.cpu.cop0.cause().exc_code);
        const exc_tag = std.enums.tagName(cpu_mod.Cop0.ExcCode, exc_code);

        _ = zgui.text("in exception: {d}", .{@intFromBool(self.cpu.in_exception)});
        _ = zgui.text("exc code: {s}", .{if (exc_tag) |t| t else "-"});
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
        // _ = zgui.checkbox("Stall", .{
        //     .v = &self.cpu.stall,
        // });

        // if (self.cpu.stall) {
        //     zgui.sameLine(.{});
        //     if (zgui.button("Step", .{})) {
        //         self.cpu.step();
        //     }
        // }

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
