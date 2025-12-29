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

        // Key registers
        zgui.separatorText("Registers");
        zgui.text("SR (12):    0x{X:0>8}", .{self.cpu.cop0.r[12]});
        zgui.text("Cause (13): 0x{X:0>8}", .{self.cpu.cop0.r[13]});
        zgui.text("EPC (14):   0x{X:0>8}", .{self.cpu.cop0.r[14]});

        // Status register details
        zgui.separatorText("Status Register");
        zgui.text("Int Enable (curr/prev/old): {d}/{d}/{d}", .{
            @intFromBool(status.curr_int_enable),
            @intFromBool(status.prev_int_enable),
            @intFromBool(status.old_int_enable),
        });
        zgui.text("User Mode (curr/prev/old):  {d}/{d}/{d}", .{
            @intFromBool(status.curr_user_mode),
            @intFromBool(status.prev_user_mode),
            @intFromBool(status.old_user_mode),
        });
        zgui.text("Interrupt Mask: 0x{X:0>2}", .{status.interrupt_mask});
        zgui.text("Isolate Cache: {s}", .{if (status.isolate_cache) "Yes" else "No"});
        zgui.text("Swap Caches: {s}", .{if (status.swap_caches) "Yes" else "No"});
        zgui.text("Boot Vectors: {s}", .{if (status.boot_vectors != 0) "BEV" else "Normal"});
        zgui.text("COP Enable: 0:{d} 1:{d} 2:{d} 3:{d}", .{
            @intFromBool(status.cop0_enable),
            @intFromBool(status.cop1_enable),
            @intFromBool(status.cop2_enable),
            @intFromBool(status.cop3_enable),
        });

        // Cause register details
        zgui.separatorText("Cause Register");
        zgui.text("Interrupt Pending: 0x{X:0>2}", .{cause.interrupt_pending});
        zgui.text("Coprocessor #: {d}", .{cause.cop_number});
        zgui.text("Branch Taken: {s}", .{if (cause.branch_taken) "Yes" else "No"});
        zgui.text("EPC at Branch: {s}", .{if (cause.epc_at_branch) "Yes" else "No"});
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
            .v = &self.bus.debug_paused,
        });

        if (self.bus.debug_paused) {
            zgui.sameLine(.{});
            if (zgui.button("Step", .{})) {
                self.bus.debugStep();
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
