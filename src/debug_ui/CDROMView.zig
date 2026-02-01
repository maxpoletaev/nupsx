const std = @import("std");
const zgui = @import("zgui");
const CDROM = @import("../cdrom.zig").CDROM;

allocator: std.mem.Allocator,
cdrom: *CDROM,

pub fn init(allocator: std.mem.Allocator, cdrom: *CDROM) !*@This() {
    const self = try allocator.create(@This());
    self.* = .{
        .allocator = allocator,
        .cdrom = cdrom,
    };
    return self;
}

pub fn deinit(self: *@This()) void {
    const allocator = self.allocator;
    allocator.destroy(self);
}

pub fn update(self: *@This()) void {
    if (zgui.begin("CD-ROM", .{})) {
        // Address Register
        if (zgui.collapsingHeader("Address Register", .{ .default_open = true })) {
            zgui.text("Bank Index: {d}", .{self.cdrom.addr.bank_index});
            zgui.text("ADPBUSY: {s}", .{if (self.cdrom.addr.adpcm_busy) "yes" else "no"});
            zgui.text("PRMWRDY: {s}", .{if (self.cdrom.addr.param_not_full) "yes" else "no"});
            zgui.text("PRMEMPT: {s}", .{if (self.cdrom.addr.param_empty) "yes" else "no"});
            zgui.text("RSLRRDY: {s}", .{if (self.cdrom.addr.result_ready) "yes" else "no"});
            zgui.text("DRQSTS: {s}", .{if (self.cdrom.addr.data_ready) "yes" else "no"});
            zgui.text("BUSYSTS: {s}", .{if (self.cdrom.addr.busy_status) "yes" else "no"});
        }

        // Status Register
        if (zgui.collapsingHeader("Status Register", .{ .default_open = true })) {
            zgui.text("Error: {s}", .{if (self.cdrom.stat.err) "yes" else "no"});
            zgui.text("Motor On: {s}", .{if (self.cdrom.stat.motor_on) "yes" else "no"});
            zgui.text("Seek Error: {s}", .{if (self.cdrom.stat.seekerr) "yes" else "no"});
            zgui.text("ID Error: {s}", .{if (self.cdrom.stat.iderr) "yes" else "no"});
            zgui.text("Shell Open: {s}", .{if (self.cdrom.stat.shellopen) "yes" else "no"});
            zgui.text("Read: {s}", .{if (self.cdrom.stat.read) "yes" else "no"});
            zgui.text("Seek: {s}", .{if (self.cdrom.stat.seek) "yes" else "no"});
            zgui.text("Play: {s}", .{if (self.cdrom.stat.play) "yes" else "no"});
        }

        // Mode Register
        if (zgui.collapsingHeader("Mode Register", .{ .default_open = true })) {
            zgui.text("CDDA: {s}", .{if (self.cdrom.mode.cdda) "on" else "off"});
            zgui.text("Auto Pause: {s}", .{if (self.cdrom.mode.auto_pause) "on" else "off"});
            zgui.text("Report: {s}", .{if (self.cdrom.mode.play_report) "on" else "off"});
            zgui.text("Ignore Bit: {s}", .{if (self.cdrom.mode.ignore_bit) "on" else "off"});
            zgui.text("XA Filter: {s}", .{if (self.cdrom.mode.xa_filter) "on" else "off"});
            zgui.text("Sector Size: {s}", .{@tagName(self.cdrom.mode.sector_size)});
            zgui.text("XA ADPCM: {s}", .{if (self.cdrom.mode.xa_adpcm) "on" else "off"});
            zgui.text("Speed: {s}", .{@tagName(self.cdrom.mode.speed)});
        }

        // Request Register
        if (zgui.collapsingHeader("Request Register", .{})) {
            zgui.text("Want Data: {s}", .{if (self.cdrom.req.want_data) "yes" else "no"});
        }

        // Command State
        if (zgui.collapsingHeader("Command State", .{ .default_open = true })) {
            if (self.cdrom.cmd) |cmd| {
                zgui.text("Current Command: 0x{x:0>2}", .{cmd});
            } else {
                zgui.text("Current Command: none", .{});
            }
            zgui.text("Command State: {s}", .{@tagName(self.cdrom.cmd_state)});
            // zgui.text("Delay Cycles: {d}", .{self.cdrom.cmd_delay});
        }

        // IRQ State
        if (zgui.collapsingHeader("IRQ State", .{})) {
            zgui.text("IRQ Mask: 0x{x:0>2}", .{self.cdrom.irq_mask.int_enable});
            zgui.text("IRQ Pending: 0x{x:0>2}", .{self.cdrom.irq_pending.ints});
        }

        // FIFOs
        if (zgui.collapsingHeader("FIFOs", .{})) {
            zgui.text("Params FIFO: {d}/16", .{self.cdrom.params.len});
            if (self.cdrom.params.len > 0) {
                zgui.indent(.{});
                var i: usize = 0;
                while (i < self.cdrom.params.len) : (i += 1) {
                    zgui.text("[{d}] 0x{x:0>2}", .{ i, self.cdrom.params.buf[i] });
                }
                zgui.unindent(.{});
            }

            zgui.text("Results FIFO: {d}/16", .{self.cdrom.results.len});
            if (self.cdrom.results.len > 0) {
                zgui.indent(.{});
                var i: usize = 0;
                while (i < self.cdrom.results.len) : (i += 1) {
                    zgui.text("[{d}] 0x{x:0>2}", .{ i, self.cdrom.results.buf[i] });
                }
                zgui.unindent(.{});
            }
        }

        // Disc State
        if (zgui.collapsingHeader("Disc State", .{})) {
            if (self.cdrom.disc) |disc| {
                zgui.text("Disc Inserted: yes", .{});
                zgui.text("Disc Position: {d} bytes", .{disc.pos});
                zgui.text("Disc Sector: {d}", .{disc.pos / 2352});
                zgui.text("Disc Size: {d} bytes", .{disc.data.len});
            } else {
                zgui.text("Disc Inserted: no", .{});
            }

            if (self.cdrom.seekloc) |loc| {
                zgui.text("Seek Location: {d}", .{loc});
            } else {
                zgui.text("Seek Location: none", .{});
            }
        }

        // Misc
        if (zgui.collapsingHeader("Misc", .{})) {
            zgui.text("Mute: {s}", .{if (self.cdrom.mute) "yes" else "no"});
        }
    }
    zgui.end();
}
