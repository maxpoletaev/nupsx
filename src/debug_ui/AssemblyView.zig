const std = @import("std");
const zgui = @import("zgui");
const mem_mod = @import("../mem.zig");
const cpu_mod = @import("../cpu.zig");
const disasm_mod = @import("../disasm.zig");

const Bus = mem_mod.Bus;
const CPU = cpu_mod.CPU;

const AddrSpace = struct {
    start: u32,
    size: u32,

    fn match(self: @This(), addr: u32) bool {
        return addr >= self.start and addr < (self.start + self.size);
    }
};

const addr_space_000 = AddrSpace{ .start = 0x00000000, .size = 0x800000 };
const addr_space_800 = AddrSpace{ .start = 0x80000000, .size = 0x800000 };
const addr_space_bfc = AddrSpace{ .start = 0xbfc00000, .size = 0x800000 };

allocator: std.mem.Allocator,
tmp_buf: std.Io.Writer.Allocating,
lc: zgui.ListClipper,
bus: *Bus,
cpu: *CPU,

follow_pc: bool,
custom_addr: u32,

pub fn init(allocator: std.mem.Allocator, cpu: *CPU, bus: *Bus) !*@This() {
    const list_clipper = zgui.ListClipper.init();

    const self = try allocator.create(@This());
    self.* = .{
        .tmp_buf = .init(allocator),
        .allocator = allocator,
        .lc = list_clipper,
        .cpu = cpu,
        .bus = bus,
        .follow_pc = false,
        .custom_addr = 0,
    };

    return self;
}

pub fn deinit(self: *@This()) void {
    self.tmp_buf.deinit();
    self.allocator.destroy(self);
}

pub fn update(self: *@This()) void {
    const pc_addr = self.cpu.instr_addr;
    if (self.follow_pc) self.custom_addr = pc_addr;

    const addr_range = if (addr_space_000.match(pc_addr)) addr_space_000 //
        else if (addr_space_800.match(pc_addr)) addr_space_800 //
        else addr_space_bfc;

    const col1_offset = 150;
    const col2_offset = col1_offset + 150;

    const items_count = @as(i32, @intCast(addr_range.size / 4));

    if (zgui.begin("Assembly", .{})) {
        _ = zgui.checkbox("[F]ollow PC", .{ .v = &self.follow_pc });
        zgui.sameLine(.{});

        _ = zgui.setNextItemWidth(300.0);
        _ = zgui.inputInt("##", .{
            .v = @ptrCast(&self.custom_addr),
            .step = 0,
            .flags = .{
                .read_only = self.follow_pc,
                .chars_hexadecimal = true,
            },
        });

        // Hotkeys
        if (zgui.isWindowFocused(.{})) {
            if (zgui.isKeyPressed(zgui.Key.f, false)) {
                self.follow_pc = !self.follow_pc;
            }

            // if (zgui.isKeyPressed(zgui.Key.space, false)) {
            //     if (self.cpu.stall) self.cpu.step();
            // }
        }

        // Scrollable area
        if (zgui.beginChild("scroll", .{})) {
            defer zgui.endChild();

            self.lc.begin(items_count, zgui.getTextLineHeightWithSpacing());
            defer self.lc.end();

            while (self.lc.step()) {
                const start = @as(usize, @intCast(self.lc.DisplayStart));
                const end = @as(usize, @intCast(self.lc.DisplayEnd));

                for (start..end) |i| {
                    const addr = @as(u32, @truncate(addr_range.start + i * 4));
                    const instr = cpu_mod.RawInstr{ .raw = self.bus.read(u32, addr) };

                    self.tmp_buf.clearRetainingCapacity();
                    disasm_mod.toWriter(instr, &self.tmp_buf.writer) catch unreachable;
                    const instr_text = self.tmp_buf.written();

                    {
                        if (pc_addr == addr) {
                            const dl = zgui.getWindowDrawList();

                            const cursor_screen_pos = zgui.getCursorScreenPos();
                            const fill_width = zgui.getContentRegionAvail()[0];

                            dl.addRectFilled(.{
                                .col = 0x888888ff,
                                .pmin = cursor_screen_pos,
                                .pmax = .{ cursor_screen_pos[0] + fill_width, cursor_screen_pos[1] + self.lc.ItemsHeight },
                            });
                        }

                        const cursor = zgui.getCursorPos();

                        zgui.text("{x}", .{addr});
                        zgui.sameLine(.{});

                        zgui.setCursorPosX(cursor[0] + col1_offset);
                        zgui.text("{x}", .{instr.raw});
                        zgui.sameLine(.{});

                        zgui.setCursorPosX(cursor[0] + col2_offset);
                        zgui.textUnformatted(instr_text);
                    }
                }
            }

            if (self.follow_pc and addr_range.match(self.custom_addr)) {
                const item_idx = @divTrunc((self.custom_addr - addr_range.start), 4);
                const item_pos_y = @as(f32, @floatCast(self.lc.StartPosY)) + self.lc.ItemsHeight * @as(f32, @floatFromInt(item_idx));
                zgui.setScrollFromPosY(.{ .local_y = item_pos_y - zgui.getWindowPos()[1] });
            }
        }
    }
    zgui.end();
}
