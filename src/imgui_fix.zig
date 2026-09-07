const zgui = @import("zgui");

// zgui.backend.newFrame() clobbers io.DisplaySize with the framebuffer size
// and pins io.DisplayFramebufferScale to 1, which breaks HiDPI.
// Fixed by https://github.com/zig-gamedev/zgui/pull/105, but not merged yet.
// This is a workaround which calls the backends directly.
extern fn ImGui_ImplGlfw_NewFrame() void;
extern fn ImGui_ImplOpenGL3_NewFrame() void;

pub fn newFrame() void {
    ImGui_ImplGlfw_NewFrame();
    ImGui_ImplOpenGL3_NewFrame();
    zgui.newFrame();
}
