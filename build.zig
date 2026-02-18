const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const zglfw = b.dependency("zglfw", .{ .target = target });
    const zgui = b.dependency("zgui", .{ .target = target, .backend = .glfw_opengl3 });
    const zaudio = b.dependency("zaudio", .{ .target = target });
    const zopengl = b.dependency("zopengl", .{});

    const nupsx_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .omit_frame_pointer = false,
        .error_tracing = true,
        .strip = false,
    });
    nupsx_mod.addImport("zglfw", zglfw.module("root"));
    nupsx_mod.addImport("zopengl", zopengl.module("root"));
    nupsx_mod.addImport("zgui", zgui.module("root"));
    nupsx_mod.addImport("zaudio", zaudio.module("root"));

    const nupsx_exe = b.addExecutable(.{
        .name = "nupsx",
        .root_module = nupsx_mod,
    });
    nupsx_exe.linkLibrary(zgui.artifact("imgui"));
    nupsx_exe.linkLibrary(zglfw.artifact("glfw"));
    nupsx_exe.linkLibrary(zaudio.artifact("miniaudio"));
    b.installArtifact(nupsx_exe);

    // run
    {
        const run_cmd = b.addRunArtifact(nupsx_exe);
        run_cmd.step.dependOn(b.getInstallStep());

        if (b.args) |args| {
            run_cmd.addArgs(args);
        }

        const run_step = b.step("run", "Run the app");
        run_step.dependOn(&run_cmd.step);
    }

    // test
    {
        const exe_unit_tests = b.addTest(.{
            .root_module = nupsx_mod,
        });
        const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);
        const test_step = b.step("test", "Run unit tests");
        test_step.dependOn(&run_exe_unit_tests.step);
    }

    // check (required for zls)
    // see: https://zigtools.org/zls/guides/build-on-save/
    {
        const check_exe = b.addExecutable(.{
            .name = "nupsx_check",
            .root_module = nupsx_mod,
        });
        const check = b.step("check", "Check if foo compiles");
        check.dependOn(&check_exe.step);
    }

    // wasm build
    {
        const wasm_mod = b.createModule(.{
            .root_source_file = b.path("src/wasm.zig"),
            .target = b.resolveTargetQuery(.{ .cpu_arch = .wasm32, .os_tag = .freestanding }),
            .optimize = .ReleaseSmall,
        });
        var wasm_exe = b.addExecutable(.{
            .name = "nupsx",
            .root_module = wasm_mod,
        });
        wasm_exe.rdynamic = true;
        wasm_exe.entry = .disabled;

        const dest_dir = "../web"; // relative to prefix dir (zig-out)
        const wasm_step = b.step("wasm", "Build WebAssembly module");
        const wasm_install = b.addInstallArtifact(wasm_exe, .{
            .dest_dir = .{ .override = .{ .custom = dest_dir } },
        });
        wasm_step.dependOn(&wasm_install.step);
    }
}
