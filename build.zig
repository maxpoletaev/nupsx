const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    const exe = b.addExecutable(.{
        .name = "nupsx",
        .root_module = exe_mod,
    });
    b.installArtifact(exe);

    // zig-gamedev
    {
        const zglfw = b.dependency("zglfw", .{
            .target = target,
        });
        exe.root_module.addImport("zglfw", zglfw.module("root"));
        exe.linkLibrary(zglfw.artifact("glfw"));

        const zopengl = b.dependency("zopengl", .{});
        exe.root_module.addImport("zopengl", zopengl.module("root"));

        const zmath = b.dependency("zmath", .{});
        exe.root_module.addImport("zmath", zmath.module("root"));

        const zgui = b.dependency("zgui", .{
            .target = target,
            .backend = .glfw_opengl3,
        });
        exe.root_module.addImport("zgui", zgui.module("root"));
        exe.linkLibrary(zgui.artifact("imgui"));

        const zaudio = b.dependency("zaudio", .{
            .target = target,
        });
        exe.root_module.addImport("zaudio", zaudio.module("root"));
        exe.linkLibrary(zaudio.artifact("miniaudio"));
    }

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const exe_unit_tests = b.addTest(.{
        .root_module = exe_mod,
    });

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_exe_unit_tests.step);
}
