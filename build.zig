const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // const imgui_lib = b.addLibrary(.{
    //     .name = "imgui",
    //     .linkage = .static,
    //     .root_module = b.createModule(.{
    //         .target = target,
    //         .optimize = optimize,
    //     }),
    // });
    // imgui_lib.addCSourceFiles(.{
    //     .root = b.path("deps/cimgui"),
    //     .files = &[_][]const u8{
    //         "cimgui.cpp",
    //         "imgui/imgui.cpp",
    //         "imgui/imgui_demo.cpp",
    //         "imgui/imgui_draw.cpp",
    //         "imgui/imgui_widgets.cpp",
    //         "imgui/imgui_tables.cpp",
    //         "imgui/backends/imgui_impl_sdl3.cpp",
    //         "imgui/backends/imgui_impl_opengl3.cpp",
    //     },
    //     .flags = &[_][]const u8{
    //         "--std=c++20",
    //         "-fno-rtti",
    //         "-fno-exceptions",
    //         "-DIMGUI_IMPL_API=extern \"C\"",
    //         "-DIMGUI_DISABLE_OBSOLETE_FUNCTIONS",
    //     },
    // });
    // imgui_lib.linkLibC();
    // imgui_lib.linkLibCpp();
    // imgui_lib.linkFramework("OpenGL");
    // imgui_lib.linkSystemLibrary("SDL3");
    // imgui_lib.addIncludePath(b.path("deps/cimgui"));
    // imgui_lib.addIncludePath(b.path("deps/cimgui/imgui"));

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    const exe = b.addExecutable(.{
        .name = "nupsx",
        .root_module = exe_mod,
    });
    // exe.linkLibC();
    // exe.linkLibCpp();
    // exe.linkFramework("OpenGL");
    // exe.linkSystemLibrary("SDL3");
    // exe.linkLibrary(imgui_lib);
    // exe.addIncludePath(b.path("deps/cimgui"));
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
