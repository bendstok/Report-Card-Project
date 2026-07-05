const std = @import("std");
const rlz = @import("raylib_zig");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // Dependencies (added via `zig fetch --save`, see README).
    const znumerics_dep = b.dependency("znumerics", .{
        .target = target,
        .optimize = optimize,
    });
    const znumerics_mod = znumerics_dep.module("znumerics");

    const raylib_dep = b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    });
    const raylib_mod = raylib_dep.module("raylib");
    const raylib_artifact = raylib_dep.artifact("raylib");

    // Executable: window loop + plotting, imports both deps.
    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "znumerics", .module = znumerics_mod },
            .{ .name = "raylib", .module = raylib_mod },
        },
    });
    exe_mod.linkLibrary(raylib_artifact);

    // --- web build: zig build -Dtarget=wasm32-emscripten --------------------
    // Produces zig-out/web/report_card.{html,js,wasm} via emcc. The emsdk
    // toolchain is fetched as a package and installed/activated by the build
    // itself on first use. ASYNCIFY keeps main.zig's plain while-loop valid
    // in the browser. `zig build run -Dtarget=wasm32-emscripten` serves it
    // through emrun.
    if (target.result.os.tag == .emscripten) {
        const wasm = b.addLibrary(.{
            .name = "report_card",
            .root_module = exe_mod,
        });

        const emcc_flags = rlz.emsdk.emccDefaultFlags(b.allocator, .{
            .optimize = optimize,
            .asyncify = true,
        });
        const emcc_settings = rlz.emsdk.emccDefaultSettings(b.allocator, .{
            .optimize = optimize,
        });

        const web_dir: std.Build.InstallDir = .{ .custom = "web" };
        const emcc_step = rlz.emsdk.emccStep(b, raylib_artifact, wasm, .{
            .optimize = optimize,
            .flags = emcc_flags,
            .settings = emcc_settings,
            // The raylib C package ships the canvas shell its own web
            // examples use; the artifact's owner is that package's builder.
            .shell_file_path = raylib_artifact.step.owner.path("src/shell.html"),
            .install_dir = web_dir,
        });
        b.getInstallStep().dependOn(emcc_step);

        const emrun_step = rlz.emsdk.emrunStep(
            b,
            b.getInstallPath(web_dir, "report_card.html"),
            &.{},
        );
        emrun_step.dependOn(emcc_step);
        b.step("run", "Serve the web build in a browser via emrun").dependOn(emrun_step);
        return;
    }

    const exe = b.addExecutable(.{
        .name = "report_card",
        .root_module = exe_mod,
    });
    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);
    const run_step = b.step("run", "Run the report card app");
    run_step.dependOn(&run_cmd.step);

    // Unit tests: src/report.zig is pure math (no raylib import), so the
    // test module only needs znumerics and runs headless.
    const report_test_mod = b.createModule(.{
        .root_source_file = b.path("src/report.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "znumerics", .module = znumerics_mod },
        },
    });
    const report_tests = b.addTest(.{ .root_module = report_test_mod });
    const run_report_tests = b.addRunArtifact(report_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_report_tests.step);
}
