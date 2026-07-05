//! System Report Card: window loop, key handling, drawing.
//! Analysis runs once per system switch or Apply — the no-alloc rule for the
//! render loop bends only during interaction (pole dragging, PID slider
//! moves), where each change re-runs the analysis.

const std = @import("std");
const builtin = @import("builtin");
const rl = @import("raylib");
const report = @import("report.zig");
const plot = @import("plot.zig");
const custom = @import("custom.zig");
const tf = @import("tf.zig");
const pidpanel = @import("pidpanel.zig");

const screen_w = 1280;
const screen_h = 720;

// Palette
const col_bg = rl.Color.init(24, 26, 32, 255);
const col_panel_border = rl.Color.init(90, 95, 110, 255);
const col_axis = rl.Color.init(120, 125, 140, 255);
const col_text = rl.Color.init(220, 222, 228, 255);
const col_dim = rl.Color.init(150, 153, 165, 255);
const col_stable_region = rl.Color.init(60, 160, 90, 45);
const col_pole = rl.Color.init(255, 203, 0, 255);
const col_zero = rl.Color.init(120, 220, 190, 255);
const col_cl_pole = rl.Color.init(230, 115, 255, 255);
const col_response = rl.Color.init(102, 191, 255, 255);
const col_response_dim = rl.Color.init(102, 191, 255, 90);
const col_setpoint = rl.Color.init(220, 222, 228, 130);
const col_stable = rl.Color.init(0, 228, 48, 255);
const col_unstable = rl.Color.init(230, 41, 55, 255);
const col_marginal = rl.Color.init(255, 161, 0, 255);

/// Pixel radius for grabbing a pole marker with the mouse.
const grab_px: i32 = 9;

const systems = [_]report.SysDesc{
    .{ .name = "Underdamped", .n = 2, .a = &.{ 0, 1, -2, -0.6 }, .b = &.{ 0, 1 }, .c = &.{ 1, 0 } },
    .{ .name = "Overdamped", .n = 2, .a = &.{ 0, 1, -2, -3 }, .b = &.{ 0, 1 }, .c = &.{ 1, 0 } },
    .{ .name = "Unstable", .n = 2, .a = &.{ 0, 1, 2, -0.5 }, .b = &.{ 0, 1 }, .c = &.{ 1, 0 } },
};

/// Input mode: preset keys vs the custom editor.
const Mode = enum { preset, custom };

/// What system the current report describes. `dragged` is entered by
/// dragging a pole and left via 1/2/3 or the editor's Apply.
const Showing = enum { preset, custom, dragged };

/// Mouse-dragged pole set. The numerator (zeros and gain) is captured once
/// at drag start and held fixed; the denominator is rebuilt from the moved
/// poles every dragged frame, then realized via tf2ss into fixed arrays.
const Drag = struct {
    active: bool = false,
    grabbed: usize = 0,
    n_groups: usize = 0,
    groups: [tf.max_deg]tf.PoleGroup = undefined,
    num: tf.Poly = .{},
    ss: tf.SsFixed = .{},

    /// View the latest realized system as a SysDesc. Slices point into
    /// `self`, which lives in main's frame for the whole run.
    fn desc(self: *const Drag) report.SysDesc {
        const n = self.ss.n;
        return .{
            .name = "Dragged",
            .n = n,
            .a = self.ss.a[0 .. n * n],
            .b = self.ss.b[0..n],
            .c = self.ss.c[0..n],
            .d = self.ss.d,
        };
    }
};

/// Plot ranges, precomputed once per analysis. The PID panel's closed-loop
/// results widen the ranges when present: `extra_y` (response) and
/// `extra_poles` (the map zooms out to keep closed-loop X's in view).
const View = struct {
    pole_vp: plot.Viewport,
    resp_vp: plot.Viewport,

    fn compute(
        rep: *const report.Report,
        extra_y: ?[]const f64,
        extra_poles: ?[]const report.Complex,
    ) View {
        // Pole map: symmetric square range around the origin, covering
        // poles and zeros (and closed-loop poles when the PID panel is up).
        var r: f64 = 1.0;
        for (rep.poles) |p| {
            r = @max(r, @abs(p.re));
            r = @max(r, @abs(p.im));
        }
        for (rep.zeros) |z| {
            r = @max(r, @abs(z.re));
            r = @max(r, @abs(z.im));
        }
        if (extra_poles) |ps| {
            for (ps) |p| {
                r = @max(r, @abs(p.re));
                r = @max(r, @abs(p.im));
            }
        }
        r *= 1.25;

        // Step response: y range from (clamped) samples, always including 0.
        var ymin: f64 = 0.0;
        var ymax: f64 = 0.0;
        for (rep.y) |v| {
            const c = std.math.clamp(v, -50.0, 50.0);
            ymin = @min(ymin, c);
            ymax = @max(ymax, c);
        }
        if (extra_y) |ys| {
            for (ys) |v| {
                const c = std.math.clamp(v, -50.0, 50.0);
                ymin = @min(ymin, c);
                ymax = @max(ymax, c);
            }
            // The setpoint line (r = 1) must stay in view even when the
            // closed-loop response never reaches it.
            ymax = @max(ymax, 1.0);
        }
        if (ymax - ymin < 1e-9) ymax = ymin + 1.0;
        const pad = 0.1 * (ymax - ymin);

        return .{
            .pole_vp = .{
                .xmin = -r,
                .xmax = r,
                .ymin = -r,
                .ymax = r,
                .px0 = 40,
                .py0 = 70,
                .pw = 300,
                .ph = 300,
            },
            .resp_vp = .{
                .xmin = 0,
                .xmax = rep.t_end,
                .ymin = ymin - pad,
                .ymax = ymax + pad,
                .px0 = 430,
                .py0 = 70,
                .pw = screen_w - 430 - 40,
                .ph = 300,
            },
        };
    }
};

/// The SysDesc behind what's currently on screen, for re-analysis.
fn currentDesc(
    showing: Showing,
    current: usize,
    editor: *custom.Editor,
    drag: *const Drag,
) ?report.SysDesc {
    return switch (showing) {
        .preset => systems[current],
        .custom => editor.buildSysDesc(),
        .dragged => drag.desc(),
    };
}

// std.debug's stderr / stack-trace machinery owns a std.Io.Threaded
// instance in Zig 0.16.0, and Io.Threaded does not compile for
// wasm32-emscripten (its child-process code trips type errors in
// std.os.emscripten). Anything that reaches that machinery breaks the
// web build; the known roots here are znumerics' bounds-check
// std.log.warn (-> defaultLog -> debug_io) and the default panic
// handler. So for the web target: silence std.log and use the minimal
// panic handler.
pub const panic = if (builtin.os.tag == .emscripten)
    std.debug.FullPanic(webPanic)
else
    std.debug.FullPanic(std.debug.defaultPanic);

/// Panic without printing: even std.debug.simple_panic writes the message
/// to stderr, which routes through the Io machinery on the web target.
/// A trap still stops the wasm instance with a RuntimeError in the console.
fn webPanic(msg: []const u8, first_trace_addr: ?usize) noreturn {
    _ = msg;
    _ = first_trace_addr;
    @trap();
}

pub const std_options: std.Options = .{
    .logFn = if (builtin.os.tag == .emscripten) webNopLog else std.log.defaultLog,
};

fn webNopLog(
    comptime level: std.log.Level,
    comptime scope: @TypeOf(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    _ = level;
    _ = scope;
    _ = format;
    _ = args;
}

pub fn main() void {
    if (builtin.os.tag == .emscripten) {
        // C allocator: DebugAllocator's leak accounting is a native-dev
        // tool, and its reporting also goes through std.debug printing.
        run(std.heap.c_allocator) catch {};
    } else {
        var gpa = std.heap.DebugAllocator(.{}){};
        defer std.debug.assert(gpa.deinit() == .ok);
        run(gpa.allocator()) catch |err| {
            std.debug.print("fatal: {s}\n", .{@errorName(err)});
            std.process.exit(1);
        };
    }
}

fn run(alloc: std.mem.Allocator) !void {
    rl.initWindow(screen_w, screen_h, "System Report Card");
    defer rl.closeWindow();
    rl.setTargetFPS(60);
    // Esc is used to leave the custom editor; don't let raylib treat it
    // as the window-exit key (KeyboardKey value 0 = none).
    rl.setExitKey(@enumFromInt(0));

    var mode: Mode = .preset;
    var showing: Showing = .preset;
    var editor = custom.Editor.init();
    var drag = Drag{};
    var pid = pidpanel.Panel{};

    var current: usize = 0;
    var t_user: ?f64 = null; // plot-time override (Up/Down keys, R = auto)
    var rep = try report.analyze(alloc, systems[current]);
    defer rep.deinit();
    var view = View.compute(&rep, null, null);

    while (!rl.windowShouldClose()) {
        switch (mode) {
            .preset => {
                var want: ?usize = null;
                if (rl.isKeyPressed(.one)) want = 0;
                if (rl.isKeyPressed(.two)) want = 1;
                if (rl.isKeyPressed(.three)) want = 2;
                if (rl.isKeyPressed(.four)) {
                    mode = .custom;
                    pid.active = false;
                    want = null;
                } else if (want) |idx| {
                    if (idx != current or showing != .preset) {
                        const next = try report.analyze(alloc, systems[idx]);
                        rep.deinit();
                        rep = next;
                        current = idx;
                        showing = .preset;
                        drag.active = false;
                        t_user = null; // back to automatic horizon
                        pid.dirty = true;
                        view = View.compute(&rep, null, null);
                    }
                }
                if (rl.isKeyPressed(.five)) {
                    pid.active = !pid.active;
                    if (pid.active) {
                        pid.dirty = true;
                    } else {
                        view = View.compute(&rep, null, null);
                    }
                }
            },
            .custom => switch (editor.update()) {
                .none => {},
                .exit => mode = .preset,
                .apply => {
                    if (editor.buildSysDesc()) |sys_base| {
                        var sys = sys_base;
                        sys.t_end = t_user;
                        if (report.analyze(alloc, sys)) |next| {
                            rep.deinit();
                            rep = next;
                            showing = .custom;
                            drag.active = false;
                            pid.dirty = true;
                            view = View.compute(&rep, null, null);
                        } else |err| {
                            editor.setError(@errorName(err));
                        }
                    }
                },
            },
        }

        // Plot-time control, active in both modes: Up doubles the horizon,
        // Down halves it, R returns to the automatic choice. (The custom
        // editor only uses Left/Right/Tab, so Up/Down are free.)
        {
            var factor: f64 = 0;
            if (rl.isKeyPressed(.up)) factor = 2.0;
            if (rl.isKeyPressed(.down)) factor = 0.5;
            const reset = rl.isKeyPressed(.r) and t_user != null;

            if (factor != 0 or reset) {
                t_user = if (reset)
                    null
                else
                    std.math.clamp((t_user orelse rep.t_end) * factor, 0.1, 1000.0);

                if (currentDesc(showing, current, &editor, &drag)) |desc_base| {
                    var desc = desc_base;
                    desc.t_end = t_user;
                    if (report.analyze(alloc, desc)) |next| {
                        rep.deinit();
                        rep = next;
                        pid.dirty = true;
                        const pole_vp = view.pole_vp;
                        view = View.compute(&rep, null, null);
                        // A frozen map stays frozen through a time-window change.
                        if (drag.active) view.pole_vp = pole_vp;
                    } else |err| {
                        if (mode == .custom) editor.setError(@errorName(err));
                    }
                }
            }
        }

        // --- pole dragging on the map --------------------------------------
        if (!drag.active and rl.isMouseButtonPressed(.left)) {
            tryStartDrag(alloc, &drag, &rep, view.pole_vp, showing, current, &editor);
        }
        if (drag.active) {
            if (!rl.isMouseButtonDown(.left)) {
                drag.active = false;
                // Re-fit the viewports that were frozen during the drag.
                const extra_y: ?[]const f64 = if (pid.active and pid.ok) pid.y[0..] else null;
                const extra_p: ?[]const report.Complex = if (pid.active and pid.ok) pid.poles() else null;
                view = View.compute(&rep, extra_y, extra_p);
            } else {
                dragFrame(alloc, &drag, &rep, &view, t_user, &showing, &pid);
            }
        }

        // --- PID playground --------------------------------------------------
        if (pid.active) {
            pid.update();
            if (pid.dirty) {
                if (currentDesc(showing, current, &editor, &drag)) |desc| {
                    pid.recompute(alloc, desc, rep.t_end);
                    if (pid.ok) {
                        const pole_vp = view.pole_vp;
                        view = View.compute(&rep, pid.y[0..], pid.poles());
                        if (drag.active) view.pole_vp = pole_vp;
                    }
                }
            }
        }

        rl.beginDrawing();
        defer rl.endDrawing();
        rl.clearBackground(col_bg);

        const pid_view: ?*const pidpanel.Panel = if (pid.active and pid.ok) &pid else null;

        drawHeader(current, showing);
        drawPoleMap(view.pole_vp, &rep, pid_view);
        drawResponse(view.resp_vp, &rep, pid_view);
        if (pid.active) pid.draw() else drawReportStrip(&rep, mode == .preset);
        if (mode == .custom) editor.draw();
    }
}

/// Grab a pole marker under the mouse and capture the drag state:
/// the grouped pole set from the report's modes, plus the TF numerator
/// (zeros and gain), which stays fixed while poles move.
fn tryStartDrag(
    alloc: std.mem.Allocator,
    drag: *Drag,
    rep: *const report.Report,
    vp: plot.Viewport,
    showing: Showing,
    current: usize,
    editor: *custom.Editor,
) void {
    const mp = rl.getMousePosition();
    const mx: i32 = @intFromFloat(mp.x);
    const my: i32 = @intFromFloat(mp.y);
    if (!vp.contains(mx, my)) return;
    if (rep.modes.len > tf.max_deg) return;

    var best: ?usize = null;
    var best_d2: i32 = grab_px * grab_px;
    for (rep.modes, 0..) |m, i| {
        const px_ = vp.px(m.re);
        // A conjugate pair has two markers; grabbing either moves both.
        const pys = [2]i32{ vp.py(m.im), vp.py(-m.im) };
        for (pys) |py_| {
            const dx = mx - px_;
            const dy = my - py_;
            const d2 = dx * dx + dy * dy;
            if (d2 <= best_d2) {
                best_d2 = d2;
                best = i;
            }
        }
    }
    const grabbed = best orelse return;

    var total: usize = 0;
    for (rep.modes, 0..) |m, i| {
        drag.groups[i] = .{ .re = m.re, .im = m.im, .pair = m.oscillatory };
        total += @as(usize, if (m.oscillatory) 2 else 1);
    }
    drag.n_groups = rep.modes.len;
    drag.grabbed = grabbed;

    const desc = currentDesc(showing, current, editor, drag) orelse return;
    // Defensive: the grouped modes must account for the full state
    // dimension, or the rebuilt denominator would change degree.
    if (desc.n != total) return;

    const f = tf.ssToTf(alloc, desc.n, desc.a, desc.b, desc.c, desc.d) catch return;
    const tol = 1e-9 * @max(1.0, @max(f.num.maxAbs(), f.den.maxAbs()));
    drag.num = f.num.trimLeading(tol);
    drag.active = true;
}

/// One frame of an active drag: move the grabbed group to the cursor,
/// rebuild den from the pole set, re-realize, re-analyze. The pole map's
/// viewport stays frozen so the map doesn't slide under the cursor.
fn dragFrame(
    alloc: std.mem.Allocator,
    drag: *Drag,
    rep: *report.Report,
    view: *View,
    t_user: ?f64,
    showing: *Showing,
    pid: *pidpanel.Panel,
) void {
    const vp = view.pole_vp;
    const mp = rl.getMousePosition();
    const gx = std.math.clamp(vp.dataX(@intFromFloat(mp.x)), vp.xmin, vp.xmax);
    const gy = std.math.clamp(vp.dataY(@intFromFloat(mp.y)), vp.ymin, vp.ymax);

    const g = &drag.groups[drag.grabbed];
    g.re = gx;
    // A pair moves as a mirror pair; a real pole slides along the axis only.
    if (g.pair) g.im = @abs(gy);

    const den = tf.polyFromPoleGroups(drag.groups[0..drag.n_groups]);
    if (drag.num.len > den.len) return;

    drag.ss = tf.tfToSs(alloc, drag.num, den) catch return;
    var desc = drag.desc();
    desc.t_end = t_user;
    const next = report.analyze(alloc, desc) catch return;
    rep.deinit();
    rep.* = next;
    showing.* = .dragged;
    pid.dirty = true;

    const pole_vp = view.pole_vp;
    view.* = View.compute(rep, null, null);
    view.pole_vp = pole_vp;
}

fn verdictColor(v: report.Verdict) rl.Color {
    return switch (v) {
        .stable => col_stable,
        .unstable => col_unstable,
        .marginal => col_marginal,
    };
}

fn drawHeader(current: usize, showing: Showing) void {
    var buf: [96]u8 = undefined;
    const title: [:0]const u8 = switch (showing) {
        .preset => std.fmt.bufPrintZ(&buf, "System {d}: {s}", .{ current + 1, systems[current].name }) catch "?",
        .custom => "System 4: Custom",
        .dragged => "Dragged system",
    };
    rl.drawText(title, 40, 20, 24, col_text);
    const hint = "1/2/3 presets   4 editor   5 PID   drag poles on the map";
    rl.drawText(hint, screen_w - 40 - rl.measureText(hint, 16), 26, 16, col_dim);
}

fn drawPoleMap(vp: plot.Viewport, rep: *const report.Report, pid: ?*const pidpanel.Panel) void {
    // Left half-plane tint: the continuous-time stability region.
    const zero_px = vp.px(0);
    const w = zero_px - vp.px0;
    if (w > 0) rl.drawRectangle(vp.px0, vp.py0, w, vp.ph, col_stable_region);

    plot.drawAxes(vp, col_axis, col_dim);
    {
        // Closed-loop poles can wander outside the (possibly frozen)
        // range; clip markers to the panel.
        rl.beginScissorMode(vp.px0, vp.py0, vp.pw, vp.ph);
        defer rl.endScissorMode();
        for (rep.poles) |p| plot.drawPoleMarker(vp, p.re, p.im, 6, col_pole);
        for (rep.zeros) |z| plot.drawZeroMarker(vp, z.re, z.im, 6.0, col_zero);
        if (pid) |pp| {
            for (pp.poles()) |p| plot.drawPoleMarker(vp, p.re, p.im, 5, col_cl_pole);
        }
    }
    plot.drawFrame(vp, col_panel_border);

    rl.drawText("Pole map (s-plane)", vp.px0, vp.py0 - 24, 18, col_text);
    rl.drawText("Re", vp.px0 + vp.pw - 22, vp.py(0) - 14, 10, col_dim);
    rl.drawText("Im", vp.px(0) + 6, vp.py0 + 2, 10, col_dim);

    // Marker legend, top-left inside the map.
    rl.drawText("X pole", vp.px0 + 6, vp.py0 + 6, 12, col_pole);
    rl.drawText("O zero", vp.px0 + 6, vp.py0 + 22, 12, col_zero);
    if (pid != null) {
        rl.drawText("X closed loop", vp.px0 + 6, vp.py0 + 38, 12, col_cl_pole);
    }
}

fn drawResponse(vp: plot.Viewport, rep: *const report.Report, pid: ?*const pidpanel.Panel) void {
    plot.drawAxes(vp, col_axis, col_dim);
    if (pid) |p| {
        plot.drawHLineDashed(vp, 1.0, 6, col_setpoint);
        plot.drawPolyline(vp, rep.t, rep.y, 2.0, col_response_dim);
        plot.drawPolyline(vp, rep.t, p.y[0..], 2.0, col_response);
        rl.drawText("r = 1", vp.px0 + vp.pw - 42, vp.py(1.0) - 16, 12, col_setpoint);
    } else {
        plot.drawPolyline(vp, rep.t, rep.y, 2.0, col_response);
    }
    plot.drawFrame(vp, col_panel_border);

    const title: [:0]const u8 = if (pid != null) "Step response y(t): closed loop (bright) vs open loop (dim)" else "Step response y(t)";
    rl.drawText(title, vp.px0, vp.py0 - 24, 18, col_text);

    var tbuf: [64]u8 = undefined;
    const thint = std.fmt.bufPrintZ(
        &tbuf,
        "t_end = {d:.2}s   Up/Down = time window, R = auto",
        .{rep.t_end},
    ) catch "?";
    rl.drawText(thint, vp.px0 + vp.pw - rl.measureText(thint, 14), vp.py0 - 22, 14, col_dim);

    rl.drawText("t [s]", vp.px0 + vp.pw - 30, vp.py0 + vp.ph + 16, 10, col_dim);
}

/// Seconds with two decimals, or "inf" for marginal modes.
fn fmtSeconds(buf: []u8, v: f64) [:0]const u8 {
    if (!std.math.isFinite(v)) return "inf";
    return std.fmt.bufPrintZ(buf, "{d:.2}s", .{v}) catch "?";
}

/// Append formatted text to buf at pos; silently truncates when full.
fn appendFmt(buf: []u8, pos: *usize, comptime fmt: []const u8, args: anytype) void {
    const s = std.fmt.bufPrint(buf[pos.*..], fmt, args) catch return;
    pos.* += s.len;
}

fn drawReportStrip(rep: *const report.Report, show_footer: bool) void {
    const x: i32 = 40;
    var yy: i32 = 380;

    rl.drawText("Verdict:", x, yy, 20, col_text);
    rl.drawText(rep.verdict.label(), x + 95, yy, 20, verdictColor(rep.verdict));
    if (rep.hidden_instability) {
        rl.drawText(
            "note: unstable mode(s) not visible in y(t) - hidden from this output or too slow for this window",
            x + 250,
            yy + 4,
            14,
            col_marginal,
        );
    }
    yy += 26;

    var line_buf: [192]u8 = undefined;
    var tau_buf: [32]u8 = undefined;
    var ts_buf: [32]u8 = undefined;

    for (rep.modes, 0..) |m, i| {
        const tau_s = fmtSeconds(&tau_buf, m.tau);
        const ts_s = fmtSeconds(&ts_buf, m.ts);
        const line = if (m.oscillatory)
            std.fmt.bufPrintZ(
                &line_buf,
                "Mode {d} (oscillatory):  s = {d:.3} +/- {d:.3}i    wn = {d:.3} rad/s    zeta = {d:.3}    tau = {s}    ts(2%) = {s}",
                .{ i + 1, m.re, m.im, m.wn, m.zeta, tau_s, ts_s },
            ) catch "?"
        else
            std.fmt.bufPrintZ(
                &line_buf,
                "Mode {d} (real):  s = {d:.3}    wn = {d:.3} rad/s    tau = {s}    ts(2%) = {s}",
                .{ i + 1, m.re, m.wn, tau_s, ts_s },
            ) catch "?";
        rl.drawText(line, x, yy, 14, col_text);
        yy += 22;
    }

    // Zeros line: where the numerator vanishes, as O markers on the map.
    {
        var zbuf: [256]u8 = undefined;
        var pos: usize = 0;
        if (rep.zeros.len == 0) {
            appendFmt(&zbuf, &pos, "Zeros: none", .{});
        } else {
            appendFmt(&zbuf, &pos, "Zeros: ", .{});
            for (rep.zeros, 0..) |z, i| {
                if (i > 0) appendFmt(&zbuf, &pos, "    ", .{});
                if (@abs(z.im) > report.sigma_tol) {
                    appendFmt(&zbuf, &pos, "s = {d:.3} {s} {d:.3}i", .{ z.re, if (z.im >= 0) "+" else "-", @abs(z.im) });
                } else {
                    appendFmt(&zbuf, &pos, "s = {d:.3}", .{z.re});
                }
            }
        }
        if (pos >= zbuf.len) pos = zbuf.len - 1;
        zbuf[pos] = 0;
        rl.drawText(zbuf[0..pos :0], x, yy, 14, col_zero);
    }

    if (show_footer) {
        rl.drawText(
            "1 = underdamped   2 = overdamped   3 = unstable   4 = custom editor   5 = PID   drag poles with the mouse",
            x,
            520,
            14,
            col_dim,
        );
    }
}
