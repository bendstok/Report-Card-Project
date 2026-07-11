//! PID playground: closed-loop control of the currently shown plant.
//!
//! Two views of the same loop, on purpose:
//!  - the honest simulation: znumerics' PID_DEO_Sim (derivative on the
//!    measurement, output clamping, anti-windup) driving the ZOH-discretized
//!    plant sample by sample;
//!  - the linear picture: closed-loop poles from the characteristic
//!    polynomial s*den(s) + (Kd*s^2 + Kp*s + Ki)*num(s).
//! When the actuator saturates the two disagree — that disagreement is the
//! lesson this panel exists to show. (Derivative-on-measurement moves the
//! closed-loop zeros, not the poles, so the pole formula matches the sim's
//! structure exactly in the linear regime.)

const std = @import("std");
const rl = @import("raylib");
const znum = @import("znumerics");
const report = @import("report.zig");
const tf = @import("tf.zig");

/// Matches report.analyze's step-response sampling.
pub const samples = 501;

/// Actuator clamp handed to the PID simulator.
pub const u_min = -10.0;
pub const u_max = 10.0;

// Root locus: the whole controller is scaled by k (Kp:Ki:Kd ratio fixed)
// and the closed-loop poles are traced as k sweeps geometrically. k = 1
// is the currently shown closed-loop pole set.
const locus_steps = 100;
const locus_k_min = 0.01;
const locus_k_max = 100.0;
/// Enough for every step's roots at the max char-poly degree seen here.
const locus_cap = locus_steps * 8;

/// f32 is plenty for plotting; halves the Panel's footprint.
pub const LocusPoint = struct { re: f32, im: f32 };

// --- layout: the panel replaces the bottom report strip --------------------

const x_panel: i32 = 40;
const y_panel: i32 = 380;

const track_x: i32 = x_panel + 44;
const track_w: i32 = 260;
const x_value: i32 = track_x + track_w + 16;
const x_info: i32 = 560;

fn rowY(i: usize) i32 {
    return y_panel + 38 + @as(i32, @intCast(i)) * 32;
}

// --- colors (match main palette) --------------------------------------------

const col_text = rl.Color.init(220, 222, 228, 255);
const col_dim = rl.Color.init(150, 153, 165, 255);
const col_track = rl.Color.init(52, 56, 68, 255);
const col_border = rl.Color.init(90, 95, 110, 255);
const col_handle = rl.Color.init(102, 191, 255, 255);
const col_stable = rl.Color.init(0, 228, 48, 255);
const col_unstable = rl.Color.init(230, 41, 55, 255);
const col_marginal = rl.Color.init(255, 161, 0, 255);

const Slider = struct {
    label: [:0]const u8,
    min: f64,
    max: f64,
};

const sliders = [_]Slider{
    .{ .label = "Kp", .min = 0, .max = 20 },
    .{ .label = "Ki", .min = 0, .max = 10 },
    .{ .label = "Kd", .min = 0, .max = 5 },
};

pub const Panel = struct {
    active: bool = false,
    kp: f64 = 1.0,
    ki: f64 = 0.0,
    kd: f64 = 0.0,

    /// Slider index currently mouse-dragged, or -1.
    dragging: i32 = -1,
    /// Set by main whenever the plant or gains change; cleared on recompute.
    dirty: bool = true,
    /// Last recompute succeeded; nothing is drawn from stale state.
    ok: bool = false,

    // Closed-loop results, all in fixed storage.
    y: [samples]f64 = [_]f64{0} ** samples,
    cl_poles: [tf.max_deg]tf.Complex = undefined,
    n_cl: usize = 0,
    cl_verdict: report.Verdict = .stable,
    saturated: bool = false,

    // Root locus (key L).
    locus_on: bool = false,
    locus_pts: [locus_cap]LocusPoint = undefined,
    n_locus: usize = 0,

    fn valPtr(self: *Panel, i: usize) *f64 {
        return switch (i) {
            0 => &self.kp,
            1 => &self.ki,
            else => &self.kd,
        };
    }

    fn val(self: *const Panel, i: usize) f64 {
        return switch (i) {
            0 => self.kp,
            1 => self.ki,
            else => self.kd,
        };
    }

    /// Mouse handling for the sliders. Call once per frame while active.
    pub fn update(self: *Panel) void {
        const mp = rl.getMousePosition();
        const mx: i32 = @intFromFloat(mp.x);
        const my: i32 = @intFromFloat(mp.y);

        if (rl.isMouseButtonPressed(.left)) {
            for (sliders, 0..) |_, i| {
                const ty = rowY(i);
                if (mx >= track_x - 8 and mx < track_x + track_w + 8 and
                    my >= ty - 6 and my < ty + 18)
                {
                    self.dragging = @intCast(i);
                }
            }
        }
        if (!rl.isMouseButtonDown(.left)) self.dragging = -1;

        if (self.dragging >= 0) {
            const i: usize = @intCast(self.dragging);
            const s = sliders[i];
            const f = std.math.clamp(
                @as(f64, @floatFromInt(mx - track_x)) / @as(f64, @floatFromInt(track_w)),
                0.0,
                1.0,
            );
            const v = s.min + f * (s.max - s.min);
            if (v != self.valPtr(i).*) {
                self.valPtr(i).* = v;
                self.dirty = true;
            }
        }
    }

    /// Re-run the closed-loop sim and the linear pole computation for the
    /// given plant. Allocation happens only here (on gain/plant changes),
    /// never on quiet frames.
    pub fn recompute(self: *Panel, alloc: std.mem.Allocator, sys: report.SysDesc, t_end: f64) void {
        self.dirty = false;
        self.ok = false;
        self.recomputeInner(alloc, sys, t_end) catch return;
        self.ok = true;
    }

    fn recomputeInner(self: *Panel, alloc: std.mem.Allocator, sys: report.SysDesc, t_end: f64) !void {
        const n = sys.n;
        std.debug.assert(n >= 1 and n <= tf.max_deg);
        const dt = t_end / @as(f64, @floatFromInt(samples - 1));

        // --- honest sim: znumerics' lsimFn with the PID in the loop -------
        // lsimFn ZOH-discretizes a clone and runs sample by sample; the
        // clamp option keeps unstable closed loops finite (NaN -> 0,
        // +/-1e30), matching what the old hand-rolled loop did.
        var ss = try znum.StateSpace.fromSlices(alloc, .Continuous, 0.0, n, sys.a, sys.b, sys.c, sys.d);
        defer ss.deinit();

        var pid = znum.control.PID_DEO_Sim{
            .K_p = self.kp,
            .K_i = self.ki,
            .K_d = self.kd,
            .dt = dt,
            .clamp_min = u_min,
            .clamp_max = u_max,
            .prev_e = 0.0,
            .integral = 0.0,
            .prev_y = 0.0,
        };

        // The controller runs inside the input callback: it reads the
        // measurement off the pre-step state, stores the panel's y, and
        // returns u. With D != 0 the loop y -> PID -> u -> y would be
        // algebraic; keeping the previous input in the feedthrough term
        // breaks it with one sample of delay. Presets and most customs
        // have D = 0. (LsimResult.y uses the current-u feedthrough, so
        // the panel keeps its own y written here.)
        const Ctx = struct {
            pid: *znum.control.PID_DEO_Sim,
            sys: *const report.SysDesc,
            panel: *Panel,
            u_prev: f64 = 0.0,

            fn input(ctx: *@This(), k: usize, t_k: f64, x: znum.Vec) f64 {
                _ = t_k;
                const s = ctx.sys;
                var yk = s.d * ctx.u_prev;
                for (0..s.n) |i| yk += s.c[i] * x.atUnsafe(i);
                if (std.math.isNan(yk)) yk = 0.0;
                yk = std.math.clamp(yk, -1e30, 1e30);
                ctx.panel.y[k] = yk;

                const uk = ctx.pid.compute(1.0, yk);
                if (uk >= u_max - 1e-12 or uk <= u_min + 1e-12) ctx.panel.saturated = true;
                ctx.u_prev = uk;
                return uk;
            }
        };

        self.saturated = false;
        var ctx = Ctx{ .pid = &pid, .sys = &sys, .panel = self };
        var res = try znum.lsim.lsimFn(alloc, ss, dt, samples, &ctx, Ctx.input, null, .{ .clamp = 1e30 });
        res.deinit();

        // --- linear closed-loop poles --------------------------------------
        const f = try tf.ssToTf(alloc, n, sys.a, sys.b, sys.c, sys.d);
        const tol = 1e-9 * @max(1.0, @max(f.num.maxAbs(), f.den.maxAbs()));
        const num = f.num.trimLeading(tol);

        const s_poly = tf.Poly.init(&.{ 1, 0 });
        const c_poly = tf.Poly.init(&.{ self.kd, self.kp, self.ki }).trimLeading(0.0);
        const char = s_poly.mul(f.den).add(c_poly.mul(num)).trimLeading(tol);

        const rts = try tf.roots(alloc, char);
        defer alloc.free(rts);
        self.n_cl = @min(rts.len, self.cl_poles.len);
        @memcpy(self.cl_poles[0..self.n_cl], rts[0..self.n_cl]);

        var max_sigma = -std.math.inf(f64);
        for (rts) |p| max_sigma = @max(max_sigma, p.re);
        self.cl_verdict = if (max_sigma > report.sigma_tol)
            .unstable
        else if (max_sigma < -report.sigma_tol)
            .stable
        else
            .marginal;

        // --- root locus ------------------------------------------------------
        // Same char poly with the controller scaled by k. Zero controller
        // means char(k) = s*den for every k: nothing to trace.
        self.n_locus = 0;
        if (self.locus_on and c_poly.maxAbs() > 0) {
            const s_den = s_poly.mul(f.den);
            const ratio = std.math.pow(
                f64,
                locus_k_max / locus_k_min,
                1.0 / @as(f64, @floatFromInt(locus_steps - 1)),
            );
            var k: f64 = locus_k_min;
            for (0..locus_steps) |_| {
                const char_k = s_den.add(c_poly.scale(k).mul(num)).trimLeading(tol);
                k *= ratio;
                if (char_k.degree() == 0) continue;
                const lr = try tf.roots(alloc, char_k);
                defer alloc.free(lr);
                for (lr) |p| {
                    if (self.n_locus >= self.locus_pts.len) break;
                    self.locus_pts[self.n_locus] = .{
                        .re = @floatCast(p.re),
                        .im = @floatCast(p.im),
                    };
                    self.n_locus += 1;
                }
            }
        }
    }

    /// Closed-loop pole markers for the pole map (drawn by main next to the
    /// open-loop X's).
    pub fn poles(self: *const Panel) []const tf.Complex {
        if (!self.ok) return &.{};
        return self.cl_poles[0..self.n_cl];
    }

    /// Root-locus dots for the pole map; empty when off or stale.
    pub fn locusPoints(self: *const Panel) []const LocusPoint {
        if (!self.ok or !self.locus_on) return &.{};
        return self.locus_pts[0..self.n_locus];
    }

    fn verdictColor(v: report.Verdict) rl.Color {
        return switch (v) {
            .stable => col_stable,
            .unstable => col_unstable,
            .marginal => col_marginal,
        };
    }

    /// Append formatted text to buf at pos; silently truncates when full.
    fn append(buf: []u8, pos: *usize, comptime fmt: []const u8, args: anytype) void {
        const s = std.fmt.bufPrint(buf[pos.*..], fmt, args) catch return;
        pos.* += s.len;
    }

    pub fn draw(self: *const Panel) void {
        rl.drawText("PID playground", x_panel, y_panel, 20, col_text);
        rl.drawText(
            "plant = system shown above    r = 1    u clamped to [-10, 10]    5 = close",
            x_panel + 200,
            y_panel + 4,
            14,
            col_dim,
        );

        // Sliders.
        var vbuf: [32]u8 = undefined;
        for (sliders, 0..) |s, i| {
            const ty = rowY(i);
            rl.drawText(s.label, x_panel, ty - 2, 16, col_text);

            rl.drawRectangle(track_x, ty + 3, track_w, 6, col_track);
            rl.drawRectangleLines(track_x, ty + 3, track_w, 6, col_border);

            const f = (self.val(i) - s.min) / (s.max - s.min);
            const hx = track_x + @as(i32, @intFromFloat(f * @as(f64, @floatFromInt(track_w))));
            rl.drawRectangle(hx - 3, ty - 3, 6, 18, col_handle);

            const vs = std.fmt.bufPrintZ(&vbuf, "{d:.2}", .{self.val(i)}) catch "?";
            rl.drawText(vs, x_value, ty - 2, 16, col_text);
        }

        rl.drawText(
            "drag the sliders; open-loop response is drawn dim, closed-loop bright",
            x_panel,
            rowY(sliders.len) + 6,
            14,
            col_dim,
        );
        rl.drawText(
            if (self.locus_on)
                "L = hide root locus (controller scaled x0.01 .. x100, ratios fixed; bright X's sit at x1)"
            else
                "L = show root locus on the pole map",
            x_panel,
            rowY(sliders.len) + 26,
            14,
            col_dim,
        );

        // Info column.
        var iy: i32 = rowY(0) - 2;
        if (!self.ok) {
            rl.drawText("analysis failed for these gains", x_info, iy, 16, col_unstable);
            return;
        }

        rl.drawText("Closed-loop verdict:", x_info, iy, 16, col_text);
        rl.drawText(
            self.cl_verdict.label(),
            x_info + 180,
            iy,
            16,
            verdictColor(self.cl_verdict),
        );
        iy += 28;

        var pbuf: [256]u8 = undefined;
        var pos: usize = 0;
        append(&pbuf, &pos, "Closed-loop poles (linear): ", .{});
        for (self.cl_poles[0..self.n_cl], 0..) |p, i| {
            if (i > 0) append(&pbuf, &pos, "   ", .{});
            if (@abs(p.im) > report.sigma_tol) {
                append(&pbuf, &pos, "{d:.3}{s}{d:.3}i", .{ p.re, if (p.im >= 0) "+" else "-", @abs(p.im) });
            } else {
                append(&pbuf, &pos, "{d:.3}", .{p.re});
            }
        }
        if (pos >= pbuf.len) pos = pbuf.len - 1;
        pbuf[pos] = 0;
        rl.drawText(pbuf[0..pos :0], x_info, iy, 13, col_text);
        iy += 26;

        if (self.saturated) {
            rl.drawText(
                "actuator saturating: the sim (with clamp + anti-windup) and",
                x_info,
                iy,
                14,
                col_marginal,
            );
            rl.drawText(
                "the linear pole picture disagree - that gap is the lesson",
                x_info,
                iy + 18,
                14,
                col_marginal,
            );
        } else {
            rl.drawText(
                "actuator within limits: the linear poles describe the sim",
                x_info,
                iy,
                14,
                col_dim,
            );
        }
        iy += 42;

        rl.drawText(
            "note: this PID differentiates the measurement, not the error;",
            x_info,
            iy,
            13,
            col_dim,
        );
        rl.drawText(
            "same poles as textbook PID, different zeros (no derivative kick)",
            x_info,
            iy + 16,
            13,
            col_dim,
        );
    }
};
