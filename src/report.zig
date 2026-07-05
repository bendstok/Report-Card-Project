//! Pure math for the system report card. No raylib imports — this file is
//! the root of the `zig build test` module and must stay graphics-free.

const std = @import("std");
const znum = @import("znumerics");
const tf = @import("tf.zig");

const Mat = znum.Mat;
pub const Complex = std.math.Complex(f64);

/// Tolerance for treating a pole's real part as zero (marginal stability).
pub const sigma_tol = 1e-9;

pub const Verdict = enum {
    stable,
    unstable,
    marginal,

    pub fn label(self: Verdict) [:0]const u8 {
        return switch (self) {
            .stable => "STABLE",
            .unstable => "UNSTABLE",
            .marginal => "MARGINAL",
        };
    }
};

/// One mode: a single real pole, or a conjugate pair reported together.
pub const Mode = struct {
    re: f64,
    im: f64, // absolute imaginary part; 0 for real modes
    oscillatory: bool,
    wn: f64, // natural frequency |lambda|
    zeta: f64, // damping ratio; only meaningful when wn > 0
    tau: f64, // time constant 1/|sigma|; inf when sigma == 0
    ts: f64, // 2% settling time ~= 4/|sigma|; inf when sigma == 0
};

/// Hardcoded SISO system description. `a` is row-major n*n.
pub const SysDesc = struct {
    name: [:0]const u8,
    n: usize,
    a: []const f64,
    b: []const f64,
    c: []const f64,
    d: f64 = 0.0,
    /// Optional plot-time override. When null, the horizon is chosen
    /// automatically as 5 * max(tau) over stable modes, clamped to [1, 100].
    t_end: ?f64 = null,
};

pub const Report = struct {
    alloc: std.mem.Allocator,
    verdict: Verdict,
    poles: []Complex,
    /// Transfer-function zeros (roots of the numerator seen from this
    /// output). Empty when the numerator is constant. A zero sitting on
    /// an unstable pole is a pole-zero cancellation made visible.
    zeros: []Complex,
    modes: []Mode,
    t: []f64,
    y: []f64,
    t_end: f64,
    /// True when the verdict is UNSTABLE but the sampled response shows no
    /// divergence: the unstable mode(s) are hidden from this output
    /// (unobservable / unexcited) or grow too slowly for the plotted window.
    hidden_instability: bool,

    pub fn deinit(self: *Report) void {
        self.alloc.free(self.poles);
        self.alloc.free(self.zeros);
        self.alloc.free(self.modes);
        self.alloc.free(self.t);
        self.alloc.free(self.y);
        self.* = undefined;
    }
};

pub fn analyze(alloc: std.mem.Allocator, sys: SysDesc) !Report {
    const n = sys.n;
    std.debug.assert(sys.a.len == n * n);
    std.debug.assert(sys.b.len == n and sys.c.len == n);

    // --- poles via shifted QR (real Schur form) --------------------------
    // qrAlgorithmComplex is called directly instead of eigenvaluesComplex:
    // the Arnoldi reduction step can break down early for some inputs.
    const poles = blk: {
        var A = try Mat.initZero(alloc, n, n);
        defer A.deinit();
        for (0..n) |i| {
            for (0..n) |j| try A.set(i, j, sys.a[i * n + j]);
        }
        break :blk try znum.eigen.qrAlgorithmComplex(alloc, A, 1000, 1e-12, null);
    };
    errdefer alloc.free(poles);

    const modes = try buildModes(alloc, poles);
    errdefer alloc.free(modes);

    // --- zeros: roots of the TF numerator ---------------------------------
    // Strictly proper systems leave the leading numerator coefficients only
    // approximately zero, so trim with a tolerance scaled to the TF.
    const zeros = blk: {
        const f = try tf.ssToTf(alloc, n, sys.a, sys.b, sys.c, sys.d);
        const tol = 1e-9 * @max(1.0, @max(f.num.maxAbs(), f.den.maxAbs()));
        break :blk try tf.roots(alloc, f.num.trimLeading(tol));
    };
    errdefer alloc.free(zeros);

    // --- verdict ---------------------------------------------------------
    var max_sigma = -std.math.inf(f64);
    for (poles) |p| max_sigma = @max(max_sigma, p.re);
    const verdict: Verdict = if (max_sigma > sigma_tol)
        .unstable
    else if (max_sigma < -sigma_tol)
        .stable
    else
        .marginal;

    // --- time horizon: 5 * max(tau) over stable modes, clamped [1, 100] ---
    var max_tau: f64 = 0.0;
    for (modes) |m| {
        if (m.re < -sigma_tol) max_tau = @max(max_tau, m.tau);
    }
    var t_end: f64 = if (max_tau > 0.0) 5.0 * max_tau else 10.0;
    t_end = std.math.clamp(t_end, 1.0, 100.0);
    if (sys.t_end) |te| t_end = std.math.clamp(te, 0.1, 1000.0);

    // --- step response: ZOH-discretize once, then iterate -----------------
    const steps = 500;
    const samples = steps + 1;
    const dt = t_end / @as(f64, @floatFromInt(steps));

    const t = try alloc.alloc(f64, samples);
    errdefer alloc.free(t);
    const y = try alloc.alloc(f64, samples);
    errdefer alloc.free(y);

    {
        var ss = try znum.StateSpace.initContinuous(alloc, n);
        defer ss.deinit();
        for (0..n) |i| {
            for (0..n) |j| try ss.A.set(i, j, sys.a[i * n + j]);
            try ss.B.set(i, sys.b[i]);
            try ss.C.set(i, sys.c[i]);
        }
        try ss.D.set(0, sys.d);

        // In-place: ss.A becomes Ad, ss.B becomes Bd.
        try znum.signal.cont2discrete(alloc, &ss, dt);

        const x = try alloc.alloc(f64, n);
        defer alloc.free(x);
        const xn = try alloc.alloc(f64, n);
        defer alloc.free(xn);
        @memset(x, 0.0);

        for (0..samples) |k| {
            t[k] = dt * @as(f64, @floatFromInt(k));

            // y = C*x + D*u with u = 1. Clamp so unstable systems stay finite.
            var yk = sys.d;
            for (0..n) |i| yk += sys.c[i] * x[i];
            if (std.math.isNan(yk)) {
                yk = 0.0;
            } else {
                yk = std.math.clamp(yk, -1e30, 1e30);
            }
            y[k] = yk;

            // x_{k+1} = Ad*x_k + Bd*u with u = 1
            for (0..n) |i| {
                var v = ss.B.atUnsafe(i);
                for (0..n) |j| v += ss.A.atUnsafe(i, j) * x[j];
                if (std.math.isNan(v)) v = 0.0;
                xn[i] = std.math.clamp(v, -1e30, 1e30);
            }
            @memcpy(x, xn);
        }
    }

    // A visible unstable mode makes the tail of the response dwarf the
    // head; if it doesn't, the instability never reached the output.
    var hidden_instability = false;
    if (verdict == .unstable) {
        const half = samples / 2;
        var head_max: f64 = 0.0;
        for (y[0..half]) |v| head_max = @max(head_max, @abs(v));
        var tail_max: f64 = 0.0;
        for (y[half..]) |v| tail_max = @max(tail_max, @abs(v));
        hidden_instability = tail_max <= 2.0 * head_max + 1e-9;
    }

    return .{
        .alloc = alloc,
        .verdict = verdict,
        .poles = poles,
        .zeros = zeros,
        .modes = modes,
        .t = t,
        .y = y,
        .t_end = t_end,
        .hidden_instability = hidden_instability,
    };
}

/// Group conjugate pairs (same re, opposite im) into single oscillatory
/// modes and compute per-mode metrics.
fn buildModes(alloc: std.mem.Allocator, poles: []const Complex) ![]Mode {
    var modes = try alloc.alloc(Mode, poles.len);
    errdefer alloc.free(modes);
    const used = try alloc.alloc(bool, poles.len);
    defer alloc.free(used);
    @memset(used, false);

    var count: usize = 0;
    for (poles, 0..) |p, i| {
        if (used[i]) continue;
        used[i] = true;

        var oscillatory = false;
        if (@abs(p.im) > sigma_tol) {
            const mag = @sqrt(p.re * p.re + p.im * p.im);
            const pair_tol = 1e-6 * @max(1.0, mag);
            for (poles[i + 1 ..], i + 1..) |q, j| {
                if (used[j]) continue;
                if (@abs(q.re - p.re) < pair_tol and @abs(q.im + p.im) < pair_tol) {
                    used[j] = true;
                    oscillatory = true;
                    break;
                }
            }
        }

        const sigma = p.re;
        const im = @abs(p.im);
        const wn = @sqrt(sigma * sigma + im * im);
        const abs_sigma = @abs(sigma);

        modes[count] = .{
            .re = sigma,
            .im = im,
            .oscillatory = oscillatory,
            .wn = wn,
            .zeta = if (wn > sigma_tol) -sigma / wn else 0.0,
            .tau = if (abs_sigma > sigma_tol) 1.0 / abs_sigma else std.math.inf(f64),
            .ts = if (abs_sigma > sigma_tol) 4.0 / abs_sigma else std.math.inf(f64),
        };
        count += 1;
    }

    if (count == modes.len) return modes;
    return try alloc.realloc(modes, count);
}

// ---------------------------------------------------------------------------
// Known-answer tests (spec section 5), tolerance 1e-4.
// ---------------------------------------------------------------------------

const testing = std.testing;

const sys_underdamped = SysDesc{
    .name = "Underdamped",
    .n = 2,
    .a = &.{ 0, 1, -2, -0.6 },
    .b = &.{ 0, 1 },
    .c = &.{ 1, 0 },
};

const sys_overdamped = SysDesc{
    .name = "Overdamped",
    .n = 2,
    .a = &.{ 0, 1, -2, -3 },
    .b = &.{ 0, 1 },
    .c = &.{ 1, 0 },
};

const sys_unstable = SysDesc{
    .name = "Unstable",
    .n = 2,
    .a = &.{ 0, 1, 2, -0.5 },
    .b = &.{ 0, 1 },
    .c = &.{ 1, 0 },
};

test "underdamped: poles, wn, zeta, verdict, overshoot" {
    var r = try analyze(testing.allocator, sys_underdamped);
    defer r.deinit();

    try testing.expectEqual(Verdict.stable, r.verdict);
    try testing.expectEqual(@as(usize, 1), r.modes.len);

    const m = r.modes[0];
    try testing.expect(m.oscillatory);
    try testing.expectApproxEqAbs(-0.3, m.re, 1e-4);
    try testing.expectApproxEqAbs(1.38203, m.im, 1e-4);
    try testing.expectApproxEqAbs(1.41421, m.wn, 1e-4);
    try testing.expectApproxEqAbs(0.21213, m.zeta, 1e-4);
    try testing.expectApproxEqAbs(1.0 / 0.3, m.tau, 1e-4);
    try testing.expectApproxEqAbs(4.0 / 0.3, m.ts, 1e-4);

    // DC gain is 0.5: the underdamped response must overshoot it,
    // then settle back near it by t_end (= 5 tau).
    var peak = -std.math.inf(f64);
    for (r.y) |v| peak = @max(peak, v);
    try testing.expect(peak > 0.55);
    try testing.expectApproxEqAbs(0.5, r.y[r.y.len - 1], 0.02);
}

test "overdamped: two real modes, no overshoot" {
    var r = try analyze(testing.allocator, sys_overdamped);
    defer r.deinit();

    try testing.expectEqual(Verdict.stable, r.verdict);
    try testing.expectEqual(@as(usize, 2), r.modes.len);
    for (r.modes) |m| try testing.expect(!m.oscillatory);

    var lo = r.modes[0].re;
    var hi = r.modes[1].re;
    if (lo > hi) std.mem.swap(f64, &lo, &hi);
    try testing.expectApproxEqAbs(-2.0, lo, 1e-4);
    try testing.expectApproxEqAbs(-1.0, hi, 1e-4);

    // Step response must never exceed the DC gain of 0.5.
    for (r.y) |v| try testing.expect(v <= 0.5 + 1e-6);
    try testing.expectApproxEqAbs(0.5, r.y[r.y.len - 1], 0.02);
}

test "unstable: poles, verdict, divergent but finite response" {
    var r = try analyze(testing.allocator, sys_unstable);
    defer r.deinit();

    try testing.expectEqual(Verdict.unstable, r.verdict);
    try testing.expectEqual(@as(usize, 2), r.modes.len);

    var has_pos = false;
    var has_neg = false;
    for (r.poles) |p| {
        try testing.expectApproxEqAbs(0.0, p.im, 1e-6);
        if (@abs(p.re - 1.18614) < 1e-4) has_pos = true;
        if (@abs(p.re + 1.68614) < 1e-4) has_neg = true;
    }
    try testing.expect(has_pos and has_neg);

    // Diverges, but every sample stays finite (plot clamps, never crashes).
    try testing.expect(@abs(r.y[r.y.len - 1]) > 5.0);
    for (r.y) |v| try testing.expect(std.math.isFinite(v));

    // The divergence is visible in y(t), so it is not a hidden instability.
    try testing.expect(!r.hidden_instability);
}

test "hidden instability: unstable mode invisible in the output" {
    // x1 is a stable first-order lag driven by u; x2 is an unstable state
    // that is neither excited by B nor measured by C. y(t) settles at 1,
    // yet A has an eigenvalue at +1.
    const sys = SysDesc{
        .name = "Hidden",
        .n = 2,
        .a = &.{ -1, 0, 0, 1 },
        .b = &.{ 1, 0 },
        .c = &.{ 1, 0 },
    };
    var r = try analyze(testing.allocator, sys);
    defer r.deinit();

    try testing.expectEqual(Verdict.unstable, r.verdict);
    try testing.expect(r.hidden_instability);
    try testing.expectApproxEqAbs(1.0, r.y[r.y.len - 1], 0.02);
}

test "t_end override is respected" {
    var sys = sys_underdamped;
    sys.t_end = 3.0;
    var r = try analyze(testing.allocator, sys);
    defer r.deinit();

    try testing.expectApproxEqAbs(3.0, r.t_end, 1e-12);
    try testing.expectApproxEqAbs(3.0, r.t[r.t.len - 1], 1e-9);
}

test "zeros: presets have none; C = [1, 1] puts one at -1" {
    var r = try analyze(testing.allocator, sys_underdamped);
    defer r.deinit();
    try testing.expectEqual(@as(usize, 0), r.zeros.len);

    var sys = sys_underdamped;
    sys.c = &.{ 1, 1 };
    var r2 = try analyze(testing.allocator, sys);
    defer r2.deinit();
    try testing.expectEqual(@as(usize, 1), r2.zeros.len);
    try testing.expectApproxEqAbs(-1.0, r2.zeros[0].re, 1e-9);
    try testing.expectApproxEqAbs(0.0, r2.zeros[0].im, 1e-9);
}

test "zeros: hidden instability shows the cancelling zero at +1" {
    const sys = SysDesc{
        .name = "Hidden",
        .n = 2,
        .a = &.{ -1, 0, 0, 1 },
        .b = &.{ 1, 0 },
        .c = &.{ 1, 0 },
    };
    var r = try analyze(testing.allocator, sys);
    defer r.deinit();

    try testing.expectEqual(@as(usize, 1), r.zeros.len);
    try testing.expectApproxEqAbs(1.0, r.zeros[0].re, 1e-9);
}

// Pull in tf.zig's own known-answer tests under `zig build test`.
test {
    _ = tf;
}

test "marginal: undamped oscillator" {
    const sys = SysDesc{
        .name = "Oscillator",
        .n = 2,
        .a = &.{ 0, 1, -1, 0 },
        .b = &.{ 0, 1 },
        .c = &.{ 1, 0 },
    };
    var r = try analyze(testing.allocator, sys);
    defer r.deinit();

    try testing.expectEqual(Verdict.marginal, r.verdict);
    try testing.expectEqual(@as(usize, 1), r.modes.len);

    const m = r.modes[0];
    try testing.expect(m.oscillatory);
    try testing.expectApproxEqAbs(1.0, m.wn, 1e-4);
    try testing.expectApproxEqAbs(0.0, m.zeta, 1e-4);
    try testing.expect(std.math.isInf(m.tau));
}
