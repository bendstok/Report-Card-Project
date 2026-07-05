//! Transfer-function math on fixed-capacity polynomials. No raylib imports;
//! allocators are only handed to znumerics calls whose results are copied
//! into caller-owned storage before returning.
//!
//! Polynomials store coefficients in DESCENDING powers, matching the
//! convention of znumerics' tf2ss / charPoly:
//!   coef[0]*s^deg + coef[1]*s^(deg-1) + ... + coef[deg].

const std = @import("std");
const znum = @import("znumerics");

const Mat = znum.Mat;
pub const Complex = std.math.Complex(f64);

/// Highest polynomial degree representable. Systems passed through here
/// must have state dimension <= max_deg too.
pub const max_deg = 10;
pub const cap = max_deg + 1;

pub const Poly = struct {
    coef: [cap]f64 = [_]f64{0} ** cap,
    len: usize = 1, // number of stored coefficients = degree + 1

    pub fn init(coeffs: []const f64) Poly {
        std.debug.assert(coeffs.len >= 1 and coeffs.len <= cap);
        var p = Poly{ .len = coeffs.len };
        @memcpy(p.coef[0..coeffs.len], coeffs);
        return p;
    }

    pub fn slice(self: *const Poly) []const f64 {
        return self.coef[0..self.len];
    }

    pub fn degree(self: *const Poly) usize {
        return self.len - 1;
    }

    /// Product a*b. Asserts the result fits in `cap` coefficients.
    pub fn mul(a: Poly, b: Poly) Poly {
        const rlen = a.len + b.len - 1;
        std.debug.assert(rlen <= cap);
        var r = Poly{ .len = rlen };
        for (a.slice(), 0..) |av, i| {
            for (b.slice(), 0..) |bv, j| r.coef[i + j] += av * bv;
        }
        return r;
    }

    /// Sum a+b, aligned at the constant term (the shorter polynomial is
    /// implicitly padded with leading zeros).
    pub fn add(a: Poly, b: Poly) Poly {
        const rlen = @max(a.len, b.len);
        var r = Poly{ .len = rlen };
        for (a.slice(), 0..) |av, i| r.coef[rlen - a.len + i] += av;
        for (b.slice(), 0..) |bv, i| r.coef[rlen - b.len + i] += bv;
        return r;
    }

    pub fn scale(self: Poly, k: f64) Poly {
        var r = self;
        for (r.coef[0..r.len]) |*v| v.* *= k;
        return r;
    }

    /// Strip leading coefficients with |c| <= tol; at least one coefficient
    /// always remains. Relative degree makes the leading numerator terms of
    /// ssToTf only approximately zero, so callers pass a tolerance scaled
    /// to the polynomials' magnitude.
    pub fn trimLeading(self: Poly, tol: f64) Poly {
        var start: usize = 0;
        while (start + 1 < self.len and @abs(self.coef[start]) <= tol) start += 1;
        return init(self.coef[start..self.len]);
    }

    /// Largest |coefficient|, for tolerance scaling.
    pub fn maxAbs(self: *const Poly) f64 {
        var m: f64 = 0;
        for (self.slice()) |v| m = @max(m, @abs(v));
        return m;
    }
};

/// One entry of a pole set: a single real pole (`pair == false`, im
/// ignored) or a complex-conjugate pair re +/- im*i (`pair == true`).
pub const PoleGroup = struct {
    re: f64,
    im: f64 = 0,
    pair: bool = false,
};

/// Monic polynomial whose roots are the given pole set. A pair contributes
/// the real quadratic (s - re)^2 + im^2 even when im == 0, so the total
/// degree never changes while a pair is dragged onto the real axis.
pub fn polyFromPoleGroups(groups: []const PoleGroup) Poly {
    var p = Poly.init(&.{1});
    for (groups) |g| {
        p = if (g.pair)
            p.mul(Poly.init(&.{ 1, -2 * g.re, g.re * g.re + g.im * g.im }))
        else
            p.mul(Poly.init(&.{ 1, -g.re }));
    }
    return p;
}

pub const Tf = struct {
    /// Same length as den, NOT trimmed: leading entries are ~0 for strictly
    /// proper systems. Trim with a scaled tolerance before rooting.
    num: Poly,
    /// Monic characteristic polynomial of A.
    den: Poly,
};

/// State space -> transfer function using the same domain-independent
/// identity as znumerics' ss2tf (which is written for z but holds in s):
///   den = charPoly(A),  num = charPoly(A - B*C^T) + (D - 1)*den.
pub fn ssToTf(
    alloc: std.mem.Allocator,
    n: usize,
    a: []const f64,
    b: []const f64,
    c: []const f64,
    d: f64,
) !Tf {
    std.debug.assert(n >= 1 and n <= max_deg);
    std.debug.assert(a.len == n * n and b.len == n and c.len == n);

    var A = try Mat.initZero(alloc, n, n);
    defer A.deinit();
    var M = try Mat.initZero(alloc, n, n);
    defer M.deinit();
    for (0..n) |i| {
        for (0..n) |j| {
            try A.set(i, j, a[i * n + j]);
            try M.set(i, j, a[i * n + j] - b[i] * c[j]);
        }
    }

    var den = Poly{ .len = n + 1 };
    try znum.mat.charPoly(alloc, A, den.coef[0 .. n + 1]);
    var num = Poly{ .len = n + 1 };
    try znum.mat.charPoly(alloc, M, num.coef[0 .. n + 1]);
    for (0..n + 1) |i| num.coef[i] += (d - 1.0) * den.coef[i];

    return .{ .num = num, .den = den };
}

/// Roots of p as the eigenvalues of its companion matrix, found with the
/// same shifted-QR routine the app already uses for poles. Caller frees.
/// A degree-0 polynomial has no roots: returns an empty slice.
pub fn roots(alloc: std.mem.Allocator, p: Poly) ![]Complex {
    const deg = p.degree();
    if (deg == 0) return try alloc.alloc(Complex, 0);
    std.debug.assert(@abs(p.coef[0]) > 0);

    var A = try Mat.initZero(alloc, deg, deg);
    defer A.deinit();
    for (0..deg) |j| try A.set(0, j, -p.coef[j + 1] / p.coef[0]);
    for (1..deg) |i| try A.set(i, i - 1, 1.0);

    return znum.eigen.qrAlgorithmComplex(alloc, A, 1000, 1e-12, null);
}

/// A state-space realization in caller-owned fixed arrays: nothing
/// allocated outlives tfToSs, so the result can back a SysDesc whose
/// slices point at stable storage.
pub const SsFixed = struct {
    n: usize = 0,
    a: [max_deg * max_deg]f64 = [_]f64{0} ** (max_deg * max_deg),
    b: [max_deg]f64 = [_]f64{0} ** max_deg,
    c: [max_deg]f64 = [_]f64{0} ** max_deg,
    d: f64 = 0,
};

/// TF -> state space via znumerics' continuous tf2ss (companion form),
/// copied into fixed arrays. Requires deg(num) <= deg(den) and a nonzero
/// leading denominator coefficient.
pub fn tfToSs(alloc: std.mem.Allocator, num: Poly, den: Poly) !SsFixed {
    std.debug.assert(num.len <= den.len);
    std.debug.assert(den.degree() >= 1 and den.degree() <= max_deg);

    var ss = try znum.signal.tf2ss(alloc, num.slice(), den.slice());
    defer ss.deinit();

    const n = ss.A.rows;
    var out = SsFixed{ .n = n };
    for (0..n) |i| {
        for (0..n) |j| out.a[i * n + j] = ss.A.atUnsafe(i, j);
        out.b[i] = ss.B.atUnsafe(i);
        out.c[i] = ss.C.atUnsafe(i);
    }
    out.d = ss.D.atUnsafe(0);
    return out;
}

// ---------------------------------------------------------------------------
// Known-answer tests, chained into `zig build test` from report.zig.
// ---------------------------------------------------------------------------

const testing = std.testing;

fn expectPoly(expected: []const f64, p: Poly, tol: f64) !void {
    try testing.expectEqual(expected.len, p.len);
    for (expected, p.slice()) |e, v| try testing.expectApproxEqAbs(e, v, tol);
}

test "poly: mul, add, scale, trim" {
    // (s + 1)(s + 2) = s^2 + 3s + 2
    const q = Poly.init(&.{ 1, 1 }).mul(Poly.init(&.{ 1, 2 }));
    try expectPoly(&.{ 1, 3, 2 }, q, 1e-12);

    // (s^2 + 3s + 2) + (s + 1) = s^2 + 4s + 3, alignment at the constant term
    const s = q.add(Poly.init(&.{ 1, 1 }));
    try expectPoly(&.{ 1, 4, 3 }, s, 1e-12);

    try expectPoly(&.{ 2, 8, 6 }, s.scale(2.0), 1e-12);

    // Leading near-zeros are stripped, exact values kept.
    const t = Poly.init(&.{ 1e-14, 0, 2, 5 }).trimLeading(1e-9);
    try expectPoly(&.{ 2, 5 }, t, 1e-12);

    // Never trims to nothing.
    const z = Poly.init(&.{ 0, 0 }).trimLeading(1e-9);
    try expectPoly(&.{0}, z, 1e-12);
}

test "polyFromPoleGroups: reals, pairs, and a pair on the real axis" {
    // {-1, -2} -> (s+1)(s+2)
    const p1 = polyFromPoleGroups(&.{
        .{ .re = -1 },
        .{ .re = -2 },
    });
    try expectPoly(&.{ 1, 3, 2 }, p1, 1e-12);

    // -0.3 +/- 1.38203i -> s^2 + 0.6s + 2 (underdamped preset denominator)
    const p2 = polyFromPoleGroups(&.{
        .{ .re = -0.3, .im = 1.3820275, .pair = true },
    });
    try testing.expectApproxEqAbs(1.0, p2.coef[0], 1e-12);
    try testing.expectApproxEqAbs(0.6, p2.coef[1], 1e-12);
    try testing.expectApproxEqAbs(2.0, p2.coef[2], 1e-4);

    // A pair dragged to im = 0 keeps degree 2: (s+1)^2
    const p3 = polyFromPoleGroups(&.{
        .{ .re = -1, .im = 0, .pair = true },
    });
    try expectPoly(&.{ 1, 2, 1 }, p3, 1e-12);
}

test "ssToTf: underdamped preset gives den = s^2+0.6s+2, num = 1" {
    const f = try ssToTf(
        testing.allocator,
        2,
        &.{ 0, 1, -2, -0.6 },
        &.{ 0, 1 },
        &.{ 1, 0 },
        0.0,
    );
    try expectPoly(&.{ 1, 0.6, 2 }, f.den, 1e-9);

    const num = f.num.trimLeading(1e-9 * @max(1.0, f.num.maxAbs()));
    try expectPoly(&.{1}, num, 1e-9);
}

test "ssToTf: C = [1, 1] adds a zero at -1" {
    const f = try ssToTf(
        testing.allocator,
        2,
        &.{ 0, 1, -2, -0.6 },
        &.{ 0, 1 },
        &.{ 1, 1 },
        0.0,
    );
    const num = f.num.trimLeading(1e-9 * @max(1.0, f.num.maxAbs()));
    try expectPoly(&.{ 1, 1 }, num, 1e-9);

    const zs = try roots(testing.allocator, num);
    defer testing.allocator.free(zs);
    try testing.expectEqual(@as(usize, 1), zs.len);
    try testing.expectApproxEqAbs(-1.0, zs[0].re, 1e-9);
    try testing.expectApproxEqAbs(0.0, zs[0].im, 1e-9);
}

test "roots: real and complex quadratics, empty for degree 0" {
    const r1 = try roots(testing.allocator, Poly.init(&.{ 1, 3, 2 }));
    defer testing.allocator.free(r1);
    try testing.expectEqual(@as(usize, 2), r1.len);
    var lo = r1[0].re;
    var hi = r1[1].re;
    if (lo > hi) std.mem.swap(f64, &lo, &hi);
    try testing.expectApproxEqAbs(-2.0, lo, 1e-9);
    try testing.expectApproxEqAbs(-1.0, hi, 1e-9);

    const r2 = try roots(testing.allocator, Poly.init(&.{ 1, 0.6, 2 }));
    defer testing.allocator.free(r2);
    try testing.expectEqual(@as(usize, 2), r2.len);
    for (r2) |z| {
        try testing.expectApproxEqAbs(-0.3, z.re, 1e-6);
        try testing.expectApproxEqAbs(1.3820275, @abs(z.im), 1e-6);
    }

    const r3 = try roots(testing.allocator, Poly.init(&.{5}));
    defer testing.allocator.free(r3);
    try testing.expectEqual(@as(usize, 0), r3.len);
}

test "tfToSs: companion form matches tf2ss, and round-trips through ssToTf" {
    const den = polyFromPoleGroups(&.{ .{ .re = -1 }, .{ .re = -2 } });
    const num = Poly.init(&.{1});
    const ss = try tfToSs(testing.allocator, num, den);

    try testing.expectEqual(@as(usize, 2), ss.n);
    try testing.expectApproxEqAbs(-3.0, ss.a[0], 1e-12);
    try testing.expectApproxEqAbs(-2.0, ss.a[1], 1e-12);
    try testing.expectApproxEqAbs(1.0, ss.a[2], 1e-12);
    try testing.expectApproxEqAbs(0.0, ss.a[3], 1e-12);
    try testing.expectApproxEqAbs(0.0, ss.d, 1e-12);

    // Round trip: the same TF comes back out.
    const f = try ssToTf(testing.allocator, ss.n, ss.a[0..4], ss.b[0..2], ss.c[0..2], ss.d);
    try expectPoly(&.{ 1, 3, 2 }, f.den, 1e-9);
    const num2 = f.num.trimLeading(1e-9 * @max(1.0, f.num.maxAbs()));
    try expectPoly(&.{1}, num2, 1e-9);
}

test "ssToTf: hidden-instability system has a zero on top of the pole at +1" {
    // A = diag(-1, +1), the unstable state neither driven nor measured.
    // H(s) = (s - 1) / ((s + 1)(s - 1)): the cancellation is a zero at +1.
    const f = try ssToTf(
        testing.allocator,
        2,
        &.{ -1, 0, 0, 1 },
        &.{ 1, 0 },
        &.{ 1, 0 },
        0.0,
    );
    try expectPoly(&.{ 1, 0, -1 }, f.den, 1e-9);

    const num = f.num.trimLeading(1e-9 * @max(1.0, f.num.maxAbs()));
    const zs = try roots(testing.allocator, num);
    defer testing.allocator.free(zs);
    try testing.expectEqual(@as(usize, 1), zs.len);
    try testing.expectApproxEqAbs(1.0, zs[0].re, 1e-9);
}
