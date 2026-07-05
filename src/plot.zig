//! Minimal raylib plotting helpers. Exactly one abstraction: Viewport,
//! a mapping from a data rectangle to a pixel rectangle. Everything else
//! is straight lines and text. Not a plotting framework.

const std = @import("std");
const rl = @import("raylib");

pub const Viewport = struct {
    // data rectangle
    xmin: f64,
    xmax: f64,
    ymin: f64,
    ymax: f64,
    // pixel rectangle (top-left corner + size)
    px0: i32,
    py0: i32,
    pw: i32,
    ph: i32,

    /// Data x -> pixel x.
    pub fn px(self: Viewport, x: f64) i32 {
        const f = (x - self.xmin) / (self.xmax - self.xmin);
        return self.px0 + clampToPx(f * @as(f64, @floatFromInt(self.pw)));
    }

    /// Data y -> pixel y (flipped: +y is up in data, down in pixels).
    pub fn py(self: Viewport, y: f64) i32 {
        const f = (y - self.ymin) / (self.ymax - self.ymin);
        return self.py0 + self.ph - clampToPx(f * @as(f64, @floatFromInt(self.ph)));
    }

    /// Pixel x -> data x (inverse of px; used for mouse interaction).
    pub fn dataX(self: Viewport, p: i32) f64 {
        const f = @as(f64, @floatFromInt(p - self.px0)) / @as(f64, @floatFromInt(self.pw));
        return self.xmin + f * (self.xmax - self.xmin);
    }

    /// Pixel y -> data y (inverse of py).
    pub fn dataY(self: Viewport, p: i32) f64 {
        const f = @as(f64, @floatFromInt(self.py0 + self.ph - p)) / @as(f64, @floatFromInt(self.ph));
        return self.ymin + f * (self.ymax - self.ymin);
    }

    /// True when the pixel lies inside the panel rectangle.
    pub fn contains(self: Viewport, x: i32, y: i32) bool {
        return x >= self.px0 and x < self.px0 + self.pw and
            y >= self.py0 and y < self.py0 + self.ph;
    }
};

/// Clamp before @intFromFloat: out-of-range or NaN would be illegal
/// behavior, and unstable systems produce huge y values by design.
fn clampToPx(v: f64) i32 {
    if (std.math.isNan(v)) return 0;
    return @intFromFloat(std.math.clamp(v, -20000.0, 20000.0));
}

fn vec2(x: i32, y: i32) rl.Vector2 {
    return rl.Vector2.init(@as(f32, @floatFromInt(x)), @as(f32, @floatFromInt(y)));
}

/// Border around the panel.
pub fn drawFrame(vp: Viewport, color: rl.Color) void {
    rl.drawRectangleLines(vp.px0, vp.py0, vp.pw, vp.ph, color);
}

/// A "nice" tick step (1/2/5 * 10^k) giving roughly 5 ticks over `range`.
pub fn niceStep(range: f64) f64 {
    if (!(range > 0.0) or !std.math.isFinite(range)) return 1.0;
    const raw = range / 5.0;
    const mag = std.math.pow(f64, 10.0, @floor(std.math.log10(raw)));
    const norm = raw / mag;
    const s: f64 = if (norm < 1.5) 1.0 else if (norm < 3.5) 2.0 else if (norm < 7.5) 5.0 else 10.0;
    return s * mag;
}

/// Format a tick value with decimals appropriate for the step size.
pub fn fmtTick(buf: []u8, v: f64, step: f64) [:0]const u8 {
    // Snap -0.0 and tiny residue to zero so labels read "0".
    const vv = if (@abs(v) < step * 1e-6) 0.0 else v;
    const r = if (step >= 0.999)
        std.fmt.bufPrintZ(buf, "{d:.0}", .{vv})
    else if (step >= 0.0999)
        std.fmt.bufPrintZ(buf, "{d:.1}", .{vv})
    else
        std.fmt.bufPrintZ(buf, "{d:.2}", .{vv});
    return r catch "?";
}

/// Axis lines (through 0 when visible, else along the panel edge),
/// ticks as short lines, tick labels as text.
pub fn drawAxes(vp: Viewport, axis_color: rl.Color, text_color: rl.Color) void {
    const x_axis_py: i32 = if (vp.ymin <= 0 and vp.ymax >= 0) vp.py(0) else vp.py0 + vp.ph;
    const y_axis_px: i32 = if (vp.xmin <= 0 and vp.xmax >= 0) vp.px(0) else vp.px0;

    rl.drawLine(vp.px0, x_axis_py, vp.px0 + vp.pw, x_axis_py, axis_color);
    rl.drawLine(y_axis_px, vp.py0, y_axis_px, vp.py0 + vp.ph, axis_color);

    var buf: [32]u8 = undefined;

    // x ticks
    const sx = niceStep(vp.xmax - vp.xmin);
    var xv = @ceil(vp.xmin / sx) * sx;
    while (xv <= vp.xmax + sx * 1e-6) : (xv += sx) {
        const p = vp.px(xv);
        rl.drawLine(p, x_axis_py - 3, p, x_axis_py + 3, axis_color);
        if (@abs(xv) > sx * 0.5 or !(vp.xmin <= 0 and vp.xmax >= 0)) {
            rl.drawText(fmtTick(&buf, xv, sx), p - 8, x_axis_py + 6, 10, text_color);
        }
    }

    // y ticks
    const sy = niceStep(vp.ymax - vp.ymin);
    var yv = @ceil(vp.ymin / sy) * sy;
    while (yv <= vp.ymax + sy * 1e-6) : (yv += sy) {
        const p = vp.py(yv);
        rl.drawLine(y_axis_px - 3, p, y_axis_px + 3, p, axis_color);
        if (@abs(yv) > sy * 0.5) {
            rl.drawText(fmtTick(&buf, yv, sy), y_axis_px + 6, p - 5, 10, text_color);
        }
    }
}

/// Polyline through (xs[i], ys[i]) as consecutive line segments,
/// clipped to the viewport's pixel rectangle so out-of-range data
/// never draws outside the panel.
pub fn drawPolyline(vp: Viewport, xs: []const f64, ys: []const f64, thick: f32, color: rl.Color) void {
    std.debug.assert(xs.len == ys.len);
    if (xs.len < 2) return;
    rl.beginScissorMode(vp.px0, vp.py0, vp.pw, vp.ph);
    defer rl.endScissorMode();
    var i: usize = 1;
    while (i < xs.len) : (i += 1) {
        const a = vec2(vp.px(xs[i - 1]), vp.py(ys[i - 1]));
        const b = vec2(vp.px(xs[i]), vp.py(ys[i]));
        rl.drawLineEx(a, b, thick, color);
    }
}

/// Horizontal dashed line at data-y across the panel (e.g. a setpoint).
/// Skipped silently when y is outside the viewport's range.
pub fn drawHLineDashed(vp: Viewport, y: f64, dash: i32, color: rl.Color) void {
    if (y < vp.ymin or y > vp.ymax) return;
    const yy = vp.py(y);
    var x = vp.px0;
    while (x < vp.px0 + vp.pw) : (x += dash * 2) {
        const x2 = @min(x + dash, vp.px0 + vp.pw);
        rl.drawLine(x, yy, x2, yy, color);
    }
}

/// A zero drawn as an O marker (doubled circle outline for weight).
pub fn drawZeroMarker(vp: Viewport, re: f64, im: f64, radius: f32, color: rl.Color) void {
    const cx = vp.px(re);
    const cy = vp.py(im);
    rl.drawCircleLines(cx, cy, radius, color);
    rl.drawCircleLines(cx, cy, radius - 1.0, color);
}

/// A pole drawn as an X marker (two crossed lines).
pub fn drawPoleMarker(vp: Viewport, re: f64, im: f64, size: i32, color: rl.Color) void {
    const cx = vp.px(re);
    const cy = vp.py(im);
    rl.drawLineEx(vec2(cx - size, cy - size), vec2(cx + size, cy + size), 2.0, color);
    rl.drawLineEx(vec2(cx - size, cy + size), vec2(cx + size, cy - size), 2.0, color);
}
