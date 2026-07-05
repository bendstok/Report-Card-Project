//! Custom-system editor: an editable grid of cells for A, B, C, D.
//!
//! Keyboard: type numbers into the selected cell, Backspace deletes,
//! Tab / Shift+Tab / Left / Right move, Enter applies, Esc goes back.
//! Mouse: click a cell to select it, click [-] [+] to change n,
//! click Apply to run the analysis.
//!
//! No allocations: all cell text lives in fixed buffers. Parsed values are
//! stored in fixed arrays inside the Editor; the returned SysDesc slices
//! point at those arrays and are consumed immediately by report.analyze,
//! which copies everything it needs.

const std = @import("std");
const rl = @import("raylib");
const report = @import("report.zig");

pub const max_n = 4;

// --- layout ---------------------------------------------------------------

const cell_w: i32 = 78;
const cell_h: i32 = 20;
const gap: i32 = 6;

const y_title: i32 = 505; // header / buttons row
const y_grid: i32 = 535; // first row of cells
const y_err: i32 = 645; // error message line

const x_a: i32 = 64; // A block (up to 4x4)
const x_b: i32 = 430; // B column
const x_c: i32 = 554; // C row
const x_d: i32 = 924; // D cell

// Title occupies x 40..~245 (font 18); the n control starts well right of it.
const x_n_label: i32 = 290; // "n" label
const btn_minus = Rect{ .x = 310, .y = y_title, .w = 22, .h = 22 };
const x_n_value: i32 = 338; // the number sits between the buttons
const btn_plus = Rect{ .x = 362, .y = y_title, .w = 22, .h = 22 };
const btn_apply = Rect{ .x = 410, .y = y_title, .w = 130, .h = 22 };

// --- colors (match main palette) -------------------------------------------

const col_cell_bg = rl.Color.init(38, 41, 50, 255);
const col_cell_border = rl.Color.init(90, 95, 110, 255);
const col_cell_sel = rl.Color.init(102, 191, 255, 255);
const col_btn = rl.Color.init(52, 56, 68, 255);
const col_text = rl.Color.init(220, 222, 228, 255);
const col_dim = rl.Color.init(150, 153, 165, 255);
const col_err = rl.Color.init(230, 41, 55, 255);

pub const Action = enum { none, apply, exit };

const Rect = struct { x: i32, y: i32, w: i32, h: i32 };

fn hitRect(r: Rect, mx: i32, my: i32) bool {
    return mx >= r.x and mx < r.x + r.w and my >= r.y and my < r.y + r.h;
}

fn rowY(row: usize) i32 {
    return y_grid + @as(i32, @intCast(row)) * (cell_h + gap);
}

fn colX(base: i32, col: usize) i32 {
    return base + @as(i32, @intCast(col)) * (cell_w + gap);
}

// --- one editable cell ------------------------------------------------------

const Cell = struct {
    buf: [24:0]u8 = std.mem.zeroes([24:0]u8),
    len: usize = 0,

    fn text(self: *const Cell) [:0]const u8 {
        return self.buf[0..self.len :0];
    }

    fn set(self: *Cell, s: []const u8) void {
        const m = @min(s.len, self.buf.len - 1);
        @memcpy(self.buf[0..m], s[0..m]);
        self.len = m;
        self.buf[m] = 0;
    }

    fn setFloat(self: *Cell, v: f64) void {
        var tmp: [24]u8 = undefined;
        const s = std.fmt.bufPrint(&tmp, "{d}", .{v}) catch "0";
        self.set(s);
    }

    fn push(self: *Cell, ch: u8) void {
        if (self.len >= self.buf.len - 1) return;
        self.buf[self.len] = ch;
        self.len += 1;
        self.buf[self.len] = 0;
    }

    fn pop(self: *Cell) void {
        if (self.len == 0) return;
        self.len -= 1;
        self.buf[self.len] = 0;
    }
};

// --- the editor --------------------------------------------------------------

pub const Editor = struct {
    n: usize = 2,
    sel: usize = 0, // linear index over active cells: A row-major, B, C, D

    // A is stored with fixed stride max_n so cells keep their place
    // when n changes.
    a: [max_n * max_n]Cell = [_]Cell{.{}} ** (max_n * max_n),
    b: [max_n]Cell = [_]Cell{.{}} ** max_n,
    c: [max_n]Cell = [_]Cell{.{}} ** max_n,
    d: Cell = .{},

    err_buf: [96:0]u8 = std.mem.zeroes([96:0]u8),
    err_len: usize = 0,

    // Parsed values; SysDesc slices point into these.
    pa: [max_n * max_n]f64 = std.mem.zeroes([max_n * max_n]f64),
    pb: [max_n]f64 = std.mem.zeroes([max_n]f64),
    pc: [max_n]f64 = std.mem.zeroes([max_n]f64),
    pd: f64 = 0,

    /// Prefilled with the underdamped demo system; every cell holds "0"
    /// so growing n never exposes empty cells.
    pub fn init() Editor {
        var e = Editor{};
        for (&e.a) |*cl| cl.set("0");
        for (&e.b) |*cl| cl.set("0");
        for (&e.c) |*cl| cl.set("0");
        e.d.set("0");

        e.a[0 * max_n + 0].setFloat(0);
        e.a[0 * max_n + 1].setFloat(1);
        e.a[1 * max_n + 0].setFloat(-2);
        e.a[1 * max_n + 1].setFloat(-0.6);
        e.b[1].setFloat(1);
        e.c[0].setFloat(1);
        return e;
    }

    fn cellCount(self: *const Editor) usize {
        return self.n * self.n + 2 * self.n + 1;
    }

    fn cellRef(self: *Editor, idx: usize) *Cell {
        const nn = self.n * self.n;
        if (idx < nn) return &self.a[(idx / self.n) * max_n + (idx % self.n)];
        if (idx < nn + self.n) return &self.b[idx - nn];
        if (idx < nn + 2 * self.n) return &self.c[idx - nn - self.n];
        return &self.d;
    }

    fn cellConst(self: *const Editor, idx: usize) *const Cell {
        const nn = self.n * self.n;
        if (idx < nn) return &self.a[(idx / self.n) * max_n + (idx % self.n)];
        if (idx < nn + self.n) return &self.b[idx - nn];
        if (idx < nn + 2 * self.n) return &self.c[idx - nn - self.n];
        return &self.d;
    }

    fn cellRect(self: *const Editor, idx: usize) Rect {
        const nn = self.n * self.n;
        if (idx < nn) {
            return .{ .x = colX(x_a, idx % self.n), .y = rowY(idx / self.n), .w = cell_w, .h = cell_h };
        }
        if (idx < nn + self.n) {
            return .{ .x = x_b, .y = rowY(idx - nn), .w = cell_w, .h = cell_h };
        }
        if (idx < nn + 2 * self.n) {
            return .{ .x = colX(x_c, idx - nn - self.n), .y = rowY(0), .w = cell_w, .h = cell_h };
        }
        return .{ .x = x_d, .y = rowY(0), .w = cell_w, .h = cell_h };
    }

    fn selNext(self: *Editor) void {
        self.sel = (self.sel + 1) % self.cellCount();
    }

    fn selPrev(self: *Editor) void {
        const cnt = self.cellCount();
        self.sel = (self.sel + cnt - 1) % cnt;
    }

    fn setN(self: *Editor, n: usize) void {
        self.n = n;
        if (self.sel >= self.cellCount()) self.sel = self.cellCount() - 1;
        // Never leave an active cell empty after a resize.
        for (0..self.cellCount()) |i| {
            const cl = self.cellRef(i);
            if (cl.len == 0) cl.set("0");
        }
    }

    /// Handle one frame of input. Returns what the caller should do.
    pub fn update(self: *Editor) Action {
        // Typed characters go into the selected cell.
        var ch = rl.getCharPressed();
        while (ch != 0) : (ch = rl.getCharPressed()) {
            if (ch > 0 and ch < 128) {
                const c: u8 = @intCast(ch);
                switch (c) {
                    '0'...'9', '.', '-', '+', 'e', 'E' => self.cellRef(self.sel).push(c),
                    else => {},
                }
            }
        }

        if (rl.isKeyPressed(.backspace) or rl.isKeyPressedRepeat(.backspace)) {
            self.cellRef(self.sel).pop();
        }

        const shift = rl.isKeyDown(.left_shift) or rl.isKeyDown(.right_shift);
        if (rl.isKeyPressed(.tab)) {
            if (shift) self.selPrev() else self.selNext();
        }
        if (rl.isKeyPressed(.right)) self.selNext();
        if (rl.isKeyPressed(.left)) self.selPrev();

        if (rl.isKeyPressed(.enter) or rl.isKeyPressed(.kp_enter)) return .apply;
        if (rl.isKeyPressed(.escape)) return .exit;

        if (rl.isMouseButtonPressed(.left)) {
            const mp = rl.getMousePosition();
            const mx: i32 = @intFromFloat(mp.x);
            const my: i32 = @intFromFloat(mp.y);

            if (hitRect(btn_minus, mx, my)) {
                if (self.n > 1) self.setN(self.n - 1);
            } else if (hitRect(btn_plus, mx, my)) {
                if (self.n < max_n) self.setN(self.n + 1);
            } else if (hitRect(btn_apply, mx, my)) {
                return .apply;
            } else {
                for (0..self.cellCount()) |i| {
                    if (hitRect(self.cellRect(i), mx, my)) {
                        self.sel = i;
                        break;
                    }
                }
            }
        }

        return .none;
    }

    /// Parse all active cells. On success returns a SysDesc whose slices
    /// point into this Editor (analyze copies them immediately). On failure
    /// sets the error message and returns null.
    pub fn buildSysDesc(self: *Editor) ?report.SysDesc {
        self.err_len = 0;
        const n = self.n;

        for (0..n) |i| {
            for (0..n) |j| {
                const v = self.parseCell(&self.a[i * max_n + j], "A", i, j) orelse return null;
                self.pa[i * n + j] = v;
            }
        }
        for (0..n) |i| {
            self.pb[i] = self.parseCell(&self.b[i], "B", i, 0) orelse return null;
        }
        for (0..n) |i| {
            self.pc[i] = self.parseCell(&self.c[i], "C", 0, i) orelse return null;
        }
        self.pd = self.parseCell(&self.d, "D", 0, 0) orelse return null;

        return .{
            .name = "Custom",
            .n = n,
            .a = self.pa[0 .. n * n],
            .b = self.pb[0..n],
            .c = self.pc[0..n],
            .d = self.pd,
        };
    }

    fn parseCell(self: *Editor, cl: *const Cell, group: []const u8, i: usize, j: usize) ?f64 {
        const v = std.fmt.parseFloat(f64, cl.buf[0..cl.len]) catch {
            self.setParseErr(group, i, j);
            return null;
        };
        if (!std.math.isFinite(v)) {
            self.setParseErr(group, i, j);
            return null;
        }
        return v;
    }

    fn setParseErr(self: *Editor, group: []const u8, i: usize, j: usize) void {
        const s = std.fmt.bufPrintZ(&self.err_buf, "Bad number in {s}[{d}][{d}]", .{ group, i + 1, j + 1 }) catch return;
        self.err_len = s.len;
    }

    /// For failures coming out of report.analyze.
    pub fn setError(self: *Editor, msg: []const u8) void {
        const s = std.fmt.bufPrintZ(&self.err_buf, "Analysis failed: {s}", .{msg}) catch return;
        self.err_len = s.len;
    }

    pub fn draw(self: *const Editor) void {
        rl.drawText("Custom system editor", 40, y_title + 3, 18, col_text);

        rl.drawText("n", x_n_label, y_title + 5, 16, col_text);
        drawButton(btn_minus, "-");
        var nbuf: [8]u8 = undefined;
        const nstr = std.fmt.bufPrintZ(&nbuf, "{d}", .{self.n}) catch "?";
        const ntw = rl.measureText(nstr, 16);
        rl.drawText(nstr, x_n_value + @divTrunc(20 - ntw, 2), y_title + 5, 16, col_text);
        drawButton(btn_plus, "+");

        drawButton(btn_apply, "Apply (Enter)");
        rl.drawText("type numbers, Tab/arrows move, Esc = back to presets", 570, y_title + 6, 14, col_dim);

        rl.drawText("A", x_a - 20, rowY(0) + 3, 16, col_text);
        rl.drawText("B", x_b - 20, rowY(0) + 3, 16, col_text);
        rl.drawText("C", x_c - 20, rowY(0) + 3, 16, col_text);
        rl.drawText("D", x_d - 20, rowY(0) + 3, 16, col_text);

        const blink = @mod(rl.getTime(), 1.0) < 0.6;
        for (0..self.cellCount()) |i| {
            const r = self.cellRect(i);
            rl.drawRectangle(r.x, r.y, r.w, r.h, col_cell_bg);
            rl.drawRectangleLines(r.x, r.y, r.w, r.h, if (i == self.sel) col_cell_sel else col_cell_border);
            const cl = self.cellConst(i);
            rl.drawText(cl.text(), r.x + 5, r.y + 3, 14, col_text);
            if (i == self.sel and blink) {
                const tw = rl.measureText(cl.text(), 14);
                rl.drawRectangle(r.x + 6 + tw, r.y + 3, 2, cell_h - 6, col_cell_sel);
            }
        }

        if (self.err_len > 0) {
            rl.drawText(self.err_buf[0..self.err_len :0], 40, y_err, 14, col_err);
        }
    }
};

fn drawButton(r: Rect, label: [:0]const u8) void {
    rl.drawRectangle(r.x, r.y, r.w, r.h, col_btn);
    rl.drawRectangleLines(r.x, r.y, r.w, r.h, col_cell_border);
    const tw = rl.measureText(label, 14);
    rl.drawText(label, r.x + @divTrunc(r.w - tw, 2), r.y + 4, 14, col_text);
}
