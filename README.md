# System Report Card

Takes a linear system (A, B, C, D) and renders a report card: stability
verdict, per-mode metrics, pole/zero map, step response. Math from
[znumerics](https://github.com/bendstok/znumerics), graphics from
[raylib-zig](https://github.com/raylib-zig/raylib-zig). Requires Zig 0.16.

## Setup (once)

```sh
zig fetch --save git+https://github.com/bendstok/znumerics
zig fetch --save git+https://github.com/raylib-zig/raylib-zig#devel
```

## Use

```sh
zig build test   # known-answer unit tests, headless
zig build run
```

- `1` / `2` / `3` — presets: underdamped, overdamped, unstable
- `4` — custom A/B/C/D editor: n = 1..4, Enter/Apply analyzes, Esc goes back,
  errors show in red and the previous plots stay up
- `5` — PID playground: Kp/Ki/Kd sliders drive the shown plant in closed loop.
  Bright curve = honest sim (derivative on measurement, u clamped to ±10,
  anti-windup); magenta X's = linear closed-loop poles. Under saturation the
  two disagree — that's the point.
- `L` — with the PID panel open: root locus. Dots trace the closed-loop
  poles as the whole controller is scaled x0.01 .. x100 (Kp:Ki:Kd ratio
  fixed); the magenta X's sit at x1, so dragging a slider scrubs along it.
- `Up` / `Down` / `R` — double / halve / auto the response time window
- mouse — drag poles on the map: conjugate pairs move mirrored, real poles
  slide along the axis; zeros and gain stay fixed and the system re-derives
  live. Zeros are the O markers (an O on top of an X is a cancellation —
  try the hidden-instability case). `1`/`2`/`3` or Apply leaves the dragged
  system.

## Web build

```sh
zig build -Dtarget=wasm32-emscripten -Doptimize=ReleaseFast   # -> zig-out/web/
zig build run -Dtarget=wasm32-emscripten                      # serve via emrun
```

First run downloads and activates the emsdk toolchain (large, one-time).
`.github/workflows/deploy-pages.yml` deploys `zig-out/web/` to GitHub Pages
on push to main — set Pages source to "GitHub Actions" in the repo settings.

## Layout

- `src/main.zig` — window loop, input, drawing, pole dragging. No allocation
  in the render loop except during interaction; DebugAllocator asserts no
  leaks on exit.
- `src/report.zig` — pure math, no raylib; the test root. Poles via shifted
  QR, mode metrics, verdict, zeros, ZOH step response.
- `src/tf.zig` — fixed-capacity polynomials: state-space <-> transfer
  function, roots via companion matrix. Tests chain in from report.zig.
- `src/pidpanel.zig` — PID sliders, sample-by-sample closed-loop sim
  (`PID_DEO_Sim`), linear closed-loop poles.
- `src/plot.zig` — `Viewport` (data rect <-> pixel rect) plus drawing helpers.
