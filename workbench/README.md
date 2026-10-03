# Controlbook Workbench

Interactive pages for working through the design studies. Open `index.html` directly in a browser; there's no build step or server. KaTeX loads from a CDN, and if you're offline the equations show as raw TeX.

Deep links: `index.html#A/ch8/explore` (study / chapter / mode).

## Modes

- **Work it**: you set the gains. Problem parts have answer boxes with **Check** and **Show solution**. Live-math cards that would give away an answer stay hidden until you click **Reveal**. Answers are saved in the browser's localStorage.
- **Explore**: gains are designed from the chapter's knobs (pole locations in Ch 7, t_r and ζ in Ch 8). Drag the closed-loop poles in the s-plane.

## Layout

```
js/core/math.js      RK4, saturate, roots, step metrics, formatting, answer matching
js/core/ui.js        sliders/toggles bound to state, KaTeX helper, storage
js/core/plot.js      TimePlot (crosshair + tooltip) and SPlane (draggable poles, ζ / ωn overlays)
js/core/sim.js       closed-loop loop matching hwNN_*Sim.py (controller sat → +d → plant sat → RK4)
js/systems/arm.js    Design Study A: dynamics, design model, feedback linearization, drawing, problem data
js/chapters/pd.js    Ch 7 (pole placement) and Ch 8 (t_r / ζ design, saturation limit)
js/app.js            state, panels, plots, playback, live-math strip
```

## Adding a design study (B–F)

Copy `systems/arm.js` to `systems/<name>.js`, register it as `WB.systems.<letter>`, and add a `<script>` tag in `index.html`. The fields the Ch 7–8 code uses are:

- `params`, `constants`, `uncertain`: nominal parameters (sliders), fixed constants, and the parameters the "true plant" mismatch perturbs.
- `f(x, u, p)`, `h(x)`, `uLimit(p)`, `x0(y0)`: the nonlinear simulation model.
- `secondOrderModel(p)`: `{b0, a1, a0, tex}` for P(s) = b0 / (s² + a1 s + a0).
- `feedbackLinearization(x, p)`, `equilibriumInput(yE, p)`, `ffTex`: the feedforward input.
- `draw(ctx, w, h, {x, r, u, uLimit, saturated})`: the animation frame.
- `problems.ch7`, `problems.ch8`: problem data (page, target poles or specs, simulation defaults).

The study's own homework (EOM, linearization, transfer function) is what goes into `f` and `secondOrderModel`. For D/E/F, write those yourself first.

Systems that need successive loop closure (B, E, F in Ch 8) will need a chapter variant with an inner and an outer loop. `pd.js` only covers a single second-order loop.

## Checks against the Python code

The JS simulation was compared with `_A_arm/python/armDynamics.py`, stepping the same PD law (nominal, saturating, derivative on error, and mismatched-plant cases). The results agree to machine precision.
