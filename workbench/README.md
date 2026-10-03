# Controlbook Workbench

Interactive pages for working through the design studies. Open `index.html` directly in a browser; there's no build step or server. KaTeX loads from a CDN, and if you're offline the equations show as raw TeX.

Deep links: `index.html#A/ch8/explore` (study / chapter / mode).

Page references (`p. 101 · Eq. 7.5`) are controlbook.pdf page numbers. They link to `../book_and_notes/controlbook.pdf#page=N`, which only resolves where the gitignored PDF exists locally. All links share one PDF tab.

Issues found in the book and the repo code while building the arm pages are listed in [BOOK_ISSUES.md](BOOK_ISSUES.md).

## Modes

- **Work it**: you set the gains. Problem parts have answer boxes with **Check** and **Show solution**. Live-math cards that would give away an answer stay hidden until you click **Reveal**. Answers are saved in the browser's localStorage.
- **Explore**: gains are designed from the chapter's knobs (pole locations in Ch 7, t_r and ζ in Ch 8). Drag the closed-loop poles in the s-plane.

## Layout

```
js/core/math.js        RK4, saturate, roots of quadratics, step metrics, formatting, answer matching
js/core/linalg.js      small matrices, polynomials, roots, ctrb/obsv, charPoly, Ackermann place()
js/core/tf.js          transfer functions: products, feedback, Bode, margins, state-space filter
js/core/ui.js          sliders/toggles bound to state, KaTeX helper, PDF page links, storage
js/core/plot.js        TimePlot, SPlane (draggable poles, root-locus branches), BodePlot (spec regions)
js/core/sim.js         closed-loop loop matching hwNN_*Sim.py (controller sat → +d → plant sat → RK4), noise
js/systems/arm.js      Design Study A: dynamics, linear models, energies, drawing, problem data for every chapter
js/chapters/models.js  Ch 2–6   kinetic energy, Euler-Lagrange (energy check), linearization, TF, state space
js/chapters/pd.js      Ch 7–8   PD pole placement, t_r/ζ design, saturation limit
js/chapters/pid.js     Ch 9–10  system type, digital PID; App. P.6 root locus vs k_I
js/chapters/ss.js      Ch 11–14 state feedback, integrator, observer, disturbance observer
js/chapters/freq.js    Ch 15–18 Bode, frequency specs, margins, loopshaping designer
js/app.js              state, panels, plots, playback, live-math strip
```

### Chapter hooks

A chapter object (`WB.chapters.chN`, with a numeric `num` that orders the tabs) provides `defaults`, `simDefaults`, `controller(ctx, {linear})`, `buildControls`, `math`, and `buildProblem`. Optional hooks: `gains`, `splane`, `onPoleDrag`, `bode`, `extraPlot`, `outputSeries`, `targets`, `reference`, `simulate`, `linearSim`, and the flags `openLoop`, `metrics: false`, `linear: false`.

### Checked against the repo

These were run with the same inputs, comparing the JS against the Python, with these results:
- `armDynamics.py` + PD (Ch 7–8): machine precision.
- `ctrlPID.py` (Ch 10, with the anti-windup fix from `fix/arm-pid-antiwindup`): 1e-16.
- `ctrlObserver.py` and `ctrlDisturbanceObserver.py` (Ch 13–14): 1e-13.
- Designed gains for Ch 10–14 match the repo classes.
- Ch 16–17 reproduce the book's numbers: B_r 44.5 dB, PM 49.0° at 10.8 rad/s, bandwidth ≈ 18 rad/s.
- The Ch 18 book preset reproduces Fig. 18-18: PM 59.7° at 14.2 rad/s.

## Adding a design study (B–F)

Copy `systems/arm.js` to `systems/<name>.js`, register it as `WB.systems.<letter>`, and add a `<script>` tag in `index.html`. The fields the chapter code uses are:

- `params`, `constants`, `uncertain`: nominal parameters (sliders), fixed constants, and the parameters the "true plant" mismatch perturbs.
- `f(x, u, p)`, `h(x)`, `uLimit(p)`, `x0(y0)`: the nonlinear simulation model.
- `secondOrderModel(p)`: `{b0, a1, a0, tex}` for P(s) = b0 / (s² + a1 s + a0).
- `feedbackLinearization(x, p)`, `equilibriumInput(yE, p)`, `ffTex`: the feedforward input.
- `draw(ctx, w, h, {x, r, u, uLimit, saturated})`: the animation frame.
- `stateSpace(p)`, `jacobian(p, yE)`, `kinetic(x, p)`, `potential(x, p)`: linear models and energies (Ch 2–6, 11–14).
- `problems.chN`: problem data (page, statement, specs, simulation defaults, optional default plant mismatch).

The study's own homework (EOM, linearization, transfer function) is what goes into `f` and `secondOrderModel`. For D/E/F, write those yourself first.

Systems that need successive loop closure (B, E, F) or more than one output will need chapter variants: the current chapter code assumes a single second-order loop with x = (y, ẏ).

