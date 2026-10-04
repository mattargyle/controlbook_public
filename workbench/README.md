# Controlbook Workbench

Interactive pages for working through the design studies. Open `index.html` directly in a browser; there's no build step or server. KaTeX loads from a CDN, and if you're offline the equations show as raw TeX.

**Hosted copy:** https://mattargyle.github.io/controlbook_public/workbench/ (GitHub Pages, serving `feature/workbench`). Page links there open the public PDF from the repo README, which can't jump to a page.

Deep links: `index.html#A/ch8/explore` (study / chapter / mode). Work mode always starts from your own gains; Explore never writes into it.

Page references (`p. 101 · Eq. 7.5`) are controlbook.pdf page numbers. They link to `../book_and_notes/controlbook.pdf#page=N`, which only resolves where the gitignored PDF exists locally. All links share one PDF tab.

All six design studies (A–F) have a tab for every problem. Issues found in the book and the repo code are summarized in [BOOK_ISSUES.md](BOOK_ISSUES.md), with full per-study detail in `js/studies/<X>/ISSUES.md`.

## Modes

- **Work it**: you set the gains. Problem parts have answer boxes with **Check** and **Show solution**. Live-math cards that would give away an answer stay hidden until you click **Reveal**. Answers are saved in the browser's localStorage.
- **Explore**: gains are designed from the chapter's knobs (pole locations in Ch 7, t_r and ζ in Ch 8). Drag the closed-loop poles in the s-plane.

## Layout

```
js/core/math.js        RK4, saturate, roots of quadratics, step metrics, formatting, answer matching
js/core/linalg.js      small matrices, polynomials, roots, det, ctrb/obsv, charPoly, Ackermann place()
js/core/place_yt.js    port of scipy's place_poles (YT), so multi-output observer gains match control.place
js/core/tf.js          transfer functions: products, feedback, Bode, margins, bandwidth, root locus, PID/lead/lag, filters
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

See [STUDY_GUIDE.md](STUDY_GUIDE.md): each study lives in `js/studies/<X>/` (system file, chapter modules, manifest) and builds on the generic blocks in `js/core/design.js`. Multi-output / multi-input systems declare `outputs`, `inputs`, `refs`, `initial`, and `disturbances`; the page builds one plot per output and controls per channel.

## Tools

```
python3 workbench/tools/smoke_test.py [--study X] [--shots DIR]   every tab × mode, buttons clicked, JS errors reported
.venv/bin/python workbench/tools/regress_A.py                     JS vs _A_arm/python controllers (machine precision)
tools/js_eval.py                                                   run a JS snippet with all workbench scripts loaded
```
