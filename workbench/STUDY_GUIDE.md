# Adding a design study (B–F)

This is the contract for a study's pages. Study A (`js/systems/arm.js`, `js/chapters/*.js`) is the reference implementation. Read it before writing anything, and match its idioms and comment density.

## Files you own

Everything for study X lives in `js/studies/X/`:

```
js/studies/X/manifest.js   lists the study's scripts in load order (already exists; fill it in)
js/studies/X/system.js     WB.systems.X: parameters, dynamics, channels, drawing, problem data
js/studies/X/<group>.js    chapter modules registering into WB.studies.X.chapters
js/studies/X/ISSUES.md     book/repo inconsistencies found while building (same format as BOOK_ISSUES.md)
tools/regress_X.py         (optional) numerical check against _X_*/python, modeled on tools/regress_A.py
```

Do not edit shared files: `js/core/*`, `js/app.js`, `js/chapters/*`, `js/systems/arm.js`, `index.html`, `css/*`. If the framework is missing something, solve it locally inside your study files, and list the gap in your final report. Before writing a math helper, check the building blocks below: most of what studies B–F once wrote for themselves (root locus, bandwidth, PID/lead/lag transfer functions, determinants, scipy's YT placement) is now shared.

Scripts are classic scripts on the global `WB` namespace, with no ES modules, so the page works from `file://`. Wrap each file in an IIFE.

Register like this:

```js
WB.systems.X = { ... };
WB.studies = WB.studies || {};
WB.studies.X = WB.studies.X || { chapters: {} };
WB.studies.X.chapters.ch8 = { ... };
```

## System object (`WB.systems.X`)

| Field | Meaning |
| --- | --- |
| `id`, `name`, `introPage` | Letter, display name, PDF page of the study's introduction |
| `params: [{key, label, unit, value, min, max, step}]` | Nominal (controller-known) parameters, shown as sliders. `value` = book value |
| `constants: {g: 9.8, ...}` | Fixed constants merged into `p` |
| `uncertain: ['m', ...]` | Parameters the "true plant vs. model" mismatch perturbs, like `<sys>Dynamics(alpha)` |
| `f(x, u, p)` | ẋ for the nonlinear model; `u` is a number (1 input) or array |
| `h(x, p)` | Outputs: a number (1 output) or array |
| `uLimit(p)` | Input saturation: a number (±), an array of numbers, or an array of `[lo, hi]` |
| `outputs: [{key, label, unit, scale, minSpan?, noiseMax?}]` | `scale` = display units per SI unit (e.g. 180/π for degrees, 1 for m). The UI shows display units |
| `inputs: [{key, label, unit}]` | |
| `refs: [{output, label, unit, scale, min, max, step?, defaults?}]` | Reference channels. `output` = index into `outputs`. Ref 0 is primary (metrics use it) |
| `initial: [{key, label, unit, scale, output?, value, min, max, step?}]` | Initial-condition sliders. Values are in display units |
| `x0(init, p)` | State vector from `init` (SI, keyed by `initial[].key`) |
| `disturbances: [{input, label, unit, min, max}]` | Input disturbances, added to `u[input]` before plant saturation |
| `draw(ctx2d, w, h, s)` | Animation frame. `s = {x, rAll, uAll, ranges, satAll, p, r, u}` (canvas helpers: `WB.plot.css('--series-1')`) |
| `problems.chN: {id, page, statement: [html...], sim: {...}, mismatch?: {...}, ...}` | Problem data, read by your chapter code |

Optional: `stateSpace(p)`, `secondOrderModel(p)`, `simBase` (SIM_BASE overrides for every chapter).

A single-output, single-input system may instead use A's scalar form (`output`, `input`, scalar `h`, `x0(y0)`, as in `arm.js`).

## Simulation state (`ctx.S.sim`)

Channel 0 uses scalar fields:
- `type`, `amplitude`, `frequency`, `tStep`: ref 0
- `y0`: `initial[0]`
- `dist`: disturbance 0
- `noise`: output 0's noise σ

Channels 1.. use arrays:
- `refs[i-1] = {type, amplitude, frequency, tStep}`
- `init[key]`
- `dists[i-1]`
- `noises[i-1]`

Shared fields: `tDist`, `tEnd`, `Ts`, `seed`.

A chapter's `simDefaults(sys)` returns overrides for any of these, partial `refs` / `init` / `dists` / `noises`, and an optional `mismatch: {param: percent}`. Amplitudes, initial values and noise are in display units.

## Chapter object

Required:

| Field | |
| --- | --- |
| `id`, `num`, `tab`, `title`, `pages` | `num` orders the tabs and groups them: 2–6 Models, 7–10.9 PID, 11–14 Observers, 15–18 Loopshaping. App. P.6 uses `num: 10.5, short: 'P.6'`. `pages: 'pp. 377–383'` |
| `defaults(sys)` | Chapter UI state (`ctx.st`), saved in localStorage. Work-mode starting gains must NOT be the answer |
| `simDefaults(sys)` | See above |
| `controller(ctx, {linear})` | Returns `{update(r, x, yMeas, t)}`. Inputs and return value are described below |
| `buildControls(parent, ctx)` | Right-panel controls via `WB.ui.section/slider/segmented` (see `js/core/ui.js`). Every control re-renders through `ctx.update()` |
| `math(ctx)` | Live-math cards: `[{title, page, theory, symbolic?, numbers?, spoiler?, note?}]`. TeX strings; `\\quad` splits lines. `spoiler: true` hides `symbolic`/`numbers` in Work mode until Reveal |
| `buildProblem(parent, ctx)` | `WB.design.problemPanel(parent, ctx, prob, parts)`. Each part is `{id, title, inputs: {name: label}, html?, check(vals) -> {ok, msg}, actions?: [{label, run(vals)}], solution() -> [{tex} \| {html}]}`. `WB.design.checkNumbers(vals, truth, labels)` gives a 1% / 1e-3 check |

`controller.update(r, x, yMeas, t)`:
- `r` is a number when there is one reference, otherwise an array.
- `yMeas` is the noisy measurement: a number or an array.
- `x` is the TRUE state. Use it only where the book's controller does (e.g. the Ch 7–8, 11–12 "full state" designs).
- Return `u` (a number or array), or `{u, someExtra: number, ...}`. Extras are recorded in `result.extras.someExtra`.
- `linear: true` asks for the same law without feedforward or equilibrium terms, for a linear overlay (if you provide one).

Optional:
- `gains(ctx)`, `targets(ctx) -> {tr, zeta}`, `splane(ctx) -> {markers, zetaRay?, wnCircle?, wnMax?, loci?, fitR?}`, `onPoleDrag(ctx, id, re, im)`, `bode(ctx) -> {title, w, lines, specs?, marks?}`
- `extraPlot(ctx, result) -> {opts: {title, yLabel, unit}, data: {series, hlines?}}`
- `outputSeries(ctx, result, scaleFn, outputIndex)`: extra traces on output plot `outputIndex`, e.g. estimates
- `simulate(ctx, common, plant)`: custom simulation; return the shape of `WB.sim.simulate`
- `linearSim(ctx, common)`: linear-model overlay. Multi-output systems get none by default, so provide this if you want the dashed trace. Use `WB.design.linearPlant`
- `reference(ctx, baseRef)`. Work mode never inherits Explore's gains: don't copy designed gains into work state on a mode switch.
- Flags: `openLoop` (no reference/metrics), `metrics: false`, `linear: false`

The result object has a scalar channel 0 (`y, r, u, uDemand, uApplied, yMeas`), arrays per channel (`yAll[i]`, `rAll[i]`, `uDemandAll[i]`, `uAppliedAll[i]`, `yMeasAll[i]`), `x[k]` (state arrays), `t`, and `extras`.

## Building blocks (`WB.design`, `WB.la`, `WB.tf`, `WB.math`)

- `WB.design.wnFromTr(tr, zeta, '2.2'|'tp')` and its inverse `trFromWn`, `polesFromWnZeta`, `pdGains({b0, a1, a0}, poles)`, `dirtyCoeffs(sigma, Ts) -> {beta, gamma}` (Eq. 10.4).
- `WB.design.pidBlock({kP, kI, kD, sigma, Ts, limit, antiwindup, vbar, deriv})`. It follows the repo conventions (error_prev = 0, y_prev = first sample, trapezoid, dirty derivative). `update(r, y, {ydot?})`.
- `WB.design.place(A, B, poles)` returns K as a flat row. Also `refGain`, `augmentIntegrator(A, B, Cr)`, `augmentDisturbance(A, B, C)`, `observerGain(A, C, poles)`.
- `WB.design.observer({A, B, C, L, Ts, x0, rhs?})` is an RK4 observer (`update(y, u)`).
- `WB.design.linearPlant(A, B, C, {xe, ue, ye})`.
- `WB.yt.place(A, B, poles)` / `WB.yt.observer(A, C, poles)`: the port of scipy's `place_poles` (YT) that `control.place` uses. Use it where a multi-input or multi-output gain must match a `ctrl*.py` (the gain is not unique); otherwise `WB.design.place` is enough.
- `WB.la`: matrices, `roots`, `eig`, `ctrb`, `obsv`, `rank`, `det`, `polyFromRoots`, `charPoly`.
- `WB.tf`: `tf`, `mul`, `feedback`, `bode`, `mag`, `db`, `margins` (all phase crossings in `crossings`, all gain crossovers in `gcs`), `crossDown`, `bandwidth`, `rootLocus`, `pid` (dirty-derivative PID/PD), `lead`, `lag`, `lpf`, `pi`, `filter` (state-space RK4 with substeps), `repoFilter` (the repo's `transferFunction` class), `texTf`, `polyTex`.
- `WB.ui` panel kit: `bind(ctx, key, obj?)` (get/set for a slider or segmented control, re-rendering via `ctx.update()`), `gainSliders(parent, ctx, spec, keys, {obj, steps})`, `onOff(parent, ctx, block)`, `readout(parent, rows)`, `metric(label, value, status?)`, `specRow(label, ok, value)`, `shown(ctx, key)` (Explore mode or revealed) and `revealButton(ctx, key, text, {hideWhenShown})`.
- `WB.design.useGains(ctx, keys, {target, extra, msg})`: the "Use my gains" problem action. `WB.sim.switchTime(S, n)` and `WB.sim.indexBefore(S, res, t)` find the sample just before a square-wave switch.
- Chapter kits from study A: `WB.pid.shapedReference(ctx, base, scale, y0)` (Ch 9 ramp/parabola references), `WB.ss.ctrbCard(A, B, title, page, opts)`, `WB.freq.gmText(mg)` and `WB.freq.marginMarks(mg, opts)` (Bode margin annotations).
- `WB.math`: `rk4Step`, `stepMetrics`, `fmt`, `tex`, `texMat`, `texPole`, `parseComplex`, `close`, `polesMatch`.
- A's modules can be reused where they genuinely fit (`WB.pd`, `WB.pid`, `WB.ss`, `WB.models`), but they assume a single loop with x = (y, ẏ).

## Book and repo conventions

- **Page numbers** are controlbook.pdf pages (book page + 8). `book_and_notes/INDEX.md` maps every problem to its page. Take equations and numbers from the PDF images, never from `controlbook.txt`, whose math is scrambled.
- **Work vs. Explore.**
  - Work mode: the user sets gains; anything that answers a problem is hidden (`spoiler`, Reveal buttons, Show solution).
  - Explore mode: design from knobs (poles, t_r/ζ, compensator blocks), with all math shown.
- **Successive loop closure (B, C, E, F).** Show both loops' poles (inner/outer). Give the outer loop the inner loop's DC gain, as the book does. Show the bandwidth separation (`M = t_r,outer / t_r,inner`) as a control.
- **Simulation semantics** match `hwNN_*Sim.py`: the controller saturates its output, then `u + d` is saturated again by the plant, then RK4 at Ts.
- **Case study notes.**
  - E: the only study using an equilibrium state x_e, an equilibrium force F_e, and feedback linearization together.
  - F: the force/torque unmixing is `[[1, 1], [d, -d]]`. Some branches have the sign flipped.
- **Studies B and C have full worked solutions** in `_B_pendulum/python` and `_C_satellite/python`. Your JS controllers should match their `ctrl*.py` to machine precision. Use `tools/js_eval.py`, as `tools/regress_A.py` does.
- **D, E, F have no book solutions.** The repo only has templates plus `testDynamics.py` (expected f(x, u) values). Your `f` must reproduce those values exactly. Derive every answer from the PDF, and document derivations in the solution text.

## Checks before you finish

1. `python3 workbench/tools/smoke_test.py --study X` reports 0 errors.
2. Screenshot every chapter (`--shots DIR`) and look at them. No overlapping labels, empty plots or NaN readouts.
3. Numerical checks as described above (B, C: vs. `ctrl*.py`; D, E, F: vs. `testDynamics.py`), recorded in `tools/regress_X.py` or your report.
4. `python3 workbench/tools/smoke_test.py --study A` still reports 0 errors (you shouldn't have touched shared code).
