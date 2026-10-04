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
| `math(ctx)` | Live-math cards: `[{title, page, theory, symbolic?, numbers?, spoiler?, answers?, note?}]`. TeX strings; `\\quad` splits lines. `theory` is always shown, so it must be a general equation from the book, never this study's result. `spoiler: true` hides `symbolic`/`numbers` (and `note`) in Work mode until Reveal. `answers: 'A.3/d'` (or a list) marks a card that gives away that problem part, from this chapter or an earlier one: in Work mode the whole card stays locked until the part is solved (a passing Check) or revealed |
| `buildProblem(parent, ctx)` | `WB.design.problemPanel(parent, ctx, prob, parts)`. Each part is `{id, title, inputs: {name: label}, html?, check(vals) -> {ok, msg}, actions?: [{label, run(vals)}], solution() -> [{tex} \| {html} \| {code}]}`. `WB.design.checkNumbers(vals, truth, labels)` gives a 1% / 1e-3 check. A part with `code: {template, check(code) -> Promise, actions?}` gets a Python editor instead (see "Python answers" below). A part with no inputs, code or check is a step done outside the workbench and shows only its html; add `done: 'button label'` when the student should say they finished it (e.g. a Bode plot drawn by hand, A.15(a)). `after: 'a'` hides a part in Work mode until part (a) is solved or done, and anything the hidden step would give away (plots, markers) should check `ctx.app.isSolved('A.15/a')` too. Give every lettered part of the book's problem its own part, in order |

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
  - Work mode: the user sets gains; anything that answers a problem is hidden (`spoiler`, `answers`, Reveal buttons, Show solution). That includes the *form* of a result: part titles use the book's wording and never state the equation (no "P = P₀ + c sin θ, enter c"), control-panel text and plot labels don't state it either, and s-plane markers that answer a part stay off until it is solved.
  - Derivation parts (energies, equations of motion, linearizations, transfer functions, state-space matrices, Evans form) are Python answers checked at random arguments and parameters, so the student has to produce the whole expression. Numeric entry is for computed quantities with fixed specs (gains, poles, margins).
  - An answer that depends on something the student can change (gain sliders, σ, spec frequencies) is a Python function of those knobs, checked at random values of them, not a number for the current sliders: the answer must not go stale when a slider moves. Examples: Ch 9 system type and steady-state errors (`system_type = …`, `e_ramp(kP, kD)`, `np.inf` for unbounded; `WB.py.check` compares infinities exactly), P.6 `kI_crit(kP, kD)`, Ch 16 `track_pct(w, kP, kI, kD, sigma)`. A check of the student's *design* ("pick k_I so…", "the current simulation passes") still reads the sliders.
  - Explore mode: design from knobs (poles, t_r/ζ, compensator blocks), with all math shown.
- **Successive loop closure (B, C, E, F).** Show both loops' poles (inner/outer). Give the outer loop the inner loop's DC gain, as the book does. Show the bandwidth separation (`M = t_r,outer / t_r,inner`) as a control.
- **Simulation semantics** match `hwNN_*Sim.py`: the controller saturates its output, then `u + d` is saturated again by the plant, then RK4 at Ts.
- **Case study notes.**
  - E: the only study using an equilibrium state x_e, an equilibrium force F_e, and feedback linearization together.
  - F: the force/torque unmixing is `[[1, 1], [d, -d]]`. Some branches have the sign flipped.
- **Studies B and C have full worked solutions** in `_B_pendulum/python` and `_C_satellite/python`. Your JS controllers should match their `ctrl*.py` to machine precision. Use `tools/js_eval.py`, as `tools/regress_A.py` does.
- **D, E, F have no book solutions.** The repo only has templates plus `testDynamics.py` (expected f(x, u) values). Your `f` must reproduce those values exactly. Derive every answer from the PDF, and document derivations in the solution text.

## Python answers (`WB.py`, `js/core/py.js`)

Student code runs in Pyodide (Python 3.14 + numpy) in a sandboxed Web Worker, loaded on the first Check. `P` is a namespace of the parameters (`P.m`, `P.ell`, ... like `<sys>Param.py`) and `np` is imported.

- `WB.py.check(ctx, spec, code)` evaluates the student's functions (`items: [{fn, args, truth(p, a), compare?}]`) or variables (`{var, truth(p)}`) at random arguments (`args`) for the nominal parameters and four random parameter sets, and compares them with `truth`. `compare: 'offset'` ignores an additive constant (potential energy), `'scale'` accepts any nonzero multiple (characteristic polynomials). `cases: [{label, fix}]` groups points so a failure says which physics is wrong ("only gravity acts"). Complex arguments (`{complex: true, re, im}`) test transfer functions; `WB.py.cx` does the complex arithmetic in `truth`.
- `WB.py.evaluate(code, samples)` and `WB.py.simulate(code, {fn, params, x0, u, Ts})` are the lower-level calls (see A.4(c) and A.3(e) in `js/chapters/models.js`).
- Arm examples: `js/chapters/models.js` (A.2–A.6), `pd.js` (A.7(b)), `pid.js` (A.P.6(a)).
- `WB.py.simulate` also takes `plant` (the workbench's own `f(state, u)` as Python source, run in its own namespace with `plantParams`, normally `ctx.pTrue`, while the student's code gets `params`, normally `ctx.pModel`) and `ctrl` (a student function of the state components added to `u[k]` once per step, then saturated at `uLimit` and disturbed by `d[k]`, like `WB.sim`). `hold` names a function `hold(state, u[k])` whose result is held as the step's input (vector inputs, e.g. a student's F_fl plus the workbench's mixing and saturation in F.4(c)). That simulates the true plant under a student's feedback-linearizing force.

### "Plot my answer" overlays (`WB.yours`, `js/core/yours.js`)

In the modeling chapters (Ch 2–6), every Python part whose answer is a model gets a button (`code.actions`) that runs the student's code, solved or not, and draws what it gives next to the workbench's own model: dotted traces labelled "your …" on the time plots and green × markers on the s-plane. It shows only the student's own result, so it never gives an answer away, and a wrong answer looks visibly wrong (an unstable eigenvalue, a curve that leaves the reference). Once plotted, the overlay reruns whenever the chapter's state changes, so it follows the sliders.

- Button: `actions: [{ label: 'Plot my A, B', run: (code) => WB.yours.plot(ctx, 'ch4.b', code, runFn, describe) }]`. `runFn(ctx, code)` calls `WB.py.evaluate`/`simulate` and resolves to overlay data (carry `n` = number of samples for time series), or `{ok: false, msg}` (use `WB.yours.pyError(out)` for Python errors). `describe(data)` is a neutral sentence saying where the overlay is drawn; the result shows no ✓/✗ (`{info: true}`).
- Drawing: `WB.yours.data(ctx, id)` in `outputSeries`, `extraPlot` and `splane`; `WB.yours.series(label, y)`, `linResponse` (MIMO, on the main simulation's time grid), `eig`/`poles`/`zeros` + `withMarkers`, `fitTf` + `tfResponse` (a student transfer function sampled at complex points and fitted with the lowest-order proper rational function).
- What to plot per kind of answer: energies along the simulated or prescribed motion; equations of motion / f(x, u) simulated from the same initial state and input (Ch 3's "Simulate my f" already does this); an equilibrium input applied to the plant ("Use my F_e" sets the Work-mode slider); a Jacobian A, B as eigenvalues plus the linear model's response next to the "linearized model" trace; a feedback-linearizing input applied to the true plant (`simulate` with `plant` + `ctrl`) next to the student's own linear model; a transfer function as poles/zeros plus its response from rest; state-space A, B, C, D as eigenvalues plus the response from the simulation's initial state.
- Study D (`js/studies/D/models.js`) is the reference implementation. `tools/py_test.py` presses every "Plot/Use/Simulate my …" button with the solution code and requires each "your …" trace to lie on some reference trace (relative deviation < 1e-3), so draw it next to the matching workbench series (same initial state, input and parameters).

### Student controllers (`WB.myCtrl`, `js/core/myctrl.js`)

From Ch 7 on, a part that asks the student to *implement* something (a controller, a dirty derivative, an integrator with anti-windup, an observer, a disturbance observer, C(s) in state space, a prefilter) is a Python part in which the student writes `class Controller` with `__init__(self)` and `update(self, r, y)`, as in the repo's `ctrl*.py`. Never give such a part as gain boxes, gain sliders, or a toggle that switches on a workbench implementation: the number and names of the gains give the structure away. Study A (`js/chapters/pd.js`, `pid.js`, `ss.js`, `freq.js`) is the reference implementation.

- **System fields.** `plantPy` is the plant as Python, `f(state, u)` and `h(state)` (the measured outputs, as a list). `py: {r, y: [...], x: [...], u}` gives the Python names used in the bare template. `pyParams(x0)` returns the extra entries of `P` (the initial state, as in `<sys>Param.py`); `P.Ts` is always there. `altParams(p)` is a second parameter set for checks, so gains must come from `P`.
- **Chapter field.** `implement: { feed: 'state' | 'y', linear?: false, params?(ctx) }`. `feed` says what `update` gets: the state (Ch 7, 8, as in the book's code) or the measured outputs (Ch 10 on). `linear: false` drops the linear overlay in Work mode (when it would need gains the student no longer sets). `params(ctx)` adds entries to `P` (A.18: the designed `C_num`, `C_den`). `plantParams(ctx)` adds entries to the true plant's parameters in Work mode, for disturbances that don't enter through the inputs (F's wind); checks pass them as `scenario({plantExtra})`. The plant's Python reads the time of the current step as `P.t`.
- **Work mode.** For such a chapter, app.js simulates the student's controller (`WB.myCtrl.simulate`): the code of their latest *Run my controller* or Check, rerun in Pyodide whenever the left panel changes. Until they run one, the plant runs with zero input. Remove the gain sliders and the implementation toggles from Work mode (keep s-plane-only gain sliders only where they reveal nothing beyond the problem statement, labelled as such), and put `WB.myCtrl.banner(section, ctx, 'A.10(c)')` in the control panel. Section titles in Work mode must not state the control law. Plots read the estimates the student returns: `update` may return `u`, `(u, x_hat)` or `(u, x_hat, d_hat)`, recorded as `extras.xhat0, xhat1, …, dhat`.
- **Parts.** `WB.myCtrl.part(ctx, { id, title, html, seed, check(code), solution })` gives a part with the bare template (or the student's saved code of the `seed` part(s), e.g. `'A.12/a2'`, so each chapter starts from their previous controller), a *Run my controller* button, and the Check. `WB.myCtrl.savedCode(ctx, 'A.13/c2')` lets a check-only part (A.13(e)) or an action (A.14(a)) run another part's code.
- **Checks.** Build fixed runs with `WB.myCtrl.scenario(ctx, {params, mismatch, ref, y0, dist, noise, seed, tEnd})` (they don't depend on the left panel), run the student's code with `WB.myCtrl.run`, and compare:
  - Where the problem fixes the design (A.7(d), A.8(a), A.11(e), A.12(a), A.18(c)), use `WB.myCtrl.matchCheck` against the workbench's own controller on the same run (`WB.myCtrl.reference`, `refCtx`), with the nominal parameters and `altParams` (`paramCases`), at a few percent of the step size. A case may give several equally valid references (`refs: [...]`, or `ref` returning an array: the closest counts), e.g. both filter conventions or with and without a feedforward term the problem doesn't fix. `tol: {outputIndex: tol}` (or `outputs` with one `tol`) compares several outputs, where one alone can't see a wrong inner loop.
  - Where the student tunes (A.10(c), A.12(c), A.13(c), A.14(b), A.18(d)), check behaviour: the error before the first switch, overshoot, the estimate against the true state in settled windows, the mean estimator bias under noise.
  - Where the book fixes an algorithm detail, probe or provoke it: `WB.myCtrl.probe` feeds a fresh controller fixed inputs (A.8(b): a huge error must give |u| ≤ u_max); a lowered `tau_max` makes a large step saturate for seconds, so integrator windup shows as overshoot (A.12(a)). Accept any anti-windup scheme that passes. An open-loop-unstable plant (B) can't survive seconds of saturation with or without anti-windup: probe the controller open loop instead (hold a large error with the output saturated, then check the integrator doesn't keep growing).
  - Never read named attributes (`self.kp`): requiring names leaks structure. With several outputs L is not unique, so check estimates, not L.
- **Loopshaping designs (Ch 18).** The student gives each compensator as coefficient lists (`C_l_num`, `C_l_den`; `np.convolve` multiplies factors), loaded with a *Use my …* button and the Check. Work mode then draws the Bode plot, s-plane and spec readouts from their compensator and hides the block menu (gain, lead, lag, LPF, presets), whose size and order mirror the solution. The implementation part gets the designed C as `P.C_num`, `P.C_den` (A.18: `implement.params`).
- **Python side.** `import control as cnt` is a small stand-in for python-control (`place`, `ctrb`, `obsv`), since python-control needs scipy. `place` is a numpy port of scipy's `place_poles` (YT), as python-control uses, so multi-input and two-output observer gains match the repo's.
- **Tests.** `tools/py_test.py` presses *Run my controller* after each solution passes; `CODE_CHECKS` in `tools/py_targets.py` lists check-only parts that run saved code.

## Checks before you finish

1. `python3 workbench/tools/smoke_test.py --study X` reports 0 errors.
2. Screenshot every chapter (`--shots DIR`) and look at them. No overlapping labels, empty plots or NaN readouts.
3. Numerical checks as described above (B, C: vs. `ctrl*.py`; D, E, F: vs. `testDynamics.py`), recorded in `tools/regress_X.py` or your report.
4. `python3 workbench/tools/smoke_test.py --study A` still reports 0 errors (you shouldn't have touched shared code).
5. If the study has Python parts: `python3 workbench/tools/py_test.py --study X` (needs network) reports 0 failures: templates fail, solutions pass, locked cards unlock.
