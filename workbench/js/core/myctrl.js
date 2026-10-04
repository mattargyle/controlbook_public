// Student controllers for the implementation parts of Ch 7-18.
//
// The student writes `class Controller` in Python (an __init__ and
// update(self, r, y) that returns u, or (u, x_hat) / (u, x_hat, d_hat)), as in the
// repo's ctrl*.py. WB.py.closedLoop runs it in Pyodide against the workbench's
// plant (sys.plantPy, with the true parameters) using the hwNN_*Sim.py loop.
//
// In Work mode a chapter with an `implement` field simulates the student's
// controller instead of the workbench's: once they press "Run my controller" (or
// Check), app.js gets its result from WB.myCtrl.simulate, which reruns their code
// whenever the left panel changes. Until then the plant runs with zero input, so
// the plots never show a working controller the student didn't write.
//
// Chapter side:
//   implement: { feed: 'state' | 'y', params(ctx) -> extra P entries }
//   parts: WB.myCtrl.part(ctx, { id, title, html, seed, check(code), solution })
//   checks: WB.myCtrl.scenario / run / reference / maxDiff (below)
window.WB = window.WB || {};

WB.myCtrl = (function () {
  const M = WB.math;
  const ACTIVE_KEY = 'wb.myctrl';
  const active = WB.ui.store.get(ACTIVE_KEY, {});  // `${sysId}.${chapter}` -> { code, part }
  const runs = {};                                  // same key -> { key, data, pending, failed, err, promise }
  const slot = (ctx) => `${ctx.S.sysId}.${ctx.S.chapter}`;
  const asArr = (v) => (Array.isArray(v) ? v : [v]);

  // Input limits as [[lo, hi], ...] (same rule as app.js).
  function ranges(sys, p) {
    const n = sys.inputs ? sys.inputs.length : 1;
    const Lm = sys.uLimit(p);
    return Array.from({ length: n }, (_, i) => {
      const v = Array.isArray(Lm) ? Lm[i] : Lm;
      return Array.isArray(v) ? v : [-v, v];
    });
  }

  // ------------------------------------------------------------ scenarios --
  // A run: { params, plantParams, x0, Ts, tEnd, reference(t), disturbance(t),
  // noise(k) | null, feed, extraP }. Signals are SI, as in WB.sim.
  function implementOf(ctx) { return (WB.studies[ctx.S.sysId].chapters[ctx.S.chapter] || {}).implement || {}; }

  // The left panel's simulation (app.js `common`) as a run.
  function displayScenario(ctx, common) {
    const imp = implementOf(ctx);
    return {
      params: ctx.pModel, plantParams: ctx.pTrue, x0: common.x0, Ts: common.Ts, tEnd: common.tEnd,
      reference: common.reference, disturbance: common.disturbance, noise: common.noise,
      feed: imp.feed || 'y', extraP: imp.params ? imp.params(ctx) : {},
    };
  }

  // A fixed run for a check. o: { params (default nominal), mismatch: {key: %},
  // ref: {type, amplitude (display units), frequency, tStep}, y0 (display units),
  // dist: value (SI, from tDist), tDist, noise: σ (SI), seed, tEnd, Ts, feed, extraP }.
  function scenario(ctx, o = {}) {
    const sys = ctx.sys, imp = implementOf(ctx);
    const params = { ...(o.params || ctx.pModel) };
    const plantParams = { ...params };
    for (const [k, v] of Object.entries(o.mismatch || {})) plantParams[k] = params[k] * (1 + v / 100);
    const out = sys.output || {};
    const scale = out.scale || 1;
    const ref = { type: 'step', amplitude: 30, frequency: 0.05, tStep: 0, ...(o.ref || {}) };
    const y0 = (o.y0 || 0) / scale;
    const g = WB.sim.makeReference({ ...ref, amplitude: ref.amplitude / scale });
    const tDist = o.tDist ?? 0, dv = o.dist || 0;
    return {
      params, plantParams, x0: sys.x0(y0), Ts: o.Ts ?? 0.01, tEnd: o.tEnd ?? 10,
      reference: (t) => (t < ref.tStep ? y0 : g(t)),
      disturbance: (t) => (t >= tDist ? dv : 0),
      noise: WB.sim.makeNoise(o.noise || 0, o.seed ?? 1),
      feed: o.feed || imp.feed || 'y',
      extraP: { ...(imp.params ? imp.params(ctx) : {}), ...(o.extraP || {}) },
    };
  }

  function payload(ctx, code, sc) {
    const sys = ctx.sys;
    const N = Math.round(sc.tEnd / sc.Ts) + 1;
    const r = [], d = [], nz = sc.noise ? [] : null;
    for (let k = 0; k < N; k++) {
      const t = k * sc.Ts;
      r.push(sc.reference(t));
      d.push(asArr(sc.disturbance ? sc.disturbance(t) : 0));
      if (nz) nz.push(asArr(sc.noise(k)));
    }
    const P = { ...sc.params, Ts: sc.Ts, ...(sys.pyParams ? sys.pyParams(sc.x0) : {}), ...sc.extraP };
    return {
      params: P, plant: sys.plantPy, plantParams: sc.plantParams, uLimit: ranges(sys, sc.plantParams),
      x0: sc.x0, Ts: sc.Ts, feed: sc.feed, r, d, noise: nz,
    };
  }

  // The Python reply as a WB.sim result (extras: xhat0, xhat1, ..., dhat, dhat1, ...).
  function toResult(out, sc) {
    const N = out.x.length, F = () => new Float64Array(N);
    const cols = (rows) => {
      const w = rows.length ? rows[0].length : 0;
      return Array.from({ length: w }, (_, i) => Float64Array.from(rows, (row) => row[i]));
    };
    const rRows = Array.from({ length: N }, (_, k) => asArr(sc.reference(k * sc.Ts)));
    const res = {
      t: Float64Array.from({ length: N }, (_, k) => k * sc.Ts),
      yAll: cols(out.y), yMeasAll: cols(out.ym), rAll: cols(rRows),
      uDemandAll: cols(out.uD), uAll: cols(out.u), uAppliedAll: cols(out.ua),
      x: out.x, extras: {},
    };
    const addVec = (rows, name) => {
      const w = Math.max(0, ...rows.map((v) => (v ? v.length : 0)));
      for (let i = 0; i < w; i++) {
        const a = F();
        rows.forEach((v, k) => { a[k] = v && v.length > i ? v[i] : NaN; });
        res.extras[i === 0 && name === 'dhat' ? 'dhat' : `${name}${i}`] = a;
      }
    };
    addVec(out.xhat, 'xhat');
    addVec(out.dhat, 'dhat');
    res.y = res.yAll[0]; res.yMeas = res.yMeasAll[0]; res.r = res.rAll[0];
    res.uDemand = res.uDemandAll[0]; res.u = res.uAll[0]; res.uApplied = res.uAppliedAll[0];
    return res;
  }

  const pyError = (out) => ({ ok: false, msg: out.timeout ? out.error : 'Python raised an error.', detail: [out.error, out.where, (out.stdout || '').trim()].filter(Boolean).join('\n') });

  // Run the student's code on a scenario: resolves to a result, or {ok: false, msg, detail}.
  async function run(ctx, code, sc) {
    if (!code || !code.trim()) return { ok: false, msg: 'Write your controller first.' };
    const out = await WB.py.closedLoop(code, payload(ctx, code, sc));
    if (out.error) return pyError(out);
    const res = toResult(out, sc);
    res.stdout = out.stdout;
    return res;
  }

  // Probe a fresh controller open loop: calls = [[r, y], ...] (y an array). Resolves
  // to {u: [[...], ...]} or {ok: false, ...}.
  async function probe(ctx, code, calls, { params = ctx.pModel, Ts = ctx.S.sim.Ts, x0 = ctx.sys.x0(0), extraP = {} } = {}) {
    const imp = implementOf(ctx);
    const P = { ...params, Ts, ...(ctx.sys.pyParams ? ctx.sys.pyParams(x0) : {}), ...(imp.params ? imp.params(ctx) : {}), ...extraP };
    const out = await WB.py.probe(code, { params: P, calls, Ts, m: ranges(ctx.sys, params).length });
    return out.error ? pyError(out) : out;
  }

  // The same scenario with a workbench (JS) controller, for comparison.
  function reference(ctx, sc, controller) {
    const sys = ctx.sys, p = sc.plantParams;
    return WB.sim.normalize(WB.sim.simulate({
      plant: { f: (x, u) => sys.f(x, u, p), h: (x) => sys.h(x, p), uLimit: sys.uLimit(p) },
      controller, reference: sc.reference, disturbance: sc.disturbance, noise: sc.noise,
      x0: sc.x0, Ts: sc.Ts, tEnd: sc.tEnd,
    }));
  }

  // A ctx whose model is the scenario's parameters, in Explore semantics (so the
  // chapter's controllers apply every term), for building reference controllers.
  function refCtx(ctx, sc, extra = {}) {
    const sys = ctx.sys, p = sc.params;
    return {
      ...ctx, pModel: p,
      model: sys.secondOrderModel ? sys.secondOrderModel(p) : null,
      ss: sys.stateSpace ? sys.stateSpace(p) : null,
      S: { ...ctx.S, mode: 'explore', sim: { ...ctx.S.sim, Ts: sc.Ts } },
      ...extra,
    };
  }

  // Largest |a.y − b.y| (output i) over samples with t ≥ t0, and when it happens.
  function maxDiff(a, b, { output = 0, t0 = 0 } = {}) {
    const ya = a.yAll[output], yb = b.yAll[output];
    let e = 0, tAt = 0;
    for (let k = 0; k < Math.min(ya.length, yb.length); k++) {
      if (a.t[k] < t0) continue;
      const v = Math.abs(ya[k] - yb[k]);
      if (!(v <= e)) { e = v; tAt = a.t[k]; }
    }
    return { e, t: tAt };
  }

  // Mean of fn(k) over samples with t in [t0, t1].
  function mean(res, t0, t1, fn) {
    let s = 0, n = 0;
    for (let k = 0; k < res.t.length; k++) if (res.t[k] >= t0 - 1e-9 && res.t[k] <= t1 + 1e-9) { s += fn(k); n++; }
    return n ? s / n : NaN;
  }

  // The nominal parameters and the system's second set (sys.altParams), labelled.
  function paramCases(ctx) {
    const p = ctx.pModel, q = ctx.sys.altParams ? ctx.sys.altParams(p) : null;
    const lab = (k) => (ctx.sys.params.find((x) => x.key === k) || { label: k }).label.replace(/<[^>]+>/g, '');
    const out = [{ params: p, label: 'nominal parameters' }];
    if (q) out.push({ params: q, label: ctx.sys.uncertain.map((k) => `${lab(k)} = ${M.fmt(q[k], 3)}`).join(', ') });
    return out;
  }

  // Run the student's code on each case and compare the output with a reference
  // run. cases: [{ sc, ref: () -> WB.sim result, label }], the first with the
  // nominal parameters; tol in SI. Resolves to {ok, msg, detail?}.
  async function matchCheck(ctx, code, cases, { tol, t0 = 0, what = 'the design' }) {
    const out = ctx.sys.output || ctx.sys.outputs[0];
    const k = out.scale || 1, unit = out.unit === '°' ? '°' : ` ${out.unit}`;
    const f = (v) => `${M.fmt(v * k, 3)}${unit}`;
    let worst = 0;
    for (let i = 0; i < cases.length; i++) {
      const c = cases[i];
      const mine = await run(ctx, code, c.sc);
      if (mine.ok === false) return mine;
      const ref = c.ref();
      const d = maxDiff(mine, ref, { t0 });
      const detail = (mine.stdout || '').trim() ? `print output:\n${mine.stdout.trim()}` : '';
      if (!(d.e <= tol)) {
        if (i > 0) {
          const keys = ctx.sys.uncertain.map((q) => `P.${q}`).join(', ');
          return { ok: false, msg: `Matches ${what} with the nominal parameters but not with ${c.label} (off by ${f(d.e)}). Compute the gains from ${keys} rather than numbers.`, detail };
        }
        const n = mine.t.length - 1;
        const off = mine.r[n] - mine.y[n], refOff = ref.r[n] - ref.y[n];
        let msg = `Your ${out.label} differs from ${what} by ${f(d.e)} at t = ${M.fmt(d.t, 3)} s.`;
        if (Math.abs(off - refOff) > tol) msg += ` At the end it is ${f(off)} from the reference (${what}: ${f(refOff)}).`;
        return { ok: false, msg, detail };
      }
      worst = Math.max(worst, d.e);
    }
    return { ok: true, msg: `Your ${out.label}(t) matches ${what} to within ${f(worst)} (${cases.map((c) => c.label).join('; ')}).` };
  }

  // ------------------------------------------------- Work-mode simulation --
  function setActive(ctx, code, part) {
    active[slot(ctx)] = { code, part };
    WB.ui.store.set(ACTIVE_KEY, active);
    const r = runs[slot(ctx)];
    if (r) r.failed = null;
  }
  const isActive = (ctx) => !!active[slot(ctx)];

  function zeroInput(ctx, common, plant) {
    const n = ctx.sys.inputs ? ctx.sys.inputs.length : 1;
    const zero = n === 1 ? 0 : new Array(n).fill(0);
    return WB.sim.simulate({ ...common, plant, controller: { update: () => zero } });
  }

  // app.js calls this in Work mode for chapters with `implement`. It returns
  // synchronously: the student's latest run for these settings, an older run while
  // a new one is computed, or the zero-input plant.
  function simulate(ctx, common, plant) {
    const a = active[slot(ctx)];
    if (!a) return zeroInput(ctx, common, plant);
    const sc = displayScenario(ctx, common);
    const N = Math.round(sc.tEnd / sc.Ts) + 1;
    const key = JSON.stringify([a.code, ctx.S.sim, ctx.pModel, ctx.pTrue, sc.extraP]);
    const r = (runs[slot(ctx)] = runs[slot(ctx)] || {});
    if (r.key !== key && r.pending !== key && r.failed !== key) {
      r.pending = key;
      r.promise = run(ctx, a.code, sc).then((d) => {
        if (r.pending !== key) return d;
        r.pending = null;
        if (d.ok === false) { r.failed = key; r.err = d; } else { r.data = d; r.key = key; r.err = null; }
        ctx.update();
        return d;
      }, (e) => { r.pending = null; r.failed = key; r.err = { ok: false, msg: String(e.message || e) }; ctx.update(); return r.err; });
    }
    if (r.key === key) return r.data;
    return r.data && r.data.t.length === N && !r.err ? r.data : zeroInput(ctx, common, plant);
  }

  // "Run my controller": make this code drive the plots; resolves once it has run.
  function use(ctx, code, part) {
    if (!code.trim()) return { ok: false, msg: 'Write your controller first.' };
    setActive(ctx, code, part);
    ctx.update();
    const r = runs[slot(ctx)];
    if (!r || !r.pending) return r && r.err ? r.err : { info: true, msg: 'Your controller drives the simulation (time plots above).' };
    return r.promise.then((d) => (d.ok === false ? d : { info: true, msg: 'Your controller drives the simulation (time plots above).' }));
  }

  function status(ctx) {
    const a = active[slot(ctx)];
    if (!a) return { state: 'off' };
    const r = runs[slot(ctx)] || {};
    if (r.pending) return { state: WB.py.status() === 'ready' ? 'running' : 'loading' };
    if (r.err) return { state: 'error', msg: r.err.msg, detail: r.err.detail };
    return { state: 'on', part: a.part };
  }

  // Control-panel note for Work mode: what the plots show.
  function banner(parent, ctx, partLabel) {
    const { el } = WB.ui;
    const p = el('p', { class: 'muted small' });
    const pre = el('pre', { class: 'py-detail', hidden: '' });
    parent.append(p, pre);
    WB.ui.addRefresher(() => {
      const s = status(ctx);
      pre.hidden = s.state !== 'error' || !s.detail;
      if (s.state === 'error') pre.textContent = s.detail || '';
      p.textContent = {
        off: `The plots show the plant with zero input until you run your controller: write it in ${partLabel} and press Run my controller.`,
        loading: 'Starting Python (the first time downloads about 15 MB)…',
        running: 'Running your controller…',
        on: 'The time plots show your controller (your latest Run or Check). They rerun when you change the left panel.',
        error: `Your controller stopped with an error, so the plots show the plant with zero input. ${s.msg || ''}`,
      }[s.state];
    });
  }

  // ------------------------------------------------------- problem parts --
  // Saved code of another part ('A.7/d'), or null.
  function savedCode(ctx, key) {
    const [probId, partId] = key.split('/');
    const saved = WB.ui.store.get(`wb.${ctx.sys.id}.${probId}.answers`, {});
    const c = saved[`${partId}.code`];
    return c && c.trim() ? c : null;
  }

  // Bare template: the class and the update signature (what the simulation passes
  // in), nothing about the controller's structure.
  function template(ctx, feed) {
    const py = ctx.sys.py;
    const arg = feed === 'state' ? 'x' : 'y';
    const what = feed === 'state'
      ? `x = np.array([[${py.x.join('], [')}]]): the state`
      : `y = np.array([[${py.y.join('], [')}]]): the measured output${py.y.length > 1 ? 's' : ''}`;
    return `class Controller:\n    def __init__(self):\n        pass\n\n    def update(self, ${py.r}, ${arg}):\n        # ${what}\n        ${py.u} = 0.0\n        return ${py.u}\n`;
  }

  // A Python part whose answer is a controller. spec: { id, title, html, seed:
  // 'A.7/d' (start from that part's saved code), feed, check(code) -> Promise,
  // solution, after }.
  function part(ctx, spec) {
    const feed = spec.feed || implementOf(ctx).feed || 'y';
    const seeds = [].concat(spec.seed || []);
    const start = seeds.map((k) => savedCode(ctx, k)).find(Boolean) || template(ctx, feed);
    return {
      id: spec.id, title: spec.title, html: spec.html, after: spec.after, solution: spec.solution,
      code: {
        template: start,
        check: (code) => {
          if (!code.trim()) return Promise.resolve({ ok: false, msg: 'Write your controller first.' });
          setActive(ctx, code, spec.id);
          ctx.update();
          return spec.check(code);
        },
        actions: [{ label: 'Run my controller', run: (code) => use(ctx, code, spec.id) }],
      },
    };
  }

  // Python help text for the implementation parts.
  function helpHtml(ctx) {
    const sys = ctx.sys;
    const extra = sys.pyParams ? Object.keys(sys.pyParams(sys.x0(0))) : [];
    return `Controller parts: write <code>class Controller</code> with <code>__init__(self)</code> and <code>update(self, ${sys.py.r}, y)</code>, as in the book's controller classes. `
      + `<code>P</code> also holds <code>P.Ts</code>${extra.map((k) => `, <code>P.${k}</code>`).join('')}. `
      + '<code>import control as cnt</code> gives <code>cnt.place</code> (one input), <code>cnt.ctrb</code> and <code>cnt.obsv</code>. '
      + 'Return the input, or a tuple (u, x̂) or (u, x̂, d̂) to have your estimates plotted.';
  }

  return { scenario, run, probe, matchCheck, paramCases, reference, refCtx, maxDiff, mean, simulate, use, status, banner, part, savedCode, setActive, isActive, template, helpHtml, ranges };
})();
