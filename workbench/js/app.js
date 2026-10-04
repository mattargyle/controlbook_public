// Page glue: state, panels, simulation runs, plots, playback, and the live-math strip.
//
// Studies register as WB.studies[id] = { chapters: { chN: {...} } } with their
// system in WB.systems[id]. Study A's chapters live in WB.chapters (the original
// flat registry) and are registered here. Systems may be single-channel (the arm:
// output/input/h(x) scalar) or multi-channel (outputs[]/inputs[]/refs[]/...; see
// STUDY_GUIDE.md). Channel 0 of every signal uses the scalar S.sim fields
// (type, amplitude, y0, dist, noise ...), so single-channel chapters are unchanged.
window.WB = window.WB || {};

(function () {
  const { el, slider, segmented, section, renderTex, store } = WB.ui;
  const M = WB.math;
  WB.studies = WB.studies || {};
  if (!WB.studies.A) {
    WB.studies.A = { chapters: Object.fromEntries(Object.entries(WB.chapters).filter(([, v]) => v && v.num !== undefined)) };
  }
  const STUDY_IDS = ['A', 'B', 'C', 'D', 'E', 'F'].filter((id) => WB.systems[id] && WB.studies[id] && Object.keys(WB.studies[id].chapters || {}).length);

  const PARTS = [
    { label: 'Models', nums: [2, 6] }, { label: 'PID', nums: [7, 10.9] },
    { label: 'Observers', nums: [11, 14] }, { label: 'Loopshaping', nums: [15, 18] },
  ];
  const chaptersOf = (id) => WB.studies[id].chapters;
  const chapterIds = (id) => Object.keys(chaptersOf(id)).filter((k) => chaptersOf(id)[k].num !== undefined)
    .sort((a, b) => chaptersOf(id)[a].num - chaptersOf(id)[b].num);

  // ------------------------------------------------------ system channel view --
  // Normalized description of a system's signals. Single-channel systems (the arm)
  // get one output, input, reference, initial condition, and disturbance.
  function view(s) {
    const outputs = s.outputs || [{ key: 'y', minSpan: 2, ...s.output }];
    const inputs = s.inputs || [{ key: 'u', ...s.input }];
    const refs = s.refs || [{ output: 0, label: 'reference r', unit: outputs[0].unit, scale: outputs[0].scale, min: -90, max: 90 }];
    const initial = s.initial || [{ key: 'y0', label: `initial ${outputs[0].label}`, unit: outputs[0].unit, scale: outputs[0].scale, output: 0, min: -90, max: 90 }];
    const dists = s.disturbances || [{ input: 0, label: 'd', unit: inputs[0].unit, min: -1, max: 1 }];
    return { outputs, inputs, refs, initial, dists, multi: !!s.outputs };
  }

  // Input limits as an array of [lo, hi] per input.
  function ranges(s, p, nIn) {
    const L = s.uLimit(p);
    return Array.from({ length: nIn }, (_, i) => {
      const v = Array.isArray(L) ? L[i] : L;
      return Array.isArray(v) ? v : [-v, v];
    });
  }

  // Every chapter starts from these simulation settings, then applies its own.
  // Channel 0 uses the scalar fields; channels 1.. use refs/init/dists/noises.
  const SIM_BASE = { type: 'step', amplitude: 30, frequency: 0.2, tStep: 0.5, y0: 0, dist: 0, tDist: 3, tEnd: 4, Ts: 0.01, noise: 0, refs: [], init: {}, dists: [], noises: [] };
  function chapterSim(c, sysId) {
    const s = WB.systems[sysId];
    const { mismatch, ...sim } = chaptersOf(sysId)[c].simDefaults(s);
    const v = view(s);
    const base = { ...SIM_BASE, ...(s.simBase || {}) };
    // defaults for extra channels
    base.refs = v.refs.slice(1).map((r) => ({ type: 'step', amplitude: 0, frequency: 0.2, tStep: 0.5, ...(r.defaults || {}) }));
    base.init = Object.fromEntries(v.initial.slice(1).map((q) => [q.key, q.value || 0]));
    base.dists = v.dists.slice(1).map(() => 0);
    base.noises = v.outputs.slice(1).map(() => 0);
    if (v.refs[0].defaults) Object.assign(base, v.refs[0].defaults);
    if (v.initial[0].value !== undefined) base.y0 = v.initial[0].value;
    const out = { ...base, ...sim };
    // chapters may override extra channels partially
    if (sim.refs) out.refs = base.refs.map((r, i) => ({ ...r, ...(sim.refs[i] || {}) }));
    if (sim.init) out.init = { ...base.init, ...sim.init };
    if (sim.dists) out.dists = base.dists.map((d, i) => (sim.dists[i] ?? d));
    if (sim.noises) out.noises = base.noises.map((n, i) => (sim.noises[i] ?? n));
    return out;
  }
  function applyChapterDefaults(c) {
    const s = WB.systems[S.sysId];
    const seed = S.sim.seed;
    S.sim = { ...chapterSim(c, S.sysId), seed };
    const mm = chaptersOf(S.sysId)[c].simDefaults(s).mismatch || {};
    S.mismatch = Object.fromEntries(s.uncertain.map((k) => [k, mm[k] || 0]));
  }
  const STATE_KEY = 'wb.state.v2';

  // ------------------------------------------------------------------ state --
  function freshState(sysId = 'A', chapter = 'ch7') {
    const sys = WB.systems[sysId];
    const ids = chapterIds(sysId);
    if (!ids.includes(chapter)) {
      // same chapter number if this study has it, else the next one up
      const num = parseFloat(String(chapter).replace(/^ch/, '')) || 7;
      chapter = ids.find((k) => chaptersOf(sysId)[k].num >= num) || ids[0];
    }
    const S = {
      sysId, chapter, mode: 'work',
      p: Object.fromEntries(sys.params.map((q) => [q.key, q.value])),
      mismatch: Object.fromEntries(sys.uncertain.map((k) => [k, 0])),
      alpha: 0.2,
      sim: { ...chapterSim(chapter, sysId), seed: 1 },
      ch: {},
    };
    for (const c of ids) S.ch[c] = chaptersOf(sysId)[c].defaults(sys);
    Object.assign(S.mismatch, chaptersOf(sysId)[chapter].simDefaults(sys).mismatch || {});
    return S;
  }

  let S = (() => {
    const saved = store.get(STATE_KEY, null);
    const base = freshState();
    if (!saved || !STUDY_IDS.includes(saved.sysId)) return base;
    const b = freshState(saved.sysId, saved.chapter);
    const ids = chapterIds(saved.sysId);
    // shallow-merge so new fields added later still get defaults
    const merged = { ...b, ...saved, p: { ...b.p, ...saved.p }, mismatch: { ...b.mismatch, ...saved.mismatch }, sim: { ...b.sim, ...saved.sim }, ch: { ...b.ch } };
    for (const c of ids) merged.ch[c] = { ...b.ch[c], ...(saved.ch || {})[c] };
    if (!ids.includes(merged.chapter)) merged.chapter = b.chapter;
    return merged;
  })();

  const revealed = new Set();
  let ctx = null, result = null, linResult = null, metrics = null;
  const viewState = { playIndex: 0, hoverIndex: null, playing: false, speed: 1, lastFrame: 0 };

  const chapter = () => chaptersOf(S.sysId)[S.chapter];
  const sys = () => WB.systems[S.sysId];

  // One long-lived ctx object, refreshed in place, so closures created when the
  // panels were built (problem checks, drag handlers) always see current values.
  function refreshCtx() {
    const s = sys();
    const pModel = { ...S.p, ...s.constants };
    const pTrue = { ...pModel };
    for (const k of s.uncertain) pTrue[k] = pModel[k] * (1 + S.mismatch[k] / 100);
    ctx = ctx || {};
    Object.assign(ctx, {
      sys: s, S, st: S.ch[S.chapter], pModel, pTrue, chan: view(s),
      model: s.secondOrderModel ? s.secondOrderModel(pModel) : null,
      ss: s.stateSpace ? s.stateSpace(pModel) : null,
      update, app,
    });
    ctx.gains = chapter().gains ? chapter().gains(ctx) : {};
  }

  // -------------------------------------------------------------- simulate --
  function buildSignals() {
    const s = sys(), v = view(s), sim = S.sim;
    const refCfg = (i) => (i === 0 ? sim : sim.refs[i - 1]);
    const initVal = (q, i) => (i === 0 ? sim.y0 : (sim.init[q.key] ?? q.value ?? 0));
    // value of the reference before it starts: the matching initial condition
    const pre = v.refs.map((rf) => {
      const qi = v.initial.findIndex((q) => q.output === rf.output);
      return qi >= 0 ? initVal(v.initial[qi], qi) / (v.initial[qi].scale || 1) : 0;
    });
    const gens = v.refs.map((rf, i) => {
      const c = refCfg(i);
      const g = WB.sim.makeReference({ type: c.type, amplitude: c.amplitude / (rf.scale || 1), offset: rf.offset || 0, frequency: c.frequency, tStep: c.tStep });
      return (t) => (t < c.tStep ? pre[i] : g(t));
    });
    const reference = gens.length === 1 ? gens[0] : (t) => gens.map((g) => g(t));
    const nIn = v.inputs.length;
    const dVals = v.dists.map((d, i) => (i === 0 ? sim.dist : sim.dists[i - 1] || 0));
    const disturbance = (t) => {
      if (t < sim.tDist) return nIn === 1 ? 0 : new Array(nIn).fill(0);
      if (nIn === 1) return dVals[0];
      const arr = new Array(nIn).fill(0);
      v.dists.forEach((d, i) => { arr[d.input] += dVals[i]; });
      return arr;
    };
    const nOut = v.outputs.length;
    const sig = v.outputs.map((o, i) => (i === 0 ? sim.noise : sim.noises[i - 1] || 0) / (o.scale || 1));
    const gensN = sig.map((sg, i) => WB.sim.makeNoise(sg, sim.seed + 1000 * i));
    const noise = nOut === 1 ? gensN[0] : (gensN.some(Boolean) ? (k) => gensN.map((g) => (g ? g(k) : 0)) : null);
    let x0;
    if (s.initial) {
      const init = Object.fromEntries(v.initial.map((q, i) => [q.key, initVal(q, i) / (q.scale || 1)]));
      x0 = s.x0(init, ctx.pModel);
    } else {
      x0 = s.x0(sim.y0 / (v.outputs[0].scale || 1));
    }
    return { reference, disturbance, noise, x0 };
  }

  function run() {
    refreshCtx();
    const s = ctx.sys, v = ctx.chan;
    const sigs = buildSignals();
    const common = { ...sigs, Ts: S.sim.Ts, tEnd: S.sim.tEnd };
    if (chapter().reference) common.reference = chapter().reference(ctx, sigs.reference);

    const plantTrue = { f: (x, u) => s.f(x, u, ctx.pTrue), h: (x) => s.h(x, ctx.pTrue), uLimit: s.uLimit(ctx.pTrue) };
    // Work mode in a chapter with an implementation part runs the student's own
    // Python controller (WB.myCtrl), or the plant with zero input until they run one.
    if (S.mode === 'work' && chapter().implement) result = WB.myCtrl.simulate(ctx, common, plantTrue);
    else if (chapter().simulate) result = chapter().simulate(ctx, common, plantTrue);
    else result = WB.sim.simulate({ ...common, plant: plantTrue, controller: chapter().controller(ctx) });

    result = WB.sim.normalize(result);

    // Linear design model: the same controller (without its feedforward term) on
    // xdot = A x + B u with nominal parameters, no saturation, no disturbance and
    // no noise. Single-channel systems get it by default; others via linearSim.
    const imp = chapter().implement;
    if (chapter().linear === false || (S.mode === 'work' && imp && imp.linear === false)) {
      linResult = null;
    } else if (chapter().linearSim) {
      linResult = chapter().linearSim(ctx, common);
    } else if (!v.multi && ctx.ss) {
      const { A, B } = ctx.ss;
      linResult = WB.sim.simulate({
        ...common,
        disturbance: null, noise: null,
        plant: { f: (x, u) => A.map((row, i) => row.reduce((acc, a, j) => acc + a * x[j], 0) + B[i][0] * u), h: (x) => x[0], uLimit: Infinity },
        controller: chapter().controller(ctx, { linear: true }),
      });
    } else {
      linResult = null;
    }

    linResult = WB.sim.normalize(linResult);

    // Metrics on the primary reference's output, from its step to the next
    // reference change or the disturbance.
    const N = result.t.length;
    const oi = v.refs[0].output;
    const yP = result.yAll[oi] || result.y, rP = result.rAll[0] || result.r;
    const idx = (t) => Math.max(0, Math.min(N, Math.round(t / S.sim.Ts)));
    const i0 = idx(S.sim.tStep);
    let i1 = N;
    if (S.sim.type === 'square') i1 = Math.min(i1, idx(S.sim.tStep + 0.5 / S.sim.frequency));
    const anyD = S.sim.dist !== 0 || (S.sim.dists || []).some((d) => d !== 0);
    if (anyD && S.sim.tDist > S.sim.tStep) i1 = Math.min(i1, idx(S.sim.tDist));
    metrics = M.stepMetrics(result.t, yP, i0, i1, yP[Math.min(i0, N - 1)], rP[Math.min(i0, N - 1)]);
    metrics.i0 = i0; metrics.i1 = i1; metrics.output = oi;
    const rg = ranges(s, ctx.pModel, result.uDemandAll.length);
    let worst = 0, worstIn = 0, satCount = 0;
    for (let k = 0; k < N; k++) {
      let satK = false;
      result.uDemandAll.forEach((ud, i) => {
        const [lo, hi] = rg[i];
        const half = (hi - lo) / 2 || 1, mid = (hi + lo) / 2;
        const ratio = Math.abs(ud[k] - mid) / half;
        if (ratio > worst) { worst = ratio; worstIn = i; }
        if (ud[k] > hi + 1e-9 || ud[k] < lo - 1e-9) satK = true;
      });
      if (satK) satCount++;
    }
    let peak = 0;
    for (let k = 0; k < N; k++) peak = Math.max(peak, Math.abs(result.uDemandAll[worstIn][k]));
    metrics.peakU = peak; metrics.peakIn = worstIn; metrics.peakRatio = worst; metrics.satFrac = satCount / N;
    if (viewState.playIndex >= N) viewState.playIndex = N - 1;
  }

  // ----------------------------------------------------------------- panels --
  function buildLeft() {
    const root = document.getElementById('left');
    root.replaceChildren();
    const s = sys(), v = view(s);

    const plant = section(root, 'Plant (nominal model)', `p. ${s.introPage}`);
    for (const q of s.params) {
      slider(plant, { label: q.label, unit: q.unit, min: q.min, max: q.max, step: q.step, get: () => S.p[q.key], set: (val) => { S.p[q.key] = val; update(); } });
    }
    plant.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Book values', onclick: () => { for (const q of s.params) S.p[q.key] = q.value; update(); } }));

    const mis = section(root, 'True plant vs. model');
    mis.append(el('p', { class: 'muted small', text: 'The simulated plant differs from the model the controller was designed with.' }));
    for (const k of s.uncertain) {
      const q = s.params.find((pp) => pp.key === k);
      slider(mis, { label: 'Δ' + q.label, unit: '%', min: -50, max: 50, step: 1, sig: 3, get: () => S.mismatch[k], set: (val) => { S.mismatch[k] = val; update(); } });
    }
    slider(mis, { label: 'α', min: 0, max: 0.5, step: 0.01, sig: 3, get: () => S.alpha, set: (val) => { S.alpha = val; WB.ui.refreshAll(); } });
    mis.append(el('div', { class: 'btn-row' },
      el('button', { type: 'button', class: 'btn', text: 'Randomize ±α', onclick: () => { for (const k of s.uncertain) S.mismatch[k] = Math.round(100 * S.alpha * (2 * Math.random() - 1)); update(); } }),
      el('button', { type: 'button', class: 'btn btn-quiet', text: 'Exact model', onclick: () => { for (const k of s.uncertain) S.mismatch[k] = 0; update(); } })));

    v.refs.forEach((rf, i) => {
      const cfg = () => (i === 0 ? S.sim : S.sim.refs[i - 1]);
      const sec = section(root, v.refs.length > 1 ? `Reference: ${rf.label}` : 'Reference');
      segmented(sec, { label: 'Signal', options: [{ value: 'step', label: 'step' }, { value: 'square', label: 'square' }, { value: 'sine', label: 'sine' }], get: () => cfg().type, set: (val) => { cfg().type = val; update(); } });
      slider(sec, { label: 'amplitude', unit: rf.unit, min: rf.min ?? -90, max: rf.max ?? 90, step: rf.step ?? 1, sig: 3, get: () => cfg().amplitude, set: (val) => { cfg().amplitude = val; update(); } });
      slider(sec, { label: 'starts at', unit: 's', min: 0, max: 10, step: 0.05, sig: 3, get: () => cfg().tStep, set: (val) => { cfg().tStep = val; update(); } });
      slider(sec, { label: 'frequency', unit: 'Hz', min: 0.01, max: 2, step: 0.01, sig: 3, get: () => cfg().frequency, set: (val) => { cfg().frequency = val; update(); }, disabled: () => cfg().type === 'step' });
    });
    const ini = section(root, 'Initial conditions');
    v.initial.forEach((q, i) => {
      slider(ini, { label: q.label, unit: q.unit, min: q.min ?? -90, max: q.max ?? 90, step: q.step ?? 1, sig: 3,
        get: () => (i === 0 ? S.sim.y0 : S.sim.init[q.key]), set: (val) => { if (i === 0) S.sim.y0 = val; else S.sim.init[q.key] = val; update(); } });
    });

    const dist = section(root, 'Disturbance & noise');
    v.dists.forEach((d, i) => {
      slider(dist, { label: d.label, unit: d.unit, min: d.min ?? -1, max: d.max ?? 1, step: d.step ?? 0.01, sig: 3,
        get: () => (i === 0 ? S.sim.dist : S.sim.dists[i - 1]), set: (val) => { if (i === 0) S.sim.dist = val; else S.sim.dists[i - 1] = val; update(); } });
    });
    slider(dist, { label: 'd starts at', unit: 's', min: 0, max: 20, step: 0.1, sig: 3, get: () => S.sim.tDist, set: (val) => { S.sim.tDist = val; update(); } });
    v.outputs.forEach((o, i) => {
      slider(dist, { label: `noise σ ${v.outputs.length > 1 ? o.label : ''}`.trim(), unit: o.unit, min: 0, max: o.noiseMax ?? 2, step: (o.noiseMax ?? 2) / 200, sig: 3,
        hint: i === v.outputs.length - 1 ? 'Gaussian noise on the measured outputs. Controllers that use the true state ignore it.' : undefined,
        get: () => (i === 0 ? S.sim.noise : S.sim.noises[i - 1]), set: (val) => { if (i === 0) S.sim.noise = val; else S.sim.noises[i - 1] = val; update(); } });
    });
    dist.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'New noise sample', onclick: () => { S.sim.seed = (S.sim.seed % 100000) + 1; update(); } }));

    const simS = section(root, 'Simulation');
    slider(simS, { label: 't<sub>end</sub>', unit: 's', min: 1, max: 100, step: 0.5, sig: 3, get: () => S.sim.tEnd, set: (val) => { S.sim.tEnd = val; update(); } });
    slider(simS, { label: 'T<sub>s</sub>', unit: 's', min: 0.001, max: 0.05, step: 0.001, sig: 3, get: () => S.sim.Ts, set: (val) => { S.sim.Ts = val; update(); } });
    simS.append(el('button', {
      type: 'button', class: 'btn btn-quiet', text: 'Reset this chapter',
      onclick: () => {
        S.ch[S.chapter] = chapter().defaults(s);
        applyChapterDefaults(S.chapter);
        rebuild();
      },
    }));
  }

  function buildRight() {
    const root = document.getElementById('right');
    root.replaceChildren();
    chapter().buildControls(root, ctx);

    if (chapter().metrics !== false) {
      const met = section(root, 'Step response', 'p. 113');
      const table = el('div', { class: 'metrics' });
      met.append(table);
      WB.ui.addRefresher(() => renderMetrics(table));
    }

    const prob = document.getElementById('problem');
    prob.replaceChildren();
    const panes = {
      problem: el('div', { class: 'problem', role: 'tabpanel' }),
      python: el('div', { class: 'help-pane', role: 'tabpanel' }),
      params: el('div', { class: 'help-pane', role: 'tabpanel' }),
    };
    const bar = el('div', { class: 'problem-tabs', role: 'tablist', 'aria-label': 'Problem panel' });
    const tabBtns = [['problem', 'Problem'], ['python', 'Python help'], ['params', 'Parameters <code>P</code>']].map(([k, label]) => {
      const b = el('button', { type: 'button', role: 'tab', class: 'tab' });
      b.innerHTML = label;
      b.addEventListener('click', () => { problemTab = k; showTab(); });
      bar.append(b);
      return [k, b];
    });
    const showTab = () => {
      for (const [k, b] of tabBtns) {
        b.classList.toggle('on', k === problemTab);
        b.setAttribute('aria-selected', String(k === problemTab));
        panes[k].hidden = k !== problemTab;
      }
    };
    prob.append(bar, panes.problem, panes.python, panes.params);
    chapter().buildProblem(panes.problem, ctx);
    buildPythonHelp(panes.python);
    buildParamsHelp(panes.params);
    showTab();
  }

  // Which tab of the problem card is showing; kept across chapter switches.
  let problemTab = 'problem';
  // Constants a system carries outside its sliders (sys.constants).
  const CONSTANT_INFO = { g: { label: 'g', unit: 'm/s²', desc: 'gravitational acceleration' } };

  function buildPythonHelp(root) {
    const p = (html) => { const n = el('p'); n.innerHTML = html; root.append(n); };
    p('Python answers run in your browser (Python 3.14 + numpy, loaded on the first Check).');
    p(`<code>np</code> (numpy) and <code>math</code> are imported, and <code>P</code> holds the study's parameters (see the Parameters tab), for example <code>P.${sys().params[0].key}</code>.`);
    p('Write answers with these, not numbers: they are checked with other parameter values too.');
    if (chapter().implement && sys().py) p(WB.myCtrl.helpHtml(ctx));
  }

  function buildParamsHelp(root) {
    const s = sys();
    const intro = el('p');
    intro.innerHTML = '<code>P</code> in Python answers holds these. Values are the current nominal parameters (left panel).';
    const rows = [
      ...s.params.map((q) => ({ key: q.key, label: q.label, unit: q.unit, desc: q.desc || '' })),
      ...Object.keys(s.constants || {}).map((k) => ({ key: k, ...(CONSTANT_INFO[k] || { label: k, unit: '', desc: '' }) })),
    ];
    const tbody = el('tbody');
    const valueCells = rows.map((r) => {
      const name = el('td'); name.append(el('code', { text: `P.${r.key}` }));
      const sym = el('td'); sym.innerHTML = r.label; // authored text (may contain <sub>)
      const val = el('td', { class: 'num' });
      tbody.append(el('tr', {}, name, sym, el('td', { text: r.desc }), val, el('td', { text: r.unit })));
      return [r.key, val];
    });
    const head = el('tr', {}, ...['Python', 'Symbol', 'Meaning', 'Value', 'Unit'].map((t) => el('th', { text: t })));
    root.append(intro, el('div', { class: 'table-scroll' }, el('table', { class: 'param-table' }, el('thead', {}, head), tbody)));
    WB.ui.addRefresher(() => { for (const [k, td] of valueCells) td.textContent = M.fmt(ctx.pModel[k], 4); });
  }

  function renderMetrics(table) {
    if (!metrics) return;
    const s = sys(), v = view(s);
    const out = v.outputs[metrics.output];
    const inp = v.inputs[metrics.peakIn];
    const rows = [];
    const row = (label, value, status) => rows.push(WB.ui.metric(label, value, status));
    const tg = chapter().targets ? chapter().targets(ctx) : {};
    const trTarget = tg.tr || null;
    const label = v.multi ? ` (${out.label})` : '';
    row(`rise time (10–90%)${label}`, isNaN(metrics.tr) ? '—' : `${M.fmt(metrics.tr, 3)} s`,
      trTarget && !isNaN(metrics.tr) ? { ok: metrics.tr <= trTarget * 1.1, text: `target ${M.fmt(trTarget, 3)} s` } : null);
    const zeta = tg.zeta || null;
    const osExp = zeta && zeta < 1 ? 100 * Math.exp(-zeta * Math.PI / Math.sqrt(1 - zeta * zeta)) : zeta ? 0 : null;
    row('overshoot', isNaN(metrics.os) ? '—' : `${M.fmt(metrics.os, 3)} %`,
      osExp !== null && !isNaN(metrics.os) ? { ok: metrics.os <= osExp + 2, text: `2nd-order ζ predicts ${M.fmt(osExp, 2)} %` } : null);
    row('settling (2%)', isNaN(metrics.ts) ? 'not settled' : `${M.fmt(metrics.ts, 3)} s`);
    row('error at window end', isNaN(metrics.ess) ? '—' : `${M.fmt(metrics.ess * (out.scale || 1), 3)} ${out.unit}`);
    const ok = metrics.satFrac === 0;
    row(`peak |${inp.label}| demanded`, `${M.fmt(metrics.peakU, 3)} ${inp.unit}`,
      { ok, text: ok ? 'within limit' : `saturated ${M.fmt(100 * metrics.satFrac, 2)}% of the time` });
    table.replaceChildren(...rows);
  }

  // ----------------------------------------------------------------- views --
  let yPlots = [], uPlot, xPlot, splane, bodePlot, armCanvas, scrub, timeLabel, armReadout, animTitle;

  function buildCenter() {
    armCanvas = el('canvas', { class: 'plot-canvas', role: 'img', 'aria-label': 'animation' });
    armReadout = el('div', { class: 'arm-readout' });
    const armBox = el('div', { class: 'plot-box anim-box' }, armCanvas, armReadout);
    new ResizeObserver(drawArm).observe(armBox);
    const playBtn = el('button', { type: 'button', class: 'btn icon-btn', 'aria-label': 'Play', text: '▶' });
    playBtn.addEventListener('click', () => togglePlay(playBtn));
    scrub = el('input', { type: 'range', min: 0, max: 1, step: 1, value: 0, 'aria-label': 'time' });
    scrub.addEventListener('input', () => { viewState.playIndex = +scrub.value; viewState.playFloat = undefined; drawCursor(); });
    timeLabel = el('span', { class: 'time-label' });
    const speed = el('select', { 'aria-label': 'playback speed' },
      ...[0.25, 0.5, 1, 2].map((val) => el('option', { value: val, text: `${val}×`, ...(val === 1 ? { selected: '' } : {}) })));
    speed.addEventListener('change', () => { viewState.speed = +speed.value; });
    animTitle = el('span', { class: 'plot-title' });
    document.getElementById('anim').replaceChildren(
      el('div', { class: 'plot-head' }, animTitle),
      armBox, el('div', { class: 'player' }, playBtn, scrub, timeLabel, speed));

    const sp = document.getElementById('splane'); sp.replaceChildren();
    splane = new WB.plot.SPlane(sp, { title: 's-plane' });
    splane.onDrag = (id, re, im) => chapter().onPoleDrag && chapter().onPoleDrag(ctx, id, re, im);

    for (const id of ['anim', 'splane']) collapsibleCard(document.getElementById(id), id);

    const pu = document.getElementById('plot-u'); pu.replaceChildren();
    uPlot = new WB.plot.TimePlot(pu, { title: '' });
    const px = document.getElementById('plot-x'); px.replaceChildren();
    xPlot = new WB.plot.TimePlot(px, { title: '' });
    const bd = document.getElementById('bode'); bd.replaceChildren();
    bodePlot = new WB.plot.BodePlot(bd, { title: 'Bode' });
    for (const id of ['plot-u', 'plot-x', 'bode']) collapsibleCard(document.getElementById(id), id);
    for (const p of [uPlot, xPlot]) p.onHover = (i) => { viewState.hoverIndex = i; drawCursor(); };
  }

  // Plot cards collapse like the panel sections: the title becomes the toggle and
  // everything below the head (plot, player) hides.
  function collapsibleCard(card, key) {
    const head = card.querySelector('.plot-head');
    const body = [...card.children].filter((c) => c !== head);
    WB.ui.collapsible(head, head.querySelector('.plot-title'), card, body, `wb.collapsed.card.${key}`);
  }

  // One card per output; rebuilt when the study changes.
  let yPlotsFor = null;
  function buildOutputPlots() {
    const s = sys(), v = view(s);
    if (yPlotsFor === S.sysId) return;
    yPlotsFor = S.sysId;
    const wrap = document.getElementById('plot-y');
    wrap.replaceChildren();
    yPlots = v.outputs.map((o) => {
      const card = el('div', { class: 'card' });
      wrap.append(card);
      const p = new WB.plot.TimePlot(card, { title: `${o.label}(t)`, yLabel: `${o.label} [${o.unit}]`, unit: o.unit, minSpan: o.minSpan ?? 0 });
      collapsibleCard(card, `plot-y.${o.label}`);
      p.onHover = (i) => { viewState.hoverIndex = i; drawCursor(); };
      return p;
    });
    animTitle.textContent = s.name;
    const uT = v.inputs.length > 1 ? v.inputs.map((q) => q.label).join(', ') : v.inputs[0].label;
    uPlot.box.previousSibling.querySelector('.plot-title').textContent = `${uT}(t)`;
    uPlot.opts = { title: `${uT}(t)`, yLabel: `${uT} [${v.inputs[0].unit}]`, unit: v.inputs[0].unit };
  }

  const SERIES = ['--series-1', '--series-3', '--series-2'];

  // The disturbances that switch on at tDist, as a vertical mark for the time plots.
  function distMark(v, t) {
    const sim = S.sim;
    const on = v.dists.map((d, i) => ({ d, val: i === 0 ? sim.dist : sim.dists[i - 1] || 0 })).filter((q) => q.val !== 0);
    if (!on.length || sim.tDist < t[0] || sim.tDist > t[t.length - 1]) return null;
    const name = (d) => d.label.replace(/<sub>(.*?)<\/sub>/g, '_$1').replace(/<[^>]*>/g, '');
    const label = on.map((q) => `${name(q.d)} = ${M.fmt(q.val, 3)} ${q.d.unit}`).join(', ');
    return { t: sim.tDist, label: `${label} on`, color: '--series-3', dash: [6, 3], at: 'bottom' };
  }

  function drawPlots() {
    const s = sys(), v = view(s);
    const t = result.t;
    const ch = chapter();
    const dMark = distMark(v, t);
    v.outputs.forEach((o, oi) => {
      const k = o.scale || 1;
      const sc = (arr) => Array.from(arr || [], (val) => val * k);
      const series = [];
      if (!ch.openLoop) {
        v.refs.forEach((rf, ri) => { if (rf.output === oi && result.rAll[ri]) series.push({ label: v.refs.length > 1 ? rf.label : 'reference r', y: sc(result.rAll[ri]), color: '--ref', dash: [6, 4], width: 1.5 }); });
      }
      const nz = oi === 0 ? S.sim.noise : S.sim.noises[oi - 1];
      if (nz > 0) series.push({ label: `${o.label} measured (noisy)`, y: sc(result.yMeasAll[oi]), color: '--text-muted', width: 1 });
      const lin = linResult && (linResult.yAll ? linResult.yAll[oi] : (oi === 0 ? linResult.y : null));
      if (lin) series.push({ label: ch.linearLabel || 'linear design model', y: sc(lin), color: '--series-2', dash: [5, 4], width: 1.5, fit: false });
      if (ch.outputSeries) series.push(...(ch.outputSeries(ctx, result, sc, oi) || []));
      series.push({ label: `${o.label} (simulated)`, y: sc(result.yAll[oi]), color: '--series-1' });
      const data = { t, series };
      if (oi === metrics.output && !ch.openLoop) {
        const yv = result.yAll[oi];
        const yf = result.rAll[0][Math.min(metrics.i0, t.length - 1)] * k;
        const tol = Math.abs(yf - yv[Math.min(metrics.i0, t.length - 1)] * k) * 0.02;
        data.bands = tol > 0 ? [{ y0: yf - tol, y1: yf + tol, color: '--text-muted', alpha: 0.12 }] : [];
        data.points = metrics.os > 0.5 ? [{ t: t[metrics.iPeak], y: yv[metrics.iPeak] * k, color: '--series-1', label: `${M.fmt(metrics.os, 3)}% OS` }] : [];
        data.vmarks = isNaN(metrics.t90) ? [] : [{ t: metrics.t90, label: `90% · tr ${M.fmt(metrics.tr, 3)} s` }];
      }
      if (dMark) data.vmarks = [...(data.vmarks || []), dMark];
      yPlots[oi].setData(data);
    });

    const rg = ranges(s, ctx.pModel, result.uDemandAll.length);
    const lims = [...new Set(rg.flat())];
    const span = Math.max(...rg.map(([lo, hi]) => hi - lo));
    const clip = (arr, [lo, hi]) => Array.from(arr, (val) => Math.max(lo - 2 * span, Math.min(hi + 2 * span, val)));
    const series = [];
    result.uDemandAll.forEach((ud, i) => {
      const lab = v.inputs.length > 1 ? v.inputs[i].label : '';
      series.push({ label: `${lab} demanded${v.inputs.length > 1 ? '' : ' (before saturation)'}`.trim(), y: clip(ud, rg[i]), color: v.inputs.length > 1 ? SERIES[i % 3] : '--series-2', dash: [5, 4], width: 1.5 });
      series.push({ label: `${lab} applied`.trim(), y: Array.from(result.uAppliedAll[i]), color: SERIES[i % 3] });
    });
    // One input and one disturbance on it: draw d(t) in the input's units next to u.
    if (dMark && v.inputs.length === 1 && v.dists.length === 1 && v.dists[0].unit === v.inputs[0].unit) {
      series.push({ label: 'disturbance d', y: Array.from(t, (tk) => (tk < S.sim.tDist ? 0 : S.sim.dist)), color: '--series-3', dash: [2, 3], width: 1.5 });
    }
    const hl = lims.map((y, j) => ({ y, color: '--critical', label: j === 0 ? 'limit' : '', fit: true }));
    const hiMax = Math.max(...rg.map((r) => r[1])), loMin = Math.min(...rg.map((r) => r[0]));
    uPlot.setData({
      t, series, hlines: hl, vmarks: dMark ? [dMark] : [],
      bands: [{ y0: hiMax, y1: 1e9, color: '--critical', alpha: 0.07 }, { y0: -1e9, y1: loMin, color: '--critical', alpha: 0.07 }],
    });
    const extra = ch.extraPlot ? ch.extraPlot(ctx, result) : null;
    document.getElementById('plot-x').hidden = !extra;
    if (extra) {
      xPlot.opts = { ...xPlot.opts, ...extra.opts };
      xPlot.box.previousSibling.querySelector('.plot-title').textContent = extra.opts.title;
      xPlot.setData({ t, ...extra.data });
    }
    scrub.max = t.length - 1;
  }

  function drawAnalysis() {
    const ch = chapter();
    const spData = ch.splane ? ch.splane(ctx) : null;
    document.getElementById('splane').hidden = !spData;
    if (spData) splane.setData(spData);
    const bd = ch.bode ? ch.bode(ctx) : null;
    document.getElementById('bode').hidden = !bd;
    if (bd) {
      bodePlot.box.previousSibling.querySelector('.plot-title').textContent = bd.title || 'Bode';
      bodePlot.setData(bd);
    }
    document.querySelector('.row-2').classList.toggle('single', !spData);
  }

  function cursorIndex() {
    return viewState.hoverIndex !== null ? viewState.hoverIndex : viewState.playIndex;
  }

  function drawArm() {
    if (!result) return;
    const s = sys(), v = view(s);
    const i = Math.max(0, Math.min(cursorIndex(), result.t.length - 1));
    const { ctx: g, w, h } = WB.plot.setupCanvas(armCanvas);
    const rg = ranges(s, ctx.pModel, result.uDemandAll.length);
    const satAll = result.uDemandAll.map((ud, j) => ud[i] > rg[j][1] + 1e-9 || ud[i] < rg[j][0] - 1e-9);
    s.draw(g, w, h, {
      x: result.x[i], r: result.r[i], u: result.uApplied[i], uLimit: Math.max(Math.abs(rg[0][0]), Math.abs(rg[0][1])), saturated: satAll[0],
      rAll: result.rAll.map((a) => a[i]), uAll: result.uAppliedAll.map((a) => a[i]), ranges: rg, satAll, p: ctx.pModel,
    });
    armReadout.textContent = [
      ...v.outputs.map((o, j) => `${o.label} ${M.fmt(result.yAll[j][i] * (o.scale || 1), 3)}${o.unit === '°' ? '°' : ' ' + o.unit}`),
      ...v.inputs.map((q, j) => `${q.label} ${M.fmt(result.uAppliedAll[j][i], 3)} ${q.unit}`),
    ].join('  ·  ');
    timeLabel.textContent = `${result.t[i].toFixed(2)} s`;
  }

  function drawCursor() {
    const i = cursorIndex();
    const cur = viewState.hoverIndex !== null || viewState.playing || viewState.playIndex > 0 ? i : null;
    yPlots.forEach((p) => p.setCursor(cur));
    uPlot.setCursor(cur);
    if (!document.getElementById('plot-x').hidden) xPlot.setCursor(cur);
    scrub.value = viewState.playIndex;
    drawArm();
  }

  function togglePlay(btn) {
    viewState.playing = !viewState.playing;
    btn.textContent = viewState.playing ? '❚❚' : '▶';
    btn.setAttribute('aria-label', viewState.playing ? 'Pause' : 'Play');
    if (viewState.playing) {
      if (viewState.playIndex >= result.t.length - 1) viewState.playIndex = 0;
      viewState.lastFrame = performance.now();
      requestAnimationFrame(function step() {
        if (!viewState.playing) return;
        // Use one clock for both ends of dt. rAF's own timestamp is the frame start,
        // which can precede the performance.now() taken on click (negative dt).
        const now = performance.now();
        const dt = Math.max(0, (now - viewState.lastFrame) / 1000);
        viewState.lastFrame = now;
        viewState.playFloat = (viewState.playFloat ?? viewState.playIndex) + (dt * viewState.speed) / S.sim.Ts;
        viewState.playIndex = Math.max(0, Math.min(result.t.length - 1, Math.floor(viewState.playFloat)));
        if (viewState.playIndex >= result.t.length - 1) { viewState.playing = false; btn.textContent = '▶'; viewState.playFloat = undefined; }
        drawCursor();
        if (viewState.playing) requestAnimationFrame(step);
      });
    } else {
      viewState.playFloat = undefined;
    }
  }

  // ------------------------------------------------------------ live math --
  // Card fields (see STUDY_GUIDE.md):
  //   theory    always shown: general equations from the book.
  //   symbolic/numbers + spoiler: hidden in Work mode until Reveal (the note too).
  //   answers: 'A.3/d' (or a list): the card gives away that problem part (this
  //     chapter's or an earlier one's). In Work mode the whole card stays locked
  //     until those parts are solved here (a passing Check) or it is revealed.
  const answersOf = (c) => (c.answers ? [].concat(c.answers) : []);
  const partLabel = (k) => { const [prob, part] = k.split('/'); return `${prob} (${part.replace(/\d+$/, '')})`; };
  // Math lines never scroll sideways. Each renders inline in display style, so
  // KaTeX wraps it at top-level relations, operators, ",\;" separators and between
  // the words of \text{}. A part that still doesn't fit (one wide fraction) is
  // zoomed down to the card width, refitted whenever the line's width changes
  // (resize, splitter, expanding a card).
  // Zooming re-wraps the line, so shrink until it fits (a few passes at most).
  function fitTex(t) {
    const k = t.firstElementChild;
    if (!k || !t.clientWidth) return;
    k.style.zoom = '';
    for (let i = 0; i < 6 && t.scrollWidth > t.clientWidth + 1; i++) {
      k.style.zoom = String((+k.style.zoom || 1) * t.clientWidth / t.scrollWidth * 0.995);
    }
  }
  const texFit = new ResizeObserver((entries) => {
    for (const { target: t, contentRect: r } of entries) {
      if (!r.width || +t.dataset.fitW === r.width) continue;
      t.dataset.fitW = r.width;
      fitTex(t);
    }
  });
  // KaTeX's fonts change line widths when they arrive, without resizing the card.
  if (document.fonts) document.fonts.addEventListener('loadingdone', () => document.querySelectorAll('#math .tex').forEach(fitTex));
  function texLine(src) {
    const d = el('div', { class: 'tex' });
    const breakable = src
      .replace(/,\\;/g, ',\\;\\allowbreak ')
      .replace(/\\text\{([^{}]*)\}/g, (m, words) => words.split(/(?<= )/).map((w) => `\\text{${w}}`).join('\\allowbreak '));  // prose wraps between words
    renderTex(d, `\\displaystyle ${breakable}`, false);
    texFit.observe(d);
    return d;
  }

  function drawMath() {
    const root = document.getElementById('math');
    root.replaceChildren();
    texFit.disconnect();   // the old lines are gone
    const cards = chapter().math(ctx);
    for (const c of cards) {
      const key = `${S.sysId}:${S.chapter}:${c.title}`;
      const card = el('article', { class: 'math-card' });
      const h4 = el('h4');
      const body = el('div', { class: 'math-body' });
      card.append(el('header', {}, h4, WB.ui.pageChip(c.page || '')), body);
      WB.ui.collapsible(h4, el('span', { text: c.title }), card, body, `wb.collapsed.math.${c.title}`);
      const need = answersOf(c).filter((k) => !isSolved(k));
      if (S.mode === 'work' && need.length && !revealed.has(key)) {
        card.classList.add('locked');
        const labels = [...new Set(need.map(partLabel))];
        body.append(el('p', { class: 'muted small locked-text', text: `This is part of the answer to ${labels.join(' and ')}. Solve it in the problem panel to unlock it.` }));
        body.append(el('button', { type: 'button', class: 'btn btn-quiet reveal', text: 'Reveal anyway', onclick: () => { revealed.add(key); drawMath(); } }));
        root.append(card);
        continue;
      }
      const hide = S.mode === 'work' && c.spoiler && !revealed.has(key);
      // Long equations are written with \\quad between parts; give each part its own line.
      const lines = (src) => src.split(/,?\\quad/).map((x) => x.trim()).filter(Boolean);
      if (c.theory) for (const line of lines(c.theory)) body.append(texLine(line));
      const answerLines = [c.symbolic, c.numbers].filter(Boolean);
      if (answerLines.length) {
        if (hide) {
          body.append(el('button', {
            type: 'button', class: 'btn btn-quiet reveal', text: 'Reveal the numbers',
            onclick: () => { revealed.add(key); drawMath(); },
          }));
        } else {
          const nums = el('div', { class: 'tex-numbers' });
          for (const src of answerLines) for (const line of lines(src)) nums.append(texLine(line));
          body.append(nums);
        }
      }
      if (c.note && !(hide && answerLines.length)) body.append(el('p', { class: 'muted small', text: c.note }));
      root.append(card);
    }
  }

  // Problem parts solved in this browser: {'A:A.3/d': true}. Keys passed around
  // are 'A.3/d'; the study prefix keeps studies apart.
  const SOLVED_KEY = 'wb.solved';
  const solved = store.get(SOLVED_KEY, {});
  const isSolved = (k) => !!solved[`${S.sysId}:${k}`];
  // The parts each chapter's problem panels can mark solved, recorded when a panel
  // is built: {A: {ch3: {'A.3': ['A.3/a', ...]}}}. A tab turns green once every
  // recorded part of its chapter is solved (so a chapter counts once visited).
  const PARTS_KEY = 'wb.parts';
  const parts = store.get(PARTS_KEY, {});
  function chapterDone(sysId, c) {
    const probs = Object.values((parts[sysId] || {})[c] || {});
    const keys = probs.flat();
    return keys.length > 0 && keys.every((k) => !!solved[`${sysId}:${k}`]);
  }
  function refreshTabs() {
    for (const b of document.querySelectorAll('#tabs .tab[data-ch]')) {
      const done = chapterDone(S.sysId, b.dataset.ch);
      b.classList.toggle('done', done);
      b.title = b.title.replace(/ · all parts solved$/, '') + (done ? ' · all parts solved' : '');
    }
  }

  // ------------------------------------------------------------ top level --
  function update() {
    run();
    WB.ui.refreshAll();
    drawPlots();
    drawAnalysis();
    drawMath();
    drawCursor();
    refreshTabs();
    store.set(STATE_KEY, S);
    try { history.replaceState(null, '', `#${S.sysId}/${S.chapter}/${S.mode}`); } catch (e) { /* file:// in some browsers */ }
  }

  function rebuild() {
    WB.ui.clearRefreshers();
    run();
    buildOutputPlots();
    buildTabs();
    buildLeft();
    buildRight();
    buildHeaderState();
    update();
  }

  function buildTabs() {
    const tabs = document.getElementById('tabs');
    tabs.replaceChildren();
    const ids = chapterIds(S.sysId);
    for (const part of PARTS) {
      const group = el('div', { class: 'tab-group', role: 'group', 'aria-label': part.label }, el('span', { class: 'tab-part', text: part.label }));
      for (const c of ids.filter((k) => chaptersOf(S.sysId)[k].num >= part.nums[0] && chaptersOf(S.sysId)[k].num <= part.nums[1])) {
        const ch = chaptersOf(S.sysId)[c];
        const b = el('button', { type: 'button', role: 'tab', class: 'tab', 'data-ch': c, title: `${ch.tab} · ${ch.title}` }, el('span', { class: 'tab-num', text: ch.short || ch.tab.replace('Ch ', '') }));
        b.addEventListener('click', () => {
          if (S.chapter === c) return;
          S.chapter = c;
          applyChapterDefaults(c);
          rebuild();
        });
        group.append(b);
      }
      if (group.children.length > 1) tabs.append(group);
    }
  }

  let modeCtl = null;
  function buildHeader() {
    const study = document.getElementById('study');
    study.replaceChildren(...['A', 'B', 'C', 'D', 'E', 'F'].map((id) => {
      const ok = STUDY_IDS.includes(id);
      const o = el('option', { value: id, text: ok ? `${id} · ${WB.systems[id].name}` : `${id} · not yet` });
      if (!ok) o.disabled = true;
      return o;
    }));
    study.value = S.sysId;
    study.addEventListener('change', () => { S = freshState(study.value, S.chapter); rebuild(); });

    // Built once, so it is refreshed in buildHeaderState (rebuild() clears refreshers).
    modeCtl = segmented(document.getElementById('mode'), {
      options: [
        { value: 'work', label: 'Work it', title: 'Set the gains yourself; derivations stay hidden until revealed' },
        { value: 'explore', label: 'Explore', title: 'Design from poles or specs; all math shown' },
      ],
      get: () => S.mode, set: (val) => app.setMode(val),
    });

    const themeBtn = document.getElementById('theme');
    const applyTheme = (t) => {
      if (t === 'auto') document.documentElement.removeAttribute('data-theme');
      else document.documentElement.setAttribute('data-theme', t);
      themeBtn.textContent = `Theme: ${t}`;
    };
    let theme = store.get('wb.theme', 'auto');
    applyTheme(theme);
    themeBtn.addEventListener('click', () => {
      theme = { auto: 'light', light: 'dark', dark: 'auto' }[theme];
      store.set('wb.theme', theme);
      applyTheme(theme);
      update();
    });
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => update());
  }

  function buildHeaderState() {
    for (const b of document.querySelectorAll('#tabs .tab[data-ch]')) {
      const on = b.dataset.ch === S.chapter;
      b.classList.toggle('on', on);
      b.setAttribute('aria-selected', String(on));
    }
    const ch = chapter();
    document.getElementById('study').value = S.sysId;
    document.getElementById('chapter-title').textContent = `${ch.tab} · ${ch.title}`;
    document.getElementById('chapter-pages').replaceChildren(document.createTextNode('controlbook.pdf '), ...WB.ui.linkPages(ch.pages || ''));
    document.body.dataset.mode = S.mode;
    if (modeCtl) modeCtl.refresh();
  }

  const app = {
    result: () => result,
    isRevealed: (key) => revealed.has(key),
    reveal: (key) => revealed.add(key),
    isSolved,
    // Called by each problem panel with the keys of the parts it can mark solved.
    registerParts(probId, keys) {
      const sys = (parts[S.sysId] = parts[S.sysId] || {});
      const ch = (sys[S.chapter] = sys[S.chapter] || {});
      if (JSON.stringify(ch[probId]) === JSON.stringify(keys)) return;
      ch[probId] = keys;
      store.set(PARTS_KEY, parts);
      refreshTabs();
    },
    markSolved(k) {
      if (isSolved(k)) return;
      solved[`${S.sysId}:${k}`] = true;
      store.set(SOLVED_KEY, solved);
      update();  // plots and s-plane markers may be gated on this part too
    },
    setMode(m) {
      if (S.mode === m) return;
      // Work mode always keeps the user's own gains: explored (designed) gains
      // are never copied in, so a Work-mode check can't pass by switching modes.
      S.mode = m;
      rebuild();
    },
  };
  WB.app = app;

  // Optional deep link: #A/ch8/explore
  function applyHash() {
    const [sysId, ch, mode] = location.hash.replace('#', '').split('/');
    if (sysId && STUDY_IDS.includes(sysId) && sysId !== S.sysId) S = freshState(sysId, ch || S.chapter);
    if (ch && chapterIds(S.sysId).includes(ch) && ch !== S.chapter) { S.chapter = ch; applyChapterDefaults(ch); }
    if (mode === 'work' || mode === 'explore') S.mode = mode;
  }

  window.addEventListener('DOMContentLoaded', () => {
    applyHash();
    buildHeader();
    buildCenter();
    const mathH = document.querySelector('.math-head h2');
    mathH.replaceChildren();
    WB.ui.collapsible(mathH, 'Math', document.querySelector('.math-wrap'), document.getElementById('math'), 'wb.collapsed.section.Math');
    rebuild();
    // Collapsible settings column (remembered per browser).
    const layout = document.querySelector('.layout');
    const toggle = document.getElementById('left-toggle');
    const setCollapsed = (on) => {
      layout.classList.toggle('left-collapsed', on);
      toggle.setAttribute('aria-expanded', String(!on));
      toggle.title = on ? 'Show settings' : 'Hide settings';
      toggle.querySelector('.collapse-icon').textContent = on ? '›' : '‹';
      store.set('wb.leftCollapsed', on);
    };
    setCollapsed(store.get('wb.leftCollapsed', false));
    toggle.addEventListener('click', () => setCollapsed(!layout.classList.contains('left-collapsed')));

    // Draggable split between the problem column (.right-col, shown second) and
    // the plot column (.center, shown last). frac is the problem column's share
    // of the two; remembered per browser.
    const splitter = document.getElementById('splitter');
    const setSplit = (frac) => {
      frac = Math.min(0.75, Math.max(0.2, frac));
      layout.style.setProperty('--problem-fr', `${frac}fr`);
      layout.style.setProperty('--plots-fr', `${1 - frac}fr`);
      splitter.setAttribute('aria-valuenow', String(Math.round(frac * 100)));
      store.set('wb.problemFrac', frac);
      return frac;
    };
    let split = setSplit(store.get('wb.problemFrac', 0.5));
    splitter.setAttribute('aria-valuemin', '20');
    splitter.setAttribute('aria-valuemax', '75');
    splitter.addEventListener('pointerdown', (e) => {
      const problem = document.querySelector('.right-col').getBoundingClientRect();
      const plots = document.querySelector('.center').getBoundingClientRect();
      splitter.setPointerCapture(e.pointerId);
      splitter.classList.add('dragging');
      document.body.classList.add('col-resizing');
      const move = (ev) => { split = setSplit((ev.clientX - problem.left) / (plots.right - problem.left)); };
      const up = () => {
        splitter.removeEventListener('pointermove', move);
        splitter.classList.remove('dragging');
        document.body.classList.remove('col-resizing');
      };
      splitter.addEventListener('pointermove', move);
      splitter.addEventListener('pointerup', up, { once: true });
      splitter.addEventListener('pointercancel', up, { once: true });
    });
    splitter.addEventListener('dblclick', () => { split = setSplit(0.5); });
    splitter.addEventListener('keydown', (e) => {
      if (e.key === 'ArrowLeft') { split = setSplit(split - 0.02); e.preventDefault(); }
      if (e.key === 'ArrowRight') { split = setSplit(split + 0.02); e.preventDefault(); }
    });

    document.getElementById('reset-all').addEventListener('click', () => {
      S = freshState(S.sysId, S.chapter);
      rebuild();
    });
  });
})();
