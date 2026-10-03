// Page glue: state, panels, simulation runs, plots, playback, and the live-math strip.
window.WB = window.WB || {};

(function () {
  const { el, slider, segmented, section, renderTex, store } = WB.ui;
  const M = WB.math;
  // Chapters register themselves as WB.chapters.chN with a numeric `num`.
  const CHAPTERS = Object.keys(WB.chapters).filter((k) => WB.chapters[k].num !== undefined).sort((a, b) => WB.chapters[a].num - WB.chapters[b].num);
  const PARTS = [
    { label: 'Models', nums: [2, 6] }, { label: 'PID', nums: [7, 10.9] },
    { label: 'Observers', nums: [11, 14] }, { label: 'Loopshaping', nums: [15, 18] },
  ];
  // Every chapter starts from these simulation settings, then applies its own.
  const SIM_BASE = { type: 'step', amplitude: 30, frequency: 0.2, tStep: 0.5, y0: 0, dist: 0, tDist: 3, tEnd: 4, Ts: 0.01, noise: 0 };
  function chapterSim(c, sys) {
    const { mismatch, ...sim } = WB.chapters[c].simDefaults(sys);
    return { ...SIM_BASE, ...sim };
  }
  // Apply a chapter's default simulation settings and plant mismatch to S.
  function applyChapterDefaults(c) {
    const s = WB.systems[S.sysId];
    Object.assign(S.sim, chapterSim(c, s));
    const mm = WB.chapters[c].simDefaults(s).mismatch || {};
    S.mismatch = Object.fromEntries(s.uncertain.map((k) => [k, mm[k] || 0]));
  }
  const STATE_KEY = 'wb.state.v1';

  // ------------------------------------------------------------------ state --
  function freshState(sysId = 'A', chapter = 'ch7') {
    const sys = WB.systems[sysId];
    const S = {
      sysId, chapter, mode: 'work',
      p: Object.fromEntries(sys.params.map((q) => [q.key, q.value])),
      mismatch: Object.fromEntries(sys.uncertain.map((k) => [k, 0])),
      alpha: 0.2,
      sim: { ...SIM_BASE, seed: 1 },
      ch: {},
    };
    for (const c of CHAPTERS) S.ch[c] = WB.chapters[c].defaults(sys);
    Object.assign(S.sim, chapterSim(chapter, sys));
    Object.assign(S.mismatch, WB.chapters[chapter].simDefaults(sys).mismatch || {});
    return S;
  }

  let S = (() => {
    const saved = store.get(STATE_KEY, null);
    const base = freshState();
    if (!saved || !WB.systems[saved.sysId]) return base;
    // shallow-merge so new fields added later still get defaults
    const merged = { ...base, ...saved, p: { ...base.p, ...saved.p }, mismatch: { ...base.mismatch, ...saved.mismatch }, sim: { ...base.sim, ...saved.sim }, ch: { ...base.ch } };
    for (const c of CHAPTERS) merged.ch[c] = { ...base.ch[c], ...(saved.ch || {})[c] };
    if (!CHAPTERS.includes(merged.chapter)) merged.chapter = 'ch7';
    for (const c of CHAPTERS) if (!merged.ch[c]) merged.ch[c] = base.ch[c];
    return merged;
  })();

  const revealed = new Set();
  let ctx = null, result = null, linResult = null, metrics = null;
  const view = { playIndex: 0, hoverIndex: null, playing: false, speed: 1, lastFrame: 0 };

  const chapter = () => WB.chapters[S.chapter];
  const sys = () => WB.systems[S.sysId];

  // One long-lived ctx object, refreshed in place, so closures created when the
  // panels were built (problem checks, drag handlers) always see current values.
  function refreshCtx() {
    const s = sys();
    const pModel = { ...S.p, ...s.constants };
    const pTrue = { ...pModel };
    for (const k of s.uncertain) pTrue[k] = pModel[k] * (1 + S.mismatch[k] / 100);
    ctx = ctx || {};
    Object.assign(ctx, { sys: s, S, st: S.ch[S.chapter], pModel, pTrue, model: s.secondOrderModel(pModel), ss: s.stateSpace(pModel), update, app });
    ctx.gains = chapter().gains ? chapter().gains(ctx) : {};
  }

  // -------------------------------------------------------------- simulate --
  function run() {
    refreshCtx();
    const s = ctx.sys;
    const scale = M.DEG;
    const reference = WB.sim.makeReference({ type: S.sim.type, amplitude: S.sim.amplitude * scale, offset: 0, frequency: S.sim.frequency, tStep: S.sim.tStep });
    const refWithStart = (t) => (t < S.sim.tStep ? S.sim.y0 * scale : reference(t));
    const disturbance = (t) => (t >= S.sim.tDist ? S.sim.dist : 0);
    const noise = WB.sim.makeNoise(S.sim.noise * scale, S.sim.seed);
    const common = { reference: chapter().reference ? chapter().reference(ctx, refWithStart) : refWithStart, disturbance, noise, x0: s.x0(S.sim.y0 * scale), Ts: S.sim.Ts, tEnd: S.sim.tEnd };

    const plantTrue = { f: (x, u) => s.f(x, u, ctx.pTrue), h: s.h, uLimit: s.uLimit(ctx.pTrue) };
    result = chapter().simulate
      ? chapter().simulate(ctx, common, plantTrue)
      : WB.sim.simulate({ ...common, plant: plantTrue, controller: chapter().controller(ctx) });

    // Linear design model: the same controller (without its feedforward term) on
    // xdot = A x + B u with nominal parameters, no saturation, no disturbance and
    // no noise. Gaps between this trace and the real response come from
    // saturation, nonlinearity, the compensation choice, mismatch, d, or noise.
    if (chapter().linear === false) {
      linResult = null;
    } else if (chapter().linearSim) {
      linResult = chapter().linearSim(ctx, common);
    } else {
      const { A, B } = ctx.ss;
      linResult = WB.sim.simulate({
        ...common,
        disturbance: null, noise: null,
        plant: { f: (x, u) => A.map((row, i) => row.reduce((acc, a, j) => acc + a * x[j], 0) + B[i][0] * u), h: (x) => x[0], uLimit: Infinity },
        controller: chapter().controller(ctx, { linear: true }),
      });
    }

    // Metrics on the first step: from tStep to the next reference change or disturbance.
    const N = result.t.length;
    const idx = (t) => Math.max(0, Math.min(N, Math.round(t / S.sim.Ts)));
    const i0 = idx(S.sim.tStep);
    let i1 = N;
    if (S.sim.type === 'square') i1 = Math.min(i1, idx(S.sim.tStep + 0.5 / S.sim.frequency));
    if (S.sim.dist !== 0 && S.sim.tDist > S.sim.tStep) i1 = Math.min(i1, idx(S.sim.tDist));
    metrics = M.stepMetrics(result.t, result.y, i0, i1, result.y[i0], result.r[Math.min(i0, N - 1)]);
    metrics.i0 = i0; metrics.i1 = i1;
    const lim = s.uLimit(ctx.pModel);
    let peak = 0, satCount = 0;
    for (let k = 0; k < N; k++) {
      peak = Math.max(peak, Math.abs(result.uDemand[k]));
      if (Math.abs(result.uDemand[k]) > lim + 1e-9) satCount++;
    }
    metrics.peakU = peak; metrics.satFrac = satCount / N;
    if (view.playIndex >= N) view.playIndex = N - 1;
  }

  // ----------------------------------------------------------------- panels --
  function buildLeft() {
    const root = document.getElementById('left');
    root.replaceChildren();
    const s = sys();

    const plant = section(root, 'Plant (nominal model)', `p. ${s.introPage}`);
    for (const q of s.params) {
      slider(plant, { label: q.label, unit: q.unit, min: q.min, max: q.max, step: q.step, get: () => S.p[q.key], set: (v) => { S.p[q.key] = v; update(); } });
    }
    plant.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Book values', onclick: () => { for (const q of s.params) S.p[q.key] = q.value; update(); } }));

    const mis = section(root, 'True plant vs. model');
    mis.append(el('p', { class: 'muted small', text: 'The simulated plant differs from the model the controller was designed with, like armDynamics(alpha).' }));
    for (const k of s.uncertain) {
      const q = s.params.find((pp) => pp.key === k);
      slider(mis, { label: 'Δ' + q.label, unit: '%', min: -50, max: 50, step: 1, sig: 3, get: () => S.mismatch[k], set: (v) => { S.mismatch[k] = v; update(); } });
    }
    slider(mis, { label: 'α', min: 0, max: 0.5, step: 0.01, sig: 3, get: () => S.alpha, set: (v) => { S.alpha = v; WB.ui.refreshAll(); } });
    mis.append(el('div', { class: 'btn-row' },
      el('button', { type: 'button', class: 'btn', text: 'Randomize ±α', onclick: () => { for (const k of s.uncertain) S.mismatch[k] = Math.round(100 * S.alpha * (2 * Math.random() - 1)); update(); } }),
      el('button', { type: 'button', class: 'btn btn-quiet', text: 'Exact model', onclick: () => { for (const k of s.uncertain) S.mismatch[k] = 0; update(); } })));

    const ref = section(root, 'Reference');
    segmented(ref, { label: 'Signal', options: [{ value: 'step', label: 'step' }, { value: 'square', label: 'square' }, { value: 'sine', label: 'sine' }], get: () => S.sim.type, set: (v) => { S.sim.type = v; update(); } });
    slider(ref, { label: 'amplitude', unit: '°', min: -90, max: 90, step: 1, sig: 3, get: () => S.sim.amplitude, set: (v) => { S.sim.amplitude = v; update(); } });
    slider(ref, { label: 'starts at', unit: 's', min: 0, max: 5, step: 0.05, sig: 3, get: () => S.sim.tStep, set: (v) => { S.sim.tStep = v; update(); } });
    slider(ref, { label: 'frequency', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, get: () => S.sim.frequency, set: (v) => { S.sim.frequency = v; update(); }, disabled: () => S.sim.type === 'step' });
    slider(ref, { label: `initial ${s.output.label}`, unit: '°', min: -90, max: 90, step: 1, sig: 3, get: () => S.sim.y0, set: (v) => { S.sim.y0 = v; update(); } });

    const dist = section(root, 'Disturbance & noise');
    slider(dist, { label: 'd', unit: s.input.unit, min: -1, max: 1, step: 0.01, sig: 3, get: () => S.sim.dist, set: (v) => { S.sim.dist = v; update(); } });
    slider(dist, { label: 'starts at', unit: 's', min: 0, max: 20, step: 0.1, sig: 3, get: () => S.sim.tDist, set: (v) => { S.sim.tDist = v; update(); } });
    slider(dist, { label: 'noise σ', unit: '°', min: 0, max: 2, step: 0.01, sig: 3, hint: `Gaussian noise on the measured ${s.output.label}. Controllers that use the true state ignore it.`, get: () => S.sim.noise, set: (v) => { S.sim.noise = v; update(); } });
    dist.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'New noise sample', onclick: () => { S.sim.seed = (S.sim.seed % 100000) + 1; update(); } }));

    const simS = section(root, 'Simulation');
    slider(simS, { label: 't<sub>end</sub>', unit: 's', min: 1, max: 60, step: 0.5, sig: 3, get: () => S.sim.tEnd, set: (v) => { S.sim.tEnd = v; update(); } });
    slider(simS, { label: 'T<sub>s</sub>', unit: 's', min: 0.001, max: 0.05, step: 0.001, sig: 3, get: () => S.sim.Ts, set: (v) => { S.sim.Ts = v; update(); } });
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
    chapter().buildProblem(prob, ctx);
  }

  function renderMetrics(table) {
    if (!metrics) return;
    const s = sys();
    const lim = s.uLimit(ctx.pModel);
    const rows = [];
    const row = (label, value, status) => {
      const r = el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: label }), el('strong', { text: value }));
      if (status) r.append(el('span', { class: 'status ' + (status.ok ? 'good' : 'bad') }, el('span', { class: 'status-icon', 'aria-hidden': 'true', text: status.ok ? '✓' : '✗' }), el('span', { text: status.text })));
      rows.push(r);
    };
    const tg = chapter().targets ? chapter().targets(ctx) : {};
    const trTarget = tg.tr || null;
    row('rise time (10–90%)', isNaN(metrics.tr) ? '—' : `${M.fmt(metrics.tr, 3)} s`,
      trTarget && !isNaN(metrics.tr) ? { ok: metrics.tr <= trTarget * 1.1, text: `target ${M.fmt(trTarget, 3)} s` } : null);
    const zeta = tg.zeta || null;
    const osExp = zeta && zeta < 1 ? 100 * Math.exp(-zeta * Math.PI / Math.sqrt(1 - zeta * zeta)) : zeta ? 0 : null;
    row('overshoot', isNaN(metrics.os) ? '—' : `${M.fmt(metrics.os, 3)} %`,
      osExp !== null && !isNaN(metrics.os) ? { ok: metrics.os <= osExp + 2, text: `2nd-order ζ predicts ${M.fmt(osExp, 2)} %` } : null);
    row('settling (2%)', isNaN(metrics.ts) ? 'not settled' : `${M.fmt(metrics.ts, 3)} s`);
    row('error at window end', isNaN(metrics.ess) ? '—' : `${M.fmt(metrics.ess / M.DEG, 3)} °`);
    row(`peak |${s.input.label}| demanded`, `${M.fmt(metrics.peakU, 3)} ${s.input.unit}`,
      { ok: metrics.peakU <= lim + 1e-9, text: metrics.peakU <= lim + 1e-9 ? 'within limit' : `saturated ${M.fmt(100 * metrics.satFrac, 2)}% of the time` });
    table.replaceChildren(...rows);
  }

  // ----------------------------------------------------------------- views --
  let thetaPlot, uPlot, xPlot, splane, bodePlot, armCanvas, scrub, timeLabel, armReadout;

  function buildCenter() {
    const s = sys();
    armCanvas = el('canvas', { class: 'plot-canvas', role: 'img', 'aria-label': `${s.name} animation` });
    armReadout = el('div', { class: 'arm-readout' });
    const armBox = el('div', { class: 'plot-box anim-box' }, armCanvas, armReadout);
    new ResizeObserver(drawArm).observe(armBox);
    const playBtn = el('button', { type: 'button', class: 'btn icon-btn', 'aria-label': 'Play', text: '▶' });
    playBtn.addEventListener('click', () => togglePlay(playBtn));
    scrub = el('input', { type: 'range', min: 0, max: 1, step: 1, value: 0, 'aria-label': 'time' });
    scrub.addEventListener('input', () => { view.playIndex = +scrub.value; view.playFloat = undefined; drawCursor(); });
    timeLabel = el('span', { class: 'time-label' });
    const speed = el('select', { 'aria-label': 'playback speed' },
      ...[0.25, 0.5, 1, 2].map((v) => el('option', { value: v, text: `${v}×`, ...(v === 1 ? { selected: '' } : {}) })));
    speed.addEventListener('change', () => { view.speed = +speed.value; });
    document.getElementById('anim').replaceChildren(
      el('div', { class: 'plot-head' }, el('span', { class: 'plot-title', text: s.name })),
      armBox, el('div', { class: 'player' }, playBtn, scrub, timeLabel, speed));

    const sp = document.getElementById('splane'); sp.replaceChildren();
    splane = new WB.plot.SPlane(sp, { title: 's-plane' });
    splane.onDrag = (id, re, im) => chapter().onPoleDrag && chapter().onPoleDrag(ctx, id, re, im);

    const py = document.getElementById('plot-y'); py.replaceChildren();
    thetaPlot = new WB.plot.TimePlot(py, { title: `${s.output.label}(t)`, yLabel: `${s.output.label} [${s.output.unit}]`, unit: s.output.unit, minSpan: 2 });
    const pu = document.getElementById('plot-u'); pu.replaceChildren();
    uPlot = new WB.plot.TimePlot(pu, { title: `${s.input.label}(t)`, yLabel: `${s.input.label} [${s.input.unit}]`, unit: s.input.unit });
    const px = document.getElementById('plot-x'); px.replaceChildren();
    xPlot = new WB.plot.TimePlot(px, { title: '' });
    const bd = document.getElementById('bode'); bd.replaceChildren();
    bodePlot = new WB.plot.BodePlot(bd, { title: 'Bode' });
    for (const p of [thetaPlot, uPlot, xPlot]) {
      p.onHover = (i) => { view.hoverIndex = i; drawCursor(); };
    }
  }

  function drawPlots() {
    const s = sys();
    const k = s.output.scale;
    const t = result.t;
    const deg = (arr) => Array.from(arr, (v) => v * k);
    const ch = chapter();
    const yf = result.r[Math.min(metrics.i0, t.length - 1)] * k;
    const tol = Math.abs(yf - result.y[metrics.i0] * k) * 0.02;
    const points = [];
    if (metrics.os > 0.5) points.push({ t: t[metrics.iPeak], y: result.y[metrics.iPeak] * k, color: '--series-1', label: `${M.fmt(metrics.os, 3)}% OS` });
    const vmarks = isNaN(metrics.t90) ? [] : [{ t: metrics.t90, label: `90% · tr ${M.fmt(metrics.tr, 3)} s` }];
    const ySeries = ch.openLoop ? [] : [{ label: 'reference r', y: deg(result.r), color: '--ref', dash: [6, 4], width: 1.5 }];
    if (S.sim.noise > 0) ySeries.push({ label: `${s.output.label} measured (noisy)`, y: deg(result.yMeas), color: '--text-muted', width: 1 });
    if (linResult) ySeries.push({ label: ch.linearLabel || 'linear design model', y: deg(linResult.y), color: '--series-2', dash: [5, 4], width: 1.5, fit: false });
    if (ch.outputSeries) ySeries.push(...ch.outputSeries(ctx, result, deg));
    ySeries.push({ label: `${s.output.label} (simulated)`, y: deg(result.y), color: '--series-1' });
    thetaPlot.setData({
      t,
      series: ySeries,
      bands: tol > 0 && !ch.openLoop ? [{ y0: yf - tol, y1: yf + tol, color: '--text-muted', alpha: 0.12 }] : [],
      points: ch.openLoop ? [] : points, vmarks: ch.openLoop ? [] : vmarks,
    });
    const lim = s.uLimit(ctx.pModel);
    const clip = (arr) => Array.from(arr, (v) => Math.max(-4 * lim, Math.min(4 * lim, v)));
    uPlot.setData({
      t,
      series: [
        { label: 'demanded (before saturation)', y: clip(result.uDemand), color: '--series-2', dash: [5, 4], width: 1.5 },
        { label: 'applied', y: Array.from(result.uApplied), color: '--series-1' },
      ],
      hlines: [{ y: lim, color: '--critical', label: `+${s.input.label}max` }, { y: -lim, color: '--critical', label: `−${s.input.label}max`, fit: false }],
      bands: [{ y0: lim, y1: 1e9, color: '--critical', alpha: 0.07 }, { y0: -1e9, y1: -lim, color: '--critical', alpha: 0.07 }],
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
    return view.hoverIndex !== null ? view.hoverIndex : view.playIndex;
  }

  function drawArm() {
    if (!result) return;
    const s = sys();
    const i = Math.max(0, Math.min(cursorIndex(), result.t.length - 1));
    const { ctx: g, w, h } = WB.plot.setupCanvas(armCanvas);
    const lim = s.uLimit(ctx.pModel);
    s.draw(g, w, h, { x: result.x[i], r: result.r[i], u: result.uApplied[i], uLimit: lim, saturated: Math.abs(result.uDemand[i]) > lim + 1e-9 });
    armReadout.textContent = `${s.output.label} ${M.fmt(result.y[i] * s.output.scale, 3)}${s.output.unit}  ·  ${s.input.label} ${M.fmt(result.uApplied[i], 3)} ${s.input.unit}`;
    timeLabel.textContent = `${result.t[i].toFixed(2)} s`;
  }

  function drawCursor() {
    const i = cursorIndex();
    const cur = view.hoverIndex !== null || view.playing || view.playIndex > 0 ? i : null;
    thetaPlot.setCursor(cur);
    uPlot.setCursor(cur);
    if (!document.getElementById('plot-x').hidden) xPlot.setCursor(cur);
    scrub.value = view.playIndex;
    drawArm();
  }

  function togglePlay(btn) {
    view.playing = !view.playing;
    btn.textContent = view.playing ? '❚❚' : '▶';
    btn.setAttribute('aria-label', view.playing ? 'Pause' : 'Play');
    if (view.playing) {
      if (view.playIndex >= result.t.length - 1) view.playIndex = 0;
      view.lastFrame = performance.now();
      requestAnimationFrame(function step() {
        if (!view.playing) return;
        // Use one clock for both ends of dt. rAF's own timestamp is the frame start,
        // which can precede the performance.now() taken on click (negative dt).
        const now = performance.now();
        const dt = Math.max(0, (now - view.lastFrame) / 1000);
        view.lastFrame = now;
        view.playFloat = (view.playFloat ?? view.playIndex) + (dt * view.speed) / S.sim.Ts;
        view.playIndex = Math.max(0, Math.min(result.t.length - 1, Math.floor(view.playFloat)));
        if (view.playIndex >= result.t.length - 1) { view.playing = false; btn.textContent = '▶'; view.playFloat = undefined; }
        drawCursor();
        if (view.playing) requestAnimationFrame(step);
      });
    } else {
      view.playFloat = undefined;
    }
  }

  // ------------------------------------------------------------ live math --
  function drawMath() {
    const root = document.getElementById('math');
    root.replaceChildren();
    const cards = chapter().math(ctx);
    for (const c of cards) {
      const key = `${S.chapter}:${c.title}`;
      const hide = S.mode === 'work' && c.spoiler && !revealed.has(key);
      const card = el('article', { class: 'math-card' });
      card.append(el('header', {}, el('h4', { text: c.title }), WB.ui.pageChip(c.page)));
      // Long equations are written with \\quad between parts; give each part its own line.
      const lines = (src) => src.split(/,?\\quad/).map((x) => x.trim()).filter(Boolean);
      if (c.theory) for (const line of lines(c.theory)) { const d = el('div', { class: 'tex' }); renderTex(d, line); card.append(d); }
      const answerLines = [c.symbolic, c.numbers].filter(Boolean);
      if (answerLines.length) {
        if (hide) {
          card.append(el('button', {
            type: 'button', class: 'btn btn-quiet reveal', text: 'Reveal the numbers',
            onclick: () => { revealed.add(key); drawMath(); },
          }));
        } else {
          const nums = el('div', { class: 'tex-numbers' });
          for (const src of answerLines) for (const line of lines(src)) { const d = el('div', { class: 'tex' }); renderTex(d, line); nums.append(d); }
          card.append(nums);
        }
      }
      if (c.note) card.append(el('p', { class: 'muted small', text: c.note }));
      root.append(card);
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
    store.set(STATE_KEY, S);
    try { history.replaceState(null, '', `#${S.sysId}/${S.chapter}/${S.mode}`); } catch (e) { /* file:// in some browsers */ }
  }

  function rebuild() {
    WB.ui.clearRefreshers();
    run();
    buildLeft();
    buildRight();
    buildHeaderState();
    update();
  }

  function buildHeader() {
    const tabs = document.getElementById('tabs');
    tabs.replaceChildren();
    for (const part of PARTS) {
      const group = el('div', { class: 'tab-group', role: 'group', 'aria-label': part.label }, el('span', { class: 'tab-part', text: part.label }));
      for (const c of CHAPTERS.filter((k) => WB.chapters[k].num >= part.nums[0] && WB.chapters[k].num <= part.nums[1])) {
        const ch = WB.chapters[c];
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

    const study = document.getElementById('study');
    study.replaceChildren(...['A', 'B', 'C', 'D', 'E', 'F'].map((id) => {
      const o = el('option', { value: id, text: WB.systems[id] ? `${id} · ${WB.systems[id].name}` : `${id} · not yet` });
      if (!WB.systems[id]) o.disabled = true;
      return o;
    }));
    study.value = S.sysId;
    study.addEventListener('change', () => { S = freshState(study.value, S.chapter); rebuild(); });

    const mode = document.getElementById('mode');
    segmented(mode, {
      options: [
        { value: 'work', label: 'Work it', title: 'Set the gains yourself; derivations stay hidden until revealed' },
        { value: 'explore', label: 'Explore', title: 'Design from poles or specs; all math shown' },
      ],
      get: () => S.mode, set: (v) => app.setMode(v),
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
    document.getElementById('chapter-title').textContent = `${ch.tab} · ${ch.title}`;
    document.getElementById('chapter-pages').replaceChildren(document.createTextNode('controlbook.pdf '), ...WB.ui.linkPages(ch.pages));
    document.body.dataset.mode = S.mode;
  }

  const app = {
    result: () => result,
    isRevealed: (key) => revealed.has(key),
    reveal: (key) => revealed.add(key),
    setMode(m) {
      if (S.mode === m) return;
      // Entering work mode from explore: start from the explored gains so the
      // response doesn't jump; entering explore leaves work gains untouched.
      if (m === 'work' && ctx) { S.ch[S.chapter].kP = ctx.gains.kP; S.ch[S.chapter].kD = ctx.gains.kD; }
      S.mode = m;
      rebuild();
    },
  };
  WB.app = app;

  // Optional deep link: #A/ch8/explore
  function applyHash() {
    const [sysId, ch, mode] = location.hash.replace('#', '').split('/');
    if (sysId && WB.systems[sysId] && sysId !== S.sysId) S = freshState(sysId, S.chapter);
    if (ch && CHAPTERS.includes(ch) && ch !== S.chapter) { S.chapter = ch; applyChapterDefaults(ch); }
    if (mode === 'work' || mode === 'explore') S.mode = mode;
  }

  window.addEventListener('DOMContentLoaded', () => {
    applyHash();
    buildHeader();
    run();
    buildCenter();
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

    document.getElementById('reset-all').addEventListener('click', () => {
      S = freshState(S.sysId, S.chapter);
      rebuild();
    });
  });
})();
