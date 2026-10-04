// Study F shared pieces: the simulation hook (disturbances, reference offsets,
// linear overlay), loop design helpers for the decoupled longitudinal / lateral
// loops, the nested PID controller (F.7–F.10), s-plane markers for successive
// loop closure, and small UI/problem helpers. Chapter files build on WB.F.
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.F = WB.studies.F || { chapters: {} };

WB.F = (function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texPole, fmt, fmtPole } = M;
  const RAD = 180 / Math.PI;
  const ZERO_EXT = { Fwind: 0, wind: 0, ah: 0 };

  // s-plane: chapters pass data.legendNames (inner/outer loop labels) and
  // data.minR (zoom floor for slow loops); both are handled in plot.js.

  // ------------------------------------------------------------- signals --
  // Disturbance values in SI, by key, active from t_dist on.
  function distValues(ctx) {
    const sim = ctx.S.sim, v = ctx.chan;
    const out = {};
    v.dists.forEach((d, i) => { out[d.key] = i === 0 ? sim.dist : (sim.dists[i - 1] || 0); });
    return out;
  }

  // Reference with offsets: F.8(e) asks for z_r = 3 ± 2.5 m, which the shared
  // reference panel (amplitude only) cannot express.
  function reference(ctx, base) {
    const S = ctx.S;
    return (t) => {
      const r = base(t);
      if (t >= S.sim.tStep) r[0] += ctx.st.hOff || 0;
      if (t >= S.sim.refs[0].tStep) r[1] += ctx.st.zOff || 0;
      return r;
    };
  }

  // Simulation on the true plant with every disturbance kind (see system.js).
  // d_F and d_τ are input disturbances: mixed onto f_r, f_ℓ and saturated with u.
  function simulate(ctx, common, plantIgnored, controller) {
    const s = ctx.sys, p = ctx.pTrue, sim = ctx.S.sim;
    const dv = distValues(ctx);
    let ext = ZERO_EXT;
    const disturbance = (t) => {
      if (t < sim.tDist) { ext = ZERO_EXT; return [0, 0]; }
      ext = { Fwind: dv.Fwind || 0, wind: dv.wind || 0, ah: dv.ah || 0 };
      return s.mix(dv.dF || 0, dv.dtau || 0, p);
    };
    return WB.sim.simulate({
      ...common, disturbance,
      plant: { f: (x, u) => s.f(x, u, p, ext), h: (x) => s.h(x), uLimit: s.uLimit(p) },
      controller: controller || ctx.chapter.controller(ctx),
    });
  }

  // Linearized plant about hover with nominal parameters, no saturation, no
  // disturbance: ż̈ = −g θ − (μ/M) ż, ḧ = F̃/M, θ̈ = τ/J, where the controller
  // (linear: true) leaves out F_e so that f_r + f_ℓ = F̃.
  function linearPlant(p) {
    const m = WB.systems.F.models(p);
    return {
      f: (x, u) => {
        const Ft = u[0] + u[1], tau = p.d * (u[0] - u[1]);
        return [x[3], x[4], x[5], -p.g * x[2] - m.a * x[3], Ft / m.M, tau / m.J];
      },
      h: (x) => [x[0], x[1], x[2]],
      uLimit: [[-Infinity, Infinity], [-Infinity, Infinity]],
    };
  }
  function linearSim(ctx, common) {
    return WB.sim.simulate({ ...common, disturbance: null, noise: null, plant: linearPlant(ctx.pModel), controller: ctx.chapter.controller(ctx, { linear: true }) });
  }

  // ------------------------------------------------------- loop design --
  // PD gains for b0/(s² + a1 s + a0) with u = kP(r − y) − kD ẏ (Ch 7).
  const pdFromPoles = WB.design.pdGains;
  function polesWZ(wn, zeta) { return WB.design.polesFromWnZeta(wn, zeta); }

  // F.8 design from the tuning knobs: t_r,h, ζ_h; t_r,θ, ζ_θ; M = t_r,z/t_r,θ, ζ_z.
  // The outer loop sees the inner loop as its DC gain k_DCθ (Fig. 8-11, p. 118).
  function designSLC(p, k) {
    const m = WB.systems.F.models(p);
    const wnh = 2.2 / k.trh, wnt = 2.2 / k.trth, trz = k.Msep * k.trth, wnz = 2.2 / trz;
    const lon = pdFromPoles(m.lon, polesWZ(wnh, k.zetah));
    const inn = pdFromPoles(m.inner, polesWZ(wnt, k.zetath));
    const kDC = m.inner.b0 * inn.kP / (m.inner.b0 * inn.kP + m.inner.a0);
    const outer = { b0: m.outer.b0 * kDC, a1: m.outer.a1, a0: m.outer.a0 };
    const out = pdFromPoles(outer, polesWZ(wnz, k.zetaz));
    return {
      kPh: lon.kP, kDh: lon.kD, kIh: k.kIh || 0,
      kPth: inn.kP, kDth: inn.kD,
      kPz: out.kP, kDz: out.kD, kIz: k.kIz || 0,
      wnh, wnt, wnz, trz, kDC,
    };
  }

  // Closed-loop characteristic polynomials (derivative on the output; same as on
  // the error, since only the numerator changes).
  function lonPoly(m, g) {
    const b = m.lon.b0;
    return g.kIh ? [1, b * g.kDh, b * g.kPh, b * g.kIh] : [1, b * g.kDh, b * g.kPh];
  }
  function innerPoly(m, g) { const b = m.inner.b0; return [1, b * g.kDth, b * g.kPth]; }
  function kDCof(m, g) { const b = m.inner.b0; return g.kPth ? b * g.kPth / (b * g.kPth + m.inner.a0) : 0; }
  function outerPoly(m, g) {
    const b = m.outer.b0 * kDCof(m, g), a = m.outer.a1;
    return g.kIz ? [1, a + b * g.kDz, b * g.kPz, b * g.kIz] : [1, a + b * g.kDz, b * g.kPz];
  }
  // Exact lateral loop (inner loop dynamics kept): (s² + as)(s² + b_θk_Dθ s + b_θk_Pθ)·s^q − g b_θ k_Pθ (k_Dz s + k_Pz [+ k_Iz/s]) = 0
  function exactLatPoly(m, g) {
    const bt = m.inner.b0, a = m.outer.a1, gg = -m.outer.b0;
    let D = L.conv([1, a, 0], [1, bt * g.kDth, bt * g.kPth]);
    let N = [gg * bt * g.kPth * g.kDz, gg * bt * g.kPth * g.kPz];
    if (g.kIz) { D = L.conv(D, [1, 0]); N = [N[0], N[1], gg * bt * g.kPth * g.kIz]; }
    return L.polyAdd(D, N.map((v) => -v));
  }
  const rootsOf = (poly) => L.roots(poly);
  const wnOfPair = (poles) => {
    const cpx = poles.filter((q) => Math.abs(q.im) > 1e-9);
    const use = cpx.length ? cpx : poles;
    return Math.min(...use.map((q) => Math.hypot(q.re, q.im)));
  };

  // ------------------------------------------------------- nested PID --
  // F.7–F.10 controller: altitude PID → F = F_e + F̃ (or F̃/cos θ compensation);
  // lateral: outer PID on z produces θ_d, inner PD on θ produces τ. Then
  // (f_r, f_ℓ) = mix(F, τ). With st.deriv === 'dirty' the loops see only the
  // (noisy) measured outputs and use dirty derivatives (Eq. 10.4); otherwise the
  // true rates, as in the F.7–F.9 code. Uses the repo's PID conventions.
  function makePID(ctx, { linear = false, fe } = {}) {
    const s = ctx.sys, p = ctx.pModel, st = ctx.st, g = ctx.gains, Ts = ctx.S.sim.Ts;
    const m = s.models(p);
    const Fe = fe ?? feOf(ctx);
    const dirty = st.deriv === 'dirty';
    const sigma = st.sigma ?? 0.05;
    const aw = st.antiwindup || 'none';
    const blk = (kP, kI, kD, vbar) => WB.design.pidBlock({ kP, kI, kD, sigma, Ts, antiwindup: aw, vbar: vbar ?? Infinity, deriv: 'y' });
    const hB = blk(g.kPh, g.kIh || 0, g.kDh, st.vbarH);
    const zB = blk(g.kPz, g.kIz || 0, g.kDz, st.vbarZ);
    const tB = blk(g.kPth, 0, g.kDth);
    const latOn = st.lat !== 'off';
    const fl = compOf(ctx) === 'fl';
    return {
      update(r, x, y) {
        const meas = dirty ? y : [x[0], x[1], x[2]];
        const Ft = hB.update(r[0], meas[1], dirty ? {} : { ydot: x[4] });
        let F = Ft;
        if (!linear) F = fl ? (Fe + Ft) / Math.cos(meas[2]) : Fe + Ft;
        let tau = 0, thD = 0;
        if (latOn) {
          thD = zB.update(r[1], meas[0], dirty ? {} : { ydot: x[3] });
          tau = tB.update(thD, meas[2], dirty ? {} : { ydot: x[5] });
        }
        return { u: s.mix(F, tau, p), thetaD: latOn ? thD : undefined, Ftilde: Ft, tau, intH: hB.state.I, intZ: zB.state.I, hdotHat: hB.state.ydot };
      },
    };
  }

  // ----------------------------------------------------------- s-plane --
  // view: 'lon' | 'lat' | 'inner' | 'outer'. Marker kinds are reused with
  // custom legend names: lateral 'obs' = inner loop, 'cl' = outer loop,
  // 'olzero' = exact coupled poles of the full lateral loop.
  function pidSplane(ctx, g, { draggable = false, targets = null } = {}) {
    const m = ctx.sys.models(ctx.pModel);
    const view = ctx.st.view || 'lat';
    const mk = [];
    const names = {};
    // open-loop poles answer F.7(a) (altitude) and F.5(c) (lateral): Work mode hides them until solved
    const olShown = showsAnswer(ctx, view === 'lon' ? 'F.7/a' : 'F.5/c');
    const push = (poles, kind, label, dragId) => { if (kind !== 'ol' || olShown) poles.forEach((q) => mk.push({ ...q, kind, label, dragId: draggable ? dragId : undefined })); };
    if (view === 'lon') {
      push([{ re: 0, im: 0 }, { re: 0, im: 0 }], 'ol', 'open-loop pole');
      push(rootsOf(lonPoly(m, g)), 'cl', 'altitude closed-loop pole', 0);
      if (targets && targets.lon) push(targets.lon, 'target', 'target pole (spec)');
      names.cl = 'altitude loop pole';
    } else {
      if (view === 'inner' || view === 'lat') push(rootsOf(innerPoly(m, g)), view === 'lat' ? 'obs' : 'cl', 'inner (θ) loop pole', 10);
      if (view === 'outer' || view === 'lat') push(rootsOf(outerPoly(m, g)), 'cl', 'outer (z) loop pole, k_DC model', 20);
      if (view === 'lat') push(rootsOf(exactLatPoly(m, g)), 'olzero', 'exact pole of the coupled lateral loop');
      if (view === 'inner') push([{ re: 0, im: 0 }, { re: 0, im: 0 }], 'ol', 'open-loop pole');
      if (view === 'outer') push([{ re: 0, im: 0 }, { re: -m.a, im: 0 }], 'ol', 'open-loop pole');
      if (targets) {
        if ((view === 'inner' || view === 'lat') && targets.inner) push(targets.inner, 'target', 'target pole (spec)');
        if ((view === 'outer' || view === 'lat') && targets.outer) push(targets.outer, 'target', 'target pole (spec)');
      }
      Object.assign(names, view === 'lat'
        ? { obs: 'inner θ loop', cl: 'outer z loop (k_DC model)', olzero: 'exact coupled lateral' }
        : { cl: view === 'inner' ? 'inner θ loop pole' : 'outer z loop pole' });
    }
    return { markers: mk, legendNames: names, minR: 0.05 };
  }

  // Drag: lon (id 0) → t_r,h, ζ_h; inner (10) → t_r,θ, ζ_θ; outer (20) → M, ζ_z.
  function pidDrag(ctx, id, re, im) {
    const k = ctx.st.k;
    re = Math.min(-1e-3, re);
    const wn = Math.hypot(re, im), zeta = Math.max(0.2, Math.min(1, -re / wn));
    if (id === 0) { k.trh = Math.max(0.05, 2.2 / wn); k.zetah = zeta; }
    if (id === 10) { k.trth = Math.max(0.02, 2.2 / wn); k.zetath = zeta; }
    if (id === 20) { k.Msep = Math.max(1, (2.2 / wn) / k.trth); k.zetaz = zeta; }
    ctx.update();
  }

  // -------------------------------------------------------- UI helpers --
  const W0 = { kPh: 0.05, kDh: 0.3, kIh: 0, kPth: 0.1, kDth: 0.05, kPz: -0.002, kDz: -0.01, kIz: 0 };
  const GAIN_SPEC = {
    kPh: ['k<sub>P<sub>h</sub></sub>', 0, 3], kDh: ['k<sub>D<sub>h</sub></sub>', 0, 3], kIh: ['k<sub>I<sub>h</sub></sub>', 0, 0.2],
    kPth: ['k<sub>P<sub>θ</sub></sub>', 0, 3], kDth: ['k<sub>D<sub>θ</sub></sub>', 0, 1],
    kPz: ['k<sub>P<sub>z</sub></sub>', -0.1, 0], kDz: ['k<sub>D<sub>z</sub></sub>', -0.3, 0], kIz: ['k<sub>I<sub>z</sub></sub>', -0.01, 0],
  };
  const gainSliders = (parent, ctx, keys, obj = () => ctx.st.w) => WB.ui.gainSliders(parent, ctx, GAIN_SPEC, keys, { obj });
  function readout(parent, ctx, keys, get = () => ctx.gains) {
    return WB.ui.readout(parent, () => { const g = get(); return keys.map((k) => [k.label || k, g[k.key || k]]); });
  }
  const metricRow = WB.ui.metric;

  // Reference offsets (the left panel only sets amplitudes).
  function offsetControls(parent, ctx) {
    const sec = section(parent, 'Reference offsets', 'F.8(e) p. 397');
    slider(sec, { label: 'z<sub>r</sub> offset', unit: 'm', min: -5, max: 10, step: 0.1, sig: 3, get: () => ctx.st.zOff || 0, set: (v) => { ctx.st.zOff = v; ctx.update(); } });
    slider(sec, { label: 'h<sub>r</sub> offset', unit: 'm', min: -5, max: 10, step: 0.1, sig: 3, get: () => ctx.st.hOff || 0, set: (v) => { ctx.st.hOff = v; ctx.update(); },
      hint: 'Added to the left-panel references once they start: z_r = offset + amplitude·(step/square/sine).' });
  }

  // Lateral (z) step metrics on the first z_r step, like the app's metrics for h.
  function zMetrics(ctx) {
    const res = ctx.app.result();
    if (!res || !res.rAll[1]) return null;
    const S = ctx.S, zc = S.sim.refs[0], N = res.t.length;
    const idx = (t) => Math.max(0, Math.min(N, Math.round(t / S.sim.Ts)));
    const i0 = idx(zc.tStep);
    let i1 = N;
    if (zc.type === 'square') i1 = Math.min(i1, idx(zc.tStep + 0.5 / zc.frequency));
    const y = res.yAll[0], r = res.rAll[1];
    const i0c = Math.min(i0, N - 1);
    const m = M.stepMetrics(res.t, y, i0, i1, y[i0c], r[Math.min(i0 + 1, N - 1)]);
    let pkTh = 0;
    for (let k = 0; k < N; k++) pkTh = Math.max(pkTh, Math.abs(res.yAll[2][k]));
    m.peakTheta = pkTh * RAD;
    return m;
  }
  function zMetricsSection(parent, ctx) {
    const sec = section(parent, 'Lateral response (z)', 'p. 113');
    const box = el('div', { class: 'metrics' });
    sec.append(box);
    WB.ui.addRefresher(() => {
      const m = zMetrics(ctx);
      if (!m) { box.replaceChildren(); return; }
      box.replaceChildren(
        metricRow('z rise time (10–90%)', isNaN(m.tr) ? '—' : `${fmt(m.tr, 3)} s`),
        metricRow('z overshoot', isNaN(m.os) ? '—' : `${fmt(m.os, 3)} %`),
        metricRow('z settling (2%)', isNaN(m.ts) ? 'not settled in window' : `${fmt(m.ts, 3)} s`),
        metricRow('z error at window end', isNaN(m.ess) ? '—' : `${fmt(m.ess, 3)} m`),
        metricRow('peak |θ|', `${fmt(m.peakTheta, 3)}°`),
      );
    });
  }

  // Bandwidth separation readout for successive loop closure (§8.1.4, p. 118).
  function separationRows(ctx, g) {
    const m = ctx.sys.models(ctx.pModel);
    const wi = wnOfPair(rootsOf(innerPoly(m, g)));
    const wo = wnOfPair(rootsOf(outerPoly(m, g)));
    return { wi, wo, ratio: wi / wo, kDC: kDCof(m, g) };
  }

  function viewControl(parent, ctx, options = ['lon', 'lat', 'inner', 'outer']) {
    const labels = { lon: 'altitude', lat: 'lateral (both)', inner: 'inner θ', outer: 'outer z' };
    segmented(parent, {
      label: 's-plane shows', options: options.map((o) => ({ value: o, label: labels[o] })),
      ...bind(ctx, 'view'),
    });
  }

  // ------------------------------------------------------ Work-mode gates --
  // In Work mode, anything that answers problem part `key` ('F.4/c') stays
  // hidden until that part is solved (a passing Check, or a "done" button).
  const showsAnswer = (ctx, key) => ctx.S.mode === 'explore' || [].concat(key).every((k) => ctx.app.isSolved(k));

  // The hover force F_e = (m_c + 2m_r)g answers F.4(a). Until it is solved, Work
  // mode applies the student's own value instead (st.FeW, a slider that starts at
  // 0), so neither the rotor-force plot nor the animation readout shows it.
  const feShown = (ctx) => showsAnswer(ctx, 'F.4/a');
  const feOf = (ctx) => (feShown(ctx) ? WB.systems.F.models(ctx.pModel).Fe : (ctx.st.FeW || 0));
  function feSection(parent, ctx, { disabled } = {}) {
    if (feShown(ctx)) return;
    if (ctx.st.FeW === undefined) ctx.st.FeW = 0;
    const sec = section(parent, 'Hover force', 'F.4(a) p. 396');
    slider(sec, { label: 'your F<sub>e</sub>', unit: 'N', min: 0, max: 30, step: 0.01, sig: 4, ...bind(ctx, 'FeW'), disabled,
      hint: 'The simulation adds this F_e to F̃. It uses the workbench\'s value once you solve F.4(a) (Ch 4 tab).' });
    WB.ui.addRefresher(() => { sec.parentElement.hidden = feShown(ctx); });   // solved while this tab is open
  }

  // Force law F = F_e + F̃ or the feedback-linearized F = (F_e + F̃)/cos θ. The
  // second one is the answer to F.4(c), so Work mode offers it only once solved.
  const compOf = (ctx) => (ctx.st.comp === 'fl' && showsAnswer(ctx, 'F.4/c') ? 'fl' : 'eq');
  function forceLawControl(parent, ctx, { label = 'Force law', eqLabel = 'F = F<sub>e</sub> + F̃' } = {}) {
    if (!showsAnswer(ctx, 'F.4/c')) {
      parent.append(el('p', { class: 'muted small', text: 'Force law: F = F_e + F̃. A second option appears here once you solve F.4(c) (Ch 4 tab).' }));
      return;
    }
    segmented(parent, {
      label,
      options: [{ value: 'eq', label: eqLabel }, { value: 'fl', label: 'F = (F<sub>e</sub> + F̃)/cos θ' }],
      get: () => compOf(ctx), set: (v) => { ctx.st.comp = v; ctx.update(); },
    });
  }

  // ------------------------------------------------------ Python answers --
  // Every parameter of the model is randomized in the checks (m_r too, which
  // the "true plant" mismatch leaves alone), so answers must use P.<name>.
  const VARY = ['mc', 'mr', 'Jc', 'd', 'mu', 'g'];
  const ARGS = {
    z: { label: 'z', lo: -5, hi: 5 },
    h: { label: 'h', lo: -5, hi: 10 },
    theta: { label: 'θ', lo: -Math.PI, hi: Math.PI },
    zdot: { label: 'ż', lo: -3, hi: 3 },
    hdot: { label: 'ḣ', lo: -3, hi: 3 },
    thetadot: { label: 'θ̇', lo: -3, hi: 3 },
    fr: { label: 'f_r', lo: 0, hi: 10 },
    fl: { label: 'f_ℓ', lo: 0, hi: 10 },
    state: { col: ['z', 'h', 'theta', 'zdot', 'hdot', 'thetadot'] },
    u: { col: ['fr', 'fl'] },
    z_e: { label: 'z_e', lo: -5, hi: 5 },
    h_e: { label: 'h_e', lo: 0, hi: 10 },
    s: { label: 's', complex: true, re: [-6, 3], im: [0.3, 12] },
  };
  // code part {template, check}; spec as in WB.py.check, with ARGS and VARY filled in.
  const pyPart = (ctx, spec, template) => ({ template, check: (code) => WB.py.check(ctx, { vary: VARY, ...spec, args: { ...ARGS, ...(spec.args || {}) } }, code) });
  const pyError = (out) => ({ ok: false, msg: out.timeout ? out.error : 'Python raised an error.', detail: [out.error, out.where, (out.stdout || '').trim()].filter(Boolean).join('\n') });
  // explain() helper: is component j of the failing value the negative of the expected one?
  const flatV = (v) => (Array.isArray(v) ? v.flatMap(flatV) : [v]);
  function negated(f) {
    if (!f.e) return false;
    const g = Number(flatV(f.e.got)[f.j]), w = Number(flatV(f.e.want)[f.j]);
    return Math.abs(w) > 1e-12 && Math.abs(g + w) < 1e-6 * Math.max(1, Math.abs(w));
  }

  // ------------------------------------------------------ problem helpers --
  const PD = () => WB.pd;
  const useGains = (ctx, map, target = () => ctx.st.w) => WB.design.useGains(ctx, map, { target, msg: 'Fill in every gain first.' });

  // Simulated check helper: the error just before t (s) on output oi vs ref ri.
  function errorBefore(ctx, tSw, oi, ri) {
    const res = ctx.app.result(), S = ctx.S;
    const i = WB.sim.indexBefore(S, res, tSw);
    return res.rAll[ri][i] - res.yAll[oi][i];
  }

  // ------------------------------------------------------ chapter factory --
  // Adds the study's simulate/reference/linear hooks and the shared controls.
  // opts.lateral: show the z metrics; opts.offsets: show the offset controls.
  function chapter(def) {
    const ch = Object.assign({
      simulate(ctx, common) { ctx.chapter = this; return simulate(ctx, common); },
      reference,
    }, def);
    if (def.linear !== false && !def.openLoop && !def.linearSim) {
      ch.linearSim = function (ctx, common) { ctx.chapter = this; return linearSim(ctx, common); };
    }
    const userBuild = def.buildControls;
    ch.buildControls = function (parent, ctx) {
      ctx.chapter = this;
      if (def.usesFe !== false) feSection(parent, ctx, { disabled: def.feDisabled ? () => def.feDisabled(ctx) : undefined });
      userBuild.call(this, parent, ctx);
      if (def.lateralMetrics !== false && !def.openLoop) zMetricsSection(parent, ctx);
      if (!def.openLoop) offsetControls(parent, ctx);
    };
    // θ_d (from the outer loop) on the θ plot, and estimates where present.
    if (!def.outputSeries) {
      ch.outputSeries = function (ctx, res, sc, oi) {
        const out = [];
        if (oi === 2 && res.extras.thetaD) out.push({ label: 'θ_d (outer-loop command)', y: sc(res.extras.thetaD), color: '--ref', dash: [3, 3], width: 1.5 });
        const key = ['zhat', 'hhat', 'thhat'][oi];
        if (res.extras[key]) out.push({ label: `${['ẑ', 'ĥ', 'θ̂'][oi]} (observer)`, y: sc(res.extras[key]), color: '--series-3', dash: [3, 3], width: 2 });
        return out;
      };
    }
    return ch;
  }
  function register(def) {
    const ch = chapter(def);
    WB.studies.F.chapters[def.id] = ch;
    return ch;
  }

  // Reference designs (the answers), computed from the book's specs on the
  // current nominal parameters. Hidden in Work mode behind Reveal / Show solution.
  function refF8(p) {
    const pr = WB.systems.F.problems.ch8;
    return designSLC(p, { trh: pr.trh, zetah: pr.zetah, trth: pr.trth, zetath: pr.zetath, Msep: pr.Msep, zetaz: pr.zetaz });
  }
  function refF10(p) {
    const pr = WB.systems.F.problems.ch10;
    return { ...refF8(p), kIh: pr.kIh, kIz: pr.kIz };
  }

  return {
    RAD, distValues, reference, simulate, linearPlant, linearSim,
    pdFromPoles, polesWZ, designSLC, lonPoly, innerPoly, outerPoly, exactLatPoly, kDCof, rootsOf, wnOfPair,
    makePID, pidSplane, pidDrag, W0, gainSliders, readout, metricRow,
    offsetControls, zMetrics, zMetricsSection, separationRows, viewControl,
    useGains, errorBefore, chapter, register, refF8, refF10,
    showsAnswer, feShown, feOf, compOf, forceLawControl, VARY, ARGS, pyPart, pyError, negated,
    tex, texPole, fmt, fmtPole,
  };
})();
