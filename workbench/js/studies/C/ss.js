// Study C, chapters 11-14: full state feedback on the four-state satellite model,
// integral augmentation on φ, two-output observers and a disturbance observer.
// Mirrors ctrlStateFeedback.py, ctrlStateFeedbackIntegrator.py, ctrlObserver.py
// and ctrlDisturbanceObserver.py. The desired poles are two second-order pairs:
// a "θ" pair from t_rθ and a slower "φ" pair from t_rφ = M t_rθ (C.11, p. 192).
(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt, fmtPole } = M;
  const PD = () => WB.pd;
  const lib = () => WB.studies.C.lib;
  const CH = WB.studies.C.chapters;
  const R2D = 180 / Math.PI;

  // --------------------------------------------------------------- design --
  const augI = (A, B) => WB.design.augmentIntegrator(A, B, [[0, 1, 0, 0]]);   // ẋ_I = φ_r − φ (C_r = [0 1 0 0])
  const augD = (A, B, C) => WB.design.augmentDisturbance(A, B, C);
  const wnPair = (st) => {
    const wnTh = lib().wnRule(st.trTh, st.zetaTh, st.rule), wnPhi = lib().wnRule(st.M * st.trTh, st.zetaPhi, st.rule);
    return { wnTh, wnPhi, th: lib().pairPoles(wnTh, st.zetaTh), ph: lib().pairPoles(wnPhi, st.zetaPhi) };
  };
  // Observer poles: 10× faster by default (tr_obs = tr/10 in ctrlObserver.py).
  function obsDefaults(pr) {
    const r = (tr, z) => lib().wnRule(tr, z, pr.rule);
    return { wTh: r(pr.trTh / pr.obsFactor, pr.zetaTh), wPh: r(pr.M * pr.trTh / pr.obsFactor, pr.zetaPhi), zTh: pr.zetaTh, zPh: pr.zetaPhi };
  }
  const obsPoles = (o) => [...lib().pairPoles(o.wTh, o.zTh), ...lib().pairPoles(o.wPh, o.zPh)];

  // level: 'sf' (C.11), 'sfi' (C.12), 'obs' (C.13), 'dobs' (C.14)
  function design(p, st, level) {
    const { A, B, C } = lib().ss(p);
    const w = wnPair(st);
    const out = { ...w, poles: [...w.ph, ...w.th] };
    if (level === 'sf') {
      const K = WB.yt.place(A, B, out.poles);
      out.K = K ? K[0] : [NaN, NaN, NaN, NaN];
      const Ai = L.inv(L.sub(A, L.mul(B, [out.K])));
      out.kr = Ai ? -1 / L.mul(L.mul([[0, 1, 0, 0]], Ai), B)[0][0] : NaN;
      return out;
    }
    const { A1, B1 } = augI(A, B);
    out.poles = [...out.poles, { re: st.pI, im: 0 }];
    const K1 = WB.yt.place(A1, B1, out.poles);
    out.K = K1 ? K1[0].slice(0, 4) : [NaN, NaN, NaN, NaN];
    out.ki = K1 ? K1[0][4] : NaN;
    if (level === 'obs' || level === 'dobs') {
      out.obsPoles = obsPoles(st.obs);
      out.L = WB.yt.observer(A, C, out.obsPoles);
    }
    if (level === 'dobs') {
      const { A2, C2 } = augD(A, B, C);
      out.dPoles = [...out.obsPoles, { re: st.pD, im: 0 }];
      out.L2 = WB.yt.observer(A2, C2, out.dPoles);
    }
    return out;
  }

  // Work mode: K (and k_r / k_I) from the sliders; observers always from poles.
  // Observer poles in use: work mode has its own (not the problem's answer).
  const obsOf = (ctx) => (ctx.S.mode === 'work' && ctx.st.obsW ? ctx.st.obsW : ctx.st.obs);
  function gainsFor(ctx, level) {
    const p = ctx.pModel, st = { ...ctx.st, obs: obsOf(ctx) };
    const d = design(p, st, level);
    if (ctx.S.mode === 'explore') return d;
    const w = ctx.st.w;
    return { ...d, K: [w.K1, w.K2, w.K3, w.K4], kr: w.kr, ki: w.ki };
  }

  // ----------------------------------------------------------- controller --
  function makeSS(ctx, level) {
    const { sys, pModel, S } = ctx;
    const st = ctx.st, g = ctx.gains;
    const { A, B, C } = lib().ss(pModel);
    const Ts = S.sim.Ts, uLim = sys.uLimit(pModel);
    const useDO = level === 'dobs' && st.dobs;
    const useObs = level === 'obs' || level === 'dobs';
    const { A2, C2 } = augD(A, B, C);
    const Ao = useDO ? A2 : A, Co = useDO ? C2 : C, Lo = useDO ? g.L2 : g.L;
    const Bo = useDO ? [...B.map((r) => r[0]), 0] : B.map((r) => r[0]);
    const no = Ao.length;
    let xh = new Array(no).fill(0);
    xh[0] = (st.xhat0 || 0) / R2D;
    let I = 0, ePrev = 0, tauPrev = 0;
    // observer_f: A x̂ + B u_{k−1} + L (y − C x̂)
    const fObs = (z, y) => {
      const innov = Co.map((row, i) => y[i] - row.reduce((s, c, j) => s + c * z[j], 0));
      return Ao.map((row, i) => row.reduce((s, a, j) => s + a * z[j], 0) + Bo[i] * tauPrev + Lo[i].reduce((s, l, j) => s + l * innov[j], 0));
    };
    const Kx = (x) => g.K[0] * x[0] + g.K[1] * x[1] + g.K[2] * x[2] + g.K[3] * x[3];
    return {
      update(r, x, yMeas) {
        let xu = x;
        if (useObs) { xh = M.rk4Step((zz) => fObs(zz, yMeas), xh, 0, Ts); xu = xh; }
        const dh = useDO ? xh[4] : 0;
        let tauU;
        if (level === 'sf') {
          tauU = -Kx(xu) + g.kr * r;
        } else {
          const e = r - xu[1];
          const Inew = I + (Ts / 2) * (e + ePrev);
          tauU = -Kx(xu) - g.ki * Inew - dh;
          // anti-windup (C.12a): hold the integrator while τ is saturated
          if (st.antiwindup === 'clamp' && Math.abs(tauU) > uLim) tauU = -Kx(xu) - g.ki * I - dh;
          else I = Inew;
          ePrev = e;
        }
        // The observer uses the saturated τ (tau_d1 in the repo); the demand is
        // returned unsaturated and the simulation clips it.
        tauPrev = M.saturate(tauU, uLim);
        return { u: tauU, thHat: xu[0], phHat: xu[1], thdHat: xu[2], phdHat: xu[3], dhat: dh, integrator: I };
      },
    };
  }

  // ------------------------------------------------------------- controls --
  function tuningSliders(parent, ctx, { pI = false } = {}) {
    const st = ctx.st;
    slider(parent, { label: 't<sub>r<sub>θ</sub></sub>', unit: 's', min: 0.2, max: 6, step: 0.01, sig: 3, get: () => st.trTh, set: (v) => { st.trTh = v; ctx.update(); } });
    slider(parent, { label: 'ζ<sub>θ</sub>', min: 0.2, max: 0.99, step: 0.005, sig: 3, get: () => st.zetaTh, set: (v) => { st.zetaTh = v; ctx.update(); } });
    slider(parent, { label: 'M = t<sub>r<sub>φ</sub></sub>/t<sub>r<sub>θ</sub></sub>', min: 1, max: 10, step: 0.05, sig: 3, get: () => st.M, set: (v) => { st.M = v; ctx.update(); } });
    slider(parent, { label: 'ζ<sub>φ</sub>', min: 0.2, max: 0.99, step: 0.005, sig: 3, get: () => st.zetaPhi, set: (v) => { st.zetaPhi = v; ctx.update(); } });
    segmented(parent, {
      label: 'ω<sub>n</sub> from t<sub>r</sub>',
      options: [{ value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))' }, { value: '2.2', label: '2.2 / t<sub>r</sub>' }],
      get: () => st.rule, set: (v) => { st.rule = v; ctx.update(); },
    });
    if (pI) slider(parent, { label: 'p<sub>I</sub>', min: -10, max: -0.05, step: 0.01, sig: 3, get: () => st.pI, set: (v) => { st.pI = v; ctx.update(); } });
  }
  function obsSliders(parent, ctx, { withD = false } = {}) {
    const o = obsOf(ctx);
    slider(parent, { label: 'ω<sub>n,obs,θ</sub>', unit: 'rad/s', min: 0.5, max: 60, step: 0.05, sig: 4, get: () => o.wTh, set: (v) => { o.wTh = v; ctx.update(); } });
    slider(parent, { label: 'ω<sub>n,obs,φ</sub>', unit: 'rad/s', min: 0.2, max: 40, step: 0.05, sig: 4, get: () => o.wPh, set: (v) => { o.wPh = v; ctx.update(); } });
    slider(parent, { label: 'ζ<sub>obs</sub>', min: 0.3, max: 0.99, step: 0.005, sig: 3, get: () => o.zTh, set: (v) => { o.zTh = v; o.zPh = v; ctx.update(); } });
    if (withD) slider(parent, { label: 'p<sub>d</sub>', min: -40, max: -0.2, step: 0.1, sig: 3, get: () => ctx.st.pD, set: (v) => { ctx.st.pD = v; ctx.update(); } });
  }
  function workSliders(parent, ctx, keys) {
    const spec = { K1: ['K<sub>1</sub>', -50, 200], K2: ['K<sub>2</sub>', -200, 1000], K3: ['K<sub>3</sub>', -20, 100], K4: ['K<sub>4</sub>', -200, 1500], kr: ['k<sub>r</sub>', 0, 200], ki: ['k<sub>I</sub>', -300, 0] };
    for (const k of keys) {
      const [label, min, max] = spec[k];
      slider(parent, { label, min, max, step: (max - min) / 4000, sig: 4, get: () => ctx.st.w[k], set: (v) => { ctx.st.w[k] = v; ctx.update(); } });
    }
  }
  function targetToggle(parent, ctx) {
    segmented(parent, {
      label: 'Target poles from the problem (s-plane rings)',
      options: [{ value: false, label: 'hidden' }, { value: true, label: 'shown' }],
      get: () => !!ctx.st.showTargets, set: (v) => { ctx.st.showTargets = v; ctx.update(); },
    });
  }
  function awControl(parent, ctx) {
    segmented(parent, {
      label: 'Anti-windup (C.12a)',
      options: [{ value: 'clamp', label: 'hold integrator while saturated' }, { value: 'none', label: 'none (repo)' }],
      get: () => ctx.st.antiwindup, set: (v) => { ctx.st.antiwindup = v; ctx.update(); },
    });
  }
  function gainsReadout(parent, ctx, keys) {
    const box = el('div', { class: 'readout wrap' });
    parent.append(box);
    WB.ui.addRefresher(() => {
      const g = ctx.gains;
      const v = { K1: g.K[0], K2: g.K[1], K3: g.K[2], K4: g.K[3], kr: g.kr, ki: g.ki };
      box.replaceChildren(...keys.map((k) => el('div', {}, el('span', { class: 'ro-label', text: k }), el('strong', { text: fmt(v[k], 4) }))));
    });
  }

  // ------------------------------------------------------------- analysis --
  function clPoles(ctx, level) {
    const { A, B } = lib().ss(ctx.pModel), g = ctx.gains;
    if (level === 'sf') return L.eig(L.sub(A, L.mul(B, [g.K])));
    const { A1, B1 } = augI(A, B);
    return L.eig(L.sub(A1, L.mul(B1, [[...g.K, g.ki]])));
  }

  function markers(ctx, level, specPoles) {
    const { A } = lib().ss(ctx.pModel), g = ctx.gains, st = ctx.st;
    const explore = ctx.S.mode === 'explore';
    const mk = L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `open-loop pole ${i + 1}` }));
    if (explore) {
      g.th.forEach((q) => mk.push({ ...q, kind: 'cl', label: 'controller pole (θ pair)', dragId: 'cth' }));
      g.ph.forEach((q) => mk.push({ ...q, kind: 'cl', label: 'controller pole (φ pair)', dragId: 'cph' }));
      if (level !== 'sf') mk.push({ re: st.pI, im: 0, kind: 'cl', label: 'integrator pole p_I', dragId: 'cI' });
    } else {
      clPoles(ctx, level).forEach((q, i) => mk.push({ ...q, kind: 'cl', label: `closed-loop pole ${i + 1}` }));
      if (st.showTargets) specPoles.forEach((q) => mk.push({ ...q, kind: 'target', label: 'target pole (problem)' }));
    }
    if (level === 'obs' || level === 'dobs') {
      const o = obsOf(ctx);
      const dg = (id) => (explore ? id : undefined);
      lib().pairPoles(o.wTh, o.zTh).forEach((q) => mk.push({ ...q, kind: 'obs', label: 'observer pole (θ pair)', dragId: dg('oth') }));
      lib().pairPoles(o.wPh, o.zPh).forEach((q) => mk.push({ ...q, kind: 'obs', label: 'observer pole (φ pair)', dragId: dg('oph') }));
      if (level === 'dobs' && st.dobs) mk.push({ re: st.pD, im: 0, kind: 'obs', label: 'disturbance-observer pole p_d', dragId: dg('od') });
    }
    return mk;
  }

  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    const wn = Math.hypot(re, im);
    const zeta = Math.max(0.2, Math.min(0.99, -re / wn));
    const inv = (w, z) => WB.design.trFromWn(w, z, st.rule);
    if (id === 'cth') { st.zetaTh = zeta; st.trTh = inv(wn, zeta); }
    else if (id === 'cph') { st.zetaPhi = zeta; st.M = Math.max(1, inv(wn, zeta) / st.trTh); }
    else if (id === 'cI') st.pI = re;
    else if (id === 'oth') { st.obs.wTh = wn; st.obs.zTh = zeta; }
    else if (id === 'oph') { st.obs.wPh = wn; st.obs.zPh = zeta; }
    else if (id === 'od') st.pD = re;
    ctx.update();
  }

  // ----------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B } = lib().ss(ctx.pModel);
    return {
      title: 'State-space model', page: 'p. 92, p. 193 · Eq. 11.40',
      theory: '\\dot x = Ax + B\\tau,\\quad x = (\\theta, \\phi, \\dot\\theta, \\dot\\phi)^\\top,\\quad y = (\\theta, \\phi)^\\top,\\quad y_r = \\phi = \\begin{bmatrix}0 & 1 & 0 & 0\\end{bmatrix}x',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`, spoiler: true,
    };
  }
  function polesCard(ctx, d, level) {
    const st = ctx.st;
    const wTex = st.rule === 'tp' ? '\\omega_n = \\frac{\\pi}{2t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}';
    return {
      title: 'Desired closed-loop poles', page: 'p. 193, p. 194 (Listing 11.3)',
      theory: `${wTex},\\quad t_{r_\\phi} = M t_{r_\\theta},\\quad \\Delta^d = (s^2 + 2\\zeta_\\theta\\omega_{n_\\theta}s + \\omega_{n_\\theta}^2)(s^2 + 2\\zeta_\\phi\\omega_{n_\\phi}s + \\omega_{n_\\phi}^2)` + (level === 'sf' ? '' : '(s - p_I)'),
      numbers: `\\omega_{n_\\theta} = ${tex(d.wnTh)},\\; \\omega_{n_\\phi} = ${tex(d.wnPhi)},\\quad \\Delta^d = ${WB.tf.polyTex(L.polyFromRoots(d.poles))}`,
      spoiler: true,
    };
  }
  const ctrbCard = (A, B, title, page) => WB.ss.ctrbCard(A, B, title, page);
  // Work-mode starting gains: a slow, stable design (t_rθ = 4 s, ζ = 0.8, M = 2.5,
  // p_I = −0.8), rounded. Deliberately not the answers.
  const W0 = { K1: 7.7, K2: 3.2, K3: 11, K4: 33.5, kr: 1.5, ki: -1.2 };
  const W0sf = { K1: 3.5, K2: -2, K3: 7, K4: 4.8, kr: 1.5, ki: -1.2 };

  function stateDefaults(pr, extra = {}) {
    return { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, pI: pr.pI ?? -2, antiwindup: 'clamp', w: { ...W0 }, ...extra };
  }
  const specOf = (ctx, pr, level) => design(ctx.pModel, { ...ctx.st, trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, pI: pr.pI ?? -2, obs: pr.obsFactor ? obsDefaults(pr) : ctx.st.obs, pD: pr.pD ?? ctx.st.pD }, level);

  const useGains = (ctx, keys) => ({
    label: 'Use my gains',
    run: (v) => {
      const vals = keys.map((k) => PD().num(v[k]));
      if (vals.some((x) => x === null)) return { ok: false, msg: `Enter ${keys.join(', ')}.` };
      ctx.app.setMode('work');
      keys.forEach((k, i) => { ctx.st.w[k] = vals[i]; });
      ctx.update(); return null;
    },
  });
  const errBeforeSwitch = (ctx) => {
    const res = ctx.app.result(), S = ctx.S;
    const tSw = S.sim.type === 'square' ? S.sim.tStep + 0.5 / S.sim.frequency : S.sim.tEnd;
    const i = Math.min(res.t.length - 1, Math.round((tSw - 0.05) / S.sim.Ts));
    return Math.abs(res.rAll[0][i] - res.yAll[1][i]) * R2D;
  };

  function base(level, num, title, pages, extra) {
    return Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages, level,
      controller(ctx) { return makeSS(ctx, level); },
      gains(ctx) { return gainsFor(ctx, level); },
      splane(ctx) {
        const pr = ctx.sys.problems[`ch${num}`];
        return { markers: markers(ctx, level, specOf(ctx, pr, level).poles), legendNames: { cl: 'controller pole', obs: 'observer pole', target: 'target pole (problem)' } };
      },
      onPoleDrag: onDrag,
      targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.M * ctx.st.trTh } : {}; },
    }, extra);
  }

  // ------------------------------------------------------------- C.11 --
  CH.ch11 = base('sf', 11, 'Full state feedback', 'pp. 192–195', {
    defaults(sys) { return stateDefaults(sys.problems.ch11, { w: { ...W0sf } }); },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'u = −Kx + k_r φ_r (true state)', 'p. 192 · Listing 11.3');
      if (ctx.S.mode === 'work') { workSliders(sec, ctx, ['K1', 'K2', 'K3', 'K4', 'kr']); targetToggle(sec, ctx); }
      else { tuningSliders(sec, ctx); gainsReadout(sec, ctx, ['K1', 'K2', 'K3', 'K4', 'kr']); sec.append(el('p', { class: 'muted small', text: 'Drag a θ-pair or φ-pair pole in the s-plane.' })); }
    },
    math(ctx) {
      const d = design(ctx.pModel, ctx.st, 'sf'), g = ctx.gains;
      const { A, B } = lib().ss(ctx.pModel);
      return [
        ssCard(ctx),
        ctrbCard(A, B, 'Controllability', 'p. 193 · Step 1'),
        { title: 'Open-loop characteristic polynomial', page: 'p. 193 · Step 2',
          theory: '\\Delta_{ol}(s) = \\det(sI - A) = s^4 + a_3s^3 + a_2s^2 + a_1s + a_0',
          numbers: `\\Delta_{ol} = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
        polesCard(ctx, d, 'sf'),
        { title: 'Pole placement and reference gain', page: 'p. 194 · Step 4, Eq. 11.35',
          theory: 'K = (\\alpha - a_A)\\mathcal{A}_A^{-1}\\mathcal{C}_{A,B}^{-1},\\quad k_r = \\frac{-1}{C_r(A - BK)^{-1}B},\\; C_r = \\begin{bmatrix}0 & 1 & 0 & 0\\end{bmatrix}',
          numbers: `K = ${texMat([d.K])},\\quad k_r = ${tex(d.kr)}`, spoiler: true,
          note: 'At steady state the spring forces θ = φ, so C_r = [1 0 0 0] (what the listing uses) gives the same k_r.' },
        { title: 'Control law', page: 'p. 195 · Listing 11.3',
          theory: '\\tau = \\text{sat}\\left(-Kx + k_r\\phi_r\\right)',
          numbers: `\\tau = -(${g.K.map((v) => tex(v)).join(',\\;')})\\,x + ${tex(g.kr)}\\,\\phi_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch11;
      const ref = () => specOf(ctx, pr, 'sf');
      const { A, B } = lib().ss(ctx.pModel);
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: `(a) Desired poles for the listing's tuning: t<sub>r<sub>θ</sub></sub> = ${pr.trTh} s, M = ${pr.M}, ζ = ${pr.zetaTh} (ω<sub>n</sub> = π/(2t<sub>r</sub>√(1−ζ²)))`,
          html: 'Δ<sup>d</sup>(s) = s⁴ + α<sub>3</sub>s³ + α<sub>2</sub>s² + α<sub>1</sub>s + α<sub>0</sub>',
          inputs: { wnTh: 'ω<sub>n<sub>θ</sub></sub>', wnPhi: 'ω<sub>n<sub>φ</sub></sub>', a3: 'α<sub>3</sub>', a2: 'α<sub>2</sub>', a1: 'α<sub>1</sub>', a0: 'α<sub>0</sub>' },
          check: (v) => { const r = ref(), c = L.polyFromRoots(r.poles); return PD().checkNumbers(v, { wnTh: r.wnTh, wnPhi: r.wnPhi, a3: c[1], a2: c[2], a1: c[3], a0: c[4] }, { wnTh: 'ωnθ', wnPhi: 'ωnφ' }); },
          solution: () => { const r = ref(); return [{ tex: `\\omega_{n_\\theta} = ${tex(r.wnTh)},\\; \\omega_{n_\\phi} = ${tex(r.wnPhi)},\\quad p = ${r.poles.map((q) => texPole(q)).join(',\\;')}` }, { tex: `\\Delta^d = ${WB.tf.polyTex(L.polyFromRoots(r.poles))}` }, { html: 'The printed solution (p. 193) uses ω<sub>θ</sub> = 1.9848, ω<sub>φ</sub> = 1.5, ζ = 0.707 instead; see ISSUES.md.' }]; },
        },
        {
          id: 'c', title: '(c) Controllability',
          inputs: { rank: 'rank 𝒞<sub>A,B</sub>', det: 'det 𝒞<sub>A,B</sub>' },
          check: (v) => { const Cm = L.ctrb(A, B); return PD().checkNumbers(v, { rank: L.rank(Cm), det: L.det(Cm) }, { det: 'det' }); },
          solution: () => { const Cm = L.ctrb(A, B); return [{ tex: `\\mathcal{C}_{A,B} = ${texMat(Cm)},\\quad \\det = ${tex(L.det(Cm))} \\ne 0` }]; },
        },
        {
          id: 'd', title: '(d) K and k<sub>r</sub>',
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], K3: r.K[2], K4: r.K[3], kr: r.kr }, {}); },
          actions: [useGains(ctx, ['K1', 'K2', 'K3', 'K4', 'kr'])],
          solution: () => { const r = ref(); return [{ tex: `K = ${texMat([r.K])},\\quad k_r = ${tex(r.kr)}` }, { html: 'Book (p. 194): K = (40.28, 255.17, 24.34, 366.18), k<sub>r</sub> = 295.46, for its own poles and a different A.' }]; },
        },
      ]);
    },
  });

  // ------------------------------------------------------------- C.12 --
  CH.ch12 = base('sfi', 12, 'Integrator with state feedback', 'pp. 210–214', {
    defaults(sys) { return stateDefaults(sys.problems.ch12); },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'u = −Kx − k_I ∫(φ_r − φ)', 'p. 211 · Listing 12.3');
      awControl(sec, ctx);
      if (ctx.S.mode === 'work') { workSliders(sec, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']); targetToggle(sec, ctx); }
      else { tuningSliders(sec, ctx, { pI: true }); gainsReadout(sec, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']); }
    },
    extraPlot(ctx, res) {
      return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [rad·s]', unit: 'rad·s' }, data: { series: [{ label: 'x_I = ∫(φ_r − φ)', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = design(ctx.pModel, ctx.st, 'sfi');
      const { A, B } = lib().ss(ctx.pModel);
      const { A1, B1 } = augI(A, B);
      return [
        ssCard(ctx),
        { title: 'Augmented system', page: 'p. 211 · Step 1',
          theory: '\\dot x_I = \\phi_r - C_rx,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C_r & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\ 0\\end{bmatrix},\\quad C_r = \\begin{bmatrix}0 & 1 & 0 & 0\\end{bmatrix}',
          numbers: `A_1 = ${texMat(A1)}` },
        ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 212'),
        polesCard(ctx, d, 'sfi'),
        { title: 'Gains', page: 'p. 212 · Step 3',
          theory: 'K_1 = \\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad \\tau = -Kx - k_I\\int_0^t(\\phi_r - \\phi)\\,d\\tau',
          numbers: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch12;
      const ref = () => specOf(ctx, pr, 'sfi');
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: `(a) Gains with the listing's tuning (t<sub>r<sub>θ</sub></sub> = ${pr.trTh}, M = ${pr.M}, ζ = ${pr.zetaTh}, p<sub>I</sub> = ${pr.pI})`,
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', ki: 'k<sub>I</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], K3: r.K[2], K4: r.K[3], ki: r.ki }, {}); },
          actions: [useGains(ctx, ['K1', 'K2', 'K3', 'K4', 'ki'])],
          solution: () => { const r = ref(); return [{ tex: `K = ${texMat([r.K])},\\quad k_I = ${tex(r.ki)}` }, { html: 'Book (p. 212): K = (19.15, 43.41, 16.72, 111.63), k<sub>I</sub> = −14.52, for p<sub>I</sub> = −1 and different poles and A.' }]; },
        },
        {
          id: 'c', title: '(b, c) Tracking with d = 1 N·m and 20% uncertainty',
          html: 'Passes when |φ<sub>r</sub> − φ| just before the first reference switch is under 0.1° with the current disturbance and mismatch.',
          check: () => { const e = errBeforeSwitch(ctx); return { ok: e < 0.1, msg: `Error before the switch: ${fmt(e, 3)}° (d = ${fmt(ctx.S.sim.dist, 3)} N·m).` }; },
        },
      ]);
    },
  });

  // ------------------------------------------------------- C.13 and C.14 --
  function obsSeries(ctx, res, sc, oi) {
    const key = oi === 0 ? 'thHat' : 'phHat';
    return [{ label: `${oi === 0 ? 'θ̂' : 'φ̂'} (observer)`, y: sc(res.extras[key] || []), color: '--series-3', dash: [3, 3], width: 2 }];
  }
  function obsCards(ctx, d, level) {
    const { A, C } = lib().ss(ctx.pModel);
    const O = L.obsv(A, C);
    const cards = [
      { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 237 · Listing 13.3',
        theory: '\\dot{\\hat x} = A\\hat x + B\\tau_{k-1} + L(y_m - C\\hat x),\\quad y = (\\theta, \\phi)^\\top,\\quad L \\in \\mathbb{R}^{4\\times 2}' },
      { title: 'Observability', page: 'p. 221',
        theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ CA^2 \\\\ CA^3\\end{bmatrix},\\quad \\operatorname{rank}\\mathcal{O}_{A,C} = 4',
        numbers: `\\operatorname{rank}\\mathcal{O}_{A,C} = ${L.rank(O)}\\;(\\text{already } ${L.rank(O.slice(0, 4))} \\text{ from } C, CA)`, spoiler: true },
      { title: 'Observer gain (two outputs)', page: 'p. 236 · Listing 13.3',
        theory: 'L = \\text{place}(A^\\top, C^\\top, q)^\\top,\\quad q = \\text{roots}\\big((s^2 + 2\\zeta\\omega_{obs,\\theta}s + \\omega_{obs,\\theta}^2)(s^2 + 2\\zeta\\omega_{obs,\\phi}s + \\omega_{obs,\\phi}^2)\\big)',
        numbers: d.L ? `q = ${d.obsPoles.map((q) => texPole(q)).join(',\\;')},\\quad L^\\top = ${texMat(L.T(d.L))}` : '', spoiler: true,
        note: 'With two outputs many L place the same poles. The one shown is what python-control\'s place (scipy\'s YT algorithm) returns, ported here so the simulation matches the repo.' },
    ];
    if (level === 'dobs') {
      const { B } = lib().ss(ctx.pModel);
      const { A2, C2 } = augD(A, B, C);
      cards.push({ title: 'Disturbance observer', page: 'p. 240–241, pp. 258–259 · Listing 14.6',
        theory: 'A_2 = \\begin{bmatrix}A & B\\\\ 0 & 0\\end{bmatrix},\\; C_2 = \\begin{bmatrix}C & 0\\end{bmatrix},\\quad \\tau = -K\\hat x - k_I\\textstyle\\int e - \\hat d',
        numbers: `\\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}` + (d.L2 ? `,\\quad L_2^\\top = ${texMat(L.T(d.L2))}` : ''), spoiler: true });
    }
    return cards;
  }

  CH.ch13 = base('obs', 13, 'Observers', 'pp. 234–238', {
    defaults(sys) { const pr = sys.problems.ch13; return stateDefaults(pr, { obs: obsDefaults(pr), obsW: obsDefaults({ ...pr, obsFactor: 4 }), xhat0: 0, antiwindup: 'none' }); },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries: obsSeries,
    extraPlot(ctx, res) {
      return { opts: { title: 'estimation error', yLabel: 'error [°]', unit: '°' }, data: { series: [
        { label: 'θ − θ̂', y: res.x.map((x, k) => (x[0] - (res.extras.thHat || [])[k]) * R2D), color: '--series-1' },
        { label: 'φ − φ̂', y: res.x.map((x, k) => (x[1] - (res.extras.phHat || [])[k]) * R2D), color: '--series-3', width: 1.5 },
      ] } };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂)', 'p. 235 · Listing 13.3');
      awControl(sec, ctx);
      if (ctx.S.mode === 'work') { workSliders(sec, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']); targetToggle(sec, ctx); }
      else tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer poles', 'p. 236');
      obsSliders(ob, ctx);
      slider(ob, { label: 'θ̂(0)', unit: '°', min: -30, max: 30, step: 0.5, sig: 3, hint: 'initial estimate (the plant starts at the left panel values)', get: () => ctx.st.xhat0, set: (v) => { ctx.st.xhat0 = v; ctx.update(); } });
      if (ctx.S.mode === 'explore') gainsReadout(ob, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']);
      ob.append(el('p', { class: 'muted small', text: 'L has eight entries and is not unique with two outputs, so in both modes you pick observer poles and L is placed for you.' }));
    },
    math(ctx) {
      const d = design(ctx.pModel, { ...ctx.st, obs: obsOf(ctx) }, 'obs');
      return [ssCard(ctx), ...obsCards(ctx, d, 'obs'), polesCard(ctx, d, 'obs'),
        { title: 'Separation principle', page: 'p. 223–224',
          theory: '\\text{eig}\\begin{bmatrix}A - BK & BK\\\\ 0 & A - LC\\end{bmatrix} = \\text{eig}(A - BK) \\cup \\text{eig}(A - LC)' }];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch13;
      const { A, C } = lib().ss(ctx.pModel);
      const od = () => obsDefaults(pr);
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'b', title: '(b) Observability',
          inputs: { rank: 'rank 𝒪<sub>A,C</sub>' },
          check: (v) => PD().checkNumbers(v, { rank: L.rank(L.obsv(A, C)) }, {}),
          solution: () => [{ tex: '\\mathcal{O}_{A,C} = \\begin{bmatrix}C\\\\ CA\\end{bmatrix} = \\begin{bmatrix}I & 0\\\\ 0 & I\\end{bmatrix} \\text{ already has rank 4}' }],
        },
        {
          id: 'c', title: `(c) Observer poles 10× faster (t<sub>r,obs</sub> = t<sub>r</sub>/${pr.obsFactor}, as in ctrlObserver.py)`,
          html: 'Desired observer polynomial s⁴ + β<sub>3</sub>s³ + β<sub>2</sub>s² + β<sub>1</sub>s + β<sub>0</sub>.',
          inputs: { wTh: 'ω<sub>n,obs,θ</sub>', wPh: 'ω<sub>n,obs,φ</sub>', b3: 'β<sub>3</sub>', b0: 'β<sub>0</sub>' },
          check: (v) => { const o = od(), c = L.polyFromRoots(obsPoles(o)); return PD().checkNumbers(v, { wTh: o.wTh, wPh: o.wPh, b3: c[1], b0: c[4] }, { wTh: 'ωobs,θ', wPh: 'ωobs,φ' }); },
          solution: () => { const o = od(), Lg = WB.yt.observer(A, C, obsPoles(o)); return [{ tex: `\\omega_{obs,\\theta} = ${tex(o.wTh)},\\; \\omega_{obs,\\phi} = ${tex(o.wPh)},\\quad \\Delta_{obs} = ${WB.tf.polyTex(L.polyFromRoots(obsPoles(o)))}` }, { tex: `L^\\top = ${texMat(L.T(Lg))}` }, { html: 'One of many valid L; this is the repo\'s (place → scipy YT).' }]; },
        },
        {
          id: 'e', title: '(e) Add an input disturbance of 1.0 N·m',
          html: 'Set d = 1 in the left panel, then measure the run. There is no single right number here; compare φ − φ̂ and φ<sub>r</sub> − φ at the end, and see C.14.',
          actions: [{ label: 'Measure', run: () => {
            const res = ctx.app.result(), n = res.t.length - 1;
            if (!(Math.abs(ctx.S.sim.dist) > 0)) return { ok: false, msg: 'Set d ≠ 0 first.' };
            const bias = (res.yAll[1][n] - res.extras.phHat[n]) * R2D;
            return { ok: true, msg: `At t_end: φ − φ̂ = ${fmt(bias, 3)}°, φ_r − φ = ${fmt((res.rAll[0][n] - res.yAll[1][n]) * R2D, 3)}°.` };
          } }],
          solution: () => [{ html: 'The integrator drives φ<sub>r</sub> − φ̂ to zero, but the observer does not model d, so φ̂ is biased and φ settles away from φ<sub>r</sub>. The disturbance observer of C.14 removes that bias.' }],
        },
      ]);
    },
  });

  CH.ch14 = base('dobs', 14, 'Disturbance observers', 'pp. 254–259', {
    defaults(sys) { const pr = sys.problems.ch14; return stateDefaults(pr, { obs: obsDefaults(pr), obsW: obsDefaults({ ...pr, obsFactor: 4 }), pD: pr.pD, dobs: true, xhat0: 0, antiwindup: 'none' }); },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries: obsSeries,
    extraPlot(ctx, res) {
      const S = ctx.S;
      return { opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N·m]', unit: 'N·m' }, data: { series: [
        { label: 'd̂ (estimate)', y: Array.from(res.extras.dhat || []), color: '--series-3', width: 2 },
        { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
      ] } };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'pp. 258–259 · Listing 14.6');
      segmented(sec, {
        label: 'Disturbance observer',
        options: [{ value: true, label: 'on' }, { value: false, label: 'off: C.13 observer (C.14a)' }],
        get: () => ctx.st.dobs, set: (v) => { ctx.st.dobs = v; ctx.update(); },
      });
      awControl(sec, ctx);
      if (ctx.S.mode === 'work') { workSliders(sec, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']); targetToggle(sec, ctx); }
      else tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer poles', 'p. 257');
      obsSliders(ob, ctx, { withD: true });
      if (ctx.S.mode === 'explore') gainsReadout(ob, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']);
    },
    math(ctx) {
      const d = design(ctx.pModel, { ...ctx.st, obs: obsOf(ctx) }, 'dobs');
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(u + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0 \\text{ for constant } d' },
        ...obsCards(ctx, d, 'dobs'), polesCard(ctx, d, 'dobs'),
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch14;
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Without the disturbance observer',
          html: 'Switch the disturbance observer off. Passes when the current run shows the C.13 controller failing to remove the 1 N·m disturbance (|φ<sub>r</sub> − φ| before the first switch above 0.1°).',
          check: () => {
            if (ctx.st.dobs) return { ok: false, msg: 'Turn the disturbance observer off first.' };
            const e = errBeforeSwitch(ctx);
            return { ok: e > 0.1, msg: `Error before the switch: ${fmt(e, 3)}°.` };
          },
        },
        {
          id: 'b0', title: '(b) Observability of the augmented model',
          inputs: { rank: 'rank 𝒪<sub>A₂,C₂</sub>' },
          check: (v) => { const { A, B, C } = lib().ss(ctx.pModel); const { A2, C2 } = augD(A, B, C); return PD().checkNumbers(v, { rank: L.rank(L.obsv(A2, C2)) }, {}); },
          solution: () => [{ tex: '\\operatorname{rank}\\mathcal{O}_{A_2,C_2} = 5' }, { html: 'The extra state d enters through B, which the outputs see after one integration, so it is observable.' }],
        },
        {
          id: 'b1', title: '(b) Estimator bias removed',
          html: 'Passes when the observer\'s steady-state error |φ − φ̂| at t<sub>end</sub> is under 0.05° and |φ<sub>r</sub> − φ| before the switch is under 0.1°.',
          check: () => {
            const res = ctx.app.result(), n = res.t.length - 1;
            if (!ctx.st.dobs) return { ok: false, msg: 'Turn the disturbance observer on.' };
            const bias = Math.abs(res.yAll[1][n] - res.extras.phHat[n]) * R2D, e = errBeforeSwitch(ctx);
            return { ok: bias < 0.05 && e < 0.1, msg: `d̂ = ${fmt(res.extras.dhat[n], 3)} vs d = ${fmt(ctx.S.sim.dist, 3)}; |φ − φ̂| = ${fmt(bias, 3)}°; error before the switch ${fmt(e, 3)}°.` };
          },
          solution: () => [{ html: 'The repo uses the C.13 observer poles plus p<sub>d</sub> = −10 (Listing 14.6, p. 258). d̂ also absorbs the torque the model gets wrong because of the parameter mismatch, so it settles near d but not exactly on it.' }],
        },
      ]);
    },
  });

  WB.studies.C.ss = { makeSS, design, obsDefaults, augI, augD };
})();
