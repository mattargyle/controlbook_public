// Design Study B, Chapters 11–14: full state feedback, integral augmentation,
// observers, and disturbance observers on the linearized pendulum (Eq. 6.17):
//   ẋ = A x + B F,  y = (z, θ) = C x,  x = (z, θ, ż, θ̇).
// The closed-loop poles are a θ pair (t_r,θ, ζ_θ) and a slower z pair
// (t_r,z = M t_r,θ, ζ_z), plus p_I and the observer poles in later chapters.
// Gains are placed with the port of scipy's place_poles (core/place_yt.js), so the
// two-output observer gains match the repo's ctrlObserver.py exactly.
(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const B = WB.studies.B;
  const lib = () => B.lib;
  const PD = () => WB.pd;
  const DEG = Math.PI / 180;

  const KEYS = ['K1', 'K2', 'K3', 'K4'];
  const tuning = (st) => ({ trTh: st.trTh, zetaTh: st.zetaTh, M: st.M, zetaZ: st.zetaZ, rule: st.rule, pI: st.pI, obsFactor: st.obsFactor, obsRule: st.obsRule, pD: st.pD });

  // Gains: designed from the knobs (explore), or K/k_r/k_I from the sliders with
  // the observer still designed from its knobs (work). Two-output observer gains
  // are not unique, so the work mode does not ask for them entry by entry.
  function gainsFor(ctx, level) {
    const d = lib().ssDesign(ctx.ss, level, tuning(ctx.st));
    if (ctx.S.mode === 'explore') return d;
    const w = ctx.st.w;
    return { ...d, K: KEYS.map((k) => w[k]), kr: w.kr, ki: w.ki };
  }

  function makeCtrl(ctx, level, { linear = false } = {}) {
    const g = ctx.gains, st = ctx.st, p = ctx.pModel, Ts = ctx.S.sim.Ts, uLim = ctx.sys.uLimit(p);
    const { A, B: Bm, C } = ctx.ss;
    if (level === 'sf') return lib().sfCtrl({ K: g.K, kr: g.kr, uLim, linear });
    if (level === 'sfi') return lib().sfiCtrl({ K: g.K, ki: g.ki, uLim, Ts, antiwindup: st.antiwindup, linear });
    const xh0 = [st.xhat0.z, st.xhat0.th * DEG, 0, 0];
    if (level === 'dobs' && st.dobs) {
      const { A2, C2, B2: B1 } = lib().augD(ctx.ss);
      return lib().obsCtrl({ Aobs: A2, Bobs: B1, Cobs: C2, L: g.L2, K: g.K, ki: g.ki, Ts, uLim, dist: true, xhat0: [...xh0, 0], antiwindup: st.antiwindup, linear });
    }
    return lib().obsCtrl({ Aobs: A, Bobs: Bm, Cobs: C, L: g.L, K: g.K, ki: g.ki, Ts, uLim, xhat0: xh0, antiwindup: st.antiwindup, linear });
  }

  // ------------------------------------------------------------- analysis --
  function clPoles(ctx, level) {
    const { A, B: Bm } = ctx.ss, g = ctx.gains;
    if (level === 'sf') return L.eig(L.sub(A, L.mul(Bm, [g.K])));
    const { A1, B1 } = lib().augI(ctx.ss);
    return L.eig(L.sub(A1, L.mul(B1, [[...g.K, g.ki]])));
  }
  function obsPolesOf(ctx, level) {
    const { A, C } = ctx.ss, g = ctx.gains;
    if (level === 'obs' || (level === 'dobs' && !ctx.st.dobs)) return L.eig(L.sub(A, L.mul(g.L, C)));
    if (level === 'dobs') { const { A2, C2 } = lib().augD(ctx.ss); return L.eig(L.sub(A2, L.mul(g.L2, C2))); }
    return [];
  }
  // Group a designed pole list into drag ids: θ pair 0, z pair 1, p_I 2 (observer +10).
  function tagged(d, off) {
    const out = [];
    const n = d.poles.length;
    d.poles.forEach((p, i) => out.push({ ...p, dragId: off + (i < 2 ? 1 : i < 4 ? 0 : 2) }));
    return n ? out : [];
  }

  function markers(ctx, level, prob) {
    const { A } = ctx.ss, g = ctx.gains, explore = ctx.S.mode === 'explore';
    const mk = L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole ${i + 1}` }));
    const names = { 0: 'θ-pair pole', 1: 'z-pair pole', 2: 'integrator pole' };
    if (explore) {
      tagged({ poles: g.poles }, 0).forEach((p) => mk.push({ ...p, kind: 'cl', label: `controller ${names[p.dragId]}` }));
      if (level === 'obs' || level === 'dobs') {
        const op = ctx.st.dobs || level === 'obs' ? g.obsPoles : g.obsPoles.slice(0, 4);
        tagged({ poles: op }, 10).forEach((p) => mk.push({ ...p, kind: 'obs', label: `observer ${p.dragId === 12 ? 'disturbance pole' : names[p.dragId - 10]}`, noFit: Math.hypot(p.re, p.im) > 80 }));
      }
    } else {
      clPoles(ctx, level).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole ${i + 1}` }));
      // Work mode: the target poles (B.11a) and the observer poles (B.13c) are answers
      if (!polesRevealed(ctx)) return mk;
      obsPolesOf(ctx, level).forEach((p, i) => mk.push({ ...p, kind: 'obs', label: `observer pole ${i + 1}`, noFit: Math.hypot(p.re, p.im) > 80 }));
      const t = lib().ssPoles({ ...prob, pI: level === 'sf' ? null : prob.pI });
      t.poles.forEach((p) => mk.push({ ...p, kind: 'target', label: 'target pole (problem)' }));
    }
    return mk;
  }

  const polesRevealed = (ctx) => !!(ctx.app && ctx.app.isRevealed(`B:${ctx.S.chapter}:poles`));
  function revealPoles(parent, ctx, what) {
    const btn = el('button', { type: 'button', class: 'btn btn-quiet', text: `Reveal ${what} in the s-plane`, onclick: () => { ctx.app.reveal(`B:${ctx.S.chapter}:poles`); ctx.update(); } });
    parent.append(btn);
    WB.ui.addRefresher(() => { btn.hidden = polesRevealed(ctx); });
  }

  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    const wn = Math.hypot(re, im);
    const zeta = Math.max(0.2, Math.min(0.99, -re / wn));
    const trFrom = WB.design.trFromWn;
    if (id === 0) { st.zetaTh = zeta; st.trTh = Math.max(0.02, trFrom(wn, zeta, st.rule)); }
    else if (id === 1) { st.zetaZ = zeta; st.M = Math.max(1.2, Math.min(50, trFrom(wn, zeta, st.rule) / st.trTh)); }
    else if (id === 2) st.pI = re;
    else if (id === 10 || id === 11) {
      const base = id === 10 ? WB.design.wnFromTr(st.trTh, st.zetaTh, st.obsRule) : WB.design.wnFromTr(st.trTh * st.M, st.zetaZ, st.obsRule);
      st.obsFactor = Math.max(1, Math.min(60, wn / base));
    } else if (id === 12) st.pD = re;
    ctx.update();
  }

  // ------------------------------------------------------------- controls --
  function workSliders(parent, ctx, keys) {
    const spec = { K1: ['K<sub>1</sub> (z)', -40, 10], K2: ['K<sub>2</sub> (θ)', -150, 10], K3: ['K<sub>3</sub> (ż)', -40, 10], K4: ['K<sub>4</sub> (θ̇)', -40, 10], kr: ['k<sub>r</sub>', -40, 10], ki: ['k<sub>I</sub>', -20, 30] };
    for (const k of keys) {
      const [label, min, max] = spec[k];
      slider(parent, { label, min, max, step: (max - min) / 4000, sig: 4, get: () => ctx.st.w[k], set: (v) => { ctx.st.w[k] = v; ctx.update(); } });
    }
  }
  function knobs(parent, ctx, { pI = false } = {}) {
    slider(parent, { label: 't<sub>r,θ</sub>', unit: 's', min: 0.05, max: 2, step: 0.005, get: () => ctx.st.trTh, set: (v) => { ctx.st.trTh = v; ctx.update(); } });
    slider(parent, { label: 'ζ<sub>θ</sub>', min: 0.2, max: 0.99, step: 0.005, get: () => ctx.st.zetaTh, set: (v) => { ctx.st.zetaTh = v; ctx.update(); } });
    slider(parent, { label: 'M = t<sub>r,z</sub>/t<sub>r,θ</sub>', min: 1.2, max: 20, step: 0.1, sig: 3, get: () => ctx.st.M, set: (v) => { ctx.st.M = v; ctx.update(); } });
    slider(parent, { label: 'ζ<sub>z</sub>', min: 0.2, max: 0.99, step: 0.005, get: () => ctx.st.zetaZ, set: (v) => { ctx.st.zetaZ = v; ctx.update(); } });
    segmented(parent, {
      label: 'ω<sub>n</sub> from t<sub>r</sub>',
      options: [{ value: '2.2', label: '2.2 / t<sub>r</sub> (B.8)' }, { value: 'tp', label: 'π/(2t<sub>r</sub>√(1−ζ²)) (listings)' }],
      get: () => ctx.st.rule, set: (v) => { ctx.st.rule = v; ctx.update(); },
    });
    if (pI) slider(parent, { label: 'p<sub>I</sub>', min: -15, max: -0.05, step: 0.01, sig: 3, get: () => ctx.st.pI, set: (v) => { ctx.st.pI = v; ctx.update(); } });
  }
  function presets(parent, ctx, prob) {
    const row = el('div', { class: 'btn-row' });
    if (prob.book) row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Book text (B.8 poles)', onclick: () => { Object.assign(ctx.st, prob.book); ctx.update(); } }));
    if (prob.repo) row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Repo listing', onclick: () => { Object.assign(ctx.st, prob.repo); ctx.update(); } }));
    parent.append(row);
  }
  function obsKnobs(parent, ctx, withD) {
    slider(parent, { label: 'speed factor', unit: '×', min: 1, max: 40, step: 0.1, sig: 3, hint: 't_r,obs = t_r / factor for each pair (repo: 10)', get: () => ctx.st.obsFactor, set: (v) => { ctx.st.obsFactor = v; ctx.update(); } });
    segmented(parent, {
      label: 'observer ω<sub>n</sub> rule',
      options: [{ value: '2.2', label: '2.2 / t<sub>r</sub>' }, { value: 'tp', label: 'π/(2t<sub>r</sub>√(1−ζ²))' }],
      get: () => ctx.st.obsRule, set: (v) => { ctx.st.obsRule = v; ctx.update(); },
    });
    if (withD) slider(parent, { label: 'p<sub>d</sub>', min: -30, max: -0.05, step: 0.01, sig: 3, hint: 'disturbance-estimate pole (repo: −1)', get: () => ctx.st.pD, set: (v) => { ctx.st.pD = v; ctx.update(); } });
  }
  function xhatKnobs(parent, ctx) {
    slider(parent, { label: 'ẑ(0)', unit: 'm', min: -1, max: 1, step: 0.01, sig: 3, get: () => ctx.st.xhat0.z, set: (v) => { ctx.st.xhat0.z = v; ctx.update(); } });
    slider(parent, { label: 'θ̂(0)', unit: '°', min: -20, max: 20, step: 0.5, sig: 3, hint: 'initial estimate (the true state starts at the left-panel values)', get: () => ctx.st.xhat0.th, set: (v) => { ctx.st.xhat0.th = v; ctx.update(); } });
  }
  function readout(parent, ctx, keys) {
    const box = el('div', { class: 'readout wrap' });
    parent.append(box);
    WB.ui.addRefresher(() => {
      const g = ctx.gains;
      const vals = { K1: g.K[0], K2: g.K[1], K3: g.K[2], K4: g.K[3], kr: g.kr, ki: g.ki };
      box.replaceChildren(...keys.map((k) => el('div', {}, el('span', { class: 'ro-label', text: k }), el('strong', { text: fmt(vals[k], 4) }))));
    });
  }
  function useGains(ctx, keys) {
    return {
      label: 'Use my gains',
      run: (v) => {
        const vals = keys.map((k) => PD().num(v[k]));
        if (vals.some((x) => x === null)) return { ok: false, msg: `Enter ${keys.join(', ')} first.` };
        ctx.app.setMode('work');
        keys.forEach((k, i) => { ctx.st.w[k] = vals[i]; });
        ctx.update();
        return null;
      },
    };
  }

  // ----------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B: Bm } = ctx.ss;
    return {
      title: 'Linearized state-space model', page: 'p. 90 · Eq. 6.17, p. 189 · Eq. 11.39',
      theory: '\\dot x = Ax + BF,\\quad y = \\begin{pmatrix}1&0&0&0\\\\0&1&0&0\\end{pmatrix}x,\\quad x = (z, \\theta, \\dot z, \\dot\\theta)^\\top,\\quad y_r = (1, 0, 0, 0)\\,x',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(Bm)}`, spoiler: true,
      note: 'Eq. 11.39 and 12.2 print row 4 for a different ℓ (ISSUES.md). The numbers here use the current ℓ.',
    };
  }
  const ctrbCard = (A, Bm, title, page) => WB.ss.ctrbCard(A, Bm, title, page, { det: true });
  function polesCard(ctx, d, level) {
    const st = ctx.st;
    return {
      title: 'Desired closed-loop poles', page: 'p. 189 · B.11(a), p. 207–208',
      theory: (st.rule === 'tp' ? '\\omega_n = \\frac{\\pi}{2t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}') + ',\\quad t_{r,z} = M\\,t_{r,\\theta},\\quad \\Delta^d_{cl} = (s^2 + 2\\zeta_\\theta\\omega_{n\\theta}s + \\omega_{n\\theta}^2)(s^2 + 2\\zeta_z\\omega_{nz}s + \\omega_{nz}^2)' + (level === 'sf' ? '' : '(s - p_I)'),
      numbers: `\\omega_{n\\theta} = ${tex(d.wnTh)},\\; \\omega_{nz} = ${tex(d.wnZ)},\\quad \\Delta^d_{cl} = ${WB.tf.polyTex(L.polyFromRoots(d.poles))}`, spoiler: true,
    };
  }

  // --------------------------------------------------------- chapter base --
  function base(level, num, title, pages, extra) {
    return Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages, level,
      gains(ctx) { return gainsFor(ctx, level); },
      controller(ctx, o) { return makeCtrl(ctx, level, o); },
      linearSim(ctx, c) {
        const { A, B: Bm, C } = ctx.ss;
        return WB.sim.simulate({ ...c, disturbance: null, noise: null, plant: WB.design.linearPlant(A, Bm, C), controller: makeCtrl(ctx, level, { linear: true }) });
      },
      splane(ctx) { return { markers: markers(ctx, level, this.prob(ctx)) }; },
      onPoleDrag: onDrag,
      targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.trTh * ctx.st.M } : {}; },
      prob(ctx) { const pr = ctx.sys.problems[`ch${num}`]; return pr.book || pr.repo; },
    }, extra);
  }
  const stateDefaults = (p) => ({ trTh: p.trTh, zetaTh: p.zetaTh, M: p.M, zetaZ: p.zetaZ, rule: p.rule, pI: p.pI ?? -2, obsFactor: p.obsFactor ?? 10, obsRule: p.obsRule ?? 'tp', pD: p.pD ?? -1, xhat0: { z: 0, th: 0 } });

  function estSeries(ctx, res, sc, oi) {
    const key = oi === 0 ? 'zhat' : 'thhat';
    return [{ label: `${oi === 0 ? 'ẑ' : 'θ̂'} (observer)`, y: sc(res.extras[key] || []), color: '--series-3', dash: [3, 3], width: 2 }];
  }
  function estExtra(ctx, res) {
    const k = 180 / Math.PI, S = ctx.S;
    if (ctx.st.extra === 'd') {
      return { opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N]', unit: 'N' }, data: { series: [
        { label: 'd̂ (estimate)', y: Array.from(res.extras.dhat || []), color: '--series-3', width: 2 },
        { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
      ] } };
    }
    if (ctx.st.extra === 'zd') {
      return { opts: { title: 'ż and its estimate', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [
        { label: 'ż̂ (observer)', y: Array.from(res.extras.zdhat || []), color: '--series-3', dash: [3, 3], width: 2 },
        { label: 'true ż', y: res.x.map((x) => x[2]), color: '--series-1' },
      ] } };
    }
    return { opts: { title: 'θ̇ and its estimate', yLabel: 'θ̇ [°/s]', unit: '°/s' }, data: { series: [
      { label: 'θ̇̂ (observer)', y: Array.from(res.extras.thdhat || [], (v) => v * k), color: '--series-3', dash: [3, 3], width: 2 },
      { label: 'true θ̇', y: res.x.map((x) => x[3] * k), color: '--series-1' },
    ] } };
  }
  // error just before the second reference switch
  function errBeforeSwitch(ctx) {
    const res = ctx.app.result(), S = ctx.S;
    const tSw = S.sim.type === 'square' ? S.sim.tStep + 1 / S.sim.frequency : S.sim.tEnd;
    const i = Math.min(res.t.length - 1, Math.round((tSw - 0.05) / S.sim.Ts));
    return { e: Math.abs(res.rAll[0][i] - res.yAll[0][i]), t: res.t[i] };
  }

  // Work-mode starting gains (not the answers): slow, stable designs
  // (t_r,θ = 0.8 s, M = 4, ζ = 0.8, 2.2/t_r, p_I = −1), rounded to two digits.
  const W11 = { K1: -0.26, K2: -22, K3: -0.8, K4: -4.4, kr: -0.26, ki: 0 };
  const W12 = { K1: -1, K2: -26, K3: -1.7, K4: -5.7, kr: 0, ki: 0.26 };   // a slow, stable design

  // ------------------------------------------------------------ Chapter 11 --
  B.chapters.ch11 = base('sf', 11, 'Full state feedback', 'pp. 173–196', {
    defaults(sys) { return { ...stateDefaults(sys.problems.ch11.book), extra: 'thd', w: { ...W11 } }; },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'F = −Kx + k_r z_r', 'p. 175 · Eq. 11.3, p. 190');
      if (ctx.S.mode === 'work') {
        workSliders(sec, ctx, [...KEYS, 'kr']);
        sec.append(el('p', { class: 'muted small', text: 'Starting values are a slow, stable design, not the answer.' }));
        revealPoles(sec, ctx, 'the B.11(a) target poles');
      } else {
        knobs(sec, ctx); presets(sec, ctx, ctx.sys.problems.ch11);
        sec.append(el('p', { class: 'muted small', text: 'Drag a θ-pair (fast) or z-pair (slow) pole.' }));
        readout(sec, ctx, [...KEYS, 'kr']);
      }
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'zd', label: 'ż' }, { value: 'thd', label: 'θ̇' }], get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); } });
    },
    extraPlot(ctx, res) {
      const zd = ctx.st.extra === 'zd';
      return { opts: { title: zd ? 'ż(t)' : 'θ̇(t)', yLabel: zd ? 'ż [m/s]' : 'θ̇ [°/s]', unit: zd ? 'm/s' : '°/s' }, data: { series: [{ label: zd ? 'ż' : 'θ̇', y: res.x.map((x) => (zd ? x[2] : x[3] * 180 / Math.PI)), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = lib().ssDesign(ctx.ss, 'sf', tuning(ctx.st)), g = ctx.gains;
      const { A, B: Bm } = ctx.ss;
      return [
        ssCard(ctx),
        ctrbCard(A, Bm, 'Controllability', 'p. 190 · Step 1'),
        { title: 'Open-loop characteristic polynomial', page: 'p. 190 · Step 2',
          theory: '\\Delta_{ol}(s) = \\det(sI - A),\\quad \\mathbf a_A = (a_{n-1}, \\dots, a_0)',
          numbers: `\\Delta_{ol} = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
        polesCard(ctx, d, 'sf'),
        { title: 'Gains (Ackermann / place)', page: 'p. 190 · Step 4, Eq. 11.32',
          theory: 'K = (\\boldsymbol\\alpha - \\mathbf a_A)\\,\\mathcal{A}_A^{-1}\\mathcal{C}_{A,B}^{-1},\\quad k_r = \\frac{-1}{C_r(A - BK)^{-1}B}',
          numbers: `K = ${texMat([d.K])},\\quad k_r = ${tex(d.kr)}`, spoiler: true },
        { title: 'Control law', page: 'p. 191 · Listing 11.2',
          theory: 'F = -Kx + k_r z_r',
          numbers: `F = -(${g.K.map((k, i) => `${tex(k)}\\,${['z', '\\theta', '\\dot z', '\\dot\\theta'][i]}`).join(' + ')}) + ${tex(g.kr)}\\,z_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch11;
      const ref = () => lib().ssDesign(ctx.ss, 'sf', { ...pr.book });
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Natural frequencies with B.8\'s ζ = 0.707, t<sub>r,θ</sub> = 0.5 s and t<sub>z</sub> = 3t<sub>θ</sub>',
          inputs: { wt: 'ω<sub>nθ</sub>', wz: 'ω<sub>nz</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { wt: r.wnTh, wz: r.wnZ }, { wt: 'ωnθ', wz: 'ωnz' }); },
          solution: () => { const r = ref(); return [{ tex: `\\omega_{n\\theta} = 2.2/0.5 = ${tex(r.wnTh)},\\; \\omega_{nz} = 2.2/1.5 = ${tex(r.wnZ)},\\quad p = ${r.poles.map((x) => texPole(x)).join(',\\;')}` }, { html: 'These give the book\'s Δ<sup>d</sup> = s⁴ + 8.2955s³ + 34.414s² + 53.533s + 41.646 (p. 190). The listing instead uses ζ = 0.9, t<sub>z</sub> = 5t<sub>θ</sub> and the π/(2t<sub>r</sub>√(1−ζ²)) rule.' }]; },
        },
        {
          id: 'c', title: '(c) Controllability',
          inputs: { rank: 'rank 𝒞', det: 'det 𝒞' },
          check: (v) => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return PD().checkNumbers(v, { rank: L.rank(Cm), det: L.det(Cm) }, { det: 'det' }); },
          solution: () => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return [{ tex: `\\mathcal{C}_{A,B} = ${texMat(Cm)},\\; \\det = ${tex(L.det(Cm))} \\ne 0` }, { html: 'The book prints det = −6104.1 (p. 190): that is the ℓ = 0.5 m value.' }]; },
        },
        {
          id: 'd', title: '(d) K and k<sub>r</sub> for the (a) poles',
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], K3: r.K[2], K4: r.K[3], kr: r.kr }, {}); },
          actions: [useGains(ctx, [...KEYS, 'kr'])],
          solution: () => { const r = ref(); return [{ tex: `K = ${texMat([r.K])},\\quad k_r = ${tex(r.kr)}` }, { html: 'The book prints K = (−1.5050, −24.9399, −1.9847, −3.5829), k<sub>r</sub> = −1.5050 (p. 190). Those are exactly the gains for ℓ = 0.5 m; with the stated ℓ = 1 m the answer is the one above. Note k<sub>r</sub> = K<sub>1</sub>: the plant has a free integrator in z, so unity DC gain needs the reference to enter like the z feedback.' }]; },
        },
        {
          id: 'e', title: '(e) A much faster response',
          html: 'Passes when the first z step rises (10–90%) in under 1.2 s, |θ| stays under 30°, and the demanded F never exceeds F<sub>max</sub> over the whole run (the later ±0.5 m switches are 1 m steps). B.8\'s successive-loop design takes about 3.9 s.',
          check: () => {
            const res = ctx.app.result(), S = ctx.S;
            const i0 = Math.round(S.sim.tStep / S.sim.Ts), i1 = S.sim.type === 'square' ? Math.round((S.sim.tStep + 0.5 / S.sim.frequency) / S.sim.Ts) : res.t.length;
            const m = M.stepMetrics(res.t, res.yAll[0], i0, Math.min(i1, res.t.length), res.yAll[0][i0], res.rAll[0][Math.min(i0 + 1, res.t.length - 1)]);
            let maxTh = 0, satd = false;
            const lim = ctx.sys.uLimit(ctx.pModel);
            for (let k = 0; k < res.t.length; k++) { maxTh = Math.max(maxTh, Math.abs(res.x[k][1])); if (Math.abs(res.uDemandAll[0][k]) > lim + 1e-9) satd = true; }
            const ok = m.tr < 1.2 && maxTh < 30 * DEG && !satd;
            return { ok, msg: `rise time ${fmt(m.tr, 3)} s, max |θ| ${fmt(maxTh / DEG, 3)}°, ${satd ? 'saturates' : 'no saturation'}.` };
          },
          solution: () => [{ html: 'One option (Explore mode, 2.2/t<sub>r</sub> rule): t<sub>r,θ</sub> = 0.5 s, M = 2.5, ζ = 0.707 rises in 1.02 s, and its largest demand, on the 1 m steps, is 4.3 N. Faster designs saturate there: t<sub>r,θ</sub> = 0.4 s, M = 3 rises in 0.95 s and needs only 3.7 N on the first 0.5 m step, but demands 7.4 N at t = 12.5 s.' }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 12 --
  B.chapters.ch12 = base('sfi', 12, 'Integrator with state feedback', 'pp. 197–214', {
    defaults(sys) { return { ...stateDefaults(sys.problems.ch12.book), antiwindup: 'clamp', extra: 'int', w: { ...W12 } }; },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'F = −Kx − k_I ∫(z_r − z)', 'p. 199 · Eq. 12.1, p. 210');
      segmented(sec, {
        label: 'Anti-windup (B.12a)',
        options: [{ value: 'clamp', label: 'hold integrator while F saturates' }, { value: 'none', label: 'none (listing)' }],
        get: () => ctx.st.antiwindup, set: (v) => { ctx.st.antiwindup = v; ctx.update(); },
      });
      if (ctx.S.mode === 'work') {
        workSliders(sec, ctx, [...KEYS, 'ki']);
        sec.append(el('p', { class: 'muted small', text: 'Starting values are a slow, stable design, not the answer.' }));
        revealPoles(sec, ctx, 'the target poles (p_I = −2)');
      } else {
        knobs(sec, ctx, { pI: true }); presets(sec, ctx, ctx.sys.problems.ch12);
        readout(sec, ctx, [...KEYS, 'ki']);
      }
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'int', label: 'integrator' }, { value: 'thd', label: 'θ̇' }], get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); } });
    },
    extraPlot(ctx, res) {
      if (ctx.st.extra === 'int') return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [m·s]', unit: 'm·s' }, data: { series: [{ label: 'x_I = ∫(z_r − z)', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
      return { opts: { title: 'θ̇(t)', yLabel: 'θ̇ [°/s]', unit: '°/s' }, data: { series: [{ label: 'θ̇', y: res.x.map((x) => x[3] * 180 / Math.PI), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = lib().ssDesign(ctx.ss, 'sfi', tuning(ctx.st));
      const { A1, B1 } = lib().augI(ctx.ss);
      return [
        ssCard(ctx),
        { title: 'Augmented system', page: 'p. 207 · Step 1',
          theory: '\\dot x_I = z_r - C_r x,\\quad A_1 = \\begin{pmatrix}A & 0\\\\ -C_r & 0\\end{pmatrix},\\quad B_1 = \\begin{pmatrix}B\\\\ 0\\end{pmatrix},\\quad C_r = (1, 0, 0, 0)',
          numbers: `A_1 = ${texMat(A1)},\\quad B_1 = ${texMat(B1)}`, spoiler: true },
        ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 207 · Step 2'),
        polesCard(ctx, d, 'sfi'),
        { title: 'Gains', page: 'p. 208 · Step 3',
          theory: 'K_1 = (K, k_I) = (\\boldsymbol\\alpha - \\mathbf a_{A_1})\\mathcal{A}_{A_1}^{-1}\\mathcal{C}_{A_1,B_1}^{-1},\\quad F = -Kx - k_I x_I',
          numbers: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch12;
      const ref = () => lib().ssDesign(ctx.ss, 'sfi', { ...pr.book });
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) K and k<sub>I</sub> for the B.11 poles with p<sub>I</sub> = −2',
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', ki: 'k<sub>I</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], K3: r.K[2], K4: r.K[3], ki: r.ki }, { ki: 'kI' }); },
          actions: [useGains(ctx, [...KEYS, 'ki'])],
          solution: () => { const r = ref(); return [{ tex: `K = ${texMat([r.K])},\\quad k_I = ${tex(r.ki)}` }, { html: 'The book prints K = (−5.3744, −32.1057, −4.5745, −5.1545), k<sub>I</sub> = 3.0101 (p. 208): the ℓ = 0.5 m gains. Its Step 2 text names p<sub>I</sub> = −10 and different B.11 poles, but the polynomial it then uses has (s + 2) and the B.8 poles.' }]; },
        },
        {
          id: 'c', title: '(b, c) Tracking with d = 0.5 N and 20% uncertainty',
          html: 'Passes when |z − r| just before the second reference switch is under 1 cm, with the current disturbance and mismatch.',
          check: () => { const { e, t } = errBeforeSwitch(ctx); return { ok: e < 0.01, msg: `|z − r| = ${fmt(e * 100, 3)} cm at t = ${fmt(t, 4)} s (d = ${fmt(ctx.S.sim.dist, 3)} N).` }; },
          solution: () => [{ html: 'The integrator removes the error from the constant disturbance and the parameter mismatch. Faster p<sub>I</sub> removes it sooner but couples into the θ loop; −2 is a reasonable start.' }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 13 --
  B.chapters.ch13 = base('obs', 13, 'Observers', 'pp. 215–238', {
    defaults(sys) { return { ...stateDefaults(sys.problems.ch13.repo), antiwindup: 'none', extra: 'thd', w: { ...W12 } }; },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries: estSeries,
    extraPlot: estExtra,
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂)', 'p. 231 · Listing 13.2');
      if (ctx.S.mode === 'work') { workSliders(sec, ctx, [...KEYS, 'ki']); revealPoles(sec, ctx, 'the target and observer poles'); }
      else { knobs(sec, ctx, { pI: true }); presets(sec, ctx, ctx.sys.problems.ch13); }
      const ob = section(parent, 'Observer', 'p. 216 · Eq. 13.3');
      obsKnobs(ob, ctx, false);
      ob.append(el('p', { class: 'muted small', text: 'Two outputs make L (4 × 2) non-unique, so it is designed from these knobs in both modes (scipy\'s YT algorithm, as control.place uses).' }));
      if (ctx.S.mode === 'explore') readout(ob, ctx, [...KEYS, 'ki']);
      xhatKnobs(ob, ctx);
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'zd', label: 'ż, ż̂' }, { value: 'thd', label: 'θ̇, θ̇̂' }], get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); } });
    },
    math(ctx) {
      const g = ctx.gains, d = lib().ssDesign(ctx.ss, 'obs', tuning(ctx.st));
      const { A, C } = ctx.ss;
      const O = L.obsv(A, C);
      return [
        ssCard(ctx),
        { title: 'Observability', page: 'p. 220–221, B.13(b)',
          theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ CA^2 \\\\ CA^3\\end{bmatrix} \\in \\mathbb{R}^{8\\times4},\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = 4',
          numbers: `\\operatorname{rank}\\mathcal{O}_{A,C} = ${L.rank(O)}`, spoiler: true },
        { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 232',
          theory: '\\dot{\\hat x} = A\\hat x + BF + L(y - C\\hat x),\\quad \\dot e = (A - LC)e,\\quad F \\text{ is the saturated force of the previous sample}' },
        { title: 'Observer poles and gain', page: 'p. 232 · Listing 13.2',
          theory: 't_{r,obs} = t_r / 10 \\text{ for each pair (repo)},\\quad L = \\text{place}(A^\\top, C^\\top, q)^\\top',
          numbers: `\\omega_{n\\theta,obs} = ${tex(g.wnThObs)},\\; \\omega_{nz,obs} = ${tex(g.wnZObs)},\\quad L^\\top = ${texMat(L.T(g.L))}`, spoiler: true,
          note: 'With two outputs many L place the same poles; this is the one scipy\'s YT iteration returns, so the page matches ctrlObserver.py.' },
        polesCard(ctx, d, 'sfi'),
        { title: 'Separation principle', page: 'p. 222–223',
          theory: '\\begin{pmatrix}\\dot x\\\\ \\dot e\\end{pmatrix} = \\begin{pmatrix}A - BK & BK\\\\ 0 & A - LC\\end{pmatrix}\\begin{pmatrix}x\\\\ e\\end{pmatrix} \\Rightarrow \\text{eig} = \\text{eig}(A - BK)\\cup\\text{eig}(A - LC)',
          note: 'Holds for the linear model only; saturation and mismatch break it.' },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch13;
      const ref = () => lib().ssDesign(ctx.ss, 'obs', { ...pr.repo });
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'b', title: '(b) Observability',
          inputs: { rank: 'rank 𝒪<sub>A,C</sub>' },
          check: (v) => PD().checkNumbers(v, { rank: L.rank(L.obsv(ctx.ss.A, ctx.ss.C)) }, {}),
          solution: () => [{ tex: '\\operatorname{rank}\\mathcal{O}_{A,C} = 4 = n' }, { html: 'Measuring z and θ makes every state observable.' }],
        },
        {
          id: 'c', title: '(c) Observer natural frequencies for the repo tuning (t<sub>r,obs</sub> = t<sub>r</sub>/10, π/(2t<sub>r</sub>√(1−ζ²)), ζ = 0.9)',
          inputs: { wt: 'ω<sub>nθ,obs</sub>', wz: 'ω<sub>nz,obs</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { wt: r.wnThObs, wz: r.wnZObs }, { wt: 'ωnθ,obs', wz: 'ωnz,obs' }); },
          solution: () => { const r = ref(); return [{ tex: `\\omega_{n\\theta,obs} = ${tex(r.wnThObs)},\\; \\omega_{nz,obs} = ${tex(r.wnZObs)},\\quad L^\\top = ${texMat(L.T(r.L))}` }, { html: 'L is the scipy/control.place result printed by ctrlObserver.py. Any L with these eigenvalues of A − LC is a valid answer.' }]; },
        },
        {
          id: 'e', title: '(e) Add d = 0.05 N',
          html: 'Set the disturbance to 0.05 N in the left panel. The integrator acts on r − ẑ, and ẑ is biased by the unmodeled d, so z settles away from r. Chapter 14 removes the bias.',
          check: () => {
            const res = ctx.app.result();
            if (!(Math.abs(ctx.S.sim.dist) > 0)) return { ok: false, msg: 'Set d ≠ 0 first.' };
            const n = res.t.length - 1;
            return { ok: true, msg: `At t_end: z − ẑ = ${fmt((res.yAll[0][n] - res.extras.zhat[n]) * 100, 3)} cm, θ − θ̂ = ${fmt((res.yAll[1][n] - res.extras.thhat[n]) / DEG, 3)}°.` };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 14 --
  B.chapters.ch14 = base('dobs', 14, 'Disturbance observers', 'pp. 239–259', {
    defaults(sys) { return { ...stateDefaults(sys.problems.ch14.repo), antiwindup: 'none', dobs: true, extra: 'd', w: { ...W12 } }; },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries: estSeries,
    extraPlot: estExtra,
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'p. 250 · Listing 14.4');
      segmented(sec, {
        label: 'Disturbance observer',
        options: [{ value: true, label: 'on' }, { value: false, label: 'off (B.14a)' }],
        get: () => ctx.st.dobs, set: (v) => { ctx.st.dobs = v; ctx.update(); },
      });
      if (ctx.S.mode === 'work') { workSliders(sec, ctx, [...KEYS, 'ki']); revealPoles(sec, ctx, 'the target and observer poles'); }
      else { knobs(sec, ctx, { pI: true }); presets(sec, ctx, ctx.sys.problems.ch14); }
      const ob = section(parent, 'Observer', 'p. 241, p. 252–253');
      obsKnobs(ob, ctx, true);
      if (ctx.S.mode === 'explore') readout(ob, ctx, [...KEYS, 'ki']);
      xhatKnobs(ob, ctx);
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'd', label: 'd, d̂' }, { value: 'zd', label: 'ż, ż̂' }, { value: 'thd', label: 'θ̇, θ̇̂' }], get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); } });
    },
    math(ctx) {
      const g = ctx.gains;
      const { A2, C2 } = lib().augD(ctx.ss);
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(F + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0 \\text{ for constant } d' },
        { title: 'Augmented model (ḋ = 0)', page: 'p. 240, Listing 14.4',
          theory: 'A_2 = \\begin{pmatrix}A & B\\\\ 0 & 0\\end{pmatrix},\\quad C_2 = \\begin{pmatrix}C & 0\\end{pmatrix}',
          numbers: `A_2 = ${texMat(A2)},\\quad \\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}`, spoiler: true },
        { title: 'Disturbance observer', page: 'p. 241, p. 253',
          theory: '\\dot{\\hat x}_2 = A_2\\hat x_2 + B_1 F + L_2(y - C_2\\hat x_2),\\quad F = -K\\hat x - k_I x_I - \\hat d' },
        { title: 'Observer poles and gain', page: 'p. 252 · Listing 14.4',
          theory: '\\omega_{n,obs} = 2.2/(t_r/10) \\text{ for each pair},\\quad p_d = -1,\\quad L_2 = \\text{place}(A_2^\\top, C_2^\\top, q)^\\top',
          numbers: ctx.st.dobs ? `L_2^\\top = ${texMat(L.T(g.L2))}` : `L^\\top = ${texMat(L.T(g.L))}`, spoiler: true,
          note: 'Listing 14.4 switches the observer to the 2.2/t_r rule while the controller keeps π/(2t_r√(1−ζ²)), and its disturbance pole at −1 is slower than every controller pole.' },
        polesCard(ctx, g, 'sfi'),
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch14;
      const ref = () => lib().ssDesign(ctx.ss, 'dobs', { ...pr.repo });
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Without the disturbance observer',
          html: 'Switch the disturbance observer off. Reports the observer bias |z − ẑ| and the tracking error just before the second reference switch.',
          check: () => {
            const res = ctx.app.result(), { e, t } = errBeforeSwitch(ctx);
            const i = Math.min(res.t.length - 1, Math.round(t / ctx.S.sim.Ts));
            const bias = Math.abs(res.yAll[0][i] - res.extras.zhat[i]);
            if (ctx.st.dobs) return { ok: false, msg: 'Turn the disturbance observer off first.' };
            return { ok: true, msg: `|z − ẑ| = ${fmt(bias * 100, 3)} cm, |z − r| = ${fmt(e * 100, 3)} cm at t = ${fmt(t, 4)} s.` };
          },
        },
        {
          id: 'b1', title: '(b) Observer poles for the repo tuning',
          html: 'ω<sub>n</sub> of each observer pair (2.2/(t<sub>r</sub>/10), ζ = 0.9) and the disturbance pole.',
          inputs: { wt: 'ω<sub>nθ,obs</sub>', wz: 'ω<sub>nz,obs</sub>', pd: 'p<sub>d</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { wt: r.wnThObs, wz: r.wnZObs, pd: pr.repo.pD }, { wt: 'ωnθ,obs', wz: 'ωnz,obs', pd: 'pd' }); },
          solution: () => { const r = ref(); return [{ tex: `\\omega_{n\\theta,obs} = ${tex(r.wnThObs)},\\; \\omega_{nz,obs} = ${tex(r.wnZObs)},\\; p_d = ${pr.repo.pD},\\quad L_2^\\top = ${texMat(L.T(r.L2))}` }]; },
        },
        {
          id: 'b2', title: '(b) Estimator bias removed',
          html: 'Passes when |z − ẑ| at t<sub>end</sub> is under 2 mm with the disturbance observer on. Compare with it off.',
          check: () => {
            const res = ctx.app.result(), n = res.t.length - 1;
            if (!ctx.st.dobs) return { ok: false, msg: 'Turn the disturbance observer on.' };
            const bias = Math.abs(res.yAll[0][n] - res.extras.zhat[n]);
            return { ok: bias < 0.002, msg: `d̂ = ${fmt(res.extras.dhat[n], 3)} N vs d = ${fmt(ctx.S.sim.dist, 3)} N; |z − ẑ| = ${fmt(bias * 1000, 3)} mm.` };
          },
          solution: () => [{ html: 'd̂ estimates the input disturbance plus whatever force the model gets wrong (mismatch), so it settles near d but not exactly on it.' }],
        },
      ]);
    },
  });

  B.ss = { gainsFor, makeCtrl, clPoles, obsPolesOf };
})();
