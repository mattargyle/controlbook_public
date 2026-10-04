// Design Study B, Chapters 8–10 and App. P.6: successive loop closure. An inner
// PD loop balances the pendulum (F → θ) and an outer PD/PID loop moves the cart
// (θ_r → z). The outer design replaces the inner loop by its DC gain k_DC and adds
// a low-pass filter that cancels the left-half-plane zero of Z̃/Θ̃ (B.8, p. 126).
//
// Modes:
//   work    - you set the four (five) gains; the designed values stay hidden.
//   explore - gains come from t_r,θ, ζ_θ, the bandwidth separation M and ζ_z;
//             drag the inner (blue) or outer (green) poles.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const { tex, texPole, fmt, fmtPole } = M;
  const B = WB.studies.B;
  const lib = () => B.lib;
  const PD = () => WB.pd;
  const DEG = Math.PI / 180;

  // ------------------------------------------------------------ analysis --
  const qOf = (p) => Math.sqrt(3 * p.g / (2 * p.ell));
  const showsAnswer = (ctx, key) => ctx.S.mode === 'explore' || ctx.app.isSolved(key);

  // Python answers (WB.py.check): arguments drawn at random, with the signs the
  // pendulum's gains have (all negative).
  const ARGS = {
    s: { label: 's', complex: true, re: [-6, 3], im: [0.3, 12] },
    kPth: { label: 'kPθ', lo: -150, hi: -20 }, kDth: { label: 'kDθ', lo: -15, hi: -1 },
    kPz: { label: 'kPz', lo: -0.5, hi: -0.02 }, kDz: { label: 'kDz', lo: -0.8, hi: -0.05 }, kIz: { label: 'kIz', lo: -0.3, hi: -0.005 },
    kDC: { label: 'k_DC', lo: 1.05, hi: 3 },
  };
  const pyCheck = (ctx, spec) => (code) => WB.py.check(ctx, { args: ARGS, ...spec }, code);

  function innerPoles(ctx, g) {
    const pin = ctx.sys.inner(ctx.pModel);
    return M.roots2(pin.b0 * g.kDth, pin.a0 + pin.b0 * g.kPth);
  }
  // Outer-loop characteristic polynomial on the DC-gain model (Fig. 8-15 / 8-17).
  //   filter on : (1 + k_D)s² + (k_P − q k_D)s − q k_P   (times s, plus k_I(s − q), with an integrator)
  //   filter off: s² + k_DC (g − (2ℓ/3)s²)(k_D s + k_P)  (times s, plus k_I term)
  function outerPoly(ctx, g, filter, withI = false) {
    const p = ctx.pModel, q = qOf(p), kI = withI ? (g.kIz || 0) : 0;
    if (filter) {
      if (!withI || !kI) return [1 + g.kDz, g.kPz - q * g.kDz, -q * g.kPz];
      return [1 + g.kDz, g.kPz - q * g.kDz, kI - q * g.kPz, -q * kI];
    }
    const c = (2 * p.ell / 3) * g.kDC;
    if (!withI || !kI) return [-c * g.kDz, 1 - c * g.kPz, p.g * g.kDC * g.kDz, p.g * g.kDC * g.kPz];
    // −(2ℓ/3)k_D s⁴ + (1/k_DC − (2ℓ/3)k_P)s³ + (g k_D − (2ℓ/3)k_I)s² + g k_P s + g k_I, times k_DC
    return [-c * g.kDz, 1 - c * g.kPz, g.kDC * (p.g * g.kDz - (2 * p.ell / 3) * kI), p.g * g.kDC * g.kPz, p.g * g.kDC * kI];
  }
  const outerPoles = (ctx, g, filter, withI) => L.roots(outerPoly(ctx, g, filter, withI));
  // Natural frequency / damping of the dominant complex pair (or the slowest pair).
  function pairInfo(poles) {
    const c = poles.filter((p) => p.im > 1e-9);
    if (c.length) { const wn = Math.hypot(c[0].re, c[0].im); return { wn, zeta: -c[0].re / wn }; }
    const r = poles.map((p) => Math.abs(p.re)).sort((a, b) => a - b);
    return { wn: Math.sqrt(r[0] * (r[1] ?? r[0])), zeta: NaN };
  }

  // Eigenvalues of the full linearized closed loop (plant + filter [+ integrator]),
  // PD on the true state. A check that the two-loop design survives the coupling.
  function fullPoles(ctx, g, filter, withI) {
    const p = ctx.pModel, { A, B: Bm } = ctx.sys.stateSpace(p);
    const zf = lib().zcFilter(p, g.kDC, ctx.S.sim.Ts);
    const kI = withI ? (g.kIz || 0) : 0;
    const n = 4 + (filter ? 1 : 0) + (withI ? 1 : 0);
    const Acl = Array.from({ length: n }, () => new Array(n).fill(0));
    const iF = 4, iI = filter ? 5 : 4;
    // outer command u_o = k_Pz(r − z) − k_Dz ż + k_I x_I ; θ_r = filter state (or u_o)
    const uo = new Array(n).fill(0); uo[0] = -g.kPz; uo[2] = -g.kDz; if (withI) uo[iI] = kI;
    const thR = filter ? (() => { const v = new Array(n).fill(0); v[iF] = 1; return v; })() : uo;
    // F = k_Pθ(θ_r − θ) − k_Dθ θ̇
    const Fr = thR.map((v) => g.kPth * v); Fr[1] -= g.kPth; Fr[3] -= g.kDth;
    for (let i = 0; i < 4; i++) for (let j = 0; j < n; j++) Acl[i][j] = (j < 4 ? A[i][j] : 0) + Bm[i][0] * Fr[j];
    if (filter) { for (let j = 0; j < n; j++) Acl[iF][j] = zf.a * uo[j]; Acl[iF][iF] += -zf.b; }
    if (withI) Acl[iI][0] = -1;
    return L.eig(Acl);
  }

  // The successive-loop markers reuse the 'cl' / 'obs' kinds for the inner and
  // outer loops; the s-plane renames them through data.legendNames (plot.js).
  const LOOP_NAMES = { obs: 'outer-loop pole', cl: 'inner-loop pole' };

  // --------------------------------------------------------------- gains --
  function designed(ctx, st = ctx.st) {
    const g = lib().slcGains(ctx.pModel, { trTh: st.trTh, zetaTh: st.zetaTh, M: st.M, zetaZ: st.zetaZ, formula: st.formula || 'book' });
    g.kIz = st.kIz ?? 0;
    return g;
  }
  function gainsFor(ctx) {
    if (ctx.S.mode === 'explore') return designed(ctx);
    const w = ctx.st.w;
    return { kPth: w.kPth, kDth: w.kDth, kPz: w.kPz, kDz: w.kDz, kIz: w.kIz ?? 0, kDC: lib().dcGain(ctx.pModel, w.kPth) };
  }

  function markers(ctx, { withI = false, filter = ctx.st.filter, targets = null } = {}) {
    const p = ctx.pModel, g = ctx.gains, pin = ctx.sys.inner(p), q = qOf(p);
    const explore = ctx.S.mode === 'explore';
    // Work mode: the plant poles and zeros answer B.5(b) and B.5(c).
    const mk = showsAnswer(ctx, 'B.5/b') ? M.roots2(0, pin.a0).map((x, i) => ({ ...x, kind: 'ol', label: `open-loop pole of P_in ${i + 1}` })) : [];
    if (showsAnswer(ctx, 'B.5/c')) {
      mk.push({ re: 0, im: 0, kind: 'ol', label: 'double pole of P_out at 0' });
      mk.push({ re: -q, im: 0, kind: 'olzero', label: 'LHP zero of P_out (cancelled by the filter)' }, { re: q, im: 0, kind: 'olzero', label: 'RHP zero of P_out' });
    }
    innerPoles(ctx, g).forEach((x, i) => mk.push({ ...x, kind: 'cl', label: `inner-loop pole ${i + 1}`, dragId: explore ? 0 : undefined }));
    outerPoles(ctx, g, filter, withI).forEach((x, i) => mk.push({ ...x, kind: 'obs', label: `outer-loop pole ${i + 1} (DC-gain model)`, dragId: explore && Math.abs(x.im) > 1e-9 ? 1 : undefined, noFit: Math.hypot(x.re, x.im) > 60 }));
    if (targets) for (const t of targets) mk.push({ ...t, kind: 'target', label: 'target pole (problem)' });
    return mk;
  }

  // Drag: inner pair → (t_r,θ, ζ_θ); outer pair → (M, ζ_z).
  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    const wn = Math.hypot(re, im), zeta = Math.max(0.2, Math.min(1, -re / wn));
    if (id === 0) { st.zetaTh = zeta; st.trTh = Math.max(0.02, 2.2 / wn); }
    if (id === 1) { st.zetaZ = zeta; st.M = Math.max(1.5, Math.min(50, (2.2 / wn) / st.trTh)); }
    ctx.update();
  }

  // ------------------------------------------------------------- controls --
  const WORK_SPEC = {
    kPth: ['k<sub>Pθ</sub>', -300, 0], kDth: ['k<sub>Dθ</sub>', -40, 0],
    kPz: ['k<sub>Pz</sub>', -1, 0.5], kDz: ['k<sub>Dz</sub>', -1.5, 0.5], kIz: ['k<sub>Iz</sub>', -0.5, 0.2],
  };
  const workSliders = (parent, ctx, keys) => WB.ui.gainSliders(parent, ctx, WORK_SPEC, keys, { steps: 2000 });
  function knobSliders(parent, ctx, { kI = false } = {}) {
    slider(parent, { label: 't<sub>r,θ</sub>', unit: 's', min: 0.05, max: 2, step: 0.005, ...bind(ctx, 'trTh') });
    slider(parent, { label: 'ζ<sub>θ</sub>', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zetaTh') });
    slider(parent, { label: 'M = t<sub>r,z</sub>/t<sub>r,θ</sub>', min: 1.5, max: 30, step: 0.1, sig: 3, hint: 'bandwidth separation between the loops (p. 118)', ...bind(ctx, 'M') });
    slider(parent, { label: 'ζ<sub>z</sub>', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zetaZ') });
    if (kI) slider(parent, { label: 'k<sub>Iz</sub>', min: -0.5, max: 0.2, step: 0.0005, sig: 3, ...bind(ctx, 'kIz') });
  }
  function readout(parent, ctx, keys) {
    const names = { kPth: 'kPθ', kDth: 'kDθ', kDC: 'kDC', kPz: 'kPz', kDz: 'kDz', kIz: 'kIz' };
    WB.ui.readout(parent, () => keys.map((k) => [names[k], ctx.gains[k]]));
  }
  // Inner/outer poles and the separation between them.
  function separationPanel(parent, ctx, { withI = false } = {}) {
    const sec = section(parent, 'Bandwidth separation', 'p. 118 · Fig. 8-11');
    const box = el('div', { class: 'metrics' });
    sec.append(box);
    WB.ui.addRefresher(() => {
      const g = ctx.gains;
      const pi = pairInfo(innerPoles(ctx, g)), po = pairInfo(outerPoles(ctx, g, ctx.st.filter, withI));
      const show = WB.ui.shown(ctx, `B:${ctx.S.chapter}:sep`);
      const row = WB.ui.metric;
      box.replaceChildren(...(show ? [
        row('inner loop ωn, ζ', `${fmt(pi.wn, 3)} rad/s, ${fmt(pi.zeta, 3)}`),
        row('outer loop ωn, ζ', `${fmt(po.wn, 3)} rad/s, ${fmt(po.zeta, 3)}`),
        row('separation ωn,in / ωn,out', `${fmt(pi.wn / po.wn, 3)}×`),
        row('inner DC gain kDC', fmt(g.kDC, 4)),
      ] : [WB.ui.revealButton(ctx, `B:${ctx.S.chapter}:sep`, 'Reveal the loop poles and separation')]));
    });
  }


  // Linear design model: Eq. 6.17 with the same controller (no saturation).
  function linearSim(ctx, common, makeCtrl) {
    const { A, B: Bm, C } = ctx.sys.stateSpace(ctx.pModel);
    return WB.sim.simulate({ ...common, disturbance: null, noise: null, plant: WB.design.linearPlant(A, Bm, C), controller: makeCtrl(true) });
  }

  // -------------------------------------------------------- math snippets --
  function innerCards(ctx, g) {
    const p = ctx.pModel, pin = ctx.sys.inner(p);
    const ip = innerPoles(ctx, g);
    return [
      { title: 'Inner-loop plant (B.5, b = 0)', page: 'p. 124 · Fig. 8-14', answers: 'B.5/b',
        theory: 'P_{in}(s) = \\frac{-\\frac{1}{m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3}}}{s^2 - \\frac{(m_1+m_2)g}{m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3}}} = \\frac{b_0}{s^2 + a_0}',
        numbers: `b_0 = ${tex(pin.b0)},\\quad a_0 = ${tex(pin.a0)}` },
      { title: 'PD on a second-order plant', page: 'p. 101 · Eq. 7.5',
        theory: 'P(s) = \\frac{b_0}{s^2 + a_1 s + a_0},\\; k_P \\text{ on the error},\\; k_D s \\text{ on } y:\\quad \\Delta_{cl}(s) = s^2 + (a_1 + b_0 k_D)\\,s + (a_0 + b_0 k_P)' },
      { title: 'Inner closed loop of the pendulum', page: 'p. 125', answers: 'B.8/b1',
        theory: '\\Delta_{cl}(s) = s^2 - \\frac{k_{D\\theta}}{J}s - \\Big(\\frac{(m_1+m_2)g}{J} + \\frac{k_{P\\theta}}{J}\\Big),\\quad J = m_1\\tfrac{\\ell}{6} + m_2\\tfrac{2\\ell}{3}',
        numbers: `\\Delta_{cl} = s^2 + ${tex(pin.b0 * g.kDth)}\\,s + ${tex(pin.a0 + pin.b0 * g.kPth)},\\quad p = ${texPole(ip[0])},\\; ${texPole(ip[1])}` },
      { title: 'Gains from the spec', page: 'p. 110 · Eq. 8.2, p. 113 · Eq. 8.5',
        theory: '\\Delta^d_{cl} = s^2 + 2\\zeta\\omega_n s + \\omega_n^2,\\; \\omega_n = \\frac{2.2}{t_r}:\\quad k_P = \\frac{\\omega_n^2 - a_0}{b_0},\\; k_D = \\frac{2\\zeta\\omega_n - a_1}{b_0},\\quad k_{DC} = \\frac{Y(s)}{R(s)}\\Big|_{s=0}' },
      { title: 'Inner gains and DC gain of the pendulum', page: 'p. 125', answers: ['B.8/b2', 'B.8/c'],
        theory: 'k_{P\\theta} = -(m_1+m_2)g - J\\,\\omega_{n\\theta}^2,\\quad k_{D\\theta} = -2\\zeta_\\theta\\omega_{n\\theta} J,\\quad k_{DC\\theta} = \\frac{k_{P\\theta}}{(m_1+m_2)g + k_{P\\theta}}',
        numbers: `k_{P\\theta} = ${tex(g.kPth)},\\quad k_{D\\theta} = ${tex(g.kDth)},\\quad k_{DC\\theta} = ${tex(g.kDC)}` },
    ];
  }
  function outerCards(ctx, g, { withI = false } = {}) {
    const p = ctx.pModel, q = qOf(p), zf = lib().zcFilter(p, g.kDC, ctx.S.sim.Ts);
    const op = outerPoles(ctx, g, ctx.st.filter, withI);
    return [
      { title: 'Outer-loop plant (B.5)', page: 'p. 125–126 · Fig. 8-15', answers: 'B.5/c',
        theory: '\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2} = -\\frac{2\\ell}{3}\\frac{(s + \\sqrt{3g/2\\ell})(s - \\sqrt{3g/2\\ell})}{s^2}',
        numbers: `\\sqrt{3g/2\\ell} = ${tex(q)}`,
        note: 'A PD controller cannot match a third-order characteristic polynomial to s² + 2ζωₙs + ωₙ², so the LHP zero is cancelled first (p. 126).' },
      { title: 'Successive loop closure', page: 'p. 117–118 · Fig. 8-10, 8-11',
        theory: '\\text{inner loop much faster than the outer:}\\quad \\text{replace it by its DC gain } k_{DC} \\text{ when designing the outer loop},\\quad t_{r,out} = M\\,t_{r,in},\\; M \\approx 5\\text{–}10' },
      { title: 'Canceling a zero', page: 'p. 126',
        theory: '\\frac{k_F}{s + p_F}\\cdot(s + p_F) = k_F \\quad(\\text{only for a left-half-plane zero } -p_F)' },
      { title: 'Zero-canceling filter of the pendulum', page: 'p. 126 · Fig. 8-16', answers: 'B.8/d1',
        theory: 'F(s) = \\frac{-\\frac{1}{k_{DC\\theta}}\\frac{3}{2\\ell}}{s + \\sqrt{3g/2\\ell}} \\;\\Rightarrow\\; k_{DC}\\,F(s)\\,\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{s - \\sqrt{3g/2\\ell}}{s^2}',
        numbers: `F(s) = \\frac{${tex(zf.a)}}{s + ${tex(zf.b)}}`,
        note: ctx.st.filter ? 'Implemented with one Euler step per sample (zeroCancelingFilter, Listing 8.3).' : 'The filter is off: θ_r is the PD output itself, and the outer loop is third order.' },
      { title: 'Outer gains of the pendulum', page: 'p. 127 · Eq. 8.12–8.13', answers: 'B.8/d2',
        theory: 'a = \\frac{k_{Pz}}{1 + k_{Dz}} = -\\sqrt{\\tfrac{2\\ell}{3g}}\\,\\omega_{nz}^2,\\quad b = \\frac{k_{Dz}}{1 + k_{Dz}} = \\sqrt{\\tfrac{2\\ell}{3g}}\\big(a - 2\\zeta_z\\omega_{nz}\\big),\\quad k_{Dz} = \\frac{b}{1 - b},\\; k_{Pz} = \\frac{a}{1 - b}',
        numbers: `k_{Pz} = ${tex(g.kPz)},\\quad k_{Dz} = ${tex(g.kDz)}${withI ? `,\\quad k_{Iz} = ${tex(g.kIz)}` : ''},\\quad p_{out} = ${op.map((x) => texPole(x)).join(',\\;')}`,
        note: ctx.st.formula === 'listing' && ctx.S.mode === 'explore' ? 'Using Listing 8.3 (ctrlPD.py). Its k_Dz expression is not Eq. 8.13, so the outer ζ comes out larger than ζ_z (ISSUES.md).' : undefined },
    ];
  }
  function fullCard(ctx, g, withI) {
    const fp = fullPoles(ctx, g, ctx.st.filter, withI);
    return {
      title: 'Check: full linearized closed loop', page: 'Eq. 6.17 + both loops (not in the book)',
      theory: '\\text{eig of the 4-state plant with both loops closed}' + (ctx.st.filter ? '\\text{, plus the filter state}' : '') + (withI ? '\\text{ and } x_I' : ''),
      numbers: fp.map((x) => texPole(x)).join(',\\;'), spoiler: true,
      note: 'With good separation these sit close to the inner and outer design poles. Stable here means the linearized loop is stable; saturation and large angles are not modeled.',
    };
  }

  // ------------------------------------------------------------- Chapter 8 --
  const W8 = { kPth: -40, kDth: -6, kPz: -0.05, kDz: -0.1, kIz: 0 };   // work-mode start: not the answer

  B.chapters.ch8 = {
    id: 'ch8', num: 8, tab: 'Ch 8', title: 'Successive loop closure', pages: 'pp. 107–136',
    defaults(sys) {
      const pr = sys.problems.ch8;
      return { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, formula: 'book', filter: true, extra: 'thetaR', w: { ...W8 } };
    },
    simDefaults(sys) { return sys.problems.ch8.sim; },
    gains: gainsFor,
    controller(ctx, { linear = false } = {}) {
      return lib().slcPD({ g: ctx.gains, p: ctx.pModel, Ts: ctx.S.sim.Ts, uLim: ctx.sys.uLimit(ctx.pModel), filter: ctx.st.filter, linear });
    },
    linearSim(ctx, c) { return linearSim(ctx, c, (lin) => this.controller(ctx, { linear: lin })); },
    targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.trTh * ctx.st.M } : {}; },

    specPoles(ctx) {
      const pr = ctx.sys.problems.ch8;
      return [...WB.design.polesFromWnZeta(2.2 / pr.trTh, pr.zetaTh), ...WB.design.polesFromWnZeta(2.2 / (pr.trTh * pr.M), pr.zetaZ)];
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'PD inner loop, PD outer loop', 'p. 124 · Fig. 8-16');
      segmented(sec, {
        label: 'Zero-canceling filter',
        options: [{ value: true, label: 'on (B.8d)' }, { value: false, label: 'off' }],
        ...bind(ctx, 'filter'),
      });
      if (ctx.S.mode === 'work') {
        workSliders(sec, ctx, ['kPth', 'kDth', 'kPz', 'kDz']);
        sec.append(el('p', { class: 'muted small', text: 'Dashed rings in the s-plane mark the problem\'s target poles (inner t_r = 0.5 s; outer M = 10).' }));
      } else {
        knobSliders(sec, ctx);
        segmented(sec, {
          label: 'Outer gains from',
          options: [{ value: 'book', label: 'Eq. 8.12–8.13' }, { value: 'listing', label: 'Listing 8.3 (ctrlPD.py)' }],
          ...bind(ctx, 'formula'),
        });
        sec.append(el('div', { class: 'btn-row' },
          el('button', { type: 'button', class: 'btn btn-quiet', text: 'B.8(b) spec', onclick: () => { Object.assign(ctx.st, { trTh: 0.5, zetaTh: 0.707, M: 10, zetaZ: 0.707, formula: 'book' }); ctx.update(); } }),
          el('button', { type: 'button', class: 'btn btn-quiet', text: 'Listing 8.3 tuning', title: 't_r,θ = 0.15, M = 15, with the listing\'s gain formulas', onclick: () => { Object.assign(ctx.st, { trTh: 0.15, zetaTh: 0.707, M: 15, zetaZ: 0.707, formula: 'listing' }); ctx.update(); } })));
        sec.append(el('p', { class: 'muted small', text: 'Drag an inner (blue) pole to set t_r,θ and ζ_θ, or an outer (green) pole to set M and ζ_z.' }));
        readout(sec, ctx, ['kPth', 'kDth', 'kDC', 'kPz', 'kDz']);
      }
      separationPanel(parent, ctx);
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'thetaR', label: 'inner loop: θ_r vs θ' }, { value: 'zdot', label: 'ż' }], ...bind(ctx, 'extra') });
    },

    extraPlot(ctx, res) {
      const k = 180 / Math.PI;
      if (ctx.st.extra === 'zdot') return { opts: { title: 'ż(t)', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [{ label: 'ż', y: res.x.map((x) => x[2]), color: '--series-1' }] } };
      return {
        opts: { title: 'inner loop: commanded θ_r and θ', yLabel: 'θ [°]', unit: '°' },
        data: { series: [
          { label: 'θ_r from the outer loop', y: Array.from(res.extras.thetaR || [], (v) => v * k), color: '--ref', dash: [6, 4], width: 1.5 },
          { label: 'θ', y: res.x.map((x) => x[1] * k), color: '--series-1' },
        ] },
      };
    },

    splane(ctx) {
      return { markers: markers(ctx, { targets: ctx.S.mode === 'work' ? this.specPoles(ctx) : null }), legendNames: LOOP_NAMES, zetaRay: ctx.S.mode === 'explore' ? ctx.st.zetaTh : null };
    },
    onPoleDrag: onDrag,

    math(ctx) {
      const g = ctx.gains;
      return [...innerCards(ctx, g), ...outerCards(ctx, g), fullCard(ctx, g, false)];
    },

    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch8;
      const ref = () => lib().slcGains(ctx.pModel, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, formula: 'book' });
      const cx = WB.py.cx;
      const inDen = (p, a) => { const pin = ctx.sys.inner(p); return cx.poly([1, pin.b0 * a.kDth, pin.a0 + pin.b0 * a.kPth], a.s); };
      const outDen = (p, a) => cx.poly(outerPoly({ pModel: p }, { kPz: a.kPz, kDz: a.kDz }, true), a.s);
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Block diagram with PD control on both loops',
          html: 'On paper: the outer controller takes r<sub>z</sub> and outputs r<sub>θ</sub>; the inner controller takes r<sub>θ</sub> and outputs F. When you are done, click the button and compare with the solution.',
          done: 'I\'ve drawn it',
          solution: () => [{ html: 'Outer: r<sub>z</sub> − z → PD (k<sub>Pz</sub> on the error, k<sub>Dz</sub>s on z) → r<sub>θ</sub>. Inner: r<sub>θ</sub> − θ → PD (k<sub>Pθ</sub> on the error, k<sub>Dθ</sub>s on θ) → F → pendulum.' }, { html: 'Figs. 8-14 to 8-16 (pp. 124–126). In the final design the outer loop is PD followed by the low-pass filter with gain, and the inner loop is replaced by k<sub>DCθ</sub> for the outer design.' }],
        },
        {
          id: 'b1', title: '(b) Inner closed loop',
          html: 'With the B.5 inner-loop plant (b = 0), k<sub>Pθ</sub> on the error and k<sub>Dθ</sub>s on θ (Fig. 8-14): return Θ̃/R̃<sub>θ</sub> and the characteristic polynomial Δ<sub>cl</sub>(s) in terms of the gains. Any nonzero multiple of Δ<sub>cl</sub> is accepted.',
          code: {
            template: 'def theta_cl(s, kP, kD):\n    # Theta(s)/R_theta(s)\n    return ...\n\ndef char_in(s, kP, kD):\n    # Delta_cl(s)\n    return ...\n',
            check: pyCheck(ctx, {
              items: [
                { fn: 'theta_cl', args: ['s', 'kPth', 'kDth'], truth: (p, a) => cx.div(ctx.sys.inner(p).b0 * a.kPth, inDen(p, a)) },
                { fn: 'char_in', args: ['s', 'kPth', 'kDth'], compare: 'scale', truth: inDen },
              ],
            }),
          },
          solution: () => [
            { tex: '\\frac{\\tilde\\Theta}{\\tilde R_\\theta} = \\frac{-\\frac{k_{P\\theta}}{J}}{s^2 - \\frac{k_{D\\theta}}{J}s - \\Big(\\frac{(m_1+m_2)g}{J} + \\frac{k_{P\\theta}}{J}\\Big)},\\quad J = m_1\\tfrac{\\ell}{6} + m_2\\tfrac{2\\ell}{3}' },
            { code: 'J = P.m1 * P.ell / 6 + P.m2 * 2 * P.ell / 3\n\ndef char_in(s, kP, kD):\n    return (s**2 - kD / J * s\n            - ((P.m1 + P.m2) * P.g + kP) / J)\n\ndef theta_cl(s, kP, kD):\n    return -kP / J / char_in(s, kP, kD)' },
            { html: 'Book: p. 125.' },
          ],
        },
        {
          id: 'b2', title: `(b) Inner PD gains for t<sub>r<sub>θ</sub></sub> = ${pr.trTh} s, ζ<sub>θ</sub> = ${pr.zetaTh}`,
          inputs: { kPth: 'k<sub>Pθ</sub>', kDth: 'k<sub>Dθ</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { kPth: r.kPth, kDth: r.kDth }, { kPth: 'kPθ', kDth: 'kDθ' }); },
          actions: [WB.design.useGains(ctx, ['kPth', 'kDth'])],
          solution: () => { const r = ref(); return [{ tex: `\\omega_{n\\theta} = 2.2/${pr.trTh} = ${tex(r.wnTh)},\\quad k_{P\\theta} = -(m_1+m_2)g - J\\omega_{n\\theta}^2 = ${tex(r.kPth)},\\quad k_{D\\theta} = -2\\zeta_\\theta\\omega_{n\\theta}J = ${tex(r.kDth)}` }, { html: 'Book: −26.0 and −4.41 (p. 125).' }]; },
        },
        {
          id: 'c', title: '(c) DC gain of the inner loop',
          html: 'Return k<sub>DCθ</sub> as a function of k<sub>Pθ</sub> (and k<sub>Dθ</sub>, if it depends on it). With your (b) gains it gives the number the book reports.',
          code: {
            template: 'def k_DC(kP, kD):\n    # DC gain of Theta/R_theta\n    return ...\n',
            check: pyCheck(ctx, { items: [{ fn: 'k_DC', args: ['kPth', 'kDth'], truth: (p, a) => lib().dcGain(p, a.kPth) }] }),
          },
          solution: () => [
            { tex: `k_{DC\\theta} = \\frac{\\tilde\\Theta}{\\tilde R_\\theta}\\Big|_{s=0} = \\frac{k_{P\\theta}}{(m_1+m_2)g + k_{P\\theta}} = ${tex(ref().kDC)} \\;(\\text{(b) gains})` },
            { code: 'def k_DC(kP, kD):\n    return kP / ((P.m1 + P.m2) * P.g + kP)' },
            { html: 'Book: 1.89 (p. 125). It is not 1: a PD inner loop on an unstable plant leaves a steady-state angle error, which the filter gain compensates.' },
          ],
        },
        {
          id: 'd1', title: '(d) Low-pass filter and outer closed loop',
          html: 'Replace the inner loop by its DC gain. Return the low-pass filter F(s) (with gain) that cancels the left-half-plane zero of the B.5 outer plant and the inner DC gain, then the outer closed loop Z̃/R̃<sub>z</sub> and its characteristic polynomial with k<sub>Pz</sub> on the error and k<sub>Dz</sub>s on z. Any nonzero multiple of Δ<sub>cl</sub> is accepted.',
          code: {
            template: 'def filt(s, k_DC):\n    # zero-canceling filter F(s)\n    return ...\n\ndef z_cl(s, kP, kD):\n    # Z(s)/R_z(s) with the filter\n    return ...\n\ndef char_out(s, kP, kD):\n    # its Delta_cl(s)\n    return ...\n',
            check: pyCheck(ctx, {
              items: [
                { fn: 'filt', args: ['s', 'kDC'], truth: (p, a) => { const zf = lib().zcFilter(p, a.kDC, 0.01); return cx.div(zf.a, cx.add(a.s, zf.b)); } },
                { fn: 'z_cl', args: ['s', 'kPz', 'kDz'], truth: (p, a) => cx.div(cx.mul(a.kPz, cx.add(a.s, -qOf(p))), outDen(p, a)) },
                { fn: 'char_out', args: ['s', 'kPz', 'kDz'], compare: 'scale', truth: outDen },
              ],
            }),
          },
          solution: () => [
            { tex: 'F(s) = \\frac{-\\frac{1}{k_{DC\\theta}}\\frac{3}{2\\ell}}{s + \\sqrt{3g/2\\ell}} \\;\\Rightarrow\\; k_{DC\\theta}F(s)\\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2} = \\frac{s - \\sqrt{3g/2\\ell}}{s^2}' },
            { tex: '\\frac{\\tilde Z}{\\tilde R_z} = \\frac{\\frac{k_{Pz}}{1 + k_{Dz}}(s - \\sqrt{3g/2\\ell})}{s^2 + \\frac{k_{Pz} - k_{Dz}\\sqrt{3g/2\\ell}}{1 + k_{Dz}}s - \\frac{k_{Pz}\\sqrt{3g/2\\ell}}{1 + k_{Dz}}}' },
            { code: 'q = np.sqrt(3 * P.g / (2 * P.ell))\n\ndef filt(s, k_DC):\n    return -3 / (2 * P.ell * k_DC) / (s + q)\n\ndef char_out(s, kP, kD):\n    return ((1 + kD) * s**2 + (kP - q * kD) * s\n            - q * kP)\n\ndef z_cl(s, kP, kD):\n    return kP * (s - q) / char_out(s, kP, kD)' },
            { html: 'Book: Figs. 8-16 and 8-17 (pp. 126–127).' },
          ],
        },
        {
          id: 'd2', title: '(d) Outer PD gains that stabilize the cart position',
          html: `The book leaves the outer specs open; this page uses a bandwidth separation M = ${pr.M} (t<sub>r,z</sub> = ${pr.M} t<sub>r,θ</sub>, ω<sub>n</sub> = 2.2/t<sub>r</sub>) and ζ<sub>z</sub> = ${pr.zetaZ}.`,
          inputs: { kPz: 'k<sub>Pz</sub>', kDz: 'k<sub>Dz</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { kPz: r.kPz, kDz: r.kDz }, { kPz: 'kPz', kDz: 'kDz' }); },
          actions: [WB.design.useGains(ctx, ['kPz', 'kDz'])],
          solution: () => {
            const r = ref();
            return [
              { tex: `\\omega_{nz} = \\frac{2.2}{${pr.M}\\cdot ${pr.trTh}} = ${tex(r.wnZ)},\\quad a = -\\sqrt{\\tfrac{2\\ell}{3g}}\\,\\omega_{nz}^2 = ${tex(r.a)},\\quad b = \\sqrt{\\tfrac{2\\ell}{3g}}(a - 2\\zeta_z\\omega_{nz}) = ${tex(r.b)}` },
              { tex: `k_{Dz} = \\frac{b}{1-b} = ${tex(r.kDz)},\\quad k_{Pz} = \\frac{a}{1-b} = ${tex(r.kPz)}` },
              { html: 'Eqs. 8.12–8.13 (p. 127). Listing 8.3 computes k<sub>Dz</sub> with a different expression; try "Listing 8.3 tuning" in Explore mode and compare the outer ζ.' },
            ];
          },
        },
        {
          id: 'e', title: '(e) Implement with |F| ≤ 5 N and balance from θ(0) = 10°',
          html: 'Uses the current simulation (Work mode gains). Requires θ(0) = 10° (left panel; the chapter default). Passes if the pendulum never passes 45° and both |θ| < 1° and |z − r| < 5 cm at t = 10 s.',
          check: () => {
            if (ctx.S.mode !== 'work') return { ok: false, msg: 'Switch to Work mode so the simulation uses your gains.' };
            if (Math.abs((ctx.S.sim.init.theta0 ?? 0) - 10) > 0.25) return { ok: false, msg: `Set θ(0) = 10° in the left panel (now ${fmt(ctx.S.sim.init.theta0 ?? 0, 3)}°).` };
            const res = ctx.app.result(), Ts = ctx.S.sim.Ts;
            const i10 = Math.min(res.t.length - 1, Math.round(10 / Ts));
            let maxTh = 0; for (const x of res.x) maxTh = Math.max(maxTh, Math.abs(x[1]));
            const th10 = Math.abs(res.x[i10][1]) / DEG, ez = Math.abs(res.rAll[0][i10] - res.x[i10][0]);
            const ok = maxTh < 45 * DEG && th10 < 1 && ez < 0.05;
            return { ok, msg: `max |θ| = ${fmt(maxTh / DEG, 3)}°, at 10 s |θ| = ${fmt(th10, 3)}°, |z − r| = ${fmt(ez, 3)} m (θ(0) = ${fmt(ctx.S.sim.init.theta0, 3)}°).` };
          },
        },
      ]);
    },
  };

  // ------------------------------------------------------------- Chapter 9 --
  B.chapters.ch9 = {
    id: 'ch9', num: 9, tab: 'Ch 9', title: 'System type & integrators', pages: 'pp. 137–154',
    defaults() { return { trTh: 0.5, zetaTh: 0.707, M: 10, zetaZ: 0.707, formula: 'book', filter: true, kIz: 0, input: 'step', w: { ...W8 } }; },
    simDefaults(sys) { return sys.problems.ch9.sim; },
    gains: gainsFor,
    controller(ctx, { linear = false } = {}) {
      return lib().slcPID({ g: ctx.gains, p: ctx.pModel, Ts: ctx.S.sim.Ts, uLim: ctx.sys.uLimit(ctx.pModel), gate: false, deriv: 'state', thetaMax: Infinity, filter: ctx.st.filter, linear });
    },
    reference: (ctx, base) => WB.pid.shapedReference(ctx, base),   // step, ramp, parabola on z_r
    linearSim(ctx, c) { return linearSim(ctx, c, (lin) => this.controller(ctx, { linear: lin })); },

    analysis(ctx) {
      const p = ctx.pModel, g = ctx.gains, mg = (p.m1 + p.m2) * p.g, q = qOf(p);
      const hasI = Math.abs(g.kIz) > 0;
      return {
        inStep: 1 / (1 + g.kPth / mg), inDist: 1 / (mg + g.kPth),
        outType: hasI ? 3 : 2, Ma: p.g * g.kPz, MaImpl: -q * g.kPz, outDistPD: 1 / g.kPz, outDistI: 1 / g.kIz, hasI,
      };
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'PD inner loop, PD/PID outer loop', 'p. 147–148');
      segmented(sec, {
        label: 'Reference shape (amplitude = size [m], slope [m/s], or coefficient [m/s²])',
        options: [{ value: 'step', label: 'step' }, { value: 'ramp', label: 'ramp' }, { value: 'parabola', label: 'parabola' }],
        ...bind(ctx, 'input'),
      });
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['kPth', 'kDth', 'kPz', 'kDz', 'kIz']);
      else { knobSliders(sec, ctx, { kI: true }); readout(sec, ctx, ['kPth', 'kDth', 'kDC', 'kPz', 'kDz', 'kIz']); }
      sec.append(el('p', { class: 'muted small', text: 'Derivatives use the true ż and θ̇ here, and the integrator is never gated, so the steady-state errors are the ones the type analysis predicts. A force disturbance is set in the left panel.' }));
      const ty = section(parent, 'System type analysis', 'p. 148–150 · Table 9-1');
      const box = el('div', { class: 'metrics' });
      ty.append(box);
      WB.ui.addRefresher(() => this.renderType(box, ctx));
    },

    renderType(box, ctx) {
      const a = this.analysis(ctx);
      const res = ctx.app.result();
      const n = res ? res.t.length - 1 : 0;
      const eEnd = res ? res.rAll[0][n] - res.yAll[0][n] : NaN;
      const row = WB.ui.metric;
      const show = WB.ui.shown(ctx, 'B:ch9:type');
      const rows = [];
      if (show) {
        rows.push(row('inner loop: type (tracking, disturbance)', 'type 0, type 0'));
        rows.push(row('inner step error per unit r_θ', fmt(a.inStep, 4)));
        rows.push(row('inner error per unit step d', `${fmt(a.inDist, 4)} rad/N`));
        rows.push(row('outer loop tracking type', `type ${a.outType}`));
        if (!a.hasI) rows.push(row('outer parabola error 1/M_a (book model)', fmt(1 / a.Ma, 4)));
        rows.push(row('outer disturbance type', a.hasI ? 'type 1' : 'type 0'));
      } else {
        rows.push(WB.ui.revealButton(ctx, 'B:ch9:type', 'Reveal the predicted types and errors'));
      }
      rows.push(row('simulated error r − z at t_end', isFinite(eEnd) ? `${fmt(eEnd, 3)} m` : '—'));
      box.replaceChildren(...rows);
    },

    splane(ctx) { return { markers: markers(ctx, { withI: true }), legendNames: LOOP_NAMES }; },
    onPoleDrag: onDrag,

    math(ctx) {
      const a = this.analysis(ctx);
      return [
        { title: 'System type and the error constants', page: 'p. 140–141 · Table 9-1',
          theory: '\\text{type} = \\text{number of free integrators in } PC,\\quad e_{step} = \\frac{1}{1 + M_p},\\; e_{ramp} = \\frac{1}{M_v},\\; e_{parab} = \\frac{1}{M_a},\\quad M_p = \\lim_{s\\to0}PC,\\; M_v = \\lim_{s\\to0}sPC,\\; M_a = \\lim_{s\\to0}s^2PC' },
        { title: 'Input disturbance', page: 'p. 143–144',
          theory: 'E(s) = \\frac{P}{1 + PC}D(s),\\quad D = \\frac{A}{s^{q+1}}:\\; e_{ss} = \\lim_{s\\to0} s\\frac{P}{1+PC}\\frac{A}{s^{q+1}}',
          note: 'The type with respect to an input disturbance depends on the integrators in C only.' },
        { title: 'Inner loop of the pendulum', page: 'p. 148–149 · Fig. 9-9', answers: ['B.9/a1', 'B.9/a2'],
          theory: 'P(s)C(s) = \\frac{-\\frac{1}{J}}{s^2 - \\frac{(m_1+m_2)g}{J}}(k_D s + k_P) \\Rightarrow \\text{type 0},\\quad e_{step} = \\frac{1}{1 + \\frac{k_P}{(m_1+m_2)g}},\\; e_{ramp} = \\infty,\\quad e_{d,step} = \\frac{1}{(m_1+m_2)g + k_P}',
          numbers: `e_{step} = ${tex(a.inStep)},\\quad e_{d,step} = ${tex(a.inDist)}` },
        { title: 'Outer loop of the pendulum (Fig. 9-10)', page: 'p. 149–150', answers: ['B.9/b1', 'B.9/b2'],
          theory: 'P(s)C(s) = \\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2}\\cdot\\frac{k_D s^2 + k_P s + k_I}{s},\\quad k_I = 0:\\; \\text{type 2},\\; e_{parab} = \\frac{1}{k_P g},\\; e_{d,step} = \\frac{1}{k_P},\\quad k_I \\ne 0:\\; \\text{type 3},\\; \\text{disturbance type 1},\\; e_{d,ramp} = \\frac{1}{k_I}',
          numbers: a.hasI ? `1/k_{Iz} = ${tex(a.outDistI)}` : `M_a = g\\,k_{Pz} = ${tex(a.Ma)},\\quad e_{parab} = ${tex(1 / a.Ma)},\\quad 1/k_{Pz} = ${tex(a.outDistPD)}`,
          note: 'Fig. 9-10 drops k_DC and the filter. With them (Fig. 8-17) the loop is (s − q)/s² · C, which changes M_a but not the type.' },
      ];
    },

    buildProblem(parent, ctx) {
      const inf = (v) => /^\s*(inf|infinity|∞)\s*$/i.test(v || '');
      const ints = (v, truth) => {
        for (const [k, want] of Object.entries(truth)) {
          if (want === Infinity) { if (!inf(v[k])) return { ok: false, msg: `Check ${k}.` }; continue; }
          const g = PD().num(v[k]);
          if (g === null) return { ok: false, msg: `Enter ${k}.` };
          if (Math.abs(g - want) > 1e-9) return { ok: false, msg: `Check ${k}.` };
        }
        return { ok: true, msg: '' };
      };
      const pin = (p) => ctx.sys.inner(p);
      const g0 = (p) => ctx.sys.outerTf(p).num[2];   // P_out(s) s² at s = 0
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch9, [
        {
          id: 'a1', title: '(a) Inner loop under PD: system types',
          html: 'Type with respect to tracking r<sub>θ</sub>, the steady-state error to a unit ramp and a unit parabola (enter <code>inf</code> for an unbounded error), and the type with respect to an input disturbance.',
          inputs: { type: 'tracking type', ramp: 'e<sub>ramp</sub>', parab: 'e<sub>parab</sub>', dtype: 'disturbance type' },
          check: (v) => ints(v, { type: 0, ramp: Infinity, parab: Infinity, dtype: 0 }),
          solution: () => [{ tex: '\\text{no free integrator in } PC \\Rightarrow \\text{type 0},\\; e_{ramp} = e_{parab} = \\infty;\\quad \\text{disturbance: type 0}' }, { html: 'Book: p. 148–149.' }],
        },
        {
          id: 'a2', title: '(a) Inner loop under PD: steady-state errors',
          html: 'Return the error to a unit step in r<sub>θ</sub> and to a unit step input disturbance, as functions of the PD gains (signed, as the limits give them).',
          code: {
            template: 'def e_step(kP, kD):\n    # unit step in r_theta\n    return ...\n\ndef e_dist(kP, kD):\n    # unit step input disturbance\n    return ...\n',
            check: pyCheck(ctx, {
              items: [
                { fn: 'e_step', args: ['kPth', 'kDth'], truth: (p, a) => 1 / (1 + pin(p).b0 * a.kPth / pin(p).a0) },
                { fn: 'e_dist', args: ['kPth', 'kDth'], truth: (p, a) => pin(p).b0 / (pin(p).a0 + pin(p).b0 * a.kPth) },
              ],
            }),
          },
          solution: () => [
            { tex: 'e_{step} = \\frac{1}{1 + \\lim_{s\\to0}PC} = \\frac{1}{1 + \\frac{k_P}{(m_1+m_2)g}},\\quad e_{d,step} = \\lim_{s\\to0}\\frac{P}{1 + PC} = \\frac{1}{(m_1+m_2)g + k_P}' },
            { code: 'def e_step(kP, kD):\n    return 1 / (1 + kP / ((P.m1 + P.m2) * P.g))\n\ndef e_dist(kP, kD):\n    return 1 / ((P.m1 + P.m2) * P.g + kP)' },
            { html: 'Book: p. 148–149. Note e_step = 1 − k_DCθ.' },
          ],
        },
        {
          id: 'b1', title: '(b) Outer loop: system types under PD and PID',
          html: 'With the book\'s outer-loop model (Fig. 9-10). Under PD: the tracking type, the errors to a unit step and a unit ramp, and the disturbance type. With the integrator (PID): the tracking type and the disturbance type.',
          inputs: { pd: 'PD type', step: 'PD e<sub>step</sub>', ramp: 'PD e<sub>ramp</sub>', pdd: 'PD disturbance type', pid: 'PID type', pidd: 'PID disturbance type' },
          check: (v) => ints(v, { pd: 2, step: 0, ramp: 0, pdd: 0, pid: 3, pidd: 1 }),
          solution: () => [{ tex: 'k_I = 0:\\; \\text{type 2},\\; e_{step} = e_{ramp} = 0,\\; \\text{disturbance type 0};\\quad k_I \\ne 0:\\; \\text{type 3 (zero error to a step, ramp and parabola)},\\; \\text{disturbance type 1}' }, { html: 'Book: p. 149–150.' }],
        },
        {
          id: 'b2', title: '(b) Outer loop: the remaining errors',
          html: 'Return, as functions of the outer gains (signed, as the limits give them): the PD error to a unit parabola, the PD error to a unit step disturbance, and the PID error to a unit ramp disturbance.',
          code: {
            template: 'def e_parab(kP, kD):\n    # PD, unit parabola\n    return ...\n\ndef e_dist_pd(kP, kD):\n    # PD, unit step disturbance\n    return ...\n\ndef e_dist_pid(kP, kI, kD):\n    # PID, unit ramp disturbance\n    return ...\n',
            check: pyCheck(ctx, {
              items: [
                { fn: 'e_parab', args: ['kPz', 'kDz'], truth: (p, a) => 1 / (a.kPz * g0(p)) },
                { fn: 'e_dist_pd', args: ['kPz', 'kDz'], truth: (p, a) => 1 / a.kPz },
                { fn: 'e_dist_pid', args: ['kPz', 'kIz', 'kDz'], truth: (p, a) => 1 / a.kIz },
              ],
            }),
          },
          solution: () => [
            { tex: 'e_{parab} = \\frac{1}{M_a} = \\frac{1}{\\lim_{s\\to0}s^2PC} = \\frac{1}{k_P g},\\quad e_{d,step} = \\frac{g}{g\\,k_P} = \\frac{1}{k_P},\\quad e_{d,ramp} = \\frac{g}{g\\,k_I} = \\frac{1}{k_I}' },
            { code: 'def e_parab(kP, kD):\n    return 1 / (kP * P.g)\n\ndef e_dist_pd(kP, kD):\n    return 1 / kP\n\ndef e_dist_pid(kP, kI, kD):\n    return 1 / kI' },
            { html: 'Book: p. 149–150 (Fig. 9-10, without k<sub>DC</sub> and the filter). Try it: the reference shape and the disturbance are on the left; the type readout on the right compares with the simulation.' },
          ],
        },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 10 --
  const W10 = { kPth: -60, kDth: -8, kPz: -0.1, kDz: -0.2, kIz: 0 };

  B.chapters.ch10 = {
    id: 'ch10', num: 10, tab: 'Ch 10', title: 'Digital PID', pages: 'pp. 155–169',
    defaults(sys) {
      const pr = sys.problems.ch10;
      return { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, kIz: pr.ki, formula: 'book', filter: true, sigma: pr.sigma, vbar: pr.vbar, gate: true, thetaMax: pr.thetaMax, deriv: 'dirty', extra: 'thetaR', w: { ...W10 } };
    },
    simDefaults(sys) { return { ...sys.problems.ch10.sim, mismatch: sys.problems.ch10.mismatch }; },
    gains: gainsFor,
    controller(ctx, { linear = false } = {}) {
      const st = ctx.st;
      return lib().slcPID({ g: ctx.gains, p: ctx.pModel, Ts: ctx.S.sim.Ts, uLim: ctx.sys.uLimit(ctx.pModel), sigma: st.sigma, vbar: st.vbar, gate: st.gate, thetaMax: st.thetaMax * DEG, filter: st.filter, deriv: st.deriv, linear });
    },
    linearSim(ctx, c) { return linearSim(ctx, c, (lin) => this.controller(ctx, { linear: lin })); },
    targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.trTh * ctx.st.M } : {}; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Outer PID, inner PD', 'p. 163 · Listing 10.3');
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['kPth', 'kDth', 'kPz', 'kDz', 'kIz']);
      else { knobSliders(sec, ctx, { kI: true }); readout(sec, ctx, ['kPth', 'kDth', 'kDC', 'kPz', 'kDz', 'kIz']); }
      separationPanel(parent, ctx, { withI: true });
      const imp = section(parent, 'Implementation', 'p. 157 · Eq. 10.4, p. 165');
      segmented(imp, { label: 'ż, θ̇ for the D terms', options: [{ value: 'dirty', label: 'dirty derivatives of z, θ' }, { value: 'state', label: 'true ż, θ̇ (cheating)' }], ...bind(ctx, 'deriv') });
      slider(imp, { label: 'σ', unit: 's', min: 0.002, max: 0.3, step: 0.001, sig: 3, ...bind(ctx, 'sigma'), disabled: () => ctx.st.deriv !== 'dirty' });
      segmented(imp, { label: 'Anti-windup', options: [{ value: true, label: 'integrate when |ż| < v̄' }, { value: false, label: 'none' }], ...bind(ctx, 'gate') });
      slider(imp, { label: 'v̄', unit: 'm/s', min: 0.005, max: 1, step: 0.005, sig: 3, ...bind(ctx, 'vbar'), disabled: () => !ctx.st.gate });
      slider(imp, { label: 'θ<sub>max</sub>', unit: '°', min: 5, max: 90, step: 1, sig: 3, hint: 'saturation on r_θ before the filter', ...bind(ctx, 'thetaMax') });
      segmented(imp, { label: 'Zero-canceling filter', options: [{ value: true, label: 'on' }, { value: false, label: 'off' }], ...bind(ctx, 'filter') });
      segmented(imp, { label: 'Extra plot', options: [{ value: 'thetaR', label: 'θ_r vs θ' }, { value: 'zdot', label: 'ż estimate' }, { value: 'int', label: 'integrator' }], ...bind(ctx, 'extra') });
    },

    extraPlot(ctx, res) {
      const k = 180 / Math.PI;
      if (ctx.st.extra === 'int') return { opts: { title: 'integrator ∫(z_r − z) dt', yLabel: '∫e dt [m·s]', unit: 'm·s' }, data: { series: [{ label: 'integrator', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
      if (ctx.st.extra === 'zdot') {
        return {
          opts: { title: 'ż(t)', yLabel: 'ż [m/s]', unit: 'm/s' },
          data: { series: [
            { label: 'controller estimate', y: Array.from(res.extras.zdotHat || []), color: '--series-2', width: 1.5 },
            { label: 'true ż', y: res.x.map((x) => x[2]), color: '--series-1' },
          ], hlines: ctx.st.gate ? [{ y: ctx.st.vbar, label: '+v̄', color: '--text-muted' }, { y: -ctx.st.vbar, label: '−v̄', color: '--text-muted', fit: false }] : [] },
        };
      }
      return {
        opts: { title: 'inner loop: commanded θ_r and θ', yLabel: 'θ [°]', unit: '°' },
        data: { series: [
          { label: 'θ_r (after the filter)', y: Array.from(res.extras.thetaR || [], (v) => v * k), color: '--ref', dash: [6, 4], width: 1.5 },
          { label: 'θ', y: res.x.map((x) => x[1] * k), color: '--series-1' },
        ] },
      };
    },

    splane(ctx) { return { markers: markers(ctx, { withI: true }), legendNames: LOOP_NAMES, zetaRay: ctx.S.mode === 'explore' ? ctx.st.zetaTh : null }; },
    onPoleDrag: onDrag,

    math(ctx) {
      const st = ctx.st, Ts = ctx.S.sim.Ts, g = ctx.gains;
      const { beta, gamma } = WB.design.dirtyCoeffs(st.sigma, Ts);
      return [
        { title: 'Outer PID → saturation → filter → inner PD', page: 'p. 164–165 · Listing 10.3',
          theory: '\\theta_r = \\text{sat}_{\\theta_{max}}\\big(k_{Pz}e_z + k_{Iz}\\textstyle\\int e_z - k_{Dz}\\dot{\\hat z}\\big) \\xrightarrow{F(s)} \\theta_r,\\quad F = k_{P\\theta}(\\theta_r - \\theta) - k_{D\\theta}\\dot{\\hat\\theta}' },
        { title: 'Dirty derivative', page: 'p. 157 · Eq. 10.4',
          theory: '\\dot{\\hat y}[n] = \\frac{2\\sigma - T_s}{2\\sigma + T_s}\\dot{\\hat y}[n-1] + \\frac{2}{2\\sigma + T_s}\\big(y[n] - y[n-1]\\big)',
          numbers: `\\frac{2\\sigma - T_s}{2\\sigma + T_s} = ${tex(beta)},\\quad \\frac{2}{2\\sigma + T_s} = ${tex(gamma)}`, spoiler: true },
        { title: 'Anti-windup', page: 'p. 157 · §10.1.1, p. 165',
          theory: '\\text{integrate only while } |\\dot{\\hat z}| < \\bar v = 0.07\\ \\text{m/s}' },
        ...innerCards(ctx, g).slice(3), outerCards(ctx, g, { withI: true })[4],
        { title: 'Stability and the coefficients', page: 'p. 164',
          theory: '\\text{a stable polynomial has all its coefficients of one sign}' },
        { title: 'Sign of k_Iz', page: 'p. 164', answers: 'B.8/d1',
          theory: '(1 + k_{Dz})s^3 + (k_{Pz} - q k_{Dz})s^2 + (k_{Iz} - q k_{Pz})s - q k_{Iz}:\\; -q k_{Iz} > 0 \\Rightarrow k_{Iz} < 0 \\text{ (the listing uses } -0.05)',
          note: 'App. P.6 plots the locus versus k_Iz.' },
      ];
    },

    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch10;
      const ref = () => lib().slcGains(ctx.pModel, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, formula: 'book' });
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Parameters vary by up to 20%',
          html: 'Set the true plant in the left panel (<em>Randomize ±α</em> with α = 0.2). The chapter starts with a fixed 20% draw so the page is repeatable.',
          check: () => {
            const mis = Object.entries(ctx.S.mismatch || {});
            return mis.some(([, v]) => Math.abs(v) > 0) ? { ok: true, msg: `Mismatch: ${mis.map(([k, v]) => `${k} ${fmt(v, 3)}%`).join(', ')}.` } : { ok: false, msg: 'The true plant equals the model. Randomize it.' };
          },
        },
        {
          id: 'b', title: '(b) The controller knows only z, θ and r<sub>z</sub>',
          html: 'The PID here gets only the (noisy) measurements z, θ and the reference; ż and θ̇ come from dirty derivatives (c). In your <code>ctrlPID.py</code>, <code>update(r, y)</code> receives y, not the state.',
        },
        {
          id: 'c1', title: `(c) Gains for the listing's t<sub>r,θ</sub> = ${pr.trTh} s, ζ<sub>θ</sub> = ${pr.zetaTh}, M = ${pr.M}, ζ<sub>z</sub> = ${pr.zetaZ}`,
          inputs: { kPth: 'k<sub>Pθ</sub>', kDth: 'k<sub>Dθ</sub>', kDC: 'k<sub>DC</sub>', kPz: 'k<sub>Pz</sub>', kDz: 'k<sub>Dz</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { kPth: r.kPth, kDth: r.kDth, kDC: r.kDC, kPz: r.kPz, kDz: r.kDz }, { kPth: 'kPθ', kDth: 'kDθ', kDC: 'kDC', kPz: 'kPz', kDz: 'kDz' }); },
          actions: [WB.design.useGains(ctx, ['kPth', 'kDth', 'kPz', 'kDz'])],
          solution: () => { const r = ref(); return [{ tex: `k_{P\\theta} = ${tex(r.kPth)},\\; k_{D\\theta} = ${tex(r.kDth)},\\; k_{DC} = ${tex(r.kDC)},\\; k_{Pz} = ${tex(r.kPz)},\\; k_{Dz} = ${tex(r.kDz)}` }, { html: 'Listing 10.3 (p. 163–164). B.10 says "use B.8", but the listing changes t<sub>r,θ</sub> from 0.5 to 0.2 s and fixes M = 10.' }]; },
        },
        {
          id: 'c2', title: '(c) Dirty-derivative coefficients for σ = 0.05, T<sub>s</sub> = 0.01',
          inputs: { a: '(2σ−T<sub>s</sub>)/(2σ+T<sub>s</sub>)', b: '2/(2σ+T<sub>s</sub>)' },
          check: (v) => PD().checkNumbers(v, { a: 0.09 / 0.11, b: 2 / 0.11 }, {}),
          solution: () => [{ tex: '\\frac{0.09}{0.11} = 0.8182,\\quad \\frac{2}{0.11} = 18.18' }],
        },
        {
          id: 'c3', title: '(c) Tune k<sub>Iz</sub> to remove the steady-state error',
          html: 'Checks the current simulation with the plant mismatch in the left panel: |z − r| just before the second reference switch must be under 1 cm.',
          check: () => {
            const res = ctx.app.result(), S = ctx.S;
            const tSw = WB.sim.switchTime(S, 2);
            const i = WB.sim.indexBefore(S, res, tSw);
            const e = Math.abs(res.rAll[0][i] - res.yAll[0][i]);
            const msg = `|z − r| = ${fmt(e * 100, 3)} cm at t = ${fmt(res.t[i], 4)} s (kIz = ${fmt(ctx.gains.kIz, 3)}).`;
            if (!(Math.abs(ctx.gains.kIz) > 0)) return { ok: false, msg: 'kIz = 0. ' + msg };
            return { ok: e < 0.01, msg };
          },
          solution: () => [{ html: `The listing uses k<sub>Iz</sub> = ${pr.ki} (p. 163). It must be negative, like k<sub>Pz</sub> and k<sub>Dz</sub>: see the last math card, and App. P.6 for the locus.` }],
        },
      ]);
    },
  };

  // ---------------------------------------------------- Appendix P.6 (root locus) --
  const maxRe = (poly) => Math.max(...L.roots(poly).map((r) => r.re));

  B.chapters.p6 = {
    id: 'p6', num: 10.5, tab: 'App. P.6', short: 'P.6', title: 'Root locus vs. k_I', pages: 'pp. 465–472',

    defaults() { return { trTh: 0.2, zetaTh: 0.707, M: 10, zetaZ: 0.707, formula: 'book', kappa: 0, model: 'filter', filter: true, kMax: 1 }; },
    simDefaults(sys) { return sys.problems.p6.sim; },
    gains(ctx) { const g = designed(ctx, { ...ctx.st, kIz: -ctx.st.kappa }); return g; },
    controller(ctx, { linear = false } = {}) {
      return lib().slcPID({ g: ctx.gains, p: ctx.pModel, Ts: ctx.S.sim.Ts, uLim: ctx.sys.uLimit(ctx.pModel), filter: ctx.st.model === 'filter', linear });
    },
    linear: false,

    // Characteristic equation den(s) + κ num(s) = 0 with k_I = −κ.
    evans(ctx) {
      const p = ctx.pModel, g = designed(ctx, { ...ctx.st, kIz: 0 }), q = qOf(p);
      if (ctx.st.model === 'filter') {
        return { den: [1 + g.kDz, g.kPz - q * g.kDz, -q * g.kPz, 0], num: [-1, q], g };
      }
      const c = 2 * p.ell / 3;
      return { den: [-c * g.kDz, 1 / g.kDC - c * g.kPz, p.g * g.kDz, p.g * g.kPz, 0], num: [c, 0, -p.g], g };
    },
    // Largest κ that keeps every root in the LHP (bisection on the max real part).
    kappaCrit(ev) {
      const f = (k) => maxRe(L.polyAdd(ev.den, L.polyScale(ev.num, k)));
      const eps = 1e-6;
      if (f(eps) >= 0) return 0;
      let lo = eps, hi = 1e-3;
      while (f(hi) < 0 && hi < 1e4) { lo = hi; hi *= 2; }
      if (hi >= 1e4) return Infinity;
      for (let i = 0; i < 60; i++) { const mid = (lo + hi) / 2; if (f(mid) < 0) lo = mid; else hi = mid; }
      return lo;
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'B.10 PD gains, then add k_Iz', 'p. 471');
      segmented(sec, {
        label: 'Outer-loop model',
        options: [{ value: 'filter', label: 'k_DC + zero-canceling filter (B.8, B.10)' }, { value: 'book', label: 'k_DC only (Fig. 6-9)' }],
        ...bind(ctx, 'model'),
      });
      knobSliders(sec, ctx);
      slider(sec, { label: 'κ = −k<sub>Iz</sub>', min: 0, max: 1, step: 0.0005, sig: 3, ...bind(ctx, 'kappa') });
      slider(sec, { label: 'locus to κ =', min: 0.05, max: 10, log: true, sig: 3, ...bind(ctx, 'kMax') });
      sec.append(el('p', { class: 'muted small', text: 'k_Pz and k_Dz are negative for this plant, so the useful integrator gain is negative too; the locus is drawn for k_Iz = −κ, κ ≥ 0. Drag a closed-loop pole along it to set κ.' }));
      // the PD gains are the B.10 answers: show them only in Explore mode
      readout(sec, ctx, ctx.S.mode === 'explore' ? ['kPth', 'kDth', 'kDC', 'kPz', 'kDz', 'kIz'] : ['kIz']);
    },

    splane(ctx) {
      const ev = this.evans(ctx);
      // Work mode: the locus and the poles/zeros of L(s) answer B.P.6(a).
      const show = showsAnswer(ctx, 'B.P.6/a');
      const loci = show ? T.rootLocus(ev.den, ev.num, Math.max(ctx.st.kMax, ctx.st.kappa * 1.2)) : [];
      const mk = show ? L.roots(ev.den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of L(s) ${i + 1}` })) : [];
      if (show) (ev.num.length > 1 ? L.roots(ev.num) : []).forEach((z, i) => mk.push({ ...z, kind: 'olzero', label: `zero of L(s) ${i + 1}` }));
      L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, ctx.st.kappa))).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole at k_Iz = ${fmt(-ctx.st.kappa, 3)}`, dragId: i }));
      const fitR = Math.max(1, ...L.roots(ev.den).map((p) => Math.hypot(p.re, p.im))) * 1.6;
      return { markers: mk, loci, fitR };
    },

    onPoleDrag(ctx, id, re, im) {
      const ev = this.evans(ctx), kMax = ctx.st.kMax;
      let best = ctx.st.kappa, bd = Infinity;
      for (let i = 0; i <= 400; i++) {
        const k = kMax * (i / 400) ** 2;
        for (const q of L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, k)))) {
          const d = Math.hypot(q.re - re, q.im - im);
          if (d < bd) { bd = d; best = k; }
        }
      }
      ctx.st.kappa = best;
      ctx.update();
    },

    math(ctx) {
      const ev = this.evans(ctx), kc = this.kappaCrit(ev);
      const book = ctx.st.model === 'book';
      return [
        { title: 'Characteristic equation', page: 'p. 465',
          theory: '1 + P(s)C(s) = 0,\\quad C(s) = k_P + \\frac{k_I}{s} \\text{ on the error},\\; k_D s \\text{ on } y' },
        { title: 'Evans form', page: 'p. 466',
          theory: '\\Delta_{cl}(s) = 0 \\iff 1 + k\\,L(s) = 0 \\quad(k \\text{ is the gain that varies along the locus})' },
        { title: book ? 'Closed loop of the pendulum, k_DC only (Fig. 6-9)' : 'Closed loop of the pendulum with the filter (Fig. 8-17 + k_I/s)', page: book ? 'p. 471' : 'p. 126–127, p. 471', answers: 'B.P.6/a',
          theory: book
            ? '-\\tfrac{2}{3}\\ell k_{Dz}s^4 + \\Big(\\frac{1}{k_{DC\\theta}} - \\tfrac23\\ell k_{Pz}\\Big)s^3 + \\big(g k_{Dz} - \\tfrac23\\ell k_{Iz}\\big)s^2 + g k_{Pz}s + g k_{Iz} = 0'
            : '(1 + k_{Dz})s^3 + (k_{Pz} - q k_{Dz})s^2 + (k_{Iz} - q k_{Pz})s - q k_{Iz} = 0,\\quad q = \\sqrt{3g/2\\ell}' },
        { title: 'Evans form of the pendulum', page: 'p. 472', answers: 'B.P.6/a',
          theory: book
            ? '1 + k_{Iz}\\,\\frac{-\\frac23\\ell s^2 + g}{-\\frac23\\ell k_{Dz}s^4 + (\\frac{1}{k_{DC\\theta}} - \\frac23\\ell k_{Pz})s^3 + g k_{Dz}s^2 + g k_{Pz}s} = 0'
            : '1 + k_{Iz}\\,\\frac{s - q}{(1 + k_{Dz})s^3 + (k_{Pz} - q k_{Dz})s^2 - q k_{Pz}s} = 0',
          numbers: `\\Delta(s) = ${WB.tf.polyTex(ev.den)},\\quad k_{Iz} = -\\kappa:\\; \\Delta(s) + \\kappa\\,(${WB.tf.polyTex(ev.num)}) = 0` },
        { title: 'Stable range', page: 'computed (not in the book)',
          theory: '\\text{largest } \\kappa = -k_{Iz} \\text{ with every closed-loop pole in the LHP}',
          numbers: kc === 0 ? '\\text{unstable already at } k_{Iz} = 0' : `\\kappa_{crit} = ${isFinite(kc) ? tex(kc) : '\\infty'}`, spoiler: true,
          note: book ? 'With the B.10 PD gains, Fig. 6-9\'s model (k_DC but no filter) has a sign change in its coefficients at k_Iz = 0, so it is unstable for every k_Iz. That loop is the one the filter was added to fix (ISSUES.md).' : 'The filter turns the outer loop into (s − q)/s² · C(s), the model B.8 designed k_Pz, k_Dz for.' },
      ];
    },

    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.p6;
      const evF = () => this.evans({ ...ctx, st: { ...ctx.st, model: 'filter' } });
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: 'Closed-loop characteristic equation in Evans form',
          html: 'Add k<sub>Iz</sub>/s to the outer controller as B.8(d) designed it (inner loop as k<sub>DCθ</sub>, plus the zero-canceling filter). Write L(s) for 1 + k<sub>Iz</sub>L(s) = 0 in terms of the outer PD gains; the check calls it at complex s.',
          code: {
            template: 'def L(s, kP, kD):\n    # 1 + kI * L(s) = 0\n    return ...\n',
            check: pyCheck(ctx, {
              items: [{ fn: 'L', args: ['s', 'kPz', 'kDz'], truth: (p, a) => {
                const cx = WB.py.cx, den = [...outerPoly({ pModel: p }, { kPz: a.kPz, kDz: a.kDz }, true), 0];
                return cx.div(cx.add(a.s, -qOf(p)), cx.poly(den, a.s));
              } }],
            }),
          },
          solution: () => { const ev = evF(); return [
            { tex: '(1 + k_{Dz})s^3 + (k_{Pz} - q k_{Dz})s^2 + (k_{Iz} - q k_{Pz})s - q k_{Iz} = 0 \\;\\Rightarrow\\; L(s) = \\frac{s - q}{(1 + k_{Dz})s^3 + (k_{Pz} - q k_{Dz})s^2 - q k_{Pz}s},\\quad q = \\sqrt{3g/2\\ell}' },
            { tex: `\\text{current PD gains: } L(s) = \\frac{s - ${tex(ev.num[1])}}{${WB.tf.polyTex(ev.den)}}` },
            { code: 'q = np.sqrt(3 * P.g / (2 * P.ell))\n\ndef L(s, kP, kD):\n    return (s - q) / ((1 + kD) * s**3\n                      + (kP - q * kD) * s**2\n                      - q * kP * s)' },
            { html: 'The book writes the Fig. 6-9 version without the filter (p. 471–472); switch the model to compare. With the B.10 gains that loop is unstable for every k<sub>Iz</sub> (ISSUES.md).' },
          ]; },
        },
        {
          id: 'b', title: 'Root locus versus k<sub>Iz</sub>: largest stable |k<sub>Iz</sub>|',
          html: 'The locus is drawn on the s-plane once the Evans form is solved, for k<sub>Iz</sub> = −κ (filter model).',
          inputs: { k: 'κ<sub>crit</sub>' },
          check: (v) => PD().checkNumbers(v, { k: this.kappaCrit(evF()) }, { k: 'κcrit' }),
          solution: () => [{ tex: `\\kappa_{crit} = ${tex(this.kappaCrit(evF()))}` }],
        },
        {
          id: 'c', title: 'Select k<sub>Iz</sub> that does not significantly change the other closed-loop poles',
          html: 'Checks the current κ on the filter model: the complex pair must stay within 10% (in |p|) of the PD-only pair, and the integrator pole must be slower than the pair.',
          check: () => {
            const ev = evF();
            if (!(ctx.st.kappa > 0)) return { ok: false, msg: 'Set κ > 0.' };
            const pd = L.roots(ev.den.slice(0, -1));
            const cl = L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, ctx.st.kappa)));
            const cpx = cl.filter((p) => Math.abs(p.im) > 1e-6), real = cl.filter((p) => Math.abs(p.im) <= 1e-6);
            if (cpx.length !== 2) return { ok: false, msg: 'The pair has split into real poles; κ is too large.' };
            const pdc = pd.find((p) => Math.abs(p.im) > 1e-6) || pd[0];
            const ratio = Math.hypot(cpx[0].re, cpx[0].im) / Math.hypot(pdc.re, pdc.im);
            const slow = real.length === 1 && real[0].re < 0 && Math.abs(real[0].re) < Math.abs(pdc.re);
            return { ok: Math.abs(ratio - 1) < 0.1 && slow, msg: `|p| ratio ${fmt(ratio, 3)}, integrator pole ${real.length ? fmtPole(real[0]) : '—'}.` };
          },
          solution: () => [{ html: 'With the B.10 gains, κ ≲ 0.02 keeps the pair within 10% (κ = 0.01: |p| ratio 0.96, integrator pole at −0.05). The listing\'s k<sub>Iz</sub> = −0.05 moves the poles more: the pair shrinks to 0.73 of its PD size and the integrator pole sits at −0.44. That trades a less damped, slower pair for faster removal of the steady-state error. The book stops at the rlocus command (p. 472).' }],
        },
      ]);
    },
  };

  B.slc = { innerPoles, outerPoles, outerPoly, fullPoles, markers, LOOP_NAMES, qOf };
})();
