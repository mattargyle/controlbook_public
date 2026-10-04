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
    const mk = M.roots2(0, pin.a0).map((x, i) => ({ ...x, kind: 'ol', label: `open-loop pole of P_in ${i + 1}` }));
    mk.push({ re: 0, im: 0, kind: 'ol', label: 'double pole of P_out at 0' });
    mk.push({ re: -q, im: 0, kind: 'olzero', label: 'LHP zero of P_out (cancelled by the filter)' }, { re: q, im: 0, kind: 'olzero', label: 'RHP zero of P_out' });
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
      { title: 'Inner-loop plant (b = 0)', page: 'p. 124 · Fig. 8-14',
        theory: 'P_{in}(s) = \\frac{-\\frac{1}{m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3}}}{s^2 - \\frac{(m_1+m_2)g}{m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3}}} = \\frac{b_0}{s^2 + a_0}',
        numbers: `b_0 = ${tex(pin.b0)},\\quad a_0 = ${tex(pin.a0)}`, spoiler: true },
      { title: 'Inner closed loop', page: 'p. 125',
        theory: '\\Delta_{cl}(s) = s^2 + b_0 k_{D\\theta}\\,s + (a_0 + b_0 k_{P\\theta})\\quad \\text{(Fig. 8-14: } k_P \\text{ on the error, } k_D s \\text{ on } \\theta)',
        symbolic: '\\Delta_{cl}(s) = s^2 - \\frac{k_{D\\theta}}{J}s - \\Big(\\frac{(m_1+m_2)g}{J} + \\frac{k_{P\\theta}}{J}\\Big),\\quad J = m_1\\tfrac{\\ell}{6} + m_2\\tfrac{2\\ell}{3}',
        numbers: `\\Delta_{cl} = s^2 + ${tex(pin.b0 * g.kDth)}\\,s + ${tex(pin.a0 + pin.b0 * g.kPth)},\\quad p = ${texPole(ip[0])},\\; ${texPole(ip[1])}`, spoiler: true },
      { title: 'Inner gains from the spec', page: 'p. 125',
        theory: '\\Delta_{cl} = s^2 + 2\\zeta\\omega_n s + \\omega_n^2,\\; \\omega_n = \\frac{2.2}{t_r}:\\quad k_P = \\frac{\\omega_n^2 - a_0}{b_0},\\; k_D = \\frac{2\\zeta\\omega_n}{b_0},\\quad k_{DC} = \\frac{\\Theta}{R_\\Theta}\\Big|_{s=0}',
        symbolic: 'k_{P\\theta} = -(m_1+m_2)g - J\\,\\omega_{n\\theta}^2,\\quad k_{D\\theta} = -2\\zeta_\\theta\\omega_{n\\theta} J,\\quad k_{DC\\theta} = \\frac{k_{P\\theta}}{(m_1+m_2)g + k_{P\\theta}}',
        numbers: `k_{P\\theta} = ${tex(g.kPth)},\\quad k_{D\\theta} = ${tex(g.kDth)},\\quad k_{DC\\theta} = ${tex(g.kDC)}`, spoiler: true },
    ];
  }
  function outerCards(ctx, g, { withI = false } = {}) {
    const p = ctx.pModel, q = qOf(p), zf = lib().zcFilter(p, g.kDC, ctx.S.sim.Ts);
    const op = outerPoles(ctx, g, ctx.st.filter, withI);
    return [
      { title: 'Outer loop with the inner loop as k_DC', page: 'p. 125–126 · Fig. 8-15',
        theory: '\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2} = -\\frac{2\\ell}{3}\\frac{(s + \\sqrt{3g/2\\ell})(s - \\sqrt{3g/2\\ell})}{s^2}',
        numbers: `\\sqrt{3g/2\\ell} = ${tex(q)}`, spoiler: true,
        note: 'A PD controller cannot match a third-order characteristic polynomial to s² + 2ζωₙs + ωₙ², so the LHP zero is cancelled first (p. 126).' },
      { title: 'Zero-canceling filter (with gain)', page: 'p. 126 · Fig. 8-16',
        theory: 'F(s) = \\frac{k_F}{s + p_F} \\text{ cancels the LHP zero and the gains } k_{DC}\\text{ and } -\\tfrac{2\\ell}{3}',
        symbolic: 'F(s) = \\frac{-\\frac{1}{k_{DC\\theta}}\\frac{3}{2\\ell}}{s + \\sqrt{3g/2\\ell}} \\;\\Rightarrow\\; k_{DC}\\,F(s)\\,\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{s - \\sqrt{3g/2\\ell}}{s^2}',
        numbers: `F(s) = \\frac{${tex(zf.a)}}{s + ${tex(zf.b)}}`, spoiler: true,
        note: ctx.st.filter ? 'Implemented with one Euler step per sample (zeroCancelingFilter, Listing 8.3).' : 'The filter is off: θ_r is the PD output itself, and the outer loop is third order.' },
      { title: 'Outer gains', page: 'p. 127 · Eq. 8.12–8.13',
        theory: '\\text{close PD } (k_{Pz} \\text{ on the error, } k_{Dz}s \\text{ on } z) \\text{ around } \\frac{s - q}{s^2} \\text{ and match } s^2 + 2\\zeta_z\\omega_{nz}s + \\omega_{nz}^2',
        symbolic: 'a = \\frac{k_{Pz}}{1 + k_{Dz}} = -\\sqrt{\\tfrac{2\\ell}{3g}}\\,\\omega_{nz}^2,\\quad b = \\frac{k_{Dz}}{1 + k_{Dz}} = \\sqrt{\\tfrac{2\\ell}{3g}}\\big(a - 2\\zeta_z\\omega_{nz}\\big),\\quad k_{Dz} = \\frac{b}{1 - b},\\; k_{Pz} = \\frac{a}{1 - b}',
        numbers: `k_{Pz} = ${tex(g.kPz)},\\quad k_{Dz} = ${tex(g.kDz)}${withI ? `,\\quad k_{Iz} = ${tex(g.kIz)}` : ''},\\quad p_{out} = ${op.map((x) => texPole(x)).join(',\\;')}`, spoiler: true,
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
        sec.append(el('p', { class: 'muted small', text: 'Both inner gains are negative because P_in has negative gain. Dashed rings in the s-plane mark the problem\'s target poles (inner t_r = 0.5 s; outer M = 10).' }));
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
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Block diagram',
          html: 'Sketch it, then compare with the solution. The animation shows r<sub>z</sub> on the track; the extra plot shows r<sub>θ</sub>.',
          solution: () => [{ html: 'Outer: r<sub>z</sub> − z → PD (k<sub>Pz</sub> on the error, k<sub>Dz</sub>s on z) → r<sub>θ</sub>. Inner: r<sub>θ</sub> − θ → PD (k<sub>Pθ</sub>, k<sub>Dθ</sub>s on θ) → F → pendulum.' }, { html: 'Figs. 8-14 to 8-16 (pp. 124–126). In the final design the outer loop is PD followed by the low-pass filter with gain, and the inner loop is replaced by k<sub>DCθ</sub> for the outer design.' }],
        },
        {
          id: 'b', title: `(b) Inner gains for t<sub>r,θ</sub> = ${pr.trTh} s, ζ<sub>θ</sub> = ${pr.zetaTh}`,
          inputs: { kPth: 'k<sub>Pθ</sub>', kDth: 'k<sub>Dθ</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { kPth: r.kPth, kDth: r.kDth }, { kPth: 'kPθ', kDth: 'kDθ' }); },
          solution: () => { const r = ref(); return [{ tex: `\\omega_{n\\theta} = 2.2/${pr.trTh} = ${tex(r.wnTh)},\\quad k_{P\\theta} = ${tex(r.kPth)},\\quad k_{D\\theta} = ${tex(r.kDth)}` }, { html: 'Book: −26.0 and −4.41 (p. 125).' }]; },
        },
        {
          id: 'c', title: '(c) Inner-loop DC gain',
          inputs: { kDC: 'k<sub>DCθ</sub>' },
          check: (v) => PD().checkNumbers(v, { kDC: ref().kDC }, { kDC: 'kDC' }),
          solution: () => [{ tex: `k_{DC\\theta} = \\frac{k_{P\\theta}}{(m_1+m_2)g + k_{P\\theta}} = ${tex(ref().kDC)}` }, { html: 'Book: 1.89 (p. 125). It is not 1: a PD inner loop on an unstable plant leaves a steady-state angle error, which the filter gain compensates.' }],
        },
        {
          id: 'd', title: `(d) Filter and outer gains (this page uses M = ${pr.M}, ζ<sub>z</sub> = ${pr.zetaZ}; the book leaves the outer specs open)`,
          html: 'Filter F(s) = k<sub>F</sub>/(s + p<sub>F</sub>).',
          inputs: { kF: 'k<sub>F</sub>', pF: 'p<sub>F</sub>', kPz: 'k<sub>Pz</sub>', kDz: 'k<sub>Dz</sub>' },
          check: (v) => { const r = ref(), zf = lib().zcFilter(ctx.pModel, r.kDC, 0.01); return PD().checkNumbers(v, { kF: zf.a, pF: zf.b, kPz: r.kPz, kDz: r.kDz }, { kF: 'kF', pF: 'pF', kPz: 'kPz', kDz: 'kDz' }); },
          actions: [WB.design.useGains(ctx, ['kPz', 'kDz'])],
          solution: () => {
            const r = ref(), zf = lib().zcFilter(ctx.pModel, r.kDC, 0.01);
            return [
              { tex: `F(s) = \\frac{${tex(zf.a)}}{s + ${tex(zf.b)}},\\quad \\omega_{nz} = \\frac{2.2}{${pr.M}\\cdot ${pr.trTh}} = ${tex(r.wnZ)}` },
              { tex: `a = ${tex(r.a)},\\; b = ${tex(r.b)} \\Rightarrow k_{Dz} = \\frac{b}{1-b} = ${tex(r.kDz)},\\; k_{Pz} = \\frac{a}{1-b} = ${tex(r.kPz)}` },
              { html: 'Eqs. 8.12–8.13 (p. 127). Listing 8.3 computes k<sub>Dz</sub> with a different expression; try "Listing 8.3 tuning" in Explore mode and compare the outer ζ.' },
            ];
          },
        },
        {
          id: 'e', title: '(e) Balance from θ(0) = 10° with |F| ≤ 5 N',
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
      const a = this.analysis(ctx), g = ctx.gains;
      return [
        { title: 'Inner loop with a disturbance', page: 'p. 148 · Fig. 9-9',
          theory: 'P(s)C(s) = \\frac{-\\frac{1}{J}}{s^2 - \\frac{(m_1+m_2)g}{J}}(k_D s + k_P)',
          symbolic: '\\text{no free integrator} \\Rightarrow \\text{type 0 for tracking and for an input disturbance}', spoiler: true },
        { title: 'Inner-loop errors', page: 'p. 148–149',
          theory: 'e_{step} = \\frac{1}{1 + M_p},\\; M_p = \\lim_{s\\to0}PC,\\quad e_{d} = \\lim_{s\\to0} s\\frac{P}{1 + PC}\\frac{1}{s}',
          symbolic: 'e_{step} = \\frac{1}{1 + \\frac{k_P}{(m_1+m_2)g}} = 1 - k_{DC},\\quad e_{ramp} = \\infty,\\quad e_{d,step} = \\frac{1}{(m_1+m_2)g + k_P}',
          numbers: `e_{step} = ${tex(a.inStep)},\\quad e_{d,step} = ${tex(a.inDist)}`, spoiler: true },
        { title: 'Outer loop (book model, Fig. 9-10)', page: 'p. 149',
          theory: 'P(s)C(s) = \\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2}\\cdot\\frac{k_D s^2 + k_P s + k_I}{s},\\quad M_a = \\lim_{s\\to0}s^2PC',
          symbolic: 'k_I = 0 \\Rightarrow \\text{type 2},\\; e_{parab} = \\frac{1}{k_P g};\\quad k_I \\ne 0 \\Rightarrow \\text{type 3}',
          numbers: a.hasI ? '\\text{type 3: zero error to a step, ramp and parabola}' : `M_a = g\\,k_{Pz} = ${tex(a.Ma)},\\quad e_{parab} = ${tex(1 / a.Ma)}`, spoiler: true,
          note: 'Fig. 9-10 drops k_DC and the filter. With them (Fig. 8-17) the loop is (s − q)/s² · C, which changes M_a but not the type.' },
        { title: 'Outer-loop disturbance', page: 'p. 150',
          theory: 'e = \\lim_{s\\to0} s\\frac{P}{1+PC}\\,\\frac{1}{s^{q+1}}',
          symbolic: 'k_I = 0:\\; e = \\frac{1}{k_P}\\;(\\text{type 0}),\\quad k_I \\ne 0:\\; \\text{type 1, ramp error } \\frac{1}{k_I}',
          numbers: a.hasI ? `1/k_{Iz} = ${tex(a.outDistI)}` : `1/k_{Pz} = ${tex(a.outDistPD)}`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const a = () => this.analysis(ctx);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch9, [
        {
          id: 'a', title: '(a) Inner loop under PD (current gains)',
          html: 'Type for tracking, the step error per unit r<sub>θ</sub>, the disturbance type, and the error per unit step disturbance [rad/N].',
          inputs: { type: 'type', step: 'e<sub>step</sub>', dtype: 'd type', dstep: 'e<sub>d</sub>' },
          check: (v) => PD().checkNumbers(v, { type: 0, step: a().inStep, dtype: 0, dstep: a().inDist }, { step: 'e_step', dstep: 'e_d' }),
          solution: () => [
            { tex: `\\text{type 0},\\; e_{step} = \\frac{1}{1 + k_P/(m_1+m_2)g} = ${tex(a().inStep)},\\; e_{ramp} = e_{parab} = \\infty` },
            { tex: `\\text{disturbance: type 0},\\; e = \\frac{1}{(m_1+m_2)g + k_P} = ${tex(a().inDist)}` },
          ],
        },
        {
          id: 'b1', title: '(b) Outer loop, PD (set k<sub>Iz</sub> = 0)',
          html: 'Type, the parabola error 1/M<sub>a</sub> with the book\'s model (Fig. 9-10), and the error per unit step disturbance.',
          inputs: { type: 'type', parab: 'e<sub>parab</sub>', dist: 'e<sub>d</sub>' },
          check: (v) => {
            if (Math.abs(ctx.gains.kIz) > 0) return { ok: false, msg: 'Set kIz = 0 first.' };
            return PD().checkNumbers(v, { type: 2, parab: 1 / a().Ma, dist: a().outDistPD }, { parab: 'e_parab', dist: 'e_d' });
          },
          solution: () => [{ tex: `\\text{type 2},\\; e_{step} = e_{ramp} = 0,\\; e_{parab} = \\frac{1}{k_P g} = ${tex(1 / a().Ma)},\\; e_d = \\frac{1}{k_P} = ${tex(a().outDistPD)}` }],
        },
        {
          id: 'b2', title: '(b) Outer loop, PID (k<sub>Iz</sub> ≠ 0)',
          html: 'Tracking type, disturbance type, and the error to a unit-ramp disturbance.',
          inputs: { type: 'type', dtype: 'd type', dramp: 'e<sub>d,ramp</sub>' },
          check: (v) => {
            if (!(Math.abs(ctx.gains.kIz) > 0)) return { ok: false, msg: 'Set kIz ≠ 0 first.' };
            return PD().checkNumbers(v, { type: 3, dtype: 1, dramp: a().outDistI }, { dramp: 'e_d,ramp' });
          },
          solution: () => [{ tex: `\\text{type 3 (zero error to step, ramp, parabola)};\\; \\text{disturbance type 1},\\; e_{d,ramp} = \\frac{1}{k_I} = ${tex(a().outDistI)}` }, { html: 'p. 149–150.' }],
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
        ...innerCards(ctx, g).slice(2), ...outerCards(ctx, g, { withI: true }).slice(2),
        { title: 'Sign of k_Iz', page: 'p. 164',
          theory: '\\text{a stable polynomial has all its coefficients of one sign}',
          symbolic: '(1 + k_{Dz})s^3 + (k_{Pz} - q k_{Dz})s^2 + (k_{Iz} - q k_{Pz})s - q k_{Iz}:\\; -q k_{Iz} > 0 \\Rightarrow k_{Iz} < 0 \\text{ (the listing uses } -0.05)', spoiler: true,
          note: 'App. P.6 plots the locus versus k_Iz.' },
      ];
    },

    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch10;
      const ref = () => lib().slcGains(ctx.pModel, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, formula: 'book' });
      PD().problemPanel(parent, ctx, pr, [
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

    defaults() { return { trTh: 0.2, zetaTh: 0.707, M: 10, zetaZ: 0.707, formula: 'book', kappa: 0.01, model: 'filter', filter: true, kMax: 1 }; },
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
      const loci = T.rootLocus(ev.den, ev.num, Math.max(ctx.st.kMax, ctx.st.kappa * 1.2));
      const mk = L.roots(ev.den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of L(s) ${i + 1}` }));
      (ev.num.length > 1 ? L.roots(ev.num) : []).forEach((z, i) => mk.push({ ...z, kind: 'olzero', label: `zero of L(s) ${i + 1}` }));
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
        { title: book ? 'Closed loop, k_DC only (Fig. 6-9)' : 'Closed loop with the filter (Fig. 8-17 + k_I/s)', page: book ? 'p. 471' : 'p. 126–127, p. 471',
          theory: '1 + P(s)C(s) = 0,\\quad C(s) = k_{Pz} + \\frac{k_{Iz}}{s} \\text{ on the error},\\; k_{Dz}s \\text{ on } z', spoiler: true,
          symbolic: book
            ? '-\\tfrac{2}{3}\\ell k_{Dz}s^4 + \\Big(\\frac{1}{k_{DC\\theta}} - \\tfrac23\\ell k_{Pz}\\Big)s^3 + \\big(g k_{Dz} - \\tfrac23\\ell k_{Iz}\\big)s^2 + g k_{Pz}s + g k_{Iz} = 0'
            : '(1 + k_{Dz})s^3 + (k_{Pz} - q k_{Dz})s^2 + (k_{Iz} - q k_{Pz})s - q k_{Iz} = 0,\\quad q = \\sqrt{3g/2\\ell}' },
        { title: 'Evans form', page: 'p. 472',
          theory: '1 + k_{Iz}\\,L(s) = 0',
          symbolic: book
            ? '1 + k_{Iz}\\,\\frac{-\\frac23\\ell s^2 + g}{-\\frac23\\ell k_{Dz}s^4 + (\\frac{1}{k_{DC\\theta}} - \\frac23\\ell k_{Pz})s^3 + g k_{Dz}s^2 + g k_{Pz}s} = 0'
            : '1 + k_{Iz}\\,\\frac{s - q}{(1 + k_{Dz})s^3 + (k_{Pz} - q k_{Dz})s^2 - q k_{Pz}s} = 0',
          numbers: `\\Delta(s) = ${WB.tf.polyTex(ev.den)},\\quad k_{Iz} = -\\kappa:\\; \\Delta(s) + \\kappa\\,(${WB.tf.polyTex(ev.num)}) = 0`, spoiler: true },
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
          id: 'a', title: 'Evans form with the filter: L(s) = (s − q)/(d<sub>3</sub>s³ + d<sub>2</sub>s² + d<sub>1</sub>s)',
          html: 'Uses the PD gains from the current t<sub>r,θ</sub>, ζ, M (B.10: 0.2 s, 0.707, 10, 0.707).',
          inputs: { q: 'q', d3: 'd<sub>3</sub>', d2: 'd<sub>2</sub>', d1: 'd<sub>1</sub>' },
          check: (v) => { const ev = evF(); return PD().checkNumbers(v, { q: ev.num[1], d3: ev.den[0], d2: ev.den[1], d1: ev.den[2] }, {}); },
          solution: () => { const ev = evF(); return [{ tex: `L(s) = \\frac{s - ${tex(ev.num[1])}}{${WB.tf.polyTex(ev.den)}}` }, { html: 'The book writes the Fig. 6-9 version without the filter (p. 471–472). Switch the model to compare.' }]; },
        },
        {
          id: 'b', title: 'Largest stable |k<sub>Iz</sub>| (filter model)',
          inputs: { k: 'κ<sub>crit</sub>' },
          check: (v) => PD().checkNumbers(v, { k: this.kappaCrit(evF()) }, { k: 'κcrit' }),
          solution: () => [{ tex: `\\kappa_{crit} = ${tex(this.kappaCrit(evF()))}` }],
        },
        {
          id: 'c', title: 'Pick k<sub>Iz</sub> that barely moves the outer PD poles',
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
