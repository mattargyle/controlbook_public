// D.11-D.14: full state feedback, integral augmentation, observers and the
// disturbance observer on the (already linear) mass-spring-damper
//   xdot = A x + B u,  y = C x,  A = [0 1; -k/m -b/m],  B = [0; 1/m],  C = [1 0].
// The equilibrium is z_e = 0, F_e = 0, so no offsets are needed. Discretization
// follows the repo's ctrl*.py: dirty derivative Eq. 10.4, trapezoid integrator,
// observer integrated with RK4 over one Ts using the previous (saturated) input.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const lib = WB.studies.D.lib;
  const ans = WB.systems.D.answers;
  const CH = WB.studies.D.chapters;
  const D = WB.design;

  // --------------------------------------------------------------- design --
  const ctrlPoles = (st) => D.polesFromWnZeta(2.2 / st.tr, st.zeta);
  const obsPoles = (st) => D.polesFromWnZeta(st.wnObs, st.zetaObs);

  // Explore-mode gains from the knobs, by numerical pole placement (WB.design.place,
  // Ackermann). answers.* holds the closed forms; regress_D.py checks both agree.
  function design(ctx, st = ctx.st, level = ctx.level) {
    const { A, B, C } = ctx.ss;
    const out = { poles: ctrlPoles(st) };
    if (level === 'sf') {
      out.K = D.place(A, B, out.poles) || [NaN, NaN];
      out.kr = D.refGain(A, B, C, out.K);
      return out;
    }
    const { A1, B1 } = D.augmentIntegrator(A, B, C);
    out.poles = [...out.poles, { re: st.pI, im: 0 }];
    const K1 = D.place(A1, B1, out.poles) || [NaN, NaN, NaN];
    out.K = [K1[0], K1[1]]; out.ki = K1[2];
    if (level === 'obs') {
      out.obsPoles = obsPoles(st);
      const Lg = D.observerGain(A, C, out.obsPoles);
      out.L = Lg ? [Lg[0][0], Lg[1][0]] : [NaN, NaN];
    }
    if (level === 'dobs') {
      const { A2, C2 } = D.augmentDisturbance(A, B, C);
      out.obsPoles = [...obsPoles(st), { re: st.pD, im: 0 }];
      const Lg = D.observerGain(A2, C2, out.obsPoles);
      out.L = Lg ? [Lg[0][0], Lg[1][0]] : [NaN, NaN];
      out.Ld = Lg ? Lg[2][0] : NaN;
    }
    return out;
  }
  function gainsFor(ctx) {
    if (ctx.S.mode === 'explore') return design(ctx);
    const w = ctx.st.w;
    return { K: [w.K1, w.K2], kr: w.kr, ki: w.ki, L: [w.L1, w.L2], Ld: w.Ld };
  }

  // ----------------------------------------------------------- controller --
  // level: 'sf' (D.11), 'sfi' (D.12), 'obs' (D.13), 'dobs' (D.14)
  function makeSS(ctx, { linear = false } = {}) {
    const st = ctx.st, level = ctx.level, g = ctx.gains, p = ctx.pModel;
    const { A, B, C } = ctx.ss;
    const Ts = ctx.S.sim.Ts, uLim = p.Fmax;
    const useObs = level === 'obs' || level === 'dobs';
    const useDO = level === 'dobs' && st.dobs;
    const sigma = st.sigma ?? 0.05;
    const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    let I = 0, ePrev = 0, yPrev = null, ydot = 0, uPrev = 0;
    let xh = [st.xhat0 || 0, 0], dh = 0;

    // xhat' = A xhat + B (u + dhat) + L (y - C xhat),  dhat' = Ld (y - C xhat)
    function fObs(z, y) {
      const innov = y - (C[0][0] * z[0] + C[0][1] * z[1]);
      const u = uPrev + (useDO ? z[2] : 0);
      return [
        A[0][0] * z[0] + A[0][1] * z[1] + B[0][0] * u + g.L[0] * innov,
        A[1][0] * z[0] + A[1][1] * z[1] + B[1][0] * u + g.L[1] * innov,
        useDO ? g.Ld * innov : 0,
      ];
    }

    return {
      update(r, x, yMeas) {
        let xu;
        if (useObs) {
          const z = M.rk4Step((zz) => fObs(zz, yMeas), [xh[0], xh[1], dh], 0, Ts);
          xh = [z[0], z[1]]; dh = z[2];
          xu = xh;
        } else if (st.est === 'dirty') {
          if (yPrev === null) yPrev = yMeas;
          ydot = beta * ydot + gamma * (yMeas - yPrev);
          yPrev = yMeas;
          xu = [yMeas, ydot];
        } else {
          xu = x;
        }
        const Kx = g.K[0] * xu[0] + g.K[1] * xu[1];
        let u;
        if (level === 'sf') {
          u = -Kx + g.kr * r;
        } else {
          const e = r - xu[0];
          const Itry = I + (Ts / 2) * (e + ePrev);
          const uTry = -Kx - g.ki * Itry - (useDO ? dh : 0);
          // anti-windup (D.12a): hold the integrator while the actuator is saturated
          if (!(st.antiwindup === 'clamp' && !linear && Math.abs(uTry) > uLim)) I = Itry;
          ePrev = e;
          u = -Kx - g.ki * I - (useDO ? dh : 0);
        }
        uPrev = linear ? u : M.saturate(u, uLim);
        return { u, xhat0: xu[0], xhat1: xu[1], dhat: dh, integrator: I };
      },
    };
  }

  // ------------------------------------------------------------- controls --
  function tuningSliders(parent, ctx, { pI } = {}) {
    slider(parent, { label: 't<sub>r</sub>', unit: 's', min: 0.2, max: 6, step: 0.005, sig: 4, ...bind(ctx, 'tr') });
    slider(parent, { label: 'ζ', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zeta') });
    if (pI) slider(parent, { label: 'p<sub>I</sub>', min: -10, max: -0.05, step: 0.01, sig: 3, ...bind(ctx, 'pI') });
  }
  function obsSliders(parent, ctx, withD) {
    slider(parent, { label: 'ω<sub>n,obs</sub>', unit: 'rad/s', min: 0.5, max: 40, step: 0.05, sig: 4, ...bind(ctx, 'wnObs') });
    slider(parent, { label: 'ζ<sub>obs</sub>', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zetaObs') });
    if (withD) slider(parent, { label: 'p<sub>d</sub>', min: -30, max: -0.1, step: 0.05, sig: 3, ...bind(ctx, 'pD') });
  }
  const SPEC = {
    K1: ['K<sub>1</sub>', 0, 20], K2: ['K<sub>2</sub>', 0, 30], kr: ['k<sub>r</sub>', 0, 30], ki: ['k<sub>i</sub>', -30, 0],
    L1: ['L<sub>1</sub>', 0, 60], L2: ['L<sub>2</sub>', 0, 600], Ld: ['L<sub>d</sub>', 0, 300],
  };
  const workSliders = (parent, ctx, keys) => WB.ui.gainSliders(parent, ctx, SPEC, keys, { steps: 2000 });
  function gainsReadout(parent, ctx, keys) {
    lib.readout(parent, ctx, keys, () => {
      const g = ctx.gains;
      return { K1: g.K[0], K2: g.K[1], kr: g.kr, ki: g.ki, L1: g.L && g.L[0], L2: g.L && g.L[1], Ld: g.Ld };
    });
  }
  function estControl(parent, ctx) {
    segmented(parent, {
      label: 'Where ż comes from',
      options: [{ value: 'dirty', label: 'dirty derivative of z (D.11e)' }, { value: 'true', label: 'true state' }],
      ...bind(ctx, 'est'),
    });
  }
  function awControl(parent, ctx) {
    segmented(parent, {
      label: 'Anti-windup (D.12a)',
      options: [{ value: 'clamp', label: 'hold integrator while saturated' }, { value: 'none', label: 'none' }],
      ...bind(ctx, 'antiwindup'),
    });
  }

  // ------------------------------------------------------------- analysis --
  function closedLoopPoles(ctx) {
    const { A, B, C } = ctx.ss, g = ctx.gains;
    if (ctx.level === 'sf') return L.eig(L.sub(A, L.mul(B, [g.K])));
    const { A1, B1 } = D.augmentIntegrator(A, B, C);
    return L.eig(L.sub(A1, L.mul(B1, [[g.K[0], g.K[1], g.ki]])));
  }
  function observerPoles(ctx) {
    const { A, B, C } = ctx.ss, g = ctx.gains;
    if (ctx.level === 'obs') return L.eig(L.sub(A, L.mul([[g.L[0]], [g.L[1]]], C)));
    if (ctx.level === 'dobs') {
      const { A2, C2 } = D.augmentDisturbance(A, B, C);
      return L.eig(L.sub(A2, L.mul([[g.L[0]], [g.L[1]], [g.Ld]], C2)));
    }
    return [];
  }
  function markers(ctx) {
    const explore = ctx.S.mode === 'explore';
    // The open-loop poles answer D.7(a): hidden in Work mode until it is solved.
    const mk = lib.showOl(ctx) ? L.eig(ctx.ss.A).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole ${i + 1}` })) : [];
    closedLoopPoles(ctx).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `controller pole ${i + 1}`, dragId: explore ? (Math.abs(p.im) > 1e-9 ? 0 : 2) : undefined }));
    observerPoles(ctx).forEach((p, i) => mk.push({ ...p, kind: 'obs', label: `observer pole ${i + 1}`, dragId: explore ? (Math.abs(p.im) > 1e-9 ? 10 : 12) : undefined }));
    return mk;
  }
  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    if (id === 0) {
      const wn = Math.hypot(re, im);
      st.zeta = Math.max(0.2, Math.min(1, -re / wn)); st.tr = 2.2 / wn;
    } else if (id === 2) {
      if (ctx.level === 'sf') { const wn = Math.abs(re); st.zeta = 1; st.tr = 2.2 / wn; } else st.pI = re;
    } else if (id === 10) {
      st.wnObs = Math.hypot(re, im); st.zetaObs = Math.max(0.2, Math.min(1, -re / st.wnObs));
    } else if (id === 12 && ctx.level === 'dobs') st.pD = re;
    ctx.update();
  }

  // ----------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B } = ctx.ss;
    return {
      title: 'State-space model (D.6)', page: 'p. 379 · D.6', answers: 'D.6/a',
      theory: 'A = \\begin{bmatrix}0 & 1\\\\ -\\frac km & -\\frac bm\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\ \\frac1m\\end{bmatrix},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix}',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`,
    };
  }
  // The controllability matrix answers D.11(c) (and for (A₁, B₁) also shows A, B).
  const ctrbCard = (A, B, title, page, answers) => ({ ...WB.ss.ctrbCard(A, B, title, page), spoiler: false, answers });
  function polesCard(ctx, d) {
    return {
      title: 'Desired closed-loop poles', page: 'p. 113 · Eq. 8.5, p. 184',
      theory: '\\omega_n = \\frac{2.2}{t_r},\\quad \\Delta^d_{cl} = (s^2 + 2\\zeta\\omega_n s + \\omega_n^2)' + (ctx.level === 'sf' ? '' : '(s - p_I)'),
      numbers: `\\omega_n = ${tex(2.2 / ctx.st.tr)},\\quad \\Delta^d_{cl} = ${WB.tf.polyTex(L.polyFromRoots(d.poles))},\\quad p = ${d.poles.map((p) => texPole(p)).join(',\\;')}`,
      answers: 'D.11/a',
    };
  }
  const poleList = (v, keys) => keys.map((k) => M.parseComplex(v[k]));
  // Conjugate pairs: if the user typed a + bj once, fill in the partner.
  function completeConj(list) {
    const out = list.slice();
    for (let i = 0; i < out.length; i++) {
      if (!out[i] && i > 0 && out[i - 1] && Math.abs(out[i - 1].im) > 1e-12) out[i] = { re: out[i - 1].re, im: -out[i - 1].im };
    }
    return out;
  }

  // --------------------------------------------------------- chapter base --
  function base(level, id, num, title, pages, extra) {
    return Object.assign({
      id, num, tab: `Ch ${num}`, title, pages, level,
      controller(ctx, o) { return makeSS(ctx, o); },
      linearSim(ctx, c) { return lib.linearSim(ctx, c, makeSS); },
      gains(ctx) { ctx.level = level; return gainsFor(ctx); },
      splane(ctx) { return { markers: markers(ctx), zetaRay: ctx.S.mode === 'explore' && ctx.st.zeta < 1 ? ctx.st.zeta : null }; },
      onPoleDrag: onDrag,
      targets(ctx) { return level === 'sf' ? { tr: ctx.st.tr, zeta: ctx.st.zeta } : { tr: ctx.st.tr }; },
    }, extra);
  }
  // Work-mode starting gains: deliberately not the answers.
  const W0 = { K1: 1, K2: 1, kr: 1, ki: -0.5, L1: 5, L2: 10, Ld: 2 };
  const estOut = (ctx, res, sc) => [{ label: 'ẑ (estimate)', y: sc(res.extras.xhat0 || []), color: '--series-3', dash: [3, 3], width: 2 }];

  // ---------------------------------------------------------------- D.11 --
  CH.ch11 = base('sf', 'ch11', 11, 'Full state feedback', 'pp. 173–196, pp. 380–381', {
    defaults(sys) { const p = sys.problems.ch11; return { est: 'dirty', sigma: 0.05, tr: p.tr, zeta: p.zeta, w: { ...W0 } }; },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'State feedback u = −Kx + k_r r', 'p. 173 · Eq. 11.3, p. 182 · Eq. 11.35');
      estControl(sec, ctx);
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['K1', 'K2', 'kr']);
      else { tuningSliders(sec, ctx); gainsReadout(sec, ctx, ['K1', 'K2', 'kr']); }
    },
    extraPlot(ctx, res) {
      return { opts: { title: 'ż(t) and the estimate used', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [
        { label: 'ż used by the controller', y: Array.from(res.extras.xhat1 || []), color: '--series-3', dash: [3, 3], width: 2 },
        { label: 'true ż', y: res.x.map((x) => x[1]), color: '--series-1' },
      ] } };
    },
    math(ctx) {
      const d = design(ctx), { A, B } = ctx.ss, g = ctx.gains;
      return [
        ssCard(ctx), ctrbCard(A, B, 'Controllability', 'p. 180 · Eq. 11.29', 'D.11/c'), polesCard(ctx, d),
        { title: 'Pole placement', page: 'p. 182 · Eq. 11.32',
          theory: 'K = (\\alpha - a_A)\\,\\mathcal{A}_A^{-1}\\,\\mathcal{C}_{A,B}^{-1}',
          numbers: `K = ${texMat([d.K])}`, answers: 'D.11/d' },
        { title: 'Reference gain', page: 'p. 182 · Eq. 11.35',
          theory: 'k_r = \\frac{-1}{C(A - BK)^{-1}B}',
          numbers: `k_r = ${tex(d.kr)}`, answers: 'D.11/d' },
        { title: 'Reference gain of the mass-spring-damper', page: 'p. 182 · Eq. 11.35', answers: 'D.11/d',
          theory: 'k_r = m\\omega_n^2 = K_1 + k',
          note: 'Unlike the arm, k_r ≠ K₁: the spring needs an extra k z_r to hold the mass at z_r.' },
        { title: 'Control law', page: 'p. 173 · Eq. 11.3',
          theory: 'F = -Kx + k_r z_r',
          numbers: `F = -(${tex(g.K[0])}\\,z + ${tex(g.K[1])}\\,\\dot z) + ${tex(g.kr)}\\,z_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch11;
      const poles = () => D.polesFromWnZeta(2.2 / prob.tr, prob.zeta);
      const ref = () => ans.stateFeedback(ctx.pModel, poles());
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Closed-loop poles from s² + 2ζω<sub>n</sub>s + ω<sub>n</sub>² = 0 (ω<sub>n</sub>, ζ from D.8)',
          html: 'Enter the poles as −σ ± jω<sub>d</sub>.',
          inputs: { sig: 'σ', wd: 'ω<sub>d</sub>' },
          check: (v) => lib.check(v, { sig: -poles()[0].re, wd: Math.abs(poles()[0].im) }, { sig: 'σ', wd: 'ωd' }),
          solution: () => [{ tex: `\\omega_n = 1.1,\\; s^2 + 1.54s + 1.21 = 0 \\Rightarrow p = ${texPole(poles()[0], 4)},\\; ${texPole(poles()[1], 4)}` }],
        },
        {
          id: 'b', title: '(b) Add A, B, C, D from D.6 to your param file',
          html: 'Use your A, B, C, D from D.6 (the D.6 tab) in your <code>massParam.py</code>. The model card in the live math unlocks once D.6 is solved.',
        },
        {
          id: 'c', title: '(c) Controllability: rank(𝒞<sub>A,B</sub>) = n',
          inputs: { c11: 'c<sub>11</sub>', c12: 'c<sub>12</sub>', c21: 'c<sub>21</sub>', c22: 'c<sub>22</sub>', rank: 'rank' },
          check: (v) => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return lib.check(v, { c11: Cm[0][0], c12: Cm[0][1], c21: Cm[1][0], c22: Cm[1][1], rank: L.rank(Cm) }, {}); },
          solution: () => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return [{ tex: `\\mathcal{C}_{A,B} = \\begin{bmatrix}0 & \\frac1m\\\\ \\frac1m & -\\frac{b}{m^2}\\end{bmatrix} = ${texMat(Cm)},\\; \\det = -\\frac{1}{m^2} = ${tex(Cm[0][0] * Cm[1][1] - Cm[0][1] * Cm[1][0])} \\ne 0` }]; },
        },
        {
          id: 'd', title: `(d) K and k<sub>r</sub> for t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta}. Why is K = (k<sub>P</sub>, k<sub>D</sub>)?`,
          html: 'Enter K and k<sub>r</sub>; the solution explains the "why".',
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => { const r = ref(); return lib.check(v, { K1: r.K[0], K2: r.K[1], kr: r.kr }, {}); },
          actions: [WB.design.useGains(ctx, ['K1', 'K2', 'kr'])],
          solution: () => {
            const r = ref();
            return [
              { tex: '\\det(sI - A + BK) = s^2 + \\frac{b + K_2}{m}s + \\frac{k + K_1}{m} \\;\\Rightarrow\\; K_1 = m\\omega_n^2 - k,\\; K_2 = 2m\\zeta\\omega_n - b' },
              { tex: `K = ${texMat([r.K])},\\quad k_r = \\frac{-1}{C(A-BK)^{-1}B} = m\\omega_n^2 = k + K_1 = ${tex(r.kr)}` },
              { html: 'Why K = (k<sub>P</sub>, k<sub>D</sub>): −Kx = −K<sub>1</sub>z − K<sub>2</sub>ż is the D.8 PD law with the derivative on the output, and the same characteristic polynomial forces the same coefficients. The difference is k<sub>r</sub>: it equals k<sub>P</sub> + k, i.e. PD plus the spring feedforward F<sub>e</sub> = kz<sub>r</sub>, which is what makes the DC gain one.' },
            ];
          },
        },
        {
          id: 'e', title: '(e) Implement with a digital differentiator',
          html: 'Checks the current simulation: ż from the dirty derivative, and |z<sub>r</sub> − z| before the first switch under 1% of the step.',
          check: () => {
            if (ctx.st.est !== 'dirty') return { ok: false, msg: 'Select the dirty derivative.' };
            const { e, amp } = lib.errorBeforeSwitch(ctx);
            return { ok: e < 0.01 * amp, msg: `Error before the switch: ${fmt(1000 * e, 3)} mm.` };
          },
          solution: () => [{ html: 'Build x = (z, ż̂) with ż̂ from Eq. 10.4 (σ = 0.05), then F = −Kx + k<sub>r</sub>z<sub>r</sub>. With exact parameters the steady-state error is zero; with mismatch it is not (no integrator yet).' }],
        },
      ]);
    },
  });

  // ---------------------------------------------------------------- D.12 --
  CH.ch12 = base('sfi', 'ch12', 12, 'Integrator with state feedback', 'pp. 197–214, p. 381', {
    defaults(sys) { const p = sys.problems.ch12; return { est: 'dirty', sigma: 0.05, tr: p.tr, zeta: p.zeta, pI: p.pIRef, antiwindup: 'clamp', w: { ...W0 } }; },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'u = −Kx − k_I ∫(r − z)', 'p. 199');
      estControl(sec, ctx);
      awControl(sec, ctx);
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['K1', 'K2', 'ki']);
      else { tuningSliders(sec, ctx, { pI: true }); gainsReadout(sec, ctx, ['K1', 'K2', 'ki']); }
    },
    extraPlot(ctx, res) {
      return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [m·s]', unit: 'm·s' }, data: { series: [{ label: 'x_I = ∫(r − z)', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = design(ctx), { A, B, C } = ctx.ss;
      const { A1, B1 } = D.augmentIntegrator(A, B, C);
      return [
        ssCard(ctx),
        { title: 'Augmented system', page: 'p. 198 · Eq. 12.1',
          theory: '\\dot x_I = r - Cx,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\ 0\\end{bmatrix}',
          numbers: `A_1 = ${texMat(A1)},\\quad B_1 = ${texMat(B1)}`, answers: 'D.6/a' },
        ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 198', ['D.6/a', 'D.11/c']),
        polesCard(ctx, d),
        { title: 'Gains', page: 'p. 199–201',
          theory: '\\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad u = -Kx - k_I\\int_0^t (r - y)\\,d\\tau',
          numbers: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}`, answers: 'D.12/a' },
        { title: 'Gains for the mass-spring-damper', page: 'p. 199–201', answers: 'D.12/a',
          theory: '\\det(sI - A_1 + B_1K_1) = s^3 + c_2s^2 + c_1s + c_0,\\quad c_2 = \\frac{b + K_2}{m},\\; c_1 = \\frac{k + K_1}{m},\\; c_0 = -\\frac{k_I}{m}' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch12;
      const ref = (pI) => ans.integralFeedback(ctx.pModel, [...D.polesFromWnZeta(2.2 / prob.tr, prob.zeta), { re: pI, im: 0 }]);
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Add an integrator with anti-windup to the D.11 controller',
          html: `Gains for t<sub>r</sub> = ${prob.tr} s, ζ = ${prob.zeta} and your integrator pole p<sub>I</sub>: the problem leaves p<sub>I</sub> to you, so enter the one you chose with your gains. Anti-windup is the switch in the controls on the right.`,
          inputs: { pI: 'p<sub>I</sub>', K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', ki: 'k<sub>I</sub>' },
          check: (v) => {
            const pI = lib.num(v.pI);
            if (pI === null || pI >= 0) return { ok: false, msg: 'Enter a negative pI.' };
            const r = ref(pI); return lib.check(v, { K1: r.K[0], K2: r.K[1], ki: r.ki }, { ki: 'kI' });
          },
          actions: [WB.design.useGains(ctx, ['K1', 'K2', 'ki'])],
          solution: () => {
            const r = ref(prob.pIRef);
            return [
              { tex: 'K_1 = m c_1 - k,\\quad K_2 = m c_2 - b,\\quad k_I = -m c_0 \\quad\\text{for } \\Delta^d = s^3 + c_2s^2 + c_1s + c_0' },
              { tex: `p_I = ${prob.pIRef}:\\; \\Delta^d = ${WB.tf.polyTex(L.polyFromRoots([...D.polesFromWnZeta(1.1, 0.7), { re: prob.pIRef, im: 0 }]))},\\; K = ${texMat([r.K])},\\; k_I = ${tex(r.ki)}` },
            ];
          },
        },
        {
          id: 'b', title: '(b) Input disturbance of 0.25 N and parameters varying up to 20%',
          html: 'Set the input disturbance d and the true-plant mismatch in the left panel (the chapter starts with d = 0.25 N and a fixed 20% draw).',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return Math.abs(S.sim.dist) > 0 && mis ? { ok: true, msg: `d = ${fmt(S.sim.dist, 3)} N with plant mismatch.` } : { ok: false, msg: 'Set both d ≠ 0 and a plant mismatch.' };
          },
        },
        {
          id: 'c', title: '(c) Tune the integrator pole for good tracking',
          html: 'Passes when anti-windup is on and |z<sub>r</sub> − z| just before the first switch is under 1% of the step, with the disturbance and mismatch in the left panel.',
          check: () => {
            const { e, amp } = lib.errorBeforeSwitch(ctx);
            if (ctx.st.antiwindup !== 'clamp') return { ok: false, msg: 'Turn the anti-windup on (part a).' };
            return { ok: e < 0.01 * amp, msg: `Error before the switch: ${fmt(1000 * e, 3)} mm (d = ${fmt(ctx.S.sim.dist, 3)} N).` };
          },
          solution: () => [{ html: `p<sub>I</sub> = ${prob.pIRef} with the D.8 pair settles inside 25 s here. A faster p<sub>I</sub> (e.g. −2) removes d sooner but adds overshoot; a slower one leaves a long tail.` }],
        },
      ]);
    },
  });

  // ---------------------------------------------------------------- D.13 --
  function obsDefaults(p) {
    return { sigma: 0.05, tr: p.tr, zeta: p.zeta, pI: p.pIRef, antiwindup: 'clamp', wnObs: p.obsFactor * 2.2 / p.tr, zetaObs: p.zetaObs, xhat0: 0.1, w: { ...W0 } };
  }
  CH.ch13 = base('obs', 'ch13', 13, 'Observers', 'pp. 215–238, pp. 381–382', {
    defaults(sys) { return obsDefaults(sys.problems.ch13); },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries: estOut,
    extraPlot(ctx, res) {
      return { opts: { title: 'ż(t) and estimate', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [
        { label: 'ż̂ (observer)', y: Array.from(res.extras.xhat1 || []), color: '--series-3', dash: [3, 3], width: 2 },
        { label: 'true ż', y: res.x.map((x) => x[1]), color: '--series-1' },
      ] } };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂)', 'p. 222 · Fig. 13-3');
      awControl(sec, ctx);
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['K1', 'K2', 'ki']);
      else tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer', 'p. 216 · Eq. 13.3');
      if (ctx.S.mode === 'work') workSliders(ob, ctx, ['L1', 'L2']);
      else { obsSliders(ob, ctx, false); gainsReadout(ob, ctx, ['K1', 'K2', 'ki', 'L1', 'L2']); }
      slider(ob, { label: 'ẑ(0)', unit: 'm', min: -1, max: 1, step: 0.01, sig: 3, hint: 'initial estimate (true z starts at the left-panel value)', ...bind(ctx, 'xhat0') });
    },
    math(ctx) {
      const d = design(ctx), { A, C } = ctx.ss;
      const O = L.obsv(A, C);
      return [
        ssCard(ctx),
        { title: 'Observer', page: 'p. 216 · Eq. 13.3',
          theory: '\\dot{\\hat x} = A\\hat x + Bu + L(y - C\\hat x),\\quad \\dot e = (A - LC)e' },
        { title: 'Observability', page: 'p. 221',
          theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ \\vdots \\\\ CA^{n-1}\\end{bmatrix},\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = n',
          numbers: `\\mathcal{O} = ${texMat(O)},\\quad \\operatorname{rank} = ${L.rank(O)}`, answers: 'D.13/b' },
        { title: 'Observer gain', page: 'p. 222 · Eq. 13.16',
          theory: 'L = \\text{place}(A^\\top, C^\\top, q)^\\top',
          numbers: `q = ${d.obsPoles.map((p) => texPole(p)).join(',\\;')},\\quad L = ${texMat(d.L)}`, answers: 'D.13/c1' },
        { title: 'Observer gain for the mass-spring-damper', page: 'p. 222', answers: 'D.13/c1',
          theory: '\\det(sI - A + LC) = s^2 + \\beta_1 s + \\beta_0,\\quad \\beta_1 = \\tfrac bm + L_1,\\; \\beta_0 = \\tfrac km + \\tfrac bm L_1 + L_2' },
        { title: 'Separation principle', page: 'p. 222–223',
          theory: '\\text{eig} = \\text{eig}(A_1 - B_1K_1) \\cup \\text{eig}(A - LC)',
          note: 'Holds for the linear model. Saturation breaks it.' },
        polesCard(ctx, d),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch13;
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Exact parameters, no input disturbance (α = 0)',
          html: 'The chapter starts with α = 0 and d = 0. <em>Exact model</em> in the left panel restores them.',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return !mis && !(Math.abs(S.sim.dist) > 0) ? { ok: true, msg: 'Exact plant, d = 0.' } : { ok: false, msg: 'Remove the plant mismatch and set d = 0.' };
          },
        },
        {
          id: 'b', title: '(b) Observability: rank(𝒪<sub>A,C</sub>) = n',
          inputs: { o11: 'o<sub>11</sub>', o12: 'o<sub>12</sub>', o21: 'o<sub>21</sub>', o22: 'o<sub>22</sub>', rank: 'rank' },
          check: (v) => { const O = L.obsv(ctx.ss.A, ctx.ss.C); return lib.check(v, { o11: O[0][0], o12: O[0][1], o21: O[1][0], o22: O[1][1], rank: L.rank(O) }, {}); },
          solution: () => [{ tex: `\\mathcal{O}_{A,C} = \\begin{bmatrix}C\\\\ CA\\end{bmatrix} = ${texMat(L.obsv(ctx.ss.A, ctx.ss.C))} \\Rightarrow \\text{rank } 2` }],
        },
        {
          id: 'c1', title: '(c) Add an observer: gain L for your observer poles',
          html: 'Enter the two observer poles you chose (complex is fine: <code>-7.8+7.8j</code>; the conjugate is filled in if you leave q<sub>2</sub> blank) and your L.',
          inputs: { q1: 'q<sub>1</sub>', q2: 'q<sub>2</sub>', L1: 'L<sub>1</sub>', L2: 'L<sub>2</sub>' },
          check: (v) => {
            const q = completeConj(poleList(v, ['q1', 'q2']));
            if (q.some((x) => !x)) return { ok: false, msg: 'Enter the observer poles.' };
            if (q.some((x) => x.re >= 0)) return { ok: false, msg: 'Observer poles must be in the left half-plane.' };
            const r = ans.observer(ctx.pModel, q);
            return lib.check(v, { L1: r.L[0], L2: r.L[1] }, {});
          },
          actions: [WB.design.useGains(ctx, ['L1', 'L2'])],
          solution: () => {
            const q = D.polesFromWnZeta(prob.obsFactor * 2.2 / prob.tr, prob.zetaObs);
            const r = ans.observer(ctx.pModel, q);
            return [
              { tex: 'L_1 = \\beta_1 - \\tfrac bm,\\quad L_2 = \\beta_0 - \\tfrac km - \\tfrac bm L_1 \\quad\\text{for } \\Delta_{obs} = s^2 + \\beta_1 s + \\beta_0' },
              { tex: `\\text{observer } ${prob.obsFactor}\\times \\text{ faster } (\\omega_{n,obs} = ${tex(prob.obsFactor * 2.2 / prob.tr)},\\ \\zeta = ${prob.zetaObs}):\\; q = ${texPole(q[0])},\\; ${texPole(q[1])},\\; L = ${texMat(r.L)}` },
            ];
          },
        },
        {
          id: 'c2', title: '(c) Tune the controller and observer poles',
          html: 'Passes when ẑ has converged (|z − ẑ| < 1 mm at t<sub>end</sub>) and |z<sub>r</sub> − z| before the first switch is under 1% of the step. Use exact parameters and d = 0 (part a).',
          check: () => {
            const res = ctx.app.result(), n = res.t.length - 1;
            const est = Math.abs(res.y[n] - (res.extras.xhat0 || [])[n]);
            const { e, amp } = lib.errorBeforeSwitch(ctx);
            return { ok: est < 1e-3 && e < 0.01 * amp, msg: `|z − ẑ| = ${fmt(1000 * est, 3)} mm at t_end; tracking error ${fmt(1000 * e, 3)} mm.` };
          },
          solution: () => [{ html: `Controller from D.12 (p<sub>I</sub> = ${prob.pIRef}), observer poles ${prob.obsFactor}× faster than the controller pair. A slower observer still converges but couples into the response; a much faster one amplifies noise (D.14).` }],
        },
        {
          id: 'd', title: '(d) Plot the state and the estimated state together',
          html: 'In your code the controller returns both u and x̂. Here the z plot shows the estimate ẑ with the true z, and the extra plot shows ż and its estimate.',
        },
        {
          id: 'e', title: '(e) Add d = 0.25 N',
          html: 'Set d = 0.25 in the left panel. The integrator drives r − ẑ to zero, but ẑ is biased by the unmodeled d, so z settles away from r.',
          check: () => {
            if (!(Math.abs(ctx.S.sim.dist) > 0)) return { ok: false, msg: 'Set d ≠ 0 first.' };
            const res = ctx.app.result(), n = res.t.length - 1;
            const bias = res.y[n] - (res.extras.xhat0 || [])[n];
            return { ok: true, msg: `At t_end: z − ẑ = ${fmt(1000 * bias, 3)} mm, r − z = ${fmt(1000 * (res.r[n] - res.y[n]), 3)} mm.` };
          },
          solution: () => {
            const { A, B, C } = ctx.ss, g = ctx.gains;
            const Lc = [[g.L[0]], [g.L[1]]];
            const Ae = L.sub(A, L.mul(Lc, C));
            const Ai = L.inv(Ae);
            const ess = Ai ? -L.mul(L.mul(C, Ai), B)[0][0] : NaN;
            return [
              { tex: '\\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} = -(A - LC)^{-1}Bd,\\quad z - \\hat z = -C(A - LC)^{-1}B\\,d' },
              { tex: `\\text{current } L:\\; z - \\hat z = ${tex(ess * 0.25)}\\,\\text{m for } d = 0.25\\,\\text{N}` },
              { html: 'With ẑ held at z<sub>r</sub> by the integrator, z = z<sub>r</sub> + (z − ẑ): the observer bias becomes the tracking error (p. 240).' },
            ];
          },
        },
      ]);
    },
  });

  // ---------------------------------------------------------------- D.14 --
  CH.ch14 = base('dobs', 'ch14', 14, 'Disturbance observer', 'pp. 239–259, p. 382', {
    defaults(sys) { const p = sys.problems.ch14; return { ...obsDefaults(p), pD: p.pDRef, dobs: true, xhat0: 0 }; },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries: estOut,
    extraPlot(ctx, res) {
      const S = ctx.S;
      return { opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N]', unit: 'N' }, data: { series: [
        { label: 'd̂ (estimate)', y: Array.from(res.extras.dhat || []), color: '--series-3', width: 2 },
        { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
      ] } };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'p. 241');
      segmented(sec, {
        label: 'Disturbance observer', options: [{ value: true, label: 'on' }, { value: false, label: 'off (D.14a)' }],
        ...bind(ctx, 'dobs'),
      });
      awControl(sec, ctx);
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['K1', 'K2', 'ki']);
      else tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer', 'p. 241');
      if (ctx.S.mode === 'work') workSliders(ob, ctx, ['L1', 'L2', 'Ld']);
      else { obsSliders(ob, ctx, true); gainsReadout(ob, ctx, ['K1', 'K2', 'ki', 'L1', 'L2', 'Ld']); }
    },
    math(ctx) {
      const d = design(ctx), { A, B, C } = ctx.ss;
      const { A2, C2 } = D.augmentDisturbance(A, B, C);
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(u + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0 \\text{ for constant } d' },
        { title: 'Augmented model (ḋ = 0)', page: 'p. 240',
          theory: 'A_2 = \\begin{bmatrix}A & B\\\\ 0 & 0\\end{bmatrix},\\quad C_2 = \\begin{bmatrix}C & 0\\end{bmatrix}',
          numbers: `A_2 = ${texMat(A2)},\\quad \\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}`, answers: 'D.6/a' },
        { title: 'Disturbance observer', page: 'p. 241',
          theory: '\\dot{\\hat x} = A\\hat x + B(u + \\hat d) + L(y - C\\hat x),\\quad \\dot{\\hat d} = L_d(y - C\\hat x),\\quad u = -K\\hat x - k_I\\textstyle\\int e - \\hat d' },
        { title: 'Observer gains', page: 'p. 241',
          theory: '\\begin{bmatrix}L\\\\ L_d\\end{bmatrix} = \\text{place}(A_2^\\top, C_2^\\top, q)^\\top',
          numbers: `q = ${d.obsPoles.map((p) => texPole(p)).join(',\\;')},\\quad L = ${texMat(d.L)},\\; L_d = ${tex(d.Ld)}`, answers: 'D.14/b1' },
        { title: 'Observer gains for the mass-spring-damper', page: 'p. 241', answers: 'D.14/b1',
          theory: '\\det(sI - A_2 + LC_2) = s^3 + c_2s^2 + c_1s + c_0,\\quad c_2 = \\tfrac bm + L_1,\\; c_1 = \\tfrac km + \\tfrac bm L_1 + L_2,\\; c_0 = \\tfrac{L_d}{m}' },
        polesCard(ctx, d),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch14;
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) α = 0.2, input disturbance 0.25, noise σ = 0.001 on z<sub>m</sub>',
          html: 'The chapter starts with a fixed 20% plant draw, d = 0.25 N and noise σ = 1 mm. Turn the disturbance observer off to see the D.13 bias come back.',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return mis && Math.abs(S.sim.dist) > 0 && S.sim.noise > 0 ? { ok: true, msg: `d = ${fmt(S.sim.dist, 3)} N, noise σ = ${fmt(S.sim.noise, 3)} m, with plant mismatch.` } : { ok: false, msg: 'Set a plant mismatch, d ≠ 0 and measurement noise.' };
          },
        },
        {
          id: 'b1', title: '(b) Add a disturbance observer: gains for your poles',
          html: 'Enter your three observer poles (q<sub>3</sub> is usually the real disturbance pole) and your gains.',
          inputs: { q1: 'q<sub>1</sub>', q2: 'q<sub>2</sub>', q3: 'q<sub>3</sub>', L1: 'L<sub>1</sub>', L2: 'L<sub>2</sub>', Ld: 'L<sub>d</sub>' },
          check: (v) => {
            const q = completeConj(poleList(v, ['q1', 'q2', 'q3']));
            if (q.some((x) => !x)) return { ok: false, msg: 'Enter all three observer poles.' };
            if (q.some((x) => x.re >= 0)) return { ok: false, msg: 'Observer poles must be in the left half-plane.' };
            const r = ans.disturbanceObserver(ctx.pModel, q);
            return lib.check(v, { L1: r.L[0], L2: r.L[1], Ld: r.Ld }, {});
          },
          actions: [WB.design.useGains(ctx, ['L1', 'L2', 'Ld'])],
          solution: () => {
            const q = [...D.polesFromWnZeta(prob.obsFactor * 2.2 / prob.tr, prob.zetaObs), { re: prob.pDRef, im: 0 }];
            const r = ans.disturbanceObserver(ctx.pModel, q);
            return [
              { tex: 'L_1 = c_2 - \\tfrac bm,\\quad L_2 = c_1 - \\tfrac km - \\tfrac bm L_1,\\quad L_d = m\\,c_0 \\quad\\text{for } \\Delta_{obs} = s^3 + c_2s^2 + c_1s + c_0' },
              { tex: `q = ${q.map((x) => texPole(x)).join(',\\;')}:\\; L = ${texMat(r.L)},\\; L_d = ${tex(r.Ld)}` },
            ];
          },
        },
        {
          id: 'b2', title: '(b) Estimator bias removed',
          html: 'Passes when the disturbance observer is on and the mean of z − ẑ over the 10 s before the first reference switch is under 0.2 mm in magnitude (noise σ = 1 mm is on; the mean filters it). Compare with the observer off.',
          check: () => {
            const res = ctx.app.result(), S = ctx.S;
            if (!ctx.st.dobs) return { ok: false, msg: 'Turn the disturbance observer on.' };
            const tSw = S.sim.type === 'square' ? S.sim.tStep + 0.5 / S.sim.frequency : S.sim.tEnd;
            const k1 = Math.min(res.t.length, Math.round(tSw / S.sim.Ts)), k0 = Math.max(0, k1 - Math.round(10 / S.sim.Ts));
            let s = 0;
            for (let k = k0; k < k1; k++) s += res.y[k] - res.extras.xhat0[k];
            const bias = Math.abs(s / Math.max(1, k1 - k0));
            const dh = (res.extras.dhat || [])[Math.max(0, k1 - 1)];
            return { ok: bias < 2e-4, msg: `d̂ = ${fmt(dh, 3)} N vs d = ${fmt(S.sim.dist, 3)} N; mean |z − ẑ| = ${fmt(1000 * bias, 3)} mm.` };
          },
          solution: () => [{ html: `With the D.13 design plus p<sub>d</sub> = ${prob.pDRef}: d̂ settles near d plus whatever force the mismatched model gets wrong (here (k̂ − k)z and the mass error during transients), so it tracks the total unmodeled input, not d alone. Raising the observer bandwidth speeds this up but lets more of the 1 mm noise into ż̂ and F.` }],
        },
      ]);
    },
  });

  WB.studies.D.ss = { makeSS, design };
})();
