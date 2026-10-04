// Chapters 11-14: full state feedback, integral augmentation, observers, and
// disturbance observers on the feedback-linearized model
//   xdot = A x + B u~,  y = C x,  u = tau_fl(theta) + u~   (A.6 / A.11, p. 187)
window.WB = window.WB || {};
WB.chapters = WB.chapters || {};

(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const PD = () => WB.pd;

  // --------------------------------------------------------------- design --
  function wnOf(st) { return PD().wnFromTr(st.tr, st.zeta, st.rule || '2.2'); }
  function ctrlPoles(st) { return PD().polesFromWnZeta(wnOf(st), st.zeta); }
  function obsPoles(st) { return PD().polesFromWnZeta(st.wnObs, st.zetaObs); }

  const augI = (ss) => WB.design.augmentIntegrator(ss.A, ss.B, ss.C);
  const augD = (ss) => WB.design.augmentDisturbance(ss.A, ss.B, ss.C);

  // Designed gains from the tuning knobs (explore mode / problem solutions).
  function design(ctx, st = ctx.st, level = ctx.level) {
    const { A, B, C } = ctx.ss;
    const out = { poles: ctrlPoles(st) };
    if (level === 'sf') {
      const K = L.place(A, B, out.poles);
      out.K = K ? K[0] : [NaN, NaN];
      const Acl = L.sub(A, L.mul(B, [out.K]));
      const Ai = L.inv(Acl);
      out.kr = Ai ? -1 / L.mul(L.mul(C, Ai), B)[0][0] : NaN;
      return out;
    }
    const { A1, B1 } = augI(ctx.ss);
    out.poles = [...out.poles, { re: st.pI, im: 0 }];
    const K1 = L.place(A1, B1, out.poles);
    out.K = K1 ? [K1[0][0], K1[0][1]] : [NaN, NaN];
    out.ki = K1 ? K1[0][2] : NaN;
    if (level === 'obs') {
      out.obsPoles = obsPoles(st);
      const Lt = L.place(L.T(A), L.T(C), out.obsPoles);
      out.L = Lt ? [Lt[0][0], Lt[0][1]] : [NaN, NaN];
    }
    if (level === 'dobs') {
      const { A2, C2 } = augD(ctx.ss);
      out.obsPoles = [...obsPoles(st), { re: st.pD, im: 0 }];
      const Lt = L.place(L.T(A2), L.T(C2), out.obsPoles);
      out.L = Lt ? [Lt[0][0], Lt[0][1]] : [NaN, NaN];
      out.Ld = Lt ? Lt[0][2] : NaN;
    }
    return out;
  }

  // ----------------------------------------------------------- controller --
  // level: 'sf' (Ch 11), 'sfi' (Ch 12), 'obs' (Ch 13), 'dobs' (Ch 14)
  function makeSS(ctx, { linear = false } = {}) {
    const { sys, pModel, S } = ctx;
    const st = ctx.st, level = ctx.level, g = ctx.gains;
    const { A, B, C } = ctx.ss;
    const Ts = S.sim.Ts, uLim = sys.uLimit(pModel);
    // Work mode adds τ_fl only once A.4(c) is solved (PD().compApplied).
    const flOn = !linear && st.comp === 'fl' && PD().compApplied(ctx);
    const ff = (th) => (flOn ? sys.feedbackLinearization([th, 0], pModel) : 0);
    const useObs = level === 'obs' || level === 'dobs';
    const useDO = level === 'dobs' && st.dobs;
    const sigma = 0.05;
    const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    let I = 0, ePrev = null, yPrev = null, ydot = 0, uPrev = 0;
    let xh = [st.xhat0 * M.DEG, 0], dh = 0;

    // Observer right-hand side (Eq. 13.3 / p. 241), with tau_fl inside the model.
    function fObs(z, y) {
      const [t, w, d] = z;
      const innov = y - (C[0][0] * t + C[0][1] * w);
      const u = uPrev - ff(t) + (useDO ? d : 0);
      return [
        A[0][0] * t + A[0][1] * w + B[0][0] * u + g.L[0] * innov,
        A[1][0] * t + A[1][1] * w + B[1][0] * u + g.L[1] * innov,
        useDO ? g.Ld * innov : 0,
      ];
    }

    return {
      update(r, x, yMeas) {
        let xUse;
        if (useObs) {
          const z = M.rk4Step((zz) => fObs(zz, yMeas), [xh[0], xh[1], dh], 0, Ts);
          xh = [z[0], z[1]]; dh = z[2];
          xUse = xh;
        } else if (st.est === 'dirty') {
          if (yPrev === null) yPrev = yMeas;
          ydot = beta * ydot + gamma * (yMeas - yPrev);
          yPrev = yMeas;
          xUse = [yMeas, ydot];
        } else {
          xUse = x;
        }
        const Kx = g.K[0] * xUse[0] + g.K[1] * xUse[1];
        let u;
        if (level === 'sf') {
          u = ff(xUse[0]) - Kx + g.kr * r;
        } else {
          const e = r - xUse[0];
          if (ePrev === null) ePrev = 0; // repo: error_d1 = 0
          const uTry = ff(xUse[0]) - Kx - g.ki * (I + (Ts / 2) * (e + ePrev)) - (useDO ? dh : 0);
          // Anti-windup (A.12a): skip integration while the actuator is saturated.
          const saturated = !linear && Math.abs(uTry) > uLim;
          if (!(st.antiwindup === 'clamp' && saturated)) I += (Ts / 2) * (e + ePrev);
          ePrev = e;
          u = ff(xUse[0]) - Kx - g.ki * I - (useDO ? dh : 0);
        }
        uPrev = linear ? u : M.saturate(u, uLim);
        return { u, xhat0: xUse[0], xhat1: xUse[1], dhat: dh, integrator: I };
      },
    };
  }

  // ------------------------------------------------------------- controls --
  function tuningSliders(parent, ctx, { pI, rule } = {}) {
    slider(parent, { label: 't<sub>r</sub>', unit: 's', min: 0.1, max: 2, step: 0.001, ...bind(ctx, 'tr') });
    slider(parent, { label: 'ζ', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zeta') });
    if (rule) {
      segmented(parent, {
        label: 'ω<sub>n</sub> from t<sub>r</sub>',
        options: [{ value: '2.2', label: '2.2 / t<sub>r</sub>' }, { value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))' }],
        ...bind(ctx, 'rule'),
      });
    }
    if (pI) slider(parent, { label: 'p<sub>I</sub>', min: -30, max: -0.1, step: 0.01, sig: 3, ...bind(ctx, 'pI') });
  }

  function obsSliders(parent, ctx, withD) {
    slider(parent, { label: 'ω<sub>n,obs</sub>', unit: 'rad/s', min: 1, max: 150, step: 0.1, sig: 4, ...bind(ctx, 'wnObs') });
    slider(parent, { label: 'ζ<sub>obs</sub>', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zetaObs') });
    if (withD) slider(parent, { label: 'p<sub>d</sub>', min: -60, max: -0.1, step: 0.1, sig: 3, ...bind(ctx, 'pD') });
  }

  const WORK_SPEC = {
    K1: ['K<sub>1</sub>', 0, 10], K2: ['K<sub>2</sub>', 0, 1], kr: ['k<sub>r</sub>', 0, 10], ki: ['k<sub>i</sub>', -30, 0],
    L1: ['L<sub>1</sub>', 0, 200], L2: ['L<sub>2</sub>', 0, 5000], Ld: ['L<sub>d</sub>', 0, 50],
  };
  const workSliders = (parent, ctx, keys) => WB.ui.gainSliders(parent, ctx, WORK_SPEC, keys, { steps: 2000 });

  function gainsReadout(parent, ctx, keys) {
    WB.ui.readout(parent, () => {
      const g = ctx.gains;
      const vals = { K1: g.K[0], K2: g.K[1], kr: g.kr, ki: g.ki, L1: g.L && g.L[0], L2: g.L && g.L[1], Ld: g.Ld };
      return keys.map((k) => [k, vals[k]]);
    });
  }

  function gainsFor(ctx) {
    if (ctx.S.mode === 'explore') return design(ctx);
    const w = ctx.st.w;
    return { K: [w.K1, w.K2], kr: w.kr, ki: w.ki, L: [w.L1, w.L2], Ld: w.Ld };
  }

  function compControls(parent, ctx) {
    segmented(parent, {
      label: 'Gravity compensation',
      options: [{ value: 'fl', label: 'feedback lin. τ_fl(θ̂)' }, { value: 'none', label: 'none' }],
      ...bind(ctx, 'comp'),
    });
    PD().compNote(parent, ctx);
  }

  // ------------------------------------------------------------- analysis --
  function closedLoopPoles(ctx) {
    const { A, B } = ctx.ss;
    const g = ctx.gains;
    if (ctx.level === 'sf') return L.eig(L.sub(A, L.mul(B, [g.K])));
    const { A1, B1 } = augI(ctx.ss);
    return L.eig(L.sub(A1, L.mul(B1, [[g.K[0], g.K[1], g.ki]])));
  }
  function observerPoles(ctx) {
    const { A, C } = ctx.ss;
    const g = ctx.gains;
    if (ctx.level === 'obs') return L.eig(L.sub(A, L.mul([[g.L[0]], [g.L[1]]], C)));
    if (ctx.level === 'dobs') {
      const { A2, C2 } = augD(ctx.ss);
      return L.eig(L.sub(A2, L.mul([[g.L[0]], [g.L[1]], [g.Ld]], C2)));
    }
    return [];
  }

  // Each chapter's part whose answer the desired (target) poles give away.
  const TARGETS_PART = { ch11: ['ch11', 'a'], ch12: ['ch12', 'a'], ch13: ['ch13', 'c'], ch14: ['ch14', 'b1'] };
  const targetsKey = (ctx) => { const [ch, part] = TARGETS_PART[ctx.S.chapter] || ['ch11', 'a']; return PD().partKey(ctx, ch, part); };

  function markers(ctx) {
    const { A } = ctx.ss;
    // In Work mode the open-loop poles answer A.7(a), and the target poles are
    // computed from the spec, so each stays off until its part is solved.
    const mk = PD().shows(ctx, PD().partKey(ctx, 'ch7', 'a')) ? L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole ${i + 1}` })) : [];
    const explore = ctx.S.mode === 'explore';
    closedLoopPoles(ctx).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `controller pole ${i + 1}`, dragId: explore ? (Math.abs(p.im) > 1e-9 ? 0 : 2) : undefined }));
    observerPoles(ctx).forEach((p, i) => mk.push({ ...p, kind: 'obs', label: `observer pole ${i + 1}`, dragId: explore ? (Math.abs(p.im) > 1e-9 ? 10 : 12) : undefined }));
    if (!explore && ctx.app.isSolved(targetsKey(ctx))) {
      const d = design(ctx);
      for (const p of d.poles) mk.push({ ...p, kind: 'target', label: 'target pole (problem)' });
    }
    return mk;
  }

  // Dragging: complex controller pole -> (t_r, ζ); real one -> p_I; observer the same.
  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    if (id === 0) {
      const wn = Math.hypot(re, im);
      st.zeta = Math.max(0.2, Math.min(1, -re / wn));
      st.tr = st.rule === 'tp' ? Math.PI / (2 * wn * Math.sqrt(Math.max(1e-6, 1 - st.zeta ** 2))) : 2.2 / wn;
    } else if (id === 2) {
      st.pI = re;
    } else if (id === 10) {
      st.wnObs = Math.hypot(re, im);
      st.zetaObs = Math.max(0.2, Math.min(1, -re / st.wnObs));
    } else if (id === 12) {
      if (ctx.level === 'dobs') st.pD = re;
    }
    ctx.update();
  }

  // ----------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B } = ctx.ss;
    return {
      title: 'State-space model (feedback linearized)', page: 'p. 88 · Eq. 6.16, p. 187', answers: `${ctx.sys.problems.ch6.id}/a`,
      theory: '\\dot x = Ax + B\\tilde\\tau,\\quad y = Cx,\\quad A = \\begin{bmatrix}0 & 1\\\\ 0 & -\\frac{3b}{m\\ell^2}\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\ \\frac{3}{m\\ell^2}\\end{bmatrix},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix}',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`,
    };
  }
  // Controllability card, shared with studies B–F. matrix: show C_AB; det: add det C_AB.
  function ctrbCard(A, B, title, page, { matrix = true, det = false, sig = 4 } = {}) {
    const Cab = L.ctrb(A, B);
    const nums = [...(matrix ? [`\\mathcal{C} = ${texMat(Cab, sig)}`] : []), `\\operatorname{rank} = ${L.rank(Cab)}`, ...(det ? [`\\det = ${tex(L.det(Cab))}`] : [])];
    return {
      title, page, theory: '\\mathcal{C}_{A,B} = \\begin{bmatrix} B & AB & \\cdots & A^{n-1}B\\end{bmatrix},\\quad \\text{controllable} \\iff \\operatorname{rank}\\mathcal{C}_{A,B} = n',
      numbers: nums.join(',\\quad '), spoiler: true,
    };
  }
  function polesCard(ctx, d) {
    const st = ctx.st;
    const wn = wnOf(st);
    return {
      title: 'Desired closed-loop poles', page: 'p. 113 · Eq. 8.5, p. 184', answers: targetsKey(ctx),
      theory: (st.rule === 'tp' ? '\\omega_n = \\frac{\\pi}{2t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}') + ',\\quad \\Delta^d_{cl} = (s^2 + 2\\zeta\\omega_n s + \\omega_n^2)' + (ctx.level === 'sf' ? '' : '(s - p_I)'),
      numbers: `\\omega_n = ${tex(wn)},\\quad \\Delta^d_{cl} = ${WB.tf.polyTex(L.polyFromRoots(d.poles))},\\quad p = ${d.poles.map((p) => texPole(p)).join(',\\;')}`,
      spoiler: true,
    };
  }

  // --------------------------------------------------------- chapter base --
  function base(level, num, title, pages, extra) {
    return Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages,
      level,
      controller(ctx, o) { return makeSS(ctx, o); },
      gains(ctx) { ctx.level = level; return gainsFor(ctx); },
      splane(ctx) { return { markers: markers(ctx), zetaRay: ctx.st.zeta < 1 ? ctx.st.zeta : null }; },
      onPoleDrag: onDrag,
      // The 2nd-order overshoot formula only applies without integrator/observer poles.
      targets(ctx) { return level === 'sf' ? { tr: ctx.st.tr, zeta: ctx.st.zeta } : { tr: ctx.st.tr }; },
    }, extra);
  }

  // Work-mode starting gains: deliberately not the answers.
  const W0 = { K1: 0.2, K2: 0.05, kr: 0.2, ki: -0.5, L1: 20, L2: 200, Ld: 2 };

  // ------------------------------------------------------------ Chapter 11 --
  WB.chapters.ch11 = base('sf', 11, 'Full state feedback', 'pp. 173–196', {
    defaults(sys) { const p = sys.problems.ch11; return { comp: 'fl', est: 'true', tr: p.tr, zeta: p.zeta, rule: '2.2', w: { ...W0 } }; },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'State feedback u = −Kx + k_r r', 'p. 173 · Eq. 11.3, p. 183 · Eq. 11.38');
      compControls(sec, ctx);
      segmented(sec, {
        label: 'Where x comes from',
        options: [{ value: 'true', label: 'true state (repo)' }, { value: 'dirty', label: 'θ + dirty derivative (A.11e)' }],
        ...bind(ctx, 'est'),
      });
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['K1', 'K2', 'kr']);
      else { tuningSliders(sec, ctx); gainsReadout(sec, ctx, ['K1', 'K2', 'kr']); }
    },
    math(ctx) {
      const d = design(ctx);
      const { A, B } = ctx.ss;
      const g = ctx.gains;
      return [
        ssCard(ctx),
        { ...ctrbCard(A, B, 'Controllability', 'p. 180 · Eq. 11.29'), answers: PD().partKey(ctx, 'ch11', 'c') },
        polesCard(ctx, d),
        { title: 'Pole placement (Ackermann)', page: 'p. 182 · Eq. 11.32', answers: PD().partKey(ctx, 'ch11', 'd'),
          theory: 'K = (\\alpha - a_A)\\,\\mathcal{A}_A^{-1}\\,\\mathcal{C}_{A,B}^{-1}',
          numbers: `K = ${texMat([d.K])}\\quad(\\text{eig}(A - BK) = ${d.poles.map((p) => texPole(p)).join(',\\;')})`, spoiler: true },
        { title: 'Reference gain', page: 'p. 182 · Eq. 11.35', answers: PD().partKey(ctx, 'ch11', 'd'),
          theory: 'k_r = \\frac{-1}{C(A - BK)^{-1}B}',
          numbers: `k_r = ${tex(d.kr)}`, spoiler: true,
          note: 'For this plant k_r = K₁: with a free integrator in the plant, unity DC gain needs the reference to enter exactly like the position feedback.' },
        { title: 'Control law with feedback linearization', page: 'p. 183 · Eq. 11.38',
          theory: '\\tau = \\tau_{fl}(\\theta) - Kx + k_r\\theta_r',
          numbers: `\\tau = \\tau_{fl}(\\theta) - (${tex(g.K[0])}\\,\\theta + ${tex(g.K[1])}\\,\\dot\\theta) + ${tex(g.kr)}\\,\\theta_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch11;
      const ref = () => design(ctx, { ...ctx.st, tr: prob.tr, zeta: prob.zeta, rule: '2.2' }, 'sf');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) Desired closed-loop poles (t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta})`,
          inputs: { p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>' },
          html: 'Use ω<sub>n</sub> = 2.2/t<sub>r</sub>. Complex values are fine: <code>-1+2j</code>.',
          check: (v) => {
            const g = [M.parseComplex(v.p1), M.parseComplex(v.p2)];
            if (!g[0] || !g[1]) return { ok: false, msg: 'Enter both poles.' };
            return M.polesMatch(g, ref().poles) ? { ok: true, msg: '' } : { ok: false, msg: 'Roots of s² + 2ζωₙs + ωₙ²?' };
          },
          solution: () => [{ tex: `\\omega_n = 2.2/${prob.tr} = ${tex(2.2 / prob.tr)},\\quad p = ${ref().poles.map((q) => texPole(q)).join(',\\;')}` }],
        },
        {
          id: 'b', title: '(b) State-space model',
          html: 'Use your A, B, C, D from A.6 (the Ch 6 tab). The model card in the live math unlocks once A.6 is solved.',
        },
        {
          id: 'c', title: '(c) Controllability',
          inputs: { c11: 'c<sub>11</sub>', c12: 'c<sub>12</sub>', c21: 'c<sub>21</sub>', c22: 'c<sub>22</sub>', rank: 'rank' },
          check: (v) => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return PD().checkNumbers(v, { c11: Cm[0][0], c12: Cm[0][1], c21: Cm[1][0], c22: Cm[1][1], rank: L.rank(Cm) }, {}); },
          solution: () => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return [{ tex: `\\mathcal{C}_{A,B} = ${texMat(Cm)},\\; \\det = ${tex(Cm[0][0] * Cm[1][1] - Cm[0][1] * Cm[1][0])} \\ne 0` }]; },
        },
        {
          id: 'd', title: `(d) K and k<sub>r</sub> for t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta}`,
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], kr: r.kr }, {}); },
          actions: [{ label: 'Use my gains', run: (v) => {
            const vals = ['K1', 'K2', 'kr'].map((k) => PD().num(v[k]));
            if (vals.some((x) => x === null)) return { ok: false, msg: 'Enter K1, K2 and kr.' };
            ctx.app.setMode('work'); [ctx.st.w.K1, ctx.st.w.K2, ctx.st.w.kr] = vals; ctx.update(); return null;
          } }],
          solution: () => {
            const r = ref();
            return [
              { tex: `\\Delta^d = ${WB.tf.polyTex(L.polyFromRoots(r.poles))},\\quad K = ${texMat([r.K])},\\quad k_r = ${tex(r.kr)}` },
              { html: 'Why K = (k<sub>P</sub>, k<sub>D</sub>) for the same poles: with x = (θ, θ̇), −Kx is −k<sub>P</sub>θ − k<sub>D</sub>θ̇, and k<sub>r</sub>θ<sub>r</sub> = k<sub>P</sub>θ<sub>r</sub>. That is exactly PD with the derivative on the output (Fig. 7-2). Book: p. 187.' },
            ];
          },
        },
        {
          id: 'e', title: '(e) Implement with a digital differentiator',
          html: 'In your <code>ctrlStateFeedback.py</code>, estimate θ̇ from θ with the dirty derivative (Eq. 10.4). The simulation here applies your K and k<sub>r</sub> from the gain sliders.',
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 12 --
  WB.chapters.ch12 = base('sfi', 12, 'Integrator with state feedback', 'pp. 197–214', {
    defaults(sys) { const p = sys.problems.ch12; return { comp: 'fl', est: 'true', tr: p.tr, zeta: p.zeta, rule: '2.2', pI: p.pI, antiwindup: 'clamp', w: { ...W0 } }; },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'u = −Kx − k_i ∫(r − y)', 'p. 199');
      compControls(sec, ctx);
      segmented(sec, {
        label: 'Anti-windup (A.12a)',
        options: [{ value: 'clamp', label: 'hold integrator while saturated' }, { value: 'none', label: 'none (repo)' }],
        ...bind(ctx, 'antiwindup'),
      });
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['K1', 'K2', 'ki']);
      else { tuningSliders(sec, ctx, { pI: true }); gainsReadout(sec, ctx, ['K1', 'K2', 'ki']); }
    },
    extraPlot(ctx, res) {
      return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [rad·s]', unit: 'rad·s' }, data: { series: [{ label: 'x_I = ∫(r − θ)', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = design(ctx);
      const { A1, B1 } = augI(ctx.ss);
      return [
        ssCard(ctx),
        // A₁, B₁ hold the numbers of A and B (A.6).
        { title: 'Augmented system', page: 'p. 198 · Eq. 12.1', answers: PD().partKey(ctx, 'ch6', 'a'),
          theory: '\\dot x_I = r - C_r x,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C_r & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\ 0\\end{bmatrix}',
          numbers: `A_1 = ${texMat(A1)},\\quad B_1 = ${texMat(B1)}` },
        { ...ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 198'), answers: PD().partKey(ctx, 'ch6', 'a') },
        polesCard(ctx, d),
        { title: 'Gains', page: 'p. 199–201', answers: PD().partKey(ctx, 'ch12', 'a'),
          theory: 'K_1 = \\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad u = -Kx - k_I\\int_0^t (r - y)\\,d\\tau',
          numbers: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch12;
      const ref = () => design(ctx, { ...ctx.st, tr: prob.tr, zeta: prob.zeta, pI: prob.pI, rule: '2.2' }, 'sfi');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) Gains with t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta}, p<sub>I</sub> = ${prob.pI}`,
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', ki: 'k<sub>I</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], ki: r.ki }, {}); },
          actions: [{ label: 'Use my gains', run: (v) => {
            const vals = ['K1', 'K2', 'ki'].map((k) => PD().num(v[k]));
            if (vals.some((x) => x === null)) return { ok: false, msg: 'Enter K1, K2 and kI.' };
            ctx.app.setMode('work'); [ctx.st.w.K1, ctx.st.w.K2, ctx.st.w.ki] = vals; ctx.update(); return null;
          } }],
          solution: () => {
            const r = ref();
            return [
              { tex: `K = ${texMat([r.K])},\\quad k_I = ${tex(r.ki)}` },
              { html: 'The book\'s printed numbers (p. 204–205: K = (1.0370, 0.1817), k<sub>I</sub> = −2.2687) correspond to t<sub>r</sub> = 0.4, not the 0.489 in its own listing and the repo, and use a<sub>A1</sub> = 0.6174 where A gives 0.667.' },
            ];
          },
        },
        {
          id: 'b', title: '(b) Disturbance and 20% uncertainty',
          html: 'Set an input disturbance d and the true-plant mismatch in the left panel (the chapter starts with d = 0.25 N·m and a 20% draw).',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch).some((v) => Math.abs(v) > 0);
            return Math.abs(S.sim.dist) > 0 && mis ? { ok: true, msg: `d = ${fmt(S.sim.dist, 3)} N·m with plant mismatch.` } : { ok: false, msg: 'Set both d ≠ 0 and a plant mismatch.' };
          },
        },
        {
          id: 'c', title: '(c) Tune for good tracking',
          html: 'Passes when the error just before the first reference switch is under 0.1° with the current disturbance and mismatch.',
          check: () => {
            const res = ctx.app.result(), S = ctx.S;
            const tSw = WB.sim.switchTime(S);
            const i = WB.sim.indexBefore(S, res, tSw);
            const e = Math.abs(res.r[i] - res.y[i]) / M.DEG;
            return { ok: e < 0.1, msg: `Error before the switch: ${fmt(e, 3)}° (d = ${fmt(S.sim.dist, 3)} N·m).` };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 13 --
  WB.chapters.ch13 = base('obs', 13, 'Observers', 'pp. 215–238', {
    defaults(sys) {
      const p = sys.problems.ch13;
      return { comp: 'fl', tr: p.tr, zeta: p.zeta, rule: '2.2', pI: p.pI, antiwindup: 'clamp', wnObs: 2.2 / (p.tr / p.trObsFactor), zetaObs: p.zetaObs, xhat0: 0, w: { ...W0 } };
    },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries(ctx, res, deg) { return [{ label: 'θ̂ (observer)', y: deg(res.extras.xhat0 || []), color: '--series-3', dash: [3, 3], width: 2 }]; },
    extraPlot(ctx, res) {
      const k = 180 / Math.PI;
      return {
        opts: { title: 'θ̇(t) and estimate', yLabel: 'θ̇ [°/s]', unit: '°/s' },
        data: { series: [
          { label: 'θ̇̂ (observer)', y: Array.from(res.extras.xhat1 || [], (v) => v * k), color: '--series-3', dash: [3, 3], width: 2 },
          { label: 'true θ̇', y: res.x.map((x) => x[1] * k), color: '--series-1' },
        ] },
      };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂)', 'p. 222 · Fig. 13-3');
      compControls(sec, ctx);
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['K1', 'K2', 'ki']);
      else tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer', 'p. 216 · Eq. 13.3');
      if (ctx.S.mode === 'work') workSliders(ob, ctx, ['L1', 'L2']);
      else { obsSliders(ob, ctx, false); gainsReadout(ob, ctx, ['K1', 'K2', 'ki', 'L1', 'L2']); }
      slider(ob, { label: 'θ̂(0)', unit: '°', min: -60, max: 60, step: 1, sig: 3, hint: 'initial estimate (true θ starts at the left panel value)', ...bind(ctx, 'xhat0') });
    },
    math(ctx) {
      const d = design(ctx);
      const { A, C } = ctx.ss;
      const O = L.obsv(A, C);
      return [
        ssCard(ctx),
        { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 224',
          theory: '\\dot{\\hat x} = A\\hat x + B(u - \\tau_{fl}(\\hat\\theta)) + L(y - C\\hat x),\\quad \\dot e = (A - LC)e' },
        { title: 'Observability', page: 'p. 221', answers: PD().partKey(ctx, 'ch13', 'b'),
          theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ \\vdots \\\\ CA^{n-1}\\end{bmatrix},\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = n',
          numbers: `\\mathcal{O} = ${texMat(O)},\\quad \\operatorname{rank} = ${L.rank(O)}`, spoiler: true },
        { title: 'Observer gain', page: 'p. 221 · Eq. 13.16, p. 222', answers: PD().partKey(ctx, 'ch13', 'c'),
          theory: 'L = \\mathcal{O}_{A,C}^{-1}\\mathcal{A}_A^{-T}(\\beta - a_A)^\\top = \\text{place}(A^\\top, C^\\top, q)^\\top',
          numbers: `q = ${d.obsPoles.map((p) => texPole(p)).join(',\\;')},\\quad L = ${texMat(d.L)}`, spoiler: true },
        { title: 'Separation principle', page: 'p. 223–224',
          theory: '\\begin{bmatrix}\\dot x\\\\ \\dot e\\end{bmatrix} = \\begin{bmatrix}A - BK & BK\\\\ 0 & A - LC\\end{bmatrix}\\begin{bmatrix}x\\\\ e\\end{bmatrix} \\Rightarrow \\text{eig} = \\text{eig}(A - BK) \\cup \\text{eig}(A - LC)',
          note: 'Holds for the linear model only. Saturation, mismatch and the nonlinear τ_fl break it (p. 224).' },
        polesCard(ctx, d),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch13;
      const ref = () => design(ctx, { ...ctx.st, tr: prob.tr, zeta: prob.zeta, pI: prob.pI, rule: '2.2', wnObs: 2.2 / (prob.tr / prob.trObsFactor), zetaObs: prob.zetaObs }, 'obs');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Exact parameters, no disturbance',
          html: 'The chapter starts with α = 0 and d = 0. <em>Exact model</em> in the left panel restores them.',
        },
        {
          id: 'b', title: '(b) Observability',
          inputs: { rank: 'rank 𝒪<sub>A,C</sub>' },
          check: (v) => PD().checkNumbers(v, { rank: L.rank(L.obsv(ctx.ss.A, ctx.ss.C)) }, {}),
          solution: () => [{ tex: `\\mathcal{O}_{A,C} = ${texMat(L.obsv(ctx.ss.A, ctx.ss.C))} \\Rightarrow \\text{rank } 2` }],
        },
        {
          id: 'c', title: `(c) Observer gain for ω<sub>n,obs</sub> = 2.2/(t<sub>r</sub>/${prob.trObsFactor}), ζ<sub>obs</sub> = ${prob.zetaObs} (t<sub>r</sub> = ${prob.tr})`,
          html: 'The repo\'s tuning (ctrlObserver.py): observer 10× faster than the controller.',
          inputs: { L1: 'L<sub>1</sub>', L2: 'L<sub>2</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { L1: r.L[0], L2: r.L[1] }, {}); },
          solution: () => { const r = ref(); return [{ tex: `L = ${texMat(r.L)},\\quad K = ${texMat([r.K])},\\; k_I = ${tex(r.ki)}` }]; },
        },
        {
          id: 'd', title: '(d) Plot the state and the estimate',
          html: 'The θ plot shows the estimate θ̂ with the true θ; the extra plot shows θ̇ and its estimate.',
        },
        {
          id: 'e', title: '(e) Add d = 0.01 N·m',
          html: 'Set d = 0.01 in the left panel. The integrator acts on r − θ̂, and θ̂ is biased by the unmodeled d, so θ settles away from r. Chapter 14 removes the bias.',
          check: () => {
            const res = ctx.app.result();
            if (!(Math.abs(ctx.S.sim.dist) > 0)) return { ok: false, msg: 'Set d ≠ 0 first.' };
            const n = res.t.length - 1;
            const bias = (res.y[n] - (res.extras.xhat0 || [])[n]) / M.DEG;
            return { ok: true, msg: `At t_end: θ − θ̂ = ${fmt(bias, 3)}°.` };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 14 --
  WB.chapters.ch14 = base('dobs', 14, 'Disturbance observers', 'pp. 239–259', {
    defaults(sys) {
      const p = sys.problems.ch14;
      return { comp: 'fl', tr: p.tr, zeta: p.zeta, rule: 'tp', pI: p.pI, antiwindup: 'clamp', wnObs: p.wnObs, zetaObs: p.zetaObs, pD: p.pD, dobs: true, xhat0: 0, w: { ...W0 } };
    },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries(ctx, res, deg) { return [{ label: 'θ̂ (observer)', y: deg(res.extras.xhat0 || []), color: '--series-3', dash: [3, 3], width: 2 }]; },
    extraPlot(ctx, res) {
      const S = ctx.S;
      return {
        opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N·m]', unit: 'N·m' },
        data: { series: [
          { label: 'd̂ (estimate)', y: Array.from(res.extras.dhat || []), color: '--series-3', width: 2 },
          { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
        ] },
      };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'p. 245');
      compControls(sec, ctx);
      segmented(sec, {
        label: 'Disturbance observer',
        options: [{ value: true, label: 'on' }, { value: false, label: 'off (A.14a)' }],
        ...bind(ctx, 'dobs'),
      });
      if (ctx.S.mode === 'work') workSliders(sec, ctx, ['K1', 'K2', 'ki']);
      else tuningSliders(sec, ctx, { pI: true, rule: true });
      const ob = section(parent, 'Observer', 'p. 241');
      if (ctx.S.mode === 'work') workSliders(ob, ctx, ['L1', 'L2', 'Ld']);
      else { obsSliders(ob, ctx, true); gainsReadout(ob, ctx, ['K1', 'K2', 'ki', 'L1', 'L2', 'Ld']); }
    },
    math(ctx) {
      const d = design(ctx);
      const { A2, C2 } = augD(ctx.ss);
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(u + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0 \\text{ for constant } d' },
        { title: 'Augmented model (ḋ = 0)', page: 'p. 240', answers: PD().partKey(ctx, 'ch6', 'a'),
          theory: 'A_2 = \\begin{bmatrix}A & B\\\\ 0 & 0\\end{bmatrix},\\quad C_2 = \\begin{bmatrix}C & 0\\end{bmatrix}',
          numbers: `A_2 = ${texMat(A2)},\\quad \\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}` },
        { title: 'Disturbance observer', page: 'p. 241',
          theory: '\\dot{\\hat x} = A\\hat x + B(u + \\hat d) + L(y - C\\hat x),\\quad \\dot{\\hat d} = L_d(y - C\\hat x),\\quad u = -K\\hat x - k_I\\textstyle\\int e - \\hat d' },
        { title: 'Observer gains', page: 'p. 241', answers: PD().partKey(ctx, 'ch14', 'b1'),
          theory: '\\begin{bmatrix}L\\\\ L_d\\end{bmatrix} = \\text{place}(A_2^\\top, C_2^\\top, q)^\\top',
          numbers: `q = ${d.obsPoles.map((p) => texPole(p)).join(',\\;')},\\quad L = ${texMat(d.L)},\\; L_d = ${tex(d.Ld)}`, spoiler: true,
          note: 'The A.14 solution\'s observer (|q| ≈ 5.5–10) is slower than its controller (ω_n ≈ 12.6), the opposite of the usual "observer 5–10× faster" rule.' },
        polesCard(ctx, d),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch14;
      const ref = () => design(ctx, { ...ctx.st, tr: prob.tr, zeta: prob.zeta, rule: 'tp', pI: prob.pI, wnObs: prob.wnObs, zetaObs: prob.zetaObs, pD: prob.pD }, 'dobs');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Large disturbance, noise, 20% uncertainty',
          html: 'The chapter starts with α = 0.2, d = 0.5 N·m and noise σ = 0.001 rad (0.0573°). Turn the disturbance observer off to see the controller fail to reject d.',
        },
        {
          id: 'b1', title: `(b) Observer gains for ω<sub>n,obs</sub> = ${prob.wnObs}, ζ<sub>obs</sub> = ${prob.zetaObs}, p<sub>d</sub> = ${prob.pD}`,
          inputs: { L1: 'L<sub>1</sub>', L2: 'L<sub>2</sub>', Ld: 'L<sub>d</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { L1: r.L[0], L2: r.L[1], Ld: r.Ld }, {}); },
          solution: () => { const r = ref(); return [{ tex: `L = ${texMat(r.L)},\\; L_d = ${tex(r.Ld)};\\quad K = ${texMat([r.K])},\\; k_I = ${tex(r.ki)}` }]; },
        },
        {
          id: 'b2', title: '(b) Estimator bias removed',
          html: 'Passes when the observer bias |θ − θ̂| at t<sub>end</sub> is under 0.05°. Compare with the observer switched off.',
          check: () => {
            const res = ctx.app.result(), S = ctx.S, n = res.t.length - 1;
            if (!ctx.st.dobs) return { ok: false, msg: 'Turn the disturbance observer on.' };
            const dh = (res.extras.dhat || [])[n], bias = Math.abs(res.y[n] - res.extras.xhat0[n]) / M.DEG;
            const ok = bias < 0.05;
            return { ok, msg: `d̂ = ${fmt(dh, 3)} vs d = ${fmt(S.sim.dist, 3)}; |θ − θ̂| = ${fmt(bias, 3)}°.` };
          },
          solution: () => [{ html: 'd̂ estimates the input disturbance plus any torque the model gets wrong (mismatch, gravity residual), so it settles near d but not exactly on it.' }],
        },
      ]);
    },
  });

  WB.ss = { makeSS, design, ctrbCard };
})();
