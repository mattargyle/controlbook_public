// D.7 (PD pole placement), D.8 (second-order design and saturation), D.9 (system
// type), D.P.6 (root locus versus k_I) and D.10 (digital PID) on
//   P(s) = b0 / (s^2 + a1 s + a0),  b0 = 1/m, a1 = b/m, a0 = k/m.
//
// Spring compensation. Fig. 7-2 PD on this type-0 plant leaves a steady-state
// error k/(k + kP) for a step. D.10's wording ("the steady state error caused by
// the uncertain parameters") implies the controller also supplies the equilibrium
// force F_e = k z_e with z_e = z_r. The 'eq' option adds F_e = k z_r; 'none' is
// Fig. 7-2 exactly (the default through D.P.6; D.10 defaults to 'eq'). Neither
// changes the closed-loop poles. See ISSUES.md.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const { tex, texPole, fmt, fmtPole } = M;
  const lib = WB.studies.D.lib;
  const ans = WB.systems.D.answers;
  const CH = WB.studies.D.chapters;

  // ---------------------------------------------------------- controllers --
  // PD with the derivative on the true output rate (Ch 7-8 code) or on the error.
  function makePD(ctx) {
    const { kP, kD } = ctx.gains;
    const st = ctx.st, p = ctx.pModel, Ts = ctx.S.sim.Ts;
    let rPrev = null;
    return {
      update(r, x) {
        const y = x[0], ydot = x[1];
        if (rPrev === null) rPrev = r;
        const rdot = (r - rPrev) / Ts;
        rPrev = r;
        const dTerm = st.arch === 'error' ? kD * (rdot - ydot) : -kD * ydot;
        const ff = lib.comp(ctx) === 'eq' ? p.k * r : 0;
        return ff + kP * (r - y) + dTerm;
      },
    };
  }

  // PID with the derivative on the output (Listing 10.1/10.2 conventions:
  // error_prev = 0, y_prev = first sample, trapezoid integrator, dirty derivative
  // Eq. 10.4). deriv 'state' uses the true ż (as the Ch 9 code does).
  function makePID(ctx, { linear = false } = {}) {
    const { kP, kI, kD } = ctx.gains;
    const st = ctx.st, p = ctx.pModel, Ts = ctx.S.sim.Ts;
    const sigma = st.sigma ?? 0.05;
    const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    const lim = p.Fmax;
    const ff = lib.comp(ctx) === 'eq';
    let I = 0, ePrev = 0, yPrev = null, ydot = 0;
    return {
      update(r, x, yMeas) {
        const dirty = st.deriv === 'dirty';
        const y = dirty ? yMeas : x[0];
        const e = r - y;
        if (yPrev === null) yPrev = y;
        ydot = dirty ? beta * ydot + gamma * (y - yPrev) : x[1];
        const integrate = st.antiwindup !== 'gate' || Math.abs(ydot) < (st.vbar ?? 0.05);
        if (kI !== 0 && integrate) I += (Ts / 2) * (e + ePrev);
        let u = (ff ? p.k * r : 0) + kP * e + kI * I - kD * ydot;
        if (!linear && st.antiwindup === 'backcalc' && kI !== 0) {
          const us = M.saturate(u, lim);
          if (us !== u) { I += (us - u) / kI; u = us; }  // unwind so u_unsat sits at the limit
        }
        ePrev = e; yPrev = y;
        return { u, integrator: I, zdotHat: ydot };
      },
    };
  }

  // Closed loop with D on the output: s^3 + (a1 + b0 kD) s^2 + (a0 + b0 kP) s + b0 kI
  function pidCharPoly(t, g) { return [1, t.a1 + t.b0 * g.kD, t.a0 + t.b0 * g.kP, t.b0 * g.kI]; }
  // Python-check truths at complex s (a = {s, kP, kD}): D.7(b) and D.P.6.
  const clDen = (p, a) => { const t = ans.tf(p); return WB.py.cx.poly([1, t.a1 + t.b0 * a.kD, t.a0 + t.b0 * a.kP], a.s); };
  const evansL = (p, a) => { const t = ans.tf(p); return WB.py.cx.div(t.b0, WB.py.cx.poly([1, t.a1 + t.b0 * a.kD, t.a0 + t.b0 * a.kP, 0], a.s)); };
  function clPoles(t, g) {
    return g.kI ? L.roots(pidCharPoly(t, g)) : M.roots2(t.a1 + t.b0 * g.kD, t.a0 + t.b0 * g.kP);
  }

  // ------------------------------------------------------------- controls --
  function compControl(parent, ctx) {
    // F_e answers D.4(a): Work mode offers the compensation only once it is solved.
    if (!lib.shows(ctx, 'D.4/a')) {
      lib.note(parent, 'Spring compensation (adding the equilibrium force F_e outside the loop) unlocks once you solve D.4(a). Until then the loop runs as in Fig. 7-2.');
      return;
    }
    segmented(parent, {
      label: 'Spring compensation',
      options: [
        { value: 'eq', label: lib.compLabel(ctx), title: 'equilibrium force F_e at z_e = z_r, added outside the loop' },
        { value: 'none', label: 'none (Fig. 7-2)', title: 'F = F̃: pure PD/PID' },
      ],
      ...bind(ctx, 'comp'),
    });
  }
  function archControl(parent, ctx) {
    segmented(parent, {
      label: 'Derivative acts on',
      options: [{ value: 'output', label: 'output z (Fig. 7-2)' }, { value: 'error', label: 'error e (Fig. 7-1)' }],
      ...bind(ctx, 'arch'),
    });
  }
  const R = { kP: 20, kD: 30, kI: 10 };
  function gainSliders(parent, ctx, keys) {
    const lab = { kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>', kI: 'k<sub>I</sub>' };
    for (const k of keys) lib.gainSlider(parent, ctx, k, lab[k], R[k]);
  }
  function specSliders(parent, ctx, { kI } = {}) {
    slider(parent, { label: 't<sub>r</sub>', unit: 's', min: 0.2, max: 6, step: 0.005, sig: 4, ...bind(ctx, 'tr') });
    slider(parent, { label: 'ζ', min: 0.1, max: 2, step: 0.005, ...bind(ctx, 'zeta') });
    if (kI) lib.gainSlider(parent, ctx, 'kIx', 'k<sub>I</sub>', R.kI);
  }
  // Gains from (t_r, ζ) with ω_n = 2.2/t_r (Eq. 8.5), plus the explore k_I.
  function designed(ctx) {
    const d = ans.spec(ctx.pModel, ctx.st.tr, ctx.st.zeta);
    return { kP: d.kP, kD: d.kD, kI: ctx.st.kIx || 0, wn: d.wn, poles: d.poles };
  }
  function readout(parent, ctx, keys = ['kP', 'kI', 'kD']) { lib.readout(parent, ctx, keys, () => ctx.gains); }
  // Drag a closed-loop pole: |p| sets ω_n (so t_r = 2.2/ω_n) and its angle sets ζ.
  function dragSpec(ctx, id, re, im) {
    re = Math.min(-0.01, re);
    const wn = Math.hypot(re, im);
    ctx.st.zeta = Math.max(0.1, Math.min(1, -re / wn));
    ctx.st.tr = Math.max(0.2, 2.2 / wn);
    ctx.update();
  }

  // The open-loop poles answer D.7(a): in Work mode they stay off until it is solved.
  function markers(ctx, { draggable = false } = {}) {
    const t = ctx.model, g = ctx.gains;
    const mk = lib.showOl(ctx) ? M.roots2(t.a1, t.a0).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole p${i + 1}` })) : [];
    clPoles(t, g).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole ${i + 1}`, dragId: draggable && Math.abs(p.im) > 1e-9 ? 0 : (draggable && !g.kI ? i : undefined) }));
    if (g.kI && g.kP) mk.push({ re: -g.kI / g.kP, im: 0, kind: 'zero', label: 'closed-loop zero −kI/kP' });
    if (!g.kI && archOf(ctx) === 'error' && g.kD > 1e-9) mk.push({ re: -g.kP / g.kD, im: 0, kind: 'zero', label: 'zero −kP/kD' });
    return mk;
  }

  // Work mode has no architecture or compensation switches (the plots run the
  // student's own controller), so the cards describe Fig. 7-2 as the problem draws it.
  const archOf = (ctx) => (ctx.S.mode === 'work' ? 'output' : ctx.st.arch);
  const explore = (ctx) => ctx.S.mode === 'explore';

  // ------------------------------------------------------------ math cards --
  function plantCard(ctx) {
    const t = ctx.model;
    return {
      title: 'Plant (D.5)', page: 'p. 378 · D.5, p. 99 · Eq. 7.1', answers: 'D.5/b',
      theory: 'P(s) = \\frac{b_0}{s^2 + a_1 s + a_0},\\quad b_0 = \\tfrac1m,\\; a_1 = \\tfrac bm,\\; a_0 = \\tfrac km',
      numbers: `P(s) = \\frac{${tex(t.b0)}}{s^2 + ${tex(t.a1)}\\,s + ${tex(t.a0)}}`,
    };
  }
  function pdLoopCard(ctx) {
    const t = ctx.model, g = ctx.gains, err = archOf(ctx) === 'error';
    const cl = M.roots2(t.a1 + t.b0 * g.kD, t.a0 + t.b0 * g.kP);
    return {
      title: `Closed loop, derivative on ${err ? 'error (Fig. 7-1)' : 'output (Fig. 7-2)'}`, page: err ? 'p. 100 · Eq. 7.4' : 'p. 101 · Eq. 7.5',
      theory: err ? '\\frac{Y}{R} = \\frac{b_0 k_D s + b_0 k_P}{s^2 + (a_1 + b_0 k_D)s + (a_0 + b_0 k_P)}' : '\\frac{Y}{R} = \\frac{b_0 k_P}{s^2 + (a_1 + b_0 k_D)s + (a_0 + b_0 k_P)}',
      symbolic: `\\Delta_{cl}(s) = s^2 + \\frac{b + k_D}{m}s + \\frac{k + k_P}{m}`,
      numbers: `\\Delta_{cl} = s^2 + ${tex(t.a1 + t.b0 * g.kD)}\\,s + ${tex(t.a0 + t.b0 * g.kP)},\\quad p_{cl} = ${texPole(cl[0])},\\; ${texPole(cl[1])}`,
      answers: 'D.7/b',
    };
  }
  function compCard(ctx) {
    const eq = lib.comp(ctx) === 'eq';
    return {
      title: 'Spring compensation', page: 'p. 59–60 (equilibrium input)', answers: eq ? 'D.4/a' : undefined,
      theory: eq ? 'F = F_e + \\tilde F,\\quad F_e = k z_e,\\; z_e = z_r' : 'F = \\tilde F \\quad(\\text{Fig. 7-2 exactly})',
      note: eq ? 'Feedforward outside the loop: the poles do not move, but the DC gain from z_r to z becomes 1 when k is known exactly.' : 'Without F_e the spring holds the mass short of z_r: D.9 asks how far.',
    };
  }

  // ------------------------------------------------- student controllers --
  const mis = (m) => Object.entries(m).map(([k, v]) => `${k} ${v > 0 ? '+' : ''}${v}%`).join(', ');
  // Work mode: the gain sliders only move the closed-loop poles in the s-plane; the
  // time plots show the student's own controller (WB.myCtrl), named by `part`.
  function workControls(parent, ctx, part, { keys = ['kP', 'kD'], note = '' } = {}) {
    const sec = section(parent, keys.length > 2 ? 'PID gains (s-plane)' : 'PD gains (s-plane)', keys.length > 2 ? 'p. 142' : 'p. 99–101');
    gainSliders(sec, ctx, keys);
    lib.note(sec, `These place the closed-loop × in the s-plane. They do not drive the simulation.${note ? ' ' + note : ''}`);
    WB.myCtrl.banner(section(parent, 'Your controller'), ctx, part);
  }

  // A student's PD controller (fed the state, as in the Ch 7-8 code) against the
  // workbench's (Fig. 7-2, derivative on the output, with or without F_e: the problem
  // doesn't say whether the PD adds the spring's equilibrium force, ISSUES.md) for a step,
  // with the nominal and a second parameter set. gainsFor(p) gives the design's kP, kD.
  function pdMatch(ctx, code, gainsFor, step, tEnd) {
    const cases = WB.myCtrl.paramCases(ctx).map((pc) => {
      const sc = WB.myCtrl.scenario(ctx, { params: pc.params, ref: { type: 'step', amplitude: step, tStep: 0 }, tEnd, feed: 'state' });
      const ref = (comp) => () => {
        const rc = WB.myCtrl.refCtx(ctx, sc);
        rc.gains = { ...gainsFor(pc.params), kI: 0 };
        rc.st = { ...ctx.st, arch: 'output', comp };
        return WB.myCtrl.reference(ctx, sc, makePD(rc));
      };
      return { sc, label: pc.label, refs: [ref('none'), ref('eq')] };
    });
    return WB.myCtrl.matchCheck(ctx, code, cases, { tol: 0.02 * step });
  }

  // Solution code: D.7(d), the D.8 rise-time design with the given tr line, and D.10(c).
  const D7_SOL = `class Controller:
    def __init__(self):
        # Delta_cl^d = (s + 1)(s + 1.5) = s^2 + 2.5 s + 1.5  (D.7(c))
        self.kp = P.m * 1.5 - P.k
        self.kd = P.m * 2.5 - P.b

    def update(self, z_r, x):
        z = x[0, 0]
        zdot = x[1, 0]
        F = self.kp * (z_r - z) - self.kd * zdot   # Fig. 7-2: D on the output
        return F
`;
  const PD_SOL = (trLine) => `class Controller:
    def __init__(self):
        ${trLine}
        zeta = 0.7
        wn = 2.2 / tr
        self.kp = P.m * wn**2 - P.k
        self.kd = 2 * P.m * zeta * wn - P.b

    def update(self, z_r, x):
        z = x[0, 0]
        zdot = x[1, 0]
        F = self.kp * (z_r - z) - self.kd * zdot
        return max(-P.Fmax, min(P.Fmax, F))
`;
  const D10_SOL = `class Controller:
    def __init__(self):
        tr = 2.0          # the D.8(a) design
        zeta = 0.7
        wn = 2.2 / tr
        self.kp = P.m * wn**2 - P.k
        self.kd = 2 * P.m * zeta * wn - P.b
        self.ki = 0.75
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.Ts) / (2 * self.sigma + P.Ts)
        self.z_dot = P.zdot0
        self.z_prev = P.z0
        self.error_prev = 0.0
        self.integrator = 0.0

    def update(self, z_r, y):
        z = y[0, 0]
        error = z_r - z
        # dirty derivative (Eq. 10.4)
        self.z_dot = self.beta * self.z_dot + 2 / (2 * self.sigma + P.Ts) * (z - self.z_prev)
        # anti-windup: integrate only while z_dot is small
        if abs(self.z_dot) < 0.05:
            self.integrator += P.Ts / 2 * (error + self.error_prev)
        F_e = P.k * z_r   # equilibrium force for z_e = z_r (D.4)
        F = F_e + self.kp * error + self.ki * self.integrator - self.kd * self.z_dot
        F = max(-P.Fmax, min(P.Fmax, F))
        self.error_prev = error
        self.z_prev = z
        return F
`;

  // -------------------------------------------------------------- D.7 --
  CH.ch7 = {
    id: 'ch7', num: 7, tab: 'Ch 7', title: 'Pole placement (PD)', pages: 'pp. 99–106, p. 379',
    controller: (ctx) => makePD(ctx),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePD),
    // Work mode simulates the student's D.7(d) controller, which gets the state.
    implement: { feed: 'state', linear: false },
    defaults(sys) {
      const pr = sys.problems.ch7;
      return { arch: 'output', comp: 'none', kP: 1, kD: 1, form: 'real', p1: pr.desiredPoles[0].re, p2: pr.desiredPoles[1].re, sigma: -1.25, wd: 0.5 };
    },
    simDefaults(sys) { return sys.problems.ch7.sim; },
    designPoles(ctx) {
      const st = ctx.st;
      return st.form === 'real' ? [{ re: st.p1, im: 0 }, { re: st.p2, im: 0 }] : [{ re: st.sigma, im: st.wd }, { re: st.sigma, im: -st.wd }];
    },
    gains(ctx) {
      if (ctx.S.mode === 'work') return { kP: ctx.st.kP, kD: ctx.st.kD, kI: 0 };
      return { ...ans.pdGains(ctx.pModel, this.designPoles(ctx)), kI: 0 };
    },

    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') {
        workControls(parent, ctx, 'D.7(d)', { note: 'The target poles from (c) are dashed rings. The open-loop poles stay hidden until you check (a).' });
        return;
      }
      const sec = section(parent, 'PD controller', 'p. 99–101');
      archControl(sec, ctx);
      compControl(sec, ctx);
      const des = section(parent, 'Desired closed-loop poles', 'p. 100');
      segmented(des, {
        label: 'Pole pair', options: [{ value: 'real', label: 'two real' }, { value: 'complex', label: 'complex pair' }],
        ...bind(ctx, 'form'),
      });
      const dis = (f) => () => ctx.st.form !== f;
      slider(des, { label: 'p<sub>1</sub>', min: -6, max: 0, step: 0.01, ...bind(ctx, 'p1'), disabled: dis('real') });
      slider(des, { label: 'p<sub>2</sub>', min: -6, max: 0, step: 0.01, ...bind(ctx, 'p2'), disabled: dis('real') });
      slider(des, { label: '−σ', min: -6, max: 0, step: 0.01, ...bind(ctx, 'sigma'), disabled: dis('complex') });
      slider(des, { label: 'ω<sub>d</sub>', unit: 'rad/s', min: 0, max: 6, step: 0.01, ...bind(ctx, 'wd'), disabled: dis('complex') });
      lib.note(des, 'Or drag a closed-loop pole. Drag off the real axis for a complex pair.');
      readout(des, ctx, ['kP', 'kD']);
    },

    splane(ctx) {
      const work = ctx.S.mode === 'work';
      const mk = markers(ctx, { draggable: !work });
      if (work) for (const p of ctx.sys.problems.ch7.desiredPoles) mk.push({ ...p, kind: 'target', label: 'target pole (problem)' });
      return { markers: mk };
    },
    onPoleDrag(ctx, id, re, im) {
      const st = ctx.st;
      re = Math.min(-0.01, re);
      if (Math.abs(im) > 0.05) { st.form = 'complex'; st.sigma = re; st.wd = Math.abs(im); } else {
        if (st.form === 'complex') st.p1 = st.p2 = st.sigma;
        st.form = 'real';
        if (id === 0) st.p1 = re; else st.p2 = re;
      }
      ctx.update();
    },

    math(ctx) {
      const t = ctx.model;
      const ol = M.roots2(t.a1, t.a0);
      const des = ctx.S.mode === 'work' ? ctx.sys.problems.ch7.desiredPoles : this.designPoles(ctx);
      const g = ans.pdGains(ctx.pModel, des);
      return [
        { title: 'Open-loop poles', page: 'p. 99',
          theory: '\\text{open-loop poles} = \\text{roots of the denominator of } P(s)' },
        plantCard(ctx),
        { title: 'Open-loop poles of the mass-spring-damper', page: 'p. 99', answers: 'D.7/a',
          theory: 'p_{ol} = -\\frac{b}{2m} \\pm \\sqrt{\\left(\\frac{b}{2m}\\right)^2 - \\frac km}',
          numbers: `p_{ol} = ${texPole(ol[0], 4)},\\; ${texPole(ol[1], 4)}` },
        pdLoopCard(ctx),
        { title: ctx.S.mode === 'work' ? 'Pole placement (problem targets)' : 'Pole placement (your design)', page: 'p. 100',
          theory: '\\Delta^d_{cl}(s) = (s - p_1)(s - p_2) = s^2 + \\alpha_1 s + \\alpha_0,\\quad \\text{set } \\Delta_{cl}(s) = \\Delta^d_{cl}(s) \\text{ and match coefficients}',
          symbolic: 'k_P = \\frac{\\alpha_0 - a_0}{b_0},\\quad k_D = \\frac{\\alpha_1 - a_1}{b_0}',
          numbers: `\\Delta^d_{cl} = s^2 + ${tex(g.alpha1)}\\,s + ${tex(g.alpha0)} \\Rightarrow k_P = ${tex(g.kP)},\\; k_D = ${tex(g.kD)}`, answers: 'D.7/c' },
        ...(explore(ctx) ? [compCard(ctx)] : []),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch7;
      const t = () => ctx.model;
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Open-loop poles (z<sub>e</sub> = 0)',
          inputs: { p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>' },
          html: 'Complex values are fine: <code>-1+2j</code>.',
          check: (v) => {
            const gp = [M.parseComplex(v.p1), M.parseComplex(v.p2)];
            if (!gp[0] || !gp[1]) return { ok: false, msg: 'Enter both poles.' };
            const ok = M.polesMatch(gp, ans.olPoles(ctx.pModel));
            return lib.passed(ctx, ok ? { ok, msg: 'Lightly damped: ζ ≈ 0.065.' } : { ok, msg: 'The open-loop poles are the roots of the denominator of P(s).' });
          },
          solution: () => {
            const ol = ans.olPoles(ctx.pModel);
            return [{ tex: `\\Delta_{ol} = s^2 + \\tfrac bm s + \\tfrac km = s^2 + ${tex(t().a1)}s + ${tex(t().a0)} \\Rightarrow p = ${texPole(ol[0], 4)},\\; ${texPole(ol[1], 4)}` }];
          },
        },
        {
          id: 'b', title: '(b) Closed-loop transfer function and closed-loop poles',
          html: 'Derivative on the output (Fig. 7-2). Write the transfer function from z<sub>r</sub> to z and the closed-loop characteristic polynomial (its roots are the closed-loop poles) in terms of k<sub>P</sub> and k<sub>D</sub>; the check calls them at complex s. Any nonzero multiple of Δ<sub>cl</sub> is accepted.',
          code: lib.pyPart(ctx, {
            args: { s: lib.sArg, kP: { label: 'kP', lo: 0.5, hi: 20 }, kD: { label: 'kD', lo: 0.5, hi: 30 } },
            items: [
              { fn: 'closed_loop', args: ['s', 'kP', 'kD'], truth: (p, a) => WB.py.cx.div(ans.tf(p).b0 * a.kP, clDen(p, a)) },
              { fn: 'char_poly', args: ['s', 'kP', 'kD'], compare: 'scale', truth: clDen },
            ],
          }, 'def closed_loop(s, kP, kD):\n    # Z(s)/Z_r(s)\n    return ...\n\ndef char_poly(s, kP, kD):\n    # Delta_cl(s)\n    return ...\n'),
          solution: () => [
            { tex: '\\frac{Z(s)}{Z_r(s)} = \\frac{k_P/m}{s^2 + \\frac{b + k_D}{m}s + \\frac{k + k_P}{m}},\\quad \\Delta_{cl}(s) = s^2 + \\frac{b + k_D}{m}s + \\frac{k + k_P}{m}' },
            { tex: 'p_{1,2} = -\\frac{b + k_D}{2m} \\pm \\sqrt{\\left(\\frac{b + k_D}{2m}\\right)^2 - \\frac{k + k_P}{m}}' },
            { code: 'def char_poly(s, kP, kD):\n    return (s**2 + (P.b + kD) / P.m * s\n            + (P.k + kP) / P.m)\n\ndef closed_loop(s, kP, kD):\n    return kP / P.m / char_poly(s, kP, kD)' },
            { html: 'From Eq. 7.5 (p. 101). With the spring compensation F = kz<sub>r</sub> + F̃ the numerator becomes (k + k<sub>P</sub>)/m, so the DC gain is 1; the poles are the same.' },
          ],
        },
        {
          id: 'c', title: '(c) Gains for poles at −1 and −1.5',
          inputs: { kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>' },
          check: (v) => { const g = ans.pdGains(ctx.pModel, prob.desiredPoles); return lib.check(v, { kP: g.kP, kD: g.kD }, { kP: 'kP', kD: 'kD' }); },
          actions: [{ label: 'Use my gains', run: (v) => {
            const kP = lib.num(v.kP), kD = lib.num(v.kD);
            if (kP === null || kD === null) return { ok: false, msg: 'Enter kP and kD first.' };
            ctx.app.setMode('work'); Object.assign(ctx.st, { kP, kD }); ctx.update(); return null;
          } }],
          solution: () => {
            const g = ans.pdGains(ctx.pModel, prob.desiredPoles);
            return [{ tex: `\\Delta^d = (s+1)(s+1.5) = s^2 + 2.5s + 1.5 \\Rightarrow k_P = m\\,(1.5) - k = ${tex(g.kP)},\\; k_D = m\\,(2.5) - b = ${tex(g.kD)}` }];
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'd', title: '(d) Implement the PD control and plot the response to a 1 m step',
          html: 'Write the controller with the gains from (c). <code>update</code> gets the reference and the state x, as in the Ch 7 code. <em>Run my controller</em> drives the time plots; the check simulates a 1 m step with the nominal and with other parameters and compares z(t) with the design (within 2% of the step).',
          check: (code) => pdMatch(ctx, code, (p) => ans.pdGains(p, prob.desiredPoles), 1, 12),
          solution: () => [
            { code: D7_SOL },
            { html: 'Fig. 7-2 exactly (D on the output, no saturation yet). Two real poles: no overshoot, and the slower pole at −1 gives a 2% settling time of about 5 s (the 4/σ rule of thumb with σ = 1 gives 4 s; the second pole stretches it). This loop settles at k<sub>P</sub>/(k + k<sub>P</sub>) of the step (D.9 explains why). Adding the equilibrium force F<sub>e</sub> = kz<sub>r</sub> (D.4) makes it reach 1 m and passes too, but with these gains it asks for 7.5 N at the step, above the 6 N limit that D.8(b) introduces.' },
          ],
        }),
      ]);
    },
  };

  // -------------------------------------------------------------- D.8 --
  // Largest kP so the demand right after the step fits: (Fmax - F_e) / step (Eq. 8.8)
  function satBound(ctx, step, zeta, comp) {
    const p = ctx.pModel, t = ans.tf(p);
    const Fe = comp === 'eq' ? p.k * step : 0;
    const kP = (p.Fmax - Fe) / step;
    const wn = Math.sqrt(t.a0 + t.b0 * kP);
    return { Fe, kP, wn, tr: 2.2 / wn, kD: (2 * zeta * wn - t.a1) / t.b0 };
  }

  CH.ch8 = {
    id: 'ch8', num: 8, tab: 'Ch 8', title: 'Second-order design', pages: 'pp. 107–136, p. 379',
    controller: (ctx) => makePD(ctx),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePD),
    targets: (ctx) => ({ tr: ctx.st.tr, zeta: ctx.st.zeta }),
    defaults(sys) { const pr = sys.problems.ch8; return { arch: 'output', comp: 'none', kP: 1, kD: 1, tr: pr.tr, zeta: pr.zeta }; },
    simDefaults(sys) { return sys.problems.ch8.sim; },
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kD: ctx.st.kD, kI: 0 } : { ...designed(ctx), kI: 0 }; },
    // Work mode simulates the student's D.8 controller, which gets the state.
    implement: { feed: 'state', linear: false },

    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') workControls(parent, ctx, 'D.8(a) and (b)');
      else {
        const sec = section(parent, 'PD controller', 'p. 119 · Fig. 8-12');
        archControl(sec, ctx);
        compControl(sec, ctx);
      }
      const spec = section(parent, ctx.S.mode === 'work' ? 'Specs (targets)' : 'Design knobs', 'p. 113 · Eq. 8.5');
      specSliders(spec, ctx);
      if (ctx.S.mode === 'explore') {
        lib.note(spec, 'Drag a closed-loop pole: its distance from the origin sets ωₙ = 2.2/t_r and its angle sets ζ. The red region marks ωₙ beyond the saturation limit.');
        readout(spec, ctx, ['kP', 'kD']);
      }
    },

    splane(ctx) {
      const explore = ctx.S.mode === 'explore';
      const out = { markers: markers(ctx, { draggable: explore }), zetaRay: ctx.st.zeta < 1 ? ctx.st.zeta : null };
      if (explore) {
        out.wnCircle = 2.2 / ctx.st.tr;
        const sb = satBound(ctx, Math.abs(ctx.S.sim.amplitude - ctx.S.sim.y0) || 1, ctx.st.zeta, lib.comp(ctx));
        if (isFinite(sb.wn)) out.wnMax = sb.wn;
      }
      return out;
    },
    onPoleDrag: dragSpec,

    math(ctx) {
      const st = ctx.st;
      const d = ans.spec(ctx.pModel, st.tr, st.zeta);
      const step = Math.abs(ctx.S.sim.amplitude - ctx.S.sim.y0) || 1;
      // Work mode has no compensation switch: the bound for Fig. 7-2 as drawn (F_e = 0).
      const sb = satBound(ctx, step, st.zeta, explore(ctx) ? lib.comp(ctx) : 'none');
      return [
        plantCard(ctx), pdLoopCard(ctx),
        { title: 'Spec → desired characteristic polynomial', page: 'p. 110 · Eq. 8.2, p. 113 · Eq. 8.5',
          theory: '\\omega_n = \\frac{2.2}{t_r},\\quad \\Delta^d_{cl} = s^2 + 2\\zeta\\omega_n s + \\omega_n^2,\\quad p = -\\zeta\\omega_n \\pm j\\omega_n\\sqrt{1-\\zeta^2}',
          numbers: `\\omega_n = ${tex(d.wn)},\\quad \\Delta^d_{cl} = s^2 + ${tex(d.alpha1)}\\,s + ${tex(d.alpha0)},\\quad p = ${texPole(d.poles[0], 4)},\\; ${texPole(d.poles[1], 4)}`, answers: 'D.8/a' },
        { title: 'Gains from the spec', page: 'p. 100',
          theory: '\\text{set } \\Delta_{cl}(s) = \\Delta^d_{cl}(s) \\text{ and match coefficients}' },
        { title: 'Gains from the spec (this plant)', page: 'p. 100', answers: 'D.8/a',
          theory: 'k_P = \\frac{\\omega_n^2 - a_0}{b_0} = m\\omega_n^2 - k,\\quad k_D = \\frac{2\\zeta\\omega_n - a_1}{b_0} = 2m\\zeta\\omega_n - b',
          numbers: `k_P = ${tex(d.kP)},\\quad k_D = ${tex(d.kD)}` },
        { title: 'Saturation limits the rise time', page: 'p. 119 · Eq. 8.8, p. 121 · Fig. 8-13',
          theory: 'k_P \\le \\frac{\\tilde{u}_{max}}{e_{max}},\\quad \\omega_n \\le \\sqrt{a_0 + b_0\\frac{\\tilde{u}_{max}}{e_{max}}},\\quad t_r \\ge \\frac{2.2}{\\omega_{n,max}},\\quad \\tilde{u}_{max} = u_{max} - |u_e|',
          numbers: `F_e = ${tex(sb.Fe)},\\; e_{max} = ${tex(step)}\\,\\text{m}\\Rightarrow k_{P,max} = ${tex(sb.kP)},\\; \\omega_{n,max} = ${tex(sb.wn)},\\; t_{r,min} = ${tex(sb.tr)}\\,\\text{s}`, answers: 'D.8/b',
          note: explore(ctx) ? 'u_e is the equilibrium force the controller adds (it depends on the spring-compensation setting), which changes the D.8(b) answer.' : 'u_e is the equilibrium force the controller adds: 0 for Fig. 7-2 as drawn, k z_r if your controller adds it.' },
        ...(explore(ctx) ? [compCard(ctx)] : []),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch8;
      const ref = () => ans.spec(ctx.pModel, prob.tr, prob.zeta);
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) Desired characteristic polynomial, poles, k<sub>P</sub> and k<sub>D</sub> (t<sub>r</sub> ≈ ${prob.tr} s, ζ = ${prob.zeta})`,
          html: 'Enter ω<sub>n</sub>, the coefficients of Δ<sup>d</sup><sub>cl</sub>(s) = s² + α<sub>1</sub>s + α<sub>0</sub>, the poles −σ ± jω<sub>d</sub>, and your gains.',
          inputs: { wn: 'ω<sub>n</sub>', alpha1: 'α<sub>1</sub>', alpha0: 'α<sub>0</sub>', sig: 'σ', wd: 'ω<sub>d</sub>', kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>' },
          check: (v) => { const r = ref(); return lib.check(v, { wn: r.wn, alpha1: r.alpha1, alpha0: r.alpha0, sig: -r.poles[0].re, wd: Math.abs(r.poles[0].im), kP: r.kP, kD: r.kD }, { wn: 'ωn', alpha1: 'α1', alpha0: 'α0', sig: 'σ', wd: 'ωd' }); },
          actions: [{ label: 'Use my gains', run: (v) => {
            const kP = lib.num(v.kP), kD = lib.num(v.kD);
            if (kP === null || kD === null) return { ok: false, msg: 'Enter kP and kD first.' };
            ctx.app.setMode('work'); Object.assign(ctx.st, { kP, kD }); ctx.update(); return null;
          } }],
          solution: () => {
            const r = ref();
            return [
              { tex: `\\omega_n = \\frac{2.2}{${prob.tr}} = ${tex(r.wn)},\\quad \\Delta^d_{cl} = s^2 + ${tex(r.alpha1)}s + ${tex(r.alpha0)},\\quad p = ${texPole(r.poles[0], 4)},\\; ${texPole(r.poles[1], 4)}` },
              { tex: `k_P = m\\omega_n^2 - k = ${tex(r.kP)},\\quad k_D = 2m\\zeta\\omega_n - b = ${tex(r.kD)}` },
              { html: 'With these gains the 10–90% rise time comes out near 1.9 s (2.2/ω<sub>n</sub> is exact only for ζ ≈ 0.707), with about 4.6% overshoot.' },
            ];
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'a2', title: '(a) Verify the step response', seed: 'D.7/d',
          html: `Change your D.7 controller to the gains for t<sub>r</sub> = ${prob.tr} s, ζ = ${prob.zeta}. The check simulates a ${prob.step} m step with the nominal and with other parameters and compares z(t) with the design (within 2% of the step).`,
          check: (code) => pdMatch(ctx, code, (p) => ans.spec(p, prob.tr, prob.zeta), prob.step, 12),
          solution: () => [
            { code: PD_SOL(`tr = ${prob.tr}`) },
            { html: 'The 10–90% rise time comes out near 1.9 s (2.2/ω<sub>n</sub> is exact only for ζ ≈ 0.707), with about 4.6% overshoot of the final value. The saturation at F<sub>max</sub> is part (b); adding it here passes too.' },
          ],
        }),
        WB.myCtrl.part(ctx, {
          id: 'b', title: `(b) Add saturation; tune t<sub>r</sub> so a ${prob.step} m step just saturates`, seed: ['D.8/a2', 'D.7/d'],
          html: `Keep ζ = ${prob.zeta}. The check gives your controller a large error (its output must stay within ±F<sub>max</sub>), then simulates a ${prob.step} m step from rest: your peak |F| must reach 95% of F<sub>max</sub>, and F may sit at the limit for at most 2 samples.`,
          check: async (code) => {
            const Fmax = ctx.sys.uLimit(ctx.pModel);
            const pr = await WB.myCtrl.probe(ctx, code, [[10, [0, 0]]]);
            if (pr.ok === false) return pr;
            const u0 = pr.u[0][0];
            if (Math.abs(u0) > Fmax * (1 + 1e-9)) return { ok: false, msg: `For z_r = 10 m from rest your controller returns F = ${fmt(u0, 4)} N. Saturate its output at ±F_max (P.Fmax).` };
            const sc = WB.myCtrl.scenario(ctx, { ref: { type: 'step', amplitude: prob.step, tStep: 0 }, tEnd: 15, feed: 'state' });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            let peak = 0, nAt = 0;
            for (const u of res.uDemand) { peak = Math.max(peak, Math.abs(u)); if (Math.abs(u) >= Fmax * (1 - 1e-6)) nAt++; }
            const pct = (100 * peak / Fmax).toFixed(1);
            if (nAt > 2) return { ok: false, msg: `F sits at the limit for ${nAt} samples, so the input saturates. Slow it down.` };
            if (peak < 0.95 * Fmax) return { ok: false, msg: `Peak |F| is ${pct}% of Fmax. You can go faster.` };
            return { ok: true, msg: `Peak |F| is ${pct}% of Fmax${nAt ? `, at the limit for ${nAt} sample${nAt > 1 ? 's' : ''}` : ''}.` };
          },
          solution: () => {
            const none = satBound(ctx, prob.step, prob.zeta, 'none'), eq = satBound(ctx, prob.step, prob.zeta, 'eq');
            return [
              { html: 'The demand is largest right after the step, when z = 0 and ż = 0, so F(0<sup>+</sup>) = F<sub>e</sub> + k<sub>P</sub>·1 m ≤ F<sub>max</sub> (Eq. 8.8, p. 119). Then ω<sub>n</sub>² = (k + k<sub>P</sub>)/m and t<sub>r</sub> = 2.2/ω<sub>n</sub>.' },
              { tex: `F_e = 0 \\text{ (Fig. 7-2)}:\; k_P = ${tex(none.kP)},\; \\omega_n = ${tex(none.wn)},\; t_r = ${tex(none.tr)}\\,\\text{s},\; k_D = ${tex(none.kD)}` },
              { tex: `F_e = kz_r = ${tex(eq.Fe)}:\; k_P = ${tex(eq.kP)},\; \\omega_n = ${tex(eq.wn)},\; t_r = ${tex(eq.tr)}\\,\\text{s},\; k_D = ${tex(eq.kD)}` },
              { code: PD_SOL(`tr = ${fmt(none.tr * 1.005, 3)}   # just above t_r,min (F_e = 0)`) },
              { html: 'With F<sub>e</sub> = kz<sub>r</sub> the (a) design (k<sub>P</sub> = 3.05) already asks for 6.05 N, so it must be slowed slightly to t<sub>r</sub> ≈ 2.01 s; that version passes too. The problem does not say which architecture to use (ISSUES.md).' },
            ];
          },
        }),
      ]);
    },
  };

  // -------------------------------------------------------------- D.9 --
  CH.ch9 = {
    id: 'ch9', num: 9, tab: 'Ch 9', title: 'System type & integrators', pages: 'pp. 137–154, p. 380',
    defaults() { return { comp: 'none', deriv: 'state', antiwindup: 'none', kP: 1, kD: 1, kI: 0, tr: 2, zeta: 0.7, kIx: 0.5, input: 'step' }; },
    simDefaults(sys) { return sys.problems.ch9.sim; },
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD } : designed(ctx); },
    controller: (ctx, o) => makePID(ctx, o),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePID),
    reference: (ctx, base) => WB.pid.shapedReference(ctx, base),

    buildControls(parent, ctx) {
      const sec = section(parent, 'PID controller', 'p. 142');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Reference shape (amplitude = size, slope, or coefficient)',
        options: [{ value: 'step', label: 'step' }, { value: 'ramp', label: 'ramp' }, { value: 'parabola', label: 'parabola' }],
        ...bind(ctx, 'input'),
      });
      if (ctx.S.mode === 'work') gainSliders(sec, ctx, ['kP', 'kI', 'kD']);
      else { specSliders(sec, ctx, { kI: true }); readout(sec, ctx); }
      const ty = section(parent, 'System type analysis', 'p. 141 · Table 9-1');
      const box = el('div', { class: 'metrics' });
      ty.append(box);
      WB.ui.addRefresher(() => this.renderType(box, ctx));
    },

    renderType(box, ctx) {
      const a = ans.typeAnalysis(ctx.pModel, ctx.gains);
      const st = ctx.st, S = ctx.S, A = S.sim.amplitude;
      // With F = k z_r + F̃ (exact k) the spring feedforward cancels a0 in 1 − T, so the
      // tracking type goes up by one: PD gives e_ramp = (a1 + b0 kD)/(a0 + b0 kP), PID gives
      // e_parab = (a1 + b0 kD)/(b0 kI) per unit of R = 1/s^3.
      const eq = lib.comp(ctx) === 'eq', t = ctx.model, g = ctx.gains;
      const c1 = t.a1 + t.b0 * g.kD;
      const e = eq ? (g.kI > 0 ? { step: 0, ramp: 0, parab: c1 / (t.b0 * g.kI), type: 2 } : { step: 0, ramp: c1 / (t.a0 + t.b0 * g.kP), parab: Infinity, type: 1 }) : a;
      const pred = st.input === 'step' ? A * e.step : st.input === 'ramp' ? A * e.ramp : 2 * A * e.parab;
      const res = ctx.app.result();
      const n = res ? res.r.length - 1 : 0;
      const iD = res ? Math.min(n, Math.max(0, Math.round(S.sim.tDist / S.sim.Ts) - 1)) : 0;
      const eEnd = res ? res.r[n] - res.y[n] : NaN, eBefore = res && S.sim.dist && S.sim.tDist > S.sim.tStep ? res.r[iD] - res.y[iD] : NaN;
      const row = WB.ui.metric;
      const show = WB.ui.shown(ctx, 'D:ch9:type');
      const rows = [];
      if (show) {
        rows.push(row('reference tracking type', eq ? `type ${a.type} loop; type ${e.type} with F = k z_r + F̃` : `type ${a.type}`));
        rows.push(row(`predicted e_ss (${st.input}, ${eq ? 'with F = k z_r + F̃' : 'Fig. 7-2 loop'})`, isFinite(pred) ? `${fmt(pred, 3)} m` : '∞'));
        rows.push(row('input-disturbance type', `type ${a.distType}`));
        rows.push(row(`predicted extra e from d = ${fmt(S.sim.dist, 3)} N`, `${fmt(Math.abs(S.sim.dist) * a.dist, 3)} m`));
      } else {
        rows.push(WB.ui.revealButton(ctx, 'D:ch9:type', 'Reveal the predicted type and errors'));
      }
      if (isFinite(eBefore)) rows.push(row('simulated r − z just before d starts', `${fmt(eBefore, 3)} m`));
      rows.push(row('simulated r − z at t_end', isFinite(eEnd) ? `${fmt(eEnd, 3)} m` : '—'));
      box.replaceChildren(...rows);
    },

    splane(ctx) { return { markers: markers(ctx) }; },

    math(ctx) {
      const g = ctx.gains, a = ans.typeAnalysis(ctx.pModel, g);
      return [
        plantCard(ctx),
        { title: 'Final value theorem and tracking error', page: 'p. 137, p. 138',
          theory: '\\lim_{t\\to\\infty} e(t) = \\lim_{s\\to 0} sE(s),\\quad E(s) = \\frac{1}{1 + P(s)C(s)}R(s)' },
        { title: 'Error constants and system type', page: 'p. 141 · Table 9-1',
          theory: 'M_p = \\lim_{s\\to0} PC,\\quad M_v = \\lim_{s\\to0} sPC,\\quad e_{step} = \\tfrac{1}{1+M_p},\\; e_{ramp} = \\tfrac{1}{M_v},\\; e_{parab} = \\tfrac{1}{M_a}' },
        { title: g.kI > 0 ? 'Type of this loop with PID' : 'Type of this loop with PD', page: 'p. 141 · Table 9-1', answers: g.kI > 0 ? 'D.9/a2' : 'D.9/a1',
          theory: g.kI > 0 ? 'PC = \\frac{(k_D s^2 + k_P s + k_I)/m}{s\\,(s^2 + \\frac bm s + \\frac km)} \\Rightarrow \\text{type 1},\\; M_v = \\frac{k_I}{k}' : 'PC = \\frac{(k_D s + k_P)/m}{s^2 + \\frac bm s + \\frac km} \\Rightarrow \\text{type 0},\\; M_p = \\frac{k_P}{k}',
          numbers: g.kI > 0 ? `M_v = ${tex(a.Mv)},\\quad e_{ramp} = \\frac{k}{k_I} = ${tex(a.ramp)}` : `M_p = ${tex(a.Mp)},\\quad e_{step} = \\frac{k}{k + k_P} = ${tex(a.step)}`,
          note: 'For this plant the Fig. 7-1 and Fig. 7-2 loops give the same limits, because P(0) is finite.' },
        { title: 'Input disturbance', page: 'p. 143 · §9.1.3, Fig. 9-5',
          theory: 'E(s) = \\cdots + \\frac{P}{1+PC}D_{in}(s),\\quad \\lim_{s\\to0}\\frac{P}{1+PC}',
          note: 'Fig. 9-5 subtracts d_in at the plant input, hence the + sign. The workbench adds d (the plant sees u + d), so here E = −P/(1+PC)·D: same magnitude, opposite sign.' },
        { title: 'Input disturbance in this loop', page: 'p. 143 · §9.1.3', answers: 'D.9/b',
          theory: "\\lim_{s\\to0}\\frac{P}{1+PC} = \\frac{1}{k + k_P} \\;(\\text{PD}),\\quad 0 \\;(\\text{PID: integrator in } C)",
          numbers: g.kI > 0 ? '\\lim_{s\\to0}\\frac{P}{1+PC} = 0' : `\\lim_{s\\to0}\\frac{P}{1+PC} = ${tex(a.dist)}\\;\\text{m/N}` },
        compCard(ctx),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch9;
      const a = () => ans.typeAnalysis(ctx.pModel, ctx.gains);
      const g = () => ctx.gains;
      lib.panel(parent, ctx, prob, [
        {
          id: 'a1', title: '(a) PD only (k<sub>I</sub> = 0)',
          html: 'Set <code>system_type</code>, and write each steady-state error for a unit input (m) as a function of the gains. Return <code>np.inf</code> for an unbounded error. The check calls your functions at random gains.',
          code: lib.pyPart(ctx, {
            args: lib.gainArgs,
            items: [
              { var: 'system_type', truth: () => 0 },
              { fn: 'e_step', args: ['kP', 'kD'], truth: (p, a) => p.k / (p.k + a.kP) },
              { fn: 'e_ramp', args: ['kP', 'kD'], truth: () => Infinity },
              { fn: 'e_parab', args: ['kP', 'kD'], truth: () => Infinity },
            ],
          }, 'system_type = ...\n\ndef e_step(kP, kD):\n    return ...\n\ndef e_ramp(kP, kD):\n    return ...\n\ndef e_parab(kP, kD):\n    return ...\n'),
          solution: () => [
            { tex: `PC = \\frac{(k_D s + k_P)/m}{s^2 + \\frac bm s + \\frac km} \\Rightarrow \\text{type 0},\\quad M_p = \\frac{k_P}{k} = ${tex(a().Mp)}\;\\text{(current gains)}` },
            { tex: `e_{step} = \\frac{1}{1 + M_p} = \\frac{k}{k + k_P},\\quad e_{ramp} = e_{parab} = \\infty` },
            { code: 'system_type = 0\n\ndef e_step(kP, kD):\n    return P.k / (P.k + kP)    # 1/(1 + M_p), M_p = kP/k\n\ndef e_ramp(kP, kD):\n    return np.inf\n\ndef e_parab(kP, kD):\n    return np.inf' },
            { html: 'Try it: compensation <em>none</em>, step input, k<sub>I</sub> = 0, and compare r − z at t_end.' },
          ],
        },
        {
          id: 'a2', title: '(a) With the integrator (k<sub>I</sub> > 0)',
          html: 'Same as above, with PID control.',
          code: lib.pyPart(ctx, {
            args: lib.gainArgs,
            items: [
              { var: 'system_type', truth: () => 1 },
              { fn: 'e_step', args: ['kP', 'kI', 'kD'], truth: () => 0 },
              { fn: 'e_ramp', args: ['kP', 'kI', 'kD'], truth: (p, a) => p.k / a.kI },
              { fn: 'e_parab', args: ['kP', 'kI', 'kD'], truth: () => Infinity },
            ],
          }, 'system_type = ...\n\ndef e_step(kP, kI, kD):\n    return ...\n\ndef e_ramp(kP, kI, kD):\n    return ...\n\ndef e_parab(kP, kI, kD):\n    return ...\n'),
          solution: () => [
            { tex: `PC = \\frac{(k_D s^2 + k_P s + k_I)/m}{s(s^2 + \\frac bm s + \\frac km)} \\Rightarrow \\text{type 1},\; e_{step} = 0,\; e_{ramp} = \\frac{k}{k_I},\; e_{parab} = \\infty` },
            { code: 'system_type = 1\n\ndef e_step(kP, kI, kD):\n    return 0.0\n\ndef e_ramp(kP, kI, kD):\n    return P.k / kI    # 1/M_v, M_v = kI/k\n\ndef e_parab(kP, kI, kD):\n    return np.inf' },
          ],
        },
        {
          id: 'b', title: '(b) Constant input disturbance',
          html: 'Steady-state error per newton of d (m/N), without and with the integrator, as functions of the gains. Either sign convention is accepted.',
          code: {
            template: 'def e_dist_pd(kP, kD):\n    return ...\n\ndef e_dist_pid(kP, kI, kD):\n    return ...\n',
            // sign depends on the convention (Fig. 9-5 subtracts d_in; the sim adds d)
            ...lib.pyEither(ctx, [1, -1].map((sg) => ({
              args: lib.gainArgs,
              items: [
                { fn: 'e_dist_pd', args: ['kP', 'kD'], truth: (p, a) => sg / (p.k + a.kP) },
                { fn: 'e_dist_pid', args: ['kP', 'kI', 'kD'], truth: () => 0 },
              ],
            }))),
          },
          solution: () => [
            { tex: `\\text{PD}: \\lim_{s\\to0}\\frac{P}{1+PC} = \\frac{1/k}{1 + k_P/k} = \\frac{1}{k + k_P} = ${tex(1 / (ctx.pModel.k + g().kP))}\;\\text{(current gains)},\\quad \\text{PID}: 0` },
            { code: 'def e_dist_pd(kP, kD):\n    return 1 / (P.k + kP)\n\ndef e_dist_pid(kP, kI, kD):\n    return 0.0' },
            { html: 'An error in k is such a disturbance: F<sub>e</sub> = k̂z<sub>r</sub> misses k z<sub>r</sub> by (k − k̂)z<sub>r</sub>. Try it: d = 0.5 N starts at t = 20 s.' },
          ],
        },
      ]);
    },
  };

  // ------------------------------------------------------------- D.P.6 --
  CH.p6 = {
    id: 'p6', num: 10.5, tab: 'App. P.6', short: 'P.6', title: 'Root locus vs. k_I', pages: 'pp. 465–474, p. 380',
    defaults(sys) { const pr = sys.problems.p6; return { comp: 'none', deriv: 'state', antiwindup: 'none', kP: 1, kD: 1, tr: pr.tr, zeta: pr.zeta, kIx: 0.1, kMaxFactor: 1, kIMax: 20 }; },
    simDefaults(sys) { return sys.problems.p6.sim; },
    // Work mode: your own PD gains (placeholders until you enter your D.8 gains); Explore: from t_r, ζ.
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kD: ctx.st.kD, kI: ctx.st.kIx } : designed(ctx); },
    controller: (ctx, o) => makePID(ctx, o),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePID),

    buildControls(parent, ctx) {
      const sec = section(parent, 'PD from D.8, then add k_I', 'p. 466');
      compControl(sec, ctx);
      if (ctx.S.mode === 'work') { gainSliders(sec, ctx, ['kP', 'kD']); lib.gainSlider(sec, ctx, 'kIx', 'k<sub>I</sub>', R.kI); } else specSliders(sec, ctx, { kI: true });
      // k_I,crit answers (b), so Work mode sets the locus range in absolute k_I.
      if (ctx.S.mode === 'work') slider(sec, { label: 'locus k<sub>I,max</sub>', min: 0.5, max: 100, log: true, sig: 3, ...bind(ctx, 'kIMax') });
      else slider(sec, { label: 'locus to', unit: '× kI,crit', min: 0.1, max: 3, step: 0.05, sig: 2, ...bind(ctx, 'kMaxFactor') });
      lib.note(sec, ctx.S.mode === 'work' ? 'Enter your D.8 gains (problem panel: Use my gains), then set k_I with the slider or by dragging a closed-loop pole. The root locus appears once you solve (a).' : 'Drag a closed-loop pole along the locus to set k_I.');
      if (ctx.S.mode === 'explore') readout(sec, ctx);
    },

    // Locus range: Explore scales k_I,crit; Work uses an absolute k_I (k_I,crit answers (b)).
    kMax(ctx, ev) { return Math.max(ctx.S.mode === 'work' ? ctx.st.kIMax ?? 20 : ev.kCrit * ctx.st.kMaxFactor, 1e-3); },
    // The poles of L(s) and the locus show the Evans form, which answers (a).
    splane(ctx) {
      const g = ctx.gains, ev = ans.evans(ctx.pModel, g);
      const locus = lib.shows(ctx, 'D.P.6/a');
      const kMax = Math.max(this.kMax(ctx, ev), g.kI * 1.2);
      const mk = locus ? L.roots(ev.den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of L(s) ${i + 1}` })) : [];
      L.roots(pidCharPoly(ctx.model, g)).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole at kI = ${fmt(g.kI, 3)}`, dragId: i }));
      const fitR = Math.max(...L.roots(ev.den).map((p) => Math.hypot(p.re, p.im))) * 1.6;
      return { markers: mk, loci: locus ? T.rootLocus(ev.den, ev.num, kMax) : undefined, fitR };
    },
    onPoleDrag(ctx, id, re, im) {
      const ev = ans.evans(ctx.pModel, ctx.gains);
      const kMax = this.kMax(ctx, ev);
      let best = ctx.st.kIx, bd = Infinity;
      for (let i = 0; i <= 400; i++) {
        const k = kMax * i / 400;
        for (const q of L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, k)))) {
          const d = Math.hypot(q.re - re, q.im - im);
          if (d < bd) { bd = d; best = k; }
        }
      }
      ctx.st.kIx = best;
      ctx.update();
    },

    math(ctx) {
      const g = ctx.gains, ev = ans.evans(ctx.pModel, g);
      return [
        { title: 'Evans form', page: 'p. 466',
          theory: '\\Delta_{cl}(s) = 0 \\iff 1 + k\\,L(s) = 0 \\quad(k \\text{ is the gain that varies along the locus, here } k_I)' },
        { title: 'Closed loop with PID (derivative on output)', page: 'p. 470 (A.P.6 pattern)', answers: 'D.P.6/a',
          theory: '\\Delta_{cl}(s) = s^3 + \\frac{b + k_D}{m}s^2 + \\frac{k + k_P}{m}s + \\frac{k_I}{m}',
          numbers: `\\Delta_{cl}(s) = ${WB.tf.polyTex(pidCharPoly(ctx.model, g))}` },
        { title: 'Evans form of the mass-spring-damper with PID', page: 'p. 466', answers: 'D.P.6/a',
          theory: '1 + k_I\\,L(s) = 0,\\quad L(s) = \\frac{1/m}{s^3 + \\frac{b + k_D}{m}s^2 + \\frac{k + k_P}{m}s}',
          numbers: `L(s) = \\frac{${tex(ctx.model.b0)}}{${WB.tf.polyTex(ev.den)}},\\quad k_P = ${tex(g.kP)},\\; k_D = ${tex(g.kD)}` },
        { title: 'Where the locus crosses into the RHP', page: 'Routh–Hurwitz (not in the book)',
          theory: 's^3 + c_2 s^2 + c_1 s + c_0 \\text{ is stable iff } c_2, c_1, c_0 > 0 \\text{ and } c_2 c_1 > c_0',
          numbers: `k_{I,crit} = \\frac{c_2 c_1}{b_0} = ${tex(ev.kCrit)}`, answers: 'D.P.6/b' },
        compCard(ctx),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.p6;
      const ev = () => ans.evans(ctx.pModel, ctx.gains);
      lib.panel(parent, ctx, prob, [
        {
          id: 'g', title: 'Start from your D.8 PD gains',
          inputs: { kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>' },
          actions: [{ label: 'Use my gains', run: (v) => {
            const kP = lib.num(v.kP), kD = lib.num(v.kD);
            if (kP === null || kD === null) return { ok: false, msg: 'Enter kP and kD first.' };
            ctx.app.setMode('work'); Object.assign(ctx.st, { kP, kD }); ctx.update(); return null;
          } }],
        },
        {
          id: 'a', title: 'Characteristic equation in Evans form (in k<sub>I</sub>)',
          html: 'Write L(s) for 1 + k<sub>I</sub>L(s) = 0 in terms of the PD gains k<sub>P</sub>, k<sub>D</sub> (derivative on the output); the check calls it at complex s.',
          code: lib.pyPart(ctx, {
            args: { s: lib.sArg, kP: { label: 'kP', lo: 0.5, hi: 20 }, kD: { label: 'kD', lo: 0.5, hi: 30 } },
            items: [{ fn: 'L', args: ['s', 'kP', 'kD'], truth: evansL }],
          }, 'def L(s, kP, kD):\n    return ...\n'),
          solution: () => [
            { tex: '\\Delta_{cl}(s) = s^3 + \\frac{b + k_D}{m}s^2 + \\frac{k + k_P}{m}s + \\frac{k_I}{m} = 0 \\;\\Rightarrow\\; L(s) = \\frac{1/m}{s^3 + \\frac{b + k_D}{m}s^2 + \\frac{k + k_P}{m}s}' },
            { tex: `\\text{current PD gains: } 1 + k_I\\frac{${tex(ctx.model.b0)}}{${WB.tf.polyTex(ev().den)}} = 0` },
            { code: 'def L(s, kP, kD):\n    return (1 / P.m) / (s**3\n        + (P.b + kD) / P.m * s**2\n        + (P.k + kP) / P.m * s)' },
            { html: '(b + k<sub>D</sub>)/m and (k + k<sub>P</sub>)/m are the coefficients of the PD design polynomial: α<sub>1</sub> = 2ζω<sub>n</sub>, α<sub>0</sub> = ω<sub>n</sub>².' },
          ],
        },
        {
          id: 'b', title: 'Largest stable k<sub>I</sub>',
          html: 'As a function of the PD gains k<sub>P</sub>, k<sub>D</sub>. The check calls it at random gains.',
          code: lib.pyPart(ctx, {
            args: { kP: lib.gainArgs.kP, kD: lib.gainArgs.kD },
            items: [{ fn: 'kI_crit', args: ['kP', 'kD'], truth: (p, a) => ans.evans(p, a).kCrit }],
          }, 'def kI_crit(kP, kD):\n    return ...\n'),
          solution: () => [
            { tex: `c_2c_1 > b_0k_I \\Rightarrow k_{I,crit} = \\frac{(a_1 + b_0k_D)(a_0 + b_0k_P)}{b_0} = \\frac{(b + k_D)(k + k_P)}{m} = ${tex(ev().kCrit)}\;\\text{(current gains)}` },
            { code: 'def kI_crit(kP, kD):\n    return (P.b + kD) * (P.k + kP) / P.m    # Routh: c2 c1 > c0 = kI/m' },
          ],
        },
        {
          id: 'c', title: 'Pick k<sub>I</sub> that barely moves the PD poles',
          html: 'Checks the current k<sub>I</sub>: the complex pair must stay within 10% (in |p|) of the PD-only poles, and the new real pole must be slower than them.',
          check: () => {
            const g = ctx.gains;
            if (!(g.kI > 0)) return { ok: false, msg: 'Set kI > 0.' };
            const pd = M.roots2(ctx.model.a1 + ctx.model.b0 * g.kD, ctx.model.a0 + ctx.model.b0 * g.kP);
            const cl = L.roots(pidCharPoly(ctx.model, g));
            const cpx = cl.filter((p) => Math.abs(p.im) > 1e-6), real = cl.filter((p) => Math.abs(p.im) <= 1e-6);
            if (cpx.length !== 2) return { ok: false, msg: 'The PD pair has split into real poles; kI is too large.' };
            const ratio = Math.hypot(cpx[0].re, cpx[0].im) / Math.hypot(pd[0].re, pd[0].im);
            const slow = real.length === 1 && Math.abs(real[0].re) < Math.abs(pd[0].re);
            return { ok: Math.abs(ratio - 1) < 0.1 && slow, msg: `|p| ratio ${fmt(ratio, 3)}, real pole ${real.length ? fmtPole(real[0]) : '—'}.` };
          },
          solution: () => [{ html: 'With the D.8(a) gains (k<sub>P</sub> = 3.05, k<sub>D</sub> = 7.2), any k<sub>I</sub> up to about 0.8 keeps |p| within 10%: k<sub>I</sub> = 0.5 gives −0.724 ± 0.743j and −0.093, and k<sub>I</sub> = 0.75 gives −0.695 ± 0.720j and −0.150. The price is a slow integrator pole, partly cancelled by the zero at −k<sub>I</sub>/k<sub>P</sub>. Values in 0.5–0.8 also pass the D.10 tracking check (the 10% |p| criterion here is a workbench choice).' }],
        },
      ]);
    },
  };

  // -------------------------------------------------------------- D.10 --
  CH.ch10 = {
    id: 'ch10', num: 10, tab: 'Ch 10', title: 'Digital PID', pages: 'pp. 155–169, p. 380',
    defaults(sys) {
      const pr = sys.problems.ch10;
      return { comp: 'eq', deriv: 'dirty', antiwindup: 'gate', vbar: 0.05, sigma: pr.sigma, kP: 1, kD: 1, kI: 0, tr: pr.tr, zeta: pr.zeta, kIx: pr.kiRef, extra: 'deriv' };
    },
    simDefaults(sys) { return { ...sys.problems.ch10.sim, mismatch: sys.problems.ch10.mismatch }; },
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD } : designed(ctx); },
    controller: (ctx, o) => makePID(ctx, o),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePID),
    targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.tr } : {}; },
    // Work mode simulates the student's D.10(c) controller, from the measured z only.
    implement: { feed: 'y', linear: false },

    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'D.10(c)', { keys: ['kP', 'kI', 'kD'] }); return; }
      const sec = section(parent, 'Digital PID', 'p. 155, p. 162 · Listing 10.2');
      compControl(sec, ctx);
      specSliders(sec, ctx, { kI: true }); readout(sec, ctx);
      const imp = section(parent, 'Implementation', 'p. 157 · Eq. 10.3–10.4');
      segmented(imp, {
        label: 'ż for the D term',
        options: [{ value: 'dirty', label: 'dirty derivative of z (D.10b)' }, { value: 'state', label: 'true ż (cheating)' }],
        ...bind(ctx, 'deriv'),
      });
      slider(imp, { label: 'σ', unit: 's', min: 0.002, max: 0.5, step: 0.001, sig: 3, hint: 'dirty-derivative bandwidth is 1/σ rad/s', ...bind(ctx, 'sigma'), disabled: () => ctx.st.deriv !== 'dirty' });
      segmented(imp, {
        label: 'Anti-windup',
        options: [
          { value: 'gate', label: 'integrate when |ż| < v̄', title: 'Listing 10.2 pattern' },
          { value: 'backcalc', label: 'back-calculation', title: 'u_I += (u_sat − u_unsat)/k_I, p. 158' },
          { value: 'none', label: 'none' },
        ],
        ...bind(ctx, 'antiwindup'),
      });
      slider(imp, { label: 'v̄', unit: 'm/s', min: 0.005, max: 1, step: 0.005, sig: 3, ...bind(ctx, 'vbar'), disabled: () => ctx.st.antiwindup !== 'gate' });
      segmented(imp, {
        label: 'Extra plot', options: [{ value: 'deriv', label: 'ż estimate' }, { value: 'int', label: 'integrator' }],
        ...bind(ctx, 'extra'),
      });
    },

    splane(ctx) { return { markers: markers(ctx) }; },

    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      if (ctx.st.extra === 'int') {
        return { opts: { title: 'integrator ∫e dt', yLabel: '∫e dt [m·s]', unit: 'm·s' }, data: { series: [{ label: '∫e dt', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
      }
      return {
        opts: { title: 'ż(t)', yLabel: 'ż [m/s]', unit: 'm/s' },
        data: {
          series: [
            { label: 'controller estimate', y: Array.from(res.extras.zdotHat || []), color: '--series-2', width: 1.5 },
            { label: 'true ż', y: res.x.map((x) => x[1]), color: '--series-1' },
          ],
          hlines: ctx.st.antiwindup === 'gate' ? [{ y: ctx.st.vbar, label: '+v̄', color: '--text-muted' }, { y: -ctx.st.vbar, label: '−v̄', color: '--text-muted', fit: false }] : [],
        },
      };
    },

    math(ctx) {
      const st = ctx.st, Ts = ctx.S.sim.Ts;
      const { beta, gamma } = WB.design.dirtyCoeffs(st.sigma, Ts);
      const d = designed(ctx);
      return [
        { title: 'PID from the measured output', page: 'p. 155',
          theory: 'F(t) = F_e + k_P e(t) + k_I \\int_{-\\infty}^{t} e(\\tau)\\,d\\tau - k_D \\dot z(t)' },
        { title: 'Trapezoidal integrator', page: 'p. 157 · Eq. 10.2–10.3',
          theory: 'u_I[n] = u_I[n-1] + \\frac{T_s}{2}\\big(e[n] + e[n-1]\\big)' },
        { title: 'Dirty derivative', page: 'p. 157 · Eq. 10.4',
          theory: '\\dot{\\hat z}[n] = \\frac{2\\sigma - T_s}{2\\sigma + T_s}\\dot{\\hat z}[n-1] + \\frac{2}{2\\sigma + T_s}\\big(z[n] - z[n-1]\\big)',
          numbers: `\\sigma = ${tex(st.sigma)},\\; T_s = ${tex(Ts)}:\\quad ${tex(beta)},\\quad ${tex(gamma)}`, answers: 'D.10/c' },
        { title: 'Anti-windup', page: 'p. 157 · §10.1.1',
          theory: '\\text{(1) integrate only when } |\\dot z| < \\bar v,\\quad \\text{(2) } u_I^+ = u_I + \\frac{1}{k_I}\\big(u_{sat} - u_{unsat}\\big)' },
        { title: 'Gain-selection guidance', page: 'p. 160 · §10.1.3',
          theory: '\\text{pick } k_P, k_D \\text{ (Ch. 8)},\\; \\text{then raise } k_I \\text{ from 0 until the steady-state error is gone}' },
        { title: 'Gains from t_r, ζ (D.8)', page: 'p. 113 · Eq. 8.5, p. 160 · §10.1.3', answers: ['D.8/a', 'D.10/c'],
          theory: '\\omega_n = \\frac{2.2}{t_r},\\quad k_P = m\\omega_n^2 - k,\\quad k_D = 2m\\zeta\\omega_n - b',
          numbers: `\\omega_n = ${tex(d.wn)},\\quad k_P = ${tex(d.kP)},\\quad k_D = ${tex(d.kD)}${explore(ctx) ? `,\\quad k_I = ${tex(d.kI)}` : ''}` },
        ...(explore(ctx) ? [compCard(ctx)] : []),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch10;
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Parameters vary by up to 20%',
          html: 'Set the true plant in the left panel (<em>Randomize ±α</em> with α = 0.2). The chapter starts with a fixed 20% draw so the page is repeatable. In your code, <code>massDynamics(alpha=0.2)</code> perturbs m, k and b.',
          check: () => {
            const mis = Object.entries(ctx.S.mismatch || {});
            return mis.some(([, v]) => Math.abs(v) > 0) ? { ok: true, msg: `Mismatch: ${mis.map(([k, v]) => `${k} ${fmt(v, 3)}%`).join(', ')}.` } : { ok: false, msg: 'The true plant equals the model. Randomize it.' };
          },
        },
        {
          id: 'b', title: '(b) The controller gets only z and z<sub>r</sub>',
          html: 'From here on your controller\'s <code>update(z_r, y)</code> receives the noisy measurement y = [[z]], not the state.',
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Implement the D.8 PID with σ = 0.05; tune the integrator', seed: ['D.8/b', 'D.8/a2', 'D.7/d'],
          html: `Start from your D.8 controller. The check runs the ±${prob.sim.amplitude} m square wave (${prob.sim.frequency} Hz) on a plant that differs from the model by ${mis(prob.mismatch)}: |z<sub>r</sub> − z| just before the first switch (t = 25 s) must be under ${prob.tolPct}% of the step (${fmt(1000 * prob.tolPct / 100 * prob.sim.amplitude, 3)} mm). The ${prob.tolPct}% threshold is a workbench choice; the book gives none.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: { type: 'square', amplitude: prob.sim.amplitude, frequency: prob.sim.frequency, tStep: 0 }, tEnd: 25, mismatch: prob.mismatch });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const k = Math.round(24.95 / sc.Ts), lim = prob.tolPct / 100 * prob.sim.amplitude;
            const e = Math.abs(res.r[k] - res.y[k]);
            return { ok: e < lim, msg: `Error before the switch: ${fmt(1000 * e, 3)} mm (limit ${fmt(1000 * lim, 3)} mm).` };
          },
          solution: () => [
            { code: D10_SOL },
            { html: `Listing 10.2 (p. 162) on the D.8(a) design: dirty derivative of the measured z (σ = 0.05), trapezoidal integrator that integrates only while |ż| is small, saturation at F<sub>max</sub>, plus the spring's equilibrium force F<sub>e</sub> = kz<sub>r</sub> (D.4). With k<sub>I</sub> = ${prob.kiRef} the error before the switch is about 1.5 mm. Any k<sub>I</sub> from about 0.5 to 0.8, the D.P.6 range, passes. Without F<sub>e</sub> the integrator has to supply the spring force itself: k<sub>I</sub> = 0.75 then leaves about 6 mm, which still passes.` },
          ],
        }),
      ]);
    },
  };

  WB.studies.D.pid = { makePD, makePID, pidCharPoly, compControl, gainSliders, designed };
})();
