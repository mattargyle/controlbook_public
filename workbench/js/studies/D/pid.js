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
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
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
        const ff = st.comp === 'eq' ? p.k * r : 0;
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
    const beta = (2 * sigma - Ts) / (2 * sigma + Ts), gamma = 2 / (2 * sigma + Ts);
    const lim = p.Fmax;
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
        let u = (st.comp === 'eq' ? p.k * r : 0) + kP * e + kI * I - kD * ydot;
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
  function clPoles(t, g) {
    return g.kI ? L.roots(pidCharPoly(t, g)) : M.roots2(t.a1 + t.b0 * g.kD, t.a0 + t.b0 * g.kP);
  }

  // ------------------------------------------------------------- controls --
  function compControl(parent, ctx) {
    segmented(parent, {
      label: 'Spring compensation',
      options: [
        { value: 'eq', label: 'F = k z<sub>r</sub> + F̃', title: 'equilibrium force F_e = k z_e with z_e = z_r' },
        { value: 'none', label: 'none (Fig. 7-2)', title: 'F = F̃: pure PD/PID' },
      ],
      get: () => ctx.st.comp, set: (v) => { ctx.st.comp = v; ctx.update(); },
    });
  }
  function archControl(parent, ctx) {
    segmented(parent, {
      label: 'Derivative acts on',
      options: [{ value: 'output', label: 'output z (Fig. 7-2)' }, { value: 'error', label: 'error e (Fig. 7-1)' }],
      get: () => ctx.st.arch, set: (v) => { ctx.st.arch = v; ctx.update(); },
    });
  }
  const R = { kP: 20, kD: 30, kI: 10 };
  function gainSliders(parent, ctx, keys) {
    const lab = { kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>', kI: 'k<sub>I</sub>' };
    for (const k of keys) lib.gainSlider(parent, ctx, k, lab[k], R[k]);
  }
  function specSliders(parent, ctx, { kI } = {}) {
    slider(parent, { label: 't<sub>r</sub>', unit: 's', min: 0.2, max: 6, step: 0.005, sig: 4, get: () => ctx.st.tr, set: (v) => { ctx.st.tr = v; ctx.update(); } });
    slider(parent, { label: 'ζ', min: 0.1, max: 2, step: 0.005, get: () => ctx.st.zeta, set: (v) => { ctx.st.zeta = v; ctx.update(); } });
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

  function markers(ctx, { draggable = false, ol = true } = {}) {
    const t = ctx.model, g = ctx.gains;
    const mk = ol ? M.roots2(t.a1, t.a0).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole p${i + 1}` })) : [];
    clPoles(t, g).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole ${i + 1}`, dragId: draggable && Math.abs(p.im) > 1e-9 ? 0 : (draggable && !g.kI ? i : undefined) }));
    if (g.kI && g.kP) mk.push({ re: -g.kI / g.kP, im: 0, kind: 'zero', label: 'closed-loop zero −kI/kP' });
    if (!g.kI && ctx.st.arch === 'error' && g.kD > 1e-9) mk.push({ re: -g.kP / g.kD, im: 0, kind: 'zero', label: 'zero −kP/kD' });
    return mk;
  }

  // ------------------------------------------------------------ math cards --
  function plantCard(ctx) {
    const t = ctx.model;
    return {
      title: 'Plant (D.5)', page: 'p. 378 · D.5, p. 99 · Eq. 7.1',
      theory: 'P(s) = \\frac{b_0}{s^2 + a_1 s + a_0}',
      symbolic: 'b_0 = \\tfrac1m,\\quad a_1 = \\tfrac bm,\\quad a_0 = \\tfrac km',
      numbers: `P(s) = \\frac{${tex(t.b0)}}{s^2 + ${tex(t.a1)}\\,s + ${tex(t.a0)}}`, spoiler: true,
    };
  }
  function pdLoopCard(ctx) {
    const t = ctx.model, g = ctx.gains, err = ctx.st.arch === 'error';
    const cl = M.roots2(t.a1 + t.b0 * g.kD, t.a0 + t.b0 * g.kP);
    return {
      title: `Closed loop, derivative on ${err ? 'error (Fig. 7-1)' : 'output (Fig. 7-2)'}`, page: err ? 'p. 100 · Eq. 7.4' : 'p. 101 · Eq. 7.5',
      theory: err ? '\\frac{Y}{R} = \\frac{b_0 k_D s + b_0 k_P}{s^2 + (a_1 + b_0 k_D)s + (a_0 + b_0 k_P)}' : '\\frac{Y}{R} = \\frac{b_0 k_P}{s^2 + (a_1 + b_0 k_D)s + (a_0 + b_0 k_P)}',
      symbolic: `\\Delta_{cl}(s) = s^2 + \\frac{b + k_D}{m}s + \\frac{k + k_P}{m}`,
      numbers: `\\Delta_{cl} = s^2 + ${tex(t.a1 + t.b0 * g.kD)}\\,s + ${tex(t.a0 + t.b0 * g.kP)},\\quad p_{cl} = ${texPole(cl[0])},\\; ${texPole(cl[1])}`,
      spoiler: true,
    };
  }
  function compCard(ctx) {
    return {
      title: 'Spring compensation', page: 'p. 59–60 (equilibrium input)',
      theory: ctx.st.comp === 'eq' ? 'F = F_e + \\tilde F,\\quad F_e = k z_e,\\; z_e = z_r' : 'F = \\tilde F \\quad(\\text{Fig. 7-2 exactly})',
      note: ctx.st.comp === 'eq' ? 'Feedforward outside the loop: the poles do not move, but the DC gain from z_r to z becomes 1 when k is known exactly.' : 'Without F_e the spring holds the mass short of z_r: D.9 asks how far.',
    };
  }

  // -------------------------------------------------------------- D.7 --
  CH.ch7 = {
    id: 'ch7', num: 7, tab: 'D.7', title: 'Pole placement (PD)', pages: 'pp. 99–106, p. 379',
    controller: (ctx) => makePD(ctx),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePD),
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
      const sec = section(parent, 'PD controller', 'p. 99–101');
      archControl(sec, ctx);
      compControl(sec, ctx);
      if (ctx.S.mode === 'work') {
        gainSliders(sec, ctx, ['kP', 'kD']);
        lib.note(sec, 'The target poles from (c) are dashed rings in the s-plane. The open-loop poles stay hidden until you check (a).');
        return;
      }
      const des = section(parent, 'Desired closed-loop poles', 'p. 100');
      segmented(des, {
        label: 'Pole pair', options: [{ value: 'real', label: 'two real' }, { value: 'complex', label: 'complex pair' }],
        get: () => ctx.st.form, set: (v) => { ctx.st.form = v; ctx.update(); },
      });
      const dis = (f) => () => ctx.st.form !== f;
      slider(des, { label: 'p<sub>1</sub>', min: -6, max: 0, step: 0.01, get: () => ctx.st.p1, set: (v) => { ctx.st.p1 = v; ctx.update(); }, disabled: dis('real') });
      slider(des, { label: 'p<sub>2</sub>', min: -6, max: 0, step: 0.01, get: () => ctx.st.p2, set: (v) => { ctx.st.p2 = v; ctx.update(); }, disabled: dis('real') });
      slider(des, { label: '−σ', min: -6, max: 0, step: 0.01, get: () => ctx.st.sigma, set: (v) => { ctx.st.sigma = v; ctx.update(); }, disabled: dis('complex') });
      slider(des, { label: 'ω<sub>d</sub>', unit: 'rad/s', min: 0, max: 6, step: 0.01, get: () => ctx.st.wd, set: (v) => { ctx.st.wd = v; ctx.update(); }, disabled: dis('complex') });
      lib.note(des, 'Or drag a closed-loop pole. Drag off the real axis for a complex pair.');
      readout(des, ctx, ['kP', 'kD']);
    },

    splane(ctx) {
      const work = ctx.S.mode === 'work';
      const mk = markers(ctx, { draggable: !work, ol: !work || ctx.app.isRevealed('D:ch7:ol') });
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
        plantCard(ctx),
        { title: 'Open-loop poles', page: 'p. 99',
          theory: '\\Delta_{ol}(s) = s^2 + a_1 s + a_0 = 0', numbers: `p_{ol} = ${texPole(ol[0], 4)},\\; ${texPole(ol[1], 4)}`, spoiler: true },
        pdLoopCard(ctx),
        { title: ctx.S.mode === 'work' ? 'Pole placement (problem targets)' : 'Pole placement (your design)', page: 'p. 100',
          theory: '\\Delta^d_{cl} = (s - p_1)(s - p_2) = s^2 + \\alpha_1 s + \\alpha_0,\\quad k_P = \\frac{\\alpha_0 - a_0}{b_0},\\quad k_D = \\frac{\\alpha_1 - a_1}{b_0}',
          numbers: `\\Delta^d_{cl} = s^2 + ${tex(g.alpha1)}\\,s + ${tex(g.alpha0)} \\Rightarrow k_P = ${tex(g.kP)},\\; k_D = ${tex(g.kD)}`, spoiler: true },
        compCard(ctx),
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
            if (ok) { ctx.app.reveal('D:ch7:ol'); ctx.update(); }
            return ok ? { ok, msg: 'Lightly damped: ζ ≈ 0.065.' } : { ok, msg: 'Roots of s² + (b/m)s + k/m?' };
          },
          solution: () => {
            const ol = ans.olPoles(ctx.pModel);
            return [{ tex: `\\Delta_{ol} = s^2 + ${tex(t().a1)}s + ${tex(t().a0)} \\Rightarrow p = ${texPole(ol[0], 4)},\\; ${texPole(ol[1], 4)}` }];
          },
        },
        {
          id: 'b', title: '(b) Closed loop: Δ<sub>cl</sub> = s² + (c<sub>1</sub> + d<sub>1</sub>k<sub>D</sub>)s + (c<sub>0</sub> + d<sub>0</sub>k<sub>P</sub>)',
          inputs: { c1: 'c<sub>1</sub>', d1: 'd<sub>1</sub>', c0: 'c<sub>0</sub>', d0: 'd<sub>0</sub>' },
          check: (v) => lib.check(v, { c1: t().a1, d1: t().b0, c0: t().a0, d0: t().b0 }, {}),
          solution: () => [
            { tex: '\\frac{Z(s)}{Z_r(s)} = \\frac{k_P/m}{s^2 + \\frac{b + k_D}{m}s + \\frac{k + k_P}{m}}' },
            { tex: `p_{1,2} = -\\frac{b + k_D}{2m} \\pm \\sqrt{\\left(\\frac{b + k_D}{2m}\\right)^2 - \\frac{k + k_P}{m}},\\quad (c_1, d_1, c_0, d_0) = (${tex(t().a1)}, ${tex(t().b0)}, ${tex(t().a0)}, ${tex(t().b0)})` },
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
        {
          id: 'd', title: '(d) Simulate a 1 m step',
          html: 'Click <em>Use my gains</em> in (c). The pure Fig. 7-2 loop settles at k<sub>P</sub>/(k + k<sub>P</sub>) of the step (D.9 explains why). Switch the spring compensation to F = kz<sub>r</sub> + F̃ to reach 1 m; with these gains that asks for 7.5 N at the step, above the 6 N limit that D.8(b) introduces.',
          check: () => {
            if (ctx.S.mode !== 'work') return { ok: false, msg: 'Switch to Work mode so the simulation uses your gains.' };
            const cl = M.roots2(t().a1 + t().b0 * ctx.st.kD, t().a0 + t().b0 * ctx.st.kP);
            return M.polesMatch(cl, prob.desiredPoles, 0.02, 0.02)
              ? { ok: true, msg: 'The simulated loop has the target poles.' }
              : { ok: false, msg: `Current closed-loop poles: ${lib.polesOf(cl)}.` };
          },
          solution: () => [{ html: 'Two real poles: no overshoot, and the slower pole at −1 gives a settling time of roughly 4 s.' }],
        },
      ]);
    },
  };

  // -------------------------------------------------------------- D.8 --
  // Peak |F| demanded for a step of `step` meters from rest, with gains from (tr, zeta).
  function peakForTr(ctx, tr, zeta, step, comp) {
    const p = ctx.pModel, sys = ctx.sys;
    const g = ans.spec(p, tr, zeta);
    const fake = { ...ctx, gains: g, st: { ...ctx.st, arch: 'output', comp } };
    const out = WB.sim.simulate({
      plant: { f: (x, u) => sys.f(x, u, p), h: sys.h, uLimit: p.Fmax },
      controller: makePD(fake),
      reference: WB.sim.makeReference({ type: 'step', amplitude: step, tStep: 0 }),
      disturbance: () => 0, x0: [0, 0], Ts: ctx.S.sim.Ts, tEnd: 15,
    });
    let peak = 0;
    for (const u of out.uDemand) peak = Math.max(peak, Math.abs(u));
    return peak / p.Fmax;
  }
  // Largest kP so the demand right after the step fits: (Fmax - F_e) / step (Eq. 8.8)
  function satBound(ctx, step, zeta, comp) {
    const p = ctx.pModel, t = ans.tf(p);
    const Fe = comp === 'eq' ? p.k * step : 0;
    const kP = (p.Fmax - Fe) / step;
    const wn = Math.sqrt(t.a0 + t.b0 * kP);
    return { Fe, kP, wn, tr: 2.2 / wn, kD: (2 * zeta * wn - t.a1) / t.b0 };
  }

  CH.ch8 = {
    id: 'ch8', num: 8, tab: 'D.8', title: 'Second-order design', pages: 'pp. 107–136, p. 379',
    controller: (ctx) => makePD(ctx),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePD),
    targets: (ctx) => ({ tr: ctx.st.tr, zeta: ctx.st.zeta }),
    defaults(sys) { const pr = sys.problems.ch8; return { arch: 'output', comp: 'none', kP: 1, kD: 1, tr: pr.tr, zeta: pr.zeta }; },
    simDefaults(sys) { return sys.problems.ch8.sim; },
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kD: ctx.st.kD, kI: 0 } : { ...designed(ctx), kI: 0 }; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'PD controller', 'p. 111 · Fig. 8-12');
      archControl(sec, ctx);
      compControl(sec, ctx);
      if (ctx.S.mode === 'work') gainSliders(sec, ctx, ['kP', 'kD']);
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
        const sb = satBound(ctx, Math.abs(ctx.S.sim.amplitude - ctx.S.sim.y0) || 1, ctx.st.zeta, ctx.st.comp);
        if (isFinite(sb.wn)) out.wnMax = sb.wn;
      }
      return out;
    },
    onPoleDrag: dragSpec,

    math(ctx) {
      const st = ctx.st;
      const d = ans.spec(ctx.pModel, st.tr, st.zeta);
      const step = Math.abs(ctx.S.sim.amplitude - ctx.S.sim.y0) || 1;
      const sb = satBound(ctx, step, st.zeta, st.comp);
      return [
        plantCard(ctx), pdLoopCard(ctx),
        { title: 'Spec → desired characteristic polynomial', page: 'p. 110 · Eq. 8.2, p. 113 · Eq. 8.5',
          theory: '\\omega_n = \\frac{2.2}{t_r},\\quad \\Delta^d_{cl} = s^2 + 2\\zeta\\omega_n s + \\omega_n^2,\\quad p = -\\zeta\\omega_n \\pm j\\omega_n\\sqrt{1-\\zeta^2}',
          numbers: `\\omega_n = ${tex(d.wn)},\\quad \\Delta^d_{cl} = s^2 + ${tex(d.alpha1)}\\,s + ${tex(d.alpha0)},\\quad p = ${texPole(d.poles[0], 4)},\\; ${texPole(d.poles[1], 4)}`, spoiler: true },
        { title: 'Gains from the spec', page: 'p. 100',
          theory: 'k_P = \\frac{\\omega_n^2 - a_0}{b_0},\\quad k_D = \\frac{2\\zeta\\omega_n - a_1}{b_0}',
          numbers: `k_P = ${tex(d.kP)},\\quad k_D = ${tex(d.kD)}`, spoiler: true },
        { title: 'Saturation limits the rise time', page: 'p. 119 · Eq. 8.8, p. 120 · Fig. 8-13',
          theory: 'u = F_e + k_P e - k_D\\dot z \\text{ peaks at } t = 0^+ \\;(\\dot z = 0):\\quad k_P \\le \\frac{F_{max} - |F_e|}{e_{max}},\\quad \\omega_n \\le \\sqrt{a_0 + b_0 k_{P,max}},\\quad t_r \\ge \\frac{2.2}{\\omega_{n,max}}',
          numbers: `F_e = ${tex(sb.Fe)},\\; e_{max} = ${tex(step)}\\,\\text{m}\\Rightarrow k_{P,max} = ${tex(sb.kP)},\\; \\omega_{n,max} = ${tex(sb.wn)},\\; t_{r,min} = ${tex(sb.tr)}\\,\\text{s}`, spoiler: true,
          note: 'F_e depends on the spring-compensation setting (k z_r or none), which changes the D.8(b) answer.' },
        compCard(ctx),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch8;
      const ref = () => ans.spec(ctx.pModel, prob.tr, prob.zeta);
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) t<sub>r</sub> = ${prob.tr} s, ζ = ${prob.zeta}`,
          html: 'Δ<sup>d</sup><sub>cl</sub>(s) = s² + α<sub>1</sub>s + α<sub>0</sub>; poles −σ ± jω<sub>d</sub>.',
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
        {
          id: 'a2', title: '(a) Verify the step response',
          html: 'Checks the current Work-mode simulation: 10–90% rise time (of the final value, so it also works without spring compensation) within 2 ± 0.3 s, and overshoot below 10%.',
          check: () => {
            if (ctx.S.mode !== 'work') return { ok: false, msg: 'Switch to Work mode.' };
            const res = ctx.app.result(), S = ctx.S, n = res.t.length;
            const i0 = Math.round(S.sim.tStep / S.sim.Ts);
            const m = M.stepMetrics(res.t, res.y, i0, n, res.y[i0], res.y[n - 1]);
            const ok = Math.abs(m.tr - 2) <= 0.3 && m.os < 10;
            return { ok, msg: `t_r = ${fmt(m.tr, 3)} s, overshoot ${fmt(m.os, 3)}% (of the final value ${fmt(res.y[n - 1], 3)} m).` };
          },
        },
        {
          id: 'b', title: '(b) t<sub>r</sub> that just saturates on a 1 m step (F<sub>max</sub> = 6 N)',
          inputs: { tr: 't<sub>r</sub> [s]' },
          html: 'Checked by simulation: a 1 m step from rest with ζ = 0.7, ω<sub>n</sub> = 2.2/t<sub>r</sub>, D on the output, and the current spring-compensation setting. The peak demanded |F| must land between 95% and 100% of F<sub>max</sub>.',
          check: (v) => {
            const tr = lib.num(v.tr);
            if (tr === null || tr <= 0) return { ok: false, msg: 'Enter a positive rise time.' };
            const pk = peakForTr(ctx, tr, prob.zeta, prob.step, ctx.st.comp);
            const pct = (100 * pk).toFixed(1);
            if (pk > 1.0005) return { ok: false, msg: `Peak demand is ${pct}% of Fmax, so it saturates. Slow it down.` };
            if (pk < 0.95) return { ok: false, msg: `Peak demand is ${pct}% of Fmax. You can go faster.` };
            return { ok: true, msg: `Peak demand is ${pct}% of Fmax (compensation: ${ctx.st.comp === 'eq' ? 'F = kz_r + F̃' : 'none'}).` };
          },
          actions: [{ label: 'Try it', run: (v) => {
            const tr = lib.num(v.tr);
            if (tr === null || tr <= 0) return { ok: false, msg: 'Enter a positive rise time.' };
            ctx.app.setMode('explore');
            Object.assign(ctx.st, { tr, zeta: prob.zeta, arch: 'output' });
            Object.assign(ctx.S.sim, { type: 'step', amplitude: prob.step, y0: 0 });
            ctx.update(); return null;
          } }],
          solution: () => {
            const none = satBound(ctx, prob.step, prob.zeta, 'none'), eq = satBound(ctx, prob.step, prob.zeta, 'eq');
            return [
              { html: 'The demand is largest right after the step, when z = 0 and ż = 0, so F(0<sup>+</sup>) = F<sub>e</sub> + k<sub>P</sub>·1 m ≤ F<sub>max</sub> (Eq. 8.8, p. 119). Then ω<sub>n</sub>² = (k + k<sub>P</sub>)/m and t<sub>r</sub> = 2.2/ω<sub>n</sub>.' },
              { tex: `\\text{none } (F_e = 0):\\; k_P = ${tex(none.kP)},\\; \\omega_n = ${tex(none.wn)},\\; t_r = ${tex(none.tr)}\\,\\text{s},\\; k_D = ${tex(none.kD)}` },
              { tex: `F = kz_r + \\tilde F \\;(F_e = ${tex(eq.Fe)}):\\; k_P = ${tex(eq.kP)},\\; \\omega_n = ${tex(eq.wn)},\\; t_r = ${tex(eq.tr)}\\,\\text{s},\\; k_D = ${tex(eq.kD)}` },
              { html: 'Fig. 7-2 as drawn (no F<sub>e</sub>) is the default here, and the (a) design leaves room to speed up. With F<sub>e</sub> = kz<sub>r</sub> the (a) design (k<sub>P</sub> = 3.05) already asks for 6.05 N, so it must be slowed slightly. The problem does not say which architecture to use (ISSUES.md).' },
            ];
          },
        },
      ]);
    },
  };

  // -------------------------------------------------------------- D.9 --
  function shapedReference(ctx, base) {
    const A = ctx.S.sim.amplitude, t0 = ctx.S.sim.tStep, y0 = ctx.S.sim.y0;
    if (ctx.st.input === 'ramp') return (t) => (t < t0 ? y0 : y0 + A * (t - t0));
    if (ctx.st.input === 'parabola') return (t) => (t < t0 ? y0 : y0 + A * (t - t0) ** 2);
    return base;
  }

  CH.ch9 = {
    id: 'ch9', num: 9, tab: 'D.9', title: 'System type & integrators', pages: 'pp. 137–154, p. 380',
    defaults() { return { comp: 'none', deriv: 'state', antiwindup: 'none', kP: 1, kD: 1, kI: 0, tr: 2, zeta: 0.7, kIx: 0.5, input: 'step' }; },
    simDefaults(sys) { return sys.problems.ch9.sim; },
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD } : designed(ctx); },
    controller: (ctx, o) => makePID(ctx, o),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePID),
    reference: shapedReference,

    buildControls(parent, ctx) {
      const sec = section(parent, 'PID controller', 'p. 142');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Reference shape (amplitude = size, slope, or coefficient)',
        options: [{ value: 'step', label: 'step' }, { value: 'ramp', label: 'ramp' }, { value: 'parabola', label: 'parabola' }],
        get: () => ctx.st.input, set: (v) => { ctx.st.input = v; ctx.update(); },
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
      const pred = st.input === 'step' ? A * a.step : st.input === 'ramp' ? A * a.ramp : 2 * A * a.parab;
      const res = ctx.app.result();
      const n = res ? res.r.length - 1 : 0;
      const iD = res ? Math.min(n, Math.max(0, Math.round(S.sim.tDist / S.sim.Ts) - 1)) : 0;
      const eEnd = res ? res.r[n] - res.y[n] : NaN, eBefore = res && S.sim.dist && S.sim.tDist > S.sim.tStep ? res.r[iD] - res.y[iD] : NaN;
      const row = (l, v) => el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: l }), el('strong', { text: v }));
      const show = S.mode === 'explore' || ctx.app.isRevealed('D:ch9:type');
      const rows = [];
      if (show) {
        rows.push(row('reference tracking type', `type ${a.type}`));
        rows.push(row(`predicted e_ss (${st.input}, Fig. 7-2 loop)`, isFinite(pred) ? `${fmt(pred, 3)} m` : '∞'));
        rows.push(row('input-disturbance type', `type ${a.distType}`));
        rows.push(row(`predicted extra e from d = ${fmt(S.sim.dist, 3)} N`, `${fmt(Math.abs(S.sim.dist) * a.dist, 3)} m`));
      } else {
        rows.push(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reveal the predicted type and errors', onclick: () => { ctx.app.reveal('D:ch9:type'); WB.ui.refreshAll(); } }));
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
          theory: 'M_p = \\lim_{s\\to0} PC,\\quad M_v = \\lim_{s\\to0} sPC,\\quad e_{step} = \\tfrac{1}{1+M_p},\\; e_{ramp} = \\tfrac{1}{M_v},\\; e_{parab} = \\tfrac{1}{M_a}',
          symbolic: g.kI > 0 ? 'PC = \\frac{(k_D s^2 + k_P s + k_I)/m}{s\\,(s^2 + \\frac bm s + \\frac km)} \\Rightarrow \\text{type 1},\\; M_v = \\frac{k_I}{k}' : 'PC = \\frac{(k_D s + k_P)/m}{s^2 + \\frac bm s + \\frac km} \\Rightarrow \\text{type 0},\\; M_p = \\frac{k_P}{k}',
          numbers: g.kI > 0 ? `M_v = ${tex(a.Mv)},\\quad e_{ramp} = \\frac{k}{k_I} = ${tex(a.ramp)}` : `M_p = ${tex(a.Mp)},\\quad e_{step} = \\frac{k}{k + k_P} = ${tex(a.step)}`,
          spoiler: true,
          note: 'For this plant the Fig. 7-1 and Fig. 7-2 loops give the same limits, because P(0) is finite.' },
        { title: 'Input disturbance', page: 'p. 143 · §9.1.3',
          theory: 'E(s) = -\\frac{P}{1+PC}D_{in}(s),\\quad \\lim_{s\\to0}\\frac{P}{1+PC}',
          numbers: g.kI > 0 ? '\\lim_{s\\to0}\\frac{P}{1+PC} = 0 \\;(\\text{integrator in } C)' : `\\lim_{s\\to0}\\frac{P}{1+PC} = \\frac{1}{k + k_P} = ${tex(a.dist)}\\;\\text{m/N}`,
          spoiler: true },
        compCard(ctx),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch9;
      const a = () => ans.typeAnalysis(ctx.pModel, ctx.gains);
      const g = () => ctx.gains;
      lib.panel(parent, ctx, prob, [
        {
          id: 'a1', title: '(a) PD only (set k<sub>I</sub> = 0)',
          html: 'Errors for a unit input, using the current k<sub>P</sub>. Type ∞ as <code>inf</code>.',
          inputs: { type: 'type', step: 'e<sub>step</sub>', ramp: 'e<sub>ramp</sub>', parab: 'e<sub>parab</sub>' },
          check: (v) => {
            if (g().kI > 0) return { ok: false, msg: 'Set kI = 0 first.' };
            const inf = (s) => /^\s*(inf|∞|infinity)\s*$/i.test(String(s));
            if (!inf(v.ramp) || !inf(v.parab)) return { ok: false, msg: 'Ramp and parabola errors: think about type 0.' };
            return lib.check({ type: v.type, step: v.step }, { type: 0, step: a().step }, { step: 'e_step' });
          },
          solution: () => [
            { tex: `PC = \\frac{(k_D s + k_P)/m}{s^2 + \\frac bm s + \\frac km} \\Rightarrow \\text{type 0},\\quad M_p = \\frac{k_P}{k} = ${tex(a().Mp)}` },
            { tex: `e_{step} = \\frac{1}{1 + M_p} = \\frac{k}{k + k_P} = ${tex(a().step)},\\quad e_{ramp} = e_{parab} = \\infty` },
            { html: 'Try it: compensation <em>none</em>, step input, k<sub>I</sub> = 0, and compare r − z at t_end.' },
          ],
        },
        {
          id: 'a2', title: '(a) With the integrator (k<sub>I</sub> > 0)',
          inputs: { type: 'type', step: 'e<sub>step</sub>', ramp: 'e<sub>ramp</sub>' },
          check: (v) => {
            if (!(g().kI > 0)) return { ok: false, msg: 'Set kI > 0 first.' };
            return lib.check(v, { type: 1, step: 0, ramp: a().ramp }, { ramp: 'e_ramp' });
          },
          solution: () => [{ tex: `PC = \\frac{(k_D s^2 + k_P s + k_I)/m}{s(s^2 + \\frac bm s + \\frac km)} \\Rightarrow \\text{type 1},\\; e_{step} = 0,\\; e_{ramp} = \\frac{k}{k_I} = ${tex(a().ramp)},\\; e_{parab} = \\infty` }],
        },
        {
          id: 'b', title: '(b) Constant input disturbance',
          html: 'Steady-state error per newton of d (m/N), without and with the integrator, at the current k<sub>P</sub>.',
          inputs: { pd: 'PD', pid: 'PID' },
          check: (v) => lib.check(v, { pd: 1 / (ctx.pModel.k + g().kP), pid: 0 }, { pd: 'PD', pid: 'PID' }),
          solution: () => [
            { tex: `\\text{PD}: \\lim_{s\\to0}\\frac{P}{1+PC} = \\frac{1/k}{1 + k_P/k} = \\frac{1}{k + k_P} = ${tex(1 / (ctx.pModel.k + g().kP))},\\quad \\text{PID}: 0` },
            { html: 'An error in k is such a disturbance: F<sub>e</sub> = k̂z<sub>r</sub> misses k z<sub>r</sub> by (k − k̂)z<sub>r</sub>. Try it: d = 0.5 N starts at t = 20 s.' },
          ],
        },
      ]);
    },
  };

  // ------------------------------------------------------------- D.P.6 --
  // Branch-tracked root locus of Delta(s) + k n(s) = 0 for k in [0, kMax].
  function rootLocus(den, num, kMax, steps = 300) {
    const branches = [];
    let prev = null;
    for (let i = 0; i <= steps; i++) {
      const k = kMax * Math.pow(i / steps, 2);
      let r = L.roots(L.polyAdd(den, L.polyScale(num, k)));
      if (prev) {
        const used = new Set(), ordered = [];
        for (const p of prev) {
          let best = -1, bd = Infinity;
          r.forEach((q, j) => { if (!used.has(j)) { const d = Math.hypot(q.re - p.re, q.im - p.im); if (d < bd) { bd = d; best = j; } } });
          used.add(best); ordered.push(r[best]);
        }
        r = ordered;
      } else r.forEach(() => branches.push([]));
      r.forEach((q, j) => branches[j].push(q));
      prev = r;
    }
    return branches;
  }

  CH.p6 = {
    id: 'p6', num: 10.5, tab: 'D.P.6', short: 'P.6', title: 'Root locus vs. k_I', pages: 'pp. 465–474, p. 380',
    defaults(sys) { const pr = sys.problems.p6; return { comp: 'none', deriv: 'state', antiwindup: 'none', tr: pr.tr, zeta: pr.zeta, kIx: 0.1, kMaxFactor: 1 }; },
    simDefaults(sys) { return sys.problems.p6.sim; },
    gains(ctx) { return designed(ctx); },
    controller: (ctx, o) => makePID(ctx, o),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePID),

    buildControls(parent, ctx) {
      const sec = section(parent, 'PD from the D.8 specs, then add k_I', 'p. 466');
      compControl(sec, ctx);
      specSliders(sec, ctx, { kI: true });
      slider(sec, { label: 'locus to', unit: '× kI,crit', min: 0.1, max: 3, step: 0.05, sig: 2, get: () => ctx.st.kMaxFactor, set: (v) => { ctx.st.kMaxFactor = v; ctx.update(); } });
      lib.note(sec, 'Drag a closed-loop pole along the locus to set k_I. In Work mode the numbers stay hidden; the locus itself is the plot the problem asks for.');
      if (ctx.S.mode === 'explore') readout(sec, ctx);
    },

    splane(ctx) {
      const g = designed(ctx), ev = ans.evans(ctx.pModel, g);
      const kMax = Math.max(ev.kCrit * ctx.st.kMaxFactor, g.kI * 1.2, 1e-3);
      const mk = L.roots(ev.den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of L(s) ${i + 1}` }));
      L.roots(pidCharPoly(ctx.model, g)).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole at kI = ${fmt(g.kI, 3)}`, dragId: i }));
      const fitR = Math.max(...L.roots(ev.den).map((p) => Math.hypot(p.re, p.im))) * 1.6;
      return { markers: mk, loci: rootLocus(ev.den, ev.num, kMax), fitR };
    },
    onPoleDrag(ctx, id, re, im) {
      const ev = ans.evans(ctx.pModel, designed(ctx));
      const kMax = Math.max(ev.kCrit * ctx.st.kMaxFactor, 1e-3);
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
      const g = designed(ctx), ev = ans.evans(ctx.pModel, g);
      return [
        { title: 'Closed loop with PID (derivative on output)', page: 'p. 470 (A.P.6 pattern)',
          theory: '\\Delta_{cl}(s) = s^3 + (a_1 + b_0k_D)s^2 + (a_0 + b_0k_P)s + b_0k_I',
          symbolic: '\\Delta_{cl}(s) = s^3 + \\frac{b + k_D}{m}s^2 + \\frac{k + k_P}{m}s + \\frac{k_I}{m}',
          numbers: `\\Delta_{cl}(s) = ${WB.tf.polyTex(pidCharPoly(ctx.model, g))}`, spoiler: true },
        { title: 'Evans form', page: 'p. 466',
          theory: '1 + K\\,L(s) = 0,\\quad K = k_I',
          symbolic: 'L(s) = \\frac{1/m}{s^3 + \\frac{b + k_D}{m}s^2 + \\frac{k + k_P}{m}s}',
          numbers: `L(s) = \\frac{${tex(ctx.model.b0)}}{${WB.tf.polyTex(ev.den)}},\\quad k_P = ${tex(g.kP)},\\; k_D = ${tex(g.kD)}`, spoiler: true },
        { title: 'Where the locus crosses into the RHP', page: 'Routh–Hurwitz (not in the book)',
          theory: 's^3 + c_2 s^2 + c_1 s + c_0 \\text{ is stable iff } c_2, c_1, c_0 > 0 \\text{ and } c_2 c_1 > c_0',
          numbers: `k_{I,crit} = \\frac{c_2 c_1}{b_0} = ${tex(ev.kCrit)}`, spoiler: true },
        compCard(ctx),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.p6;
      const ev = () => ans.evans(ctx.pModel, designed(ctx));
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: 'Evans form: L(s) = c / (s³ + d<sub>2</sub>s² + d<sub>1</sub>s)',
          html: 'Uses the PD gains from the current t<sub>r</sub>, ζ (D.8(a): 2 s, 0.7).',
          inputs: { c: 'c', d2: 'd<sub>2</sub>', d1: 'd<sub>1</sub>' },
          check: (v) => lib.check(v, { c: ctx.model.b0, d2: ev().den[1], d1: ev().den[2] }, {}),
          solution: () => [{ tex: `1 + k_I\\frac{${tex(ctx.model.b0)}}{${WB.tf.polyTex(ev().den)}} = 0` }, { html: 'd<sub>2</sub> = (b + k<sub>D</sub>)/m and d<sub>1</sub> = (k + k<sub>P</sub>)/m are the coefficients of the PD design polynomial: α<sub>1</sub> = 2ζω<sub>n</sub>, α<sub>0</sub> = ω<sub>n</sub>².' }],
        },
        {
          id: 'b', title: 'Largest stable k<sub>I</sub>',
          inputs: { k: 'k<sub>I,crit</sub>' },
          check: (v) => lib.check(v, { k: ev().kCrit }, { k: 'kI,crit' }),
          solution: () => [{ tex: `c_2c_1 > b_0k_I \\Rightarrow k_{I,crit} = \\frac{(2\\zeta\\omega_n)(\\omega_n^2)}{1/m} = ${tex(ev().kCrit)}` }],
        },
        {
          id: 'c', title: 'Pick k<sub>I</sub> that barely moves the PD poles',
          html: 'Checks the current k<sub>I</sub>: the complex pair must stay within 10% (in |p|) of the PD-only poles, and the new real pole must be slower than them.',
          check: () => {
            const g = designed(ctx);
            if (!(g.kI > 0)) return { ok: false, msg: 'Set kI > 0.' };
            const pd = M.roots2(ctx.model.a1 + ctx.model.b0 * g.kD, ctx.model.a0 + ctx.model.b0 * g.kP);
            const cl = L.roots(pidCharPoly(ctx.model, g));
            const cpx = cl.filter((p) => Math.abs(p.im) > 1e-6), real = cl.filter((p) => Math.abs(p.im) <= 1e-6);
            if (cpx.length !== 2) return { ok: false, msg: 'The PD pair has split into real poles; kI is too large.' };
            const ratio = Math.hypot(cpx[0].re, cpx[0].im) / Math.hypot(pd[0].re, pd[0].im);
            const slow = real.length === 1 && Math.abs(real[0].re) < Math.abs(pd[0].re);
            return { ok: Math.abs(ratio - 1) < 0.1 && slow, msg: `|p| ratio ${fmt(ratio, 3)}, real pole ${real.length ? fmtPole(real[0]) : '—'}.` };
          },
          solution: () => [{ html: 'With the D.8(a) gains (k<sub>P</sub> = 3.05, k<sub>D</sub> = 7.2), any k<sub>I</sub> up to about 0.8 keeps |p| within 10%: k<sub>I</sub> = 0.5 gives −0.724 ± 0.743j and −0.093. The price is a slow integrator pole, partly cancelled by the zero at −k<sub>I</sub>/k<sub>P</sub>.' }],
        },
      ]);
    },
  };

  // -------------------------------------------------------------- D.10 --
  CH.ch10 = {
    id: 'ch10', num: 10, tab: 'D.10', title: 'Digital PID', pages: 'pp. 155–169, p. 380',
    defaults(sys) {
      const pr = sys.problems.ch10;
      return { comp: 'eq', deriv: 'dirty', antiwindup: 'gate', vbar: 0.05, sigma: pr.sigma, kP: 1, kD: 1, kI: 0, tr: pr.tr, zeta: pr.zeta, kIx: pr.kiRef, extra: 'deriv' };
    },
    simDefaults(sys) { return { ...sys.problems.ch10.sim, mismatch: sys.problems.ch10.mismatch }; },
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD } : designed(ctx); },
    toWork(ctx) { const g = ctx.gains; Object.assign(ctx.st, { kP: g.kP, kD: g.kD, kI: g.kI }); },
    controller: (ctx, o) => makePID(ctx, o),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, makePID),
    targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.tr } : {}; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Digital PID', 'p. 155 · Listing 10.2 p. 161');
      compControl(sec, ctx);
      if (ctx.S.mode === 'work') gainSliders(sec, ctx, ['kP', 'kI', 'kD']);
      else { specSliders(sec, ctx, { kI: true }); readout(sec, ctx); }
      const imp = section(parent, 'Implementation', 'p. 157 · Eq. 10.3–10.4');
      segmented(imp, {
        label: 'ż for the D term',
        options: [{ value: 'dirty', label: 'dirty derivative of z (D.10b)' }, { value: 'state', label: 'true ż (cheating)' }],
        get: () => ctx.st.deriv, set: (v) => { ctx.st.deriv = v; ctx.update(); },
      });
      slider(imp, { label: 'σ', unit: 's', min: 0.002, max: 0.5, step: 0.001, sig: 3, hint: 'dirty-derivative bandwidth is 1/σ rad/s', get: () => ctx.st.sigma, set: (v) => { ctx.st.sigma = v; ctx.update(); }, disabled: () => ctx.st.deriv !== 'dirty' });
      segmented(imp, {
        label: 'Anti-windup',
        options: [
          { value: 'gate', label: 'integrate when |ż| < v̄', title: 'Listing 10.2 pattern' },
          { value: 'backcalc', label: 'back-calculation', title: 'u_I += (u_sat − u_unsat)/k_I, p. 158' },
          { value: 'none', label: 'none' },
        ],
        get: () => ctx.st.antiwindup, set: (v) => { ctx.st.antiwindup = v; ctx.update(); },
      });
      slider(imp, { label: 'v̄', unit: 'm/s', min: 0.005, max: 1, step: 0.005, sig: 3, get: () => ctx.st.vbar, set: (v) => { ctx.st.vbar = v; ctx.update(); }, disabled: () => ctx.st.antiwindup !== 'gate' });
      segmented(imp, {
        label: 'Extra plot', options: [{ value: 'deriv', label: 'ż estimate' }, { value: 'int', label: 'integrator' }],
        get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); },
      });
    },

    splane(ctx) { return { markers: markers(ctx) }; },

    extraPlot(ctx, res) {
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
      const beta = (2 * st.sigma - Ts) / (2 * st.sigma + Ts), gamma = 2 / (2 * st.sigma + Ts);
      const d = designed(ctx);
      return [
        { title: 'PID from the measured output', page: 'p. 155',
          theory: 'F(t) = F_e + k_P e(t) + k_I \\int_{-\\infty}^{t} e(\\tau)\\,d\\tau - k_D \\dot z(t)' },
        { title: 'Trapezoidal integrator', page: 'p. 157 · Eq. 10.2–10.3',
          theory: 'u_I[n] = u_I[n-1] + \\frac{T_s}{2}\\big(e[n] + e[n-1]\\big)' },
        { title: 'Dirty derivative', page: 'p. 157 · Eq. 10.4',
          theory: '\\dot{\\hat z}[n] = \\frac{2\\sigma - T_s}{2\\sigma + T_s}\\dot{\\hat z}[n-1] + \\frac{2}{2\\sigma + T_s}\\big(z[n] - z[n-1]\\big)',
          numbers: `\\sigma = ${tex(st.sigma)},\\; T_s = ${tex(Ts)}:\\quad ${tex(beta)},\\quad ${tex(gamma)}`, spoiler: true },
        { title: 'Anti-windup', page: 'p. 157 · §10.1.1',
          theory: '\\text{(1) integrate only when } |\\dot z| < \\bar v,\\quad \\text{(2) } u_I^+ = u_I + \\frac{1}{k_I}\\big(u_{sat} - u_{unsat}\\big)' },
        { title: 'Gains from t_r, ζ (D.8)', page: 'p. 113 · Eq. 8.5, p. 160 · §10.1.3',
          theory: '\\omega_n = \\frac{2.2}{t_r},\\quad k_P = \\frac{\\omega_n^2 - a_0}{b_0},\\quad k_D = \\frac{2\\zeta\\omega_n - a_1}{b_0},\\quad \\text{then raise } k_I \\text{ from 0}',
          numbers: `\\omega_n = ${tex(d.wn)},\\quad k_P = ${tex(d.kP)},\\quad k_D = ${tex(d.kD)},\\quad k_I = ${tex(d.kI)}`, spoiler: true },
        compCard(ctx),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch10;
      const ref = () => ans.spec(ctx.pModel, prob.tr, prob.zeta);
      lib.panel(parent, ctx, prob, [
        {
          id: 'c1', title: `(c) PD part for t<sub>r</sub> = ${prob.tr} s, ζ = ${prob.zeta}`,
          inputs: { kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>' },
          check: (v) => lib.check(v, { kP: ref().kP, kD: ref().kD }, { kP: 'kP', kD: 'kD' }),
          actions: [{ label: 'Use my gains', run: (v) => {
            const kP = lib.num(v.kP), kD = lib.num(v.kD);
            if (kP === null || kD === null) return { ok: false, msg: 'Enter kP and kD first.' };
            ctx.app.setMode('work'); Object.assign(ctx.st, { kP, kD }); ctx.update(); return null;
          } }],
          solution: () => [{ tex: `k_P = ${tex(ref().kP)},\\quad k_D = ${tex(ref().kD)}\\quad(\\text{D.8(a)})` }],
        },
        {
          id: 'c2', title: '(c) Dirty-derivative coefficients for σ = 0.05, T<sub>s</sub> = 0.01',
          inputs: { a: '(2σ−T<sub>s</sub>)/(2σ+T<sub>s</sub>)', b: '2/(2σ+T<sub>s</sub>)' },
          check: (v) => lib.check(v, { a: 0.09 / 0.11, b: 2 / 0.11 }, {}),
          solution: () => [{ tex: '\\frac{0.09}{0.11} = 0.8182,\\quad \\frac{2}{0.11} = 18.18' }],
        },
        {
          id: 'c3', title: '(c) Tune k<sub>I</sub> for the uncertain plant',
          html: 'Checks the current simulation (plant mismatch in the left panel): |z<sub>r</sub> − z| just before the first reference switch must be under 1% of the step.',
          check: () => {
            const { e, amp } = lib.errorBeforeSwitch(ctx);
            const msg = `Error before the switch: ${fmt(e * 1000, 3)} mm (limit ${fmt(10 * amp, 3)} mm).`;
            if (!(ctx.gains.kI > 0)) return { ok: false, msg: `kI = 0. ${msg}` };
            return { ok: e < 0.01 * amp, msg };
          },
          solution: () => [{ html: `With the D.8(a) PD gains, k<sub>I</sub> ≈ ${prob.kiRef} works well here (closed-loop poles about −0.66 ± 0.70j and −0.22). Smaller k<sub>I</sub> leaves a slow tail (the integrator pole approaches 0); much larger k<sub>I</sub> erodes the damping (k<sub>I,crit</sub> ≈ 9.3, see D.P.6).` }],
        },
      ]);
    },
  };

  WB.studies.D.pid = { makePD, makePID, pidCharPoly, compControl, gainSliders, designed };
})();
