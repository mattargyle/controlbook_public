// Study F, Chapters 7–10 and Appendix P.6 (F.7–F.10, F.P.6): PD altitude control,
// successive loop closure for the lateral dynamics, system type, root locus
// versus the integrator gains, and the digital nested PID from measured outputs.
//
// Work mode: you set the loop gains; the s-plane shows target rings from the
// specs, and every derived number is behind Reveal / Show solution.
// Explore mode: gains come from the specs (t_r, ζ, separation M); drag the poles.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const F = WB.F;
  const T = WB.tf;
  const { tex, texPole, fmt, fmtPole } = M;
  const PD = () => WB.pd;
  const ALL = ['kPh', 'kDh', 'kIh', 'kPth', 'kDth', 'kPz', 'kDz', 'kIz'];
  const pick = (g, keys) => Object.fromEntries(keys.map((k) => [k, g[k] || 0]));
  const m0 = (p) => WB.systems.F.models(p);
  // F.7(b) truth: Δ_cl(s) = s² + b₀k_D s + b₀k_P with b₀ = 1/(m_c + 2m_r)
  const lonDen = (p, a) => { const b = m0(p).lon.b0; return WB.py.cx.poly([1, b * a.kD, b * a.kP], a.s); };

  // spec knobs from F.8 (book values); explore mode designs from these
  function f8Knobs(sys) {
    const pr = sys.problems.ch8;
    return { trh: pr.trh, zetah: pr.zetah, trth: pr.trth, zetath: pr.zetath, Msep: pr.Msep, zetaz: pr.zetaz, kIh: 0, kIz: 0 };
  }
  // target poles from the knobs (rings in Work mode)
  function knobTargets(ctx, k = ctx.st.k) {
    const d = F.designSLC(ctx.pModel, k);
    return { lon: F.polesWZ(d.wnh, k.zetah), inner: F.polesWZ(d.wnt, k.zetath), outer: F.polesWZ(d.wnz, k.zetaz) };
  }
  // The rings are the pole locations F.8(a, b, d) ask for: each appears once its part is solved.
  function solvedTargets(ctx) {
    const t = knobTargets(ctx);
    return { lon: F.showsAnswer(ctx, 'F.8/a') ? t.lon : null, inner: F.showsAnswer(ctx, 'F.8/b') ? t.inner : null, outer: F.showsAnswer(ctx, 'F.8/d') ? t.outer : null };
  }

  // ------------------------------------------------------- controls --
  function workSections(parent, ctx, { lon = true, lat = true, kI = false } = {}) {
    if (lon) {
      const s1 = section(parent, 'Altitude loop gains', 'p. 101 · Fig. 7-2');
      F.gainSliders(s1, ctx, kI ? ['kPh', 'kDh', 'kIh'] : ['kPh', 'kDh']);
    }
    if (lat) {
      const s2 = section(parent, 'Lateral: inner θ loop (PD)', 'p. 118 · Fig. 8-10');
      F.gainSliders(s2, ctx, ['kPth', 'kDth']);
      const s3 = section(parent, 'Lateral: outer z loop', 'p. 118 · Fig. 8-11');
      F.gainSliders(s3, ctx, kI ? ['kPz', 'kDz', 'kIz'] : ['kPz', 'kDz']);
      s3.append(el('p', { class: 'muted small', text: 'Mind the sign of Z/Θ (F.5) when you pick the outer gains.' }));
    }
  }
  function knobSections(parent, ctx, { lon = true, lat = true, title = 'Design knobs' } = {}) {
    const k = ctx.st.k;
    if (lon) {
      const s1 = section(parent, `${title}: altitude`, 'p. 113 · Eq. 8.5');
      slider(s1, { label: 't<sub>r,h</sub>', unit: 's', min: 0.5, max: 20, step: 0.01, sig: 3, ...bind(ctx, 'trh', () => k) });
      slider(s1, { label: 'ζ<sub>h</sub>', min: 0.2, max: 1.5, step: 0.005, sig: 3, ...bind(ctx, 'zetah', () => k) });
    }
    if (lat) {
      const s2 = section(parent, `${title}: lateral`, 'p. 118 · §8.1.4');
      slider(s2, { label: 't<sub>r,θ</sub>', unit: 's', min: 0.1, max: 3, step: 0.005, sig: 3, ...bind(ctx, 'trth', () => k) });
      slider(s2, { label: 'ζ<sub>θ</sub>', min: 0.2, max: 1.5, step: 0.005, sig: 3, ...bind(ctx, 'zetath', () => k) });
      slider(s2, { label: 'M = t<sub>r,z</sub>/t<sub>r,θ</sub>', min: 1, max: 30, step: 0.1, sig: 3, ...bind(ctx, 'Msep', () => k), hint: 'bandwidth separation (5–10 is the rule of thumb, p. 118)' });
      slider(s2, { label: 'ζ<sub>z</sub>', min: 0.2, max: 1.5, step: 0.005, sig: 3, ...bind(ctx, 'zetaz', () => k) });
    }
  }
  function separationSection(parent, ctx) {
    const sec = section(parent, 'Loop separation', 'p. 118');
    const box = el('div', { class: 'metrics' });
    sec.append(box);
    WB.ui.addRefresher(() => {
      const r = F.separationRows(ctx, ctx.gains);
      const show = WB.ui.shown(ctx, 'F:sep') || F.showsAnswer(ctx, ['F.8/c', 'F.8/d']);
      box.replaceChildren(...(show ? [
        F.metricRow('inner ω_n,θ', `${fmt(r.wi, 3)} rad/s`),
        F.metricRow('outer ω_n,z (k_DC model)', `${fmt(r.wo, 3)} rad/s`),
        F.metricRow('separation ω_n,θ/ω_n,z ≈ t_r,z/t_r,θ', `${fmt(r.ratio, 3)}`),
        F.metricRow('inner DC gain k_DC,θ', `${fmt(r.kDC, 4)}`),
      ] : [WB.ui.revealButton(ctx, 'F:sep', 'Reveal the loop poles and separation')]));
    });
  }

  // --------------------------------------------------------- math cards --
  function lonCard(ctx, g) {
    const m = ctx.sys.models(ctx.pModel);
    const cl = F.rootsOf(F.lonPoly(m, g));
    return {
      title: 'Altitude loop (PD, derivative on h)', page: 'F.7(b) p. 397', answers: 'F.7/b',
      theory: '\\frac{\\tilde H}{\\tilde H_r} = \\frac{k_{P_h}/(m_c+2m_r)}{s^2 + \\frac{k_{D_h}}{m_c+2m_r}s + \\frac{k_{P_h}}{m_c+2m_r}},\\quad F = F_e + \\tilde F',
      numbers: `\\Delta_{cl} = ${T.polyTex(F.lonPoly(m, g))},\\quad p = ${cl.map((q) => texPole(q)).join(',\\;')}`,
      note: 'Numbers for the current gains.',
    };
  }
  function slcCards(ctx, g) {
    const m = ctx.sys.models(ctx.pModel);
    const kDC = F.kDCof(m, g);
    return [
      { title: 'Successive loop closure', page: 'p. 118 · Fig. 8-10, 8-11',
        theory: 'T_{in}(s) = \\frac{P_1C_1}{1 + P_1C_1},\\quad k_{DC_1} = \\lim_{s\\to0}T_{in}(s),\\quad \\text{outer loop sees } k_{DC_1}P_2(s),\\quad 1 + C_{out}(s)\\,T_{in}(s)\\,P_2(s) = 0 \\;(\\text{exact})' },
      { title: 'Inner loop τ → θ', page: 'F.8(b, c) p. 397', answers: ['F.8/b', 'F.8/c'],
        theory: '\\frac{\\Theta}{\\Theta_d} = \\frac{k_{P_\\theta}/J}{s^2 + \\frac{k_{D_\\theta}}{J}s + \\frac{k_{P_\\theta}}{J}},\\quad J = J_c + 2m_rd^2,\\quad k_{DC_\\theta} = \\frac{k_{P_\\theta}/J}{k_{P_\\theta}/J} = 1',
        numbers: `\\Delta_\\theta = ${T.polyTex(F.innerPoly(m, g))},\\quad p_\\theta = ${F.rootsOf(F.innerPoly(m, g)).map((q) => texPole(q)).join(',\\;')},\\quad k_{DC_\\theta} = ${tex(kDC)}` },
      { title: 'Outer loop θ_d → z (inner loop as its DC gain)', page: 'F.8(d) p. 397', answers: 'F.8/d',
        theory: '\\frac{Z}{Z_r} = \\frac{-g k_{DC_\\theta}k_{P_z}}{s^2 + \\big(\\frac{\\mu}{m_c+2m_r} - g k_{DC_\\theta}k_{D_z}\\big)s - g k_{DC_\\theta}k_{P_z}}',
        numbers: `\\Delta_z = ${T.polyTex(F.outerPoly(m, g))},\\quad p_z = ${F.rootsOf(F.outerPoly(m, g)).map((q) => texPole(q)).join(',\\;')}` },
      { title: 'Exact lateral loop (inner dynamics kept)', page: 'checks the k_DC approximation', answers: 'F.8/d',
        theory: '\\Big(s^2 + \\tfrac{\\mu}{M}s\\Big)\\Big(s^2 + \\tfrac{k_{D_\\theta}}{J}s + \\tfrac{k_{P_\\theta}}{J}\\Big) - \\tfrac{g k_{P_\\theta}}{J}\\big(k_{D_z}s + k_{P_z}\\big) = 0',
        numbers: `p = ${F.rootsOf(F.exactLatPoly(m, g)).map((q) => texPole(q)).join(',\\;')}`,
        note: 'The open circles in the lateral s-plane view. With good separation they sit next to the inner and outer design poles.' },
    ];
  }
  const specCards = (ctx, k) => {
    const d = F.designSLC(ctx.pModel, k);
    return [{
      title: 'Specs → gains', page: 'p. 113 · Eq. 8.5, p. 100',
      theory: '\\omega_n = \\frac{2.2}{t_r},\\quad k_P = \\frac{\\omega_n^2 - a_0}{b_0},\\quad k_D = \\frac{2\\zeta\\omega_n - a_1}{b_0}\\;(\\text{plant } \\tfrac{b_0}{s^2 + a_1s + a_0}),\\quad t_{r_z} = M\\,t_{r_\\theta}',
    }, {
      title: 'Gains for the spec knobs', page: 'F.8(a, b, d) p. 397', answers: ['F.8/a', 'F.8/b', 'F.8/d'],
      numbers: `\\omega_{n_h} = ${tex(d.wnh)},\\; k_{P_h} = ${tex(d.kPh)},\\; k_{D_h} = ${tex(d.kDh)}\\quad \\omega_{n_\\theta} = ${tex(d.wnt)},\\; k_{P_\\theta} = ${tex(d.kPth)},\\; k_{D_\\theta} = ${tex(d.kDth)}\\quad \\omega_{n_z} = ${tex(d.wnz)},\\; k_{P_z} = ${tex(d.kPz)},\\; k_{D_z} = ${tex(d.kDz)}`,
    }];
  };

  // ------------------------------------------------------------- Chapter 7 --
  F.register({
    id: 'ch7', num: 7, tab: 'Ch 7', title: 'Pole placement (PD altitude)', pages: 'pp. 99–106, F.7 p. 397',
    lateralMetrics: false,
    controller: (ctx, o) => F.makePID(ctx, o),
    defaults(sys) {
      const pr = sys.problems.ch7;
      return { comp: 'eq', lat: 'off', view: 'lon', zOff: 0, hOff: 0, w: { ...F.W0 }, form: 'real', p1: pr.desiredPoles[0].re, p2: pr.desiredPoles[1].re, sigma: -0.25, wd: 0.2 };
    },
    simDefaults(sys) { return { ...sys.problems.ch7.sim, refs: [{ amplitude: 0 }] }; },
    designPoles(ctx) {
      const st = ctx.st;
      return st.form === 'real' ? [{ re: st.p1, im: 0 }, { re: st.p2, im: 0 }] : [{ re: st.sigma, im: st.wd }, { re: st.sigma, im: -st.wd }];
    },
    gains(ctx) {
      const z = pick({}, ALL);
      if (ctx.S.mode === 'work') return { ...z, kPh: ctx.st.w.kPh, kDh: ctx.st.w.kDh };
      const m = ctx.sys.models(ctx.pModel);
      const g = F.pdFromPoles(m.lon, this.designPoles(ctx));
      return { ...z, kPh: g.kP, kDh: g.kD };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'PD altitude controller', 'p. 99 · F.7 p. 397');
      F.forceLawControl(sec, ctx);
      if (ctx.S.mode === 'work') {
        F.gainSliders(sec, ctx, ['kPh', 'kDh']);
        sec.append(el('p', { class: 'muted small', text: 'The lateral loop is off (τ = 0), as in F.7. Target poles are the dashed rings.' }));
        return;
      }
      const des = section(parent, 'Desired closed-loop poles', 'p. 100');
      segmented(des, { label: 'Pole pair', options: [{ value: 'real', label: 'two real' }, { value: 'complex', label: 'complex pair' }], ...bind(ctx, 'form') });
      slider(des, { label: 'p<sub>1</sub>', min: -2, max: 0, step: 0.001, ...bind(ctx, 'p1'), disabled: () => ctx.st.form !== 'real' });
      slider(des, { label: 'p<sub>2</sub>', min: -2, max: 0, step: 0.001, ...bind(ctx, 'p2'), disabled: () => ctx.st.form !== 'real' });
      slider(des, { label: '−σ', min: -2, max: 0, step: 0.001, ...bind(ctx, 'sigma'), disabled: () => ctx.st.form !== 'complex' });
      slider(des, { label: 'ω<sub>d</sub>', unit: 'rad/s', min: 0, max: 2, step: 0.001, ...bind(ctx, 'wd'), disabled: () => ctx.st.form !== 'complex' });
      F.readout(des, ctx, [{ key: 'kPh', label: 'kP' }, { key: 'kDh', label: 'kD' }]);
    },
    splane(ctx) {
      const tg = ctx.S.mode === 'work' ? { lon: ctx.sys.problems.ch7.desiredPoles } : null;
      // the open-loop poles answer F.7(a): pidSplane leaves them off in Work mode until it is solved
      return F.pidSplane(ctx, ctx.gains, { draggable: ctx.S.mode === 'explore', targets: tg });
    },
    onPoleDrag(ctx, id, re, im) {
      const st = ctx.st;
      re = Math.min(-0.001, re);
      if (Math.abs(im) > 0.01) { st.form = 'complex'; st.sigma = re; st.wd = Math.abs(im); }
      else { if (st.form === 'complex') st.p1 = st.p2 = st.sigma; st.form = 'real'; if (Math.abs(re - st.p1) < Math.abs(re - st.p2)) st.p1 = re; else st.p2 = re; }
      ctx.update();
    },
    math(ctx) {
      const m = ctx.sys.models(ctx.pModel);
      const des = ctx.S.mode === 'work' ? ctx.sys.problems.ch7.desiredPoles : this.designPoles(ctx);
      const gd = F.pdFromPoles(m.lon, des);
      const { alpha1, alpha0 } = M.polyFromPoles(des[0], des[1]);
      return [
        { title: 'PD with the derivative on the output', page: 'p. 101 · Fig. 7-2, Eq. 7.5',
          theory: 'P(s) = \\frac{b_0}{s^2 + a_1s + a_0},\\; u = k_P(y_r - y) - k_D\\dot y:\\quad \\frac{Y}{Y_r} = \\frac{b_0k_P}{s^2 + (a_1 + b_0k_D)s + (a_0 + b_0k_P)}' },
        { title: 'Plant (F.5)', page: 'F.5(b) p. 396', answers: 'F.5/b',
          theory: 'P_{lon}(s) = \\frac{\\tilde H(s)}{\\tilde F(s)} = \\frac{1/(m_c+2m_r)}{s^2}', numbers: `P_{lon}(s) = \\frac{${tex(m.lon.b0)}}{s^2}` },
        { title: 'Open-loop poles', page: 'F.7(a) p. 397', answers: 'F.7/a', theory: 'p_{ol} = 0,\\; 0 \\;(\\text{a double integrator})' },
        lonCard(ctx, ctx.gains),
        { title: 'Pole placement', page: 'p. 100',
          theory: '\\Delta^d_{cl} = (s - p_1)(s - p_2) = s^2 + \\alpha_1 s + \\alpha_0,\\quad k_P = \\frac{\\alpha_0 - a_0}{b_0},\\quad k_D = \\frac{\\alpha_1 - a_1}{b_0}' },
        { title: ctx.S.mode === 'work' ? 'Gains for the problem targets' : 'Gains for your design', page: 'F.7(c) p. 397', answers: 'F.7/c',
          theory: 'k_P = (m_c+2m_r)\\alpha_0,\\quad k_D = (m_c+2m_r)\\alpha_1',
          numbers: `\\Delta^d_{cl} = s^2 + ${tex(alpha1)}s + ${tex(alpha0)} \\Rightarrow k_P = ${tex(gd.kP)},\\; k_D = ${tex(gd.kD)}` },
        { title: 'Equilibrium force (F.4)', page: 'F.4(a) p. 396', answers: 'F.4/a',
          theory: 'F = F_e + \\tilde F,\\quad F_e = (m_c+2m_r)g', numbers: `F_e = ${tex(m.Fe)}\\,\\text{N}` },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch7;
      const m = () => ctx.sys.models(ctx.pModel);
      const ans = () => F.pdFromPoles(m().lon, prob.desiredPoles);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Open-loop poles', inputs: { p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>' }, html: 'Complex values are fine: <code>-1+2j</code>.',
          check: (v) => { const g = [M.parseComplex(v.p1), M.parseComplex(v.p2)]; if (!g[0] || !g[1]) return { ok: false, msg: 'Enter both poles.' }; return M.polesMatch(g, [{ re: 0, im: 0 }, { re: 0, im: 0 }]) ? { ok: true, msg: '' } : { ok: false, msg: 'Roots of the denominator of H̃/F̃?' }; },
          solution: () => [{ tex: `\\frac{\\tilde H}{\\tilde F} = \\frac{${tex(m().lon.b0)}}{s^2} \\Rightarrow p_{ol} = 0, 0 \\;(\\text{a double integrator})` }],
        },
        {
          id: 'b', title: '(b) Closed-loop transfer function and poles',
          html: 'PD with the derivative on the output (Fig. 7-2). Write the transfer function from h̃<sub>r</sub> to h̃ and the closed-loop characteristic polynomial (its roots are the poles) in terms of k<sub>P</sub> and k<sub>D</sub>; the check calls them at complex s. Any nonzero multiple of Δ<sub>cl</sub> is accepted.',
          code: F.pyPart(ctx, {
            args: { kP: { label: 'kP', lo: 0.02, hi: 2 }, kD: { label: 'kD', lo: 0.05, hi: 3 } },
            items: [
              { fn: 'closed_loop', args: ['s', 'kP', 'kD'], truth: (p, a) => WB.py.cx.div(m0(p).lon.b0 * a.kP, lonDen(p, a)) },
              { fn: 'char_poly', args: ['s', 'kP', 'kD'], compare: 'scale', truth: lonDen },
            ],
          }, 'def closed_loop(s, kP, kD):\n    # H~(s)/H~_r(s)\n    return ...\n\ndef char_poly(s, kP, kD):\n    # Delta_cl(s)\n    return ...\n'),
          solution: () => [
            { tex: '\\frac{\\tilde H}{\\tilde H_r} = \\frac{k_P/(m_c+2m_r)}{s^2 + \\frac{k_D}{m_c+2m_r}s + \\frac{k_P}{m_c+2m_r}},\\quad p = -\\frac{k_D}{2(m_c+2m_r)} \\pm \\sqrt{\\Big(\\frac{k_D}{2(m_c+2m_r)}\\Big)^2 - \\frac{k_P}{m_c+2m_r}}' },
            { code: 'b0 = 1 / (P.mc + 2 * P.mr)\n\ndef char_poly(s, kP, kD):\n    return s**2 + b0 * kD * s + b0 * kP\n\ndef closed_loop(s, kP, kD):\n    return b0 * kP / char_poly(s, kP, kD)' },
            { html: 'Eq. 7.5 (p. 101) with b<sub>0</sub> = 1/(m<sub>c</sub>+2m<sub>r</sub>), a<sub>1</sub> = a<sub>0</sub> = 0.' },
          ],
        },
        {
          id: 'c', title: '(c) Gains for poles at −0.2 and −0.3',
          inputs: { kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>' },
          check: (v) => PD().checkNumbers(v, ans(), { kP: 'kP', kD: 'kD' }),
          actions: [F.useGains(ctx, { kP: 'kPh', kD: 'kDh' })],
          solution: () => { const a = ans(); return [{ tex: `\\Delta^d = (s+0.2)(s+0.3) = s^2 + 0.5s + 0.06 \\Rightarrow k_P = ${tex(m().M)}\\cdot 0.06 = ${tex(a.kP)},\\; k_D = ${tex(m().M)}\\cdot 0.5 = ${tex(a.kD)}` }]; },
        },
        {
          id: 'd', title: '(d) Simulate',
          html: 'Click <em>Use my gains</em>, then compare the × with the dashed rings and look at the step response. The dashed trace is the linear design model.',
          check: () => {
            if (ctx.S.mode !== 'work') return { ok: false, msg: 'Switch to Work mode so the simulation uses your gains.' };
            const cl = F.rootsOf(F.lonPoly(m(), ctx.gains));
            return M.polesMatch(cl, prob.desiredPoles, 0.02, 0.005) ? { ok: true, msg: 'The closed loop has the target poles.' } : { ok: false, msg: `Current poles: ${cl.map((q) => fmtPole(q)).join(', ')}.` };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 8 --
  // Simulated peak rotor demand for F.8(f), as a fraction of the room left.
  function satRun(ctx, k) {
    const g = F.designSLC(ctx.pModel, k);
    const fake = { ...ctx, gains: g, st: { ...ctx.st, deriv: 'state', lat: 'on', comp: 'eq' } };
    // the reference answer, so always with the workbench's F_e (never shown in Work mode)
    const res = F.simulate(fake, satCommon(ctx), null, F.makePID(fake, { fe: ctx.sys.models(ctx.pModel).Fe }));
    const [lo, hi] = ctx.sys.uLimit(ctx.pModel)[0];
    let sat = false;
    for (const ud of res.uDemandAll) for (let i = 0; i < ud.length; i++) if (ud[i] > hi + 1e-9 || ud[i] < lo - 1e-9) { sat = true; break; }
    return sat;
  }
  function satCommon(ctx) {
    const S = ctx.S, v = ctx.chan;
    const gens = v.refs.map((rf, i) => {
      const c = i === 0 ? S.sim : S.sim.refs[i - 1];
      const off = i === 0 ? ctx.st.hOff || 0 : ctx.st.zOff || 0;
      const g = WB.sim.makeReference({ type: c.type, amplitude: c.amplitude, frequency: c.frequency, tStep: c.tStep });
      return (t) => (t < c.tStep ? 0 : g(t) + off);
    });
    const init = Object.fromEntries(v.initial.map((q, i) => [q.key, (i === 0 ? S.sim.y0 : S.sim.init[q.key] ?? 0) / (q.scale || 1)]));
    return { reference: (t) => gens.map((gg) => gg(t)), noise: null, x0: ctx.sys.x0(init), Ts: S.sim.Ts, tEnd: S.sim.tEnd };
  }
  // Reference answer to F.8(f): bisect t_r,h with the book's t_r,z, then t_r,z.
  let fastestMemo = { key: null, val: null };
  function fastestNoSat(ctx) {
    const key = JSON.stringify([ctx.pTrue, ctx.S.sim, ctx.st.zOff, ctx.st.hOff]);
    if (fastestMemo.key === key) return fastestMemo.val;
    const val = fastestNoSatRaw(ctx);
    fastestMemo = { key, val };
    return val;
  }
  function fastestNoSatRaw(ctx) {
    const base = f8Knobs(ctx.sys);
    const bis = (fn, lo, hi) => { for (let i = 0; i < 18; i++) { const mid = Math.sqrt(lo * hi); if (fn(mid)) lo = mid; else hi = mid; } return hi; };
    const trh = bis((tr) => satRun(ctx, { ...base, trh: tr }), 0.2, 20);
    const Msep = bis((Mm) => satRun(ctx, { ...base, trh, Msep: Mm }), 0.3, 40);
    const MAlone = bis((Mm) => satRun(ctx, { ...base, Msep: Mm }), 0.3, 40);
    return { trh, trz: Msep * base.trth, trzAlone: MAlone * base.trth };
  }

  F.register({
    id: 'ch8', num: 8, tab: 'Ch 8', title: 'Second-order design & successive loop closure', pages: 'pp. 107–136, F.8 pp. 397–398',
    controller: (ctx, o) => F.makePID(ctx, o),
    defaults(sys) { return { comp: 'eq', lat: 'on', view: 'lat', zOff: 3, hOff: 0, w: { ...F.W0 }, k: f8Knobs(sys) }; },
    simDefaults(sys) { return sys.problems.ch8.sim; },
    gains(ctx) {
      if (ctx.S.mode === 'work') return { ...pick(ctx.st.w, ALL), kIh: 0, kIz: 0 };
      return { ...F.designSLC(ctx.pModel, ctx.st.k), kIh: 0, kIz: 0 };
    },
    targets(ctx) { return { tr: ctx.st.k.trh, zeta: ctx.st.k.zetah }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller', 'F.8 p. 397');
      F.viewControl(sec, ctx);
      if (ctx.S.mode === 'work') {
        workSections(parent, ctx);
        knobSections(parent, ctx, { title: 'Specs' });
        parent.append(el('p', { class: 'muted small', text: 'The target poles for these specs appear as dashed rings once you solve F.8(a), (b) and (d).' }));
      } else {
        knobSections(parent, ctx);
        const ro = section(parent, 'Designed gains', 'p. 100');
        F.readout(ro, ctx, [{ key: 'kPh', label: 'kP,h' }, { key: 'kDh', label: 'kD,h' }, { key: 'kPth', label: 'kP,θ' }, { key: 'kDth', label: 'kD,θ' }, { key: 'kPz', label: 'kP,z' }, { key: 'kDz', label: 'kD,z' }]);
        ro.append(el('p', { class: 'muted small', text: 'Drag a pole: altitude → t_r,h and ζ_h; inner → t_r,θ and ζ_θ; outer → M and ζ_z.' }));
      }
      separationSection(parent, ctx);
    },
    splane(ctx) {
      const work = ctx.S.mode === 'work';
      return F.pidSplane(ctx, ctx.gains, { draggable: !work, targets: work ? solvedTargets(ctx) : null });
    },
    onPoleDrag: F.pidDrag,
    math(ctx) { return [lonCard(ctx, ctx.gains), ...slcCards(ctx, ctx.gains), ...specCards(ctx, ctx.st.k)]; },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch8;
      const p = () => ctx.pModel;
      const ref = () => F.refF8(p());
      const m = () => ctx.sys.models(p());
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Altitude: t<sub>r</sub> = 8 s, ζ = 0.707', html: 'Δ<sup>d</sup><sub>cl</sub>(s) = s² + α<sub>1</sub>s + α<sub>0</sub>',
          inputs: { wn: 'ω<sub>n</sub>', a1: 'α<sub>1</sub>', a0: 'α<sub>0</sub>', p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>', kP: 'k<sub>P<sub>h</sub></sub>', kD: 'k<sub>D<sub>h</sub></sub>' },
          check: (v) => {
            const r = ref(), g = [M.parseComplex(v.p1), M.parseComplex(v.p2)];
            const nums = PD().checkNumbers({ wn: v.wn, a1: v.a1, a0: v.a0, kP: v.kP, kD: v.kD }, { wn: r.wnh, a1: 2 * prob.zetah * r.wnh, a0: r.wnh ** 2, kP: r.kPh, kD: r.kDh }, { wn: 'ωn', a1: 'α1', a0: 'α0', kP: 'kP,h', kD: 'kD,h' });
            if (!nums.ok) return nums;
            if (!g[0] || !g[1]) return { ok: false, msg: 'Enter both poles (complex values like -1+2j are fine).' };
            return M.polesMatch(g, F.polesWZ(r.wnh, prob.zetah)) ? nums : { ok: false, msg: 'The poles are the roots of Δ^d_cl(s).' };
          },
          actions: [F.useGains(ctx, { kP: 'kPh', kD: 'kDh' })],
          solution: () => { const r = ref(); return [
            { tex: `\\omega_n = \\frac{2.2}{8} = ${tex(r.wnh)},\\quad \\Delta^d_{cl} = s^2 + ${tex(2 * prob.zetah * r.wnh)}s + ${tex(r.wnh ** 2)},\\quad p = ${F.polesWZ(r.wnh, prob.zetah).map((q) => texPole(q, 4)).join(',\\;')}` },
            { tex: `k_{P_h} = (m_c+2m_r)\\omega_n^2 = ${tex(r.kPh)},\\quad k_{D_h} = (m_c+2m_r)\\,2\\zeta\\omega_n = ${tex(r.kDh)}` },
            { html: 'Eq. 8.5 (p. 113) and the PD pole placement of Ch 7 (p. 100).' },
          ]; },
        },
        {
          id: 'b', title: '(b) Inner loop: t<sub>r<sub>θ</sub></sub> = 0.8 s, ζ<sub>θ</sub> = 0.707',
          inputs: { kP: 'k<sub>P<sub>θ</sub></sub>', kD: 'k<sub>D<sub>θ</sub></sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { kP: r.kPth, kD: r.kDth }, { kP: 'kP,θ', kD: 'kD,θ' }); },
          actions: [F.useGains(ctx, { kP: 'kPth', kD: 'kDth' })],
          solution: () => { const r = ref(); return [
            { tex: `\\frac{\\Theta}{\\Theta_d} = \\frac{k_{P_\\theta}/J}{s^2 + \\frac{k_{D_\\theta}}{J}s + \\frac{k_{P_\\theta}}{J}},\\quad J = J_c + 2m_rd^2 = ${tex(m().J)}` },
            { tex: `\\omega_{n_\\theta} = \\frac{2.2}{0.8} = ${tex(r.wnt)},\\quad k_{P_\\theta} = J\\omega_{n_\\theta}^2 = ${tex(r.kPth)},\\quad k_{D_\\theta} = 2\\zeta_\\theta\\omega_{n_\\theta}J = ${tex(r.kDth)}` },
          ]; },
        },
        {
          id: 'c', title: '(c) Inner-loop DC gain', inputs: { k: 'k<sub>DC<sub>θ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { k: 1 }, { k: 'kDC,θ' }),
          solution: () => [{ tex: 'k_{DC_\\theta} = \\lim_{s\\to0}\\frac{k_{P_\\theta}/J}{s^2 + \\frac{k_{D_\\theta}}{J}s + \\frac{k_{P_\\theta}}{J}} = 1' }, { html: 'The plant is a double integrator, so the inner loop tracks a constant θ<sub>d</sub> exactly (compare B.8, where k<sub>DC</sub> = 1.89, p. 125).' }],
        },
        {
          id: 'd', title: '(d) Outer loop: t<sub>r<sub>z</sub></sub> = 10 t<sub>r<sub>θ</sub></sub>, ζ<sub>z</sub> = 0.707',
          inputs: { kP: 'k<sub>P<sub>z</sub></sub>', kD: 'k<sub>D<sub>z</sub></sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { kP: r.kPz, kD: r.kDz }, { kP: 'kP,z', kD: 'kD,z' }); },
          actions: [F.useGains(ctx, { kP: 'kPz', kD: 'kDz' })],
          solution: () => { const r = ref(); return [
            { tex: '\\frac{Z}{Z_r} = \\frac{-gk_{DC_\\theta}k_{P_z}}{s^2 + \\big(\\frac{\\mu}{m_c+2m_r} - gk_{DC_\\theta}k_{D_z}\\big)s - gk_{DC_\\theta}k_{P_z}}' },
            { tex: `t_{r_z} = 8\\,\\text{s},\\; \\omega_{n_z} = ${tex(r.wnz)}:\\quad k_{P_z} = -\\frac{\\omega_{n_z}^2}{g} = ${tex(r.kPz)},\\quad k_{D_z} = \\frac{\\mu/(m_c+2m_r) - 2\\zeta_z\\omega_{n_z}}{g} = ${tex(r.kDz)}` },
          ]; },
        },
        {
          id: 'e', title: '(e) Implement with z<sub>r</sub> = 3 ± 2.5 m at 0.08 Hz',
          html: 'Use your (b) and (d) gains (Work mode). The reference is set up already: square wave of amplitude 2.5 at 0.08 Hz with a 3 m offset (right panel). Passes when your lateral gains match the design within 1%.',
          check: () => {
            if (ctx.S.mode !== 'work') return { ok: false, msg: 'Switch to Work mode.' };
            const r = ref(), w = ctx.st.w;
            const ok = ['kPth', 'kDth', 'kPz', 'kDz'].every((k) => M.close(w[k], r[k]));
            const zm = F.zMetrics(ctx);
            return ok ? { ok: true, msg: `${isFinite(zm.tr) ? `z rise time ${fmt(zm.tr, 3)} s, overshoot ${fmt(zm.os, 3)}%.` : 'z does not reach 90% of the step before the reference switches.'} The 6.25 s half period is shorter than the z response.` } : { ok: false, msg: 'Your lateral gains do not match (b) and (d) yet.' };
          },
        },
        {
          id: 'f', title: '(f) Fastest t<sub>r,h</sub> and t<sub>r,z</sub> without saturation',
          inputs: { trh: 't<sub>r,h</sub> [s]', trz: 't<sub>r,z</sub> [s]' },
          html: 'Checked by simulation with the current references and initial conditions (default: h<sub>r</sub> a 2 m step, z<sub>r</sub> = 3 ± 2.5 m), ζ = 0.707 and t<sub>r,θ</sub> = 0.8 s. Passes when neither rotor saturates, but 10% faster in either loop does.',
          check: (v) => {
            const trh = PD().num(v.trh), trz = PD().num(v.trz);
            if (!(trh > 0) || !(trz > 0)) return { ok: false, msg: 'Enter two positive rise times.' };
            const base = f8Knobs(ctx.sys), mk = (a, b) => ({ ...base, trh: a, Msep: b / base.trth });
            if (satRun(ctx, mk(trh, trz))) return { ok: false, msg: 'A rotor saturates. Slow down.' };
            const fh = satRun(ctx, mk(0.9 * trh, trz)), fz = satRun(ctx, mk(trh, 0.9 * trz));
            if (fh && fz) return { ok: true, msg: 'No saturation, and 10% faster in either loop saturates.' };
            return { ok: false, msg: `No saturation. You can still go faster in ${!fh && !fz ? 'both loops' : !fh ? 'the altitude loop' : 'the z loop'}.` };
          },
          actions: [{
            label: 'Try it', run: (v) => {
              const trh = PD().num(v.trh), trz = PD().num(v.trz);
              if (!(trh > 0) || !(trz > 0)) return { ok: false, msg: 'Enter two positive rise times.' };
              ctx.app.setMode('explore');
              Object.assign(ctx.st.k, { trh, Msep: trz / ctx.st.k.trth });
              ctx.update(); return null;
            },
          }],
          solution: () => {
            const r = fastestNoSat(ctx);
            return [
              { html: `Bisection on the simulation with the current references. Altitude alone (t<sub>r,z</sub> = 8 s): t<sub>r,h</sub> ≈ ${fmt(r.trh, 3)} s. Lateral alone (t<sub>r,h</sub> = 8 s): t<sub>r,z</sub> ≈ ${fmt(r.trzAlone, 3)} s (M ≈ ${fmt(r.trzAlone / 0.8, 3)}).` },
              { html: `Both steps start at t = 0, so the loops share the rotor headroom and you cannot have both limits at once. Any pair on the trade-off curve passes, for example t<sub>r,h</sub> ≈ ${fmt(r.trh, 3)} s with t<sub>r,z</sub> ≈ ${fmt(r.trz, 3)} s: with the altitude loop at its limit, the book's lateral design is already the fastest that fits.` },
              { tex: '\\text{Altitude estimate (Eq. 8.8): } k_{P_h} \\le \\frac{2(f_{max} - F_e/2)}{e_{max}} \\Rightarrow \\omega_{n_h} \\le \\sqrt{\\frac{2f_{max} - F_e}{(m_c+2m_r)\\,e_{max}}}' },
              { html: 'The altitude limit comes from the room above hover (f<sub>max</sub> − F<sub>e</sub>/2 = 2.64 N per rotor). The lateral loop barely uses the rotors until the outer loop is almost as fast as the inner loop, so saturation alone would let you push M near 1, which breaks the separation assumption. Watch the exact coupled poles (open circles) as M drops.' },
            ];
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 9 --
  // E(s)/R(s) and Y(s)/D_in(s) for b0/(s² + a1 s + a0) with C = kP + kI/s + kD s.
  //   arch 'error': unity feedback, C in the forward path (Fig. 9-1, the book's analysis)
  //   arch 'output': derivative on the output (Fig. 7-2, as implemented)
  function loopTFs(mdl, kP, kI, kD, arch) {
    const D = [1, mdl.a1, mdl.a0];
    const Ds = kI ? L.conv(D, [1, 0]) : D;
    const N = kI ? [kD * mdl.b0, kP * mdl.b0, kI * mdl.b0] : [kD * mdl.b0, kP * mdl.b0];
    const den = L.polyAdd(Ds, N);
    const yr = arch === 'error' ? N : (kI ? [kP * mdl.b0, kI * mdl.b0] : [kP * mdl.b0]);
    const eNum = L.polyAdd(den, yr.map((v) => -v));
    const dNum = kI ? [mdl.b0, 0] : [mdl.b0];
    return { den, eNum, dNum };
  }
  // lim_{s→0} s · num/den · 1/s^k  (k = 1 step, 2 ramp, 3 parabola)
  function ssError(num, den, k) {
    let z = 0;
    const n = L.trimLeading(num.map((v) => (Math.abs(v) < 1e-14 ? 0 : v)));
    for (let i = n.length - 1; i > 0 && Math.abs(n[i]) < 1e-14; i--) z++;
    if (n.length === 1 && Math.abs(n[0]) < 1e-14) return 0;
    const need = k - 1;
    if (z > need) return 0;
    if (z < need) return Infinity;
    return n[n.length - 1 - z] / den[den.length - 1];
  }
  function typeOf(num) {
    let z = 0;
    for (let i = num.length - 1; i > 0 && Math.abs(num[i]) < 1e-14; i--) z++;
    return z;
  }
  function loopAnalysis(mdl, kP, kI, kD, arch) {
    const t = loopTFs(mdl, kP, kI, kD, arch);
    return {
      type: typeOf(t.eNum), step: ssError(t.eNum, t.den, 1), ramp: ssError(t.eNum, t.den, 2), parab: ssError(t.eNum, t.den, 3),
      dType: typeOf(t.dNum), dStep: ssError(t.dNum, t.den, 1), dRamp: ssError(t.dNum, t.den, 2),
    };
  }
  const efmt = (v) => (isFinite(v) ? fmt(v, 3) : '∞');

  function shapedRef(ctx, base) {
    const withOff = F.reference(ctx, base);
    const shape = ctx.st.input;
    if (shape === 'step') return withOff;
    const S = ctx.S;
    return (t) => {
      const r = withOff(t);
      const cfg = [S.sim, S.sim.refs[0]];
      [0, 1].forEach((i) => {
        const t0 = cfg[i].tStep, A = cfg[i].amplitude;
        if (t >= t0) r[i] = (i === 0 ? ctx.st.hOff || 0 : ctx.st.zOff || 0) + (shape === 'ramp' ? A * (t - t0) : 0.5 * A * (t - t0) ** 2);
      });
      return r;
    };
  }

  // Random gains for the F.9 Python answers (signs as in F.8 / F.10).
  const G9 = {
    lon: { kP: { label: 'kP', lo: 0.05, hi: 2 }, kD: { label: 'kD', lo: 0.1, hi: 3 } },
    inner: { kP: { label: 'kP', lo: 0.05, hi: 2 }, kD: { label: 'kD', lo: 0.02, hi: 1 } },
    outer: { kP: { label: 'kP', lo: -0.05, hi: -0.001 }, kI: { label: 'kI', lo: -0.003, hi: -0.0001 }, kD: { label: 'kD', lo: -0.1, hi: -0.005 } },
  };
  // Outer z plant with the inner loop replaced by its DC gain (1: the θ plant is a double integrator).
  const outerOf = (q) => { const m = m0(q); return { b0: m.outer.b0, a1: m.outer.a1 }; };

  F.register({
    id: 'ch9', num: 9, tab: 'Ch 9', title: 'System type & integrators', pages: 'pp. 137–154, F.9 p. 398',
    controller: (ctx, o) => F.makePID(ctx, o),
    defaults(sys) { return { comp: 'eq', lat: 'on', view: 'lat', zOff: 0, hOff: 0, input: 'step', w: { ...F.W0 }, k: { ...f8Knobs(sys), kIh: 0, kIz: 0 } }; },
    simDefaults(sys) { return { ...sys.problems.ch9.sim, refs: [{ type: 'step', amplitude: 2 }] }; },
    reference: shapedRef,
    gains(ctx) { return ctx.S.mode === 'work' ? pick(ctx.st.w, ALL) : F.designSLC(ctx.pModel, ctx.st.k); },
    analysis(ctx, g = ctx.gains) {
      const m = ctx.sys.models(ctx.pModel);
      const outer = { b0: m.outer.b0 * F.kDCof(m, g), a1: m.outer.a1, a0: 0 };
      const a = (mdl, kP, kI, kD) => ({ book: loopAnalysis(mdl, kP, kI, kD, 'error'), impl: loopAnalysis(mdl, kP, kI, kD, 'output') });
      return {
        lon: a(m.lon, g.kPh, g.kIh, g.kDh), inner: a(m.inner, g.kPth, 0, g.kDth), outer: a(outer, g.kPz, g.kIz, g.kDz),
        m, outerMdl: outer,
      };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Nested PID', 'F.9 p. 398');
      segmented(sec, {
        label: 'Reference shape for h<sub>r</sub> and z<sub>r</sub> (amplitude = size, slope, or curvature r̈)',
        options: [{ value: 'step', label: 'step' }, { value: 'ramp', label: 'ramp' }, { value: 'parabola', label: 'parabola' }],
        ...bind(ctx, 'input'),
      });
      F.viewControl(sec, ctx);
      if (ctx.S.mode === 'work') workSections(parent, ctx, { kI: true });
      else {
        knobSections(parent, ctx);
        const ki = section(parent, 'Integrators', 'p. 142');
        slider(ki, { label: 'k<sub>I<sub>h</sub></sub>', min: 0, max: 0.05, step: 0.0001, sig: 3, ...bind(ctx, 'kIh', () => ctx.st.k) });
        slider(ki, { label: 'k<sub>I<sub>z</sub></sub>', min: -0.003, max: 0, step: 0.00001, sig: 3, ...bind(ctx, 'kIz', () => ctx.st.k) });
      }
      const ty = section(parent, 'System type (current gains)', 'p. 141 · Table 9-1');
      const box = el('div', { class: 'metrics' });
      ty.append(box);
      WB.ui.addRefresher(() => {
        const A = this.analysis(ctx);
        const res = ctx.app.result();
        const n = res ? res.t.length - 1 : 0;
        const show = WB.ui.shown(ctx, 'F:ch9type');
        const rows = [];
        if (show) {
          for (const [key, name] of [['lon', 'altitude'], ['inner', 'inner θ'], ['outer', 'outer z']]) {
            const b = A[key].book, im = A[key].impl;
            rows.push(F.metricRow(`${name}: type (book / D on y)`, `${b.type} / ${im.type}`));
            rows.push(F.metricRow(`${name}: e ramp, parab (book)`, `${efmt(b.ramp)}, ${efmt(b.parab)}`));
            rows.push(F.metricRow(`${name}: e ramp, parab (D on y)`, `${efmt(im.ramp)}, ${efmt(im.parab)}`));
            rows.push(F.metricRow(`${name}: d_in type, e per unit step d`, `${b.dType}, ${efmt(b.dStep)}`));
          }
        } else rows.push(WB.ui.revealButton(ctx, 'F:ch9type', 'Reveal the predicted types and errors'));
        if (res) {
          rows.push(F.metricRow('simulated h_r − h at t_end', `${fmt(res.rAll[0][n] - res.yAll[1][n], 3)} m`));
          rows.push(F.metricRow('simulated z_r − z at t_end', `${fmt(res.rAll[1][n] - res.yAll[0][n], 3)} m`));
        }
        box.replaceChildren(...rows);
      });
    },
    splane(ctx) { return F.pidSplane(ctx, ctx.gains, {}); },
    math(ctx) {
      const A = this.analysis(ctx), g = ctx.gains;
      return [
        { title: 'System type (unity feedback)', page: 'p. 141 · Table 9-1',
          theory: 'E = \\frac{1}{1 + PC}R,\\quad M_p = \\lim_{s\\to0}PC,\\; M_v = \\lim_{s\\to0}sPC,\\; M_a = \\lim_{s\\to0}s^2PC' },
        { title: 'Altitude loop', page: 'F.9(a) p. 398', answers: 'F.9/a',
          theory: 'PC = P_{lon}(s)\\,C(s)', symbolic: 'PC = \\frac{k_{D_h}s^2 + k_{P_h}s + k_{I_h}}{(m_c+2m_r)\\,s^3}\\;(\\text{PID}),\\quad \\frac{k_{D_h}s + k_{P_h}}{(m_c+2m_r)s^2}\\;(\\text{PD})',
          numbers: `\\text{PD: type 2},\\; e_{parab} = \\frac{m_c+2m_r}{k_{P_h}} = ${tex(A.m.M / g.kPh)}\\quad \\text{PID: type 3}\\quad d_{in}: \\text{PD type 0 } (e = 1/k_{P_h} = ${tex(1 / g.kPh)}),\\; \\text{PID type 1}` },
        { title: 'Inner θ loop (PD)', page: 'F.9(b) p. 398', answers: 'F.9/b',
          theory: 'PC = P_{\\theta}(s)\\,C(s)', symbolic: 'PC = \\frac{k_{D_\\theta}s + k_{P_\\theta}}{J s^2}',
          numbers: `\\text{type 2},\\; e_{parab} = \\frac{J}{k_{P_\\theta}} = ${tex(A.m.J / g.kPth)},\\quad d_{in}: \\text{type 0},\\; e = \\frac{1}{k_{P_\\theta}} = ${tex(1 / g.kPth)}` },
        { title: 'Outer z loop', page: 'F.9(c) p. 398', answers: 'F.9/c',
          theory: 'PC = k_{DC_\\theta}P_z(s)\\,C(s)', symbolic: 'PC = \\frac{-g k_{DC_\\theta}(k_{D_z}s + k_{P_z})}{s(s + \\mu/(m_c+2m_r))}\\;(\\text{PD})',
          numbers: `\\text{PD: type 1},\\; M_v = \\frac{-g k_{P_z}}{\\mu/(m_c+2m_r)} = ${tex(1 / A.outer.book.ramp)},\\; e_{ramp} = ${tex(A.outer.book.ramp)}\\quad \\text{PID: type 2},\\; e_{parab} = \\frac{\\mu/(m_c+2m_r)}{-g k_{I_z}}\\quad d_{in}: \\text{PD type 0, PID type 1}` },
        { title: 'Derivative on the output changes the tracking type', page: 'Fig. 7-2 p. 101 vs. Fig. 9-1 p. 138',
          theory: '\\frac{E}{R} = \\frac{s^2 + b_0k_Ds}{s^2 + b_0k_Ds + b_0k_P}\\;(\\text{PD, }D\\text{ on }y)\\;\\Rightarrow\\; e_{ramp} = \\frac{k_D}{k_P} \\ne 0',
          note: 'Table 9-1 assumes C = k_P + k_D s acts on the error. The book\'s own loops differentiate y, which only removes the zero from the numerator: the input-disturbance type is unchanged, but each loop loses one type for tracking. The readout shows both.' },
      ];
    },
    buildProblem(parent, ctx) {
      const A = () => this.analysis(ctx);
      const g = () => ctx.gains;
      const m = () => ctx.sys.models(ctx.pModel);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch9, [
        {
          id: 'a', title: '(a) Longitudinal controller: PD, then PID',
          html: 'Book convention (Table 9-1, C acting on the error). Set the reference-tracking and input-disturbance system types, and write the PD errors for unit inputs (step 1/s, ramp 1/s², parabola 1/s³) and the error magnitude per unit step d<sub>in</sub> [m/N] as functions of the gains. Return <code>np.inf</code> for an unbounded error. The check calls them at random gains.',
          code: F.pyPart(ctx, {
            args: G9.lon,
            items: [
              { var: 'type_pd', truth: () => 2 }, { var: 'type_pid', truth: () => 3 },
              { var: 'dist_type_pd', truth: () => 0 }, { var: 'dist_type_pid', truth: () => 1 },
              { fn: 'e_step', args: ['kP', 'kD'], truth: () => 0 },
              { fn: 'e_ramp', args: ['kP', 'kD'], truth: () => 0 },
              { fn: 'e_parab', args: ['kP', 'kD'], truth: (q, a) => m0(q).M / a.kP },
              { fn: 'e_dist', args: ['kP', 'kD'], truth: (q, a) => 1 / a.kP },
            ],
            explain: (it, f) => (it.fn === 'e_ramp' && f.e.a && M.close(Number(f.e.got), f.e.a.kD / f.e.a.kP, 1e-4, 1e-9) ? 'That is the implemented loop (derivative on h); Table 9-1 assumes C acts on the error.' : ''),
          }, 'type_pd = ...\ntype_pid = ...\ndist_type_pd = ...\ndist_type_pid = ...\n\ndef e_step(kP, kD):\n    return ...\n\ndef e_ramp(kP, kD):\n    return ...\n\ndef e_parab(kP, kD):\n    return ...\n\ndef e_dist(kP, kD):\n    return ...\n'),
          solution: () => [
            { tex: `PC_{PD} = \\frac{k_{D_h}s + k_{P_h}}{(m_c+2m_r)s^2}: \\text{type 2},\; M_a = \\frac{k_{P_h}}{m_c+2m_r},\; e_{parab} = ${tex(m().M / g().kPh)}\;\\text{(current gains)}` },
            { tex: '\\text{PID adds } 1/s: \\text{type 3 (step, ramp, parabola errors all } 0)' },
            { tex: `\\frac{P}{1+PC}\\Big|_{s\\to0} = \\frac{1}{k_{P_h}} = ${tex(1 / g().kPh)}\;(\\text{type 0}),\\quad \\text{PID: } \\frac{s}{k_{I_h}} \\to 0\;(\\text{type 1})` },
            { html: 'pp. 141–145. With the derivative on h (as implemented), tracking drops one type: PD gives e<sub>ramp</sub> = k<sub>D</sub>/k<sub>P</sub>; the readout on the right shows both.' },
            { code: 'type_pd = 2\ntype_pid = 3\ndist_type_pd = 0\ndist_type_pid = 1\n\ndef e_step(kP, kD):\n    return 0.0\n\ndef e_ramp(kP, kD):\n    return 0.0\n\ndef e_parab(kP, kD):\n    return (P.mc + 2 * P.mr) / kP   # 1/M_a\n\ndef e_dist(kP, kD):\n    return 1 / kP' },
          ],
        },
        {
          id: 'b', title: '(b) Inner loop of the lateral controller, PD',
          html: 'Same as (a) for unit θ̃<sup>d</sup> inputs: the system types, the errors [rad] and the error per unit step d [rad/N·m] as functions of k<sub>P<sub>θ</sub></sub>, k<sub>D<sub>θ</sub></sub>.',
          code: F.pyPart(ctx, {
            args: G9.inner,
            items: [
              { var: 'system_type', truth: () => 2 }, { var: 'dist_type', truth: () => 0 },
              { fn: 'e_step', args: ['kP', 'kD'], truth: () => 0 },
              { fn: 'e_ramp', args: ['kP', 'kD'], truth: () => 0 },
              { fn: 'e_parab', args: ['kP', 'kD'], truth: (q, a) => m0(q).J / a.kP },
              { fn: 'e_dist', args: ['kP', 'kD'], truth: (q, a) => 1 / a.kP },
            ],
          }, 'system_type = ...\ndist_type = ...\n\ndef e_step(kP, kD):\n    return ...\n\ndef e_ramp(kP, kD):\n    return ...\n\ndef e_parab(kP, kD):\n    return ...\n\ndef e_dist(kP, kD):\n    return ...\n'),
          solution: () => [
            { tex: `PC = \\frac{k_{D_\\theta}s + k_{P_\\theta}}{Js^2}: \\text{type 2},\; e_{parab} = \\frac{J}{k_{P_\\theta}} = ${tex(m().J / g().kPth)};\\quad d_{in}: \\text{type 0},\; e = \\frac{1}{k_{P_\\theta}} = ${tex(1 / g().kPth)}\;\\text{(current gains)}` },
            { code: 'system_type = 2\ndist_type = 0\n\ndef e_step(kP, kD):\n    return 0.0\n\ndef e_ramp(kP, kD):\n    return 0.0\n\ndef e_parab(kP, kD):\n    J = P.Jc + 2 * P.mr * P.d**2\n    return J / kP\n\ndef e_dist(kP, kD):\n    return 1 / kP' },
          ],
        },
        {
          id: 'c', title: '(c) Outer loop of the lateral controller: PD, then PID',
          html: 'Inner loop replaced by its DC gain. Set the system types, and write e<sub>step</sub>, e<sub>ramp</sub> for PD and e<sub>parab</sub> for PID [m] as functions of the outer gains (k<sub>P</sub>, k<sub>D</sub>, k<sub>I</sub> &lt; 0, as in F.8 and F.10).',
          code: F.pyPart(ctx, {
            args: G9.outer,
            items: [
              { var: 'type_pd', truth: () => 1 }, { var: 'type_pid', truth: () => 2 },
              { var: 'dist_type_pd', truth: () => 0 }, { var: 'dist_type_pid', truth: () => 1 },
              { fn: 'e_step', args: ['kP', 'kD'], truth: () => 0 },
              { fn: 'e_ramp', args: ['kP', 'kD'], truth: (q, a) => outerOf(q).a1 / (outerOf(q).b0 * a.kP) },
              { fn: 'e_parab', args: ['kP', 'kI', 'kD'], truth: (q, a) => outerOf(q).a1 / (outerOf(q).b0 * a.kI) },
            ],
          }, 'type_pd = ...\ntype_pid = ...\ndist_type_pd = ...\ndist_type_pid = ...\n\ndef e_step(kP, kD):\n    return ...\n\ndef e_ramp(kP, kD):\n    return ...\n\ndef e_parab(kP, kI, kD):\n    return ...\n'),
          solution: () => {
            const o = A().outerMdl, Mv = o.b0 * g().kPz / o.a1, Ma = g().kIz ? o.b0 * g().kIz / o.a1 : NaN;
            return [
              { tex: `PC = \\frac{-gk_{DC_\\theta}(k_{D_z}s + k_{P_z})}{s(s + \\mu/M)}: \\text{type 1},\; M_v = \\frac{-g k_{P_z}M}{\\mu} = ${tex(Mv)},\; e_{ramp} = ${tex(1 / Mv)}\;\\text{(current gains)}` },
              { tex: `\\text{PID: type 2},\; M_a = \\frac{-g k_{I_z}M}{\\mu} = ${tex(Ma)},\; e_{parab} = ${tex(1 / Ma)};\\quad d_{in}: \\text{PD type 0, PID type 1}` },
              { code: 'type_pd = 1\ntype_pid = 2\ndist_type_pd = 0\ndist_type_pid = 1\n\na = P.mu / (P.mc + 2 * P.mr)\nb0 = -P.g          # times k_DC = 1\n\ndef e_step(kP, kD):\n    return 0.0\n\ndef e_ramp(kP, kD):\n    return a / (b0 * kP)    # 1/M_v\n\ndef e_parab(kP, kI, kD):\n    return a / (b0 * kI)    # 1/M_a' },
            ];
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------- Appendix P.6 --
  // Evans form 1 + K·n(s)/d(s) = 0 for the altitude (K = k_Ih) or outer loop (K = −k_Iz).
  function evans(ctx, loop, g) {
    const m = ctx.sys.models(ctx.pModel);
    if (loop === 'lon') {
      const b = m.lon.b0;
      const den = [1, b * g.kDh, b * g.kPh, 0];
      return { den, num: [b], kCrit: den[1] * den[2] / b, K: g.kIh, pdPoly: [1, b * g.kDh, b * g.kPh] };
    }
    const b = m.outer.b0 * F.kDCof(m, g), a = m.outer.a1;
    const den = [1, a + b * g.kDz, b * g.kPz, 0];
    return { den, num: [-b], kCrit: den[1] * den[2] / (-b), K: -g.kIz, pdPoly: [1, a + b * g.kDz, b * g.kPz] };
  }
  // "Does not move the other poles much": pair within 10% in |p|, new real pole slower.
  function kIOk(ev, K) {
    const pd = L.roots(ev.pdPoly);
    const cl = L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, K)));
    const cpx = cl.filter((q) => Math.abs(q.im) > 1e-7), real = cl.filter((q) => Math.abs(q.im) <= 1e-7);
    if (cpx.length !== 2 || real.length !== 1 || !(K > 0)) return { ok: false, cl, ratio: NaN };
    const ratio = Math.hypot(cpx[0].re, cpx[0].im) / Math.hypot(pd[0].re, pd[0].im);
    return { ok: Math.abs(ratio - 1) < 0.1 && Math.abs(real[0].re) < Math.abs(pd[0].re) && real[0].re < 0, cl, ratio, real: real[0] };
  }
  function kIRange(ev) {
    let best = 0;
    for (let i = 1; i <= 400; i++) { const K = ev.kCrit * i / 400; if (kIOk(ev, K).ok) best = K; }
    return best;
  }

  // F.P.6 truths: 1 + k_I L(s) = 0 with the derivative on the output.
  //   altitude: Δ = s³ + b₀k_D s² + b₀k_P s + b₀k_I, b₀ = 1/(m_c + 2m_r)
  //   outer:    Δ = s³ + (μ/M + b k_D)s² + b k_P s + b k_I, b = −g·k_DC,θ with k_DC,θ = 1
  //             (the inner loop's DC gain for the double-integrator roll dynamics, F.8(c))
  const evansTruth = (loop) => (p, a) => {
    const cxx = WB.py.cx, m = m0(p);
    if (loop === 'lon') { const b = m.lon.b0; return cxx.div(b, cxx.poly([1, b * a.kD, b * a.kP, 0], a.s)); }
    const b = m.outer.b0;
    return cxx.div(b, cxx.poly([1, m.outer.a1 + b * a.kD, b * a.kP, 0], a.s));
  };
  // Work-mode k_I sliders start at 0 (not an answer); Explore keeps its own values.
  const kIKey = (ctx, k) => (ctx.S.mode === 'work' ? k : `${k}X`);

  F.register({
    id: 'p6', num: 10.5, tab: 'App. P.6', short: 'P.6', title: 'Root locus vs. k_I', pages: 'pp. 465–474, F.P.6 p. 398',
    controller: (ctx, o) => F.makePID(ctx, o),
    defaults(sys) { return { comp: 'eq', lat: 'on', view: 'lon', zOff: 3, hOff: 0, src: 'mine', kIh: 0, kIz: 0, kIhX: 0.003, kIzX: -0.0002, kMaxFactor: 1.5, k: f8Knobs(sys) }; },
    simDefaults(sys) { return sys.problems.p6.sim; },
    // PD gains: in Work mode your own F.8 gains (Ch 8 tab), else the F.8 design.
    pdGains(ctx) {
      if (ctx.S.mode === 'work') return { ...pick(ctx.S.ch.ch8 ? ctx.S.ch.ch8.w : F.W0, ALL) };
      return F.designSLC(ctx.pModel, ctx.st.k);
    },
    gains(ctx) { return { ...this.pdGains(ctx), kIh: ctx.st[kIKey(ctx, 'kIh')] ?? 0, kIz: ctx.st[kIKey(ctx, 'kIz')] ?? 0 }; },
    // The locus and the poles of L(s) answer the Evans-form parts: Work mode shows them once solved.
    locusShown(ctx, loop) { return F.showsAnswer(ctx, loop === 'lon' ? 'F.P.6/a' : 'F.P.6/b'); },
    buildControls(parent, ctx) {
      const sec = section(parent, 'PID: PD from F.8, then add k_I', 'p. 470 · F.P.6 p. 398');
      segmented(sec, { label: 'Root locus of', options: [{ value: 'lon', label: 'altitude vs. k<sub>I<sub>h</sub></sub>' }, { value: 'outer', label: 'outer z vs. k<sub>I<sub>z</sub></sub>' }], ...bind(ctx, 'view') });
      sec.append(el('p', { class: 'muted small', text: ctx.S.mode === 'work' ? 'Work mode uses your own F.8 gains from the Ch 8 tab. The root locus appears once you put that loop in Evans form below.' : 'Explore mode uses the F.8 design from the specs below.' }));
      slider(sec, { label: 'k<sub>I<sub>h</sub></sub>', min: 0, max: 0.05, step: 0.0001, sig: 3, get: () => ctx.st[kIKey(ctx, 'kIh')] ?? 0, set: (v) => { ctx.st[kIKey(ctx, 'kIh')] = v; ctx.update(); } });
      slider(sec, { label: 'k<sub>I<sub>z</sub></sub>', min: -0.003, max: 0, step: 0.00001, sig: 3, get: () => ctx.st[kIKey(ctx, 'kIz')] ?? 0, set: (v) => { ctx.st[kIKey(ctx, 'kIz')] = v; ctx.update(); } });
      slider(sec, { label: 'locus to', unit: '× k_I,crit', min: 0.2, max: 4, step: 0.1, sig: 2, ...bind(ctx, 'kMaxFactor') });
      sec.append(el('p', { class: 'muted small', text: 'Drag a closed-loop pole along the locus to set k_I.' }));
      F.readout(sec, ctx, [{ key: 'kPh', label: 'kP,h' }, { key: 'kDh', label: 'kD,h' }, { key: 'kPz', label: 'kP,z' }, { key: 'kDz', label: 'kD,z' }]);
      if (ctx.S.mode === 'explore') knobSections(parent, ctx, { title: 'F.8 specs' });
    },
    splane(ctx) {
      const loop = ctx.st.view === 'outer' ? 'outer' : 'lon';
      const ev = evans(ctx, loop, ctx.gains);
      const shown = this.locusShown(ctx, loop);
      const kMax = Math.max(ev.kCrit * ctx.st.kMaxFactor, ev.K * 1.2, 1e-6);
      const loci = shown ? WB.tf.rootLocus(ev.den, ev.num, kMax) : [];
      const mk = shown ? L.roots(ev.den).map((q) => ({ ...q, kind: 'ol', label: 'pole of L(s)' })) : [];
      L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, ev.K))).forEach((q, i) => mk.push({ ...q, kind: 'cl', label: `closed-loop pole at k_I = ${fmt(loop === 'lon' ? ev.K : -ev.K, 3)}`, dragId: shown ? i : undefined }));
      const fitR = Math.max(...L.roots(ev.den).map((q) => Math.hypot(q.re, q.im))) * 1.6;
      return { markers: mk, loci, fitR, minR: 0.05 };
    },
    onPoleDrag(ctx, id, re, im) {
      const loop = ctx.st.view === 'outer' ? 'outer' : 'lon';
      const ev = evans(ctx, loop, ctx.gains);
      const kMax = Math.max(ev.kCrit * ctx.st.kMaxFactor, 1e-6);
      let best = ev.K, bd = Infinity;
      for (let i = 0; i <= 400; i++) {
        const K = kMax * i / 400;
        for (const q of L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, K)))) { const d = Math.hypot(q.re - re, q.im - im); if (d < bd) { bd = d; best = K; } }
      }
      if (loop === 'lon') ctx.st[kIKey(ctx, 'kIh')] = best; else ctx.st[kIKey(ctx, 'kIz')] = -best;
      ctx.update();
    },
    math(ctx) {
      const g = ctx.gains, eh = evans(ctx, 'lon', g), ez = evans(ctx, 'outer', g);
      return [
        { title: 'Evans form', page: 'p. 466',
          theory: '\\Delta_{cl}(s) = 0 \\iff 1 + k\\,L(s) = 0 \\quad(k \\text{ is the gain that varies along the locus})' },
        { title: 'Altitude loop in Evans form', page: 'F.P.6(a) p. 398', answers: 'F.P.6/a',
          theory: '\\Delta = s^3 + \\frac{k_{D_h}}{M}s^2 + \\frac{k_{P_h}}{M}s + \\frac{k_{I_h}}{M},\\quad 1 + k_{I_h}\\frac{1/M}{s^3 + \\frac{k_{D_h}}{M}s^2 + \\frac{k_{P_h}}{M}s} = 0 \\;(M = m_c + 2m_r)',
          numbers: `L_h(s) = \\frac{${tex(eh.num[0])}}{${T.polyTex(eh.den)}},\\quad k_{I_h,crit} = \\frac{k_{D_h}k_{P_h}}{M} = ${tex(eh.kCrit)}` },
        { title: 'Outer loop in Evans form', page: 'F.P.6(b) p. 398', answers: 'F.P.6/b',
          theory: '1 + k_{I_z}\\frac{-g\\,k_{DC_\\theta}}{s^3 + (\\frac{\\mu}{M} - gk_{DC_\\theta}k_{D_z})s^2 - gk_{DC_\\theta}k_{P_z}s} = 0\\quad (k_{I_z} < 0)',
          numbers: `L_z(s) = \\frac{${tex(-ez.num[0])}}{${T.polyTex(ez.den)}},\\quad k_{I_z,crit} = ${tex(-ez.kCrit)}`,
          note: 'The locus on the right is drawn for K = −k_I,z > 0.' },
        { title: 'Where the locus crosses into the RHP', page: 'Routh–Hurwitz (not in the book)',
          theory: 's^3 + c_2 s^2 + c_1 s + c_0 \\text{ is stable iff } c_2, c_1, c_0 > 0,\\; c_2c_1 > c_0',
          note: 'The book stops at rlocus (p. 471).' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.p6;
      const parts = [];
      const KP = { lon: { label: 'kP', lo: 0.02, hi: 1 }, outer: { label: 'kP', lo: -0.05, hi: -0.001 } };
      const KD = { lon: { label: 'kD', lo: 0.1, hi: 2 }, outer: { label: 'kD', lo: -0.1, hi: -0.005 } };
      for (const [loop, letter, name] of [['lon', 'a', 'longitudinal controller'], ['outer', 'b', 'outer loop of the lateral controller']]) {
        const ev = () => evans(ctx, loop, ctx.gains);
        const fn = loop === 'lon' ? 'L_h' : 'L_z';
        const sub = loop === 'lon' ? 'h' : 'z';
        parts.push({
          id: letter, title: `(${letter}) Evans form for the ${name}`,
          html: `Add an integrator (PID, derivative on the output). Return L(s) for 1 + k<sub>I<sub>${sub}</sub></sub>L(s) = 0 in terms of that loop's PD gains k<sub>P</sub>, k<sub>D</sub>; the check calls it at complex s.${loop === 'outer' ? ' Replace the inner loop by its DC gain, as in F.8.' : ''} Once it passes, the root locus versus k<sub>I<sub>${sub}</sub></sub> appears in the s-plane (select it above).`,
          code: F.pyPart(ctx, {
            args: { kP: KP[loop], kD: KD[loop] },
            items: [{ fn, args: ['s', 'kP', 'kD'], truth: evansTruth(loop) }],
          }, `def ${fn}(s, kP, kD):\n    # 1 + k_I${sub} * ${fn}(s) = 0\n    return ...\n`),
          solution: () => {
            const e = ev();
            return loop === 'lon' ? [
              { tex: '\\Delta = s^3 + \\frac{k_{D_h}}{M}s^2 + \\frac{k_{P_h}}{M}s + \\frac{k_{I_h}}{M} \\;\\Rightarrow\\; L_h(s) = \\frac{1/M}{s^3 + \\frac{k_{D_h}}{M}s^2 + \\frac{k_{P_h}}{M}s},\\quad M = m_c + 2m_r' },
              { tex: `\\text{current PD gains: } L_h(s) = \\frac{${tex(e.num[0])}}{${T.polyTex(e.den)}}` },
              { code: 'def L_h(s, kP, kD):\n    b0 = 1 / (P.mc + 2 * P.mr)\n    return b0 / (s**3 + b0 * kD * s**2 + b0 * kP * s)' },
            ] : [
              { tex: '\\Delta = s^3 + \\Big(\\frac{\\mu}{M} - gk_{DC_\\theta}k_{D_z}\\Big)s^2 - gk_{DC_\\theta}k_{P_z}s - gk_{DC_\\theta}k_{I_z},\\quad k_{DC_\\theta} = 1' },
              { tex: `L_z(s) = \\frac{-g}{s^3 + (\\frac{\\mu}{M} - gk_{D_z})s^2 - gk_{P_z}s};\\quad \\text{current PD gains: } \\frac{${tex(-e.num[0])}}{${T.polyTex(e.den)}}` },
              { code: 'def L_z(s, kP, kD):\n    a = P.mu / (P.mc + 2 * P.mr)\n    b = -P.g  # times k_DC = 1\n    return b / (s**3 + (a + b * kD) * s**2\n                + b * kP * s)' },
              { html: 'k<sub>I<sub>z</sub></sub> < 0 moves along this locus (the plant gain is negative). The s-plane draws it for K = −k<sub>I<sub>z</sub></sub>.' },
            ];
          },
        });
        parts.push({
          id: `${letter}2`, title: `(${letter}) Select k<sub>I<sub>${sub}</sub></sub> that does not significantly change the other closed-loop poles`,
          html: 'Uses the PD gains shown on the right and checks the current slider value: the complex pair must stay within 10% (in |p|) of the PD-only poles, and the new real pole must be slower than them.',
          check: () => {
            const e = ev(), r = kIOk(e, e.K);
            if (!(e.K > 0)) return { ok: false, msg: `Set ${loop === 'lon' ? 'k_I,h > 0' : 'k_I,z < 0'}.` };
            return { ok: r.ok, msg: `|p| ratio ${fmt(r.ratio, 3)}, real pole ${r.real ? fmtPole(r.real) : '—'}.` };
          },
          solution: () => { const e = ev(), kmax = kIRange(e); return [{ html: `With these PD gains any ${loop === 'lon' ? 'k<sub>I<sub>h</sub></sub>' : '−k<sub>I<sub>z</sub></sub>'} up to about ${fmt(kmax, 3)} passes (critical gain ${fmt(e.kCrit, 3)}). The F.10 tab uses ${loop === 'lon' ? `k<sub>I<sub>h</sub></sub> = ${ctx.sys.problems.ch10.kIh}` : `k<sub>I<sub>z</sub></sub> = ${ctx.sys.problems.ch10.kIz}`}.` }]; },
        });
      }
      PD().problemPanel(parent, ctx, prob, parts);
    },
  });

  // ------------------------------------------------------------ Chapter 10 --
  F.register({
    id: 'ch10', num: 10, tab: 'Ch 10', title: 'Digital PID from measured outputs', pages: 'pp. 155–169, F.10 p. 399',
    controller: (ctx, o) => F.makePID(ctx, o),
    defaults(sys) {
      const pr = sys.problems.ch10;
      return { comp: 'eq', lat: 'on', view: 'lat', zOff: 3, hOff: 0, deriv: 'dirty', sigma: pr.sigma, antiwindup: 'gate', vbarH: 0.5, vbarZ: 0.5, extra: 'int',
        w: { ...F.W0 }, k: { ...f8Knobs(sys), kIh: pr.kIh, kIz: pr.kIz } };
    },
    simDefaults(sys) { return { ...sys.problems.ch10.sim, refs: [{ type: 'step', amplitude: 2.5 }], mismatch: sys.problems.ch10.mismatch }; },
    gains(ctx) { return ctx.S.mode === 'work' ? pick(ctx.st.w, ALL) : F.designSLC(ctx.pModel, ctx.st.k); },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Implementation', 'p. 157 · Eq. 10.4, p. 163');
      segmented(sec, { label: 'Rates for the D terms', options: [{ value: 'dirty', label: 'dirty derivative of y' }, { value: 'state', label: 'true rates (cheating)' }], ...bind(ctx, 'deriv') });
      slider(sec, { label: 'σ', unit: 's', min: 0.005, max: 0.5, step: 0.001, sig: 3, ...bind(ctx, 'sigma'), disabled: () => ctx.st.deriv !== 'dirty' });
      segmented(sec, { label: 'Anti-windup', options: [{ value: 'gate', label: 'integrate when |ẏ| < v̄' }, { value: 'none', label: 'none' }], ...bind(ctx, 'antiwindup') });
      slider(sec, { label: 'v̄<sub>h</sub>', unit: 'm/s', min: 0.01, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'vbarH'), disabled: () => ctx.st.antiwindup !== 'gate' });
      slider(sec, { label: 'v̄<sub>z</sub>', unit: 'm/s', min: 0.01, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'vbarZ'), disabled: () => ctx.st.antiwindup !== 'gate' });
      segmented(sec, { label: 'Extra plot', options: [{ value: 'int', label: 'integrators' }, { value: 'rates', label: 'ḣ estimate' }], ...bind(ctx, 'extra') });
      F.viewControl(sec, ctx);
      if (ctx.S.mode === 'work') workSections(parent, ctx, { kI: true });
      else {
        knobSections(parent, ctx);
        const ki = section(parent, 'Integrators', 'p. 160 · §10.1.3');
        slider(ki, { label: 'k<sub>I<sub>h</sub></sub>', min: 0, max: 0.05, step: 0.0001, sig: 3, ...bind(ctx, 'kIh', () => ctx.st.k) });
        slider(ki, { label: 'k<sub>I<sub>z</sub></sub>', min: -0.003, max: 0, step: 0.00001, sig: 3, ...bind(ctx, 'kIz', () => ctx.st.k) });
        F.readout(ki, ctx, ALL.map((k) => ({ key: k, label: k })));
      }
    },
    splane(ctx) { return F.pidSplane(ctx, ctx.gains, { draggable: ctx.S.mode === 'explore' }); },
    onPoleDrag: F.pidDrag,
    extraPlot(ctx, res) {
      if (ctx.st.extra === 'rates') {
        return { opts: { title: 'ḣ and its estimate', yLabel: 'ḣ [m/s]', unit: 'm/s' }, data: { series: [
          { label: 'controller estimate', y: Array.from(res.extras.hdotHat || []), color: '--series-2', width: 1.5 },
          { label: 'true ḣ', y: res.x.map((x) => x[4]), color: '--series-1' },
        ] } };
      }
      return { opts: { title: 'integrators', yLabel: '∫e dt [m·s]', unit: 'm·s' }, data: { series: [
        { label: 'altitude ∫(h_r − h)', y: Array.from(res.extras.intH || []), color: '--series-1' },
        { label: 'position ∫(z_r − z)', y: Array.from(res.extras.intZ || []), color: '--series-3' },
      ] } };
    },
    math(ctx) {
      const st = ctx.st, Ts = ctx.S.sim.Ts;
      const { beta, gamma } = WB.design.dirtyCoeffs(st.sigma, Ts);
      const m = ctx.sys.models(ctx.pModel);
      return [
        { title: 'Nested PID from measured outputs', page: 'p. 155, p. 163–165',
          theory: 'F = F_e + k_{P_h}e_h + k_{I_h}\\!\\int e_h - k_{D_h}\\dot h,\\quad \\theta_d = k_{P_z}e_z + k_{I_z}\\!\\int e_z - k_{D_z}\\dot z,\\quad \\tau = k_{P_\\theta}(\\theta_d - \\theta) - k_{D_\\theta}\\dot\\theta' },
        { title: 'Dirty derivative', page: 'p. 157 · Eq. 10.4',
          theory: '\\dot{\\hat y}[n] = \\frac{2\\sigma - T_s}{2\\sigma + T_s}\\dot{\\hat y}[n-1] + \\frac{2}{2\\sigma + T_s}(y[n] - y[n-1])',
          numbers: `\\frac{2\\sigma - T_s}{2\\sigma + T_s} = ${tex(beta)},\\quad \\frac{2}{2\\sigma + T_s} = ${tex(gamma)}`, spoiler: true },
        { title: 'Why the altitude loop needs k_I', page: 'F.10(b), p. 143–145', answers: ['F.4/a', 'F.9/a'],
          theory: '\\text{true weight } (m_c + 2m_r)_{true}\\,g \\ne F_e \\;\\Rightarrow\\; \\text{constant input disturbance},\\quad e_{ss} = \\frac{\\Delta W}{k_{P_h}}\\;(\\text{PD})',
          numbers: `F_e = ${tex(m.Fe)}\\,\\text{N (nominal)}`, spoiler: true,
          note: 'The z loop has no gravity term, so mass errors do not bias it; k_I,z handles wind and the F.12 disturbance.' },
        { title: 'Anti-windup', page: 'p. 157 · §10.1.1',
          theory: '\\text{integrate only while } |\\dot{\\hat y}| < \\bar v' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch10;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Parameters vary by up to 20% (α = 0.2)',
          html: 'In your dynamics, scale m<sub>c</sub>, J<sub>c</sub>, d and μ by random factors in [1 − α, 1 + α] each run. Here the true plant is set by the plant-mismatch sliders in the left panel (the chapter starts with a fixed 20% draw). Passes when every mismatch is within ±20% and at least one is nonzero.',
          check: () => {
            const mis = ctx.S.mismatch || {};
            const v = ['mc', 'Jc', 'd', 'mu'].map((k) => mis[k] || 0);
            if (!v.some((x) => Math.abs(x) > 0)) return { ok: false, msg: 'Set a plant mismatch in the left panel.' };
            return v.every((x) => Math.abs(x) <= 20.0001) ? { ok: true, msg: `m_c ${fmt(v[0], 3)}%, J_c ${fmt(v[1], 3)}%, d ${fmt(v[2], 3)}%, μ ${fmt(v[3], 3)}%.` } : { ok: false, msg: 'Keep every parameter within ±20%.' };
          },
        },
        {
          id: 'b1', title: '(b) Dirty-derivative coefficients for σ = 0.05, T<sub>s</sub> = 0.01',
          inputs: { a: '(2σ−T<sub>s</sub>)/(2σ+T<sub>s</sub>)', b: '2/(2σ+T<sub>s</sub>)' },
          check: (v) => PD().checkNumbers(v, { a: 0.09 / 0.11, b: 2 / 0.11 }, {}),
          solution: () => [{ tex: '\\frac{0.09}{0.11} = 0.8182,\\quad \\frac{2}{0.11} = 18.18' }],
        },
        {
          id: 'b2', title: '(b) No steady-state error with α = 0.2',
          html: 'Checks the current simulation (mismatch in the left panel): |h<sub>r</sub> − h| and |z<sub>r</sub> − z| at t<sub>end</sub> both under 1 cm, using only measured outputs.',
          check: () => {
            if (ctx.st.deriv !== 'dirty') return { ok: false, msg: 'Use the dirty derivative (measured outputs only).' };
            const res = ctx.app.result(), n = res.t.length - 1;
            const eh = Math.abs(res.rAll[0][n] - res.yAll[1][n]), ez = Math.abs(res.rAll[1][n] - res.yAll[0][n]);
            return { ok: eh < 0.01 && ez < 0.01, msg: `|e_h| = ${fmt(eh, 3)} m, |e_z| = ${fmt(ez, 3)} m at t_end.` };
          },
          solution: () => [
            { html: `The F.8 gains plus k<sub>I<sub>h</sub></sub> = ${prob.kIh} and k<sub>I<sub>z</sub></sub> = ${prob.kIz}, σ = 0.05, and anti-windup gates at v̄ = 0.5 m/s. Both integrator gains are inside the critical gains from P.6 (0.044 and −0.0030), but they move the PD poles more than P.6's 10% guideline (which allows only about k<sub>I<sub>h</sub></sub> ≤ 0.0037, |k<sub>I<sub>z</sub></sub>| ≤ 0.00026). That is needed here: the soft F.8 altitude loop sags meters under a 20% mass error, and with k<sub>I<sub>h</sub></sub> = 0.003 the altitude error is still about 1.6 m at 80 s. A tighter gate (e.g. 0.1 m/s) keeps the integrator off during the sag. The z loop is type 1, so its step error is zero even without k<sub>I<sub>z</sub></sub>; a small k<sub>I<sub>z</sub></sub> adds a slow closed-loop pole, and with |k<sub>I<sub>z</sub></sub>| = 0.0002 z is still 3 cm short at 80 s.` },
          ],
        },
      ]);
    },
  });
  WB.F.pid = { evans, loopAnalysis, kIOk, kIRange, fastestNoSat, satRun, f8Knobs };
})();
