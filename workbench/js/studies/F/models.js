// Study F, Chapters 2–6 (F.2–F.6): kinetic energy, Euler-Lagrange equations,
// equilibria and linearization, transfer functions, state-space models.
// Open-loop experiments: the rotor forces come from an input program in (F, τ).
(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const F = WB.F;
  const { tex, texMat, texPole, fmt } = M;
  const PD = () => WB.pd;
  const sysF = () => WB.systems.F;

  // ------------------------------------------------ open-loop input program --
  function prog(spec, t) {
    const { shape, amp, freq, width = 0.5, t0 = 0.5 } = spec;
    if (t < t0) return 0;
    const tt = t - t0;
    switch (shape) {
      case 'const': return amp;
      case 'pulse': return tt < width ? amp : 0;
      case 'doublet': return tt < width ? amp : tt < 2 * width ? -amp : 0;
      case 'square': return (tt % (1 / freq)) <= 0.5 / freq ? amp : -amp;
      case 'sine': return amp * Math.sin(2 * Math.PI * freq * tt);
      default: return 0;
    }
  }
  // F = F_e + F̃(t) (hover on) or F̃(t); comp 'fl' divides by cos θ (F.4c).
  function openLoop(ctx, { linear = false } = {}) {
    const s = ctx.sys, p = ctx.pModel, st = ctx.st;
    const m = s.models(p);
    return {
      update(r, x, y, t) {
        const Ft = prog(st.inF, t), tau = prog(st.inT, t);
        let Fc = Ft;
        if (!linear && st.hover) Fc = st.comp === 'fl' ? (m.Fe + Ft) / Math.cos(x[2]) : m.Fe + Ft;
        return { u: s.mix(Fc, tau, p), Ftilde: Ft, tau };
      },
    };
  }
  const SHAPES = { zero: 'none', const: 'constant', pulse: 'pulse', doublet: 'doublet', square: 'square', sine: 'sine' };
  function progControls(parent, ctx, key, label, unit, ampMax) {
    segmented(parent, {
      label, options: ['zero', 'const', 'pulse', 'doublet', 'square', 'sine'].map((v) => ({ value: v, label: SHAPES[v] })),
      get: () => ctx.st[key].shape, set: (v) => { ctx.st[key].shape = v; ctx.update(); },
    });
    const off = () => ctx.st[key].shape === 'zero';
    slider(parent, { label: 'amplitude', unit, min: -ampMax, max: ampMax, step: ampMax / 400, sig: 3, get: () => ctx.st[key].amp, set: (v) => { ctx.st[key].amp = v; ctx.update(); }, disabled: off });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.01, max: 2, step: 0.005, sig: 3, get: () => ctx.st[key].freq, set: (v) => { ctx.st[key].freq = v; ctx.update(); }, disabled: () => !['square', 'sine'].includes(ctx.st[key].shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.05, max: 5, step: 0.05, sig: 3, get: () => ctx.st[key].width, set: (v) => { ctx.st[key].width = v; ctx.update(); }, disabled: () => !['pulse', 'doublet'].includes(ctx.st[key].shape) });
  }
  function hoverControls(parent, ctx) {
    segmented(parent, {
      label: 'Add hover force F<sub>e</sub>',
      options: [{ value: true, label: 'F = F<sub>e</sub> + F̃' }, { value: false, label: 'F = F̃ only' }],
      get: () => ctx.st.hover, set: (v) => { ctx.st.hover = v; ctx.update(); },
    });
  }
  const olBase = { openLoop: true, metrics: false, lateralMetrics: false, controller: (ctx, o) => openLoop(ctx, o), reference: () => () => [NaN, 0] };
  const inDefaults = (o = {}) => ({
    hover: true, comp: 'eq', view: 'lat',
    inF: { shape: 'zero', amp: 0.5, freq: 0.2, width: 1, t0: 0.5, ...(o.inF || {}) },
    inT: { shape: 'doublet', amp: 0.005, freq: 0.2, width: 0.5, t0: 0.5, ...(o.inT || {}) },
  });

  // ----------------------------------------------------------- Chapter 2 --
  // Prescribed motion (no dynamics), as in hw02: energy is computed, not simulated.
  function prescribed(ctx) {
    const { sys, pModel: p, st, S } = ctx;
    const Ts = S.sim.Ts, N = Math.round(S.sim.tEnd / Ts) + 1;
    const m = sys.models(p);
    const Fa = () => new Float64Array(N);
    const res = { t: Fa(), rAll: [Fa(), Fa()], yAll: [Fa(), Fa(), Fa()], uDemandAll: [Fa(), Fa()], x: [], extras: { Kt: Fa(), Kr: Fa(), Kc: Fa(), K: Fa() } };
    const w = 2 * Math.PI * st.f;
    for (let k = 0; k < N; k++) {
      const t = k * Ts;
      const z = st.Az * Math.sin(w * t), zd = st.Az * w * Math.cos(w * t);
      const hh = st.h0 + st.Ah * Math.sin(w * t), hd = st.Ah * w * Math.cos(w * t);
      const th = st.Ath * M.DEG * Math.sin(w * t), thd = st.Ath * M.DEG * w * Math.cos(w * t);
      const x = [z, hh, th, zd, hd, thd];
      res.t[k] = t; res.x.push(x);
      res.yAll[0][k] = z; res.yAll[1][k] = hh; res.yAll[2][k] = th;
      res.rAll[0][k] = NaN; res.rAll[1][k] = 0;
      res.extras.Kt[k] = 0.5 * m.M * (zd * zd + hd * hd);
      res.extras.Kc[k] = 0.5 * p.Jc * thd * thd;
      res.extras.Kr[k] = p.mr * p.d * p.d * thd * thd;
      res.extras.K[k] = sys.kinetic(x, p);
    }
    res.yMeasAll = res.yAll; res.uAll = res.uDemandAll; res.uAppliedAll = res.uDemandAll;
    return res;
  }

  F.register(Object.assign({}, olBase, {
    id: 'ch2', num: 2, tab: 'Ch 2', title: 'Kinetic energy', pages: 'pp. 19–40, F.2 p. 395',
    linear: false,
    defaults() { return { ...inDefaults(), Az: 3, Ah: 1, h0: 2, Ath: 20, f: 0.2 }; },
    simDefaults(sys) { return sys.problems.ch2.sim; },
    simulate(ctx) { return prescribed(ctx); },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Prescribed motion (sinusoids at f)', 'F.2(b) p. 395');
      slider(sec, { label: 'z amplitude', unit: 'm', min: 0, max: 6, step: 0.05, sig: 3, get: () => ctx.st.Az, set: (v) => { ctx.st.Az = v; ctx.update(); } });
      slider(sec, { label: 'h mean', unit: 'm', min: 0, max: 8, step: 0.05, sig: 3, get: () => ctx.st.h0, set: (v) => { ctx.st.h0 = v; ctx.update(); } });
      slider(sec, { label: 'h amplitude', unit: 'm', min: 0, max: 4, step: 0.05, sig: 3, get: () => ctx.st.Ah, set: (v) => { ctx.st.Ah = v; ctx.update(); } });
      slider(sec, { label: 'θ amplitude', unit: '°', min: 0, max: 60, step: 0.5, sig: 3, get: () => ctx.st.Ath, set: (v) => { ctx.st.Ath = v; ctx.update(); } });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.02, max: 1, step: 0.01, sig: 3, get: () => ctx.st.f, set: (v) => { ctx.st.f = v; ctx.update(); } });
      sec.append(el('p', { class: 'muted small', text: 'No dynamics here: the motion is imposed, as in an hw02 animation test, so the rotor forces are zero. The plot below splits K into translation, pod rotation and rotor rotation.' }));
    },
    extraPlot(ctx, res) {
      const mJ = (a) => Array.from(a);
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [J]', unit: 'J' },
        data: { series: [
          { label: 'translation ½(m_c+2m_r)(ż²+ḣ²)', y: mJ(res.extras.Kt), color: '--series-2', width: 1.5 },
          { label: 'pod rotation ½J_cθ̇²', y: mJ(res.extras.Kc), color: '--series-3', width: 1.5 },
          { label: 'rotors m_r d²θ̇²', y: mJ(res.extras.Kr), color: '--text-muted', width: 1.5 },
          { label: 'total K', y: mJ(res.extras.K), color: '--series-1' },
        ] },
      };
    },
    math(ctx) {
      const p = ctx.pModel, m = ctx.sys.models(p);
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Rotor positions (point masses at ±d)', page: 'Fig. 21-1 p. 393',
          theory: '\\mathbf p_{r} = \\begin{bmatrix} z + d\\cos\\theta \\\\ h + d\\sin\\theta\\end{bmatrix},\\quad \\mathbf p_{\\ell} = \\begin{bmatrix} z - d\\cos\\theta \\\\ h - d\\sin\\theta\\end{bmatrix},\\quad \\|\\dot{\\mathbf p}_{r,\\ell}\\|^2 = \\dot z^2 + \\dot h^2 + d^2\\dot\\theta^2 \\mp 2d\\dot\\theta(\\dot z\\sin\\theta - \\dot h\\cos\\theta)' },
        { title: 'Kinetic energy', page: 'F.2(a) p. 395',
          theory: 'K = \\tfrac12 m_c(\\dot z^2 + \\dot h^2) + \\tfrac12 J_c\\dot\\theta^2 + \\tfrac12 m_r\\|\\dot{\\mathbf p}_r\\|^2 + \\tfrac12 m_\\ell\\|\\dot{\\mathbf p}_\\ell\\|^2',
          symbolic: 'K = \\tfrac12(m_c + 2m_r)(\\dot z^2 + \\dot h^2) + \\tfrac12(J_c + 2m_r d^2)\\dot\\theta^2',
          numbers: `K = ${tex(m.M / 2)}(\\dot z^2 + \\dot h^2) + ${tex(m.J / 2)}\\,\\dot\\theta^2`, spoiler: true,
          note: 'The cross terms cancel because the rotors sit symmetrically at ±d (and m_ℓ = m_r).' },
      ];
    },
    buildProblem(parent, ctx) {
      const m = () => ctx.sys.models(ctx.pModel);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) K = c<sub>1</sub>(ż² + ḣ²) + c<sub>2</sub>θ̇²',
          inputs: { c1: 'c<sub>1</sub> [kg]', c2: 'c<sub>2</sub> [kg·m²]' },
          check: (v) => PD().checkNumbers(v, { c1: m().M / 2, c2: m().J / 2 }, {}),
          solution: () => [
            { tex: '\\|\\dot{\\mathbf p}_r\\|^2 + \\|\\dot{\\mathbf p}_\\ell\\|^2 = 2(\\dot z^2 + \\dot h^2) + 2d^2\\dot\\theta^2 \\;(\\text{cross terms cancel})' },
            { tex: `K = \\tfrac12(m_c + 2m_r)(\\dot z^2 + \\dot h^2) + \\tfrac12(J_c + 2m_r d^2)\\dot\\theta^2 = ${tex(m().M / 2)}(\\dot z^2 + \\dot h^2) + ${tex(m().J / 2)}\\dot\\theta^2` },
            { html: 'Kinetic energy of a rigid body, Eq. 2.3 (p. 25), with the rotors as point masses (p. 393).' },
          ],
        },
        {
          id: 'b', title: '(b) Animation',
          html: 'The animation above takes z, h, θ and the target position z<sub>t</sub> (drawn on the ground at z<sub>r</sub>), as VTOLAnimation.py does. Move the sliders on the right to see it.',
        },
      ]);
    },
  }));

  // ----------------------------------------------------------- Chapter 3 --
  F.register(Object.assign({}, olBase, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 41–56, F.3 p. 395',
    linear: false,
    defaults() { return inDefaults(); },
    simDefaults(sys) { return sys.problems.ch3.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop inputs', 'F.3(e) p. 395');
      hoverControls(sec, ctx);
      progControls(sec, ctx, 'inF', 'Force F̃(t)', 'N', 10);
      progControls(sec, ctx, 'inT', 'Torque τ(t)', 'N·m', 0.5);
      sec.append(el('p', { class: 'muted small', text: 'f_r = F/2 + τ/2d and f_ℓ = F/2 − τ/2d, then each is saturated to [0, f_max]. A torque doublet tilts the VTOL; the tilted thrust then pushes it sideways. The energy plot checks the equations of motion.' }));
    },
    extraPlot(ctx, res) {
      const { sys, pModel: p } = ctx;
      const n = res.t.length, Ts = ctx.S.sim.Ts;
      const E = new Float64Array(n), W = new Float64Array(n);
      const E0 = sys.kinetic(res.x[0], p) + sys.potential(res.x[0], p);
      // power of the generalized forces: F(−ż sinθ + ḣ cosθ) + τθ̇ − μż²
      const pw = (x, u) => { const Fn = u[0] + u[1], tau = p.d * (u[0] - u[1]); return Fn * (-x[3] * Math.sin(x[2]) + x[4] * Math.cos(x[2])) + tau * x[5] - p.mu * x[3] * x[3]; };
      let w = 0;
      for (let k = 0; k < n; k++) {
        E[k] = sys.kinetic(res.x[k], p) + sys.potential(res.x[k], p) - E0;
        if (k > 0) {
          const u = [res.uAppliedAll[0][k - 1], res.uAppliedAll[1][k - 1]];
          w += 0.5 * Ts * (pw(res.x[k - 1], u) + pw(res.x[k], u));
        }
        W[k] = w;
      }
      return {
        opts: { title: 'energy balance', yLabel: 'energy [J]', unit: 'J' },
        data: { series: [
          { label: '∫ (Q·q̇ − μż²) dt', y: W, color: '--series-2', dash: [5, 4], width: 2 },
          { label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' },
        ] },
      };
    },
    math(ctx) {
      const p = ctx.pModel, m = ctx.sys.models(p);
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 43',
          theory: 'L = K - P,\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy, coordinates, forces', page: 'F.3(a)–(c) p. 395',
          theory: 'q = (z, h, \\theta)^\\top',
          symbolic: 'P = (m_c + 2m_r)\\,g\\,h,\\quad Q = \\begin{bmatrix} -F\\sin\\theta \\\\ F\\cos\\theta \\\\ \\tau\\end{bmatrix},\\quad B\\dot q = \\begin{bmatrix}\\mu\\dot z\\\\ 0\\\\ 0\\end{bmatrix}',
          numbers: `P = ${tex(m.M * p.g)}\\,h`, spoiler: true,
          note: 'The thrust F acts along the body normal (−sin θ, cos θ); the drag −μż acts on z only (p. 393).' },
        { title: 'Equations of motion', page: 'F.3(d) p. 395',
          theory: 'M(q)\\ddot q + c(q,\\dot q) + \\frac{\\partial P}{\\partial q} = Q - B\\dot q',
          symbolic: '(m_c + 2m_r)\\ddot z = -F\\sin\\theta - \\mu\\dot z,\\quad (m_c + 2m_r)\\ddot h = F\\cos\\theta - (m_c + 2m_r)g,\\quad (J_c + 2m_r d^2)\\ddot\\theta = \\tau',
          numbers: `\\ddot z = ${tex(-1 / m.M)}F\\sin\\theta ${tex(-p.mu / m.M)}\\dot z,\\quad \\ddot h = ${tex(1 / m.M)}F\\cos\\theta - ${tex(p.g)},\\quad \\ddot\\theta = ${tex(1 / m.J)}\\,\\tau`, spoiler: true },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from the EOM',
          theory: '\\frac{d}{dt}(K + P) = Q^\\top\\dot q - \\mu\\dot z^2 = F(-\\dot z\\sin\\theta + \\dot h\\cos\\theta) + \\tau\\dot\\theta - \\mu\\dot z^2',
          note: 'Only the input forces enter this balance; the disturbance sliders (wind etc.) break it.' },
      ];
    },
    buildProblem(parent, ctx) {
      const p = () => ctx.pModel, m = () => ctx.sys.models(p());
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch3, [
        {
          id: 'a', title: '(a) P = P<sub>0</sub> + c·h',
          inputs: { c: 'c [N]' },
          check: (v) => PD().checkNumbers(v, { c: m().M * p().g }, {}),
          solution: () => [{ tex: `P = (m_c + m_r + m_\\ell)\\,g\\,h = ${tex(m().M * p().g)}\\,h` }, { html: 'All three masses rise with h; θ only moves the rotors up and down symmetrically, so it cancels.' }],
        },
        {
          id: 'c', title: '(b, c) Generalized forces at F = 15 N, τ = 0.1 N·m, θ = 10°, ż = 1 m/s',
          html: 'Enter the total generalized force on each coordinate, including the drag −μż on z.',
          inputs: { Qz: 'Q<sub>z</sub> − μż [N]', Qh: 'Q<sub>h</sub> [N]', Qt: 'Q<sub>θ</sub> [N·m]' },
          check: (v) => PD().checkNumbers(v, { Qz: -15 * Math.sin(10 * M.DEG) - p().mu * 1, Qh: 15 * Math.cos(10 * M.DEG), Qt: 0.1 }, {}),
          solution: () => [
            { tex: 'q = (z, h, \\theta),\\quad Q = (-F\\sin\\theta,\\; F\\cos\\theta,\\; \\tau),\\quad B = \\operatorname{diag}(\\mu, 0, 0)' },
            { tex: `Q_z - \\mu\\dot z = ${tex(-15 * Math.sin(10 * M.DEG) - p().mu)},\\; Q_h = ${tex(15 * Math.cos(10 * M.DEG))},\\; Q_\\theta = 0.1` },
          ],
        },
        {
          id: 'd', title: '(d) z̈ = c<sub>1</sub>F sin θ + c<sub>2</sub>ż, ḧ = c<sub>3</sub>F cos θ + c<sub>4</sub>, θ̈ = c<sub>5</sub>τ',
          inputs: { c1: 'c<sub>1</sub>', c2: 'c<sub>2</sub>', c3: 'c<sub>3</sub>', c4: 'c<sub>4</sub>', c5: 'c<sub>5</sub>' },
          check: (v) => PD().checkNumbers(v, { c1: -1 / m().M, c2: -p().mu / m().M, c3: 1 / m().M, c4: -p().g, c5: 1 / m().J }, {}),
          solution: () => [
            { tex: 'M(q) = \\operatorname{diag}(m_c + 2m_r,\\; m_c + 2m_r,\\; J_c + 2m_r d^2),\\quad c(q,\\dot q) = 0,\\quad \\tfrac{\\partial P}{\\partial q} = (0,\\; (m_c+2m_r)g,\\; 0)' },
            { tex: `c_1 = -\\tfrac{1}{m_c+2m_r} = ${tex(-1 / m().M)},\\; c_2 = -\\tfrac{\\mu}{m_c+2m_r} = ${tex(-p().mu / m().M)},\\; c_3 = ${tex(1 / m().M)},\\; c_4 = -g = ${tex(-p().g)},\\; c_5 = \\tfrac{1}{J_c + 2m_r d^2} = ${tex(1 / m().J)}` },
            { html: 'These reproduce every expected value in <code>_F_planar_vtol/python/testDynamics.py</code> (checked by <code>workbench/tools/regress_F.py</code>).' },
          ],
        },
      ]);
    },
  }));

  // ----------------------------------------------------------- Chapter 4 --
  F.register(Object.assign({}, olBase, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68, F.4 pp. 395–396',
    defaults() { return inDefaults({ inT: { shape: 'pulse', amp: 0.002, width: 0.5 } }); },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    linearSim(ctx, common) { ctx.chapter = this; return F.linearSim(ctx, common); },
    linearLabel: 'Jacobian linearization',
    buildControls(parent, ctx) {
      const sec = section(parent, 'Around hover', 'F.4 p. 396');
      segmented(sec, {
        label: 'Force law',
        options: [{ value: 'eq', label: 'F = F<sub>e</sub> + F̃ (Jacobian)' }, { value: 'fl', label: 'F = (F<sub>e</sub> + F̃)/cos θ' }],
        get: () => ctx.st.comp, set: (v) => { ctx.st.comp = v; ctx.update(); },
      });
      progControls(sec, ctx, 'inF', 'Force F̃(t)', 'N', 5);
      progControls(sec, ctx, 'inT', 'Torque τ̃(t)', 'N·m', 0.05);
      sec.append(el('p', { class: 'muted small', text: 'Start at hover, nudge with F̃ and τ̃, and compare with the linearized model (dashed). Set θ(0) in the left panel to see how the linearization of sin θ and cos θ degrades with angle.' }));
    },
    splane(ctx) {
      const m = ctx.sys.models(ctx.pModel);
      const mk = [...L.eig(m.lonSS.A).map((q) => ({ ...q, kind: 'ol', label: 'eig A_lon' })), ...L.eig(m.latSS.A).map((q) => ({ ...q, kind: 'cl', label: 'eig A_lat' }))];
      return { markers: mk, fitR: 0.3, legendNames: { ol: 'eig of A_lon', cl: 'eig of A_lat' } };
    },
    math(ctx) {
      const p = ctx.pModel, m = ctx.sys.models(p);
      return [
        { title: 'Equilibria', page: 'F.4(a) p. 396 · p. 59',
          theory: '\\dot q = 0,\\; \\ddot q = 0:\\quad F\\sin\\theta_e = 0,\\; F\\cos\\theta_e = (m_c + 2m_r)g,\\; \\tau_e = 0',
          symbolic: 'z_e, h_e \\text{ arbitrary},\\quad \\theta_e = 0,\\quad F_e = (m_c+2m_r)g,\\quad f_{r,e} = f_{\\ell,e} = \\tfrac12 F_e',
          numbers: `F_e = ${tex(m.Fe)}\\,\\text{N},\\quad f_{r,e} = f_{\\ell,e} = ${tex(m.Fe / 2)}\\,\\text{N}`, spoiler: true },
        { title: 'Jacobian linearization', page: 'p. 59–61 · Eq. 4.1',
          theory: '\\sin\\theta \\approx \\tilde\\theta,\\; \\cos\\theta \\approx 1:\\quad F\\sin\\theta \\approx F_e\\tilde\\theta,\\; F\\cos\\theta \\approx F_e + \\tilde F',
          symbolic: '\\ddot{\\tilde z} = -\\frac{F_e}{m_c+2m_r}\\tilde\\theta - \\frac{\\mu}{m_c+2m_r}\\dot{\\tilde z} = -g\\tilde\\theta - \\frac{\\mu}{m_c+2m_r}\\dot{\\tilde z},\\quad \\ddot{\\tilde h} = \\frac{\\tilde F}{m_c+2m_r},\\quad \\ddot{\\tilde\\theta} = \\frac{\\tilde\\tau}{J_c + 2m_r d^2}',
          numbers: `\\ddot{\\tilde z} = ${tex(-p.g)}\\tilde\\theta ${tex(-m.a)}\\dot{\\tilde z},\\quad \\ddot{\\tilde h} = ${tex(1 / m.M)}\\tilde F,\\quad \\ddot{\\tilde\\theta} = ${tex(1 / m.J)}\\tilde\\tau`, spoiler: true },
        { title: 'Feedback linearization', page: 'p. 62 · F.4(c)',
          theory: 'F = \\frac{(m_c+2m_r)g + \\tilde F}{\\cos\\theta} \\;\\Rightarrow\\; (m_c+2m_r)\\ddot h = \\tilde F \\;\\text{exactly } (|\\theta| < 90^\\circ)',
          note: 'Only the altitude channel. The lateral channel stays nonlinear: z̈ = −(g + F̃/M) tan θ − (μ/M)ż, and θ is a state, not an input, so no static feedback of τ alone linearizes z. The book\'s designs use the Jacobian model.' },
      ];
    },
    buildProblem(parent, ctx) {
      const p = () => ctx.pModel, m = () => ctx.sys.models(p());
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibrium inputs',
          inputs: { Fe: 'F<sub>e</sub> [N]', te: 'τ<sub>e</sub> [N·m]', fr: 'f<sub>r,e</sub> [N]', fl: 'f<sub>ℓ,e</sub> [N]', th: 'θ<sub>e</sub> [°]' },
          check: (v) => PD().checkNumbers(v, { Fe: m().Fe, te: 0, fr: m().Fe / 2, fl: m().Fe / 2, th: 0 }, {}),
          solution: () => [{ tex: `\\theta_e = 0,\\; F_e = (m_c+2m_r)g = ${tex(m().Fe)},\\; \\tau_e = 0,\\; f_{r,e} = f_{\\ell,e} = ${tex(m().Fe / 2)};\\; z_e, h_e \\text{ arbitrary}` }],
        },
        {
          id: 'b', title: '(b) Jacobian: z̃̈ = a<sub>1</sub>θ̃ + a<sub>2</sub>z̃̇, h̃̈ = b<sub>1</sub>F̃, θ̃̈ = c<sub>1</sub>τ̃',
          inputs: { a1: 'a<sub>1</sub>', a2: 'a<sub>2</sub>', b1: 'b<sub>1</sub>', c1: 'c<sub>1</sub>' },
          check: (v) => PD().checkNumbers(v, { a1: -p().g, a2: -m().a, b1: 1 / m().M, c1: 1 / m().J }, {}),
          solution: () => [
            { tex: '\\frac{\\partial}{\\partial\\theta}\\Big(-\\frac{F\\sin\\theta}{m_c+2m_r}\\Big)\\Big|_e = -\\frac{F_e}{m_c+2m_r} = -g' },
            { tex: `a_1 = ${tex(-p().g)},\\; a_2 = ${tex(-m().a)},\\; b_1 = ${tex(1 / m().M)},\\; c_1 = ${tex(1 / m().J)}` },
          ],
        },
        {
          id: 'c', title: '(c) Feedback linearization: F needed to hold altitude at θ = 10° (F̃ = 0)',
          inputs: { F: 'F [N]' },
          check: (v) => PD().checkNumbers(v, { F: m().Fe / Math.cos(10 * M.DEG) }, {}),
          solution: () => [
            { tex: `F = \\frac{(m_c + 2m_r)g}{\\cos 10^\\circ} = ${tex(m().Fe / Math.cos(10 * M.DEG))}\\,\\text{N}` },
            { html: 'The altitude channel can be feedback linearized with F = (F<sub>e</sub> + F̃)/cos θ. The lateral channel cannot be linearized the same way: switch the force law above and compare.' },
          ],
        },
      ]);
    },
  }));

  // ------------------------------------------------------ Chapters 5 and 6 --
  function tfInputSection(parent, ctx, page) {
    const sec = section(parent, 'Inputs around hover', page);
    progControls(sec, ctx, 'inF', 'Force F̃(t)', 'N', 5);
    progControls(sec, ctx, 'inT', 'Torque τ̃(t)', 'N·m', 0.05);
    sec.append(el('p', { class: 'muted small', text: 'F = F_e + F̃. Dashed traces: the linear model. A torque pulse tilts the VTOL by a constant angle (two integrators), and the tilted thrust then accelerates it sideways against the drag.' }));
  }
  const tfSplane = (ctx) => {
    const m = ctx.sys.models(ctx.pModel);
    return {
      markers: [
        { re: 0, im: 0, kind: 'ol', label: 'H/F̃ pole (double at 0)' },
        { re: 0, im: 0, kind: 'cl', label: 'Θ/τ̃ pole (double at 0)' },
        { re: 0, im: 0, kind: 'obs', label: 'Z/Θ pole at 0' }, { re: -m.a, im: 0, kind: 'obs', label: 'Z/Θ pole at −μ/M' },
      ],
      fitR: 0.12, legendNames: { ol: 'H/F̃ poles', cl: 'Θ/τ̃ poles', obs: 'Z/Θ poles' },
    };
  };

  F.register(Object.assign({}, olBase, {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 69–80, F.5 p. 396',
    defaults() { return inDefaults({ inT: { shape: 'doublet', amp: 0.003, width: 0.5 }, inF: { shape: 'pulse', amp: 0.5, width: 1 } }); },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearSim(ctx, common) { ctx.chapter = this; return F.linearSim(ctx, common); },
    linearLabel: 'transfer-function model',
    buildControls(parent, ctx) { tfInputSection(parent, ctx, 'F.5 p. 396'); },
    splane: tfSplane,
    math(ctx) {
      const p = ctx.pModel, m = ctx.sys.models(p);
      return [
        { title: 'Laplace transform (zero initial conditions)', page: 'p. 69–70',
          theory: '(m_c+2m_r)s^2\\tilde H = \\tilde F,\\quad (J_c+2m_rd^2)s^2\\tilde\\Theta = \\tilde\\tau,\\quad \\big(s^2 + \\tfrac{\\mu}{m_c+2m_r}s\\big)\\tilde Z = -g\\tilde\\Theta' },
        { title: 'Longitudinal', page: 'F.5(b) p. 396',
          theory: '\\frac{\\tilde H(s)}{\\tilde F(s)} = P_{lon}(s)',
          symbolic: 'P_{lon}(s) = \\frac{1/(m_c+2m_r)}{s^2}', numbers: `P_{lon}(s) = \\frac{${tex(1 / m.M)}}{s^2}`, spoiler: true },
        { title: 'Lateral (cascade, p. 72–74)', page: 'F.5(c) p. 396',
          theory: '\\frac{\\tilde Z}{\\tilde\\tau} = \\frac{\\tilde Z}{\\tilde\\Theta}\\cdot\\frac{\\tilde\\Theta}{\\tilde\\tau}',
          symbolic: '\\frac{\\tilde\\Theta}{\\tilde\\tau} = \\frac{1/(J_c+2m_rd^2)}{s^2},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{-g}{s^2 + \\frac{\\mu}{m_c+2m_r}s},\\quad \\frac{\\tilde Z}{\\tilde\\tau} = \\frac{-g/(J_c+2m_rd^2)}{s^3\\big(s + \\frac{\\mu}{m_c+2m_r}\\big)}',
          numbers: `\\frac{\\tilde\\Theta}{\\tilde\\tau} = \\frac{${tex(1 / m.J)}}{s^2},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{${tex(-p.g)}}{s^2 + ${tex(m.a)}s},\\quad \\frac{\\tilde Z}{\\tilde\\tau} = \\frac{${tex(-p.g / m.J)}}{s^4 + ${tex(m.a)}s^3}`, spoiler: true },
        { title: 'Open-loop block diagrams', page: 'F.5(d) p. 396',
          theory: '\\tilde F \\to \\boxed{\\tfrac{1/(m_c+2m_r)}{s^2}} \\to \\tilde H \\qquad \\tilde\\tau \\to \\boxed{\\tfrac{1/(J_c+2m_rd^2)}{s^2}} \\xrightarrow{\\;\\tilde\\Theta\\;} \\boxed{\\tfrac{-g}{s(s + \\mu/(m_c+2m_r))}} \\to \\tilde Z',
          note: 'The minus sign: a positive roll tilts the thrust to the left (−z).' },
      ];
    },
    buildProblem(parent, ctx) {
      const p = () => ctx.pModel, m = () => ctx.sys.models(p());
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'b', title: '(b) H̃/F̃ = b<sub>0</sub>/s²',
          inputs: { b0: 'b<sub>0</sub>' },
          check: (v) => PD().checkNumbers(v, { b0: 1 / m().M }, {}),
          solution: () => [{ tex: `\\frac{\\tilde H}{\\tilde F} = \\frac{1/(m_c+2m_r)}{s^2} = \\frac{${tex(1 / m().M)}}{s^2}` }],
        },
        {
          id: 'c', title: '(c) Θ̃/τ̃ = b<sub>θ</sub>/s², Z̃/Θ̃ = b<sub>z</sub>/(s² + a<sub>z</sub>s)',
          inputs: { bt: 'b<sub>θ</sub>', bz: 'b<sub>z</sub>', az: 'a<sub>z</sub>' },
          check: (v) => PD().checkNumbers(v, { bt: 1 / m().J, bz: -p().g, az: m().a }, {}),
          solution: () => [
            { tex: `\\frac{\\tilde\\Theta}{\\tilde\\tau} = \\frac{${tex(1 / m().J)}}{s^2},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{${tex(-p().g)}}{s^2 + ${tex(m().a)}s},\\quad \\frac{\\tilde Z}{\\tilde\\tau} = \\frac{${tex(-p().g / m().J)}}{s^3(s + ${tex(m().a)})}` },
          ],
        },
        { id: 'd', title: '(d) Block diagrams', html: 'See the <em>Open-loop block diagrams</em> card under Live math.' },
      ]);
    },
  }));

  F.register(Object.assign({}, olBase, {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 81–93, F.6 p. 396',
    defaults() { return inDefaults({ inT: { shape: 'doublet', amp: 0.003, width: 0.5 }, inF: { shape: 'pulse', amp: 0.5, width: 1 } }); },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearSim(ctx, common) { ctx.chapter = this; return F.linearSim(ctx, common); },
    linearLabel: 'ẋ = Ax + Bu',
    buildControls(parent, ctx) { tfInputSection(parent, ctx, 'F.6 p. 396'); },
    splane: tfSplane,
    extraPlot(ctx, res) {
      return { opts: { title: 'velocities ż, ḣ', yLabel: 'velocity [m/s]', unit: 'm/s' }, data: { series: [
        { label: 'ḣ', y: res.x.map((x) => x[4]), color: '--series-3', width: 1.5 },
        { label: 'ż', y: res.x.map((x) => x[3]), color: '--series-1' },
      ] } };
    },
    math(ctx) {
      const m = ctx.sys.models(ctx.pModel);
      return [
        { title: 'Longitudinal state space', page: 'F.6(a) p. 396 · p. 83',
          theory: 'x_{lon} = (\\tilde h, \\dot{\\tilde h})^\\top,\\; u_{lon} = \\tilde F,\\; y_{lon} = \\tilde h',
          symbolic: 'A = \\begin{bmatrix}0 & 1\\\\ 0 & 0\\end{bmatrix},\\; B = \\begin{bmatrix}0\\\\ \\frac{1}{m_c+2m_r}\\end{bmatrix},\\; C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\; D = 0',
          numbers: `A = ${texMat(m.lonSS.A)},\\; B = ${texMat(m.lonSS.B)}`, spoiler: true },
        { title: 'Lateral state space', page: 'F.6(b) p. 396',
          theory: 'x_{lat} = (\\tilde z, \\tilde\\theta, \\dot{\\tilde z}, \\dot{\\tilde\\theta})^\\top,\\; u_{lat} = \\tilde\\tau,\\; y_{lat} = (\\tilde z, \\tilde\\theta)^\\top',
          symbolic: 'A = \\begin{bmatrix}0&0&1&0\\\\0&0&0&1\\\\0&-g&-\\frac{\\mu}{m_c+2m_r}&0\\\\0&0&0&0\\end{bmatrix},\\; B = \\begin{bmatrix}0\\\\0\\\\0\\\\ \\frac{1}{J_c+2m_rd^2}\\end{bmatrix},\\; C = \\begin{bmatrix}1&0&0&0\\\\0&1&0&0\\end{bmatrix},\\; D = 0',
          numbers: `A = ${texMat(m.latSS.A)},\\; B = ${texMat(m.latSS.B)}`, spoiler: true },
        { title: 'Back to transfer functions', page: 'p. 85 · Eq. 6.14',
          theory: 'P(s) = C(sI - A)^{-1}B + D',
          numbers: `\\det(sI - A_{lat}) = ${WB.tf.polyTex(L.charPoly(m.latSS.A))}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const m = () => ctx.sys.models(ctx.pModel);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [
        {
          id: 'a', title: '(a) Longitudinal: A<sub>12</sub>, B<sub>2</sub>',
          inputs: { a12: 'A<sub>12</sub>', b2: 'B<sub>2</sub>' },
          check: (v) => PD().checkNumbers(v, { a12: 1, b2: 1 / m().M }, {}),
          solution: () => [{ tex: `A = ${texMat(m().lonSS.A)},\\; B = ${texMat(m().lonSS.B)},\\; C = \\begin{bmatrix}1&0\\end{bmatrix},\\; D = 0` }],
        },
        {
          id: 'b', title: '(b) Lateral: the nonzero entries A<sub>13</sub>, A<sub>24</sub>, A<sub>32</sub>, A<sub>33</sub>, B<sub>4</sub>',
          inputs: { a13: 'A<sub>13</sub>', a24: 'A<sub>24</sub>', a32: 'A<sub>32</sub>', a33: 'A<sub>33</sub>', b4: 'B<sub>4</sub>' },
          check: (v) => { const A = m().latSS.A; return PD().checkNumbers(v, { a13: A[0][2], a24: A[1][3], a32: A[2][1], a33: A[2][2], b4: m().latSS.B[3][0] }, {}); },
          solution: () => [{ tex: `A = ${texMat(m().latSS.A)},\\; B = ${texMat(m().latSS.B)},\\; C = \\begin{bmatrix}1&0&0&0\\\\0&1&0&0\\end{bmatrix},\\; D = 0` }],
        },
      ]);
    },
  }));
})();
