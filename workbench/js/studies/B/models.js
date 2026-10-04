// Design Study B, Chapters 2–6: modeling the pendulum on a cart. Kinetic energy
// (B.2), Euler-Lagrange equations with an energy-balance check (B.3), equilibria
// and Jacobian linearization (B.4), the transfer-function cascade (B.5), and the
// state-space model (B.6). All open loop: the force is a prescribed input.
(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const S = WB.studies.B;
  const PD = () => WB.pd;
  const DEG = Math.PI / 180;

  // ------------------------------------------------------- open-loop force --
  const inputForce = (inp, t) => WB.models.inputTorque({ inp }, t);   // same shapes as Study A

  function inputControls(parent, ctx, { title = 'Input force F(t)', ampMax = 5 } = {}) {
    const names = { zero: 'none', const: 'constant', pulse: 'pulse', square: 'square', sine: 'sine' };
    segmented(parent, {
      label: title,
      options: Object.keys(names).map((s) => ({ value: s, label: names[s] })),
      get: () => ctx.st.inp.shape, set: (v) => { ctx.st.inp.shape = v; ctx.update(); },
    });
    slider(parent, { label: 'amplitude', unit: 'N', min: -ampMax, max: ampMax, step: ampMax / 500, sig: 3, get: () => ctx.st.inp.amp, set: (v) => { ctx.st.inp.amp = v; ctx.update(); }, disabled: () => ctx.st.inp.shape === 'zero' });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.01, max: 3, step: 0.01, sig: 3, get: () => ctx.st.inp.freq, set: (v) => { ctx.st.inp.freq = v; ctx.update(); }, disabled: () => !['square', 'sine'].includes(ctx.st.inp.shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.02, max: 3, step: 0.01, sig: 3, get: () => ctx.st.inp.width, set: (v) => { ctx.st.inp.width = v; ctx.update(); }, disabled: () => ctx.st.inp.shape !== 'pulse' });
  }

  const common = {
    openLoop: true, metrics: false,
    reference: () => () => NaN,   // no reference: keeps the track marker off
    controller(ctx) { return { update: (r, x, y, t) => inputForce(ctx.st.inp, t) }; },
  };

  // Linear overlay ẋ̃ = A x̃ + B F̃ about (z_e, θ_e) with the same input.
  function linearRun(ctx, c, { thetaE = 0, x0, p = ctx.pModel } = {}) {
    const { A, B } = ctx.sys.linearize(p, thetaE);
    const ze = x0[0];
    return WB.sim.simulate({
      ...c, disturbance: null, noise: null, reference: () => 0,
      x0: [0, x0[1] - thetaE, x0[2], x0[3]],
      plant: {
        f: (x, u) => A.map((row, i) => row.reduce((s, a, j) => s + a * x[j], 0) + B[i][0] * u),
        // hide the model once it has left any physically meaningful range
        h: (x) => [ze + x[0], Math.abs(x[1]) > Math.PI ? NaN : thetaE + x[1]],
        uLimit: Infinity,
      },
      controller: { update: (r, x, y, t) => inputForce(ctx.st.inp, t) },
    });
  }

  function stateExtra(res, which) {
    const k = 180 / Math.PI;
    if (which === 'zd') return { opts: { title: 'ż(t)', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [{ label: 'ż', y: res.x.map((x) => x[2]), color: '--series-1' }] } };
    return { opts: { title: 'θ̇(t)', yLabel: 'θ̇ [°/s]', unit: '°/s' }, data: { series: [{ label: 'θ̇', y: res.x.map((x) => x[3] * k), color: '--series-1' }] } };
  }

  // ------------------------------------------------------------- Chapter 2 --
  // Prescribed motion as in hw02_pendulumSim.py; energy is computed, not simulated.
  S.chapters.ch2 = Object.assign({}, common, {
    id: 'ch2', num: 2, tab: 'Ch 2', title: 'Kinetic energy', pages: 'pp. 19–40',
    linear: false,
    defaults() { return { Az: 0.5, fz: 0.1, Ath: 45, fth: 0.5, inp: { shape: 'zero', amp: 0, freq: 0.1 } }; },
    simDefaults(sys) { return sys.problems.ch2.sim; },

    simulate(ctx) {
      const { sys, pModel: p, st, S: St } = ctx;
      const Ts = St.sim.Ts, N = Math.round(St.sim.tEnd / Ts) + 1;
      const F = () => new Float64Array(N);
      const z = F(), th = F(), zero = F(), Kc = F(), Kt = F(), Kr = F(), K = F();
      const x = [];
      const wz = 2 * Math.PI * st.fz, wt = 2 * Math.PI * st.fth, At = st.Ath * DEG;
      for (let k = 0; k < N; k++) {
        const t = k * Ts;
        const xs = [st.Az * Math.sin(wz * t), At * Math.sin(wt * t), st.Az * wz * Math.cos(wz * t), At * wt * Math.cos(wt * t)];
        z[k] = xs[0]; th[k] = xs[1]; x.push(xs);
        Kc[k] = 0.5 * p.m2 * xs[2] ** 2;
        const v1x = xs[2] + p.ell / 2 * xs[3] * Math.cos(xs[1]), v1y = -p.ell / 2 * xs[3] * Math.sin(xs[1]);
        Kt[k] = 0.5 * p.m1 * (v1x ** 2 + v1y ** 2);
        Kr[k] = 0.5 * (p.m1 * p.ell ** 2 / 12) * xs[3] ** 2;
        K[k] = sys.kinetic(xs, p);
      }
      const t = Float64Array.from({ length: N }, (_, k) => k * Ts);
      return { t, yAll: [z, th], yMeasAll: [z, th], rAll: [new Float64Array(N).fill(NaN)], uDemandAll: [zero], uAll: [zero], uAppliedAll: [zero], x, extras: { Kc, Kt, Kr, K } };
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Prescribed motion', 'p. 31 · B.2(b)');
      slider(sec, { label: 'z amplitude', unit: 'm', min: 0, max: 1.5, step: 0.01, sig: 3, get: () => ctx.st.Az, set: (v) => { ctx.st.Az = v; ctx.update(); } });
      slider(sec, { label: 'z frequency', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, get: () => ctx.st.fz, set: (v) => { ctx.st.fz = v; ctx.update(); } });
      slider(sec, { label: 'θ amplitude', unit: '°', min: 0, max: 90, step: 1, sig: 3, get: () => ctx.st.Ath, set: (v) => { ctx.st.Ath = v; ctx.update(); } });
      slider(sec, { label: 'θ frequency', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, get: () => ctx.st.fth, set: (v) => { ctx.st.fth = v; ctx.update(); } });
      sec.append(el('p', { class: 'muted small', text: 'z(t) = A_z sin(2πf_z t) and θ(t) = A_θ sin(2πf_θ t). No dynamics here: the motion is imposed, as in hw02_pendulumSim.py (which uses a square wave for θ). The plot below splits K into the cart, the rod\'s translation, and the rod\'s rotation.' }));
    },

    extraPlot(ctx, res) {
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [J]', unit: 'J' },
        data: { series: [
          { label: 'cart ½m₂ż²', y: Array.from(res.extras.Kc), color: '--series-2', width: 1.5 },
          { label: 'rod translation ½m₁‖v₁‖²', y: Array.from(res.extras.Kt), color: '--series-3', width: 1.5 },
          { label: 'rod rotation ½J_cm θ̇²', y: Array.from(res.extras.Kr), color: '--text-muted', width: 1.5 },
          { label: 'total K (Eq. 2.4)', y: Array.from(res.extras.K), color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Kinetic energy of rigid bodies', page: 'p. 32',
          theory: 'K = \\tfrac12 m_1\\mathbf v_1^\\top\\mathbf v_1 + \\tfrac12\\boldsymbol\\omega^\\top J_{rod,cm}\\boldsymbol\\omega + \\tfrac12 m_2\\mathbf v_2^\\top\\mathbf v_2' },
        { title: 'Positions and velocities', page: 'p. 31–32 · Fig. 2-9',
          theory: '\\mathbf p_1 = \\begin{pmatrix} z + \\frac{\\ell}{2}\\sin\\theta\\\\ \\frac{\\ell}{2}\\cos\\theta\\\\ 0\\end{pmatrix},\\quad \\mathbf v_1 = \\begin{pmatrix}\\dot z + \\frac{\\ell}{2}\\dot\\theta\\cos\\theta\\\\ -\\frac{\\ell}{2}\\dot\\theta\\sin\\theta\\\\ 0\\end{pmatrix},\\quad \\mathbf v_2 = \\begin{pmatrix}\\dot z\\\\0\\\\0\\end{pmatrix}' },
        { title: 'Thin rod about its center', page: 'p. 32',
          theory: '\\tfrac12\\boldsymbol\\omega^\\top J_{rod,cm}\\boldsymbol\\omega = \\tfrac12\\,\\frac{m_1\\ell^2}{12}\\,\\dot\\theta^2' },
        { title: 'Result', page: 'p. 32 · Eq. 2.4',
          theory: 'K = \\tfrac12 m_1\\|\\mathbf v_1\\|^2 + \\tfrac12\\frac{m_1\\ell^2}{12}\\dot\\theta^2 + \\tfrac12 m_2\\dot z^2 \\;\\Rightarrow\\; \\text{expand and collect}',
          symbolic: 'K = \\tfrac12(m_1 + m_2)\\dot z^2 + \\tfrac12 m_1\\frac{\\ell^2}{3}\\dot\\theta^2 + m_1\\frac{\\ell}{2}\\dot z\\dot\\theta\\cos\\theta',
          numbers: `K = ${tex((p.m1 + p.m2) / 2)}\\,\\dot z^2 + ${tex(p.m1 * p.ell ** 2 / 6)}\\,\\dot\\theta^2 + ${tex(p.m1 * p.ell / 2)}\\,\\dot z\\dot\\theta\\cos\\theta`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) K = a ż² + c θ̇² + e żθ̇ cos θ',
          inputs: { a: 'a [kg]', c: 'c [kg·m²]', e: 'e [kg·m]' },
          check: (v) => PD().checkNumbers(v, { a: (p().m1 + p().m2) / 2, c: p().m1 * p().ell ** 2 / 6, e: p().m1 * p().ell / 2 }, {}),
          solution: () => [
            { tex: `a = \\tfrac12(m_1+m_2) = ${tex((p().m1 + p().m2) / 2)},\\quad c = \\tfrac{m_1\\ell^2}{6} = ${tex(p().m1 * p().ell ** 2 / 6)},\\quad e = \\tfrac{m_1\\ell}{2} = ${tex(p().m1 * p().ell / 2)}` },
            { html: 'The rod\'s ½m₁(ℓ/2)²θ̇² translation term and its ½(m₁ℓ²/12)θ̇² rotation term add to ½(m₁ℓ²/3)θ̇². The cross term comes from v₁ mixing ż and θ̇ (p. 32).' },
          ],
        },
        {
          id: 'b', title: '(b) Animation',
          html: 'The animation above draws the cart, the rod, and the bob from (z, θ), like pendulumAnimation.py (Listing 2.3, p. 33–34). Change the prescribed motion to see it move.',
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 3 --
  S.chapters.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 41–56',
    linear: false,
    defaults(sys) { const o = sys.problems.ch3.openLoop; return { inp: { shape: o.shape, amp: o.amp, freq: o.freq, width: 0.5 }, extra: 'energy' }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 47 · B.3(e)');
      inputControls(sec, ctx);
      segmented(sec, {
        label: 'Extra plot',
        options: [{ value: 'energy', label: 'energy balance' }, { value: 'zd', label: 'ż' }, { value: 'thd', label: 'θ̇' }],
        get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); },
      });
      sec.append(el('p', { class: 'muted small', text: 'hw03_pendulumSim.py pushes the cart with F = sin(2πt) N. Nothing holds the pendulum up, so it falls and swings under the track. The energy plot checks the EOM: E(t) − E(0) must equal the work done by F minus the friction loss.' }));
    },

    extraPlot(ctx, res) {
      if (ctx.st.extra !== 'energy') return stateExtra(res, ctx.st.extra);
      const { sys, pModel: p } = ctx;
      const n = res.t.length, Ts = ctx.S.sim.Ts;
      const E = new Float64Array(n), W = new Float64Array(n);
      const E0 = sys.kinetic(res.x[0], p) + sys.potential(res.x[0], p);
      let w = 0;
      const pw = (xx, u) => (u - p.b * xx[2]) * xx[2];
      for (let k = 0; k < n; k++) {
        const x = res.x[k];
        E[k] = sys.kinetic(x, p) + sys.potential(x, p) - E0;
        if (k > 0) w += 0.5 * Ts * (pw(res.x[k - 1], res.uApplied[k - 1]) + pw(x, res.uApplied[k - 1]));
        W[k] = w;
      }
      return {
        opts: { title: 'energy balance', yLabel: 'energy [J]', unit: 'J' },
        data: { series: [
          { label: '∫(F − bż)ż dt', y: W, color: '--series-2', dash: [5, 4], width: 2 },
          { label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Potential energy', page: 'p. 47 · Fig. 3-4',
          theory: 'P = P_0 + m_1 g\\,h_{cm}(\\theta),\\quad h_{cm} = \\text{height of the rod’s center of mass above } \\theta = 0',
          symbolic: 'P = P_0 + m_1 g\\frac{\\ell}{2}(\\cos\\theta - 1)',
          numbers: `P - P_0 = ${tex(p.m1 * p.g * p.ell / 2)}\\,(\\cos\\theta - 1)`, spoiler: true },
        { title: 'Coordinates, forces, damping', page: 'p. 47',
          theory: 'q = \\text{configuration variables},\\quad \\tau = \\text{generalized forces along } q,\\quad -B\\dot q = \\text{damping}',
          symbolic: 'q = (z, \\theta)^\\top,\\quad \\tau = (F, 0)^\\top,\\quad -B\\dot q = (-b\\dot z, 0)^\\top', spoiler: true },
        { title: 'Euler-Lagrange equations', page: 'p. 48',
          theory: 'L = K - P,\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Equations of motion', page: 'p. 49 · Eq. 3.2',
          theory: 'M(q)\\,\\ddot q = \\text{(velocity, gravity, force and damping terms)}',
          symbolic: '\\begin{pmatrix} m_1 + m_2 & m_1\\frac{\\ell}{2}\\cos\\theta\\\\ m_1\\frac{\\ell}{2}\\cos\\theta & m_1\\frac{\\ell^2}{3}\\end{pmatrix}\\begin{pmatrix}\\ddot z\\\\ \\ddot\\theta\\end{pmatrix} = \\begin{pmatrix} m_1\\frac{\\ell}{2}\\dot\\theta^2\\sin\\theta + F - b\\dot z\\\\ m_1 g\\frac{\\ell}{2}\\sin\\theta\\end{pmatrix}',
          numbers: `\\begin{pmatrix} ${tex(p.m1 + p.m2)} & ${tex(p.m1 * p.ell / 2)}\\cos\\theta\\\\ ${tex(p.m1 * p.ell / 2)}\\cos\\theta & ${tex(p.m1 * p.ell ** 2 / 3)}\\end{pmatrix}\\ddot q = \\begin{pmatrix} ${tex(p.m1 * p.ell / 2)}\\,\\dot\\theta^2\\sin\\theta + F - ${tex(p.b)}\\,\\dot z\\\\ ${tex(p.m1 * p.g * p.ell / 2)}\\sin\\theta\\end{pmatrix}`, spoiler: true },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from Eq. 3.2',
          theory: '\\frac{d}{dt}(K + P) = (F - b\\dot z)\\,\\dot z' },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch3, [
        {
          id: 'a', title: '(a) P = P<sub>0</sub> + c (cos θ − 1)',
          inputs: { c: 'c [J]' },
          check: (v) => PD().checkNumbers(v, { c: p().m1 * p().g * p().ell / 2 }, {}),
          solution: () => [{ tex: `c = m_1 g\\tfrac{\\ell}{2} = ${tex(p().m1 * p().g * p().ell / 2)}` }, { html: 'ℓ/2 cos θ is the height of the rod\'s center of mass, so P decreases as θ grows (p. 47).' }],
        },
        {
          id: 'c', title: '(b, c) Generalized forces τ = (τ<sub>1</sub>, τ<sub>2</sub>) and damping −Bq̇ with B = diag(b<sub>1</sub>, b<sub>2</sub>)',
          html: 'q = (z, θ). Enter τ for F = 1 N.',
          inputs: { t1: 'τ<sub>1</sub>', t2: 'τ<sub>2</sub>', b1: 'b<sub>1</sub>', b2: 'b<sub>2</sub>' },
          check: (v) => PD().checkNumbers(v, { t1: 1, t2: 0, b1: p().b, b2: 0 }, {}),
          solution: () => [{ tex: '\\tau = (F, 0)^\\top,\\quad -B\\dot q = (-b\\dot z, 0)^\\top' }],
        },
        {
          id: 'd', title: '(d) M(q) q̈ = c(q, q̇): M<sub>11</sub>, M<sub>12</sub> = k cos θ, M<sub>22</sub>, and the gravity term g<sub>2</sub> sin θ',
          inputs: { m11: 'M<sub>11</sub>', k: 'k', m22: 'M<sub>22</sub>', g2: 'g<sub>2</sub>' },
          check: (v) => PD().checkNumbers(v, { m11: p().m1 + p().m2, k: p().m1 * p().ell / 2, m22: p().m1 * p().ell ** 2 / 3, g2: p().m1 * p().g * p().ell / 2 }, {}),
          solution: () => [
            { tex: `M_{11} = m_1 + m_2 = ${tex(p().m1 + p().m2)},\\; k = m_1\\tfrac{\\ell}{2} = ${tex(p().m1 * p().ell / 2)},\\; M_{22} = m_1\\tfrac{\\ell^2}{3} = ${tex(p().m1 * p().ell ** 2 / 3)},\\; g_2 = m_1 g\\tfrac{\\ell}{2} = ${tex(p().m1 * p().g * p().ell / 2)}` },
            { html: 'Eq. 3.2 (p. 49). The energy plot is a numerical check: with F = 0 and b = 0, E(t) stays constant.' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 4 --
  S.chapters.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68',
    defaults() { return { thetaE: 0, dth0: 2, inp: { shape: 'zero', amp: 0.2, freq: 0.5, width: 0.2 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    linearLabel: 'Jacobian-linearized model',

    x0(ctx) { return [ctx.S.sim.y0, (ctx.st.thetaE + ctx.st.dth0) * DEG, 0, 0]; },
    simulate(ctx, c, plant) {
      return WB.sim.simulate({ ...c, x0: this.x0(ctx), plant, controller: this.controller(ctx) });
    },
    linearSim(ctx, c) { return linearRun(ctx, c, { thetaE: ctx.st.thetaE * DEG, x0: this.x0(ctx) }); },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Operating point', 'p. 65 · Eq. 4.10–4.11');
      segmented(sec, {
        label: 'Equilibrium θ<sub>e</sub> = kπ',
        options: [{ value: 0, label: 'k even: upright (0°)' }, { value: 180, label: 'k odd: hanging (180°)' }],
        get: () => ctx.st.thetaE, set: (v) => { ctx.st.thetaE = v; ctx.update(); },
      });
      slider(sec, { label: 'δθ(0)', unit: '°', min: -30, max: 30, step: 0.5, sig: 3, hint: 'initial offset from θₑ (z(0) is in the left panel)', get: () => ctx.st.dth0, set: (v) => { ctx.st.dth0 = v; ctx.update(); } });
      const inp = section(parent, 'Input F̃(t) (F_e = 0)', 'p. 66');
      inputControls(inp, ctx, { title: 'Input force F̃(t)', ampMax: 2 });
      inp.append(el('p', { class: 'muted small', text: 'Upright, a 2° tilt grows like e^{4.2t}: the linear model tracks the nonlinear one for a fraction of a second, then they part. Hanging, both oscillate and agree for small angles.' }));
    },

    outputSeries(ctx, res, sc, oi) {
      return oi === 1 ? [{ label: 'θₑ', y: Array.from(res.t, () => ctx.st.thetaE * DEG * (180 / Math.PI)), color: '--ref', dash: [6, 4], width: 1.5 }] : [];
    },

    splane(ctx) {
      const { A } = ctx.sys.linearize(ctx.pModel, ctx.st.thetaE * DEG);
      return { markers: L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `eigenvalue of A ${i + 1}` })) };
    },

    math(ctx) {
      const p = ctx.pModel, lin = ctx.sys.linearize(p, ctx.st.thetaE * DEG);
      return [
        { title: 'Equations of motion', page: 'p. 65 · Eq. 4.9',
          theory: '(m_1 + m_2)\\ddot z + m_1\\frac{\\ell}{2}\\ddot\\theta\\cos\\theta = m_1\\frac{\\ell}{2}\\dot\\theta^2\\sin\\theta - b\\dot z + F,\\quad m_1\\frac{\\ell}{2}\\ddot z\\cos\\theta + m_1\\frac{\\ell^2}{3}\\ddot\\theta = m_1 g\\frac{\\ell}{2}\\sin\\theta' },
        { title: 'Equilibria', page: 'p. 65 · Eq. 4.10–4.11',
          theory: '\\text{set } \\dot z = \\ddot z = \\dot\\theta = \\ddot\\theta = 0 \\text{ in Eq. 4.9}',
          symbolic: '\\dot z = \\ddot z = \\dot\\theta = \\ddot\\theta = 0 \\;\\Rightarrow\\; F_e = 0,\\quad m_1 g\\frac{\\ell}{2}\\sin\\theta_e = 0 \\Rightarrow \\theta_e = k\\pi,\\; z_e \\text{ arbitrary}', spoiler: true,
          numbers: 'F_e = 0,\\quad \\theta_e \\in \\{0, \\pm\\pi, \\dots\\}' },
        { title: 'Small-angle terms about θₑ = 0', page: 'p. 66',
          theory: '\\ddot\\theta\\cos\\theta \\approx \\ddot{\\tilde\\theta},\\quad \\dot\\theta^2\\sin\\theta \\approx 0,\\quad \\ddot z\\cos\\theta \\approx \\ddot{\\tilde z},\\quad \\sin\\theta \\approx \\tilde\\theta' },
        { title: 'Linearized equations', page: 'p. 66 · Eq. 4.12',
          theory: '\\text{substitute the small-angle terms into Eq. 4.9, drop products of small quantities}',
          symbolic: '\\begin{pmatrix} m_1 + m_2 & m_1\\frac{\\ell}{2}\\\\ m_1\\frac{\\ell}{2} & m_1\\frac{\\ell^2}{3}\\end{pmatrix}\\begin{pmatrix}\\ddot{\\tilde z}\\\\ \\ddot{\\tilde\\theta}\\end{pmatrix} = \\begin{pmatrix} -b\\dot{\\tilde z} + \\tilde F\\\\ m_1 g\\frac{\\ell}{2}\\tilde\\theta\\end{pmatrix}',
          numbers: `A(\\theta_e = ${ctx.st.thetaE}^\\circ) = ${texMat(lin.A)},\\quad \\text{eig}(A) = ${L.eig(lin.A).map((q) => texPole(q)).join(',\\;')}`, spoiler: true,
          note: 'For odd k, cos θₑ = −1 flips the sign of the coupling and gravity terms: the eigenvalues move from ±4.2 to ±4.2j (with ℓ = 1 m).' },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibria',
          html: 'Enter F<sub>e</sub>, and the two smallest non-negative θ<sub>e</sub> in degrees.',
          inputs: { Fe: 'F<sub>e</sub> [N]', t1: 'θ<sub>e,1</sub> [°]', t2: 'θ<sub>e,2</sub> [°]' },
          check: (v) => {
            const a = [PD().num(v.t1), PD().num(v.t2)].sort((x, y) => x - y);
            if (a.some((x) => x === null) || PD().num(v.Fe) === null) return { ok: false, msg: 'Enter all three.' };
            return Math.abs(PD().num(v.Fe)) < 1e-6 && Math.abs(a[0]) < 1e-6 && Math.abs(a[1] - 180) < 1e-6 ? { ok: true, msg: 'z_e is arbitrary.' } : { ok: false, msg: 'Set the velocities and accelerations to zero in Eq. 4.9.' };
          },
          solution: () => [{ tex: 'F_e = 0,\\quad m_1 g\\tfrac{\\ell}{2}\\sin\\theta_e = 0 \\Rightarrow \\theta_e = k\\pi,\\quad z_e\\text{ arbitrary (Eq. 4.10–4.11)}' }],
        },
        {
          id: 'b', title: '(b) Eq. 4.12 about θ<sub>e</sub> = 0: matrix entries and the gravity coefficient k<sub>θ</sub> in m<sub>1</sub>g(ℓ/2)θ̃',
          inputs: { m11: 'M<sub>11</sub>', m12: 'M<sub>12</sub>', m22: 'M<sub>22</sub>', kth: 'k<sub>θ</sub>' },
          check: (v) => PD().checkNumbers(v, { m11: p().m1 + p().m2, m12: p().m1 * p().ell / 2, m22: p().m1 * p().ell ** 2 / 3, kth: p().m1 * p().g * p().ell / 2 }, {}),
          solution: () => [
            { tex: `\\begin{pmatrix} ${tex(p().m1 + p().m2)} & ${tex(p().m1 * p().ell / 2)}\\\\ ${tex(p().m1 * p().ell / 2)} & ${tex(p().m1 * p().ell ** 2 / 3)}\\end{pmatrix}\\ddot{\\tilde q} = \\begin{pmatrix} -${tex(p().b)}\\dot{\\tilde z} + \\tilde F\\\\ ${tex(p().m1 * p().g * p().ell / 2)}\\,\\tilde\\theta\\end{pmatrix}` },
            { html: 'cos θₑ = 1 and sin θₑ = 0, and θ̇² sin θ is second order in small quantities (p. 66).' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------- Chapters 5 and 6 --
  function tfMarkers(ctx) {
    const pin = ctx.sys.inner(ctx.pModel);
    const q = Math.sqrt(3 * ctx.pModel.g / (2 * ctx.pModel.ell));
    const mk = M.roots2(0, pin.a0).map((p, i) => ({ ...p, kind: 'ol', label: `pole of Θ̃/F̃ (b = 0) ${i + 1}` }));
    mk.push({ re: 0, im: 0, kind: 'ol', label: 'double pole of Z̃/Θ̃ at 0' });
    mk.push({ re: q, im: 0, kind: 'olzero', label: 'zero of Z̃/Θ̃ (RHP)' }, { re: -q, im: 0, kind: 'olzero', label: 'zero of Z̃/Θ̃ (LHP)' });
    return { markers: mk };
  }
  const olSim = (lin) => ({
    simulate(ctx, c, plant) { return WB.sim.simulate({ ...c, plant, controller: this.controller(ctx) }); },
    linearSim(ctx, c) {
      const x0 = [ctx.S.sim.y0, (ctx.S.sim.init.theta0 || 0) * DEG, 0, 0];
      const p = lin === 'b0' && ctx.st.b0 ? { ...ctx.pModel, b: 0 } : ctx.pModel;
      return linearRun(ctx, c, { x0, p });
    },
  });
  const pulseSection = (parent, ctx, title, page) => {
    const inp = section(parent, title, page);
    inputControls(inp, ctx, { title: 'Input force F̃(t)', ampMax: 2 });
    inp.append(el('p', { class: 'muted small', text: 'Open loop the upright pendulum is unstable, so even a small push tips it over within a second or two. The dashed trace is the linear model: it matches while θ is small, then the nonlinear pendulum swings down and the linear one runs away (it is hidden past ±180°).' }));
  };

  S.chapters.ch5 = Object.assign({}, common, olSim('b0'), {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 69–80',
    defaults() { return { b0: true, inp: { shape: 'pulse', amp: 0.2, freq: 0.5, width: 0.1 } }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearLabel: 'transfer-function model',
    buildControls(parent, ctx) {
      pulseSection(parent, ctx, 'Input F̃(t)', 'p. 75');
      const md = section(parent, 'Linear model', 'p. 76');
      segmented(md, { label: 'Damping in the dashed model', options: [{ value: true, label: 'b = 0 (B.5b)' }, { value: false, label: 'with b' }], get: () => ctx.st.b0, set: (v) => { ctx.st.b0 = v; ctx.update(); } });
    },
    splane: tfMarkers,
    math(ctx) {
      const p = ctx.pModel, pin = ctx.sys.inner(p), J = pin.J;
      const q = Math.sqrt(3 * p.g / (2 * p.ell));
      return [
        { title: 'Laplace transform of Eq. 4.12', page: 'p. 75–76',
          theory: '\\mathcal L\\{\\ddot{\\tilde z}\\} = s^2\\tilde Z(s) \\text{ (zero initial conditions)}',
          symbolic: '\\begin{pmatrix}(m_1+m_2)s^2 + bs & m_1\\frac{\\ell}{2}s^2\\\\ s^2 & \\frac{2\\ell}{3}s^2 - g\\end{pmatrix}\\begin{pmatrix}\\tilde Z(s)\\\\ \\tilde\\Theta(s)\\end{pmatrix} = \\begin{pmatrix}\\tilde F(s)\\\\ 0\\end{pmatrix}', spoiler: true },
        { title: 'Transfer functions', page: 'p. 76',
          theory: '\\text{invert the } 2\\times2 \\text{ matrix (App. P.7)}',
          symbolic: '\\tilde Z = \\frac{\\frac{2\\ell}{3}s^2 - g}{(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^4 + b\\frac{2\\ell}{3}s^3 - (m_1+m_2)gs^2 - bgs}\\tilde F,\\quad \\tilde\\Theta = \\frac{-s^2}{(\\cdots)}\\tilde F', spoiler: true },
        { title: 'With b = 0', page: 'p. 76',
          theory: 'b \\approx 0 \\text{ removes the } s^3 \\text{ and } s \\text{ terms}',
          symbolic: '\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{-1}{(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^2 - (m_1+m_2)g},\\quad \\frac{\\tilde Z}{\\tilde F} = \\frac{\\frac{2\\ell}{3}s^2 - g}{s^2\\big[(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^2 - (m_1+m_2)g\\big]}',
          numbers: `\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{${tex(pin.b0)}}{s^2 ${tex(pin.a0).startsWith('-') ? '' : '+'}${tex(pin.a0)}},\\quad \\text{poles } \\pm${tex(Math.sqrt(-pin.a0))}`, spoiler: true,
          note: 'b ż is small next to the other forces, and ignoring it is conservative: all the damping must then come from the controller (p. 76).' },
        { title: 'Cascade', page: 'p. 76 · Fig. 5-2',
          theory: '\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{\\tilde Z/\\tilde F}{\\tilde\\Theta/\\tilde F}',
          symbolic: '\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{\\tilde Z/\\tilde F}{\\tilde\\Theta/\\tilde F} = \\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2}',
          numbers: `\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{${tex(-2 * p.ell / 3)}\\,s^2 + ${tex(p.g)}}{s^2},\\quad \\text{zeros } \\pm${tex(q)}`, spoiler: true,
          note: `A push in +z tips the rod toward −θ (the minus sign); the rod's tilt then drives the cart like a double integrator.` },
      ];
    },
    buildProblem(parent, ctx) {
      const p = () => ctx.pModel, pin = () => ctx.sys.inner(ctx.pModel);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'b', title: '(b) With b = 0: Θ̃/F̃ = −1/(a s² − c)',
          inputs: { a: 'a', c: 'c', p: 'unstable pole' },
          check: (v) => PD().checkNumbers(v, { a: pin().J, c: (p().m1 + p().m2) * p().g, p: Math.sqrt(-pin().a0) }, {}),
          solution: () => [
            { tex: `a = m_1\\tfrac{\\ell}{6} + m_2\\tfrac{2\\ell}{3} = ${tex(pin().J)},\\quad c = (m_1+m_2)g = ${tex((p().m1 + p().m2) * p().g)},\\quad p = \\sqrt{c/a} = ${tex(Math.sqrt(-pin().a0))}` },
            { html: 'Setting b = 0 removes the s³ and s terms, and the s² factor cancels in Θ̃/F̃ (p. 76). It is reasonable because bż is small, and conservative because it hands all the damping to the controller.' },
          ],
        },
        {
          id: 'c', title: '(c) Z̃/Θ̃ = (n<sub>2</sub>s² + n<sub>0</sub>)/s²',
          inputs: { n2: 'n<sub>2</sub>', n0: 'n<sub>0</sub>' },
          check: (v) => PD().checkNumbers(v, { n2: -2 * p().ell / 3, n0: p().g }, {}),
          solution: () => [{ tex: `\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{${tex(-2 * p().ell / 3)}s^2 + ${tex(p().g)}}{s^2}` }, { html: 'Block diagram: F̃ → [Θ̃/F̃] → Θ̃ → [Z̃/Θ̃] → Z̃ (Fig. 5-2, p. 76).' }],
        },
        {
          id: 'd', title: '(d) Physical interpretation',
          html: 'Try it: apply a small positive pulse on F and watch the signs of θ and z, and how fast θ grows.',
          solution: () => [{ html: 'A positive force on the cart makes the pendulum fall the other way (the minus sign). It falls in an unstable motion (the right-half-plane pole of Θ̃/F̃). The tilt then accelerates the cart, which is why Z̃/Θ̃ has a double integrator (p. 76–77).' }],
        },
      ]);
    },
  });

  S.chapters.ch6 = Object.assign({}, common, olSim('full'), {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 81–93',
    defaults() { return { extra: 'thd', inp: { shape: 'pulse', amp: 0.2, freq: 0.5, width: 0.1 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearLabel: 'ẋ = Ax + Bu model',
    buildControls(parent, ctx) {
      pulseSection(parent, ctx, 'Input ũ = F̃(t)', 'p. 89');
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'zd', label: 'x₃ = ż' }, { value: 'thd', label: 'x₄ = θ̇' }], get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); } });
    },
    splane(ctx) {
      const { A } = ctx.sys.stateSpace(ctx.pModel);
      return { markers: L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `eigenvalue of A ${i + 1}` })) };
    },
    extraPlot(ctx, res) { return stateExtra(res, ctx.st.extra); },
    math(ctx) {
      const { A, B } = ctx.sys.stateSpace(ctx.pModel);
      return [
        { title: 'Solve Eq. 4.12 for the accelerations', page: 'p. 89',
          theory: '\\begin{pmatrix}\\ddot{\\tilde z}\\\\ \\ddot{\\tilde\\theta}\\end{pmatrix} = M^{-1}\\begin{pmatrix}-b\\dot{\\tilde z} + \\tilde F\\\\ m_1 g\\frac{\\ell}{2}\\tilde\\theta\\end{pmatrix},\\quad M = \\begin{pmatrix} m_1 + m_2 & m_1\\frac{\\ell}{2}\\\\ m_1\\frac{\\ell}{2} & m_1\\frac{\\ell^2}{3}\\end{pmatrix}',
          symbolic: '\\begin{pmatrix}\\ddot{\\tilde z}\\\\ \\ddot{\\tilde\\theta}\\end{pmatrix} = \\begin{pmatrix} -\\frac{b}{\\frac14 m_1 + m_2}\\dot{\\tilde z} + \\frac{1}{\\frac14 m_1 + m_2}\\tilde F - \\frac{\\frac34 m_1 g}{\\frac14 m_1 + m_2}\\tilde\\theta\\\\ \\frac{3b}{2(\\frac14 m_1 + m_2)\\ell}\\dot{\\tilde z} - \\frac{3}{2(\\frac14 m_1 + m_2)\\ell}\\tilde F + \\frac{3(m_1+m_2)g}{2(\\frac14 m_1 + m_2)\\ell}\\tilde\\theta\\end{pmatrix}', spoiler: true },
        { title: 'State-space model', page: 'p. 90 · Eq. 6.17',
          theory: '\\dot{\\tilde x} = A\\tilde x + B\\tilde u,\\quad \\tilde y = \\begin{pmatrix}1&0&0&0\\\\0&1&0&0\\end{pmatrix}\\tilde x,\\quad D = 0',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`, spoiler: true,
          note: 'The B.11–B.12 solutions print row 4 of A and B for a different ℓ than the stated one (ISSUES.md).' },
        { title: 'Back to transfer functions', page: 'p. 85 · Eq. 6.14',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\det(sI - A) = 0 \\text{ gives the poles}',
          numbers: `\\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const ss = () => ctx.sys.stateSpace(ctx.pModel);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'Nonzero entries of A and B (rows 3 and 4)',
        inputs: { a32: 'A<sub>32</sub>', a33: 'A<sub>33</sub>', a42: 'A<sub>42</sub>', a43: 'A<sub>43</sub>', b3: 'B<sub>3</sub>', b4: 'B<sub>4</sub>' },
        check: (v) => { const { A, B } = ss(); return PD().checkNumbers(v, { a32: A[2][1], a33: A[2][2], a42: A[3][1], a43: A[3][2], b3: B[2][0], b4: B[3][0] }, {}); },
        solution: () => { const { A, B } = ss(); return [{ tex: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = \\begin{pmatrix}1&0&0&0\\\\0&1&0&0\\end{pmatrix},\\quad D = 0` }, { html: 'Eq. 6.17 (p. 90). The rows 1–2 just say ż̃ = x₃ and θ̇̃ = x₄.' }]; },
      }]);
    },
  });

  S.models = { inputForce, inputControls, linearRun };
})();
