// Study E, Chapters 2–6 (E.2–E.6): kinetic energy, Euler-Lagrange equations,
// equilibria and linearization, transfer functions, and the state-space model.
// Open-loop experiments on the block and beam. The open-loop system is unstable
// (eig(A) = ±λ, ±jλ), so these runs are short.
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.E = WB.studies.E || { chapters: {} };

(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const E = () => WB.E;
  const PD = () => WB.pd;
  const CH = WB.studies.E.chapters;

  // ---------------------------------------------------- open-loop force input --
  const inputForce = (st, t) => WB.models.inputTorque(st, t);   // same shapes as Study A

  // F = F_comp + F_in(t), with F_comp = 0, F_e (at z_e) or F_fl(z).
  function openLoop(ctx, { linear = false } = {}) {
    const { sys, pModel } = ctx;
    const st = ctx.st;
    return {
      update(r, x, yMeas, t) {
        const fin = inputForce(st, t);
        if (linear || st.comp === 'none') return { u: fin, Fin: fin };
        const z0 = st.zE !== undefined ? st.zE : sys.ze(pModel);
        const ff = st.comp === 'eq' ? sys.Fe(pModel, z0) : sys.Ffl(x[0], pModel);
        return { u: ff + fin, Fin: fin };
      },
    };
  }

  function inputControls(parent, ctx, { comps } = {}) {
    const names = { zero: 'none', const: 'constant', pulse: 'pulse', square: 'square', sine: 'sine' };
    segmented(parent, {
      label: 'Input force F<sub>in</sub>(t)',
      options: ['zero', 'const', 'pulse', 'square', 'sine'].map((s) => ({ value: s, label: names[s] })),
      get: () => ctx.st.inp.shape, set: (v) => { ctx.st.inp.shape = v; ctx.update(); },
    });
    slider(parent, { label: 'amplitude', unit: 'N', min: -3, max: 3, step: 0.01, sig: 3, get: () => ctx.st.inp.amp, set: (v) => { ctx.st.inp.amp = v; ctx.update(); }, disabled: () => ctx.st.inp.shape === 'zero' });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.05, max: 5, step: 0.01, sig: 3, get: () => ctx.st.inp.freq, set: (v) => { ctx.st.inp.freq = v; ctx.update(); }, disabled: () => !['square', 'sine'].includes(ctx.st.inp.shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.02, max: 2, step: 0.01, sig: 3, get: () => ctx.st.inp.width, set: (v) => { ctx.st.inp.width = v; ctx.update(); }, disabled: () => ctx.st.inp.shape !== 'pulse' });
    if (comps) {
      segmented(parent, {
        label: 'Added to F<sub>in</sub>',
        options: comps,
        get: () => ctx.st.comp, set: (v) => { ctx.st.comp = v; ctx.update(); },
      });
    }
  }
  const COMPS = [
    { value: 'none', label: 'nothing' },
    { value: 'eq', label: 'F<sub>e</sub> (at z<sub>e</sub>)', title: 'constant equilibrium force' },
    { value: 'fl', label: 'F<sub>fl</sub>(z)', title: 'feedback linearization with the measured z (E.8e)' },
  ];

  const common = {
    openLoop: true, metrics: false,
    controller(ctx, o) { return openLoop(ctx, o); },
  };

  // Hand-built result with per-channel arrays (for prescribed-motion runs).
  function blankResult(N, nOut) {
    const F = () => new Float64Array(N);
    return { t: F(), yAll: Array.from({ length: nOut }, F), yMeasAll: null, rAll: [F()], uDemandAll: [F()], uAll: [F()], uAppliedAll: [F()], x: [], extras: {} };
  }

  // Linear open-loop overlay x̃̇ = A x̃ + B F_in about (z_e, 0, 0, 0).
  function linearOpenLoop(ctx, c, A, B, x0, zE) {
    const plant = WB.design.linearPlant(A, B, [[1, 0, 0, 0], [0, 1, 0, 0]], { xe: [zE, 0, 0, 0] });
    return WB.sim.simulate({ ...c, x0, disturbance: null, noise: null, plant, controller: { update: (r, x, y, t) => inputForce(ctx.st, t) } });
  }

  // ------------------------------------------------------------- Chapter 2 --
  // Prescribed motion z(t), θ(t) (as hw02 does); the energy is computed, not simulated.
  CH.ch2 = Object.assign({}, common, {
    id: 'ch2', num: 2, tab: 'Ch 2', title: 'Kinetic energy', pages: 'pp. 19–40',
    linear: false,
    defaults() { return { Az: 0.15, Ath: 10, f: 0.2, inp: { shape: 'zero', amp: 0, freq: 0.1 } }; },
    simDefaults(sys) { return sys.problems.ch2.sim; },

    simulate(ctx) {
      const { sys, pModel, st, S } = ctx;
      const Ts = S.sim.Ts, N = Math.round(S.sim.tEnd / Ts) + 1;
      const w = 2 * Math.PI * st.f, Ath = st.Ath * M.DEG, z0 = sys.ze(pModel);
      const res = blankResult(N, 2);
      res.extras = { Kr: new Float64Array(N), Kt: new Float64Array(N), Kb: new Float64Array(N), K: new Float64Array(N) };
      for (let k = 0; k < N; k++) {
        const t = k * Ts;
        const z = z0 + st.Az * Math.sin(w * t), zd = st.Az * w * Math.cos(w * t);
        const th = Ath * Math.sin(0.5 * w * t), thd = 0.5 * w * Ath * Math.cos(0.5 * w * t);
        const x = [z, th, zd, thd];
        res.t[k] = t; res.yAll[0][k] = z; res.yAll[1][k] = th; res.rAll[0][k] = z;
        res.x.push(x);
        res.extras.Kr[k] = 0.5 * pModel.m1 * zd * zd;                          // along the beam
        res.extras.Kt[k] = 0.5 * pModel.m1 * z * z * thd * thd;                // swinging with the beam
        res.extras.Kb[k] = 0.5 * (pModel.m2 * pModel.ell ** 2 / 3) * thd * thd; // beam about the pivot
        res.extras.K[k] = sys.kinetic(x, pModel);
      }
      res.yMeasAll = res.yAll;
      return res;
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Prescribed motion', 'p. 386 · E.2(b)');
      slider(sec, { label: 'z amplitude', unit: 'm', min: 0, max: 0.25, step: 0.005, sig: 3, get: () => ctx.st.Az, set: (v) => { ctx.st.Az = v; ctx.update(); } });
      slider(sec, { label: 'θ amplitude', unit: '°', min: 0, max: 45, step: 0.5, sig: 3, get: () => ctx.st.Ath, set: (v) => { ctx.st.Ath = v; ctx.update(); } });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, get: () => ctx.st.f, set: (v) => { ctx.st.f = v; ctx.update(); } });
      sec.append(el('p', { class: 'muted small', text: 'z(t) = z_e + A_z sin(2πft), θ(t) = A_θ sin(πft). No dynamics: the motion is imposed. The energy plot splits K into three parts: watch which ones depend on z.' }));
    },

    extraPlot(ctx, res) {
      const mJ = (a) => Array.from(a, (v) => v * 1000);
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [mJ]', unit: 'mJ' },
        data: { series: [
          { label: 'block, along the beam', y: mJ(res.extras.Kr), color: '--series-2', width: 1.5 },
          { label: 'block, carried by the beam', y: mJ(res.extras.Kt), color: '--series-3', width: 1.5 },
          { label: 'beam', y: mJ(res.extras.Kb), color: '--text-muted', width: 1.5 },
          { label: 'total K', y: mJ(res.extras.K), color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Block (point mass on the beam surface)', page: 'p. 385 · Fig. 20-1',
          theory: '\\mathbf p_1 = z\\begin{bmatrix}\\cos\\theta\\\\ \\sin\\theta\\\\ 0\\end{bmatrix},\\quad \\mathbf v_1 = \\dot{\\mathbf p}_1',
          symbolic: '\\mathbf v_1 = \\dot z\\begin{bmatrix}\\cos\\theta\\\\ \\sin\\theta\\\\ 0\\end{bmatrix} + z\\dot\\theta\\begin{bmatrix}-\\sin\\theta\\\\ \\cos\\theta\\\\ 0\\end{bmatrix},\\quad \\|\\mathbf v_1\\|^2 = \\dot z^2 + z^2\\dot\\theta^2', spoiler: true },
        { title: 'Beam (thin rod pivoted at its end)', page: 'p. 23 · Eq. 2.2',
          theory: 'J_{cm} = \\frac{m\\ell^2}{12}\\;\\text{(thin rod)},\\quad J_{pivot} = J_{cm} + m d^2\\;\\text{(parallel axis)}',
          symbolic: 'J_{pivot} = \\frac{m_2\\ell^2}{12} + m_2\\left(\\frac{\\ell}{2}\\right)^2 = \\frac{m_2\\ell^2}{3}', spoiler: true },
        { title: 'Result', page: 'p. 386 · E.2(a)',
          theory: 'K = K_{block} + K_{beam}',
          symbolic: 'K = \\tfrac12 m_1\\big(\\dot z^2 + z^2\\dot\\theta^2\\big) + \\tfrac12\\frac{m_2\\ell^2}{3}\\dot\\theta^2',
          numbers: `K = ${tex(p.m1 / 2)}\\,\\dot z^2 + ${tex(p.m1 / 2)}\\,z^2\\dot\\theta^2 + ${tex(p.m2 * p.ell ** 2 / 6)}\\,\\dot\\theta^2`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) K = c<sub>1</sub>ż² + c<sub>2</sub>z²θ̇² + c<sub>3</sub>θ̇²',
          inputs: { c1: 'c<sub>1</sub>', c2: 'c<sub>2</sub>', c3: 'c<sub>3</sub>' },
          check: (v) => PD().checkNumbers(v, { c1: p().m1 / 2, c2: p().m1 / 2, c3: p().m2 * p().ell ** 2 / 6 }, {}),
          solution: () => [
            { html: 'The block at distance z along the beam has velocity ż along the beam and zθ̇ across it. The beam turns about its end, so J = m₂ℓ²/12 + m₂(ℓ/2)² = m₂ℓ²/3 (parallel-axis theorem, p. 23).' },
            { tex: `c_1 = c_2 = \\tfrac{m_1}{2} = ${tex(p().m1 / 2)},\\quad c_3 = \\tfrac{m_2\\ell^2}{6} = ${tex(p().m2 * p().ell ** 2 / 6)}` },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 3 --
  CH.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 41–56',
    linear: false,
    defaults() { return { comp: 'fl', inp: { shape: 'square', amp: 0.2, freq: 1, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 386 · E.3(e)');
      inputControls(sec, ctx, { comps: COMPS });
      sec.append(el('p', { class: 'muted small', text: 'The block and beam has no stable equilibrium: any tilt makes the block slide and the beam tip further. The energy plot checks the EOM: E(t) − E(0) must equal the work done by F.' }));
    },

    extraPlot(ctx, res) {
      const { sys, pModel } = ctx;
      const n = res.t.length, Ts = ctx.S.sim.Ts;
      const En = new Float64Array(n), W = new Float64Array(n);
      const E0 = sys.kinetic(res.x[0], pModel) + sys.potential(res.x[0], pModel);
      let w = 0;
      const pw = (xx, u) => u * pModel.ell * Math.cos(xx[1]) * xx[3];  // τ_θ θ̇
      for (let k = 0; k < n; k++) {
        const x = res.x[k];
        En[k] = (sys.kinetic(x, pModel) + sys.potential(x, pModel) - E0) * 1000;
        if (k > 0) w += 0.5 * Ts * (pw(res.x[k - 1], res.uApplied[k - 1]) + pw(x, res.uApplied[k - 1]));
        W[k] = w * 1000;
      }
      return {
        opts: { title: 'energy balance', yLabel: 'energy [mJ]', unit: 'mJ' },
        data: { series: [
          { label: 'work done by F', y: W, color: '--series-2', dash: [5, 4], width: 2 },
          { label: 'E(t) − E(0) = ΔK + ΔP', y: En, color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      const J0 = p.m2 * p.ell ** 2 / 3;
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 18 · Eq. 1.8, p. 43',
          theory: 'L = K - P,\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy', page: 'p. 386 · E.3(a)',
          theory: 'P = \\textstyle\\sum_i m_i g\\,h_i\\quad(h_i = \\text{height of each center of mass})',
          symbolic: 'P = m_1 g z\\sin\\theta + m_2 g\\tfrac{\\ell}{2}\\sin\\theta \\quad(P = 0 \\text{ with the beam level})',
          numbers: `P = (${tex(p.m1 * p.g)}\\,z + ${tex(p.m2 * p.g * p.ell / 2)})\\sin\\theta`, spoiler: true },
        { title: 'Generalized forces', page: 'p. 386 · E.3(b, c)',
          theory: '\\tau_i = \\text{virtual work of the external forces per unit } \\delta q_i',
          symbolic: 'q = (z, \\theta)^\\top,\\quad \\tau = \\begin{bmatrix}0\\\\ F\\ell\\cos\\theta\\end{bmatrix},\\quad B = 0',
          spoiler: true },
        { title: 'Equations of motion', page: 'p. 386 · E.3(d)',
          theory: '\\text{apply the Euler-Lagrange equation to } q_1 = z \\text{ and } q_2 = \\theta',
          symbolic: 'm_1\\ddot z - m_1 z\\dot\\theta^2 + m_1 g\\sin\\theta = 0,\\quad \\left(\\frac{m_2\\ell^2}{3} + m_1 z^2\\right)\\ddot\\theta + 2m_1 z\\dot z\\dot\\theta + \\left(m_1 g z + m_2 g\\frac{\\ell}{2}\\right)\\cos\\theta = F\\ell\\cos\\theta',
          numbers: `\\ddot z = z\\dot\\theta^2 - ${tex(p.g)}\\sin\\theta,\\quad (${tex(J0)} + ${tex(p.m1)}\\,z^2)\\,\\ddot\\theta = ${tex(p.ell)}F\\cos\\theta - ${tex(2 * p.m1)}\\,z\\dot z\\dot\\theta - (${tex(p.m1 * p.g)}\\,z + ${tex(p.m2 * p.g * p.ell / 2)})\\cos\\theta`, spoiler: true },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from Eq. 1.8',
          theory: '\\frac{d}{dt}(K + P) = \\tau^\\top\\dot q - \\dot q^\\top B\\dot q' },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch3, [
        {
          id: 'a', title: '(a) P = (c<sub>1</sub> z + c<sub>2</sub>) sin θ',
          inputs: { c1: 'c<sub>1</sub> [N]', c2: 'c<sub>2</sub> [N·m]' },
          check: (v) => PD().checkNumbers(v, { c1: p().m1 * p().g, c2: p().m2 * p().g * p().ell / 2 }, {}),
          solution: () => [{ tex: `P = m_1 g z\\sin\\theta + m_2 g\\tfrac{\\ell}{2}\\sin\\theta:\\quad c_1 = m_1 g = ${tex(p().m1 * p().g)},\\; c_2 = m_2 g\\tfrac{\\ell}{2} = ${tex(p().m2 * p().g * p().ell / 2)}` }],
        },
        {
          id: 'c', title: '(b, c) Generalized coordinates and forces: τ = (τ<sub>z</sub>, c F cos θ)',
          inputs: { tz: 'τ<sub>z</sub>', c: 'c [m]', b: 'damping b' },
          check: (v) => PD().checkNumbers(v, { tz: 0, c: p().ell, b: 0 }, { tz: 'τz', c: 'c', b: 'b' }),
          solution: () => [
            { html: 'q = (z, θ)ᵀ. The force acts at the tip, at height ℓ sin θ, so δW = F ℓ cos θ δθ: τ = (0, Fℓ cos θ)ᵀ. The problem models no friction, so B = 0.' },
          ],
        },
        {
          id: 'd', title: '(d) Check your EOM: evaluate at z = 0.4 m, θ = 10°, ż = 0.1 m/s, θ̇ = 0.2 rad/s, F = 12 N',
          inputs: { zdd: 'z̈ [m/s²]', thdd: 'θ̈ [rad/s²]' },
          check: (v) => { const d = ctx.sys.f([0.4, 10 * M.DEG, 0.1, 0.2], 12, p()); return PD().checkNumbers(v, { zdd: d[2], thdd: d[3] }, { zdd: 'z̈', thdd: 'θ̈' }); },
          solution: () => {
            const d = ctx.sys.f([0.4, 10 * M.DEG, 0.1, 0.2], 12, p());
            return [
              { tex: 'L = \\tfrac12 m_1(\\dot z^2 + z^2\\dot\\theta^2) + \\tfrac{m_2\\ell^2}{6}\\dot\\theta^2 - (m_1 g z + m_2 g\\tfrac{\\ell}{2})\\sin\\theta' },
              { tex: 'z:\\; m_1\\ddot z - m_1 z\\dot\\theta^2 + m_1 g\\sin\\theta = 0' },
              { tex: '\\theta:\\; \\tfrac{d}{dt}\\big[(m_1 z^2 + \\tfrac{m_2\\ell^2}{3})\\dot\\theta\\big] + (m_1 g z + m_2 g\\tfrac{\\ell}{2})\\cos\\theta = F\\ell\\cos\\theta' },
              { tex: `\\ddot z = ${tex(d[2])},\\quad \\ddot\\theta = ${tex(d[3])}` },
              { html: 'Same as the f(x, u) that passes <code>_E_blockbeam/python/testDynamics.py</code> (with g = 9.81 there; see ISSUES).' },
            ];
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 4 --
  CH.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68',
    defaults() { return { zE: 0.25, dz0: 0, dth0: 0.5, method: 'jacobian', comp: 'eq', inp: { shape: 'zero', amp: 0.05, freq: 1, width: 0.2 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    linearLabel: 'linearized model',

    lin(ctx) { return ctx.sys.linear(ctx.pModel, { z: ctx.st.zE, comp: ctx.st.method === 'fl' ? 'fl' : 'eq' }); },
    controller(ctx, o) { ctx.st.comp = ctx.st.method === 'fl' ? 'fl' : 'eq'; return openLoop(ctx, o); },
    x0(ctx) { return [ctx.st.zE + ctx.st.dz0, ctx.st.dth0 * M.DEG, 0, 0]; },
    simulate(ctx, c, plant) {
      return WB.sim.simulate({ ...c, x0: this.x0(ctx), plant, controller: this.controller(ctx), reference: () => ctx.st.zE });
    },
    linearSim(ctx, c) {
      const { A, B } = this.lin(ctx);
      return linearOpenLoop(ctx, { ...c, reference: () => ctx.st.zE }, A, B, this.x0(ctx), ctx.st.zE);
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Operating point', 'p. 386 · E.4');
      slider(sec, { label: 'z<sub>e</sub>', unit: 'm', min: 0, max: 0.5, step: 0.005, sig: 3, get: () => ctx.st.zE, set: (v) => { ctx.st.zE = v; ctx.update(); } });
      slider(sec, { label: 'δz(0)', unit: 'm', min: -0.1, max: 0.1, step: 0.001, sig: 3, get: () => ctx.st.dz0, set: (v) => { ctx.st.dz0 = v; ctx.update(); } });
      slider(sec, { label: 'δθ(0)', unit: '°', min: -5, max: 5, step: 0.05, sig: 3, get: () => ctx.st.dth0, set: (v) => { ctx.st.dth0 = v; ctx.update(); } });
      segmented(sec, {
        label: 'Linearization',
        options: [{ value: 'jacobian', label: 'Jacobian: F = F<sub>e</sub> + F̃' }, { value: 'fl', label: 'feedback: F = F<sub>fl</sub>(z) + F̃' }],
        get: () => ctx.st.method, set: (v) => { ctx.st.method = v; ctx.update(); },
      });
      const inp = section(parent, 'Input F̃(t)', 'p. 386');
      inputControls(inp, ctx);
      inp.append(el('p', { class: 'muted small', text: 'Start a hair off equilibrium and watch the linear model (dashed) and the nonlinear plant separate as the block slides. Compare the eigenvalues of the two linearizations in the s-plane.' }));
    },

    outputSeries(ctx, res, sc, oi) {
      return oi === 0 ? [{ label: 'zₑ', y: Array.from(res.t, () => ctx.st.zE), color: '--ref', dash: [6, 4], width: 1.5 }] : [];
    },

    splane(ctx) {
      const { A } = this.lin(ctx);
      return { markers: L.eig(A).map((p) => ({ ...p, kind: 'ol', label: `eigenvalue of A (${ctx.st.method})` })), fitR: 2 };
    },

    math(ctx) {
      const p = ctx.pModel, zE = ctx.st.zE;
      const jac = ctx.sys.linear(p, { z: zE, comp: 'eq' });
      const lam = Math.pow(p.m1 * p.g * p.g / jac.De, 0.25);
      return [
        { title: 'Nonlinear model', page: 'p. 386 · E.3(d)',
          theory: '\\dot x = f(x, u),\\quad x = (z, \\theta, \\dot z, \\dot\\theta)^\\top,\\quad u = F',
          symbolic: '\\dot x = \\begin{bmatrix}\\dot z\\\\ \\dot\\theta\\\\ z\\dot\\theta^2 - g\\sin\\theta\\\\ \\dfrac{F\\ell\\cos\\theta - 2m_1z\\dot z\\dot\\theta - (m_1gz + m_2g\\frac{\\ell}{2})\\cos\\theta}{\\frac{m_2\\ell^2}{3} + m_1z^2}\\end{bmatrix}',
          spoiler: true },
        { title: 'Equilibria', page: 'p. 60 · Eq. 4.1, E.4(a)',
          theory: 'f(x_e, u_e) = 0',
          symbolic: '\\theta_e = 0,\\; \\dot z_e = \\dot\\theta_e = 0,\\; z_e \\text{ arbitrary},\\quad F_e = \\frac{m_1 g z_e}{\\ell} + \\frac{m_2 g}{2}',
          numbers: `F_e(z_e = ${fmt(zE, 3)}) = ${tex(jac.Fe)}\\;\\text{N}`, spoiler: true },
        { title: 'Jacobian linearization', page: 'p. 60 · Eq. 4.1, p. 83',
          theory: '\\dot{\\tilde x} = \\frac{\\partial f}{\\partial x}\\Big|_e\\tilde x + \\frac{\\partial f}{\\partial u}\\Big|_e\\tilde u',
          symbolic: 'J_e = \\frac{m_2\\ell^2}{3} + m_1z_e^2:\\quad \\ddot{\\tilde z} = -g\\tilde\\theta,\\quad \\ddot{\\tilde\\theta} = \\frac{\\ell\\tilde F - m_1 g\\tilde z}{J_e},\\quad \\det(sI - A) = s^4 - \\frac{m_1g^2}{J_e}',
          numbers: `A = ${texMat(jac.A)},\\quad B = ${texMat(jac.B)},\\quad \\text{eig}(A) = \\pm${tex(lam)},\\;\\pm ${tex(lam)}j`, spoiler: true },
        { title: 'Feedback linearization', page: 'p. 64',
          theory: 'u = u_{fl}(x) + \\tilde u,\\quad u_{fl} \\text{ chosen to cancel the nonlinear terms}',
          symbolic: 'F = F_{fl}(z) + \\tilde F,\\; F_{fl} = \\frac{m_1 g z}{\\ell} + \\frac{m_2 g}{2} \\;\\Rightarrow\\; J(z)\\ddot\\theta = \\ell\\cos\\theta\\,\\tilde F - 2m_1z\\dot z\\dot\\theta\\quad(\\text{the } z \\text{ equation has no } F\\text{: only the } \\theta \\text{ subsystem is linearized this simply})',
          spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel, zE = () => ctx.st.zE;
      const jac = () => ctx.sys.linear(p(), { z: zE(), comp: 'eq' });
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibrium at the current z<sub>e</sub>',
          inputs: { the: 'θ<sub>e</sub> [rad]', Fe: 'F<sub>e</sub> [N]' },
          check: (v) => PD().checkNumbers(v, { the: 0, Fe: jac().Fe }, { the: 'θe', Fe: 'Fe' }),
          solution: () => [
            { html: 'ẋ = 0 needs ż = θ̇ = 0, then z̈ = 0 ⇒ sin θ = 0 ⇒ θ<sub>e</sub> = 0 (θ = π is the beam upside down), and θ̈ = 0 ⇒ F<sub>e</sub>ℓ = m₁gz<sub>e</sub> + m₂gℓ/2. Any z<sub>e</sub> on the beam works.' },
            { tex: `F_e = \\frac{m_1 g z_e}{\\ell} + \\frac{m_2 g}{2} = ${tex(jac().Fe)}\\;\\text{N}` },
          ],
        },
        {
          id: 'b', title: '(b) Jacobian at the current z<sub>e</sub>: nonzero entries of A and B',
          html: 'With x̃ = (z̃, θ̃, z̃̇, θ̃̇): a<sub>32</sub> multiplies θ̃ in z̃̈, a<sub>41</sub> multiplies z̃ in θ̃̈, b<sub>4</sub> multiplies F̃.',
          inputs: { a32: 'a<sub>32</sub>', a41: 'a<sub>41</sub>', b4: 'b<sub>4</sub>' },
          check: (v) => PD().checkNumbers(v, { a32: jac().A[2][1], a41: jac().A[3][0], b4: jac().B[3][0] }, {}),
          solution: () => [
            { tex: `a_{32} = -g = ${tex(jac().A[2][1])},\\quad a_{41} = -\\frac{m_1 g}{J_e} = ${tex(jac().A[3][0])},\\quad b_4 = \\frac{\\ell}{J_e} = ${tex(jac().B[3][0])}` },
            { html: 'The z θ̇², 2m₁zżθ̇ and sin θ·(…) terms all vanish to first order at the equilibrium, because each is a product with a zero factor there.' },
          ],
        },
        {
          id: 'c', title: '(c) Feedback linearization F = c<sub>1</sub> z + c<sub>2</sub> + F̃',
          inputs: { c1: 'c<sub>1</sub> [N/m]', c2: 'c<sub>2</sub> [N]' },
          check: (v) => PD().checkNumbers(v, { c1: p().m1 * p().g / p().ell, c2: p().m2 * p().g / 2 }, {}),
          solution: () => [
            { tex: `F_{fl}(z) = \\frac{m_1 g}{\\ell}z + \\frac{m_2 g}{2} = ${tex(p().m1 * p().g / p().ell)}\\,z + ${tex(p().m2 * p().g / 2)}` },
            { html: 'This makes the θ equation linear near θ = 0 (and exactly linear in θ̈ if you also cancel the Coriolis term and divide by cos θ). The z equation stays nonlinear, so the whole system is only partly feedback-linearized. The book uses this F_fl in E.8(e).' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------- Chapters 5 and 6 --
  // Open-loop pulse with F_fl(z) added: the beam sees ≈ b0/s² and the block −g/s².
  function pulseSim(self) {
    return {
      linearLabel: 'linear model',
      simulate(ctx, c, plant) { ctx.st.comp = ctx.st.model === 'full' ? 'eq' : 'fl'; return WB.sim.simulate({ ...c, plant, controller: openLoop(ctx), reference: () => ctx.sys.ze(ctx.pModel) }); },
      linearSim(ctx, c) {
        const lin = ctx.sys.linear(ctx.pModel, { comp: ctx.st.model === 'full' ? 'eq' : 'fl' });
        return linearOpenLoop(ctx, { ...c, reference: () => lin.ze }, lin.A, lin.B, c.x0, lin.ze);
      },
    };
  }
  function modelToggle(parent, ctx) {
    segmented(parent, {
      label: 'Model and compensation',
      options: [
        { value: 'simple', label: 'F = F<sub>fl</sub>(z) + F̃, E.5(c) model' },
        { value: 'full', label: 'F = F<sub>e</sub> + F̃, full Jacobian' },
      ],
      get: () => ctx.st.model, set: (v) => { ctx.st.model = v; ctx.update(); },
    });
  }

  CH.ch5 = Object.assign({}, common, pulseSim(), {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 69–80',
    defaults() { return { comp: 'fl', model: 'simple', inp: { shape: 'pulse', amp: 0.05, freq: 1, width: 0.2 } }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Input F̃(t)', 'p. 386 · E.5');
      modelToggle(sec, ctx);
      inputControls(sec, ctx);
      sec.append(el('p', { class: 'muted small', text: 'A force pulse spins the beam up, and the tilt makes the block accelerate. The dashed trace is the linear model you choose above.' }));
    },
    splane(ctx) {
      const lin = ctx.sys.linear(ctx.pModel, { comp: ctx.st.model === 'full' ? 'eq' : 'fl' });
      return { markers: L.eig(lin.A).map((p) => ({ ...p, kind: 'ol', label: `pole (${ctx.st.model === 'full' ? 'full' : 'E.5(c)'} model)` })), fitR: 2 };
    },
    math(ctx) {
      const lin = ctx.sys.linear(ctx.pModel, { comp: 'eq' });
      const a = ctx.pModel.m1 * ctx.pModel.g ** 2 / lin.De;
      return [
        { title: 'Laplace transform of the linear EOM', page: 'p. 69–70, E.5(a)',
          theory: '\\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)\\;\\to\\; s^2Y(s)\\;(\\text{zero initial conditions})',
          symbolic: 's^2\\tilde Z = -g\\tilde\\Theta,\\quad J_e s^2\\tilde\\Theta = \\ell\\tilde F - m_1 g\\tilde Z', spoiler: true },
        { title: 'Full transfer functions', page: 'p. 387 · E.5(b)',
          theory: '\\text{solve the transformed equations for } \\tilde\\Theta/\\tilde F,\\; \\tilde Z/\\tilde F,\\; \\tilde Z/\\tilde\\Theta',
          symbolic: '\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{\\frac{\\ell}{J_e}s^2}{s^4 - \\frac{m_1g^2}{J_e}},\\quad \\frac{\\tilde Z}{\\tilde F} = \\frac{-\\frac{g\\ell}{J_e}}{s^4 - \\frac{m_1g^2}{J_e}},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = -\\frac{g}{s^2}',
          numbers: `\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{${tex(lin.b0)}\\,s^2}{s^4 - ${tex(a)}},\\quad \\frac{\\tilde Z}{\\tilde F} = \\frac{${tex(-ctx.pModel.g * lin.b0)}}{s^4 - ${tex(a)}}`, spoiler: true },
        { title: 'Simplified cascade (drop m₁g z̃)', page: 'p. 72–74 · §5.3, p. 387 · E.5(c, d)',
          theory: 'P(s) \\approx P_{in}(s)\\,P_{out}(s)\\;\\text{(cascade approximation)}',
          symbolic: 'P_{in}(s) = \\frac{\\tilde\\Theta}{\\tilde F} = \\frac{\\ell / (\\frac{m_2\\ell^2}{3} + m_1 z_e^2)}{s^2},\\quad P_{out}(s) = \\frac{\\tilde Z}{\\tilde\\Theta} = -\\frac{g}{s^2}',
          numbers: `P_{in} = \\frac{${tex(lin.b0)}}{s^2},\\quad P_{out} = \\frac{${tex(-ctx.pModel.g)}}{s^2}`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const lin = () => ctx.sys.linear(ctx.pModel, { comp: 'eq' });
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'b', title: '(b) Θ̃/F̃ = b s² / (s⁴ − a) and Z̃/Θ̃ = c / s²',
          inputs: { b: 'b', a: 'a', c: 'c' },
          check: (v) => PD().checkNumbers(v, { b: lin().b0, a: ctx.pModel.m1 * ctx.pModel.g ** 2 / lin().De, c: -ctx.pModel.g }, {}),
          solution: () => {
            const l = lin(), a = ctx.pModel.m1 * ctx.pModel.g ** 2 / l.De;
            return [
              { tex: '\\tilde Z = -\\frac{g}{s^2}\\tilde\\Theta \\Rightarrow J_e s^2\\tilde\\Theta = \\ell\\tilde F + \\frac{m_1 g^2}{s^2}\\tilde\\Theta \\Rightarrow \\frac{\\tilde\\Theta}{\\tilde F} = \\frac{(\\ell/J_e)s^2}{s^4 - m_1g^2/J_e}' },
              { tex: `b = \\frac{\\ell}{J_e} = ${tex(l.b0)},\\quad a = \\frac{m_1 g^2}{J_e} = ${tex(a)},\\quad c = -g` },
            ];
          },
        },
        {
          id: 'c', title: '(c) Simplified: Θ̃/F̃ = b<sub>0</sub>/s²',
          inputs: { b0: 'b<sub>0</sub>' },
          check: (v) => PD().checkNumbers(v, { b0: lin().b0 }, {}),
          solution: () => [
            { tex: `b_0 = \\frac{\\ell}{\\frac{m_2\\ell^2}{3} + m_1 z_e^2} = ${tex(lin().b0)},\\quad z_e = \\tfrac{\\ell}{2}` },
            { html: '(d) Cascade: F̃ → [b₀/s²] → Θ̃ → [−g/s²] → Z̃. The inner (beam) and outer (block) blocks are each a double integrator, which sets up the successive loop closure of E.8.' },
          ],
        },
      ]);
    },
  });

  CH.ch6 = Object.assign({}, common, pulseSim(), {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 81–93',
    defaults() { return { comp: 'eq', model: 'full', inp: { shape: 'pulse', amp: 0.05, freq: 1, width: 0.2 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Input ũ = F̃(t)', 'p. 387 · E.6');
      modelToggle(sec, ctx);
      inputControls(sec, ctx);
    },
    splane(ctx) {
      const lin = ctx.sys.linear(ctx.pModel, { comp: ctx.st.model === 'full' ? 'eq' : 'fl' });
      return { markers: L.eig(lin.A).map((p, i) => ({ ...p, kind: 'ol', label: `eigenvalue of A ${i + 1}` })), fitR: 2 };
    },
    extraPlot(ctx, res) {
      const k = 180 / Math.PI;
      return { opts: { title: 'velocity states', yLabel: 'ż [m/s], θ̇ [°/s]/100', unit: '' }, data: { series: [
        { label: 'ż [m/s]', y: res.x.map((x) => x[2]), color: '--series-1' },
        { label: 'θ̇ [°/s] ÷ 100', y: res.x.map((x) => x[3] * k / 100), color: '--series-3' },
      ] } };
    },
    math(ctx) {
      const { A, B, C } = ctx.sys.linear(ctx.pModel, { comp: 'eq' });
      return [
        { title: 'Jacobian linearization revisited', page: 'p. 83–84',
          theory: 'A = \\frac{\\partial f}{\\partial x}\\Big|_e,\\quad B = \\frac{\\partial f}{\\partial u}\\Big|_e,\\quad C = \\frac{\\partial h}{\\partial x}\\Big|_e,\\quad D = \\frac{\\partial h}{\\partial u}\\Big|_e' },
        { title: 'State-space model', page: 'p. 387 · E.6',
          theory: '\\dot{\\tilde x} = A\\tilde x + B\\tilde u,\\quad \\tilde y = C\\tilde x + D\\tilde u',
          symbolic: 'A = \\begin{bmatrix}0&0&1&0\\\\0&0&0&1\\\\0&-g&0&0\\\\-\\frac{m_1g}{J_e}&0&0&0\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\0\\\\0\\\\ \\frac{\\ell}{J_e}\\end{bmatrix},\\quad C = \\begin{bmatrix}1&0&0&0\\\\0&1&0&0\\end{bmatrix}',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(C)},\\quad D = 0`, spoiler: true },
        { title: 'Back to the transfer functions', page: 'pp. 85–86 · Eq. 6.14–6.15',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\text{poles: } \\det(sI - A) = 0',
          numbers: `\\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const ss = () => ctx.sys.linear(ctx.pModel, { comp: 'eq' });
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'Nonzero entries of A and B (other than the 1s that make ż, θ̇ states), and D',
        inputs: { a32: 'A<sub>32</sub>', a41: 'A<sub>41</sub>', b4: 'B<sub>4</sub>', d: 'D (both entries)' },
        check: (v) => { const { A, B } = ss(); return PD().checkNumbers(v, { a32: A[2][1], a41: A[3][0], b4: B[3][0], d: 0 }, {}); },
        solution: () => { const { A, B, C } = ss(); return [{ tex: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(C)},\\quad D = \\begin{bmatrix}0\\\\0\\end{bmatrix}` }]; },
      }]);
    },
  });

  WB.E.models = { inputForce, openLoop };
})();
