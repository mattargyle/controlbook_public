// Study C, chapters 2-6: modeling the satellite. Open-loop experiments: kinetic
// energy (C.2), Euler-Lagrange equations with an energy-balance check (C.3),
// equilibria (C.4), the transfer matrix and the cascade approximation (C.5), and
// the state-space model with the star-tracker/strain-gauge outputs (C.6).
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const PD = () => WB.pd;
  const lib = () => WB.studies.C.lib;
  const CH = WB.studies.C.chapters;
  const R2D = 180 / Math.PI;

  // ------------------------------------------------- open-loop torque input --
  const torque = (st, t) => WB.models.inputTorque(st, t);

  function inputControls(parent, ctx, { shapes = ['zero', 'const', 'pulse', 'square', 'sine'] } = {}) {
    const names = { zero: 'none', const: 'constant', pulse: 'pulse', square: 'square', sine: 'sine' };
    segmented(parent, {
      label: 'Body torque τ(t)',
      options: shapes.map((s) => ({ value: s, label: names[s] })),
      ...bind(ctx, 'shape', () => ctx.st.inp),
    });
    slider(parent, { label: 'amplitude', unit: 'N·m', min: -5, max: 5, step: 0.01, sig: 3, ...bind(ctx, 'amp', () => ctx.st.inp), disabled: () => ctx.st.inp.shape === 'zero' });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.005, max: 1, step: 0.005, sig: 3, ...bind(ctx, 'freq', () => ctx.st.inp), disabled: () => !['square', 'sine'].includes(ctx.st.inp.shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.1, max: 20, step: 0.1, sig: 3, ...bind(ctx, 'width', () => ctx.st.inp), disabled: () => ctx.st.inp.shape !== 'pulse' });
  }

  const common = {
    openLoop: true, metrics: false,
    controller(ctx) { return { update: (r, x, y, t) => torque(ctx.st, t) }; },
  };

  // Linear-model overlay: the given plant (A, B) from the same initial state,
  // driven by the same torque, without saturation, disturbance or noise.
  function linearOverlay(ctx, c, A, B) {
    return WB.sim.simulate({
      ...c, disturbance: null, noise: null,
      plant: { f: (x, u) => A.map((row, i) => row.reduce((s, a, j) => s + a * x[j], 0) + B[i][0] * u), h: (x) => [x[0], x[1]], uLimit: Infinity },
      controller: { update: (r, x, y, t) => torque(ctx.st, t) },
    });
  }

  // ------------------------------------------------------------- C.2 --
  // Prescribed motion as in hw02_satelliteSim.py: θ and φ are sinusoids; nothing is simulated.
  CH.ch2 = Object.assign({}, common, {
    id: 'ch2', num: 2, tab: 'Ch 2', title: 'Kinetic energy', pages: 'pp. 35–38',
    linear: false,
    defaults() { return { Ath: 60, Aph: 28.6, f: 0.1, lag: 0, inp: { shape: 'zero', amp: 0, freq: 0.1, width: 1 } }; },
    simDefaults(sys) { return sys.problems.ch2.sim; },

    simulate(ctx) {
      const { pModel: p, st, S } = ctx;
      const Ts = S.sim.Ts, N = Math.round(S.sim.tEnd / Ts) + 1;
      const w = 2 * Math.PI * st.f, Ath = st.Ath / R2D, Aph = st.Aph / R2D, lag = st.lag / R2D;
      const F = () => new Float64Array(N);
      const res = { t: F(), yAll: [F(), F()], yMeasAll: null, rAll: [F()], uDemandAll: [F()], uAll: [F()], uAppliedAll: [F()], x: [], extras: { Ks: F(), Kp: F(), K: F() } };
      for (let k = 0; k < N; k++) {
        const t = k * Ts;
        const th = Ath * Math.sin(w * t), ph = Aph * Math.sin(w * t - lag);
        const thd = Ath * w * Math.cos(w * t), phd = Aph * w * Math.cos(w * t - lag);
        res.t[k] = t; res.yAll[0][k] = th; res.yAll[1][k] = ph; res.rAll[0][k] = ph;
        res.x.push([th, ph, thd, phd]);
        res.extras.Ks[k] = 0.5 * p.Js * thd ** 2;
        res.extras.Kp[k] = 0.5 * p.Jp * phd ** 2;
        res.extras.K[k] = res.extras.Ks[k] + res.extras.Kp[k];
      }
      res.yMeasAll = res.yAll;
      return res;
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Prescribed motion', 'pp. 38–39 · Listing 2.6');
      slider(sec, { label: 'θ amplitude', unit: '°', min: 0, max: 360, step: 1, sig: 3, ...bind(ctx, 'Ath') });
      slider(sec, { label: 'φ amplitude', unit: '°', min: 0, max: 360, step: 0.1, sig: 3, ...bind(ctx, 'Aph') });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.01, max: 1, step: 0.01, sig: 3, ...bind(ctx, 'f') });
      slider(sec, { label: 'φ lags θ by', unit: '°', min: 0, max: 180, step: 1, sig: 3, ...bind(ctx, 'lag') });
      sec.append(el('p', { class: 'muted small', text: 'No dynamics here: the angles are imposed, as in hw02_satelliteSim.py (which uses θ = 2π sin(0.2πt) rad and φ = 0.5 sin(0.2πt) rad). The plot below splits K into the body and panel parts.' }));
    },

    extraPlot(ctx, res) {
      const show = ctx.S.mode === 'explore';
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [J]', unit: 'J' },
        data: { series: [
          { label: show ? 'body ½Jsθ̇²' : 'body', y: Array.from(res.extras.Ks), color: '--series-2', width: 1.5 },
          { label: show ? 'panel ½Jpφ̇²' : 'panel', y: Array.from(res.extras.Kp), color: '--series-3', width: 1.5 },
          { label: 'total K', y: Array.from(res.extras.K), color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Two rotating bodies, no translation', page: 'p. 36',
          theory: '\\mathbf v_{cm} = 0,\\quad \\boldsymbol\\omega_s = \\dot\\theta\\,\\hat k,\\quad \\boldsymbol\\omega_p = \\dot\\phi\\,\\hat k' },
        { title: 'Result', page: 'p. 36 · Eq. 2.5',
          theory: 'K = \\textstyle\\sum_i \\tfrac12\\,\\boldsymbol\\omega_i^\\top J_i\\boldsymbol\\omega_i \\text{ for bodies that only rotate}',
          symbolic: 'K = \\tfrac12 J_s\\dot\\theta^2 + \\tfrac12 J_p\\dot\\phi^2',
          numbers: `K = ${tex(p.Js / 2)}\\,\\dot\\theta^2 + ${tex(p.Jp / 2)}\\,\\dot\\phi^2`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [{
        id: 'a', title: '(a) K = c<sub>1</sub> θ̇² + c<sub>2</sub> φ̇²',
        inputs: { c1: 'c<sub>1</sub> [kg·m²]', c2: 'c<sub>2</sub> [kg·m²]' },
        check: (v) => PD().checkNumbers(v, { c1: p().Js / 2, c2: p().Jp / 2 }, { c1: 'c1', c2: 'c2' }),
        solution: () => [
          { tex: `K = \\tfrac12 J_s\\dot\\theta^2 + \\tfrac12 J_p\\dot\\phi^2 \\Rightarrow c_1 = ${tex(p().Js / 2)},\\; c_2 = ${tex(p().Jp / 2)}` },
          { html: 'Book: Eq. 2.5 (p. 36). Each body only rotates about the common axis, so there is no translational term.' },
        ],
      }]);
    },
  });

  // ------------------------------------------------------------- C.3 --
  CH.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 51–55',
    linear: false,
    defaults(sys) { const o = sys.problems.ch3.openLoop; return { inp: { shape: o.shape, amp: o.amp, freq: o.freq, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 51 · C.3(e), Listing 3.5');
      inputControls(sec, ctx);
      sec.append(el('p', { class: 'muted small', text: 'hw03_satelliteSim.py drives the body with τ = 0.1 sin(0.2πt) N·m. The energy plot checks the EOM: E(t) − E(0) must equal the work done by τ minus what the damper dissipates.' }));
    },

    extraPlot(ctx, res) {
      const p = ctx.pModel, n = res.t.length, Ts = ctx.S.sim.Ts;
      const energy = (x) => 0.5 * p.Js * x[2] ** 2 + 0.5 * p.Jp * x[3] ** 2 + 0.5 * p.k * (x[1] - x[0]) ** 2;
      const power = (x, u) => u * x[2] - p.b * (x[2] - x[3]) ** 2;
      const E = new Float64Array(n), W = new Float64Array(n);
      const E0 = energy(res.x[0]);
      let w = 0;
      for (let k = 0; k < n; k++) {
        E[k] = (energy(res.x[k]) - E0) * 1000;
        if (k > 0) w += 0.5 * Ts * (power(res.x[k - 1], res.uApplied[k - 1]) + power(res.x[k], res.uApplied[k - 1]));
        W[k] = w * 1000;
      }
      return {
        opts: { title: 'energy balance', yLabel: 'energy [mJ]', unit: 'mJ' },
        data: { series: [
          { label: ctx.S.mode === 'explore' ? '∫(τθ̇ − b(θ̇−φ̇)²) dt' : 'work by τ − damper loss', y: W, color: '--series-2', dash: [5, 4], width: 2 },
          { label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Generalized coordinates and forces', page: 'p. 52',
          theory: '\\text{generalized forces: torques acting on each coordinate},\\quad \\text{friction: } -B\\dot q',
          symbolic: 'q = (\\theta, \\phi)^\\top,\\quad \\tau = (\\tau, 0)^\\top,\\quad -B\\dot q = -\\begin{bmatrix} b & -b\\\\ -b & b\\end{bmatrix}\\begin{bmatrix}\\dot\\theta\\\\ \\dot\\phi\\end{bmatrix}', spoiler: true },
        { title: 'Potential energy', page: 'p. 52',
          theory: '\\text{torsional spring with deflection } \\delta:\\; P = \\tfrac12 k\\,\\delta^2,\\quad L = K - P',
          symbolic: 'P = \\tfrac12 k(\\phi - \\theta)^2,\\quad L = \\tfrac12 J_s\\dot\\theta^2 + \\tfrac12 J_p\\dot\\phi^2 - \\tfrac12 k(\\phi-\\theta)^2', spoiler: true },
        { title: 'Euler-Lagrange equations', page: 'p. 18 · Eq. 1.8, p. 53',
          theory: '\\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Equations of motion', page: 'p. 53 · Eq. 3.3',
          theory: 'M\\ddot q = \\tau - B\\dot q - \\frac{\\partial P}{\\partial q} \\;\\text{(constant } M\\text{)}',
          symbolic: '\\begin{bmatrix}J_s & 0\\\\ 0 & J_p\\end{bmatrix}\\begin{bmatrix}\\ddot\\theta\\\\ \\ddot\\phi\\end{bmatrix} = \\begin{bmatrix}\\tau - b(\\dot\\theta - \\dot\\phi) - k(\\theta - \\phi)\\\\ -b(\\dot\\phi - \\dot\\theta) - k(\\phi - \\theta)\\end{bmatrix}',
          numbers: `\\ddot\\theta = ${tex(1 / p.Js)}\\tau - ${tex(p.b / p.Js)}(\\dot\\theta - \\dot\\phi) - ${tex(p.k / p.Js)}(\\theta - \\phi),\\quad \\ddot\\phi = -${tex(p.b / p.Jp)}(\\dot\\phi - \\dot\\theta) - ${tex(p.k / p.Jp)}(\\phi - \\theta)`, spoiler: true },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from Eq. 3.3',
          theory: '\\frac{d}{dt}(K + P) = \\tau^\\top\\dot q - \\dot q^\\top B\\dot q',
          symbolic: '\\frac{d}{dt}(K + P) = \\tau\\dot\\theta - b(\\dot\\theta - \\dot\\phi)^2', spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch3, [
        {
          id: 'a', title: '(a) P = c·δ², where δ is the deflection of the spring',
          inputs: { c: 'c [N·m]' },
          check: (v) => PD().checkNumbers(v, { c: p().k / 2 }, {}),
          solution: () => [{ tex: `P = \\tfrac12 k(\\phi - \\theta)^2 \\Rightarrow c = ${tex(p().k / 2)}` }],
        },
        {
          id: 'd', title: '(d) θ̈ = c<sub>1</sub>τ + c<sub>2</sub>θ̇ + c<sub>3</sub>φ̇ + c<sub>4</sub>θ + c<sub>5</sub>φ and φ̈ = d<sub>1</sub>θ̇ + d<sub>2</sub>φ̇ + d<sub>3</sub>θ + d<sub>4</sub>φ',
          inputs: { c1: 'c<sub>1</sub>', c2: 'c<sub>2</sub>', c3: 'c<sub>3</sub>', c4: 'c<sub>4</sub>', c5: 'c<sub>5</sub>', d1: 'd<sub>1</sub>', d2: 'd<sub>2</sub>', d3: 'd<sub>3</sub>', d4: 'd<sub>4</sub>' },
          check: (v) => { const q = p(); return PD().checkNumbers(v, { c1: 1 / q.Js, c2: -q.b / q.Js, c3: q.b / q.Js, c4: -q.k / q.Js, c5: q.k / q.Js, d1: q.b / q.Jp, d2: -q.b / q.Jp, d3: q.k / q.Jp, d4: -q.k / q.Jp }, {}); },
          solution: () => [
            { tex: `\\ddot\\theta = \\tfrac{1}{J_s}\\tau - \\tfrac{b}{J_s}(\\dot\\theta - \\dot\\phi) - \\tfrac{k}{J_s}(\\theta - \\phi),\\quad \\ddot\\phi = -\\tfrac{b}{J_p}(\\dot\\phi - \\dot\\theta) - \\tfrac{k}{J_p}(\\phi - \\theta)` },
            { tex: `c_1 = ${tex(1 / p().Js)},\\; c_2 = ${tex(-p().b / p().Js)},\\; c_3 = ${tex(p().b / p().Js)},\\; c_4 = ${tex(-p().k / p().Js)},\\; c_5 = ${tex(p().k / p().Js)}` },
            { tex: `d_1 = ${tex(p().b / p().Jp)},\\; d_2 = ${tex(-p().b / p().Jp)},\\; d_3 = ${tex(p().k / p().Jp)},\\; d_4 = ${tex(-p().k / p().Jp)}` },
            { html: 'Book: Eq. 3.3 (p. 53).' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- C.4 --
  // From rest at θ0 ≠ φ0 with τ = 0, angular momentum Js θ̇ + Jp φ̇ stays 0, so the
  // pair settles at the inertia-weighted mean angle: one of the equilibria (4.15).
  CH.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria', pages: 'pp. 67–68',
    defaults() { return { inp: { shape: 'zero', amp: 0.05, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    linear: false,

    finalAngle(ctx) {
      const p = ctx.pModel, th0 = ctx.S.sim.y0, ph0 = ctx.S.sim.init.phi0 ?? 0;
      return (p.Js * th0 + p.Jp * ph0) / (p.Js + p.Jp);
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Release from rest', 'p. 67 · Eq. 4.15');
      sec.append(el('p', { class: 'muted small', text: 'Set θ(0) ≠ φ(0) in the left panel (initial conditions) and let go with τ = 0. The spring and damper pull the two bodies together; they come to rest at an equilibrium.' }));
      inputControls(sec, ctx);
    },

    outputSeries(ctx, res, sc, oi) {
      if (oi !== 0 || ctx.st.inp.shape !== 'zero') return [];
      const fa = this.finalAngle(ctx);
      if (!(WB.ui.shown(ctx, 'C:ch4:final'))) return [];
      return [{ label: '(Jsθ₀ + Jpφ₀)/(Js + Jp)', y: Array.from(res.t, () => fa), color: '--ref', dash: [6, 4], width: 1.5 }];
    },

    extraPlot(ctx, res) {
      const p = ctx.pModel;
      return {
        opts: { title: 'angular momentum and spring deflection', yLabel: 'value', unit: '' },
        data: { series: [
          { label: ctx.S.mode === 'explore' ? 'Js θ̇ + Jp φ̇ [kg·m²/s]' : 'angular momentum [kg·m²/s]', y: res.x.map((x) => p.Js * x[2] + p.Jp * x[3]), color: '--series-2', width: 1.5 },
          { label: 'φ − θ [°]', y: res.x.map((x) => (x[1] - x[0]) * R2D), color: '--series-1' },
        ] },
      };
    },

    splane(ctx) {
      const { A } = lib().ss(ctx.pModel);
      return { markers: L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `eigenvalue of A ${i + 1}` })) };
    },

    math(ctx) {
      const p = ctx.pModel;
      const { A } = lib().ss(p);
      return [
        { title: 'Equations of motion', page: 'p. 67 · Eq. 4.13–4.14',
          theory: 'J_s\\ddot\\theta + b(\\dot\\theta - \\dot\\phi) + k(\\theta - \\phi) = \\tau,\\quad J_p\\ddot\\phi + b(\\dot\\phi - \\dot\\theta) + k(\\phi - \\theta) = 0' },
        { title: 'Equilibria f(xₑ, uₑ) = 0', page: 'p. 67 · Eq. 4.15',
          theory: '\\dot x = f(x, u) = 0 \\text{ at } (x_e, u_e)',
          symbolic: '\\theta_e = \\phi_e \\text{ (any value)},\\quad \\dot\\theta_e = \\dot\\phi_e = 0,\\quad \\tau_e = 0', spoiler: true,
          note: 'The model is already linear, so no linearization is needed.' },
        { title: 'Where it comes to rest (not in the book)', page: 'conservation of angular momentum',
          theory: '\\text{no external torque} \\Rightarrow \\text{total angular momentum is constant}',
          symbolic: '\\tau = 0 \\Rightarrow \\tfrac{d}{dt}(J_s\\dot\\theta + J_p\\dot\\phi) = 0 \\Rightarrow \\theta_\\infty = \\phi_\\infty = \\frac{J_s\\theta_0 + J_p\\phi_0}{J_s + J_p}',
          numbers: `\\theta_\\infty = ${tex(this.finalAngle(ctx))}^\\circ`, spoiler: true },
        { title: 'Eigenvalues of A', page: 'p. 92 (A from C.6)',
          theory: '\\det(sI - A) = 0',
          symbolic: '\\det(sI - A) = s^2\\left(s^2 + \\frac{b(J_s + J_p)}{J_sJ_p}s + \\frac{k(J_s + J_p)}{J_sJ_p}\\right)',
          numbers: `\\text{eig}(A) = ${L.eig(A).map((q) => texPole(q)).join(',\\;')}`, spoiler: true,
          note: 'Hover the s-plane markers to read the eigenvalues.' },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibrium input and relative angle',
          inputs: { te: 'τ<sub>e</sub> [N·m]', d: 'θ<sub>e</sub> − φ<sub>e</sub> [°]' },
          check: (v) => PD().checkNumbers(v, { te: 0, d: 0 }, { te: 'τe', d: 'θe − φe' }),
          solution: () => [{ tex: '\\theta_e = \\phi_e\\text{ arbitrary},\\quad \\dot\\theta_e = \\dot\\phi_e = 0,\\quad \\tau_e = 0' }, { html: 'Book: Eq. 4.15 (p. 67). Any common angle is an equilibrium.' }],
        },
        {
          id: 'x', title: 'Extension: final angle after release from the current θ(0), φ(0) with τ = 0',
          html: 'Not in the book. Hint: no external torque acts on the pair. Revealing the answer also draws it on the θ plot.',
          inputs: { th: 'θ<sub>∞</sub> [°]' },
          check: (v) => PD().checkNumbers(v, { th: this.finalAngle(ctx) }, { th: 'θ∞' }),
          actions: [{ label: 'Reveal on plot', run: () => { ctx.app.reveal('C:ch4:final'); ctx.update(); return null; } }],
          solution: () => [{ tex: `\\theta_\\infty = \\frac{J_s\\theta_0 + J_p\\phi_0}{J_s + J_p} = ${tex(this.finalAngle(ctx))}^\\circ` }],
        },
      ]);
    },
  });

  // ----------------------------------------------------- C.5 and C.6 --
  function tfData(p) {
    const { Js, Jp, k, b } = p, J = Js + Jp;
    return {
      num: [1 / Js, b / (Js * Jp), k / (Js * Jp)],                  // Θ/τ numerator
      den: [1, b * J / (Js * Jp), k * J / (Js * Jp), 0, 0],          // s²(s² + ...)
      out: { num: [b / Jp, k / Jp], den: [1, b / Jp, k / Jp] },     // Φ/Θ (Eq. 5.5)
      J,
    };
  }
  // Cascade approximation (Fig. 5-3): θ̈ = τ/(Js + Jp), panel driven by θ.
  function cascadeAB(p) {
    const J = p.Js + p.Jp;
    return {
      A: [[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0], [p.k / p.Jp, -p.k / p.Jp, p.b / p.Jp, -p.b / p.Jp]],
      B: [[0], [0], [1 / J], [0]],
    };
  }

  CH.ch5 = Object.assign({}, common, {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 77–79',
    defaults() { return { inp: { shape: 'pulse', amp: 0.1, freq: 0.05, width: 2 }, overlay: 'cascade' }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearLabel: 'cascade approximation (parts d–e)',
    linearSim(ctx, c) {
      if (ctx.st.overlay !== 'cascade') return null;
      const { A, B } = cascadeAB(ctx.pModel);
      return linearOverlay(ctx, c, A, B);
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Input τ(t)', 'p. 77');
      inputControls(sec, ctx);
      segmented(sec, {
        label: 'Dashed overlay',
        options: [{ value: 'cascade', label: 'cascade approximation (d–e)' }, { value: 'none', label: 'none' }],
        ...bind(ctx, 'overlay'),
      });
      sec.append(el('p', { class: 'muted small', text: 'The dashed traces are the cascade approximation of parts (d) and (e) (Fig. 5-3) driven by the same torque. Compare them with the full model, and vary Jp/Js in the left panel.' }));
    },
    splane(ctx) {
      const d = tfData(ctx.pModel);
      const mk = L.roots(d.den).map((q, i) => ({ ...q, kind: 'ol', label: `pole of Θ/τ ${i + 1}` }));
      L.roots(d.num).forEach((q) => mk.push({ ...q, kind: 'olzero', label: 'zero of Θ/τ (= pole of Φ/Θ)' }));
      return { markers: mk };
    },
    math(ctx) {
      const p = ctx.pModel, d = tfData(p);
      return [
        { title: 'Laplace transform of the EOM', page: 'p. 78 · Eq. 5.3–5.4',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)',
          spoiler: true,
          symbolic: '\\left(s^2 + \\tfrac{b}{J_s}s + \\tfrac{k}{J_s}\\right)\\Theta = \\left(\\tfrac{b}{J_s}s + \\tfrac{k}{J_s}\\right)\\Phi + \\tfrac{1}{J_s}\\tau,\\quad \\left(s^2 + \\tfrac{b}{J_p}s + \\tfrac{k}{J_p}\\right)\\Phi = \\left(\\tfrac{b}{J_p}s + \\tfrac{k}{J_p}\\right)\\Theta' },
        { title: 'Transfer matrix', page: 'p. 78–79 · Eq. 5.6',
          theory: '\\begin{bmatrix}\\Theta\\\\ \\Phi\\end{bmatrix} = M(s)^{-1}\\begin{bmatrix}1/J_s\\\\ 0\\end{bmatrix}\\tau',
          symbolic: '\\frac{\\Theta}{\\tau} = \\frac{\\frac{1}{J_s}s^2 + \\frac{b}{J_sJ_p}s + \\frac{k}{J_sJ_p}}{s^2\\left(s^2 + \\frac{b(J_s+J_p)}{J_sJ_p}s + \\frac{k(J_s+J_p)}{J_sJ_p}\\right)},\\quad \\frac{\\Phi}{\\tau} = \\frac{\\frac{b}{J_sJ_p}s + \\frac{k}{J_sJ_p}}{s^2\\left(s^2 + \\cdots\\right)}',
          numbers: `\\frac{\\Theta}{\\tau} = \\frac{${WB.tf.polyTex(d.num)}}{${WB.tf.polyTex(d.den)}}`, spoiler: true },
        { title: 'Panel subsystem', page: 'p. 78 · Eq. 5.5',
          theory: '\\frac{\\Phi(s)}{\\Theta(s)} = \\frac{\\Phi/\\tau}{\\Theta/\\tau}',
          symbolic: '\\frac{\\Phi(s)}{\\Theta(s)} = \\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}',
          numbers: `\\frac{\\Phi}{\\Theta} = \\frac{${WB.tf.polyTex(d.out.num)}}{${WB.tf.polyTex(d.out.den)}}`, spoiler: true },
        { title: 'Body subsystem, (Js + Jp)/Js ≈ 1', page: 'p. 79',
          theory: '\\text{factor } \\tfrac{J_s + J_p}{J_s} \\text{ out of the denominator, then set } \\tfrac{J_s}{J_s + J_p} \\approx 1',
          symbolic: '\\frac{\\Theta}{\\tau} = \\frac{\\frac{1}{J_s}\\left(s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}\\right)}{s^2\\frac{J_s+J_p}{J_s}\\left(\\frac{J_s}{J_s+J_p}s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}\\right)} \\approx \\frac{1}{(J_s + J_p)s^2}',
          numbers: `\\frac{\\Theta}{\\tau} \\approx \\frac{1}{${tex(d.J)}\\,s^2},\\quad \\frac{J_s + J_p}{J_s} = ${tex(d.J / p.Js)}`, spoiler: true,
          note: 'The cancellation is only approximate; compare the poles and zeros in the s-plane.' },
      ];
    },
    buildProblem(parent, ctx) {
      const d = () => tfData(ctx.pModel);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'b', title: '(b) Θ/τ = (n<sub>2</sub>s² + n<sub>1</sub>s + n<sub>0</sub>) / (s²(s² + d<sub>1</sub>s + d<sub>0</sub>))',
          inputs: { n2: 'n<sub>2</sub>', n1: 'n<sub>1</sub>', n0: 'n<sub>0</sub>', d1: 'd<sub>1</sub>', d0: 'd<sub>0</sub>' },
          check: (v) => { const x = d(); return PD().checkNumbers(v, { n2: x.num[0], n1: x.num[1], n0: x.num[2], d1: x.den[1], d0: x.den[2] }, {}); },
          solution: () => { const x = d(); return [{ tex: `\\frac{\\Theta}{\\tau} = \\frac{${WB.tf.polyTex(x.num)}}{s^2(s^2 + ${tex(x.den[1])}s + ${tex(x.den[2])})}` }, { html: 'Book: Eq. 5.6 (p. 79).' }]; },
        },
        {
          id: 'c', title: '(c) Φ/Θ = (n<sub>1</sub>s + n<sub>0</sub>) / (s² + d<sub>1</sub>s + d<sub>0</sub>)',
          inputs: { n1: 'n<sub>1</sub>', n0: 'n<sub>0</sub>', d1: 'd<sub>1</sub>', d0: 'd<sub>0</sub>' },
          check: (v) => PD().checkNumbers(v, { n1: d().out.num[0], n0: d().out.num[1], d1: d().out.den[1], d0: d().out.den[2] }, {}),
          solution: () => [{ tex: `n_1 = d_1 = \\tfrac{b}{J_p} = ${tex(d().out.num[0])},\\quad n_0 = d_0 = \\tfrac{k}{J_p} = ${tex(d().out.num[1])}` }, { html: 'Book: Eq. 5.5 (p. 78).' }],
        },
        {
          id: 'd', title: '(d) Θ/τ ≈ 1/(J s²)',
          inputs: { J: 'J [kg·m²]' },
          check: (v) => PD().checkNumbers(v, { J: d().J }, {}),
          solution: () => [{ tex: `J = J_s + J_p = ${tex(d().J)}` }, { html: 'Book: p. 79 and Fig. 5-3. (e): the body behaves like a rigid Js + Jp and drives the panel through the spring and damper; when Jp ≪ Js the panel barely pushes back.' }],
        },
      ]);
    },
  });

  CH.ch6 = Object.assign({}, common, {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 91–93',
    defaults() { return { inp: { shape: 'pulse', amp: 0.1, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearLabel: 'ẋ = Ax + Bu (nominal parameters)',
    linearSim(ctx, c) {
      const { A, B } = lib().ss(ctx.pModel);
      return linearOverlay(ctx, c, A, B);
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Input u = τ(t)', 'p. 91');
      inputControls(sec, ctx);
      sec.append(el('p', { class: 'muted small', text: 'The satellite is linear, so ẋ = Ax + Bu reproduces the simulation exactly. The dashed trace only separates when you add plant mismatch, saturation or a disturbance. The lower plot shows the strain-gauge output y₂ = φ − θ of C.6.' }));
    },
    splane(ctx) {
      const { A } = lib().ss(ctx.pModel);
      return { markers: L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `eigenvalue of A ${i + 1}` })) };
    },
    extraPlot(ctx, res) {
      return { opts: { title: 'measured outputs of C.6', yLabel: 'y [°]', unit: '°' }, data: { series: [
        { label: 'y₂ = φ − θ (strain gauge)', y: res.x.map((x) => (x[1] - x[0]) * R2D), color: '--series-1' },
      ] } };
    },
    math(ctx) {
      const { A, B, Cbook } = lib().ss(ctx.pModel);
      return [
        { title: 'States, input and measured outputs', page: 'p. 91',
          theory: 'x = (\\theta, \\phi, \\dot\\theta, \\dot\\phi)^\\top,\\quad u = \\tau,\\quad y = (\\theta,\\; \\phi - \\theta)^\\top' },
        { title: 'State-space model', page: 'p. 92–93',
          theory: '\\dot x = Ax + Bu,\\quad y = Cx + Du,\\quad A = \\frac{\\partial f}{\\partial x},\\; B = \\frac{\\partial f}{\\partial u}',
          symbolic: 'A = \\begin{bmatrix}0 & 0 & 1 & 0\\\\ 0 & 0 & 0 & 1\\\\ -\\frac{k}{J_s} & \\frac{k}{J_s} & -\\frac{b}{J_s} & \\frac{b}{J_s}\\\\ \\frac{k}{J_p} & -\\frac{k}{J_p} & \\frac{b}{J_p} & -\\frac{b}{J_p}\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\0\\\\ \\frac{1}{J_s}\\\\ 0\\end{bmatrix},\\quad C = \\begin{bmatrix}1 & 0 & 0 & 0\\\\ -1 & 1 & 0 & 0\\end{bmatrix},\\quad D = 0',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(Cbook)}`, spoiler: true,
          note: 'The later chapters and _C_satellite/python measure y = (θ, φ) instead.' },
        { title: 'Transfer function from the state space', page: 'p. 93',
          theory: 'H(s) = C(sI - A)^{-1}B + D',
          numbers: `\\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const s = () => lib().ss(ctx.pModel);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [
        {
          id: 'a', title: 'Rows 3 and 4 of A, and B<sub>3</sub>',
          inputs: { a31: 'A<sub>31</sub>', a32: 'A<sub>32</sub>', a33: 'A<sub>33</sub>', a34: 'A<sub>34</sub>', a41: 'A<sub>41</sub>', a42: 'A<sub>42</sub>', a43: 'A<sub>43</sub>', a44: 'A<sub>44</sub>', b3: 'B<sub>3</sub>' },
          check: (v) => { const { A, B } = s(); return PD().checkNumbers(v, { a31: A[2][0], a32: A[2][1], a33: A[2][2], a34: A[2][3], a41: A[3][0], a42: A[3][1], a43: A[3][2], a44: A[3][3], b3: B[2][0] }, {}); },
          solution: () => { const { A, B } = s(); return [{ tex: `A = ${texMat(A)},\\quad B = ${texMat(B)}` }, { html: 'Book: p. 93. Note that the C.11 and C.12 solutions print different numbers for the same A (see ISSUES.md).' }]; },
        },
        {
          id: 'c', title: 'Second row of C for y<sub>2</sub> = φ − θ',
          inputs: { c1: 'C<sub>21</sub>', c2: 'C<sub>22</sub>', c3: 'C<sub>23</sub>', c4: 'C<sub>24</sub>' },
          check: (v) => PD().checkNumbers(v, { c1: -1, c2: 1, c3: 0, c4: 0 }, {}),
          solution: () => [{ tex: 'C = \\begin{bmatrix}1 & 0 & 0 & 0\\\\ -1 & 1 & 0 & 0\\end{bmatrix},\\quad D = \\begin{bmatrix}0\\\\0\\end{bmatrix}' }],
        },
      ]);
    },
  });
})();
