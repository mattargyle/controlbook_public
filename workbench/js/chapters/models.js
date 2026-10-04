// Chapters 2-6: modeling. Open-loop experiments on the arm: kinetic energy (Ch 2),
// Euler-Lagrange equations with an energy balance check (Ch 3), equilibria and
// linearization (Ch 4), the transfer function (Ch 5), and the state-space model (Ch 6).
window.WB = window.WB || {};
WB.chapters = WB.chapters || {};

(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const PD = () => WB.pd;

  // ------------------------------------------------- open-loop torque input --
  function inputTorque(st, t) {
    const { shape, amp, freq, t0 = 0, width = 0.5 } = st.inp;
    if (t < t0) return 0;
    const tt = t - t0;
    switch (shape) {
      case 'const': return amp;
      case 'pulse': return tt < width ? amp : 0;
      case 'square': return (tt % (1 / freq)) <= 0.5 / freq ? amp : -amp;
      case 'sine': return amp * Math.sin(2 * Math.PI * freq * tt);
      default: return 0;
    }
  }

  // τ = compensation + τ_in(t). comp: 'none' | 'eq' (τ_e at θ_e) | 'fl' (τ_fl(θ)).
  function openLoop(ctx, { linear = false } = {}) {
    const { sys, pModel } = ctx;
    const st = ctx.st;
    const yE = (st.yE || 0) * M.DEG;
    return {
      update(r, x, yMeas, t) {
        const tin = inputTorque(st, t);
        if (linear || st.comp === 'none') return tin;
        if (st.comp === 'eq') return sys.equilibriumInput(yE, pModel) + tin;
        return sys.feedbackLinearization(x, pModel) + tin;
      },
    };
  }

  function inputControls(parent, ctx, { shapes = ['zero', 'const', 'pulse', 'square', 'sine'], comps } = {}) {
    const names = { zero: 'none', const: 'constant', pulse: 'pulse', square: 'square', sine: 'sine' };
    segmented(parent, {
      label: 'Input torque τ<sub>in</sub>(t)',
      options: shapes.map((s) => ({ value: s, label: names[s] })),
      ...bind(ctx, 'shape', () => ctx.st.inp),
    });
    slider(parent, { label: 'amplitude', unit: 'N·m', min: -1, max: 1, step: 0.005, sig: 3, ...bind(ctx, 'amp', () => ctx.st.inp), disabled: () => ctx.st.inp.shape === 'zero' });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.01, max: 2, step: 0.005, sig: 3, ...bind(ctx, 'freq', () => ctx.st.inp), disabled: () => !['square', 'sine'].includes(ctx.st.inp.shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.05, max: 5, step: 0.05, sig: 3, ...bind(ctx, 'width', () => ctx.st.inp), disabled: () => ctx.st.inp.shape !== 'pulse' });
    if (comps) {
      segmented(parent, {
        label: 'Gravity compensation added to τ<sub>in</sub>',
        options: comps,
        ...bind(ctx, 'comp'),
      });
    }
  }

  const common = {
    openLoop: true, metrics: false,
    controller(ctx, o) { return openLoop(ctx, o); },
  };

  function inputSeries(res) { return Array.from(res.uApplied); }

  // ------------------------------------------------------------- Chapter 2 --
  // Prescribed motion θ(t) = A sin(2π f t) (hw02_armSim.py); energy is computed, not simulated.
  WB.chapters.ch2 = Object.assign({}, common, {
    id: 'ch2', num: 2, tab: 'Ch 2', title: 'Kinetic energy', pages: 'pp. 19–40',
    defaults() { return { A: 30, f: 0.25, inp: { shape: 'zero', amp: 0, freq: 0.1 } }; },
    simDefaults(sys) { return sys.problems.ch2.sim; },
    linear: false,

    simulate(ctx, common_) {
      const { sys, pModel, st, S } = ctx;
      const Ts = S.sim.Ts, N = Math.round(S.sim.tEnd / Ts) + 1;
      const A = st.A * M.DEG, w = 2 * Math.PI * st.f;
      const res = { t: new Float64Array(N), r: new Float64Array(N), y: new Float64Array(N), yMeas: new Float64Array(N), uDemand: new Float64Array(N), u: new Float64Array(N), uApplied: new Float64Array(N), x: [], extras: { Kt: new Float64Array(N), Kr: new Float64Array(N), K: new Float64Array(N) } };
      const J = pModel.m * pModel.ell ** 2;
      for (let k = 0; k < N; k++) {
        const t = k * Ts, th = A * Math.sin(w * t), thd = A * w * Math.cos(w * t);
        res.t[k] = t; res.r[k] = th; res.y[k] = th; res.yMeas[k] = th;
        res.x.push([th, thd]);
        res.extras.Kt[k] = 0.5 * pModel.m * (pModel.ell / 2) ** 2 * thd ** 2;   // ½ m v_cmᵀ v_cm
        res.extras.Kr[k] = 0.5 * (J / 12) * thd ** 2;                           // ½ ωᵀ J ω
        res.extras.K[k] = sys.kinetic([th, thd], pModel);
      }
      return res;
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Prescribed motion θ(t) = A sin(2πft)', 'p. 28 · A.2(b)');
      slider(sec, { label: 'A', unit: '°', min: 0, max: 180, step: 1, sig: 3, ...bind(ctx, 'A') });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'f') });
      sec.append(el('p', { class: 'muted small', text: 'No dynamics here: the motion is imposed, as in hw02_armSim.py. The plot below splits the kinetic energy into translation of the center of mass and rotation about it.' }));
    },

    extraPlot(ctx, res) {
      const mJ = (a) => Array.from(a, (v) => v * 1000);
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [mJ]', unit: 'mJ' },
        data: { series: [
          { label: 'translation ½m‖v_cm‖²', y: mJ(res.extras.Kt), color: '--series-2', width: 1.5 },
          { label: 'rotation ½ωᵀJω', y: mJ(res.extras.Kr), color: '--series-3', width: 1.5 },
          { label: 'total K', y: mJ(res.extras.K), color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Thin rod inertia', page: 'p. 23 · Eq. 2.2',
          theory: 'J_{cm} = \\operatorname{diag}\\!\\left(0,\\; \\tfrac{m\\ell^2}{12},\\; \\tfrac{m\\ell^2}{12}\\right)' },
        { title: 'Arm kinematics', page: 'p. 28 · Fig. 2-8',
          theory: '\\mathbf p_{cm} = \\tfrac{\\ell}{2}\\begin{bmatrix}\\cos\\theta\\\\ \\sin\\theta\\\\ 0\\end{bmatrix},\\quad \\mathbf v_{cm} = \\tfrac{\\ell}{2}\\dot\\theta\\begin{bmatrix}-\\sin\\theta\\\\ \\cos\\theta\\\\ 0\\end{bmatrix},\\quad \\boldsymbol\\omega = \\begin{bmatrix}0\\\\0\\\\ \\dot\\theta\\end{bmatrix}' },
        { title: 'Result', page: 'p. 29',
          theory: 'K = \\tfrac12 m\\tfrac{\\ell^2}{4}\\dot\\theta^2 + \\tfrac12\\tfrac{m\\ell^2}{12}\\dot\\theta^2',
          numbers: `K = \\tfrac12\\,\\frac{m\\ell^2}{3}\\,\\dot\\theta^2 = ${tex(p.m * p.ell ** 2 / 6)}\\,\\dot\\theta^2`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) K = ½ J<sub>p</sub> θ̇²',
          html: 'J<sub>p</sub> is the inertia about the pivot; c is the coefficient in K = c θ̇².',
          inputs: { J: 'J<sub>p</sub> [kg·m²]', c: 'c' },
          check: (v) => PD().checkNumbers(v, { J: p().m * p().ell ** 2 / 3, c: p().m * p().ell ** 2 / 6 }, { J: 'Jp', c: 'c' }),
          solution: () => [{ tex: `J_p = \\frac{m\\ell^2}{12} + m\\left(\\frac{\\ell}{2}\\right)^2 = \\frac{m\\ell^2}{3} = ${tex(p().m * p().ell ** 2 / 3)},\\quad c = ${tex(p().m * p().ell ** 2 / 6)}` }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 3 --
  WB.chapters.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 41–56',
    linear: false,
    defaults(sys) { const o = sys.problems.ch3.openLoop; return { comp: 'none', inp: { shape: o.shape, amp: o.amp, freq: o.freq, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 43 · A.3(e), Listing 3.2');
      inputControls(sec, ctx, { comps: [{ value: 'none', label: 'none (hw03)' }, { value: 'fl', label: 'τ_fl(θ)' }] });
      sec.append(el('p', { class: 'muted small', text: 'hw03_armSim.py drives the arm with a ±0.2 N·m square wave. Gravity needs 0.735 N·m at θ = 0, so the arm falls and swings. The energy plot checks the EOM: E(t) − E(0) must equal the work done by τ minus the friction loss.' }));
    },

    extraPlot(ctx, res) {
      const { sys, pModel } = ctx;
      const n = res.t.length, Ts = ctx.S.sim.Ts;
      const E = new Float64Array(n), W = new Float64Array(n);
      const E0 = sys.kinetic(res.x[0], pModel) + sys.potential(res.x[0], pModel);
      let w = 0;
      for (let k = 0; k < n; k++) {
        const x = res.x[k];
        E[k] = (sys.kinetic(x, pModel) + sys.potential(x, pModel) - E0) * 1000;
        if (k > 0) {
          const pw = (xx, u) => (u - pModel.b * xx[1]) * xx[1];
          w += 0.5 * Ts * (pw(res.x[k - 1], res.uApplied[k - 1]) + pw(x, res.uApplied[k - 1]));
        }
        W[k] = w * 1000;
      }
      return {
        opts: { title: 'energy balance', yLabel: 'energy [mJ]', unit: 'mJ' },
        data: { series: [
          { label: '∫(τ − bθ̇)θ̇ dt', y: W, color: '--series-2', dash: [5, 4], width: 2 },
          { label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      const J = p.m * p.ell ** 2;
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 18 · Eq. 1.8, p. 43',
          theory: 'L = K - P,\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy', page: 'p. 44 · Fig. 3-3',
          theory: 'P = P_0 + mg\\tfrac{\\ell}{2}\\sin\\theta',
          numbers: `P - P_0 = ${tex(p.m * p.g * p.ell / 2)}\\,\\sin\\theta`, spoiler: true },
        { title: 'Equation of motion', page: 'p. 44 · Eq. 3.1',
          theory: '\\frac{m\\ell^2}{3}\\ddot\\theta + mg\\frac{\\ell}{2}\\cos\\theta = \\tau - b\\dot\\theta',
          numbers: `\\ddot\\theta = ${tex(3 / J)}\\,\\tau - ${tex(3 * p.b / J)}\\,\\dot\\theta - ${tex(3 * p.g / (2 * p.ell))}\\cos\\theta`, spoiler: true },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from Eq. 3.1',
          theory: '\\frac{d}{dt}(K + P) = \\big(\\tau - b\\dot\\theta\\big)\\dot\\theta' },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel, J = () => p().m * p().ell ** 2;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch3, [
        {
          id: 'a', title: '(a) P = P<sub>0</sub> + c sin θ',
          inputs: { c: 'c [N·m]' },
          check: (v) => PD().checkNumbers(v, { c: p().m * p().g * p().ell / 2 }, {}),
          solution: () => [{ tex: `c = mg\\tfrac{\\ell}{2} = ${tex(p().m * p().g * p().ell / 2)}` }],
        },
        {
          id: 'd', title: '(d) θ̈ = c<sub>1</sub>τ + c<sub>2</sub>θ̇ + c<sub>3</sub> cos θ',
          inputs: { c1: 'c<sub>1</sub>', c2: 'c<sub>2</sub>', c3: 'c<sub>3</sub>' },
          check: (v) => PD().checkNumbers(v, { c1: 3 / J(), c2: -3 * p().b / J(), c3: -3 * p().g / (2 * p().ell) }, {}),
          solution: () => [{ tex: `c_1 = \\tfrac{3}{m\\ell^2} = ${tex(3 / J())},\\; c_2 = -\\tfrac{3b}{m\\ell^2} = ${tex(-3 * p().b / J())},\\; c_3 = -\\tfrac{3g}{2\\ell} = ${tex(-3 * p().g / (2 * p().ell))}` }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 4 --
  WB.chapters.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68',
    defaults() { return { yE: 0, dy0: 10, method: 'jacobian', comp: 'eq', inp: { shape: 'zero', amp: 0.02, freq: 0.5, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },

    controller(ctx, o) { ctx.st.comp = ctx.st.method === 'fl' ? 'fl' : 'eq'; return openLoop(ctx, o); },

    // Override x0: start at θ_e + δθ0. The app passes x0 from the left panel, so
    // run both simulations here.
    simulate(ctx, c, plant) {
      const x0 = [(ctx.st.yE + ctx.st.dy0) * M.DEG, 0];
      return WB.sim.simulate({ ...c, x0, plant, controller: this.controller(ctx), reference: () => ctx.st.yE * M.DEG });
    },
    linearSim(ctx, c) {
      const yE = ctx.st.yE * M.DEG;
      const lin = ctx.st.method === 'fl' ? ctx.sys.stateSpace(ctx.pModel) : ctx.sys.jacobian(ctx.pModel, yE);
      const { A, B } = lin;
      const off = ctx.st.method === 'fl' ? 0 : yE;  // FL model is in absolute θ
      const res = WB.sim.simulate({
        ...c, disturbance: null, noise: null, reference: () => yE,
        x0: [(ctx.st.yE + ctx.st.dy0) * M.DEG - off, 0],
        plant: { f: (x, u) => A.map((row, i) => row[0] * x[0] + row[1] * x[1] + B[i][0] * u), h: (x) => x[0] + off, uLimit: Infinity },
        controller: { update: (r, x, y, t) => inputTorque(ctx.st, t) },
      });
      return res;
    },
    linearLabel: 'linearized model',

    buildControls(parent, ctx) {
      const sec = section(parent, 'Operating point', 'p. 63 · Eq. 4.4');
      slider(sec, { label: 'θ<sub>e</sub>', unit: '°', min: -90, max: 90, step: 1, sig: 3, ...bind(ctx, 'yE') });
      slider(sec, { label: 'δθ(0)', unit: '°', min: -60, max: 60, step: 0.5, sig: 3, hint: 'initial offset from θₑ', ...bind(ctx, 'dy0') });
      segmented(sec, {
        label: 'Linearization',
        options: [{ value: 'jacobian', label: 'Jacobian: τ = τ<sub>e</sub> + τ̃' }, { value: 'fl', label: 'feedback: τ = τ<sub>fl</sub>(θ) + τ̃' }],
        ...bind(ctx, 'method'),
      });
      const inp = section(parent, 'Input τ̃(t)', 'p. 64');
      inputControls(inp, ctx);
    },

    outputSeries(ctx, res) {
      return [{ label: 'θₑ', y: Array.from(res.t, () => ctx.st.yE), color: '--ref', dash: [6, 4], width: 1.5 }];
    },

    splane(ctx) {
      const lin = ctx.st.method === 'fl' ? ctx.sys.stateSpace(ctx.pModel) : ctx.sys.jacobian(ctx.pModel, ctx.st.yE * M.DEG);
      return { markers: L.eig(lin.A).map((p, i) => ({ ...p, kind: 'ol', label: `eigenvalue of A (${ctx.st.method})` })) };
    },

    math(ctx) {
      const p = ctx.pModel, yE = ctx.st.yE * M.DEG;
      const jac = ctx.sys.jacobian(p, yE);
      return [
        { title: 'Nonlinear model', page: 'p. 63 · Eq. 4.3',
          theory: '\\dot x = \\begin{bmatrix}\\dot\\theta\\\\ \\frac{3}{m\\ell^2}\\tau - \\frac{3b}{m\\ell^2}\\dot\\theta - \\frac{3g}{2\\ell}\\cos\\theta\\end{bmatrix}' },
        { title: 'Equilibria', page: 'p. 63 · Eq. 4.4',
          theory: '\\theta_e \\text{ arbitrary},\\quad \\dot\\theta_e = 0,\\quad \\tau_e = \\frac{mg\\ell}{2}\\cos\\theta_e',
          numbers: `\\tau_e(${fmt(ctx.st.yE, 3)}^\\circ) = ${tex(jac.ue)}\\;\\text{N}\\cdot\\text{m}`, spoiler: true },
        { title: 'Jacobian linearization', page: 'p. 60 · Eq. 4.1, p. 64 · Eq. 4.5',
          theory: '\\frac{m\\ell^2}{3}\\ddot{\\tilde\\theta} - \\frac{mg\\ell}{2}\\sin\\theta_e\\,\\tilde\\theta = \\tilde\\tau - b\\dot{\\tilde\\theta}',
          numbers: `A = ${texMat(jac.A)},\\quad \\text{eig}(A) = ${L.eig(jac.A).map((q) => texPole(q)).join(',\\;')}`, spoiler: true,
          note: 'Above the horizontal (θₑ > 0) the linearization has a right-half-plane eigenvalue: gravity pulls the arm away. Below it, the arm hangs like a pendulum.' },
        { title: 'Feedback linearization', page: 'p. 64 · Eq. 4.6–4.7',
          theory: '\\tau = \\frac{mg\\ell}{2}\\cos\\theta + \\tilde\\tau \\;\\Rightarrow\\; \\frac{m\\ell^2}{3}\\ddot\\theta = \\tilde\\tau - b\\dot\\theta \\quad\\text{(exact for all }\\theta)' },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel, yE = () => ctx.st.yE * M.DEG;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibrium torque at the current θ<sub>e</sub>',
          inputs: { te: 'τ<sub>e</sub> [N·m]' },
          check: (v) => PD().checkNumbers(v, { te: ctx.sys.equilibriumInput(yE(), p()) }, {}),
          solution: () => [{ tex: `\\tau_e = \\frac{mg\\ell}{2}\\cos\\theta_e = ${tex(ctx.sys.equilibriumInput(yE(), p()))}` }],
        },
        {
          id: 'b', title: '(b) Jacobian: θ̃̈ = a<sub>21</sub>θ̃ + a<sub>22</sub>θ̃̇ + b<sub>2</sub>τ̃ at the current θ<sub>e</sub>',
          inputs: { a21: 'a<sub>21</sub>', a22: 'a<sub>22</sub>', b2: 'b<sub>2</sub>' },
          check: (v) => { const j = ctx.sys.jacobian(p(), yE()); return PD().checkNumbers(v, { a21: j.A[1][0], a22: j.A[1][1], b2: j.B[1][0] }, {}); },
          solution: () => { const j = ctx.sys.jacobian(p(), yE()); return [{ tex: `a_{21} = \\frac{3g}{2\\ell}\\sin\\theta_e = ${tex(j.A[1][0])},\\; a_{22} = ${tex(j.A[1][1])},\\; b_2 = ${tex(j.B[1][0])}` }]; },
        },
        {
          id: 'c', title: '(c) Feedback linearization τ = c cos θ + τ̃',
          inputs: { c: 'c' },
          check: (v) => PD().checkNumbers(v, { c: p().m * p().g * p().ell / 2 }, {}),
          solution: () => [{ tex: `c = \\frac{mg\\ell}{2} = ${tex(p().m * p().g * p().ell / 2)}` }],
        },
      ]);
    },
  });

  // ------------------------------------------------------- Chapters 5 and 6 --
  function tfMarkers(ctx) {
    const { A } = ctx.sys.stateSpace(ctx.pModel);
    return { markers: L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `pole of P(s) ${i + 1}` })) };
  }
  const flInputSection = (parent, ctx, title, page) => {
    const inp = section(parent, title, page);
    inputControls(inp, ctx);
    inp.append(el('p', { class: 'muted small', text: 'The simulation adds τ_fl(θ) to this input, so the arm sees exactly the linear model. The dashed trace is the linear model; they overlap unless you add plant mismatch, saturation, or a disturbance.' }));
  };

  WB.chapters.ch5 = Object.assign({}, common, {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 69–80',
    defaults() { return { comp: 'fl', inp: { shape: 'pulse', amp: 0.02, freq: 0.5, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearLabel: 'P(s) response',
    buildControls(parent, ctx) { flInputSection(parent, ctx, 'Input τ̃(t)', 'p. 71'); },
    splane: tfMarkers,
    math(ctx) {
      const m = ctx.model;
      return [
        { title: 'Laplace transform', page: 'p. 69–70',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)' },
        { title: 'Start from the feedback-linearized EOM', page: 'p. 64 · Eq. 4.7',
          theory: '\\frac{m\\ell^2}{3}\\ddot\\theta + b\\dot\\theta = \\tilde\\tau \\;\\xrightarrow{\\mathcal L,\\ \\text{zero IC}}\\; \\left(\\frac{m\\ell^2}{3}s^2 + bs\\right)\\Theta(s) = \\tilde\\tau(s)' },
        { title: 'Transfer function', page: 'p. 72 · Eq. 5.2',
          theory: 'P(s) = \\frac{\\Theta(s)}{\\tilde\\tau(s)} = \\frac{3/m\\ell^2}{s^2 + \\frac{3b}{m\\ell^2}s}',
          numbers: `P(s) = \\frac{${tex(m.b0)}}{s^2 + ${tex(m.a1)}\\,s},\\quad \\text{poles } 0,\\; ${tex(-m.a1)}`, spoiler: true,
          note: 'The pole at 0 is the free integrator: a torque pulse leaves the arm turning, so θ ramps.' },
      ];
    },
    buildProblem(parent, ctx) {
      const m = () => ctx.model;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [{
        id: 'a', title: 'P(s) = b<sub>0</sub> / (s² + a<sub>1</sub>s + a<sub>0</sub>)',
        inputs: { b0: 'b<sub>0</sub>', a1: 'a<sub>1</sub>', a0: 'a<sub>0</sub>' },
        check: (v) => PD().checkNumbers(v, { b0: m().b0, a1: m().a1, a0: m().a0 }, {}),
        solution: () => [{ tex: `P(s) = \\frac{${tex(m().b0)}}{s^2 + ${tex(m().a1)}s}` }],
      }]);
    },
  });

  WB.chapters.ch6 = Object.assign({}, common, {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 81–93',
    defaults() { return { comp: 'fl', inp: { shape: 'pulse', amp: 0.02, freq: 0.5, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearLabel: 'ẋ = Ax + Bũ response',
    buildControls(parent, ctx) { flInputSection(parent, ctx, 'Input ũ = τ̃(t)', 'p. 87'); },
    splane: tfMarkers,
    extraPlot(ctx, res) {
      const k = 180 / Math.PI;
      return { opts: { title: 'x₂ = θ̇(t)', yLabel: 'θ̇ [°/s]', unit: '°/s' }, data: { series: [{ label: 'θ̇', y: res.x.map((x) => x[1] * k), color: '--series-1' }] } };
    },
    math(ctx) {
      const { A, B, C } = ctx.sys.stateSpace(ctx.pModel);
      return [
        { title: 'Jacobian linearization revisited', page: 'p. 83–84',
          theory: 'A = \\frac{\\partial f}{\\partial x}\\Big|_e,\\quad B = \\frac{\\partial f}{\\partial u}\\Big|_e,\\quad C = \\frac{\\partial h}{\\partial x}\\Big|_e,\\quad D = \\frac{\\partial h}{\\partial u}\\Big|_e' },
        { title: 'State-space model', page: 'p. 88 · Eq. 6.16',
          theory: 'x = \\begin{bmatrix}\\theta\\\\ \\dot\\theta\\end{bmatrix},\\quad \\dot x = \\begin{bmatrix}0 & 1\\\\ 0 & -\\frac{3b}{m\\ell^2}\\end{bmatrix}x + \\begin{bmatrix}0\\\\ \\frac{3}{m\\ell^2}\\end{bmatrix}\\tilde\\tau,\\quad y = \\begin{bmatrix}1 & 0\\end{bmatrix}x',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(C)},\\quad D = 0`, spoiler: true },
        { title: 'Back to the transfer function', page: 'p. 85 · Eq. 6.14–6.15',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\text{poles: } \\det(sI - A) = 0',
          numbers: `\\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const ss = () => ctx.sys.stateSpace(ctx.pModel);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'A and B',
        inputs: { a11: 'A<sub>11</sub>', a12: 'A<sub>12</sub>', a21: 'A<sub>21</sub>', a22: 'A<sub>22</sub>', b1: 'B<sub>1</sub>', b2: 'B<sub>2</sub>' },
        check: (v) => { const { A, B } = ss(); return PD().checkNumbers(v, { a11: A[0][0], a12: A[0][1], a21: A[1][0], a22: A[1][1], b1: B[0][0], b2: B[1][0] }, {}); },
        solution: () => { const { A, B } = ss(); return [{ tex: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\quad D = 0` }]; },
      }]);
    },
  });

  WB.models = { inputTorque, openLoop };
})();
