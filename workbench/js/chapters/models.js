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
    const flOn = flApplied(ctx);
    return {
      update(r, x, yMeas, t) {
        const tin = inputTorque(st, t);
        if (linear || !st.comp || st.comp === 'none') return tin;
        // A.4 in Work mode applies the student's own τ_e (st.tauEW, set by the slider
        // or "Use my τ_e"), so the torque readout never shows the answer; τ_fl is
        // applied only once A.4(c) is solved.
        if (st.comp === 'eq') return (ctx.S.mode === 'work' && st.tauEW !== undefined ? st.tauEW : sys.equilibriumInput(yE, pModel)) + tin;
        return (flOn ? sys.feedbackLinearization(x, pModel) : 0) + tin;
      },
    };
  }
  const flApplied = (ctx) => PD().compApplied(ctx, 'fl');

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

  // ------------------------------------------------- Python answer parts --
  // Arguments for the arm's Python answers (WB.py.check draws them at random).
  const ARGS = {
    theta: { label: 'θ', lo: -Math.PI, hi: Math.PI },
    thetadot: { label: 'θ̇', lo: -3, hi: 3 },
    tau: { label: 'τ', lo: -1, hi: 1 },
    state: { col: ['theta', 'thetadot'] },
    theta_e: { label: 'θₑ', lo: -1.5, hi: 1.5 },
    tau_tilde: { label: 'τ̃', lo: -1, hi: 1 },
    s: { label: 's', complex: true, re: [-6, 3], im: [0.3, 12] },
    kP: { label: 'kP', lo: 0.02, hi: 2 },
    kD: { label: 'kD', lo: 0.01, hi: 0.5 },
  };
  // code part: {template, check}; spec as in WB.py.check, with ARGS filled in.
  const pyPart = (ctx, spec, template) => ({ template, check: (code) => WB.py.check(ctx, { args: ARGS, ...spec }, code) });
  const thetaddotOf = (ctx, p, th, thd, tau) => ctx.sys.f([th, thd], tau, p)[1];
  const pyError = (out) => ({ ok: false, msg: out.timeout ? out.error : 'Python raised an error.', detail: [out.error, out.where, (out.stdout || '').trim()].filter(Boolean).join('\n') });
  // In Work mode, poles/eigenvalues that answer `key` stay off the s-plane until it is solved.
  const showsAnswer = (ctx, key) => ctx.S.mode === 'explore' || ctx.app.isSolved(key);

  // "Plot my answer" overlays (js/core/yours.js). Overlays are drawn in the plots'
  // units (degrees, mJ); a "your" trace joins the autoscale while it stays within
  // a few times the range of the reference it belongs next to (for angles at least
  // a full turn, so a wrong model that swings the arm around is still drawn whole).
  const Y = WB.yours;
  const tauIn = (ctx) => (t) => inputTorque(ctx.st, t);
  const yours = (label, y, ref, more, floor = 1) => Y.series(label, y, { lim: Math.max(floor, 3 * ref.reduce((m, v) => Math.max(m, Math.abs(v)), 0)), ...more });
  const yoursDeg = (label, y, ref, more) => yours(label, y, ref, more, 360);
  const toDeg = (arr) => Array.from(arr, (v) => v / M.DEG);
  // The arm's f(state, tau) in Python. WB.py.simulate runs it with the true-plant
  // parameters (plantParams) while the student's τ_fl gets the nominal ones, as the
  // workbench's own controller does.
  const PLANT_PY = 'def f(state, tau):\n    theta = state[0][0]\n    thetadot = state[1][0]\n    return np.array([[thetadot], [3 / (P.m * P.ell**2) * (tau - P.b * thetadot - P.m * P.g * P.ell / 2 * np.cos(theta))]])\n';
  // A student function of (θ, θ̇, τ) wrapped as f(state, tau) for WB.py.simulate.
  const asStateFn = (fn) => `\n\ndef _wb_f(state, tau):\n    return np.array([[state[1][0]], [${fn}(state[0][0], state[1][0], tau)]])\n`;

  // ------------------------------------------------------------- Chapter 2 --
  // A.2(a): the student's kinetic(θ, θ̇) along the prescribed motion.
  async function runKinetic(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: res.x.map(([th, thd]) => ({ name: 'kinetic', args: [th, thd] })) }]);
    if (out.error) return pyError(out);
    const K = out.rows[0].calls;
    if (!K.every((v) => typeof v === 'number')) return { ok: false, msg: 'kinetic should return one number.' };
    return { n: K.length, K };
  }

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
      sec.append(el('p', { class: 'muted small', text: ctx.S.mode === 'work'
        ? 'No dynamics here: the motion is imposed, as in hw02_armSim.py. Once (a) is solved, or you click Plot my K, the plot below shows the kinetic energy along that motion.'
        : 'No dynamics here: the motion is imposed, as in hw02_armSim.py. The plot below splits the kinetic energy into translation of the center of mass and rotation about it.' }));
    },

    // K(t) (and its split) answers A.2(a), so in Work mode the plot is empty until (a)
    // is solved, apart from the student's own K once they plot it.
    extraPlot(ctx, res) {
      const mJ = (a) => Array.from(a, (v) => v * 1000);
      const y = Y.data(ctx, 'ch2.a');
      const show = showsAnswer(ctx, 'A.2/a');
      if (!show) return { opts: { title: 'kinetic energy', yLabel: 'K [mJ]', unit: 'mJ' }, data: { series: y ? [Y.series('your K (Python)', mJ(y.K))] : [] } };
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [mJ]', unit: 'mJ' },
        data: { series: [
          { label: 'translation ½m‖v_cm‖²', y: mJ(res.extras.Kt), color: '--series-2', width: 1.5 },
          { label: 'rotation ½ωᵀJω', y: mJ(res.extras.Kr), color: '--series-3', width: 1.5 },
          { label: 'total K', y: mJ(res.extras.K), color: '--series-1' },
          ...(y ? [yours('your K (Python)', mJ(y.K), mJ(res.extras.K))] : []),
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
        { title: 'Arm kinematics', page: 'p. 28 · Fig. 2-8', answers: 'A.2/a',
          theory: '\\mathbf p_{cm} = \\tfrac{\\ell}{2}\\begin{bmatrix}\\cos\\theta\\\\ \\sin\\theta\\\\ 0\\end{bmatrix},\\quad \\mathbf v_{cm} = \\tfrac{\\ell}{2}\\dot\\theta\\begin{bmatrix}-\\sin\\theta\\\\ \\cos\\theta\\\\ 0\\end{bmatrix},\\quad \\boldsymbol\\omega = \\begin{bmatrix}0\\\\0\\\\ \\dot\\theta\\end{bmatrix}' },
        { title: 'Kinetic energy of the arm', page: 'p. 29', answers: 'A.2/a',
          theory: 'K = \\tfrac12 m\\tfrac{\\ell^2}{4}\\dot\\theta^2 + \\tfrac12\\tfrac{m\\ell^2}{12}\\dot\\theta^2 = \\tfrac12\\,\\frac{m\\ell^2}{3}\\,\\dot\\theta^2',
          numbers: `K = ${tex(p.m * p.ell ** 2 / 6)}\\,\\dot\\theta^2` },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) Kinetic energy',
          html: 'Write K as a function of θ and θ̇.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'kinetic', args: ['theta', 'thetadot'], truth: (p, a) => 0.5 * (p.m * p.ell ** 2 / 3) * a.thetadot ** 2 }],
          }, 'def kinetic(theta, thetadot):\n    # kinetic energy K of the arm\n    K = ...\n    return K\n'), {
            actions: [{ label: 'Plot my K', run: (code) => Y.plot(ctx, 'ch2.a', code, runKinetic, () => 'Your K(t) along the motion set on the right is the dotted trace on the kinetic energy plot.') }],
          }),
          solution: () => [
            { tex: 'K = \\tfrac12 m\\,v_{cm}^2 + \\tfrac12 J_{cm}\\dot\\theta^2 = \\tfrac12 m\\tfrac{\\ell^2}{4}\\dot\\theta^2 + \\tfrac12\\tfrac{m\\ell^2}{12}\\dot\\theta^2 = \\tfrac12\\,\\frac{m\\ell^2}{3}\\,\\dot\\theta^2' },
            { code: 'def kinetic(theta, thetadot):\n    return 0.5 * (P.m * P.ell**2 / 3) * thetadot**2' },
            { html: 'Book: A.2 solution (p. 29).' },
          ],
        },
        {
          id: 'b', title: '(b) Animate the arm',
          html: 'You write this class in your own <code>armAnimation.py</code> (hw02, p. 30). The animation at the top of this page shows the same prescribed motion θ(t) = A sin(2πft), set in the controls on the right.',
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 3 --
  // Last "Simulate my f" run, shown while the simulation it was run against is unchanged.
  const mine = {};
  const simSig = (ctx, res) => JSON.stringify([ctx.S.sysId, ctx.S.chapter, ctx.pTrue, ctx.S.sim.Ts, res.t.length, res.x[0], Array.from(res.uApplied).reduce((a, v, i) => a + v * (i + 1), 0)]);
  async function simulateMine(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.simulate(code, { fn: 'f', params: ctx.pTrue, x0: res.x[0], u: Array.from(res.uApplied), Ts: ctx.S.sim.Ts });
    if (out.error) return pyError(out);
    mine.ch3 = { sig: simSig(ctx, res), theta: out.x.map((x) => x[0]) };
    let dev = 0;
    out.x.forEach((x, k) => { dev = Math.max(dev, Math.abs(x[0] - res.x[k][0])); });
    ctx.update();
    const msg = `Your f is the dashed trace on the θ plot. Largest difference from the workbench's arm: ${fmt(dev / M.DEG, 3)}°.`;
    return { ok: dev / M.DEG < 0.01, msg: dev / M.DEG < 0.01 ? msg : `${msg} They should overlap.` };
  }

  // A.3(a): the student's potential(θ) along the simulated motion.
  async function runPotential(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: res.x.map(([th]) => ({ name: 'potential', args: [th] })) }]);
    if (out.error) return pyError(out);
    const P = out.rows[0].calls;
    if (!P.every((v) => typeof v === 'number' && Number.isFinite(v))) return { ok: false, msg: 'potential should return one number.' };
    return { n: P.length, P };
  }

  // A.3(c): the student's τ and −Bq̇ along the simulated motion, integrated into the
  // work they do with the same trapezoid rule as the workbench's energy balance.
  async function runForces(ctx, code) {
    const res = ctx.app.result(), n = res.t.length, Ts = ctx.S.sim.Ts;
    const pts = [];
    for (let k = 1; k < n; k++) pts.push([...res.x[k - 1], res.uApplied[k - 1]], [...res.x[k], res.uApplied[k - 1]]);
    const calls = ['generalized_force', 'damping_force'].flatMap((name) => pts.map((args) => ({ name, args })));
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls }]);
    if (out.error) return pyError(out);
    const c = out.rows[0].calls, m = pts.length;
    if (!c.every((v) => typeof v === 'number' && Number.isFinite(v))) return { ok: false, msg: 'generalized_force and damping_force should each return one number.' };
    const W = new Float64Array(n);
    let w = 0;
    for (let k = 1; k < n; k++) {
      const i = 2 * (k - 1), pw = (j) => (c[j] + c[m + j]) * pts[j][1];
      w += 0.5 * Ts * (pw(i) + pw(i + 1));
      W[k] = w;
    }
    return { n, W };
  }

  // A.3(d): the arm simulated with the student's thetaddot, same input as the arm above.
  async function runEom(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.simulate(code + asStateFn('thetaddot'), { fn: '_wb_f', params: ctx.pTrue, x0: res.x[0], u: Array.from(res.uApplied), Ts: ctx.S.sim.Ts });
    if (out.error) return pyError(out);
    return { n: out.x.length, theta: out.x.map((x) => x[0]) };
  }

  WB.chapters.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 41–56',
    linear: false,
    defaults(sys) { const o = sys.problems.ch3.openLoop; return { comp: 'none', inp: { shape: o.shape, amp: o.amp, freq: o.freq, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 43 · A.3(e), Listing 3.2');
      inputControls(sec, ctx, { comps: [{ value: 'none', label: 'none (hw03)' }, { value: 'fl', label: 'τ_fl(θ)' }] });
      sec.append(el('p', { class: 'muted small', text: ctx.S.mode === 'work'
        ? 'hw03_armSim.py drives the arm with a ±0.2 N·m square wave. That is less than gravity needs to hold the arm level, so the arm falls and swings. The energy plot checks the EOM: the change in K + P must equal the net work of the nonconservative forces. It stays empty until you plot your own P or forces (Plot my P / Plot my forces) or solve (a) or (c); E(t) − E(0) appears once (a) is solved and the work curve once (c) is solved. τ_fl is added once A.4(c) is solved (Ch 4 tab).'
        : 'hw03_armSim.py drives the arm with a ±0.2 N·m square wave. That is less than gravity needs to hold the arm level, so the arm falls and swings. The energy plot checks the EOM: E(t) − E(0) must equal the work done by the torque minus the friction loss.' }));
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
      // E(t) − E(0) uses the workbench's P (A.3(a)) and the work curve its forces
      // (A.3(c)), so Work mode draws each once its part is solved. The student's own
      // overlays are drawn whenever plotted.
      const showE = showsAnswer(ctx, 'A.3/a'), showW = showsAnswer(ctx, 'A.3/c');
      const series = [
        ...(showW ? [{ label: 'work done by τ and damping', y: W, color: '--series-2', dash: [5, 4], width: 2 }] : []),
        ...(showE ? [{ label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' }] : []),
      ];
      // Your P (any P₀) in the energy, and your forces in the work.
      const yp = Y.data(ctx, 'ch3.a'), yf = Y.data(ctx, 'ch3.c');
      if (yp) {
        const K0 = sys.kinetic(res.x[0], pModel);
        series.push(yours('E(t) − E(0) with your P', res.x.map((x, k) => (sys.kinetic(x, pModel) - K0 + yp.P[k] - yp.P[0]) * 1000), E));
      }
      if (yf) series.push(yours('work done by your τ and damping', Array.from(yf.W, (v) => v * 1000), W, { dash: [8, 3] }));
      return {
        opts: { title: 'energy balance', yLabel: 'energy [mJ]', unit: 'mJ' },
        data: { series },
      };
    },

    // A.3(e): the student's f, simulated with the same input as the arm above;
    // A.3(d): the same with their thetaddot.
    outputSeries(ctx, res, sc) {
      const m = mine.ch3, out = [];
      if (m && m.sig === simSig(ctx, res)) out.push({ label: 'your f (Python)', y: m.theta.map((v) => v / M.DEG), color: '--series-2', dash: [5, 4], width: 2 });
      const y = Y.data(ctx, 'ch3.d');
      if (y) out.push(yoursDeg('your θ̈ (Python)', toDeg(y.theta), sc(res.y)));
      return out;
    },

    math(ctx) {
      const p = ctx.pModel;
      const J = p.m * p.ell ** 2;
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 43 · §3.1.4',
          theory: 'L(q, \\dot q) = K(q, \\dot q) - P(q),\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy', page: 'p. 41–42 · §3.1.1',
          theory: '\\text{gravity: } P = mgy + P_0 \\;(y = \\text{height of the mass}),\\quad \\text{spring: } P = \\tfrac12 k z^2',
          note: 'For a rigid body, y is the height of its center of mass.' },
        { title: 'Generalized coordinates, forces, damping', page: 'p. 42–43 · §3.1.2–3.1.3',
          theory: 'q = \\text{minimum set of configuration variables},\\quad \\tau = \\text{applied (nonconservative) forces along } q,\\quad -B\\dot q = \\text{damping forces}' },
        { title: 'Kinetic energy of the arm (A.2)', page: 'p. 29', answers: 'A.2/a',
          theory: 'K = \\tfrac12\\,\\frac{m\\ell^2}{3}\\,\\dot\\theta^2',
          numbers: `K = ${tex(p.m * p.ell ** 2 / 6)}\\,\\dot\\theta^2` },
        { title: 'Potential energy of the arm', page: 'p. 44 · Fig. 3-3', answers: 'A.3/a',
          theory: 'P = P_0 + mg\\tfrac{\\ell}{2}\\sin\\theta',
          numbers: `P - P_0 = ${tex(p.m * p.g * p.ell / 2)}\\,\\sin\\theta` },
        { title: 'Equation of motion of the arm', page: 'p. 44 · Eq. 3.1', answers: 'A.3/d',
          theory: '\\frac{m\\ell^2}{3}\\ddot\\theta + mg\\frac{\\ell}{2}\\cos\\theta = \\tau - b\\dot\\theta',
          numbers: `\\ddot\\theta = ${tex(3 / J)}\\,\\tau - ${tex(3 * p.b / J)}\\,\\dot\\theta - ${tex(3 * p.g / (2 * p.ell))}\\cos\\theta` },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from p. 43',
          theory: '\\frac{d}{dt}(K + P) = \\dot q^\\top\\big(\\tau - B\\dot q\\big)',
          note: 'The plot below integrates the right side and compares it with the arm\'s energy.' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch3;
      const q = (v) => String(v || '').toLowerCase().replace(/\s+/g, '').replace(/^q(1)?=/, '').replace(/[()[\]{}]|\^t|ᵀ|'/g, '');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Potential energy',
          html: 'Write P as a function of θ. Any constant P<sub>0</sub> is accepted.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'potential', args: ['theta'], compare: 'offset', truth: (p, a) => p.m * p.g * (p.ell / 2) * Math.sin(a.theta) }],
          }, 'def potential(theta):\n    # potential energy P of the arm\n    return ...\n'), {
            actions: [{ label: 'Plot my P', run: (code) => Y.plot(ctx, 'ch3.a', code, runPotential, () => 'Dotted on the energy plot: E(t) − E(0) along the simulated motion with your P in place of the workbench\'s. With the right P it balances the work done by τ and damping.') }],
          }),
          solution: () => [
            { tex: 'P = P_0 + mg\\tfrac{\\ell}{2}\\sin\\theta \\quad(\\text{height of the center of mass: } \\tfrac{\\ell}{2}\\sin\\theta)' },
            { code: 'def potential(theta):\n    return P.m * P.g * (P.ell / 2) * np.sin(theta)' },
          ],
        },
        {
          id: 'b', title: '(b) Generalized coordinates',
          inputs: { q: 'q =' },
          html: 'Name them, comma separated (e.g. <code>theta</code> or <code>θ</code>).',
          check: (v) => {
            const g = q(v.q);
            if (!g) return { ok: false, msg: 'Enter the generalized coordinates.' };
            if (/dot|̇|ω|omega/.test(g)) return { ok: false, msg: 'Generalized coordinates are configuration variables (positions and angles), not velocities.' };
            return ['theta', 'θ', 'q1'].includes(g) ? { ok: true, msg: 'One coordinate, so q is a scalar here.' } : { ok: false, msg: 'Which variables fix the configuration of the arm? Use the minimum number.' };
          },
          solution: () => [{ tex: 'q = \\theta' }, { html: 'Book: p. 42 (§3.1.2) and p. 44.' }],
        },
        {
          id: 'c', title: '(c) Generalized forces and damping forces',
          html: 'Write the damping term the way the book does, as the force −Bq̇ (p. 43).',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'generalized_force', args: ['theta', 'thetadot', 'tau'], truth: (p, a) => a.tau },
              { fn: 'damping_force', args: ['theta', 'thetadot', 'tau'], truth: (p, a) => -p.b * a.thetadot },
            ],
            explain: (it, f) => {
              const [g, w] = [f.e.got, f.e.want].map(Number);
              return it.fn === 'damping_force' && Math.abs(g + w) < 1e-6 * Math.max(1, Math.abs(w)) ? 'Check the sign: the book writes the damping force as −Bq̇.' : '';
            },
          }, 'def generalized_force(theta, thetadot, tau):\n    # applied force along q\n    return ...\n\ndef damping_force(theta, thetadot, tau):\n    # damping force, -B*q_dot\n    return ...\n'), {
            actions: [{ label: 'Plot my forces', run: (code) => Y.plot(ctx, 'ch3.c', code, runForces, () => 'Dotted on the energy plot: the work your generalized force and damping force do along the simulated motion. With the right forces it balances E(t) − E(0).') }],
          }),
          solution: () => [
            { tex: '\\tau_1 = \\tau,\\quad -B\\dot q = -b\\dot\\theta' },
            { code: 'def generalized_force(theta, thetadot, tau):\n    return tau\n\ndef damping_force(theta, thetadot, tau):\n    return -P.b * thetadot' },
            { html: 'Book: p. 44.' },
          ],
        },
        {
          id: 'd', title: '(d) Equations of motion',
          html: 'Apply the Euler-Lagrange equation, then solve for θ̈.',
          code: Object.assign(pyPart(ctx, {
            cases: [
              { label: 'with θ̇ = 0 and τ = 0 (only gravity acts)', fix: { thetadot: 0, tau: 0 } },
              { label: 'with θ = 90° and θ̇ = 0 (only the torque acts)', fix: { theta: Math.PI / 2, thetadot: 0 } },
              { label: 'with θ = 90° and τ = 0 (only damping acts)', fix: { theta: Math.PI / 2, tau: 0 } },
              { label: '' },
            ],
            items: [{ fn: 'thetaddot', args: ['theta', 'thetadot', 'tau'], truth: (p, a) => thetaddotOf(ctx, p, a.theta, a.thetadot, a.tau) }],
          }, 'def thetaddot(theta, thetadot, tau):\n    # from the equation of motion\n    return ...\n'), {
            actions: [{ label: 'Simulate my θ̈', run: (code) => Y.plot(ctx, 'ch3.d', code, runEom, () => 'The arm simulated with your thetaddot (RK4, same torque input and true-plant parameters as above) is the dotted trace on the θ plot.') }],
          }),
          solution: () => [
            { tex: 'L = \\tfrac12\\frac{m\\ell^2}{3}\\dot\\theta^2 - P_0 - mg\\tfrac{\\ell}{2}\\sin\\theta,\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot\\theta} = \\frac{m\\ell^2}{3}\\ddot\\theta,\\quad \\frac{\\partial L}{\\partial\\theta} = -mg\\tfrac{\\ell}{2}\\cos\\theta' },
            { tex: '\\frac{m\\ell^2}{3}\\ddot\\theta + mg\\frac{\\ell}{2}\\cos\\theta = \\tau - b\\dot\\theta' },
            { code: 'def thetaddot(theta, thetadot, tau):\n    return 3 / (P.m * P.ell**2) * (tau - P.b * thetadot - P.m * P.g * P.ell / 2 * np.cos(theta))' },
            { html: 'Book: Eq. 3.1 (p. 44).' },
          ],
        },
        {
          id: 'e', title: '(e) Implement and simulate',
          html: 'Write f(x, u) as in <code>armDynamics.py</code>: the state is a 2×1 column. <em>Check</em> tests it at random states like <code>testDynamics.py</code>. <em>Simulate my f</em> runs it with RK4 on the same torque input as the arm above (true-plant parameters) and draws θ dashed on the plot.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'f', args: ['state', 'tau'], truth: (p, a) => [[a.thetadot], [thetaddotOf(ctx, p, a.theta, a.thetadot, a.tau)]] }],
          }, 'def f(state, tau):\n    theta = state[0][0]\n    thetadot = state[1][0]\n    thetaddot = ...\n    return np.array([[thetadot], [thetaddot]])\n'), {
            actions: [{ label: 'Simulate my f', run: (code) => simulateMine(ctx, code) }],
          }),
          solution: () => [{ code: 'def f(state, tau):\n    theta = state[0][0]\n    thetadot = state[1][0]\n    thetaddot = 3 / (P.m * P.ell**2) * (tau - P.b * thetadot - P.m * P.g * P.ell / 2 * np.cos(theta))\n    return np.array([[thetadot], [thetaddot]])' }, { html: 'Book: Listing 3.2 (p. 45).' }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 4 --
  // A.4(a): τ_e(θ_e) at the current θ_e becomes the "your τ_e" torque.
  async function useMyTauE(ctx, code) {
    const yE = ctx.st.yE;
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: [{ name: 'tau_e', args: [yE * M.DEG] }] }]);
    if (out.error) return pyError(out);
    const te = out.rows[0].calls[0];
    if (typeof te !== 'number' || !Number.isFinite(te)) return { ok: false, msg: 'tau_e should return one number.' };
    Object.assign(ctx.st, { tauEW: te, method: 'jacobian' });
    ctx.update();
    const head = `Your τ_e(${fmt(yE, 3)}°) = ${fmt(te, 4)} N·m.`;
    if (ctx.S.mode !== 'work') return { info: true, msg: `${head} Explore mode applies the workbench's τ_e; switch to Work mode to apply yours.` };
    return { info: true, msg: `${head} It is now "your τ_e" on the right: with the right τ_e and δθ(0) = 0 the arm stays at θₑ. Click again after moving θₑ.` };
  }

  // A.4(b): the student's A_jac, B_jac at the current θ_e.
  async function runJacobian(ctx, code) {
    const yE = ctx.st.yE * M.DEG;
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: [{ name: 'A_jac', args: [yE] }, { name: 'B_jac', args: [yE] }] }]);
    if (out.error) return pyError(out);
    const A = Y.asMat(out.rows[0].calls[0], 2, 2), B = Y.asMat(out.rows[0].calls[1], 2, 1);
    if (!A) return { ok: false, msg: 'A_jac should return a 2×2 array of numbers.' };
    if (!B) return { ok: false, msg: 'B_jac should return a 2×1 array of numbers.' };
    return { A, B };
  }

  // A.4(c): the true arm with the student's τ = τ_fl(θ, θ̇) + τ̃ (simulated in Python
  // like the workbench's own run: same input, disturbance and saturation; τ_fl uses
  // the nominal parameters), the student's thetaddot_fl model driven by τ̃, and that
  // model's A.
  async function runFeedbackLin(ctx, code) {
    const res = ctx.app.result(), Ts = ctx.S.sim.Ts;
    const tin = Array.from(res.t, (t) => inputTorque(ctx.st, t));
    const d = Array.from(res.uApplied, (v, k) => v - res.u[k]);
    const arm = await WB.py.simulate(code, { plant: PLANT_PY, plantParams: ctx.pTrue, ctrl: 'tau_fl', params: ctx.pModel, x0: res.x[0], u: tin, d, uLimit: ctx.sys.uLimit(ctx.pTrue), Ts });
    if (arm.error) return pyError(arm);
    const model = await WB.py.simulate(code + asStateFn('thetaddot_fl'), { fn: '_wb_f', params: ctx.pModel, x0: res.x[0], u: tin, Ts });
    if (model.error) return pyError(model);
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: [[0, 0, 0], [1, 0, 0], [0, 1, 0]].map((args) => ({ name: 'thetaddot_fl', args })) }]);
    if (out.error) return pyError(out);
    const [c0, cth, cthd] = out.rows[0].calls.map(Number);
    return { n: res.t.length, theta: arm.x.map((x) => x[0]), thetaM: model.x.map((x) => x[0]), A: [[0, 1], [cth - c0, cthd - c0]] };
  }

  WB.chapters.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68',
    defaults() { return { yE: 0, dy0: 10, method: 'jacobian', comp: 'eq', tauEW: 0, inp: { shape: 'zero', amp: 0.02, freq: 0.5, width: 0.5 } }; },
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
      if (ctx.S.mode === 'work') {
        slider(sec, { label: 'your τ<sub>e</sub>', unit: 'N·m', min: -1.5, max: 1.5, step: 0.001, sig: 3, ...bind(ctx, 'tauEW'), disabled: () => ctx.st.method !== 'jacobian' });
        sec.append(el('p', { class: 'muted small', text: 'Jacobian: Work mode applies your τ_e. Test an answer: with the right τ_e and δθ(0) = 0 the arm stays at θₑ. Feedback: the feedback-linearizing torque is applied once you solve (c); until then τ = τ̃ (Simulate my τ_fl applies yours).' }));
      } else {
        sec.append(el('p', { class: 'muted small', text: 'Explore applies the correct equilibrium or feedback-linearizing torque.' }));
      }
      const inp = section(parent, 'Input τ̃(t)', 'p. 64');
      inputControls(inp, ctx);
    },

    // Your models next to the linearized model (same δθ(0), τ̃ and θₑ offset), and
    // the arm with your τ_fl next to the simulated arm.
    outputSeries(ctx, res, sc) {
      const out = [{ label: 'θₑ', y: Array.from(res.t, () => ctx.st.yE), color: '--ref', dash: [6, 4], width: 1.5 }];
      const ref = sc(res.y);
      if (ctx.st.method === 'fl') {
        const y = Y.data(ctx, 'ch4.c');
        if (y) out.push(yoursDeg('arm with your τ_fl', toDeg(y.theta), ref), yoursDeg('your thetaddot_fl model', toDeg(y.thetaM), ref, { dash: [8, 3] }));
      } else {
        const y = Y.data(ctx, 'ch4.b');
        if (y) {
          const r = Y.linResponse(ctx, { ...y, C: [[1, 0]], D: [[0]] }, [ctx.st.dy0 * M.DEG, 0], tauIn(ctx)).y[0];
          out.push(yoursDeg('your A_jac, B_jac', Array.from(r, (v) => v / M.DEG + ctx.st.yE), ref));
        }
      }
      return out;
    },

    splane(ctx) {
      const fl = ctx.st.method === 'fl';
      let sp = { markers: [] };
      if (showsAnswer(ctx, fl ? 'A.4/c' : 'A.4/b')) {
        const lin = fl ? ctx.sys.stateSpace(ctx.pModel) : ctx.sys.jacobian(ctx.pModel, ctx.st.yE * M.DEG);
        sp = { markers: L.eig(lin.A).map((p, i) => ({ ...p, kind: 'ol', label: `eigenvalue of A (${ctx.st.method})` })) };
      }
      const y = Y.data(ctx, fl ? 'ch4.c' : 'ch4.b');
      if (!y) return sp;
      return fl ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your thetaddot_fl model'), { obs: 'your τ_fl model' })
        : Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your A_jac'), { obs: 'your A_jac' });
    },

    math(ctx) {
      const p = ctx.pModel, yE = ctx.st.yE * M.DEG;
      const jac = ctx.sys.jacobian(p, yE);
      return [
        { title: 'Equilibria', page: 'p. 60',
          theory: '\\dot x = f(x, u):\\quad (x_e, u_e) \\text{ is an equilibrium when } f(x_e, u_e) = 0' },
        { title: 'Jacobian linearization', page: 'p. 60 · Eq. 4.1',
          theory: '\\tilde x = x - x_e,\\; \\tilde u = u - u_e:\\quad \\dot{\\tilde x} \\approx \\frac{\\partial f}{\\partial x}\\Big|_{(x_e, u_e)}\\tilde x + \\frac{\\partial f}{\\partial u}\\Big|_{(x_e, u_e)}\\tilde u' },
        { title: 'Feedback linearization', page: 'p. 64',
          theory: 'u = u_{fl}(x) + \\tilde u',
          note: 'u_fl cancels the nonlinear terms, so the model from ũ is linear for every x, not only near an equilibrium.' },
        { title: 'Equation of motion (A.3)', page: 'p. 44 · Eq. 3.1', answers: 'A.3/d',
          theory: '\\dot x = \\begin{bmatrix}\\dot\\theta\\\\ \\frac{3}{m\\ell^2}\\tau - \\frac{3b}{m\\ell^2}\\dot\\theta - \\frac{3g}{2\\ell}\\cos\\theta\\end{bmatrix}' },
        { title: 'Equilibria of the arm', page: 'p. 63 · Eq. 4.4', answers: 'A.4/a',
          theory: '\\theta_e \\text{ arbitrary},\\quad \\dot\\theta_e = 0,\\quad \\tau_e = \\frac{mg\\ell}{2}\\cos\\theta_e',
          numbers: `\\tau_e(${fmt(ctx.st.yE, 3)}^\\circ) = ${tex(jac.ue)}\\;\\text{N}\\cdot\\text{m}` },
        { title: 'Jacobian-linearized arm', page: 'p. 64 · Eq. 4.5', answers: 'A.4/b',
          theory: '\\frac{m\\ell^2}{3}\\ddot{\\tilde\\theta} - \\frac{mg\\ell}{2}\\sin\\theta_e\\,\\tilde\\theta = \\tilde\\tau - b\\dot{\\tilde\\theta}',
          numbers: `A = ${texMat(jac.A)},\\quad \\text{eig}(A) = ${L.eig(jac.A).map((q) => texPole(q)).join(',\\;')}`,
          note: 'Above the horizontal (θₑ > 0) the linearization has a right-half-plane eigenvalue: gravity pulls the arm away. Below it, the arm hangs like a pendulum.' },
        { title: 'Feedback-linearized arm', page: 'p. 64 · Eq. 4.6–4.7', answers: 'A.4/c',
          theory: '\\tau = \\frac{mg\\ell}{2}\\cos\\theta + \\tilde\\tau \\;\\Rightarrow\\; \\frac{m\\ell^2}{3}\\ddot\\theta = \\tilde\\tau - b\\dot\\theta \\quad\\text{(exact for all }\\theta)' },
      ];
    },

    buildProblem(parent, ctx) {
      const tf = (p, th) => p.m * p.g * (p.ell / 2) * Math.cos(th);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibria',
          html: 'Which states can be equilibria, and what torque holds the arm there? Write that torque as a function of the equilibrium angle.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'tau_e', args: ['theta_e'], truth: (p, a) => tf(p, a.theta_e) }],
          }, 'def tau_e(theta_e):\n    # holds the arm at rest at theta_e\n    return ...\n'), {
            actions: [{ label: 'Use my τ_e', run: (code) => useMyTauE(ctx, code) }],
          }),
          solution: () => [
            { tex: '\\dot\\theta_e = 0,\\; \\ddot\\theta_e = 0 \\Rightarrow \\tau_e = \\frac{mg\\ell}{2}\\cos\\theta_e \\quad(\\theta_e \\text{ arbitrary})' },
            { code: 'def tau_e(theta_e):\n    return P.m * P.g * P.ell / 2 * np.cos(theta_e)' },
          ],
        },
        {
          id: 'b', title: '(b) Jacobian linearization',
          html: 'With x̃ = (θ̃, θ̃̇)ᵀ and ũ = τ̃, return the matrices of x̃̇ = A x̃ + B ũ about the equilibrium at θ<sub>e</sub>.',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'A_jac', args: ['theta_e'], truth: (p, a) => ctx.sys.jacobian(p, a.theta_e).A },
              { fn: 'B_jac', args: ['theta_e'], truth: (p, a) => ctx.sys.jacobian(p, a.theta_e).B },
            ],
          }, 'def A_jac(theta_e):\n    return ...\n\ndef B_jac(theta_e):\n    return ...\n'), {
            actions: [{ label: 'Plot my A, B', run: (code) => { ctx.st.method = 'jacobian'; return Y.plot(ctx, 'ch4.b', code, runJacobian, () => 'Your eigenvalues are the × markers on the s-plane, and your linear model (from the same δθ(0) and τ̃, shifted by θₑ) is dotted on the θ plot next to the linearized model. They follow θₑ and the input.'); } }],
          }),
          solution: () => [
            { tex: '\\frac{m\\ell^2}{3}\\ddot{\\tilde\\theta} - \\frac{mg\\ell}{2}\\sin\\theta_e\\,\\tilde\\theta = \\tilde\\tau - b\\dot{\\tilde\\theta}' },
            { code: 'def A_jac(theta_e):\n    return np.array([[0, 1],\n                     [3 * P.g / (2 * P.ell) * np.sin(theta_e), -3 * P.b / (P.m * P.ell**2)]])\n\ndef B_jac(theta_e):\n    return np.array([[0], [3 / (P.m * P.ell**2)]])' },
            { html: 'Book: Eq. 4.5 (p. 64).' },
          ],
        },
        {
          id: 'c', title: '(c) Feedback linearization',
          html: 'Choose τ = τ<sub>fl</sub>(θ, θ̇) + τ̃ so the resulting model is linear for every θ, and return that model\'s θ̈. Any τ<sub>fl</sub> that does this is accepted.',
          code: { template: 'def tau_fl(theta, thetadot):\n    return ...\n\ndef thetaddot_fl(theta, thetadot, tau_tilde):\n    # theta_ddot when tau = tau_fl + tau_tilde\n    return ...\n', check: (code) => checkFeedbackLin(ctx, code),
            actions: [{ label: 'Simulate my τ_fl', run: (code) => { ctx.st.method = 'fl'; return Y.plot(ctx, 'ch4.c', code, runFeedbackLin, () => 'Dotted on the θ plot: the arm with your τ = τ_fl + τ̃, and (long dashes) your thetaddot_fl model driven by τ̃. With a correct answer they overlap. The × markers are your model\'s eigenvalues.'); } }] },
          solution: () => [
            { tex: '\\tau = \\frac{mg\\ell}{2}\\cos\\theta + \\tilde\\tau \\;\\Rightarrow\\; \\frac{m\\ell^2}{3}\\ddot\\theta = \\tilde\\tau - b\\dot\\theta' },
            { code: 'def tau_fl(theta, thetadot):\n    return P.m * P.g * P.ell / 2 * np.cos(theta)\n\ndef thetaddot_fl(theta, thetadot, tau_tilde):\n    return 3 / (P.m * P.ell**2) * (tau_tilde - P.b * thetadot)' },
            { html: 'Book: Eqs. 4.6–4.7 (p. 64).' },
          ],
        },
      ]);
    },
  });

  // A.4(c): the arm's θ̈ with τ = tau_fl + τ̃ must equal the student's thetaddot_fl,
  // and thetaddot_fl must be linear in (θ, θ̇, τ̃).
  async function checkFeedbackLin(ctx, code) {
    if (!code.trim()) return { ok: false, msg: 'Write your code first.' };
    const sets = WB.py.paramSets(ctx);
    let seed = 99;
    const r = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    const pt = () => [(2 * r() - 1) * Math.PI, (2 * r() - 1) * 3, (2 * r() - 1)];
    const samples = sets.map(() => {
      const pts = Array.from({ length: 6 }, pt), pairs = Array.from({ length: 3 }, () => [pt(), pt()]);
      const lin = pairs.flatMap(([x, y]) => [x, y, x.map((v, i) => v + y[i])]);
      return { pts, lin, calls: [
        ...pts.map(([th, thd]) => ({ name: 'tau_fl', args: [th, thd] })),
        ...pts.map((x) => ({ name: 'thetaddot_fl', args: x })),
        ...lin.map((x) => ({ name: 'thetaddot_fl', args: x })),
        { name: 'thetaddot_fl', args: [0, 0, 0] },
      ] };
    });
    const out = await WB.py.evaluate(code, sets.map((s, i) => ({ params: s.p, calls: samples[i].calls })));
    if (out.error) return pyError(out);
    const close = (a, b) => Math.abs(a - b) <= 1e-6 * Math.max(1, Math.abs(a), Math.abs(b));
    for (let i = 0; i < sets.length; i++) {
      const c = out.rows[i].calls.map(Number), { pts, lin } = samples[i], n = pts.length;
      const g = (k) => c[2 * n + k];
      const linear = close(c[c.length - 1], 0) && [0, 1, 2].every((k) => close(g(3 * k) + g(3 * k + 1), g(3 * k + 2)));
      if (!linear) return { ok: false, msg: `thetaddot_fl is not linear in (θ, θ̇, τ̃) with ${sets.text(sets[i])}. τ_fl has to cancel every nonlinear term.` };
      for (let k = 0; k < n; k++) {
        const [th, thd, tt] = pts[k];
        const want = thetaddotOf(ctx, sets[i].p, th, thd, c[k] + tt), got = c[n + k];
        if (!close(got, want)) {
          if (i > 0) {  // the nominal set (i = 0) passed
            return { ok: false, msg: `Matches at the nominal parameters but not with ${sets.text(sets[i])}. Write it with ${sets.vary.map((v) => `P.${v}`).join(', ')} rather than numbers.` };
          }
          return { ok: false, msg: `At θ = ${fmt(th, 3)}, θ̇ = ${fmt(thd, 3)}, τ̃ = ${fmt(tt, 3)}: the arm with τ = tau_fl + τ̃ has θ̈ = ${fmt(want, 4)}, but thetaddot_fl gives ${fmt(got, 4)}.` };
        }
      }
    }
    return { ok: true, msg: `Consistent with the arm and linear, at ${sets.length} parameter sets.` };
  }

  // ------------------------------------------------------- Chapters 5 and 6 --
  function tfMarkers(ctx, key) {
    if (!showsAnswer(ctx, key)) return { markers: [] };
    const { A } = ctx.sys.stateSpace(ctx.pModel);
    return { markers: L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `pole of P(s) ${i + 1}` })) };
  }
  const flInputSection = (parent, ctx, title, page) => {
    const inp = section(parent, title, page);
    inputControls(inp, ctx);
    inp.append(el('p', { class: 'muted small', text: ctx.S.mode === 'work'
      ? `Once A.4(c) is solved, the simulation adds τ_fl to this input, so the arm sees exactly the linear model; until then τ = τ̃ and the arm falls under gravity. The dashed trace is the linear model.`
      : 'The simulation adds τ_fl(θ) to this input, so the arm sees exactly the linear model. The dashed trace is the linear model; they overlap unless you add plant mismatch, saturation, or a disturbance.' }));
  };

  // A.5: the student's transfer_function(s), fitted with the lowest-order proper
  // rational function that matches it.
  const runTf = (ctx, code) => Y.fitTf(ctx, code, 'transfer_function');

  // A.6: the student's A, B, C, D.
  async function runSS(ctx, code) {
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, vars: ['A', 'B', 'C', 'D'] }]);
    if (out.error) return pyError(out);
    const v = out.rows[0].vars;
    const A = Y.asMat(v.A, 2, 2), B = Y.asMat(v.B, 2, 1), C = Y.asMat(v.C, 1, 2), D = Y.asMat(v.D, 1, 1);
    const bad = [[A, 'A', '2×2'], [B, 'B', '2×1'], [C, 'C', '1×2'], [D, 'D', '1×1']].find(([m]) => !m);
    if (bad) return { ok: false, msg: `${bad[1]} should be a ${bad[2]} array of numbers (x = (θ, θ̇), u = τ̃, y = θ).` };
    return { A, B, C, D };
  }

  WB.chapters.ch5 = Object.assign({}, common, {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 69–80',
    defaults() { return { comp: 'fl', inp: { shape: 'pulse', amp: 0.02, freq: 0.5, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearLabel: 'P(s) response',
    buildControls(parent, ctx) { flInputSection(parent, ctx, 'Input τ̃(t)', 'p. 71'); },
    splane(ctx) {
      const sp = tfMarkers(ctx, 'A.5/a');
      const y = Y.data(ctx, 'ch5.a');
      if (!y) return sp;
      return Y.withMarkers(sp, [...Y.poles(WB.tf.poles(y.G), 'pole of your P(s)'), ...Y.zeros(WB.tf.zeros(y.G), 'zero of your P(s)')],
        { obs: 'pole of your P(s)', zero: 'zero of your P(s)' });
    },
    // Your P(s) driven by the same τ̃ from rest (a transfer function assumes zero initial conditions).
    outputSeries(ctx, res, sc) {
      const y = Y.data(ctx, 'ch5.a');
      return y ? [yoursDeg('your P(s)', toDeg(Y.tfResponse(ctx, y.G, tauIn(ctx))), sc(res.y))] : [];
    },
    math(ctx) {
      const m = ctx.model;
      return [
        { title: 'Laplace transform', page: 'p. 69–70',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)' },
        { title: 'Transfer function', page: 'p. 70–71',
          theory: '\\text{zero initial conditions:}\\quad P(s) = \\frac{Y(s)}{U(s)}' },
        { title: 'Feedback-linearized EOM (A.4)', page: 'p. 64 · Eq. 4.7', answers: 'A.4/c',
          theory: '\\frac{m\\ell^2}{3}\\ddot\\theta + b\\dot\\theta = \\tilde\\tau' },
        { title: 'Transfer function of the arm', page: 'p. 72 · Eq. 5.2', answers: 'A.5/a',
          theory: '\\left(\\frac{m\\ell^2}{3}s^2 + bs\\right)\\Theta(s) = \\tilde\\tau(s) \\;\\Rightarrow\\; P(s) = \\frac{\\Theta(s)}{\\tilde\\tau(s)} = \\frac{3/m\\ell^2}{s^2 + \\frac{3b}{m\\ell^2}s}',
          numbers: `P(s) = \\frac{${tex(m.b0)}}{s^2 + ${tex(m.a1)}\\,s},\\quad \\text{poles } 0,\\; ${tex(-m.a1)}`,
          note: 'The pole at 0 is the free integrator: a torque pulse leaves the arm turning, so θ ramps.' },
      ];
    },
    buildProblem(parent, ctx) {
      const cx = WB.py.cx;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [{
        id: 'a', title: 'Transfer function from τ̃ to θ',
        html: 'Return P(s) = Θ(s)/τ̃(s) evaluated at a complex s (the check calls it at several).',
        code: Object.assign(pyPart(ctx, {
          items: [{ fn: 'transfer_function', args: ['s'], truth: (p, a) => { const m = ctx.sys.secondOrderModel(p); return cx.div(m.b0, cx.poly([1, m.a1, m.a0], a.s)); } }],
        }, 'def transfer_function(s):\n    # Theta(s)/tau_tilde(s) at complex s\n    return ...\n'), {
          actions: [{ label: 'Plot my P(s)', run: (code) => Y.plot(ctx, 'ch5.a', code, runTf, (d) => `Your P(s) has ${d.G.den.length - 1} pole${d.G.den.length === 2 ? '' : 's'} (× on the s-plane); its response to the same τ̃, from rest, is dotted on the θ plot.`) }],
        }),
        solution: () => [
          { tex: `P(s) = \\frac{3/m\\ell^2}{s^2 + \\frac{3b}{m\\ell^2}s} = \\frac{${tex(ctx.model.b0)}}{s^2 + ${tex(ctx.model.a1)}s}` },
          { code: 'def transfer_function(s):\n    return (3 / (P.m * P.ell**2)) / (s**2 + 3 * P.b / (P.m * P.ell**2) * s)' },
          { html: 'Book: Eq. 5.2 (p. 72).' },
        ],
      }]);
    },
  });

  WB.chapters.ch6 = Object.assign({}, common, {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 81–93',
    defaults() { return { comp: 'fl', inp: { shape: 'pulse', amp: 0.02, freq: 0.5, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearLabel: 'ẋ = Ax + Bũ response',
    buildControls(parent, ctx) { flInputSection(parent, ctx, 'Input ũ = τ̃(t)', 'p. 87'); },
    splane(ctx) {
      const sp = tfMarkers(ctx, 'A.6/a');
      const y = Y.data(ctx, 'ch6.a');
      return y ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your A'), { obs: 'eigenvalue of your A' }) : sp;
    },
    // Your model from the simulation's initial state, driven by the same τ̃.
    yourRun(ctx, res) {
      const y = Y.data(ctx, 'ch6.a');
      return y ? Y.linResponse(ctx, y, res.x[0], tauIn(ctx)) : null;
    },
    outputSeries(ctx, res, sc) {
      const r = this.yourRun(ctx, res);
      return r ? [yoursDeg('your A, B, C, D', toDeg(r.y[0]), sc(res.y))] : [];
    },
    extraPlot(ctx, res) {
      const thd = res.x.map((x) => x[1] / M.DEG);
      // The linear model's x₂ too (as on the θ plot): τ_fl is held over each step of
      // the simulated arm, so its θ̇ drifts slightly from the exact linear model.
      const lin = Y.linResponse(ctx, ctx.ss, res.x[0], tauIn(ctx)).x.map((x) => x[1] / M.DEG);
      const series = [{ label: this.linearLabel, y: lin, color: '--series-2', dash: [5, 4], width: 1.5, fit: false }, { label: 'θ̇', y: thd, color: '--series-1' }];
      const r = this.yourRun(ctx, res);
      if (r) series.push(yoursDeg('your x₂', r.x.map((x) => x[1] / M.DEG), thd));
      return { opts: { title: 'x₂ = θ̇(t)', yLabel: 'θ̇ [°/s]', unit: '°/s' }, data: { series } };
    },
    math(ctx) {
      const { A, B, C } = ctx.sys.stateSpace(ctx.pModel);
      return [
        { title: 'Jacobian linearization revisited', page: 'p. 83–84',
          theory: 'A = \\frac{\\partial f}{\\partial x}\\Big|_e,\\quad B = \\frac{\\partial f}{\\partial u}\\Big|_e,\\quad C = \\frac{\\partial h}{\\partial x}\\Big|_e,\\quad D = \\frac{\\partial h}{\\partial u}\\Big|_e' },
        { title: 'State-space model of the arm', page: 'p. 88 · Eq. 6.16', answers: 'A.6/a',
          theory: 'x = \\begin{bmatrix}\\theta\\\\ \\dot\\theta\\end{bmatrix},\\quad \\dot x = \\begin{bmatrix}0 & 1\\\\ 0 & -\\frac{3b}{m\\ell^2}\\end{bmatrix}x + \\begin{bmatrix}0\\\\ \\frac{3}{m\\ell^2}\\end{bmatrix}\\tilde\\tau,\\quad y = \\begin{bmatrix}1 & 0\\end{bmatrix}x',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(C)},\\quad D = 0` },
        { title: 'Back to the transfer function', page: 'p. 85 · Eq. 6.14–6.15',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\text{poles: } \\det(sI - A) = 0',
          numbers: `\\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true, answers: ['A.5/a', 'A.6/a'] },
      ];
    },
    buildProblem(parent, ctx) {
      const ss = (p) => ctx.sys.stateSpace(p);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'A, B, C, D',
        code: Object.assign(pyPart(ctx, {
          items: ['A', 'B', 'C', 'D'].map((k) => ({ var: k, truth: (p) => ss(p)[k] })),
        }, '# x = (theta, thetadot), u = tau_tilde, y = theta\nA = ...\nB = ...\nC = ...\nD = ...\n'), {
          actions: [{ label: 'Plot my model', run: (code) => Y.plot(ctx, 'ch6.a', code, runSS, () => 'Your eigenvalues of A are the × markers on the s-plane; your model\'s y and x₂, from the same initial state and τ̃, are dotted on the plots.') }],
        }),
        solution: () => { const { A, B } = ss(ctx.pModel); return [
          { tex: `A = \\begin{bmatrix}0 & 1\\\\ 0 & -\\frac{3b}{m\\ell^2}\\end{bmatrix} = ${texMat(A)},\\quad B = \\begin{bmatrix}0\\\\ \\frac{3}{m\\ell^2}\\end{bmatrix} = ${texMat(B)},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\quad D = 0` },
          { code: 'A = np.array([[0, 1],\n              [0, -3 * P.b / (P.m * P.ell**2)]])\nB = np.array([[0], [3 / (P.m * P.ell**2)]])\nC = np.array([[1, 0]])\nD = np.array([[0]])' },
          { html: 'Book: Eq. 6.16 (p. 88).' },
        ]; },
      }]);
    },
  });

  WB.models = { inputTorque, openLoop };
})();
