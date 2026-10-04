// Design Study B, Chapters 2–6: modeling the pendulum on a cart. Kinetic energy
// (B.2), Euler-Lagrange equations with an energy-balance check (B.3), equilibria
// and Jacobian linearization (B.4), the transfer-function cascade (B.5), and the
// state-space model (B.6). All open loop: the force is a prescribed input.
//
// Work mode: every derivation is a Python answer (WB.py.check), checked against
// this study's own f, linearization and state-space model at random arguments
// and random parameter sets. Live-math cards that state this study's results
// carry `answers` and stay locked until the part is solved.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
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
      ...bind(ctx, 'shape', () => ctx.st.inp),
    });
    slider(parent, { label: 'amplitude', unit: 'N', min: -ampMax, max: ampMax, step: ampMax / 500, sig: 3, ...bind(ctx, 'amp', () => ctx.st.inp), disabled: () => ctx.st.inp.shape === 'zero' });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.01, max: 3, step: 0.01, sig: 3, ...bind(ctx, 'freq', () => ctx.st.inp), disabled: () => !['square', 'sine'].includes(ctx.st.inp.shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.02, max: 3, step: 0.01, sig: 3, ...bind(ctx, 'width', () => ctx.st.inp), disabled: () => ctx.st.inp.shape !== 'pulse' });
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
        // not hidden once it runs away: the trace doesn't join the autoscale, so it
        // just leaves the plot (and "your" overlays can be compared with it)
        h: (x) => [ze + x[0], thetaE + x[1]],
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

  // ------------------------------------------------- Python answer parts --
  // Arguments for the pendulum's Python answers (WB.py.check draws them at random).
  const ARGS = {
    z: { label: 'z', lo: -2, hi: 2 },
    theta: { label: 'θ', lo: -Math.PI, hi: Math.PI },
    zdot: { label: 'ż', lo: -2, hi: 2 },
    thetadot: { label: 'θ̇', lo: -3, hi: 3 },
    F: { label: 'F', lo: -5, hi: 5 },
    state: { col: ['z', 'theta', 'zdot', 'thetadot'] },
    k: { label: 'k', lo: 0, hi: 0 },
    z_e: { label: 'zₑ', lo: -2, hi: 2 },
    th_e: { label: 'θₑ', lo: 0, hi: 0 },
    zt: { label: 'z̃', lo: -1, hi: 1 },
    tht: { label: 'θ̃', lo: -0.5, hi: 0.5 },
    zdt: { label: 'ż̃', lo: -1, hi: 1 },
    thdt: { label: 'θ̇̃', lo: -1, hi: 1 },
    x_t: { col: ['zt', 'tht', 'zdt', 'thdt'] },
    F_t: { label: 'F̃', lo: -2, hi: 2 },
    s: { label: 's', complex: true, re: [-6, 3], im: [0.3, 12] },
  };
  // code part: {template, check}; spec as in WB.py.check, with ARGS filled in.
  const pyPart = (ctx, spec, template) => ({ template, check: (code) => WB.py.check(ctx, { args: ARGS, ...spec }, code) });
  const pyError = (out) => ({ ok: false, msg: out.timeout ? out.error : 'Python raised an error.', detail: [out.error, out.where, (out.stdout || '').trim()].filter(Boolean).join('\n') });
  // In Work mode, poles/eigenvalues that answer `key` stay off the s-plane until it is solved.
  const showsAnswer = (ctx, key) => ctx.S.mode === 'explore' || ctx.app.isSolved(key);
  const accelOf = (ctx, p, a) => { const d = ctx.sys.f([a.z, a.theta, a.zdot, a.thetadot], a.F, p); return [d[2], d[3]]; };

  // C (sI − A)⁻¹ B at a complex s, for each output (single input, D = 0).
  function tfAt(ss, s) {
    const cx = WB.py.cx, n = ss.A.length;
    const neg = (v) => cx.mul(-1, v), abs = (v) => Math.hypot(v.re, v.im);
    const Mx = ss.A.map((row, i) => [...row.map((a, j) => cx.add(i === j ? s : 0, -a)), cx.of(ss.B[i][0])]);
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (abs(Mx[r][c]) > abs(Mx[piv][c])) piv = r;
      [Mx[c], Mx[piv]] = [Mx[piv], Mx[c]];
      for (let r = c + 1; r < n; r++) {
        const f = cx.div(Mx[r][c], Mx[c][c]);
        for (let k = c; k <= n; k++) Mx[r][k] = cx.add(Mx[r][k], neg(cx.mul(f, Mx[c][k])));
      }
    }
    const x = new Array(n);
    for (let r = n - 1; r >= 0; r--) {
      let acc = Mx[r][n];
      for (let k = r + 1; k < n; k++) acc = cx.add(acc, neg(cx.mul(Mx[r][k], x[k])));
      x[r] = cx.div(acc, Mx[r][r]);
    }
    return ss.C.map((row) => row.reduce((acc, c, j) => cx.add(acc, cx.mul(c, x[j])), cx.of(0)));
  }
  // [Z̃/F̃, Θ̃/F̃] at s, from Eq. 6.17 (b0: with b = 0, as B.5(b) simplifies them).
  const tfs = (ctx, p, s, b0 = false) => tfAt(ctx.sys.stateSpace(b0 ? { ...p, b: 0 } : p), s);

  // ------------------------------------------------- "Plot my answer" overlays --
  // js/core/yours.js. Linear models are drawn next to the linear-model trace (same
  // initial deviation, input F̃ and equilibrium offset), not the nonlinear pendulum.
  const Y = WB.yours;
  const fin = (ctx) => (t) => inputForce(ctx.st.inp, t);
  const finArr = (ctx) => Array.from(ctx.app.result().t, fin(ctx));
  // A linear model that runs away joins the autoscale only below these |z| [m], |θ| [°].
  const LIN_LIM = [5, 200];
  const isNum = (v) => typeof v === 'number' && Number.isFinite(v);
  const isVec = (v, n) => Array.isArray(v) && v.length === n && v.every(isNum);
  // Student's accelerations as a 4-state f for WB.py.simulate.
  const accelF = (fn, call) => `\n\ndef _wb_f(state, F):\n    _a = np.asarray(${call}, dtype=float).flatten()\n    if _a.size != 2:\n        raise ValueError(f'${fn} returned {_a.size} values; expected 2')\n    return np.array([[state[2][0]], [state[3][0]], [_a[0]], [_a[1]]])\n`;
  // Y.fitTf on a helper function, with the student's name in the messages.
  async function fitMine(ctx, code, fn, name, opts) {
    const r = await Y.fitTf(ctx, code, fn, opts);
    return r.ok === false && !r.detail ? { ...r, msg: r.msg.split(fn).join(name) } : r;
  }

  // ------------------------------------------------------------- Chapter 2 --
  // B.2(a): the student's kinetic(z, θ, ż, θ̇) along the prescribed motion.
  async function runKinetic(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: res.x.map((x) => ({ name: 'kinetic', args: x })) }]);
    if (out.error) return pyError(out);
    const K = out.rows[0].calls;
    if (!K.every(isNum)) return { ok: false, msg: 'kinetic should return one number.' };
    return { n: K.length, K };
  }

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
      slider(sec, { label: 'z amplitude', unit: 'm', min: 0, max: 1.5, step: 0.01, sig: 3, ...bind(ctx, 'Az') });
      slider(sec, { label: 'z frequency', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'fz') });
      slider(sec, { label: 'θ amplitude', unit: '°', min: 0, max: 90, step: 1, sig: 3, ...bind(ctx, 'Ath') });
      slider(sec, { label: 'θ frequency', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'fth') });
      const plot = ctx.S.mode === 'explore' ? 'The plot below splits K into the cart, the rod\'s translation, and the rod\'s rotation.' : 'Once (a) is solved, or you click Plot my K, the plot below shows the kinetic energy along that motion.';
      sec.append(el('p', { class: 'muted small', text: `z(t) = A_z sin(2πf_z t) and θ(t) = A_θ sin(2πf_θ t). No dynamics here: the motion is imposed, as in hw02_pendulumSim.py (which uses a square wave for θ). ${plot}` }));
    },

    // K(t) and its split answer B.2(a), so in Work mode the plot is empty until (a) is
    // solved, apart from the student's own K once they plot it.
    extraPlot(ctx, res) {
      const show = showsAnswer(ctx, 'B.2/a'), y = Y.data(ctx, 'ch2.a');
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [J]', unit: 'J' },
        data: { series: [
          ...(show ? [
            { label: 'cart', y: Array.from(res.extras.Kc), color: '--series-2', width: 1.5 },
            { label: 'rod translation ½m₁‖v₁‖²', y: Array.from(res.extras.Kt), color: '--series-3', width: 1.5 },
            { label: 'rod rotation ½ωᵀJω', y: Array.from(res.extras.Kr), color: '--text-muted', width: 1.5 },
            { label: 'total K', y: Array.from(res.extras.K), color: '--series-1' },
          ] : []),
          ...(y ? [Y.series('your K (Python)', y.K)] : []),
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Kinetic energy of rigid bodies', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\sum_i \\Big(\\tfrac12 m_i\\,\\mathbf v_{cm,i}^\\top\\mathbf v_{cm,i} + \\tfrac12\\boldsymbol\\omega_i^\\top J_{cm,i}\\boldsymbol\\omega_i\\Big)',
          note: 'A point mass has no rotational term. Velocities are the time derivatives of the positions in the inertial frame.' },
        { title: 'Thin rod inertia', page: 'p. 23 · Eq. 2.2',
          theory: 'J_{cm} = \\operatorname{diag}\\!\\left(0,\\; \\tfrac{m\\ell^2}{12},\\; \\tfrac{m\\ell^2}{12}\\right)' },
        { title: 'Positions and velocities', page: 'p. 31–32 · Fig. 2-9', answers: 'B.2/a',
          theory: '\\mathbf p_1 = \\begin{pmatrix} z + \\frac{\\ell}{2}\\sin\\theta\\\\ \\frac{\\ell}{2}\\cos\\theta\\\\ 0\\end{pmatrix},\\quad \\mathbf v_1 = \\begin{pmatrix}\\dot z + \\frac{\\ell}{2}\\dot\\theta\\cos\\theta\\\\ -\\frac{\\ell}{2}\\dot\\theta\\sin\\theta\\\\ 0\\end{pmatrix},\\quad \\mathbf v_2 = \\begin{pmatrix}\\dot z\\\\0\\\\0\\end{pmatrix}' },
        { title: 'Kinetic energy of the pendulum on a cart', page: 'p. 32 · Eq. 2.4', answers: 'B.2/a',
          theory: 'K = \\tfrac12 m_1\\mathbf v_1^\\top\\mathbf v_1 + \\tfrac12\\frac{m_1\\ell^2}{12}\\dot\\theta^2 + \\tfrac12 m_2\\dot z^2 = \\tfrac12(m_1 + m_2)\\dot z^2 + \\tfrac12 m_1\\frac{\\ell^2}{3}\\dot\\theta^2 + m_1\\frac{\\ell}{2}\\dot z\\dot\\theta\\cos\\theta',
          numbers: `K = ${tex((p.m1 + p.m2) / 2)}\\,\\dot z^2 + ${tex(p.m1 * p.ell ** 2 / 6)}\\,\\dot\\theta^2 + ${tex(p.m1 * p.ell / 2)}\\,\\dot z\\dot\\theta\\cos\\theta` },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) Kinetic energy',
          html: 'Using the configuration variables z and θ, write K as a function of z, θ, ż and θ̇.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'kinetic', args: ['z', 'theta', 'zdot', 'thetadot'], truth: (p, a) => ctx.sys.kinetic([a.z, a.theta, a.zdot, a.thetadot], p) }],
          }, 'def kinetic(z, theta, zdot, thetadot):\n    # kinetic energy K of the system\n    K = ...\n    return K\n'), {
            actions: [{ label: 'Plot my K', run: (code) => Y.plot(ctx, 'ch2.a', code, runKinetic, () => 'Your K(t) along the motion set on the right is the dotted trace on the kinetic-energy plot.') }],
          }),
          solution: () => [
            { tex: 'K = \\tfrac12 m_1\\Big[(\\dot z + \\tfrac{\\ell}{2}\\dot\\theta\\cos\\theta)^2 + (\\tfrac{\\ell}{2}\\dot\\theta\\sin\\theta)^2\\Big] + \\tfrac12\\frac{m_1\\ell^2}{12}\\dot\\theta^2 + \\tfrac12 m_2\\dot z^2' },
            { tex: '= \\tfrac12(m_1 + m_2)\\dot z^2 + \\tfrac12 m_1\\frac{\\ell^2}{3}\\dot\\theta^2 + m_1\\frac{\\ell}{2}\\dot z\\dot\\theta\\cos\\theta' },
            { code: 'def kinetic(z, theta, zdot, thetadot):\n    return (0.5 * (P.m1 + P.m2) * zdot**2\n            + 0.5 * P.m1 * P.ell**2 / 3 * thetadot**2\n            + P.m1 * P.ell / 2 * zdot * thetadot\n            * np.cos(theta))' },
            { html: 'Book: Eq. 2.4 (p. 32). The rod\'s translation and rotation terms in θ̇² add to ½(m₁ℓ²/3)θ̇²; the cross term comes from v₁ mixing ż and θ̇.' },
          ],
        },
        {
          id: 'b', title: '(b) Animate the inverted pendulum',
          html: 'You write this class in your own <code>pendulumAnimation.py</code> (hw02, Listing 2.3, p. 33–34). The animation at the top of this page draws the cart, the rod and the bob from q = (z, θ)ᵀ with the sinusoidal motion set in the controls on the right.',
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
    mine.ch3 = { sig: simSig(ctx, res), z: out.x.map((x) => x[0]), theta: out.x.map((x) => x[1]) };
    let dz = 0, dth = 0;
    out.x.forEach((x, k) => { dz = Math.max(dz, Math.abs(x[0] - res.x[k][0])); dth = Math.max(dth, Math.abs(x[1] - res.x[k][1])); });
    ctx.update();
    const ok = dz < 1e-3 && dth / DEG < 0.05;
    const msg = `Your f is the dashed trace on the z and θ plots. Largest difference from the workbench's pendulum: ${fmt(dz * 1000, 3)} mm in z, ${fmt(dth / DEG, 3)}° in θ.`;
    return { ok, msg: ok ? msg : `${msg} They should overlap.` };
  }

  // B.3(a): E(t) − E(0) along the simulated motion, with the student's P (and K as in B.2).
  async function runPotential(ctx, code) {
    const res = ctx.app.result(), p = ctx.pModel;
    const out = await WB.py.evaluate(code, [{ params: p, calls: res.x.map((x) => ({ name: 'potential', args: [x[0], x[1]] })) }]);
    if (out.error) return pyError(out);
    const P = out.rows[0].calls;
    if (!P.every(isNum)) return { ok: false, msg: 'potential should return one number.' };
    const E = res.x.map((x, k) => ctx.sys.kinetic(x, p) + P[k]), E0 = E[0];
    return { n: E.length, E: E.map((v) => v - E0) };
  }

  // B.3(c): work done by the student's τ and −Bq̇, ∫ q̇ᵀ(τ − Bq̇) dt, integrated like
  // the workbench's trace (trapezoid rule, force held over each step).
  async function runForces(ctx, code) {
    const res = ctx.app.result(), n = res.t.length, Ts = ctx.S.sim.Ts;
    const calls = [];
    for (let k = 1; k < n; k++) {
      for (const x of [res.x[k - 1], res.x[k]]) calls.push(...['tau', 'damping'].map((name) => ({ name, args: [...x, res.uApplied[k - 1]] })));
    }
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls }]);
    if (out.error) return pyError(out);
    const v = out.rows[0].calls.map((c) => [c].flat(Infinity));
    const bad = v.findIndex((c) => !isVec(c, 2));
    if (bad >= 0) return { ok: false, msg: `${bad % 2 ? 'damping' : 'tau'} should return two numbers, along (z, θ).` };
    const pw = (x, t, d) => x[2] * (t[0] + d[0]) + x[3] * (t[1] + d[1]);
    const W = new Float64Array(n);
    for (let k = 1; k < n; k++) {
      const i = 4 * (k - 1);
      W[k] = W[k - 1] + 0.5 * Ts * (pw(res.x[k - 1], v[i], v[i + 1]) + pw(res.x[k], v[i + 2], v[i + 3]));
    }
    return { n, W };
  }

  // B.3(d): the student's accelerations simulated like "Simulate my f" (same input,
  // initial state and true-plant parameters).
  async function runAccel(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.simulate(code + accelF('accel', 'accel(*state[:, 0].tolist(), F)'), { fn: '_wb_f', params: ctx.pTrue, x0: res.x[0], u: Array.from(res.uApplied), Ts: ctx.S.sim.Ts });
    if (out.error) return pyError(out);
    return { n: out.x.length, z: out.x.map((x) => x[0]), theta: out.x.map((x) => x[1]) };
  }

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
        ...bind(ctx, 'extra'),
      });
      sec.append(el('p', { class: 'muted small', text: `hw03_pendulumSim.py pushes the cart with F = sin(2πt) N. Nothing holds the pendulum up, so it falls and swings under the track. The energy plot checks the EOM: E(t) − E(0) must equal the net work done by the nonconservative forces.${ctx.S.mode === 'explore' ? '' : ' In Work mode it shows E(t) − E(0) once (a) is solved and the work once (c) is solved, and is empty until then; Plot my P and Plot my forces draw yours.'}` }));
    },

    extraPlot(ctx, res) {
      if (ctx.st.extra !== 'energy') return stateExtra(res, ctx.st.extra);
      const yP = Y.data(ctx, 'ch3.a'), yF = Y.data(ctx, 'ch3.c');
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
      // E(t) − E(0) uses the workbench's P (B.3a) and the work curve its forces and
      // damping (B.3c): Work mode shows each once its part is solved, the student's
      // own overlays whenever plotted, and nothing until then.
      const showE = showsAnswer(ctx, 'B.3/a'), showW = showsAnswer(ctx, 'B.3/c');
      return {
        opts: { title: 'energy balance', yLabel: 'energy [J]', unit: 'J' },
        data: { series: [
          ...(showW ? [{ label: 'work done by F and damping', y: W, color: '--series-2', dash: [5, 4], width: 2 }] : []),
          ...(showE ? [{ label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' }] : []),
          ...(yP ? [Y.series('E(t) − E(0) with your P', yP.E)] : []),
          ...(yF ? [{ ...Y.series('work done by your τ and damping', yF.W), dash: [8, 3] }] : []),
        ] },
      };
    },

    // B.3(e): the student's f, simulated with the same input as the pendulum above.
    // B.3(d): the student's accelerations, simulated the same way.
    outputSeries(ctx, res, sc, oi) {
      const m = mine.ch3, out = [];
      if (m && m.sig === simSig(ctx, res)) out.push({ label: 'your f (Python)', y: sc(oi === 0 ? m.z : m.theta), color: '--series-2', dash: [5, 4], width: 2 });
      const y = Y.data(ctx, 'ch3.d');
      if (y) out.push(Y.series('your accel', sc(oi === 0 ? y.z : y.theta), { lim: oi === 0 ? 50 : 1e4 }));
      return out;
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 43 · §3.1.4',
          theory: 'L(q, \\dot q) = K(q, \\dot q) - P(q),\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy', page: 'p. 41–42 · §3.1.1',
          theory: '\\text{gravity: } P = mgy + P_0 \\;(y = \\text{height of the mass}),\\quad \\text{spring: } P = \\tfrac12 k z^2',
          note: 'For a rigid body, y is the height of its center of mass.' },
        { title: 'Generalized coordinates, forces, damping', page: 'p. 42–43 · §3.1.2–3.1.3',
          theory: 'q = \\text{minimum set of configuration variables},\\quad \\tau = \\text{applied (nonconservative) forces along } q,\\quad -B\\dot q = \\text{damping forces}' },
        { title: 'Kinetic energy of the pendulum on a cart (B.2)', page: 'p. 32 · Eq. 2.4', answers: 'B.2/a',
          theory: 'K = \\tfrac12(m_1 + m_2)\\dot z^2 + \\tfrac12 m_1\\frac{\\ell^2}{3}\\dot\\theta^2 + m_1\\frac{\\ell}{2}\\dot z\\dot\\theta\\cos\\theta',
          numbers: `K = ${tex((p.m1 + p.m2) / 2)}\\,\\dot z^2 + ${tex(p.m1 * p.ell ** 2 / 6)}\\,\\dot\\theta^2 + ${tex(p.m1 * p.ell / 2)}\\,\\dot z\\dot\\theta\\cos\\theta` },
        { title: 'Potential energy of the pendulum', page: 'p. 47 · Fig. 3-4', answers: 'B.3/a',
          theory: 'P = P_0 + m_1 g\\frac{\\ell}{2}(\\cos\\theta - 1)',
          numbers: `P - P_0 = ${tex(p.m1 * p.g * p.ell / 2)}\\,(\\cos\\theta - 1)` },
        { title: 'Coordinates, forces and damping of the pendulum', page: 'p. 47', answers: ['B.3/b', 'B.3/c'],
          theory: 'q = (z, \\theta)^\\top,\\quad \\tau = (F, 0)^\\top,\\quad -B\\dot q = (-b\\dot z, 0)^\\top' },
        { title: 'Equations of motion of the pendulum', page: 'p. 49 · Eq. 3.2', answers: 'B.3/d',
          theory: '\\begin{pmatrix} m_1 + m_2 & m_1\\frac{\\ell}{2}\\cos\\theta\\\\ m_1\\frac{\\ell}{2}\\cos\\theta & m_1\\frac{\\ell^2}{3}\\end{pmatrix}\\begin{pmatrix}\\ddot z\\\\ \\ddot\\theta\\end{pmatrix} = \\begin{pmatrix} m_1\\frac{\\ell}{2}\\dot\\theta^2\\sin\\theta + F - b\\dot z\\\\ m_1 g\\frac{\\ell}{2}\\sin\\theta\\end{pmatrix}',
          numbers: `\\begin{pmatrix} ${tex(p.m1 + p.m2)} & ${tex(p.m1 * p.ell / 2)}\\cos\\theta\\\\ ${tex(p.m1 * p.ell / 2)}\\cos\\theta & ${tex(p.m1 * p.ell ** 2 / 3)}\\end{pmatrix}\\ddot q = \\begin{pmatrix} ${tex(p.m1 * p.ell / 2)}\\,\\dot\\theta^2\\sin\\theta + F - ${tex(p.b)}\\,\\dot z\\\\ ${tex(p.m1 * p.g * p.ell / 2)}\\sin\\theta\\end{pmatrix}` },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from p. 43',
          theory: '\\frac{d}{dt}(K + P) = \\dot q^\\top\\big(\\tau - B\\dot q\\big)',
          note: 'The energy plot integrates the right side and compares it with the pendulum\'s energy.' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch3;
      const norm = (v) => String(v || '').toLowerCase().replace(/\s+/g, '').replace(/^q=/, '').replace(/[()[\]{}]|\^t|ᵀ|'/g, '').replace(/θ/g, 'theta');
      const ff = (it, f) => (Array.isArray(f.e.got) ? f.e.got : [f.e.got]).map(Number);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Potential energy',
          html: 'Write P as a function of z and θ. Any constant P<sub>0</sub> is accepted.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'potential', args: ['z', 'theta'], compare: 'offset', truth: (p, a) => ctx.sys.potential([a.z, a.theta, 0, 0], p) }],
          }, 'def potential(z, theta):\n    # potential energy P of the system\n    return ...\n'), {
            actions: [{ label: 'Plot my P', run: (code) => { ctx.st.extra = 'energy'; return Y.plot(ctx, 'ch3.a', code, runPotential, () => 'Dotted on the energy plot: E(t) − E(0) of the simulated pendulum with your P (and K from B.2). The energy balance says it should follow the work done by F and the damping.'); } }],
          }),
          solution: () => [
            { tex: 'P = P_0 + m_1 g\\tfrac{\\ell}{2}(\\cos\\theta - 1) \\quad(\\text{height of the rod center: } \\tfrac{\\ell}{2}\\cos\\theta)' },
            { code: 'def potential(z, theta):\n    return P.m1 * P.g * P.ell / 2 * (np.cos(theta) - 1)' },
            { html: 'Book: p. 47. The cart moves horizontally, so it adds only a constant.' },
          ],
        },
        {
          id: 'b', title: '(b) Generalized coordinates',
          inputs: { q: 'q =' },
          html: 'Name them in order, comma separated (e.g. <code>x, phi</code>).',
          check: (v) => {
            const g = norm(v.q);
            if (!g) return { ok: false, msg: 'Enter the generalized coordinates.' };
            if (/dot|̇|ω|omega|v/.test(g)) return { ok: false, msg: 'Generalized coordinates are configuration variables (positions and angles), not velocities.' };
            if (g === 'z,theta' || g === 'q1,q2') return { ok: true, msg: 'Two coordinates, so q, τ and −Bq̇ are 2-vectors.' };
            if (g === 'theta,z') return { ok: false, msg: 'Right variables. Use the book\'s order, cart position first, so the vectors below line up.' };
            return { ok: false, msg: 'Which variables fix the configuration of the cart and the rod? Use the minimum number.' };
          },
          solution: () => [{ tex: 'q = (z, \\theta)^\\top' }, { html: 'Book: p. 47.' }],
        },
        {
          id: 'c', title: '(c) Generalized forces and damping forces',
          html: 'Return each as a 2-vector along your generalized coordinates, in the order of (b). Write the damping term the way the book does, as the force −Bq̇ (p. 43).',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'tau', args: ['z', 'theta', 'zdot', 'thetadot', 'F'], truth: (p, a) => [a.F, 0] },
              { fn: 'damping', args: ['z', 'theta', 'zdot', 'thetadot', 'F'], truth: (p, a) => [-p.b * a.zdot, 0] },
            ],
            explain: (it, f) => {
              const g = ff(it, f), w = [].concat(f.e.want).map(Number);
              return it.fn === 'damping' && g.length === 2 && Math.abs(g[0] + w[0]) < 1e-6 * Math.max(1, Math.abs(w[0])) ? 'Check the sign: the book writes the damping force as −Bq̇.' : '';
            },
          }, 'def tau(z, theta, zdot, thetadot, F):\n    # generalized forces along q\n    return np.array([..., ...])\n\ndef damping(z, theta, zdot, thetadot, F):\n    # damping forces, -B @ qdot\n    return np.array([..., ...])\n'), {
            actions: [{ label: 'Plot my forces', run: (code) => { ctx.st.extra = 'energy'; return Y.plot(ctx, 'ch3.c', code, runForces, () => 'Dotted (long dashes) on the energy plot: the work your τ and damping forces do on the simulated pendulum, ∫ q̇ᵀ(τ − Bq̇) dt. The energy balance says it should follow E(t) − E(0).'); } }],
          }),
          solution: () => [
            { tex: '\\tau = (F, 0)^\\top,\\quad -B\\dot q = (-b\\dot z, 0)^\\top' },
            { code: 'def tau(z, theta, zdot, thetadot, F):\n    return np.array([F, 0])\n\ndef damping(z, theta, zdot, thetadot, F):\n    return np.array([-P.b * zdot, 0])' },
            { html: 'Book: p. 47. F pushes along z; nothing applies a torque along θ, and only the cart has friction.' },
          ],
        },
        {
          id: 'd', title: '(d) Equations of motion',
          html: 'Apply the Euler-Lagrange equations, then solve for both accelerations.',
          code: Object.assign(pyPart(ctx, {
            cases: [
              { label: 'with ż = θ̇ = 0 and F = 0 (only gravity acts)', fix: { zdot: 0, thetadot: 0, F: 0 } },
              { label: 'with θ = 0, ż = θ̇ = 0 (only the force acts)', fix: { theta: 0, zdot: 0, thetadot: 0 } },
              { label: 'with θ = 0, θ̇ = 0 and F = 0 (only damping acts)', fix: { theta: 0, thetadot: 0, F: 0 } },
              { label: 'with ż = 0 and F = 0 (gravity and the θ̇² terms)', fix: { zdot: 0, F: 0 } },
              { label: '' },
            ],
            items: [{ fn: 'accel', args: ['z', 'theta', 'zdot', 'thetadot', 'F'], truth: (p, a) => accelOf(ctx, p, a) }],
          }, 'def accel(z, theta, zdot, thetadot, F):\n    # from the equations of motion\n    zddot = ...\n    thetaddot = ...\n    return np.array([zddot, thetaddot])\n'), {
            actions: [{ label: 'Simulate my accel', run: (code) => Y.plot(ctx, 'ch3.d', code, runAccel, () => 'Dotted on the z and θ plots: your equations of motion simulated with RK4 from the same initial state and force as the pendulum above.') }],
          }),
          solution: () => [
            { tex: '\\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} = \\begin{pmatrix}(m_1 + m_2)\\ddot z + m_1\\frac{\\ell}{2}\\ddot\\theta\\cos\\theta - m_1\\frac{\\ell}{2}\\dot\\theta^2\\sin\\theta\\\\ m_1\\frac{\\ell^2}{3}\\ddot\\theta + m_1\\frac{\\ell}{2}\\ddot z\\cos\\theta - m_1\\frac{\\ell}{2}\\dot z\\dot\\theta\\sin\\theta\\end{pmatrix},\\quad \\frac{\\partial L}{\\partial q} = \\begin{pmatrix}0\\\\ -m_1\\frac{\\ell}{2}\\dot z\\dot\\theta\\sin\\theta + m_1 g\\frac{\\ell}{2}\\sin\\theta\\end{pmatrix}' },
            { tex: '\\begin{pmatrix} m_1 + m_2 & m_1\\frac{\\ell}{2}\\cos\\theta\\\\ m_1\\frac{\\ell}{2}\\cos\\theta & m_1\\frac{\\ell^2}{3}\\end{pmatrix}\\begin{pmatrix}\\ddot z\\\\ \\ddot\\theta\\end{pmatrix} = \\begin{pmatrix} m_1\\frac{\\ell}{2}\\dot\\theta^2\\sin\\theta + F - b\\dot z\\\\ m_1 g\\frac{\\ell}{2}\\sin\\theta\\end{pmatrix}' },
            { code: 'def accel(z, theta, zdot, thetadot, F):\n    c, s = np.cos(theta), np.sin(theta)\n    Mq = np.array([\n        [P.m1 + P.m2, P.m1 * P.ell / 2 * c],\n        [P.m1 * P.ell / 2 * c, P.m1 * P.ell**2 / 3]])\n    rhs = np.array([\n        P.m1 * P.ell / 2 * thetadot**2 * s\n        + F - P.b * zdot,\n        P.m1 * P.g * P.ell / 2 * s])\n    return np.linalg.solve(Mq, rhs)' },
            { html: 'Book: Eq. 3.2 (p. 49).' },
          ],
        },
        {
          id: 'e', title: '(e) Implement and simulate',
          html: 'Write f(x, u) as in <code>pendulumDynamics.py</code>: the state is a 4×1 column (z, θ, ż, θ̇). <em>Check</em> tests it at random states like <code>testDynamics.py</code>. <em>Simulate my f</em> runs it with RK4 on the same force input as the pendulum above (true-plant parameters) and draws z and θ dashed on the plots. In your own code, connect it to the B.2 animation.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'f', args: ['state', 'F'], truth: (p, a) => ctx.sys.f([a.z, a.theta, a.zdot, a.thetadot], a.F, p).map((v) => [v]) }],
          }, 'def f(state, F):\n    z = state[0][0]\n    theta = state[1][0]\n    zdot = state[2][0]\n    thetadot = state[3][0]\n    zddot = ...\n    thetaddot = ...\n    return np.array([[zdot], [thetadot],\n                     [zddot], [thetaddot]])\n'), {
            actions: [{ label: 'Simulate my f', run: (code) => simulateMine(ctx, code) }],
          }),
          solution: () => [
            { code: 'def f(state, F):\n    theta = state[1][0]\n    zdot = state[2][0]\n    thetadot = state[3][0]\n    c, s = np.cos(theta), np.sin(theta)\n    Mq = np.array([\n        [P.m1 + P.m2, P.m1 * P.ell / 2 * c],\n        [P.m1 * P.ell / 2 * c, P.m1 * P.ell**2 / 3]])\n    rhs = np.array([\n        P.m1 * P.ell / 2 * thetadot**2 * s\n        + F - P.b * zdot,\n        P.m1 * P.g * P.ell / 2 * s])\n    zddot, thetaddot = np.linalg.solve(Mq, rhs)\n    return np.array([[zdot], [thetadot],\n                     [zddot], [thetaddot]])' },
            { html: 'Book: Listing 3.3 (p. 49–50).' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 4 --
  const EQ_CASES = [0, 1, -1, 2].map((k) => ({ label: `with θₑ = ${k === 0 ? '0' : k === 1 ? 'π' : k === -1 ? '−π' : '2π'}`, fix: { k, th_e: k * Math.PI } }));

  // Operating point θₑ [°]. The hanging equilibrium answers B.4(a), so Work mode
  // stays upright (the study's premise, θ = 0) until (a) is solved.
  const thetaE = (ctx) => (showsAnswer(ctx, 'B.4/a') ? ctx.st.thetaE : 0);
  // Initial state of the B.4 runs: z(0) from the left panel, θ = θₑ + δθ(0).
  const ch4X0 = (ctx) => [ctx.S.sim.y0, (thetaE(ctx) + ctx.st.dth0) * DEG, 0, 0];

  // B.4(a): the pendulum released at rest at the student's θₑ(k) (k for the operating
  // point on the right) and z(0), with the student's F_e held on the cart.
  const EQ_PY = '\n\ndef _wb_eq(k, z_e):\n    th = theta_e(k)\n    return [th, F_e(z_e, th)]\n';
  async function runEquilibrium(ctx, code) {
    const res = ctx.app.result(), p = ctx.pTrue, k = Math.round(thetaE(ctx) / 180), z0 = ctx.S.sim.y0;
    const out = await WB.py.evaluate(code + EQ_PY, [{ params: ctx.pModel, calls: [{ name: '_wb_eq', args: [k, z0] }] }]);
    if (out.error) return pyError(out);
    const [th, Fe] = out.rows[0].calls[0];
    if (!isNum(th)) return { ok: false, msg: 'theta_e should return one number.' };
    if (!isNum(Fe)) return { ok: false, msg: 'F_e should return one number.' };
    const F = M.saturate(Fe, ctx.sys.uLimit(p)), Ts = ctx.S.sim.Ts;
    let x = [z0, th, 0, 0];
    const theta = [];
    for (let i = 0; i < res.t.length; i++) {
      theta.push(x[1]);
      x = M.rk4Step((xx, u) => ctx.sys.f(xx, u, p), x, F, Ts);
    }
    return { n: theta.length, theta, th, Fe, k };
  }

  // B.4(b): the student's accel_lin about the operating point, simulated from the same
  // δθ(0) and F̃ as the Jacobian-linearized model; A, B for the eigenvalues are read off
  // accel_lin at unit deviations (exact when it is linear).
  async function runLinear(ctx, code) {
    const thE = thetaE(ctx) * DEG, x0 = ch4X0(ctx);
    const sim = await WB.py.simulate(code + accelF('accel_lin', `accel_lin(state.copy(), F, ${thE})`), { fn: '_wb_f', params: ctx.pModel, x0: [0, x0[1] - thE, 0, 0], u: finArr(ctx), Ts: ctx.S.sim.Ts });
    if (sim.error) return pyError(sim);
    const unit = [-1, 0, 1, 2, 3].map((j) => ({ name: 'accel_lin', args: [{ col: [0, 1, 2, 3].map((i) => (i === j ? 1 : 0)) }, 0, thE] }));
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: [...unit, { name: 'accel_lin', args: [{ col: [0, 0, 0, 0] }, 1, thE] }] }]);
    if (out.error) return pyError(out);
    const a = out.rows[0].calls.map((c) => [c].flat(Infinity));
    if (!a.every((c) => isVec(c, 2))) return { ok: false, msg: 'accel_lin should return two numbers, (z̃̈, θ̃̈).' };
    const d = (c) => [c[0] - a[0][0], c[1] - a[0][1]];
    const cols = [1, 2, 3, 4].map((j) => d(a[j])), b = d(a[5]);
    const A = [[0, 0, 1, 0], [0, 0, 0, 1], cols.map((c) => c[0]), cols.map((c) => c[1])];
    return { n: sim.x.length, x: sim.x, A, B: [[0], [0], [b[0]], [b[1]]], z0: x0[0], thE };
  }

  S.chapters.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68',
    defaults() { return { thetaE: 0, dth0: 2, inp: { shape: 'zero', amp: 0.2, freq: 0.5, width: 0.2 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    linearLabel: 'Jacobian-linearized model',

    x0: ch4X0,
    simulate(ctx, c, plant) {
      return WB.sim.simulate({ ...c, x0: this.x0(ctx), plant, controller: this.controller(ctx) });
    },
    linearSim(ctx, c) { return linearRun(ctx, c, { thetaE: thetaE(ctx) * DEG, x0: this.x0(ctx) }); },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Operating point', 'p. 65');
      const op = segmented(sec, {
        label: 'Operating point θ<sub>e</sub>',
        options: [{ value: 0, label: 'upright (0°)' }, { value: 180, label: 'hanging (180°)' }],
        ...bind(ctx, 'thetaE'),
      });
      const up = el('p', { class: 'muted small', text: 'Operating point: upright, θ = 0 (the pendulum this study balances). Once (a) is solved you can also pick the other equilibria.' });
      sec.append(up);
      WB.ui.addRefresher(() => { const all = showsAnswer(ctx, 'B.4/a'); op.row.hidden = !all; up.hidden = all; });
      slider(sec, { label: 'δθ(0)', unit: '°', min: -30, max: 30, step: 0.5, sig: 3, hint: 'initial offset from θₑ (z(0) is in the left panel)', ...bind(ctx, 'dth0') });
      const inp = section(parent, 'Input F̃(t) = F − F_e', 'p. 66');
      inputControls(inp, ctx, { title: 'Input force F̃(t)', ampMax: 2 });
      const note = el('p', { class: 'muted small' });
      inp.append(note);
      WB.ui.addRefresher(() => { note.textContent = `The dashed trace is the linearized model with the same input. Upright, a small tilt grows: the two agree for a fraction of a second, then part.${showsAnswer(ctx, 'B.4/a') ? ' Hanging, both oscillate and agree for small angles.' : ''}`; });
    },

    outputSeries(ctx, res, sc, oi) {
      const out = oi === 1 ? [{ label: 'θₑ', y: Array.from(res.t, () => thetaE(ctx)), color: '--ref', dash: [6, 4], width: 1.5 }] : [];
      const e = Y.data(ctx, 'ch4.a'), y = Y.data(ctx, 'ch4.b');
      if (e && oi === 1) out.push(Y.series('your equilibrium', sc(e.theta), { lim: 1e3 }));
      if (y) out.push(Y.series('your accel_lin', oi === 0 ? y.x.map((x) => y.z0 + x[0]) : sc(y.x.map((x) => y.thE + x[1])), { lim: LIN_LIM[oi] }));
      return out;
    },

    splane(ctx) {
      const sp = { markers: [] };
      if (showsAnswer(ctx, 'B.4/b')) {
        const { A } = ctx.sys.linearize(ctx.pModel, thetaE(ctx) * DEG);
        sp.markers = L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `eigenvalue of A ${i + 1}` }));
      }
      const y = Y.data(ctx, 'ch4.b');
      return y ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your accel_lin model'), { obs: 'your accel_lin' }) : sp;
    },

    math(ctx) {
      const p = ctx.pModel, lin = ctx.sys.linearize(p, thetaE(ctx) * DEG);
      return [
        { title: 'Equilibria', page: 'p. 60',
          theory: '\\dot x = f(x, u):\\quad (x_e, u_e) \\text{ is an equilibrium when } f(x_e, u_e) = 0',
          note: 'For a mechanical system: all velocities and accelerations are zero.' },
        { title: 'Jacobian linearization', page: 'p. 60 · Eq. 4.1',
          theory: '\\tilde x = x - x_e,\\; \\tilde u = u - u_e:\\quad \\dot{\\tilde x} \\approx \\frac{\\partial f}{\\partial x}\\Big|_{(x_e, u_e)}\\tilde x + \\frac{\\partial f}{\\partial u}\\Big|_{(x_e, u_e)}\\tilde u' },
        { title: 'First-order Taylor series', page: 'p. 59–60',
          theory: 'g(y) \\approx g(y_e) + \\frac{\\partial g}{\\partial y}\\Big|_{y_e}(y - y_e) \\quad(\\text{apply it to each nonlinear term})' },
        { title: 'Equations of motion (B.3)', page: 'p. 65 · Eq. 4.9', answers: 'B.3/d',
          theory: '(m_1 + m_2)\\ddot z + m_1\\frac{\\ell}{2}\\ddot\\theta\\cos\\theta = m_1\\frac{\\ell}{2}\\dot\\theta^2\\sin\\theta - b\\dot z + F,\\quad m_1\\frac{\\ell}{2}\\ddot z\\cos\\theta + m_1\\frac{\\ell^2}{3}\\ddot\\theta = m_1 g\\frac{\\ell}{2}\\sin\\theta' },
        { title: 'Equilibria of the pendulum', page: 'p. 65 · Eq. 4.10–4.11', answers: 'B.4/a',
          theory: 'F_e = 0,\\quad m_1 g\\frac{\\ell}{2}\\sin\\theta_e = 0 \\Rightarrow \\theta_e = k\\pi,\\; z_e \\text{ arbitrary}' },
        { title: 'Small-angle terms about θₑ = 0', page: 'p. 66', answers: 'B.4/b',
          theory: '\\ddot\\theta\\cos\\theta \\approx \\ddot{\\tilde\\theta},\\quad \\dot\\theta^2\\sin\\theta \\approx 0,\\quad \\ddot z\\cos\\theta \\approx \\ddot{\\tilde z},\\quad \\sin\\theta \\approx \\tilde\\theta' },
        { title: 'Linearized equations of the pendulum', page: 'p. 66 · Eq. 4.12', answers: 'B.4/b',
          theory: '\\begin{pmatrix} m_1 + m_2 & m_1\\frac{\\ell}{2}\\cos\\theta_e\\\\ m_1\\frac{\\ell}{2}\\cos\\theta_e & m_1\\frac{\\ell^2}{3}\\end{pmatrix}\\begin{pmatrix}\\ddot{\\tilde z}\\\\ \\ddot{\\tilde\\theta}\\end{pmatrix} = \\begin{pmatrix} -b\\dot{\\tilde z} + \\tilde F\\\\ m_1 g\\frac{\\ell}{2}\\cos\\theta_e\\,\\tilde\\theta\\end{pmatrix}',
          numbers: `A(\\theta_e = ${thetaE(ctx)}^\\circ) = ${texMat(lin.A)},\\quad \\text{eig}(A) = ${L.eig(lin.A).map((q) => texPole(q)).join(',\\;')}`,
          note: 'Eq. 4.12 is the k even case (cos θₑ = 1). For odd k, cos θₑ = −1 flips the sign of the coupling and gravity terms: the eigenvalues move from ±4.2 to ±4.2j (with ℓ = 1 m).' },
      ];
    },

    buildProblem(parent, ctx) {
      const Fe = (p, a) => {
        const x = [a.z_e, a.th_e, 0, 0], a0 = ctx.sys.f(x, 0, p)[2], a1 = ctx.sys.f(x, 1, p)[2] - a0;
        return -a0 / a1;   // the F that zeros z̈ (F enters linearly)
      };
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibria',
          html: 'Which (z<sub>e</sub>, θ<sub>e</sub>, F<sub>e</sub>) are equilibria? Return the equilibrium angles as θ<sub>e</sub>(k) for integer k (θ<sub>e</sub>(0) = 0, then counting up in the +θ direction), and the force that holds the system at rest there. The check calls F_e at equilibrium angles and random z<sub>e</sub>.',
          code: Object.assign(pyPart(ctx, {
            cases: [-1, 0, 1, 2, 3].map((k) => ({ label: `with k = ${k}`, fix: { k, th_e: k * Math.PI } })),
            perCase: 2,
            items: [
              { fn: 'theta_e', args: ['k'], truth: (p, a) => a.k * Math.PI },
              { fn: 'F_e', args: ['z_e', 'th_e'], truth: (p, a) => Fe(p, a) },
            ],
          }, 'def theta_e(k):\n    # k-th equilibrium angle (k integer)\n    return ...\n\ndef F_e(z_e, theta_e):\n    # force that holds the equilibrium\n    return ...\n'), {
            actions: [{ label: 'Plot my equilibrium', run: (code) => Y.plot(ctx, 'ch4.a', code, runEquilibrium, (d) => `Your θₑ(${d.k}) = ${fmt(d.th / DEG, 4)}° and F_e = ${fmt(d.Fe, 4)} N (k from the operating point on the right). Dotted on the θ plot: the pendulum released at rest there, with your F_e held on the cart.`) }],
          }),
          solution: () => [
            { tex: '\\dot z = \\ddot z = \\dot\\theta = \\ddot\\theta = 0 \\text{ in Eq. 4.9}:\\quad F_e = 0,\\quad m_1 g\\tfrac{\\ell}{2}\\sin\\theta_e = 0 \\Rightarrow \\theta_e = k\\pi,\\quad z_e \\text{ arbitrary}' },
            { code: 'def theta_e(k):\n    return k * np.pi\n\ndef F_e(z_e, theta_e):\n    return 0.0' },
            { html: 'Book: Eqs. 4.10–4.11 (p. 65). Even k is upright, odd k hanging.' },
          ],
        },
        {
          id: 'b', title: '(b) Jacobian linearization about the equilibria',
          html: 'With x̃ = (z̃, θ̃, ż̃, θ̇̃)ᵀ and F̃ = F − F<sub>e</sub>, return the accelerations (z̃̈, θ̃̈) of the model linearized about the equilibrium at θ<sub>e</sub>. The check calls it at θ<sub>e</sub> = 0, π, −π and 2π.',
          code: Object.assign(pyPart(ctx, {
            cases: EQ_CASES,
            items: [{ fn: 'accel_lin', args: ['x_t', 'F_t', 'th_e'], truth: (p, a) => {
              const { A, B } = ctx.sys.linearize(p, a.th_e), x = [a.zt, a.tht, a.zdt, a.thdt];
              return [2, 3].map((i) => A[i].reduce((s, v, j) => s + v * x[j], 0) + B[i][0] * a.F_t);
            } }],
          }, 'def accel_lin(x_t, F_t, theta_e):\n    # x_t: 4x1 column of deviations\n    zt, tht, zdt, thdt = x_t.flatten()\n    zddot_t = ...\n    thetaddot_t = ...\n    return np.array([zddot_t, thetaddot_t])\n'), {
            actions: [{ label: 'Plot my linear model', run: (code) => Y.plot(ctx, 'ch4.b', code, runLinear, () => 'Your linear model about the operating point, from the same δθ(0) and F̃, is dotted on the z and θ plots; its eigenvalues are the × markers on the s-plane. They follow θₑ and the input.') }],
          }),
          solution: () => [
            { tex: '\\ddot\\theta\\cos\\theta \\approx \\cos\\theta_e\\,\\ddot{\\tilde\\theta},\\quad \\dot\\theta^2\\sin\\theta \\approx 0,\\quad \\ddot z\\cos\\theta \\approx \\cos\\theta_e\\,\\ddot{\\tilde z},\\quad \\sin\\theta \\approx \\cos\\theta_e\\,\\tilde\\theta \\quad(\\sin\\theta_e = 0)' },
            { tex: '\\begin{pmatrix} m_1 + m_2 & m_1\\frac{\\ell}{2}\\cos\\theta_e\\\\ m_1\\frac{\\ell}{2}\\cos\\theta_e & m_1\\frac{\\ell^2}{3}\\end{pmatrix}\\begin{pmatrix}\\ddot{\\tilde z}\\\\ \\ddot{\\tilde\\theta}\\end{pmatrix} = \\begin{pmatrix} -b\\dot{\\tilde z} + \\tilde F\\\\ m_1 g\\frac{\\ell}{2}\\cos\\theta_e\\,\\tilde\\theta\\end{pmatrix}' },
            { code: 'def accel_lin(x_t, F_t, theta_e):\n    zt, tht, zdt, thdt = x_t.flatten()\n    c = np.cos(theta_e)\n    Mq = np.array([\n        [P.m1 + P.m2, P.m1 * P.ell / 2 * c],\n        [P.m1 * P.ell / 2 * c, P.m1 * P.ell**2 / 3]])\n    rhs = np.array([\n        -P.b * zdt + F_t,\n        P.m1 * P.g * P.ell / 2 * c * tht])\n    return np.linalg.solve(Mq, rhs)' },
            { html: 'Book: Eq. 4.12 (p. 66), written there for even k (cos θₑ = 1). The note on p. 65: the nonlinear terms are not all in the channel of F, so the pendulum cannot be feedback linearized.' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------- Chapters 5 and 6 --
  function tfMarkers(ctx) {
    const mk = [];
    if (showsAnswer(ctx, 'B.5/b')) {
      const pin = ctx.sys.inner(ctx.pModel);
      M.roots2(0, pin.a0).forEach((p, i) => mk.push({ ...p, kind: 'ol', label: `pole of Θ̃/F̃ (b = 0) ${i + 1}` }));
    }
    if (showsAnswer(ctx, 'B.5/c')) {
      const q = Math.sqrt(3 * ctx.pModel.g / (2 * ctx.pModel.ell));
      mk.push({ re: 0, im: 0, kind: 'ol', label: 'double pole of Z̃/Θ̃ at 0' });
      mk.push({ re: q, im: 0, kind: 'olzero', label: 'zero of Z̃/Θ̃ (RHP)' }, { re: -q, im: 0, kind: 'olzero', label: 'zero of Z̃/Θ̃ (LHP)' });
    }
    return { markers: mk };
  }
  // Initial state of the B.5/B.6 runs (left panel); the linear models start from its
  // deviation about (z(0), θₑ = 0).
  const olX0 = (ctx) => [ctx.S.sim.y0, (ctx.S.sim.init.theta0 || 0) * DEG, 0, 0];
  const olSim = (lin) => ({
    simulate(ctx, c, plant) { return WB.sim.simulate({ ...c, plant, controller: this.controller(ctx) }); },
    linearSim(ctx, c) {
      const p = lin === 'b0' && ctx.st.b0 ? { ...ctx.pModel, b: 0 } : ctx.pModel;
      return linearRun(ctx, c, { x0: olX0(ctx), p });
    },
  });

  // B.5: the student's transfer functions, fitted with the lowest-order proper rational
  // functions that match them (Y.fitTf), and driven from rest by the same F̃.
  // (a) Z̃/F̃ and Θ̃/F̃ from the student's two transformed equations (with b).
  const EQS_PY = '\n\ndef _wb_eqs(s, i):\n    _M = np.array([np.asarray(eq1(s), dtype=complex).flatten(), np.asarray(eq2(s), dtype=complex).flatten()])\n    if _M.shape != (2, 2):\n        raise ValueError("eq1 and eq2 should each return two coefficients")\n    return np.linalg.solve(_M, np.array([1, 0], dtype=complex))[i]\n';
  async function runEqs(ctx, code) {
    const G = [];
    for (const i of [0, 1]) {
      const r = await fitMine(ctx, code + EQS_PY, '_wb_eqs', i ? 'Θ̃/F̃ from eq1, eq2' : 'Z̃/F̃ from eq1, eq2', { extraArgs: [i] });
      if (r.ok === false) return r;
      G.push(r.G);
    }
    return { G };
  }
  // (b) the pair that matches the dashed model: Z_F0, Th_F0 with b = 0, else Z_F, Th_F.
  async function runTfs(ctx, code) {
    const b0 = !!ctx.st.b0, names = b0 ? ['Z_F0', 'Th_F0'] : ['Z_F', 'Th_F'], G = [];
    for (const fn of names) {
      const r = await Y.fitTf(ctx, code, fn);
      if (r.ok === false) return r;
      G.push(r.G);
    }
    return { G, b0, names };
  }
  // (c) Z̃/Θ̃ in cascade after the workbench's Θ̃/F̃ (b = 0), so it is driven by F̃ too.
  async function runCascade(ctx, code) {
    const r = await Y.fitTf(ctx, code, 'Z_Th');
    if (r.ok === false) return r;
    const pin = ctx.sys.inner(ctx.pModel);
    return { G: r.G, Gz: WB.tf.mul(r.G, WB.tf.tf([pin.b0], [1, 0, pin.a0])) };
  }
  const pz = (G, name) => [...Y.poles(WB.tf.poles(G), `pole of your ${name}`), ...Y.zeros(WB.tf.zeros(G), `zero of your ${name}`)];

  const pulseSection = (parent, ctx, title, page) => {
    const inp = section(parent, title, page);
    inputControls(inp, ctx, { title: 'Input force F̃(t)', ampMax: 2 });
    inp.append(el('p', { class: 'muted small', text: 'Open loop the upright pendulum is unstable, so even a small push tips it over within a second or two. The dashed trace is the linear model: it matches while θ is small, then the nonlinear pendulum swings down and the linear one runs away off the plot.' }));
  };
  const eq412 = { title: 'Linearized equations (B.4)', page: 'p. 66 · Eq. 4.12', answers: 'B.4/b',
    theory: '\\begin{pmatrix} m_1 + m_2 & m_1\\frac{\\ell}{2}\\\\ m_1\\frac{\\ell}{2} & m_1\\frac{\\ell^2}{3}\\end{pmatrix}\\begin{pmatrix}\\ddot{\\tilde z}\\\\ \\ddot{\\tilde\\theta}\\end{pmatrix} = \\begin{pmatrix} -b\\dot{\\tilde z} + \\tilde F\\\\ m_1 g\\frac{\\ell}{2}\\tilde\\theta\\end{pmatrix}' };

  S.chapters.ch5 = Object.assign({}, common, olSim('b0'), {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 69–80',
    defaults() { return { b0: true, inp: { shape: 'pulse', amp: 0.2, freq: 0.5, width: 0.1 } }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearLabel: 'transfer-function model',
    buildControls(parent, ctx) {
      pulseSection(parent, ctx, 'Input F̃(t)', 'p. 75');
      const md = section(parent, 'Linear model', 'p. 76');
      segmented(md, { label: 'Damping in the dashed model', options: [{ value: true, label: 'b = 0 (B.5b)' }, { value: false, label: 'with b' }], ...bind(ctx, 'b0') });
    },
    // Each overlay is drawn only next to the dashed model it matches: (a) keeps b,
    // (b) is the pair for the chosen damping, (c) is built on the b = 0 equations.
    yours(ctx) {
      const b0 = !!ctx.st.b0, a = Y.data(ctx, 'ch5.a'), b = Y.data(ctx, 'ch5.b'), c = Y.data(ctx, 'ch5.c');
      return { a: !b0 && a, b: b && b.b0 === b0 && b, c: b0 && c };
    },
    splane(ctx) {
      const { a, b, c } = this.yours(ctx);
      const mk = [];
      if (a) mk.push(...pz(a.G[0], 'Z̃/F̃ (eq1, eq2)'));
      if (b) b.G.forEach((G, i) => mk.push(...pz(G, b.names[i])));
      if (c) mk.push(...pz(c.G, 'Z_Th'));
      return Y.withMarkers(tfMarkers(ctx), mk, { obs: 'your poles', zero: 'your zeros' });
    },
    // Your transfer functions from rest, driven by the same F̃, shifted to z(0).
    outputSeries(ctx, res, sc, oi) {
      const out = [], off = oi === 0 ? olX0(ctx)[0] : 0;
      const resp = (label, G) => Y.series(label, sc(Y.tfResponse(ctx, G, fin(ctx)).map((v) => v + off)), { lim: LIN_LIM[oi] });
      const { a, b, c } = this.yours(ctx);
      if (a) out.push(resp(`your eq1, eq2 (${oi ? 'Θ̃' : 'Z̃'}/F̃)`, a.G[oi]));
      if (b) out.push(resp(`your ${b.names[oi]}`, b.G[oi]));
      if (c && oi === 0) out.push(resp('your Z_Th after Θ̃/F̃', c.Gz));
      return out;
    },
    math(ctx) {
      const p = ctx.pModel, pin = ctx.sys.inner(p);
      const q = Math.sqrt(3 * p.g / (2 * p.ell));
      return [
        { title: 'Laplace transform', page: 'p. 69–70',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)' },
        { title: 'Transfer function', page: 'p. 70–71',
          theory: '\\text{zero initial conditions:}\\quad P(s) = \\frac{Y(s)}{U(s)}' },
        { title: 'Two equations, two outputs', page: 'p. 76 · App. P.7',
          theory: '\\begin{pmatrix} a & b\\\\ c & d\\end{pmatrix}^{-1} = \\frac{1}{ad - bc}\\begin{pmatrix} d & -b\\\\ -c & a\\end{pmatrix}' },
        { title: 'Cascade', page: 'p. 76',
          theory: '\\frac{Y_2}{Y_1} = \\frac{Y_2/U}{Y_1/U}' },
        eq412,
        { title: 'Laplace transform of Eq. 4.12', page: 'p. 75–76', answers: 'B.5/a',
          theory: '\\begin{pmatrix}(m_1+m_2)s^2 + bs & m_1\\frac{\\ell}{2}s^2\\\\ s^2 & \\frac{2\\ell}{3}s^2 - g\\end{pmatrix}\\begin{pmatrix}\\tilde Z(s)\\\\ \\tilde\\Theta(s)\\end{pmatrix} = \\begin{pmatrix}\\tilde F(s)\\\\ 0\\end{pmatrix}' },
        { title: 'Transfer functions of the pendulum', page: 'p. 76', answers: 'B.5/b',
          theory: '\\tilde Z = \\frac{\\frac{2\\ell}{3}s^2 - g}{(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^4 + b\\frac{2\\ell}{3}s^3 - (m_1+m_2)gs^2 - bgs}\\tilde F,\\quad \\tilde\\Theta = \\frac{-s^2}{(\\cdots)}\\tilde F' },
        { title: 'With b = 0', page: 'p. 76', answers: 'B.5/b',
          theory: '\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{-1}{(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^2 - (m_1+m_2)g},\\quad \\frac{\\tilde Z}{\\tilde F} = \\frac{\\frac{2\\ell}{3}s^2 - g}{s^2\\big[(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^2 - (m_1+m_2)g\\big]}',
          numbers: `\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{${tex(pin.b0)}}{s^2 ${tex(pin.a0).startsWith('-') ? '' : '+'}${tex(pin.a0)}},\\quad \\text{poles } \\pm${tex(Math.sqrt(-pin.a0))}`,
          note: 'b ż is small next to the other forces, and ignoring it is conservative: all the damping must then come from the controller (p. 76).' },
        { title: 'Cascade of the pendulum', page: 'p. 76 · Fig. 5-2', answers: 'B.5/c',
          theory: '\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{\\tilde Z/\\tilde F}{\\tilde\\Theta/\\tilde F} = \\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2}',
          numbers: `\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{${tex(-2 * p.ell / 3)}\\,s^2 + ${tex(p.g)}}{s^2},\\quad \\text{zeros } \\pm${tex(q)}`,
          note: 'A push in +z tips the rod toward −θ (the minus sign); the rod\'s tilt then drives the cart like a double integrator.' },
      ];
    },
    buildProblem(parent, ctx) {
      const cx = WB.py.cx;
      const eom = (p) => ctx.sys.linearEOM(p, 0);
      const sq = (s) => cx.mul(s, s);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'a', title: '(a) Laplace transform of the linearized equations',
          html: 'Start from your B.4 linearized equations (zero initial conditions). Return the coefficients of Z̃(s) and Θ̃(s) in each transformed equation: the first with F̃(s) on the right, the second with 0 (any constant multiple of the second is accepted).',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'eq1', args: ['s'], truth: (p, a) => { const e = eom(p); return [cx.add(cx.mul(e.M[0][0], sq(a.s)), cx.mul(e.b, a.s)), cx.mul(e.M[0][1], sq(a.s))]; } },
              { fn: 'eq2', args: ['s'], compare: 'scale', truth: (p, a) => { const e = eom(p); return [cx.mul(e.M[1][0], sq(a.s)), cx.add(cx.mul(e.M[1][1], sq(a.s)), -e.kTh)]; } },
            ],
          }, 'def eq1(s):\n    # [a, b]: a Z(s) + b Theta(s) = F(s)\n    return np.array([..., ...])\n\ndef eq2(s):\n    # [c, d]: c Z(s) + d Theta(s) = 0\n    return np.array([..., ...])\n'), {
            actions: [{ label: 'Plot my equations', run: (code) => { ctx.st.b0 = false; return Y.plot(ctx, 'ch5.a', code, runEqs, 'Your two equations solved for Z̃ and Θ̃ give transfer functions from F̃: their responses from rest are dotted on the z and θ plots (next to the dashed model with damping b), and their poles and zeros are on the s-plane.'); } }],
          }),
          solution: () => [
            { tex: '\\big[(m_1+m_2)s^2 + bs\\big]\\tilde Z(s) + m_1\\frac{\\ell}{2}s^2\\tilde\\Theta(s) = \\tilde F(s),\\quad s^2\\tilde Z(s) + \\Big(\\frac{2\\ell}{3}s^2 - g\\Big)\\tilde\\Theta(s) = 0' },
            { code: 'def eq1(s):\n    return np.array([(P.m1 + P.m2) * s**2 + P.b * s,\n                     P.m1 * P.ell / 2 * s**2])\n\ndef eq2(s):\n    return np.array([s**2, 2 * P.ell / 3 * s**2 - P.g])' },
            { html: 'Book: p. 75–76. The second equation was divided by m₁ℓ/2 first.' },
          ],
        },
        {
          id: 'b', title: '(b) Transfer functions from F̃ to Z̃ and Θ̃',
          html: 'Return Z̃/F̃ and Θ̃/F̃ at a complex s, first with the damping b, then simplified with b = 0 (those two must not use P.b). How does b = 0 simplify them, and is it a reasonable assumption?',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'Z_F', args: ['s'], truth: (p, a) => tfs(ctx, p, a.s)[0] },
              { fn: 'Th_F', args: ['s'], truth: (p, a) => tfs(ctx, p, a.s)[1] },
              { fn: 'Z_F0', args: ['s'], label: 'Z_F0 (b = 0)', truth: (p, a) => tfs(ctx, p, a.s, true)[0] },
              { fn: 'Th_F0', args: ['s'], label: 'Th_F0 (b = 0)', truth: (p, a) => tfs(ctx, p, a.s, true)[1] },
            ],
            // A b = 0 function that still equals the damped transfer function kept P.b.
            explain: (it, f) => {
              if (!/0$/.test(it.fn)) return '';
              const damped = tfs(ctx, f.s.p, f.e.a.s)[it.fn === 'Z_F0' ? 0 : 1], g = f.e.got;
              const got = typeof g === 'object' ? g : { re: g, im: 0 };
              return Math.hypot(got.re - damped.re, got.im - damped.im) <= 1e-6 * Math.hypot(damped.re, damped.im) ? `That is the transfer function with damping: ${it.fn} is the b = 0 simplification, without P.b.` : '';
            },
          }, 'def Z_F(s):\n    # Z(s)/F(s)\n    return ...\n\ndef Th_F(s):\n    # Theta(s)/F(s)\n    return ...\n\ndef Z_F0(s):\n    # Z(s)/F(s) with b = 0\n    return ...\n\ndef Th_F0(s):\n    # Theta(s)/F(s) with b = 0\n    return ...\n'), {
            actions: [{ label: 'Plot my transfer functions', run: (code) => Y.plot(ctx, 'ch5.b', code, runTfs, (d) => `Your ${d.names.join(' and ')} (the pair for the damping chosen on the right) driven from rest by the same F̃ are dotted on the z and θ plots; their poles and zeros are on the s-plane. Switch the damping to plot the other pair.`) }],
          }),
          solution: () => [
            { tex: '\\tilde Z = \\frac{\\frac{2\\ell}{3}s^2 - g}{(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^4 + b\\frac{2\\ell}{3}s^3 - (m_1+m_2)gs^2 - bgs}\\tilde F,\\quad \\tilde\\Theta = \\frac{-s^2}{(\\cdots)}\\tilde F' },
            { tex: 'b = 0:\\quad \\frac{\\tilde Z}{\\tilde F} = \\frac{\\frac{2\\ell}{3}s^2 - g}{s^2\\big[(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^2 - (m_1+m_2)g\\big]},\\quad \\frac{\\tilde\\Theta}{\\tilde F} = \\frac{-1}{(m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3})s^2 - (m_1+m_2)g}' },
            { code: 'J = P.m1 * P.ell / 6 + P.m2 * 2 * P.ell / 3\n\ndef den(s, b):\n    return (J * s**4 + b * 2 * P.ell / 3 * s**3\n            - (P.m1 + P.m2) * P.g * s**2\n            - b * P.g * s)\n\ndef Z_F(s):\n    return (2 * P.ell / 3 * s**2 - P.g) / den(s, P.b)\n\ndef Th_F(s):\n    return -s**2 / den(s, P.b)\n\ndef Z_F0(s):\n    return (2 * P.ell / 3 * s**2 - P.g) / den(s, 0)\n\ndef Th_F0(s):\n    return -s**2 / den(s, 0)' },
            { html: 'Book: p. 76. With b = 0 the s³ and s terms vanish and s² cancels in Θ̃/F̃, leaving a second-order system. Reasonable: bż is small next to the other forces. Conservative: the design must then supply all the damping itself.' },
          ],
        },
        {
          id: 'c', title: '(c) Z̃(s)/Θ̃(s) and the block diagram',
          html: 'From the simplified (b = 0) transfer functions, return Z̃/Θ̃ at a complex s. Then draw the block diagram as a cascade F̃ → Θ̃ → Z̃ on paper.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'Z_Th', args: ['s'], truth: (p, a) => { const g = tfs(ctx, p, a.s, true); return cx.div(g[0], g[1]); } }],
          }, 'def Z_Th(s):\n    # Z(s)/Theta(s)\n    return ...\n'), {
            actions: [{ label: 'Plot my Z_Th', run: (code) => { ctx.st.b0 = true; return Y.plot(ctx, 'ch5.c', code, runCascade, 'Dotted on the z plot: your Z_Th in cascade after the b = 0 angle dynamics Θ̃/F̃, driven from rest by the same F̃. Its poles and zeros are on the s-plane.'); } }],
          }),
          solution: () => [
            { tex: '\\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{\\tilde Z/\\tilde F}{\\tilde\\Theta/\\tilde F} = \\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2}' },
            { code: 'def Z_Th(s):\n    return (-2 * P.ell / 3 * s**2 + P.g) / s**2' },
            { html: 'Block diagram (Fig. 5-2, p. 77): F̃ → [−1/((m₁ℓ/6 + 2m₂ℓ/3)s² − (m₁+m₂)g)] → Θ̃ → [−((2ℓ/3)s² − g)/s²] → Z̃: a fast angle subsystem driving a slow position subsystem.' },
          ],
        },
        {
          id: 'd', title: '(d) How the cascade makes sense physically',
          html: 'How does force influence the pendulum angle? How does the angle influence the cart position? Try it: apply a small positive pulse on F and watch the signs of θ and z, and how fast θ grows. Then open the solution.',
          solution: () => [{ html: 'A positive force on the cart makes the pendulum fall the other way (the minus sign), in an unstable motion (the right-half-plane pole of Θ̃/F̃). If the pendulum falls in +θ, the cart shoots off in −z: the minus sign and the right-half-plane zero of Z̃/Θ̃, with the double integrator s² below it (p. 76–77). Hanging down, the signs of the gravity terms flip and these equations change.' }],
        },
      ]);
    },
  });

  // B.6: the student's A, B, C, D.
  async function runSS(ctx, code) {
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, vars: ['A', 'B', 'C', 'D'] }]);
    if (out.error) return pyError(out);
    const v = out.rows[0].vars;
    const A = Y.asMat(v.A, 4, 4), B = Y.asMat(v.B, 4, 1), C = Y.asMat(v.C, 2, 4), D = Y.asMat(v.D, 2, 1);
    const bad = [[A, 'A', '4×4'], [B, 'B', '4×1'], [C, 'C', '2×4'], [D, 'D', '2×1']].find(([m]) => !m);
    if (bad) return { ok: false, msg: `${bad[1]} should be a ${bad[2]} array of numbers (x̃ = (z̃, θ̃, ż̃, θ̇̃), ũ = F̃, ỹ = (z̃, θ̃)).` };
    return { A, B, C, D };
  }

  S.chapters.ch6 = Object.assign({}, common, olSim('full'), {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 81–93',
    defaults() { return { extra: 'thd', inp: { shape: 'pulse', amp: 0.2, freq: 0.5, width: 0.1 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearLabel: 'ẋ = Ax + Bu model',
    buildControls(parent, ctx) {
      pulseSection(parent, ctx, 'Input ũ = F̃(t)', 'p. 89');
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'zd', label: 'x₃ = ż' }, { value: 'thd', label: 'x₄ = θ̇' }], ...bind(ctx, 'extra') });
    },
    splane(ctx) {
      const sp = { markers: [] };
      if (showsAnswer(ctx, 'B.6/a')) {
        const { A } = ctx.sys.stateSpace(ctx.pModel);
        sp.markers = L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `eigenvalue of A ${i + 1}` }));
      }
      const y = Y.data(ctx, 'ch6.a');
      return y ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your A'), { obs: 'eigenvalue of your A' }) : sp;
    },
    // Your model from the same initial deviation and F̃; ỹ is shifted back by (z(0), θₑ = 0).
    outputSeries(ctx, res, sc, oi) {
      const y = Y.data(ctx, 'ch6.a');
      if (!y) return [];
      const x0 = olX0(ctx), r = Y.linResponse(ctx, y, [0, x0[1], 0, 0], fin(ctx));
      return [Y.series('your A, B, C, D', sc(r.y[oi].map((v) => v + (oi === 0 ? x0[0] : 0))), { lim: LIN_LIM[oi] })];
    },
    extraPlot(ctx, res) { return stateExtra(res, ctx.st.extra); },
    math(ctx) {
      const { A, B } = ctx.sys.stateSpace(ctx.pModel);
      return [
        { title: 'Jacobian linearization revisited', page: 'p. 83–84',
          theory: 'A = \\frac{\\partial f}{\\partial x}\\Big|_e,\\quad B = \\frac{\\partial f}{\\partial u}\\Big|_e,\\quad C = \\frac{\\partial h}{\\partial x}\\Big|_e,\\quad D = \\frac{\\partial h}{\\partial u}\\Big|_e' },
        { title: 'From second-order equations', page: 'p. 83',
          theory: 'M\\ddot{\\tilde q} = \\dots \\;\\Rightarrow\\; \\ddot{\\tilde q} = M^{-1}(\\dots),\\quad \\tilde x = (\\tilde q, \\dot{\\tilde q})' },
        eq412,
        { title: 'Accelerations of the linearized pendulum', page: 'p. 89', answers: 'B.6/a',
          theory: '\\begin{pmatrix}\\ddot{\\tilde z}\\\\ \\ddot{\\tilde\\theta}\\end{pmatrix} = \\begin{pmatrix} -\\frac{b}{\\frac14 m_1 + m_2}\\dot{\\tilde z} + \\frac{1}{\\frac14 m_1 + m_2}\\tilde F - \\frac{\\frac34 m_1 g}{\\frac14 m_1 + m_2}\\tilde\\theta\\\\ \\frac{3b}{2(\\frac14 m_1 + m_2)\\ell}\\dot{\\tilde z} - \\frac{3}{2(\\frac14 m_1 + m_2)\\ell}\\tilde F + \\frac{3(m_1+m_2)g}{2(\\frac14 m_1 + m_2)\\ell}\\tilde\\theta\\end{pmatrix}' },
        { title: 'State-space model of the pendulum', page: 'p. 90 · Eq. 6.17', answers: 'B.6/a',
          theory: '\\dot{\\tilde x} = A\\tilde x + B\\tilde u,\\quad \\tilde y = \\begin{pmatrix}1&0&0&0\\\\0&1&0&0\\end{pmatrix}\\tilde x,\\quad D = 0',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`,
          note: 'The B.11–B.12 solutions print row 4 of A and B for a different ℓ than the stated one (ISSUES.md).' },
        { title: 'Back to transfer functions', page: 'p. 85 · Eq. 6.14',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\det(sI - A) = 0 \\text{ gives the poles}',
          numbers: `\\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const ss = (p) => ctx.sys.stateSpace(p);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'A, B, C, D of the linear state-space model',
        html: 'Module-level numpy arrays, with x̃ = (z̃, θ̃, ż̃, θ̇̃)ᵀ, ũ = F̃ and ỹ = (z̃, θ̃)ᵀ.',
        code: Object.assign(pyPart(ctx, {
          items: ['A', 'B', 'C', 'D'].map((k) => ({ var: k, truth: (p) => ss(p)[k] })),
        }, '# x = (z, theta, zdot, thetadot)\n# u = F, y = (z, theta)\nA = ...\nB = ...\nC = ...\nD = ...\n'), {
          actions: [{ label: 'Plot my model', run: (code) => Y.plot(ctx, 'ch6.a', code, runSS, 'Your eigenvalues of A are the × markers on the s-plane; your model\'s outputs, from the same initial state and F̃, are dotted on the z and θ plots.') }],
        }),
        solution: () => { const { A, B } = ss(ctx.pModel); return [
          { tex: 'A = \\begin{pmatrix}0&0&1&0\\\\0&0&0&1\\\\0&-\\frac{\\frac34 m_1 g}{\\frac14 m_1 + m_2}&-\\frac{b}{\\frac14 m_1 + m_2}&0\\\\0&\\frac{3(m_1+m_2)g}{2(\\frac14 m_1 + m_2)\\ell}&\\frac{3b}{2(\\frac14 m_1 + m_2)\\ell}&0\\end{pmatrix},\\quad B = \\begin{pmatrix}0\\\\0\\\\ \\frac{1}{\\frac14 m_1 + m_2}\\\\ \\frac{-3}{2(\\frac14 m_1 + m_2)\\ell}\\end{pmatrix}' },
          { tex: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = \\begin{pmatrix}1&0&0&0\\\\0&1&0&0\\end{pmatrix},\\quad D = \\begin{pmatrix}0\\\\0\\end{pmatrix}` },
          { code: 'd = 0.25 * P.m1 + P.m2\nA = np.array([\n    [0, 0, 1, 0],\n    [0, 0, 0, 1],\n    [0, -0.75 * P.m1 * P.g / d,\n     -P.b / d, 0],\n    [0, 1.5 * (P.m1 + P.m2) * P.g / (d * P.ell),\n     1.5 * P.b / (d * P.ell), 0]])\nB = np.array([[0], [0], [1 / d],\n              [-1.5 / (d * P.ell)]])\nC = np.array([[1, 0, 0, 0],\n              [0, 1, 0, 0]])\nD = np.array([[0], [0]])' },
          { html: 'Book: Eq. 6.17 (p. 90): solve Eq. 4.12 for the accelerations, or take the Jacobians of the nonlinear f (p. 90–91). Rows 1–2 just say ż̃ = x₃ and θ̇̃ = x₄.' },
        ]; },
      }]);
    },
  });

  S.models = { inputForce, inputControls, linearRun, tfAt, ARGS };
})();
