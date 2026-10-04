// Study E, Chapters 2–6 (E.2–E.6): kinetic energy, Euler-Lagrange equations,
// equilibria and linearization, transfer functions, and the state-space model.
// Open-loop experiments on the block and beam. The open-loop system is unstable
// (eig(A) = ±λ, ±jλ), so these runs are short.
//
// Work mode: every derivation (K, P, generalized forces, EOM, f(x, u), equilibria,
// Jacobian, feedback linearization, Laplace-domain equations, transfer functions,
// A, B, C, D) is a Python answer checked at random arguments and parameters
// (WB.py.check). Cards that show this study's results carry `answers`.
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.E = WB.studies.E || { chapters: {} };

(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const PD = () => WB.pd;
  const CH = WB.studies.E.chapters;

  // ---------------------------------------------------- open-loop force input --
  const inputForce = (st, t) => WB.models.inputTorque(st, t);   // same shapes as Study A

  // F = F_comp + F_in(t), with F_comp = 0, F_e (at z_e) or F_fl(z).
  // Fe overrides the workbench's F_e (E.4 in Work mode applies the student's own).
  // In Work mode F_e and F_fl answer E.4(a) and E.4(c), so neither reaches the
  // force plot before that part is solved: F_e is then the student's "your F_e"
  // (st.FeW, 0 without one) and F_fl adds nothing (WB.E.ffOf).
  function openLoop(ctx, { linear = false, Fe } = {}) {
    const { sys, pModel } = ctx;
    const st = ctx.st;
    const z0 = st.zE !== undefined ? st.zE : sys.ze(pModel);
    const FeV = Fe !== undefined ? Fe : WB.E.shows(ctx, E4A(ctx)) ? sys.Fe(pModel, z0) : st.FeW || 0;
    const ffl = WB.E.ffOf(ctx, 'fl');
    return {
      update(r, x, yMeas, t) {
        const fin = inputForce(st, t);
        if (linear || st.comp === 'none') return { u: fin, Fin: fin };
        const ff = st.comp === 'eq' ? FeV : ffl(x[0]);
        return { u: ff + fin, Fin: fin };
      },
    };
  }
  const E4A = (ctx) => `${ctx.sys.problems.ch4.id}/a`;
  const E4C = (ctx) => `${ctx.sys.problems.ch4.id}/c`;

  function inputControls(parent, ctx, { comps } = {}) {
    const names = { zero: 'none', const: 'constant', pulse: 'pulse', square: 'square', sine: 'sine' };
    segmented(parent, {
      label: 'Input force F<sub>in</sub>(t)',
      options: ['zero', 'const', 'pulse', 'square', 'sine'].map((s) => ({ value: s, label: names[s] })),
      ...bind(ctx, 'shape', () => ctx.st.inp),
    });
    slider(parent, { label: 'amplitude', unit: 'N', min: -3, max: 3, step: 0.01, sig: 3, ...bind(ctx, 'amp', () => ctx.st.inp), disabled: () => ctx.st.inp.shape === 'zero' });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.05, max: 5, step: 0.01, sig: 3, ...bind(ctx, 'freq', () => ctx.st.inp), disabled: () => !['square', 'sine'].includes(ctx.st.inp.shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.02, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'width', () => ctx.st.inp), disabled: () => ctx.st.inp.shape !== 'pulse' });
    if (comps) {
      segmented(parent, {
        label: 'Added to F<sub>in</sub>',
        options: comps,
        ...bind(ctx, 'comp'),
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

  // ------------------------------------------------- Python answer parts --
  // Arguments for the block-and-beam Python answers (WB.py.check draws them at random).
  const cplx = (label) => ({ label, complex: true, re: [-6, 3], im: [0.3, 12] });
  const ARGS = {
    z: { label: 'z', lo: -0.2, hi: 0.7 },
    theta: { label: 'θ', lo: -1.2, hi: 1.2 },
    zdot: { label: 'ż', lo: -1, hi: 1 },
    thetadot: { label: 'θ̇', lo: -2, hi: 2 },
    F: { label: 'F', lo: -15, hi: 15 },
    state: { col: ['z', 'theta', 'zdot', 'thetadot'] },
    z_e: { label: 'zₑ', lo: 0.02, hi: 0.5 },
    s: cplx('s'), Z: cplx('Z'), Theta: cplx('Θ'), Fs: cplx('F'),
  };
  // P.length is an alias of P.ell, as in blockbeamParam.py.
  const withLength = (p) => ({ ...p, length: p.ell });
  // code part: {template, check}; spec as in WB.py.check, with ARGS filled in.
  const pyPart = (ctx, spec, template) => ({ template, check: (code) => WB.py.check(ctx, { args: ARGS, params: withLength, ...spec }, code) });
  const xOf = (a) => [a.z, a.theta, a.zdot, a.thetadot];
  const accel = (ctx, p, a, F = a.F) => { const d = ctx.sys.f(xOf(a), F, p); return [d[2], d[3]]; };
  const pyError = (out) => ({ ok: false, msg: out.timeout ? out.error : 'Python raised an error.', detail: [out.error, out.where, (out.stdout || '').trim()].filter(Boolean).join('\n') });
  // In Work mode, poles/eigenvalues that answer `key` stay off the s-plane until it is solved.
  const showsAnswer = (ctx, key) => ctx.S.mode === 'explore' || ctx.app.isSolved(key);
  const LEN_NOTE = ' <code>P.length</code> is the same as <code>P.ell</code>, as in <code>blockbeamParam.py</code>.';

  // "Plot my answer" overlays (js/core/yours.js).
  const Y = WB.yours;
  const fin = (ctx) => (t) => inputForce(ctx.st, t);
  // A "your" trace that joins the autoscale only while it stays within a few times
  // the range of the reference it is drawn against, so a model that blows up
  // doesn't flatten the plot.
  const ser = (label, y, ref, more) => Y.series(label, y, { lim: 4 * Math.max(1e-6, ...Array.from(ref, Math.abs)), ...more });
  const OUT2 = { C: [[1, 0, 0, 0], [0, 1, 0, 0]], D: [[0], [0]] };   // y = (z, θ)
  // The student's model x̃ = x − x_e on the z and θ plots: z_e added back, θ in plot units.
  const outTrace = (lr, oi, sc, zE) => (oi === 0 ? Array.from(lr.y[0], (v) => v + zE) : sc(lr.y[1]));
  // One number from a Python call, or NaN.
  const num = (v) => (typeof v === 'number' ? v : NaN);

  // ------------------------------------------------------------- Chapter 2 --
  // E.2(a): the student's kinetic(z, θ, ż, θ̇) along the prescribed motion.
  async function runKinetic(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: withLength(ctx.pModel), calls: res.x.map((x) => ({ name: 'kinetic', args: x })) }]);
    if (out.error) return Y.pyError(out);
    const K = out.rows[0].calls;
    if (!K.every((v) => typeof v === 'number')) return { ok: false, msg: 'kinetic should return one number.' };
    return { n: K.length, K };
  }

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
      slider(sec, { label: 'z amplitude', unit: 'm', min: 0, max: 0.25, step: 0.005, sig: 3, ...bind(ctx, 'Az') });
      slider(sec, { label: 'θ amplitude', unit: '°', min: 0, max: 45, step: 0.5, sig: 3, ...bind(ctx, 'Ath') });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'f') });
      sec.append(el('p', { class: 'muted small', text: 'z(t) = z_e + A_z sin(2πft), θ(t) = A_θ sin(πft). No dynamics: the motion is imposed, as in hw02. In Work mode the plot below is empty until you click Plot my K (your K) or solve E.2(a) (K along the motion and its parts).' }));
    },

    // K(t) and its split into parts answer E.2(a), so in Work mode the plot is empty
    // until (a) is solved, then K; "Plot my K" adds the student's own K.
    extraPlot(ctx, res) {
      const mJ = (a) => Array.from(a, (v) => v * 1000);
      const show = showsAnswer(ctx, `${ctx.sys.problems.ch2.id}/a`);
      const y = Y.data(ctx, 'ch2.a'), K = mJ(res.extras.K);
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [mJ]', unit: 'mJ' },
        data: { series: [
          ...(show ? [
            { label: 'block, along the beam', y: mJ(res.extras.Kr), color: '--series-2', width: 1.5 },
            { label: 'block, carried by the beam', y: mJ(res.extras.Kt), color: '--series-3', width: 1.5 },
            { label: 'beam', y: mJ(res.extras.Kb), color: '--text-muted', width: 1.5 },
            { label: 'total K', y: K, color: '--series-1' },
          ] : []),
          ...(y ? [ser('your K (Python)', mJ(y.K), show ? K : mJ(y.K))] : []),
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel, id = ctx.sys.problems.ch2.id;
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Thin rod inertia, parallel axis', page: 'p. 23 · Eq. 2.2',
          theory: 'J_{cm} = \\frac{m\\ell^2}{12}\\;\\text{(thin rod about its center)},\\quad J_{O} = J_{cm} + m d^2\\;\\text{(about a parallel axis a distance } d \\text{ away)}' },
        { title: 'Block and beam kinematics', page: 'p. 385 · Fig. 20-1', answers: `${id}/a`,
          theory: '\\mathbf p_1 = z\\begin{bmatrix}\\cos\\theta\\\\ \\sin\\theta\\\\ 0\\end{bmatrix},\\quad \\mathbf v_1 = \\dot z\\begin{bmatrix}\\cos\\theta\\\\ \\sin\\theta\\\\ 0\\end{bmatrix} + z\\dot\\theta\\begin{bmatrix}-\\sin\\theta\\\\ \\cos\\theta\\\\ 0\\end{bmatrix},\\quad J_{beam} = \\frac{m_2\\ell^2}{12} + m_2\\left(\\frac{\\ell}{2}\\right)^2 = \\frac{m_2\\ell^2}{3}' },
        { title: 'Kinetic energy of the block and beam', page: 'p. 386 · E.2(a)', answers: `${id}/a`,
          theory: 'K = \\tfrac12 m_1\\big(\\dot z^2 + z^2\\dot\\theta^2\\big) + \\tfrac12\\frac{m_2\\ell^2}{3}\\dot\\theta^2',
          numbers: `K = ${tex(p.m1 / 2)}\\,\\dot z^2 + ${tex(p.m1 / 2)}\\,z^2\\dot\\theta^2 + ${tex(p.m2 * p.ell ** 2 / 6)}\\,\\dot\\theta^2` },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) Kinetic energy',
          html: 'Write K as a function of z, θ, ż and θ̇. The block is a point mass near the surface of the beam.' + LEN_NOTE,
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'kinetic', args: ['z', 'theta', 'zdot', 'thetadot'], truth: (p, a) => ctx.sys.kinetic(xOf(a), p) }],
          }, 'def kinetic(z, theta, zdot, thetadot):\n    # kinetic energy of the system\n    K = ...\n    return K\n'), {
            actions: [{ label: 'Plot my K', run: (code) => Y.plot(ctx, 'ch2.a', code, runKinetic, () => 'Your K(t) along the motion set on the right is the dotted trace on the energy plot.') }],
          }),
          solution: () => [
            { html: 'The block at distance z along the beam moves with ż along the beam and zθ̇ across it. The beam turns about its end, so J = m₂ℓ²/12 + m₂(ℓ/2)² = m₂ℓ²/3 (parallel-axis theorem).' },
            { tex: 'K = \\tfrac12 m_1\\big(\\dot z^2 + z^2\\dot\\theta^2\\big) + \\tfrac12\\frac{m_2\\ell^2}{3}\\dot\\theta^2' },
            { code: 'def kinetic(z, theta, zdot, thetadot):\n    Kb = 0.5 * P.m1 * (zdot**2\n                      + z**2 * thetadot**2)\n    J = P.m2 * P.ell**2 / 3\n    return Kb + 0.5 * J * thetadot**2' },
            { html: 'No book solution for E: derived from Fig. 20-1 (p. 385) and Eq. 2.3 (p. 25).' },
          ],
        },
        {
          id: 'b', title: '(b) Animate the block on beam',
          html: 'You write this class in your own <code>blockbeamAnimation.py</code> with z and θ as inputs, and turn in a screen capture. The animation at the top of this page shows the prescribed motion set in the controls on the right.',
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
    const out = await WB.py.simulate(code, { fn: 'f', params: withLength(ctx.pTrue), x0: res.x[0], u: Array.from(res.uApplied), Ts: ctx.S.sim.Ts });
    if (out.error) return pyError(out);
    mine.ch3 = { sig: simSig(ctx, res), z: out.x.map((x) => x[0]), theta: out.x.map((x) => x[1]) };
    let dz = 0, dth = 0;
    out.x.forEach((x, k) => { dz = Math.max(dz, Math.abs(x[0] - res.x[k][0])); dth = Math.max(dth, Math.abs(x[1] - res.x[k][1])); });
    ctx.update();
    const ok = dz < 1e-4 && dth / M.DEG < 0.01;
    const msg = `Your f is the dashed trace on the z and θ plots. Largest difference from the workbench's block and beam: ${fmt(1000 * dz, 3)} mm in z, ${fmt(dth / M.DEG, 3)}° in θ.`;
    return { ok, msg: ok ? msg : `${msg} They should overlap.` };
  }

  // E.3(a): the student's potential(z, θ) along the simulated motion.
  async function runPotential(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: withLength(ctx.pModel), calls: res.x.map((x) => ({ name: 'potential', args: [x[0], x[1]] })) }]);
    if (out.error) return Y.pyError(out);
    const Pv = out.rows[0].calls;
    if (!Pv.every((v) => typeof v === 'number')) return { ok: false, msg: 'potential should return one number.' };
    return { n: Pv.length, P: Pv };
  }

  // E.3(c): the work done by the student's τ and −Bq̇ along the simulated motion,
  // with the same trapezoid rule (and force samples) as the workbench's "work done by F".
  async function runForces(ctx, code) {
    const res = ctx.app.result(), n = res.t.length, Ts = ctx.S.sim.Ts, u = res.uApplied;
    const at = (x, F) => [{ name: 'generalized_force', args: [...x, F] }, { name: 'damping_force', args: [...x, F] }];
    const calls = [...res.x.flatMap((x, k) => at(x, u[k])), ...res.x.slice(1).flatMap((x, k) => at(x, u[k]))];
    const out = await WB.py.evaluate(code, [{ params: withLength(ctx.pModel), calls }]);
    if (out.error) return Y.pyError(out);
    const c = out.rows[0].calls.map((v) => [v].flat(Infinity));
    const bad = c.findIndex((v) => v.length !== 2 || !v.every((q) => typeof q === 'number'));
    if (bad >= 0) return { ok: false, msg: `${bad % 2 ? 'damping_force' : 'generalized_force'} should return two numbers, one along z and one along θ.` };
    // power (τ − Bq̇)·q̇ at sample i of call block `off`, with state x
    const pw = (off, i, x) => (c[off + 2 * i][0] + c[off + 2 * i + 1][0]) * x[2] + (c[off + 2 * i][1] + c[off + 2 * i + 1][1]) * x[3];
    const W = new Float64Array(n);
    let w = 0;
    for (let k = 1; k < n; k++) {
      w += 0.5 * Ts * (pw(0, k - 1, res.x[k - 1]) + pw(2 * n, k - 1, res.x[k]));
      W[k] = w * 1000;
    }
    return { n, W };
  }

  // E.3(d): the student's accelerations, simulated like "Simulate my f".
  const ACCEL_F_PY = '\n\ndef _wb_eom(state, F):\n    a = np.asarray(accelerations(*state[:, 0].tolist(), F), dtype=float).ravel()\n    if a.size != 2:\n        raise ValueError(f"accelerations returned {a.size} values; expected two (zddot, thetaddot)")\n    return np.array([[state[2][0]], [state[3][0]], [a[0]], [a[1]]])\n';
  async function runAccel(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.simulate(code + ACCEL_F_PY, { fn: '_wb_eom', params: withLength(ctx.pTrue), x0: res.x[0], u: Array.from(res.uApplied), Ts: ctx.S.sim.Ts });
    if (out.error) return Y.pyError(out);
    return { n: out.x.length, z: out.x.map((x) => x[0]), theta: out.x.map((x) => x[1]) };
  }

  CH.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 41–56',
    linear: false,
    defaults() { return { comp: 'fl', inp: { shape: 'square', amp: 0.2, freq: 1, width: 0.5 } }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 386 · E.3(e)');
      inputControls(sec, ctx, { comps: COMPS });
      WB.E.ffWorkControls(sec, ctx);
      sec.append(el('p', { class: 'muted small', text: 'The block and beam has no stable equilibrium: any tilt makes the block slide and the beam tip further. The energy plot checks the EOM: E(t) − E(0) must equal the work done by F. In Work mode it is empty until you plot your own P or forces or solve (a) or (c): E(t) − E(0) appears with (a), the work done by F with (c), and your own curves whenever you plot them.' }));
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
      // In Work mode E(t) − E(0) uses the answers to E.2(a) and E.3(a), and the work
      // curve the forces of E.3(c): each waits for its part (P alone for the energy, so
      // the chapter stands on its own). Until then the plot is empty; the
      // student's own "Plot my P" / "Plot my forces" curves show whenever plotted.
      const id = ctx.sys.problems.ch3.id;
      const showE = showsAnswer(ctx, `${id}/a`), showW = showsAnswer(ctx, `${id}/c`);
      const yP = Y.data(ctx, 'ch3.a'), yF = Y.data(ctx, 'ch3.c');
      const series = [
        ...(showW ? [{ label: 'work done by F', y: W, color: '--series-2', dash: [5, 4], width: 2 }] : []),
        ...(showE ? [{ label: 'E(t) − E(0) = ΔK + ΔP', y: En, color: '--series-1' }] : []),
      ];
      // E.3(a): the energy with the student's P (the workbench's K); E.3(c): the work of their forces.
      if (yP) {
        const K0 = sys.kinetic(res.x[0], pModel);
        const mineE = res.x.map((x, k) => (sys.kinetic(x, pModel) - K0 + yP.P[k] - yP.P[0]) * 1000);
        series.push(ser('ΔK + ΔP with your P', mineE, showE ? En : mineE));
      }
      if (yF) series.push(ser('work done by your τ − Bq̇', yF.W, showW ? W : yF.W, { dash: [1, 3] }));
      return { opts: { title: 'energy balance', yLabel: 'energy [mJ]', unit: 'mJ' }, data: { series } };
    },

    // E.3(e): the student's f, simulated with the same input as the system above.
    // E.3(d): the student's accelerations from the same initial state and input.
    outputSeries(ctx, res, sc, oi) {
      const out = [], m = mine.ch3, y = Y.data(ctx, 'ch3.d');
      if (m && m.sig === simSig(ctx, res)) out.push({ label: 'your f (Python)', y: sc(oi === 0 ? m.z : m.theta), color: '--series-2', dash: [5, 4], width: 2 });
      if (y) out.push(ser('your equations of motion', sc(oi === 0 ? y.z : y.theta), sc(res.yAll[oi])));
      return out;
    },

    math(ctx) {
      const p = ctx.pModel, id = ctx.sys.problems.ch3.id;
      const J0 = p.m2 * p.ell ** 2 / 3;
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 43 · §3.1.4',
          theory: 'L(q, \\dot q) = K(q, \\dot q) - P(q),\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy', page: 'p. 41–42 · §3.1.1',
          theory: '\\text{gravity: } P = \\textstyle\\sum_i m_i g\\,h_i + P_0\\quad(h_i = \\text{height of each center of mass})' },
        { title: 'Generalized coordinates, forces, damping', page: 'p. 42–43 · §3.1.2–3.1.3',
          theory: 'q = \\text{minimum set of configuration variables},\\quad \\tau_i = \\text{virtual work of the external forces per unit } \\delta q_i,\\quad -B\\dot q = \\text{damping forces}' },
        { title: 'Potential energy of the block and beam', page: 'p. 386 · E.3(a)', answers: `${id}/a`,
          theory: 'P = P_0 + m_1 g z\\sin\\theta + m_2 g\\tfrac{\\ell}{2}\\sin\\theta',
          numbers: `P - P_0 = (${tex(p.m1 * p.g)}\\,z + ${tex(p.m2 * p.g * p.ell / 2)})\\sin\\theta` },
        { title: 'Generalized coordinates and forces of the block and beam', page: 'p. 386 · E.3(b, c)', answers: [`${id}/b`, `${id}/c`],
          theory: 'q = (z, \\theta)^\\top,\\quad \\tau = \\begin{bmatrix}0\\\\ F\\ell\\cos\\theta\\end{bmatrix},\\quad B = 0' },
        { title: 'Equations of motion of the block and beam', page: 'p. 386 · E.3(d)', answers: `${id}/d`,
          theory: 'm_1\\ddot z - m_1 z\\dot\\theta^2 + m_1 g\\sin\\theta = 0,\\quad \\left(\\frac{m_2\\ell^2}{3} + m_1 z^2\\right)\\ddot\\theta + 2m_1 z\\dot z\\dot\\theta + \\left(m_1 g z + m_2 g\\frac{\\ell}{2}\\right)\\cos\\theta = F\\ell\\cos\\theta',
          numbers: `\\ddot z = z\\dot\\theta^2 - ${tex(p.g)}\\sin\\theta,\\quad (${tex(J0)} + ${tex(p.m1)}\\,z^2)\\,\\ddot\\theta = ${tex(p.ell)}F\\cos\\theta - ${tex(2 * p.m1)}\\,z\\dot z\\dot\\theta - (${tex(p.m1 * p.g)}\\,z + ${tex(p.m2 * p.g * p.ell / 2)})\\cos\\theta` },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from p. 43',
          theory: '\\frac{d}{dt}(K + P) = \\tau^\\top\\dot q - \\dot q^\\top B\\dot q',
          note: 'The plot below integrates the right side and compares it with the energy of the simulated system.' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch3;
      const ARG4 = ['z', 'theta', 'zdot', 'thetadot'];
      const norm = (v) => String(v || '').toLowerCase().replace(/\s+/g, '').replace(/^q=/, '').replace(/[()[\]{}]|\^t|ᵀ|'/g, '').replace(/θ/g, 'theta');
      const EOM = 'zddot = z * thetadot**2 - P.g * np.sin(theta)\n    J = P.m2 * P.ell**2 / 3 + P.m1 * z**2\n    thetaddot = (F * P.ell * np.cos(theta)\n                 - 2 * P.m1 * z * zdot * thetadot\n                 - P.m1 * P.g * z * np.cos(theta)\n                 - P.m2 * P.g * P.ell / 2\n                 * np.cos(theta)) / J';
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Potential energy',
          html: 'Write P as a function of z and θ. Any constant P<sub>0</sub> is accepted.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'potential', args: ['z', 'theta'], compare: 'offset', truth: (p, a) => ctx.sys.potential([a.z, a.theta, 0, 0], p) }],
          }, 'def potential(z, theta):\n    # potential energy of the system\n    return ...\n'), {
            actions: [{ label: 'Plot my P', run: (code) => Y.plot(ctx, 'ch3.a', code, runPotential, () => 'Dotted on the energy plot: the change in K + P along the simulated motion with your P (and the workbench\'s K). With F doing the only work, it should follow the work done by F.') }],
          }),
          solution: () => [
            { tex: 'P = P_0 + m_1 g z\\sin\\theta + m_2 g\\tfrac{\\ell}{2}\\sin\\theta \\quad(\\text{heights } z\\sin\\theta \\text{ and } \\tfrac{\\ell}{2}\\sin\\theta)' },
            { code: 'def potential(z, theta):\n    return (P.m1 * P.g * z\n            + P.m2 * P.g * P.ell / 2) * np.sin(theta)' },
          ],
        },
        {
          id: 'b', title: '(b) Generalized coordinates',
          inputs: { q: 'q =' },
          html: 'Name them, comma separated (e.g. <code>x, phi</code> or <code>x, φ</code>).',
          check: (v) => {
            const g = norm(v.q);
            if (!g) return { ok: false, msg: 'Enter the generalized coordinates.' };
            if (/dot|̇|omega|ω/.test(g)) return { ok: false, msg: 'Generalized coordinates are configuration variables (positions and angles), not velocities.' };
            const set = g.split(',').filter(Boolean).sort().join(',');
            if (set === 'theta,z') return { ok: true, msg: g.startsWith('z') ? 'Two coordinates: q is a 2-vector.' : 'Right. The book orders them (z, θ), and so does the state x = (z, θ, ż, θ̇).' };
            return { ok: false, msg: 'Which variables fix the configuration of the block and beam? Use the minimum number.' };
          },
          solution: () => [{ tex: 'q = (z, \\theta)^\\top' }, { html: 'Book: p. 42 (§3.1.2).' }],
        },
        {
          id: 'c', title: '(c) Generalized forces and damping forces',
          html: 'Return each as the 2-vector along q = (z, θ). Write the damping term the way the book does, as the force −Bq̇ (p. 43).',
          code: pyPart(ctx, {
            items: [
              { fn: 'generalized_force', args: [...ARG4, 'F'], truth: (p, a) => [0, a.F * p.ell * Math.cos(a.theta)] },
              { fn: 'damping_force', args: [...ARG4, 'F'], truth: () => [0, 0] },
            ],
            explain: (it, f) => {
              if (it.fn === 'damping_force') return 'Look at Fig. 20-1 and the problem text (p. 385): what friction or damping does the model include?';
              const g = [].concat(f.e.got).map(Number), w = [].concat(f.e.want).map(Number);
              if (g.length === 2 && Math.abs(g[1] + w[1]) < 1e-6 * Math.max(1, Math.abs(w[1]))) return 'Check the sign: F points up and θ is measured up from level.';
              return '';
            },
          }, 'def generalized_force(z, theta, zdot,\n                      thetadot, F):\n    # applied forces along (z, theta)\n    return np.array([..., ...])\n\ndef damping_force(z, theta, zdot,\n                  thetadot, F):\n    # damping forces, -B*q_dot\n    return np.array([..., ...])\n'),
          actions: [{ label: 'Plot my forces', run: (code) => Y.plot(ctx, 'ch3.c', code, runForces, () => 'Dotted on the energy plot: the work your generalized and damping forces do along the simulated motion, ∫(τ − Bq̇)·q̇ dt. It should follow the change in energy.') }],
          solution: () => [
            { tex: '\\tau = \\begin{bmatrix}0\\\\ F\\ell\\cos\\theta\\end{bmatrix},\\quad -B\\dot q = \\begin{bmatrix}0\\\\0\\end{bmatrix}' },
            { html: 'F acts at the tip, at height ℓ sin θ, so its virtual work is F δ(ℓ sin θ) = Fℓ cos θ δθ. Nothing does work along z. The problem models no friction, so B = 0.' },
            { code: 'def generalized_force(z, theta, zdot,\n                      thetadot, F):\n    return np.array([0,\n                     F * P.ell * np.cos(theta)])\n\ndef damping_force(z, theta, zdot,\n                  thetadot, F):\n    return np.array([0, 0])' },
          ],
        },
        {
          id: 'd', title: '(d) Equations of motion',
          html: 'Apply the Euler-Lagrange equations, then solve for both accelerations.',
          code: pyPart(ctx, {
            cases: [
              { label: 'with ż = θ̇ = 0 and F = 0 (only gravity acts)', fix: { zdot: 0, thetadot: 0, F: 0 } },
              { label: 'with ż = θ̇ = 0 (gravity and F act)', fix: { zdot: 0, thetadot: 0 } },
              { label: 'with ż = 0, θ = 0 and F = 0 (the beam is turning)', fix: { zdot: 0, theta: 0, F: 0 } },
              { label: 'with θ = 0 and F = 0 (block sliding on a turning beam)', fix: { theta: 0, F: 0 } },
              { label: '' },
            ],
            items: [{ fn: 'accelerations', args: [...ARG4, 'F'], label: '(z̈, θ̈)', truth: (p, a) => accel(ctx, p, a) }],
          }, 'def accelerations(z, theta, zdot,\n                  thetadot, F):\n    # from the equations of motion\n    zddot = ...\n    thetaddot = ...\n    return zddot, thetaddot\n'),
          actions: [{ label: 'Simulate my EOM', run: (code) => Y.plot(ctx, 'ch3.d', code, runAccel, () => 'Your equations of motion, run with RK4 from the same initial state and force (true-plant parameters), are dotted on the z and θ plots.') }],
          solution: () => [
            { tex: 'L = \\tfrac12 m_1(\\dot z^2 + z^2\\dot\\theta^2) + \\tfrac{m_2\\ell^2}{6}\\dot\\theta^2 - P_0 - (m_1 g z + m_2 g\\tfrac{\\ell}{2})\\sin\\theta' },
            { tex: 'z:\\; m_1\\ddot z - m_1 z\\dot\\theta^2 + m_1 g\\sin\\theta = 0' },
            { tex: '\\theta:\\; \\tfrac{d}{dt}\\big[(m_1 z^2 + \\tfrac{m_2\\ell^2}{3})\\dot\\theta\\big] + (m_1 g z + m_2 g\\tfrac{\\ell}{2})\\cos\\theta = F\\ell\\cos\\theta' },
            { code: `def accelerations(z, theta, zdot,\n                  thetadot, F):\n    ${EOM}\n    return zddot, thetaddot` },
            { html: 'Same as the f(x, u) that passes <code>_E_blockbeam/python/testDynamics.py</code> (with g = 9.81 there; see ISSUES).' },
          ],
        },
        {
          id: 'e', title: '(e) Implement and simulate',
          html: 'Write f(x, u) as in <code>blockbeamDynamics.py</code>: the state is a 4×1 column (z, θ, ż, θ̇). <em>Check</em> tests it at random states like <code>testDynamics.py</code>. <em>Simulate my f</em> runs it with RK4 on the same force input as the system above (true-plant parameters) and draws z and θ dashed. Connecting it to your E.2 animation is up to you.' + LEN_NOTE,
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'f', args: ['state', 'F'], truth: (p, a) => { const d = ctx.sys.f(xOf(a), a.F, p); return d.map((v) => [v]); } }],
          }, 'def f(state, F):\n    z = state[0][0]\n    theta = state[1][0]\n    zdot = state[2][0]\n    thetadot = state[3][0]\n    zddot = ...\n    thetaddot = ...\n    return np.array([[zdot], [thetadot],\n                     [zddot], [thetaddot]])\n'), {
            actions: [{ label: 'Simulate my f', run: (code) => simulateMine(ctx, code) }],
          }),
          solution: () => [
            { code: `def f(state, F):\n    z = state[0][0]\n    theta = state[1][0]\n    zdot = state[2][0]\n    thetadot = state[3][0]\n    ${EOM}\n    return np.array([[zdot], [thetadot],\n                     [zddot], [thetaddot]])` },
            { html: 'Appendices P.1–P.3 (p. 437 on) describe the class structure.' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- Chapter 4 --
  // E.4(c): the beam's θ̈ with F = F_fl + F̃ must equal the student's thetaddot_fl,
  // and with F̃ = 0 the beam must stay put (θ̈ = 0) at rest for every z and θ.
  async function checkFeedbackLin(ctx, code) {
    if (!code.trim()) return { ok: false, msg: 'Write your code first.' };
    const sets = WB.py.paramSets(ctx, { params: withLength });
    let seed = 99;
    const r = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    const pt = () => [-0.2 + 0.9 * r(), (2 * r() - 1) * 1.2, (2 * r() - 1), (2 * r() - 1) * 2, (2 * r() - 1) * 5];
    const rest = () => [-0.2 + 0.9 * r(), (2 * r() - 1) * 1.2, 0, 0, 0];
    const samples = sets.map(() => {
      const pts = [...Array.from({ length: 6 }, pt), ...Array.from({ length: 4 }, rest)];
      return { pts, calls: [
        ...pts.map((x) => ({ name: 'F_fl', args: x.slice(0, 4) })),
        ...pts.map((x) => ({ name: 'thetaddot_fl', args: x })),
      ] };
    });
    const out = await WB.py.evaluate(code, sets.map((s, i) => ({ params: s.p, calls: samples[i].calls })));
    if (out.error) return pyError(out);
    const close = (a, b) => Math.abs(a - b) <= 1e-6 * Math.max(1, Math.abs(a), Math.abs(b));
    const at = (x) => `z = ${fmt(x[0], 3)}, θ = ${fmt(x[1], 3)}, ż = ${fmt(x[2], 3)}, θ̇ = ${fmt(x[3], 3)}`;
    for (let i = 0; i < sets.length; i++) {
      const c = out.rows[i].calls.map(Number), { pts } = samples[i], n = pts.length;
      for (let k = 0; k < n; k++) {
        const x = pts[k], tt = x[4];
        const want = ctx.sys.f(x.slice(0, 4), c[k] + tt, sets[i].p)[3], got = c[n + k];
        const restPt = k >= 6;
        if (!close(got, want) || (restPt && !close(want, 0))) {
          if (i > 0) return { ok: false, msg: `Matches at the nominal parameters but not with ${sets.text(sets[i])}. Write it with ${sets.vary.map((v) => `P.${v}`).join(', ')} rather than numbers.` };
          if (!close(got, want)) return { ok: false, msg: `At ${at(x)}, F̃ = ${fmt(tt, 3)}: the block and beam with F = F_fl + F̃ has θ̈ = ${fmt(want, 4)}, but thetaddot_fl gives ${fmt(got, 4)}.` };
          return { ok: false, msg: `At rest (ż = θ̇ = 0) with F̃ = 0, z = ${fmt(x[0], 3)}, θ = ${fmt(x[1], 3)}: F = F_fl gives θ̈ = ${fmt(want, 4)}, not 0. F_fl should hold the beam still wherever the block is.` };
        }
      }
    }
    return { ok: true, msg: `Consistent with the block and beam, and F_fl holds the beam at every block position, at ${sets.length} parameter sets.` };
  }

  // E.4(a): the student's x_e(z_e) and F_e(z_e) at the current z_e become the
  // Work-mode start state (plus the offsets) and the "your F_e" force, as D.4(a).
  async function useMyEquilibrium(ctx, code) {
    const zE = ctx.st.zE;
    const out = await WB.py.evaluate(code, [{ params: withLength(ctx.pModel), calls: [{ name: 'x_e', args: [zE] }, { name: 'F_e', args: [zE] }] }]);
    if (out.error) return Y.pyError(out);
    const xe = Y.asMat(out.rows[0].calls[0], 4, 1), Fe = num(out.rows[0].calls[1]);
    if (!xe) return { ok: false, msg: 'x_e should return four numbers (z, θ, ż, θ̇).' };
    if (!Number.isFinite(Fe)) return { ok: false, msg: 'F_e should return one number.' };
    Object.assign(ctx.st, { FeW: Fe, xeW: xe.map((r) => r[0]), method: 'jacobian' });
    ctx.update();
    const head = `Your x_e(${fmt(zE, 3)} m) = (${ctx.st.xeW.map((v) => fmt(v, 4)).join(', ')}), F_e = ${fmt(Fe, 4)} N.`;
    if (ctx.S.mode !== 'work') return { info: true, msg: `${head} Explore mode applies the workbench's; switch to Work mode to apply yours.` };
    return { info: true, msg: `${head} The run on the right now starts at your x_e (plus δz(0), δθ(0)) with F = your F_e + F̃. With δz(0) = δθ(0) = 0, no input and the right answers, the block and beam stay at rest. Click again after moving zₑ.` };
  }

  // E.4(b): the student's A_jac, B_jac at the current z_e.
  async function runJacobian(ctx, code) {
    const zE = ctx.st.zE;
    const out = await WB.py.evaluate(code, [{ params: withLength(ctx.pModel), calls: [{ name: 'A_jac', args: [zE] }, { name: 'B_jac', args: [zE] }] }]);
    if (out.error) return Y.pyError(out);
    const A = Y.asMat(out.rows[0].calls[0], 4, 4), B = Y.asMat(out.rows[0].calls[1], 4, 1);
    if (!A) return { ok: false, msg: 'A_jac should return a 4×4 array of numbers.' };
    if (!B) return { ok: false, msg: 'B_jac should return a 4×1 array of numbers.' };
    return { A, B };
  }

  // E.4(c): the true block and beam with the student's F = F_fl(x) + F̃ (simulated in
  // Python like the workbench's own run: same input, disturbance and saturation), and
  // their thetaddot_fl model driven by F̃. thetaddot_fl gives only θ̈, so the model
  // takes z̈ from the plant's own z equation. The eigenvalues are those of that model
  // linearized at (z_e, 0, 0, 0) (central differences).
  const PLANT_PY = 'def f(state, F):\n    z, theta, zdot, thetadot = state[:, 0].tolist()\n    zddot = (1.0 / P.m1) * (P.m1 * z * thetadot**2 - P.m1 * P.g * np.sin(theta))\n    thetaddot = (1.0 / ((P.m2 * P.ell**2) / 3.0 + P.m1 * z**2)) * (F * P.ell * np.cos(theta) - 2.0 * P.m1 * z * zdot * thetadot - P.m1 * P.g * z * np.cos(theta) - P.m2 * P.g * P.ell / 2.0 * np.cos(theta))\n    return np.array([[zdot], [thetadot], [zddot], [thetaddot]])\n';
  const FL_MODEL_PY = '\n\ndef _wb_fl_model(state, F_tilde):\n    z, theta, zdot, thetadot = state[:, 0].tolist()\n    a = np.asarray(thetaddot_fl(z, theta, zdot, thetadot, F_tilde), dtype=float)\n    if a.size != 1:\n        raise ValueError(f"thetaddot_fl returned {a.size} values; expected one number")\n    return np.array([[zdot], [thetadot], [z * thetadot**2 - P.g * np.sin(theta)], [a.item()]])\n';
  async function runFeedbackLin(ctx, code) {
    const res = ctx.app.result(), Ts = ctx.S.sim.Ts, zE = ctx.st.zE;
    const u = Array.from(res.t, fin(ctx));
    const d = Array.from(res.uApplied, (v, k) => v - res.u[k]);
    const plant = await WB.py.simulate(code, { plant: PLANT_PY, plantParams: withLength(ctx.pTrue), ctrl: 'F_fl', params: withLength(ctx.pModel), x0: res.x[0], u, d, uLimit: ctx.sys.uLimit(ctx.pTrue), Ts });
    if (plant.error) return Y.pyError(plant);
    const model = await WB.py.simulate(code + FL_MODEL_PY, { fn: '_wb_fl_model', params: withLength(ctx.pModel), x0: res.x[0], u, Ts });
    if (model.error) return Y.pyError(model);
    const h = 1e-5, x0 = [zE, 0, 0, 0, 0];
    const pts = [0, 1, 2, 3].flatMap((i) => [1, -1].map((sg) => x0.map((v, j) => (j === i ? v + sg * h : v))));
    const out = await WB.py.evaluate(code, [{ params: withLength(ctx.pModel), calls: pts.map((args) => ({ name: 'thetaddot_fl', args })) }]);
    if (out.error) return Y.pyError(out);
    const c = out.rows[0].calls.map(num);
    const row = [0, 1, 2, 3].map((i) => (c[2 * i] - c[2 * i + 1]) / (2 * h));
    const g = ctx.pModel.g;
    return {
      n: res.t.length,
      z: plant.x.map((x) => x[0]), theta: plant.x.map((x) => x[1]),
      zm: model.x.map((x) => x[0]), thm: model.x.map((x) => x[1]),
      A: row.every(Number.isFinite) ? [[0, 0, 1, 0], [0, 0, 0, 1], [0, -g, 0, 0], row] : null,
    };
  }

  CH.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68',
    defaults() { return { zE: 0.25, dz0: 0, dth0: 0.5, method: 'jacobian', comp: 'eq', FeW: 0, inp: { shape: 'zero', amp: 0.05, freq: 1, width: 0.2 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    linearLabel: 'linearized model',

    lin(ctx) { return ctx.sys.linear(ctx.pModel, { z: ctx.st.zE, comp: ctx.st.method === 'fl' ? 'fl' : 'eq' }); },
    controller(ctx, o) { ctx.st.comp = ctx.st.method === 'fl' ? 'fl' : 'eq'; return openLoop(ctx, o); },
    x0(ctx) { return [ctx.st.zE + ctx.st.dz0, ctx.st.dth0 * M.DEG, 0, 0]; },
    // Work mode, Jacobian: F = your F_e + F̃ (the "your F_e" slider, which "Use my
    // x_e, F_e" sets), starting at your x_e (plus the offsets) once you have used it.
    // Feedback: F_fl(z) is applied once E.4(c) is solved; until then F = F̃.
    simulate(ctx, c, plant) {
      const work = ctx.S.mode === 'work' && ctx.st.method !== 'fl';
      const xe = work ? ctx.st.xeW : null;
      const x0 = xe ? xe.map((v, i) => v + [ctx.st.dz0, ctx.st.dth0 * M.DEG, 0, 0][i]) : this.x0(ctx);
      const controller = this.controller(ctx, work ? { Fe: ctx.st.FeW || 0 } : {});
      return WB.sim.simulate({ ...c, x0, plant, controller, reference: () => ctx.st.zE });
    },
    // The feedback-linearized model answers E.4(c): no dashed trace for it before then.
    linearSim(ctx, c) {
      if (ctx.st.method === 'fl' && !showsAnswer(ctx, E4C(ctx))) return null;
      const { A, B } = this.lin(ctx);
      return linearOpenLoop(ctx, { ...c, reference: () => ctx.st.zE }, A, B, this.x0(ctx), ctx.st.zE);
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Operating point', 'p. 386 · E.4');
      slider(sec, { label: 'z<sub>e</sub>', unit: 'm', min: 0, max: 0.5, step: 0.005, sig: 3, ...bind(ctx, 'zE') });
      slider(sec, { label: 'δz(0)', unit: 'm', min: -0.1, max: 0.1, step: 0.001, sig: 3, ...bind(ctx, 'dz0') });
      slider(sec, { label: 'δθ(0)', unit: '°', min: -5, max: 5, step: 0.05, sig: 3, ...bind(ctx, 'dth0') });
      segmented(sec, {
        label: 'Linearization',
        options: [{ value: 'jacobian', label: 'Jacobian: F = F<sub>e</sub> + F̃' }, { value: 'fl', label: 'feedback: F = F<sub>fl</sub>(z) + F̃' }],
        ...bind(ctx, 'method'),
      });
      const inp = section(parent, 'Input F̃(t)', 'p. 386');
      inputControls(inp, ctx);
      inp.append(el('p', { class: 'muted small', text: 'Start a hair off equilibrium and watch the linear model (dashed) and the nonlinear plant separate as the block slides. Once you have solved (b) and (c), the s-plane shows the eigenvalues of each linearization.' }));
      if (ctx.S.mode === 'work') {
        slider(sec, { label: 'your F<sub>e</sub>', unit: 'N', min: 0, max: 30, step: 0.01, sig: 4, ...bind(ctx, 'FeW'), disabled: () => ctx.st.method !== 'jacobian' });
        sec.append(el('p', { class: 'muted small', text: 'Jacobian: Work mode applies your F_e. Use my x_e, F_e in (a) sets it and starts the run at your x_e (plus the offsets): with the right answers the block and beam stay at rest. Feedback: the feedback-linearizing force is applied once you solve (c); until then F = F̃.' }));
      } else {
        sec.append(el('p', { class: 'muted small', text: 'Explore applies the correct equilibrium or feedback-linearizing force.' }));
      }
    },

    outputSeries(ctx, res, sc, oi) {
      const out = oi === 0 ? [{ label: 'zₑ', y: Array.from(res.t, () => ctx.st.zE), color: '--ref', dash: [6, 4], width: 1.5 }] : [];
      const ref = sc(res.yAll[oi]);
      if (ctx.st.method === 'fl') {
        const y = Y.data(ctx, 'ch4.c');
        if (y) {
          const pick = (a, b) => sc(oi === 0 ? a : b);
          out.push(ser('block and beam with your F_fl', pick(y.z, y.theta), ref), ser('your thetaddot_fl model', pick(y.zm, y.thm), ref, { dash: [8, 3] }));
        }
      } else {
        const y = Y.data(ctx, 'ch4.b');
        if (y) {
          const lr = Y.linResponse(ctx, { ...y, ...OUT2 }, [ctx.st.dz0, ctx.st.dth0 * M.DEG, 0, 0], fin(ctx));
          out.push(ser('your A_jac, B_jac', outTrace(lr, oi, sc, ctx.st.zE), ref));
        }
      }
      return out;
    },

    splane(ctx) {
      const id = ctx.sys.problems.ch4.id, fl = ctx.st.method === 'fl';
      let sp = { markers: [], fitR: 2 };
      if (showsAnswer(ctx, fl ? `${id}/c` : `${id}/b`)) {
        const { A } = this.lin(ctx);
        sp = { markers: L.eig(A).map((p) => ({ ...p, kind: 'ol', label: `eigenvalue of A (${ctx.st.method})` })), fitR: 2 };
      }
      const y = Y.data(ctx, fl ? 'ch4.c' : 'ch4.b');
      if (!y || !y.A) return sp;
      return fl ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your thetaddot_fl model at zₑ'), { obs: 'your F_fl model' })
        : Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your A_jac'), { obs: 'your A_jac' });
    },

    math(ctx) {
      const p = ctx.pModel, zE = ctx.st.zE, id = ctx.sys.problems.ch4.id;
      const jac = ctx.sys.linear(p, { z: zE, comp: 'eq' });
      const lam = Math.pow(p.m1 * p.g * p.g / jac.De, 0.25);
      return [
        { title: 'Equilibria', page: 'p. 60',
          theory: '\\dot x = f(x, u):\\quad (x_e, u_e) \\text{ is an equilibrium when } f(x_e, u_e) = 0' },
        { title: 'Jacobian linearization', page: 'p. 60 · Eq. 4.1',
          theory: '\\tilde x = x - x_e,\\; \\tilde u = u - u_e:\\quad \\dot{\\tilde x} \\approx \\frac{\\partial f}{\\partial x}\\Big|_{(x_e, u_e)}\\tilde x + \\frac{\\partial f}{\\partial u}\\Big|_{(x_e, u_e)}\\tilde u' },
        { title: 'Feedback linearization', page: 'p. 64',
          theory: 'u = u_{fl}(x) + \\tilde u',
          note: 'u_fl cancels nonlinear terms, so the model from ũ holds away from the equilibrium too, not only near it.' },
        { title: 'Equations of motion (E.3)', page: 'p. 386 · E.3(d)', answers: `${ctx.sys.problems.ch3.id}/d`,
          theory: '\\dot x = \\begin{bmatrix}\\dot z\\\\ \\dot\\theta\\\\ z\\dot\\theta^2 - g\\sin\\theta\\\\ \\dfrac{F\\ell\\cos\\theta - 2m_1z\\dot z\\dot\\theta - (m_1gz + m_2g\\frac{\\ell}{2})\\cos\\theta}{\\frac{m_2\\ell^2}{3} + m_1z^2}\\end{bmatrix}' },
        { title: 'Equilibria of the block and beam', page: 'p. 386 · E.4(a)', answers: `${id}/a`,
          theory: '\\theta_e = 0,\\; \\dot z_e = \\dot\\theta_e = 0,\\; z_e \\text{ arbitrary},\\quad F_e = \\frac{m_1 g z_e}{\\ell} + \\frac{m_2 g}{2}',
          numbers: `F_e(z_e = ${fmt(zE, 3)}) = ${tex(jac.Fe)}\\;\\text{N}` },
        { title: 'Jacobian-linearized block and beam', page: 'p. 386 · E.4(b)', answers: `${id}/b`,
          theory: 'J_e = \\frac{m_2\\ell^2}{3} + m_1z_e^2:\\quad \\ddot{\\tilde z} = -g\\tilde\\theta,\\quad \\ddot{\\tilde\\theta} = \\frac{\\ell\\tilde F - m_1 g\\tilde z}{J_e},\\quad \\det(sI - A) = s^4 - \\frac{m_1g^2}{J_e}',
          numbers: `A = ${texMat(jac.A)},\\quad B = ${texMat(jac.B)},\\quad \\text{eig}(A) = \\pm${tex(lam)},\\;\\pm ${tex(lam)}j` },
        { title: 'Feedback-linearized block and beam', page: 'p. 386 · E.4(c), p. 388 · E.8(e)', answers: `${id}/c`,
          theory: 'F = F_{fl}(z) + \\tilde F,\\; F_{fl} = \\frac{m_1 g z}{\\ell} + \\frac{m_2 g}{2} \\;\\Rightarrow\\; \\Big(\\frac{m_2\\ell^2}{3} + m_1z^2\\Big)\\ddot\\theta = \\ell\\cos\\theta\\,\\tilde F - 2m_1z\\dot z\\dot\\theta',
          note: 'The z equation has no F in it, so it stays nonlinear: only the gravity terms of the θ equation are cancelled. Near θ = 0 this gives J_e θ̈ = ℓF̃ with no m₁g z̃ term (the E.5(c) model).' },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibria',
          html: 'Which states can be equilibria, and what force holds the system there? Write the equilibrium state x<sub>e</sub> = (z, θ, ż, θ̇) and the force F<sub>e</sub> as functions of the block position z<sub>e</sub>. Take the configuration of Fig. 20-1 (|θ| &lt; 90°).',
          code: pyPart(ctx, {
            items: [
              { fn: 'x_e', args: ['z_e'], truth: (p, a) => [[a.z_e], [0], [0], [0]] },
              { fn: 'F_e', args: ['z_e'], truth: (p, a) => ctx.sys.linear(p, { z: a.z_e }).Fe },
            ],
            explain: (it, f) => {
              const g = [].concat(...[].concat(f.e.got)).map(Number);
              return it.fn === 'x_e' && g.length === 4 && Math.abs(Math.abs(g[1]) - Math.PI) < 1e-6 ? 'θ = π also makes f(x, F) = 0, but that turns the beam around; use the configuration of Fig. 20-1.' : '';
            },
          }, 'def x_e(z_e):\n    # equilibrium state (z, theta, zdot,\n    # thetadot) with the block at z_e\n    return ...\n\ndef F_e(z_e):\n    # force that holds it there\n    return ...\n'),
          actions: [{ label: 'Use my x_e, F_e', run: (code) => useMyEquilibrium(ctx, code) }],
          solution: () => [
            { html: 'ẋ = 0 needs ż = θ̇ = 0; then z̈ = 0 ⇒ sin θ = 0 ⇒ θ<sub>e</sub> = 0, and θ̈ = 0 ⇒ F<sub>e</sub>ℓ = m₁gz<sub>e</sub> + m₂gℓ/2. Any z<sub>e</sub> on the beam works.' },
            { tex: 'x_e = (z_e, 0, 0, 0)^\\top,\\quad F_e = \\frac{m_1 g z_e}{\\ell} + \\frac{m_2 g}{2}' },
            { code: 'def x_e(z_e):\n    return np.array([[z_e], [0], [0], [0]])\n\ndef F_e(z_e):\n    return (P.m1 * P.g * z_e / P.ell\n            + P.m2 * P.g / 2)' },
          ],
        },
        {
          id: 'b', title: '(b) Jacobian linearization',
          html: 'With x̃ = x − x<sub>e</sub> = (z̃, θ̃, z̃̇, θ̃̇)ᵀ and F̃ = F − F<sub>e</sub>, return the matrices of x̃̇ = A x̃ + B F̃ about the equilibrium with the block at z<sub>e</sub>.',
          code: pyPart(ctx, {
            items: [
              { fn: 'A_jac', args: ['z_e'], truth: (p, a) => ctx.sys.linear(p, { z: a.z_e }).A },
              { fn: 'B_jac', args: ['z_e'], truth: (p, a) => ctx.sys.linear(p, { z: a.z_e }).B },
            ],
          }, 'def A_jac(z_e):\n    return ...\n\ndef B_jac(z_e):\n    return ...\n'),
          actions: [{ label: 'Plot my A, B', run: (code) => { ctx.st.method = 'jacobian'; ctx.update(); return Y.plot(ctx, 'ch4.b', code, runJacobian, () => 'Your eigenvalues are the × markers on the s-plane, and your linear model (from the same δz(0), δθ(0) and F̃) is dotted on the z and θ plots. They follow zₑ and the input.'); } }],
          solution: () => [
            { tex: 'J_e = \\frac{m_2\\ell^2}{3} + m_1z_e^2:\\quad \\ddot{\\tilde z} = -g\\tilde\\theta,\\quad J_e\\ddot{\\tilde\\theta} = \\ell\\tilde F - m_1 g\\tilde z' },
            { html: 'The zθ̇², 2m₁zżθ̇ and sin θ·(…) terms all vanish to first order at the equilibrium, because each is a product with a zero factor there. The θ̈ numerator is zero at the equilibrium, so only its derivatives divided by J<sub>e</sub> remain.' },
            { code: 'def A_jac(z_e):\n    Je = P.m2 * P.ell**2 / 3 + P.m1 * z_e**2\n    return np.array([\n        [0, 0, 1, 0],\n        [0, 0, 0, 1],\n        [0, -P.g, 0, 0],\n        [-P.m1 * P.g / Je, 0, 0, 0]])\n\ndef B_jac(z_e):\n    Je = P.m2 * P.ell**2 / 3 + P.m1 * z_e**2\n    return np.array([[0], [0], [0],\n                     [P.ell / Je]])' },
          ],
        },
        {
          id: 'c', title: '(c) Feedback linearization, if possible',
          html: 'Choose F = F<sub>fl</sub>(z, θ, ż, θ̇) + F̃ so that, with F̃ = 0, the beam stays at rest for every block position and angle, and return the resulting θ̈. Any F<sub>fl</sub> that does this is accepted. Then ask yourself whether the z equation can be linearized the same way.',
          code: { template: 'def F_fl(z, theta, zdot, thetadot):\n    return ...\n\ndef thetaddot_fl(z, theta, zdot,\n                 thetadot, F_tilde):\n    # theta_ddot when F = F_fl + F_tilde\n    return ...\n', check: (code) => checkFeedbackLin(ctx, code),
            actions: [{ label: 'Simulate my F_fl', run: (code) => { ctx.st.method = 'fl'; ctx.update(); return Y.plot(ctx, 'ch4.c', code, runFeedbackLin, () => 'Dotted on the z and θ plots: the block and beam with your F = F_fl + F̃, and (long dashes) your thetaddot_fl model driven by F̃, with z̈ from the plant\'s z equation. With a correct answer they overlap. The × markers are the eigenvalues of that model linearized at zₑ.'); } }] },
          solution: () => [
            { tex: 'F = \\frac{m_1 g z}{\\ell} + \\frac{m_2 g}{2} + \\tilde F \\;\\Rightarrow\\; \\Big(\\frac{m_2\\ell^2}{3} + m_1z^2\\Big)\\ddot\\theta = \\ell\\cos\\theta\\,\\tilde F - 2m_1z\\dot z\\dot\\theta' },
            { html: 'Gravity on the block and the beam both enter the θ equation multiplied by cos θ, like F, so F<sub>fl</sub>(z) cancels them for every θ. The z equation, z̈ = zθ̇² − g sin θ, has no F in it, so it cannot be linearized this way: the system is only partly feedback-linearized. Near θ = 0 the θ equation becomes J<sub>e</sub>θ̈ = ℓF̃ with no m₁g z̃ term, the model E.5(c) uses, and E.8(e) uses this F<sub>fl</sub> with the measured z.' },
            { code: 'def F_fl(z, theta, zdot, thetadot):\n    return (P.m1 * P.g * z / P.ell\n            + P.m2 * P.g / 2)\n\ndef thetaddot_fl(z, theta, zdot,\n                 thetadot, F_tilde):\n    J = P.m2 * P.ell**2 / 3 + P.m1 * z**2\n    return (P.ell * np.cos(theta) * F_tilde\n            - 2 * P.m1 * z * zdot\n            * thetadot) / J' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------- Chapters 5 and 6 --
  // Open-loop pulse with F_fl(z) added: the beam sees ≈ b0/s² and the block −g/s².
  function pulseSim() {
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
      ...bind(ctx, 'model'),
    });
  }
  const ZE_NOTE = 'Linearize about the block at z<sub>e</sub> = ℓ/2, the design point E.8 uses.';
  // Transfer functions of the linear model at complex s, from the Jacobian A, B.
  const cx = () => WB.py.cx;
  function tfs(ctx, p, s, comp = 'eq') {
    const { A, B } = ctx.sys.linear(p, { comp });
    const a32 = A[2][1], a41 = A[3][0], b4 = B[3][0], C = cx();
    const s2 = C.mul(s, s), den = C.add(C.mul(s2, s2), -a32 * a41);
    return { thF: C.div(C.mul(b4, s2), den), zF: C.div(a32 * b4, den), zTh: C.div(a32, s2) };
  }

  // E.5: the student's transfer functions, each fitted with the lowest-order proper
  // rational function that matches it. For (a), Python solves their two transformed
  // equations for Z̃ and Θ̃ with F̃ = 1 at each sample s. (b) also draws the cascade
  // Z̃/Θ̃ · Θ̃/F̃. (a) and (b) go with the full Jacobian model, (c) with the E.5(c) one.
  const LAP_PY = `

def _wb_lap(s):
    e = lambda Z, T, F: np.array([complex(z_equation(s, Z, T, F)), complex(theta_equation(s, Z, T, F))])
    e0 = e(0, 0, 0)
    M = np.column_stack([e(1, 0, 0) - e0, e(0, 1, 0) - e0])
    if abs(np.linalg.det(M)) <= 1e-12 * max(1.0, np.abs(M).max()) ** 2:
        raise ValueError('your two equations do not determine Z and Theta for a given F')
    return np.linalg.solve(M, -(e(0, 0, 1) - e0))

def _wb_lap_th(s):
    return _wb_lap(s)[1]

def _wb_lap_z(s):
    return _wb_lap(s)[0]

def _wb_casc(s):
    return Z_over_Theta(s) * Theta_over_F(s)
`;
  const TF_PARTS = {
    a: { fns: ['_wb_lap_th', '_wb_lap_z'], names: ['Θ̃/F̃ from your equations', 'Z̃/F̃ from your equations'] },
    b: { fns: ['Theta_over_F', 'Z_over_F'], names: ['your Θ̃/F̃', 'your Z̃/F̃'], casc: true },
    c: { fns: ['Theta_over_F', 'Z_over_F'], names: ['your simplified Θ̃/F̃', 'your simplified Z̃/F̃'] },
  };
  async function runTfs(ctx, code, part) {
    const spec = TF_PARTS[part], src = code + LAP_PY;
    const fit = async (fn, name) => {
      const r = await Y.fitTf(ctx, src, fn, { params: withLength(ctx.pModel), maxN: 6 });
      return r.ok === false && r.msg ? { ...r, msg: r.msg.split(fn).join(name) } : r;
    };
    const th = await fit(spec.fns[0], spec.names[0]);
    if (th.ok === false) return th;
    const z = await fit(spec.fns[1], spec.names[1]);
    if (z.ok === false) return z;
    const out = { thF: th.G, zF: z.G, names: spec.names };
    if (spec.casc) {
      const c = await fit('_wb_casc', 'Z_over_Theta(s) * Theta_over_F(s)');
      if (c.ok === false) return c;
      out.casc = c.G;
    }
    return out;
  }
  // The overlay that goes with the model chosen on the right: the last of (a), (b)
  // plotted for the full model, (c) for the E.5(c) one.
  const last5 = { full: 'ch5.b' };
  const yours5 = (ctx) => Y.data(ctx, ctx.st.model === 'full' ? last5.full : 'ch5.c');
  function plotTfs(ctx, code, part) {
    ctx.st.model = part === 'c' ? 'simple' : 'full';
    if (part !== 'c') last5.full = `ch5.${part}`;
    ctx.update();
    return Y.plot(ctx, `ch5.${part}`, code, (c2, cd) => runTfs(c2, cd, part), (d) => {
      const n = d.thF.den.length - 1;
      return `Your Θ̃/F̃ has ${n} pole${n === 1 ? '' : 's'} (× on the s-plane, with those of Z̃/F̃). Their responses to the same F̃, from rest, are dotted on the z and θ plots${d.casc ? ', and Z̃/Θ̃ · Θ̃/F̃ (long dashes) on the z plot' : ''}. The ${part === 'c' ? 'E.5(c)' : 'full Jacobian'} model is selected on the right.`;
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
      WB.E.ffWorkControls(sec, ctx);
      sec.append(el('p', { class: 'muted small', text: 'A force pulse spins the beam up, and the tilt makes the block accelerate. The dashed trace is the linear model you choose above.' }));
    },
    splane(ctx) {
      const id = ctx.sys.problems.ch5.id, full = ctx.st.model === 'full';
      let sp = { markers: [], fitR: 2 };
      if (showsAnswer(ctx, full ? `${id}/b` : `${id}/c`)) {
        const lin = ctx.sys.linear(ctx.pModel, { comp: full ? 'eq' : 'fl' });
        sp = { markers: L.eig(lin.A).map((p) => ({ ...p, kind: 'ol', label: `pole (${full ? 'full' : 'E.5(c)'} model)` })), fitR: 2 };
      }
      const y = yours5(ctx);
      if (!y) return sp;
      return Y.withMarkers(sp, [
        ...Y.poles(WB.tf.poles(y.thF), `pole of ${y.names[0]}`), ...Y.zeros(WB.tf.zeros(y.thF), `zero of ${y.names[0]}`),
        ...Y.poles(WB.tf.poles(y.zF), `pole of ${y.names[1]}`),
      ], { obs: 'pole of your transfer functions', zero: `zero of ${y.names[0]}` });
    },
    // Your transfer functions driven by the same F̃ from rest, about z_e.
    outputSeries(ctx, res, sc, oi) {
      const y = yours5(ctx);
      if (!y) return [];
      const ref = sc(res.yAll[oi]), tr = (G) => Y.tfResponse(ctx, G, fin(ctx)), ze = ctx.sys.ze(ctx.pModel);
      if (oi === 1) return [ser(y.names[0], sc(tr(y.thF)), ref)];
      const out = [ser(y.names[1], Array.from(tr(y.zF), (v) => v + ze), ref)];
      if (y.casc) out.push(ser('your Z̃/Θ̃ · Θ̃/F̃', Array.from(tr(y.casc), (v) => v + ze), ref, { dash: [8, 3] }));
      return out;
    },
    math(ctx) {
      const lin = ctx.sys.linear(ctx.pModel, { comp: 'eq' });
      const a = ctx.pModel.m1 * ctx.pModel.g ** 2 / lin.De, id = ctx.sys.problems.ch5.id;
      return [
        { title: 'Laplace transform', page: 'p. 69–70',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)' },
        { title: 'Transfer function', page: 'p. 70–71',
          theory: '\\text{zero initial conditions:}\\quad P(s) = \\frac{Y(s)}{U(s)}' },
        { title: 'Cascade of transfer functions', page: 'p. 72–74 · §5.3',
          theory: 'Y(s) = P_2(s)\\,W(s),\\; W(s) = P_1(s)\\,U(s) \\;\\Rightarrow\\; \\frac{Y(s)}{U(s)} = P_2(s)\\,P_1(s)' },
        { title: 'Linearized equations of motion (E.4)', page: 'p. 386 · E.4(b)', answers: `${ctx.sys.problems.ch4.id}/b`,
          theory: '\\ddot{\\tilde z} = -g\\tilde\\theta,\\quad J_e\\ddot{\\tilde\\theta} = \\ell\\tilde F - m_1 g\\tilde z,\\quad J_e = \\frac{m_2\\ell^2}{3} + m_1z_e^2' },
        { title: 'Transformed equations of motion', page: 'p. 386 · E.5(a)', answers: `${id}/a`,
          theory: 's^2\\tilde Z(s) = -g\\tilde\\Theta(s),\\quad J_e s^2\\tilde\\Theta(s) = \\ell\\tilde F(s) - m_1 g\\tilde Z(s)' },
        { title: 'Transfer functions of the block and beam', page: 'p. 387 · E.5(b)', answers: `${id}/b`,
          theory: '\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{\\frac{\\ell}{J_e}s^2}{s^4 - \\frac{m_1g^2}{J_e}},\\quad \\frac{\\tilde Z}{\\tilde F} = \\frac{-\\frac{g\\ell}{J_e}}{s^4 - \\frac{m_1g^2}{J_e}},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = -\\frac{g}{s^2}',
          numbers: `\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{${tex(lin.b0)}\\,s^2}{s^4 - ${tex(a)}},\\quad \\frac{\\tilde Z}{\\tilde F} = \\frac{${tex(-ctx.pModel.g * lin.b0)}}{s^4 - ${tex(a)}}` },
        { title: 'Simplified cascade (m₁g z̃ dropped)', page: 'p. 387 · E.5(c, d)', answers: `${id}/c`,
          theory: 'P_{in}(s) = \\frac{\\tilde\\Theta}{\\tilde F} = \\frac{\\ell / (\\frac{m_2\\ell^2}{3} + m_1 z_e^2)}{s^2},\\quad P_{out}(s) = \\frac{\\tilde Z}{\\tilde\\Theta} = -\\frac{g}{s^2}',
          numbers: `P_{in} = \\frac{${tex(lin.b0)}}{s^2},\\quad P_{out} = \\frac{${tex(-ctx.pModel.g)}}{s^2}` },
      ];
    },

    buildProblem(parent, ctx) {
      const s = ['s'];
      const JE = 'Je = (P.m2 * P.ell**2 / 3\n      + P.m1 * (P.ell / 2)**2)\n';
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'a', title: '(a) Laplace transform of the linearized equations',
          html: `${ZE_NOTE} Use zero initial conditions. Return each transformed equation as (left side − right side), so it is zero when the equation holds: first the z̃ equation, then the θ̃ equation, as they come out of the transform (no dividing through by s). Each is accepted up to a constant factor. The check calls them at complex values of s, Z̃(s), Θ̃(s) and F̃(s).`,
          code: pyPart(ctx, {
            items: [
              { fn: 'z_equation', args: ['s', 'Z', 'Theta', 'Fs'], compare: 'scale', truth: (p, a) => { const { A } = ctx.sys.linear(p), X = cx(); return X.add(X.mul(X.mul(a.s, a.s), a.Z), X.mul(-A[2][1], a.Theta)); } },
              { fn: 'theta_equation', args: ['s', 'Z', 'Theta', 'Fs'], compare: 'scale', truth: (p, a) => { const { A, B } = ctx.sys.linear(p), X = cx(); return X.add(X.add(X.mul(X.mul(a.s, a.s), a.Theta), X.mul(-A[3][0], a.Z)), X.mul(-B[3][0], a.Fs)); } },
            ],
          }, 'def z_equation(s, Z, Theta, F):\n    # (left - right) of the z equation\n    return ...\n\ndef theta_equation(s, Z, Theta, F):\n    # (left - right) of the theta equation\n    return ...\n'),
          actions: [{ label: 'Plot my equations', run: (code) => plotTfs(ctx, code, 'a') }],
          solution: () => [
            { tex: 's^2\\tilde Z(s) + g\\tilde\\Theta(s) = 0,\\quad J_e s^2\\tilde\\Theta(s) + m_1 g\\tilde Z(s) - \\ell\\tilde F(s) = 0,\\quad J_e = \\frac{m_2\\ell^2}{3} + m_1\\frac{\\ell^2}{4}' },
            { code: `${JE}\ndef z_equation(s, Z, Theta, F):\n    return s**2 * Z + P.g * Theta\n\ndef theta_equation(s, Z, Theta, F):\n    return (Je * s**2 * Theta\n            + P.m1 * P.g * Z - P.ell * F)` },
          ],
        },
        {
          id: 'b', title: '(b) Transfer functions',
          html: `${ZE_NOTE} Return Θ̃(s)/F̃(s), Z̃(s)/F̃(s) and Z̃(s)/Θ̃(s) evaluated at a complex s (the check calls them at several).`,
          code: pyPart(ctx, {
            items: [
              { fn: 'Theta_over_F', args: s, truth: (p, a) => tfs(ctx, p, a.s).thF },
              { fn: 'Z_over_F', args: s, truth: (p, a) => tfs(ctx, p, a.s).zF },
              { fn: 'Z_over_Theta', args: s, truth: (p, a) => tfs(ctx, p, a.s).zTh },
            ],
          }, 'def Theta_over_F(s):\n    return ...\n\ndef Z_over_F(s):\n    return ...\n\ndef Z_over_Theta(s):\n    return ...\n'),
          actions: [{ label: 'Plot my transfer functions', run: (code) => plotTfs(ctx, code, 'b') }],
          solution: () => [
            { tex: '\\tilde Z = -\\frac{g}{s^2}\\tilde\\Theta \\Rightarrow J_e s^2\\tilde\\Theta = \\ell\\tilde F + \\frac{m_1 g^2}{s^2}\\tilde\\Theta \\Rightarrow \\frac{\\tilde\\Theta}{\\tilde F} = \\frac{(\\ell/J_e)s^2}{s^4 - m_1g^2/J_e},\\quad \\frac{\\tilde Z}{\\tilde F} = \\frac{-g\\ell/J_e}{s^4 - m_1g^2/J_e}' },
            { code: `${JE}den = lambda s: s**4 - P.m1 * P.g**2 / Je\n\ndef Theta_over_F(s):\n    return P.ell / Je * s**2 / den(s)\n\ndef Z_over_F(s):\n    return -P.g * P.ell / Je / den(s)\n\ndef Z_over_Theta(s):\n    return -P.g / s**2` },
          ],
        },
        {
          id: 'c', title: '(c) Simplified transfer functions',
          html: 'Drop the m<sub>1</sub>g z̃ term from the second equation of motion, as the problem says, and return the simplified Θ̃(s)/F̃(s) and Z̃(s)/F̃(s) at complex s.',
          code: pyPart(ctx, {
            items: [
              { fn: 'Theta_over_F', args: s, truth: (p, a) => tfs(ctx, p, a.s, 'fl').thF },
              { fn: 'Z_over_F', args: s, truth: (p, a) => tfs(ctx, p, a.s, 'fl').zF },
            ],
          }, 'def Theta_over_F(s):\n    return ...\n\ndef Z_over_F(s):\n    return ...\n'),
          actions: [{ label: 'Plot my simplified model', run: (code) => plotTfs(ctx, code, 'c') }],
          solution: () => [
            { tex: '\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{b_0}{s^2},\\; b_0 = \\frac{\\ell}{\\frac{m_2\\ell^2}{3} + m_1 z_e^2},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = -\\frac{g}{s^2}\\;(\\text{unchanged}),\\quad \\frac{\\tilde Z}{\\tilde F} = -\\frac{g b_0}{s^4}' },
            { code: `${JE}b0 = P.ell / Je\n\ndef Theta_over_F(s):\n    return b0 / s**2\n\ndef Z_over_F(s):\n    return -P.g * b0 / s**4` },
          ],
        },
        {
          id: 'd', title: '(d) Block diagram of the cascade',
          html: 'On paper: draw the simplified system as a cascade from F̃(s) to Θ̃(s) and then to Z̃(s).',
          done: 'I have drawn it',
          solution: () => [{ html: 'F̃ → [b₀/s²] → Θ̃ → [−g/s²] → Z̃. The inner (beam) and outer (block) blocks are each a double integrator, which sets up the successive loop closure of E.8.' }],
        },
      ]);
    },
  });

  // E.6: the student's A, B, C, D.
  async function runSS(ctx, code) {
    const out = await WB.py.evaluate(code, [{ params: withLength(ctx.pModel), vars: ['A', 'B', 'C', 'D'] }]);
    if (out.error) return Y.pyError(out);
    const v = out.rows[0].vars;
    const A = Y.asMat(v.A, 4, 4), B = Y.asMat(v.B, 4, 1), C = Y.asMat(v.C, 2, 4), D = Y.asMat(v.D, 2, 1);
    const bad = [[A, 'A', '4×4'], [B, 'B', '4×1'], [C, 'C', '2×4'], [D, 'D', '2×1']].find(([m]) => !m);
    if (bad) return { ok: false, msg: `${bad[1]} should be a ${bad[2]} array of numbers (x̃ = (z̃, θ̃, z̃̇, θ̃̇), ũ = F̃, ỹ = (z̃, θ̃)).` };
    return { A, B, C, D };
  }

  CH.ch6 = Object.assign({}, common, pulseSim(), {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 81–93',
    defaults() { return { comp: 'eq', model: 'full', inp: { shape: 'pulse', amp: 0.05, freq: 1, width: 0.2 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Input ũ = F̃(t)', 'p. 387 · E.6');
      modelToggle(sec, ctx);
      inputControls(sec, ctx);
      WB.E.ffWorkControls(sec, ctx);
    },
    splane(ctx) {
      let sp = { markers: [], fitR: 2 };
      // The E.5(c) model's eigenvalues also answer E.5(c).
      if (showsAnswer(ctx, `${ctx.sys.problems.ch6.id}/a`) && (ctx.st.model === 'full' || showsAnswer(ctx, `${ctx.sys.problems.ch5.id}/c`))) {
        const lin = ctx.sys.linear(ctx.pModel, { comp: ctx.st.model === 'full' ? 'eq' : 'fl' });
        sp = { markers: L.eig(lin.A).map((p, i) => ({ ...p, kind: 'ol', label: `eigenvalue of A ${i + 1}` })), fitR: 2 };
      }
      const y = Y.data(ctx, 'ch6.a');
      return y ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your A'), { obs: 'eigenvalue of your A' }) : sp;
    },
    // Your model from the simulation's initial state (as a deviation from x_e = (ℓ/2, 0, 0, 0)),
    // driven by the same F̃. Its outputs cover both channels, so the velocity plot is left alone.
    outputSeries(ctx, res, sc, oi) {
      const y = Y.data(ctx, 'ch6.a');
      if (!y) return [];
      const ze = ctx.sys.ze(ctx.pModel);
      const r = Y.linResponse(ctx, y, res.x[0].map((v, i) => (i === 0 ? v - ze : v)), fin(ctx));
      return [ser('your A, B, C, D', outTrace(r, oi, sc, ze), sc(res.yAll[oi]))];
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
        { title: 'Back to the transfer function', page: 'p. 85 · Eq. 6.14–6.15',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\text{poles: } \\det(sI - A) = 0' },
        { title: 'State-space model of the block and beam', page: 'p. 387 · E.6', answers: `${ctx.sys.problems.ch6.id}/a`,
          theory: 'A = \\begin{bmatrix}0&0&1&0\\\\0&0&0&1\\\\0&-g&0&0\\\\-\\frac{m_1g}{J_e}&0&0&0\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\0\\\\0\\\\ \\frac{\\ell}{J_e}\\end{bmatrix},\\quad C = \\begin{bmatrix}1&0&0&0\\\\0&1&0&0\\end{bmatrix},\\quad D = \\begin{bmatrix}0\\\\0\\end{bmatrix}',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(C)},\\quad \\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}` },
      ];
    },

    buildProblem(parent, ctx) {
      const ss = (p) => ctx.sys.stateSpace(p);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'A, B, C, D',
        html: 'Linearize about the block at z<sub>e</sub> = ℓ/2 with F = F<sub>e</sub> + F̃ (the Jacobian model of E.4(b)). Define A, B, C and D as numpy arrays with the shapes of the state-space equations.',
        code: pyPart(ctx, {
          items: ['A', 'B', 'C', 'D'].map((k) => ({ var: k, truth: (p) => ss(p)[k] })),
        }, '# x = (z, theta, zdot, thetadot),\n# u = F_tilde, y = (z, theta)\nA = ...\nB = ...\nC = ...\nD = ...\n'),
        actions: [{ label: 'Plot my model', run: (code) => { ctx.st.model = 'full'; ctx.update(); return Y.plot(ctx, 'ch6.a', code, runSS, () => 'Your eigenvalues of A are the × markers on the s-plane; your model\'s z and θ, from the same initial state and F̃, are dotted on the output plots (next to the full Jacobian model, now selected on the right).'); } }],
        solution: () => { const { A, B } = ss(ctx.pModel); return [
          { tex: `J_e = \\frac{m_2\\ell^2}{3} + m_1\\frac{\\ell^2}{4}:\\quad A = \\begin{bmatrix}0&0&1&0\\\\0&0&0&1\\\\0&-g&0&0\\\\-\\frac{m_1g}{J_e}&0&0&0\\end{bmatrix} = ${texMat(A)},\\quad B = \\begin{bmatrix}0\\\\0\\\\0\\\\ \\frac{\\ell}{J_e}\\end{bmatrix} = ${texMat(B)}` },
          { tex: 'C = \\begin{bmatrix}1&0&0&0\\\\0&1&0&0\\end{bmatrix},\\quad D = \\begin{bmatrix}0\\\\0\\end{bmatrix}' },
          { code: 'Je = (P.m2 * P.ell**2 / 3\n      + P.m1 * (P.ell / 2)**2)\nA = np.array([[0, 0, 1, 0],\n              [0, 0, 0, 1],\n              [0, -P.g, 0, 0],\n              [-P.m1 * P.g / Je, 0, 0, 0]])\nB = np.array([[0], [0], [0], [P.ell / Je]])\nC = np.array([[1, 0, 0, 0],\n              [0, 1, 0, 0]])\nD = np.array([[0], [0]])' },
        ]; },
      }]);
    },
  });

  WB.E.models = { inputForce, openLoop, showsAnswer };
})();
