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
      slider(sec, { label: 'z amplitude', unit: 'm', min: 0, max: 0.25, step: 0.005, sig: 3, ...bind(ctx, 'Az') });
      slider(sec, { label: 'θ amplitude', unit: '°', min: 0, max: 45, step: 0.5, sig: 3, ...bind(ctx, 'Ath') });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'f') });
      sec.append(el('p', { class: 'muted small', text: 'z(t) = z_e + A_z sin(2πft), θ(t) = A_θ sin(πft). No dynamics: the motion is imposed, as in hw02. Once E.2(a) is solved (or in Explore mode) the energy plot splits K into its parts.' }));
    },

    // The split into parts shows the form of K, so in Work mode it waits for E.2(a).
    extraPlot(ctx, res) {
      const mJ = (a) => Array.from(a, (v) => v * 1000);
      const split = showsAnswer(ctx, `${ctx.sys.problems.ch2.id}/a`);
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [mJ]', unit: 'mJ' },
        data: { series: [
          ...(split ? [
            { label: 'block, along the beam', y: mJ(res.extras.Kr), color: '--series-2', width: 1.5 },
            { label: 'block, carried by the beam', y: mJ(res.extras.Kt), color: '--series-3', width: 1.5 },
            { label: 'beam', y: mJ(res.extras.Kb), color: '--text-muted', width: 1.5 },
          ] : []),
          { label: 'total K', y: mJ(res.extras.K), color: '--series-1' },
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
          code: pyPart(ctx, {
            items: [{ fn: 'kinetic', args: ['z', 'theta', 'zdot', 'thetadot'], truth: (p, a) => ctx.sys.kinetic(xOf(a), p) }],
          }, 'def kinetic(z, theta, zdot, thetadot):\n    # kinetic energy of the system\n    K = ...\n    return K\n'),
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

    // E.3(e): the student's f, simulated with the same input as the system above.
    outputSeries(ctx, res, sc, oi) {
      const m = mine.ch3;
      if (!m || m.sig !== simSig(ctx, res)) return [];
      return [{ label: 'your f (Python)', y: sc(oi === 0 ? m.z : m.theta), color: '--series-2', dash: [5, 4], width: 2 }];
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
          code: pyPart(ctx, {
            items: [{ fn: 'potential', args: ['z', 'theta'], compare: 'offset', truth: (p, a) => ctx.sys.potential([a.z, a.theta, 0, 0], p) }],
          }, 'def potential(z, theta):\n    # potential energy of the system\n    return ...\n'),
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
    },

    outputSeries(ctx, res, sc, oi) {
      return oi === 0 ? [{ label: 'zₑ', y: Array.from(res.t, () => ctx.st.zE), color: '--ref', dash: [6, 4], width: 1.5 }] : [];
    },

    splane(ctx) {
      const id = ctx.sys.problems.ch4.id;
      if (!showsAnswer(ctx, ctx.st.method === 'fl' ? `${id}/c` : `${id}/b`)) return { markers: [], fitR: 2 };
      const { A } = this.lin(ctx);
      return { markers: L.eig(A).map((p) => ({ ...p, kind: 'ol', label: `eigenvalue of A (${ctx.st.method})` })), fitR: 2 };
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
          solution: () => [
            { tex: 'J_e = \\frac{m_2\\ell^2}{3} + m_1z_e^2:\\quad \\ddot{\\tilde z} = -g\\tilde\\theta,\\quad J_e\\ddot{\\tilde\\theta} = \\ell\\tilde F - m_1 g\\tilde z' },
            { html: 'The zθ̇², 2m₁zżθ̇ and sin θ·(…) terms all vanish to first order at the equilibrium, because each is a product with a zero factor there. The θ̈ numerator is zero at the equilibrium, so only its derivatives divided by J<sub>e</sub> remain.' },
            { code: 'def A_jac(z_e):\n    Je = P.m2 * P.ell**2 / 3 + P.m1 * z_e**2\n    return np.array([\n        [0, 0, 1, 0],\n        [0, 0, 0, 1],\n        [0, -P.g, 0, 0],\n        [-P.m1 * P.g / Je, 0, 0, 0]])\n\ndef B_jac(z_e):\n    Je = P.m2 * P.ell**2 / 3 + P.m1 * z_e**2\n    return np.array([[0], [0], [0],\n                     [P.ell / Je]])' },
          ],
        },
        {
          id: 'c', title: '(c) Feedback linearization, if possible',
          html: 'Choose F = F<sub>fl</sub>(z, θ, ż, θ̇) + F̃ so that, with F̃ = 0, the beam stays at rest for every block position and angle, and return the resulting θ̈. Any F<sub>fl</sub> that does this is accepted. Then ask yourself whether the z equation can be linearized the same way.',
          code: { template: 'def F_fl(z, theta, zdot, thetadot):\n    return ...\n\ndef thetaddot_fl(z, theta, zdot,\n                 thetadot, F_tilde):\n    # theta_ddot when F = F_fl + F_tilde\n    return ...\n', check: (code) => checkFeedbackLin(ctx, code) },
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
      const id = ctx.sys.problems.ch5.id, full = ctx.st.model === 'full';
      if (!showsAnswer(ctx, full ? `${id}/b` : `${id}/c`)) return { markers: [], fitR: 2 };
      const lin = ctx.sys.linear(ctx.pModel, { comp: full ? 'eq' : 'fl' });
      return { markers: L.eig(lin.A).map((p) => ({ ...p, kind: 'ol', label: `pole (${full ? 'full' : 'E.5(c)'} model)` })), fitR: 2 };
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
      if (!showsAnswer(ctx, `${ctx.sys.problems.ch6.id}/a`)) return { markers: [], fitR: 2 };
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
