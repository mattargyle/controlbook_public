// Study F, Chapters 2–6 (F.2–F.6): kinetic energy, Euler-Lagrange equations,
// equilibria and linearization, transfer functions, state-space models.
// Open-loop experiments: the rotor forces come from an input program in (F, τ).
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const F = WB.F;
  const { tex, texMat, texPole, fmt } = M;
  const PD = () => WB.pd;
  const sysF = () => WB.systems.F;
  const { pyPart, pyError, showsAnswer, negated } = F;
  const cx = () => WB.py.cx;

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
    const fl = F.compOf(ctx) === 'fl';
    return {
      update(r, x, y, t) {
        const Ft = prog(st.inF, t), tau = prog(st.inT, t);
        let Fc = Ft;
        if (!linear && st.hover) Fc = fl ? (m.Fe + Ft) / Math.cos(x[2]) : m.Fe + Ft;
        return { u: s.mix(Fc, tau, p), Ftilde: Ft, tau };
      },
    };
  }
  const SHAPES = { zero: 'none', const: 'constant', pulse: 'pulse', doublet: 'doublet', square: 'square', sine: 'sine' };
  function progControls(parent, ctx, key, label, unit, ampMax) {
    segmented(parent, {
      label, options: ['zero', 'const', 'pulse', 'doublet', 'square', 'sine'].map((v) => ({ value: v, label: SHAPES[v] })),
      ...bind(ctx, 'shape', () => ctx.st[key]),
    });
    const off = () => ctx.st[key].shape === 'zero';
    slider(parent, { label: 'amplitude', unit, min: -ampMax, max: ampMax, step: ampMax / 400, sig: 3, ...bind(ctx, 'amp', () => ctx.st[key]), disabled: off });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.01, max: 2, step: 0.005, sig: 3, ...bind(ctx, 'freq', () => ctx.st[key]), disabled: () => !['square', 'sine'].includes(ctx.st[key].shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.05, max: 5, step: 0.05, sig: 3, ...bind(ctx, 'width', () => ctx.st[key]), disabled: () => !['pulse', 'doublet'].includes(ctx.st[key].shape) });
  }
  function hoverControls(parent, ctx) {
    segmented(parent, {
      label: 'Add hover force F<sub>e</sub>',
      options: [{ value: true, label: 'F = F<sub>e</sub> + F̃' }, { value: false, label: 'F = F̃ only' }],
      ...bind(ctx, 'hover'),
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
      slider(sec, { label: 'z amplitude', unit: 'm', min: 0, max: 6, step: 0.05, sig: 3, ...bind(ctx, 'Az') });
      slider(sec, { label: 'h mean', unit: 'm', min: 0, max: 8, step: 0.05, sig: 3, ...bind(ctx, 'h0') });
      slider(sec, { label: 'h amplitude', unit: 'm', min: 0, max: 4, step: 0.05, sig: 3, ...bind(ctx, 'Ah') });
      slider(sec, { label: 'θ amplitude', unit: '°', min: 0, max: 60, step: 0.5, sig: 3, ...bind(ctx, 'Ath') });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.02, max: 1, step: 0.01, sig: 3, ...bind(ctx, 'f') });
      sec.append(el('p', { class: 'muted small', text: 'No dynamics here: the motion is imposed, as in an hw02 animation test, so the rotor forces are zero. The plot below shows the kinetic energy of this motion (split into its parts once F.2(a) is solved).' }));
    },
    extraPlot(ctx, res) {
      const mJ = (a) => Array.from(a);
      // the split into parts shows the form of K (F.2a)
      const split = showsAnswer(ctx, 'F.2/a') ? [
        { label: 'translation ½(m_c+2m_r)(ż²+ḣ²)', y: mJ(res.extras.Kt), color: '--series-2', width: 1.5 },
        { label: 'pod rotation ½J_cθ̇²', y: mJ(res.extras.Kc), color: '--series-3', width: 1.5 },
        { label: 'rotors m_r d²θ̇²', y: mJ(res.extras.Kr), color: '--text-muted', width: 1.5 },
      ] : [];
      return {
        opts: { title: 'kinetic energy', yLabel: 'K [J]', unit: 'J' },
        data: { series: [...split, { label: 'total K', y: mJ(res.extras.K), color: '--series-1' }] },
      };
    },
    math(ctx) {
      const p = ctx.pModel, m = ctx.sys.models(p);
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Point mass', page: 'p. 25 · Eq. 2.3 with J = 0',
          theory: 'K = \\tfrac12 m\\,\\mathbf v^\\top\\mathbf v = \\tfrac12 m\\,\\|\\dot{\\mathbf p}\\|^2',
          note: 'The kinetic energy of a system is the sum over its bodies.' },
        { title: 'Rotor positions', page: 'Fig. 21-1 p. 393', answers: 'F.2/a',
          theory: '\\mathbf p_{r} = \\begin{bmatrix} z + d\\cos\\theta \\\\ h + d\\sin\\theta\\end{bmatrix},\\quad \\mathbf p_{\\ell} = \\begin{bmatrix} z - d\\cos\\theta \\\\ h - d\\sin\\theta\\end{bmatrix},\\quad \\|\\dot{\\mathbf p}_{r,\\ell}\\|^2 = \\dot z^2 + \\dot h^2 + d^2\\dot\\theta^2 \\mp 2d\\dot\\theta(\\dot z\\sin\\theta - \\dot h\\cos\\theta)' },
        { title: 'Kinetic energy of the VTOL', page: 'F.2(a) p. 395', answers: 'F.2/a',
          theory: 'K = \\tfrac12 m_c(\\dot z^2 + \\dot h^2) + \\tfrac12 J_c\\dot\\theta^2 + \\tfrac12 m_r\\|\\dot{\\mathbf p}_r\\|^2 + \\tfrac12 m_\\ell\\|\\dot{\\mathbf p}_\\ell\\|^2,\\quad K = \\tfrac12(m_c + 2m_r)(\\dot z^2 + \\dot h^2) + \\tfrac12(J_c + 2m_r d^2)\\dot\\theta^2',
          numbers: `K = ${tex(m.M / 2)}(\\dot z^2 + \\dot h^2) + ${tex(m.J / 2)}\\,\\dot\\theta^2`,
          note: 'The cross terms of the two rotors cancel because they sit at ±d.' },
      ];
    },
    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) Kinetic energy',
          html: 'Write K as a function of the configuration variables and their rates. <code>P.mr</code> is the mass of each rotor (m<sub>r</sub> = m<sub>ℓ</sub> in the book).',
          code: pyPart(ctx, {
            items: [{ fn: 'kinetic', args: ['z', 'h', 'theta', 'zdot', 'hdot', 'thetadot'], truth: (p, a) => sysF().kinetic([a.z, a.h, a.theta, a.zdot, a.hdot, a.thetadot], p) }],
          }, 'def kinetic(z, h, theta,\n            zdot, hdot, thetadot):\n    # kinetic energy K of the VTOL\n    K = ...\n    return K\n'),
          solution: () => [
            { tex: '\\|\\dot{\\mathbf p}_r\\|^2 + \\|\\dot{\\mathbf p}_\\ell\\|^2 = 2(\\dot z^2 + \\dot h^2) + 2d^2\\dot\\theta^2 \\;(\\text{cross terms cancel})' },
            { tex: '\\displaystyle K = \\tfrac12(m_c + 2m_r)(\\dot z^2 + \\dot h^2) + \\tfrac12(J_c + 2m_r d^2)\\dot\\theta^2' },
            { code: 'def kinetic(z, h, theta,\n            zdot, hdot, thetadot):\n    m = P.mc + 2 * P.mr\n    J = P.Jc + 2 * P.mr * P.d**2\n    return (0.5 * m * (zdot**2 + hdot**2)\n            + 0.5 * J * thetadot**2)' },
            { html: 'Eq. 2.3 (p. 25) for the pod, with the rotors as point masses (p. 393).' },
          ],
        },
        {
          id: 'b', title: '(b) Animation',
          html: 'You write this class in your own <code>VTOLAnimation.py</code>; its inputs are z<sub>v</sub>, z<sub>t</sub>, h and θ. The animation at the top of this page draws the same picture (the target z<sub>t</sub> is the box on the ground) for the motion set on the right.',
        },
      ]);
    },
  }));

  // ----------------------------------------------------------- Chapter 3 --
  // Last "Simulate my f" run, shown while the simulation it was run against is unchanged.
  const mine = {};
  const uHash = (res) => res.uAppliedAll.map((u) => Array.from(u).reduce((a, v, i) => a + v * (i + 1), 0));
  const simSig = (ctx, res) => JSON.stringify([ctx.S.sysId, ctx.S.chapter, ctx.pTrue, ctx.S.sim.Ts, res.t.length, res.x[0], uHash(res)]);
  async function simulateMine(ctx, code) {
    const res = ctx.app.result();
    // u is passed as a 2×1 nested list [[f_r], [f_ℓ]], so u[0][0] works as in testDynamics.py
    const u = Array.from(res.uAppliedAll[0], (v, k) => [[v], [res.uAppliedAll[1][k]]]);
    const out = await WB.py.simulate(code, { fn: 'f', params: ctx.pTrue, x0: res.x[0], u, Ts: ctx.S.sim.Ts });
    if (out.error) return pyError(out);
    mine.ch3 = { sig: simSig(ctx, res), x: out.x };
    let dp = 0, dth = 0;
    out.x.forEach((x, k) => {
      dp = Math.max(dp, Math.abs(x[0] - res.x[k][0]), Math.abs(x[1] - res.x[k][1]));
      dth = Math.max(dth, Math.abs(x[2] - res.x[k][2]));
    });
    ctx.update();
    const ok = dp < 1e-3 && dth / M.DEG < 0.01;
    const msg = `Your f is the dashed trace on the z, h and θ plots. Largest difference from the workbench's VTOL: ${fmt(dp, 3)} m in position, ${fmt(dth / M.DEG, 3)}° in θ.`;
    return { ok, msg: ok ? msg : `${msg} They should overlap (with the wind and d_h disturbances at zero).` };
  }
  // q = (z, h, θ) in any order and spelling (z, z_v; theta, θ)
  function checkCoords(v) {
    const raw = String(v.q || '').toLowerCase().replace(/\s+/g, '').replace(/^q=/, '').replace(/[()[\]{}]|\^t|ᵀ|'/g, '');
    if (!raw) return { ok: false, msg: 'Enter the generalized coordinates.' };
    if (/dot|̇|ω|omega/.test(raw)) return { ok: false, msg: 'Generalized coordinates are configuration variables (positions and angles), not velocities.' };
    const name = (t) => ({ z: 'z', zv: 'z', z_v: 'z', h: 'h', theta: 'θ', θ: 'θ', th: 'θ' }[t] || t);
    const got = raw.split(/[,;]/).filter(Boolean).map(name);
    const ok = got.length === 3 && new Set(got).size === 3 && ['z', 'h', 'θ'].every((k) => got.includes(k));
    if (ok) return { ok: true, msg: 'Three coordinates, so q is a 3-vector; the answers below use the order (z, h, θ).' };
    return { ok: false, msg: 'Which variables fix the configuration of the VTOL? Use the minimum number.' };
  }
  const accel = (p, a) => sysF().f([a.z, a.h, a.theta, a.zdot, a.hdot, a.thetadot], [a.fr, a.fl], p).slice(3);
  const ACC_CODE = 'def accelerations(z, h, theta, zdot, hdot,\n                  thetadot, fr, fl):\n    m = P.mc + 2 * P.mr\n    J = P.Jc + 2 * P.mr * P.d**2\n    F = fr + fl\n    tau = P.d * (fr - fl)\n    zddot = (-F * np.sin(theta) - P.mu * zdot) / m\n    hddot = (-m * P.g + F * np.cos(theta)) / m\n    thetaddot = tau / J\n    return zddot, hddot, thetaddot';

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
      sec.append(el('p', { class: 'muted small', text: 'f_r = F/2 + τ/2d and f_ℓ = F/2 − τ/2d (F.4, p. 395), then each is saturated to [0, f_max]. The energy plot checks the equations of motion: E(t) − E(0) must equal the work done by the inputs minus the drag loss.' }));
    },
    extraPlot(ctx, res) {
      const { sys, pModel: p } = ctx;
      const n = res.t.length, Ts = ctx.S.sim.Ts;
      const E = new Float64Array(n), W = new Float64Array(n);
      const E0 = sys.kinetic(res.x[0], p) + sys.potential(res.x[0], p);
      // power of the generalized forces and the drag
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
          { label: 'work done by the inputs and the drag', y: W, color: '--series-2', dash: [5, 4], width: 2 },
          { label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' },
        ] },
      };
    },
    // F.3(e): the student's f, simulated with the same inputs as the VTOL above.
    outputSeries(ctx, res, sc, oi) {
      const m = mine.ch3;
      if (!m || m.sig !== simSig(ctx, res)) return [];
      return [{ label: 'your f (Python)', y: sc(m.x.map((x) => x[oi])), color: '--series-2', dash: [5, 4], width: 2 }];
    },
    math(ctx) {
      const p = ctx.pModel, m = ctx.sys.models(p);
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 43 · §3.1.4',
          theory: 'L(q, \\dot q) = K(q, \\dot q) - P(q),\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy', page: 'p. 41–42 · §3.1.1',
          theory: '\\text{gravity: } P = mgy + P_0 \\;(y = \\text{height of the mass})',
          note: 'For a rigid body, y is the height of its center of mass. The energy of a system is the sum over its bodies.' },
        { title: 'Generalized coordinates, forces, damping', page: 'p. 42–43 · §3.1.2–3.1.3',
          theory: 'q = \\text{minimum set of configuration variables},\\quad \\tau = \\text{applied (nonconservative) forces along } q,\\quad -B\\dot q = \\text{damping forces}' },
        { title: 'Potential energy of the VTOL', page: 'F.3(a) p. 395', answers: 'F.3/a',
          theory: 'P = P_0 + (m_c + 2m_r)\\,g\\,h',
          numbers: `P - P_0 = ${tex(m.M * p.g)}\\,h` },
        { title: 'Generalized forces and damping of the VTOL', page: 'F.3(b, c) p. 395', answers: ['F.3/b2', 'F.3/c'],
          theory: 'q = (z, h, \\theta)^\\top,\\quad \\tau = \\begin{bmatrix} -F\\sin\\theta \\\\ F\\cos\\theta \\\\ \\tau\\end{bmatrix},\\; F = f_r + f_\\ell,\\; \\tau = d(f_r - f_\\ell),\\quad -B\\dot q = \\begin{bmatrix}-\\mu\\dot z\\\\ 0\\\\ 0\\end{bmatrix}' },
        { title: 'Equations of motion of the VTOL', page: 'F.3(d) p. 395', answers: 'F.3/d',
          theory: '(m_c + 2m_r)\\ddot z = -F\\sin\\theta - \\mu\\dot z,\\quad (m_c + 2m_r)\\ddot h = F\\cos\\theta - (m_c + 2m_r)g,\\quad (J_c + 2m_r d^2)\\ddot\\theta = \\tau',
          numbers: `\\ddot z = ${tex(-1 / m.M)}F\\sin\\theta ${tex(-p.mu / m.M)}\\dot z,\\quad \\ddot h = ${tex(1 / m.M)}F\\cos\\theta - ${tex(p.g)},\\quad \\ddot\\theta = ${tex(1 / m.J)}\\,\\tau` },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from p. 43',
          theory: '\\frac{d}{dt}(K + P) = \\dot q^\\top\\big(\\tau - B\\dot q\\big)',
          note: 'The plot below integrates the right side and compares it with the energy. Only the inputs enter this balance: the wind and d_h sliders break it.' },
      ];
    },
    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch3, [
        {
          id: 'a', title: '(a) Potential energy',
          html: 'Write P as a function of the configuration. Any constant P<sub>0</sub> is accepted.',
          code: pyPart(ctx, {
            items: [{ fn: 'potential', args: ['z', 'h', 'theta'], compare: 'offset', truth: (p, a) => sysF().potential([a.z, a.h, a.theta, 0, 0, 0], p) }],
          }, 'def potential(z, h, theta):\n    # potential energy P of the VTOL\n    return ...\n'),
          solution: () => [
            { tex: 'P = P_0 + (m_c + m_r + m_\\ell)\\,g\\,h = P_0 + (m_c + 2m_r)\\,g\\,h' },
            { code: 'def potential(z, h, theta):\n    return (P.mc + 2 * P.mr) * P.g * h' },
            { html: 'The rotors sit at h ± d sin θ, so their θ terms cancel. Book: §3.1.1 (p. 41).' },
          ],
        },
        {
          id: 'b', title: '(b) Generalized coordinates',
          inputs: { q: 'q =' },
          html: 'Name them, comma separated (e.g. <code>x, y</code>; Greek letters or their names both work).',
          check: (v) => checkCoords(v),
          solution: () => [{ tex: 'q = (z, h, \\theta)^\\top' }, { html: 'Book: p. 42 (§3.1.2).' }],
        },
        {
          id: 'b2', title: '(b) Damping forces',
          html: 'Return the damping force along each generalized coordinate, written the way the book does, as −Bq̇ (p. 43).',
          code: pyPart(ctx, {
            items: [{ fn: 'damping_force', args: ['zdot', 'hdot', 'thetadot'], truth: (p, a) => [-p.mu * a.zdot, 0, 0] }],
            explain: (it, f) => (negated(f) ? 'Check the sign: the book writes the damping force as −Bq̇.' : ''),
          }, 'def damping_force(zdot, hdot, thetadot):\n    # -B*q_dot, one entry per coordinate\n    return ...\n'),
          solution: () => [
            { tex: '-B\\dot q = \\begin{bmatrix}-\\mu\\dot z\\\\ 0\\\\ 0\\end{bmatrix},\\quad B = \\operatorname{diag}(\\mu, 0, 0)' },
            { code: 'def damping_force(zdot, hdot, thetadot):\n    return [-P.mu * zdot, 0, 0]' },
            { html: 'Momentum drag F<sub>drag</sub> = −μż<sub>v</sub> (p. 393).' },
          ],
        },
        {
          id: 'c', title: '(c) Generalized forces',
          html: 'Return the generalized force from the rotors along each coordinate, in the order (z, h, θ).',
          code: pyPart(ctx, {
            items: [{ fn: 'generalized_force', args: ['z', 'h', 'theta', 'fr', 'fl'], truth: (p, a) => { const { F: Fn, tau } = sysF().unmix([a.fr, a.fl], p); return [-Fn * Math.sin(a.theta), Fn * Math.cos(a.theta), tau]; } }],
            explain: (it, f) => {
              if (!negated(f)) return '';
              if (f.j === 2) return 'Check the sign of the torque: a larger f_r should increase θ (Fig. 21-1).';
              if (f.j === 0) return 'Check the sign: the thrust is perpendicular to the arm, so tilting by a positive θ pushes toward −z.';
              return '';
            },
          }, 'def generalized_force(z, h, theta, fr, fl):\n    # (Q_z, Q_h, Q_theta)\n    return ...\n'),
          solution: () => [
            { tex: 'F = f_r + f_\\ell,\\; \\tau = d(f_r - f_\\ell):\\quad \\tau_q = \\begin{bmatrix} -F\\sin\\theta \\\\ F\\cos\\theta \\\\ \\tau\\end{bmatrix}' },
            { code: 'def generalized_force(z, h, theta, fr, fl):\n    F = fr + fl\n    tau = P.d * (fr - fl)\n    return [-F * np.sin(theta),\n            F * np.cos(theta),\n            tau]' },
            { html: 'The total thrust acts along the body normal (−sin θ, cos θ); the torque is about the center of mass (F.3(c) hint, p. 395).' },
          ],
        },
        {
          id: 'd', title: '(d) Equations of motion',
          html: 'Apply the Euler-Lagrange equations, then solve for the three accelerations.',
          code: pyPart(ctx, {
            cases: [
              { label: 'with f_r = f_ℓ = 0 and the VTOL at rest (only gravity acts)', fix: { fr: 0, fl: 0, zdot: 0, hdot: 0, thetadot: 0 } },
              { label: 'level (θ = 0) and at rest with f_r = f_ℓ = 4 N (thrust and gravity only)', fix: { theta: 0, zdot: 0, hdot: 0, thetadot: 0, fr: 4, fl: 4 } },
              { label: 'tilted, with f_r = f_ℓ = 4 N and ż = 0 (the thrust is tilted)', fix: { fr: 4, fl: 4, zdot: 0 } },
              { label: 'with θ = 0 and f_r = f_ℓ = 0 (only gravity and drag act)', fix: { theta: 0, fr: 0, fl: 0 } },
              { label: 'level and still, with f_r = 3 N and f_ℓ = 1 N (a torque acts)', fix: { theta: 0, zdot: 0, fr: 3, fl: 1 } },
              { label: '' },
            ],
            items: [{ fn: 'accelerations', args: ['z', 'h', 'theta', 'zdot', 'hdot', 'thetadot', 'fr', 'fl'], truth: accel }],
          }, 'def accelerations(z, h, theta, zdot, hdot,\n                  thetadot, fr, fl):\n    zddot = ...\n    hddot = ...\n    thetaddot = ...\n    return zddot, hddot, thetaddot\n'),
          solution: () => [
            { tex: 'M(q) = \\operatorname{diag}(m_c + 2m_r,\\; m_c + 2m_r,\\; J_c + 2m_r d^2),\\quad \\frac{\\partial P}{\\partial q} = \\begin{bmatrix}0\\\\ (m_c + 2m_r)g\\\\ 0\\end{bmatrix}' },
            { tex: '(m_c + 2m_r)\\ddot z = -F\\sin\\theta - \\mu\\dot z,\\quad (m_c + 2m_r)\\ddot h = F\\cos\\theta - (m_c + 2m_r)g,\\quad (J_c + 2m_r d^2)\\ddot\\theta = \\tau' },
            { code: ACC_CODE },
            { html: 'M(q) is constant, so there are no Coriolis terms. These reproduce every expected value in <code>_F_planar_vtol/python/testDynamics.py</code> (<code>workbench/tools/regress_F.py</code>).' },
          ],
        },
        {
          id: 'e', title: '(e) Implement and simulate',
          html: 'Write f(x, u) as in your <code>VTOLDynamics.py</code>: x = (z, h, θ, ż, ḣ, θ̇) is a 6×1 column and u = (f<sub>r</sub>, f<sub>ℓ</sub>) a 2×1 column. <em>Check</em> tests it at random states like <code>testDynamics.py</code>. <em>Simulate my f</em> runs it with RK4 on the same inputs as the VTOL above (true-plant parameters) and draws z, h and θ dashed on the plots.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'f', args: ['state', 'u'], truth: (p, a) => sysF().f([a.z, a.h, a.theta, a.zdot, a.hdot, a.thetadot], [a.fr, a.fl], p).map((v) => [v]) }],
          }, 'def f(state, u):\n    z = state[0][0]\n    h = state[1][0]\n    theta = state[2][0]\n    zdot = state[3][0]\n    hdot = state[4][0]\n    thetadot = state[5][0]\n    fr = u[0][0]\n    fl = u[1][0]\n    zddot = ...\n    hddot = ...\n    thetaddot = ...\n    return np.array([[zdot], [hdot],\n                     [thetadot], [zddot],\n                     [hddot], [thetaddot]])\n'), {
            actions: [{ label: 'Simulate my f', run: (code) => simulateMine(ctx, code) }],
          }),
          solution: () => [
            { code: 'def f(state, u):\n    z = state[0][0]\n    h = state[1][0]\n    theta = state[2][0]\n    zdot = state[3][0]\n    hdot = state[4][0]\n    thetadot = state[5][0]\n    fr = u[0][0]\n    fl = u[1][0]\n    m = P.mc + 2 * P.mr\n    J = P.Jc + 2 * P.mr * P.d**2\n    F = fr + fl\n    zddot = (-F * np.sin(theta) - P.mu * zdot) / m\n    hddot = (-m * P.g + F * np.cos(theta)) / m\n    thetaddot = P.d * (fr - fl) / J\n    return np.array([[zdot], [hdot],\n                     [thetadot], [zddot],\n                     [hddot], [thetaddot]])' },
            { html: 'Appendices P.1–P.3: the dynamics class saturates u, then takes an RK4 step with this f.' },
          ],
        },
      ]);
    },
  }));

  // ----------------------------------------------------------- Chapter 4 --
  // Full 6-state Jacobian about hover, x̃ = (z, h, θ, ż, ḣ, θ̇), ũ = (F̃, τ̃),
  // assembled from the decoupled models (system.js models()).
  function jac6(p) {
    const m = sysF().models(p);
    const A = Array.from({ length: 6 }, () => new Array(6).fill(0));
    const B = Array.from({ length: 6 }, () => [0, 0]);
    const lat = [0, 2, 3, 5], lon = [1, 4];
    m.latSS.A.forEach((r, i) => r.forEach((v, j) => { A[lat[i]][lat[j]] = v; }));
    m.lonSS.A.forEach((r, i) => r.forEach((v, j) => { A[lon[i]][lon[j]] = v; }));
    m.lonSS.B.forEach((r, i) => { B[lon[i]][0] = r[0]; });
    m.latSS.B.forEach((r, i) => { B[lat[i]][1] = r[0]; });
    return { A, B };
  }

  // F.4(c): the VTOL's ḧ with F = F_fl(θ, F̃) must equal hddot_fl(θ, F̃), and
  // hddot_fl must be linear in (θ, F̃).
  async function checkAltFL(ctx, code) {
    if (!code.trim()) return { ok: false, msg: 'Write your code first.' };
    const sets = WB.py.paramSets(ctx, { vary: F.VARY });
    let seed = 99;
    const r = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    const pt = () => [(2 * r() - 1) * 0.6, (2 * r() - 1) * 3];
    const samples = sets.map(() => {
      const pts = Array.from({ length: 6 }, pt), pairs = Array.from({ length: 3 }, () => [pt(), pt()]);
      const lin = pairs.flatMap(([x, y]) => [x, y, x.map((v, i) => v + y[i])]);
      return { pts, lin, calls: [
        ...pts.map((x) => ({ name: 'F_fl', args: x })),
        ...pts.map((x) => ({ name: 'hddot_fl', args: x })),
        ...lin.map((x) => ({ name: 'hddot_fl', args: x })),
        { name: 'hddot_fl', args: [0, 0] },
      ] };
    });
    const out = await WB.py.evaluate(code, sets.map((s, i) => ({ params: s.p, calls: samples[i].calls })));
    if (out.error) return pyError(out);
    const close = (a, b) => Math.abs(a - b) <= 1e-6 * Math.max(1, Math.abs(a), Math.abs(b));
    for (let i = 0; i < sets.length; i++) {
      const c = out.rows[i].calls.map(Number), { pts } = samples[i], n = pts.length;
      if (c.some((v) => !isFinite(v))) return { ok: false, msg: 'F_fl and hddot_fl must each return one number.' };
      const g = (k) => c[2 * n + k];
      const linear = close(c[c.length - 1], 0) && [0, 1, 2].every((k) => close(g(3 * k) + g(3 * k + 1), g(3 * k + 2)));
      if (!linear) return { ok: false, msg: `hddot_fl is not linear in (θ, F̃) with ${sets.text(sets[i])}. F_fl has to cancel every nonlinear term in the altitude equation.` };
      for (let k = 0; k < n; k++) {
        const [th, ft] = pts[k];
        const want = sysF().f([0, 0, th, 0, 0, 0], [c[k] / 2, c[k] / 2], sets[i].p)[4], got = c[n + k];
        if (!close(got, want)) {
          if (i > 0) return { ok: false, msg: `Matches at the nominal parameters but not with ${sets.text(sets[i])}. Write it with ${sets.vary.map((v) => `P.${v}`).join(', ')} rather than numbers.` };
          return { ok: false, msg: `At θ = ${fmt(th, 3)}, F̃ = ${fmt(ft, 3)}: the VTOL with F = F_fl has ḧ = ${fmt(want, 4)}, but hddot_fl gives ${fmt(got, 4)}.` };
        }
      }
    }
    return { ok: true, msg: `Consistent with the VTOL and linear, at ${sets.length} parameter sets.` };
  }

  F.register(Object.assign({}, olBase, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68, F.4 pp. 395–396',
    defaults() { return inDefaults({ inT: { shape: 'pulse', amp: 0.002, width: 0.5 } }); },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    linearSim(ctx, common) { ctx.chapter = this; return F.linearSim(ctx, common); },
    linearLabel: 'Jacobian linearization',
    buildControls(parent, ctx) {
      const sec = section(parent, 'Around hover', 'F.4 p. 396');
      F.forceLawControl(sec, ctx, { eqLabel: 'F = F<sub>e</sub> + F̃ (Jacobian)' });
      progControls(sec, ctx, 'inF', 'Force F̃(t)', 'N', 5);
      progControls(sec, ctx, 'inT', 'Torque τ̃(t)', 'N·m', 0.05);
      sec.append(el('p', { class: 'muted small', text: 'Start at hover, nudge with F̃ and τ̃, and compare with the linearized model (dashed). Set θ(0) in the left panel to see how the linearization degrades with angle. The eigenvalues appear in the s-plane once F.4(b) is solved.' }));
    },
    splane(ctx) {
      if (!showsAnswer(ctx, 'F.4/b')) return { markers: [] };   // eigenvalues answer F.4(b)
      const m = ctx.sys.models(ctx.pModel);
      const mk = [...L.eig(m.lonSS.A).map((q) => ({ ...q, kind: 'ol', label: 'eig A_lon' })), ...L.eig(m.latSS.A).map((q) => ({ ...q, kind: 'cl', label: 'eig A_lat' }))];
      return { markers: mk, fitR: 0.3, legendNames: { ol: 'eig of A_lon', cl: 'eig of A_lat' } };
    },
    math(ctx) {
      const p = ctx.pModel, m = ctx.sys.models(p);
      return [
        { title: 'Equilibria', page: 'p. 60',
          theory: '\\dot x = f(x, u):\\quad (x_e, u_e) \\text{ is an equilibrium when } f(x_e, u_e) = 0' },
        { title: 'Jacobian linearization', page: 'p. 60 · Eq. 4.1',
          theory: '\\tilde x = x - x_e,\\; \\tilde u = u - u_e:\\quad \\dot{\\tilde x} \\approx \\frac{\\partial f}{\\partial x}\\Big|_{(x_e, u_e)}\\tilde x + \\frac{\\partial f}{\\partial u}\\Big|_{(x_e, u_e)}\\tilde u' },
        { title: 'Feedback linearization', page: 'p. 64',
          theory: 'u = u_{fl}(x) + \\tilde u',
          note: 'u_fl cancels the nonlinear terms, so the model from ũ is linear for every x, not only near an equilibrium. It needs an input that enters the nonlinear equation directly.' },
        { title: 'Equations of motion (F.3)', page: 'F.3(d) p. 395', answers: 'F.3/d',
          theory: '(m_c + 2m_r)\\ddot z = -F\\sin\\theta - \\mu\\dot z,\\quad (m_c + 2m_r)\\ddot h = F\\cos\\theta - (m_c + 2m_r)g,\\quad (J_c + 2m_r d^2)\\ddot\\theta = \\tau' },
        { title: 'Equilibria of the VTOL', page: 'F.4(a) p. 396', answers: 'F.4/a',
          theory: 'z_e, h_e \\text{ arbitrary},\\quad \\theta_e = 0,\\quad \\dot z_e = \\dot h_e = \\dot\\theta_e = 0,\\quad F_e = (m_c + 2m_r)g,\\quad \\tau_e = 0',
          numbers: `F_e = ${tex(m.Fe)}\\,\\text{N},\\quad f_{r,e} = f_{\\ell,e} = ${tex(m.Fe / 2)}\\,\\text{N}` },
        { title: 'Jacobian-linearized VTOL', page: 'F.4(b) p. 396', answers: 'F.4/b',
          theory: '\\ddot{\\tilde z} = -g\\tilde\\theta - \\frac{\\mu}{m_c + 2m_r}\\dot{\\tilde z},\\quad \\ddot{\\tilde h} = \\frac{\\tilde F}{m_c + 2m_r},\\quad \\ddot{\\tilde\\theta} = \\frac{\\tilde\\tau}{J_c + 2m_r d^2}',
          numbers: `\\ddot{\\tilde z} = ${tex(-p.g)}\\tilde\\theta ${tex(-m.a)}\\dot{\\tilde z},\\quad \\ddot{\\tilde h} = ${tex(1 / m.M)}\\tilde F,\\quad \\ddot{\\tilde\\theta} = ${tex(1 / m.J)}\\tilde\\tau` },
        { title: 'Feedback linearization of the VTOL', page: 'F.4(c) p. 396', answers: 'F.4/c',
          theory: 'F = \\frac{(m_c + 2m_r)g + \\tilde F}{\\cos\\theta} \\;\\Rightarrow\\; (m_c + 2m_r)\\ddot h = \\tilde F \\;\\text{ exactly } (|\\theta| < 90^\\circ)',
          note: 'The lateral channel stays nonlinear: θ is a state, not an input, so −F sin θ cannot be cancelled by τ.' },
      ];
    },
    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibria',
          html: 'Which states can be equilibria (the rotors can only push, f ≥ 0), and what inputs (F, τ) hold the VTOL there? Write both as functions of the equilibrium position.',
          code: pyPart(ctx, {
            items: [
              { fn: 'x_e', args: ['z_e', 'h_e'], truth: (p, a) => [a.z_e, a.h_e, 0, 0, 0, 0] },
              { fn: 'u_e', args: ['z_e', 'h_e'], truth: (p) => [sysF().equilibriumForce(p), 0] },
            ],
          }, 'def x_e(z_e, h_e):\n    # (z, h, theta, zdot, hdot, thetadot)\n    return ...\n\ndef u_e(z_e, h_e):\n    # (F, tau)\n    return ...\n'),
          solution: () => [
            { tex: 'F\\sin\\theta_e = 0,\\; F\\cos\\theta_e = (m_c + 2m_r)g,\\; \\tau_e = 0 \\;\\Rightarrow\\; \\theta_e = 0,\\; F_e = (m_c + 2m_r)g,\\; z_e, h_e \\text{ arbitrary}' },
            { code: 'def x_e(z_e, h_e):\n    return [z_e, h_e, 0, 0, 0, 0]\n\ndef u_e(z_e, h_e):\n    F_e = (P.mc + 2 * P.mr) * P.g\n    return [F_e, 0]' },
            { html: 'θ<sub>e</sub> = π would need F < 0, which the rotors cannot produce. f<sub>r,e</sub> = f<sub>ℓ,e</sub> = F<sub>e</sub>/2.' },
          ],
        },
        {
          id: 'b', title: '(b) Jacobian linearization',
          html: 'With x̃ = (z̃, h̃, θ̃, z̃̇, h̃̇, θ̃̇)ᵀ and ũ = (F̃, τ̃)ᵀ, give the matrices of x̃̇ = A x̃ + B ũ about the equilibria of (a).',
          code: pyPart(ctx, {
            items: [
              { var: 'A', truth: (p) => jac6(p).A },
              { var: 'B', truth: (p) => jac6(p).B },
            ],
          }, '# x~ = (z, h, theta, zdot, hdot, thetadot)\n# u~ = (F~, tau~)\nA = ...\nB = ...\n'),
          solution: () => [
            { tex: '\\frac{\\partial}{\\partial\\theta}\\Big(\\frac{-F\\sin\\theta}{m_c + 2m_r}\\Big)\\Big|_e = -\\frac{F_e}{m_c + 2m_r} = -g,\\quad \\frac{\\partial}{\\partial F}\\Big(\\frac{F\\cos\\theta}{m_c + 2m_r}\\Big)\\Big|_e = \\frac{1}{m_c + 2m_r}' },
            { tex: '\\ddot{\\tilde z} = -g\\tilde\\theta - \\frac{\\mu}{m_c + 2m_r}\\dot{\\tilde z},\\quad \\ddot{\\tilde h} = \\frac{\\tilde F}{m_c + 2m_r},\\quad \\ddot{\\tilde\\theta} = \\frac{\\tilde\\tau}{J_c + 2m_r d^2}' },
            { code: 'M = P.mc + 2 * P.mr\nJ = P.Jc + 2 * P.mr * P.d**2\nA = np.zeros((6, 6))\nA[0, 3] = A[1, 4] = A[2, 5] = 1\nA[3, 2] = -P.g\nA[3, 3] = -P.mu / M\nB = np.zeros((6, 2))\nB[4, 0] = 1 / M\nB[5, 1] = 1 / J' },
            { html: 'Eq. 4.1 (p. 60). The model splits into an altitude part (h, ḣ; F̃) and a lateral part (z, θ, ż, θ̇; τ̃), as F.5 uses.' },
          ],
        },
        {
          id: 'c', title: '(c) Feedback linearization, if possible',
          html: 'Can a choice F = F<sub>fl</sub>(θ, F̃) make the altitude dynamics linear for every θ? If so, return that F and the resulting ḧ. Any F<sub>fl</sub> that does this is accepted. (Think about the lateral dynamics too: see the solution.)',
          code: { template: 'def F_fl(theta, F_tilde):\n    return ...\n\ndef hddot_fl(theta, F_tilde):\n    # h_ddot when F = F_fl, tau = 0\n    return ...\n', check: (code) => checkAltFL(ctx, code) },
          solution: () => [
            { tex: 'F = \\frac{(m_c + 2m_r)g + \\tilde F}{\\cos\\theta} \\;\\Rightarrow\\; (m_c + 2m_r)\\ddot h = \\tilde F \\quad (|\\theta| < 90^\\circ)' },
            { code: 'def F_fl(theta, F_tilde):\n    m = P.mc + 2 * P.mr\n    return (m * P.g + F_tilde) / np.cos(theta)\n\ndef hddot_fl(theta, F_tilde):\n    return F_tilde / (P.mc + 2 * P.mr)' },
            { html: 'Only the altitude channel can be feedback linearized. In the lateral channel, z̈ = −F sin θ/(m<sub>c</sub> + 2m<sub>r</sub>) − μż/(m<sub>c</sub> + 2m<sub>r</sub>): θ is a state, not an input, so τ cannot cancel the sin θ. Switch the force law above to compare with the Jacobian model.' },
          ],
        },
      ]);
    },
  }));

  // ------------------------------------------------------ Chapters 5 and 6 --
  function tfInputSection(parent, ctx, page, what) {
    const sec = section(parent, 'Inputs around hover', page);
    progControls(sec, ctx, 'inF', 'Force F̃(t)', 'N', 5);
    progControls(sec, ctx, 'inT', 'Torque τ̃(t)', 'N·m', 0.05);
    sec.append(el('p', { class: 'muted small', text: `F = F_e + F̃. Dashed traces: the linear model. Watch what a force pulse does to h, and what a torque pulse does to θ and then to z. ${what} appear in the s-plane once solved.` }));
  }
  const BLOCK_TEX = '\\tilde F \\to \\boxed{\\tfrac{1/(m_c+2m_r)}{s^2}} \\to \\tilde H \\qquad \\tilde\\tau \\to \\boxed{\\tfrac{1/(J_c+2m_rd^2)}{s^2}} \\xrightarrow{\\;\\tilde\\Theta\\;} \\boxed{\\tfrac{-g}{s(s + \\mu/(m_c+2m_r))}} \\to \\tilde Z';
  // Poles of the altitude / lateral models, each shown once `keys` = [lon, lat] is solved.
  const tfSplane = (ctx, [lonKey, latKey]) => {
    const m = ctx.sys.models(ctx.pModel);
    const markers = [];
    if (showsAnswer(ctx, lonKey)) markers.push({ re: 0, im: 0, kind: 'ol', label: 'altitude pole (double at 0)' });
    if (showsAnswer(ctx, latKey)) {
      markers.push({ re: 0, im: 0, kind: 'cl', label: 'Θ/τ̃ pole (double at 0)' });
      markers.push({ re: 0, im: 0, kind: 'obs', label: 'Z/Θ pole at 0' }, { re: -m.a, im: 0, kind: 'obs', label: 'Z/Θ pole at −μ/M' });
    }
    return { markers, fitR: 0.12, legendNames: { ol: 'altitude poles', cl: 'Θ/τ̃ poles', obs: 'Z/Θ poles' } };
  };
  const tfOf = (b0, den) => (p, a) => cx().div(b0(p), cx().poly(den(p), a.s));
  const mdl = (p) => sysF().models(p);
  const P_LON = tfOf((p) => mdl(p).lon.b0, () => [1, 0, 0]);
  const P_TH = tfOf((p) => mdl(p).inner.b0, () => [1, 0, 0]);
  const P_ZTH = tfOf((p) => mdl(p).outer.b0, (p) => [1, mdl(p).outer.a1, 0]);
  const P_Z = (p, a) => cx().mul(P_TH(p, a), P_ZTH(p, a));

  F.register(Object.assign({}, olBase, {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 69–80, F.5 p. 396',
    defaults() { return inDefaults({ inT: { shape: 'doublet', amp: 0.003, width: 0.5 }, inF: { shape: 'pulse', amp: 0.5, width: 1 } }); },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearSim(ctx, common) { ctx.chapter = this; return F.linearSim(ctx, common); },
    linearLabel: 'transfer-function model',
    buildControls(parent, ctx) { tfInputSection(parent, ctx, 'F.5 p. 396', 'The poles'); },
    splane(ctx) { return tfSplane(ctx, ['F.5/b', 'F.5/c']); },
    math(ctx) {
      const p = ctx.pModel, m = ctx.sys.models(p);
      return [
        { title: 'Laplace transform', page: 'p. 69–70',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)' },
        { title: 'Transfer function', page: 'p. 70–71',
          theory: '\\text{zero initial conditions:}\\quad P(s) = \\frac{Y(s)}{U(s)}' },
        { title: 'Cascade', page: 'p. 72–74',
          theory: '\\frac{Y(s)}{U(s)} = \\frac{Y(s)}{X(s)}\\cdot\\frac{X(s)}{U(s)}\\quad (X \\text{ an intermediate signal})' },
        { title: 'Linearized equations (F.4)', page: 'F.4(b) p. 396', answers: 'F.4/b',
          theory: '\\ddot{\\tilde z} = -g\\tilde\\theta - \\frac{\\mu}{m_c + 2m_r}\\dot{\\tilde z},\\quad \\ddot{\\tilde h} = \\frac{\\tilde F}{m_c + 2m_r},\\quad \\ddot{\\tilde\\theta} = \\frac{\\tilde\\tau}{J_c + 2m_r d^2}' },
        { title: 'Longitudinal transfer function', page: 'F.5(b) p. 396', answers: 'F.5/b',
          theory: '(m_c + 2m_r)s^2\\tilde H = \\tilde F \\;\\Rightarrow\\; P_{lon}(s) = \\frac{\\tilde H(s)}{\\tilde F(s)} = \\frac{1/(m_c+2m_r)}{s^2}',
          numbers: `P_{lon}(s) = \\frac{${tex(1 / m.M)}}{s^2}` },
        { title: 'Lateral transfer functions', page: 'F.5(c) p. 396', answers: 'F.5/c',
          theory: '\\frac{\\tilde\\Theta}{\\tilde\\tau} = \\frac{1/(J_c+2m_rd^2)}{s^2},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{-g}{s^2 + \\frac{\\mu}{m_c+2m_r}s},\\quad \\frac{\\tilde Z}{\\tilde\\tau} = \\frac{-g/(J_c+2m_rd^2)}{s^3\\big(s + \\frac{\\mu}{m_c+2m_r}\\big)}',
          numbers: `\\frac{\\tilde\\Theta}{\\tilde\\tau} = \\frac{${tex(1 / m.J)}}{s^2},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{${tex(-p.g)}}{s^2 + ${tex(m.a)}s},\\quad \\frac{\\tilde Z}{\\tilde\\tau} = \\frac{${tex(-p.g / m.J)}}{s^4 + ${tex(m.a)}s^3}` },
        { title: 'Open-loop block diagrams', page: 'F.5(d) p. 396', answers: ['F.5/b', 'F.5/c', 'F.5/d'],
          theory: BLOCK_TEX, note: 'The minus sign: a positive roll tilts the thrust toward −z.' },
      ];
    },
    buildProblem(parent, ctx) {
      const s0 = (code) => ({ items: [code] });
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'a', title: '(a) Laplace transform of the linearized equations',
          html: 'On paper: take the Laplace transform of each linearized equation of motion from F.4(b), with zero initial conditions. Parts (b) and (c) check the transfer functions that follow.',
        },
        {
          id: 'b', title: '(b) Longitudinal: F̃(s) to H̃(s)',
          html: 'Return the transfer function evaluated at a complex s (the check calls it at several).',
          code: pyPart(ctx, s0({ fn: 'P_lon', args: ['s'], truth: P_LON }), 'def P_lon(s):\n    # H~(s)/F~(s)\n    return ...\n'),
          solution: () => [
            { tex: `(m_c + 2m_r)s^2\\tilde H(s) = \\tilde F(s) \\;\\Rightarrow\\; \\frac{\\tilde H}{\\tilde F} = \\frac{1/(m_c+2m_r)}{s^2} = \\frac{${tex(1 / mdl(ctx.pModel).M)}}{s^2}` },
            { code: 'def P_lon(s):\n    return 1 / (P.mc + 2 * P.mr) / s**2' },
          ],
        },
        {
          id: 'c', title: '(c) Lateral: τ̃(s) to Θ̃(s) and Z̃(s); Θ̃(s) to Z̃(s)',
          html: 'Return the three transfer functions at a complex s.',
          code: pyPart(ctx, { items: [
            { fn: 'P_theta', args: ['s'], truth: P_TH },
            { fn: 'P_z', args: ['s'], truth: P_Z },
            { fn: 'P_z_theta', args: ['s'], truth: P_ZTH },
          ] }, 'def P_theta(s):\n    # Theta~(s)/tau~(s)\n    return ...\n\ndef P_z(s):\n    # Z~(s)/tau~(s)\n    return ...\n\ndef P_z_theta(s):\n    # Z~(s)/Theta~(s)\n    return ...\n'),
          solution: () => {
            const p = ctx.pModel, m = mdl(p);
            return [
              { tex: '(J_c + 2m_rd^2)s^2\\tilde\\Theta = \\tilde\\tau,\\quad \\Big(s^2 + \\frac{\\mu}{m_c+2m_r}s\\Big)\\tilde Z = -g\\tilde\\Theta' },
              { tex: `\\frac{\\tilde\\Theta}{\\tilde\\tau} = \\frac{${tex(1 / m.J)}}{s^2},\\quad \\frac{\\tilde Z}{\\tilde\\Theta} = \\frac{${tex(-p.g)}}{s^2 + ${tex(m.a)}s},\\quad \\frac{\\tilde Z}{\\tilde\\tau} = \\frac{${tex(-p.g / m.J)}}{s^3(s + ${tex(m.a)})}` },
              { code: 'M = P.mc + 2 * P.mr\nJ = P.Jc + 2 * P.mr * P.d**2\n\ndef P_theta(s):\n    return (1 / J) / s**2\n\ndef P_z_theta(s):\n    return -P.g / (s**2 + P.mu / M * s)\n\ndef P_z(s):\n    return P_z_theta(s) * P_theta(s)' },
            ];
          },
        },
        {
          id: 'd', title: '(d) Open-loop block diagrams',
          html: 'On paper: draw the longitudinal and the lateral open-loop block diagrams. Click the button when you are done to compare with the live-math card (it unlocks once (b) and (c) are solved too).',
          done: "I've drawn them",
          solution: () => [{ tex: BLOCK_TEX }, { html: 'The minus sign: a positive roll tilts the thrust toward −z.' }],
        },
      ]);
    },
  }));

  const SS_A = (p) => mdl(p).lonSS, SS_B = (p) => mdl(p).latSS;
  F.register(Object.assign({}, olBase, {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space models', pages: 'pp. 81–93, F.6 p. 396',
    defaults() { return inDefaults({ inT: { shape: 'doublet', amp: 0.003, width: 0.5 }, inF: { shape: 'pulse', amp: 0.5, width: 1 } }); },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearSim(ctx, common) { ctx.chapter = this; return F.linearSim(ctx, common); },
    linearLabel: 'ẋ = Ax + Bu',
    buildControls(parent, ctx) { tfInputSection(parent, ctx, 'F.6 p. 396', 'The eigenvalues'); },
    splane(ctx) { return tfSplane(ctx, ['F.6/a', 'F.6/b']); },   // eig(A) answers F.6
    extraPlot(ctx, res) {
      return { opts: { title: 'velocities ż, ḣ', yLabel: 'velocity [m/s]', unit: 'm/s' }, data: { series: [
        { label: 'ḣ', y: res.x.map((x) => x[4]), color: '--series-3', width: 1.5 },
        { label: 'ż', y: res.x.map((x) => x[3]), color: '--series-1' },
      ] } };
    },
    math(ctx) {
      const m = ctx.sys.models(ctx.pModel);
      return [
        { title: 'Jacobian linearization revisited', page: 'p. 83–84',
          theory: 'A = \\frac{\\partial f}{\\partial x}\\Big|_e,\\quad B = \\frac{\\partial f}{\\partial u}\\Big|_e,\\quad C = \\frac{\\partial h}{\\partial x}\\Big|_e,\\quad D = \\frac{\\partial h}{\\partial u}\\Big|_e' },
        { title: 'Back to transfer functions', page: 'p. 85 · Eq. 6.14–6.15',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\text{poles: } \\det(sI - A) = 0' },
        { title: 'Longitudinal state space', page: 'F.6(a) p. 396', answers: 'F.6/a',
          theory: 'A = \\begin{bmatrix}0 & 1\\\\ 0 & 0\\end{bmatrix},\\; B = \\begin{bmatrix}0\\\\ \\frac{1}{m_c+2m_r}\\end{bmatrix},\\; C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\; D = 0',
          numbers: `A = ${texMat(m.lonSS.A)},\\; B = ${texMat(m.lonSS.B)}` },
        { title: 'Lateral state space', page: 'F.6(b) p. 396', answers: 'F.6/b',
          theory: 'A = \\begin{bmatrix}0&0&1&0\\\\0&0&0&1\\\\0&-g&-\\frac{\\mu}{m_c+2m_r}&0\\\\0&0&0&0\\end{bmatrix},\\; B = \\begin{bmatrix}0\\\\0\\\\0\\\\ \\frac{1}{J_c+2m_rd^2}\\end{bmatrix},\\; C = \\begin{bmatrix}1&0&0&0\\\\0&1&0&0\\end{bmatrix},\\; D = 0',
          numbers: `A = ${texMat(m.latSS.A)},\\; B = ${texMat(m.latSS.B)},\\quad \\det(sI - A) = ${WB.tf.polyTex(L.charPoly(m.latSS.A))}` },
      ];
    },
    buildProblem(parent, ctx) {
      const vars = (ss) => ['A', 'B', 'C', 'D'].map((k) => ({ var: k, truth: (p) => ss(p)[k] }));
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [
        {
          id: 'a', title: '(a) Longitudinal A, B, C, D',
          code: pyPart(ctx, { items: vars(SS_A) }, '# x = (h~, hdot~), u = F~, y = h~\nA = ...\nB = ...\nC = ...\nD = ...\n'),
          solution: () => { const m = mdl(ctx.pModel); return [
            { tex: `A = \\begin{bmatrix}0 & 1\\\\ 0 & 0\\end{bmatrix},\\; B = \\begin{bmatrix}0\\\\ \\frac{1}{m_c+2m_r}\\end{bmatrix} = ${texMat(m.lonSS.B)},\\; C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\; D = 0` },
            { code: 'A = np.array([[0, 1],\n              [0, 0]])\nB = np.array([[0], [1 / (P.mc + 2 * P.mr)]])\nC = np.array([[1, 0]])\nD = np.array([[0]])' },
            { html: 'From ḧ̃ = F̃/(m<sub>c</sub> + 2m<sub>r</sub>) (F.4(b)); p. 83–84.' },
          ]; },
        },
        {
          id: 'b', title: '(b) Lateral A, B, C, D',
          code: pyPart(ctx, { items: vars(SS_B) }, '# x = (z~, theta~, zdot~, thetadot~)\n# u = tau~, y = (z~, theta~)\nA = ...\nB = ...\nC = ...\nD = ...\n'),
          solution: () => { const m = mdl(ctx.pModel); return [
            { tex: `A = ${texMat(m.latSS.A)},\\; B = ${texMat(m.latSS.B)},\\; C = \\begin{bmatrix}1&0&0&0\\\\0&1&0&0\\end{bmatrix},\\; D = \\begin{bmatrix}0\\\\0\\end{bmatrix}` },
            { code: 'M = P.mc + 2 * P.mr\nJ = P.Jc + 2 * P.mr * P.d**2\nA = np.array([[0, 0, 1, 0],\n              [0, 0, 0, 1],\n              [0, -P.g, -P.mu / M, 0],\n              [0, 0, 0, 0]])\nB = np.array([[0], [0], [0], [1 / J]])\nC = np.array([[1, 0, 0, 0],\n              [0, 1, 0, 0]])\nD = np.array([[0], [0]])' },
            { html: 'y has two entries, so D is 2×1.' },
          ]; },
        },
      ]);
    },
  }));
})();
