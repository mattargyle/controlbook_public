// D.2-D.6: modeling the mass-spring-damper. Open-loop experiments: kinetic energy
// with prescribed motion (D.2), Euler-Lagrange equations with an energy-balance
// check (D.3), equilibria and linearization (D.4), the transfer function (D.5) and
// the state-space model (D.6).
//
// Work mode: every derivation is a Python answer (WB.py.check), checked at random
// arguments and random parameter sets, so the student writes the whole expression.
// `theory` lines are general book equations; cards with this study's results carry
// `answers` and stay locked until that part is solved.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const lib = WB.studies.D.lib;
  const ans = WB.systems.D.answers;
  const CH = WB.studies.D.chapters;
  const inputForce = (st, t) => WB.models.inputTorque(st, t);  // same shapes as Study A
  const shows = lib.shows;

  // F = compensation + F_in(t). comp: 'none' | 'eq' (F_e at z_e) | 'fl' (feedback linearization)
  function openLoop(ctx, { linear = false } = {}) {
    const st = ctx.st, p = ctx.pModel;
    return {
      update(r, x, yMeas, t) {
        const fin = inputForce(st, t);
        if (linear || !st.comp || st.comp === 'none') return fin;
        // D.4 in Work mode applies the student's own F_e (st.FeW), so the force
        // readout never shows the answer; the feedback-linearizing force is applied
        // only once D.4(c) is solved.
        const work = ctx.S.mode === 'work' && st.FeW !== undefined;
        if (st.comp === 'eq') return (work ? st.FeW : p.k * (st.zE || 0)) + fin;
        return (flApplied(ctx) ? p.k * x[0] : 0) + fin;
      },
    };
  }
  const flApplied = (ctx) => shows(ctx, 'D.4/c');

  function inputControls(parent, ctx, { comps, label = 'Input force F<sub>in</sub>(t)' } = {}) {
    const names = { zero: 'none', const: 'constant', pulse: 'pulse', square: 'square', sine: 'sine' };
    segmented(parent, {
      label, options: Object.keys(names).map((s) => ({ value: s, label: names[s] })),
      ...bind(ctx, 'shape', () => ctx.st.inp),
    });
    slider(parent, { label: 'amplitude', unit: 'N', min: -6, max: 6, step: 0.01, sig: 3, ...bind(ctx, 'amp', () => ctx.st.inp), disabled: () => ctx.st.inp.shape === 'zero' });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.005, max: 1, step: 0.005, sig: 3, ...bind(ctx, 'freq', () => ctx.st.inp), disabled: () => !['square', 'sine'].includes(ctx.st.inp.shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.1, max: 10, step: 0.1, sig: 3, ...bind(ctx, 'width', () => ctx.st.inp), disabled: () => ctx.st.inp.shape !== 'pulse' });
    if (comps) {
      segmented(parent, { label: 'Added to F<sub>in</sub>', options: comps, ...bind(ctx, 'comp') });
    }
  }

  const common = {
    openLoop: true, metrics: false,
    controller(ctx, o) { return openLoop(ctx, o); },
  };
  // Eigenvalue markers. In Work mode they answer `key` (and D.7(a)), so they stay
  // off the s-plane until that part is solved.
  function eigMarkers(ctx, A, label, key) {
    if (!shows(ctx, key)) return { markers: [] };
    return { markers: L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `${label} ${i + 1}` })) };
  }
  const zSeries = (res) => res.x.map((x) => x[1]);

  // "Plot my answer" overlays (js/core/yours.js).
  const Y = WB.yours;
  const fin = (ctx) => (t) => inputForce(ctx.st, t);

  // ------------------------------------------------- Python answer parts --
  // Arguments for D's Python answers (WB.py.check draws them at random).
  const ARGS = {
    z: { label: 'z', lo: -2, hi: 2 },
    zdot: { label: 'ż', lo: -2, hi: 2 },
    F: { label: 'F', lo: -6, hi: 6 },
    state: { col: ['z', 'zdot'] },
    z_e: { label: 'zₑ', lo: -1, hi: 2 },
    s: { label: 's', complex: true, re: [-3, 2], im: [0.2, 6] },
    Z: { label: 'Z', complex: true, re: [-2, 2], im: [-2, 2] },
  };
  const pyPart = (ctx, spec, template) => lib.pyPart(ctx, { args: ARGS, ...spec }, template);
  const zddotOf = (ctx, p, z, zd, F) => ctx.sys.f([z, zd], F, p)[1];

  // ------------------------------------------------------------------ D.2 --
  // The student's kinetic(z, zdot) along the prescribed motion.
  async function runKinetic(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: res.x.map(([z, zd]) => ({ name: 'kinetic', args: [z, zd] })) }]);
    if (out.error) return lib.pyError(out);
    const K = out.rows[0].calls;
    if (!K.every((v) => typeof v === 'number')) return { ok: false, msg: 'kinetic should return one number.' };
    return { n: K.length, K };
  }

  // Prescribed motion z(t) = A sin(2πft), as in hw02; nothing is simulated.
  CH.ch2 = Object.assign({}, common, {
    id: 'ch2', num: 2, tab: 'Ch 2', title: 'Kinetic energy', pages: 'pp. 19–40, p. 378',
    linear: false,
    defaults() { return { A: 0.5, f: 0.2, inp: { shape: 'zero', amp: 0, freq: 0.1, width: 1 } }; },
    simDefaults(sys) { return sys.problems.ch2.sim; },

    simulate(ctx) {
      const { pModel, st, S } = ctx;
      const Ts = S.sim.Ts, N = Math.round(S.sim.tEnd / Ts) + 1;
      const w = 2 * Math.PI * st.f;
      const res = { t: new Float64Array(N), r: new Float64Array(N), y: new Float64Array(N), uDemand: new Float64Array(N), u: new Float64Array(N), uApplied: new Float64Array(N), x: [], extras: { K: new Float64Array(N) } };
      for (let k = 0; k < N; k++) {
        const t = k * Ts, z = st.A * Math.sin(w * t), zd = st.A * w * Math.cos(w * t);
        res.t[k] = t; res.r[k] = z; res.y[k] = z;
        res.x.push([z, zd]);
        res.extras.K[k] = ctx.sys.kinetic([z, zd], pModel);
      }
      return res;
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Prescribed motion z(t) = A sin(2πft)', 'p. 378 · D.2(b)');
      slider(sec, { label: 'A', unit: 'm', min: 0, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'A') });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'f') });
      lib.note(sec, 'No dynamics here: the motion is imposed, as in a hw02 animation script. Once (a) is solved, or you click Plot my K, the plot below shows the kinetic energy along that motion.');
    },

    // K(t) shows the answer to D.2(a), so in Work mode the plot is empty until (a) is
    // solved, apart from the student's own K once they plot it.
    extraPlot(ctx, res) {
      const show = shows(ctx, 'D.2/a'), y = Y.data(ctx, 'ch2.a');
      const series = show ? [{ label: 'K', y: Array.from(res.extras.K), color: '--series-1' }] : [];
      if (y) series.push(Y.series('your K (Python)', y.K));
      return { opts: { title: 'kinetic energy K(t)', yLabel: 'K [J]', unit: 'J' }, data: { series } };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Kinetic energy of the mass', page: 'p. 377 · Fig. 19-1', answers: 'D.2/a',
          theory: '\\mathbf p_{cm} = (z, 0, 0)^\\top,\\quad \\boldsymbol\\omega = 0 \\;\\Rightarrow\\; K = \\tfrac12 m\\dot z^2',
          numbers: `K = ${tex(p.m / 2)}\\,\\dot z^2`,
          note: 'The block slides without rotating, so only the translational term remains.' },
      ];
    },

    buildProblem(parent, ctx) {
      lib.panel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) Kinetic energy',
          html: 'Write K as a function of z and ż.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'kinetic', args: ['z', 'zdot'], truth: (p, a) => 0.5 * p.m * a.zdot ** 2 }],
          }, 'def kinetic(z, zdot):\n    # kinetic energy K of the system\n    K = ...\n    return K\n'), {
            actions: [{ label: 'Plot my K', run: (code) => Y.plot(ctx, 'ch2.a', code, runKinetic, () => 'Your K(t) along the motion set on the right is the dotted trace on the lower plot.') }],
          }),
          solution: () => [
            { tex: '\\mathbf v = (\\dot z, 0, 0)^\\top,\\;\\boldsymbol\\omega = 0 \\Rightarrow K = \\tfrac12 m\\dot z^2' },
            { code: 'def kinetic(z, zdot):\n    return 0.5 * P.m * zdot**2' },
            { html: 'Same form as the masses on p. 26 (Eq. 2.3 with ω = 0). The plot peaks at ½mA²(2πf)² when the mass passes z = 0.' },
          ],
        },
        {
          id: 'b', title: '(b) Animate the system',
          html: 'You write this in your own animation code. The animation at the top of this page shows the same kind of variable input z(t) = A sin(2πft), set in the controls on the right.',
        },
      ]);
    },
  });

  // ------------------------------------------------------------------ D.3 --
  // Last "Simulate my f" run, shown while the simulation it was run against is unchanged.
  const mine = {};
  const simSig = (ctx, res) => JSON.stringify([ctx.S.sysId, ctx.S.chapter, ctx.pTrue, ctx.S.sim.Ts, res.t.length, res.x[0], Array.from(res.uApplied).reduce((a, v, i) => a + v * (i + 1), 0)]);
  async function simulateMine(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.simulate(code, { fn: 'f', params: ctx.pTrue, x0: res.x[0], u: Array.from(res.uApplied), Ts: ctx.S.sim.Ts });
    if (out.error) return lib.pyError(out);
    mine.ch3 = { sig: simSig(ctx, res), z: out.x.map((x) => x[0]) };
    let dev = 0;
    out.x.forEach((x, k) => { dev = Math.max(dev, Math.abs(x[0] - res.x[k][0])); });
    ctx.update();
    const msg = `Your f is the dashed trace on the z plot. Largest difference from the workbench's mass: ${fmt(1000 * dev, 3)} mm.`;
    return { ok: dev < 1e-4, msg: dev < 1e-4 ? msg : `${msg} They should overlap.` };
  }

  // D.3(a): the student's potential(z) along the simulated motion.
  async function runPotential(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: res.x.map(([z]) => ({ name: 'potential', args: [z] })) }]);
    if (out.error) return lib.pyError(out);
    const P = out.rows[0].calls;
    if (!P.every((v) => typeof v === 'number' && Number.isFinite(v))) return { ok: false, msg: 'potential should return one number.' };
    return { n: P.length, P };
  }

  // D.3(c): the student's τ and −Bq̇ along the simulated motion, integrated into the
  // work they do with the same trapezoid rule as the workbench's energy balance.
  async function runForces(ctx, code) {
    const res = ctx.app.result(), n = res.t.length, Ts = ctx.S.sim.Ts;
    const pts = [];
    for (let k = 1; k < n; k++) pts.push([...res.x[k - 1], res.uApplied[k - 1]], [...res.x[k], res.uApplied[k - 1]]);
    const calls = ['generalized_force', 'damping_force'].flatMap((name) => pts.map((args) => ({ name, args })));
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls }], 15000);
    if (out.error) return lib.pyError(out);
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

  CH.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 41–56, p. 378',
    linear: false,
    defaults() { return { comp: 'none', inp: { shape: 'square', amp: 1, freq: 0.05, width: 1 } }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 378 · D.3(e)');
      inputControls(sec, ctx);
      lib.note(sec, ctx.S.mode === 'work'
        ? 'The lower plot is an energy balance that checks the equations of motion: the change in K + P must equal the net work done by the nonconservative forces. It is empty until you click Plot my P or Plot my forces (your curves) or solve (a) or (c) (the workbench\'s E(t) − E(0) and work).'
        : 'The energy plot checks the equations of motion: the change in K + P must equal the net work done by the nonconservative forces.');
    },

    // E(t) − E(0) uses the workbench's P (D.3(a)) and the work curve its forces
    // (D.3(c)), so Work mode draws each once its part is solved (P alone for the
    // energy, so the chapter stands on its own). The student's own curves show
    // whenever plotted; with neither, the plot is empty.
    extraPlot(ctx, res) {
      const { sys, pModel } = ctx;
      const showE = shows(ctx, 'D.3/a'), showW = shows(ctx, 'D.3/c');
      const n = res.t.length, Ts = ctx.S.sim.Ts;
      const E = new Float64Array(n), Wk = new Float64Array(n);
      const E0 = sys.kinetic(res.x[0], pModel) + sys.potential(res.x[0], pModel);
      let w = 0;
      // power into the mass from the force the model assumes (nominal b), trapezoid rule
      const pw = (xx, u) => (u - pModel.b * xx[1]) * xx[1];
      for (let k = 0; k < n; k++) {
        E[k] = sys.kinetic(res.x[k], pModel) + sys.potential(res.x[k], pModel) - E0;
        if (k > 0) w += 0.5 * Ts * (pw(res.x[k - 1], res.uApplied[k - 1]) + pw(res.x[k], res.uApplied[k - 1]));
        Wk[k] = w;
      }
      const series = [
        ...(showW ? [{ label: '∫(F − bż)ż dt', y: Wk, color: '--series-2', dash: [5, 4], width: 2 }] : []),
        ...(showE ? [{ label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' }] : []),
      ];
      // Your P (any P₀) with the workbench's K, and the work of your forces.
      const yp = Y.data(ctx, 'ch3.a'), yf = Y.data(ctx, 'ch3.c');
      if (yp) {
        const K0 = sys.kinetic(res.x[0], pModel);
        series.push(Y.series('E(t) − E(0) with your P', res.x.map((x, k) => sys.kinetic(x, pModel) - K0 + yp.P[k] - yp.P[0])));
      }
      if (yf) series.push({ ...Y.series('work done by your forces', yf.W), dash: [8, 3] });
      return { opts: { title: 'energy balance', yLabel: 'energy [J]', unit: 'J' }, data: { series } };
    },

    // D.3(e): the student's f, simulated with the same input as the mass above.
    outputSeries(ctx, res) {
      const m = mine.ch3;
      if (!m || m.sig !== simSig(ctx, res)) return [];
      return [{ label: 'your f (Python)', y: m.z, color: '--series-2', dash: [5, 4], width: 2 }];
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 43 · §3.1.4',
          theory: 'L(q, \\dot q) = K(q, \\dot q) - P(q),\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy', page: 'p. 41–42 · §3.1.1',
          theory: '\\text{gravity: } P = mgy + P_0,\\quad \\text{spring: } P = \\tfrac12 k\\,\\Delta\\ell^2 \\;(\\Delta\\ell = \\text{stretch from the rest length})' },
        { title: 'Generalized coordinates, forces, damping', page: 'p. 42–43 · §3.1.2–3.1.3',
          theory: 'q = \\text{minimum set of configuration variables},\\quad \\tau = \\text{applied (nonconservative) forces along } q,\\quad -B\\dot q = \\text{damping forces}' },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from p. 43',
          theory: '\\frac{d}{dt}(K + P) = \\dot q^\\top\\big(\\tau - B\\dot q\\big)',
          note: ctx.S.mode === 'work' ? 'Once (a) and (c) are solved, the plot below integrates the right side and compares it with the energy of the mass.' : 'The plot below integrates the right side and compares it with the energy of the mass.' },
        { title: 'Kinetic energy of the mass (D.2)', page: 'p. 377 · Fig. 19-1', answers: 'D.2/a',
          theory: 'K = \\tfrac12 m\\dot z^2',
          numbers: `K = ${tex(p.m / 2)}\\,\\dot z^2` },
        { title: 'Potential energy of the mass-spring-damper', page: 'p. 42 · Fig. 3-2', answers: 'D.3/a',
          theory: 'P = \\tfrac12 k z^2 \\quad(z = 0 \\text{ where the spring is not stretched})',
          numbers: `P = ${tex(p.k / 2)}\\,z^2` },
        { title: 'Coordinates and forces of the mass-spring-damper', page: 'p. 42–43', answers: ['D.3/b', 'D.3/c'],
          theory: 'q = z,\\quad \\tau = F,\\quad -B\\dot q = -b\\dot z' },
        { title: 'Equation of motion of the mass-spring-damper', page: 'p. 43', answers: 'D.3/d',
          theory: 'L = \\tfrac12 m\\dot z^2 - \\tfrac12 k z^2 \\;\\Rightarrow\\; m\\ddot z + k z = F - b\\dot z',
          numbers: `\\ddot z = ${tex(1 / p.m)}\\,F - ${tex(p.b / p.m)}\\,\\dot z - ${tex(p.k / p.m)}\\,z` },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch3;
      const q = (v) => String(v || '').toLowerCase().replace(/\s+/g, '').replace(/^q(1)?=/, '').replace(/[()[\]{}]|\^t|ᵀ|'/g, '');
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Potential energy',
          html: 'Write P as a function of z. Any constant P<sub>0</sub> is accepted.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'potential', args: ['z'], compare: 'offset', truth: (p, a) => 0.5 * p.k * a.z ** 2 }],
          }, 'def potential(z):\n    # potential energy P of the system\n    return ...\n'), {
            actions: [{ label: 'Plot my P', run: (code) => Y.plot(ctx, 'ch3.a', code, runPotential, () => 'Dotted on the energy plot: E(t) − E(0) along the simulated motion with your P in place of the workbench\'s. With the right P it balances the work of the nonconservative forces.') }],
          }),
          solution: () => [
            { tex: 'P = \\tfrac12 k z^2 \\quad(\\text{the spring is stretched by } z;\\; \\text{the mass moves on a level surface, so gravity does no work})' },
            { code: 'def potential(z):\n    return 0.5 * P.k * z**2' },
            { html: 'Book: the spring of Fig. 3-2 (p. 42).' },
          ],
        },
        {
          id: 'b', title: '(b) Generalized coordinates',
          inputs: { q: 'q =' },
          html: 'Name them, comma separated (e.g. <code>theta</code> or <code>θ</code>).',
          check: (v) => {
            const g = q(v.q);
            if (!g) return { ok: false, msg: 'Enter the generalized coordinates.' };
            if (/dot|̇|ż|v$|velocity/.test(g)) return { ok: false, msg: 'Generalized coordinates are configuration variables (positions and angles), not velocities.' };
            return lib.passed(ctx, ['z', 'q1'].includes(g) ? { ok: true, msg: 'One coordinate, so q is a scalar here.' } : { ok: false, msg: 'Which variables fix the configuration of the system? Use the minimum number.' });
          },
          solution: () => [{ tex: 'q = z' }, { html: 'Book: p. 42 (§3.1.2). One variable places the mass.' }],
        },
        {
          id: 'c', title: '(c) Generalized forces and damping forces',
          html: 'Write the damping term the way the book does, as the force −Bq̇ (p. 43).',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'generalized_force', args: ['z', 'zdot', 'F'], truth: (p, a) => a.F },
              { fn: 'damping_force', args: ['z', 'zdot', 'F'], truth: (p, a) => -p.b * a.zdot },
            ],
            explain: (it, f) => {
              const [g, w] = [f.e.got, f.e.want].map(Number);
              return it.fn === 'damping_force' && Math.abs(g + w) < 1e-6 * Math.max(1, Math.abs(w)) ? 'Check the sign: the book writes the damping force as −Bq̇.' : '';
            },
          }, 'def generalized_force(z, zdot, F):\n    # applied force along q\n    return ...\n\ndef damping_force(z, zdot, F):\n    # damping force, -B*q_dot\n    return ...\n'), {
            actions: [{ label: 'Plot my forces', run: (code) => Y.plot(ctx, 'ch3.c', code, runForces, () => 'Dotted on the energy plot: the work your generalized force and damping force do along the simulated motion. With the right forces it balances E(t) − E(0).') }],
          }),
          solution: () => [
            { tex: '\\tau = F,\\quad -B\\dot q = -b\\dot z' },
            { code: 'def generalized_force(z, zdot, F):\n    return F\n\ndef damping_force(z, zdot, F):\n    return -P.b * zdot' },
            { html: 'F is the only applied force; the damper is the only damping force. The spring is conservative, so it is in P, not τ. Book: p. 43.' },
          ],
        },
        {
          id: 'd', title: '(d) Equations of motion',
          html: 'Apply the Euler-Lagrange equation, then solve for z̈.',
          code: pyPart(ctx, {
            cases: [
              { label: 'with ż = 0 and F = 0 (only the spring acts)', fix: { zdot: 0, F: 0 } },
              { label: 'with z = 0 and ż = 0 (only the force acts)', fix: { z: 0, zdot: 0 } },
              { label: 'with z = 0 and F = 0 (only damping acts)', fix: { z: 0, F: 0 } },
              { label: '' },
            ],
            items: [{ fn: 'zddot', args: ['z', 'zdot', 'F'], truth: (p, a) => zddotOf(ctx, p, a.z, a.zdot, a.F) }],
          }, 'def zddot(z, zdot, F):\n    # from the equation of motion\n    return ...\n'),
          solution: () => [
            { tex: 'L = \\tfrac12 m\\dot z^2 - \\tfrac12 kz^2,\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot z} = m\\ddot z,\\quad \\frac{\\partial L}{\\partial z} = -kz' },
            { tex: 'm\\ddot z + kz = F - b\\dot z' },
            { code: 'def zddot(z, zdot, F):\n    return (F - P.b * zdot - P.k * z) / P.m' },
          ],
        },
        {
          id: 'e', title: '(e) Implement and simulate',
          html: 'Write f(x, u): the state is a 2×1 column. <em>Check</em> tests it at random states. <em>Simulate my f</em> runs it with RK4 on the same force input as the mass above (true-plant parameters) and draws z dashed on the plot. The output connects to the D.2 animation at the top of the page.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'f', args: ['state', 'F'], truth: (p, a) => [[a.zdot], [zddotOf(ctx, p, a.z, a.zdot, a.F)]] }],
          }, 'def f(state, F):\n    z = state[0][0]\n    zdot = state[1][0]\n    zddot = ...\n    return np.array([[zdot], [zddot]])\n'), {
            actions: [{ label: 'Simulate my f', run: (code) => simulateMine(ctx, code) }],
          }),
          solution: () => [
            { code: 'def f(state, F):\n    z = state[0][0]\n    zdot = state[1][0]\n    zddot = (F - P.b * zdot - P.k * z) / P.m\n    return np.array([[zdot], [zddot]])' },
            { html: 'Book: Appendix P.1–P.3 (the Listing 3.2 pattern, p. 45).' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------------ D.4 --
  // D.4(c): the mass with F = F_fl + F~ must have the student's zddot_fl, and
  // zddot_fl must be linear in (z, ż, F~).
  async function checkFeedbackLin(ctx, code) {
    if (!code.trim()) return { ok: false, msg: 'Write your code first.' };
    const sets = WB.py.paramSets(ctx);
    let seed = 99;
    const r = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    const pt = () => [(2 * r() - 1) * 2, (2 * r() - 1) * 2, (2 * r() - 1) * 6];
    const samples = sets.map(() => {
      const pts = Array.from({ length: 6 }, pt), pairs = Array.from({ length: 3 }, () => [pt(), pt()]);
      const lin = pairs.flatMap(([x, y]) => [x, y, x.map((v, i) => v + y[i])]);
      return { pts, lin, calls: [
        ...pts.map(([z, zd]) => ({ name: 'F_fl', args: [z, zd] })),
        ...pts.map((x) => ({ name: 'zddot_fl', args: x })),
        ...lin.map((x) => ({ name: 'zddot_fl', args: x })),
        { name: 'zddot_fl', args: [0, 0, 0] },
      ] };
    });
    const out = await WB.py.evaluate(code, sets.map((s, i) => ({ params: s.p, calls: samples[i].calls })));
    if (out.error) return lib.pyError(out);
    const close = (a, b) => Math.abs(a - b) <= 1e-6 * Math.max(1, Math.abs(a), Math.abs(b));
    for (let i = 0; i < sets.length; i++) {
      const c = out.rows[i].calls.map(Number), { pts } = samples[i], n = pts.length;
      const g = (k) => c[2 * n + k];
      const linear = close(c[c.length - 1], 0) && [0, 1, 2].every((k) => close(g(3 * k) + g(3 * k + 1), g(3 * k + 2)));
      if (!linear) return { ok: false, msg: `zddot_fl is not linear in (z, ż, F̃) with ${sets.text(sets[i])}. F_fl has to cancel every nonlinear term.` };
      for (let k = 0; k < n; k++) {
        const [z, zd, ft] = pts[k];
        const want = zddotOf(ctx, sets[i].p, z, zd, c[k] + ft), got = c[n + k];
        if (!close(got, want)) {
          if (i > 0) return { ok: false, msg: `Matches at the nominal parameters but not with ${sets.text(sets[i])}. Write it with ${sets.vary.map((v) => `P.${v}`).join(', ')} rather than numbers.` };
          return { ok: false, msg: `At z = ${fmt(z, 3)}, ż = ${fmt(zd, 3)}, F̃ = ${fmt(ft, 3)}: the mass with F = F_fl + F̃ has z̈ = ${fmt(want, 4)}, but zddot_fl gives ${fmt(got, 4)}.` };
        }
      }
    }
    return { ok: true, msg: `Consistent with the mass and linear, at ${sets.length} parameter sets.` };
  }

  // D.4(a): F_e(z_e) at the current z_e becomes the "your F_e" force.
  async function useMyFe(ctx, code) {
    const zE = ctx.st.zE;
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: [{ name: 'F_e', args: [zE] }] }]);
    if (out.error) return lib.pyError(out);
    const Fe = out.rows[0].calls[0];
    if (typeof Fe !== 'number' || !Number.isFinite(Fe)) return { ok: false, msg: 'F_e should return one number.' };
    Object.assign(ctx.st, { FeW: Fe, method: 'jacobian' });
    ctx.update();
    const head = `Your F_e(${fmt(zE, 3)} m) = ${fmt(Fe, 4)} N.`;
    if (ctx.S.mode !== 'work') return { info: true, msg: `${head} Explore mode applies the workbench's F_e; switch to Work mode to apply yours.` };
    return { info: true, msg: `${head} It is now "your F_e" on the right: with the right F_e the mass settles at zₑ. Click again after moving zₑ.` };
  }

  // D.4(b): the student's A_jac, B_jac at the current z_e.
  async function runJacobian(ctx, code) {
    const zE = ctx.st.zE;
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: [{ name: 'A_jac', args: [zE] }, { name: 'B_jac', args: [zE] }] }]);
    if (out.error) return lib.pyError(out);
    const A = Y.asMat(out.rows[0].calls[0], 2, 2), B = Y.asMat(out.rows[0].calls[1], 2, 1);
    if (!A) return { ok: false, msg: 'A_jac should return a 2×2 array of numbers.' };
    if (!B) return { ok: false, msg: 'B_jac should return a 2×1 array of numbers.' };
    return { A, B };
  }

  // D.4(c): the true mass with the student's F = F_fl(z, ż) + F̃ (simulated in
  // Python like the workbench's own run: same input, disturbance and saturation),
  // the student's zddot_fl model driven by F̃, and that model's A.
  const PLANT_PY = 'def f(state, F):\n    z = state[0][0]\n    zdot = state[1][0]\n    return np.array([[zdot], [(F - P.b * zdot - P.k * z) / P.m]])\n';
  const FL_MODEL_PY = '\n\ndef _wb_fl_model(state, F_tilde):\n    return np.array([[state[1][0]], [zddot_fl(state[0][0], state[1][0], F_tilde)]])\n';
  async function runFeedbackLin(ctx, code) {
    const res = ctx.app.result(), Ts = ctx.S.sim.Ts;
    const fin = Array.from(res.t, (t) => inputForce(ctx.st, t));
    const d = Array.from(res.uApplied, (v, k) => v - res.u[k]);
    const mass = await WB.py.simulate(code, { plant: PLANT_PY, plantParams: ctx.pTrue, ctrl: 'F_fl', params: ctx.pModel, x0: res.x[0], u: fin, d, uLimit: ctx.sys.uLimit(ctx.pTrue), Ts });
    if (mass.error) return lib.pyError(mass);
    const model = await WB.py.simulate(code + FL_MODEL_PY, { fn: '_wb_fl_model', params: ctx.pModel, x0: res.x[0], u: fin, Ts });
    if (model.error) return lib.pyError(model);
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: [[0, 0, 0], [1, 0, 0], [0, 1, 0]].map((args) => ({ name: 'zddot_fl', args })) }]);
    if (out.error) return lib.pyError(out);
    const [c0, cz, czd] = out.rows[0].calls.map(Number);
    return { n: res.t.length, z: mass.x.map((x) => x[0]), zm: model.x.map((x) => x[0]), A: [[0, 1], [cz - c0, czd - c0]] };
  }

  CH.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria & linearization', pages: 'pp. 59–68, p. 378',
    defaults() { return { zE: 0.5, dz0: 0.3, method: 'jacobian', comp: 'eq', FeW: 0, inp: { shape: 'zero', amp: 0.5, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    controller(ctx, o) { ctx.st.comp = ctx.st.method === 'fl' ? 'fl' : 'eq'; return openLoop(ctx, o); },

    // Start at z_e + δz(0) regardless of the left panel, and hold z_r = z_e for the plot.
    simulate(ctx, c, plant) {
      return WB.sim.simulate({ ...c, x0: [ctx.st.zE + ctx.st.dz0, 0], plant, controller: this.controller(ctx), reference: () => ctx.st.zE });
    },
    // Linearized model in deviation coordinates, shifted back to absolute z for the plot.
    // In Work mode before D.4(c) the feedback option applies no compensation, so the
    // reference model is then the plant itself.
    linearSim(ctx, c) {
      const p = ctx.pModel, t = ans.tf(p);
      const fl = ctx.st.method === 'fl';
      const A = fl && flApplied(ctx) ? [[0, 1], [0, -t.a1]] : ans.ss(p).A, B = [[0], [t.b0]];
      const off = fl ? 0 : ctx.st.zE;  // the FL model is in absolute z
      return WB.sim.simulate({
        ...c, disturbance: null, noise: null, reference: () => ctx.st.zE,
        x0: [ctx.st.zE + ctx.st.dz0 - off, 0],
        plant: { f: (x, u) => A.map((row, i) => row[0] * x[0] + row[1] * x[1] + B[i][0] * u), h: (x) => x[0] + off, uLimit: Infinity },
        controller: { update: (r, x, y, tt) => inputForce(ctx.st, tt) },
      });
    },
    linearLabel: 'linearized model',

    buildControls(parent, ctx) {
      const sec = section(parent, 'Operating point', 'p. 59–60');
      slider(sec, { label: 'z<sub>e</sub>', unit: 'm', min: -1, max: 2, step: 0.01, sig: 3, ...bind(ctx, 'zE') });
      slider(sec, { label: 'δz(0)', unit: 'm', min: -1, max: 1, step: 0.01, sig: 3, hint: 'initial offset from zₑ', ...bind(ctx, 'dz0') });
      segmented(sec, {
        label: 'Linearization',
        options: [{ value: 'jacobian', label: 'Jacobian: F = F<sub>e</sub> + F̃' }, { value: 'fl', label: 'feedback: F = F<sub>fl</sub>(x) + F̃' }],
        ...bind(ctx, 'method'),
      });
      if (ctx.S.mode === 'work') {
        slider(sec, { label: 'your F<sub>e</sub>', unit: 'N', min: -10, max: 10, step: 0.01, sig: 3, ...bind(ctx, 'FeW'), disabled: () => ctx.st.method !== 'jacobian' });
        lib.note(sec, 'Jacobian: Work mode applies your F_e. Test an answer: with the right F_e the mass settles at zₑ. Feedback: the feedback-linearizing force is applied once you solve (c); until then F = F̃.');
      } else {
        lib.note(sec, 'Explore applies the correct equilibrium or feedback-linearizing force.');
      }
      const inp = section(parent, 'Input F̃(t)', 'p. 60');
      inputControls(inp, ctx, { label: 'Input F̃(t)' });
    },

    outputSeries(ctx, res) {
      const out = [{ label: 'zₑ', y: Array.from(res.t, () => ctx.st.zE), color: '--ref', dash: [6, 4], width: 1.5 }];
      if (ctx.st.method === 'fl') {
        const y = Y.data(ctx, 'ch4.c');
        if (y) out.push(Y.series('mass with your F_fl', y.z), { ...Y.series('your zddot_fl model', y.zm), dash: [8, 3] });
      } else {
        const y = Y.data(ctx, 'ch4.b');
        if (y) out.push(Y.series('your A_jac, B_jac', Y.linResponse(ctx, { ...y, C: [[1, 0]], D: [[0]] }, [ctx.st.dz0, 0], fin(ctx)).y[0].map((v) => v + ctx.st.zE)));
      }
      return out;
    },

    splane(ctx) {
      const t = ans.tf(ctx.pModel), fl = ctx.st.method === 'fl';
      const A = fl ? [[0, 1], [0, -t.a1]] : ans.ss(ctx.pModel).A;
      const sp = eigMarkers(ctx, A, `eigenvalue of A (${ctx.st.method})`, fl ? 'D.4/c' : 'D.4/b');
      const y = Y.data(ctx, fl ? 'ch4.c' : 'ch4.b');
      if (!y) return sp;
      return fl ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your zddot_fl model'), { obs: 'your F_fl model' })
        : Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your A_jac'), { obs: 'your A_jac' });
    },

    math(ctx) {
      const p = ctx.pModel, t = ans.tf(p);
      const Afl = [[0, 1], [0, -t.a1]];
      return [
        { title: 'Equilibria', page: 'p. 60',
          theory: '\\dot x = f(x, u):\\quad (x_e, u_e) \\text{ is an equilibrium when } f(x_e, u_e) = 0' },
        { title: 'Jacobian linearization', page: 'p. 60 · Eq. 4.1',
          theory: '\\tilde x = x - x_e,\\; \\tilde u = u - u_e:\\quad \\dot{\\tilde x} \\approx \\frac{\\partial f}{\\partial x}\\Big|_{(x_e, u_e)}\\tilde x + \\frac{\\partial f}{\\partial u}\\Big|_{(x_e, u_e)}\\tilde u' },
        { title: 'Feedback linearization', page: 'p. 64',
          theory: 'u = u_{fl}(x) + \\tilde u',
          note: 'u_fl cancels the nonlinear terms, so the model from ũ is linear for every x, not only near an equilibrium.' },
        { title: 'Equation of motion (D.3)', page: 'p. 43', answers: 'D.3/d',
          theory: '\\dot x = \\begin{bmatrix}\\dot z\\\\ \\frac{1}{m}F - \\frac{b}{m}\\dot z - \\frac{k}{m}z\\end{bmatrix}' },
        { title: 'Equilibria of the mass-spring-damper', page: 'p. 59–60', answers: 'D.4/a',
          theory: '\\dot z_e = 0,\\quad z_e \\text{ arbitrary},\\quad F_e = k z_e',
          numbers: `F_e(${fmt(ctx.st.zE, 3)}\\,\\text{m}) = ${tex(p.k * ctx.st.zE)}\\;\\text{N}` },
        { title: 'Jacobian-linearized mass-spring-damper', page: 'p. 60', answers: 'D.4/b',
          theory: 'm\\ddot{\\tilde z} + b\\dot{\\tilde z} + k\\tilde z = \\tilde F \\quad(\\text{identical for every } z_e)',
          numbers: `A = ${texMat(ans.ss(p).A)},\\quad B = ${texMat(ans.ss(p).B)},\\quad \\text{eig}(A) = ${L.eig(ans.ss(p).A).map((q) => texPole(q)).join(',\\;')}`,
          note: 'The plant is already linear, so the linearization is exact: the dashed trace lies on the simulated one.' },
        { title: 'Feedback-linearized mass-spring-damper', page: 'p. 64', answers: 'D.4/c',
          theory: 'F = kz + \\tilde F \\;\\Rightarrow\\; m\\ddot z + b\\dot z = \\tilde F',
          numbers: `A_{fl} = ${texMat(Afl)},\\quad \\text{eig} = 0,\\; ${tex(-t.a1)}`,
          note: 'Cancelling the spring leaves a free integrator: the mass then drifts after any force pulse. The plant is already linear, so F_fl = 0 (no cancellation) also gives a linear model.' },
      ];
    },

    buildProblem(parent, ctx) {
      lib.panel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibria',
          html: 'Which states can be equilibria, and what force holds the mass there? Write that force as a function of the equilibrium position.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'F_e', args: ['z_e'], truth: (p, a) => p.k * a.z_e }],
          }, 'def F_e(z_e):\n    # holds the mass at rest at z_e\n    return ...\n'), {
            actions: [{ label: 'Use my F_e', run: (code) => useMyFe(ctx, code) }],
          }),
          solution: () => [
            { tex: '\\dot z_e = 0,\\; \\ddot z_e = 0 \\Rightarrow F_e = kz_e \\quad(z_e \\text{ arbitrary})' },
            { code: 'def F_e(z_e):\n    return P.k * z_e' },
          ],
        },
        {
          id: 'b', title: '(b) Jacobian linearization',
          html: 'With x̃ = (z̃, z̃̇)ᵀ and ũ = F̃, return the matrices of x̃̇ = A x̃ + B ũ about the equilibrium at z<sub>e</sub>.',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'A_jac', args: ['z_e'], truth: (p) => ans.ss(p).A },
              { fn: 'B_jac', args: ['z_e'], truth: (p) => ans.ss(p).B },
            ],
          }, 'def A_jac(z_e):\n    return ...\n\ndef B_jac(z_e):\n    return ...\n'), {
            actions: [{ label: 'Plot my A, B', run: (code) => { ctx.st.method = 'jacobian'; return Y.plot(ctx, 'ch4.b', code, runJacobian, () => 'Your eigenvalues are the × markers on the s-plane, and your linear model (from the same δz(0) and F̃) is dotted on the z plot. They follow zₑ and the input.'); } }],
          }),
          solution: () => [
            { tex: 'm\\ddot{\\tilde z} + b\\dot{\\tilde z} + k\\tilde z = \\tilde F \\quad(\\text{the same for every } z_e)' },
            { code: 'def A_jac(z_e):\n    return np.array([[0, 1],\n                     [-P.k / P.m, -P.b / P.m]])\n\ndef B_jac(z_e):\n    return np.array([[0], [1 / P.m]])' },
            { html: 'None of these depend on z<sub>e</sub>: the system is linear. Book: Eq. 4.1 (p. 60).' },
          ],
        },
        {
          id: 'c', title: '(c) Feedback linearization',
          html: 'Choose F = F<sub>fl</sub>(z, ż) + F̃ so the resulting model is linear for every state, and return that model\'s z̈. Any F<sub>fl</sub> that does this is accepted.',
          code: { template: 'def F_fl(z, zdot):\n    return ...\n\ndef zddot_fl(z, zdot, F_tilde):\n    # z_ddot when F = F_fl + F_tilde\n    return ...\n', check: (code) => lib.refresh(ctx, checkFeedbackLin(ctx, code)),
            actions: [{ label: 'Simulate my F_fl', run: (code) => { ctx.st.method = 'fl'; return Y.plot(ctx, 'ch4.c', code, runFeedbackLin, () => 'Dotted on the z plot: the mass with your F = F_fl + F̃, and (long dashes) your zddot_fl model driven by F̃. With a correct answer they overlap. The × markers are your model\'s eigenvalues.'); } }] },
          solution: () => [
            { tex: 'F = kz + \\tilde F \\;\\Rightarrow\\; m\\ddot z + b\\dot z = \\tilde F' },
            { code: 'def F_fl(z, zdot):\n    return P.k * z\n\ndef zddot_fl(z, zdot, F_tilde):\n    return (F_tilde - P.b * zdot) / P.m' },
            { html: 'This cancels the spring force, as the book does for gravity on the arm (Eqs. 4.6–4.7, p. 64). The plant is already linear, so F<sub>fl</sub> = 0 or F<sub>fl</sub> = kz + bż (z̈ = F̃/m) are also accepted (ISSUES.md).' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------- D.5 and D.6 --
  const flInput = (parent, ctx, title, page) => {
    const inp = section(parent, title, page);
    inputControls(inp, ctx);
    lib.note(inp, 'The dashed trace is the linear model driven by the same force. The plant is linear, so the two coincide unless you add mismatch, saturation or a disturbance.');
  };
  const olLinear = (ctx, c) => lib.linearSim(ctx, c, openLoop);

  // D.5(b): the student's transfer_function(s), fitted with the lowest-order
  // proper rational function that matches it.
  const runTf = (ctx, code) => Y.fitTf(ctx, code, 'transfer_function');

  // D.6: the student's A, B, C, D.
  async function runSS(ctx, code) {
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, vars: ['A', 'B', 'C', 'D'] }]);
    if (out.error) return lib.pyError(out);
    const v = out.rows[0].vars;
    const A = Y.asMat(v.A, 2, 2), B = Y.asMat(v.B, 2, 1), C = Y.asMat(v.C, 1, 2), D = Y.asMat(v.D, 1, 1);
    const bad = [[A, 'A', '2×2'], [B, 'B', '2×1'], [C, 'C', '1×2'], [D, 'D', '1×1']].find(([m]) => !m);
    if (bad) return { ok: false, msg: `${bad[1]} should be a ${bad[2]} array of numbers (x = (z, ż), u = F, y = z).` };
    return { A, B, C, D };
  }

  CH.ch5 = Object.assign({}, common, {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer function', pages: 'pp. 69–80, p. 378',
    defaults() { return { comp: 'none', inp: { shape: 'pulse', amp: 1, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearSim: olLinear,
    linearLabel: 'P(s) response',
    buildControls(parent, ctx) { flInput(parent, ctx, 'Input F(t)', 'p. 378'); },
    splane(ctx) {
      const sp = eigMarkers(ctx, ans.ss(ctx.pModel).A, 'pole of P(s)', 'D.5/b');
      const y = Y.data(ctx, 'ch5.b');
      if (!y) return sp;
      return Y.withMarkers(sp, [...Y.poles(WB.tf.poles(y.G), 'pole of your P(s)'), ...Y.zeros(WB.tf.zeros(y.G), 'zero of your P(s)')],
        { obs: 'pole of your P(s)', zero: 'zero of your P(s)' });
    },
    // Your P(s) driven by the same F from rest (a transfer function assumes zero initial conditions).
    outputSeries(ctx) {
      const y = Y.data(ctx, 'ch5.b');
      if (!y) return [];
      return [Y.series('your P(s)', Y.tfResponse(ctx, y.G, fin(ctx)))];
    },
    math(ctx) {
      const t = ans.tf(ctx.pModel);
      const ol = ans.olPoles(ctx.pModel);
      return [
        { title: 'Laplace transform', page: 'p. 69–70',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)' },
        { title: 'Transfer function', page: 'p. 70–71',
          theory: '\\text{zero initial conditions:}\\quad P(s) = \\frac{Y(s)}{U(s)}' },
        { title: 'Block diagram', page: 'p. 70',
          theory: 'U(s) \\longrightarrow \\boxed{P(s)} \\longrightarrow Y(s)' },
        { title: 'Equation of motion (D.3)', page: 'p. 43', answers: 'D.3/d',
          theory: 'm\\ddot z + b\\dot z + kz = F' },
        { title: 'Transfer function of the mass-spring-damper', page: 'p. 70–71', answers: 'D.5/b',
          theory: '(ms^2 + bs + k)Z(s) = F(s) \\;\\Rightarrow\\; P(s) = \\frac{Z(s)}{F(s)} = \\frac{1/m}{s^2 + \\frac bm s + \\frac km}',
          numbers: `P(s) = \\frac{${tex(t.b0)}}{s^2 + ${tex(t.a1)}\\,s + ${tex(t.a0)}},\\quad \\text{poles } ${texPole(ol[0])},\\; ${texPole(ol[1])}` },
        { title: 'Block diagram of the mass-spring-damper', page: 'p. 70', answers: 'D.5/c',
          theory: 'F \\to \\boxed{\\tfrac1m} \\to \\ddot z \\to \\boxed{\\tfrac1s} \\to \\dot z \\to \\boxed{\\tfrac1s} \\to z,\\quad -b\\dot z - kz \\text{ fed back into the sum before } \\tfrac1m' },
      ];
    },
    buildProblem(parent, ctx) {
      const cx = WB.py.cx;
      const t = () => ans.tf(ctx.pModel);
      lib.panel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'a', title: '(a) Laplace transform of the equations of motion',
          html: 'With zero initial conditions, return F(s) for a given Z(s), both complex (the check calls it at several s and Z).',
          code: pyPart(ctx, {
            items: [{ fn: 'F_of', args: ['s', 'Z'], truth: (p, a) => cx.mul(cx.poly([p.m, p.b, p.k], a.s), a.Z) }],
          }, 'def F_of(s, Z):\n    # F(s) that gives the motion Z(s)\n    return ...\n'),
          solution: () => [
            { tex: 'm\\ddot z + b\\dot z + kz = F \\;\\xrightarrow{\\mathcal L}\\; (ms^2 + bs + k)Z(s) = F(s) \\quad(z(0) = \\dot z(0) = 0)' },
            { code: 'def F_of(s, Z):\n    return (P.m * s**2 + P.b * s + P.k) * Z' },
            { html: 'Book: §5.1, p. 69–70.' },
          ],
        },
        {
          id: 'b', title: '(b) Transfer function from F to z',
          html: 'Return P(s) = Z(s)/F(s) evaluated at a complex s (the check calls it at several).',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'transfer_function', args: ['s'], truth: (p, a) => { const m = ans.tf(p); return cx.div(m.b0, cx.poly([1, m.a1, m.a0], a.s)); } }],
          }, 'def transfer_function(s):\n    # Z(s)/F(s) at complex s\n    return ...\n'), {
            actions: [{ label: 'Plot my P(s)', run: (code) => Y.plot(ctx, 'ch5.b', code, runTf, (d) => `Your P(s) has ${d.G.den.length - 1} pole${d.G.den.length === 2 ? '' : 's'} (× on the s-plane); its response to the same F, from rest, is dotted on the z plot.`) }],
          }),
          solution: () => [
            { tex: `P(s) = \\frac{1/m}{s^2 + \\frac bm s + \\frac km} = \\frac{${tex(t().b0)}}{s^2 + ${tex(t().a1)}s + ${tex(t().a0)}}` },
            { code: 'def transfer_function(s):\n    return (1 / P.m) / (s**2 + P.b / P.m * s + P.k / P.m)' },
            { html: 'Cross-checked against C(sI − A)<sup>−1</sup>B from D.6.' },
          ],
        },
        {
          id: 'c', title: '(c) Draw the associated block diagram',
          html: 'On paper. When you are done, click the button and compare with the solution.',
          done: 'I\'ve drawn it',
          solution: () => [{ html: 'F(s) → [1/(ms² + bs + k)] → Z(s). Expanded: a summing junction forms F − bż − kz, then 1/m gives z̈, and two integrators 1/s give ż and z, with gains b and k feeding ż and z back to the junction.' }],
        },
      ]);
    },
  });

  CH.ch6 = Object.assign({}, common, {
    id: 'ch6', num: 6, tab: 'Ch 6', title: 'State-space model', pages: 'pp. 81–93, p. 379',
    defaults() { return { comp: 'none', inp: { shape: 'pulse', amp: 1, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearSim: olLinear,
    linearLabel: 'ẋ = Ax + Bu response',
    buildControls(parent, ctx) { flInput(parent, ctx, 'Input u = F(t)', 'p. 379'); },
    splane(ctx) {
      const sp = eigMarkers(ctx, ans.ss(ctx.pModel).A, 'eigenvalue of A', 'D.6/a');
      const y = Y.data(ctx, 'ch6.a');
      return y ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your A'), { obs: 'eigenvalue of your A' }) : sp;
    },
    // Your model from the simulation's initial state, driven by the same F.
    yourRun(ctx, res) {
      const y = Y.data(ctx, 'ch6.a');
      return y ? Y.linResponse(ctx, y, res.x[0], fin(ctx)) : null;
    },
    outputSeries(ctx, res) {
      const r = this.yourRun(ctx, res);
      return r ? [Y.series('your A, B, C, D', r.y[0])] : [];
    },
    extraPlot(ctx, res) {
      const series = [{ label: 'ż', y: zSeries(res), color: '--series-1' }];
      const r = this.yourRun(ctx, res);
      if (r) series.push(Y.series('your x₂', r.x.map((x) => x[1])));
      return { opts: { title: 'x₂ = ż(t)', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series } };
    },
    math(ctx) {
      const { A, B, C } = ans.ss(ctx.pModel);
      return [
        { title: 'Jacobian linearization revisited', page: 'p. 83–84',
          theory: 'A = \\frac{\\partial f}{\\partial x}\\Big|_e,\\quad B = \\frac{\\partial f}{\\partial u}\\Big|_e,\\quad C = \\frac{\\partial h}{\\partial x}\\Big|_e,\\quad D = \\frac{\\partial h}{\\partial u}\\Big|_e' },
        { title: 'Back to the transfer function', page: 'p. 85 · Eq. 6.14, p. 86 · Eq. 6.15',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\text{poles: } \\det(sI - A) = 0' },
        { title: 'State-space model of the mass-spring-damper', page: 'p. 379 · D.6', answers: 'D.6/a',
          theory: 'A = \\begin{bmatrix}0 & 1\\\\ -\\frac km & -\\frac bm\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\ \\frac1m\\end{bmatrix},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\quad D = 0',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(C)},\\quad \\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}` },
      ];
    },
    buildProblem(parent, ctx) {
      const ss = (p) => ans.ss(p);
      lib.panel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'Linear state-space equations',
        html: 'Define A, B, C, D as numpy arrays (module-level variables).',
        code: Object.assign(pyPart(ctx, {
          items: ['A', 'B', 'C', 'D'].map((k) => ({ var: k, truth: (p) => ss(p)[k] })),
        }, '# x = (z, zdot), u = F, y = z\nA = ...\nB = ...\nC = ...\nD = ...\n'), {
          actions: [{ label: 'Plot my model', run: (code) => Y.plot(ctx, 'ch6.a', code, runSS, () => 'Your eigenvalues of A are the × markers on the s-plane; your model\'s y and x₂, from the same initial state and F, are dotted on the plots.') }],
        }),
        solution: () => { const { A, B } = ss(ctx.pModel); return [
          { tex: `A = \\begin{bmatrix}0 & 1\\\\ -\\frac km & -\\frac bm\\end{bmatrix} = ${texMat(A)},\\quad B = \\begin{bmatrix}0\\\\ \\frac1m\\end{bmatrix} = ${texMat(B)},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\quad D = 0` },
          { code: 'A = np.array([[0, 1],\n              [-P.k / P.m, -P.b / P.m]])\nB = np.array([[0], [1 / P.m]])\nC = np.array([[1, 0]])\nD = np.array([[0]])' },
          { html: 'Check: C(sI − A)<sup>−1</sup>B reproduces the D.5 transfer function.' },
        ]; },
      }]);
    },
  });

  WB.studies.D.models = { openLoop, inputControls };
})();
