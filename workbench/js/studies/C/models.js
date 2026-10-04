// Study C, chapters 2-6: modeling the satellite. Open-loop experiments: kinetic
// energy (C.2), Euler-Lagrange equations with an energy-balance check (C.3),
// equilibria (C.4), the transfer matrix and the cascade approximation (C.5), and
// the state-space model with the star-tracker/strain-gauge outputs (C.6).
//
// Work mode: every derivation is a Python answer (WB.py.check), checked at random
// arguments and parameters; cards with the satellite's own results stay locked
// until the part that derives them is solved.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const PD = () => WB.pd;
  const lib = () => WB.studies.C.lib;
  const CH = WB.studies.C.chapters;
  const R2D = 180 / Math.PI;
  const cx = WB.py.cx;

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

  // ------------------------------------------------- Python answer parts --
  // Arguments for the satellite's Python answers (WB.py.check draws them at random).
  // s stays away from the lightly damped panel mode near 0.3j.
  const ARGS = {
    theta: { label: 'θ', lo: -1.5, hi: 1.5 },
    phi: { label: 'φ', lo: -1.5, hi: 1.5 },
    thetadot: { label: 'θ̇', lo: -2, hi: 2 },
    phidot: { label: 'φ̇', lo: -2, hi: 2 },
    tau: { label: 'τ', lo: -2, hi: 2 },
    state: { col: ['theta', 'phi', 'thetadot', 'phidot'] },
    theta_e: { label: 'θₑ', lo: -3, hi: 3 },
    s: { label: 's', complex: true, re: [-1.5, 1], im: [0.8, 4] },
  };
  const pyPart = (ctx, spec, template) => ({ template, check: (code) => WB.py.check(ctx, { args: ARGS, ...spec }, code) });
  const pyError = (out) => ({ ok: false, msg: out.timeout ? out.error : 'Python raised an error.', detail: [out.error, out.where, (out.stdout || '').trim()].filter(Boolean).join('\n') });
  const accel = (ctx, p, a) => { const xd = ctx.sys.f([a.theta, a.phi, a.thetadot, a.phidot], a.tau, p); return [xd[2], xd[3]]; };
  // In Work mode, poles/eigenvalues that answer `key` stay off the s-plane until it is solved.
  const showsAnswer = (ctx, key) => ctx.S.mode === 'explore' || ctx.app.isSolved(key);
  // True when every value of `got` is minus the matching value of `want` (a sign slip).
  const negated = (got, want) => {
    const g = [].concat(got).flat(3).map(Number), w = [].concat(want).flat(3).map(Number);
    return g.length === w.length && w.some((v) => Math.abs(v) > 1e-9) && g.every((v, i) => Math.abs(v + w[i]) < 1e-6 * Math.max(1, Math.abs(w[i])));
  };

  // "Plot my answer" overlays (js/core/yours.js).
  const Y = WB.yours;
  const fin = (ctx) => (t) => torque(ctx.st, t);
  const maxAbs = (y) => Array.prototype.reduce.call(y, (m, v) => Math.max(m, Math.abs(v)), 0);
  // A "your" trace (already in plot units) that joins the autoscale only while it
  // stays within a few times the plot's own trace `ref`.
  const yours = (label, y, ref, more) => Y.series(label, y, { lim: Math.max(10, 3 * maxAbs(ref)), ...more });
  // The student's functions `fns` of s, sampled at Y.S_PTS: [[complex per point] per fn].
  async function sampleS(ctx, code, fns) {
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: Y.S_PTS.flatMap((s) => fns.map((name) => ({ name, args: [s] }))) }]);
    if (out.error) return Y.pyError(out);
    const v = out.rows[0].calls;
    return { vals: fns.map((_, j) => Y.S_PTS.map((_, i) => v[i * fns.length + j])) };
  }
  const isCx = (v) => v !== undefined && v !== null && Number.isFinite(cx.of(v).re) && Number.isFinite(cx.of(v).im);
  // Fit sampled values with the lowest-order proper G(s), or explain why not.
  function fitSamples(P, what) {
    if (!P.every((v) => isCx(v) && !Array.isArray(v))) return { ok: false, msg: `${what} should be one (complex) number at each s.` };
    const G = WB.tf.fit(Y.S_PTS, P.map(cx.of), 4);
    return G ? { G } : { ok: false, msg: `${what} is not a proper ratio of polynomials in s of degree 4 or less, so there is no model to plot.` };
  }
  const tfMarkers = (G, name) => [...Y.poles(WB.tf.poles(G), `pole of ${name}`), ...Y.zeros(WB.tf.zeros(G), `zero of ${name}`)];

  // ------------------------------------------------------------- C.2 --
  // The student's kinetic(θ, φ, θ̇, φ̇) along the prescribed motion.
  async function runKinetic(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: res.x.map((x) => ({ name: 'kinetic', args: x })) }], 15000);
    if (out.error) return Y.pyError(out);
    const K = out.rows[0].calls;
    if (!K.every((v) => typeof v === 'number')) return { ok: false, msg: 'kinetic should return one number.' };
    return { n: K.length, K };
  }

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
      sec.append(el('p', { class: 'muted small', text: ctx.S.mode === 'explore'
        ? 'No dynamics here: the angles are imposed. The plot below splits K into the body and panel parts.'
        : 'No dynamics here: the angles are imposed. Once (a) is solved, or you click Plot my K, the plot below shows the kinetic energy along this motion; until then it is empty.' }));
    },

    // K(t) answers C.2(a), so in Work mode the plot is empty until (a) is solved, apart
    // from the student's own K once they plot it.
    extraPlot(ctx, res) {
      const show = showsAnswer(ctx, 'C.2/a'), y = Y.data(ctx, 'ch2.a');
      const series = show ? [
        { label: 'body ½Jsθ̇²', y: Array.from(res.extras.Ks), color: '--series-2', width: 1.5 },
        { label: 'panel ½Jpφ̇²', y: Array.from(res.extras.Kp), color: '--series-3', width: 1.5 },
        { label: 'total K', y: Array.from(res.extras.K), color: '--series-1' },
      ] : [];
      if (y) series.push(yours('your K (Python)', y.K, show ? res.extras.K : y.K, { color: '--text-primary' }));  // --series-3 is the panel
      return { opts: { title: 'kinetic energy', yLabel: 'K [J]', unit: 'J' }, data: { series } };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Several rigid bodies', page: 'p. 25',
          theory: 'K = \\sum_{j=1}^{n}\\left[\\tfrac12 m_j\\mathbf v_{cm,j}^\\top\\mathbf v_{cm,j} + \\tfrac12\\boldsymbol\\omega_j^\\top J_{cm,j}\\boldsymbol\\omega_j\\right]' },
        { title: 'Kinetic energy of the satellite', page: 'p. 36 · Eq. 2.5', answers: 'C.2/a',
          theory: '\\mathbf v_{cm} = 0,\\quad \\boldsymbol\\omega_s = \\dot\\theta\\,\\hat k,\\quad \\boldsymbol\\omega_p = \\dot\\phi\\,\\hat k,\\quad K = \\tfrac12 J_s\\dot\\theta^2 + \\tfrac12 J_p\\dot\\phi^2',
          numbers: `K = ${tex(p.Js / 2)}\\,\\dot\\theta^2 + ${tex(p.Jp / 2)}\\,\\dot\\phi^2` },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) Kinetic energy',
          html: 'Write K as a function of the configuration variables and their rates.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'kinetic', args: ['theta', 'phi', 'thetadot', 'phidot'], truth: (p, a) => 0.5 * p.Js * a.thetadot ** 2 + 0.5 * p.Jp * a.phidot ** 2 }],
          }, 'def kinetic(theta, phi, thetadot, phidot):\n    # kinetic energy K of the satellite\n    K = ...\n    return K\n'), {
            actions: [{ label: 'Plot my K', run: (code) => Y.plot(ctx, 'ch2.a', code, runKinetic, () => 'Your K(t) along the motion set on the right is the dotted trace on the lower plot.') }],
          }),
          solution: () => [
            { tex: 'K = \\tfrac12 J_s\\dot\\theta^2 + \\tfrac12 J_p\\dot\\phi^2' },
            { code: 'def kinetic(theta, phi, thetadot, phidot):\n    return (0.5 * P.Js * thetadot**2\n            + 0.5 * P.Jp * phidot**2)' },
            { html: 'Book: Eq. 2.5 (p. 36). Each body only rotates about the common axis, so there is no translational term.' },
          ],
        },
        {
          id: 'b', title: '(b) Animate the satellite',
          html: 'You write this class in your own animation code (Listing 2.5, pp. 36–38). The animation at the top of this page shows the same kind of prescribed motion, set in the controls on the right.',
        },
      ]);
    },
  });

  // ------------------------------------------------------------- C.3 --
  // Last "Simulate my f" run, shown while the simulation it was run against is unchanged.
  const mine = {};
  const simSig = (ctx, res) => JSON.stringify([ctx.S.sysId, ctx.S.chapter, ctx.pTrue, ctx.S.sim.Ts, res.t.length, res.x[0], Array.from(res.uApplied).reduce((a, v, i) => a + v * (i + 1), 0)]);
  async function simulateMine(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.simulate(code, { fn: 'f', params: ctx.pTrue, x0: res.x[0], u: Array.from(res.uApplied), Ts: ctx.S.sim.Ts });
    if (out.error) return pyError(out);
    mine.ch3 = { sig: simSig(ctx, res), theta: out.x.map((x) => x[0]), phi: out.x.map((x) => x[1]) };
    let dev = 0;
    out.x.forEach((x, k) => { dev = Math.max(dev, Math.abs(x[0] - res.x[k][0]), Math.abs(x[1] - res.x[k][1])); });
    ctx.update();
    const msg = `Your f is the dashed trace on the θ and φ plots. Largest difference from the workbench's satellite: ${fmt(dev * R2D, 3)}°.`;
    return { ok: dev * R2D < 0.01, msg: dev * R2D < 0.01 ? msg : `${msg} They should overlap.` };
  }

  // C.3(a): the student's potential(θ, φ) along the simulated motion.
  async function runPotential(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: res.x.map((x) => ({ name: 'potential', args: [x[0], x[1]] })) }], 15000);
    if (out.error) return Y.pyError(out);
    const P = out.rows[0].calls;
    if (!P.every((v) => typeof v === 'number')) return { ok: false, msg: 'potential should return one number.' };
    return { n: P.length, P };
  }

  // C.3(c): the work done by the student's τ and −Bq̇ along the simulated motion,
  // integrated like the energy plot's own curve (trapezoid rule, input held per step).
  const WORK_PY = `

def _wb_work(xs, us, Ts):
    def power(x, u):
        g = np.asarray(generalized_force(*x, u), dtype=float).reshape(-1)
        d = np.asarray(damping_force(*x, u), dtype=float).reshape(-1)
        if g.size != 2 or d.size != 2:
            raise ValueError('generalized_force and damping_force should each return 2 numbers')
        return x[2] * (g[0] + d[0]) + x[3] * (g[1] + d[1])
    w, out = 0.0, [0.0]
    for k in range(1, len(xs)):
        w += 0.5 * Ts * (power(xs[k - 1], us[k - 1]) + power(xs[k], us[k - 1]))
        out.append(w)
    return out
`;
  async function runForces(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.evaluate(code + WORK_PY, [{ params: ctx.pModel, calls: [{ name: '_wb_work', args: [res.x, Array.from(res.uApplied), ctx.S.sim.Ts] }] }], 15000);
    if (out.error) return Y.pyError(out);
    const W = out.rows[0].calls[0];
    if (!Array.isArray(W) || !W.every(Number.isFinite)) return { ok: false, msg: 'generalized_force and damping_force should return real numbers.' };
    return { n: W.length, W };
  }

  // C.3(d): the satellite with the student's accelerations, simulated like "Simulate my f".
  const ACCEL_PY = `

def _wb_f(state, tau):
    theta, phi, thetadot, phidot = (float(v) for v in state.reshape(-1))
    a = np.asarray(accelerations(theta, phi, thetadot, phidot, tau), dtype=float).reshape(-1)
    if a.size != 2:
        raise ValueError('accelerations should return 2 numbers (thetaddot, phiddot)')
    return np.array([[thetadot], [phidot], [a[0]], [a[1]]])
`;
  async function runAccel(ctx, code) {
    const res = ctx.app.result();
    const out = await WB.py.simulate(code + ACCEL_PY, { fn: '_wb_f', params: ctx.pTrue, x0: res.x[0], u: Array.from(res.uApplied), Ts: ctx.S.sim.Ts });
    if (out.error) return Y.pyError(out);
    return { n: out.x.length, theta: out.x.map((x) => x[0]), phi: out.x.map((x) => x[1]) };
  }

  CH.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'Ch 3', title: 'Euler-Lagrange equations', pages: 'pp. 51–55',
    linear: false,
    defaults(sys) { const o = sys.problems.ch3.openLoop; return { inp: { shape: o.shape, amp: o.amp, freq: o.freq, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 51 · C.3(e), Listing 3.5');
      inputControls(sec, ctx);
      sec.append(el('p', { class: 'muted small', text: ctx.S.mode === 'explore'
        ? 'The body is driven with τ = 0.1 sin(0.2πt) N·m. The energy plot checks the EOM: E(t) − E(0) must equal the work done by τ minus what the damper dissipates.'
        : 'The body is driven with τ = 0.1 sin(0.2πt) N·m. The lower energy-balance plot is empty until you plot your own P or forces or solve (a) or (c): E(t) − E(0) after (a) and the net work of the nonconservative forces after (c), which must agree. Plot my P / Plot my forces draw your own versions any time.' }));
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
      // E(t) − E(0) is built from K and P (C.3(a)) and the work curve from the forces
      // (C.3(c)), so in Work mode each appears once its part is solved; until then
      // (and with no overlay of the student's own) the plot is empty.
      const showE = showsAnswer(ctx, 'C.3/a'), showW = showsAnswer(ctx, 'C.3/c');
      const yp = Y.data(ctx, 'ch3.a'), yf = Y.data(ctx, 'ch3.c');
      const series = [];
      if (showW) series.push({ label: '∫(τθ̇ − b(θ̇−φ̇)²) dt', y: W, color: '--series-2', dash: [5, 4], width: 2 });
      if (showE) series.push({ label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' });
      // C.3(a): the satellite's ΔK plus the student's ΔP; C.3(c): the work of the student's forces.
      if (yp) {
        const K = (x) => 0.5 * p.Js * x[2] ** 2 + 0.5 * p.Jp * x[3] ** 2;
        const mine = res.x.map((x, k) => (K(x) - K(res.x[0]) + yp.P[k] - yp.P[0]) * 1000);
        series.push(yours('ΔK + your ΔP', mine, showE ? E : mine));
      }
      if (yf) { const mine = yf.W.map((v) => v * 1000); series.push(yours('work of your forces', mine, showW ? W : mine)); }
      return { opts: { title: 'energy balance', yLabel: 'energy [mJ]', unit: 'mJ' }, data: { series } };
    },

    // C.3(e): the student's f, simulated with the same input as the satellite above;
    // C.3(d): the same with the student's accelerations.
    outputSeries(ctx, res, sc, oi) {
      const out = [], m = mine.ch3, y = Y.data(ctx, 'ch3.d');
      if (m && m.sig === simSig(ctx, res)) out.push({ label: 'your f (Python)', y: sc(oi === 0 ? m.theta : m.phi), color: '--series-2', dash: [5, 4], width: 2 });
      if (y) out.push(yours('your accelerations', sc(oi === 0 ? y.theta : y.phi), sc(res.yAll[oi])));
      return out;
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 43 · §3.1.4',
          theory: 'L(q, \\dot q) = K(q, \\dot q) - P(q),\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Potential energy', page: 'p. 41–42 · §3.1.1',
          theory: '\\text{gravity: } P = mgy + P_0,\\quad \\text{spring: } P = \\tfrac12 k z^2 \\;(z = \\text{deflection})',
          note: 'A torsional spring stores energy the same way, with z its angular deflection.' },
        { title: 'Generalized coordinates, forces, damping', page: 'p. 42–43 · §3.1.2–3.1.3',
          theory: 'q = \\text{minimum set of configuration variables},\\quad \\tau = \\text{applied (nonconservative) forces along } q,\\quad -B\\dot q = \\text{damping forces}' },
        { title: 'Kinetic energy of the satellite (C.2)', page: 'p. 36 · Eq. 2.5', answers: 'C.2/a',
          theory: 'K = \\tfrac12 J_s\\dot\\theta^2 + \\tfrac12 J_p\\dot\\phi^2',
          numbers: `K = ${tex(p.Js / 2)}\\,\\dot\\theta^2 + ${tex(p.Jp / 2)}\\,\\dot\\phi^2` },
        { title: 'Potential energy of the satellite', page: 'p. 52', answers: 'C.3/a',
          theory: 'P = \\tfrac12 k(\\phi - \\theta)^2',
          numbers: `P = ${tex(p.k / 2)}\\,(\\phi - \\theta)^2` },
        { title: 'Generalized forces and damping of the satellite', page: 'p. 52', answers: 'C.3/c',
          theory: 'q = (\\theta, \\phi)^\\top,\\quad \\tau = (\\tau, 0)^\\top,\\quad -B\\dot q = -\\begin{bmatrix} b & -b\\\\ -b & b\\end{bmatrix}\\begin{bmatrix}\\dot\\theta\\\\ \\dot\\phi\\end{bmatrix}' },
        { title: 'Equations of motion of the satellite', page: 'p. 53 · Eq. 3.3', answers: 'C.3/d',
          theory: '\\begin{bmatrix}J_s & 0\\\\ 0 & J_p\\end{bmatrix}\\begin{bmatrix}\\ddot\\theta\\\\ \\ddot\\phi\\end{bmatrix} = \\begin{bmatrix}\\tau - b(\\dot\\theta - \\dot\\phi) - k(\\theta - \\phi)\\\\ -b(\\dot\\phi - \\dot\\theta) - k(\\phi - \\theta)\\end{bmatrix}',
          numbers: `\\ddot\\theta = ${tex(1 / p.Js)}\\tau - ${tex(p.b / p.Js)}(\\dot\\theta - \\dot\\phi) - ${tex(p.k / p.Js)}(\\theta - \\phi),\\quad \\ddot\\phi = -${tex(p.b / p.Jp)}(\\dot\\phi - \\dot\\theta) - ${tex(p.k / p.Jp)}(\\phi - \\theta)` },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from p. 43',
          theory: '\\frac{d}{dt}(K + P) = \\dot q^\\top\\big(\\tau - B\\dot q\\big)',
          note: 'Once the parts it uses are solved, the plot below integrates the right side and compares it with the energy.' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch3;
      const norm = (v) => String(v || '').toLowerCase().replace(/\s+/g, '').replace(/^q=/, '').replace(/[()[\]{}]|\^t|ᵀ|'/g, '');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Potential energy',
          html: 'Write P as a function of the configuration variables. Any constant P<sub>0</sub> is accepted.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'potential', args: ['theta', 'phi'], compare: 'offset', truth: (p, a) => 0.5 * p.k * (a.phi - a.theta) ** 2 }],
          }, 'def potential(theta, phi):\n    # potential energy P of the satellite\n    return ...\n'), {
            actions: [{ label: 'Plot my P', run: (code) => Y.plot(ctx, 'ch3.a', code, runPotential, () => 'On the energy plot, the dotted trace is the satellite\'s ΔK plus your ΔP along the simulated motion. With the right P it follows E(t) − E(0).') }],
          }),
          solution: () => [
            { tex: 'P = \\tfrac12 k(\\phi - \\theta)^2 \\quad(\\text{the spring is twisted by } \\phi - \\theta)' },
            { code: 'def potential(theta, phi):\n    return 0.5 * P.k * (phi - theta)**2' },
            { html: 'Book: p. 52.' },
          ],
        },
        {
          id: 'b', title: '(b) Generalized coordinates',
          inputs: { q: 'q =' },
          html: 'Name them, comma separated (e.g. <code>x, y</code> or <code>θ, φ</code>).',
          check: (v) => {
            const g = norm(v.q);
            if (!g) return { ok: false, msg: 'Enter the generalized coordinates.' };
            if (/dot|̇|ω|omega/.test(g)) return { ok: false, msg: 'Generalized coordinates are configuration variables (positions and angles), not velocities.' };
            const parts = g.split(/[,;]/).filter(Boolean).map((x) => ({ θ: 'theta', φ: 'phi', ϕ: 'phi', q1: 'theta', q2: 'phi' })[x] || x);
            const ok = parts.length === 2 && parts.includes('theta') && parts.includes('phi');
            if (ok) return { ok: true, msg: parts[0] === 'theta' ? 'Two coordinates, so q is a 2-vector.' : 'Right. The book orders them q = (θ, φ)ᵀ; the later parts use that order.' };
            return { ok: false, msg: parts.length < 2 ? 'Which variables fix the configuration of the satellite? One is not enough.' : 'Which variables fix the configuration of the satellite? Use the minimum number.' };
          },
          solution: () => [{ tex: 'q = (\\theta, \\phi)^\\top' }, { html: 'Book: p. 51.' }],
        },
        {
          id: 'c', title: '(c) Generalized forces and damping forces',
          html: 'Return each as a 2-vector along q = (θ, φ)ᵀ. Write the damping term the way the book does, as the force −Bq̇ (p. 43).',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'generalized_force', args: ['theta', 'phi', 'thetadot', 'phidot', 'tau'], truth: (p, a) => [a.tau, 0] },
              { fn: 'damping_force', args: ['theta', 'phi', 'thetadot', 'phidot', 'tau'], truth: (p, a) => [-p.b * (a.thetadot - a.phidot), -p.b * (a.phidot - a.thetadot)] },
            ],
            explain: (it, f) => (it.fn === 'damping_force' && negated(f.e.got, f.e.want) ? 'Check the sign: the book writes the damping force as −Bq̇.' : ''),
          }, 'def generalized_force(theta, phi, thetadot,\n                      phidot, tau):\n    # applied forces along q\n    return ...\n\ndef damping_force(theta, phi, thetadot,\n                  phidot, tau):\n    # damping forces, -B*q_dot\n    return ...\n'), {
            actions: [{ label: 'Plot my forces', run: (code) => Y.plot(ctx, 'ch3.c', code, runForces, () => 'On the energy plot, the dotted trace is the work your τ and −Bq̇ do along the simulated motion. With the right forces it lies on the dashed work curve.') }],
          }),
          solution: () => [
            { tex: '\\tau = \\begin{bmatrix}\\tau\\\\ 0\\end{bmatrix},\\quad -B\\dot q = \\begin{bmatrix}-b(\\dot\\theta - \\dot\\phi)\\\\ -b(\\dot\\phi - \\dot\\theta)\\end{bmatrix} = -\\begin{bmatrix}b & -b\\\\ -b & b\\end{bmatrix}\\begin{bmatrix}\\dot\\theta\\\\ \\dot\\phi\\end{bmatrix}' },
            { code: 'def generalized_force(theta, phi, thetadot,\n                      phidot, tau):\n    return np.array([tau, 0.0])\n\ndef damping_force(theta, phi, thetadot,\n                  phidot, tau):\n    return np.array([-P.b * (thetadot - phidot),\n                     -P.b * (phidot - thetadot)])' },
            { html: 'Book: p. 52. The thrusters act on the body only; the damper acts on the relative motion.' },
          ],
        },
        {
          id: 'd', title: '(d) Equations of motion',
          html: 'Apply the Euler-Lagrange equations, then solve for both accelerations.',
          code: Object.assign(pyPart(ctx, {
            cases: [
              { label: 'with θ̇ = φ̇ = 0 and τ = 0 (only the spring acts)', fix: { thetadot: 0, phidot: 0, tau: 0 } },
              { label: 'with θ = φ = 0 and τ = 0 (only the damper acts)', fix: { theta: 0, phi: 0, tau: 0 } },
              { label: 'with θ = φ = 0 and θ̇ = φ̇ = 0 (only the torque acts)', fix: { theta: 0, phi: 0, thetadot: 0, phidot: 0 } },
              { label: '' },
            ],
            items: [{ fn: 'accelerations', args: ['theta', 'phi', 'thetadot', 'phidot', 'tau'], truth: (p, a) => accel(ctx, p, a) }],
          }, 'def accelerations(theta, phi, thetadot,\n                  phidot, tau):\n    # from the equations of motion\n    thetaddot = ...\n    phiddot = ...\n    return thetaddot, phiddot\n'), {
            actions: [{ label: 'Simulate my accelerations', run: (code) => Y.plot(ctx, 'ch3.d', code, runAccel, () => 'Your accelerations, integrated with RK4 from the same initial state and torque as the satellite above (true-plant parameters), are dotted on the θ and φ plots.') }],
          }),
          solution: () => [
            { tex: 'L = \\tfrac12 J_s\\dot\\theta^2 + \\tfrac12 J_p\\dot\\phi^2 - \\tfrac12 k(\\phi - \\theta)^2,\\quad \\frac{\\partial L}{\\partial\\dot q} = \\begin{bmatrix}J_s\\dot\\theta\\\\ J_p\\dot\\phi\\end{bmatrix},\\quad \\frac{\\partial L}{\\partial q} = \\begin{bmatrix}k(\\phi - \\theta)\\\\ -k(\\phi - \\theta)\\end{bmatrix}' },
            { tex: '\\begin{bmatrix}J_s & 0\\\\ 0 & J_p\\end{bmatrix}\\begin{bmatrix}\\ddot\\theta\\\\ \\ddot\\phi\\end{bmatrix} = \\begin{bmatrix}\\tau - b(\\dot\\theta - \\dot\\phi) - k(\\theta - \\phi)\\\\ -b(\\dot\\phi - \\dot\\theta) - k(\\phi - \\theta)\\end{bmatrix}' },
            { code: 'def accelerations(theta, phi, thetadot,\n                  phidot, tau):\n    thetaddot = (tau - P.b * (thetadot - phidot)\n                 - P.k * (theta - phi)) / P.Js\n    phiddot = (-P.b * (phidot - thetadot)\n               - P.k * (phi - theta)) / P.Jp\n    return thetaddot, phiddot' },
            { html: 'Book: Eq. 3.3 (p. 53).' },
          ],
        },
        {
          id: 'e', title: '(e) Implement and simulate',
          html: 'Write f(x, u): the state x = (θ, φ, θ̇, φ̇)ᵀ is a 4×1 column. <em>Check</em> tests it at random states. <em>Simulate my f</em> runs it with RK4 on the same torque input as the satellite above (true-plant parameters) and draws θ and φ dashed on the plots.',
          code: Object.assign(pyPart(ctx, {
            items: [{ fn: 'f', args: ['state', 'tau'], truth: (p, a) => { const [td, pd] = accel(ctx, p, a); return [[a.thetadot], [a.phidot], [td], [pd]]; } }],
          }, 'def f(state, tau):\n    theta = state[0][0]\n    phi = state[1][0]\n    thetadot = state[2][0]\n    phidot = state[3][0]\n    thetaddot = ...\n    phiddot = ...\n    return np.array([[thetadot], [phidot],\n                     [thetaddot], [phiddot]])\n'), {
            actions: [{ label: 'Simulate my f', run: (code) => simulateMine(ctx, code) }],
          }),
          solution: () => [{ code: 'def f(state, tau):\n    theta = state[0][0]\n    phi = state[1][0]\n    thetadot = state[2][0]\n    phidot = state[3][0]\n    thetaddot = (tau - P.b * (thetadot - phidot)\n                 - P.k * (theta - phi)) / P.Js\n    phiddot = (-P.b * (phidot - thetadot)\n               - P.k * (phi - theta)) / P.Jp\n    return np.array([[thetadot], [phidot],\n                     [thetaddot], [phiddot]])' }, { html: 'Book: Listing 3.5 (p. 54).' }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------- C.4 --
  // C.4(a): the true satellite started at rest at the student's equilibrium for
  // θₑ (slider), with their τₑ held (RK4 at Ts, saturated like the plant).
  async function runEquilibrium(ctx, code) {
    const thE = ctx.st.thE / R2D;
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, calls: [{ name: 'phi_e', args: [thE] }, { name: 'tau_e', args: [thE] }] }]);
    if (out.error) return Y.pyError(out);
    const [phE, tauE] = out.rows[0].calls;
    if (typeof phE !== 'number' || !Number.isFinite(phE)) return { ok: false, msg: 'phi_e should return one number.' };
    if (typeof tauE !== 'number' || !Number.isFinite(tauE)) return { ok: false, msg: 'tau_e should return one number.' };
    const n = ctx.app.result().t.length, Ts = ctx.S.sim.Ts, p = ctx.pTrue, lim = ctx.sys.uLimit(p);
    const u = Math.max(-lim, Math.min(lim, tauE)), f = (x, uu) => ctx.sys.f(x, uu, p);
    let x = [thE, phE, 0, 0];
    const theta = [], phi = [];
    for (let k = 0; k < n; k++) {
      theta.push(x[0]); phi.push(x[1]);
      x = M.rk4Step(f, x, u, Ts);
    }
    return { n, thE, phE, tauE, theta, phi };
  }

  // From rest at θ0 ≠ φ0 with τ = 0, angular momentum Js θ̇ + Jp φ̇ stays 0, so the
  // pair settles at the inertia-weighted mean angle: one of the equilibria (4.15).
  CH.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'Ch 4', title: 'Equilibria', pages: 'pp. 67–68',
    defaults() { return { thE: 30, inp: { shape: 'zero', amp: 0.05, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    linear: false,

    finalAngle(ctx) {
      const p = ctx.pModel, th0 = ctx.S.sim.y0, ph0 = ctx.S.sim.init.phi0 ?? 0;
      return (p.Js * th0 + p.Jp * ph0) / (p.Js + p.Jp);
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Release from rest', 'p. 67 · Eq. 4.15');
      sec.append(el('p', { class: 'muted small', text: 'Set θ(0) ≠ φ(0) in the left panel (initial conditions) and let go with τ = 0. Watch where the two bodies come to rest.' }));
      inputControls(sec, ctx);
      const eq = section(parent, 'Test an equilibrium', 'C.4(a)');
      slider(eq, { label: 'θ<sub>e</sub>', unit: '°', min: -170, max: 170, step: 1, sig: 3, ...bind(ctx, 'thE') });
      eq.append(el('p', { class: 'muted small', text: 'Simulate my equilibrium (part a) starts the satellite at rest at this θₑ and your φₑ, holds τ = your τₑ, and draws it dotted. At an equilibrium it stays where it started.' }));
    },

    outputSeries(ctx, res, sc, oi) {
      const out = [];
      if (oi === 0 && ctx.st.inp.shape === 'zero' && WB.ui.shown(ctx, 'C:ch4:final')) {
        const fa = this.finalAngle(ctx);
        out.push({ label: '(Jsθ₀ + Jpφ₀)/(Js + Jp)', y: Array.from(res.t, () => fa), color: '--ref', dash: [6, 4], width: 1.5 });
      }
      // C.4(a): where the test started (dashed) and how the satellite moves from there (dotted).
      const y = Y.data(ctx, 'ch4.a');
      if (y) {
        const y0 = (oi === 0 ? y.thE : y.phE) * R2D;
        out.push({ label: 'starting angle', y: Array.from(res.t, () => y0), color: '--text-muted', dash: [6, 4], width: 1.5 });
        out.push(yours('satellite from your equilibrium', sc(oi === 0 ? y.theta : y.phi), [...sc(res.yAll[oi]), y0]));
      }
      return out;
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

    // The eigenvalues of A (the poles of C.5(b), from the C.6 model).
    splane(ctx) {
      if (!lib().showsOl(ctx)) return { markers: [] };
      const { A } = lib().ss(ctx.pModel);
      return { markers: L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `eigenvalue of A ${i + 1}` })) };
    },

    math(ctx) {
      const p = ctx.pModel;
      const { A } = lib().ss(p);
      return [
        { title: 'Equilibria', page: 'p. 60',
          theory: '\\dot x = f(x, u):\\quad (x_e, u_e) \\text{ is an equilibrium when } f(x_e, u_e) = 0' },
        { title: 'Equations of motion (C.3)', page: 'p. 67 · Eq. 4.13–4.14', answers: 'C.3/d',
          theory: 'J_s\\ddot\\theta + b(\\dot\\theta - \\dot\\phi) + k(\\theta - \\phi) = \\tau,\\quad J_p\\ddot\\phi + b(\\dot\\phi - \\dot\\theta) + k(\\phi - \\theta) = 0' },
        { title: 'Equilibria of the satellite', page: 'p. 67 · Eq. 4.15', answers: 'C.4/a',
          theory: '\\theta_e = \\phi_e \\text{ (any value)},\\quad \\dot\\theta_e = \\dot\\phi_e = 0,\\quad \\tau_e = 0',
          note: 'The model is already linear, so no linearization is needed.' },
        { title: 'Where it comes to rest (not in the book)', page: 'conservation of angular momentum',
          theory: '\\text{no external torque} \\Rightarrow \\text{total angular momentum is constant}',
          symbolic: '\\tau = 0 \\Rightarrow \\tfrac{d}{dt}(J_s\\dot\\theta + J_p\\dot\\phi) = 0 \\Rightarrow \\theta_\\infty = \\phi_\\infty = \\frac{J_s\\theta_0 + J_p\\phi_0}{J_s + J_p}',
          numbers: `\\theta_\\infty = ${tex(this.finalAngle(ctx))}^\\circ`, spoiler: true },
        { title: 'Eigenvalues of A (C.6)', page: 'p. 92', answers: 'C.6/a',
          theory: '\\det(sI - A) = s^2\\left(s^2 + \\frac{b(J_s + J_p)}{J_sJ_p}s + \\frac{k(J_s + J_p)}{J_sJ_p}\\right)',
          numbers: `\\text{eig}(A) = ${L.eig(A).map((q) => texPole(q)).join(',\\;')}`,
          note: 'Hover the s-plane markers to read the eigenvalues.' },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibria',
          html: 'Which states can be equilibria, and what input holds the satellite there? For an equilibrium with body angle θ<sub>e</sub>, return the panel angle and the torque.',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'phi_e', args: ['theta_e'], truth: (p, a) => a.theta_e },
              { fn: 'tau_e', args: ['theta_e'], truth: () => 0 },
            ],
          }, 'def phi_e(theta_e):\n    # panel angle at the equilibrium\n    return ...\n\ndef tau_e(theta_e):\n    # torque that holds it there\n    return ...\n'), {
            actions: [{ label: 'Simulate my equilibrium', run: (code) => Y.plot(ctx, 'ch4.a', code, runEquilibrium, (d) => `Your equilibrium for θₑ = ${fmt(ctx.st.thE, 3)}°: φₑ = ${fmt(d.phE * R2D, 4)}°, τₑ = ${fmt(d.tauE, 4)} N·m. Dotted on the θ and φ plots: the satellite started there at rest with τ = τₑ; the dashed lines mark where it started. Move θₑ on the right to try another.`) }],
          }),
          solution: () => [
            { tex: '\\dot\\theta_e = \\dot\\phi_e = 0,\\; \\ddot\\theta_e = \\ddot\\phi_e = 0 \\Rightarrow k(\\theta_e - \\phi_e) = \\tau_e,\\; k(\\phi_e - \\theta_e) = 0 \\Rightarrow \\phi_e = \\theta_e,\\; \\tau_e = 0' },
            { code: 'def phi_e(theta_e):\n    return theta_e\n\ndef tau_e(theta_e):\n    return 0.0' },
            { html: 'Book: Eq. 4.15 (p. 67). Any common angle is an equilibrium.' },
          ],
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
      numPhi: [b / (Js * Jp), k / (Js * Jp)],                       // Φ/τ numerator
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
  // Transfer functions at complex s, for the Python checks.
  const tfAt = {
    thetaTau: (p, s) => { const d = tfData(p); return cx.div(cx.poly(d.num, s), cx.poly(d.den, s)); },
    phiTau: (p, s) => { const d = tfData(p); return cx.div(cx.poly(d.numPhi, s), cx.poly(d.den, s)); },
    phiTheta: (p, s) => { const d = tfData(p); return cx.div(cx.poly(d.out.num, s), cx.poly(d.out.den, s)); },
    thetaTauApprox: (p, s) => cx.div(1, cx.mul(p.Js + p.Jp, cx.mul(s, s))),
  };

  // C.5 overlays. (a): the transfer matrix that the student's two transformed
  // equations imply (Cramer's rule at each sample s); (b): their Θ/τ and Φ/τ;
  // (c): their Φ/Θ in cascade with the satellite's own Θ/τ, i.e. driven by its θ;
  // (d), (e): their approximate Θ/τ and cascade Φ/τ. Each is fitted with the
  // lowest-order proper rational function and driven from rest by the same τ.
  async function runEquations(ctx, code) {
    const r = await sampleS(ctx, code, ['eq_theta', 'eq_phi']);
    if (r.ok === false) return r;
    const [e1, e2] = r.vals;
    const bad = [[e1, 'eq_theta'], [e2, 'eq_phi']].find(([e]) => !e.every((v) => Array.isArray(v) && v.length === 3 && v.every(isCx)));
    if (bad) return { ok: false, msg: `${bad[1]} should return three (complex) numbers (c_Theta, c_Phi, c_tau).` };
    const T = [], F = [];
    for (let i = 0; i < e1.length; i++) {
      const [a, b, c] = e1[i].map(cx.of), [d, e, g] = e2[i].map(cx.of);
      const det = cx.add(cx.mul(a, e), cx.mul(-1, cx.mul(b, d)));
      if (Math.hypot(det.re, det.im) < 1e-12) return { ok: false, msg: 'Your two equations do not determine Θ(s) and Φ(s) (they are multiples of each other).' };
      T.push(cx.div(cx.add(cx.mul(c, e), cx.mul(-1, cx.mul(b, g))), det));
      F.push(cx.div(cx.add(cx.mul(a, g), cx.mul(-1, cx.mul(d, c))), det));
    }
    const GT = fitSamples(T, 'The Θ(s)/τ(s) of your equations'), GP = fitSamples(F, 'The Φ(s)/τ(s) of your equations');
    return GT.ok === false ? GT : GP.ok === false ? GP : { GT: GT.G, GP: GP.G };
  }
  async function runTfMatrix(ctx, code) {
    const r = await sampleS(ctx, code, ['theta_over_tau', 'phi_over_tau']);
    if (r.ok === false) return r;
    const GT = fitSamples(r.vals[0], 'theta_over_tau'), GP = fitSamples(r.vals[1], 'phi_over_tau');
    return GT.ok === false ? GT : GP.ok === false ? GP : { GT: GT.G, GP: GP.G };
  }
  const runTf = (fn) => async (ctx, code) => {
    const r = await sampleS(ctx, code, [fn]);
    return r.ok === false ? r : fitSamples(r.vals[0], fn);
  };
  // Overlays per part: [output index, label, G(ctx, data), style].
  const TF_OVERLAYS = {
    'ch5.a': [[0, 'Θ from your equations', (ctx, y) => y.GT], [1, 'Φ from your equations', (ctx, y) => y.GP]],
    'ch5.b': [[0, 'your Θ/τ', (ctx, y) => y.GT, { dash: [8, 3] }], [1, 'your Φ/τ', (ctx, y) => y.GP, { dash: [8, 3] }]],
    'ch5.c': [[1, 'your Φ/Θ driven by θ', (ctx, y) => { const d = tfData(ctx.pModel); return WB.tf.mul(y.G, WB.tf.tf(d.num, d.den)); }, { dash: [4, 2, 1, 2] }]],
    'ch5.d': [[0, 'your approximate Θ/τ', (ctx, y) => y.G, { color: '--text-primary' }]],
    'ch5.e': [[1, 'your cascade Φ/τ', (ctx, y) => y.G, { color: '--text-primary', dash: [8, 3] }]],
  };
  const TF_MARKERS = { 'ch5.a': ['Θ/τ from your equations', 'Φ/τ from your equations'], 'ch5.b': ['your Θ/τ', 'your Φ/τ'], 'ch5.c': ['your Φ/Θ'], 'ch5.d': ['your approximate Θ/τ'], 'ch5.e': ['your cascade Φ/τ'] };
  const tfDescribe = (what, plot) => (d) => {
    const Gs = d.G ? [d.G] : [d.GT, d.GP];
    const np = Gs.map((G) => G.den.length - 1).join(' and ');
    return `${what} ${Gs.length > 1 ? 'have' : 'has'} ${np} pole${np === '1' ? '' : 's'} (× on the s-plane); ${plot}`;
  };

  CH.ch5 = Object.assign({}, common, {
    id: 'ch5', num: 5, tab: 'Ch 5', title: 'Transfer functions', pages: 'pp. 77–79',
    defaults() { return { inp: { shape: 'pulse', amp: 0.1, freq: 0.05, width: 2 }, overlay: 'cascade' }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearLabel: 'cascade approximation (parts d–e)',
    // The cascade's θ is the response of C.5(d)'s Θ/τ and its φ that of C.5(e)'s
    // Φ/τ, so in Work mode each trace appears once its part is solved.
    linearSim(ctx, c) {
      if (ctx.st.overlay !== 'cascade') return null;
      const showTh = showsAnswer(ctx, 'C.5/d'), showPh = showsAnswer(ctx, 'C.5/e');
      if (!showTh && !showPh) return null;
      const { A, B } = cascadeAB(ctx.pModel);
      const res = linearOverlay(ctx, c, A, B);
      res.yAll = [showTh ? res.yAll[0] : null, showPh ? res.yAll[1] : null];
      res.y = res.yAll[0]; res.yMeasAll = res.yAll; res.yMeas = res.y;
      return res;
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Input τ(t)', 'p. 77');
      inputControls(sec, ctx);
      segmented(sec, {
        label: 'Dashed overlay',
        options: [{ value: 'cascade', label: 'cascade approximation (d–e)' }, { value: 'none', label: 'none' }],
        ...bind(ctx, 'overlay'),
      });
      sec.append(el('p', { class: 'muted small', text: 'The dashed traces are the cascade approximation of parts (d) and (e) driven by the same torque. Compare them with the full model, and vary Jp/Js in the left panel.'
        + (ctx.S.mode === 'work' ? ' In Work mode the θ trace appears once (d) is solved and the φ trace once (e) is solved.' : '') }));
    },
    // Poles and zeros of Θ/τ answer C.5(b).
    splane(ctx) {
      let sp = { markers: [] };
      if (showsAnswer(ctx, 'C.5/b')) {
        const d = tfData(ctx.pModel);
        L.roots(d.den).forEach((q, i) => sp.markers.push({ ...q, kind: 'ol', label: `pole of Θ/τ ${i + 1}` }));
        L.roots(d.num).forEach((q) => sp.markers.push({ ...q, kind: 'olzero', label: 'zero of Θ/τ (= pole of Φ/Θ)' }));
      }
      for (const [id, names] of Object.entries(TF_MARKERS)) {
        const y = Y.data(ctx, id);
        if (y) sp = Y.withMarkers(sp, (y.G ? [y.G] : [y.GT, y.GP]).flatMap((G, i) => tfMarkers(G, names[i])), { obs: 'your poles', zero: 'your zeros' });
      }
      return sp;
    },
    // The student's transfer functions driven from rest by the same τ (a transfer
    // function assumes zero initial conditions).
    outputSeries(ctx, res, sc, oi) {
      const out = [];
      for (const [id, list] of Object.entries(TF_OVERLAYS)) {
        const y = Y.data(ctx, id);
        if (!y) continue;
        list.filter(([o]) => o === oi).forEach(([, label, G, more]) => out.push(yours(label, sc(Y.tfResponse(ctx, G(ctx, y), fin(ctx))), sc(res.yAll[oi]), more)));
      }
      return out;
    },
    math(ctx) {
      const p = ctx.pModel, d = tfData(p);
      return [
        { title: 'Laplace transform', page: 'p. 69–70',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)' },
        { title: 'Transfer functions of coupled equations', page: 'p. 70–71, p. 78',
          theory: '\\text{zero initial conditions:}\\quad M(s)\\begin{bmatrix}Y_1(s)\\\\ Y_2(s)\\end{bmatrix} = N(s)\\,U(s) \\;\\Rightarrow\\; \\begin{bmatrix}Y_1\\\\ Y_2\\end{bmatrix} = M(s)^{-1}N(s)\\,U(s)' },
        { title: 'Equations of motion (C.3)', page: 'p. 53 · Eq. 3.3', answers: 'C.3/d',
          theory: 'J_s\\ddot\\theta + b(\\dot\\theta - \\dot\\phi) + k(\\theta - \\phi) = \\tau,\\quad J_p\\ddot\\phi + b(\\dot\\phi - \\dot\\theta) + k(\\phi - \\theta) = 0' },
        { title: 'Laplace transform of the satellite EOM', page: 'p. 78 · Eq. 5.3–5.4', answers: 'C.5/a',
          theory: '\\left(s^2 + \\tfrac{b}{J_s}s + \\tfrac{k}{J_s}\\right)\\Theta = \\left(\\tfrac{b}{J_s}s + \\tfrac{k}{J_s}\\right)\\Phi + \\tfrac{1}{J_s}\\tau,\\quad \\left(s^2 + \\tfrac{b}{J_p}s + \\tfrac{k}{J_p}\\right)\\Phi = \\left(\\tfrac{b}{J_p}s + \\tfrac{k}{J_p}\\right)\\Theta' },
        { title: 'Transfer matrix of the satellite', page: 'p. 78–79 · Eq. 5.6', answers: 'C.5/b',
          theory: '\\frac{\\Theta}{\\tau} = \\frac{\\frac{1}{J_s}s^2 + \\frac{b}{J_sJ_p}s + \\frac{k}{J_sJ_p}}{s^2\\left(s^2 + \\frac{b(J_s+J_p)}{J_sJ_p}s + \\frac{k(J_s+J_p)}{J_sJ_p}\\right)},\\quad \\frac{\\Phi}{\\tau} = \\frac{\\frac{b}{J_sJ_p}s + \\frac{k}{J_sJ_p}}{s^2\\left(s^2 + \\cdots\\right)}',
          numbers: `\\frac{\\Theta}{\\tau} = \\frac{${WB.tf.polyTex(d.num)}}{${WB.tf.polyTex(d.den)}}` },
        { title: 'Panel subsystem', page: 'p. 78 · Eq. 5.5', answers: 'C.5/c',
          theory: '\\frac{\\Phi(s)}{\\Theta(s)} = \\frac{\\Phi/\\tau}{\\Theta/\\tau} = \\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}',
          numbers: `\\frac{\\Phi}{\\Theta} = \\frac{${WB.tf.polyTex(d.out.num)}}{${WB.tf.polyTex(d.out.den)}}` },
        { title: 'Body subsystem, (Js + Jp)/Js ≈ 1', page: 'p. 79', answers: 'C.5/d',
          theory: '\\frac{\\Theta}{\\tau} = \\frac{\\frac{1}{J_s}\\left(s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}\\right)}{s^2\\frac{J_s+J_p}{J_s}\\left(\\frac{J_s}{J_s+J_p}s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}\\right)} \\approx \\frac{1}{(J_s + J_p)s^2}',
          numbers: `\\frac{\\Theta}{\\tau} \\approx \\frac{1}{${tex(d.J)}\\,s^2},\\quad \\frac{J_s + J_p}{J_s} = ${tex(d.J / p.Js)}`,
          note: 'The cancellation is only approximate; compare the poles and zeros in the s-plane.' },
        { title: 'Cascade approximation', page: 'p. 79 · Fig. 5-3', answers: 'C.5/e',
          theory: '\\tau \\to \\boxed{\\frac{1}{(J_s+J_p)s^2}} \\to \\Theta \\to \\boxed{\\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}} \\to \\Phi',
          note: 'The body moves like a rigid Js + Jp and drives the panel through the spring and damper; the panel barely pushes back when Jp ≪ Js.' },
      ];
    },
    buildProblem(parent, ctx) {
      const d = () => tfData(ctx.pModel);
      const tfPart = (fn, truth, comment) => pyPart(ctx, { items: [{ fn, args: ['s'], truth: (p, a) => truth(p, a.s) }] }, `def ${fn}(s):\n    # ${comment}\n    return ...\n`);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'a', title: '(a) Laplace transform of the equations of motion',
          html: 'Zero initial conditions. Write each transformed equation as c<sub>Θ</sub>Θ(s) + c<sub>Φ</sub>Φ(s) = c<sub>τ</sub>τ(s) and return (c<sub>Θ</sub>, c<sub>Φ</sub>, c<sub>τ</sub>) at a complex s: <code>eq_theta</code> from the θ equation, <code>eq_phi</code> from the φ equation. Any nonzero multiple of an equation is accepted.',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'eq_theta', args: ['s'], compare: 'scale', truth: (p, a) => { const g = cx.poly([p.b / p.Js, p.k / p.Js], a.s); return [cx.poly([1, p.b / p.Js, p.k / p.Js], a.s), cx.mul(-1, g), 1 / p.Js]; } },
              { fn: 'eq_phi', args: ['s'], compare: 'scale', truth: (p, a) => { const g = cx.poly([p.b / p.Jp, p.k / p.Jp], a.s); return [cx.mul(-1, g), cx.poly([1, p.b / p.Jp, p.k / p.Jp], a.s), 0]; } },
            ],
          }, '# (c_Theta, c_Phi, c_tau) of\n# c_Theta*Theta + c_Phi*Phi = c_tau*tau\ndef eq_theta(s):\n    return ...\n\ndef eq_phi(s):\n    return ...\n'), {
            actions: [{ label: 'Plot my equations', run: (code) => Y.plot(ctx, 'ch5.a', code, runEquations, tfDescribe('The Θ/τ and Φ/τ your two equations imply', 'their responses to the same τ, from rest, are dotted on the θ and φ plots.')) }],
          }),
          solution: () => [
            { tex: '\\left(s^2 + \\tfrac{b}{J_s}s + \\tfrac{k}{J_s}\\right)\\Theta - \\left(\\tfrac{b}{J_s}s + \\tfrac{k}{J_s}\\right)\\Phi = \\tfrac{1}{J_s}\\tau' },
            { tex: '-\\left(\\tfrac{b}{J_p}s + \\tfrac{k}{J_p}\\right)\\Theta + \\left(s^2 + \\tfrac{b}{J_p}s + \\tfrac{k}{J_p}\\right)\\Phi = 0' },
            { code: 'def eq_theta(s):\n    g = (P.b * s + P.k) / P.Js\n    return np.array([s**2 + g, -g, 1 / P.Js])\n\ndef eq_phi(s):\n    g = (P.b * s + P.k) / P.Jp\n    return np.array([-g, s**2 + g, 0])' },
            { html: 'Book: Eqs. 5.3–5.4 (p. 78).' },
          ],
        },
        {
          id: 'b', title: '(b) Transfer matrix from τ(s) to Θ(s) and Φ(s)',
          html: 'Return Θ(s)/τ(s) and Φ(s)/τ(s) evaluated at a complex s (the check calls them at several).',
          code: Object.assign(pyPart(ctx, {
            items: [
              { fn: 'theta_over_tau', args: ['s'], truth: (p, a) => tfAt.thetaTau(p, a.s) },
              { fn: 'phi_over_tau', args: ['s'], truth: (p, a) => tfAt.phiTau(p, a.s) },
            ],
          }, 'def theta_over_tau(s):\n    return ...\n\ndef phi_over_tau(s):\n    return ...\n'), {
            actions: [{ label: 'Plot my transfer matrix', run: (code) => Y.plot(ctx, 'ch5.b', code, runTfMatrix, tfDescribe('Your Θ/τ and Φ/τ', 'their responses to the same τ, from rest, are dotted on the θ and φ plots.')) }],
          }),
          solution: () => [
            { tex: '\\begin{bmatrix}\\Theta\\\\ \\Phi\\end{bmatrix} = \\frac{1}{s^2\\left(s^2 + \\frac{b(J_s+J_p)}{J_sJ_p}s + \\frac{k(J_s+J_p)}{J_sJ_p}\\right)}\\begin{bmatrix}\\frac{1}{J_s}s^2 + \\frac{b}{J_sJ_p}s + \\frac{k}{J_sJ_p}\\\\ \\frac{b}{J_sJ_p}s + \\frac{k}{J_sJ_p}\\end{bmatrix}\\tau' },
            { tex: `\\frac{\\Theta}{\\tau} = \\frac{${WB.tf.polyTex(d().num)}}{${WB.tf.polyTex(d().den)}}` },
            { code: 'def den(s):\n    Js, Jp = P.Js, P.Jp\n    return s**2 * (s**2\n        + P.b * (Js + Jp) / (Js * Jp) * s\n        + P.k * (Js + Jp) / (Js * Jp))\n\ndef theta_over_tau(s):\n    Js, Jp = P.Js, P.Jp\n    num = (s**2 / Js + P.b / (Js * Jp) * s\n           + P.k / (Js * Jp))\n    return num / den(s)\n\ndef phi_over_tau(s):\n    Js, Jp = P.Js, P.Jp\n    num = (P.b * s + P.k) / (Js * Jp)\n    return num / den(s)' },
            { html: 'Book: p. 78 and Eq. 5.6 (p. 79). Invert the 2×2 matrix of part (a).' },
          ],
        },
        {
          id: 'c', title: '(c) Transfer function from Θ(s) to Φ(s)',
          code: Object.assign(tfPart('phi_over_theta', tfAt.phiTheta, 'Phi(s)/Theta(s) at complex s'), {
            actions: [{ label: 'Plot my Φ/Θ', run: (code) => Y.plot(ctx, 'ch5.c', code, runTf('phi_over_theta'), tfDescribe('Your Φ/Θ', 'driven by the satellite\'s θ (from rest), its φ is dotted on the φ plot.')) }],
          }),
          solution: () => [
            { tex: `\\frac{\\Phi}{\\Theta} = \\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}} = \\frac{${WB.tf.polyTex(d().out.num)}}{${WB.tf.polyTex(d().out.den)}}` },
            { code: 'def phi_over_theta(s):\n    g = (P.b * s + P.k) / P.Jp\n    return g / (s**2 + g)' },
            { html: 'Book: Eq. 5.5 (p. 78): divide Φ/τ by Θ/τ.' },
          ],
        },
        {
          id: 'd', title: '(d) Second-order approximation of Θ(s)/τ(s)',
          html: 'Assume (J<sub>s</sub> + J<sub>p</sub>)/J<sub>s</sub> ≈ 1.',
          code: Object.assign(tfPart('theta_over_tau_approx', tfAt.thetaTauApprox, 'approximate Theta(s)/tau(s)'), {
            actions: [{ label: 'Plot my approximate Θ/τ', run: (code) => Y.plot(ctx, 'ch5.d', code, runTf('theta_over_tau_approx'), tfDescribe('Your approximate Θ/τ', 'its response to the same τ, from rest, is dotted on the θ plot, with the dashed cascade approximation and the full model.')) }],
          }),
          solution: () => [
            { tex: `\\frac{\\Theta}{\\tau} \\approx \\frac{1}{(J_s + J_p)s^2} = \\frac{1}{${tex(d().J)}\\,s^2}` },
            { code: 'def theta_over_tau_approx(s):\n    return 1 / ((P.Js + P.Jp) * s**2)' },
            { html: 'Book: p. 79. Factor (J<sub>s</sub> + J<sub>p</sub>)/J<sub>s</sub> out of the denominator; with J<sub>s</sub>/(J<sub>s</sub> + J<sub>p</sub>) ≈ 1 the remaining quadratic cancels the numerator.' },
          ],
        },
        {
          id: 'e', title: '(e) The approximate cascade, and why it makes sense',
          html: 'Return the cascade Φ(s)/τ(s) built from parts (c) and (d). Then justify it physically, on paper.',
          code: Object.assign(tfPart('phi_over_tau_approx', (p, s) => cx.mul(tfAt.thetaTauApprox(p, s), tfAt.phiTheta(p, s)), 'cascade Phi(s)/tau(s)'), {
            actions: [{ label: 'Plot my cascade', run: (code) => Y.plot(ctx, 'ch5.e', code, runTf('phi_over_tau_approx'), tfDescribe('Your cascade Φ/τ', 'its response to the same τ, from rest, is dotted on the φ plot, with the dashed cascade approximation and the full model.')) }],
          }),
          solution: () => [
            { tex: '\\frac{\\Phi}{\\tau} \\approx \\frac{1}{(J_s + J_p)s^2}\\cdot\\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}' },
            { code: 'def phi_over_tau_approx(s):\n    g = (P.b * s + P.k) / P.Jp\n    body = 1 / ((P.Js + P.Jp) * s**2)\n    return body * g / (s**2 + g)' },
            { html: 'Book: p. 79 and Fig. 5-3. The body behaves like a rigid J<sub>s</sub> + J<sub>p</sub> and drives the panel through the spring and damper. When J<sub>p</sub> ≪ J<sub>s</sub> the panel barely pushes back on the body, so the coupling runs one way: body → panel.' },
          ],
        },
      ]);
    },
  });

  // C.6: the student's A, B, C, D.
  async function runSS(ctx, code) {
    const out = await WB.py.evaluate(code, [{ params: ctx.pModel, vars: ['A', 'B', 'C', 'D'] }]);
    if (out.error) return Y.pyError(out);
    const v = out.rows[0].vars;
    const A = Y.asMat(v.A, 4, 4), B = Y.asMat(v.B, 4, 1), C = Y.asMat(v.C, 2, 4), D = Y.asMat(v.D, 2, 1);
    const bad = [[A, 'A', '4×4'], [B, 'B', '4×1'], [C, 'C', '2×4'], [D, 'D', '2×1']].find(([m]) => !m);
    if (bad) return { ok: false, msg: `${bad[1]} should be a ${bad[2]} array of numbers (x = (θ, φ, θ̇, φ̇), u = τ, y = (θ, φ − θ)).` };
    return { A, B, C, D };
  }

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
      sec.append(el('p', { class: 'muted small', text: 'The satellite is linear, so ẋ = Ax + Bu reproduces the simulation exactly. The dashed trace only separates when you add plant mismatch, saturation or a disturbance. The lower plot shows the strain-gauge output of C.6.' }));
    },
    splane(ctx) {
      const sp = lib().showsOl(ctx)
        ? { markers: L.eig(lib().ss(ctx.pModel).A).map((q, i) => ({ ...q, kind: 'ol', label: `eigenvalue of A ${i + 1}` })) }
        : { markers: [] };
      const y = Y.data(ctx, 'ch6.a');
      return y ? Y.withMarkers(sp, Y.eig(y.A, 'eigenvalue of your A'), { obs: 'eigenvalue of your A' }) : sp;
    },
    // Your model from the simulation's initial state, driven by the same τ.
    yourRun(ctx, res) {
      const y = Y.data(ctx, 'ch6.a');
      return y ? Y.linResponse(ctx, y, res.x[0], fin(ctx)) : null;
    },
    // θ plot: your y₁; φ plot: your x₂ (the model's φ).
    outputSeries(ctx, res, sc, oi) {
      const r = this.yourRun(ctx, res);
      if (!r) return [];
      return [oi === 0 ? yours('your y₁', sc(r.y[0]), sc(res.yAll[0])) : yours('your x₂', sc(r.x.map((x) => x[1])), sc(res.yAll[1]))];
    },
    extraPlot(ctx, res) {
      const y2 = res.x.map((x) => (x[1] - x[0]) * R2D);
      const series = [{ label: 'y₂ = φ − θ (strain gauge)', y: y2, color: '--series-1' }];
      const r = this.yourRun(ctx, res);
      if (r) series.push(yours('your y₂', r.y[1].map((v) => v * R2D), y2));
      return { opts: { title: 'measured outputs of C.6', yLabel: 'y [°]', unit: '°' }, data: { series } };
    },
    math(ctx) {
      const { A, B, Cbook } = lib().ss(ctx.pModel);
      return [
        { title: 'States, input and measured outputs', page: 'p. 91',
          theory: 'x = (\\theta, \\phi, \\dot\\theta, \\dot\\phi)^\\top,\\quad u = \\tau,\\quad y = (\\theta,\\; \\phi - \\theta)^\\top' },
        { title: 'State-space model', page: 'p. 83–84',
          theory: '\\dot x = Ax + Bu,\\quad y = Cx + Du,\\quad A = \\frac{\\partial f}{\\partial x},\\; B = \\frac{\\partial f}{\\partial u},\\; C = \\frac{\\partial h}{\\partial x},\\; D = \\frac{\\partial h}{\\partial u}',
          note: 'For a linear model, each row of A and B is read off the matching state equation.' },
        { title: 'State-space model of the satellite', page: 'p. 92–93', answers: 'C.6/a',
          theory: 'A = \\begin{bmatrix}0 & 0 & 1 & 0\\\\ 0 & 0 & 0 & 1\\\\ -\\frac{k}{J_s} & \\frac{k}{J_s} & -\\frac{b}{J_s} & \\frac{b}{J_s}\\\\ \\frac{k}{J_p} & -\\frac{k}{J_p} & \\frac{b}{J_p} & -\\frac{b}{J_p}\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\0\\\\ \\frac{1}{J_s}\\\\ 0\\end{bmatrix},\\quad C = \\begin{bmatrix}1 & 0 & 0 & 0\\\\ -1 & 1 & 0 & 0\\end{bmatrix},\\quad D = 0',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(Cbook)}`,
          note: 'The later chapters and _C_satellite/python measure y = (θ, φ) instead.' },
        { title: 'Transfer function from the state space', page: 'p. 93',
          theory: 'H(s) = C(sI - A)^{-1}B + D,\\quad \\text{poles: } \\det(sI - A) = 0' },
        { title: 'Characteristic polynomial of the satellite', page: 'p. 93', answers: 'C.6/a',
          theory: `\\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}` },
      ];
    },
    buildProblem(parent, ctx) {
      const ss = (p) => { const s = lib().ss(p); return { A: s.A, B: s.B, C: s.Cbook, D: [[0], [0]] }; };
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'A, B, C, D',
        html: 'Module-level variables, with the state, input and output ordered as in the problem.',
        code: Object.assign(pyPart(ctx, {
          items: ['A', 'B', 'C', 'D'].map((k) => ({ var: k, truth: (p) => ss(p)[k] })),
        }, '# x = (theta, phi, thetadot, phidot)\n# u = tau, y = (theta, phi - theta)\nA = ...\nB = ...\nC = ...\nD = ...\n'), {
          actions: [{ label: 'Plot my model', run: (code) => Y.plot(ctx, 'ch6.a', code, runSS, () => 'Your eigenvalues of A are the × markers on the s-plane. Your model\'s y₁, x₂ and y₂, from the same initial state and τ, are dotted on the θ, φ and strain-gauge plots.') }],
        }),
        solution: () => { const { A, B } = ss(ctx.pModel); return [
          { tex: 'A = \\begin{bmatrix}0 & 0 & 1 & 0\\\\ 0 & 0 & 0 & 1\\\\ -\\frac{k}{J_s} & \\frac{k}{J_s} & -\\frac{b}{J_s} & \\frac{b}{J_s}\\\\ \\frac{k}{J_p} & -\\frac{k}{J_p} & \\frac{b}{J_p} & -\\frac{b}{J_p}\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\0\\\\ \\frac{1}{J_s}\\\\ 0\\end{bmatrix}' },
          { tex: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = \\begin{bmatrix}1 & 0 & 0 & 0\\\\ -1 & 1 & 0 & 0\\end{bmatrix},\\quad D = \\begin{bmatrix}0\\\\ 0\\end{bmatrix}` },
          { code: 'Js, Jp, k, b = P.Js, P.Jp, P.k, P.b\nA = np.array([\n    [0, 0, 1, 0],\n    [0, 0, 0, 1],\n    [-k / Js, k / Js, -b / Js, b / Js],\n    [k / Jp, -k / Jp, b / Jp, -b / Jp]])\nB = np.array([[0], [0], [1 / Js], [0]])\nC = np.array([[1, 0, 0, 0],\n              [-1, 1, 0, 0]])\nD = np.array([[0], [0]])' },
          { html: 'Book: pp. 92–93. Note that the C.11 and C.12 solutions print different numbers for the same A (see ISSUES.md).' },
        ]; },
      }]);
    },
  });

  WB.studies.C.models = { tfData, tfAt };
})();
