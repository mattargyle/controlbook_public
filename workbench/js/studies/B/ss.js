// Design Study B, Chapters 11–14: full state feedback, integral augmentation,
// observers, and disturbance observers on the linearized pendulum (Eq. 6.17):
//   ẋ = A x + B F,  y = (z, θ) = C x,  x = (z, θ, ż, θ̇).
// The closed-loop poles are a θ pair (t_r,θ, ζ_θ) and a slower z pair
// (t_r,z = M t_r,θ, ζ_z), plus p_I and the observer poles in later chapters.
// Gains are placed with the port of scipy's place_poles (core/place_yt.js), so the
// two-output observer gains match the repo's ctrlObserver.py exactly.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const B = WB.studies.B;
  const lib = () => B.lib;
  const PD = () => WB.pd;
  const DEG = Math.PI / 180;

  const KEYS = ['K1', 'K2', 'K3', 'K4'];
  // Python answers: module-level matrices, checked at random parameter sets.
  const pyVars = (ctx, names, truth) => (code) => WB.py.check(ctx, { items: names.map((k) => ({ var: k, truth: (p) => truth(p)[k] })) }, code);
  // Every pole in `given` matches a distinct pole in `truth` (1%).
  function polesMatchAll(given, truth) {
    const left = truth.slice();
    for (const g of given) {
      const i = left.findIndex((q) => M.close(g.re, q.re, 0.01, 1e-3) && M.close(g.im, q.im, 0.01, 1e-3));
      if (i < 0) return false;
      left.splice(i, 1);
    }
    return left.length === 0;
  }
  const tuning = (st) => ({ trTh: st.trTh, zetaTh: st.zetaTh, M: st.M, zetaZ: st.zetaZ, rule: st.rule, pI: st.pI, obsFactor: st.obsFactor, obsRule: st.obsRule, pD: st.pD });

  // Gains designed from the knobs. Work mode has no gains of its own: the plots
  // run the student's controller, and the cards that show these are locked.
  function gainsFor(ctx, level) { return lib().ssDesign(ctx.ss, level, tuning(ctx.st)); }

  // Solution controllers: the repo's ctrlStateFeedback, ctrlStateFeedbackIntegrator,
  // ctrlObserver and ctrlDisturbanceObserver, plus anti-windup from B.12 on.
  const SOL = {
    ch11: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr_th = 0.5
        zeta = 0.707
        M = 2.5              # t_r,z = M t_r,theta: faster than B.8
        d = 0.25 * P.m1 + P.m2
        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [0.0, -0.75 * P.m1 * P.g / d, -P.b / d, 0.0],
                      [0.0, 1.5 * (P.m1 + P.m2) * P.g / (d * P.ell), 1.5 * P.b / (d * P.ell), 0.0]])
        B = np.array([[0.0], [0.0], [1 / d], [-1.5 / (d * P.ell)]])
        C = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        wn_th = 2.2 / tr_th
        wn_z = 2.2 / (M * tr_th)
        poles = np.roots(np.convolve([1, 2 * zeta * wn_z, wn_z**2], [1, 2 * zeta * wn_th, wn_th**2]))
        self.K = cnt.place(A, B, poles)
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
        self.kr = -1.0 / (Cr @ np.linalg.inv(A - B @ self.K) @ B)[0, 0]

    def update(self, z_r, x):
        F = -(self.K @ x)[0, 0] + self.kr * z_r
        return max(-P.F_max, min(P.F_max, F))
`,
    ch12: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr_th = 0.5          # the B.11 poles (B.8 values, t_z = 3 t_theta)
        zeta = 0.707
        M = 3.0
        p_I = -2.0
        d = 0.25 * P.m1 + P.m2
        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [0.0, -0.75 * P.m1 * P.g / d, -P.b / d, 0.0],
                      [0.0, 1.5 * (P.m1 + P.m2) * P.g / (d * P.ell), 1.5 * P.b / (d * P.ell), 0.0]])
        B = np.array([[0.0], [0.0], [1 / d], [-1.5 / (d * P.ell)]])
        C = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
        A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        wn_th = 2.2 / tr_th
        wn_z = 2.2 / (M * tr_th)
        poles = np.roots(np.convolve(np.convolve([1, 2 * zeta * wn_z, wn_z**2], [1, 2 * zeta * wn_th, wn_th**2]), [1, -p_I]))
        K1 = cnt.place(A1, B1, poles)
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, z_r, x):
        error = z_r - x[0, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        F = -(self.K @ x)[0, 0] - self.ki * integ
        # anti-windup: hold the integrator while F saturates
        if abs(F) <= P.F_max:
            self.integrator = integ
        self.error_prev = error
        F = -(self.K @ x)[0, 0] - self.ki * self.integrator
        return max(-P.F_max, min(P.F_max, F))
`,
    ch13: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr_th = 0.5          # the repo's tuning (ctrlObserver.py)
        zeta = 0.9
        M = 3.0
        p_I = -2.0
        d = 0.25 * P.m1 + P.m2
        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [0.0, -0.75 * P.m1 * P.g / d, -P.b / d, 0.0],
                      [0.0, 1.5 * (P.m1 + P.m2) * P.g / (d * P.ell), 1.5 * P.b / (d * P.ell), 0.0]])
        B = np.array([[0.0], [0.0], [1 / d], [-1.5 / (d * P.ell)]])
        C = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        self.A, self.B, self.C = A, B, C
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
        A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        wn = lambda tr: 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        pair = lambda tr: [1, 2 * zeta * wn(tr), wn(tr)**2]
        poles = np.roots(np.convolve(np.convolve(pair(M * tr_th), pair(tr_th)), [1, -p_I]))
        K1 = cnt.place(A1, B1, poles)
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        # observer 10x faster (rise time / 10 for each pair)
        obs_poles = np.roots(np.convolve(pair(M * tr_th / 10), pair(tr_th / 10)))
        self.L = cnt.place(A.T, C.T, obs_poles).T
        self.x_hat = np.zeros((4, 1))
        self.F_prev = 0.0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, z_r, y):
        x_hat = self.update_observer(y)
        error = z_r - x_hat[0, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        F = -(self.K @ x_hat)[0, 0] - self.ki * integ
        if abs(F) <= P.F_max:          # anti-windup
            self.integrator = integ
        self.error_prev = error
        F = -(self.K @ x_hat)[0, 0] - self.ki * self.integrator
        F = max(-P.F_max, min(P.F_max, F))
        self.F_prev = F
        return F, x_hat

    def update_observer(self, y):
        F1 = self.observer_f(self.x_hat, y)
        F2 = self.observer_f(self.x_hat + P.Ts / 2 * F1, y)
        F3 = self.observer_f(self.x_hat + P.Ts / 2 * F2, y)
        F4 = self.observer_f(self.x_hat + P.Ts * F3, y)
        self.x_hat = self.x_hat + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y):
        return self.A @ x_hat + self.B * self.F_prev + self.L @ (y - self.C @ x_hat)
`,
    ch14: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr_th = 0.5          # the repo's tuning (ctrlDisturbanceObserver.py)
        zeta = 0.9
        M = 3.0
        p_I = -2.0
        p_d = -1.0           # disturbance-estimate pole
        d = 0.25 * P.m1 + P.m2
        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [0.0, -0.75 * P.m1 * P.g / d, -P.b / d, 0.0],
                      [0.0, 1.5 * (P.m1 + P.m2) * P.g / (d * P.ell), 1.5 * P.b / (d * P.ell), 0.0]])
        B = np.array([[0.0], [0.0], [1 / d], [-1.5 / (d * P.ell)]])
        C = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
        A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        wn = lambda tr: 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        pair = lambda tr: [1, 2 * zeta * wn(tr), wn(tr)**2]
        poles = np.roots(np.convolve(np.convolve(pair(M * tr_th), pair(tr_th)), [1, -p_I]))
        K1 = cnt.place(A1, B1, poles)
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        # observer for (x, d) with d constant
        self.A2 = np.block([[A, B], [np.zeros((1, 5))]])
        self.B2 = np.vstack([B, [[0.0]]])
        self.C2 = np.hstack([C, np.zeros((2, 1))])
        wn_obs = lambda tr: 2.2 / tr
        pair_obs = lambda tr: [1, 2 * zeta * wn_obs(tr), wn_obs(tr)**2]
        obs_poles = np.roots(np.convolve(np.convolve(pair_obs(M * tr_th / 10), pair_obs(tr_th / 10)), [1, -p_d]))
        self.L2 = cnt.place(self.A2.T, self.C2.T, obs_poles).T
        self.obs = np.zeros((5, 1))
        self.F_prev = 0.0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, z_r, y):
        self.update_observer(y)
        x_hat = self.obs[0:4]
        d_hat = self.obs[4, 0]
        error = z_r - x_hat[0, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        F = -(self.K @ x_hat)[0, 0] - self.ki * integ - d_hat
        if abs(F) <= P.F_max:          # anti-windup
            self.integrator = integ
        self.error_prev = error
        F = -(self.K @ x_hat)[0, 0] - self.ki * self.integrator - d_hat
        F = max(-P.F_max, min(P.F_max, F))
        self.F_prev = F
        return F, x_hat, d_hat

    def update_observer(self, y):
        F1 = self.observer_f(self.obs, y)
        F2 = self.observer_f(self.obs + P.Ts / 2 * F1, y)
        F3 = self.observer_f(self.obs + P.Ts / 2 * F2, y)
        F4 = self.observer_f(self.obs + P.Ts * F3, y)
        self.obs = self.obs + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def observer_f(self, xo, y):
        return self.A2 @ xo + self.B2 * self.F_prev + self.L2 @ (y - self.C2 @ xo)
`,
  };

  const SQUARE = (amplitude, frequency = 0.04) => ({ type: 'square', amplitude, frequency, tStep: 0 });
  const misText = (m) => Object.entries(m).map(([k, v]) => `${k} ${v > 0 ? '+' : ''}${v}%`).join(', ');
  // |z − z_r| (m) at time t.
  const errAt = (res, t) => { const k = Math.round(t / (res.t[1] - res.t[0])); return Math.abs(res.rAll[0][k] - res.yAll[0][k]); };
  const maxTheta = (res) => res.x.reduce((m, x) => Math.max(m, Math.abs(x[1])), 0);
  // The student's estimate must come back from update as (F, x_hat[, d_hat]).
  function needXhat(res) {
    const ok = [0, 1, 2, 3].every((i) => res.extras[`xhat${i}`] && Array.prototype.every.call(res.extras[`xhat${i}`], Number.isFinite));
    return ok ? null : { ok: false, msg: 'Return (F, x_hat) from update, with x_hat = [[z_hat], [theta_hat], [zdot_hat], [thetadot_hat]], so the check can see your estimate.' };
  }
  // Largest |x_i − x̂_i| over the given time windows.
  function estErr(res, windows) {
    const e = [0, 0, 0, 0];
    res.t.forEach((t, k) => {
      if (!windows.some(([a, b]) => t >= a - 1e-9 && t <= b + 1e-9)) return;
      for (let i = 0; i < 4; i++) e[i] = Math.max(e[i], Math.abs(res.x[k][i] - res.extras[`xhat${i}`][k]));
    });
    return e;
  }
  const meanBias = (res, i, t0, t1) => WB.myCtrl.mean(res, t0, t1, (k) => res.x[k][i] - res.extras[`xhat${i}`][k]);
  // B.12(a) windup probes. The pendulum can't stay up through seconds of
  // saturation (it falls with or without anti-windup), so the check feeds a fresh
  // controller a fixed state for 4 s with z_r = 1 m, which saturates F while the
  // error persists, then reverses z_r: F must come off the limit within 0.2 s. The
  // state is held at rest (catches a held integrator or back-calculation), or
  // moving at ż = 1 m/s (catches integrating only while |ż| is small).
  const WINDUP = { hold: 400, after: 100, maxSat: 20, zdots: [0, 1] };
  async function windupCheck(ctx, code) {
    const lim = ctx.sys.uLimit(ctx.pModel);
    const runs = [];
    for (const v of WINDUP.zdots) {
      const calls = [...Array.from({ length: WINDUP.hold }, () => [1, [0, 0, v, 0]]), ...Array.from({ length: WINDUP.after }, () => [-1, [0, 0, 0, 0]])];
      const pb = await WB.myCtrl.probe(ctx, code, calls);
      if (pb.ok === false) return pb;
      let n = 0;
      for (let k = WINDUP.hold; k < calls.length && Math.abs(pb.u[k][0]) >= lim * (1 - 1e-6); k++) n++;
      runs.push(n);
    }
    return { n: Math.min(...runs), runs };
  }

  function makeCtrl(ctx, level, { linear = false } = {}) {
    const g = ctx.gains, st = ctx.st, p = ctx.pModel, Ts = ctx.S.sim.Ts, uLim = ctx.sys.uLimit(p);
    const { A, B: Bm, C } = ctx.ss;
    if (level === 'sf') return lib().sfCtrl({ K: g.K, kr: g.kr, uLim, linear });
    if (level === 'sfi') return lib().sfiCtrl({ K: g.K, ki: g.ki, uLim, Ts, antiwindup: st.antiwindup, linear });
    const xh0 = [st.xhat0.z, st.xhat0.th * DEG, 0, 0];
    if (level === 'dobs' && st.dobs) {
      const { A2, C2, B2: B1 } = lib().augD(ctx.ss);
      return lib().obsCtrl({ Aobs: A2, Bobs: B1, Cobs: C2, L: g.L2, K: g.K, ki: g.ki, Ts, uLim, dist: true, xhat0: [...xh0, 0], antiwindup: st.antiwindup, linear });
    }
    return lib().obsCtrl({ Aobs: A, Bobs: Bm, Cobs: C, L: g.L, K: g.K, ki: g.ki, Ts, uLim, xhat0: xh0, antiwindup: st.antiwindup, linear });
  }

  // ------------------------------------------------------------- analysis --
  function clPoles(ctx, level) {
    const { A, B: Bm } = ctx.ss, g = ctx.gains;
    if (level === 'sf') return L.eig(L.sub(A, L.mul(Bm, [g.K])));
    const { A1, B1 } = lib().augI(ctx.ss);
    return L.eig(L.sub(A1, L.mul(B1, [[...g.K, g.ki]])));
  }
  function obsPolesOf(ctx, level) {
    const { A, C } = ctx.ss, g = ctx.gains;
    if (level === 'obs' || (level === 'dobs' && !ctx.st.dobs)) return L.eig(L.sub(A, L.mul(g.L, C)));
    if (level === 'dobs') { const { A2, C2 } = lib().augD(ctx.ss); return L.eig(L.sub(A2, L.mul(g.L2, C2))); }
    return [];
  }
  // Group a designed pole list into drag ids: θ pair 0, z pair 1, p_I 2 (observer +10).
  function tagged(d, off) {
    const out = [];
    const n = d.poles.length;
    d.poles.forEach((p, i) => out.push({ ...p, dragId: off + (i < 2 ? 1 : i < 4 ? 0 : 2) }));
    return n ? out : [];
  }

  function markers(ctx, level, prob) {
    const { A } = ctx.ss, g = ctx.gains, explore = ctx.S.mode === 'explore';
    // Work mode: the open-loop poles (eig A) answer B.6 until it is solved.
    const mk = explore || ctx.app.isSolved('B.6/a') ? L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole ${i + 1}` })) : [];
    const names = { 0: 'θ-pair pole', 1: 'z-pair pole', 2: 'integrator pole' };
    if (explore) {
      tagged({ poles: g.poles }, 0).forEach((p) => mk.push({ ...p, kind: 'cl', label: `controller ${names[p.dragId]}` }));
      if (level === 'obs' || level === 'dobs') {
        const op = ctx.st.dobs || level === 'obs' ? g.obsPoles : g.obsPoles.slice(0, 4);
        tagged({ poles: op }, 10).forEach((p) => mk.push({ ...p, kind: 'obs', label: `observer ${p.dragId === 12 ? 'disturbance pole' : names[p.dragId - 10]}`, noFit: Math.hypot(p.re, p.im) > 80 }));
      }
    } else {
      // Work mode has no gains (the plots run the student's own controller). The
      // target poles (B.11(a)) are an answer: shown once revealed or solved.
      if (polesRevealed(ctx) || ctx.app.isSolved('B.11/a')) {
        const t = lib().ssPoles({ ...prob, pI: level === 'sf' ? null : prob.pI });
        t.poles.forEach((p) => mk.push({ ...p, kind: 'target', label: 'target pole (problem)' }));
      }
    }
    return mk;
  }

  const polesRevealed = (ctx) => !!(ctx.app && ctx.app.isRevealed(`B:${ctx.S.chapter}:poles`));
  function revealPoles(parent, ctx, what) {
    const btn = WB.ui.revealButton(ctx, `B:${ctx.S.chapter}:poles`, `Reveal ${what} in the s-plane`);
    parent.append(btn);
    WB.ui.addRefresher(() => { btn.hidden = polesRevealed(ctx); });
  }

  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    const wn = Math.hypot(re, im);
    const zeta = Math.max(0.2, Math.min(0.99, -re / wn));
    const trFrom = WB.design.trFromWn;
    if (id === 0) { st.zetaTh = zeta; st.trTh = Math.max(0.02, trFrom(wn, zeta, st.rule)); }
    else if (id === 1) { st.zetaZ = zeta; st.M = Math.max(1.2, Math.min(50, trFrom(wn, zeta, st.rule) / st.trTh)); }
    else if (id === 2) st.pI = re;
    else if (id === 10 || id === 11) {
      const base = id === 10 ? WB.design.wnFromTr(st.trTh, st.zetaTh, st.obsRule) : WB.design.wnFromTr(st.trTh * st.M, st.zetaZ, st.obsRule);
      st.obsFactor = Math.max(1, Math.min(60, wn / base));
    } else if (id === 12) st.pD = re;
    ctx.update();
  }

  // ------------------------------------------------------------- controls --
  function knobs(parent, ctx, { pI = false } = {}) {
    slider(parent, { label: 't<sub>r,θ</sub>', unit: 's', min: 0.05, max: 2, step: 0.005, ...bind(ctx, 'trTh') });
    slider(parent, { label: 'ζ<sub>θ</sub>', min: 0.2, max: 0.99, step: 0.005, ...bind(ctx, 'zetaTh') });
    slider(parent, { label: 'M = t<sub>r,z</sub>/t<sub>r,θ</sub>', min: 1.2, max: 20, step: 0.1, sig: 3, ...bind(ctx, 'M') });
    slider(parent, { label: 'ζ<sub>z</sub>', min: 0.2, max: 0.99, step: 0.005, ...bind(ctx, 'zetaZ') });
    segmented(parent, {
      label: 'ω<sub>n</sub> from t<sub>r</sub>',
      options: [{ value: '2.2', label: '2.2 / t<sub>r</sub> (B.8)' }, { value: 'tp', label: 'π/(2t<sub>r</sub>√(1−ζ²)) (listings)' }],
      ...bind(ctx, 'rule'),
    });
    if (pI) slider(parent, { label: 'p<sub>I</sub>', min: -15, max: -0.05, step: 0.01, sig: 3, ...bind(ctx, 'pI') });
  }
  function presets(parent, ctx, prob) {
    const row = el('div', { class: 'btn-row' });
    if (prob.book) row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Book text (B.8 poles)', onclick: () => { Object.assign(ctx.st, prob.book); ctx.update(); } }));
    if (prob.repo) row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Repo listing', onclick: () => { Object.assign(ctx.st, prob.repo); ctx.update(); } }));
    parent.append(row);
  }
  // Explore only (Work mode has no observer of its own).
  function obsKnobs(parent, ctx, withD) {
    slider(parent, { label: 'speed factor', unit: '×', min: 1, max: 40, step: 0.1, sig: 3, hint: 't_r,obs = t_r / factor for each pair (repo: 10)', ...bind(ctx, 'obsFactor') });
    segmented(parent, {
      label: 'observer ω<sub>n</sub> rule',
      options: [{ value: '2.2', label: '2.2 / t<sub>r</sub>' }, { value: 'tp', label: 'π/(2t<sub>r</sub>√(1−ζ²))' }],
      ...bind(ctx, 'obsRule'),
    });
    if (withD) slider(parent, { label: 'p<sub>d</sub>', min: -30, max: -0.05, step: 0.01, sig: 3, hint: 'disturbance-estimate pole (repo: −1)', ...bind(ctx, 'pD') });
  }
  function xhatKnobs(parent, ctx) {
    slider(parent, { label: 'ẑ(0)', unit: 'm', min: -1, max: 1, step: 0.01, sig: 3, ...bind(ctx, 'z', () => ctx.st.xhat0) });
    slider(parent, { label: 'θ̂(0)', unit: '°', min: -20, max: 20, step: 0.5, sig: 3, hint: 'initial estimate (the true state starts at the left-panel values)', ...bind(ctx, 'th', () => ctx.st.xhat0) });
  }
  function readout(parent, ctx, keys) {
    WB.ui.readout(parent, () => {
      const g = ctx.gains;
      const vals = { K1: g.K[0], K2: g.K[1], K3: g.K[2], K4: g.K[3], kr: g.kr, ki: g.ki };
      return keys.map((k) => [k, vals[k]]);
    });
  }

  // ----------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B: Bm } = ctx.ss;
    return {
      title: 'Linearized state-space model (B.6)', page: 'p. 90 · Eq. 6.17, p. 189 · Eq. 11.39', answers: 'B.6/a',
      theory: '\\dot x = Ax + BF,\\quad y = \\begin{pmatrix}1&0&0&0\\\\0&1&0&0\\end{pmatrix}x,\\quad x = (z, \\theta, \\dot z, \\dot\\theta)^\\top,\\quad y_r = (1, 0, 0, 0)\\,x',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(Bm)}`,
      note: 'Eq. 11.39 and 12.2 print row 4 for a different ℓ (ISSUES.md). The numbers here use the current ℓ.',
    };
  }
  const ctrbCard = (A, Bm, title, page) => WB.ss.ctrbCard(A, Bm, title, page, { det: true });
  function polesCard(ctx, d, level, answers) {
    const st = ctx.st;
    return {
      title: 'Desired closed-loop poles', page: 'p. 189 · B.11(a), p. 207–208', answers,
      theory: (st.rule === 'tp' ? '\\omega_n = \\frac{\\pi}{2t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}') + ',\\quad t_{r,z} = M\\,t_{r,\\theta},\\quad \\Delta^d_{cl} = (s^2 + 2\\zeta_\\theta\\omega_{n\\theta}s + \\omega_{n\\theta}^2)(s^2 + 2\\zeta_z\\omega_{nz}s + \\omega_{nz}^2)' + (level === 'sf' ? '' : '(s - p_I)'),
      numbers: `\\omega_{n\\theta} = ${tex(d.wnTh)},\\; \\omega_{nz} = ${tex(d.wnZ)},\\quad \\Delta^d_{cl} = ${WB.tf.polyTex(L.polyFromRoots(d.poles))}`, spoiler: true,
    };
  }

  // --------------------------------------------------------- chapter base --
  function base(level, num, title, pages, extra) {
    return Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages, level,
      gains(ctx) { return gainsFor(ctx, level); },
      controller(ctx, o) { return makeCtrl(ctx, level, o); },
      // Work mode simulates the student's controller (WB.myCtrl): fed the state in
      // Ch 11-12, as Listings 11.2 and 12.2 are, and the measured z, θ in Ch 13-14.
      implement: { feed: level === 'sf' || level === 'sfi' ? 'state' : 'y', linear: false },
      linearSim(ctx, c) {
        const { A, B: Bm, C } = ctx.ss;
        return WB.sim.simulate({ ...c, disturbance: null, noise: null, plant: WB.design.linearPlant(A, Bm, C), controller: makeCtrl(ctx, level, { linear: true }) });
      },
      splane(ctx) { return { markers: markers(ctx, level, this.prob(ctx)) }; },
      onPoleDrag: onDrag,
      targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.trTh * ctx.st.M } : {}; },
      prob(ctx) { const pr = ctx.sys.problems[`ch${num}`]; return pr.book || pr.repo; },
    }, extra);
  }
  const stateDefaults = (p) => ({ trTh: p.trTh, zetaTh: p.zetaTh, M: p.M, zetaZ: p.zetaZ, rule: p.rule, pI: p.pI ?? -2, obsFactor: p.obsFactor ?? 10, obsRule: p.obsRule ?? 'tp', pD: p.pD ?? -1, xhat0: { z: 0, th: 0 } });

  // Estimates: the workbench's observer (Explore) or what the student's update
  // returns as x_hat, d_hat (Work: extras xhat0..xhat3, dhat).
  const EST = { zhat: 'xhat0', thhat: 'xhat1', zdhat: 'xhat2', thdhat: 'xhat3', dhat: 'dhat' };
  const est = (ctx, res, key) => res.extras[ctx.S.mode === 'work' ? EST[key] : key] || [];
  function estSeries(ctx, res, sc, oi) {
    const work = ctx.S.mode === 'work';
    return [{ label: `${oi === 0 ? 'ẑ' : 'θ̂'} (${work ? 'your estimate' : 'observer'})`, y: sc(est(ctx, res, oi === 0 ? 'zhat' : 'thhat')), color: '--series-3', dash: [3, 3], width: 2 }];
  }
  function estExtra(ctx, res) {
    const k = 180 / Math.PI, S = ctx.S;
    res = { ...res, extras: Object.fromEntries(Object.keys(EST).map((key) => [key, est(ctx, res, key)])) };
    if (ctx.st.extra === 'd') {
      return { opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N]', unit: 'N' }, data: { series: [
        { label: 'd̂ (estimate)', y: Array.from(res.extras.dhat || []), color: '--series-3', width: 2 },
        { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
      ] } };
    }
    if (ctx.st.extra === 'zd') {
      return { opts: { title: 'ż and its estimate', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [
        { label: 'ż̂ (observer)', y: Array.from(res.extras.zdhat || []), color: '--series-3', dash: [3, 3], width: 2 },
        { label: 'true ż', y: res.x.map((x) => x[2]), color: '--series-1' },
      ] } };
    }
    return { opts: { title: 'θ̇ and its estimate', yLabel: 'θ̇ [°/s]', unit: '°/s' }, data: { series: [
      { label: 'θ̇̂ (observer)', y: Array.from(res.extras.thdhat || [], (v) => v * k), color: '--series-3', dash: [3, 3], width: 2 },
      { label: 'true θ̇', y: res.x.map((x) => x[3] * k), color: '--series-1' },
    ] } };
  }
  // Work-mode control panel: the plots show the student's controller.
  function workPanel(parent, ctx, part, what) {
    WB.myCtrl.banner(section(parent, 'Your controller'), ctx, part);
    revealPoles(section(parent, 'Target poles'), ctx, what);
  }

  // ------------------------------------------------------------ Chapter 11 --
  B.chapters.ch11 = base('sf', 11, 'Full state feedback', 'pp. 173–196', {
    defaults(sys) { return { ...stateDefaults(sys.problems.ch11.book), extra: 'thd' }; },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') workPanel(parent, ctx, 'B.11(e)', 'the B.11(a) target poles');
      else {
        const sec = section(parent, 'F = −Kx + k_r z_r', 'p. 175 · Eq. 11.3, p. 190');
        knobs(sec, ctx); presets(sec, ctx, ctx.sys.problems.ch11);
        sec.append(el('p', { class: 'muted small', text: 'Drag a θ-pair (fast) or z-pair (slow) pole.' }));
        readout(sec, ctx, [...KEYS, 'kr']);
      }
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'zd', label: 'ż' }, { value: 'thd', label: 'θ̇' }], ...bind(ctx, 'extra') });
    },
    extraPlot(ctx, res) {
      const zd = ctx.st.extra === 'zd';
      return { opts: { title: zd ? 'ż(t)' : 'θ̇(t)', yLabel: zd ? 'ż [m/s]' : 'θ̇ [°/s]', unit: zd ? 'm/s' : '°/s' }, data: { series: [{ label: zd ? 'ż' : 'θ̇', y: res.x.map((x) => (zd ? x[2] : x[3] * 180 / Math.PI)), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = lib().ssDesign(ctx.ss, 'sf', tuning(ctx.st)), g = ctx.gains;
      const { A, B: Bm } = ctx.ss;
      return [
        ssCard(ctx),
        { ...ctrbCard(A, Bm, 'Controllability', 'p. 190 · Step 1'), answers: ['B.6/a', 'B.11/c'] },
        { title: 'Open-loop characteristic polynomial', page: 'p. 190 · Step 2',
          theory: '\\Delta_{ol}(s) = \\det(sI - A),\\quad \\mathbf a_A = (a_{n-1}, \\dots, a_0)',
          numbers: `\\Delta_{ol} = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true, answers: 'B.6/a' },
        polesCard(ctx, d, 'sf', ctx.S.mode === 'work' ? 'B.11/a' : undefined),
        { title: 'Gains (Ackermann / place)', page: 'p. 190 · Step 4, Eq. 11.32',
          theory: 'K = (\\boldsymbol\\alpha - \\mathbf a_A)\\,\\mathcal{A}_A^{-1}\\mathcal{C}_{A,B}^{-1},\\quad k_r = \\frac{-1}{C_r(A - BK)^{-1}B}',
          numbers: `K = ${texMat([d.K])},\\quad k_r = ${tex(d.kr)}`, spoiler: true, answers: 'B.11/d' },
        { title: 'Control law', page: 'p. 191 · Listing 11.2', answers: 'B.11/e',
          theory: 'F = -Kx + k_r z_r',
          numbers: ctx.S.mode === 'work' ? null : `F = -(${g.K.map((k, i) => `${tex(k)}\\,${['z', '\\theta', '\\dot z', '\\dot\\theta'][i]}`).join(' + ')}) + ${tex(g.kr)}\\,z_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch11;
      const ref = () => lib().ssDesign(ctx.ss, 'sf', { ...pr.book });
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Desired closed-loop poles',
          html: 'Use ζ<sub>z</sub>, ω<sub>n<sub>θ</sub></sub> and ζ<sub>θ</sub> from B.8 (t<sub>r<sub>θ</sub></sub> = 0.5 s, ζ = 0.707, ω<sub>n</sub> = 2.2/t<sub>r</sub>) and an ω<sub>n<sub>z</sub></sub> that gives t<sub>z</sub> = 3t<sub>θ</sub>. Complex values are fine: <code>-1+2j</code>.',
          inputs: { p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>', p3: 'p<sub>3</sub>', p4: 'p<sub>4</sub>' },
          check: (v) => {
            const g = ['p1', 'p2', 'p3', 'p4'].map((k) => M.parseComplex(v[k]));
            if (g.some((x) => !x)) return { ok: false, msg: 'Enter all four poles.' };
            return polesMatchAll(g, ref().poles) ? { ok: true, msg: '' } : { ok: false, msg: 'Roots of (s² + 2ζ_θω_nθ s + ω_nθ²)(s² + 2ζ_zω_nz s + ω_nz²)?' };
          },
          solution: () => { const r = ref(); return [{ tex: `\\omega_{n\\theta} = 2.2/0.5 = ${tex(r.wnTh)},\; \\omega_{nz} = 2.2/1.5 = ${tex(r.wnZ)},\\quad p = ${r.poles.map((x) => texPole(x)).join(',\;')}` }, { html: 'These give the book\'s Δ<sup>d</sup> = s⁴ + 8.2955s³ + 34.414s² + 53.533s + 41.646 (p. 190). The listing instead uses ζ = 0.9, t<sub>z</sub> = 5t<sub>θ</sub> and the π/(2t<sub>r</sub>√(1−ζ²)) rule.' }]; },
        },
        {
          id: 'b', title: '(b) Add A, B, C, D from B.6 to the parameter file',
          html: 'Use your A, B, C, D from B.6 (the Ch 6 tab). The model card in the Math section unlocks once B.6 is solved.',
        },
        {
          id: 'c', title: '(c) Controllability',
          inputs: { rank: 'rank 𝒞', det: 'det 𝒞' },
          check: (v) => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return PD().checkNumbers(v, { rank: L.rank(Cm), det: L.det(Cm) }, { det: 'det' }); },
          solution: () => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return [{ tex: `\\mathcal{C}_{A,B} = ${texMat(Cm)},\; \\det = ${tex(L.det(Cm))} \\ne 0` }, { html: 'The book prints det = −6104.1 (p. 190): that is the ℓ = 0.5 m value.' }]; },
        },
        {
          id: 'd', title: '(d) K and k<sub>r</sub> for the (a) poles',
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], K3: r.K[2], K4: r.K[3], kr: r.kr }, {}); },
          solution: () => { const r = ref(); return [{ tex: `K = ${texMat([r.K])},\\quad k_r = ${tex(r.kr)}` }, { html: 'The book prints K = (−1.5050, −24.9399, −1.9847, −3.5829), k<sub>r</sub> = −1.5050 (p. 190). Those are exactly the gains for ℓ = 0.5 m; with the stated ℓ = 1 m the answer is the one above. Note k<sub>r</sub> = K<sub>1</sub>: the plant has a free integrator in z, so unity DC gain needs the reference to enter like the z feedback.' }]; },
        },
        WB.myCtrl.part(ctx, {
          id: 'e', title: '(e) Implement state feedback and tune the poles for a good response', seed: 'B.8/e',
          html: '<code>update(z_r, x)</code> gets the state, as in Listing 11.2. The check runs the ±0.5 m square wave (0.04 Hz) with the nominal parameters: the first step must rise (10–90%) in under 1.2 s, |θ| must stay under 30°, and F may sit at F<sub>max</sub> for at most 2 samples over the whole run (the later switches are 1 m steps). B.8\'s successive-loop design takes about 3.9 s.',
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.5), tEnd: 25 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const i1 = Math.round(12.5 / sc.Ts);
            const m = M.stepMetrics(res.t, res.yAll[0], 0, i1, res.yAll[0][0], res.rAll[0][1]);
            const lim = ctx.sys.uLimit(ctx.pModel);
            let nAt = 0;
            for (const u of res.uDemandAll[0]) if (Math.abs(u) >= lim * (1 - 1e-6)) nAt++;
            const th = maxTheta(res);
            const ok = m.tr < 1.2 && th < 30 * DEG && nAt <= 2;
            return { ok, msg: `Rise time ${fmt(m.tr, 3)} s, max |θ| ${fmt(th / DEG, 3)}°, F at the limit for ${nAt} sample${nAt === 1 ? '' : 's'}.` };
          },
          solution: () => [
            { code: SOL.ch11 },
            { html: 'As the repo\'s ctrlStateFeedback.py (Listing 11.2), with t<sub>r,θ</sub> = 0.5 s, M = 2.5, ζ = 0.707 and the 2.2/t<sub>r</sub> rule: the first step rises in 1.02 s, and the largest demand, on the 1 m steps, is 4.3 N. Faster designs saturate there: t<sub>r,θ</sub> = 0.4 s, M = 3 rises in 0.95 s but demands 7.4 N at t = 12.5 s.' },
          ],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 12 --
  B.chapters.ch12 = base('sfi', 12, 'Integrator with state feedback', 'pp. 197–214', {
    defaults(sys) { return { ...stateDefaults(sys.problems.ch12.book), antiwindup: 'clamp', extra: 'int' }; },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workPanel(parent, ctx, 'B.12(a) and (c)', 'the target poles (p_I = −2)'); return; }
      const sec = section(parent, 'F = −Kx − k_I ∫(z_r − z)', 'p. 199 · Eq. 12.1, p. 210');
      segmented(sec, {
        label: 'Anti-windup (B.12a)',
        options: [{ value: 'clamp', label: 'hold integrator while F saturates' }, { value: 'none', label: 'none (listing)' }],
        ...bind(ctx, 'antiwindup'),
      });
      knobs(sec, ctx, { pI: true }); presets(sec, ctx, ctx.sys.problems.ch12);
      readout(sec, ctx, [...KEYS, 'ki']);
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'int', label: 'integrator' }, { value: 'thd', label: 'θ̇' }], ...bind(ctx, 'extra') });
    },
    extraPlot(ctx, res) {
      // the student's controller reports no integrator: Work mode plots θ̇
      if (ctx.st.extra === 'int' && ctx.S.mode !== 'work') return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [m·s]', unit: 'm·s' }, data: { series: [{ label: 'x_I = ∫(z_r − z)', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
      return { opts: { title: 'θ̇(t)', yLabel: 'θ̇ [°/s]', unit: '°/s' }, data: { series: [{ label: 'θ̇', y: res.x.map((x) => x[3] * 180 / Math.PI), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = lib().ssDesign(ctx.ss, 'sfi', tuning(ctx.st));
      const { A1, B1 } = lib().augI(ctx.ss);
      return [
        ssCard(ctx),
        { title: 'Integral augmentation', page: 'p. 198 · Eq. 12.1',
          theory: '\\dot x_I = r - C_r x,\\quad A_1 = \\begin{pmatrix}A & 0\\\\ -C_r & 0\\end{pmatrix},\\quad B_1 = \\begin{pmatrix}B\\\\ 0\\end{pmatrix}' },
        { title: 'Augmented pendulum model', page: 'p. 207 · Step 1', answers: 'B.12/a1',
          theory: 'C_r = (1, 0, 0, 0)',
          numbers: `A_1 = ${texMat(A1)},\\quad B_1 = ${texMat(B1)}` },
        { ...ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 207 · Step 2'), answers: 'B.12/a1' },
        polesCard(ctx, d, 'sfi', ctx.S.mode === 'work' ? 'B.11/a' : undefined),
        { title: 'Gains', page: 'p. 208 · Step 3',
          theory: 'K_1 = (K, k_I) = (\\boldsymbol\\alpha - \\mathbf a_{A_1})\\mathcal{A}_{A_1}^{-1}\\mathcal{C}_{A_1,B_1}^{-1},\\quad F = -Kx - k_I x_I',
          numbers: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}`, spoiler: true, answers: 'B.12/a2' },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch12;
      const ref = () => lib().ssDesign(ctx.ss, 'sfi', { ...pr.book });
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a1', title: '(a) Integrator on the position: augmented system',
          html: 'Module-level numpy arrays A1, B1 of the system augmented with x<sub>I</sub> = ∫(z<sub>r</sub> − z) (state (x, x<sub>I</sub>)). Write A and B from B.6 first.',
          code: {
            template: '# x1 = (z, theta, zdot, thetadot, xI)\nA1 = ...\nB1 = ...\n',
            check: pyVars(ctx, ['A1', 'B1'], (p) => lib().augI(ctx.sys.stateSpace(p))),
          },
          solution: () => [
            { tex: 'A_1 = \\begin{pmatrix}A & 0\\\\ -C_r & 0\\end{pmatrix},\\quad B_1 = \\begin{pmatrix}B\\\\ 0\\end{pmatrix},\\quad C_r = (1, 0, 0, 0)' },
            { code: 'd = 0.25 * P.m1 + P.m2\nA = np.array([\n    [0, 0, 1, 0],\n    [0, 0, 0, 1],\n    [0, -0.75 * P.m1 * P.g / d,\n     -P.b / d, 0],\n    [0, 1.5 * (P.m1 + P.m2) * P.g / (d * P.ell),\n     1.5 * P.b / (d * P.ell), 0]])\nB = np.array([[0], [0], [1 / d],\n              [-1.5 / (d * P.ell)]])\nCr = np.array([[1, 0, 0, 0]])\nA1 = np.block([[A, np.zeros((4, 1))],\n               [-Cr, np.zeros((1, 1))]])\nB1 = np.vstack([B, [[0]]])' },
            { html: 'Book: Step 1 (p. 207).' },
          ],
        },
        {
          id: 'a2', title: '(a) K and k<sub>I</sub> for the B.11 poles with p<sub>I</sub> = −2',
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', ki: 'k<sub>I</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], K3: r.K[2], K4: r.K[3], ki: r.ki }, { ki: 'kI' }); },
          solution: () => { const r = ref(); return [{ tex: `K = ${texMat([r.K])},\\quad k_I = ${tex(r.ki)}` }, { html: 'The book prints K = (−5.3744, −32.1057, −4.5745, −5.1545), k<sub>I</sub> = 3.0101 (p. 208): the ℓ = 0.5 m gains. Its Step 2 text names p<sub>I</sub> = −10 and different B.11 poles, but the polynomial it then uses has (s + 2) and the B.8 poles.' }]; },
        },
        WB.myCtrl.part(ctx, {
          id: 'a3', title: '(a) Add the integrator with anti-windup to your B.11 controller', seed: 'B.11/e',
          html: `Use the gains from above. The check (1) runs a ±0.2 m square wave with the nominal and with other parameters and compares z(t) with the design (within 3%), then (2) feeds your controller a fixed state for ${WINDUP.hold / 100} s with z<sub>r</sub> = 1 m, so F saturates while the error persists (once with the cart at rest, once moving at ż = 1 m/s), and reverses z<sub>r</sub>: in one of the two, F must come off the limit within ${WINDUP.maxSat / 100} s.`,
          check: async (code) => {
            const cases = WB.myCtrl.paramCases(ctx).map((pc) => {
              const sc = WB.myCtrl.scenario(ctx, { params: pc.params, ref: SQUARE(0.2), tEnd: 20, feed: 'state' });
              const d = lib().ssDesign(ctx.sys.stateSpace(sc.params), 'sfi', { ...pr.book });
              return { sc, label: pc.label, ref: () => WB.myCtrl.reference(ctx, sc, lib().sfiCtrl({ K: d.K, ki: d.ki, uLim: ctx.sys.uLimit(sc.params), Ts: sc.Ts, antiwindup: 'clamp' })) };
            });
            const m = await WB.myCtrl.matchCheck(ctx, code, cases, { tol: 0.03 * 0.2 });
            if (!m.ok) return m;
            const w = await windupCheck(ctx, code);
            if (w.ok === false) return w;
            if (!(w.n <= WINDUP.maxSat)) return { ok: false, msg: `Tracking matches, but after ${WINDUP.hold / 100} s of saturation F stays at the limit for ${fmt(w.n / 100, 3)} s once z_r reverses: the integrator winds up while F is saturated.` };
            return { ok: true, msg: `${m.msg} After the saturated hold F leaves the limit after ${w.n} sample${w.n === 1 ? '' : 's'}.` };
          },
          solution: () => [
            { code: SOL.ch12 },
            { html: 'The repo\'s ctrlStateFeedbackIntegrator.py (which has no anti-windup) with the B.11 poles and p<sub>I</sub> = −2, holding the integrator while F saturates. Unwinding by (F<sub>sat</sub> − F<sub>unsat</sub>)/k<sub>I</sub> passes too. Integrating only while |ż| &lt; v̄ passes only with v̄ above the cart\'s speed in ordinary tracking (about 0.24 m/s on the ±0.2 m steps): with B.10\'s v̄ = 0.07 m/s the integrator is off for most of every step, and the response is no longer the design\'s.' },
          ],
        }),
        {
          id: 'b', title: '(b) Constant input disturbance of 0.5 N, parameters vary by up to 20%',
          html: 'Set in the left panel (the chapter defaults: d = 0.5 N from t = 0, a fixed 20% draw of the true plant).',
          check: () => {
            const mis = Object.values(ctx.S.mismatch || {}).some((v) => Math.abs(v) > 0);
            const d = ctx.S.sim.dist;
            if (Math.abs(d - 0.5) > 1e-9) return { ok: false, msg: `Set the disturbance to 0.5 N (now ${fmt(d, 3)} N).` };
            return mis ? { ok: true, msg: 'd = 0.5 N and the true plant differs from the model.' } : { ok: false, msg: 'The true plant equals the model. Randomize it (α = 0.2).' };
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Tune the integrator pole (and other gains if needed) for good tracking', seed: ['B.12/a3', 'B.11/e'],
          html: `The check runs the ±0.5 m square wave with d = ${pr.sim.dist} N and the plant off by ${misText(pr.mismatch)}: |z − z<sub>r</sub>| just before the first switch (t = 12.5 s) must be under 1 cm.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.5), tEnd: 12.5, dist: pr.sim.dist, mismatch: pr.mismatch, feed: 'state' });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const e = errAt(res, 12.45);
            return { ok: e < 0.01, msg: `|z − z_r| = ${fmt(e * 100, 3)} cm just before the first switch (max |θ| ${fmt(maxTheta(res) / DEG, 3)}°).` };
          },
          solution: () => [{ code: SOL.ch12 }, { html: `The design from (a) (p<sub>I</sub> = ${pr.book.pI}) already passes: the integrator removes the error the constant disturbance and the mismatch leave. A faster p<sub>I</sub> removes it sooner but couples into the θ loop.` }],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 13 --
  // Settled windows before the square wave's switches (0.05 Hz: t = 10, 20 s),
  // where the estimates are compared: the observer uses the previous sample's
  // force, so every switch starts a short estimation transient.
  const OBS_WINDOWS = [[8, 9.95], [18, 19.95]];
  B.chapters.ch13 = base('obs', 13, 'Observers', 'pp. 215–238', {
    defaults(sys) { return { ...stateDefaults(sys.problems.ch13.repo), antiwindup: 'none', extra: 'thd' }; },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries: estSeries,
    extraPlot: estExtra,
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') workPanel(parent, ctx, 'B.13(c)', 'the target poles');
      else {
        const sec = section(parent, 'Controller (uses x̂)', 'p. 231 · Listing 13.2');
        knobs(sec, ctx, { pI: true }); presets(sec, ctx, ctx.sys.problems.ch13);
        const ob = section(parent, 'Observer', 'p. 216 · Eq. 13.3');
        obsKnobs(ob, ctx, false);
        ob.append(el('p', { class: 'muted small', text: 'Two outputs make L (4 × 2) non-unique; this page designs it with scipy\'s YT algorithm, as control.place uses.' }));
        readout(ob, ctx, [...KEYS, 'ki']);
        xhatKnobs(ob, ctx);
      }
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'zd', label: 'ż, ż̂' }, { value: 'thd', label: 'θ̇, θ̇̂' }], ...bind(ctx, 'extra') });
    },
    math(ctx) {
      const g = ctx.gains, d = lib().ssDesign(ctx.ss, 'obs', tuning(ctx.st));
      const { A, C } = ctx.ss;
      const O = L.obsv(A, C);
      return [
        ssCard(ctx),
        { title: 'Observability', page: 'p. 220–221, B.13(b)',
          theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ CA^2 \\\\ CA^3\\end{bmatrix} \\in \\mathbb{R}^{8\\times4},\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = 4',
          numbers: `\\operatorname{rank}\\mathcal{O}_{A,C} = ${L.rank(O)}`, spoiler: true, answers: 'B.13/b' },
        { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 232', answers: 'B.13/c',
          theory: '\\dot{\\hat x} = A\\hat x + BF + L(y - C\\hat x),\\quad \\dot e = (A - LC)e,\\quad F \\text{ is the saturated force of the previous sample}' },
        { title: 'Observer poles and gain', page: 'p. 232 · Listing 13.2',
          theory: 't_{r,obs} = t_r / 10 \\text{ for each pair (repo)},\\quad L = \\text{place}(A^\\top, C^\\top, q)^\\top',
          numbers: `\\omega_{n\\theta,obs} = ${tex(g.wnThObs)},\; \\omega_{nz,obs} = ${tex(g.wnZObs)},\\quad L^\\top = ${texMat(L.T(g.L))}`, spoiler: true, answers: 'B.13/c',
          note: 'With two outputs many L place the same poles; this is the one scipy\'s YT iteration returns.' },
        polesCard(ctx, d, 'sfi'),
        { title: 'Separation principle', page: 'p. 222–223',
          theory: '\\begin{pmatrix}\\dot x\\\\ \\dot e\\end{pmatrix} = \\begin{pmatrix}A - BK & BK\\\\ 0 & A - LC\\end{pmatrix}\\begin{pmatrix}x\\\\ e\\end{pmatrix} \\Rightarrow \\text{eig} = \\text{eig}(A - BK)\\cup\\text{eig}(A - LC)',
          note: 'Holds for the linear model only; saturation and mismatch break it.' },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch13;
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Exact parameters, no input disturbance',
          html: 'The chapter starts this way: no plant mismatch (α = 0) and d = 0 in the left panel.',
          check: () => {
            const mis = Object.values(ctx.S.mismatch || {}).some((v) => Math.abs(v) > 0);
            if (mis) return { ok: false, msg: 'Clear the plant mismatch (α = 0).' };
            if (Math.abs(ctx.S.sim.dist) > 0) return { ok: false, msg: 'Set the disturbance to 0.' };
            return { ok: true, msg: 'The controller knows the true parameters, and d = 0.' };
          },
        },
        {
          id: 'b', title: '(b) Observability',
          inputs: { rank: 'rank 𝒪<sub>A,C</sub>' },
          check: (v) => PD().checkNumbers(v, { rank: L.rank(L.obsv(ctx.ss.A, ctx.ss.C)) }, {}),
          solution: () => [{ tex: '\\operatorname{rank}\\mathcal{O}_{A,C} = 4 = n' }, { html: 'Measuring z and θ makes every state observable.' }],
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Add an observer and use x̂ in your B.12 controller; tune the controller and observer poles', seed: ['B.12/c', 'B.12/a3'],
          html: '<code>update(z_r, y)</code> now gets the measured y = [[z], [θ]]; return <code>(F, x_hat)</code>. With two outputs L is not unique, so the check looks at your estimates, not at L. The check starts the rod at θ = 5° (your estimate starts wherever you start it) and runs the ±0.5 m square wave (0.05 Hz) with exact parameters: over the 2 s before each switch, |z − ẑ| must stay under 1 mm and |θ − θ̂| under 0.05°, and |z − z<sub>r</sub>| just before the first switch under 1 cm.',
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.5, 0.05), init: { z0: 0, theta0: 5 }, tEnd: 20 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const ee = estErr(res, OBS_WINDOWS), e = errAt(res, 9.95);
            const msg = `Before the switches: |z − ẑ| ≤ ${fmt(ee[0] * 1000, 3)} mm, |θ − θ̂| ≤ ${fmt(ee[1] / DEG, 3)}°; |z − z_r| before the first switch ${fmt(e * 100, 3)} cm.`;
            return { ok: ee[0] < 1e-3 && ee[1] < 0.05 * DEG && e < 0.01, msg };
          },
          solution: () => [
            { code: SOL.ch13 },
            { html: 'As the repo\'s ctrlObserver.py (Listing 13.2: ζ = 0.9, t<sub>z</sub> = 3t<sub>θ</sub>, the observer 10× faster, RK4 with the previous saturated F), plus the B.12 anti-windup. The repo\'s L comes from scipy\'s place; any L that places the observer poles works, and this one keeps the z and θ error pairs separate.' },
          ],
        }),
        {
          id: 'd', title: '(d) Plot the states and their estimates',
          html: 'Run your controller: the z and θ plots show your ẑ and θ̂ with the true states, and the extra plot your ż̂ or θ̇̂.',
        },
        {
          id: 'e', title: '(e) Add an input disturbance of 0.05 N',
          html: 'Runs your controller from (c) with d = 0.05 N and exact parameters. Your observer has no model of d, so x̂ is biased, and the integrator, which acts on z<sub>r</sub> − ẑ, leaves z off z<sub>r</sub>. Passes when the run shows the bias (Chapter 14 removes it).',
          check: async () => {
            const code = WB.myCtrl.savedCode(ctx, 'B.13/c');
            if (!code) return { ok: false, msg: 'Write your controller in B.13(c) first.' };
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.5, 0.05), tEnd: 30, dist: 0.05 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const bz = meanBias(res, 0, 28, 29.95), bth = meanBias(res, 1, 28, 29.95);
            const ez = WB.myCtrl.mean(res, 28, 29.95, (k) => res.yAll[0][k] - res.rAll[0][k]);
            const msg = `With d = 0.05 N, over the 2 s before t = 30 s: z − ẑ = ${fmt(bz * 1000, 3)} mm, θ − θ̂ = ${fmt(bth / DEG, 3)}°, z − z_r = ${fmt(ez * 1000, 3)} mm.`;
            if (Math.abs(bz) < 1e-6 && Math.abs(bth) < 1e-5) return { ok: false, msg: `${msg} x̂ follows the true state exactly: is it coming from an observer of the model?` };
            return { ok: true, msg };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 14 --
  B.chapters.ch14 = base('dobs', 14, 'Disturbance observers', 'pp. 239–259', {
    defaults(sys) { return { ...stateDefaults(sys.problems.ch14.repo), antiwindup: 'none', dobs: true, extra: 'd' }; },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries: estSeries,
    extraPlot: estExtra,
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') workPanel(parent, ctx, 'B.14(b)', 'the target poles');
      else {
        const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'p. 250 · Listing 14.4');
        segmented(sec, {
          label: 'Disturbance observer',
          options: [{ value: true, label: 'on' }, { value: false, label: 'off (B.14a)' }],
          ...bind(ctx, 'dobs'),
        });
        knobs(sec, ctx, { pI: true }); presets(sec, ctx, ctx.sys.problems.ch14);
        const ob = section(parent, 'Observer', 'p. 241, p. 252–253');
        obsKnobs(ob, ctx, true);
        readout(ob, ctx, [...KEYS, 'ki']);
        xhatKnobs(ob, ctx);
      }
      const ex = section(parent, 'Extra plot');
      segmented(ex, { options: [{ value: 'd', label: 'd, d̂' }, { value: 'zd', label: 'ż, ż̂' }, { value: 'thd', label: 'θ̇, θ̇̂' }], ...bind(ctx, 'extra') });
    },
    math(ctx) {
      const g = ctx.gains;
      const { A2, C2 } = lib().augD(ctx.ss);
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(F + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0 \\text{ for constant } d' },
        { title: 'Augmented model (ḋ = 0)', page: 'p. 240, Listing 14.4',
          theory: 'A_2 = \\begin{pmatrix}A & B\\\\ 0 & 0\\end{pmatrix},\\quad B_2 = \\begin{pmatrix}B\\\\ 0\\end{pmatrix},\\quad C_2 = \\begin{pmatrix}C & 0\\end{pmatrix}' },
        { title: 'Augmented pendulum model', page: 'p. 252 · Listing 14.4', answers: 'B.14/b1',
          theory: '\\hat x_2 = (\\hat z, \\hat\\theta, \\hat{\\dot z}, \\hat{\\dot\\theta}, \\hat d)',
          numbers: `A_2 = ${texMat(A2)},\\quad \\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}` },
        { title: 'Disturbance observer', page: 'p. 241, p. 253', answers: 'B.14/b',
          theory: '\\dot{\\hat x}_2 = A_2\\hat x_2 + B_1 F + L_2(y - C_2\\hat x_2),\\quad F = -K\\hat x - k_I x_I - \\hat d' },
        { title: 'Observer poles and gain', page: 'p. 252 · Listing 14.4',
          theory: '\\omega_{n,obs} = 2.2/(t_r/10) \\text{ for each pair},\\quad p_d = -1,\\quad L_2 = \\text{place}(A_2^\\top, C_2^\\top, q)^\\top',
          numbers: ctx.st.dobs ? `L_2^\\top = ${texMat(L.T(g.L2))}` : `L^\\top = ${texMat(L.T(g.L))}`, spoiler: true, answers: 'B.14/b',
          note: 'Listing 14.4 switches the observer to the 2.2/t_r rule while the controller keeps π/(2t_r√(1−ζ²)), and its disturbance pole at −1 is slower than every controller pole.' },
        polesCard(ctx, g, 'sfi'),
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch14;
      const noises = [pr.sim.noise, pr.sim.noise];   // σ = 0.001 on z_m [m] and θ_m [rad]
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) α = 0.2, d = 0.5 N, noise σ = 0.001, without a disturbance observer',
          html: `The chapter starts with these in the left panel (the plant off by ${misText(pr.mismatch)}). Run your B.13 controller here to see how it handles the large disturbance: compare ẑ with z.`,
          actions: [{ label: 'Run my B.13 controller', run: () => {
            const code = WB.myCtrl.savedCode(ctx, 'B.13/c');
            if (!code) return { ok: false, msg: 'Write your controller in B.13(c) first (Ch 13 tab).' };
            return WB.myCtrl.use(ctx, code, 'a');
          } }],
        },
        {
          id: 'b1', title: '(b) Add a disturbance observer: augmented model',
          html: 'Model the input disturbance as a constant state d (ḋ = 0) entering with F. Return module-level A2, B2, C2 for the state (x, d). Write A, B, C from B.6 first.',
          code: {
            template: '# x2 = (z, theta, zdot, thetadot, d)\nA2 = ...\nB2 = ...\nC2 = ...\n',
            check: pyVars(ctx, ['A2', 'B2', 'C2'], (p) => lib().augD(ctx.sys.stateSpace(p))),
          },
          solution: () => [
            { tex: 'A_2 = \\begin{pmatrix}A & B\\\\ 0 & 0\\end{pmatrix},\\quad B_2 = \\begin{pmatrix}B\\\\ 0\\end{pmatrix},\\quad C_2 = \\begin{pmatrix}C & 0\\end{pmatrix}' },
            { code: 'd = 0.25 * P.m1 + P.m2\nA = np.array([\n    [0, 0, 1, 0],\n    [0, 0, 0, 1],\n    [0, -0.75 * P.m1 * P.g / d,\n     -P.b / d, 0],\n    [0, 1.5 * (P.m1 + P.m2) * P.g / (d * P.ell),\n     1.5 * P.b / (d * P.ell), 0]])\nB = np.array([[0], [0], [1 / d],\n              [-1.5 / (d * P.ell)]])\nC = np.array([[1, 0, 0, 0],\n              [0, 1, 0, 0]])\nA2 = np.block([[A, B],\n               [np.zeros((1, 5))]])\nB2 = np.vstack([B, [[0]]])\nC2 = np.hstack([C, np.zeros((2, 1))])' },
            { html: 'Book: p. 240 and Listing 14.4 (p. 252).' },
          ],
        },
        WB.myCtrl.part(ctx, {
          id: 'b', title: '(b) Add a disturbance observer, verify that the estimator\'s steady-state error is gone, and tune', seed: ['B.13/c'],
          html: `Return <code>(F, x_hat)</code> (or <code>(F, x_hat, d_hat)</code> to plot your d̂). The check runs the ±0.5 m square wave with the plant off by ${misText(pr.mismatch)}, d = ${pr.sim.dist} N and noise σ = ${pr.sim.noise} on z<sub>m</sub> and θ<sub>m</sub>: the mean of z − ẑ over the 5 s before the second switch (t = 25 s) must be under 0.1 mm, and the mean |z − z<sub>r</sub>| over the 0.5 s before it under 1 cm.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.5), tEnd: 25, dist: pr.sim.dist, mismatch: pr.mismatch, noises, seed: 1 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const b = meanBias(res, 0, 20, 24.95);
            const e = WB.myCtrl.mean(res, 24.45, 24.95, (k) => res.rAll[0][k] - res.yAll[0][k]);
            const th = maxTheta(res);
            const msg = `Mean z − ẑ over 20–25 s: ${fmt(b * 1000, 3)} mm; mean z_r − z before the switch: ${fmt(e * 100, 3)} cm; max |θ| ${fmt(th / DEG, 3)}°.`;
            return { ok: Math.abs(b) < 1e-4 && Math.abs(e) < 0.01 && th < 45 * DEG, msg };
          },
          solution: () => [
            { code: SOL.ch14 },
            { html: 'As the repo\'s ctrlDisturbanceObserver.py (Listing 14.4: the observer pairs at 2.2/(t<sub>r</sub>/10), disturbance pole −1), plus anti-windup. d̂ estimates the input disturbance plus whatever force the model gets wrong (mismatch), so it settles near d but not exactly on it.' },
          ],
        }),
      ]);
    },
  });

  B.ss = { gainsFor, makeCtrl, clPoles, obsPolesOf };
})();
