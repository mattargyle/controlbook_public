// Study E, Chapters 11–14: full state feedback, integral augmentation, observers
// and disturbance observers on the 4-state linear model of E.6
//   x̃ = x − x_e,  F = F_ff + F̃,  ẋ̃ = A x̃ + B F̃,  ỹ = C x̃ = (z̃, θ̃)
// with F_ff = F_e (Jacobian A, as E.6 and E.11 ask) or F_fl(z) (then A₄₁ = 0).
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.E = WB.studies.E || { chapters: {} };

(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const E = WB.E;
  const PD = () => WB.pd;
  const CH = WB.studies.E.chapters;
  const showsAnswer = (ctx, key) => ctx.S.mode === 'explore' || ctx.app.isSolved(key);
  const pid = (ctx, ch) => ctx.sys.problems[ch].id;

  // Solution controllers (sample designs; F = F_e + F̃ on the Jacobian model of E.6).
  const SOL = {
    ch11: String.raw`import control as cnt

class Controller:
    def __init__(self):
        # linearize about x_e = (z_e, 0, 0, 0) with F_e (E.4, E.6)
        self.ze = P.ell / 2
        self.Fe = P.m1 * P.g * self.ze / P.ell + P.m2 * P.g / 2
        Je = P.m2 * P.ell**2 / 3 + P.m1 * self.ze**2
        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [0.0, -P.g, 0.0, 0.0],
                      [-P.m1 * P.g / Je, 0.0, 0.0, 0.0]])
        B = np.array([[0.0], [0.0], [0.0], [P.ell / Je]])
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
        # E.11(a): a fast pair (t_r = 0.5 s) and a slow pair (t_r = 1.5 s), zeta = 0.8
        zeta = 0.8
        wn_th = 2.2 / 0.5
        wn_z = 2.2 / 1.5
        poles = np.roots(np.convolve([1, 2 * zeta * wn_th, wn_th**2],
                                     [1, 2 * zeta * wn_z, wn_z**2]))
        self.K = cnt.place(A, B, poles)
        self.kr = -1.0 / (Cr @ np.linalg.inv(A - B @ self.K) @ B)[0, 0]
        # dirty derivatives for zdot and thetadot (sigma = 0.05)
        sigma = 0.05
        self.beta = (2 * sigma - P.Ts) / (2 * sigma + P.Ts)
        self.gamma = 2 / (2 * sigma + P.Ts)
        self.z_prev = P.z0
        self.theta_prev = P.theta0
        self.zdot = P.zdot0
        self.thetadot = P.thetadot0

    def update(self, z_r, y):
        z = y[0, 0]
        theta = y[1, 0]
        self.zdot = self.beta * self.zdot + self.gamma * (z - self.z_prev)
        self.thetadot = self.beta * self.thetadot + self.gamma * (theta - self.theta_prev)
        self.z_prev = z
        self.theta_prev = theta
        x_tilde = np.array([[z - self.ze], [theta], [self.zdot], [self.thetadot]])
        F = self.Fe - (self.K @ x_tilde)[0, 0] + self.kr * (z_r - self.ze)
        return max(-P.F_max, min(P.F_max, F))
`,
    ch12: String.raw`import control as cnt

class Controller:
    def __init__(self):
        self.ze = P.ell / 2
        self.Fe = P.m1 * P.g * self.ze / P.ell + P.m2 * P.g / 2
        Je = P.m2 * P.ell**2 / 3 + P.m1 * self.ze**2
        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [0.0, -P.g, 0.0, 0.0],
                      [-P.m1 * P.g / Je, 0.0, 0.0, 0.0]])
        B = np.array([[0.0], [0.0], [0.0], [P.ell / Je]])
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
        # augment with the integrator of z_r - z (Eq. 12.1)
        A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        # poles: pairs from t_r = 0.2 s (zeta 0.8) and 0.7 s (zeta 0.85), p_I = -2
        wn_th, zeta_th = 2.2 / 0.2, 0.8
        wn_z, zeta_z = 2.2 / 0.7, 0.85
        p_I = -2.0
        poles = np.roots(np.convolve(np.convolve([1, 2 * zeta_th * wn_th, wn_th**2],
                                                 [1, 2 * zeta_z * wn_z, wn_z**2]), [1, -p_I]))
        K1 = cnt.place(A1, B1, poles)
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        sigma = 0.05
        self.beta = (2 * sigma - P.Ts) / (2 * sigma + P.Ts)
        self.gamma = 2 / (2 * sigma + P.Ts)
        self.z_prev = P.z0
        self.theta_prev = P.theta0
        self.zdot = P.zdot0
        self.thetadot = P.thetadot0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, z_r, y):
        z = y[0, 0]
        theta = y[1, 0]
        self.zdot = self.beta * self.zdot + self.gamma * (z - self.z_prev)
        self.thetadot = self.beta * self.thetadot + self.gamma * (theta - self.theta_prev)
        self.z_prev = z
        self.theta_prev = theta
        x_tilde = np.array([[z - self.ze], [theta], [self.zdot], [self.thetadot]])
        error = z_r - z
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        self.error_prev = error
        F = self.Fe - (self.K @ x_tilde)[0, 0] - self.ki * integ
        # anti-windup: hold the integrator while F saturates
        if abs(F) <= P.F_max:
            self.integrator = integ
        F = self.Fe - (self.K @ x_tilde)[0, 0] - self.ki * self.integrator
        return max(-P.F_max, min(P.F_max, F))
`,
    ch13: String.raw`import control as cnt

class Controller:
    def __init__(self):
        self.ze = P.ell / 2
        self.Fe = P.m1 * P.g * self.ze / P.ell + P.m2 * P.g / 2
        Je = P.m2 * P.ell**2 / 3 + P.m1 * self.ze**2
        self.A = np.array([[0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0],
                           [0.0, -P.g, 0.0, 0.0],
                           [-P.m1 * P.g / Je, 0.0, 0.0, 0.0]])
        self.B = np.array([[0.0], [0.0], [0.0], [P.ell / Je]])
        self.C = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]])
        Cr = self.C[0:1, :]
        A1 = np.block([[self.A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([self.B, [[0.0]]])
        # controller: pairs from t_r = 0.5 s and 1.5 s (zeta 0.8), p_I = -1
        zeta = 0.8
        wn_th, wn_z = 2.2 / 0.5, 2.2 / 1.5
        p_I = -1.0
        char = np.convolve(np.convolve([1, 2 * zeta * wn_th, wn_th**2],
                                       [1, 2 * zeta * wn_z, wn_z**2]), [1, -p_I])
        K1 = cnt.place(A1, B1, np.roots(char))
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        # observer 5x faster than the controller pairs (zeta 0.8)
        wo_th, wo_z = 5 * wn_th, 5 * wn_z
        obs_char = np.convolve([1, 2 * zeta * wo_th, wo_th**2], [1, 2 * zeta * wo_z, wo_z**2])
        self.L = cnt.place(self.A.T, self.C.T, np.roots(obs_char)).T
        self.x_hat = np.zeros((4, 1))       # estimate of x - x_e
        self.F_prev = self.Fe
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, z_r, y):
        x_hat = self.update_observer(y)
        z_hat = x_hat[0, 0] + self.ze
        error = z_r - z_hat
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        self.error_prev = error
        F = self.Fe - (self.K @ x_hat)[0, 0] - self.ki * integ
        if abs(F) <= P.F_max:              # anti-windup
            self.integrator = integ
        F = self.Fe - (self.K @ x_hat)[0, 0] - self.ki * self.integrator
        F = max(-P.F_max, min(P.F_max, F))
        self.F_prev = F
        return F, x_hat + np.array([[self.ze], [0.0], [0.0], [0.0]])

    def update_observer(self, y):
        y_tilde = y - np.array([[self.ze], [0.0]])
        F1 = self.observer_f(self.x_hat, y_tilde)
        F2 = self.observer_f(self.x_hat + P.Ts / 2 * F1, y_tilde)
        F3 = self.observer_f(self.x_hat + P.Ts / 2 * F2, y_tilde)
        F4 = self.observer_f(self.x_hat + P.Ts * F3, y_tilde)
        self.x_hat = self.x_hat + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y_tilde):
        return (self.A @ x_hat + self.B * (self.F_prev - self.Fe)
                + self.L @ (y_tilde - self.C @ x_hat))
`,
    ch14: String.raw`import control as cnt

class Controller:
    def __init__(self):
        self.ze = P.ell / 2
        self.Fe = P.m1 * P.g * self.ze / P.ell + P.m2 * P.g / 2
        Je = P.m2 * P.ell**2 / 3 + P.m1 * self.ze**2
        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [0.0, -P.g, 0.0, 0.0],
                      [-P.m1 * P.g / Je, 0.0, 0.0, 0.0]])
        B = np.array([[0.0], [0.0], [0.0], [P.ell / Je]])
        C = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        Cr = C[0:1, :]
        A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        zeta = 0.8
        wn_th, wn_z = 2.2 / 0.5, 2.2 / 1.5
        p_I = -1.0
        char = np.convolve(np.convolve([1, 2 * zeta * wn_th, wn_th**2],
                                       [1, 2 * zeta * wn_z, wn_z**2]), [1, -p_I])
        K1 = cnt.place(A1, B1, np.roots(char))
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        # observer for x and the input disturbance d (d constant): A2 = [[A, B], [0, 0]]
        self.A2 = np.block([[A, B], [np.zeros((1, 5))]])
        self.B2 = np.vstack([B, [[0.0]]])
        self.C2 = np.hstack([C, np.zeros((2, 1))])
        # observer 5x faster than the controller pairs (zeta 0.8), disturbance pole p_d = -5
        wo_th, wo_z = 5 * wn_th, 5 * wn_z
        p_d = -5.0
        obs_char = np.convolve(np.convolve([1, 2 * zeta * wo_th, wo_th**2],
                                           [1, 2 * zeta * wo_z, wo_z**2]), [1, -p_d])
        self.L2 = cnt.place(self.A2.T, self.C2.T, np.roots(obs_char)).T
        self.obs = np.zeros((5, 1))       # [x_hat - x_e; d_hat]
        self.F_prev = self.Fe
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, z_r, y):
        self.update_observer(y)
        x_hat = self.obs[0:4]
        d_hat = self.obs[4, 0]
        error = z_r - (x_hat[0, 0] + self.ze)
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        self.error_prev = error
        F = self.Fe - (self.K @ x_hat)[0, 0] - self.ki * integ - d_hat
        if abs(F) <= P.F_max:              # anti-windup
            self.integrator = integ
        F = self.Fe - (self.K @ x_hat)[0, 0] - self.ki * self.integrator - d_hat
        F = max(-P.F_max, min(P.F_max, F))
        self.F_prev = F
        return F, x_hat + np.array([[self.ze], [0.0], [0.0], [0.0]]), d_hat

    def update_observer(self, y):
        y_tilde = y - np.array([[self.ze], [0.0]])
        F1 = self.observer_f(self.obs, y_tilde)
        F2 = self.observer_f(self.obs + P.Ts / 2 * F1, y_tilde)
        F3 = self.observer_f(self.obs + P.Ts / 2 * F2, y_tilde)
        F4 = self.observer_f(self.obs + P.Ts * F3, y_tilde)
        self.obs = self.obs + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def observer_f(self, obs, y_tilde):
        return (self.A2 @ obs + self.B2 * (self.F_prev - self.Fe)
                + self.L2 @ (y_tilde - self.C2 @ obs))
`,
  };

  const knobsOf = (st) => ({ trTh: st.trTh, zetaTh: st.zetaTh, trZ: st.trZ, zetaZ: st.zetaZ, pI: st.pI, obsFactor: st.obsFactor, zetaObs: st.zetaObs, pD: st.pD, comp: st.comp, obsMode: st.obsMode });
  // Work-mode starting gains: a sluggish, underdamped design (ζ = 0.6 breaks the E.11(a) rule).
  const W0K = { trTh: 1.5, zetaTh: 0.6, trZ: 5, zetaZ: 0.6, pI: -0.4, obsFactor: 3, zetaObs: 0.7, pD: -1, comp: 'eq', obsMode: 'decoupled' };

  function workDefaults(sys, level) {
    const p = { ...Object.fromEntries(sys.params.map((q) => [q.key, q.value])), ...sys.constants };
    const d = E.ssDesign(p, W0K, level === 'sf' ? 'sf' : level);
    return { K: d.K.slice(), kr: d.kr ?? 0, ki: d.ki ?? 0, L: d.L ? d.L.map((r) => r.slice()) : null };
  }

  function gainsFor(ctx, level) {
    if (ctx.S.mode === 'explore') return E.ssDesign(ctx.pModel, knobsOf(ctx.st), level);
    const w = ctx.st.w;
    return { K: w.K, kr: w.kr, ki: w.ki, L: w.L, lin: ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp }) };
  }
  // Closed-loop and observer eigenvalues for the current gains.
  function clPoles(ctx, level) {
    const { A, B, C } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
    const g = ctx.gains;
    if (level === 'sf') return L.eig(L.sub(A, L.mul(B, [g.K])));
    const { A1, B1 } = E.augI(A, B);
    return L.eig(L.sub(A1, L.mul(B1, [[...g.K, g.ki]])));
  }
  function obsPolesOf(ctx, level, Lg = ctx.gains.L) {
    if (!Lg) return [];
    const { A, B, C } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
    if (level === 'obs') return L.eig(L.sub(A, L.mul(Lg, C)));
    if (level === 'dobs') { const { A2, C2 } = E.augD(A, B, C); return L.eig(L.sub(A2, L.mul(Lg, C2))); }
    return [];
  }

  // ------------------------------------------------------------- controls --
  // The simplified model (A₄₁ = 0) answers E.5(c): Work mode names it only once solved.
  const flModelShown = (ctx) => showsAnswer(ctx, `${pid(ctx, 'ch5')}/c`) && showsAnswer(ctx, `${pid(ctx, 'ch4')}/c`);
  function compControl(parent, ctx) {
    const fl = flModelShown(ctx), work = ctx.S.mode === 'work';
    segmented(parent, {
      label: work ? 'Model for the math cards and s-plane' : 'Equilibrium force and model',
      options: [
        { value: 'eq', label: 'F = F<sub>e</sub> + F̃, Jacobian A', title: 'consistent with E.6 / E.11' },
        { value: 'fl', label: `F = ${E.flName(ctx)} + F̃${fl ? ', A₄₁ = 0' : ''}`, title: fl ? 'feedback linearization; the m1 g z̃ coupling is cancelled' : 'feedback linearization (your E.4(c)) with the matching linear model' },
      ],
      ...bind(ctx, 'comp'),
    });
  }
  // Work-mode control panel: the model choice for the cards, and what the plots show.
  function workControls(parent, ctx, part) {
    compControl(section(parent, 'Model'), ctx);
    WB.myCtrl.banner(section(parent, 'Your controller'), ctx, part);
  }

  function poleKnobs(parent, ctx, { pI = false } = {}) {
    E.knob(parent, ctx, 'trTh', 't<sub>r,θ</sub> (fast pair)', 0.05, 3, 0.005, { unit: 's' });
    E.knob(parent, ctx, 'zetaTh', 'ζ<sub>θ</sub>', 0.3, 1.5, 0.005);
    E.knob(parent, ctx, 'trZ', 't<sub>r,z</sub> (slow pair)', 0.1, 15, 0.01, { unit: 's' });
    E.knob(parent, ctx, 'zetaZ', 'ζ<sub>z</sub>', 0.3, 1.5, 0.005);
    if (pI) E.knob(parent, ctx, 'pI', 'p<sub>I</sub>', -10, -0.01, 0.01, { sig: 3 });
  }
  function obsKnobs(parent, ctx, { pD = false } = {}) {
    E.knob(parent, ctx, 'obsFactor', 'observer / controller ω<sub>n</sub>', 1, 20, 0.1, { sig: 3 });
    E.knob(parent, ctx, 'zetaObs', 'ζ<sub>obs</sub>', 0.3, 1.5, 0.005);
    if (pD) E.knob(parent, ctx, 'pD', 'p<sub>d</sub>', -40, -0.1, 0.1, { sig: 3 });
    segmented(parent, {
      label: 'Observer gain structure',
      options: [
        { value: 'decoupled', label: 'decoupled (uses z and θ)', title: 'L cancels the A cross-couplings: a z block and a θ block' },
        { value: 'zonly', label: 'from z only', title: 'Ackermann on C = [1 0 0 0]; ignores the θ measurement' },
      ],
      ...bind(ctx, 'obsMode'),
    });
  }

  function gainsReadout(parent, ctx, level) {
    E.readout(parent, () => {
      const g = ctx.gains;
      const rows = g.K.map((k, i) => [`K${i + 1}`, fmt(k, 4)]);
      if (level === 'sf') rows.push(['kr', fmt(g.kr, 4)]); else rows.push(['kI', fmt(g.ki, 4)]);
      if (g.L) g.L.forEach((r, i) => rows.push([`L${i + 1}·`, `${fmt(r[0], 4)}, ${fmt(r[1], 4)}`]));
      return rows;
    });
  }


  // ------------------------------------------------------------- markers --
  function markers(ctx, level) {
    const { A } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
    const explore = ctx.S.mode === 'explore';
    // eig(A) comes from the E.6 model (or the E.5(c) one): in Work mode it waits for them.
    const olShown = showsAnswer(ctx, `${pid(ctx, 'ch6')}/a`) && (ctx.st.comp !== 'fl' || flModelShown(ctx));
    const mk = olShown ? L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `open-loop pole ${i + 1}` })) : [];
    if (!explore) {
      // Work mode has no gains (the plots run the student's own controller). The knob
      // poles are a valid answer to E.11(a) (and tuned designs for E.12–E.14), so the
      // targets are the student's own E.11(a) poles (and p_I from E.12(a)).
      for (const q of myTargets(ctx, level)) mk.push({ ...q, kind: 'target', label: 'your pole (E.11(a), E.12(a))' });
      return mk;
    }
    const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), level);
    const near = (q, list) => list.reduce((b, c, j) => (Math.hypot(c.re - q.re, c.im - q.im) < Math.hypot(list[b].re - q.re, list[b].im - q.im) ? j : b), 0);
    clPoles(ctx, level).forEach((q, i) => {
      const j = near(q, d.poles);
      mk.push({ ...q, kind: 'cl', label: j < 2 ? 'controller pole (fast pair)' : j < 4 ? 'controller pole (slow pair)' : 'integrator pole', dragId: j < 2 ? 0 : j < 4 ? 1 : 2 });
    });
    obsPolesOf(ctx, level).forEach((q) => mk.push({ ...q, kind: 'obs', label: 'observer pole', dragId: Math.abs(q.im) > 1e-9 ? 10 : 12, noFit: ctx.st.zoom === 'ctrl' }));
    return mk;
  }
  // The student's E.11(a) poles (if valid) plus their E.12(a) p_I for the integrator levels.
  function myTargets(ctx, level) {
    const ps = userPoles(ctx);
    if (!validatePoles(ps).ok) return [];
    const pI = PD().num(ansKey(ctx, pid(ctx, 'ch12'))['a.pI']);
    return level === 'sf' || pI === null || !(pI < 0) ? ps : [...ps, { re: pI, im: 0 }];
  }

  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.005, re);
    const wn = Math.hypot(re, im), zeta = Math.max(0.3, Math.min(1, -re / wn));
    if (id === 0) { st.trTh = 2.2 / wn; st.zetaTh = zeta; }
    else if (id === 1) { st.trZ = 2.2 / wn; st.zetaZ = zeta; }
    else if (id === 2) st.pI = re;
    else if (id === 10) { st.obsFactor = Math.max(1, wn / (2.2 / st.trTh)); st.zetaObs = zeta; }
    else if (id === 12) { if (st.pD !== undefined) st.pD = re; }
    ctx.update();
  }

  // ---------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
    return {
      title: `State-space model (${ctx.st.comp === 'fl' ? (flModelShown(ctx) ? 'F_fl(z), A₄₁ = 0' : 'F_fl') : 'Jacobian, F_e'})`, page: 'p. 387 · E.6, p. 183',
      answers: ctx.st.comp === 'fl' ? [`${pid(ctx, 'ch6')}/a`, `${pid(ctx, 'ch4')}/c`, `${pid(ctx, 'ch5')}/c`] : `${pid(ctx, 'ch6')}/a`,
      theory: '\\dot{\\tilde x} = A\\tilde x + B\\tilde F,\\quad \\tilde x = x - x_e,\\; x_e = (\\tfrac{\\ell}{2}, 0, 0, 0),\\quad \\tilde F = F - F_{ff}',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`,
    };
  }
  // `answers`: the problem part whose rank/det this card gives away (if any).
  const ctrbCard = (A, B, title, page, answers) => ({ ...WB.ss.ctrbCard(A, B, title, page, { matrix: false, det: true }), answers });
  function polesCard(ctx, d, level) {
    return {
      title: 'Desired closed-loop poles', page: 'p. 190 (B.11), p. 113 · Eq. 8.5',
      theory: '\\Delta^d = (s^2 + 2\\zeta_\\theta\\omega_{n_\\theta}s + \\omega_{n_\\theta}^2)(s^2 + 2\\zeta_z\\omega_{n_z}s + \\omega_{n_z}^2)' + (level === 'sf' ? '' : '(s - p_I)') + ',\\quad \\omega_n = \\frac{2.2}{t_r}',
      numbers: `p = ${d.poles.map((q) => texPole(q)).join(',\\;')}`,
      spoiler: true,
      note: 'E.11(a): every pole needs ωₙ > ωₙ,z = 0.22 rad/s and ζ > 0.707 (the E.8 outer loop).',
    };
  }

  // ------------------------------------------------------------- problems --
  const ansKey = (ctx, id) => E.answersOf(ctx, id);
  // Poles typed into E.11(a), validated; null if missing.
  function userPoles(ctx) {
    const a = ansKey(ctx, 'E.11');
    const ps = ['p1', 'p2', 'p3', 'p4'].map((k) => M.parseComplex(a[`a.${k}`]));
    return ps.every(Boolean) ? ps : null;
  }
  function validatePoles(ps) {
    if (!ps || ps.some((q) => !q)) return { ok: false, msg: 'Enter four poles (e.g. -1+0.5j).' };
    for (const q of ps) {
      if (Math.abs(q.im) > 1e-9 && !ps.some((c) => M.close(c.re, q.re) && M.close(c.im, -q.im))) return { ok: false, msg: `${M.fmtPole(q)} needs its conjugate.` };
    }
    const wnz = 2.2 / 10, bad = [];
    for (const q of ps) {
      const wn = Math.hypot(q.re, q.im), zeta = -q.re / wn;
      if (!(q.re < 0)) bad.push(`${M.fmtPole(q)} is not stable`);
      else if (wn <= wnz) bad.push(`|${M.fmtPole(q)}| = ${fmt(wn, 3)} ≤ ωn,z = 0.22`);
      else if (zeta < 0.707 - 1e-3) bad.push(`${M.fmtPole(q)} has ζ = ${fmt(zeta, 3)} < 0.707`);
    }
    return bad.length ? { ok: false, msg: bad.join('; ') + '.' } : { ok: true, msg: 'Valid choice. Parts (d) and E.12 use these poles.' };
  }
  // --------------------------------------------------------- chapter base --
  function base(level, num, title, pages, extra) {
    return Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages, level,
      gains(ctx) { return gainsFor(ctx, level); },
      // Work mode simulates the student's controller (WB.myCtrl), from y = (z, θ) only.
      implement: { feed: 'y', linear: false },
      controller(ctx, o) { return E.makeSS(ctx, ctx.gains, level, { comp: ctx.st.comp, est: ctx.st.est, antiwindup: ctx.st.antiwindup, dobs: ctx.st.dobs, zhat0: ctx.st.zhat0 || 0 }, o); },
      splane(ctx) { return { markers: markers(ctx, level), kindNames: { cl: 'controller pole', obs: 'observer pole', ol: 'open-loop pole', target: 'target pole' } }; },
      onPoleDrag: onDrag,
      targets(ctx) { return level === 'sf' && ctx.S.mode === 'explore' ? { tr: ctx.st.trZ } : {}; },
    }, extra);
  }
  function defaultsFor(sys, key, level, extra = {}) {
    const p = sys.problems[key];
    return { comp: 'eq', est: 'true', antiwindup: 'clamp', obsMode: 'decoupled', dobs: true, zhat0: 0, zoom: 'all',
      trTh: p.trTh, zetaTh: p.zetaTh, trZ: p.trZ, zetaZ: p.zetaZ, pI: p.pI ?? -1, obsFactor: p.obsFactor ?? 5, zetaObs: p.zetaObs ?? 0.8, pD: p.pD ?? -5,
      w: workDefaults(sys, level), ...extra };
  }
  // Estimates on the z and θ plots: the workbench observer's (Explore), or in Work mode
  // the x̂ the student's update returns (extras xhat0.., WB.myCtrl).
  function estimateSeries(ctx, res, sc, oi) {
    if (ctx.S.mode === 'work') {
      const xh = E.xhatOf(res, ctx.sys.ze(ctx.pModel));
      return xh ? [{ label: `${oi === 0 ? 'ẑ' : 'θ̂'} (estimate)`, y: sc(xh[oi]), color: '--series-3', dash: [3, 3], width: 2 }] : [];
    }
    const key = oi === 0 ? 'zhat' : 'thhat';
    if (!res.extras[key]) return [];
    return [{ label: `${oi === 0 ? 'ẑ' : 'θ̂'} (observer)`, y: sc(res.extras[key]), color: '--series-3', dash: [3, 3], width: 2 }];
  }

  // ------------------------------------------------- student-controller checks --
  const SQUARE = (amplitude) => ({ type: 'square', amplitude, frequency: 0.05, tStep: 0 });
  const STEP = (amplitude) => ({ type: 'step', amplitude, tStep: 0 });
  const misText = (m) => Object.entries(m).map(([k, v]) => `${k === 'ell' ? 'ℓ' : k} ${v > 0 ? '+' : '−'}${Math.abs(v)}%`).join(', ');
  const atLimit = (res, Fmax) => { let n = 0; for (const u of res.uDemand) if (Math.abs(u) >= Fmax * (1 - 1e-6)) n++; return n; };
  // Tracking with the nominal and the second parameter set (exact model each time):
  // on the beam, |z_r − z| before the first switch under tol, and (noSat) F never at
  // the limit. o: scenario options. Resolves to {ok, msg} (or a Python error).
  async function trackBoth(ctx, code, o, { tol, noSat = false }) {
    const cases = WB.myCtrl.paramCases(ctx);
    const parts = [];
    for (let i = 0; i < cases.length; i++) {
      const sc = WB.myCtrl.scenario(ctx, { ...o, params: cases[i].params, tEnd: 10 });
      const res = await WB.myCtrl.run(ctx, code, sc);
      if (res.ok === false) return res;
      const ob = E.onBeamRes(res, sc.plantParams.ell), e = E.errAt(res, 9.95), nAt = noSat ? atLimit(res, sc.plantParams.F_max) : 0;
      const msg = `${ob.msg}; |z_r − z| before the switch ${fmt(1000 * e, 3)} mm${nAt ? `; F at the limit for ${nAt} samples` : ''}`;
      if (!(ob.ok && e < tol && !nAt)) {
        const hint = i > 0 && !parts.some((x) => !x.ok) ? ' It works with the nominal parameters: compute the gains, x_e and F_e from P.m1, P.m2, P.ell rather than numbers.' : '';
        return { ok: false, msg: `With ${cases[i].label}: ${msg}.${hint}` };
      }
      parts.push({ ok: true, msg });
    }
    return { ok: true, msg: `${parts[0].msg} (${cases.map((c) => c.label).join('; ')}).` };
  }
  // Mean of x_i − x̂_i over [t0, t1] (the student's x̂, absolute z).
  const biasOf = (res, xh, i, t0, t1) => WB.myCtrl.mean(res, t0, t1, (k) => res.x[k][i] - xh[i][k]);

  // ------------------------------------------------------------ Chapter 11 --
  CH.ch11 = base('sf', 11, 'Full state feedback', 'pp. 173–196', {
    defaults(sys) { return defaultsFor(sys, 'ch11', 'sf'); },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    linearSim(ctx, c) {
      const lin = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const plant = WB.design.linearPlant(lin.A, lin.B, lin.C, { xe: [lin.ze, 0, 0, 0] });
      return WB.sim.simulate({ ...c, disturbance: null, noise: null, plant, controller: E.makeSS(ctx, ctx.gains, 'sf', { comp: ctx.st.comp }, { linear: true }) });
    },
    linearLabel: 'linear model',
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workControls(parent, ctx, `${ctx.sys.problems.ch11.id}(e)`); return; }
      const sec = section(parent, 'F̃ = −K x̃ + k_r z̃_r', 'p. 183 · Eq. 11.38');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Where x comes from',
        options: [{ value: 'true', label: 'true state' }, { value: 'dirty', label: 'z, θ + dirty derivatives' }],
        ...bind(ctx, 'est'),
      });
      const spec = section(parent, 'Pole knobs', 'p. 389 · E.11(a)');
      poleKnobs(spec, ctx);
      gainsReadout(spec, ctx, 'sf');
    },
    math(ctx) {
      const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'sf');
      const { A, B } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const g = ctx.gains;
      return [
        ssCard(ctx), ctrbCard(A, B, 'Controllability', 'p. 180 · Eq. 11.29', `${pid(ctx, 'ch11')}/c`), polesCard(ctx, d, 'sf'),
        { title: 'Pole placement (Ackermann)', page: 'p. 182 · Eq. 11.32',
          theory: 'K = (\\alpha - a_A)\\,\\mathcal{A}_A^{-1}\\,\\mathcal{C}_{A,B}^{-1},\\quad \\Delta_{ol}(s) = \\det(sI - A)' },
        { title: 'Reference gain', page: 'p. 182 · Eq. 11.35',
          theory: 'k_r = \\frac{-1}{C_r(A - BK)^{-1}B},\\quad C_r = \\begin{bmatrix}1 & 0 & 0 & 0\\end{bmatrix}' },
        { title: 'Gains for the pole knobs', page: 'p. 389 · E.11(d)', answers: `${pid(ctx, 'ch11')}/d`,
          theory: `\\Delta_{ol} = ${WB.tf.polyTex(L.charPoly(A))},\\quad K = ${texMat([d.K])},\\quad k_r = ${tex(d.kr)}` },
        { title: 'Control law', page: 'p. 183 · Eq. 11.38', answers: `${pid(ctx, 'ch11')}/e`,
          theory: 'F = F_{ff} - K(x - x_e) + k_r(z_r - z_e)',
          numbers: ctx.S.mode === 'work' ? null : `F = F_{ff} - ${texMat([g.K])}\\tilde x + ${tex(g.kr)}\\,\\tilde z_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch11;
      const jac = () => ctx.sys.linear(ctx.pModel, { comp: 'eq' });
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Four closed-loop poles',
          html: 'Complex poles in conjugate pairs, e.g. <code>-1.2+0.9j</code> and <code>-1.2-0.9j</code>. From E.8: ω<sub>n<sub>z</sub></sub> = 2.2/10 = 0.22 rad/s, ζ<sub>z</sub> = 0.707.',
          inputs: { p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>', p3: 'p<sub>3</sub>', p4: 'p<sub>4</sub>' },
          check: (v) => validatePoles(['p1', 'p2', 'p3', 'p4'].map((k) => M.parseComplex(v[k]))),
          solution: () => [{ html: 'Many answers work. One choice, in the spirit of B.11 (p. 190): a fast pair from t<sub>r</sub> = 0.5 s and a slow pair from t<sub>r</sub> = 1.5 s, both with ζ = 0.8: −3.52 ± 2.64j and −1.173 ± 0.88j (the Explore defaults). Faster poles need more force: watch the F plot as you move them.' }],
        },
        {
          id: 'b', title: '(b) State-space matrices from E.6',
          html: 'Add your A, B, C, D from E.6 (the Ch 6 tab) to your param file. The model card in the Math section unlocks once E.6 is solved.',
        },
        {
          id: 'c', title: '(c) Controllability',
          inputs: { rank: 'rank 𝒞<sub>A,B</sub>', det: 'det 𝒞<sub>A,B</sub>' },
          check: (v) => { const Cm = L.ctrb(jac().A, jac().B); return PD().checkNumbers(v, { rank: L.rank(Cm), det: L.det(Cm) }, { det: 'det' }); },
          solution: () => {
            const { A, B, b0 } = jac(), Cm = L.ctrb(A, B);
            return [{ tex: `\\mathcal{C}_{A,B} = ${texMat(Cm)},\\quad \\det = -g^2b_0^4 = ${tex(L.det(Cm))} \\ne 0 \\Rightarrow \\text{rank } 4` }, { html: `b₀ = ℓ/J<sub>e</sub> = ${fmt(b0, 4)}. The sign of the determinant depends on column order; its magnitude is what matters.` }];
          },
        },
        {
          id: 'd', title: '(d) K and k<sub>r</sub> for your poles from (a) (Jacobian A, F = F<sub>e</sub> + F̃)',
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => {
            const ps = userPoles(ctx), ok = validatePoles(ps);
            if (!ok.ok) return { ok: false, msg: `Part (a) first: ${ok.msg}` };
            const d = E.ssDesign(ctx.pModel, { comp: 'eq' }, 'sf', ps);
            return PD().checkNumbers(v, { K1: d.K[0], K2: d.K[1], K3: d.K[2], K4: d.K[3], kr: d.kr }, {});
          },
          solution: () => {
            const ps = userPoles(ctx);
            const use = validatePoles(ps).ok ? ps : E.ssPoles({ trTh: 0.5, zetaTh: 0.8, trZ: 1.5, zetaZ: 0.8 });
            const d = E.ssDesign(ctx.pModel, { comp: 'eq' }, 'sf', use);
            return [
              { html: validatePoles(ps).ok ? 'For your poles from (a):' : 'Part (a) is not filled in yet, so this uses −3.52 ± 2.64j, −1.173 ± 0.88j:' },
              { tex: `\\Delta^d = ${WB.tf.polyTex(L.polyFromRoots(use))},\\quad \\Delta_{ol} = ${WB.tf.polyTex(L.charPoly(jac().A))}` },
              { tex: `K = ${texMat([d.K])},\\quad k_r = ${tex(d.kr)}` },
              { html: 'Cross-checked against python-control <code>place</code>.' },
            ];
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'e', title: '(e) Implement the state feedback scheme in simulation and tune the closed-loop poles',
          seed: [`${pid(ctx, 'ch10')}/c`, `${pid(ctx, 'ch10')}/b`],
          html: 'Your controller gets only the measured z, θ (estimate the velocities yourself). Watch how the pole locations trade speed against force. The check runs the square wave (0.25 ± 0.15 m, 0.05 Hz) with exact parameters, nominal and another set: the block must stay on the beam, |z<sub>r</sub> − z| just before the first switch (t = 10 s) must be under 2 mm, and F must never reach F<sub>max</sub>.',
          check: (code) => trackBoth(ctx, code, { ref: SQUARE(0.15) }, { tol: 0.002, noSat: true }),
          solution: () => [
            { code: SOL.ch11 },
            { html: 'As B.11 (p. 190), on the E.6 model: F = F<sub>e</sub> − K(x − x<sub>e</sub>) + k<sub>r</sub>(z<sub>r</sub> − z<sub>e</sub>), with ż and θ̇ from dirty derivatives (σ = 0.05). The poles are the E.11(a) sample (pairs from t<sub>r</sub> = 0.5 s and 1.5 s, ζ = 0.8); any pole set that passes is fine. The peak force is about 12.6 N.' },
          ],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 12 --
  // E.12(a) checks: tracking under a constant disturbance (the integrator), and a
  // saturated move with F_max lowered (windup shows as undershoot).
  const WINDUP = { dTrack: 0.5, Fmax: 12.6, z0: 0.4, step: -0.15, os: 0.01 };
  CH.ch12 = base('sfi', 12, 'Integrator with state feedback', 'pp. 197–214', {
    defaults(sys) { return defaultsFor(sys, 'ch12', 'sfi'); },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workControls(parent, ctx, `${ctx.sys.problems.ch12.id}(a) and (c)`); return; }
      const sec = section(parent, 'F̃ = −K x̃ − k_I ∫(z_r − z)', 'p. 199');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Anti-windup (E.12a)',
        options: [{ value: 'clamp', label: 'hold integrator while saturated' }, { value: 'none', label: 'none' }],
        ...bind(ctx, 'antiwindup'),
      });
      const spec = section(parent, 'Pole knobs', 'p. 389 · E.12(c)');
      poleKnobs(spec, ctx, { pI: true });
      gainsReadout(spec, ctx, 'sfi');
    },
    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [m·s]', unit: 'm·s' }, data: { series: [{ label: 'x_I = ∫(z_r − z) dt', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'sfi');
      const { A, B } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const { A1, B1 } = E.augI(A, B);
      return [
        ssCard(ctx),
        { title: 'Augmented system', page: 'p. 198 · Eq. 12.1',
          theory: '\\dot x_I = z_r - C_r x,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C_r & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\ 0\\end{bmatrix}' },
        ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 198'),
        polesCard(ctx, d, 'sfi'),
        { title: 'Gains and control law', page: 'p. 199–201', answers: `${pid(ctx, 'ch12')}/a2`,
          theory: '\\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad F = F_{ff} - K\\tilde x - k_I\\textstyle\\int_0^t (z_r - z)\\,d\\tau' },
        { title: 'Gains for the pole knobs', page: 'p. 389 · E.12(a)', answers: `${pid(ctx, 'ch12')}/a`,
          theory: `A_1 = ${texMat(A1)},\\quad K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}` },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch12;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Gains for your E.11(a) poles plus an integrator pole p<sub>I</sub> (Jacobian A)',
          inputs: { pI: 'p<sub>I</sub>', K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', ki: 'k<sub>I</sub>' },
          check: (v) => {
            const ps = userPoles(ctx), ok = validatePoles(ps);
            if (!ok.ok) return { ok: false, msg: `Fill in E.11(a) first: ${ok.msg}` };
            const pI = PD().num(v.pI);
            if (pI === null || pI >= 0) return { ok: false, msg: 'Enter a negative pI.' };
            const d = E.ssDesign(ctx.pModel, { comp: 'eq', pI }, 'sfi', ps);
            return PD().checkNumbers(v, { K1: d.K[0], K2: d.K[1], K3: d.K[2], K4: d.K[3], ki: d.ki }, {});
          },
          solution: () => {
            const ps = userPoles(ctx);
            const use = validatePoles(ps).ok ? ps : E.ssPoles({ trTh: 0.5, zetaTh: 0.8, trZ: 1.5, zetaZ: 0.8 });
            const pI = PD().num(ansKey(ctx, prob.id)['a.pI']) ?? prob.pI;
            const d = E.ssDesign(ctx.pModel, { comp: 'eq', pI }, 'sfi', use);
            return [
              { tex: `p = ${use.map((q) => texPole(q)).join(',\\;')},\\; p_I = ${tex(pI)}` },
              { tex: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}` },
              { html: 'k<sub>I</sub> comes out positive with this sign convention (F̃ = −K x̃ − k<sub>I</sub>∫(z<sub>r</sub> − z)) because positive F tilts the beam up and moves the block toward the pivot.' },
            ];
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'a2', title: '(a) Add an integrator with anti-windup to the feedback loop for z in your E.11 controller',
          seed: `${pid(ctx, 'ch11')}/e`,
          html: `Use your gains from above. The check (1) runs a ±0.1 m square wave (0.05 Hz) with a ${WINDUP.dTrack} N input disturbance and exact parameters, nominal and another set: the block must stay on the beam and |z<sub>r</sub> − z| just before the first switch must be under 1 mm; then (2) lowers F<sub>max</sub> to ${WINDUP.Fmax} N and sends the block from z = ${WINDUP.z0} m to z<sub>r</sub> = ${fmt(0.25 + WINDUP.step, 3)} m, so F saturates for most of a second: z may undershoot z<sub>r</sub> by at most ${WINDUP.os * 1000} mm.`,
          check: async (code) => {
            const t = await trackBoth(ctx, code, { ref: SQUARE(0.1), dist: WINDUP.dTrack }, { tol: 0.001 });
            if (!t.ok) return t;
            const sc = WB.myCtrl.scenario(ctx, { params: { ...ctx.pModel, F_max: WINDUP.Fmax }, init: { z0: WINDUP.z0 }, ref: STEP(WINDUP.step), tEnd: 15 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const us = (0.25 + WINDUP.step) - Math.min(...res.yAll[0]);
            if (!(us <= WINDUP.os)) return { ok: false, msg: `Tracking passes, but with F_max = ${WINDUP.Fmax} N the block undershoots z_r by ${fmt(1000 * us, 3)} mm: the integrator winds up while F is saturated.` };
            return { ok: true, msg: `${t.msg} Saturated move: undershoot ${fmt(1000 * Math.max(0, us), 3)} mm.` };
          },
          solution: () => [
            { code: SOL.ch12 },
            { html: 'Anti-windup here holds the integrator while F is saturated (without it the saturated move undershoots by about 0.14 m). Here, unlike E.10, F saturates for long stretches, so holding the integrator then is what works: the |ż| gate of E.10(c) alone still undershoots by about 26 mm. The poles are faster than the E.11 sample (pairs from t<sub>r</sub> = 0.2 s, ζ = 0.8, and 0.7 s, ζ = 0.85, p<sub>I</sub> = −2) because part (c) needs them; the E.11 sample poles with p<sub>I</sub> = −1 pass the check here as well (and undershoot by about 33 mm without anti-windup).' },
          ],
        }),
        {
          id: 'b', title: '(b) Input disturbance of 1 N and 20% parameter variation',
          html: 'Set the input disturbance d and the true-plant mismatch in the left panel (the chapter starts with d = 1 N and a fixed 20% draw).',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return Math.abs(S.sim.dist) > 0 && mis ? { ok: true, msg: `d = ${fmt(S.sim.dist, 3)} N with plant mismatch.` } : { ok: false, msg: 'Set both d ≠ 0 and a plant mismatch.' };
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Tune the integrator pole (and other gains if necessary) to get good tracking performance',
          seed: [`${prob.id}/a2`],
          html: `The check runs the square wave (0.25 ± 0.15 m, 0.05 Hz) with d = ${prob.sim.dist} N from t = 0 and the plant off by ${misText(prob.mismatch)}: the block must stay on the beam (0 ≤ z ≤ ℓ of that plant), and |z<sub>r</sub> − z| just before the first switch (t = 10 s) must be under 2 mm.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.15), tEnd: 10, dist: prob.sim.dist, mismatch: prob.mismatch });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const ob = E.onBeamRes(res, sc.plantParams.ell), e = E.errAt(res, 9.95);
            return { ok: ob.ok && e < 0.002, msg: `${ob.msg}; |z_r − z| before the switch: ${fmt(1000 * e, 3)} mm.` };
          },
          solution: () => [
            { code: SOL.ch12 },
            { html: 'The slower poles of E.11 (pairs from t<sub>r</sub> = 0.5 and 1.5 s, p<sub>I</sub> = −1) are not enough: with d = 1 N from t = 0 and the default mismatch, the block dips to about −0.16 m (off the pivot end) before the integrator catches up, and only 7 of 30 random α = 0.2 draws pass.' },
            { html: 'The faster poles of the solution to (a) fix it: a pair from t<sub>r</sub> = 0.2 s (ζ = 0.8), a pair from t<sub>r</sub> = 0.7 s (ζ = 0.85), and p<sub>I</sub> = −2 (the Explore defaults). Robustness, tested in Python and JS: with d = +1 N, 88 of 90 random draws pass (30/30, 29/30, 29/30 for three seeds). With d = −1 N, which adds to the load, 24 of 30 pass (checked in Python). The peak force stays under the 15 N limit for the default draw.' },
            { html: 'Without the integrator (E.11 gains), a constant d shifts the block by d/k<sub>r</sub> at steady state: with k<sub>r</sub> ≈ −1.6, 1 N moves it about 0.6 m, off the beam.' },
          ],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 13 --
  // E.13(c) check: the block starts off z_e; estimate errors before each switch.
  const OBS = { z0: 0.35, z: 1e-4, th: 0.01, zd: 1e-3, thd: 0.1 };
  CH.ch13 = base('obs', 13, 'Observers', 'pp. 215–238', {
    defaults(sys) { return defaultsFor(sys, 'ch13', 'obs'); },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries: estimateSeries,
    extraPlot(ctx, res) {
      const xh = ctx.S.mode === 'work' ? E.xhatOf(res, ctx.sys.ze(ctx.pModel)) : null;
      return {
        opts: { title: 'ż(t) and estimate', yLabel: 'ż [m/s]', unit: 'm/s' },
        data: { series: [
          ...(ctx.S.mode !== 'work' || xh ? [{ label: 'ż̂ (estimate)', y: Array.from((xh ? xh[2] : res.extras.zdhat) || []), color: '--series-3', dash: [3, 3], width: 2 }] : []),
          { label: 'true ż', y: res.x.map((x) => x[2]), color: '--series-1' },
        ] },
      };
    },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workControls(parent, ctx, `${ctx.sys.problems.ch13.id}(c)`); return; }
      const sec = section(parent, 'Controller (uses x̂)', 'p. 222 · Fig. 13-3');
      compControl(sec, ctx);
      const spec = section(parent, 'Controller and observer knobs', 'p. 224 · §13.2');
      poleKnobs(spec, ctx, { pI: true }); obsKnobs(spec, ctx);
      slider(spec, { label: 'ẑ(0) − z<sub>e</sub>', unit: 'm', min: -0.2, max: 0.2, step: 0.005, sig: 3, hint: 'initial estimate error', ...bind(ctx, 'zhat0') });
      gainsReadout(spec, ctx, 'obs');
    },
    math(ctx) {
      const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'obs');
      const { A, C } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const O = L.obsv(A, C);
      return [
        ssCard(ctx),
        { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 224', answers: `${pid(ctx, 'ch13')}/c`,
          theory: '\\dot{\\hat x} = A\\hat x + B(u - F_{ff}) + L(\\tilde y - C\\hat x),\\quad \\dot e = (A - LC)e' },
        { title: 'Observability', page: 'p. 221',
          theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ CA^2 \\\\ CA^3\\end{bmatrix}\\;(8\\times4),\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = 4' },
        { title: 'Observability of the block and beam', page: 'p. 390 · E.13(b)', answers: `${pid(ctx, 'ch13')}/b`,
          theory: `\\operatorname{rank}\\mathcal{O}_{A,C} = ${L.rank(O)}` },
        { title: 'Observer gain with two outputs', page: 'p. 225',
          theory: '\\text{with two outputs } L \\text{ is } 4\\times2 \\text{ and not unique: any } L \\text{ with the desired eig}(A - LC) \\text{ works}' },
        { title: ctx.S.mode === 'explore' ? 'Decoupled observer gain for the block and beam' : 'Observer gain for the block and beam', page: 'p. 225 (B.13 uses place(Aᵀ, Cᵀ)ᵀ)', answers: `${pid(ctx, 'ch13')}/c`,
          theory: 'L =\\begin{bmatrix}\\beta_{z1} & 0\\\\ 0 & \\beta_{\\theta1}\\\\ \\beta_{z0} & a_{32}\\\\ a_{41} & \\beta_{\\theta0}\\end{bmatrix} \\Rightarrow A - LC = \\text{blockdiag}\\left(\\begin{bmatrix}-\\beta_{z1} & 1\\\\ -\\beta_{z0} & 0\\end{bmatrix}, \\begin{bmatrix}-\\beta_{\\theta1} & 1\\\\ -\\beta_{\\theta0} & 0\\end{bmatrix}\\right)',
          numbers: `q = ${d.obsPoles.map((q) => texPole(q)).join(',\\;')},\\quad L = ${texMat(d.L)}`,
          note:'With two outputs L is not unique: python\'s place gives a different, dense L with the same eigenvalues. Any L with the right eig(A − LC) answers (c).' },
        { title: 'Separation principle', page: 'p. 222–223',
          theory: '\\text{eig} = \\text{eig}(A_1 - B_1K_1) \\cup \\text{eig}(A - LC)',
          note: 'Holds for the linear model only; saturation, mismatch and the nonlinear plant break it.' },
        polesCard(ctx, d, 'obs'),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch13;
      const jac = () => ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Exact parameters, no input disturbance',
          html: 'The chapter starts with α = 0 and d = 0. <em>Exact model</em> in the left panel restores them.',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return !mis && !(Math.abs(S.sim.dist) > 0) ? { ok: true, msg: 'Exact parameters, no disturbance.' } : { ok: false, msg: 'Set the true plant equal to the model and d = 0.' };
          },
        },
        {
          id: 'b', title: '(b) Observability',
          inputs: { rank: 'rank 𝒪<sub>A,C</sub>' },
          check: (v) => PD().checkNumbers(v, { rank: L.rank(L.obsv(jac().A, jac().C)) }, {}),
          solution: () => [{ html: 'C picks z̃ and θ̃; CA adds z̃̇ and θ̃̇, so the first four rows of 𝒪 are already the 4×4 identity: rank 4. (z alone would also do: z, ż, z̈ = −gθ, z⃛ = −gθ̇.)' }],
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) In the control block, add an observer to estimate the state x̂, and use the estimate in your feedback controller; tune the controller and observer poles',
          seed: [`${pid(ctx, 'ch12')}/c`, `${pid(ctx, 'ch12')}/a2`],
          html: `Return <code>(F, x_hat)</code> from <code>update</code>, with x_hat the estimate of x = (z, θ, ż, θ̇) (z itself or z − z<sub>e</sub>; either is read). With two outputs L is 4×2 and not unique; any L with the eigenvalues you want works. The check starts the block at z = ${OBS.z0} m (your estimate starts wherever you start it) and runs the square wave (0.25 ± 0.15 m, 0.05 Hz) with exact parameters: before each switch, |z − ẑ| must be under ${OBS.z * 1000} mm, |θ − θ̂| under ${OBS.th}°, |ż − ż̂| under ${OBS.zd * 1000} mm/s and |θ̇ − θ̇̂| under ${OBS.thd}°/s; the block must stay on the beam, and |z<sub>r</sub> − z| just before the first switch must be under 2 mm.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.15), tEnd: 20, init: { z0: OBS.z0 } });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const xh = E.xhatOf(res, ctx.sys.ze(ctx.pModel));
            if (!xh) return E.NEED_XHAT;
            const ee = E.estErr(res, xh, [[8, 9.95], [18, 20]]);
            const ob = E.onBeamRes(res, sc.plantParams.ell), e = E.errAt(res, 9.95);
            const ok = ee[0] < OBS.z && ee[1] < OBS.th * M.DEG && ee[2] < OBS.zd && ee[3] < OBS.thd * M.DEG && ob.ok && e < 0.002;
            return { ok, msg: `Before the switches: |z − ẑ| ≤ ${fmt(1000 * ee[0], 3)} mm, |θ − θ̂| ≤ ${fmt(ee[1] / M.DEG, 3)}°, |ż − ż̂| ≤ ${fmt(1000 * ee[2], 3)} mm/s, |θ̇ − θ̇̂| ≤ ${fmt(ee[3] / M.DEG, 3)}°/s; ${ob.msg}; |z_r − z| before the first switch ${fmt(1000 * e, 3)} mm.` };
          },
          solution: () => [
            { code: SOL.ch13 },
            { html: 'As the repo\'s ctrlObserver.py (RK4 observer with the previous saturated F, L = place(Aᵀ, Cᵀ, q)ᵀ with observer poles 5× faster than the controller pairs), in deviation variables x̃ = x − x<sub>e</sub>, F̃ = F − F<sub>e</sub>, plus the E.12 anti-windup. Other L work too: one that cancels the two cross-couplings of A − LC (L₃₂ = a₃₂ = −g, L₄₁ = a₄₁) and places a (z, ż) block from z and a (θ, θ̇) block from θ, or observing everything from z alone (C = [1 0 0 0]).' },
          ],
        }),
        {
          id: 'd', title: '(d) Output both u and x̂, and plot the state and the estimated state on the same graph',
          html: 'Run your controller: the z and θ plots show your ẑ and θ̂ with the true states, and the extra plot your ż̂ with the true ż.',
        },
        {
          id: 'e', title: '(e) Add an input disturbance of 0.5 N and observe the steady-state error even though there is an integrator',
          html: 'Runs your controller from (c) with d = 0.5 N and exact parameters. Your observer has no model of d, so x̂ is biased, and the integrator acting on ẑ leaves z off z<sub>r</sub> when the bias reaches ẑ. Passes when the run shows the bias in ẑ or θ̂ (Chapter 14 removes it).',
          check: async () => {
            const code = WB.myCtrl.savedCode(ctx, `${prob.id}/c`);
            if (!code) return { ok: false, msg: 'Write your controller in (c) first.' };
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.15), tEnd: 20, dist: 0.5 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const xh = E.xhatOf(res, ctx.sys.ze(ctx.pModel));
            if (!xh) return E.NEED_XHAT;
            const bz = biasOf(res, xh, 0, 18, 20), bth = biasOf(res, xh, 1, 18, 20) / M.DEG;
            const ez = WB.myCtrl.mean(res, 18, 20, (k) => res.yAll[0][k] - res.r[k]);
            const msg = `With d = 0.5 N, over the last 2 s: z − ẑ = ${fmt(1000 * bz, 3)} mm, θ − θ̂ = ${fmt(bth, 3)}°, z − z_r = ${fmt(1000 * ez, 3)} mm.`;
            if (Math.abs(bz) < 1e-4 && Math.abs(bth) < 0.01) return { ok: false, msg: `${msg} x̂ follows x exactly: is x̂ coming from an observer of the model?` };
            return { ok: true, msg };
          },
          solution: () => [{ html: 'The observer error obeys ė = (A − LC)e + Bd, so it settles at e<sub>ss</sub> = −(A − LC)⁻¹Bd ≠ 0. The integrator drives z<sub>r</sub> − ẑ to zero, so z ends up off by the ẑ bias, as the book says. With the solution\'s L (from place) the bias reaches ẑ (about 11 mm) and z settles about 10 mm from z<sub>r</sub>. An L that decouples the (z, ż) and (θ, θ̇) blocks keeps ẑ unbiased, since d enters only θ̈: then z still tracks and the bias shows up in θ̂ only (about 0.16°).' }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 14 --
  CH.ch14 = base('dobs', 14, 'Disturbance observers', 'pp. 239–259', {
    defaults(sys) { return defaultsFor(sys, 'ch14', 'dobs'); },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries: estimateSeries,
    extraPlot(ctx, res) {
      const S = ctx.S;
      // Work mode: the d̂ the student's update returns as (F, x_hat, d_hat), if any.
      const dh = res.extras.dhat;
      return {
        opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N]', unit: 'N' },
        data: { series: [
          ...(dh ? [{ label: 'd̂ (estimate)', y: Array.from(dh), color: '--series-3', width: 2 }] : []),
          { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
        ] },
      };
    },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workControls(parent, ctx, `${ctx.sys.problems.ch14.id}(b)`); return; }
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'p. 241');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Disturbance observer',
        options: [{ value: true, label: 'on' }, { value: false, label: 'off (E.14a)' }],
        ...bind(ctx, 'dobs'),
      });
      const spec = section(parent, 'Controller and observer knobs', 'p. 241');
      poleKnobs(spec, ctx, { pI: true });
      obsKnobs(spec, ctx, { pD: true });
      gainsReadout(spec, ctx, 'dobs');
    },
    math(ctx) {
      const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'dobs');
      const { A, B, C } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const { A2, C2 } = E.augD(A, B, C);
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(u + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0' },
        { title: 'Augmented model (ḋ = 0)', page: 'p. 240',
          theory: 'A_2 = \\begin{bmatrix}A & B\\\\ 0 & 0\\end{bmatrix},\\quad C_2 = \\begin{bmatrix}C & 0\\end{bmatrix}',
          numbers: `\\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}` },
        { title: 'Disturbance observer', page: 'p. 241', answers: `${pid(ctx, 'ch14')}/b`,
          theory: '\\dot{\\hat x} = A\\hat x + B(u - F_{ff} + \\hat d) + L(\\tilde y - C\\hat x),\\quad \\dot{\\hat d} = L_d(\\tilde y - C\\hat x),\\quad \\tilde F = -K\\hat x - k_I\\textstyle\\int e - \\hat d' },
        { title: ctx.S.mode === 'explore' ? 'Decoupled gains: a z block and a θ–d block' : 'Disturbance-observer gain of the block and beam', page: 'p. 241', answers: `${pid(ctx, 'ch14')}/b`,
          theory: '\\theta\\text{–}d \\text{ block: } s^3 + \\beta_2 s^2 + \\beta_1 s + b_0 L_d = (s^2 + 2\\zeta\\omega s + \\omega^2)(s - p_d)',
          numbers: `L_2 = ${texMat(d.L)}` },
        polesCard(ctx, d, 'dobs'),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch14;
      const e13 = `${pid(ctx, 'ch13')}/c`;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) α = 0.2, input disturbance 0.5, noise σ = 0.001 on z<sub>m</sub> and θ<sub>m</sub>',
          html: `The chapter starts with the plant off by ${misText(prob.mismatch)} (a fixed 20% draw), d = ${prob.sim.dist} N and noise of 0.001 m on z and 0.001 rad (0.0573°) on θ. Run your E.13 controller here to see the bias it leaves.`,
          actions: [{ label: 'Run my E.13 controller', run: () => {
            const code = WB.myCtrl.savedCode(ctx, e13);
            if (!code) return { ok: false, msg: `Write your controller in ${e13.replace('/', '(')}) first (Ch 13 tab).` };
            return WB.myCtrl.use(ctx, code, 'a');
          } }],
        },
        WB.myCtrl.part(ctx, {
          id: 'b', title: '(b) Add a disturbance observer to the controller, verify that the steady-state error in the estimator has been removed, and tune for a good response',
          seed: [e13],
          html: `Return <code>(F, x_hat)</code> (or <code>(F, x_hat, d_hat)</code> to plot your d̂). The check runs the square wave (0.25 ± 0.15 m, 0.05 Hz) with the plant off by ${misText(prob.mismatch)}, d = ${prob.sim.dist} N and the noise above: over the last 2 s the mean of z − ẑ must be under 1 mm and the mean of θ − θ̂ under 0.05°, the mean |z<sub>r</sub> − z| over the 0.5 s before the second switch (t = 20 s) under 2 mm, and the block must stay on the beam.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.15), tEnd: 20, dist: prob.sim.dist, mismatch: prob.mismatch, noises: [0.001, 0.001], seed: 1 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const xh = E.xhatOf(res, ctx.sys.ze(ctx.pModel));
            if (!xh) return E.NEED_XHAT;
            const bz = biasOf(res, xh, 0, 18, 20), bth = biasOf(res, xh, 1, 18, 20) / M.DEG;
            const e = Math.abs(WB.myCtrl.mean(res, 19.45, 19.95, (k) => res.r[k] - res.yAll[0][k]));
            const ob = E.onBeamRes(res, sc.plantParams.ell);
            const ok = Math.abs(bz) < 0.001 && Math.abs(bth) < 0.05 && e < 0.002 && ob.ok;
            return { ok, msg: `Over the last 2 s: mean z − ẑ = ${fmt(1000 * bz, 3)} mm, mean θ − θ̂ = ${fmt(bth, 3)}°; mean |z_r − z| before the switch ${fmt(1000 * e, 3)} mm; ${ob.msg}.` };
          },
          solution: () => [
            { code: SOL.ch14 },
            { html: 'As the repo\'s ctrlDisturbanceObserver.py: the observer of the augmented model A₂ = [A B; 0 0], C₂ = [C 0] estimates x̃ and d, and F̃ subtracts d̂. L₂ = place(A₂ᵀ, C₂ᵀ, q)ᵀ with observer poles 5× faster than the controller pairs and a disturbance pole at −5. d̂ estimates the input disturbance plus every force the model gets wrong (the default mismatch changes the force needed to hold the beam by about 0.9 N), so it settles near d + ΔF, not d. That is fine: what matters is that x̂ is unbiased and F̃ cancels the total.' },
          ],
        }),
      ]);
    },
  });

  WB.E.ss = { knobsOf };
})();
