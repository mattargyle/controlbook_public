// Chapters 11-14: full state feedback, integral augmentation, observers, and
// disturbance observers on the feedback-linearized model
//   xdot = A x + B u~,  y = C x,  u = tau_fl(theta) + u~   (A.6 / A.11, p. 187)
window.WB = window.WB || {};
WB.chapters = WB.chapters || {};

(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const PD = () => WB.pd;

  // Solution controllers (the book's designs; anti-windup added in A.12 onward).
  const SOL = {
    ch11: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr = 0.489
        zeta = 0.707
        J = P.m * P.ell**2
        A = np.array([[0.0, 1.0], [0.0, -3 * P.b / J]])
        B = np.array([[0.0], [3 / J]])
        C = np.array([[1.0, 0.0]])
        wn = 2.2 / tr
        poles = np.roots([1, 2 * zeta * wn, wn**2])
        self.K = cnt.place(A, B, poles)
        self.kr = -1.0 / (C @ np.linalg.inv(A - B @ self.K) @ B)[0, 0]
        # dirty derivative for thetadot
        sigma = 0.05
        self.beta = (2 * sigma - P.Ts) / (2 * sigma + P.Ts)
        self.gamma = 2 / (2 * sigma + P.Ts)
        self.theta_prev = P.theta0
        self.theta_dot = P.thetadot0

    def update(self, theta_r, y):
        theta = y[0, 0]
        self.theta_dot = self.beta * self.theta_dot + self.gamma * (theta - self.theta_prev)
        self.theta_prev = theta
        x = np.array([[theta], [self.theta_dot]])
        tau_fl = P.m * P.g * P.ell / 2 * np.cos(theta)
        tau = tau_fl - (self.K @ x)[0, 0] + self.kr * theta_r
        return max(-P.tau_max, min(P.tau_max, tau))
`,
    ch12: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr = 0.489
        zeta = 0.707
        p_I = -5.0
        J = P.m * P.ell**2
        A = np.array([[0.0, 1.0], [0.0, -3 * P.b / J]])
        B = np.array([[0.0], [3 / J]])
        C = np.array([[1.0, 0.0]])
        A1 = np.block([[A, np.zeros((2, 1))], [-C, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        wn = 2.2 / tr
        poles = np.roots(np.convolve([1, 2 * zeta * wn, wn**2], [1, -p_I]))
        K1 = cnt.place(A1, B1, poles)
        self.K = K1[:, 0:2]
        self.ki = K1[0, 2]
        sigma = 0.05
        self.beta = (2 * sigma - P.Ts) / (2 * sigma + P.Ts)
        self.gamma = 2 / (2 * sigma + P.Ts)
        self.theta_prev = P.theta0
        self.theta_dot = P.thetadot0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, theta_r, y):
        theta = y[0, 0]
        self.theta_dot = self.beta * self.theta_dot + self.gamma * (theta - self.theta_prev)
        self.theta_prev = theta
        x = np.array([[theta], [self.theta_dot]])
        error = theta_r - theta
        tau_fl = P.m * P.g * P.ell / 2 * np.cos(theta)
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        tau = tau_fl - (self.K @ x)[0, 0] - self.ki * integ
        # anti-windup: keep the integrator still while the torque saturates
        if abs(tau) <= P.tau_max:
            self.integrator = integ
        self.error_prev = error
        tau = tau_fl - (self.K @ x)[0, 0] - self.ki * self.integrator
        return max(-P.tau_max, min(P.tau_max, tau))
`,
    ch13: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr = 0.4
        zeta = 0.707
        p_I = -9.0
        J = P.m * P.ell**2
        self.A = np.array([[0.0, 1.0], [0.0, -3 * P.b / J]])
        self.B = np.array([[0.0], [3 / J]])
        self.C = np.array([[1.0, 0.0]])
        A1 = np.block([[self.A, np.zeros((2, 1))], [-self.C, np.zeros((1, 1))]])
        B1 = np.vstack([self.B, [[0.0]]])
        wn = 2.2 / tr
        K1 = cnt.place(A1, B1, np.roots(np.convolve([1, 2 * zeta * wn, wn**2], [1, -p_I])))
        self.K = K1[:, 0:2]
        self.ki = K1[0, 2]
        wn_obs = 2.2 / (tr / 10)          # observer 10x faster
        zeta_obs = 0.707
        self.L = cnt.place(self.A.T, self.C.T, np.roots([1, 2 * zeta_obs * wn_obs, wn_obs**2])).T
        self.x_hat = np.zeros((2, 1))
        self.tau_prev = 0.0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, theta_r, y):
        x_hat = self.update_observer(y)
        theta_hat = x_hat[0, 0]
        error = theta_r - theta_hat
        tau_fl = P.m * P.g * P.ell / 2 * np.cos(theta_hat)
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        tau = tau_fl - (self.K @ x_hat)[0, 0] - self.ki * integ
        if abs(tau) <= P.tau_max:          # anti-windup
            self.integrator = integ
        self.error_prev = error
        tau = tau_fl - (self.K @ x_hat)[0, 0] - self.ki * self.integrator
        tau = max(-P.tau_max, min(P.tau_max, tau))
        self.tau_prev = tau
        return tau, x_hat

    def update_observer(self, y):
        F1 = self.observer_f(self.x_hat, y)
        F2 = self.observer_f(self.x_hat + P.Ts / 2 * F1, y)
        F3 = self.observer_f(self.x_hat + P.Ts / 2 * F2, y)
        F4 = self.observer_f(self.x_hat + P.Ts * F3, y)
        self.x_hat = self.x_hat + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y):
        tau_fl = P.m * P.g * P.ell / 2 * np.cos(x_hat[0, 0])
        return self.A @ x_hat + self.B * (self.tau_prev - tau_fl) + self.L @ (y - self.C @ x_hat)
`,
    ch14: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr = 0.4
        zeta = 0.95
        p_I = -9.0
        J = P.m * P.ell**2
        A = np.array([[0.0, 1.0], [0.0, -3 * P.b / J]])
        B = np.array([[0.0], [3 / J]])
        C = np.array([[1.0, 0.0]])
        A1 = np.block([[A, np.zeros((2, 1))], [-C, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        wn = 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        K1 = cnt.place(A1, B1, np.roots(np.convolve([1, 2 * zeta * wn, wn**2], [1, -p_I])))
        self.K = K1[:, 0:2]
        self.ki = K1[0, 2]
        # observer for x and the input disturbance d (d constant)
        self.A2 = np.block([[A, B], [np.zeros((1, 3))]])
        self.B2 = np.vstack([B, [[0.0]]])
        self.C2 = np.hstack([C, [[0.0]]])
        wn_obs = 10.0
        zeta_obs = 0.707
        p_d = -5.5
        poles = np.roots(np.convolve([1, 2 * zeta_obs * wn_obs, wn_obs**2], [1, -p_d]))
        self.L2 = cnt.place(self.A2.T, self.C2.T, poles).T
        self.obs = np.zeros((3, 1))
        self.tau_prev = 0.0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, theta_r, y):
        self.update_observer(y)
        x_hat = self.obs[0:2]
        d_hat = self.obs[2, 0]
        theta_hat = x_hat[0, 0]
        error = theta_r - theta_hat
        tau_fl = P.m * P.g * P.ell / 2 * np.cos(theta_hat)
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        tau = tau_fl - (self.K @ x_hat)[0, 0] - self.ki * integ - d_hat
        if abs(tau) <= P.tau_max:          # anti-windup
            self.integrator = integ
        self.error_prev = error
        tau = tau_fl - (self.K @ x_hat)[0, 0] - self.ki * self.integrator - d_hat
        tau = max(-P.tau_max, min(P.tau_max, tau))
        self.tau_prev = tau
        return tau, x_hat, d_hat

    def update_observer(self, y):
        F1 = self.observer_f(self.obs, y)
        F2 = self.observer_f(self.obs + P.Ts / 2 * F1, y)
        F3 = self.observer_f(self.obs + P.Ts / 2 * F2, y)
        F4 = self.observer_f(self.obs + P.Ts * F3, y)
        self.obs = self.obs + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def observer_f(self, z, y):
        tau_fl = P.m * P.g * P.ell / 2 * np.cos(z[0, 0])
        return self.A2 @ z + self.B2 * (self.tau_prev - tau_fl) + self.L2 @ (y - self.C2 @ z)
`,
  };

  // --------------------------------------------------------------- design --
  function wnOf(st) { return PD().wnFromTr(st.tr, st.zeta, st.rule || '2.2'); }
  function ctrlPoles(st) { return PD().polesFromWnZeta(wnOf(st), st.zeta); }
  function obsPoles(st) { return PD().polesFromWnZeta(st.wnObs, st.zetaObs); }

  const augI = (ss) => WB.design.augmentIntegrator(ss.A, ss.B, ss.C);
  const augD = (ss) => WB.design.augmentDisturbance(ss.A, ss.B, ss.C);

  // Designed gains from the tuning knobs (explore mode / problem solutions).
  function design(ctx, st = ctx.st, level = ctx.level) {
    const { A, B, C } = ctx.ss;
    const out = { poles: ctrlPoles(st) };
    if (level === 'sf') {
      const K = L.place(A, B, out.poles);
      out.K = K ? K[0] : [NaN, NaN];
      const Acl = L.sub(A, L.mul(B, [out.K]));
      const Ai = L.inv(Acl);
      out.kr = Ai ? -1 / L.mul(L.mul(C, Ai), B)[0][0] : NaN;
      return out;
    }
    const { A1, B1 } = augI(ctx.ss);
    out.poles = [...out.poles, { re: st.pI, im: 0 }];
    const K1 = L.place(A1, B1, out.poles);
    out.K = K1 ? [K1[0][0], K1[0][1]] : [NaN, NaN];
    out.ki = K1 ? K1[0][2] : NaN;
    if (level === 'obs') {
      out.obsPoles = obsPoles(st);
      const Lt = L.place(L.T(A), L.T(C), out.obsPoles);
      out.L = Lt ? [Lt[0][0], Lt[0][1]] : [NaN, NaN];
    }
    if (level === 'dobs') {
      const { A2, C2 } = augD(ctx.ss);
      out.obsPoles = [...obsPoles(st), { re: st.pD, im: 0 }];
      const Lt = L.place(L.T(A2), L.T(C2), out.obsPoles);
      out.L = Lt ? [Lt[0][0], Lt[0][1]] : [NaN, NaN];
      out.Ld = Lt ? Lt[0][2] : NaN;
    }
    return out;
  }

  // ----------------------------------------------------------- controller --
  // level: 'sf' (Ch 11), 'sfi' (Ch 12), 'obs' (Ch 13), 'dobs' (Ch 14)
  function makeSS(ctx, { linear = false } = {}) {
    const { sys, pModel, S } = ctx;
    const st = ctx.st, level = ctx.level, g = ctx.gains;
    const { A, B, C } = ctx.ss;
    const Ts = S.sim.Ts, uLim = sys.uLimit(pModel);
    // Work mode adds τ_fl only once A.4(c) is solved (PD().compApplied).
    const flOn = !linear && st.comp === 'fl' && PD().compApplied(ctx);
    const ff = (th) => (flOn ? sys.feedbackLinearization([th, 0], pModel) : 0);
    const useObs = level === 'obs' || level === 'dobs';
    const useDO = level === 'dobs' && st.dobs;
    const sigma = 0.05;
    const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    let I = 0, ePrev = null, yPrev = null, ydot = 0, uPrev = 0;
    let xh = [st.xhat0 * M.DEG, 0], dh = 0;

    // Observer right-hand side (Eq. 13.3 / p. 241), with tau_fl inside the model.
    function fObs(z, y) {
      const [t, w, d] = z;
      const innov = y - (C[0][0] * t + C[0][1] * w);
      const u = uPrev - ff(t) + (useDO ? d : 0);
      return [
        A[0][0] * t + A[0][1] * w + B[0][0] * u + g.L[0] * innov,
        A[1][0] * t + A[1][1] * w + B[1][0] * u + g.L[1] * innov,
        useDO ? g.Ld * innov : 0,
      ];
    }

    return {
      update(r, x, yMeas) {
        let xUse;
        if (useObs) {
          const z = M.rk4Step((zz) => fObs(zz, yMeas), [xh[0], xh[1], dh], 0, Ts);
          xh = [z[0], z[1]]; dh = z[2];
          xUse = xh;
        } else if (st.est === 'dirty') {
          if (yPrev === null) yPrev = yMeas;
          ydot = beta * ydot + gamma * (yMeas - yPrev);
          yPrev = yMeas;
          xUse = [yMeas, ydot];
        } else {
          xUse = x;
        }
        const Kx = g.K[0] * xUse[0] + g.K[1] * xUse[1];
        let u;
        if (level === 'sf') {
          u = ff(xUse[0]) - Kx + g.kr * r;
        } else {
          const e = r - xUse[0];
          if (ePrev === null) ePrev = 0; // repo: error_d1 = 0
          const uTry = ff(xUse[0]) - Kx - g.ki * (I + (Ts / 2) * (e + ePrev)) - (useDO ? dh : 0);
          // Anti-windup (A.12a): skip integration while the actuator is saturated.
          const saturated = !linear && Math.abs(uTry) > uLim;
          if (!(st.antiwindup === 'clamp' && saturated)) I += (Ts / 2) * (e + ePrev);
          ePrev = e;
          u = ff(xUse[0]) - Kx - g.ki * I - (useDO ? dh : 0);
        }
        uPrev = linear ? u : M.saturate(u, uLim);
        return { u, xhat0: xUse[0], xhat1: xUse[1], dhat: dh, integrator: I };
      },
    };
  }

  // ------------------------------------------------------------- controls --
  function tuningSliders(parent, ctx, { pI, rule } = {}) {
    slider(parent, { label: 't<sub>r</sub>', unit: 's', min: 0.1, max: 2, step: 0.001, ...bind(ctx, 'tr') });
    slider(parent, { label: 'ζ', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zeta') });
    if (rule) {
      segmented(parent, {
        label: 'ω<sub>n</sub> from t<sub>r</sub>',
        options: [{ value: '2.2', label: '2.2 / t<sub>r</sub>' }, { value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))' }],
        ...bind(ctx, 'rule'),
      });
    }
    if (pI) slider(parent, { label: 'p<sub>I</sub>', min: -30, max: -0.1, step: 0.01, sig: 3, ...bind(ctx, 'pI') });
  }

  function obsSliders(parent, ctx, withD) {
    slider(parent, { label: 'ω<sub>n,obs</sub>', unit: 'rad/s', min: 1, max: 150, step: 0.1, sig: 4, ...bind(ctx, 'wnObs') });
    slider(parent, { label: 'ζ<sub>obs</sub>', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zetaObs') });
    if (withD) slider(parent, { label: 'p<sub>d</sub>', min: -60, max: -0.1, step: 0.1, sig: 3, ...bind(ctx, 'pD') });
  }

  function gainsReadout(parent, ctx, keys) {
    WB.ui.readout(parent, () => {
      const g = ctx.gains;
      const vals = { K1: g.K[0], K2: g.K[1], kr: g.kr, ki: g.ki, L1: g.L && g.L[0], L2: g.L && g.L[1], Ld: g.Ld };
      return keys.map((k) => [k, vals[k]]);
    });
  }

  function gainsFor(ctx) {
    if (ctx.S.mode === 'explore') return design(ctx);
    const w = ctx.st.w;
    return { K: [w.K1, w.K2], kr: w.kr, ki: w.ki, L: [w.L1, w.L2], Ld: w.Ld };
  }

  function compControls(parent, ctx) {
    segmented(parent, {
      label: 'Gravity compensation',
      options: [{ value: 'fl', label: 'feedback lin. τ_fl(θ̂)' }, { value: 'none', label: 'none' }],
      ...bind(ctx, 'comp'),
    });
    PD().compNote(parent, ctx);
  }

  // ------------------------------------------------------------- analysis --
  function closedLoopPoles(ctx) {
    const { A, B } = ctx.ss;
    const g = ctx.gains;
    if (ctx.level === 'sf') return L.eig(L.sub(A, L.mul(B, [g.K])));
    const { A1, B1 } = augI(ctx.ss);
    return L.eig(L.sub(A1, L.mul(B1, [[g.K[0], g.K[1], g.ki]])));
  }
  function observerPoles(ctx) {
    const { A, C } = ctx.ss;
    const g = ctx.gains;
    if (ctx.level === 'obs') return L.eig(L.sub(A, L.mul([[g.L[0]], [g.L[1]]], C)));
    if (ctx.level === 'dobs') {
      const { A2, C2 } = augD(ctx.ss);
      return L.eig(L.sub(A2, L.mul([[g.L[0]], [g.L[1]], [g.Ld]], C2)));
    }
    return [];
  }

  // Each chapter's part whose answer the desired (target) poles give away.
  const TARGETS_PART = { ch11: ['ch11', 'a'], ch12: ['ch12', 'a'], ch13: ['ch13', 'c'], ch14: ['ch14', 'b'] };
  const targetsKey = (ctx) => { const [ch, part] = TARGETS_PART[ctx.S.chapter] || ['ch11', 'a']; return PD().partKey(ctx, ch, part); };

  function markers(ctx) {
    const { A } = ctx.ss;
    // In Work mode the open-loop poles answer A.7(a), and the target poles are
    // computed from the spec, so each stays off until its part is solved.
    const mk = PD().shows(ctx, PD().partKey(ctx, 'ch7', 'a')) ? L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole ${i + 1}` })) : [];
    const explore = ctx.S.mode === 'explore';
    // Work mode has no gains: the plots run the student's own controller.
    if (explore) closedLoopPoles(ctx).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `controller pole ${i + 1}`, dragId: explore ? (Math.abs(p.im) > 1e-9 ? 0 : 2) : undefined }));
    if (explore) observerPoles(ctx).forEach((p, i) => mk.push({ ...p, kind: 'obs', label: `observer pole ${i + 1}`, dragId: explore ? (Math.abs(p.im) > 1e-9 ? 10 : 12) : undefined }));
    // Ch 12-14's poles are the student's tuning (the book gives no values): no targets there.
    if (!explore && ctx.level === 'sf' && ctx.app.isSolved(targetsKey(ctx))) {
      const d = design(ctx);
      for (const p of d.poles) mk.push({ ...p, kind: 'target', label: 'target pole (problem)' });
    }
    return mk;
  }

  // Dragging: complex controller pole -> (t_r, ζ); real one -> p_I; observer the same.
  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    if (id === 0) {
      const wn = Math.hypot(re, im);
      st.zeta = Math.max(0.2, Math.min(1, -re / wn));
      st.tr = st.rule === 'tp' ? Math.PI / (2 * wn * Math.sqrt(Math.max(1e-6, 1 - st.zeta ** 2))) : 2.2 / wn;
    } else if (id === 2) {
      st.pI = re;
    } else if (id === 10) {
      st.wnObs = Math.hypot(re, im);
      st.zetaObs = Math.max(0.2, Math.min(1, -re / st.wnObs));
    } else if (id === 12) {
      if (ctx.level === 'dobs') st.pD = re;
    }
    ctx.update();
  }

  // ----------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B } = ctx.ss;
    return {
      title: 'State-space model (feedback linearized)', page: 'p. 88 · Eq. 6.16, p. 187', answers: `${ctx.sys.problems.ch6.id}/a`,
      theory: '\\dot x = Ax + B\\tilde\\tau,\\quad y = Cx,\\quad A = \\begin{bmatrix}0 & 1\\\\ 0 & -\\frac{3b}{m\\ell^2}\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\ \\frac{3}{m\\ell^2}\\end{bmatrix},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix}',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`,
    };
  }
  // Controllability card, shared with studies B–F. matrix: show C_AB; det: add det C_AB.
  function ctrbCard(A, B, title, page, { matrix = true, det = false, sig = 4 } = {}) {
    const Cab = L.ctrb(A, B);
    const nums = [...(matrix ? [`\\mathcal{C} = ${texMat(Cab, sig)}`] : []), `\\operatorname{rank} = ${L.rank(Cab)}`, ...(det ? [`\\det = ${tex(L.det(Cab))}`] : [])];
    return {
      title, page, theory: '\\mathcal{C}_{A,B} = \\begin{bmatrix} B & AB & \\cdots & A^{n-1}B\\end{bmatrix},\\quad \\text{controllable} \\iff \\operatorname{rank}\\mathcal{C}_{A,B} = n',
      numbers: nums.join(',\\quad '), spoiler: true,
    };
  }
  // Ch 12-14's poles are the student's tuning, so Work mode shows only the book's rule there.
  function polesCard(ctx, d) {
    const st = ctx.st;
    const wn = wnOf(st);
    const mine = ctx.S.mode === 'work' && ctx.level !== 'sf';
    return {
      title: 'Desired closed-loop poles', page: 'p. 113 · Eq. 8.5, p. 184', answers: targetsKey(ctx),
      theory: (st.rule === 'tp' && !mine ? '\\omega_n = \\frac{\\pi}{2t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}') + ',\\quad \\Delta^d_{cl} = (s^2 + 2\\zeta\\omega_n s + \\omega_n^2)' + (ctx.level === 'sf' ? '' : '(s - p_I)'),
      numbers: mine ? null : `\\omega_n = ${tex(wn)},\\quad \\Delta^d_{cl} = ${WB.tf.polyTex(L.polyFromRoots(d.poles))},\\quad p = ${d.poles.map((p) => texPole(p)).join(',\\;')}`,
      spoiler: true,
    };
  }

  // --------------------------------------------------------- chapter base --
  function base(level, num, title, pages, extra) {
    return Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages,
      level,
      controller(ctx, o) { return makeSS(ctx, o); },
      // Work mode simulates the student's controller (WB.myCtrl), from y only.
      implement: { feed: 'y', linear: false },
      gains(ctx) { ctx.level = level; return gainsFor(ctx); },
      // Work mode shows a ζ ray and rise-time target only where the book gives them (Ch 11).
      splane(ctx) { return { markers: markers(ctx), zetaRay: ctx.st.zeta < 1 && (level === 'sf' || ctx.S.mode !== 'work') ? ctx.st.zeta : null }; },
      onPoleDrag: onDrag,
      // The 2nd-order overshoot formula only applies without integrator/observer poles.
      targets(ctx) {
        if (level === 'sf') return { tr: ctx.st.tr, zeta: ctx.st.zeta };
        return ctx.S.mode === 'work' ? {} : { tr: ctx.st.tr };
      },
    }, extra);
  }

  // Work-mode starting gains: deliberately not the answers.
  const W0 = { K1: 0.2, K2: 0.05, kr: 0.2, ki: -0.5, L1: 20, L2: 200, Ld: 2 };

  // Work-mode control panel: the plots show the student's controller.
  function workBanner(parent, ctx, part) {
    WB.myCtrl.banner(section(parent, 'Your controller'), ctx, part);
  }

  // The workbench's controller at level with the problem's tuning `tune`, on a
  // check scenario (reference for WB.myCtrl.matchCheck).
  function refRun(ctx, sc, level, tune, st = {}) {
    const rc = WB.myCtrl.refCtx(ctx, sc, { level });
    rc.st = { ...ctx.st, comp: 'fl', est: 'dirty', antiwindup: 'clamp', xhat0: 0, dobs: true, ...tune, ...st };
    const d = design(rc, rc.st, level);
    rc.gains = { K: d.K, kr: d.kr, ki: d.ki, L: d.L, Ld: d.Ld };
    return WB.myCtrl.reference(ctx, sc, makeSS(rc));
  }

  // A.12(a) windup test: τ_max lowered so a large step saturates for seconds.
  const WINDUP = { tauMax: 0.76, step: 60, os: 5 };
  // A.12(a) integrator test: a constant input disturbance [N·m] the integrator must remove.
  const INTEG = { d: 0.25 };
  const SOL_A12A = String.raw`import control as cnt

def gains(p_I):
    tr, zeta = 0.489, 0.707          # A.11
    J = P.m * P.ell**2
    A = np.array([[0.0, 1.0], [0.0, -3 * P.b / J]])
    B = np.array([[0.0], [3 / J]])
    C = np.array([[1.0, 0.0]])
    A1 = np.block([[A, np.zeros((2, 1))], [-C, np.zeros((1, 1))]])
    B1 = np.vstack([B, [[0.0]]])
    wn = 2.2 / tr
    K1 = cnt.place(A1, B1, np.roots(np.convolve([1, 2 * zeta * wn, wn**2], [1, -p_I])))
    return K1[0, 0], K1[0, 1], K1[0, 2]
`;
  const SOL_A13C = String.raw`import control as cnt

def L_obs(wn_obs, zeta_obs):
    J = P.m * P.ell**2
    A = np.array([[0.0, 1.0], [0.0, -3 * P.b / J]])
    C = np.array([[1.0, 0.0]])
    L = cnt.place(A.T, C.T, np.roots([1, 2 * zeta_obs * wn_obs, wn_obs**2])).T
    return L[0, 0], L[1, 0]
`;
  const SQUARE = (amplitude) => ({ type: 'square', amplitude, frequency: 0.05, tStep: 0 });
  const errAt = (res, t) => { const k = Math.round(t / (res.t[1] - res.t[0])); return Math.abs(res.r[k] - res.y[k]) / M.DEG; };
  // The student's estimate must come back from update as (u, x_hat).
  function needXhat(res) {
    const a = res.extras.xhat0, b = res.extras.xhat1;
    if (!a || !b || Array.prototype.some.call(a, (v) => !Number.isFinite(v))) return { ok: false, msg: 'Return (tau, x_hat) from update, with x_hat = [[theta_hat], [thetadot_hat]], so the check can see your estimate.' };
    return null;
  }
  // Largest |θ − θ̂| (deg) and |θ̇ − θ̇̂| (deg/s) over the given time windows.
  function estErr(res, windows) {
    let e0 = 0, e1 = 0;
    res.t.forEach((t, k) => {
      if (!windows.some(([a, b]) => t >= a - 1e-9 && t <= b + 1e-9)) return;
      e0 = Math.max(e0, Math.abs(res.x[k][0] - res.extras.xhat0[k]));
      e1 = Math.max(e1, Math.abs(res.x[k][1] - res.extras.xhat1[k]));
    });
    return { th: e0 / M.DEG, thd: e1 / M.DEG };
  }
  const bias = (res, t0, t1) => WB.myCtrl.mean(res, t0, t1, (k) => res.x[k][0] - res.extras.xhat0[k]) / M.DEG;


  // ------------------------------------------------------------ Chapter 11 --
  WB.chapters.ch11 = base('sf', 11, 'Full state feedback', 'pp. 173–196', {
    defaults(sys) { const p = sys.problems.ch11; return { comp: 'fl', est: 'true', tr: p.tr, zeta: p.zeta, rule: '2.2', w: { ...W0 } }; },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, `${ctx.sys.problems.ch11.id}(e)`); return; }
      const sec = section(parent, 'State feedback u = −Kx + k_r r', 'p. 173 · Eq. 11.3, p. 183 · Eq. 11.38');
      compControls(sec, ctx);
      segmented(sec, {
        label: 'Where x comes from',
        options: [{ value: 'true', label: 'true state (repo)' }, { value: 'dirty', label: 'θ + dirty derivative (A.11e)' }],
        ...bind(ctx, 'est'),
      });
      tuningSliders(sec, ctx); gainsReadout(sec, ctx, ['K1', 'K2', 'kr']);
    },
    math(ctx) {
      const d = design(ctx);
      const { A, B } = ctx.ss;
      const g = ctx.gains;
      return [
        ssCard(ctx),
        { ...ctrbCard(A, B, 'Controllability', 'p. 180 · Eq. 11.29'), answers: PD().partKey(ctx, 'ch11', 'c') },
        polesCard(ctx, d),
        { title: 'Pole placement (Ackermann)', page: 'p. 182 · Eq. 11.32', answers: PD().partKey(ctx, 'ch11', 'd'),
          theory: 'K = (\\alpha - a_A)\\,\\mathcal{A}_A^{-1}\\,\\mathcal{C}_{A,B}^{-1}',
          numbers: `K = ${texMat([d.K])}\\quad(\\text{eig}(A - BK) = ${d.poles.map((p) => texPole(p)).join(',\\;')})`, spoiler: true },
        { title: 'Reference gain', page: 'p. 182 · Eq. 11.35', answers: PD().partKey(ctx, 'ch11', 'd'),
          theory: 'k_r = \\frac{-1}{C(A - BK)^{-1}B}',
          numbers: `k_r = ${tex(d.kr)}`, spoiler: true,
          note: 'For this plant k_r = K₁: with a free integrator in the plant, unity DC gain needs the reference to enter exactly like the position feedback.' },
        { title: 'Control law with feedback linearization', page: 'p. 183 · Eq. 11.38', answers: PD().partKey(ctx, 'ch11', 'e'),
          theory: '\\tau = \\tau_{fl}(\\theta) - Kx + k_r\\theta_r',
          numbers: ctx.S.mode === 'work' ? null : `\\tau = \\tau_{fl}(\\theta) - (${tex(g.K[0])}\\,\\theta + ${tex(g.K[1])}\\,\\dot\\theta) + ${tex(g.kr)}\\,\\theta_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch11;
      const ref = () => design(ctx, { ...ctx.st, tr: prob.tr, zeta: prob.zeta, rule: '2.2' }, 'sf');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) Desired closed-loop poles (t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta})`,
          inputs: { p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>' },
          html: 'Use ω<sub>n</sub> = 2.2/t<sub>r</sub>. Complex values are fine: <code>-1+2j</code>.',
          check: (v) => {
            const g = [M.parseComplex(v.p1), M.parseComplex(v.p2)];
            if (!g[0] || !g[1]) return { ok: false, msg: 'Enter both poles.' };
            return M.polesMatch(g, ref().poles) ? { ok: true, msg: '' } : { ok: false, msg: 'Roots of s² + 2ζωₙs + ωₙ²?' };
          },
          solution: () => [{ tex: `\\omega_n = 2.2/${prob.tr} = ${tex(2.2 / prob.tr)},\\quad p = ${ref().poles.map((q) => texPole(q)).join(',\\;')}` }],
        },
        {
          id: 'b', title: '(b) State-space model',
          html: 'Use your A, B, C, D from A.6 (the Ch 6 tab). The model card in the Math section unlocks once A.6 is solved.',
        },
        {
          id: 'c', title: '(c) Controllability',
          inputs: { c11: 'c<sub>11</sub>', c12: 'c<sub>12</sub>', c21: 'c<sub>21</sub>', c22: 'c<sub>22</sub>', rank: 'rank' },
          check: (v) => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return PD().checkNumbers(v, { c11: Cm[0][0], c12: Cm[0][1], c21: Cm[1][0], c22: Cm[1][1], rank: L.rank(Cm) }, {}); },
          solution: () => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return [{ tex: `\\mathcal{C}_{A,B} = ${texMat(Cm)},\\; \\det = ${tex(Cm[0][0] * Cm[1][1] - Cm[0][1] * Cm[1][0])} \\ne 0` }]; },
        },
        {
          id: 'd', title: `(d) K and k<sub>r</sub> for t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta}`,
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { K1: r.K[0], K2: r.K[1], kr: r.kr }, {}); },
          solution: () => {
            const r = ref();
            return [
              { tex: `\\Delta^d = ${WB.tf.polyTex(L.polyFromRoots(r.poles))},\\quad K = ${texMat([r.K])},\\quad k_r = ${tex(r.kr)}` },
            ];
          },
        },
        whyPart(ctx, {
          id: 'd2', title: '(d) Why is K = (k<sub>P</sub>, k<sub>D</sub>) when the poles match A.8?',
          html: 'Answer with two functions of the gains: the closed-loop characteristic polynomial under τ̃ = −Kx + k<sub>r</sub>θ<sub>r</sub>, and the k<sub>r</sub> that makes the DC gain one. Then compare them with your A.7(b) PD loop. Any nonzero multiple of the polynomial is accepted.',
          ranges: { K1: [0.02, 2], K2: [0.01, 0.5] },
          explain: 'Same polynomial as the PD loop of A.7(b) with k<sub>P</sub> = K<sub>1</sub>, k<sub>D</sub> = K<sub>2</sub>, and k<sub>r</sub> = K<sub>1</sub>: with x = (θ, θ̇), −Kx + k<sub>r</sub>θ<sub>r</sub> = k<sub>P</sub>(θ<sub>r</sub> − θ) − k<sub>D</sub>θ̇, which is exactly PD with the derivative on the output (Fig. 7-2). Book: p. 187.',
          code: 'b0 = 3 / (P.m * P.ell**2)\na1 = 3 * P.b / (P.m * P.ell**2)\n\ndef char_poly(s, K1, K2):\n    return s**2 + (a1 + b0 * K2) * s + b0 * K1\n\ndef k_r(K1, K2):\n    return K1     # -1 / (C (A - B K)^-1 B)',
        }),
        WB.myCtrl.part(ctx, {
          id: 'e', title: '(e) Implement the state-feedback controller, using a digital differentiator for θ̇',
          seed: `${ctx.sys.problems.ch10.id}/c`,
          html: `Your controller gets only the measured θ. The check runs the ±30° square wave with the nominal and with other parameters and compares θ(t) with the design for t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta} (within 3%).`,
          check: (code) => {
            const tune = { tr: prob.tr, zeta: prob.zeta, rule: '2.2' };
            const cases = WB.myCtrl.paramCases(ctx).map((pc) => {
              const sc = WB.myCtrl.scenario(ctx, { params: pc.params, ref: SQUARE(30), tEnd: 12 });
              return { sc, label: pc.label, ref: () => refRun(ctx, sc, 'sf', tune) };
            });
            return WB.myCtrl.matchCheck(ctx, code, cases, { tol: 0.03 * 30 * M.DEG });
          },
          solution: () => [{ code: SOL.ch11 }, { html: 'As the repo\'s ctrlStateFeedback.py, with θ̇ from the dirty derivative (σ = 0.05) instead of the true state.' }],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 12 --
  WB.chapters.ch12 = base('sfi', 12, 'Integrator with state feedback', 'pp. 197–214', {
    defaults(sys) { const p = sys.problems.ch12; return { comp: 'fl', est: 'true', tr: p.tr, zeta: p.zeta, rule: '2.2', pI: p.pI, antiwindup: 'clamp', w: { ...W0 } }; },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, `${ctx.sys.problems.ch12.id}(a) and (c)`); return; }
      const sec = section(parent, 'u = −Kx − k_i ∫(r − y)', 'p. 199');
      compControls(sec, ctx);
      segmented(sec, {
        label: 'Anti-windup (A.12a)',
        options: [{ value: 'clamp', label: 'hold integrator while saturated' }, { value: 'none', label: 'none (repo)' }],
        ...bind(ctx, 'antiwindup'),
      });
      tuningSliders(sec, ctx, { pI: true }); gainsReadout(sec, ctx, ['K1', 'K2', 'ki']);
    },
    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [rad·s]', unit: 'rad·s' }, data: { series: [{ label: 'x_I = ∫(r − θ)', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = design(ctx);
      const { A1, B1 } = augI(ctx.ss);
      return [
        ssCard(ctx),
        // A₁, B₁ hold the numbers of A and B (A.6).
        { title: 'Augmented system', page: 'p. 198 · Eq. 12.1', answers: PD().partKey(ctx, 'ch6', 'a'),
          theory: '\\dot x_I = r - C_r x,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C_r & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\ 0\\end{bmatrix}',
          numbers: `A_1 = ${texMat(A1)},\\quad B_1 = ${texMat(B1)}` },
        { ...ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 198'), answers: PD().partKey(ctx, 'ch6', 'a') },
        polesCard(ctx, d),
        { title: 'Gains', page: 'p. 199–201', answers: PD().partKey(ctx, 'ch12', 'a'),
          theory: 'K_1 = \\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad u = -Kx - k_I\\int_0^t (r - y)\\,d\\tau',
          numbers: ctx.S.mode === 'work' ? null : `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch12;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Gains for the A.11 poles plus an integrator pole p<sub>I</sub>',
          html: `Keep the A.11 poles (t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta}) and add an integrator pole p<sub>I</sub> &lt; 0 of your choice. Write K<sub>1</sub>, K<sub>2</sub> and k<sub>I</sub> as a function of p<sub>I</sub>; the check calls it at random p<sub>I</sub>.`,
          code: {
            template: 'def gains(p_I):\n    # returns K1, K2, kI\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { pI: { label: 'p_I', lo: -15, hi: -1 } },
              items: [{ fn: 'gains', args: ['pI'], label: 'gains(p_I)', truth: (p, a) => {
                const sub = { ...ctx, ss: ctx.sys.stateSpace(p) };
                const r = design(sub, { ...ctx.st, tr: prob.tr, zeta: prob.zeta, pI: a.pI, rule: '2.2' }, 'sfi');
                return [r.K[0], r.K[1], r.ki];
              } }],
            }, code),
          },
          solution: () => [
            { code: SOL_A12A },
            { html: 'The book\'s printed numbers (p. 204–205: K = (1.0370, 0.1817), k<sub>I</sub> = −2.2687) correspond to t<sub>r</sub> = 0.4, not the 0.489 in its own listing and the repo, and use a<sub>A1</sub> = 0.6174 where A gives 0.667. The repo uses p<sub>I</sub> = −5.' },
          ],
        },
        WB.myCtrl.part(ctx, {
          id: 'a2', title: '(a) Add the integrator with anti-windup to your A.11 controller', seed: `${ctx.sys.problems.ch11.id}/e`,
          html: `Use your gains from above. The check (1) adds d = ${INTEG.d} N·m with the nominal and with other parameters: on the ±10° square wave the error just before the first switch must be under 0.1°, so the integrator removes the disturbance. Then (2) it lowers τ<sub>max</sub> to ${WINDUP.tauMax} N·m and steps θ<sub>r</sub> to ${WINDUP.step}°, so τ saturates for seconds: θ may overshoot by at most ${WINDUP.os}°.`,
          check: async (code) => {
            for (const pc of WB.myCtrl.paramCases(ctx)) {
              const r = await WB.myCtrl.run(ctx, code, WB.myCtrl.scenario(ctx, { params: pc.params, ref: SQUARE(10), tEnd: 10, dist: INTEG.d }));
              if (r.ok === false) return r;
              const e = errAt(r, 9.95);
              if (!(e < 0.1)) return { ok: false, msg: `With d = ${INTEG.d} N·m (${pc.label}) the error before the switch is ${fmt(e, 3)}°: the integrator should remove a constant disturbance${pc.params === ctx.pModel ? '' : ' (compute the gains from P.m, P.ell, P.b)'}.` };
            }
            const m = { msg: `The integrator removes d = ${INTEG.d} N·m (nominal and other parameters).` };
            const sc = WB.myCtrl.scenario(ctx, { params: { ...ctx.pModel, tau_max: WINDUP.tauMax }, ref: { type: 'step', amplitude: WINDUP.step, tStep: 0 }, tEnd: 15 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const os = Math.max(...res.y) / M.DEG - WINDUP.step;
            if (!(os <= WINDUP.os)) return { ok: false, msg: `The integrator works, but with τ_max = ${WINDUP.tauMax} N·m the ${WINDUP.step}° step overshoots by ${fmt(os, 3)}°: the integrator winds up while τ is saturated.` };
            return { ok: true, msg: `${m.msg} Saturated ${WINDUP.step}° step: overshoot ${fmt(Math.max(0, os), 3)}°.` };
          },
          solution: () => [{ code: SOL.ch12 }, { html: 'Anti-windup here holds the integrator while τ is saturated. Integrating only while |θ̇| is small (Listing 10.2) or unwinding by (τ<sub>sat</sub> − τ<sub>unsat</sub>)/k<sub>I</sub> pass too. The repo\'s ctrlStateFeedbackIntegrator.py has none.' }],
        }),
        {
          id: 'b', title: '(b) Disturbance and 20% uncertainty',
          html: 'Set an input disturbance d and the true-plant mismatch in the left panel (the chapter starts with d = 0.25 N·m and a 20% draw).',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch).some((v) => Math.abs(v) > 0);
            return Math.abs(S.sim.dist) > 0 && mis ? { ok: true, msg: `d = ${fmt(S.sim.dist, 3)} N·m with plant mismatch.` } : { ok: false, msg: 'Set both d ≠ 0 and a plant mismatch.' };
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Tune the integrator pole (and other gains if needed) for good tracking', seed: [`${ctx.sys.problems.ch12.id}/a2`],
          html: `The check runs the ±30° square wave with d = ${prob.sim.dist} N·m and the plant off by ${Object.entries(prob.mismatch).map(([k, v]) => `${k} ${v > 0 ? '+' : ''}${v}%`).join(', ')}: the error just before the first switch (t = 10 s) must be under 0.1°.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(30), tEnd: 10, dist: prob.sim.dist, mismatch: prob.mismatch });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const e = errAt(res, 9.95);
            return { ok: e < 0.1, msg: `Error before the switch: ${fmt(e, 3)}°.` };
          },
          solution: () => [{ code: SOL.ch12 }, { html: `The design from (a) (p<sub>I</sub> = ${prob.pI}) already passes.` }],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 13 --
  WB.chapters.ch13 = base('obs', 13, 'Observers', 'pp. 215–238', {
    defaults(sys) {
      const p = sys.problems.ch13;
      return { comp: 'fl', tr: p.tr, zeta: p.zeta, rule: '2.2', pI: p.pI, antiwindup: 'clamp', wnObs: 2.2 / (p.tr / p.trObsFactor), zetaObs: p.zetaObs, xhat0: 0, w: { ...W0 } };
    },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries(ctx, res, deg) { return [{ label: 'θ̂ (observer)', y: deg(res.extras.xhat0 || []), color: '--series-3', dash: [3, 3], width: 2 }]; },
    extraPlot(ctx, res) {
      const k = 180 / Math.PI;
      return {
        opts: { title: 'θ̇(t) and estimate', yLabel: 'θ̇ [°/s]', unit: '°/s' },
        data: { series: [
          { label: 'θ̇̂ (observer)', y: Array.from(res.extras.xhat1 || [], (v) => v * k), color: '--series-3', dash: [3, 3], width: 2 },
          { label: 'true θ̇', y: res.x.map((x) => x[1] * k), color: '--series-1' },
        ] },
      };
    },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, `${ctx.sys.problems.ch13.id}(c)`); return; }
      const sec = section(parent, 'Controller (uses x̂)', 'p. 222 · Fig. 13-3');
      compControls(sec, ctx);
      tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer', 'p. 216 · Eq. 13.3');
      obsSliders(ob, ctx, false); gainsReadout(ob, ctx, ['K1', 'K2', 'ki', 'L1', 'L2']);
      slider(ob, { label: 'θ̂(0)', unit: '°', min: -60, max: 60, step: 1, sig: 3, hint: 'initial estimate (true θ starts at the left panel value)', ...bind(ctx, 'xhat0') });
    },
    math(ctx) {
      const d = design(ctx);
      const { A, C } = ctx.ss;
      const O = L.obsv(A, C);
      return [
        ssCard(ctx),
        { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 224', answers: PD().partKey(ctx, 'ch13', 'c2'),
          theory: '\\dot{\\hat x} = A\\hat x + B(u - \\tau_{fl}(\\hat\\theta)) + L(y - C\\hat x),\\quad \\dot e = (A - LC)e' },
        { title: 'Observability', page: 'p. 221', answers: PD().partKey(ctx, 'ch13', 'b'),
          theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ \\vdots \\\\ CA^{n-1}\\end{bmatrix},\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = n',
          numbers: `\\mathcal{O} = ${texMat(O)},\\quad \\operatorname{rank} = ${L.rank(O)}`, spoiler: true },
        { title: 'Observer gain', page: 'p. 221 · Eq. 13.16, p. 222', answers: PD().partKey(ctx, 'ch13', 'c'),
          theory: 'L = \\mathcal{O}_{A,C}^{-1}\\mathcal{A}_A^{-T}(\\beta - a_A)^\\top = \\text{place}(A^\\top, C^\\top, q)^\\top',
          numbers: ctx.S.mode === 'work' ? null : `q = ${d.obsPoles.map((p) => texPole(p)).join(',\\;')},\\quad L = ${texMat(d.L)}`, spoiler: true },
        { title: 'Separation principle', page: 'p. 223–224',
          theory: '\\begin{bmatrix}\\dot x\\\\ \\dot e\\end{bmatrix} = \\begin{bmatrix}A - BK & BK\\\\ 0 & A - LC\\end{bmatrix}\\begin{bmatrix}x\\\\ e\\end{bmatrix} \\Rightarrow \\text{eig} = \\text{eig}(A - BK) \\cup \\text{eig}(A - LC)',
          note: 'Holds for the linear model only. Saturation, mismatch and the nonlinear τ_fl break it (p. 224).' },
        polesCard(ctx, d),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch13;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Exact parameters, no disturbance',
          html: 'The chapter starts with α = 0 and d = 0. <em>Exact model</em> in the left panel restores them.',
        },
        {
          id: 'b', title: '(b) Observability',
          inputs: { rank: 'rank 𝒪<sub>A,C</sub>' },
          check: (v) => PD().checkNumbers(v, { rank: L.rank(L.obsv(ctx.ss.A, ctx.ss.C)) }, {}),
          solution: () => [{ tex: `\\mathcal{O}_{A,C} = ${texMat(L.obsv(ctx.ss.A, ctx.ss.C))} \\Rightarrow \\text{rank } 2` }],
        },
        {
          id: 'c', title: '(c) Observer gain L for observer poles set by ω<sub>n,obs</sub>, ζ<sub>obs</sub>',
          html: 'You choose the observer poles (Δ<sub>obs</sub> = s² + 2ζ<sub>obs</sub>ω<sub>n,obs</sub>s + ω<sub>n,obs</sub>²). Write L<sub>1</sub>, L<sub>2</sub> as a function of ω<sub>n,obs</sub> and ζ<sub>obs</sub>; the check calls it at random values.',
          code: {
            template: 'def L_obs(wn_obs, zeta_obs):\n    # returns L1, L2\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { wn: { label: 'wn_obs', lo: 5, hi: 100 }, zeta: { label: 'zeta_obs', lo: 0.4, hi: 0.95 } },
              items: [{ fn: 'L_obs', args: ['wn', 'zeta'], label: 'L_obs', truth: (p, a) => {
                const { A, C } = ctx.sys.stateSpace(p);
                return WB.design.observerGain(A, C, PD().polesFromWnZeta(a.wn, a.zeta)).map((r) => r[0]);
              } }],
            }, code),
          },
          solution: () => [{ code: SOL_A13C }, { html: `The repo's ctrlObserver.py makes the observer 10× faster than its controller: ω<sub>n,obs</sub> = 2.2/(t<sub>r</sub>/${prob.trObsFactor}), ζ<sub>obs</sub> = ${prob.zetaObs} with t<sub>r</sub> = ${prob.tr}.` }],
        },
        WB.myCtrl.part(ctx, {
          id: 'c2', title: '(c) Add the observer and use x̂ in your A.12 controller; tune', seed: [`${ctx.sys.problems.ch12.id}/c`, `${ctx.sys.problems.ch12.id}/a2`],
          html: 'Return <code>(tau, x_hat)</code> from <code>update</code>. The check starts the arm at 10° (your estimate starts wherever you start it) and runs the ±30° square wave with exact parameters: before each switch, |θ − θ̂| must be under 0.01° and |θ̇ − θ̇̂| under 0.1°/s, and the error just before the first switch under 0.1°.',
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(30), tEnd: 20, y0: 10 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const ee = estErr(res, [[8, 9.95], [18, 20]]);
            const e = errAt(res, 9.95);
            const msg = `Before the switches: |θ − θ̂| ≤ ${fmt(ee.th, 3)}°, |θ̇ − θ̇̂| ≤ ${fmt(ee.thd, 3)}°/s; error before the first switch ${fmt(e, 3)}°.`;
            return { ok: ee.th < 0.01 && ee.thd < 0.1 && e < 0.1, msg };
          },
          solution: () => [{ code: SOL.ch13 }, { html: 'As the repo\'s ctrlObserver.py (observer 10× faster than the controller, RK4 with the previous saturated τ), plus the A.12 anti-windup.' }],
        }),
        {
          id: 'd', title: '(d) Plot the state and the estimate',
          html: 'Run your controller: the θ plot shows your θ̂ with the true θ, and the extra plot your θ̇̂ with the true θ̇.',
        },
        {
          id: 'e', title: '(e) Add d = 0.01 N·m',
          html: 'Runs your controller from (c) with d = 0.01 N·m and exact parameters. Your observer has no model of d, so x̂ is biased, and an integrator acting on θ̂ leaves θ off r. Passes when the run shows the bias (Chapter 14 removes it).',
          check: async () => {
            const code = WB.myCtrl.savedCode(ctx, `${prob.id}/c2`);
            if (!code) return { ok: false, msg: 'Write your controller in (c) first.' };
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(30), tEnd: 20, dist: 0.01 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const b = bias(res, 18, 20), e = WB.myCtrl.mean(res, 18, 20, (k) => res.y[k] - res.r[k]) / M.DEG;
            const msg = `With d = 0.01 N·m: θ − θ̂ = ${fmt(b, 3)}° and θ − θ_r = ${fmt(e, 3)}° at the end.`;
            if (Math.abs(b) < 1e-3) return { ok: false, msg: `${msg} θ̂ follows θ exactly: is x̂ coming from an observer of the model?` };
            return { ok: true, msg };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 14 --
  WB.chapters.ch14 = base('dobs', 14, 'Disturbance observers', 'pp. 239–259', {
    defaults(sys) {
      const p = sys.problems.ch14;
      return { comp: 'fl', tr: p.tr, zeta: p.zeta, rule: 'tp', pI: p.pI, antiwindup: 'clamp', wnObs: p.wnObs, zetaObs: p.zetaObs, pD: p.pD, dobs: true, xhat0: 0, w: { ...W0 } };
    },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries(ctx, res, deg) { return [{ label: 'θ̂ (observer)', y: deg(res.extras.xhat0 || []), color: '--series-3', dash: [3, 3], width: 2 }]; },
    extraPlot(ctx, res) {
      const S = ctx.S;
      return {
        opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N·m]', unit: 'N·m' },
        data: { series: [
          { label: 'd̂ (estimate)', y: Array.from(res.extras.dhat || []), color: '--series-3', width: 2 },
          { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
        ] },
      };
    },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, `${ctx.sys.problems.ch14.id}(b)`); return; }
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'p. 245');
      compControls(sec, ctx);
      segmented(sec, {
        label: 'Disturbance observer',
        options: [{ value: true, label: 'on' }, { value: false, label: 'off (A.14a)' }],
        ...bind(ctx, 'dobs'),
      });
      tuningSliders(sec, ctx, { pI: true, rule: true });
      const ob = section(parent, 'Observer', 'p. 241');
      obsSliders(ob, ctx, true); gainsReadout(ob, ctx, ['K1', 'K2', 'ki', 'L1', 'L2', 'Ld']);
    },
    math(ctx) {
      const d = design(ctx);
      const { A2, C2 } = augD(ctx.ss);
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(u + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0 \\text{ for constant } d' },
        { title: 'Augmented model (ḋ = 0)', page: 'p. 240', answers: PD().partKey(ctx, 'ch6', 'a'),
          theory: 'A_2 = \\begin{bmatrix}A & B\\\\ 0 & 0\\end{bmatrix},\\quad C_2 = \\begin{bmatrix}C & 0\\end{bmatrix}',
          numbers: `A_2 = ${texMat(A2)},\\quad \\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}` },
        { title: 'Disturbance observer', page: 'p. 241', answers: PD().partKey(ctx, 'ch14', 'b'),
          theory: '\\dot{\\hat x} = A\\hat x + B(u + \\hat d) + L(y - C\\hat x),\\quad \\dot{\\hat d} = L_d(y - C\\hat x),\\quad u = -K\\hat x - k_I\\textstyle\\int e - \\hat d' },
        { title: 'Observer gains', page: 'p. 241', answers: PD().partKey(ctx, 'ch14', 'b'),
          theory: '\\begin{bmatrix}L\\\\ L_d\\end{bmatrix} = \\text{place}(A_2^\\top, C_2^\\top, q)^\\top',
          numbers: ctx.S.mode === 'work' ? null : `q = ${d.obsPoles.map((p) => texPole(p)).join(',\\;')},\\quad L = ${texMat(d.L)},\\; L_d = ${tex(d.Ld)}`, spoiler: true,
          note: 'The A.14 solution\'s observer (|q| ≈ 5.5–10) is slower than its controller (ω_n ≈ 12.6), the opposite of the usual "observer 5–10× faster" rule.' },
        polesCard(ctx, d),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch14;
      const a13 = `${ctx.sys.problems.ch13.id}/c2`;
      const mis = Object.entries(prob.mismatch).map(([k, v]) => `${k} ${v > 0 ? '+' : ''}${v}%`).join(', ');
      const noise = prob.sim.noise * M.DEG;  // display ° -> rad
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Large disturbance, noise, 20% uncertainty',
          html: `The chapter starts with the plant off by ${mis}, d = ${prob.sim.dist} N·m and noise σ = ${fmt(noise, 3)} rad (${prob.sim.noise}°). Run your A.13 controller here to see the bias it leaves.`,
          actions: [{ label: 'Run my A.13 controller', run: () => {
            const code = WB.myCtrl.savedCode(ctx, a13);
            if (!code) return { ok: false, msg: `Write your controller in ${a13.replace('/', '(')}) first (Ch 13 tab).` };
            return WB.myCtrl.use(ctx, code, 'a');
          } }],
        },
        WB.myCtrl.part(ctx, {
          id: 'b', title: '(b) Add a disturbance observer, verify the estimator\'s steady-state error is removed, and tune', seed: [a13],
          html: `Return <code>(tau, x_hat)</code> (or <code>(tau, x_hat, d_hat)</code> to plot your d̂). The check runs the ±30° square wave with the plant off by ${mis}, d = ${prob.sim.dist} N·m and noise σ = ${fmt(noise, 3)} rad: the mean of θ − θ̂ over the last 2 s must be under 0.05°, and the mean error over the 0.5 s before the first switch under 0.1°.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(30), tEnd: 20, dist: prob.sim.dist, mismatch: prob.mismatch, noise, seed: 1 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const b = bias(res, 18, 20);
            const e = WB.myCtrl.mean(res, 9.45, 9.95, (k) => res.r[k] - res.y[k]) / M.DEG;
            return { ok: Math.abs(b) < 0.05 && Math.abs(e) < 0.1, msg: `Mean θ − θ̂ over the last 2 s: ${fmt(b, 3)}°; mean error before the first switch: ${fmt(e, 3)}°.` };
          },
          solution: () => [
            { code: SOL.ch14 },
            { html: `As the repo's ctrlDisturbanceObserver.py (ω<sub>n,obs</sub> = ${prob.wnObs}, ζ<sub>obs</sub> = ${prob.zetaObs}, disturbance pole ${prob.pD}), plus anti-windup. d̂ estimates the input disturbance plus any torque the model gets wrong (mismatch, gravity residual), so it settles near d but not exactly on it.` },
          ],
        }),
      ]);
    },
  });

  // X.11(d)'s "why is K = (k_P, k_D)?" (A and D): the closed-loop characteristic
  // polynomial under u = −Kx + k_r r and the unit-DC-gain k_r, as Python functions
  // of K, checked at random gains. Comparing them with the X.7(b) PD loop is the why.
  // o: { id, title, html, ranges: {K1: [lo, hi], K2: [lo, hi]}, explain (html), code }
  function whyPart(ctx, o) {
    const cx = WB.py.cx;
    const AB = (p, a) => { const { A, B, C } = ctx.sys.stateSpace(p); return { A, B, C, K: [a.K1, a.K2] }; };
    return {
      id: o.id, title: o.title, html: o.html,
      code: {
        template: 'def char_poly(s, K1, K2):\n    # det(sI - (A - B K)) with K = [K1, K2]\n    return ...\n\ndef k_r(K1, K2):\n    return ...\n',
        check: async (code) => {
          const r = await WB.py.check(ctx, {
            args: { s: { label: 's', complex: true, re: [-6, 3], im: [0.3, 12] }, K1: { label: 'K1', lo: o.ranges.K1[0], hi: o.ranges.K1[1] }, K2: { label: 'K2', lo: o.ranges.K2[0], hi: o.ranges.K2[1] } },
            items: [
              { fn: 'char_poly', args: ['s', 'K1', 'K2'], compare: 'scale', truth: (p, a) => { const m = AB(p, a); return cx.poly(L.charPoly(L.sub(m.A, L.mul(m.B, [m.K]))), a.s); } },
              { fn: 'k_r', args: ['K1', 'K2'], truth: (p, a) => { const m = AB(p, a); return WB.design.refGain(m.A, m.B, m.C, m.K); } },
            ],
          }, code);
          return r.ok ? { ...r, msg: `${r.msg} ${o.explain}` } : r;
        },
      },
      solution: () => [{ code: o.code }, { html: o.explain }],
    };
  }

  WB.ss = { makeSS, design, ctrbCard, whyPart };
})();
