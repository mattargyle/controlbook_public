// D.11-D.14: full state feedback, integral augmentation, observers and the
// disturbance observer on the (already linear) mass-spring-damper
//   xdot = A x + B u,  y = C x,  A = [0 1; -k/m -b/m],  B = [0; 1/m],  C = [1 0].
// The equilibrium is z_e = 0, F_e = 0, so no offsets are needed. Discretization
// follows the repo's ctrl*.py: dirty derivative Eq. 10.4, trapezoid integrator,
// observer integrated with RK4 over one Ts using the previous (saturated) input.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const lib = WB.studies.D.lib;
  const ans = WB.systems.D.answers;
  const CH = WB.studies.D.chapters;
  const D = WB.design;

  // Solution controllers (repo ctrl*.py conventions; anti-windup from D.12 on).
  const SOL = {
    ch11: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr = 2.0
        zeta = 0.7
        self.A = np.array([[0.0, 1.0], [-P.k / P.m, -P.b / P.m]])
        self.B = np.array([[0.0], [1 / P.m]])
        self.C = np.array([[1.0, 0.0]])
        wn = 2.2 / tr
        self.K = cnt.place(self.A, self.B, np.roots([1, 2 * zeta * wn, wn**2]))
        self.kr = -1.0 / (self.C @ np.linalg.inv(self.A - self.B @ self.K) @ self.B)[0, 0]
        # dirty derivative for zdot (Eq. 10.4)
        sigma = 0.05
        self.beta = (2 * sigma - P.Ts) / (2 * sigma + P.Ts)
        self.gamma = 2 / (2 * sigma + P.Ts)
        self.z_prev = P.z0
        self.z_dot = P.zdot0

    def update(self, z_r, y):
        z = y[0, 0]
        self.z_dot = self.beta * self.z_dot + self.gamma * (z - self.z_prev)
        self.z_prev = z
        x = np.array([[z], [self.z_dot]])
        F = -(self.K @ x)[0, 0] + self.kr * z_r
        return max(-P.Fmax, min(P.Fmax, F))
`,
    ch12: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr = 2.0
        zeta = 0.7
        p_I = -1.0
        self.A = np.array([[0.0, 1.0], [-P.k / P.m, -P.b / P.m]])
        self.B = np.array([[0.0], [1 / P.m]])
        self.C = np.array([[1.0, 0.0]])
        A1 = np.block([[self.A, np.zeros((2, 1))], [-self.C, np.zeros((1, 1))]])
        B1 = np.vstack([self.B, [[0.0]]])
        wn = 2.2 / tr
        K1 = cnt.place(A1, B1, np.roots(np.convolve([1, 2 * zeta * wn, wn**2], [1, -p_I])))
        self.K = K1[:, 0:2]
        self.ki = K1[0, 2]
        self.integrator = 0.0
        self.error_prev = 0.0
        sigma = 0.05
        self.beta = (2 * sigma - P.Ts) / (2 * sigma + P.Ts)
        self.gamma = 2 / (2 * sigma + P.Ts)
        self.z_prev = P.z0
        self.z_dot = P.zdot0

    def update(self, z_r, y):
        z = y[0, 0]
        self.z_dot = self.beta * self.z_dot + self.gamma * (z - self.z_prev)
        self.z_prev = z
        x = np.array([[z], [self.z_dot]])
        error = z_r - x[0, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        F = -(self.K @ x)[0, 0] - self.ki * integ
        # anti-windup: keep the integrator still while F saturates
        if abs(F) <= P.Fmax:
            self.integrator = integ
        self.error_prev = error
        F = -(self.K @ x)[0, 0] - self.ki * self.integrator
        return max(-P.Fmax, min(P.Fmax, F))
`,
    ch13: String.raw`import control as cnt

class Controller:
    def __init__(self):
        tr = 2.0
        zeta = 0.7
        p_I = -1.0
        self.A = np.array([[0.0, 1.0], [-P.k / P.m, -P.b / P.m]])
        self.B = np.array([[0.0], [1 / P.m]])
        self.C = np.array([[1.0, 0.0]])
        A1 = np.block([[self.A, np.zeros((2, 1))], [-self.C, np.zeros((1, 1))]])
        B1 = np.vstack([self.B, [[0.0]]])
        wn = 2.2 / tr
        K1 = cnt.place(A1, B1, np.roots(np.convolve([1, 2 * zeta * wn, wn**2], [1, -p_I])))
        self.K = K1[:, 0:2]
        self.ki = K1[0, 2]
        self.integrator = 0.0
        self.error_prev = 0.0
        wn_obs = 10 * wn          # observer 10x faster than the controller
        zeta_obs = 0.707
        self.L = cnt.place(self.A.T, self.C.T, np.roots([1, 2 * zeta_obs * wn_obs, wn_obs**2])).T
        self.x_hat = np.zeros((2, 1))
        self.F_prev = 0.0

    def update(self, z_r, y):
        x = self.update_observer(y)
        error = z_r - x[0, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        F = -(self.K @ x)[0, 0] - self.ki * integ
        # anti-windup: keep the integrator still while F saturates
        if abs(F) <= P.Fmax:
            self.integrator = integ
        self.error_prev = error
        F = -(self.K @ x)[0, 0] - self.ki * self.integrator
        F = max(-P.Fmax, min(P.Fmax, F))
        self.F_prev = F
        return F, x

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
        tr = 2.0
        zeta = 0.7
        p_I = -1.0
        self.A = np.array([[0.0, 1.0], [-P.k / P.m, -P.b / P.m]])
        self.B = np.array([[0.0], [1 / P.m]])
        self.C = np.array([[1.0, 0.0]])
        A1 = np.block([[self.A, np.zeros((2, 1))], [-self.C, np.zeros((1, 1))]])
        B1 = np.vstack([self.B, [[0.0]]])
        wn = 2.2 / tr
        K1 = cnt.place(A1, B1, np.roots(np.convolve([1, 2 * zeta * wn, wn**2], [1, -p_I])))
        self.K = K1[:, 0:2]
        self.ki = K1[0, 2]
        self.integrator = 0.0
        self.error_prev = 0.0
        # observer for x and the input disturbance d (d constant)
        self.A2 = np.block([[self.A, self.B], [np.zeros((1, 3))]])
        self.B2 = np.vstack([self.B, [[0.0]]])
        self.C2 = np.hstack([self.C, [[0.0]]])
        wn_obs = 10 * wn
        zeta_obs = 0.707
        p_d = -5.0
        poles = np.roots(np.convolve([1, 2 * zeta_obs * wn_obs, wn_obs**2], [1, -p_d]))
        self.L2 = cnt.place(self.A2.T, self.C2.T, poles).T
        self.obs = np.zeros((3, 1))
        self.F_prev = 0.0

    def update(self, z_r, y):
        self.update_observer(y)
        x = self.obs[0:2]
        d_hat = self.obs[2, 0]
        error = z_r - x[0, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        F = -(self.K @ x)[0, 0] - self.ki * integ - d_hat
        # anti-windup: keep the integrator still while F saturates
        if abs(F) <= P.Fmax:
            self.integrator = integ
        self.error_prev = error
        F = -(self.K @ x)[0, 0] - self.ki * self.integrator - d_hat
        F = max(-P.Fmax, min(P.Fmax, F))
        self.F_prev = F
        return F, x, d_hat

    def update_observer(self, y):
        F1 = self.observer_f(self.obs, y)
        F2 = self.observer_f(self.obs + P.Ts / 2 * F1, y)
        F3 = self.observer_f(self.obs + P.Ts / 2 * F2, y)
        F4 = self.observer_f(self.obs + P.Ts * F3, y)
        self.obs = self.obs + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def observer_f(self, z, y):
        return self.A2 @ z + self.B2 * self.F_prev + self.L2 @ (y - self.C2 @ z)
`,
  };

  // --------------------------------------------------------------- design --
  const ctrlPoles = (st) => D.polesFromWnZeta(2.2 / st.tr, st.zeta);
  const obsPoles = (st) => D.polesFromWnZeta(st.wnObs, st.zetaObs);

  // Explore-mode gains from the knobs, by numerical pole placement (WB.design.place,
  // Ackermann). answers.* holds the closed forms; regress_D.py checks both agree.
  function design(ctx, st = ctx.st, level = ctx.level) {
    const { A, B, C } = ctx.ss;
    const out = { poles: ctrlPoles(st) };
    if (level === 'sf') {
      out.K = D.place(A, B, out.poles) || [NaN, NaN];
      out.kr = D.refGain(A, B, C, out.K);
      return out;
    }
    const { A1, B1 } = D.augmentIntegrator(A, B, C);
    out.poles = [...out.poles, { re: st.pI, im: 0 }];
    const K1 = D.place(A1, B1, out.poles) || [NaN, NaN, NaN];
    out.K = [K1[0], K1[1]]; out.ki = K1[2];
    if (level === 'obs') {
      out.obsPoles = obsPoles(st);
      const Lg = D.observerGain(A, C, out.obsPoles);
      out.L = Lg ? [Lg[0][0], Lg[1][0]] : [NaN, NaN];
    }
    if (level === 'dobs') {
      const { A2, C2 } = D.augmentDisturbance(A, B, C);
      out.obsPoles = [...obsPoles(st), { re: st.pD, im: 0 }];
      const Lg = D.observerGain(A2, C2, out.obsPoles);
      out.L = Lg ? [Lg[0][0], Lg[1][0]] : [NaN, NaN];
      out.Ld = Lg ? Lg[2][0] : NaN;
    }
    return out;
  }
  function gainsFor(ctx) {
    if (ctx.S.mode === 'explore') return design(ctx);
    const w = ctx.st.w;
    return { K: [w.K1, w.K2], kr: w.kr, ki: w.ki, L: [w.L1, w.L2], Ld: w.Ld };
  }

  // ----------------------------------------------------------- controller --
  // level: 'sf' (D.11), 'sfi' (D.12), 'obs' (D.13), 'dobs' (D.14)
  function makeSS(ctx, { linear = false } = {}) {
    const st = ctx.st, level = ctx.level, g = ctx.gains, p = ctx.pModel;
    const { A, B, C } = ctx.ss;
    const Ts = ctx.S.sim.Ts, uLim = p.Fmax;
    const useObs = level === 'obs' || level === 'dobs';
    const useDO = level === 'dobs' && st.dobs;
    const sigma = st.sigma ?? 0.05;
    const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    let I = 0, ePrev = 0, yPrev = null, ydot = 0, uPrev = 0;
    let xh = [st.xhat0 || 0, 0], dh = 0;

    // xhat' = A xhat + B (u + dhat) + L (y - C xhat),  dhat' = Ld (y - C xhat)
    function fObs(z, y) {
      const innov = y - (C[0][0] * z[0] + C[0][1] * z[1]);
      const u = uPrev + (useDO ? z[2] : 0);
      return [
        A[0][0] * z[0] + A[0][1] * z[1] + B[0][0] * u + g.L[0] * innov,
        A[1][0] * z[0] + A[1][1] * z[1] + B[1][0] * u + g.L[1] * innov,
        useDO ? g.Ld * innov : 0,
      ];
    }

    return {
      update(r, x, yMeas) {
        let xu;
        if (useObs) {
          const z = M.rk4Step((zz) => fObs(zz, yMeas), [xh[0], xh[1], dh], 0, Ts);
          xh = [z[0], z[1]]; dh = z[2];
          xu = xh;
        } else if (st.est === 'dirty') {
          if (yPrev === null) yPrev = yMeas;
          ydot = beta * ydot + gamma * (yMeas - yPrev);
          yPrev = yMeas;
          xu = [yMeas, ydot];
        } else {
          xu = x;
        }
        const Kx = g.K[0] * xu[0] + g.K[1] * xu[1];
        let u;
        if (level === 'sf') {
          u = -Kx + g.kr * r;
        } else {
          const e = r - xu[0];
          const Itry = I + (Ts / 2) * (e + ePrev);
          const uTry = -Kx - g.ki * Itry - (useDO ? dh : 0);
          // anti-windup (D.12a): hold the integrator while the actuator is saturated
          if (!(st.antiwindup === 'clamp' && !linear && Math.abs(uTry) > uLim)) I = Itry;
          ePrev = e;
          u = -Kx - g.ki * I - (useDO ? dh : 0);
        }
        uPrev = linear ? u : M.saturate(u, uLim);
        return { u, xhat0: xu[0], xhat1: xu[1], dhat: dh, integrator: I };
      },
    };
  }

  // ------------------------------------------------------------- controls --
  function tuningSliders(parent, ctx, { pI } = {}) {
    slider(parent, { label: 't<sub>r</sub>', unit: 's', min: 0.2, max: 6, step: 0.005, sig: 4, ...bind(ctx, 'tr') });
    slider(parent, { label: 'ζ', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zeta') });
    if (pI) slider(parent, { label: 'p<sub>I</sub>', min: -10, max: -0.05, step: 0.01, sig: 3, ...bind(ctx, 'pI') });
  }
  function obsSliders(parent, ctx, withD) {
    slider(parent, { label: 'ω<sub>n,obs</sub>', unit: 'rad/s', min: 0.5, max: 40, step: 0.05, sig: 4, ...bind(ctx, 'wnObs') });
    slider(parent, { label: 'ζ<sub>obs</sub>', min: 0.2, max: 1.5, step: 0.005, ...bind(ctx, 'zetaObs') });
    if (withD) slider(parent, { label: 'p<sub>d</sub>', min: -30, max: -0.1, step: 0.05, sig: 3, ...bind(ctx, 'pD') });
  }
  function gainsReadout(parent, ctx, keys) {
    lib.readout(parent, ctx, keys, () => {
      const g = ctx.gains;
      return { K1: g.K[0], K2: g.K[1], kr: g.kr, ki: g.ki, L1: g.L && g.L[0], L2: g.L && g.L[1], Ld: g.Ld };
    });
  }
  function estControl(parent, ctx) {
    segmented(parent, {
      label: 'Where ż comes from',
      options: [{ value: 'dirty', label: 'dirty derivative of z (D.11e)' }, { value: 'true', label: 'true state' }],
      ...bind(ctx, 'est'),
    });
  }
  function awControl(parent, ctx) {
    segmented(parent, {
      label: 'Anti-windup (D.12a)',
      options: [{ value: 'clamp', label: 'hold integrator while saturated' }, { value: 'none', label: 'none' }],
      ...bind(ctx, 'antiwindup'),
    });
  }

  // ------------------------------------------------------------- analysis --
  function closedLoopPoles(ctx) {
    const { A, B, C } = ctx.ss, g = ctx.gains;
    if (ctx.level === 'sf') return L.eig(L.sub(A, L.mul(B, [g.K])));
    const { A1, B1 } = D.augmentIntegrator(A, B, C);
    return L.eig(L.sub(A1, L.mul(B1, [[g.K[0], g.K[1], g.ki]])));
  }
  function observerPoles(ctx) {
    const { A, B, C } = ctx.ss, g = ctx.gains;
    if (ctx.level === 'obs') return L.eig(L.sub(A, L.mul([[g.L[0]], [g.L[1]]], C)));
    if (ctx.level === 'dobs') {
      const { A2, C2 } = D.augmentDisturbance(A, B, C);
      return L.eig(L.sub(A2, L.mul([[g.L[0]], [g.L[1]], [g.Ld]], C2)));
    }
    return [];
  }
  // Work mode's target poles: the problem's design, shown once the part that
  // computes it is solved (D.11: the pair; D.12: plus p_I, fixed for (a)).
  const TARGETS = { sf: 'D.11/a', sfi: 'D.12/a' };
  function markers(ctx) {
    const explore = ctx.S.mode === 'explore';
    // The open-loop poles answer D.7(a): hidden in Work mode until it is solved.
    const mk = lib.showOl(ctx) ? L.eig(ctx.ss.A).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole ${i + 1}` })) : [];
    // Work mode has no gains: the plots run the student's own controller.
    if (explore) {
      closedLoopPoles(ctx).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `controller pole ${i + 1}`, dragId: Math.abs(p.im) > 1e-9 ? 0 : 2 }));
      observerPoles(ctx).forEach((p, i) => mk.push({ ...p, kind: 'obs', label: `observer pole ${i + 1}`, dragId: Math.abs(p.im) > 1e-9 ? 10 : 12 }));
    } else if (TARGETS[ctx.level] && ctx.app.isSolved(TARGETS[ctx.level])) {
      const pr = ctx.sys.problems[ctx.level === 'sf' ? 'ch11' : 'ch12'];
      const poles = D.polesFromWnZeta(2.2 / pr.tr, pr.zeta);
      if (ctx.level === 'sfi') poles.push({ re: pr.pIRef, im: 0 });
      for (const q of poles) mk.push({ ...q, kind: 'target', label: 'target pole (problem)' });
    }
    return mk;
  }
  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    if (id === 0) {
      const wn = Math.hypot(re, im);
      st.zeta = Math.max(0.2, Math.min(1, -re / wn)); st.tr = 2.2 / wn;
    } else if (id === 2) {
      if (ctx.level === 'sf') { const wn = Math.abs(re); st.zeta = 1; st.tr = 2.2 / wn; } else st.pI = re;
    } else if (id === 10) {
      st.wnObs = Math.hypot(re, im); st.zetaObs = Math.max(0.2, Math.min(1, -re / st.wnObs));
    } else if (id === 12 && ctx.level === 'dobs') st.pD = re;
    ctx.update();
  }

  // ----------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B } = ctx.ss;
    return {
      title: 'State-space model (D.6)', page: 'p. 379 · D.6', answers: 'D.6/a',
      theory: 'A = \\begin{bmatrix}0 & 1\\\\ -\\frac km & -\\frac bm\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\ \\frac1m\\end{bmatrix},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix}',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`,
    };
  }
  // The controllability matrix answers D.11(c) (and for (A₁, B₁) also shows A, B).
  const ctrbCard = (A, B, title, page, answers) => ({ ...WB.ss.ctrbCard(A, B, title, page), spoiler: false, answers });
  function polesCard(ctx, d) {
    return {
      title: 'Desired closed-loop poles', page: 'p. 113 · Eq. 8.5, p. 184',
      theory: '\\omega_n = \\frac{2.2}{t_r},\\quad \\Delta^d_{cl} = (s^2 + 2\\zeta\\omega_n s + \\omega_n^2)' + (ctx.level === 'sf' ? '' : '(s - p_I)'),
      numbers: `\\omega_n = ${tex(2.2 / ctx.st.tr)},\\quad \\Delta^d_{cl} = ${WB.tf.polyTex(L.polyFromRoots(d.poles))},\\quad p = ${d.poles.map((p) => texPole(p)).join(',\\;')}`,
      answers: ctx.level === 'sf' ? 'D.11/a' : ['D.11/a', 'D.12/a'],
    };
  }
  // --------------------------------------------------------- chapter base --
  function base(level, id, num, title, pages, extra) {
    return Object.assign({
      id, num, tab: `Ch ${num}`, title, pages, level,
      controller(ctx, o) { return makeSS(ctx, o); },
      linearSim(ctx, c) { return lib.linearSim(ctx, c, makeSS); },
      // Work mode simulates the student's controller (WB.myCtrl), from y only.
      implement: { feed: 'y', linear: false },
      gains(ctx) { ctx.level = level; return gainsFor(ctx); },
      splane(ctx) { return { markers: markers(ctx), zetaRay: ctx.S.mode === 'explore' && ctx.st.zeta < 1 ? ctx.st.zeta : null }; },
      onPoleDrag: onDrag,
      targets(ctx) { return level === 'sf' ? { tr: ctx.st.tr, zeta: ctx.st.zeta } : { tr: ctx.st.tr }; },
    }, extra);
  }
  // Gains for Work mode's ctx.gains (no control sets them; the plots run the student's controller).
  const W0 = { K1: 1, K2: 1, kr: 1, ki: -0.5, L1: 5, L2: 10, Ld: 2 };

  // Work-mode control panel: the plots show the student's controller.
  function workBanner(parent, ctx, part) {
    WB.myCtrl.banner(section(parent, 'Your controller'), ctx, part);
  }

  // The workbench's controller at `level` with the problem's tuning `tune`, on a
  // check scenario (reference for WB.myCtrl.matchCheck).
  function refRun(ctx, sc, level, tune) {
    const rc = WB.myCtrl.refCtx(ctx, sc, { level });
    rc.st = { ...ctx.st, est: 'dirty', sigma: 0.05, antiwindup: 'clamp', xhat0: 0, dobs: true, ...tune };
    rc.gains = design(rc, rc.st, level);
    return WB.myCtrl.reference(ctx, sc, makeSS(rc));
  }

  // D.12(a) windup test: the mass starts 1 m from z_r = 0 with F_max lowered, so F
  // saturates for about 3 s. Without anti-windup z undershoots by 0.55 m; holding the
  // integrator (0.33 m) or back-calculation (0.08 m) pass. Unsaturated: 0.31 m.
  const WINDUP = { Fmax: 1.5, z0: 1, us: 0.44 };
  const SQUARE = (amplitude, frequency = 0.04) => ({ type: 'square', amplitude, frequency, tStep: 0 });
  const errAt = (res, t) => { const k = Math.round(t / (res.t[1] - res.t[0])); return Math.abs(res.r[k] - res.y[k]); };
  const mm = (v) => `${fmt(1000 * v, 3)} mm`;
  const mis = (m) => Object.entries(m).map(([k, v]) => `${k} ${v > 0 ? '+' : ''}${v}%`).join(', ');
  // The student's estimate must come back from update as (F, x_hat).
  function needXhat(res) {
    const a = res.extras.xhat0, b = res.extras.xhat1;
    if (!a || !b || Array.prototype.some.call(a, (v) => !Number.isFinite(v))) return { ok: false, msg: 'Return (F, x_hat) from update, with x_hat = [[z_hat], [zdot_hat]], so the check can see your estimate.' };
    return null;
  }
  // Largest |z − ẑ| (m) and |ż − ż̂| (m/s) over the given time windows.
  function estErr(res, windows) {
    let e0 = 0, e1 = 0;
    res.t.forEach((t, k) => {
      if (!windows.some(([a, b]) => t >= a - 1e-9 && t <= b + 1e-9)) return;
      e0 = Math.max(e0, Math.abs(res.x[k][0] - res.extras.xhat0[k]));
      e1 = Math.max(e1, Math.abs(res.x[k][1] - res.extras.xhat1[k]));
    });
    return { z: e0, zd: e1 };
  }
  // D.11(e): the D.10 PID with F_e = k z_r is this controller plus an integrator (D.11(d)'s
  // "why"), and its integrator moves z by 10 mm here; a backward difference for ż moves it
  // 13 mm, σ = 0.02 instead of 0.05 9 mm. The same algorithm matches to 10⁻¹³ m.
  const C11 = { tol: 0.002 };
  // D.13(c): estimate accuracy in settled windows (the estimate lags briefly after each
  // switch, as in the repo's ctrlObserver.py timing). The solution gives ~10⁻⁶ m.
  const C13 = { z0: 0.2, win: 3.5, ez: 1e-4, ezd: 1e-3 };
  // D.14(b): the estimator bias under mismatch, d and noise. The D.13 controller (no d̂)
  // gives 0.8 mm, the solution 0.03 mm; its mean error before the switch is 0.12 mm.
  const C14 = { bias: 2e-4, err: 2.5e-3 };
  const bias = (res, t0, t1) => WB.myCtrl.mean(res, t0, t1, (k) => res.x[k][0] - res.extras.xhat0[k]);
  const estOut = (ctx, res, sc) => [{ label: 'ẑ (estimate)', y: sc(res.extras.xhat0 || []), color: '--series-3', dash: [3, 3], width: 2 }];

  // ---------------------------------------------------------------- D.11 --
  CH.ch11 = base('sf', 'ch11', 11, 'Full state feedback', 'pp. 173–196, pp. 380–381', {
    defaults(sys) { const p = sys.problems.ch11; return { est: 'dirty', sigma: 0.05, tr: p.tr, zeta: p.zeta, w: { ...W0 } }; },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, 'D.11(e)'); return; }
      const sec = section(parent, 'State feedback u = −Kx + k_r r', 'p. 173 · Eq. 11.3, p. 182 · Eq. 11.35');
      estControl(sec, ctx);
      tuningSliders(sec, ctx); gainsReadout(sec, ctx, ['K1', 'K2', 'kr']);
    },
    extraPlot(ctx, res) {
      // Work mode: only if the student's controller returns (F, x_hat).
      if (ctx.S.mode === 'work' && !res.extras.xhat1) return null;
      return { opts: { title: 'ż(t) and the estimate used', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [
        { label: 'ż used by the controller', y: Array.from(res.extras.xhat1 || []), color: '--series-3', dash: [3, 3], width: 2 },
        { label: 'true ż', y: res.x.map((x) => x[1]), color: '--series-1' },
      ] } };
    },
    math(ctx) {
      const d = design(ctx), { A, B } = ctx.ss, g = ctx.gains;
      return [
        ssCard(ctx), ctrbCard(A, B, 'Controllability', 'p. 180 · Eq. 11.29', 'D.11/c'), polesCard(ctx, d),
        { title: 'Pole placement', page: 'p. 182 · Eq. 11.32',
          theory: 'K = (\\alpha - a_A)\\,\\mathcal{A}_A^{-1}\\,\\mathcal{C}_{A,B}^{-1}',
          numbers: `K = ${texMat([d.K])}`, answers: 'D.11/d' },
        { title: 'Reference gain', page: 'p. 182 · Eq. 11.35',
          theory: 'k_r = \\frac{-1}{C(A - BK)^{-1}B}',
          numbers: `k_r = ${tex(d.kr)}`, answers: 'D.11/d' },
        { title: 'Reference gain of the mass-spring-damper', page: 'p. 182 · Eq. 11.35', answers: 'D.11/d',
          theory: 'k_r = m\\omega_n^2 = K_1 + k',
          note: 'Unlike the arm, k_r ≠ K₁: the spring needs an extra k z_r to hold the mass at z_r.' },
        { title: 'Control law', page: 'p. 173 · Eq. 11.3', answers: 'D.11/e',
          theory: 'F = -Kx + k_r z_r',
          numbers: ctx.S.mode === 'work' ? null : `F = -(${tex(g.K[0])}\\,z + ${tex(g.K[1])}\\,\\dot z) + ${tex(g.kr)}\\,z_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch11;
      const poles = () => D.polesFromWnZeta(2.2 / prob.tr, prob.zeta);
      const ref = () => ans.stateFeedback(ctx.pModel, poles());
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Closed-loop poles from s² + 2ζω<sub>n</sub>s + ω<sub>n</sub>² = 0 (ω<sub>n</sub>, ζ from D.8)',
          html: 'Enter the poles as −σ ± jω<sub>d</sub>.',
          inputs: { sig: 'σ', wd: 'ω<sub>d</sub>' },
          check: (v) => lib.check(v, { sig: -poles()[0].re, wd: Math.abs(poles()[0].im) }, { sig: 'σ', wd: 'ωd' }),
          solution: () => [{ tex: `\\omega_n = 1.1,\\; s^2 + 1.54s + 1.21 = 0 \\Rightarrow p = ${texPole(poles()[0], 4)},\\; ${texPole(poles()[1], 4)}` }],
        },
        {
          id: 'b', title: '(b) Add A, B, C, D from D.6 to your param file',
          html: 'Use your A, B, C, D from D.6 (the D.6 tab). The model card in the Math section unlocks once D.6 is solved.',
        },
        {
          id: 'c', title: '(c) Controllability: rank(𝒞<sub>A,B</sub>) = n',
          inputs: { c11: 'c<sub>11</sub>', c12: 'c<sub>12</sub>', c21: 'c<sub>21</sub>', c22: 'c<sub>22</sub>', rank: 'rank' },
          check: (v) => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return lib.check(v, { c11: Cm[0][0], c12: Cm[0][1], c21: Cm[1][0], c22: Cm[1][1], rank: L.rank(Cm) }, {}); },
          solution: () => { const Cm = L.ctrb(ctx.ss.A, ctx.ss.B); return [{ tex: `\\mathcal{C}_{A,B} = \\begin{bmatrix}0 & \\frac1m\\\\ \\frac1m & -\\frac{b}{m^2}\\end{bmatrix} = ${texMat(Cm)},\\; \\det = -\\frac{1}{m^2} = ${tex(Cm[0][0] * Cm[1][1] - Cm[0][1] * Cm[1][0])} \\ne 0` }]; },
        },
        {
          id: 'd', title: `(d) K and k<sub>r</sub> for t<sub>r</sub> = ${prob.tr}, ζ = ${prob.zeta}`,
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => { const r = ref(); return lib.check(v, { K1: r.K[0], K2: r.K[1], kr: r.kr }, {}); },
          solution: () => {
            const r = ref();
            return [
              { tex: '\\det(sI - A + BK) = s^2 + \\frac{b + K_2}{m}s + \\frac{k + K_1}{m} \\;\\Rightarrow\\; K_1 = m\\omega_n^2 - k,\\; K_2 = 2m\\zeta\\omega_n - b' },
              { tex: `K = ${texMat([r.K])},\\quad k_r = \\frac{-1}{C(A-BK)^{-1}B} = m\\omega_n^2 = k + K_1 = ${tex(r.kr)}` },
            ];
          },
        },
        WB.ss.whyPart(ctx, {
          id: 'd2', title: '(d) Why is K = (k<sub>P</sub>, k<sub>D</sub>) when the poles match D.8?',
          html: 'Answer with two functions of the gains: the closed-loop characteristic polynomial under F = −Kx + k<sub>r</sub>z<sub>r</sub>, and the k<sub>r</sub> that makes the DC gain one. Then compare them with your D.7(b) PD loop. Any nonzero multiple of the polynomial is accepted.',
          ranges: { K1: [0.5, 10], K2: [0.5, 10] },
          explain: 'Same polynomial as the PD loop of D.7(b) with k<sub>P</sub> = K<sub>1</sub>, k<sub>D</sub> = K<sub>2</sub>: −Kx = −K<sub>1</sub>z − K<sub>2</sub>ż is PD with the derivative on the output (Fig. 7-2). The difference is k<sub>r</sub> = k + K<sub>1</sub>, i.e. k<sub>P</sub> plus the spring feedforward F<sub>e</sub> = kz<sub>r</sub>, which is what makes the DC gain one.',
          code: 'def char_poly(s, K1, K2):\n    return P.m * s**2 + (P.b + K2) * s + P.k + K1\n\ndef k_r(K1, K2):\n    return P.k + K1     # -1 / (C (A - B K)^-1 B)',
        }),
        WB.myCtrl.part(ctx, {
          id: 'e', title: '(e) Implement the state-feedback controller, using a digital differentiator to estimate ż',
          seed: 'D.10/c',
          html: `Your controller gets only the measured z. The check runs a ±0.5 m square wave (0.04 Hz) with the nominal and with other parameters and compares z(t) with the design for t<sub>r</sub> = ${prob.tr} s, ζ = ${prob.zeta}, with ż from the D.10 digital differentiator (σ = 0.05), to within ${mm(C11.tol)}.`,
          check: (code) => {
            const tune = { tr: prob.tr, zeta: prob.zeta };
            const cases = WB.myCtrl.paramCases(ctx).map((pc) => {
              const sc = WB.myCtrl.scenario(ctx, { params: pc.params, ref: SQUARE(0.5), tEnd: 20 });
              return { sc, label: pc.label, ref: () => refRun(ctx, sc, 'sf', tune) };
            });
            return WB.myCtrl.matchCheck(ctx, code, cases, { tol: C11.tol });
          },
          solution: () => [{ code: SOL.ch11 }, { html: 'x = (z, ż̂) with ż̂ from the dirty derivative (Eq. 10.4, σ = 0.05), then F = −Kx + k<sub>r</sub>z<sub>r</sub>, saturated at F<sub>max</sub>. With exact parameters the steady-state error is zero; with mismatch it is not (no integrator yet).' }],
        }),
      ]);
    },
  });

  // ---------------------------------------------------------------- D.12 --
  CH.ch12 = base('sfi', 'ch12', 12, 'Integrator with state feedback', 'pp. 197–214, p. 381', {
    defaults(sys) { const p = sys.problems.ch12; return { est: 'dirty', sigma: 0.05, tr: p.tr, zeta: p.zeta, pI: p.pIRef, antiwindup: 'clamp', w: { ...W0 } }; },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, 'D.12(a) and (c)'); return; }
      const sec = section(parent, 'u = −Kx − k_I ∫(r − z)', 'p. 199');
      estControl(sec, ctx);
      awControl(sec, ctx);
      tuningSliders(sec, ctx, { pI: true }); gainsReadout(sec, ctx, ['K1', 'K2', 'ki']);
    },
    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [m·s]', unit: 'm·s' }, data: { series: [{ label: 'x_I = ∫(r − z)', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = design(ctx), { A, B, C } = ctx.ss;
      const { A1, B1 } = D.augmentIntegrator(A, B, C);
      return [
        ssCard(ctx),
        { title: 'Augmented system', page: 'p. 198 · Eq. 12.1',
          theory: '\\dot x_I = r - Cx,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\ 0\\end{bmatrix}',
          numbers: `A_1 = ${texMat(A1)},\\quad B_1 = ${texMat(B1)}`, answers: 'D.6/a' },
        ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 198', ['D.6/a', 'D.11/c']),
        polesCard(ctx, d),
        { title: 'Gains', page: 'p. 199–201',
          theory: '\\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad u = -Kx - k_I\\int_0^t (r - y)\\,d\\tau',
          numbers: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}`, answers: 'D.12/a' },
        { title: 'Gains for the mass-spring-damper', page: 'p. 199–201', answers: 'D.12/a',
          theory: '\\det(sI - A_1 + B_1K_1) = s^3 + c_2s^2 + c_1s + c_0,\\quad c_2 = \\frac{b + K_2}{m},\\; c_1 = \\frac{k + K_1}{m},\\; c_0 = -\\frac{k_I}{m}' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch12;
      const ref = (pI) => ans.integralFeedback(ctx.pModel, [...D.polesFromWnZeta(2.2 / prob.tr, prob.zeta), { re: pI, im: 0 }]);
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) Gains for t<sub>r</sub> = ${prob.tr} s, ζ = ${prob.zeta}, p<sub>I</sub> = ${prob.pIRef}`,
          html: `The book leaves the integrator pole to you. Here (a) uses p<sub>I</sub> = ${prob.pIRef} (a workbench choice, so the controller can be compared with a design), and (c) tunes it.`,
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', ki: 'k<sub>I</sub>' },
          check: (v) => { const r = ref(prob.pIRef); return lib.check(v, { K1: r.K[0], K2: r.K[1], ki: r.ki }, { ki: 'kI' }); },
          solution: () => {
            const r = ref(prob.pIRef);
            return [
              { tex: 'K_1 = m c_1 - k,\\quad K_2 = m c_2 - b,\\quad k_I = -m c_0 \\quad\\text{for } \\Delta^d = s^3 + c_2s^2 + c_1s + c_0' },
              { tex: `p_I = ${prob.pIRef}:\; \\Delta^d = ${WB.tf.polyTex(L.polyFromRoots([...D.polesFromWnZeta(1.1, 0.7), { re: prob.pIRef, im: 0 }]))},\; K = ${texMat([r.K])},\; k_I = ${tex(r.ki)}` },
            ];
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'a2', title: '(a) Add the integrator with anti-windup to your D.11 controller', seed: 'D.11/e',
          html: `Use the gains from above. The check (1) runs a ±0.5 m square wave (0.04 Hz) with the nominal and with other parameters and compares z(t) with the design (within 3% of the step), then (2) starts the mass at z = ${WINDUP.z0} m with z<sub>r</sub> = 0 and lowers F<sub>max</sub> to ${WINDUP.Fmax} N, so F saturates for about 3 s: z may undershoot past 0 by at most ${WINDUP.us} m (the design without saturation undershoots by 0.31 m).`,
          check: async (code) => {
            const tune = { tr: prob.tr, zeta: prob.zeta, pI: prob.pIRef };
            const cases = WB.myCtrl.paramCases(ctx).map((pc) => {
              const sc = WB.myCtrl.scenario(ctx, { params: pc.params, ref: SQUARE(0.5), tEnd: 20 });
              return { sc, label: pc.label, ref: () => refRun(ctx, sc, 'sfi', tune) };
            });
            const m = await WB.myCtrl.matchCheck(ctx, code, cases, { tol: 0.03 * 0.5 });
            if (!m.ok) return m;
            const sc = WB.myCtrl.scenario(ctx, { params: { ...ctx.pModel, Fmax: WINDUP.Fmax }, init: { z0: WINDUP.z0 }, ref: { type: 'step', amplitude: 0, tStep: 0 }, tEnd: 20 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const us = -Math.min(...res.y);
            if (!(us <= WINDUP.us)) return { ok: false, msg: `Tracking matches, but with F_max = ${WINDUP.Fmax} N, z undershoots past 0 by ${fmt(us, 3)} m: the integrator winds up while F is saturated.` };
            return { ok: true, msg: `${m.msg} Saturated run: undershoot ${fmt(Math.max(0, us), 3)} m.` };
          },
          solution: () => [{ code: SOL.ch12 }, { html: 'Integral state feedback with the D.11 dirty derivative. Anti-windup here holds the integrator while F is saturated; unwinding it by (F<sub>sat</sub> − F<sub>unsat</sub>)/k<sub>I</sub> passes too. Integrating only while |ż| is small (Listing 10.2) does not fit this controller: the reference enters only through the integrator, so freezing it during every motion changes the tracking.' }],
        }),
        {
          id: 'b', title: '(b) Input disturbance of 0.25 N and parameters varying up to 20%',
          html: 'Set the input disturbance d and the true-plant mismatch in the left panel (the chapter starts with d = 0.25 N and a fixed 20% draw).',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return Math.abs(S.sim.dist) > 0 && mis ? { ok: true, msg: `d = ${fmt(S.sim.dist, 3)} N with plant mismatch.` } : { ok: false, msg: 'Set both d ≠ 0 and a plant mismatch.' };
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Tune the integrator pole (and other gains if needed) for good tracking', seed: ['D.12/a2'],
          html: `The check runs the ±${prob.sim.amplitude} m square wave (${prob.sim.frequency} Hz) with d = ${prob.sim.dist} N and the plant off by ${mis(prob.mismatch)}: |z<sub>r</sub> − z| just before the first switch (t = 25 s) must be under 1% of the step (${mm(0.01 * prob.sim.amplitude)}).`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(prob.sim.amplitude, prob.sim.frequency), tEnd: 25, dist: prob.sim.dist, mismatch: prob.mismatch });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const e = errAt(res, 24.95);
            return { ok: e < 0.01 * prob.sim.amplitude, msg: `Error before the switch: ${mm(e)}.` };
          },
          solution: () => [{ code: SOL.ch12 }, { html: `The design from (a) (p<sub>I</sub> = ${prob.pIRef}) already passes: the error before the switch is about 10⁻⁶ mm. A faster p<sub>I</sub> (e.g. −2) removes d sooner but adds overshoot; a slower one leaves a long tail.` }],
        }),
      ]);
    },
  });

  // ---------------------------------------------------------------- D.13 --
  function obsDefaults(p) {
    return { sigma: 0.05, tr: p.tr, zeta: p.zeta, pI: p.pIRef, antiwindup: 'clamp', wnObs: p.obsFactor * 2.2 / p.tr, zetaObs: p.zetaObs, xhat0: 0.1, w: { ...W0 } };
  }
  CH.ch13 = base('obs', 'ch13', 13, 'Observers', 'pp. 215–238, pp. 381–382', {
    defaults(sys) { return obsDefaults(sys.problems.ch13); },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries: estOut,
    extraPlot(ctx, res) {
      return { opts: { title: 'ż(t) and estimate', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [
        { label: 'ż̂ (observer)', y: Array.from(res.extras.xhat1 || []), color: '--series-3', dash: [3, 3], width: 2 },
        { label: 'true ż', y: res.x.map((x) => x[1]), color: '--series-1' },
      ] } };
    },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, 'D.13(c)'); return; }
      const sec = section(parent, 'Controller (uses x̂)', 'p. 222 · Fig. 13-3');
      awControl(sec, ctx);
      tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer', 'p. 216 · Eq. 13.3');
      obsSliders(ob, ctx, false); gainsReadout(ob, ctx, ['K1', 'K2', 'ki', 'L1', 'L2']);
      slider(ob, { label: 'ẑ(0)', unit: 'm', min: -1, max: 1, step: 0.01, sig: 3, hint: 'initial estimate (true z starts at the left-panel value)', ...bind(ctx, 'xhat0') });
    },
    math(ctx) {
      const d = design(ctx), { A, C } = ctx.ss;
      const O = L.obsv(A, C);
      return [
        ssCard(ctx),
        { title: 'Observer', page: 'p. 216 · Eq. 13.3', answers: 'D.13/c2',
          theory: '\\dot{\\hat x} = A\\hat x + Bu + L(y - C\\hat x),\\quad \\dot e = (A - LC)e' },
        { title: 'Observability', page: 'p. 221',
          theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ \\vdots \\\\ CA^{n-1}\\end{bmatrix},\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = n',
          numbers: `\\mathcal{O} = ${texMat(O)},\\quad \\operatorname{rank} = ${L.rank(O)}`, answers: 'D.13/b' },
        { title: 'Observer gain', page: 'p. 222 · Eq. 13.16',
          theory: 'L = \\text{place}(A^\\top, C^\\top, q)^\\top',
          numbers: `q = ${d.obsPoles.map((p) => texPole(p)).join(',\\;')},\\quad L = ${texMat(d.L)}`, answers: 'D.13/c2' },
        { title: 'Observer gain for the mass-spring-damper', page: 'p. 222', answers: 'D.13/c2',
          theory: '\\det(sI - A + LC) = s^2 + \\beta_1 s + \\beta_0,\\quad \\beta_1 = \\tfrac bm + L_1,\\; \\beta_0 = \\tfrac km + \\tfrac bm L_1 + L_2' },
        { title: 'Separation principle', page: 'p. 222–223',
          theory: '\\text{eig} = \\text{eig}(A_1 - B_1K_1) \\cup \\text{eig}(A - LC)',
          note: 'Holds for the linear model. Saturation breaks it.' },
        polesCard(ctx, d),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch13;
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Exact parameters, no input disturbance (α = 0)',
          html: 'The chapter starts with α = 0 and d = 0. <em>Exact model</em> in the left panel restores them.',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return !mis && !(Math.abs(S.sim.dist) > 0) ? { ok: true, msg: 'Exact plant, d = 0.' } : { ok: false, msg: 'Remove the plant mismatch and set d = 0.' };
          },
        },
        {
          id: 'b', title: '(b) Observability: rank(𝒪<sub>A,C</sub>) = n',
          inputs: { o11: 'o<sub>11</sub>', o12: 'o<sub>12</sub>', o21: 'o<sub>21</sub>', o22: 'o<sub>22</sub>', rank: 'rank' },
          check: (v) => { const O = L.obsv(ctx.ss.A, ctx.ss.C); return lib.check(v, { o11: O[0][0], o12: O[0][1], o21: O[1][0], o22: O[1][1], rank: L.rank(O) }, {}); },
          solution: () => [{ tex: `\\mathcal{O}_{A,C} = \\begin{bmatrix}C\\\\ CA\\end{bmatrix} = ${texMat(L.obsv(ctx.ss.A, ctx.ss.C))} \\Rightarrow \\text{rank } 2` }],
        },
        WB.myCtrl.part(ctx, {
          id: 'c2', title: '(c) Add an observer that estimates x̂, use x̂ in your D.12 controller, and tune', seed: ['D.12/c', 'D.12/a2'],
          html: `Return <code>(F, x_hat)</code> from <code>update</code>. The check starts the mass at z = ${C13.z0} m (your estimate starts wherever you start it) and runs a ±0.5 m square wave (0.04 Hz) with exact parameters: in the ${C13.win} s before each switch, |z − ẑ| must stay under ${mm(C13.ez)} and |ż − ż̂| under ${fmt(1000 * C13.ezd, 3)} mm/s, and |z<sub>r</sub> − z| just before the first switch under ${mm(0.005)}.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.5), tEnd: 25, init: { z0: C13.z0 } });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const ee = estErr(res, [[12.45 - C13.win, 12.45], [24.95 - C13.win, 24.95]]);
            const e = errAt(res, 12.45);
            const msg = `Before the switches: |z − ẑ| ≤ ${mm(ee.z)}, |ż − ż̂| ≤ ${fmt(1000 * ee.zd, 3)} mm/s; error before the first switch ${mm(e)}.`;
            return { ok: ee.z < C13.ez && ee.zd < C13.ezd && e < 0.005, msg };
          },
          solution: () => [{ code: SOL.ch13 }, { html: `As the repo's ctrlObserver.py: observer poles ${prob.obsFactor}× faster than the controller pair (ω<sub>n,obs</sub> = 11 rad/s, ζ = ${prob.zetaObs}), integrated with RK4 over one T<sub>s</sub> using the previous saturated F, and the D.12 controller (p<sub>I</sub> = ${prob.pIRef}, anti-windup) acting on x̂. The estimate lags briefly after each switch (the observer integrates with y at the end of the step), then converges.` }],
        }),
        {
          id: 'd', title: '(d) Plot the state and the estimated state together',
          html: 'Run your controller: the z plot shows your ẑ with the true z, and the extra plot your ż̂ with the true ż.',
        },
        {
          id: 'e', title: '(e) Add an input disturbance of 0.25 N',
          html: 'Runs your controller from (c) with d = 0.25 N and exact parameters. Your observer has no model of d, so x̂ is biased, and an integrator acting on ẑ leaves z off z<sub>r</sub>. Passes when the run shows the bias (D.14 removes it).',
          check: async () => {
            const code = WB.myCtrl.savedCode(ctx, 'D.13/c2');
            if (!code) return { ok: false, msg: 'Write your controller in (c) first.' };
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.5, 0.02), tEnd: 25, dist: 0.25 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const b = bias(res, 15, 24.95), e = WB.myCtrl.mean(res, 15, 24.95, (k) => res.y[k] - res.r[k]);
            const msg = `With d = 0.25 N: z − ẑ = ${mm(b)} and z − z_r = ${mm(e)} over the 10 s before the first switch.`;
            if (Math.abs(b) < 1e-6) return { ok: false, msg: `${msg} ẑ follows z exactly: is x̂ coming from an observer of the model (without a disturbance estimate)?` };
            return { ok: true, msg };
          },
          solution: () => {
            const d = design(ctx, { ...ctx.st, tr: prob.tr, zeta: prob.zeta, pI: prob.pIRef, wnObs: prob.obsFactor * 2.2 / prob.tr, zetaObs: prob.zetaObs }, 'obs');
            const { A, B, C } = ctx.ss;
            const Ai = L.inv(L.sub(A, L.mul([[d.L[0]], [d.L[1]]], C)));
            const ess = Ai ? -L.mul(L.mul(C, Ai), B)[0][0] : NaN;
            return [
              { tex: '\\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} = -(A - LC)^{-1}Bd,\\quad z - \\hat z = -C(A - LC)^{-1}B\\,d' },
              { tex: `\\text{observer } ${prob.obsFactor}\\times \\text{ faster}:\; z - \\hat z = ${tex(ess * 0.25)}\\,\\text{m for } d = 0.25\\,\\text{N}` },
              { html: 'With ẑ held at z<sub>r</sub> by the integrator, z = z<sub>r</sub> + (z − ẑ): the observer bias becomes the tracking error (p. 240). It is small with a fast observer; a slower one shows it clearly.' },
            ];
          },
        },
      ]);
    },
  });

  // ---------------------------------------------------------------- D.14 --
  CH.ch14 = base('dobs', 'ch14', 14, 'Disturbance observer', 'pp. 239–259, p. 382', {
    defaults(sys) { const p = sys.problems.ch14; return { ...obsDefaults(p), pD: p.pDRef, dobs: true, xhat0: 0 }; },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries: estOut,
    extraPlot(ctx, res) {
      const S = ctx.S;
      return { opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N]', unit: 'N' }, data: { series: [
        { label: 'd̂ (estimate)', y: Array.from(res.extras.dhat || []), color: '--series-3', width: 2 },
        { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
      ] } };
    },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, 'D.14(b)'); return; }
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'p. 241');
      segmented(sec, {
        label: 'Disturbance observer', options: [{ value: true, label: 'on' }, { value: false, label: 'off (D.14a)' }],
        ...bind(ctx, 'dobs'),
      });
      awControl(sec, ctx);
      tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer', 'p. 241');
      obsSliders(ob, ctx, true); gainsReadout(ob, ctx, ['K1', 'K2', 'ki', 'L1', 'L2', 'Ld']);
    },
    math(ctx) {
      const d = design(ctx), { A, B, C } = ctx.ss;
      const { A2, C2 } = D.augmentDisturbance(A, B, C);
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(u + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0 \\text{ for constant } d' },
        { title: 'Augmented model (ḋ = 0)', page: 'p. 240',
          theory: 'A_2 = \\begin{bmatrix}A & B\\\\ 0 & 0\\end{bmatrix},\\quad C_2 = \\begin{bmatrix}C & 0\\end{bmatrix}',
          numbers: `A_2 = ${texMat(A2)},\\quad \\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}`, answers: 'D.6/a' },
        { title: 'Disturbance observer', page: 'p. 241', answers: 'D.14/b',
          theory: '\\dot{\\hat x} = A\\hat x + B(u + \\hat d) + L(y - C\\hat x),\\quad \\dot{\\hat d} = L_d(y - C\\hat x),\\quad u = -K\\hat x - k_I\\textstyle\\int e - \\hat d' },
        { title: 'Observer gains', page: 'p. 241',
          theory: '\\begin{bmatrix}L\\\\ L_d\\end{bmatrix} = \\text{place}(A_2^\\top, C_2^\\top, q)^\\top',
          numbers: `q = ${d.obsPoles.map((p) => texPole(p)).join(',\\;')},\\quad L = ${texMat(d.L)},\\; L_d = ${tex(d.Ld)}`, answers: 'D.14/b' },
        { title: 'Observer gains for the mass-spring-damper', page: 'p. 241', answers: 'D.14/b',
          theory: '\\det(sI - A_2 + LC_2) = s^3 + c_2s^2 + c_1s + c_0,\\quad c_2 = \\tfrac bm + L_1,\\; c_1 = \\tfrac km + \\tfrac bm L_1 + L_2,\\; c_0 = \\tfrac{L_d}{m}' },
        polesCard(ctx, d),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch14;
      const noise = prob.sim.noise;  // m (display scale 1)
      lib.panel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) α = 0.2, input disturbance 0.25, noise σ = 0.001 on z<sub>m</sub>',
          html: `The chapter starts with the plant off by ${mis(prob.mismatch)}, d = ${prob.sim.dist} N and noise σ = ${noise} m. Run your D.13 controller here to see the bias it leaves.`,
          actions: [{ label: 'Run my D.13 controller', run: () => {
            const code = WB.myCtrl.savedCode(ctx, 'D.13/c2');
            if (!code) return { ok: false, msg: 'Write your controller in D.13(c) first (Ch 13 tab).' };
            return WB.myCtrl.use(ctx, code, 'a');
          } }],
        },
        WB.myCtrl.part(ctx, {
          id: 'b', title: '(b) Add a disturbance observer, verify that the estimator\'s steady-state error is removed, and tune', seed: ['D.13/c2'],
          html: `Return <code>(F, x_hat)</code> (or <code>(F, x_hat, d_hat)</code> to plot your d̂). The check runs the ±${prob.sim.amplitude} m square wave (${prob.sim.frequency} Hz) with the plant off by ${mis(prob.mismatch)}, d = ${prob.sim.dist} N and noise σ = ${noise} m: the mean of z − ẑ over the 10 s before the first switch must be under ${mm(C14.bias)}, and the mean error over the 0.5 s before it under ${mm(C14.err)}.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(prob.sim.amplitude, prob.sim.frequency), tEnd: 25, dist: prob.sim.dist, mismatch: prob.mismatch, noise, seed: 1 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const b = bias(res, 15, 24.95);
            const e = WB.myCtrl.mean(res, 24.45, 24.95, (k) => res.r[k] - res.y[k]);
            return { ok: Math.abs(b) < C14.bias && Math.abs(e) < C14.err, msg: `Mean z − ẑ over the 10 s before the switch: ${mm(b)}; mean error before the switch: ${mm(e)}.` };
          },
          solution: () => [
            { code: SOL.ch14 },
            { html: `As the repo's ctrlDisturbanceObserver.py: the observer of (x, d) with A<sub>2</sub> = [A B; 0 0], poles at the D.13 pair plus p<sub>d</sub> = ${prob.pDRef}, and F = −Kx̂ − k<sub>I</sub>∫e − d̂ with the D.12 anti-windup. Without d̂ the D.13 controller leaves a bias of about 0.8 mm here; with it, about 0.03 mm. d̂ settles near d plus whatever force the mismatched model gets wrong (here (k̂ − k)z and the mass error during transients), so it tracks the total unmodeled input, not d alone. A faster observer speeds this up but lets more of the 1 mm noise into ż̂ and F.` },
          ],
        }),
      ]);
    },
  });

  WB.studies.D.ss = { makeSS, design };
})();
