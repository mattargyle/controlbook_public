// Study C, chapters 11-14: full state feedback on the four-state satellite model,
// integral augmentation on φ, two-output observers and a disturbance observer.
// Mirrors ctrlStateFeedback.py, ctrlStateFeedbackIntegrator.py, ctrlObserver.py
// and ctrlDisturbanceObserver.py. The desired poles are two second-order pairs:
// a "θ" pair from t_rθ and a slower "φ" pair from t_rφ = M t_rθ (C.11, p. 192).
//
// Work mode: C.11(e), C.12(a, c), C.13(c) and C.14(b) are the student's own Python
// controllers (WB.myCtrl), which drive the plots; the gains and observer poles here
// stay in Explore mode.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt, fmtPole } = M;
  const PD = () => WB.pd;
  const lib = () => WB.studies.C.lib;
  const CH = WB.studies.C.chapters;
  const R2D = 180 / Math.PI;


  // ---------------------------------------------------- solution controllers --
  // The repo's tuning (t_rθ = 2 s, M = 3, ζ = 0.9, ω_n = π/(2t_r√(1−ζ²)), p_I = −2),
  // with the anti-windup the book asks for from C.12 on. C.11 and C.12 get the state
  // (as hw11/hw12); C.13 and C.14 get y = (θ, φ).
  const MODEL_PY = `        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [-P.k / P.Js, P.k / P.Js, -P.b / P.Js, P.b / P.Js],
                      [P.k / P.Jp, -P.k / P.Jp, P.b / P.Jp, -P.b / P.Jp]])
        B = np.array([[0.0], [0.0], [1.0 / P.Js], [0.0]])
`;
  const TUNE_PY = `        tr_th = 2.0          # the repo's tuning
        M = 3.0
        zeta_th = 0.9
        zeta_phi = 0.9
`;
  const SOL = {
    ch11: `import control as cnt

class Controller:
    def __init__(self):
${TUNE_PY}${MODEL_PY}        Cr = np.array([[0.0, 1.0, 0.0, 0.0]])   # phi
        wn_th = 0.5 * np.pi / (tr_th * np.sqrt(1 - zeta_th**2))
        wn_phi = 0.5 * np.pi / (M * tr_th * np.sqrt(1 - zeta_phi**2))
        poles = np.roots(np.convolve([1, 2 * zeta_th * wn_th, wn_th**2],
                                     [1, 2 * zeta_phi * wn_phi, wn_phi**2]))
        self.K = cnt.place(A, B, poles)
        self.kr = -1.0 / (Cr @ np.linalg.inv(A - B @ self.K) @ B)[0, 0]

    def update(self, phi_r, x):
        tau = -(self.K @ x)[0, 0] + self.kr * phi_r
        return max(-P.tau_max, min(P.tau_max, tau))
`,
    ch12: `import control as cnt

class Controller:
    def __init__(self):
${TUNE_PY}        p_I = -2.0
${MODEL_PY}        Cr = np.array([[0.0, 1.0, 0.0, 0.0]])   # integrate phi_r - phi
        A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        wn_th = 0.5 * np.pi / (tr_th * np.sqrt(1 - zeta_th**2))
        wn_phi = 0.5 * np.pi / (M * tr_th * np.sqrt(1 - zeta_phi**2))
        poles = np.roots(np.convolve(np.convolve([1, 2 * zeta_th * wn_th, wn_th**2],
                                                 [1, 2 * zeta_phi * wn_phi, wn_phi**2]),
                                     [1, -p_I]))
        K1 = cnt.place(A1, B1, poles)
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, phi_r, x):
        error = phi_r - x[1, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        self.error_prev = error
        tau = -(self.K @ x)[0, 0] - self.ki * integ
        # anti-windup: hold the integrator while the torque saturates
        if abs(tau) <= P.tau_max:
            self.integrator = integ
        tau = -(self.K @ x)[0, 0] - self.ki * self.integrator
        return max(-P.tau_max, min(P.tau_max, tau))
`,
    ch13: `import control as cnt

class Controller:
    def __init__(self):
${TUNE_PY}        p_I = -2.0
${MODEL_PY}        self.A, self.B = A, B
        self.C = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]])
        Cr = np.array([[0.0, 1.0, 0.0, 0.0]])
        A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        wn = lambda tr, zeta: 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        pair = lambda tr, zeta: [1, 2 * zeta * wn(tr, zeta), wn(tr, zeta)**2]
        poles = np.roots(np.convolve(np.convolve(pair(tr_th, zeta_th), pair(M * tr_th, zeta_phi)), [1, -p_I]))
        K1 = cnt.place(A1, B1, poles)
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        # observer poles 10x faster than the controller's
        obs_poles = np.roots(np.convolve(pair(tr_th / 10, zeta_th), pair(M * tr_th / 10, zeta_phi)))
        self.L = cnt.place(A.T, self.C.T, obs_poles).T
        self.x_hat = np.zeros((4, 1))
        self.tau_prev = 0.0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, phi_r, y):
        x_hat = self.update_observer(y)
        error = phi_r - x_hat[1, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        self.error_prev = error
        tau = -(self.K @ x_hat)[0, 0] - self.ki * integ
        if abs(tau) <= P.tau_max:          # anti-windup
            self.integrator = integ
        tau = -(self.K @ x_hat)[0, 0] - self.ki * self.integrator
        tau = max(-P.tau_max, min(P.tau_max, tau))
        self.tau_prev = tau
        return tau, x_hat

    def update_observer(self, y):
        # RK4 on the observer, with the previous (saturated) torque
        F1 = self.observer_f(self.x_hat, y)
        F2 = self.observer_f(self.x_hat + P.Ts / 2 * F1, y)
        F3 = self.observer_f(self.x_hat + P.Ts / 2 * F2, y)
        F4 = self.observer_f(self.x_hat + P.Ts * F3, y)
        self.x_hat = self.x_hat + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y):
        return self.A @ x_hat + self.B * self.tau_prev + self.L @ (y - self.C @ x_hat)
`,
    ch14: `import control as cnt

class Controller:
    def __init__(self):
${TUNE_PY}        p_I = -2.0
        p_d = -10.0          # disturbance-observer pole
${MODEL_PY}        C = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        Cr = np.array([[0.0, 1.0, 0.0, 0.0]])
        A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])
        B1 = np.vstack([B, [[0.0]]])
        wn = lambda tr, zeta: 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        pair = lambda tr, zeta: [1, 2 * zeta * wn(tr, zeta), wn(tr, zeta)**2]
        poles = np.roots(np.convolve(np.convolve(pair(tr_th, zeta_th), pair(M * tr_th, zeta_phi)), [1, -p_I]))
        K1 = cnt.place(A1, B1, poles)
        self.K = K1[:, 0:4]
        self.ki = K1[0, 4]
        # observer for (x, d): x' = A x + B (tau + d), d' = 0
        self.A2 = np.block([[A, B], [np.zeros((1, 5))]])
        self.B2 = np.vstack([B, [[0.0]]])
        self.C2 = np.hstack([C, np.zeros((2, 1))])
        # observer poles: the C.13 pairs (10x faster) plus p_d
        obs_poles = np.roots(np.convolve(np.convolve(pair(tr_th / 10, zeta_th), pair(M * tr_th / 10, zeta_phi)), [1, -p_d]))
        self.L2 = cnt.place(self.A2.T, self.C2.T, obs_poles).T
        self.obs = np.zeros((5, 1))
        self.tau_prev = 0.0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update(self, phi_r, y):
        self.update_observer(y)
        x_hat = self.obs[0:4]
        d_hat = self.obs[4, 0]
        error = phi_r - x_hat[1, 0]
        integ = self.integrator + P.Ts / 2 * (error + self.error_prev)
        self.error_prev = error
        tau = -(self.K @ x_hat)[0, 0] - self.ki * integ - d_hat
        if abs(tau) <= P.tau_max:          # anti-windup
            self.integrator = integ
        tau = -(self.K @ x_hat)[0, 0] - self.ki * self.integrator - d_hat
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
        return self.A2 @ z + self.B2 * self.tau_prev + self.L2 @ (y - self.C2 @ z)
`,
  };

  // --------------------------------------------------------------- design --
  const augI = (A, B) => WB.design.augmentIntegrator(A, B, [[0, 1, 0, 0]]);   // ẋ_I = φ_r − φ (C_r = [0 1 0 0])
  const augD = (A, B, C) => WB.design.augmentDisturbance(A, B, C);
  const wnPair = (st) => {
    const wnTh = lib().wnRule(st.trTh, st.zetaTh, st.rule), wnPhi = lib().wnRule(st.M * st.trTh, st.zetaPhi, st.rule);
    return { wnTh, wnPhi, th: lib().pairPoles(wnTh, st.zetaTh), ph: lib().pairPoles(wnPhi, st.zetaPhi) };
  };
  // Observer poles: 10× faster by default (tr_obs = tr/10 in ctrlObserver.py).
  function obsDefaults(pr) {
    const r = (tr, z) => lib().wnRule(tr, z, pr.rule);
    return { wTh: r(pr.trTh / pr.obsFactor, pr.zetaTh), wPh: r(pr.M * pr.trTh / pr.obsFactor, pr.zetaPhi), zTh: pr.zetaTh, zPh: pr.zetaPhi };
  }
  const obsPoles = (o) => [...lib().pairPoles(o.wTh, o.zTh), ...lib().pairPoles(o.wPh, o.zPh)];

  // level: 'sf' (C.11), 'sfi' (C.12), 'obs' (C.13), 'dobs' (C.14)
  function design(p, st, level) {
    const { A, B, C } = lib().ss(p);
    const w = wnPair(st);
    const out = { ...w, poles: [...w.ph, ...w.th] };
    if (level === 'sf') {
      const K = WB.yt.place(A, B, out.poles);
      out.K = K ? K[0] : [NaN, NaN, NaN, NaN];
      const Ai = L.inv(L.sub(A, L.mul(B, [out.K])));
      out.kr = Ai ? -1 / L.mul(L.mul([[0, 1, 0, 0]], Ai), B)[0][0] : NaN;
      return out;
    }
    const { A1, B1 } = augI(A, B);
    out.poles = [...out.poles, { re: st.pI, im: 0 }];
    const K1 = WB.yt.place(A1, B1, out.poles);
    out.K = K1 ? K1[0].slice(0, 4) : [NaN, NaN, NaN, NaN];
    out.ki = K1 ? K1[0][4] : NaN;
    if (level === 'obs' || level === 'dobs') {
      out.obsPoles = obsPoles(st.obs);
      out.L = WB.yt.observer(A, C, out.obsPoles);
    }
    if (level === 'dobs') {
      const { A2, C2 } = augD(A, B, C);
      out.dPoles = [...out.obsPoles, { re: st.pD, im: 0 }];
      out.L2 = WB.yt.observer(A2, C2, out.dPoles);
    }
    return out;
  }

  // Gains from the tuning knobs. Work mode runs the student's controller, so these
  // only feed the (locked) math cards there.
  const obsOf = (ctx) => ctx.st.obs;
  function gainsFor(ctx, level) {
    return design(ctx.pModel, { ...ctx.st, obs: obsOf(ctx) }, level);
  }

  // ----------------------------------------------------------- controller --
  function makeSS(ctx, level) {
    const { sys, pModel, S } = ctx;
    const st = ctx.st, g = ctx.gains;
    const { A, B, C } = lib().ss(pModel);
    const Ts = S.sim.Ts, uLim = sys.uLimit(pModel);
    const useDO = level === 'dobs' && st.dobs;
    const useObs = level === 'obs' || level === 'dobs';
    const { A2, C2 } = augD(A, B, C);
    const Ao = useDO ? A2 : A, Co = useDO ? C2 : C, Lo = useDO ? g.L2 : g.L;
    const Bo = useDO ? [...B.map((r) => r[0]), 0] : B.map((r) => r[0]);
    const no = Ao.length;
    let xh = new Array(no).fill(0);
    xh[0] = (st.xhat0 || 0) / R2D;
    let I = 0, ePrev = 0, tauPrev = 0;
    // observer_f: A x̂ + B u_{k−1} + L (y − C x̂)
    const fObs = (z, y) => {
      const innov = Co.map((row, i) => y[i] - row.reduce((s, c, j) => s + c * z[j], 0));
      return Ao.map((row, i) => row.reduce((s, a, j) => s + a * z[j], 0) + Bo[i] * tauPrev + Lo[i].reduce((s, l, j) => s + l * innov[j], 0));
    };
    const Kx = (x) => g.K[0] * x[0] + g.K[1] * x[1] + g.K[2] * x[2] + g.K[3] * x[3];
    return {
      update(r, x, yMeas) {
        let xu = x;
        if (useObs) { xh = M.rk4Step((zz) => fObs(zz, yMeas), xh, 0, Ts); xu = xh; }
        const dh = useDO ? xh[4] : 0;
        let tauU;
        if (level === 'sf') {
          tauU = -Kx(xu) + g.kr * r;
        } else {
          const e = r - xu[1];
          const Inew = I + (Ts / 2) * (e + ePrev);
          tauU = -Kx(xu) - g.ki * Inew - dh;
          // anti-windup (C.12a): hold the integrator while τ is saturated
          if (st.antiwindup === 'clamp' && Math.abs(tauU) > uLim) tauU = -Kx(xu) - g.ki * I - dh;
          else I = Inew;
          ePrev = e;
        }
        // The observer uses the saturated τ (tau_d1 in the repo); the demand is
        // returned unsaturated and the simulation clips it.
        tauPrev = M.saturate(tauU, uLim);
        return { u: tauU, thHat: xu[0], phHat: xu[1], thdHat: xu[2], phdHat: xu[3], dhat: dh, integrator: I };
      },
    };
  }

  // ------------------------------------------------------------- controls --
  function tuningSliders(parent, ctx, { pI = false } = {}) {
    const st = ctx.st;
    slider(parent, { label: 't<sub>r<sub>θ</sub></sub>', unit: 's', min: 0.2, max: 6, step: 0.01, sig: 3, ...bind(ctx, 'trTh', () => st) });
    slider(parent, { label: 'ζ<sub>θ</sub>', min: 0.2, max: 0.99, step: 0.005, sig: 3, ...bind(ctx, 'zetaTh', () => st) });
    slider(parent, { label: 'M = t<sub>r<sub>φ</sub></sub>/t<sub>r<sub>θ</sub></sub>', min: 1, max: 10, step: 0.05, sig: 3, ...bind(ctx, 'M', () => st) });
    slider(parent, { label: 'ζ<sub>φ</sub>', min: 0.2, max: 0.99, step: 0.005, sig: 3, ...bind(ctx, 'zetaPhi', () => st) });
    segmented(parent, {
      label: 'ω<sub>n</sub> from t<sub>r</sub>',
      options: [{ value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))' }, { value: '2.2', label: '2.2 / t<sub>r</sub>' }],
      ...bind(ctx, 'rule', () => st),
    });
    if (pI) slider(parent, { label: 'p<sub>I</sub>', min: -10, max: -0.05, step: 0.01, sig: 3, ...bind(ctx, 'pI', () => st) });
  }
  function obsSliders(parent, ctx, { withD = false } = {}) {
    const o = obsOf(ctx);
    slider(parent, { label: 'ω<sub>n,obs,θ</sub>', unit: 'rad/s', min: 0.5, max: 60, step: 0.05, sig: 4, ...bind(ctx, 'wTh', () => o) });
    slider(parent, { label: 'ω<sub>n,obs,φ</sub>', unit: 'rad/s', min: 0.2, max: 40, step: 0.05, sig: 4, ...bind(ctx, 'wPh', () => o) });
    slider(parent, { label: 'ζ<sub>obs</sub>', min: 0.3, max: 0.99, step: 0.005, sig: 3, get: () => o.zTh, set: (v) => { o.zTh = v; o.zPh = v; ctx.update(); } });
    if (withD) slider(parent, { label: 'p<sub>d</sub>', min: -40, max: -0.2, step: 0.1, sig: 3, ...bind(ctx, 'pD') });
  }
  // Work-mode control panel: the plots show the student's controller (part `part`).
  function workBanner(parent, ctx, part) {
    const sec = section(parent, 'Your controller');
    WB.myCtrl.banner(sec, ctx, part);
  }
  function awControl(parent, ctx) {
    segmented(parent, {
      label: 'Anti-windup (C.12a)',
      options: [{ value: 'clamp', label: 'hold integrator while saturated' }, { value: 'none', label: 'none (repo)' }],
      ...bind(ctx, 'antiwindup'),
    });
  }
  function gainsReadout(parent, ctx, keys) {
    WB.ui.readout(parent, () => {
      const g = ctx.gains;
      const v = { K1: g.K[0], K2: g.K[1], K3: g.K[2], K4: g.K[3], kr: g.kr, ki: g.ki };
      return keys.map((k) => [k, v[k]]);
    });
  }

  // ------------------------------------------------------------- analysis --
  function clPoles(ctx, level) {
    const { A, B } = lib().ss(ctx.pModel), g = ctx.gains;
    if (level === 'sf') return L.eig(L.sub(A, L.mul(B, [g.K])));
    const { A1, B1 } = augI(A, B);
    return L.eig(L.sub(A1, L.mul(B1, [[...g.K, g.ki]])));
  }

  function markers(ctx, level, specPoles) {
    const { A } = lib().ss(ctx.pModel), g = ctx.gains, st = ctx.st;
    const explore = ctx.S.mode === 'explore';
    // The open-loop poles answer C.5(b) / C.6: hidden in Work mode until one is solved.
    const mk = lib().showsOl(ctx) ? L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `open-loop pole ${i + 1}` })) : [];
    if (explore) {
      g.th.forEach((q) => mk.push({ ...q, kind: 'cl', label: 'controller pole (θ pair)', dragId: 'cth' }));
      g.ph.forEach((q) => mk.push({ ...q, kind: 'cl', label: 'controller pole (φ pair)', dragId: 'cph' }));
      if (level !== 'sf') mk.push({ re: st.pI, im: 0, kind: 'cl', label: 'integrator pole p_I', dragId: 'cI' });
    } else {
      // Work mode has no gains (the plots run the student's own controller) and no
      // target poles (the problem gives no tuning beyond C.8's; the listing's is the solution's).
      return mk;
    }
    if (level === 'obs' || level === 'dobs') {
      const o = obsOf(ctx);
      const dg = (id) => (explore ? id : undefined);
      lib().pairPoles(o.wTh, o.zTh).forEach((q) => mk.push({ ...q, kind: 'obs', label: 'observer pole (θ pair)', dragId: dg('oth') }));
      lib().pairPoles(o.wPh, o.zPh).forEach((q) => mk.push({ ...q, kind: 'obs', label: 'observer pole (φ pair)', dragId: dg('oph') }));
      if (level === 'dobs' && st.dobs) mk.push({ re: st.pD, im: 0, kind: 'obs', label: 'disturbance-observer pole p_d', dragId: dg('od') });
    }
    return mk;
  }

  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.01, re);
    const wn = Math.hypot(re, im);
    const zeta = Math.max(0.2, Math.min(0.99, -re / wn));
    const inv = (w, z) => WB.design.trFromWn(w, z, st.rule);
    if (id === 'cth') { st.zetaTh = zeta; st.trTh = inv(wn, zeta); }
    else if (id === 'cph') { st.zetaPhi = zeta; st.M = Math.max(1, inv(wn, zeta) / st.trTh); }
    else if (id === 'cI') st.pI = re;
    else if (id === 'oth') { st.obs.wTh = wn; st.obs.zTh = zeta; }
    else if (id === 'oph') { st.obs.wPh = wn; st.obs.zPh = zeta; }
    else if (id === 'od') st.pD = re;
    ctx.update();
  }

  // ----------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B } = lib().ss(ctx.pModel);
    return {
      title: 'State-space model (C.6)', page: 'p. 92, p. 193 · Eq. 11.40', answers: 'C.6/a',
      theory: '\\dot x = Ax + B\\tau,\\quad x = (\\theta, \\phi, \\dot\\theta, \\dot\\phi)^\\top,\\quad y = (\\theta, \\phi)^\\top,\\quad y_r = \\phi = \\begin{bmatrix}0 & 1 & 0 & 0\\end{bmatrix}x',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`,
    };
  }
  // The general method, then this tuning's numbers. In Work mode the tuning is the
  // problem's (C.11-C.14 share it), so its poles answer C.11(a) and stay locked.
  function polesCard(ctx, d, level, answers = 'C.11/a') {
    const st = ctx.st, work = ctx.S.mode === 'work';
    const wTex = st.rule === 'tp' ? '\\omega_n = \\frac{\\pi}{2t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}';
    // Work mode: only the general form (the tuning knobs default to the listing's values).
    if (work) return [{ title: 'Desired closed-loop poles', page: 'p. 193', theory: '\\Delta^d = (s^2 + 2\\zeta_\\theta\\omega_{n_\\theta}s + \\omega_{n_\\theta}^2)(s^2 + 2\\zeta_\\phi\\omega_{n_\\phi}s + \\omega_{n_\\phi}^2)' + (level === 'sf' ? '' : '(s - p_I)') }];
    return [
      { title: 'Desired closed-loop poles', page: 'p. 193, p. 194 (Listing 11.3)',
        theory: `${wTex},\\quad t_{r_\\phi} = M t_{r_\\theta},\\quad \\Delta^d = (s^2 + 2\\zeta_\\theta\\omega_{n_\\theta}s + \\omega_{n_\\theta}^2)(s^2 + 2\\zeta_\\phi\\omega_{n_\\phi}s + \\omega_{n_\\phi}^2)` + (level === 'sf' ? '' : '(s - p_I)') },
      { title: 'Desired poles for this tuning', page: 'p. 193',
        theory: `\\omega_{n_\\theta} = ${tex(d.wnTh)},\\; \\omega_{n_\\phi} = ${tex(d.wnPhi)},\\quad \\Delta^d = ${WB.tf.polyTex(L.polyFromRoots(d.poles))}`,
        note: 'For the t_r, ζ, M tuning (set in Explore mode; the problem\'s tuning by default).',
        answers },
    ];
  }
  const ctrbCard = (A, B, title, page) => WB.ss.ctrbCard(A, B, title, page);
  function stateDefaults(pr, extra = {}) {
    return { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, pI: pr.pI ?? -2, antiwindup: 'clamp', ...extra };
  }
  const specOf = (ctx, pr, level) => design(ctx.pModel, { ...ctx.st, trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, pI: pr.pI ?? -2, obs: pr.obsFactor ? obsDefaults(pr) : ctx.st.obs, pD: pr.pD ?? ctx.st.pD }, level);

  // ------------------------------------------------------ controller checks --
  const SQUARE = (amplitude, frequency) => ({ type: 'square', amplitude, frequency, tStep: 0 });
  const errAt = (res, t) => lib().phiErrAt(res, t);
  // The student's estimate must come back from update as (tau, x_hat).
  function needXhat(res) {
    const xs = [0, 1, 2, 3].map((i) => res.extras[`xhat${i}`]);
    if (xs.some((a) => !a || Array.prototype.some.call(a, (v) => !Number.isFinite(v)))) return { ok: false, msg: 'Return (tau, x_hat) from update, with x_hat = [[theta_hat], [phi_hat], [thetadot_hat], [phidot_hat]], so the check can see your estimate.' };
    return null;
  }
  // Largest |x − x̂| per state (deg, deg/s) over the given time windows.
  function estErr(res, windows) {
    const e = [0, 0, 0, 0];
    res.t.forEach((t, k) => {
      if (!windows.some(([a, b]) => t >= a - 1e-9 && t <= b + 1e-9)) return;
      for (let i = 0; i < 4; i++) e[i] = Math.max(e[i], Math.abs(res.x[k][i] - res.extras[`xhat${i}`][k]));
    });
    return e.map((v) => v * R2D);
  }
  // Mean θ − θ̂ and φ − φ̂ (deg) over [t0, t1].
  const biasOf = (res, t0, t1) => [0, 1].map((i) => WB.myCtrl.mean(res, t0, t1, (k) => res.x[k][i] - res.extras[`xhat${i}`][k]) * R2D);

  // Check settings. C.11: square wave (°, Hz) and the error before a switch (°).
  const C11 = { amp: 10, freq: 0.04, tol: 0.1 };
  // C.12(a): a small square wave with a constant disturbance (the integrator must
  // remove the error; τ stays unsaturated); then the C.12(b) disturbance on a plant
  // with J_s above the model (an α = 0.2 draw for which hw12 diverges without anti-windup).
  const C12 = { amp: 5, dist: 0.5, mismatch: { Js: 12, Jp: 9, k: 15, b: -7 }, phiMax: 20 };
  // C.13(c): initial angles (°; larger ones saturate τ while x̂ converges), settled
  // window before each switch (s), estimate tolerances (°, °/s).
  const C13 = { init: 2, freq: 0.03, settle: 4.6, tolAngle: 0.02, tolRate: 0.1 };
  // C.14(b): mean estimator bias (°) under d, noise and mismatch.
  const C14 = { tolBias: 0.02 };

  // C.11(a, d), C.12(a): Python functions of the desired poles (the problem gives
  // C.8's specs, not the listing's tuning), checked at random values.
  const cx = WB.py.cx;
  const POLE_ARGS = {
    s: { label: 's', complex: true, re: [-3, 1], im: [0.3, 4] },
    wn_th: { label: 'ωn,θ', lo: 0.8, hi: 4 }, zeta_th: { label: 'ζθ', lo: 0.6, hi: 0.95 },
    wn_phi: { label: 'ωn,φ', lo: 0.1, hi: 0.7 }, zeta_phi: { label: 'ζφ', lo: 0.6, hi: 0.95 },
  };
  const pairsOf = (a) => [...lib().pairPoles(a.wn_th, a.zeta_th), ...lib().pairPoles(a.wn_phi, a.zeta_phi)];
  // C.8's poles (t_rθ = 1 s, t_rφ = 10 s, ζ = 0.9, the C.8 solution's ω_n rule), for solutions.
  function c8Pairs() {
    const c8 = WB.systems.C.problems.ch8, w = (tr, z) => lib().wnRule(tr, z, c8.rule);
    const wnTh = w(c8.trTh, c8.zetaTh), wnPhi = w(c8.M * c8.trTh, c8.zetaPhi);
    return { wnTh, wnPhi, poles: [...lib().pairPoles(wnTh, c8.zetaTh), ...lib().pairPoles(wnPhi, c8.zetaPhi)] };
  }
  function sfGains(p, poles) {
    const { A, B } = lib().ss(p);
    const K = WB.yt.place(A, B, poles)[0];
    const Ai = L.inv(L.sub(A, L.mul(B, [K])));
    return { K, kr: -1 / L.mul(L.mul([[0, 1, 0, 0]], Ai), B)[0][0] };
  }
  function sfiGains(p, poles) {
    const { A, B } = lib().ss(p);
    const { A1, B1 } = augI(A, B);
    const K1 = WB.yt.place(A1, B1, poles)[0];
    return { K: K1.slice(0, 4), ki: K1[4] };
  }

  function base(level, num, title, pages, extra) {
    return Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages, level,
      controller(ctx) { return makeSS(ctx, level); },
      gains(ctx) { return gainsFor(ctx, level); },
      splane(ctx) {
        const pr = ctx.sys.problems[`ch${num}`];
        return { markers: markers(ctx, level, specOf(ctx, pr, level).poles), legendNames: { cl: 'controller pole', obs: 'observer pole', target: 'target pole (problem)' } };
      },
      onPoleDrag: onDrag,
      targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.M * ctx.st.trTh } : {}; },
    }, extra);
  }

  // ------------------------------------------------------------- C.11 --
  CH.ch11 = base('sf', 11, 'Full state feedback', 'pp. 192–195', {
    // Work mode simulates the student's C.11(e) controller, from the full state (hw11).
    implement: { feed: 'state', linear: false },
    defaults(sys) { return stateDefaults(sys.problems.ch11); },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, 'C.11(e)'); return; }
      const sec = section(parent, 'u = −Kx + k_r φ_r (true state)', 'p. 192 · Listing 11.3');
      tuningSliders(sec, ctx); gainsReadout(sec, ctx, ['K1', 'K2', 'K3', 'K4', 'kr']);
      sec.append(el('p', { class: 'muted small', text: 'Drag a θ-pair or φ-pair pole in the s-plane.' }));
    },
    math(ctx) {
      const d = design(ctx.pModel, ctx.st, 'sf'), g = ctx.gains;
      const { A, B } = lib().ss(ctx.pModel);
      return [
        ssCard(ctx),
        { ...ctrbCard(A, B, 'Controllability', 'p. 193 · Step 1'), answers: 'C.11/c' },
        { title: 'Open-loop characteristic polynomial', page: 'p. 193 · Step 2',
          theory: '\\Delta_{ol}(s) = \\det(sI - A) = s^4 + a_3s^3 + a_2s^2 + a_1s + a_0',
          numbers: `\\Delta_{ol} = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
        ...polesCard(ctx, d, 'sf', 'C.11/a'),
        { title: 'Pole placement and reference gain', page: 'p. 194 · Step 4, Eq. 11.35',
          theory: 'K = (\\alpha - a_A)\\mathcal{A}_A^{-1}\\mathcal{C}_{A,B}^{-1},\\quad k_r = \\frac{-1}{C_r(A - BK)^{-1}B},\; C_r = \\begin{bmatrix}0 & 1 & 0 & 0\\end{bmatrix}',
          note: 'At steady state the spring forces θ = φ, so C_r = [1 0 0 0] (what the listing uses) gives the same k_r.' },
        ...(ctx.S.mode === 'work' ? [] : [{ title: 'K and k_r for this tuning', page: 'p. 194',
          theory: `K = ${texMat([d.K])},\\quad k_r = ${tex(d.kr)}` }]),
        { title: 'Control law', page: 'p. 195 · Listing 11.3', answers: 'C.11/e',
          theory: '\\tau = \\text{sat}\\left(-Kx + k_r\\phi_r\\right)',
          numbers: ctx.S.mode === 'work' ? null : `\\tau = -(${g.K.map((v) => tex(v)).join(',\;')})\\,x + ${tex(g.kr)}\\,\\phi_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch11;
      const { A, B } = lib().ss(ctx.pModel);
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Desired closed-loop poles from the C.8 ω<sub>n</sub> and ζ',
          html: 'Write the desired characteristic polynomial Δ<sup>d</sup>(s) as a function of the two loops\' ω<sub>n</sub> and ζ (yours from C.8: t<sub>r<sub>θ</sub></sub> = 1 s, t<sub>r<sub>φ</sub></sub> = 10 t<sub>r<sub>θ</sub></sub>, ζ = 0.9). The check calls it at complex s and random values.',
          code: {
            template: 'def desired_char_poly(s, wn_th, zeta_th, wn_phi, zeta_phi):\n    # Delta^d(s)\n    return ...\n',
            check: (code) => WB.py.check(ctx, { args: POLE_ARGS, items: [{ fn: 'desired_char_poly', args: ['s', 'wn_th', 'zeta_th', 'wn_phi', 'zeta_phi'], truth: (p, a) => cx.poly(L.polyFromRoots(pairsOf(a)), a.s) }] }, code),
          },
          solution: () => { const c8 = c8Pairs(); return [
            { code: 'def desired_char_poly(s, wn_th, zeta_th, wn_phi, zeta_phi):\n    return ((s**2 + 2 * zeta_th * wn_th * s + wn_th**2)\n            * (s**2 + 2 * zeta_phi * wn_phi * s + wn_phi**2))\n' },
            { tex: `\\text{C.8 values } (\\omega_n = \\pi/(2t_r\\sqrt{1-\\zeta^2})):\\; \\omega_{n_\\theta} = ${tex(c8.wnTh)},\\; \\omega_{n_\\phi} = ${tex(c8.wnPhi)},\\quad \\Delta^d = ${WB.tf.polyTex(L.polyFromRoots(c8.poles))}` },
            { html: 'The printed solution (p. 193) uses ω<sub>θ</sub> = 1.9848, ω<sub>φ</sub> = 1.5, ζ = 0.707, and the listing t<sub>r<sub>θ</sub></sub> = 2 s, M = 3; see ISSUES.md.' },
          ]; },
        },
        {
          id: 'b', title: '(b) Add A, B, C, D from C.6',
          html: 'Use your matrices from C.6 (the Ch 6 tab). From here on the outputs are y = (θ, φ), as in the C.13 solution, so the second row of C is (0, 1, 0, 0). The model card in the Math section unlocks once C.6 is solved.',
        },
        {
          id: 'c', title: '(c) Controllability',
          inputs: { rank: 'rank 𝒞<sub>A,B</sub>', det: 'det 𝒞<sub>A,B</sub>' },
          check: (v) => { const Cm = L.ctrb(A, B); return PD().checkNumbers(v, { rank: L.rank(Cm), det: L.det(Cm) }, { det: 'det' }); },
          solution: () => { const Cm = L.ctrb(A, B); return [{ tex: `\\mathcal{C}_{A,B} = ${texMat(Cm)},\\quad \\det = ${tex(L.det(Cm))} \\ne 0` }]; },
        },
        {
          id: 'd', title: '(d) K and k<sub>r</sub>',
          html: 'As a function of the desired poles\' ω<sub>n</sub>, ζ (as in (a)): return K (1×4) and k<sub>r</sub>, with the model from <code>P</code>. <code>import control as cnt</code> gives <code>cnt.place</code>. The check calls it at random values.',
          code: {
            template: 'def gains(wn_th, zeta_th, wn_phi, zeta_phi):\n    # K (1x4) and k_r\n    return K, kr\n',
            check: (code) => WB.py.check(ctx, { args: POLE_ARGS, items: [{ fn: 'gains', args: ['wn_th', 'zeta_th', 'wn_phi', 'zeta_phi'], truth: (p, a) => { const g = sfGains(p, pairsOf(a)); return [g.K, g.kr]; } }] }, code),
          },
          solution: () => { const g = sfGains(ctx.pModel, c8Pairs().poles); return [
            { code: `import control as cnt\n\ndef gains(wn_th, zeta_th, wn_phi, zeta_phi):\n${MODEL_PY.replace(/^ {4}/gm, '')}    Cr = np.array([[0.0, 1.0, 0.0, 0.0]])\n    poles = np.roots(np.convolve([1, 2 * zeta_th * wn_th, wn_th**2],\n                                 [1, 2 * zeta_phi * wn_phi, wn_phi**2]))\n    K = cnt.place(A, B, poles)\n    kr = -1.0 / (Cr @ np.linalg.inv(A - B @ K) @ B)[0, 0]\n    return K, kr\n` },
            { tex: `\\text{C.8 poles: } K = ${texMat([g.K])},\\quad k_r = ${tex(g.kr)}` },
            { html: 'Book (p. 194): K = (40.28, 255.17, 24.34, 366.18), k<sub>r</sub> = 295.46, for its own poles and a different A.' },
          ]; },
        },
        WB.myCtrl.part(ctx, {
          id: 'e', title: '(e) Implement the state feedback and tune the closed-loop poles',
          html: `<code>update</code> gets φ<sub>r</sub> and the full state x = (θ, φ, θ̇, φ̇), as in the book's C.11 code. <code>import control as cnt</code> gives <code>cnt.place</code>. The check runs a ±${C11.amp}° square wave (${C11.freq} Hz) with the nominal and with other parameters: |φ<sub>r</sub> − φ| just before each of the first two switches must be under ${C11.tol}°. Compare the response with the successive-loop-closure designs of Ch 8 and Ch 10.`,
          check: async (code) => {
            const tSw = 0.5 / C11.freq, msgs = [];
            for (const pc of WB.myCtrl.paramCases(ctx)) {
              const sc = WB.myCtrl.scenario(ctx, { params: pc.params, ref: SQUARE(C11.amp, C11.freq), tEnd: 2 * tSw });
              const res = await WB.myCtrl.run(ctx, code, sc);
              if (res.ok === false) return res;
              const e = Math.max(errAt(res, tSw - 0.05), errAt(res, 2 * tSw - 0.05));
              if (!(e < C11.tol)) return { ok: false, msg: `With ${pc.label}: |φ_r − φ| before a switch is ${fmt(e, 3)}°.${msgs.length ? ' (Nominal parameters pass: compute the gains from P.)' : ''}` };
              msgs.push(`${fmt(e, 3)}° (${pc.label})`);
            }
            return { ok: true, msg: `Largest error before a switch: ${msgs.join('; ')}.` };
          },
          solution: () => [{ code: SOL.ch11 }, { html: 'As the repo\'s ctrlStateFeedback.py (the listing\'s tuning, k<sub>r</sub> with C<sub>r</sub> on φ), with the output saturated at τ<sub>max</sub>.' }],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------- C.12 --
  CH.ch12 = base('sfi', 12, 'Integrator with state feedback', 'pp. 210–214', {
    // Work mode simulates the student's C.12 controller, from the full state (hw12).
    implement: { feed: 'state', linear: false },
    defaults(sys) { return stateDefaults(sys.problems.ch12); },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, 'C.12(a) or (c)'); return; }
      const sec = section(parent, 'u = −Kx − k_I ∫(φ_r − φ)', 'p. 211 · Listing 12.3');
      awControl(sec, ctx);
      tuningSliders(sec, ctx, { pI: true }); gainsReadout(sec, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']);
    },
    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [rad·s]', unit: 'rad·s' }, data: { series: [{ label: 'x_I = ∫(φ_r − φ)', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = design(ctx.pModel, ctx.st, 'sfi');
      const { A, B } = lib().ss(ctx.pModel);
      const { A1, B1 } = augI(A, B);
      return [
        ssCard(ctx),
        { title: 'Augmented system', page: 'p. 211 · Step 1', answers: 'C.12/a2',
          theory: '\\dot x_I = \\phi_r - C_rx,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C_r & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\ 0\\end{bmatrix},\\quad C_r = \\begin{bmatrix}0 & 1 & 0 & 0\\end{bmatrix}',
          numbers: `A_1 = ${texMat(A1)}`, spoiler: true },
        ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 212'),
        ...polesCard(ctx, d, 'sfi'),
        { title: 'Gains', page: 'p. 212 · Step 3', answers: 'C.12/a2',
          theory: 'K_1 = \\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad \\tau = -Kx - k_I\\int_0^t(\\phi_r - \\phi)\\,d\\tau' },
        ...(ctx.S.mode === 'work' ? [] : [{ title: 'K and k_I for this tuning', page: 'p. 212',
          theory: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}` }]),
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch12;
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Gains with the integrator on φ',
          html: 'As a function of the desired poles (the two pairs as in C.11, plus the integrator pole p<sub>I</sub>): return K (1×4) and k<sub>I</sub> for τ = −Kx − k<sub>I</sub>∫(φ<sub>r</sub> − φ). The check calls it at random values.',
          code: {
            template: 'def gains(wn_th, zeta_th, wn_phi, zeta_phi, p_I):\n    # K (1x4) and k_I\n    return K, kI\n',
            check: (code) => WB.py.check(ctx, { args: { ...POLE_ARGS, p_I: { label: 'p_I', lo: -5, hi: -0.3 } }, items: [{ fn: 'gains', args: ['wn_th', 'zeta_th', 'wn_phi', 'zeta_phi', 'p_I'], truth: (p, a) => { const g = sfiGains(p, [...pairsOf(a), { re: a.p_I, im: 0 }]); return [g.K, g.ki]; } }] }, code),
          },
          solution: () => [
            { code: `import control as cnt\n\ndef gains(wn_th, zeta_th, wn_phi, zeta_phi, p_I):\n${MODEL_PY.replace(/^ {4}/gm, '')}    Cr = np.array([[0.0, 1.0, 0.0, 0.0]])\n    A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]])\n    B1 = np.vstack([B, [[0.0]]])\n    poles = np.roots(np.convolve(np.convolve([1, 2 * zeta_th * wn_th, wn_th**2],\n                                             [1, 2 * zeta_phi * wn_phi, wn_phi**2]),\n                                 [1, -p_I]))\n    K1 = cnt.place(A1, B1, poles)\n    return K1[:, 0:4], K1[0, 4]\n` },
            { html: 'Book (p. 212): K = (19.15, 43.41, 16.72, 111.63), k<sub>I</sub> = −14.52, for p<sub>I</sub> = −1 and different poles and A. The repo uses t<sub>r<sub>θ</sub></sub> = 2 s, M = 3, ζ = 0.9, p<sub>I</sub> = −2.' },
          ],
        },
        WB.myCtrl.part(ctx, {
          id: 'a2', title: '(a) Add an integrator with anti-windup on φ to your C.11 controller', seed: 'C.11/e',
          html: `Use gains from (a) with poles you choose. The check (1) runs a ±${C12.amp}° square wave (0.02 Hz) with a ${C12.dist} N·m input disturbance, with the nominal and with other parameters: |φ<sub>r</sub> − φ| just before each of the first two switches must be under 0.1°; then (2) runs the ±15° square wave (0.01 Hz) with d = 1 N·m on a plant that differs from the model by ${lib().misText(C12.mismatch)}: τ saturates at the switches, and |φ| must stay under ${C12.phiMax}°.`,
          check: async (code) => {
            // (1) tracking with a constant disturbance: the integrator must remove the error
            const errs = [];
            for (const pc of WB.myCtrl.paramCases(ctx)) {
              const sc1 = WB.myCtrl.scenario(ctx, { params: pc.params, ref: SQUARE(C12.amp, 0.02), tEnd: 50, dist: C12.dist });
              const r1 = await WB.myCtrl.run(ctx, code, sc1);
              if (r1.ok === false) return r1;
              const e = Math.max(errAt(r1, 24.95), errAt(r1, 49.95));
              if (!(e < 0.1)) return { ok: false, msg: `With d = ${C12.dist} N·m (${pc.label}) |φ_r − φ| before a switch is ${fmt(e, 3)}°.` };
              errs.push(`${fmt(e, 3)}° (${pc.label})`);
            }
            const m = { msg: `Error before a switch with d = ${C12.dist} N·m: ${errs.join('; ')}.` };
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(15, 0.01), tEnd: 100, dist: 1, mismatch: C12.mismatch });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const peak = Math.max(...Array.from(res.yAll[1], Math.abs)) * R2D;
            if (!(peak <= C12.phiMax)) return { ok: false, msg: `Tracking works, but with d = 1 N·m on the ${lib().misText(C12.mismatch)} plant |φ| reaches ${fmt(peak, 3)}°: the integrator winds up while τ is saturated.` };
            return { ok: true, msg: `${m.msg} With d = 1 N·m and saturation: largest |φ| ${fmt(peak, 3)}°.` };
          },
          solution: () => [{ code: SOL.ch12 }, { html: 'Anti-windup here holds the integrator while τ is saturated; unwinding it by (τ<sub>sat</sub> − τ<sub>unsat</sub>)/k<sub>I</sub> passes too. Without it the second check diverges. The repo\'s ctrlStateFeedbackIntegrator.py has none (and integrates θ instead of φ; see ISSUES.md).' }],
        }),
        {
          id: 'b', title: '(b) Input disturbance of 1 N·m and 20% parameter uncertainty',
          html: 'Set an input disturbance d and the true-plant mismatch in the left panel (the chapter starts with d = 1 N·m and a fixed 20% draw).',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return Math.abs(S.sim.dist) > 0 && mis ? { ok: true, msg: `d = ${fmt(S.sim.dist, 3)} N·m with plant mismatch.` } : { ok: false, msg: 'Set both d ≠ 0 and a plant mismatch.' };
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Tune the integrator pole (and other gains if needed) for good tracking', seed: 'C.12/a2',
          html: `The check runs the ±15° square wave (0.01 Hz) with d = ${pr.sim.dist} N·m and the plant off by ${lib().misText(pr.mismatch)}: |φ<sub>r</sub> − φ| just before the first switch (t = 50 s) must be under 0.1°.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(15, 0.01), tEnd: 50, dist: pr.sim.dist, mismatch: pr.mismatch });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const e = errAt(res, 49.95);
            return { ok: e < 0.1, msg: `Error before the switch: ${fmt(e, 3)}°.` };
          },
          solution: () => [{ code: SOL.ch12 }, { html: `The design from (a) (p<sub>I</sub> = ${pr.pI}) already passes.` }],
        }),
      ]);
    },
  });

  // ------------------------------------------------------- C.13 and C.14 --
  // Estimates: the workbench's observer (Explore) or the student's x̂ (Work).
  const hatOf = (ctx, res, i) => (ctx.S.mode === 'work' ? res.extras[`xhat${i}`] : res.extras[['thHat', 'phHat'][i]]) || [];
  function obsSeries(ctx, res, sc, oi) {
    const y = hatOf(ctx, res, oi);
    if (!y.length) return [];
    return [{ label: `${oi === 0 ? 'θ̂' : 'φ̂'} (${ctx.S.mode === 'work' ? 'your estimate' : 'observer'})`, y: sc(y), color: '--series-3', dash: [3, 3], width: 2 }];
  }
  function obsCards(ctx, d, level) {
    const { A, C } = lib().ss(ctx.pModel);
    const O = L.obsv(A, C);
    const cards = [
      { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 237 · Listing 13.3', answers: 'C.13/c',
        theory: '\\dot{\\hat x} = A\\hat x + B\\tau_{k-1} + L(y_m - C\\hat x),\\quad y = (\\theta, \\phi)^\\top,\\quad L \\in \\mathbb{R}^{4\\times 2}' },
      { title: 'Observability', page: 'p. 221',
        theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ \\vdots \\\\ CA^{n-1}\\end{bmatrix},\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = n' },
      { title: 'Observability of the satellite', page: 'p. 234', answers: 'C.13/b',
        theory: `\\operatorname{rank}\\mathcal{O}_{A,C} = ${L.rank(O)}\;(\\text{already } ${L.rank(O.slice(0, 4))} \\text{ from } C, CA)` },
      { title: 'Observer gain (two outputs)', page: 'p. 236 · Listing 13.3', answers: 'C.13/c',
        theory: 'L = \\text{place}(A^\\top, C^\\top, q)^\\top,\\quad q = \\text{roots}\\big((s^2 + 2\\zeta\\omega_{obs,\\theta}s + \\omega_{obs,\\theta}^2)(s^2 + 2\\zeta\\omega_{obs,\\phi}s + \\omega_{obs,\\phi}^2)\\big)',
        numbers: d.L && ctx.S.mode !== 'work' ? `q = ${d.obsPoles.map((q) => texPole(q)).join(',\;')},\\quad L^\\top = ${texMat(L.T(d.L))}` : '', spoiler: true,
        note: 'With two outputs many L place the same poles. The one shown is what python-control\'s place (scipy\'s YT algorithm) returns, ported here so the simulation matches the repo.' },
    ];
    if (level === 'dobs') {
      const { B } = lib().ss(ctx.pModel);
      const { A2, C2 } = augD(A, B, C);
      cards.push({ title: 'Disturbance observer', page: 'p. 240–241, pp. 258–259 · Listing 14.6', answers: 'C.14/b',
        theory: 'A_2 = \\begin{bmatrix}A & B\\\\ 0 & 0\\end{bmatrix},\; C_2 = \\begin{bmatrix}C & 0\\end{bmatrix},\\quad \\tau = -K\\hat x - k_I\\textstyle\\int e - \\hat d',
        numbers: d.L2 && ctx.S.mode !== 'work' ? `L_2^\\top = ${texMat(L.T(d.L2))}` : '', spoiler: true });
      cards.push({ title: 'Observability of the augmented model', page: 'p. 254', answers: 'C.14/b',
        theory: `\\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}` });
    }
    return cards;
  }

  CH.ch13 = base('obs', 13, 'Observers', 'pp. 234–238', {
    // Work mode simulates the student's C.13(c) controller, from y = (θ, φ).
    implement: { feed: 'y', linear: false },
    defaults(sys) { const pr = sys.problems.ch13; return stateDefaults(pr, { obs: obsDefaults(pr), xhat0: 0, antiwindup: 'none' }); },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries: obsSeries,
    extraPlot(ctx, res) {
      const th = hatOf(ctx, res, 0), ph = hatOf(ctx, res, 1);
      if (!th.length) return null;
      return { opts: { title: 'estimation error', yLabel: 'error [°]', unit: '°' }, data: { series: [
        { label: 'θ − θ̂', y: res.x.map((x, k) => (x[0] - th[k]) * R2D), color: '--series-1' },
        { label: 'φ − φ̂', y: res.x.map((x, k) => (x[1] - ph[k]) * R2D), color: '--series-3', width: 1.5 },
      ] } };
    },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, 'C.13(c)'); return; }
      const sec = section(parent, 'Controller (uses x̂)', 'p. 235 · Listing 13.3');
      awControl(sec, ctx);
      tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer poles', 'p. 236');
      obsSliders(ob, ctx);
      slider(ob, { label: 'θ̂(0)', unit: '°', min: -30, max: 30, step: 0.5, sig: 3, hint: 'initial estimate (the plant starts at the left panel values)', ...bind(ctx, 'xhat0') });
      gainsReadout(ob, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']);
      ob.append(el('p', { class: 'muted small', text: 'L has eight entries and is not unique with two outputs, so you pick observer poles and L is placed for you.' }));
    },
    math(ctx) {
      const d = design(ctx.pModel, { ...ctx.st, obs: obsOf(ctx) }, 'obs');
      return [ssCard(ctx), ...obsCards(ctx, d, 'obs'), ...polesCard(ctx, d, 'obs'),
        { title: 'Separation principle', page: 'p. 223–224',
          theory: '\\text{eig}\\begin{bmatrix}A - BK & BK\\\\ 0 & A - LC\\end{bmatrix} = \\text{eig}(A - BK) \\cup \\text{eig}(A - LC)' }];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch13;
      const { A, C } = lib().ss(ctx.pModel);
      const tSw = 0.5 / C13.freq;
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) Exact parameters, no input disturbance',
          html: 'Set α = 0 in your dynamics. Here, the chapter starts with no mismatch and d = 0; <em>Exact model</em> in the left panel restores them.',
          check: () => {
            const S = ctx.S, mis = Object.values(S.mismatch || {}).some((v) => Math.abs(v) > 0);
            return !mis && !(Math.abs(S.sim.dist) > 0) ? { ok: true, msg: 'Exact model, no disturbance.' } : { ok: false, msg: 'Remove the plant mismatch and the disturbance.' };
          },
        },
        {
          id: 'b', title: '(b) Observability',
          inputs: { rank: 'rank 𝒪<sub>A,C</sub>' },
          check: (v) => PD().checkNumbers(v, { rank: L.rank(L.obsv(A, C)) }, {}),
          solution: () => [{ tex: '\\mathcal{O}_{A,C} = \\begin{bmatrix}C\\\\ CA\\end{bmatrix} = \\begin{bmatrix}I & 0\\\\ 0 & I\\end{bmatrix} \\text{ already has rank 4}' }],
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Add an observer and use x̂ in your C.12 controller; tune the controller and observer poles', seed: ['C.12/c', 'C.12/a2'],
          html: `<code>update</code> now gets only y = [[θ], [φ]]. Return <code>(tau, x_hat)</code>. With two outputs L is not unique; <code>cnt.place</code> gives one. The check starts the satellite at θ = φ = ${C13.init}° (your estimate starts wherever you start it) and runs the ±15° square wave (${C13.freq} Hz) with exact parameters: in the ${C13.settle} s before each of the first two switches, |θ − θ̂| and |φ − φ̂| must stay under ${C13.tolAngle}° and the rate errors under ${C13.tolRate}°/s, and |φ<sub>r</sub> − φ| just before the first switch must be under 0.1°.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(15, C13.freq), tEnd: 2 * tSw, init: { theta0: C13.init, phi0: C13.init } });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const ee = estErr(res, [[tSw - C13.settle, tSw - 0.05], [2 * tSw - C13.settle, 2 * tSw - 0.05]]);
            const e = errAt(res, tSw - 0.05);
            const msg = `Before the switches: |θ − θ̂| ≤ ${fmt(ee[0], 3)}°, |φ − φ̂| ≤ ${fmt(ee[1], 3)}°, rate errors ≤ ${fmt(Math.max(ee[2], ee[3]), 3)}°/s; error before the first switch ${fmt(e, 3)}°.`;
            return { ok: ee[0] < C13.tolAngle && ee[1] < C13.tolAngle && ee[2] < C13.tolRate && ee[3] < C13.tolRate && e < 0.1, msg };
          },
          solution: () => [{ code: SOL.ch13 }, { html: 'The repo\'s ctrlObserver.py (observer 10× faster than the controller, RK4 with the previous saturated τ) plus the C.12 anti-windup. <code>cnt.place</code> here is scipy\'s YT algorithm, as python-control uses, so L is the repo\'s.' }],
        }),
        {
          id: 'd', title: '(d) Plot the states and their estimates',
          html: 'Run your controller: the θ and φ plots show your θ̂, φ̂ dashed with the true angles, and the extra plot shows the estimation errors.',
        },
        {
          id: 'e', title: '(e) Add an input disturbance of 1.0 N·m',
          html: 'Runs your controller from (c) with d = 1 N·m and exact parameters. Your observer has no model of d, so x̂ is biased, and the integrator acts on φ̂. Passes when the run shows the bias (Chapter 14 removes it).',
          check: async () => {
            const code = WB.myCtrl.savedCode(ctx, 'C.13/c');
            if (!code) return { ok: false, msg: 'Write your controller in C.13(c) first.' };
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(15, C13.freq), tEnd: 2 * tSw, dist: 1 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const [bt, bp] = biasOf(res, 2 * tSw - 3, 2 * tSw);
            const e = WB.myCtrl.mean(res, 2 * tSw - 3, 2 * tSw, (k) => res.yAll[1][k] - res.r[k]) * R2D;
            const msg = `With d = 1 N·m, over the last 3 s: θ − θ̂ = ${fmt(bt, 3)}°, φ − φ̂ = ${fmt(bp, 3)}°, φ − φ_r = ${fmt(e, 3)}°.`;
            if (Math.max(Math.abs(bt), Math.abs(bp)) < 1e-3) return { ok: false, msg: `${msg} x̂ follows the measured angles exactly: is x̂ coming from an observer of the model?` };
            return { ok: true, msg };
          },
          solution: () => [{ html: 'The observer does not model d, so x̂ settles with a bias (where it shows depends on L), and with it the controller. The disturbance observer of C.14 removes that bias.' }],
        },
      ]);
    },
  });

  CH.ch14 = base('dobs', 14, 'Disturbance observers', 'pp. 254–259', {
    // Work mode simulates the student's C.14(b) controller (or, from (a), their C.13 one).
    implement: { feed: 'y', linear: false },
    defaults(sys) { const pr = sys.problems.ch14; return stateDefaults(pr, { obs: obsDefaults(pr), pD: pr.pD, dobs: true, xhat0: 0, antiwindup: 'none' }); },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries: obsSeries,
    extraPlot(ctx, res) {
      const S = ctx.S;
      const dh = Array.from(res.extras.dhat || []);
      const series = [{ label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 }];
      if (dh.length && dh.some(Number.isFinite)) series.unshift({ label: `d̂ (${ctx.S.mode === 'work' ? 'your estimate' : 'estimate'})`, y: dh, color: '--series-3', width: 2 });
      return { opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N·m]', unit: 'N·m' }, data: { series } };
    },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workBanner(parent, ctx, 'C.14(b)'); return; }
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'pp. 258–259 · Listing 14.6');
      segmented(sec, {
        label: 'Disturbance observer',
        options: [{ value: true, label: 'on' }, { value: false, label: 'off: C.13 observer (C.14a)' }],
        ...bind(ctx, 'dobs'),
      });
      awControl(sec, ctx);
      tuningSliders(sec, ctx, { pI: true });
      const ob = section(parent, 'Observer poles', 'p. 257');
      obsSliders(ob, ctx, { withD: true });
      gainsReadout(ob, ctx, ['K1', 'K2', 'K3', 'K4', 'ki']);
    },
    math(ctx) {
      const d = design(ctx.pModel, { ...ctx.st, obs: obsOf(ctx) }, 'dobs');
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(u + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0 \\text{ for constant } d' },
        ...obsCards(ctx, d, 'dobs'), ...polesCard(ctx, d, 'dobs'),
      ];
    },
    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch14;
      const noise = pr.sim.noise * M.DEG;   // display ° -> rad
      const tSw = 0.5 / pr.sim.frequency;
      PD().problemPanel(parent, ctx, pr, [
        {
          id: 'a', title: '(a) α = 0.2, an input disturbance of 1 N·m, and measurement noise',
          html: `The chapter starts with the plant off by ${lib().misText(pr.mismatch)}, d = ${pr.sim.dist} N·m and noise σ = ${fmt(noise, 3)} rad (${pr.sim.noise}°) on both angles. Run your C.13 controller here to see the bias it leaves in x̂.`,
          actions: [{ label: 'Run my C.13 controller', run: () => {
            const code = WB.myCtrl.savedCode(ctx, 'C.13/c');
            if (!code) return { ok: false, msg: 'Write your controller in C.13(c) first (Ch 13 tab).' };
            return WB.myCtrl.use(ctx, code, 'a');
          } }],
        },
        WB.myCtrl.part(ctx, {
          id: 'b', title: '(b) Add a disturbance observer, verify that the estimator\'s steady-state error is removed, and tune', seed: ['C.13/c'],
          html: `Return <code>(tau, x_hat)</code> (or <code>(tau, x_hat, d_hat)</code> to plot your d̂). The check runs the ±15° square wave (${pr.sim.frequency} Hz) with the plant off by ${lib().misText(pr.mismatch)}, d = ${pr.sim.dist} N·m and noise σ = ${fmt(noise, 3)} rad on both angles: over the last 3 s the mean of θ − θ̂ and of φ − φ̂ must be under ${C14.tolBias}°, and the mean |φ<sub>r</sub> − φ| over the 0.5 s before the first switch under 0.1°.`,
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(15, pr.sim.frequency), tEnd: 2 * tSw, dist: pr.sim.dist, mismatch: pr.mismatch, noises: [noise, noise], seed: 1 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = needXhat(res);
            if (nx) return nx;
            const [bt, bp] = biasOf(res, 2 * tSw - 3, 2 * tSw);
            const e = WB.myCtrl.mean(res, tSw - 0.55, tSw - 0.05, (k) => res.r[k] - res.yAll[1][k]) * R2D;
            const ok = Math.abs(bt) < C14.tolBias && Math.abs(bp) < C14.tolBias && Math.abs(e) < 0.1;
            return { ok, msg: `Mean over the last 3 s: θ − θ̂ = ${fmt(bt, 3)}°, φ − φ̂ = ${fmt(bp, 3)}°; mean error before the first switch ${fmt(e, 3)}°.` };
          },
          solution: () => [
            { code: SOL.ch14 },
            { html: `As the repo's ctrlDisturbanceObserver.py (the C.13 observer poles plus p<sub>d</sub> = ${pr.pD}, Listing 14.6, p. 258), plus anti-windup. d̂ also absorbs the torque the model gets wrong because of the parameter mismatch, so it settles near d but not exactly on it.` },
          ],
        }),
      ]);
    },
  });

  WB.studies.C.ss = { makeSS, design, obsDefaults, augI, augD };
})();
