// Study C, chapters 8-10 and Appendix P.6: successive loop closure. The inner
// loop puts PD on the body angle θ (plant 1/((Js+Jp)s²)); the outer loop puts PD
// (or PID) on the panel angle φ and outputs the body reference θ_r, designed with
// the inner loop replaced by its DC gain k_DCθ = 1 (C.8, pp. 130–132).
//
// Modes (as in study A):
//   work    - C.8(e, f) and C.10(c) are your own Python controllers (WB.myCtrl), which
//             drive the plots; the five gain sliders only place the s-plane poles (and
//             drive the C.9 and P.6 analyses). Anything that answers a problem stays hidden.
//   explore - gains come from (t_rθ, ζ_θ, M, ζ_φ); drag the inner- or outer-loop poles.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const { tex, texPole, fmt, fmtPole } = M;
  const PD = () => WB.pd;
  const lib = () => WB.studies.C.lib;
  const CH = WB.studies.C.chapters;
  const R2D = 180 / Math.PI, DEG = Math.PI / 180;

  // --------------------------------------------------------- controller --
  // One law for C.8 (ctrlPD.py), C.9, C.P.6 and C.10 (ctrlPID.py):
  //   θ_r = sat(k_Pφ e_φ + k_Iφ ∫e_φ − k_Dφ φ̇ [+ φ_r], θ_max)
  //   τ   = k_Pθ (θ_r − θ) − k_Dθ θ̇   (returned unsaturated: the simulation clips it at
  //         τ_max, as satelliteDynamics.update does, so the plot shows the true demand)
  // deriv 'state' uses the true θ̇, φ̇ (ctrlPD.py); 'dirty' differentiates the
  // measured angles (Eq. 10.4). antiwindup 'repo' is ctrlPID.py's
  // u_I += (T_s/k_I)(θ_r − θ_r,unsat).
  function makeCascade(ctx) {
    const { sys, pModel, S } = ctx;
    const st = ctx.st, g = ctx.gains;
    const Ts = S.sim.Ts, sigma = st.sigma ?? 0.05;
    const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    const thMax = (st.thetaMaxDeg ?? 30) * DEG;
    const dirty = st.deriv === 'dirty';
    let I = 0, ePrev = 0, thPrev = null, phPrev = null, thd = 0, phd = 0;
    return {
      update(r, x, yMeas) {
        const th = dirty ? yMeas[0] : x[0], ph = dirty ? yMeas[1] : x[1];
        if (thPrev === null) { thPrev = th; phPrev = ph; }
        const e = r - ph;
        I = I + (Ts / 2) * (e + ePrev);
        phd = dirty ? beta * phd + gamma * (ph - phPrev) : x[3];
        let thrU = g.kPphi * e + g.kIphi * I - g.kDphi * phd;
        if (st.ff) thrU = thrU + r;
        const thr = M.saturate(thrU, thMax);
        if (st.antiwindup === 'repo' && g.kIphi !== 0) I = I + Ts / g.kIphi * (thr - thrU);
        thd = dirty ? beta * thd + gamma * (th - thPrev) : x[2];
        const tauU = g.kPth * (thr - th) - g.kDth * thd;
        ePrev = e; phPrev = ph; thPrev = th;
        return { u: tauU, thetaR: thr, thetaRunsat: thrU, integrator: I, thdHat: thd, phdHat: phd };
      },
    };
  }

  // ------------------------------------------------- solution controllers --
  // C.8(e): ctrlPD.py with the C.8 spec (t_rθ = 1 s), φ_r fed forward (Fig. 8-20).
  // The C.8(f) solution adds the torque limit and a slower t_rθ.
  const SOL8 = `class Controller:
    def __init__(self):
        tr_th = 1.0          # C.8(b)
        zeta_th = 0.9
        M = 10.0             # C.8(d): t_r,phi = M t_r,theta
        zeta_phi = 0.9
        # inner loop on 1/((Js + Jp) s^2)
        J = P.Js + P.Jp
        wn_th = 0.5 * np.pi / (tr_th * np.sqrt(1 - zeta_th**2))
        self.kp_th = wn_th**2 * J
        self.kd_th = 2 * zeta_th * wn_th * J
        DC_th = 1.0          # C.8(c)
        # outer loop with the inner loop replaced by its DC gain
        wn_phi = 0.5 * np.pi / (M * tr_th * np.sqrt(1 - zeta_phi**2))
        AA = np.array([[P.k * DC_th, -P.b * DC_th * wn_phi**2],
                       [P.b * DC_th, P.k * DC_th - 2 * zeta_phi * wn_phi * P.b * DC_th]])
        bb = np.array([[-P.k + P.Jp * wn_phi**2],
                       [-P.b + 2 * P.Jp * zeta_phi * wn_phi]])
        gains = np.linalg.solve(AA, bb)
        self.kp_phi = gains[0, 0]
        self.kd_phi = gains[1, 0]

    def update(self, phi_r, x):
        theta = x[0, 0]
        phi = x[1, 0]
        thetadot = x[2, 0]
        phidot = x[3, 0]
        # outer loop: body reference, with phi_r fed forward (Fig. 8-20)
        theta_r = self.kp_phi * (phi_r - phi) - self.kd_phi * phidot + phi_r
        # inner loop: torque on the body
        tau = self.kp_th * (theta_r - theta) - self.kd_th * thetadot
        return tau
`;
  const SOL8F = SOL8.replace('tr_th = 1.0          # C.8(b)', 'tr_th = 1.9          # tuned: the 30 deg step just reaches tau_max')
    .replace('        return tau\n', '        return max(-P.tau_max, min(P.tau_max, tau))\n');
  // C.10(c): ctrlPID.py (Listing 10.4) as a Controller.
  const SOL10 = `class Controller:
    def __init__(self):
        # the book's C.10 tuning (Listing 10.4)
        tr_th = 0.4
        zeta_th = 0.9
        M = 15.0
        zeta_phi = 0.9
        self.ki_phi = 0.15
        self.theta_max = 30 * np.pi / 180      # limit on theta_r
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.Ts) / (2 * self.sigma + P.Ts)
        # inner loop
        J = P.Js + P.Jp
        wn_th = 2.2 / tr_th
        self.kp_th = wn_th**2 * J
        self.kd_th = 2 * zeta_th * wn_th * J
        # outer loop (inner loop -> DC gain 1)
        wn_phi = 2.2 / (M * tr_th)
        AA = np.array([[P.k, -P.b * wn_phi**2],
                       [P.b, P.k - 2 * zeta_phi * wn_phi * P.b]])
        bb = np.array([[-P.k + P.Jp * wn_phi**2],
                       [-P.b + 2 * P.Jp * zeta_phi * wn_phi]])
        gains = np.linalg.solve(AA, bb)
        self.kp_phi = gains[0, 0]
        self.kd_phi = gains[1, 0]
        # integrator and dirty derivatives
        self.integrator = 0.0
        self.error_prev = 0.0
        self.phi_dot = P.phidot0
        self.phi_prev = P.phi0
        self.theta_dot = P.thetadot0
        self.theta_prev = P.theta0

    def update(self, phi_r, y):
        theta = y[0, 0]
        phi = y[1, 0]
        # outer loop: PID on phi, derivative of the measured phi (Eq. 10.4)
        error = phi_r - phi
        self.integrator += P.Ts / 2 * (error + self.error_prev)
        self.phi_dot = self.beta * self.phi_dot + 2 / (2 * self.sigma + P.Ts) * (phi - self.phi_prev)
        theta_r_unsat = self.kp_phi * error + self.ki_phi * self.integrator - self.kd_phi * self.phi_dot
        theta_r = max(-self.theta_max, min(self.theta_max, theta_r_unsat))
        # anti-windup: unwind the integrator by what the limit cut off
        self.integrator += P.Ts / self.ki_phi * (theta_r - theta_r_unsat)
        # inner loop: PD on theta
        self.theta_dot = self.beta * self.theta_dot + 2 / (2 * self.sigma + P.Ts) * (theta - self.theta_prev)
        tau = self.kp_th * (theta_r - theta) - self.kd_th * self.theta_dot
        tau = max(-P.tau_max, min(P.tau_max, tau))
        self.error_prev = error
        self.phi_prev = phi
        self.theta_prev = theta
        return tau
`;

  // C.10(c): |φ_r − φ| before the first switch (the book only says "tune the integrator").
  const C10_TOL = 0.5;

  // The workbench's cascade with given gains on a check scenario (reference for
  // WB.myCtrl.matchCheck): true-state derivatives, no |θ_r| limit, with or without the
  // φ_r feedforward.
  function cascadeRef(ctx, sc, g, ff) {
    const rc = WB.myCtrl.refCtx(ctx, sc);
    rc.gains = g;
    rc.st = { ...ctx.st, ff, deriv: 'state', antiwindup: 'none', thetaMaxDeg: 1e9 };
    return WB.myCtrl.reference(ctx, sc, makeCascade(rc));
  }

  // Gains: work mode from the sliders, explore mode from the design knobs.
  function designOf(ctx, st = ctx.st) {
    return lib().slcDesign(ctx.pModel, { trTh: st.trTh, zetaTh: st.zetaTh, M: st.M, zetaPhi: st.zetaPhi, rule: st.rule, ki: st.kIx || 0 });
  }
  function gainsFor(ctx) {
    if (ctx.S.mode === 'work') return { ...ctx.st.w };
    return designOf(ctx);
  }

  // φ_r → φ for the outer design model (inner loop → k_DCθ), Eq. 8.14 / Fig. 6-10.
  function outerModelTf(p, g, ff) {
    const G = [p.b, p.k];                                   // (bs + k), times k_DCθ = 1
    const num = L.conv(G, [g.kPphi + (ff ? 1 : 0), g.kIphi || 0]);
    const den = L.polyAdd(L.conv([1, 0], [p.Jp, p.b, p.k]), L.conv(G, [g.kDphi, g.kPphi, g.kIphi || 0]));
    return T.tf(num, den);
  }
  // Dashed overlay: the design model's φ for the same reference (no saturation).
  function designOverlay(ctx, c) {
    const Ts = ctx.S.sim.Ts, N = Math.round(ctx.S.sim.tEnd / Ts) + 1;
    const filt = T.filter(outerModelTf(ctx.pModel, ctx.gains, ctx.st.ff), Ts);
    const F = () => new Float64Array(N);
    const res = { t: F(), yAll: [undefined, F()], rAll: [F()], uDemandAll: [F()], uAll: [F()], uAppliedAll: [F()], x: [] };
    const phi0 = (ctx.S.sim.init.phi0 ?? 0) * DEG;
    for (let k = 0; k < N; k++) {
      const t = k * Ts, r = c.reference(t);
      res.t[k] = t; res.rAll[0][k] = r;
      res.yAll[1][k] = phi0 + filt.step(r - phi0);
    }
    res.yMeasAll = res.yAll;
    return res;
  }

  // ------------------------------------------------------------- s-plane --
  const LEGEND = { obs: 'inner loop (design)', cl: 'outer loop (design, inner loop → k_DCθ)', zero: 'full 4-state closed loop', target: 'target (spec)', ol: 'open-loop pole' };

  function cascadeMarkers(ctx, { targets = null, sigma = null, drag = true } = {}) {
    const p = ctx.pModel, g = ctx.gains;
    const { A } = lib().ss(p);
    const explore = ctx.S.mode === 'explore' && drag;
    const inner = lib().innerPoles(p, g), outer = lib().outerPoles(p, g);
    const full = lib().fullLoopPoles(p, g, { sigma });
    const outerR = Math.max(...outer.map((q) => Math.hypot(q.re, q.im)), 0.05);
    const zoomOuter = ctx.st.zoom === 'outer';
    const nf = (q) => zoomOuter && Math.hypot(q.re, q.im) > 3 * outerR;
    // The open-loop poles answer C.5(b) / C.6: hidden in Work mode until one is solved.
    const mk = lib().showsOl(ctx) ? L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `open-loop pole ${i + 1}`, noFit: nf(q) })) : [];
    inner.forEach((q, i) => mk.push({ ...q, kind: 'obs', label: `inner loop pole ${i + 1} (design)`, dragId: explore ? 'in' : undefined, noFit: nf(q) }));
    outer.forEach((q, i) => mk.push({ ...q, kind: 'cl', label: `outer loop pole ${i + 1} (design)`, dragId: explore ? 'out' : undefined }));
    full.forEach((q, i) => mk.push({ ...q, kind: 'zero', label: `full-model closed-loop pole ${i + 1}`, noFit: nf(q) || Math.hypot(q.re, q.im) > 60 }));
    if (targets) targets.forEach((q) => mk.push({ ...q, kind: 'target', label: 'target pole (spec)', noFit: nf(q) }));
    return { markers: mk, legendNames: LEGEND, fitR: zoomOuter ? 2.2 * outerR : undefined };
  }

  // Dragging an inner pole sets (t_rθ, ζ_θ); an outer pole sets (M, ζ_φ).
  const invRule = WB.design.trFromWn;
  function onCascadeDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.002, re);
    const wn = Math.hypot(re, im);
    const zeta = Math.max(0.2, Math.min(st.rule === 'tp' ? 0.99 : 1.5, -re / wn));
    if (id === 'in') {
      st.zetaTh = zeta; st.trTh = Math.max(0.05, invRule(wn, zeta, st.rule));
    } else if (id === 'out') {
      st.zetaPhi = zeta; st.M = Math.max(1, Math.min(100, invRule(wn, zeta, st.rule) / st.trTh));
    }
    ctx.update();
  }

  // ------------------------------------------------------------ controls --
  const WORK_SPEC = { kPth: ['k<sub>P<sub>θ</sub></sub>', 0, 400], kDth: ['k<sub>D<sub>θ</sub></sub>', 0, 150], kPphi: ['k<sub>P<sub>φ</sub></sub>', -0.5, 5], kDphi: ['k<sub>D<sub>φ</sub></sub>', 0, 30], kIphi: ['k<sub>I<sub>φ</sub></sub>', 0, 2] };
  const workSliders = (parent, ctx, withI) => WB.ui.gainSliders(parent, ctx, WORK_SPEC, ['kPth', 'kDth', 'kPphi', 'kDphi', ...(withI ? ['kIphi'] : [])]);
  function designSliders(parent, ctx, { withI = false, withRule = true } = {}) {
    const st = ctx.st;
    slider(parent, { label: 't<sub>r<sub>θ</sub></sub>', unit: 's', min: 0.1, max: 5, step: 0.005, sig: 3, ...bind(ctx, 'trTh', () => st) });
    slider(parent, { label: 'ζ<sub>θ</sub>', min: 0.2, max: 0.99, step: 0.005, sig: 3, ...bind(ctx, 'zetaTh', () => st) });
    slider(parent, { label: 'M = t<sub>r<sub>φ</sub></sub>/t<sub>r<sub>θ</sub></sub>', min: 1, max: 40, step: 0.1, sig: 3, hint: 'bandwidth separation between the loops', ...bind(ctx, 'M', () => st) });
    slider(parent, { label: 'ζ<sub>φ</sub>', min: 0.2, max: 0.99, step: 0.005, sig: 3, ...bind(ctx, 'zetaPhi', () => st) });
    if (withRule) {
      segmented(parent, {
        label: 'ω<sub>n</sub> from t<sub>r</sub>',
        options: [{ value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))', title: 'C.8 solution' }, { value: '2.2', label: '2.2 / t<sub>r</sub>', title: 'Eq. 8.5' }],
        ...bind(ctx, 'rule', () => st),
      });
    }
    if (withI) slider(parent, { label: 'k<sub>I<sub>φ</sub></sub>', min: 0, max: 2, step: 0.001, sig: 3, ...bind(ctx, 'kIx', () => st) });
  }
  function readout(parent, ctx, keys) {
    const names = { kPth: 'kPθ', kDth: 'kDθ', kPphi: 'kPφ', kDphi: 'kDφ', kIphi: 'kIφ' };
    WB.ui.readout(parent, () => keys.map((k) => [names[k], ctx.gains[k]]));
  }
  function commonControls(parent, ctx, { ff = true, deriv = false, aw = false } = {}) {
    const st = ctx.st;
    if (ff) {
      segmented(parent, {
        label: 'Feedforward φ<sub>r</sub> into θ<sub>r</sub>',
        options: [{ value: true, label: 'on (Fig. 8-20 p. 133)' }, { value: false, label: 'off (Fig. 8-19)' }],
        ...bind(ctx, 'ff', () => st),
      });
    }
    slider(parent, { label: '|θ<sub>r</sub>| limit', unit: '°', min: 5, max: 360, step: 1, sig: 3, hint: 'saturation of the outer loop\'s output (θ_max in the repo)', ...bind(ctx, 'thetaMaxDeg', () => st) });
    if (deriv) {
      segmented(parent, {
        label: 'θ̇, φ̇ for the D terms',
        options: [{ value: 'dirty', label: 'dirty derivative of y' }, { value: 'state', label: 'true state' }],
        ...bind(ctx, 'deriv', () => st),
      });
      slider(parent, { label: 'σ', unit: 's', min: 0.005, max: 0.5, step: 0.001, sig: 3, hint: 'dirty-derivative bandwidth is 1/σ rad/s', ...bind(ctx, 'sigma', () => st), disabled: () => st.deriv !== 'dirty' });
    }
    if (aw) {
      segmented(parent, {
        label: 'Anti-windup',
        options: [{ value: 'repo', label: 'u<sub>I</sub> += (T<sub>s</sub>/k<sub>I</sub>)(θ<sub>r</sub> − θ<sub>r,unsat</sub>)', title: 'Listing 10.4' }, { value: 'none', label: 'none' }],
        ...bind(ctx, 'antiwindup', () => st),
      });
    }
    zoomControl(parent, ctx);
  }
  function zoomControl(parent, ctx) {
    segmented(parent, {
      label: 's-plane view',
      options: [{ value: 'all', label: 'all poles' }, { value: 'outer', label: 'zoom on outer loop' }],
      ...bind(ctx, 'zoom', () => ctx.st),
    });
  }

  // Work mode (C.8, C.10): the gain sliders only place the s-plane poles; the time
  // plots show the student's own controller, written in part `part`.
  function workControls(parent, ctx, { withI, targetKey, part }) {
    const sec = section(parent, 'Gains (s-plane)', 'p. 129 · C.8(a)');
    workSliders(sec, ctx, withI);
    sec.append(el('p', { class: 'muted small', text: 'These place the design-model × and the full-loop circles in the s-plane. They do not drive the simulation.' }));
    zoomControl(sec, ctx);
    targetToggle(sec, ctx, targetKey);
    WB.myCtrl.banner(section(parent, 'Your controller'), ctx, part);
  }

  // Loads C.8's loops into the Work-mode sliders, keeping the current k_Iφ. Used where
  // C.8's gains are the starting point (C.9, C.P.6). "Load my C.8 gains" copies the
  // Work-mode gains of the Ch 8 tab; the C.8 design itself (t_rθ = 1 s, M = 10,
  // ζ = 0.9) answers C.8(b) and (d), so that button appears once both are solved.
  function loadC8Button(parent, ctx) {
    const set = (g) => { Object.assign(ctx.st.w, { kPth: g.kPth, kDth: g.kDth, kPphi: g.kPphi, kDphi: g.kDphi }); ctx.update(); };
    const row = el('div', { class: 'btn-row' }, el('button', {
      type: 'button', class: 'btn btn-quiet', text: 'Load my C.8 gains', title: 'Copy the Work-mode gains from the Ch 8 tab',
      onclick: () => { const c8 = ctx.S.ch.ch8; if (c8 && c8.w) set(c8.w); },
    }));
    if (lib().shows(ctx, 'C.8/b', 'C.8/d')) {
      row.append(el('button', {
        type: 'button', class: 'btn btn-quiet', text: 'Load the C.8 gains',
        onclick: () => {
          const pr = ctx.sys.problems.ch8;
          set(designOf(ctx, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule }));
        },
      }));
    }
    parent.append(row);
  }

  // Work mode: the spec's target poles give away ω_n (and with it the gains), so the
  // toggle appears only once part `key` (the inner-loop gains) is solved.
  function targetToggle(parent, ctx, key) {
    const seg = segmented(parent, {
      label: 'Target poles from the spec (s-plane rings)',
      options: [{ value: false, label: 'hidden' }, { value: true, label: 'shown' }],
      get: () => !!ctx.st.showTargets, set: (v) => { ctx.st.showTargets = v; ctx.update(); },
    });
    const note = el('p', { class: 'muted small', text: `The spec's target poles can be shown here once ${key.replace('/', '(')}) is solved.` });
    parent.append(note);
    WB.ui.addRefresher(() => { const ok = lib().shows(ctx, key); seg.row.hidden = !ok; note.hidden = ok; });
  }

  // Bandwidth-separation readout (shown in both modes; it is not an answer).
  function separationBox(parent, ctx, sigmaFn) {
    const box = el('div', { class: 'metrics' });
    parent.append(box);
    WB.ui.addRefresher(() => {
      const p = ctx.pModel, g = ctx.gains;
      const inner = lib().innerPoles(p, g), outer = lib().outerPoles(p, g);
      const wi = Math.min(...inner.map((q) => Math.hypot(q.re, q.im))), wo = Math.max(...outer.map((q) => Math.hypot(q.re, q.im)));
      const full = lib().fullLoopPoles(p, g, { sigma: sigmaFn ? sigmaFn() : null });
      const stable = full.every((q) => q.re < 0);
      const row = (l, v, ok) => WB.ui.metric(l, v, ok === undefined ? null : { ok, text: ok ? 'stable' : 'unstable' });
      box.replaceChildren(
        row('|p_inner| / |p_outer| (slowest inner ÷ fastest outer)', isFinite(wi / wo) ? `${fmt(wi / wo, 3)}×` : '—'),
        row('full 4-state loop', full.length ? `slowest pole ${fmtPole(full.reduce((a, q) => (q.re > a.re ? q : a)))}` : '—', stable),
      );
    });
  }

  // ------------------------------------------------------------ math cards --
  // Theory lines are the general book forms. Cards with the satellite's own
  // results carry `answers` (the part that derives them) and stay locked in Work
  // mode until that part is solved.
  function generalCards(ctx) {
    const st = ctx.st;
    const wTex = st.rule === 'tp' ? '\\omega_n = \\frac{1}{2}\\frac{\\pi}{t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}';
    return [
      { title: 'PD with the derivative on the output', page: 'p. 101 · Eq. 7.5, Fig. 7-2',
        theory: 'P(s) = \\frac{b_0}{s^2 + a_1s + a_0},\\quad u = k_P(r - y) - k_D\\dot y \\;\\Rightarrow\\; \\frac{Y}{R} = \\frac{b_0k_P}{s^2 + (a_1 + b_0k_D)s + (a_0 + b_0k_P)}' },
      { title: 'Spec → desired characteristic polynomial', page: 'p. 110 · Eq. 8.2, p. 113',
        theory: `${wTex},\\quad \\Delta^d_{cl}(s) = s^2 + 2\\zeta\\omega_n s + \\omega_n^2 \\;\\text{(match coefficients with } \\Delta_{cl})` },
      { title: 'DC gain', page: 'p. 132',
        theory: 'k_{DC} = \\lim_{s\\to0} T(s) \\quad (T = \\text{closed-loop transfer function})' },
    ];
  }
  function innerCards(ctx, g) {
    const p = ctx.pModel, J = p.Js + p.Jp;
    const pin = lib().innerPoles(p, g);
    return [
      { title: 'Inner-loop plant (C.5)', page: 'p. 79 · Fig. 5-3', answers: 'C.5/d',
        theory: 'P_{in}(s) = \\frac{\\Theta(s)}{\\tau(s)} \\approx \\frac{1}{(J_s + J_p)s^2}' },
      { title: 'Inner loop (body angle)', page: 'p. 130 · Fig. 8-18', answers: 'C.8/b1',
        theory: '\\frac{\\Theta}{\\Theta_r} = \\frac{\\frac{k_{P_\\theta}}{J_s + J_p}}{s^2 + \\frac{k_{D_\\theta}}{J_s + J_p}s + \\frac{k_{P_\\theta}}{J_s + J_p}},\\quad k_{DC_\\theta} = 1',
        numbers: `\\Delta_{in}(s) = s^2 + ${tex(g.kDth / J)}\\,s + ${tex(g.kPth / J)},\\quad p_{in} = ${pin.map((q) => texPole(q)).join(',\\;')}` },
    ];
  }
  function outerCard(ctx, g, { withI = false } = {}) {
    const p = ctx.pModel;
    const poly = lib().outerCharPoly(p, g);
    return {
      title: withI ? 'Outer loop with PID (inner loop → k_DCθ)' : 'Outer loop (inner loop → k_DCθ)', page: withI ? 'p. 472 · Fig. 6-10' : 'p. 131 · Fig. 8-19, Eq. 8.14',
      answers: withI ? 'C.P.6/a' : 'C.8/d1',
      theory: (withI
        ? '\\theta_r = k_{DC_\\theta}\\Big[\\big(k_P + \\tfrac{k_I}{s}\\big)(\\Phi_r - \\Phi) - k_Ds\\Phi\\Big],\\quad \\Phi = \\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}\\,\\theta_r'
        : '\\theta_r = k_{DC_\\theta}\\big[k_P(\\Phi_r - \\Phi) - k_Ds\\Phi\\big],\\quad \\Phi = \\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}\\,\\theta_r')
        + ',\\quad ' + (withI
          ? '\\Delta_{out} = (J_p + bk_{DC}k_D)s^3 + (b + bk_{DC}k_P + kk_{DC}k_D)s^2\\quad + (k + kk_{DC}k_P + bk_{DC}k_I)s + kk_{DC}k_I'
          : '\\Delta_{out}(s)\\,\\Phi = (bk_{DC}k_Ps + kk_{DC}k_P)\\,\\Phi_r,\\quad \\Delta_{out} = (J_p + bk_{DC}k_D)s^2\\quad + (b + bk_{DC}k_P + kk_{DC}k_D)s + (k + kk_{DC}k_P)'),
      numbers: `\\Delta_{out}(s) = ${T.polyTex(poly)},\\quad p_{out} = ${L.roots(poly).map((q) => texPole(q)).join(',\\;')}`,
    };
  }
  // ans: the parts these gains answer (C.8's spec in Ch 8, the listing's tuning in Ch 10).
  function designCards(ctx, d, ans = { inner: 'C.8/b', outer: 'C.8/d' }) {
    return [
      { title: 'Inner-loop gains', page: 'p. 131', answers: ans.inner,
        theory: 'k_{P_\\theta} = \\omega_{n_\\theta}^2(J_s + J_p),\\quad k_{D_\\theta} = 2\\zeta_\\theta\\omega_{n_\\theta}(J_s + J_p)',
        numbers: `\\omega_{n_\\theta} = ${tex(d.wnTh)},\\quad k_{P_\\theta} = ${tex(d.kPth)},\\quad k_{D_\\theta} = ${tex(d.kDth)}` },
      { title: 'Outer-loop gains', page: 'p. 132', answers: ans.outer,
        theory: 't_{r_\\phi} = M t_{r_\\theta},\\quad \\begin{bmatrix}kk_{DC} & -bk_{DC}\\omega_{n_\\phi}^2\\\\ bk_{DC} & kk_{DC} - 2bk_{DC}\\zeta_\\phi\\omega_{n_\\phi}\\end{bmatrix}\\begin{bmatrix}k_{P_\\phi}\\\\ k_{D_\\phi}\\end{bmatrix} = \\begin{bmatrix}-k + J_p\\omega_{n_\\phi}^2\\\\ -b + 2J_p\\zeta_\\phi\\omega_{n_\\phi}\\end{bmatrix}',
        numbers: `t_{r_\\phi} = ${tex(d.trPhi)},\\; \\omega_{n_\\phi} = ${tex(d.wnPhi)}\\quad \\Rightarrow k_{P_\\phi} = ${tex(d.kPphi)},\\; k_{D_\\phi} = ${tex(d.kDphi)}` },
    ];
  }
  function dcCard(ctx, g) {
    const p = ctx.pModel;
    const kdc = p.k * g.kPphi / (p.k + p.k * g.kPphi);
    return {
      title: 'Outer-loop DC gain and the feedforward', page: 'p. 132, p. 133 · Fig. 8-20', answers: 'C.8/d',
      theory: 'k_{DC_\\phi} = \\frac{kk_{DC_\\theta}k_{P_\\phi}}{k + kk_{DC_\\theta}k_{P_\\phi}} < 1,\\quad \\theta_r = k_{P_\\phi}(\\phi_r - \\phi) - k_{D_\\phi}\\dot\\phi + \\phi_r \\;(\\text{Fig. 8-20})',
      numbers: `k_{DC_\\phi} = ${tex(kdc)}\\;(\\text{your gains})`,
      note: 'The feedforward term is the book\'s fix for an outer DC gain below one.',
    };
  }

  // Python parts: the loop transfer functions of C.8 as functions of the gains.
  const PYARGS = {
    s: { label: 's', complex: true, re: [-3, 1], im: [0.3, 6] },
    kPin: { label: 'kP', lo: 10, hi: 200 }, kDin: { label: 'kD', lo: 5, hi: 80 },
    kPout: { label: 'kP', lo: 0.1, hi: 3 }, kDout: { label: 'kD', lo: 1, hi: 15 }, kIout: { label: 'kI', lo: 0.005, hi: 0.5 },
  };
  const cx = WB.py.cx;
  const innerDen = (p, s, kP, kD) => { const J = p.Js + p.Jp; return cx.poly([1, kD / J, kP / J], s); };
  const outerDen = (p, s, kP, kD) => cx.poly([p.Jp + p.b * kD, p.b + p.b * kP + p.k * kD, p.k + p.k * kP], s);

  // ------------------------------------------------------------- C.8 --
  CH.ch8 = {
    id: 'ch8', num: 8, tab: 'Ch 8', title: 'Successive loop closure (PD)', pages: 'pp. 129–134',
    controller: (ctx) => makeCascade(ctx),
    // Work mode simulates the student's C.8(e)/(f) controller, which gets the state (as
    // ctrlPD.py); no linear overlay (it would need gains the student doesn't set).
    implement: { feed: 'state', linear: false },
    defaults(sys) {
      const pr = sys.problems.ch8;
      return {
        ff: true, deriv: 'state', antiwindup: 'none', thetaMaxDeg: pr.thetaMaxDeg, zoom: 'all',
        trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, kIx: 0,
        w: { kPth: 40, kDth: 20, kPphi: 0.5, kDphi: 4, kIphi: 0 },    // work-mode start (not the answer)
      };
    },
    simDefaults(sys) { return sys.problems.ch8.sim; },
    gains: gainsFor,
    targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.M * ctx.st.trTh } : {}; },
    linearLabel: 'outer design model (inner loop → its DC gain)',   // k_DCθ = 1 answers C.8(c)
    linearSim: designOverlay,
    outputSeries(ctx, res, sc, oi) {
      if (oi !== 0 || !res.extras.thetaR) return [];
      return [{ label: 'θ_r (outer-loop output)', y: sc(res.extras.thetaR), color: '--series-2', dash: [4, 3], width: 1.5 }];
    },

    // Problem's spec design (always the book's numbers, independent of the knobs).
    spec(ctx) { const pr = ctx.sys.problems.ch8; return designOf(ctx, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule }); },

    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') {
        workControls(parent, ctx, { withI: false, targetKey: 'C.8/b', part: 'C.8(e) or (f)' });
      } else {
        const sec = section(parent, 'PD inner loop, PD outer loop', 'p. 129 · C.8(a)');
        commonControls(sec, ctx, { ff: true });
        const des = section(parent, 'Design knobs', 'p. 131–132');
        designSliders(des, ctx);
        des.append(el('p', { class: 'muted small', text: 'Drag an inner-loop pole (green ×) to set t_rθ and ζ_θ, or an outer-loop pole (blue ×) to set M and ζ_φ.' }));
        readout(des, ctx, ['kPth', 'kDth', 'kPphi', 'kDphi']);
      }
      const sep = section(parent, 'Bandwidth separation', 'p. 129');
      separationBox(sep, ctx);
      sep.append(el('p', { class: 'muted small', text: 'Circles are the poles of the real closed loop (four states, both loops together). When the loops are well separated they sit on the design ×s. Lower M to watch them part.' }));
    },

    splane(ctx) {
      // Inner targets once C.8(b) is solved, outer targets once C.8(d) is.
      const s = this.spec(ctx), on = ctx.S.mode === 'work' && ctx.st.showTargets;
      const tg = on ? [...(lib().shows(ctx, 'C.8/b') ? lib().innerPoles(ctx.pModel, s) : []), ...(lib().shows(ctx, 'C.8/d') ? lib().outerPoles(ctx.pModel, s) : [])] : null;
      return cascadeMarkers(ctx, { targets: tg });
    },
    onPoleDrag: onCascadeDrag,

    math(ctx) {
      const g = ctx.gains;
      const d = ctx.S.mode === 'work' ? this.spec(ctx) : designOf(ctx);
      return [...generalCards(ctx), ...innerCards(ctx, g), ...designCards(ctx, d), outerCard(ctx, g), dcCard(ctx, g),
        { title: 'Why successive loop closure works', page: 'p. 129, p. 130',
          theory: '\\text{inner loop much faster than outer:}\\; \\frac{\\Theta}{\\Theta_r} \\approx k_{DC_\\theta} \\text{ over the outer loop\'s bandwidth},\\quad t_{r_\\phi} = M\\,t_{r_\\theta},\\; M \\approx 5\\text{–}10',
          note: 'The book replaces the inner loop by its DC gain. The circles in the s-plane show what the full model actually does with these gains.' }];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch8;
      const s = () => this.spec(ctx);
      // Peak demanded |τ| for a satStepDeg step from rest, designed from trTh (M = 10).
      const TAU_MAX = ctx.sys.params.find((q) => q.key === 'tau_max').value;   // the problem's 5 N·m
      // C.8(e) check step: small enough that the feedforward never drives θ_r near
      // ctrlPD.py's 30° limit, with the nominal or the second parameter set.
      const E_STEP = 5;
      const peakFor = (trTh) => {
        const p = ctx.pModel;
        const g = lib().slcDesign(p, { trTh, zetaTh: prob.zetaTh, M: prob.M, zetaPhi: prob.zetaPhi, rule: prob.rule });
        const fake = { ...ctx, gains: g, st: { ...ctx.st, ff: true, deriv: 'state', antiwindup: 'none', thetaMaxDeg: prob.thetaMaxDeg } };
        const out = WB.sim.simulate({
          plant: { f: (x, u) => ctx.sys.f(x, u, p), h: ctx.sys.h, uLimit: TAU_MAX },
          controller: makeCascade(fake),
          reference: WB.sim.makeReference({ type: 'step', amplitude: prob.satStepDeg * DEG, tStep: 0 }),
          disturbance: () => [0], x0: [0, 0, 0, 0], Ts: ctx.S.sim.Ts, tEnd: Math.min(30, 3 * prob.M * trTh),
        });
        let peak = 0;
        for (const u of out.uDemand) peak = Math.max(peak, Math.abs(u));
        return peak / TAU_MAX;
      };
      // Each part's "Use my gains" sets the gains it asks for (a part's action sees only its own inputs).
      const useInner = WB.design.useGains(ctx, ['kPth', 'kDth'], { msg: 'Enter kPθ and kDθ first.' });
      const useOuter = WB.design.useGains(ctx, ['kPphi', 'kDphi'], { extra: { kIphi: 0 }, msg: 'Enter kPφ and kDφ first.' });
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Block diagram',
          html: 'On paper: draw the successive-loop-closure block diagram with PD control in both loops, φ<sub>r</sub> → outer controller → θ<sub>r</sub> → inner controller → τ → satellite. Use the plant models from C.5 and put the derivative on the measured angle, as in Fig. 7-2. Then go on to (b).',
          done: 'I\'ve drawn it: next',
          solution: () => [{ html: 'Book: Fig. 8-18 (inner loop, p. 130) and Fig. 8-19 (outer loop with the inner loop replaced by its DC gain, p. 131).' }],
        },
        {
          id: 'b1', title: '(b) Inner loop: closed-loop transfer function',
          html: 'Write Θ(s)/Θ<sub>r</sub>(s) and the closed-loop characteristic polynomial in terms of k<sub>P<sub>θ</sub></sub>, k<sub>D<sub>θ</sub></sub>; the check calls them at complex s. Any nonzero multiple of Δ<sub>cl</sub> is accepted.',
          code: {
            template: 'def inner_cl(s, kP, kD):\n    # Theta(s)/Theta_r(s)\n    return ...\n\ndef inner_char_poly(s, kP, kD):\n    # Delta_cl(s)\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { s: PYARGS.s, kP: PYARGS.kPin, kD: PYARGS.kDin },
              items: [
                { fn: 'inner_cl', args: ['s', 'kP', 'kD'], truth: (p, a) => cx.div(a.kP / (p.Js + p.Jp), innerDen(p, a.s, a.kP, a.kD)) },
                { fn: 'inner_char_poly', args: ['s', 'kP', 'kD'], compare: 'scale', truth: (p, a) => innerDen(p, a.s, a.kP, a.kD) },
              ],
            }, code),
          },
          solution: () => [
            { tex: '\\frac{\\Theta}{\\Theta_r} = \\frac{\\frac{k_{P_\\theta}}{J_s + J_p}}{s^2 + \\frac{k_{D_\\theta}}{J_s + J_p}s + \\frac{k_{P_\\theta}}{J_s + J_p}}' },
            { code: 'J = P.Js + P.Jp\n\ndef inner_char_poly(s, kP, kD):\n    return s**2 + kD / J * s + kP / J\n\ndef inner_cl(s, kP, kD):\n    return kP / J / inner_char_poly(s, kP, kD)' },
            { html: 'Book: p. 130 (Fig. 8-18).' },
          ],
        },
        {
          id: 'b', title: `(b) Inner loop: t<sub>r<sub>θ</sub></sub> = ${prob.trTh} s, ζ<sub>θ</sub> = ${prob.zetaTh}`,
          html: 'Use ω<sub>n</sub> = π/(2 t<sub>r</sub>√(1−ζ²)) as the C.8 solution does (p. 131).',
          inputs: { wn: 'ω<sub>n<sub>θ</sub></sub>', kPth: 'k<sub>P<sub>θ</sub></sub>', kDth: 'k<sub>D<sub>θ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { wn: s().wnTh, kPth: s().kPth, kDth: s().kDth }, { wn: 'ωnθ', kPth: 'kPθ', kDth: 'kDθ' }),
          actions: [useInner],
          solution: () => [
            { tex: `\\omega_{n_\\theta} = \\frac{\\pi}{2(${prob.trTh})\\sqrt{1 - ${prob.zetaTh}^2}} = ${tex(s().wnTh)},\\quad k_{P_\\theta} = \\omega_{n_\\theta}^2(J_s+J_p) = ${tex(s().kPth)},\\quad k_{D_\\theta} = ${tex(s().kDth)}` },
            { html: 'Book: 77.9 and 38.9 (p. 131).' },
          ],
        },
        {
          id: 'c', title: '(c) DC gain of the inner loop',
          inputs: { kdc: 'k<sub>DC<sub>θ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { kdc: 1 }, { kdc: 'kDCθ' }),
          solution: () => [{ tex: '\\lim_{s\\to0}\\frac{k_{P_\\theta}/(J_s+J_p)}{s^2 + \\frac{k_{D_\\theta}}{J_s+J_p}s + \\frac{k_{P_\\theta}}{J_s+J_p}} = 1' }, { html: 'The plant has two free integrators, so the inner loop tracks constant θ<sub>r</sub> exactly.' }],
        },
        {
          id: 'd1', title: '(d) Outer loop: closed-loop transfer function',
          html: 'Replace the inner loop by its DC gain from (c), with no feedforward. Write Φ(s)/Φ<sub>r</sub>(s) and the closed-loop characteristic polynomial in terms of k<sub>P<sub>φ</sub></sub>, k<sub>D<sub>φ</sub></sub>. Any nonzero multiple of Δ<sub>cl</sub> is accepted.',
          code: {
            template: 'def outer_cl(s, kP, kD):\n    # Phi(s)/Phi_r(s)\n    return ...\n\ndef outer_char_poly(s, kP, kD):\n    # Delta_cl(s)\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { s: PYARGS.s, kP: PYARGS.kPout, kD: PYARGS.kDout },
              items: [
                { fn: 'outer_cl', args: ['s', 'kP', 'kD'], truth: (p, a) => cx.div(cx.poly([p.b * a.kP, p.k * a.kP], a.s), outerDen(p, a.s, a.kP, a.kD)) },
                { fn: 'outer_char_poly', args: ['s', 'kP', 'kD'], compare: 'scale', truth: (p, a) => outerDen(p, a.s, a.kP, a.kD) },
              ],
            }, code),
          },
          solution: () => [
            { tex: '\\Phi = \\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}\\big[k_{DC_\\theta}k_{P_\\phi}(\\Phi_r - \\Phi) - k_{DC_\\theta}k_{D_\\phi}s\\Phi\\big]' },
            { tex: '\\big[(J_p + bk_{DC_\\theta}k_{D_\\phi})s^2 + (b + bk_{DC_\\theta}k_{P_\\phi} + kk_{DC_\\theta}k_{D_\\phi})s + (k + kk_{DC_\\theta}k_{P_\\phi})\\big]\\Phi = \\big[bk_{DC_\\theta}k_{P_\\phi}s + kk_{DC_\\theta}k_{P_\\phi}\\big]\\Phi_r' },
            { code: 'def outer_char_poly(s, kP, kD):\n    Jp, b, k = P.Jp, P.b, P.k\n    return ((Jp + b * kD) * s**2\n            + (b + b * kP + k * kD) * s\n            + (k + k * kP))\n\ndef outer_cl(s, kP, kD):\n    num = P.b * kP * s + P.k * kP\n    return num / outer_char_poly(s, kP, kD)' },
            { html: 'Book: Eq. 8.14 (p. 131), with k<sub>DC<sub>θ</sub></sub> = 1.' },
          ],
        },
        {
          id: 'd', title: `(d) Outer loop: t<sub>r<sub>φ</sub></sub> = ${prob.M} t<sub>r<sub>θ</sub></sub>, ζ<sub>φ</sub> = ${prob.zetaPhi}`,
          inputs: { kPphi: 'k<sub>P<sub>φ</sub></sub>', kDphi: 'k<sub>D<sub>φ</sub></sub>', kdc: 'k<sub>DC<sub>φ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { kPphi: s().kPphi, kDphi: s().kDphi, kdc: s().kDCphi }, { kPphi: 'kPφ', kDphi: 'kDφ', kdc: 'kDCφ' }),
          actions: [useOuter],
          solution: () => [
            { tex: `\\omega_{n_\\phi} = ${tex(s().wnPhi)},\\quad \\begin{bmatrix}k_{P_\\phi}\\\\ k_{D_\\phi}\\end{bmatrix} = ${M.texMat(s().AA)}^{-1}${M.texMat(s().bb)} = \\begin{bmatrix}${tex(s().kPphi)}\\\\ ${tex(s().kDphi)}\\end{bmatrix}` },
            { tex: `k_{DC_\\phi} = \\frac{k\\,k_{P_\\phi}}{k + k\\,k_{P_\\phi}} = ${tex(s().kDCphi)}` },
            { html: 'Book: (0.834, 8.253) and 0.455 (p. 132). The k<sub>D<sub>φ</sub></sub> printed there differs from what its own formula gives; see ISSUES.md.' },
          ],
        },
        WB.myCtrl.part(ctx, {
          id: 'e', title: '(e) Implement the successive-loop-closure design and simulate the 15° square wave',
          html: `Write the controller with the gains from (b) and (d). <code>update</code> gets φ<sub>r</sub> and the state x = (θ, φ, θ̇, φ̇), as in the book's C.8 code. <em>Run my controller</em> drives the time plots. Saturation only enters in (f): with t<sub>r<sub>θ</sub></sub> = 1 s the square wave demands several times τ<sub>max</sub>, so raise τ<sub>max</sub> in the left panel (up to 100) to see the design run unsaturated. The check simulates a ${E_STEP}° step on φ<sub>r</sub> with τ<sub>max</sub> raised out of the way, with the nominal and with other parameters, and compares θ(t) and φ(t) with the design (within 3% of the step).`,
          check: (code) => {
            const cases = WB.myCtrl.paramCases(ctx).map((pc) => {
              const params = { ...pc.params, tau_max: 1000 };
              const sc = WB.myCtrl.scenario(ctx, { params, ref: { type: 'step', amplitude: E_STEP, tStep: 0 }, tEnd: 60 });
              const g = lib().slcDesign(params, { trTh: prob.trTh, zetaTh: prob.zetaTh, M: prob.M, zetaPhi: prob.zetaPhi, rule: prob.rule });
              // with or without the φ_r feedforward of Fig. 8-20
              return { sc, label: pc.label, refs: [() => cascadeRef(ctx, sc, g, true), () => cascadeRef(ctx, sc, g, false)] };
            });
            return WB.myCtrl.matchCheck(ctx, code, cases, { tol: 0.03 * E_STEP * DEG, outputs: [0, 1] });
          },
          solution: () => [
            { code: SOL8 },
            { html: 'As the repo\'s ctrlPD.py with t<sub>r<sub>θ</sub></sub> = 1 s: PD on both loops with the derivative on the measured angle, and φ<sub>r</sub> fed forward into θ<sub>r</sub> (Fig. 8-20) so the outer DC gain is one. Without the feedforward (Fig. 8-19) the check passes too. ctrlPD.py also limits |θ<sub>r</sub>| to 30°, which this step never reaches.' },
          ],
        }),
        WB.myCtrl.part(ctx, {
          id: 'f', title: `(f) Saturate τ at τ<sub>max</sub> = 5 N·m; tune the outer rise time for the fastest ${prob.satStepDeg}° step without saturation`, seed: 'C.8/e',
          html: `Your controller must keep its output within ±τ<sub>max</sub> (<code>P.tau_max</code>). Keep ζ = ${prob.zetaTh} in both loops. The check gives your controller a large error (its output must stay within ±τ<sub>max</sub>), then simulates a ${prob.satStepDeg}° step on φ<sub>r</sub> from rest: your peak |τ| must reach 95% of τ<sub>max</sub>, and τ may sit at the limit for at most 2 samples.`,
          check: async (code) => {
            const tmax = TAU_MAX;
            const pr = await WB.myCtrl.probe(ctx, code, [[Math.PI, [0, 0, 0, 0]]], { params: { ...ctx.pModel, tau_max: tmax } });
            if (pr.ok === false) return pr;
            const u0 = pr.u[0][0];
            if (Math.abs(u0) > tmax * (1 + 1e-9)) return { ok: false, msg: `For φ_r = 180° from rest your controller returns τ = ${fmt(u0, 4)} N·m. Saturate its output at ±τ_max (P.tau_max).` };
            const sc = WB.myCtrl.scenario(ctx, { params: { ...ctx.pModel, tau_max: tmax }, ref: { type: 'step', amplitude: prob.satStepDeg, tStep: 0 }, tEnd: 40 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            let peak = 0, nAt = 0;
            for (const u of res.uDemand) { peak = Math.max(peak, Math.abs(u)); if (Math.abs(u) >= tmax * (1 - 1e-6)) nAt++; }
            const pct = (100 * peak / tmax).toFixed(1);
            if (nAt > 2) return { ok: false, msg: `τ sits at the limit for ${nAt} samples, so the input saturates. Slow it down.` };
            if (peak < 0.95 * tmax) return { ok: false, msg: `Peak |τ| is ${pct}% of τmax. You can go faster.` };
            return { ok: true, msg: `Peak |τ| is ${pct}% of τmax${nAt ? `, at the limit for ${nAt} sample${nAt > 1 ? 's' : ''}` : ''}.` };
          },
          solution: () => {
            const p = ctx.pModel, thMax = prob.thetaMaxDeg * DEG, step = prob.satStepDeg * DEG;
            // Fastest t_rθ whose peak demand is exactly τ_max (bisection on the simulated peak).
            let lo = 0.3, hi = 20;
            for (let i = 0; i < 30; i++) { const mid = Math.sqrt(lo * hi); if (peakFor(mid) > 1) lo = mid; else hi = mid; }
            const g = lib().slcDesign(p, { trTh: hi, zetaTh: prob.zetaTh, M: prob.M, zetaPhi: prob.zetaPhi, rule: prob.rule });
            const thr0 = Math.min(thMax, Math.abs(1 + g.kPphi) * step);
            return [
              { html: 'The largest demand is the first sample after the step: θ = φ = θ̇ = φ̇ = 0, so θ<sub>r</sub>(0) = sat((1 + k<sub>P<sub>φ</sub></sub>)·30°, 30°) and τ(0) = k<sub>P<sub>θ</sub></sub>θ<sub>r</sub>(0) ≤ τ<sub>max</sub> (Eq. 8.8). Both gains depend on t<sub>rθ</sub> (k<sub>P<sub>φ</sub></sub> turns negative for a slow outer loop), so solve numerically:' },
              { tex: `t_{r_\\theta} = ${tex(hi)}\\,\\text{s}:\\quad k_{P_\\theta} = ${tex(g.kPth)},\\; k_{P_\\phi} = ${tex(g.kPphi)},\\; \\theta_r(0) = ${tex(thr0 / DEG)}^\\circ,\\; \\tau(0) = ${tex(g.kPth * thr0)}\\,\\text{N·m}` },
              { code: SOL8F },
              { html: `This tunes t<sub>r<sub>θ</sub></sub> with t<sub>r<sub>φ</sub></sub> = ${prob.M} t<sub>r<sub>θ</sub></sub>, as the repo does; the problem's knob, the outer rise time alone, works too (it needs a much slower outer loop). Without the feedforward the fastest t<sub>r<sub>θ</sub></sub> is about 2.49 s. The repo\'s PD controller uses t<sub>r<sub>θ</sub></sub> = ${prob.repoTr} s ("tuned to not saturate the input"); with it the step demands ${(100 * peakFor(prob.repoTr)).toFixed(0)}% of τ<sub>max</sub>.` },
            ];
          },
        }),
      ]);
    },
  };

  // ------------------------------------------------------------- C.9 --
  // Reference shapes on φ_r: step, ramp, parabola (amplitude = size, slope, coefficient).

  CH.ch9 = {
    id: 'ch9', num: 9, tab: 'Ch 9', title: 'System type & integrators', pages: 'pp. 150–153',
    controller: (ctx) => makeCascade(ctx),
    reference: (ctx, base) => WB.pid.shapedReference(ctx, base, DEG, (ctx.S.sim.init.phi0 ?? 0) * DEG),
    defaults(sys) {
      const pr = sys.problems.ch8;
      return {
        ff: false, deriv: 'state', antiwindup: 'none', thetaMaxDeg: 360, zoom: 'all', input: 'step',
        trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, kIx: 0,
        w: { kPth: 40, kDth: 20, kPphi: 0.5, kDphi: 4, kIphi: 0 },   // placeholder; 'Load the C.8 gains' fills in the given design
      };
    },
    simDefaults(sys) { return sys.problems.ch9.sim; },
    gains: gainsFor,

    analysis(ctx) {
      const p = ctx.pModel, g = ctx.gains, J = p.Js + p.Jp;
      const hasI = g.kIphi > 0;
      return {
        J, hasI,
        inner: { type: 2, parab: J / g.kPth, dType: 0, dStep: 1 / g.kPth },
        outer: hasI ? { type: 1, step: 0, ramp: 1 / g.kIphi, dType: 1, dStep: 0, dRamp: 1 / g.kIphi }
          : { type: 0, step: 1 / (1 + g.kPphi), ramp: Infinity, dType: 0, dStep: 1 / (1 + g.kPphi) },
      };
    },

    // Predicted φ_r − φ at steady state for the current input, d and feedforward.
    predicted(ctx) {
      const a = this.analysis(ctx), g = ctx.gains, S = ctx.S;
      const A = S.sim.amplitude * DEG, d = S.sim.dist;
      if (ctx.st.ff && !a.hasI && ctx.st.input === 'step') return -(d / g.kPth) / (1 + g.kPphi);
      let e;
      if (ctx.st.input === 'step') e = A * a.outer.step;
      else if (ctx.st.input === 'ramp') e = a.hasI ? A * a.outer.ramp : Infinity;
      else e = Infinity;
      // d on τ shifts θ by d/k_Pθ at steady state, which the outer loop sees as an input disturbance
      if (!a.hasI && isFinite(e)) e -= (d / g.kPth) / (1 + g.kPphi);
      return e;
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Cascade (true-state derivatives)', 'p. 150–151');
      segmented(sec, {
        label: 'φ<sub>r</sub> shape (amplitude = size, slope or coefficient)',
        options: [{ value: 'step', label: 'step' }, { value: 'ramp', label: 'ramp' }, { value: 'parabola', label: 'parabola' }],
        ...bind(ctx, 'input'),
      });
      commonControls(sec, ctx, { ff: true });
      if (ctx.S.mode === 'work') {
        workSliders(sec, ctx, true);
        loadC8Button(sec, ctx);
        sec.append(el('p', { class: 'muted small', text: 'C.9 analyzes the C.8 loops: load your Ch 8 gains or set any others. The |θ_r| limit is opened up so ramps are not clipped.' }));
      } else { designSliders(sec, ctx, { withI: true }); readout(sec, ctx, ['kPth', 'kDth', 'kPphi', 'kDphi', 'kIphi']); }
      const ty = section(parent, 'System type analysis', 'p. 141 · Table 9-1');
      const box = el('div', { class: 'metrics' });
      ty.append(box);
      WB.ui.addRefresher(() => this.renderType(box, ctx));
    },

    renderType(box, ctx) {
      const a = this.analysis(ctx), S = ctx.S;
      const res = ctx.app.result();
      const n = res ? res.t.length - 1 : 0;
      const eEnd = res ? (res.rAll[0][n] - res.yAll[1][n]) * R2D : NaN;
      const row = WB.ui.metric;
      const show = WB.ui.shown(ctx, 'C:ch9:type') || ['C.9/a', 'C.9/b1', 'C.9/b2'].every((k) => ctx.app.isSolved(k));
      const rows = [];
      if (show) {
        const pr = this.predicted(ctx);
        rows.push(row('inner loop vs. θ_r / vs. d', `type ${a.inner.type} / type ${a.inner.dType}`));
        rows.push(row('outer loop vs. φ_r / vs. d₂ (k_DCθ = 1)', `type ${a.outer.type} / type ${a.outer.dType}`));
        rows.push(row(`predicted φ_r − φ (${ctx.st.input}${ctx.st.ff ? ', feedforward' : ''}, d = ${fmt(S.sim.dist, 3)})`, isFinite(pr) ? `${fmt(pr * R2D, 3)}°` : '∞ (grows)'));
      } else {
        rows.push(WB.ui.revealButton(ctx, 'C:ch9:type', 'Reveal the predicted types and errors'));
      }
      rows.push(row('simulated φ_r − φ at t_end', isFinite(eEnd) ? `${fmt(eEnd, 3)}°` : '—'));
      box.replaceChildren(...rows);
    },

    splane(ctx) { return cascadeMarkers(ctx, { drag: false }); },

    math(ctx) {
      const a = this.analysis(ctx), g = ctx.gains;
      return [
        { title: 'Final value theorem', page: 'p. 137',
          theory: '\\lim_{t\\to\\infty} e(t) = \\lim_{s\\to 0} sE(s),\\quad E = \\frac{1}{1 + PC}R' },
        { title: 'System type (Table 9-1)', page: 'p. 141',
          theory: '\\text{type} = \\text{number of free integrators in } PC,\\quad e_{step} = \\frac{1}{1 + M_p},\\; e_{ramp} = \\frac{1}{M_v},\\; e_{parab} = \\frac{1}{M_a}\\quad M_p = \\lim_{s\\to0}PC,\\; M_v = \\lim_{s\\to0}sPC,\\; M_a = \\lim_{s\\to0}s^2PC' },
        { title: 'Input disturbance', page: 'p. 141–142',
          theory: 'D(s) = \\frac{1}{s^{q+1}}:\\quad \\lim_{t\\to\\infty} e = \\lim_{s\\to0} s\\frac{P}{1+PC}\\frac{1}{s^{q+1}}' },
        { title: 'Inner loop of the satellite (Fig. 9-11)', page: 'p. 151', answers: 'C.9/a',
          theory: 'P(s)C(s) = \\frac{k_D s + k_P}{(J_s+J_p)s^2}\\;(\\text{PD on the error}),\\quad \\text{type 2},\\; e_{step} = e_{ramp} = 0,\\; e_{parab} = \\frac{J_s+J_p}{k_P},\\quad \\text{input disturbance: type 0},\\; e = \\frac{1}{k_P}',
          numbers: `e_{parab} = ${tex(a.inner.parab)},\\quad e_{d} = \\frac{1}{k_{P_\\theta}} = ${tex(a.inner.dStep)},\\quad \\text{derivative on }\\theta\\text{: type 1},\\; e_{ramp} = \\frac{k_{D_\\theta}}{k_{P_\\theta}} = ${tex(g.kDth / g.kPth)}`,
          note: 'The repo controllers differentiate θ, not the error (Fig. 7-2). That changes the reference type of this loop; see the solution.' },
        { title: 'Outer loop of the satellite (Fig. 9-12)', page: 'p. 151–153', answers: ['C.9/b1', 'C.9/b2'],
          theory: 'P(s)C(s) = \\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}\\cdot\\frac{k_Ds^2 + k_Ps + k_I}{s},\\quad k_I = 0 \\Rightarrow \\text{type 0},\\; e_{step} = \\frac{1}{1 + k_P};\\quad k_I > 0 \\Rightarrow \\text{type 1},\\; e_{ramp} = \\frac{1}{k_I},\\quad \\text{disturbance: } \\frac{1}{1+k_P}\\;(k_I = 0),\\; \\frac{1}{k_I}\\text{ for a ramp}\\;(k_I \\ne 0)',
          numbers: a.hasI ? `e_{ramp} = \\frac{1}{k_I} = ${tex(a.outer.ramp)}` : `e_{step} = \\frac{1}{1 + k_P} = ${tex(a.outer.step)}`,
          note: 'In the simulation, d acts on the body torque; it reaches the outer loop through the inner loop\'s steady-state offset.' },
        { title: 'Feedforward', page: 'p. 133 · Fig. 8-20',
          theory: '\\theta_r = k_P(\\phi_r - \\phi) - k_D\\dot\\phi + \\phi_r',
          symbolic: '\\phi_{ss} = \\phi_r \\text{ for a step, even with } k_I = 0', spoiler: true,
          note: `The C.9 analysis has no feedforward, so it is off by default here. Current gains: kPφ = ${fmt(g.kPphi, 4)}, kIφ = ${fmt(g.kIphi, 4)}.` },
      ];
    },

    buildProblem(parent, ctx) {
      const a = () => this.analysis(ctx);
      const J = (p) => p.Js + p.Jp;
      // C.9(a) has two right answers: type 2 for PD on the error (the book, Fig. 9-11), type 1
      // for the derivative on θ (ctrlPD.py). The student's ref_type picks which one is checked.
      const innerItems = (type) => [
        { var: 'ref_type', truth: () => type },
        { fn: 'e_ref', args: ['kP', 'kD'], truth: type === 2 ? (p, x) => J(p) / x.kP : (p, x) => x.kD / x.kP },
        { var: 'dist_type', truth: () => 0 },
        { fn: 'e_dist', args: ['kP', 'kD'], truth: (p, x) => 1 / x.kP },
      ];
      const checkInner = async (code) => {
        if (!code.trim()) return { ok: false, msg: 'Write your code first.' };
        const out = await WB.py.evaluate(code, [{ params: { ...ctx.pModel }, vars: ['ref_type'] }]);
        const t = out.rows ? out.rows[0].vars.ref_type : null;
        if (out.rows && t !== 1 && t !== 2) return { ok: false, msg: 'Count the free integrators in the loop (and mind where the derivative acts).' };
        const r = await WB.py.check(ctx, { args: { kP: PYARGS.kPin, kD: PYARGS.kDin }, items: innerItems(t === 1 ? 1 : 2) }, code);
        if (!r.ok) return r;
        return { ...r, msg: t === 2 ? `${r.msg} The book's answer, for PD on the error (Fig. 9-11). The repo differentiates θ instead, which makes it type 1 (see the solution).`
          : `${r.msg} Right for the implemented controller (derivative on θ). The book answers type 2 for PD on the error (Fig. 9-11).` };
      };
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch9, [
        {
          id: 'a', title: '(a) Inner loop with PD',
          html: 'Set the reference type <code>ref_type</code> and write <code>e_ref</code>, the error for the lowest-order input with a finite nonzero error (unit input, R = 1/s<sup>q+1</sup> as in Table 9-1, rad). Then the disturbance type <code>dist_type</code> and <code>e_dist</code>, the error magnitude for a unit step d. Errors are functions of the inner-loop gains; the check calls them at random gains.',
          code: {
            template: 'ref_type = ...\n\ndef e_ref(kP, kD):\n    return ...\n\ndist_type = ...\n\ndef e_dist(kP, kD):\n    return ...\n',
            check: checkInner,
          },
          solution: () => [
            { tex: `\\text{Fig. 9-11 (PD on the error)}:\; PC = \\frac{k_Ds + k_P}{(J_s+J_p)s^2} \\Rightarrow \\text{type 2},\; e_{parab} = \\frac{J_s+J_p}{k_{P_\\theta}} = ${tex(a().inner.parab)}` },
            { tex: `\\text{derivative on } \\theta:\; \\frac{E}{R} = \\frac{(J_s+J_p)s^2 + k_Ds}{(J_s+J_p)s^2 + k_Ds + k_P} \\Rightarrow \\text{type 1},\; e_{ramp} = \\frac{k_{D_\\theta}}{k_{P_\\theta}} = ${tex(ctx.gains.kDth / ctx.gains.kPth)}` },
            { tex: `\\text{input disturbance (either form): type 0},\; e = \\frac{1}{k_{P_\\theta}} = ${tex(a().inner.dStep)}\;\\text{(current gains)}` },
            { html: 'Book: p. 151. Its Notes (p. 153) say the type does not change when the derivative moves to the output; for this loop it does, because the plant has no damping of its own.' },
            { code: '# PD on the error (the book); ref_type = 1 with e_ref = kD / kP is right for derivative on theta\nref_type = 2\n\ndef e_ref(kP, kD):\n    return (P.Js + P.Jp) / kP     # e_parab = 1/M_a\n\ndist_type = 0\n\ndef e_dist(kP, kD):\n    return 1 / kP' },
          ],
        },
        {
          id: 'b1', title: '(b) Outer loop with PD (k<sub>I<sub>φ</sub></sub> = 0)',
          html: 'Inner loop replaced by its DC gain (Fig. 9-12). Set <code>system_type</code> and write the errors for a unit step, ramp and parabola in φ<sub>r</sub> (rad; <code>np.inf</code> if unbounded), then the disturbance type <code>dist_type</code> and <code>e_dist</code>, the error magnitude for a unit step d<sub>2</sub>. All as functions of the outer-loop gains.',
          code: {
            template: 'system_type = ...\n\ndef e_step(kP, kD):\n    return ...\n\ndef e_ramp(kP, kD):\n    return ...\n\ndef e_parab(kP, kD):\n    return ...\n\ndist_type = ...\n\ndef e_dist(kP, kD):\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { kP: PYARGS.kPout, kD: PYARGS.kDout },
              items: [
                { var: 'system_type', truth: () => 0 },
                { fn: 'e_step', args: ['kP', 'kD'], truth: (p, x) => 1 / (1 + x.kP) },
                { fn: 'e_ramp', args: ['kP', 'kD'], truth: () => Infinity },
                { fn: 'e_parab', args: ['kP', 'kD'], truth: () => Infinity },
                { var: 'dist_type', truth: () => 0 },
                { fn: 'e_dist', args: ['kP', 'kD'], truth: (p, x) => 1 / (1 + x.kP) },
              ],
            }, code),
          },
          solution: () => [{ tex: `\\text{type 0}:\; e_{step} = \\frac{1}{1 + k_{P_\\phi}} = ${tex(1 / (1 + ctx.gains.kPphi))},\; e_{ramp} = \\infty;\; \\text{disturbance: type 0, } \\frac{1}{1 + k_{P_\\phi}}` }, { html: 'Book: p. 152. Try it: feedforward off, step input, k<sub>I<sub>φ</sub></sub> = 0.' },
            { code: 'system_type = 0\n\ndef e_step(kP, kD):\n    return 1 / (1 + kP)      # M_p = lim PC = kP (P_out(0) = 1)\n\ndef e_ramp(kP, kD):\n    return np.inf\n\ndef e_parab(kP, kD):\n    return np.inf\n\ndist_type = 0\n\ndef e_dist(kP, kD):\n    return 1 / (1 + kP)      # lim P/(1 + PC), P_out(0) = 1' }],
        },
        {
          id: 'b2', title: '(b) Outer loop with an integrator (k<sub>I<sub>φ</sub></sub> > 0)',
          html: 'Same, with PID: <code>system_type</code>, the unit step, ramp and parabola errors, then <code>dist_type</code> and <code>e_dist</code>, the error magnitude for a unit <em>ramp</em> d<sub>2</sub>.',
          code: {
            template: 'system_type = ...\n\ndef e_step(kP, kI, kD):\n    return ...\n\ndef e_ramp(kP, kI, kD):\n    return ...\n\ndef e_parab(kP, kI, kD):\n    return ...\n\ndist_type = ...\n\ndef e_dist(kP, kI, kD):\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { kP: PYARGS.kPout, kI: PYARGS.kIout, kD: PYARGS.kDout },
              items: [
                { var: 'system_type', truth: () => 1 },
                { fn: 'e_step', args: ['kP', 'kI', 'kD'], truth: () => 0 },
                { fn: 'e_ramp', args: ['kP', 'kI', 'kD'], truth: (p, x) => 1 / x.kI },
                { fn: 'e_parab', args: ['kP', 'kI', 'kD'], truth: () => Infinity },
                { var: 'dist_type', truth: () => 1 },
                { fn: 'e_dist', args: ['kP', 'kI', 'kD'], truth: (p, x) => 1 / x.kI },
              ],
            }, code),
          },
          solution: () => [{ tex: `\\text{type 1}:\; e_{step} = 0,\; e_{ramp} = \\frac{1}{k_{I_\\phi}};\; \\text{disturbance: type 1, ramp error } \\frac{1}{k_{I_\\phi}}` }, { html: 'Book: p. 152–153. Try a ramp input with k<sub>I<sub>φ</sub></sub> > 0.' },
            { code: 'system_type = 1\n\ndef e_step(kP, kI, kD):\n    return 0.0\n\ndef e_ramp(kP, kI, kD):\n    return 1 / kI            # M_v = lim s PC = kI\n\ndef e_parab(kP, kI, kD):\n    return np.inf\n\ndist_type = 1\n\ndef e_dist(kP, kI, kD):\n    return 1 / kI            # unit ramp d2: lim s P/(1 + PC) / s^2' }],
        },
      ]);
    },
  };

  // ---------------------------------------------------- Appendix C.P.6 --
  CH.p6 = {
    id: 'p6', num: 10.5, tab: 'App. P.6', short: 'P.6', title: 'Root locus vs. k_Iφ', pages: 'pp. 472–473',
    controller: (ctx) => makeCascade(ctx),
    defaults(sys) {
      const pr = sys.problems.ch8;
      return { ff: false, deriv: 'state', antiwindup: 'none', thetaMaxDeg: 30, zoom: 'outer', trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, kIx: 0.02, kMaxFactor: 1.5,
        w: { kPth: 40, kDth: 20, kPphi: 0.5, kDphi: 4, kIphi: 0.02 } };   // Work-mode placeholder gains
    },
    simDefaults(sys) { return sys.problems.p6.sim; },
    gains: gainsFor,
    linearLabel: 'outer design model (inner loop → its DC gain)',   // k_DCθ = 1 answers C.8(c)
    linearSim: designOverlay,

    // C.8's PD gains, which the problem tells you to start from.
    specGains(ctx) { const pr = ctx.sys.problems.ch8; return designOf(ctx, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule }); },

    // Evans form 1 + k_I L(s) = 0 with L = k_DC(bs + k)/(a3 s³ + a2 s² + a1 s), made monic.
    evans(ctx, g = ctx.gains) {
      const p = ctx.pModel;
      const a3 = p.Jp + p.b * g.kDphi, a2 = p.b + p.b * g.kPphi + p.k * g.kDphi, a1 = p.k + p.k * g.kPphi;
      const den = [1, a2 / a3, a1 / a3, 0], num = [p.b / a3, p.k / a3];
      const c1 = num[0], c0 = num[1], d2 = den[1], d1 = den[2];
      const kCrit = c0 > d2 * c1 ? d2 * d1 / (c0 - d2 * c1) : Infinity;
      return { den, num, kCrit, g, a3 };
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'C.8 PD loops, then add k_Iφ', 'p. 472');
      if (ctx.S.mode === 'work') { workSliders(sec, ctx, true); loadC8Button(sec, ctx); }
      else designSliders(sec, ctx, { withI: true });
      slider(sec, { label: 'locus to', unit: '× kI,crit', min: 0.2, max: 5, step: 0.1, sig: 2, ...bind(ctx, 'kMaxFactor') });
      segmented(sec, {
        label: 's-plane view',
        options: [{ value: 'all', label: 'all poles' }, { value: 'outer', label: 'zoom on outer loop' }],
        ...bind(ctx, 'zoom'),
      });
      sec.append(el('p', { class: 'muted small', text: 'Drag a closed-loop pole along the locus to set k_Iφ. The locus is for the book\'s outer design model; the circles are the full four-state loop with the same gains.' }));
      readout(sec, ctx, ['kPth', 'kDth', 'kPphi', 'kDphi', 'kIphi']);
    },

    splane(ctx) {
      const ev = this.evans(ctx);
      const kMax = Math.max(isFinite(ev.kCrit) ? ev.kCrit * ctx.st.kMaxFactor : 2 * ctx.st.kMaxFactor, (ev.g.kIphi || 0) * 1.2, 1e-3);
      const loci = T.rootLocus(ev.den, ev.num, kMax);
      const base = cascadeMarkers(ctx, { drag: false });
      const markers = base.markers.filter((m) => m.kind !== 'cl');
      // The locus and the poles/zero of L(s) answer C.P.6(a): Work mode draws them once it is solved.
      const drawn = ctx.S.mode === 'explore' || ctx.app.isSolved('C.P.6/a');
      if (drawn) {
        L.roots(ev.den).forEach((q, i) => markers.push({ ...q, kind: 'ol', label: `pole of L(s) ${i + 1}` }));
        L.roots(ev.num).forEach((q) => markers.push({ ...q, kind: 'olzero', label: 'zero of L(s)' }));
      }
      L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, ev.g.kIphi))).forEach((q, i) => markers.push({ ...q, kind: 'cl', label: `outer closed-loop pole at kIφ = ${fmt(ev.g.kIphi, 3)}`, dragId: i }));
      const fitR = ctx.st.zoom === 'outer' ? Math.max(...L.roots(ev.den).map((q) => Math.hypot(q.re, q.im)), Math.abs(ev.num[1] / ev.num[0])) * 1.4 : undefined;
      return { markers, loci: drawn ? loci : undefined, fitR, legendNames: { ...LEGEND, cl: 'outer closed loop (design model)', olzero: 'zero of L(s)', ol: 'open-loop / L(s) pole' } };
    },

    onPoleDrag(ctx, id, re, im) {
      const ev = this.evans(ctx);
      const kMax = Math.max(isFinite(ev.kCrit) ? ev.kCrit * ctx.st.kMaxFactor : 2 * ctx.st.kMaxFactor, 1e-3);
      let bestK = ev.g.kIphi || 0, bd = Infinity;
      for (let i = 0; i <= 400; i++) {
        const k = kMax * i / 400;
        for (const q of L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, k)))) {
          const d = Math.hypot(q.re - re, q.im - im);
          if (d < bd) { bd = d; bestK = k; }
        }
      }
      if (ctx.S.mode === 'work') ctx.st.w.kIphi = bestK; else ctx.st.kIx = bestK;
      ctx.update();
    },

    math(ctx) {
      const ev = this.evans(ctx);
      return [
        { title: 'Evans form', page: 'p. 466',
          theory: '\\Delta_{cl}(s) = D(s) + k\\,N(s) = 0 \\iff 1 + k\\,L(s) = 0,\\quad L(s) = \\frac{N(s)}{D(s)} \\quad(k \\text{ is the gain that varies along the locus})' },
        outerCard(ctx, ev.g, { withI: true }),
        { title: 'Evans form of the outer loop', page: 'p. 472–473', answers: 'C.P.6/a',
          theory: '1 + k_I\\frac{k_{DC}(bs + k)}{a_3s^3 + a_2s^2 + a_1s} = 0,\\quad a_3 = J_p + bk_{DC}k_D,\\; a_1 = k + kk_{DC}k_P,\\quad a_2 = b + bk_{DC}k_P + kk_{DC}k_D',
          numbers: `L(s) = \\frac{${T.polyTex(ev.num)}}{${T.polyTex(ev.den)}}`,
          note: 'The book jumps from the closed-loop transfer function straight to the Matlab command (p. 473); this card fills in the step it leaves out.' },
        { title: 'Where the locus crosses into the RHP', page: 'Routh–Hurwitz (not in the book)', answers: 'C.P.6/a',
          theory: 's^3 + d_2s^2 + (d_1 + k_Ic_1)s + k_Ic_0 \\text{ is stable iff}\\quad d_2(d_1 + k_Ic_1) > k_Ic_0 \\;(\\text{all } c_i, d_i > 0)',
          numbers: `k_{I,crit} = \\frac{d_2d_1}{c_0 - d_2c_1} = ${isFinite(ev.kCrit) ? tex(ev.kCrit) : '\\infty'}`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.p6;
      const ev = () => this.evans(ctx, this.specGains(ctx));
      const evNow = () => this.evans(ctx);
      const Lof = (p, a) => cx.div(cx.poly([p.b, p.k], a.s), cx.poly([p.Jp + p.b * a.kD, p.b + p.b * a.kP + p.k * a.kD, p.k + p.k * a.kP, 0], a.s));
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Characteristic equation in Evans form',
          html: 'Add the integrator to the outer loop (Fig. 6-10, inner loop replaced by k<sub>DC<sub>θ</sub></sub> = 1). Write L(s) for 1 + k<sub>I</sub>L(s) = 0 in terms of the PD gains k<sub>P</sub>, k<sub>D</sub>; the check calls it at complex s.',
          code: {
            template: 'def L(s, kP, kD):\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { s: PYARGS.s, kP: PYARGS.kPout, kD: PYARGS.kDout },
              items: [{ fn: 'L', args: ['s', 'kP', 'kD'], truth: Lof }],
            }, code),
          },
          solution: () => { const e = ev(); return [
            { tex: '\\Delta_{cl} = (J_p + bk_D)s^3 + (b + bk_P + kk_D)s^2 + (k + kk_P)s + k_I(bs + k) = 0' },
            { tex: '\\Rightarrow\\; 1 + k_I\\frac{bs + k}{(J_p + bk_D)s^3 + (b + bk_P + kk_D)s^2 + (k + kk_P)s} = 0' },
            { tex: `\\text{C.8 PD gains: } L(s) = \\frac{${T.polyTex(e.num)}}{${T.polyTex(e.den)}}` },
            { code: 'def L(s, kP, kD):\n    Jp, b, k = P.Jp, P.b, P.k\n    den = ((Jp + b * kD) * s**3\n           + (b + b * kP + k * kD) * s**2\n           + (k + k * kP) * s)\n    return (b * s + k) / den' },
            { html: 'Book: p. 472–473 (it gives only the Matlab command; see ISSUES.md).' },
          ]; },
        },
        {
          id: 'b', title: '(b) Root locus versus k<sub>I</sub>', after: 'a',
          html: 'Use <code>rlocus</code> (or <code>control.root_locus</code>) on your L(s) with the C.8 PD gains. The s-plane here now draws the locus for the gains in the controls; <em>Load my C.8 gains</em> copies yours from the Ch 8 tab. Compare with your plot.',
        },
        {
          id: 'c', title: '(c) Select a k<sub>I</sub> that does not significantly change the other closed-loop poles',
          html: 'Checks the current k<sub>I<sub>φ</sub></sub>: the outer complex pair must stay within 10% (in |p|) of the PD-only poles, and the new real pole must be slower than them.',
          check: () => {
            const e = evNow();
            if (!(e.g.kIphi > 0)) return { ok: false, msg: 'Set kIφ > 0.' };
            const pd = L.roots(e.den.slice(0, 3));
            const cl = L.roots(L.polyAdd(e.den, L.polyScale(e.num, e.g.kIphi)));
            const cpx = cl.filter((q) => Math.abs(q.im) > 1e-6), real = cl.filter((q) => Math.abs(q.im) <= 1e-6);
            if (cpx.length !== 2) return { ok: false, msg: 'The PD pair has split into real poles; kIφ is too large.' };
            const ratio = Math.hypot(cpx[0].re, cpx[0].im) / Math.hypot(pd[0].re, pd[0].im);
            const slow = real.length === 1 && Math.abs(real[0].re) < Math.abs(pd[0].re);
            const ok = Math.abs(ratio - 1) < 0.1 && slow;
            return { ok, msg: `|p| ratio ${fmt(ratio, 3)}, real pole ${real.length ? fmtPole(real[0]) : '—'}.` };
          },
          solution: () => [{ html: 'With the C.8 gains, k<sub>I<sub>φ</sub></sub> ≈ 0.01–0.03 leaves the pair near −0.32 ± 0.16j and adds a slow real pole. The book stops at the rlocus command; C.10\'s listing uses k<sub>I<sub>φ</sub></sub> = 0.15 with different PD gains.' }],
        },
        {
          id: 'x', title: 'Extension: largest stable k<sub>I<sub>φ</sub></sub> (design model)',
          html: 'Not in the book. As a function of the outer PD gains k<sub>P</sub>, k<sub>D</sub>; return <code>np.inf</code> if no k<sub>I</sub> destabilizes the loop. The check calls it at random gains.',
          code: {
            template: 'def kI_crit(kP, kD):\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { kP: PYARGS.kPout, kD: PYARGS.kDout },
              items: [{ fn: 'kI_crit', args: ['kP', 'kD'], truth: (p, x) => this.evans({ pModel: p }, { kPphi: x.kP, kDphi: x.kD }).kCrit }],
            }, code),
          },
          solution: () => [
            { tex: `k_{I,crit} = \\frac{d_2d_1}{c_0 - d_2c_1}\;(c_0 > d_2c_1,\\text{ else } \\infty);\\quad \\text{C.8 gains: } ${isFinite(ev().kCrit) ? tex(ev().kCrit) : '\\infty'}` },
            { code: 'def kI_crit(kP, kD):\n    Jp, b, k = P.Jp, P.b, P.k\n    a3 = Jp + b * kD\n    d2 = (b + b * kP + k * kD) / a3\n    d1 = (k + k * kP) / a3\n    c1, c0 = b / a3, k / a3\n    # Routh on s^3 + d2 s^2 + (d1 + kI c1) s + kI c0\n    return d2 * d1 / (c0 - d2 * c1) if c0 > d2 * c1 else np.inf' },
          ],
        },
      ]);
    },
  };

  // ------------------------------------------------------------- C.10 --
  CH.ch10 = {
    id: 'ch10', num: 10, tab: 'Ch 10', title: 'Digital PID (successive loops)', pages: 'pp. 166–169',
    controller: (ctx) => makeCascade(ctx),
    // Work mode simulates the student's C.10(c) controller, from the measured y = (θ, φ).
    implement: { feed: 'y', linear: false },
    defaults(sys) {
      const pr = sys.problems.ch10;
      return {
        ff: false, deriv: 'dirty', antiwindup: 'repo', sigma: pr.sigma, thetaMaxDeg: pr.thetaMaxDeg, zoom: 'all', extra: 'int',
        trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, kIx: pr.ki,
        w: { kPth: 100, kDth: 40, kPphi: 0.5, kDphi: 5, kIphi: 0 },
      };
    },
    simDefaults(sys) { return { ...sys.problems.ch10.sim, mismatch: sys.problems.ch10.mismatch }; },
    gains: gainsFor,
    targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.M * ctx.st.trTh } : {}; },
    linearLabel: 'outer design model (inner loop → its DC gain)',   // k_DCθ = 1 answers C.8(c)
    linearSim: designOverlay,
    outputSeries(ctx, res, sc, oi) {
      if (oi !== 0 || !res.extras.thetaR) return [];
      return [{ label: 'θ_r (outer-loop output)', y: sc(res.extras.thetaR), color: '--series-2', dash: [4, 3], width: 1.5 }];
    },
    spec(ctx) { const pr = ctx.sys.problems.ch10; return designOf(ctx, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, kIx: pr.ki }); },
    // σ of the s-plane's full-loop circles: the problem's in Work mode.
    sigmaOf(ctx) { return ctx.S.mode === 'work' ? ctx.sys.problems.ch10.sigma : ctx.st.deriv === 'dirty' ? ctx.st.sigma : null; },

    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') {
        workControls(parent, ctx, { withI: true, targetKey: 'C.10/c', part: 'C.10(c)' });
      } else {
        const sec = section(parent, 'Digital PID, measured angles only', 'p. 166 · Listing 10.4');
        designSliders(sec, ctx, { withI: true }); readout(sec, ctx, ['kPth', 'kDth', 'kPphi', 'kDphi', 'kIphi']);
        const imp = section(parent, 'Implementation', 'p. 157 · Eq. 10.3–10.4');
        commonControls(imp, ctx, { ff: true, deriv: true, aw: true });
        segmented(imp, {
          label: 'Extra plot',
          options: [{ value: 'int', label: 'integrator' }, { value: 'deriv', label: 'rate estimates' }],
          ...bind(ctx, 'extra'),
        });
      }
      const sep = section(parent, 'Bandwidth separation', 'p. 129');
      separationBox(sep, ctx, () => this.sigmaOf(ctx));
    },

    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      if (ctx.st.extra === 'deriv') {
        return { opts: { title: 'rate estimates', yLabel: 'rate [°/s]', unit: '°/s' }, data: { series: [
          { label: 'θ̇ estimate', y: Array.from(res.extras.thdHat || [], (v) => v * R2D), color: '--series-2', width: 1.5 },
          { label: 'true θ̇', y: res.x.map((x) => x[2] * R2D), color: '--series-1' },
          { label: 'φ̇ estimate', y: Array.from(res.extras.phdHat || [], (v) => v * R2D), color: '--series-3', width: 1.5, dash: [4, 3] },
        ] } };
      }
      return { opts: { title: 'outer integrator ∫(φ_r − φ) dt', yLabel: '∫e [rad·s]', unit: 'rad·s' }, data: { series: [{ label: 'integrator', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },

    splane(ctx) {
      const s = this.spec(ctx);
      const tg = ctx.S.mode === 'work' && ctx.st.showTargets && lib().shows(ctx, 'C.10/c') ? [...lib().innerPoles(ctx.pModel, s), ...lib().outerPoles(ctx.pModel, { ...s, kIphi: 0 })] : null;
      return cascadeMarkers(ctx, { targets: tg, sigma: this.sigmaOf(ctx) });
    },
    onPoleDrag: onCascadeDrag,

    math(ctx) {
      const st = ctx.st, Ts = ctx.S.sim.Ts;
      const sigma = this.sigmaOf(ctx) ?? st.sigma;
      const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
      const g = ctx.gains, d = ctx.S.mode === 'work' ? this.spec(ctx) : designOf(ctx);
      return [
        ...generalCards(ctx), ...innerCards(ctx, g), outerCard(ctx, { ...g, kIphi: 0 }), ...designCards(ctx, d, { inner: 'C.10/c', outer: 'C.10/c' }),
        { title: 'Dirty derivative of the measured angles', page: 'p. 157 · Eq. 10.4',
          theory: '\\dot{\\hat y}[n] = \\frac{2\\sigma - T_s}{2\\sigma + T_s}\\dot{\\hat y}[n-1] + \\frac{2}{2\\sigma + T_s}\\big(y[n] - y[n-1]\\big)' },
        { title: 'Dirty-derivative coefficients', page: 'p. 157 · Eq. 10.4', answers: 'C.10/c',
          theory: `\\sigma = ${tex(sigma)},\; T_s = ${tex(Ts)}:\\quad \\frac{2\\sigma - T_s}{2\\sigma + T_s} = ${tex(beta)},\\quad \\frac{2}{2\\sigma + T_s} = ${tex(gamma)}` },
        { title: 'Outer PID and anti-windup (Listing 10.4)', page: 'p. 167–168', answers: 'C.10/c',
          theory: '\\theta_r = \\text{sat}\\big(k_{P_\\phi}e + k_{I_\\phi}u_I - k_{D_\\phi}\\dot{\\hat\\phi}\\big),\\quad u_I \\mathrel{+}= \\frac{T_s}{k_{I_\\phi}}(\\theta_r - \\theta_{r,unsat})',
          note: 'The listing uses t_rθ = 0.4 s with ω_n = 2.2/t_r and M = 15, not the C.8 values, and drops C.8\'s φ_r feedforward (the integrator removes the error instead).' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch10;
      const misSet = () => Object.values(ctx.S.mismatch || {}).some((v) => Math.abs(v) > 0);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Uncertain parameters (α = 0.2)',
          html: 'In your dynamics, use the <code>alpha</code> argument. Here, the true plant differs from the model by the mismatch in the left panel (the chapter starts with a fixed 20% draw).',
          check: () => (misSet() ? { ok: true, msg: `Mismatch: ${Object.entries(ctx.S.mismatch).map(([k, v]) => `${k} ${v > 0 ? '+' : ''}${fmt(v, 3)}%`).join(', ')}.` } : { ok: false, msg: 'Set a plant mismatch in the left panel.' }),
        },
        {
          id: 'b', title: '(b) Controller uses only the measured outputs',
          html: 'From here on your controller\'s <code>update(phi_r, y)</code> receives the measured y = [[θ], [φ]], not the state.',
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Implement the nested PID loops of C.8 with σ = 0.05; tune the integrator', seed: ['C.8/f', 'C.8/e'],
          html: `Start from your C.8 controller. The check (1) feeds your controller a small constant error and requires its torque to change over 3 s (integral action), then (2) runs the ±15° square wave (0.02 Hz) on a plant that differs from the model by ${lib().misText(prob.mismatch)}: |φ<sub>r</sub> − φ| just before the first switch (t = 25 s) must be under ${C10_TOL}°.`,
          check: async (code) => {
            // (1) integral action: a constant 0.1° error from rest must move τ
            const calls = Array.from({ length: 301 }, () => [0.1 * DEG, [0, 0]]);
            const pr = await WB.myCtrl.probe(ctx, code, calls);
            if (pr.ok === false) return pr;
            const u1 = pr.u[10][0], u2 = pr.u[300][0];
            if (!(Math.abs(u2 - u1) > 1e-6 + 1e-4 * Math.abs(u1))) return { ok: false, msg: `With φ_r = 0.1° and y = 0 held for 3 s your τ stays at ${fmt(u1, 4)} N·m: the controller has no integral action.` };
            // (2) tracking on the mismatched plant
            const sc = WB.myCtrl.scenario(ctx, { ref: { type: 'square', amplitude: 15, frequency: 0.02, tStep: 0 }, tEnd: 25, mismatch: prob.mismatch });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const e = lib().phiErrAt(res, 24.95);
            return { ok: e < C10_TOL, msg: `Error before the switch: ${fmt(e, 3)}°.` };
          },
          solution: () => [
            { code: SOL10 },
            { html: `The repo's ctrlPID.py (Listing 10.4, p. 167): dirty derivatives of the measured angles (σ = 0.05), |θ<sub>r</sub>| ≤ 30° with the listing's back-calculation anti-windup, k<sub>I<sub>φ</sub></sub> = ${prob.ki}. It uses t<sub>r<sub>θ</sub></sub> = ${prob.trTh} s, M = ${prob.M} and ω<sub>n</sub> = 2.2/t<sub>r</sub>, not the C.8 values; the C.8 loops (t<sub>r<sub>θ</sub></sub> = 1 s, M = 10) with k<sub>I<sub>φ</sub></sub> ≈ 0.15 pass too. With this satellite the parameter error alone leaves no steady-state error (P<sub>out</sub>(0) = 1): the integrator removes the 1/(1 + k<sub>P<sub>φ</sub></sub>) error of PD without the φ<sub>r</sub> feedforward (see ISSUES.md).` },
          ],
        }),
      ]);
    },
  };

  WB.studies.C.cascade = { makeCascade, slc: (p, o) => lib().slcDesign(p, o) };
})();
