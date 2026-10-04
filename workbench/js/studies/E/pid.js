// Study E, Chapters 8–10 and Appendix P.6: successive loop closure with PD in both
// loops (E.8), system type (E.9), root locus versus the outer integrator gain
// (E.P.6), and the digital nested PID (E.10).
//
// Inner loop θ: F̃ = kPθ(θ_r − θ) − kDθ θ̇ on P_in = b0/s².
// Outer loop z: θ_r = kPz(z_r − z) − kDz ż (+ kIz ∫e) on P_out = −g/s², with the
// inner loop replaced by its DC gain (Fig. 8-11, p. 118). F = F_fl(z) + F̃.
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.E = WB.studies.E || { chapters: {} };

(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texPole, fmt, fmtPole } = M;
  const E = WB.E;
  const PD = () => WB.pd;
  const CH = WB.studies.E.chapters;

  const KNOBS = (st) => ({ trTh: st.trTh, zetaTh: st.zetaTh, M: st.M, zetaZ: st.zetaZ, rule: st.rule || '2.2' });
  const designed = (ctx, st = ctx.st) => E.pdDesign(ctx.pModel, KNOBS(st));
  const workGains = (st) => ({ kPth: st.kPth, kDth: st.kDth, kPz: st.kPz, kDz: st.kDz, kIz: st.kIz || 0, kIth: st.kIth || 0 });
  // Work-mode starting gains: stable on the exact model, deliberately not an answer.
  const W0 = { kPth: 3, kDth: 1.5, kPz: -0.01, kDz: -0.05 };
  const showsAnswer = (ctx, key) => ctx.S.mode === 'explore' || ctx.app.isSolved(key);
  const ids = (ctx) => ({ e4: ctx.sys.problems.ch4.id, e5: ctx.sys.problems.ch5.id, e8: ctx.sys.problems.ch8.id, e9: ctx.sys.problems.ch9.id, p6: ctx.sys.problems.p6.id, e10: ctx.sys.problems.ch10.id });

  // ------------------------------------------------ solution controllers --
  // E.8: nested PD from the state, F = F_fl(z) + F̃. trLines set tr_th (and tr_z);
  // sat adds the E.8(f) saturation.
  const E8_SOL = (trLines, sat) => `class Controller:
    def __init__(self):
        # inner loop (theta) on P_in = b0/s^2, the block at z_e = ell/2 (E.5(c))
        ze = P.ell / 2
        b0 = P.ell / (P.m2 * P.ell**2 / 3 + P.m1 * ze**2)
        ${trLines}
        zeta_th = 0.707
        wn_th = 2.2 / tr_th
        self.kp_th = wn_th**2 / b0
        self.kd_th = 2 * zeta_th * wn_th / b0
        # outer loop (z) on P_out = -g/s^2, the inner loop as its DC gain (1)
        zeta_z = 0.707
        wn_z = 2.2 / tr_z
        self.kp_z = -wn_z**2 / P.g
        self.kd_z = -2 * zeta_z * wn_z / P.g

    def update(self, z_r, x):
        z = x[0, 0]
        theta = x[1, 0]
        zdot = x[2, 0]
        thetadot = x[3, 0]
        theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot
        F_tilde = self.kp_th * (theta_r - theta) - self.kd_th * thetadot
        # feedback linearization (E.4(c)) with the actual block position
        F_fl = P.m1 * P.g * z / P.ell + P.m2 * P.g / 2
${sat ? '        F = F_fl + F_tilde\n        return max(-P.F_max, min(P.F_max, F))\n' : '        return F_fl + F_tilde\n'}`;

  // E.10: nested PID from the measured z, θ (dirty derivatives, both integrators).
  // gate adds the E.10(c) anti-windup.
  const E10_SOL = (gate) => `class Controller:
    def __init__(self):
        # the E.8 loops, made much faster so they survive 20% parameter error
        ze = P.ell / 2
        b0 = P.ell / (P.m2 * P.ell**2 / 3 + P.m1 * ze**2)
        tr_th = 0.15
        zeta_th = 0.707
        wn_th = 2.2 / tr_th
        self.kp_th = wn_th**2 / b0
        self.kd_th = 2 * zeta_th * wn_th / b0
        self.ki_th = 20.0
        tr_z = 8 * tr_th
        zeta_z = 0.707
        wn_z = 2.2 / tr_z
        self.kp_z = -wn_z**2 / P.g
        self.kd_z = -2 * zeta_z * wn_z / P.g
        self.ki_z = -0.05            # negative, like kp_z and kd_z
        # dirty derivatives (Eq. 10.4), sigma = 0.05
        sigma = 0.05
        self.beta = (2 * sigma - P.Ts) / (2 * sigma + P.Ts)
        self.gamma = 2 / (2 * sigma + P.Ts)
        self.z_prev = P.z0
        self.theta_prev = P.theta0
        self.zdot = P.zdot0
        self.thetadot = P.thetadot0
        self.int_z = 0.0
        self.err_z_prev = 0.0
        self.int_th = 0.0
        self.err_th_prev = 0.0

    def update(self, z_r, y):
        z = y[0, 0]
        theta = y[1, 0]
        self.zdot = self.beta * self.zdot + self.gamma * (z - self.z_prev)
        self.thetadot = self.beta * self.thetadot + self.gamma * (theta - self.theta_prev)
        self.z_prev = z
        self.theta_prev = theta
        # outer loop: PID on z gives theta_r
        err_z = z_r - z
${gate ? '        # anti-windup: integrate only while the block is nearly still\n        if abs(self.zdot) < 0.05:\n            self.int_z += P.Ts / 2 * (err_z + self.err_z_prev)\n' : '        self.int_z += P.Ts / 2 * (err_z + self.err_z_prev)\n'}        self.err_z_prev = err_z
        theta_r = self.kp_z * err_z + self.ki_z * self.int_z - self.kd_z * self.zdot
        # inner loop: PID on theta gives F_tilde
        err_th = theta_r - theta
        self.int_th += P.Ts / 2 * (err_th + self.err_th_prev)
        self.err_th_prev = err_th
        F_tilde = self.kp_th * err_th + self.ki_th * self.int_th - self.kd_th * self.thetadot
        F = P.m1 * P.g * z / P.ell + P.m2 * P.g / 2 + F_tilde
        return max(-P.F_max, min(P.F_max, F))
`;

  // Work mode for the implementation chapters (E.8, E.10): the PD gain sliders only
  // place the s-plane poles (their names are in the problem statement); the time
  // plots show the student's own controller (WB.myCtrl), named by `part`.
  function workControls(parent, ctx, part) {
    const sec = section(parent, 'PD gains (s-plane)', 'p. 118 · Fig. 8-10');
    workGainSliders(sec, ctx);
    separationReadout(sec, ctx);
    sec.append(el('p', { class: 'muted small', text: 'These place the inner (θ) and outer (z) poles in the s-plane. They do not drive the simulation.' }));
    WB.myCtrl.banner(section(parent, 'Your controller'), ctx, part);
  }
  const STEP = (amplitude) => ({ type: 'step', amplitude, tStep: 0 });
  const SQUARE = (amplitude, frequency) => ({ type: 'square', amplitude, frequency, tStep: 0 });
  const mis = (m) => Object.entries(m).map(([k, v]) => `${k === 'ell' ? 'ℓ' : k} ${v > 0 ? '+' : '−'}${Math.abs(v)}%`).join(', ');

  // ------------------------------------------------------------ shared UI --
  function compControl(parent, ctx) {
    segmented(parent, {
      label: 'Gravity / equilibrium force',
      options: [
        { value: 'fl', label: `${E.flName(ctx)} (E.8e)`, title: E.shows(ctx, `${ctx.sys.problems.ch4.id}/c`) ? 'feedback linearization with the measured z' : 'feedback linearization (your E.4(c))' },
        { value: 'eq', label: 'F<sub>e</sub> at z<sub>e</sub>', title: 'constant equilibrium force' },
        { value: 'none', label: 'none' },
      ],
      ...bind(ctx, 'comp'),
    });
    E.ffWorkControls(parent, ctx);  // Work mode: F_fl, F_e wait for E.4(c), E.4(a)
  }

  function workGainSliders(parent, ctx, { kI = false, kIth = false } = {}) {
    E.knob(parent, ctx, 'kPth', 'k<sub>P<sub>θ</sub></sub>', 0, 150, 0.01);
    E.knob(parent, ctx, 'kDth', 'k<sub>D<sub>θ</sub></sub>', 0, 15, 0.001);
    E.knob(parent, ctx, 'kPz', 'k<sub>P<sub>z</sub></sub>', -0.5, 0, 0.00001, { unit: 'rad/m' });
    E.knob(parent, ctx, 'kDz', 'k<sub>D<sub>z</sub></sub>', -0.5, 0, 0.00001, { unit: 'rad·s/m' });
    if (kI) E.knob(parent, ctx, 'kIz', 'k<sub>I<sub>z</sub></sub>', -0.2, 0, 0.00001, { unit: 'rad/(m·s)' });
    if (kIth) E.knob(parent, ctx, 'kIth', 'k<sub>I<sub>θ</sub></sub>', 0, 50, 0.01);
  }

  function designKnobs(parent, ctx, { rule = true } = {}) {
    E.knob(parent, ctx, 'trTh', 't<sub>r<sub>θ</sub></sub>', 0.05, 3, 0.005, { unit: 's' });
    E.knob(parent, ctx, 'zetaTh', 'ζ<sub>θ</sub>', 0.3, 1.5, 0.005);
    E.knob(parent, ctx, 'M', 'M = t<sub>r<sub>z</sub></sub>/t<sub>r<sub>θ</sub></sub>', 1.5, 30, 0.1, { sig: 3, hint: 'bandwidth separation; the book suggests 5–10 (p. 118)' });
    E.knob(parent, ctx, 'zetaZ', 'ζ<sub>z</sub>', 0.3, 1.5, 0.005);
    if (rule) {
      segmented(parent, {
        label: 'ω<sub>n</sub> from t<sub>r</sub>',
        options: [{ value: '2.2', label: '2.2 / t<sub>r</sub>', title: 'Eq. 8.5' }, { value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))' }],
        ...bind(ctx, 'rule'),
      });
    }
  }

  function gainReadout(parent, ctx, keys = ['kPth', 'kDth', 'kPz', 'kDz']) {
    const names = { kPth: 'kPθ', kDth: 'kDθ', kPz: 'kPz', kDz: 'kDz', kIz: 'kIz', kIth: 'kIθ' };
    E.readout(parent, () => keys.map((k) => [names[k], fmt(ctx.gains[k], 4)]));
  }

  // Inner/outer separation readout from the current gains.
  function separationReadout(parent, ctx) {
    E.readout(parent, () => {
      const g = ctx.gains, b0 = ctx.sys.linear(ctx.pModel).b0;
      const wi = Math.sqrt(Math.max(0, b0 * g.kPth)), wo = Math.sqrt(Math.max(0, -ctx.pModel.g * g.kPz));
      return [['ωn,θ', `${fmt(wi, 3)} rad/s`], ['ωn,z', `${fmt(wo, 3)} rad/s`], ['ratio', fmt(wi / wo, 3)]];
    });
  }

  // s-plane: inner loop (θ) poles and outer loop (z) poles, plus target rings.
  function loopMarkers(ctx, { drag = false, targets = null } = {}) {
    const g = ctx.gains, p = ctx.pModel;
    const b0 = ctx.sys.linear(p).b0;
    // The design model's poles at 0 answer E.5(c), so in Work mode they wait for it.
    const mk = showsAnswer(ctx, `${ctx.sys.problems.ch5.id}/c`) ? [{ re: 0, im: 0, kind: 'ol', label: 'open-loop poles of the design model' }] : [];
    const zoomOuter = ctx.st.zoom === 'outer';
    E.innerPoles(b0, g).forEach((q) => mk.push({ ...q, kind: 'obs', label: 'inner-loop pole (θ)', dragId: drag ? 0 : undefined, noFit: zoomOuter }));
    E.outerPoles(p, g).forEach((q) => mk.push({ ...q, kind: 'cl', label: 'outer-loop pole (z, inner loop as k_DC)', dragId: drag ? 1 : undefined }));
    if (targets) {
      const d = targets;
      E.innerPoles(b0, d).forEach((q) => mk.push({ ...q, kind: 'target', label: 'target pole (inner spec)', noFit: zoomOuter }));
      E.outerPoles(p, d).forEach((q) => mk.push({ ...q, kind: 'target', label: 'target pole (outer spec)' }));
    }
    return mk;
  }
  const KIND_NAMES = { obs: 'inner-loop pole (θ)', cl: 'outer-loop pole (z)', ol: 'open-loop poles', target: 'target pole (spec)' };

  function loopDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-1e-3, re);
    const wn = Math.hypot(re, im);
    const zeta = Math.max(0.3, Math.min(1, -re / wn));
    const tr = st.rule === 'tp' ? Math.PI / (2 * wn * Math.sqrt(Math.max(1e-6, 1 - zeta * zeta))) : 2.2 / wn;
    if (id === 0) { const trZ = st.M * st.trTh; st.zetaTh = zeta; st.trTh = Math.max(0.02, tr); st.M = Math.max(1.5, trZ / st.trTh); }
    else { st.zetaZ = zeta; st.M = Math.max(1.5, tr / st.trTh); }
    ctx.update();
  }

  function zoomControl(parent, ctx) {
    segmented(parent, {
      label: 's-plane scale',
      options: [{ value: 'both', label: 'both loops' }, { value: 'outer', label: 'zoom to outer loop' }],
      ...bind(ctx, 'zoom'),
    });
  }

  function thetaRSeries(ctx, res, sc, oi) {
    if (oi !== 1 || !res.extras.thetaR) return [];
    return [{ label: 'θᵣ (outer-loop command)', y: sc(res.extras.thetaR), color: '--series-3', dash: [4, 3], width: 1.5 }];
  }

  // Math cards shared by the successive-loop chapters. General book forms are
  // always shown; cards with E's own results carry `answers` (locked in Work mode
  // until that part is solved).
  function cascadeCard() {
    return {
      title: 'Successive loop closure', page: 'p. 117–118 · Fig. 8-10, 8-11',
      theory: '\\text{inner loop first; the outer loop sees the closed inner loop},\\quad \\text{inner loop} \\approx k_{DC} \\text{ when it is much faster than the outer loop}',
    };
  }
  function designModelCard(ctx) {
    const lin = ctx.sys.linear(ctx.pModel);
    return {
      title: 'Design model of the block and beam', page: 'p. 387 · E.5(c)', answers: `${ids(ctx).e5}/c`,
      theory: 'P_{in}(s) = \\frac{\\tilde\\Theta}{\\tilde F} = \\frac{b_0}{s^2},\\; b_0 = \\frac{\\ell}{\\frac{m_2\\ell^2}{3} + m_1 z_e^2},\\quad P_{out}(s) = \\frac{\\tilde Z}{\\tilde\\Theta} = -\\frac{g}{s^2},\\quad z_e = \\tfrac{\\ell}{2}',
      numbers: `b_0 = ${tex(lin.b0)},\\quad P_{in} = \\frac{${tex(lin.b0)}}{s^2},\\quad P_{out} = \\frac{${tex(-ctx.pModel.g)}}{s^2}`,
    };
  }
  function equilibriumCard(ctx) {
    const p = ctx.pModel, s = ctx.sys, e4 = ids(ctx).e4;
    return {
      title: 'Equilibrium force and feedback linearization', page: 'p. 386 · E.4, p. 388 · E.8(e)', answers: [`${e4}/a`, `${e4}/c`],
      theory: 'x_e = (\\tfrac{\\ell}{2}, 0, 0, 0),\\quad F_e = \\frac{m_1 g z_e}{\\ell} + \\frac{m_2 g}{2},\\quad F_{fl}(z) = \\frac{m_1 g z}{\\ell} + \\frac{m_2 g}{2}\\;(\\text{follows the block, cancels the } m_1 g\\tilde z \\text{ term})',
      numbers: `F_e = ${tex(s.Fe(p))}\\,\\text{N},\\quad F_{fl}(z) = ${tex(p.m1 * p.g / p.ell)}\\,z + ${tex(p.m2 * p.g / 2)}`,
    };
  }
  function pdTheoryCard() {
    return {
      title: 'PD on a second-order plant', page: 'p. 99–101 · Eq. 7.5, p. 113 · Eq. 8.5',
      theory: 'P = \\frac{b_0}{s^2 + a_1 s + a_0}:\\; \\Delta_{cl} = s^2 + (a_1 + b_0 k_D)s + (a_0 + b_0 k_P) = s^2 + 2\\zeta\\omega_n s + \\omega_n^2,\\quad \\omega_n = \\frac{2.2}{t_r},\\quad k_{DC} = \\lim_{s\\to0}\\tfrac{Y}{R}',
    };
  }
  function innerCard(ctx, d) {
    const e8 = ids(ctx).e8;
    return {
      title: 'Inner loop (θ) of the block and beam', page: 'p. 387 · E.8(b, c)', answers: [`${e8}/b`, `${e8}/c`],
      theory: 'k_{P_\\theta} = \\frac{\\omega_{n_\\theta}^2}{b_0},\\; k_{D_\\theta} = \\frac{2\\zeta_\\theta\\omega_{n_\\theta}}{b_0},\\quad k_{DC_\\theta} = 1',
      numbers: `\\omega_{n_\\theta} = ${tex(d.wTh)}\\;\\Rightarrow\\; k_{P_\\theta} = ${tex(d.kPth)},\\; k_{D_\\theta} = ${tex(d.kDth)},\\quad p = ${E.innerPoles(d.b0, d).map((q) => texPole(q)).join(',\\;')}`,
    };
  }
  function outerCard(ctx, d) {
    return {
      title: 'Outer loop (z), inner loop as its DC gain', page: 'p. 387 · E.8(d)', answers: `${ids(ctx).e8}/d`,
      theory: '\\frac{Z}{Z_r} = \\frac{-g k_{DC}k_{P_z}}{s^2 - g k_{DC}k_{D_z}s - g k_{DC}k_{P_z}},\\quad k_{P_z} = -\\frac{\\omega_{n_z}^2}{g k_{DC}},\\; k_{D_z} = -\\frac{2\\zeta_z\\omega_{n_z}}{g k_{DC}}\\;(\\text{both negative})',
      numbers: `t_{r_z} = ${tex(d.trZ)}\\,\\text{s},\\; \\omega_{n_z} = ${tex(d.wZ)}\\;\\Rightarrow\\; k_{P_z} = ${tex(d.kPz)},\\; k_{D_z} = ${tex(d.kDz)}`,
    };
  }
  function separationCard(ctx) {
    const g = ctx.gains, p = ctx.pModel;
    const b0 = ctx.sys.linear(p).b0;
    const pi = E.innerPoles(b0, g), po = E.outerPoles(p, g);
    const full = E.nestedPoles(p, g, 'fl');
    const wi = Math.sqrt(Math.max(0, b0 * g.kPth)), wo = Math.sqrt(Math.max(0, -p.g * g.kPz));
    return {
      title: 'Bandwidth separation (current gains)', page: 'p. 117–118, pp. 315–317 (B.17c)',
      theory: 't_{r_z} = M\\,t_{r_\\theta},\\; M \\approx 5\\text{–}10',
      numbers: `\\text{inner } ${pi.map((q) => texPole(q)).join(',\\;')},\\quad \\text{outer } ${po.map((q) => texPole(q)).join(',\\;')},\\quad \\omega_{n_\\theta}/\\omega_{n_z} = ${tex(wi / wo, 3)}`,
      note: `Poles of the actual 4th-order nested loop (design model, current gains): ${E.poleText(full)}. They sit close to the two separate designs when the separation is large.`,
    };
  }

  // The sampled F_fl(z_k) is held for T_s while gravity on the block changes
  // continuously; with the tiny E.8 outer gains this matters (see ISSUES.md).
  function holdCard(ctx) {
    const p = ctx.pModel, g = ctx.gains, Ts = ctx.S.sim.Ts;
    return {
      title: 'Sampling the feedback-linearizing force', page: 'p. 155–158 (digital implementation)', answers: `${ids(ctx).e4}/c`,
      theory: 'u_{fl}(x_k) \\text{ is held over } [t_k, t_k + T_s] \\text{ while the state keeps moving}',
      symbolic: '\\text{uncancelled } \\frac{m_1 g}{\\ell}\\big(z(t) - z_k\\big) \\approx \\frac{m_1 g}{\\ell}\\dot z\\,(t - t_k)\\;\\Rightarrow\\;\\text{average } \\frac{m_1 g T_s}{2\\ell}\\dot z \\;(\\text{negative damping on } \\dot z)',
      numbers: `\\frac{m_1 g T_s}{2\\ell} = ${tex(p.m1 * p.g * Ts / (2 * p.ell))}\\;\\text{N·s/m vs. } |k_{P_\\theta}k_{D_z}| = ${tex(Math.abs(g.kPth * g.kDz))}`,
      spoiler: true,
      note: 'If the simulated z overshoots much more than the dashed linear model, try a smaller T_s in the left panel. The linear model has no sampled feedforward.',
    };
  }

  // Peak |F| demanded for a z̃_r step from rest at z_e, gains designed from t_rz
  // (with t_rθ = t_rz/M), F_fl(z) on, no saturation. Used for E.8(f).
  function peakForTrZ(ctx, trZ, k) {
    const p = ctx.pModel, s = ctx.sys;
    const d = E.pdDesign(p, { ...k, trTh: trZ / k.M });
    const z0 = s.ze(p);
    const out = WB.sim.simulate({
      plant: { f: (x, u) => s.f(x, u, p), h: s.h, uLimit: Infinity },
      controller: E.nestedPID(ctx, d, { comp: 'fl', meas: 'state', ffAlways: true }),  // only the peak is reported
      reference: () => z0 + k.step, disturbance: null, noise: null,
      x0: [z0, 0, 0, 0], Ts: ctx.S.sim.Ts, tEnd: Math.max(5, 5 * trZ),
    });
    let peak = 0;
    for (const u of out.uDemand) peak = Math.max(peak, Math.abs(u));
    return peak;
  }
  function fastestTrZ(ctx, k) {
    const Fmax = ctx.sys.uLimit(ctx.pModel);
    let lo = 0.1, hi = 10;  // E.8 nominal t_rz; much slower loops go unstable on this step
    if (peakForTrZ(ctx, hi, k) > Fmax) return NaN;
    for (let i = 0; i < 40; i++) {
      const mid = 0.5 * (lo + hi);
      if (peakForTrZ(ctx, mid, k) > Fmax) lo = mid; else hi = mid;
    }
    return hi;
  }

  function useGains(ctx, vals) {
    ctx.app.setMode('work');
    Object.assign(ctx.st, vals);
    ctx.update();
  }

  // Name of the applied feedforward on the force-split plot (Work mode: what ffOf applies).
  function ffLabel(ctx) {
    const c = ctx.st.comp;
    if (c === 'none') return 'no feedforward';
    if (E.ffApplied(ctx, c)) return c === 'fl' ? 'F_fl(z)' : 'F_e';
    return c === 'fl' ? 'no feedforward (F_fl waits for E.4(c))' : 'your F_e';
  }

  // ------------------------------------------------------------- Chapter 8 --
  const STEP_E = 0.15;  // E.8(e) check: z̃_r step [m]
  CH.ch8 = {
    id: 'ch8', num: 8, tab: 'Ch 8', title: 'Successive loop closure (PD)', pages: 'pp. 107–136',
    // Work mode simulates the student's E.8(e)/(f) controller, which gets the state
    // (as the book's Ch 8 code); no linear overlay (it would need the student's gains).
    implement: { feed: 'state', linear: false },

    defaults(sys) {
      const pr = sys.problems.ch8;
      return { comp: 'fl', ...W0, trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, rule: '2.2', zoom: 'both' };
    },
    simDefaults(sys) { return sys.problems.ch8.sim; },
    gains(ctx) { return ctx.S.mode === 'work' ? workGains(ctx.st) : designed(ctx); },
    controller(ctx, o) { return E.nestedPID(ctx, ctx.gains, { comp: ctx.st.comp, meas: 'state' }, o); },
    linearSim(ctx, c) { return E.nestedLinearSim(ctx, c, ctx.gains, {}); },
    linearLabel: 'linear design model',
    targets(ctx) { return { tr: designed(ctx).trZ, zeta: ctx.st.zetaZ }; },
    outputSeries: thetaRSeries,

    buildControls(parent, ctx) {
      const work = ctx.S.mode === 'work';
      if (work) workControls(parent, ctx, `${ctx.sys.problems.ch8.id}(e) or (f)`);
      else {
        const sec = section(parent, 'PD inner and outer loops', 'p. 118 · Fig. 8-10');
        compControl(sec, ctx);
        separationReadout(sec, ctx);
      }
      const spec = section(parent, work ? 'Specs (target rings)' : 'Design knobs', 'p. 387 · E.8(b, d)');
      designKnobs(spec, ctx, { rule: !work });
      if (!work) {
        spec.append(el('p', { class: 'muted small', text: 'Drag an inner (θ) or outer (z) pole. Moving the outer pair changes M; moving the inner pair keeps t_rz.' }));
        gainReadout(spec, ctx);
      }
      zoomControl(spec, ctx);
    },

    splane(ctx) {
      const work = ctx.S.mode === 'work';
      return { markers: loopMarkers(ctx, { drag: !work, targets: work ? designed(ctx) : null }), kindNames: KIND_NAMES, zetaRay: ctx.st.zetaZ < 1 ? ctx.st.zetaZ : null };
    },
    onPoleDrag: loopDrag,

    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      return {
        opts: { title: 'force split F = F_ff + F̃', yLabel: 'F [N]', unit: 'N' },
        data: { series: [
          { label: ffLabel(ctx), y: Array.from(res.extras.ff || []), color: '--text-muted', width: 1.5 },
          { label: 'F̃ (PD)', y: Array.from(res.extras.Ft || []), color: '--series-2', width: 1.5 },
        ] },
      };
    },

    math(ctx) {
      const d = designed(ctx);
      const Fmax = ctx.sys.uLimit(ctx.pModel);
      return [
        cascadeCard(), pdTheoryCard(), designModelCard(ctx), equilibriumCard(ctx), innerCard(ctx, d), outerCard(ctx, d), separationCard(ctx), holdCard(ctx),
        { title: 'Saturation and the rise time', page: 'p. 119–121 · Eq. 8.8, Fig. 8-13',
          theory: '|u_{ff} + \\tilde u| \\le u_{max} \\;\\Rightarrow\\; \\tilde u_{max} = u_{max} - |u_{ff}|' },
        { title: 'Saturation of the block and beam', page: 'p. 388 · E.8(f)', answers: [`${ids(ctx).e8}/f`, `${ids(ctx).e4}/c`],
          theory: '\\tilde F(0^+) = k_{P_\\theta}k_{P_z}\\,\\tilde z_r \\;(\\text{negative for } \\tilde z_r > 0\\text{: plenty of room}),\\quad \\text{the binding limit is braking near the tip, where } F_{fl}(z) \\text{ has grown}',
          numbers: `F_{max} = ${tex(Fmax)}\\,\\text{N},\\quad F_{fl}(z_e + 0.25) = ${tex(ctx.sys.Ffl(ctx.sys.ze(ctx.pModel) + 0.25, ctx.pModel))}\\,\\text{N}` },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch8;
      const spec = () => E.pdDesign(ctx.pModel, { trTh: prob.trTh, zetaTh: prob.zetaTh, M: prob.M, zetaZ: prob.zetaZ, rule: '2.2' });
      const num = PD().num;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Block diagram for successive loop closure',
          html: 'On paper: draw the block diagram with PD control in both loops (see Fig. 8-10, p. 118, for the pattern). The outer loop takes z<sub>r</sub> and gives θ<sub>r</sub>; the inner loop takes θ<sub>r</sub> and gives F̃. Design about z<sub>e</sub> = ℓ/2.',
          done: 'I have drawn it',
          solution: () => [
            { html: 'Block diagram: z<sub>r</sub> → [k<sub>P<sub>z</sub></sub>, k<sub>D<sub>z</sub></sub>s] → θ<sub>r</sub> → [k<sub>P<sub>θ</sub></sub>, k<sub>D<sub>θ</sub></sub>s] → F̃ → (+ F<sub>fl</sub>) → plant, with θ fed back to the inner loop and z to the outer loop.' },
            { tex: `z_e = \\tfrac{\\ell}{2} = ${tex(ctx.sys.ze(ctx.pModel))},\\quad F_e = \\frac{m_1 g z_e}{\\ell} + \\frac{m_2 g}{2} = \\frac{(m_1 + m_2)g}{2} = ${tex(ctx.sys.Fe(ctx.pModel))}\\,\\text{N}` },
          ],
        },
        {
          id: 'b', title: `(b) Inner loop: t<sub>r<sub>θ</sub></sub> = ${prob.trTh} s, ζ<sub>θ</sub> = ${prob.zetaTh}`,
          inputs: { kPth: 'k<sub>P<sub>θ</sub></sub>', kDth: 'k<sub>D<sub>θ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { kPth: spec().kPth, kDth: spec().kDth }, { kPth: 'kPθ', kDth: 'kDθ' }),
          solution: () => {
            const d = spec();
            return [
              { tex: `b_0 = \\frac{\\ell}{m_2\\ell^2/3 + m_1\\ell^2/4} = ${tex(d.b0)},\\quad \\omega_{n_\\theta} = \\frac{2.2}{${prob.trTh}} = ${tex(d.wTh)}` },
              { tex: `s^2 + b_0k_{D_\\theta}s + b_0k_{P_\\theta} = s^2 + 2\\zeta\\omega_n s + \\omega_n^2 \\Rightarrow k_{P_\\theta} = ${tex(d.kPth)},\\; k_{D_\\theta} = ${tex(d.kDth)}` },
              { html: 'Same steps as B.8(b) (pp. 124–125), with P<sub>in</sub> = b₀/s² from E.5(c).' },
            ];
          },
        },
        {
          id: 'c', title: '(c) Inner-loop DC gain',
          inputs: { kdc: 'k<sub>DC<sub>θ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { kdc: 1 }, { kdc: 'kDC' }),
          solution: () => [{ tex: 'k_{DC_\\theta} = \\lim_{s\\to0}\\frac{b_0k_{P_\\theta}}{s^2 + b_0k_{D_\\theta}s + b_0k_{P_\\theta}} = 1' }, { html: 'P<sub>in</sub> has no a₀ term (no gravity spring once F<sub>fl</sub> is used), unlike the pendulum\'s k<sub>DC</sub> = 1.89 in B.8.' }],
        },
        {
          id: 'd', title: `(d) Outer loop: t<sub>r<sub>z</sub></sub> = ${prob.M}·t<sub>r<sub>θ</sub></sub>, ζ<sub>z</sub> = ${prob.zetaZ}`,
          inputs: { kPz: 'k<sub>P<sub>z</sub></sub>', kDz: 'k<sub>D<sub>z</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { kPz: spec().kPz, kDz: spec().kDz }, { kPz: 'kPz', kDz: 'kDz' }),
          actions: [{
            label: 'Use my gains',
            run: (v) => {
              const a = E.answersOf(ctx, prob.id);
              const vals = { kPth: num(a['b.kPth']), kDth: num(a['b.kDth']), kPz: num(v.kPz), kDz: num(v.kDz) };
              if (Object.values(vals).some((x) => x === null)) return { ok: false, msg: 'Enter kPθ, kDθ in (b) and kPz, kDz here first.' };
              useGains(ctx, vals); return null;
            },
          }],
          solution: () => {
            const d = spec();
            return [
              { tex: `\\omega_{n_z} = \\frac{2.2}{${prob.M}\\cdot${prob.trTh}} = ${tex(d.wZ)},\\quad s^2 - g k_{D_z}s - g k_{P_z} = s^2 + 2\\zeta_z\\omega_{n_z}s + \\omega_{n_z}^2` },
              { tex: `k_{P_z} = -\\frac{\\omega_{n_z}^2}{g} = ${tex(d.kPz)},\\quad k_{D_z} = -\\frac{2\\zeta_z\\omega_{n_z}}{g} = ${tex(d.kDz)}` },
              { html: 'Unlike the pendulum (B.8(d), pp. 125–127), −g/s² has no zeros, so no zero-cancelling filter is needed and plain PD gets the full second-order match.' },
            ];
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'e', title: '(e) Implement the successive loop closure design in simulation, using the actual block position in the feedback-linearizing term',
          html: `Write the nested PD controller with your gains from (b) and (d). <code>update</code> gets z<sub>r</sub> and the state x = (z, θ, ż, θ̇), as in the book's Ch 8 code, and returns F. <em>Run my controller</em> drives the time plots (the chapter starts with the book's square wave). The check simulates a ${STEP_E} m step of z̃<sub>r</sub> for 30 s with the nominal and with other parameters and compares z(t) with the design (within 2% of the step).`,
          check: (code) => {
            const tune = { trTh: prob.trTh, zetaTh: prob.zetaTh, M: prob.M, zetaZ: prob.zetaZ, rule: '2.2' };
            const cases = WB.myCtrl.paramCases(ctx).map((pc) => {
              const sc = WB.myCtrl.scenario(ctx, { params: pc.params, ref: STEP(STEP_E), tEnd: 30, feed: 'state' });
              return {
                sc, label: pc.label,
                ref: () => WB.myCtrl.reference(ctx, sc, E.nestedPID(WB.myCtrl.refCtx(ctx, sc), E.pdDesign(sc.params, tune), { comp: 'fl', meas: 'state', ffAlways: true })),
              };
            });
            return WB.myCtrl.matchCheck(ctx, code, cases, { tol: 0.02 * STEP_E });
          },
          solution: () => [
            { code: E8_SOL('tr_th = 1.0\n        tr_z = 10 * tr_th', false) },
            { html: 'With the E.8 gains the design is a slow second-order step (t<sub>r</sub> ≈ 10 s, ~4% overshoot), and F stays between about 9.5 and 13 N. The simulation at T<sub>s</sub> = 0.01 s overshoots by ~48% because F<sub>fl</sub>(z) is sampled and held (see the "Sampling the feedback-linearizing force" card); at T<sub>s</sub> = 0.001 s it is ~7%. Replace F<sub>fl</sub>(z) by the constant F<sub>e</sub> to see why it matters: the uncancelled m₁g z̃ term shifts the loop poles.' },
          ],
        }),
        WB.myCtrl.part(ctx, {
          id: 'f', title: `(f) Saturate F at F<sub>max</sub> = ${ctx.pModel.F_max} N; tune the outer rise time for the fastest response without saturation on a ${prob.stepZ} m step of z̃<sub>r</sub>`,
          seed: [`${prob.id}/e`],
          html: `Keep ζ = ${prob.zetaTh} in both loops and t<sub>r<sub>θ</sub></sub> = t<sub>r<sub>z</sub></sub>/${prob.M}. The check gives your controller a large error (its output must stay within ±F<sub>max</sub>), then simulates a ${prob.stepZ} m step of z̃<sub>r</sub> with the block at rest at z<sub>e</sub>: your peak |F| must reach 95% of F<sub>max</sub>, and F may sit at the limit for at most 2 samples.`,
          check: async (code) => {
            const p = ctx.pModel, Fmax = ctx.sys.uLimit(p), z0 = ctx.sys.ze(p);
            const pr = await WB.myCtrl.probe(ctx, code, [[z0 + 5, [z0, 0, 0, 0]], [z0 - 5, [z0, 0, 0, 0]]]);
            if (pr.ok === false) return pr;
            const big = pr.u.map((u) => u[0]).find((u) => Math.abs(u) > Fmax * (1 + 1e-9));
            if (big !== undefined) return { ok: false, msg: `For a 5 m error from rest your controller returns F = ${fmt(big, 4)} N. Saturate its output at ±F_max (P.F_max).` };
            const sc = WB.myCtrl.scenario(ctx, { ref: STEP(prob.stepZ), init: { z0 }, tEnd: 8, feed: 'state' });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            let peak = 0, nAt = 0;
            for (const u of res.uDemand) { peak = Math.max(peak, Math.abs(u)); if (Math.abs(u) >= Fmax * (1 - 1e-6)) nAt++; }
            const pct = (100 * peak / Fmax).toFixed(1);
            if (nAt > 2) return { ok: false, msg: `F sits at the limit for ${nAt} samples, so the input saturates. Slow it down.` };
            if (peak < 0.95 * Fmax) return { ok: false, msg: `Peak |F| is ${pct}% of Fmax. You can go faster.` };
            return { ok: true, msg: `Peak |F| is ${pct}% of Fmax${nAt ? `, at the limit for ${nAt} sample${nAt > 1 ? 's' : ''}` : ''}.` };
          },
          solution: () => {
            const k = { trTh: 1, zetaTh: prob.zetaTh, M: prob.M, zetaZ: prob.zetaZ, rule: '2.2', step: prob.stepZ };
            const tr = fastestTrZ(ctx, k);
            const p = ctx.pModel, z0 = ctx.sys.ze(p);
            return [
              { html: `The Eq. 8.8 bound alone does not settle it here. Right after the step, ż = θ̇ = θ = 0, so F̃(0⁺) = k<sub>P<sub>θ</sub></sub>k<sub>P<sub>z</sub></sub>·0.25 is <em>negative</em> (the beam tilts down to start the block moving out). On that side the room is F<sub>fl</sub>(z<sub>e</sub>) + F<sub>max</sub> = ${fmt(ctx.sys.Ffl(z0, p) + ctx.sys.uLimit(p), 3)} N. The binding limit is the braking phase: the beam tilts up while the block nears z = ${fmt(z0 + prob.stepZ, 3)} m, where F<sub>fl</sub> = ${fmt(ctx.sys.Ffl(z0 + prob.stepZ, p), 3)} N leaves only ${fmt(ctx.sys.uLimit(p) - ctx.sys.Ffl(z0 + prob.stepZ, p), 3)} N for F̃.` },
              { tex: `\\text{bisection on the simulated peak}:\\quad t_{r_z} \\approx ${tex(tr, 4)}\\,\\text{s}\\;(t_{r_\\theta} = t_{r_z}/${prob.M})` },
              { code: E8_SOL(`tr_z = ${M.fmt(tr * 1.02, 4)}   # just above the fastest t_r,z\n        tr_th = tr_z / ${prob.M}`, true) },
              { html: 'This reads "step of 0.25 on z̃<sub>r</sub>" as z<sub>r</sub> going from z<sub>e</sub> to z<sub>e</sub> + 0.25 with the block starting at z<sub>e</sub>. Starting at z = 0 instead gives t<sub>r<sub>z</sub></sub> ≈ 1.04 s (see ISSUES.md).' },
            ];
          },
        }),
      ]);
    },
  };

  // ------------------------------------------------------------- Chapter 9 --
  CH.ch9 = {
    id: 'ch9', num: 9, tab: 'Ch 9', title: 'System type (nested loops)', pages: 'pp. 137–154',
    defaults(sys) {
      const pr = sys.problems.ch8;
      return { comp: 'fl', ...W0, kIz: 0, trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, rule: '2.2', kIzx: 0, input: 'step', zoom: 'both' };
    },
    simDefaults(sys) { return sys.problems.ch9.sim; },
    gains(ctx) { return ctx.S.mode === 'work' ? workGains(ctx.st) : { ...designed(ctx), kIz: ctx.st.kIzx }; },
    controller(ctx, o) { return E.nestedPID(ctx, ctx.gains, { comp: ctx.st.comp, meas: 'state' }, o); },
    reference: (ctx, base) => WB.pid.shapedReference(ctx, base),   // on z: step, ramp [m/s], parabola r = A t² [m/s²]
    outputSeries: thetaRSeries,

    analysis(ctx) {
      const g = ctx.gains, p = ctx.pModel, b0 = ctx.sys.linear(p).b0;
      const hasI = Math.abs(g.kIz) > 0;
      const MaIn = b0 * g.kPth;                // lim s² P_in C_in, C_in = kP + kD s
      const MaOut = -p.g * g.kPz;              // lim s² P_out C_out (PD)
      return {
        inType: 2, inParab: 1 / MaIn, inDistType: 0, inDist: 1 / g.kPth,
        outType: hasI ? 3 : 2, outParab: hasI ? 0 : 1 / MaOut,
        outDistType: hasI ? 1 : 0, outDist: hasI ? 1 / g.kIz : 1 / g.kPz,
        forceToZ: hasI ? 0 : -1 / (g.kPth * g.kPz),   // whole nested loop, force d [N] → z [m]
        hasI,
      };
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Nested PD / PID', 'pp. 147–149 (B.9)');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Reference on z (amplitude = size [m], slope [m/s] or coefficient [m/s²])',
        options: [{ value: 'step', label: 'step' }, { value: 'ramp', label: 'ramp' }, { value: 'parabola', label: 'parabola' }],
        ...bind(ctx, 'input'),
      });
      if (ctx.S.mode === 'work') workGainSliders(sec, ctx, { kI: true });
      else {
        designKnobs(sec, ctx, { rule: false });
        E.knob(sec, ctx, 'kIzx', 'k<sub>I<sub>z</sub></sub>', -0.002, 0, 0.000005, { unit: 'rad/(m·s)' });
        gainReadout(sec, ctx, ['kPth', 'kDth', 'kPz', 'kDz', 'kIz']);
      }
      const ty = section(parent, 'System type analysis', 'p. 141 · Table 9-1');
      const box = el('div', { class: 'metrics' });
      ty.append(box);
      WB.ui.addRefresher(() => this.renderType(box, ctx));
    },

    renderType(box, ctx) {
      const a = this.analysis(ctx), S = ctx.S, st = ctx.st;
      const row = WB.ui.metric;
      const show = WB.ui.shown(ctx, 'E:ch9:type');
      const A = S.sim.amplitude;
      const predZ = st.input === 'parabola' ? 2 * A * a.outParab : 0;  // r = A t² ⇒ R = 2A/s³
      const res = ctx.app.result();
      const n = res ? res.t.length - 1 : 0;
      const eEnd = res ? res.rAll[0][n] - res.yAll[0][n] : NaN;
      const rows = [];
      if (show) {
        rows.push(row('inner loop (θ) tracking type', `type ${a.inType}`));
        rows.push(row('inner input-disturbance type', `type ${a.inDistType}`));
        rows.push(row('outer loop (z) tracking type', `type ${a.outType}`));
        rows.push(row('outer input-disturbance type', `type ${a.outDistType}`));
        rows.push(row(`predicted e_z (${st.input})`, isFinite(predZ) ? `${fmt(predZ, 3)} m` : '∞'));
        rows.push(row(`predicted e_z from force d = ${fmt(S.sim.dist, 3)} N`, `${fmt(a.forceToZ * S.sim.dist, 3)} m`));
      } else {
        rows.push(WB.ui.revealButton(ctx, 'E:ch9:type', 'Reveal the predicted types and errors'));
      }
      rows.push(row('simulated z_r − z at t_end', isFinite(eEnd) ? `${fmt(eEnd, 3)} m` : '—'));
      box.replaceChildren(...rows);
    },

    splane(ctx) { return { markers: loopMarkers(ctx), kindNames: KIND_NAMES }; },

    math(ctx) {
      const a = this.analysis(ctx), e9 = ids(ctx).e9;
      return [
        designModelCard(ctx),
        { title: 'Error constants and system type', page: 'p. 141 · Table 9-1',
          theory: 'M_p = \\lim_{s\\to0} PC,\\quad M_v = \\lim_{s\\to0} sPC,\\quad M_a = \\lim_{s\\to0} s^2PC,\\quad e_{step} = \\tfrac{1}{1+M_p},\\; e_{ramp} = \\tfrac{1}{M_v},\\; e_{parab} = \\tfrac{1}{M_a}' },
        { title: 'Type with respect to a disturbance', page: 'p. 143–145',
          theory: '\\text{type} = \\text{number of free integrators in } PC,\\quad \\text{input disturbance: } E(s) = -\\frac{P}{1 + PC}D_{in}(s),\\quad C_{PID} = \\frac{k_D s^2 + k_P s + k_I}{s}' },
        { title: 'Inner loop of the block and beam under PD', page: 'p. 388 · E.9(a)', answers: [`${e9}/a1`, `${e9}/a2`],
          theory: 'P_{in}C_{in} = \\frac{b_0(k_D s + k_P)}{s^2} \\Rightarrow \\text{type 2},\\; M_a = b_0 k_{P_\\theta};\\quad \\frac{P_{in}}{1 + P_{in}C_{in}}\\Big|_{s\\to0} = \\frac{1}{k_{P_\\theta}} \\Rightarrow \\text{type 0}',
          numbers: `e_{parab} = \\frac{1}{b_0k_{P_\\theta}} = ${tex(a.inParab)},\\quad e_{d,step} = \\frac{1}{k_{P_\\theta}} = ${tex(a.inDist)}\\;\\text{rad/N}` },
        { title: 'Outer loop of the block and beam under PD / PID', page: 'p. 388 · E.9(b)', answers: [`${e9}/b1`, `${e9}/b2`, `${e9}/b3`],
          theory: 'k_I = 0: \\text{type 2},\\; M_a = -gk_{P_z};\\quad k_I \\ne 0: \\text{type 3},\\quad \\frac{P_{out}}{1+P_{out}C_{out}}\\Big|_{s\\to0} = \\frac{1}{k_{P_z}} \\;\\text{(PD, type 0)},\\; \\frac{1}{k_{I_z}}\\frac{1}{s} \\;\\text{(PID, type 1)}',
          numbers: a.hasI ? `\\text{type 3; input-disturbance type 1, ramp error } \\frac{1}{k_{I_z}} = ${tex(a.outDist)}` : `e_{parab} = -\\frac{1}{gk_{P_z}} = ${tex(a.outParab)},\\quad e_{d,step} = \\frac{1}{k_{P_z}} = ${tex(a.outDist)}\\;\\text{m/rad}` },
        { title: 'A force disturbance through both loops', page: 'follows from p. 143–144', answers: `${e9}/b3`,
          theory: '\\tilde F = -d \\Rightarrow \\theta_r = -\\frac{d}{k_{P_\\theta}} \\Rightarrow e_z = -\\frac{d}{k_{P_\\theta}k_{P_z}}\\;(\\text{PD}),\\quad 0\\;(\\text{PID})',
          numbers: `-\\frac{1}{k_{P_\\theta}k_{P_z}} = ${tex(a.forceToZ)}\\;\\text{m/N (current gains)}`,
          note: 'A force d enters the inner loop; at steady state θ = 0 and ż = 0.' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch9;
      const a = () => this.analysis(ctx);
      // Answers are functions of the gains, checked at random gains (inner gains > 0, outer < 0).
      const IN = { kP: { label: 'kPθ', lo: 0.5, hi: 20 }, kD: { label: 'kDθ', lo: 0.1, hi: 5 } };
      const OUT = { kP: { label: 'kPz', lo: -0.2, hi: -0.001 }, kI: { label: 'kIz', lo: -0.01, hi: -1e-5 }, kD: { label: 'kDz', lo: -0.3, hi: -0.005 } };
      const b0 = (p) => ctx.sys.linear(p).b0;
      const signHint = (it, f) => (Math.abs(Number(f.e.got) + Number(f.e.want)) <= 1e-6 * Math.abs(Number(f.e.want)) ? 'Give the magnitude (the outer gains are negative).' : '');
      const check = (code, args, items, extra = {}) => WB.py.check(ctx, { args, items, ...extra }, code);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a1', title: '(a) Inner loop under PD: tracking',
          html: 'Set <code>system_type</code>, and write each steady-state error for a unit input (unit parabola: R(s) = 1/s³) as a function of the inner PD gains. Return <code>np.inf</code> for an unbounded error. The check calls your functions at random gains.',
          code: {
            template: 'system_type = ...\n\ndef e_step(kP, kD):\n    return ...\n\ndef e_ramp(kP, kD):\n    return ...\n\ndef e_parab(kP, kD):\n    return ...\n',
            check: (code) => check(code, IN, [
              { var: 'system_type', truth: () => 2 },
              { fn: 'e_step', args: ['kP', 'kD'], truth: () => 0 },
              { fn: 'e_ramp', args: ['kP', 'kD'], truth: () => 0 },
              { fn: 'e_parab', args: ['kP', 'kD'], truth: (p, x) => 1 / (b0(p) * x.kP) },
            ]),
          },
          solution: () => [
            { tex: `P_{in}C_{in} = \\frac{b_0(k_{D_\\theta}s + k_{P_\\theta})}{s^2} \\Rightarrow \\text{type 2},\\quad e_{parab} = \\frac{1}{b_0 k_{P_\\theta}} = ${tex(a().inParab)}\;\\text{(current gains)}` },
            { html: 'Step and ramp errors are zero. Book convention (pp. 146–152; Notes and References, p. 153): the loop gain with C = k<sub>P</sub> + k<sub>D</sub>s; with the derivative on the output the type is the same but the error values can differ.' },
            { code: 'system_type = 2\n\ndef e_step(kP, kD):\n    return 0.0\n\ndef e_ramp(kP, kD):\n    return 0.0\n\ndef e_parab(kP, kD):\n    b0 = P.ell / (P.m2 * P.ell**2 / 3 + P.m1 * (P.ell / 2)**2)\n    return 1 / (b0 * kP)     # 1/M_a, M_a = b0 kP' },
          ],
        },
        {
          id: 'a2', title: '(a) Inner loop: input disturbance',
          html: 'Set <code>system_type</code> (with respect to the disturbance), and write the steady-state error magnitude per newton of a step d (rad/N) as a function of the inner PD gains.',
          code: {
            template: 'system_type = ...\n\ndef e_dist(kP, kD):\n    return ...\n',
            check: (code) => check(code, IN, [
              { var: 'system_type', truth: () => 0 },
              { fn: 'e_dist', args: ['kP', 'kD'], truth: (p, x) => 1 / x.kP },
            ]),
          },
          solution: () => [
            { tex: `\\lim_{s\\to0}\\frac{P_{in}}{1 + P_{in}C_{in}} = \\lim_{s\\to0}\\frac{b_0}{s^2 + b_0(k_{D_\\theta}s + k_{P_\\theta})} = \\frac{1}{k_{P_\\theta}} = ${tex(a().inDist)} \\Rightarrow \\text{type 0}` },
            { code: 'system_type = 0\n\ndef e_dist(kP, kD):\n    return 1 / kP' },
          ],
        },
        {
          id: 'b1', title: '(b) Outer loop under PD (k<sub>I<sub>z</sub></sub> = 0)',
          html: 'As in (a), for the outer loop with the inner loop as its DC gain, as functions of the outer PD gains (negative). Unit parabola, in m.',
          code: {
            template: 'system_type = ...\n\ndef e_parab(kP, kD):\n    return ...\n',
            check: (code) => check(code, OUT, [
              { var: 'system_type', truth: () => 2 },
              { fn: 'e_parab', args: ['kP', 'kD'], truth: (p, x) => -1 / (p.g * x.kP) },
            ], { explain: signHint }),
          },
          solution: () => [
            { tex: `P_{out}C_{out} = \\frac{-g(k_{D_z}s + k_{P_z})}{s^2} \\Rightarrow \\text{type 2},\\quad e_{parab} = \\frac{1}{M_a} = -\\frac{1}{gk_{P_z}}` },
            { html: 'Step and ramp errors are zero.' },
            { code: 'system_type = 2\n\ndef e_parab(kP, kD):\n    return -1 / (P.g * kP)   # M_a = -g kP > 0' },
          ],
        },
        {
          id: 'b2', title: '(b) Outer loop with an integrator (k<sub>I<sub>z</sub></sub> ≠ 0)',
          code: {
            template: 'system_type = ...\n',
            check: (code) => check(code, {}, [{ var: 'system_type', truth: () => 3 }]),
          },
          solution: () => [
            { tex: 'P_{out}C_{out} = \\frac{-g(k_Ds^2 + k_Ps + k_I)}{s^3} \\Rightarrow \\text{type 3: zero error to steps, ramps and parabolas}' },
            { code: 'system_type = 3' },
          ],
        },
        {
          id: 'b3', title: '(b) Outer loop: input disturbance (on the beam angle)',
          html: 'Disturbance types with PD and with PID, and the PD error magnitude per unit step d (m/rad) as a function of the outer gains.',
          code: {
            template: 'type_pd = ...\ntype_pid = ...\n\ndef e_dist_pd(kP, kD):\n    return ...\n',
            check: (code) => check(code, OUT, [
              { var: 'type_pd', truth: () => 0 },
              { var: 'type_pid', truth: () => 1 },
              { fn: 'e_dist_pd', args: ['kP', 'kD'], truth: (p, x) => Math.abs(1 / x.kP) },
            ], { explain: signHint }),
          },
          solution: () => [
            { tex: '\\text{PD: }\\lim_{s\\to0}\\frac{-g}{s^2 - g(k_{D_z}s + k_{P_z})} = \\frac{1}{k_{P_z}}\;(\\text{type 0})' },
            { tex: '\\text{PID: }\\lim_{s\\to0} s\\frac{-g}{s^3 - g(k_{D_z}s^2 + k_{P_z}s + k_{I_z})}\\frac{1}{s^2} = \\frac{1}{k_{I_z}}\;(\\text{type 1})' },
            { html: 'The "Force disturbance" card shows what this means for a force on the beam: divide again by k<sub>P<sub>θ</sub></sub>.' },
            { code: 'type_pd = 0\ntype_pid = 1\n\ndef e_dist_pd(kP, kD):\n    return abs(1 / kP)' },
          ],
        },
      ]);
    },
  };

  // ------------------------------------------------- Appendix P.6 (root locus) --
  CH.p6 = {
    id: 'p6', num: 10.5, tab: 'App. P.6', short: 'P.6', title: 'Root locus vs. k_I (outer loop)', pages: 'pp. 465–474',
    defaults(sys) {
      const pr = sys.problems.ch8;
      return { comp: 'fl', trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, rule: '2.2', kI: -0.0003, kMaxFactor: 1.5, zoom: 'outer' };
    },
    simDefaults(sys) { return sys.problems.p6.sim; },
    gains(ctx) { return { ...designed(ctx), kIz: ctx.st.kI }; },
    controller(ctx, o) { return E.nestedPID(ctx, ctx.gains, { comp: ctx.st.comp, meas: 'state' }, o); },
    outputSeries: thetaRSeries,

    // 1 + k L(s) = 0 with k = −k_I ≥ 0 and L(s) = g k_DC / (s³ − g k_DC k_Dz s² − g k_DC k_Pz s)
    evans(ctx) {
      const d = designed(ctx), g = ctx.pModel.g;
      const den = [1, -g * d.kDC * d.kDz, -g * d.kDC * d.kPz, 0];
      const num = [g * d.kDC];
      return { d, den, num, kCrit: den[1] * den[2] / num[0] };
    },
    clPoly(ev, kI) { return L.polyAdd(ev.den, L.polyScale(ev.num, -kI)); },

    // The locus starts at the E.8 PD poles: in Work mode it waits for the Evans
    // form (E.P.6) or a Reveal.
    locusShown(ctx) { return ctx.S.mode === 'explore' || ctx.app.isRevealed('E:p6:locus') || ctx.app.isSolved(`${ids(ctx).p6}/a`); },

    buildControls(parent, ctx) {
      const sec = section(parent, 'PD from E.8, then add k_I < 0', 'p. 388 · E.P.6');
      compControl(sec, ctx);
      designKnobs(sec, ctx, { rule: false });
      E.knob(sec, ctx, 'kI', 'k<sub>I<sub>z</sub></sub>', -0.003, 0, 0.000002, { unit: 'rad/(m·s)', sig: 3 });
      E.knob(sec, ctx, 'kMaxFactor', 'locus to', 0.2, 5, 0.1, { unit: '× |kI,crit|', sig: 2 });
      sec.append(el('p', { class: 'muted small', text: 'Drag a closed-loop pole along the locus to set k_I.' }));
      if (ctx.S.mode === 'explore') gainReadout(sec, ctx, ['kPz', 'kDz', 'kIz']);  // E.8(d) answers stay hidden in Work mode
      else if (!this.locusShown(ctx)) {
        sec.append(el('p', { class: 'muted small', text: 'The root locus appears once the Evans form is solved.' }));
        sec.append(WB.ui.revealButton(ctx, 'E:p6:locus', 'Reveal the root locus (uses the E.8 gains)'));
      }
      zoomControl(sec, ctx);
    },

    splane(ctx) {
      if (!this.locusShown(ctx)) return null;
      const ev = this.evans(ctx);
      const kMax = Math.max(ev.kCrit * ctx.st.kMaxFactor, -ctx.st.kI * 1.2, 1e-6);
      const loci = WB.tf.rootLocus(ev.den, ev.num, kMax);
      const markers = L.roots(ev.den).map((q, i) => ({ ...q, kind: 'ol', label: `pole of L(s) ${i + 1}` }));
      L.roots(this.clPoly(ev, ctx.st.kI)).forEach((q, i) => markers.push({ ...q, kind: 'cl', label: `closed-loop pole at kI = ${fmt(ctx.st.kI, 3)}`, dragId: i }));
      const fitR = Math.max(...L.roots(ev.den).map((q) => Math.hypot(q.re, q.im))) * 1.6;
      return { markers, loci, fitR };
    },

    onPoleDrag(ctx, id, re, im) {
      const ev = this.evans(ctx);
      const kMax = Math.max(ev.kCrit * ctx.st.kMaxFactor, 1e-6);
      let best = -ctx.st.kI, bd = Infinity;
      for (let i = 0; i <= 400; i++) {
        const k = kMax * i / 400;
        for (const q of L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, k)))) {
          const d = Math.hypot(q.re - re, q.im - im);
          if (d < bd) { bd = d; best = k; }
        }
      }
      ctx.st.kI = -best;
      ctx.update();
    },

    math(ctx) {
      const ev = this.evans(ctx), p6 = ids(ctx).p6;
      return [
        { title: 'Evans form', page: 'p. 466',
          theory: '\\Delta_{cl}(s) = 0 \\iff 1 + k\\,L(s) = 0 \\quad(k \\text{ is the gain that varies along the locus})' },
        { title: 'PID with the derivative on the output', page: 'p. 470',
          theory: '\\Delta_{cl}(s) = \\text{numerator of } 1 + P\\,C,\\quad C = \\frac{k_D s^2 + k_P s + k_I}{s}' },
        { title: 'Closed outer loop of the block and beam with PID', page: 'p. 388 · E.P.6', answers: [`${p6}/a`, `${ids(ctx).e8}/d`],
          theory: '\\Delta_{cl}(s) = s^3 - gk_{D_z}s^2 - gk_{P_z}s - gk_{I_z}\\quad(\\text{inner loop as } k_{DC_\\theta} = 1)',
          numbers: `\\Delta_{cl}(s) = ${WB.tf.polyTex(this.clPoly(ev, ctx.st.kI))}` },
        { title: 'Evans form of the outer loop (negated, since k_I < 0)', page: 'p. 388 · E.P.6', answers: [`${p6}/a`, `${ids(ctx).e8}/d`],
          theory: '1 + (-k_{I_z})\\,L(s) = 0,\\quad L(s) = \\frac{g}{s^3 - gk_{D_z}s^2 - gk_{P_z}s}',
          numbers: `L(s) = \\frac{${tex(ev.num[0])}}{${WB.tf.polyTex(ev.den)}},\\quad k_{P_z} = ${tex(ev.d.kPz)},\\; k_{D_z} = ${tex(ev.d.kDz)}` },
        { title: 'Where the locus crosses into the RHP', page: 'Routh–Hurwitz (not in the book)',
          theory: 's^3 + c_2 s^2 + c_1 s + c_0 \\text{ stable} \\iff c_i > 0,\\; c_2c_1 > c_0' },
        { title: 'Critical k_I of the outer loop', page: 'Routh–Hurwitz', answers: `${p6}/b`,
          theory: 'k_{I,crit} = -g\\,k_{D_z}k_{P_z}\\;(\\text{stable for } k_{I,crit} < k_I < 0)',
          numbers: `k_{I,crit} = ${tex(-ev.kCrit)}` },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.p6;
      const cx = WB.py.cx;
      // 1 + (−k_I) L(s) = 0 with the inner loop as its DC gain (1): L = −a32/(s³ + a32 kD s² + a32 kP s), a32 = −g.
      const Ltruth = (p, a) => { const a32 = ctx.sys.linear(p).A[2][1]; return cx.div(-a32, cx.poly([1, a32 * a.kD, a32 * a.kP, 0], a.s)); };
      const negOf = (g, w) => { const G = cx.of(g), Wv = cx.of(w); return Math.hypot(G.re + Wv.re, G.im + Wv.im) <= 1e-6 * Math.max(1, Math.hypot(Wv.re, Wv.im)); };
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: 'Characteristic equation in Evans form',
          html: 'Add an integrator to the outer loop, with the inner loop replaced by its DC gain (E.8(c)). k<sub>P</sub>, k<sub>D</sub> are the outer-loop PD gains. Because k<sub>I</sub> is negative, write the Evans form negated, 1 + (−k<sub>I</sub>)L(s) = 0, as the problem says, and return L(s); the check calls it at complex s.',
          code: {
            template: 'def L(s, kP, kD):\n    # 1 + (-kI) * L(s) = 0\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { s: { label: 's', complex: true, re: [-3, 1], im: [0.05, 3] }, kP: { label: 'kP', lo: -0.2, hi: -0.001 }, kD: { label: 'kD', lo: -0.3, hi: -0.005 } },
              params: (p) => ({ ...p, length: p.ell }),
              items: [{ fn: 'L', args: ['s', 'kP', 'kD'], truth: Ltruth }],
              explain: (it, f) => (negOf(f.e.got, f.e.want) ? 'That is the Evans form in k_I, 1 + k_I L(s) = 0. The problem asks for it negated, with −k_I as the gain.' : ''),
            }, code),
          },
          solution: () => {
            const ev = this.evans(ctx);
            return [
              { tex: 's^3 - gk_{D_z}s^2 - gk_{P_z}s - gk_{I_z} = 0 \\;\\Rightarrow\\; 1 + (-k_{I_z})\\frac{g}{s^3 - gk_{D_z}s^2 - gk_{P_z}s} = 0' },
              { tex: `\\text{current PD gains: } L(s) = \\frac{${tex(ev.num[0])}}{${WB.tf.polyTex(ev.den)}}` },
              { code: 'def L(s, kP, kD):\n    g = P.g\n    return g / (s**3 - g * kD * s**2\n                - g * kP * s)' },
              { html: 'Derived like A.P.6 (p. 470): the outer loop with C = k<sub>P</sub> + k<sub>I</sub>/s and the derivative on z, the inner loop as k<sub>DC</sub> = 1, P<sub>out</sub> = −g/s².' },
            ];
          },
        },
        {
          id: 'b', title: 'Root locus: most negative stable k<sub>I</sub>',
          html: 'As a function of the outer PD gains k<sub>P</sub>, k<sub>D</sub> (negative), with the inner loop as its DC gain as in the Evans form. The check calls it at random gains.',
          code: {
            template: 'def kI_crit(kP, kD):\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { kP: { label: 'kP', lo: -0.2, hi: -0.001 }, kD: { label: 'kD', lo: -0.3, hi: -0.005 } },
              params: (p) => ({ ...p, length: p.ell }),
              items: [{ fn: 'kI_crit', args: ['kP', 'kD'], truth: (p, x) => -p.g * x.kD * x.kP }],
            }, code),
          },
          solution: () => [
            { tex: `c_2c_1 > c_0:\; (-gk_{D_z})(-gk_{P_z}) > -gk_{I_z} \\Rightarrow k_{I_z} > -gk_{D_z}k_{P_z} = ${tex(-this.evans(ctx).kCrit)}\;\\text{(E.8 gains)}` },
            { code: 'def kI_crit(kP, kD):\n    return -P.g * kD * kP' },
          ],
        },
        {
          id: 'c', title: 'Select k<sub>I</sub> that does not significantly change the other closed-loop poles',
          html: 'Checks the current k<sub>I</sub>: the complex pair must stay within 10% (in |p|) of the PD-only poles, and the new real pole must be slower than them.',
          check: () => {
            const ev = this.evans(ctx), kI = ctx.st.kI;
            if (!(kI < 0)) return { ok: false, msg: 'Set kI < 0.' };
            const pd = E.outerPoles(ctx.pModel, ev.d);
            const cl = L.roots(this.clPoly(ev, kI));
            const cpx = cl.filter((q) => Math.abs(q.im) > 1e-9), real = cl.filter((q) => Math.abs(q.im) <= 1e-9);
            if (cpx.length !== 2) return { ok: false, msg: 'The PD pair has split; |kI| is too large.' };
            const ratio = Math.hypot(cpx[0].re, cpx[0].im) / Math.hypot(pd[0].re, pd[0].im);
            const slow = real.length === 1 && Math.abs(real[0].re) < Math.abs(pd[0].re);
            return { ok: Math.abs(ratio - 1) < 0.1 && slow, msg: `|p| ratio ${fmt(ratio, 3)}, real pole ${real.length ? fmtPole(real[0]) : '—'}.` };
          },
          solution: () => {
            const ev = this.evans(ctx);
            const cl = L.roots(this.clPoly(ev, -1e-4));
            return [{ html: `With the E.8 gains, k<sub>I</sub> ≈ −1×10⁻⁴ moves the pair by about 7% (poles ${E.poleText(cl)}); anything in roughly (−1.3×10⁻⁴, 0) passes the 10% test. The integrator pole is then very slow (time constant ≈ ${fmt(-1 / cl.find((q) => Math.abs(q.im) < 1e-9).re, 3)} s), so it takes minutes to remove an error. That is the price of the weak E.8 outer loop.` }];
          },
        },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 10 --
  // E.10(c) checks: a constant θ-measurement offset only a z integrator removes, and a
  // step during which a z integrator that integrates all the time winds up.
  const OFFSET_DEG = 0.5, WIND_STEP = 0.15, WIND_OS = 0.01;
  // The E.10(b) test: the book's square wave on the fixed α = 0.2 plant. Resolves to
  // {ok, msg, res}, or a Python error {ok: false, msg, detail}.
  async function trackCheck(ctx, code, prob) {
    const sc = WB.myCtrl.scenario(ctx, { ref: SQUARE(0.15, 0.01), tEnd: 50, mismatch: prob.mismatch });
    const res = await WB.myCtrl.run(ctx, code, sc);
    if (res.ok === false) return res;
    const ob = E.onBeamRes(res, sc.plantParams.ell), e = E.errAt(res, 49.95);
    return { ok: ob.ok && e < 0.002, msg: `${ob.msg}; |z_r − z| before the first switch: ${fmt(1000 * e, 3)} mm.`, res };
  }
  CH.ch10 = {
    id: 'ch10', num: 10, tab: 'Ch 10', title: 'Digital nested PID', pages: 'pp. 155–169',
    defaults(sys) {
      const pr = sys.problems.ch10;
      return {
        comp: 'fl', antiwindup: 'gate', vbar: 0.05, sigma: pr.sigma, extra: 'deriv',
        ...W0, kIz: 0, kIth: 0,
        trTh: 0.15, zetaTh: 0.707, M: 8, zetaZ: 0.707, rule: '2.2', kIzx: -0.05, kIthx: 20, zoom: 'both',
      };
    },
    simDefaults(sys) { return { ...sys.problems.ch10.sim, mismatch: sys.problems.ch10.mismatch }; },
    gains(ctx) { return ctx.S.mode === 'work' ? workGains(ctx.st) : { ...designed(ctx), kIz: ctx.st.kIzx, kIth: ctx.st.kIthx }; },
    opts(ctx) { return { comp: ctx.st.comp, meas: 'dirty', sigma: ctx.st.sigma, antiwindup: ctx.st.antiwindup, vbar: ctx.st.vbar }; },
    controller(ctx, o) { return E.nestedPID(ctx, ctx.gains, this.opts(ctx), o); },
    // Work mode simulates the student's E.10(b)/(c) controller, from the measured z, θ.
    implement: { feed: 'y', linear: false },
    linearSim(ctx, c) { return E.nestedLinearSim(ctx, c, ctx.gains, this.opts(ctx)); },
    linearLabel: 'linear design model (exact parameters)',
    targets(ctx) { return ctx.S.mode === 'explore' ? { tr: designed(ctx).trZ } : {}; },
    outputSeries: thetaRSeries,

    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') {
        workControls(parent, ctx, `${ctx.sys.problems.ch10.id}(b) or (c)`);
        zoomControl(section(parent, 'View'), ctx);
        return;
      }
      const sec = section(parent, 'Nested PID from measured z, θ', 'p. 155, p. 389 · E.10(b)');
      designKnobs(sec, ctx, { rule: false });
      E.knob(sec, ctx, 'kIzx', 'k<sub>I<sub>z</sub></sub>', -0.2, 0, 0.00001, { unit: 'rad/(m·s)' });
      E.knob(sec, ctx, 'kIthx', 'k<sub>I<sub>θ</sub></sub>', 0, 50, 0.01);
      gainReadout(sec, ctx, ['kPth', 'kDth', 'kPz', 'kDz', 'kIz', 'kIth']);
      separationReadout(sec, ctx);
      const imp = section(parent, 'Implementation', 'p. 157 · Eq. 10.3–10.4');
      E.knob(imp, ctx, 'sigma', 'σ', 0.002, 0.5, 0.001, { unit: 's', sig: 3, hint: 'dirty-derivative bandwidth 1/σ rad/s' });
      segmented(imp, {
        label: 'Anti-windup on the z integrator',
        options: [{ value: 'gate', label: 'integrate when |ż| < v̄ (E.10c)' }, { value: 'none', label: 'none' }],
        ...bind(ctx, 'antiwindup'),
      });
      E.knob(imp, ctx, 'vbar', 'v̄', 0.005, 0.5, 0.005, { unit: 'm/s', sig: 3, disabled: () => ctx.st.antiwindup !== 'gate' });
      compControl(imp, ctx);
      segmented(imp, {
        label: 'Extra plot',
        options: [{ value: 'deriv', label: 'ż estimate' }, { value: 'int', label: 'z integrator' }],
        ...bind(ctx, 'extra'),
      });
      zoomControl(imp, ctx);
    },

    splane(ctx) { return { markers: loopMarkers(ctx), kindNames: KIND_NAMES }; },

    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      if (ctx.st.extra === 'int') {
        return { opts: { title: 'z integrator ∫e_z dt', yLabel: '∫e dt [m·s]', unit: 'm·s' },
          data: { series: [{ label: '∫(z_r − z) dt', y: Array.from(res.extras.Iz || []), color: '--series-1' }] } };
      }
      return {
        opts: { title: 'ż(t)', yLabel: 'ż [m/s]', unit: 'm/s' },
        data: {
          series: [
            { label: 'dirty-derivative estimate', y: Array.from(res.extras.zdHat || []), color: '--series-2', width: 1.5 },
            { label: 'true ż', y: res.x.map((x) => x[2]), color: '--series-1' },
          ],
          hlines: ctx.st.antiwindup === 'gate' ? [{ y: ctx.st.vbar, label: '+v̄', color: '--text-muted', fit: false }, { y: -ctx.st.vbar, label: '−v̄', color: '--text-muted', fit: false }] : [],
        },
      };
    },

    math(ctx) {
      const st = ctx.st, Ts = ctx.S.sim.Ts;
      const { beta, gamma } = WB.design.dirtyCoeffs(st.sigma, Ts);
      const d = designed(ctx);
      return [
        { title: 'Nested digital PID', page: 'p. 160 · Listing 10.1, p. 163 (B.10)', answers: `${ids(ctx).e10}/b`,
          theory: '\\theta_r = k_{P_z}e_z + k_{I_z}\\textstyle\\int e_z - k_{D_z}\\dot{\\hat z},\\quad \\tilde F = k_{P_\\theta}(\\theta_r - \\theta) + k_{I_\\theta}\\textstyle\\int e_\\theta - k_{D_\\theta}\\dot{\\hat\\theta},\\quad F = F_{fl}(z) + \\tilde F' },
        { title: 'Dirty derivative', page: 'p. 157 · Eq. 10.4',
          theory: '\\dot{\\hat y}[n] = \\frac{2\\sigma - T_s}{2\\sigma + T_s}\\dot{\\hat y}[n-1] + \\frac{2}{2\\sigma + T_s}\\big(y[n] - y[n-1]\\big)' },
        { title: 'Dirty-derivative coefficients', page: 'p. 389 · E.10(b)', answers: `${ids(ctx).e10}/b`,
          theory: `\\sigma = ${tex(st.sigma)},\\; T_s = ${tex(Ts)}:\\quad \\frac{2\\sigma - T_s}{2\\sigma + T_s} = ${tex(beta)},\\quad \\frac{2}{2\\sigma + T_s} = ${tex(gamma)}` },
        { title: 'Anti-windup when k_I < 0', page: 'p. 157 · §10.1.1, p. 389 · E.10(c)', answers: `${ids(ctx).e10}/c`,
          theory: '\\text{integrate } e_z \\text{ only while } |\\dot{\\hat z}| < \\bar v' },
        { title: 'Gains from the knobs (E.8 formulas)', page: 'p. 387 · E.8', answers: [`${ids(ctx).e8}/b`, `${ids(ctx).e8}/d`],
          theory: 'k_{P_\\theta} = \\frac{\\omega_{n_\\theta}^2}{b_0},\\; k_{D_\\theta} = \\frac{2\\zeta_\\theta\\omega_{n_\\theta}}{b_0},\\quad k_{P_z} = -\\frac{\\omega_{n_z}^2}{g},\\; k_{D_z} = -\\frac{2\\zeta_z\\omega_{n_z}}{g},\\quad \\omega_n = 2.2/t_r',
          numbers: `k_{P_\\theta} = ${tex(d.kPth)},\\; k_{D_\\theta} = ${tex(d.kDth)},\\; k_{P_z} = ${tex(d.kPz)},\\; k_{D_z} = ${tex(d.kDz)},\\quad k_{I,crit} = ${tex(-ctx.pModel.g * d.kDz * d.kPz)}` },
        separationCard(ctx),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch10;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Parameters vary by up to 20%',
          html: 'Set the true plant in the left panel (<em>Randomize ±α</em> with α = 0.2). The chapter starts with a fixed 20% draw (m₁ +12%, m₂ −9%, ℓ +15%) so the page is repeatable.',
          check: () => {
            const mis = Object.values(ctx.S.mismatch || {});
            return mis.some((v) => Math.abs(v) > 0) ? { ok: true, msg: `Mismatch: ${Object.entries(ctx.S.mismatch).map(([k, v]) => `${k} ${fmt(v, 3)}%`).join(', ')}.` } : { ok: false, msg: 'The true plant equals the model. Randomize it.' };
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'b', title: '(b) Implement the nested PID loops of E.8 using only the measured z, θ and z<sub>r</sub>, with dirty derivatives (σ = 0.05); tune the integrators so there is no steady-state error',
          seed: [`${ids(ctx).e8}/f`, `${ids(ctx).e8}/e`],
          html: `Start from your E.8 controller. From here on <code>update(z_r, y)</code> gets the noisy measurement y = (z, θ), not the state. The check runs the book's square wave (0.25 ± 0.15 m, 0.01 Hz) on a plant that differs from the model by ${mis(prob.mismatch)} (a fixed α = 0.2 draw): the block must stay on the beam (0 ≤ z ≤ ℓ of that plant), and |z<sub>r</sub> − z| just before the first switch (t = 50 s) must be under 2 mm.`,
          check: async (code) => {
            const r = await trackCheck(ctx, code, prob);
            return r.res ? { ok: r.ok, msg: r.msg } : r;
          },
          solution: () => [
            { code: E10_SOL(false) },
            { html: 'With the E.8 gains the loop cannot survive even 1% error in m₂: an unmodeled force ΔF tilts the beam by ΔF/k<sub>P<sub>θ</sub></sub> before the outer loop (whose authority is only |k<sub>P<sub>z</sub></sub>| ≈ 0.005 rad per metre of error) can react, and the block slides off (see the E.9 force-disturbance card). No integrator gain fixes that.' },
            { html: 'The loops have to be much faster. This design survives most α = 0.2 draws: t<sub>r<sub>θ</sub></sub> = 0.15 s, M = 8 (t<sub>r<sub>z</sub></sub> = 1.2 s), ζ = 0.707 in both loops, k<sub>I<sub>z</sub></sub> = −0.05 and an inner integrator k<sub>I<sub>θ</sub></sub> = 20 (k<sub>I,crit</sub> ≈ −0.89 for these gains). These are the Explore-mode defaults. With the page\'s fixed draw the block stays between 0.25 m and 0.41 m and the error before the switch is about 0.03 mm. The inner integrator alone already removes a constant force error (at rest θ = 0 forces θ<sub>r</sub> = 0, so z = z<sub>r</sub>); the z integrator is needed for errors that act on z itself (part (c)).' },
            { html: 'Robustness, tested in Python and JS over 90 random draws (each of m₁, m₂, ℓ uniform within ±20%, three seeds of 30), with the |ż| gate of (c): 74 of 90 stay on the beam with the error under 2 mm (23, 26 and 25 per seed). Counting "stays within 0 ≤ z ≤ 0.5 m" instead of the true ℓ, 83 of 90 pass. The 7 draws that diverge all need at least 14.45 N of the 15 N limit just to hold the block at z = 0.4 m (two need more than 15 N), so there is no force left to brake and no gain choice saves them. Nothing in this family passes every draw.' },
          ],
        }),
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) The integrator gain on z is negative; replace the old anti-windup scheme with one that integrates only when |ż| is small',
          seed: [`${prob.id}/b`],
          html: `The check runs (1) the tracking test of (b); (2) a beam-angle measurement that reads ${OFFSET_DEG}° off (a sensor mounted slightly off level), with z<sub>r</sub> = z<sub>e</sub> and exact parameters: only an integrator on z removes the resulting offset, so |z<sub>r</sub> − z| must be under 1 mm at t = 30 s; (3) a ${WIND_STEP} m step of z̃<sub>r</sub> from rest with exact parameters, during which a z integrator that keeps integrating winds up: z may overshoot by at most ${WIND_OS * 1000} mm.`,
          check: async (code) => {
            const r = await trackCheck(ctx, code, prob);
            if (!r.res) return r;
            if (!r.ok) return { ok: false, msg: `Tracking test (b): ${r.msg}` };
            const sc = WB.myCtrl.scenario(ctx, { ref: STEP(0), tEnd: 30 });
            const off = OFFSET_DEG * M.DEG;
            sc.noise = () => [0, off];   // a constant θ measurement offset
            const ro = await WB.myCtrl.run(ctx, code, sc);
            if (ro.ok === false) return ro;
            const eo = E.errAt(ro, 29.95);
            if (!(eo < 0.001)) return { ok: false, msg: `Tracking passes, but with θ measured ${OFFSET_DEG}° off, z is ${fmt(1000 * eo, 3)} mm from z_r at t = 30 s. An integrator on z must remove this offset.` };
            const sw = WB.myCtrl.scenario(ctx, { ref: STEP(WIND_STEP), tEnd: 15 });
            const rw = await WB.myCtrl.run(ctx, code, sw);
            if (rw.ok === false) return rw;
            const os = Math.max(...rw.yAll[0]) - (ctx.sys.refs[0].offset + WIND_STEP);
            const msg = `θ offset: ${fmt(1000 * eo, 3)} mm at 30 s; ${WIND_STEP} m step: overshoot ${fmt(1000 * Math.max(0, os), 3)} mm.`;
            if (!(os <= WIND_OS)) return { ok: false, msg: `${msg} The z integrator winds up during the move.` };
            return { ok: true, msg: `${r.msg} ${msg}` };
          },
          solution: () => [
            { code: E10_SOL(true) },
            { html: 'k<sub>P<sub>z</sub></sub> and k<sub>D<sub>z</sub></sub> are negative because a positive beam angle accelerates the block toward the pivot (z̈ = −gθ), so k<sub>I<sub>z</sub></sub> must be negative as well. The saturation is on F in the inner loop, so the outer integrator never sees it, and holding it while F saturates changes nothing (F saturates for only a few samples). Integrating only while |ż| &lt; 0.05 m/s stops the windup during large moves: without the gate the same gains overshoot the 0.15 m step by about 18 mm (7 mm with it).' },
          ],
        }),
      ]);
    },
  };

  WB.E.pid = { workGains, designed, compControl, workGainSliders, designKnobs, gainReadout, separationReadout, loopMarkers, KIND_NAMES, thetaRSeries, designModelCard, separationCard, peakForTrZ, fastestTrZ };
})();
