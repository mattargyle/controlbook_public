// Study C, chapters 8-10 and Appendix P.6: successive loop closure. The inner
// loop puts PD on the body angle θ (plant 1/((Js+Jp)s²)); the outer loop puts PD
// (or PID) on the panel angle φ and outputs the body reference θ_r, designed with
// the inner loop replaced by its DC gain k_DCθ = 1 (C.8, pp. 130–132).
//
// Modes (as in study A):
//   work    - you set the five gains; anything that answers a problem stays hidden.
//   explore - gains come from (t_rθ, ζ_θ, M, ζ_φ); drag the inner- or outer-loop poles.
(function () {
  const { el, slider, segmented, section } = WB.ui;
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
    const beta = (2 * sigma - Ts) / (2 * sigma + Ts), gamma = 2 / (2 * sigma + Ts);
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
  const LEGEND = { obs: 'inner loop (design)', cl: 'outer loop (design, k_DCθ = 1)', zero: 'full 4-state closed loop', target: 'target (spec)', ol: 'open-loop pole' };

  function cascadeMarkers(ctx, { targets = null, sigma = null, drag = true } = {}) {
    const p = ctx.pModel, g = ctx.gains;
    const { A } = lib().ss(p);
    const explore = ctx.S.mode === 'explore' && drag;
    const inner = lib().innerPoles(p, g), outer = lib().outerPoles(p, g);
    const full = lib().fullLoopPoles(p, g, { sigma });
    const outerR = Math.max(...outer.map((q) => Math.hypot(q.re, q.im)), 0.05);
    const zoomOuter = ctx.st.zoom === 'outer';
    const nf = (q) => zoomOuter && Math.hypot(q.re, q.im) > 3 * outerR;
    const mk = L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `open-loop pole ${i + 1}`, noFit: nf(q) }));
    inner.forEach((q, i) => mk.push({ ...q, kind: 'obs', label: `inner loop pole ${i + 1} (design)`, dragId: explore ? 'in' : undefined, noFit: nf(q) }));
    outer.forEach((q, i) => mk.push({ ...q, kind: 'cl', label: `outer loop pole ${i + 1} (design)`, dragId: explore ? 'out' : undefined }));
    full.forEach((q, i) => mk.push({ ...q, kind: 'zero', label: `full-model closed-loop pole ${i + 1}`, noFit: nf(q) || Math.hypot(q.re, q.im) > 60 }));
    if (targets) targets.forEach((q) => mk.push({ ...q, kind: 'target', label: 'target pole (spec)', noFit: nf(q) }));
    return { markers: mk, legendNames: LEGEND, fitR: zoomOuter ? 2.2 * outerR : undefined };
  }

  // Dragging an inner pole sets (t_rθ, ζ_θ); an outer pole sets (M, ζ_φ).
  function invRule(wn, zeta, rule) { return rule === 'tp' ? 0.5 * Math.PI / (wn * Math.sqrt(Math.max(1e-6, 1 - zeta * zeta))) : 2.2 / wn; }
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
  function workSliders(parent, ctx, withI) {
    const w = ctx.st.w;
    const spec = [['kPth', 'k<sub>P<sub>θ</sub></sub>', 0, 400], ['kDth', 'k<sub>D<sub>θ</sub></sub>', 0, 150], ['kPphi', 'k<sub>P<sub>φ</sub></sub>', -0.5, 5], ['kDphi', 'k<sub>D<sub>φ</sub></sub>', 0, 30]];
    if (withI) spec.push(['kIphi', 'k<sub>I<sub>φ</sub></sub>', 0, 2]);
    for (const [key, label, min, max] of spec) {
      slider(parent, { label, min, max, step: (max - min) / 4000, sig: 4, get: () => w[key], set: (v) => { w[key] = v; ctx.update(); } });
    }
  }
  function designSliders(parent, ctx, { withI = false, withRule = true } = {}) {
    const st = ctx.st;
    slider(parent, { label: 't<sub>r<sub>θ</sub></sub>', unit: 's', min: 0.1, max: 5, step: 0.005, sig: 3, get: () => st.trTh, set: (v) => { st.trTh = v; ctx.update(); } });
    slider(parent, { label: 'ζ<sub>θ</sub>', min: 0.2, max: 0.99, step: 0.005, sig: 3, get: () => st.zetaTh, set: (v) => { st.zetaTh = v; ctx.update(); } });
    slider(parent, { label: 'M = t<sub>r<sub>φ</sub></sub>/t<sub>r<sub>θ</sub></sub>', min: 1, max: 40, step: 0.1, sig: 3, hint: 'bandwidth separation between the loops', get: () => st.M, set: (v) => { st.M = v; ctx.update(); } });
    slider(parent, { label: 'ζ<sub>φ</sub>', min: 0.2, max: 0.99, step: 0.005, sig: 3, get: () => st.zetaPhi, set: (v) => { st.zetaPhi = v; ctx.update(); } });
    if (withRule) {
      segmented(parent, {
        label: 'ω<sub>n</sub> from t<sub>r</sub>',
        options: [{ value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))', title: 'C.8 solution and ctrlPD.py' }, { value: '2.2', label: '2.2 / t<sub>r</sub>', title: 'Eq. 8.5; ctrlPID.py' }],
        get: () => st.rule, set: (v) => { st.rule = v; ctx.update(); },
      });
    }
    if (withI) slider(parent, { label: 'k<sub>I<sub>φ</sub></sub>', min: 0, max: 2, step: 0.001, sig: 3, get: () => st.kIx, set: (v) => { st.kIx = v; ctx.update(); } });
  }
  function readout(parent, ctx, keys) {
    const box = el('div', { class: 'readout wrap' });
    parent.append(box);
    const names = { kPth: 'kPθ', kDth: 'kDθ', kPphi: 'kPφ', kDphi: 'kDφ', kIphi: 'kIφ' };
    WB.ui.addRefresher(() => {
      const g = ctx.gains;
      box.replaceChildren(...keys.map((k) => el('div', {}, el('span', { class: 'ro-label', text: names[k] }), el('strong', { text: fmt(g[k], 4) }))));
    });
  }
  function commonControls(parent, ctx, { ff = true, deriv = false, aw = false } = {}) {
    const st = ctx.st;
    if (ff) {
      segmented(parent, {
        label: 'Feedforward φ<sub>r</sub> into θ<sub>r</sub>',
        options: [{ value: true, label: 'on (Fig. 8-20 p. 133, ctrlPD.py)' }, { value: false, label: 'off (Fig. 8-19)' }],
        get: () => st.ff, set: (v) => { st.ff = v; ctx.update(); },
      });
    }
    slider(parent, { label: '|θ<sub>r</sub>| limit', unit: '°', min: 5, max: 360, step: 1, sig: 3, hint: 'saturation of the outer loop\'s output (θ_max in the repo)', get: () => st.thetaMaxDeg, set: (v) => { st.thetaMaxDeg = v; ctx.update(); } });
    if (deriv) {
      segmented(parent, {
        label: 'θ̇, φ̇ for the D terms',
        options: [{ value: 'dirty', label: 'dirty derivative of y' }, { value: 'state', label: 'true state' }],
        get: () => st.deriv, set: (v) => { st.deriv = v; ctx.update(); },
      });
      slider(parent, { label: 'σ', unit: 's', min: 0.005, max: 0.5, step: 0.001, sig: 3, hint: 'dirty-derivative bandwidth is 1/σ rad/s', get: () => st.sigma, set: (v) => { st.sigma = v; ctx.update(); }, disabled: () => st.deriv !== 'dirty' });
    }
    if (aw) {
      segmented(parent, {
        label: 'Anti-windup',
        options: [{ value: 'repo', label: 'u<sub>I</sub> += (T<sub>s</sub>/k<sub>I</sub>)(θ<sub>r</sub> − θ<sub>r,unsat</sub>)', title: 'ctrlPID.py / Listing 10.4' }, { value: 'none', label: 'none' }],
        get: () => st.antiwindup, set: (v) => { st.antiwindup = v; ctx.update(); },
      });
    }
    segmented(parent, {
      label: 's-plane view',
      options: [{ value: 'all', label: 'all poles' }, { value: 'outer', label: 'zoom on outer loop' }],
      get: () => st.zoom, set: (v) => { st.zoom = v; ctx.update(); },
    });
  }

  // Loads the C.8 design (t_rθ = 1 s, M = 10, ζ = 0.9) into the Work-mode sliders,
  // keeping the current k_Iφ. Used where C.8's gains are given data (C.9, C.P.6).
  function loadC8Button(parent, ctx) {
    parent.append(el('div', { class: 'btn-row' }, el('button', {
      type: 'button', class: 'btn btn-quiet', text: 'Load the C.8 gains',
      onclick: () => {
        const pr = ctx.sys.problems.ch8;
        const g = designOf(ctx, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule });
        Object.assign(ctx.st.w, { kPth: g.kPth, kDth: g.kDth, kPphi: g.kPphi, kDphi: g.kDphi });
        ctx.update();
      },
    })));
  }

  // Work mode: the spec's target poles are an answer, so they stay hidden until asked for.
  function targetToggle(parent, ctx) {
    segmented(parent, {
      label: 'Target poles from the spec (s-plane rings)',
      options: [{ value: false, label: 'hidden' }, { value: true, label: 'shown' }],
      get: () => !!ctx.st.showTargets, set: (v) => { ctx.st.showTargets = v; ctx.update(); },
    });
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
      const row = (l, v, ok) => {
        const r = el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: l }), el('strong', { text: v }));
        if (ok !== undefined) r.append(el('span', { class: 'status ' + (ok ? 'good' : 'bad') }, el('span', { class: 'status-icon', 'aria-hidden': 'true', text: ok ? '✓' : '✗' }), el('span', { text: ok ? 'stable' : 'unstable' })));
        return r;
      };
      box.replaceChildren(
        row('|p_inner| / |p_outer| (slowest inner ÷ fastest outer)', isFinite(wi / wo) ? `${fmt(wi / wo, 3)}×` : '—'),
        row('full 4-state loop', full.length ? `slowest pole ${fmtPole(full.reduce((a, q) => (q.re > a.re ? q : a)))}` : '—', stable),
      );
    });
  }

  // ------------------------------------------------------------ math cards --
  // Theory lines are the general book forms; anything specific to the satellite
  // (and so an answer to some part) goes into symbolic/numbers, hidden in Work mode.
  function innerCards(ctx, g, { spoiler = true } = {}) {
    const p = ctx.pModel, J = p.Js + p.Jp;
    const pin = lib().innerPoles(p, g);
    return {
      title: 'Inner loop (body angle)', page: 'p. 130 · Fig. 8-18',
      theory: 'P_{in}(s) = \\frac{1}{(J_s + J_p)s^2}\\;(\\text{C.5}),\\quad \\tau = k_P(\\theta_r - \\theta) - k_D\\dot\\theta \\;(\\text{Fig. 7-2})',
      symbolic: '\\frac{\\Theta}{\\Theta_r} = \\frac{\\frac{k_{P_\\theta}}{J_s + J_p}}{s^2 + \\frac{k_{D_\\theta}}{J_s + J_p}s + \\frac{k_{P_\\theta}}{J_s + J_p}},\\quad k_{DC_\\theta} = 1',
      numbers: `\\Delta_{in}(s) = s^2 + ${tex(g.kDth / J)}\\,s + ${tex(g.kPth / J)},\\quad p_{in} = ${pin.map((q) => texPole(q)).join(',\\;')}`,
      spoiler,
    };
  }
  function outerCard(ctx, g, { spoiler = true, withI = false } = {}) {
    const p = ctx.pModel;
    const poly = lib().outerCharPoly(p, g);
    return {
      title: withI ? 'Outer loop with PID (inner loop → k_DCθ)' : 'Outer loop (inner loop → k_DCθ)', page: withI ? 'p. 472 · Fig. 6-10' : 'p. 131 · Fig. 8-19, Eq. 8.14',
      theory: withI
        ? '\\theta_r = k_{DC_\\theta}\\Big[\\big(k_P + \\tfrac{k_I}{s}\\big)(\\Phi_r - \\Phi) - k_Ds\\Phi\\Big],\\quad \\Phi = P_{out}(s)\\,\\theta_r'
        : '\\theta_r = k_{DC_\\theta}\\big[k_P(\\Phi_r - \\Phi) - k_Ds\\Phi\\big],\\quad \\Phi = P_{out}(s)\\,\\theta_r',
      symbolic: withI
        ? '\\Delta_{out} = (J_p + bk_{DC}k_D)s^3 + (b + bk_{DC}k_P + kk_{DC}k_D)s^2\\quad + (k + kk_{DC}k_P + bk_{DC}k_I)s + kk_{DC}k_I'
        : '\\Delta_{out}(s)\\,\\Phi = (bk_{DC}k_Ps + kk_{DC}k_P)\\,\\Phi_r,\\quad \\Delta_{out} = (J_p + bk_{DC}k_D)s^2\\quad + (b + bk_{DC}k_P + kk_{DC}k_D)s + (k + kk_{DC}k_P)',
      numbers: `\\Delta_{out}(s) = ${T.polyTex(poly)},\\quad p_{out} = ${L.roots(poly).map((q) => texPole(q)).join(',\\;')}`,
      spoiler,
    };
  }
  function designCards(ctx, d) {
    const st = ctx.st;
    const wTex = st.rule === 'tp' ? '\\omega_n = \\frac{1}{2}\\frac{\\pi}{t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}';
    return [
      { title: 'Inner-loop gains', page: 'p. 131',
        theory: `${wTex},\\quad \\text{match } \\Delta_{in}(s) \\text{ to } s^2 + 2\\zeta\\omega_n s + \\omega_n^2`,
        symbolic: 'k_{P_\\theta} = \\omega_{n_\\theta}^2(J_s + J_p),\\quad k_{D_\\theta} = 2\\zeta_\\theta\\omega_{n_\\theta}(J_s + J_p)',
        numbers: `\\omega_{n_\\theta} = ${tex(d.wnTh)},\\quad k_{P_\\theta} = ${tex(d.kPth)},\\quad k_{D_\\theta} = ${tex(d.kDth)}`, spoiler: true },
      { title: 'Outer-loop gains', page: 'p. 132',
        theory: 't_{r_\\phi} = M t_{r_\\theta},\\quad \\text{match } \\Delta_{out}(s) \\text{ to } s^2 + 2\\zeta_\\phi\\omega_{n_\\phi}s + \\omega_{n_\\phi}^2',
        symbolic: '\\begin{bmatrix}kk_{DC} & -bk_{DC}\\omega_{n_\\phi}^2\\\\ bk_{DC} & kk_{DC} - 2bk_{DC}\\zeta_\\phi\\omega_{n_\\phi}\\end{bmatrix}\\begin{bmatrix}k_{P_\\phi}\\\\ k_{D_\\phi}\\end{bmatrix} = \\begin{bmatrix}-k + J_p\\omega_{n_\\phi}^2\\\\ -b + 2J_p\\zeta_\\phi\\omega_{n_\\phi}\\end{bmatrix}',
        numbers: `t_{r_\\phi} = ${tex(d.trPhi)},\\; \\omega_{n_\\phi} = ${tex(d.wnPhi)}\\quad \\Rightarrow k_{P_\\phi} = ${tex(d.kPphi)},\\; k_{D_\\phi} = ${tex(d.kDphi)}`, spoiler: true },
    ];
  }
  function dcCard(ctx, g) {
    const p = ctx.pModel;
    const kdc = p.k * g.kPphi / (p.k + p.k * g.kPphi);
    return {
      title: 'Outer-loop DC gain and the feedforward', page: 'p. 132, p. 133 · Fig. 8-20',
      theory: 'k_{DC} = \\lim_{s\\to0} T(s),\\quad \\theta_r = k_{P_\\phi}(\\phi_r - \\phi) - k_{D_\\phi}\\dot\\phi + \\phi_r \\;(\\text{Fig. 8-20})',
      symbolic: 'k_{DC_\\phi} = \\frac{kk_{DC_\\theta}k_{P_\\phi}}{k + kk_{DC_\\theta}k_{P_\\phi}} < 1',
      numbers: `k_{DC_\\phi} = ${tex(kdc)}`, spoiler: true,
      note: 'The feedforward term is the book\'s fix for an outer DC gain below one.',
    };
  }

  // ------------------------------------------------------------- C.8 --
  CH.ch8 = {
    id: 'ch8', num: 8, tab: 'Ch 8', title: 'Successive loop closure (PD)', pages: 'pp. 129–134',
    controller: (ctx) => makeCascade(ctx),
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
    linearLabel: 'outer design model (inner loop → 1)',
    linearSim: designOverlay,
    outputSeries(ctx, res, sc, oi) {
      if (oi !== 0 || !res.extras.thetaR) return [];
      return [{ label: 'θ_r (outer-loop output)', y: sc(res.extras.thetaR), color: '--series-2', dash: [4, 3], width: 1.5 }];
    },

    // Problem's spec design (always the book's numbers, independent of the knobs).
    spec(ctx) { const pr = ctx.sys.problems.ch8; return designOf(ctx, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule }); },

    buildControls(parent, ctx) {
      const sec = section(parent, 'PD inner loop, PD outer loop', 'p. 129 · C.8(a)');
      commonControls(sec, ctx, { ff: true });
      if (ctx.S.mode === 'work') {
        workSliders(sec, ctx, false);
        targetToggle(sec, ctx);
      } else {
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
      const s = this.spec(ctx);
      const tg = ctx.S.mode === 'work' && ctx.st.showTargets ? [...lib().innerPoles(ctx.pModel, s), ...lib().outerPoles(ctx.pModel, s)] : null;
      return cascadeMarkers(ctx, { targets: tg });
    },
    onPoleDrag: onCascadeDrag,

    math(ctx) {
      const g = ctx.gains;
      const d = ctx.S.mode === 'work' ? this.spec(ctx) : designOf(ctx);
      return [innerCards(ctx, g), ...designCards(ctx, d), outerCard(ctx, g), dcCard(ctx, g),
        { title: 'Why successive loop closure works', page: 'p. 129, p. 130',
          theory: '\\text{inner loop much faster than outer:}\\; \\frac{\\Theta}{\\Theta_r} \\approx k_{DC_\\theta} \\text{ over the outer loop\'s bandwidth},\\quad t_{r_\\phi} = M\\,t_{r_\\theta},\\; M \\approx 5\\text{–}10',
          note: 'The book replaces the inner loop by its DC gain. The circles in the s-plane show what the full model actually does with these gains.' }];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch8;
      const s = () => this.spec(ctx);
      // Peak demanded |τ| for a satStepDeg step from rest, designed from trTh (M = 10).
      const TAU_MAX = ctx.sys.params.find((q) => q.key === 'tau_max').value;   // the problem's 5 N·m
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
      const useGains = {
        label: 'Use my gains',
        run: (v) => {
          const vals = ['kPth', 'kDth', 'kPphi', 'kDphi'].map((k) => PD().num(v[k]));
          if (vals.some((x) => x === null)) return { ok: false, msg: 'Enter all four gains in (b) and (d) first.' };
          ctx.app.setMode('work');
          Object.assign(ctx.st.w, { kPth: vals[0], kDth: vals[1], kPphi: vals[2], kDphi: vals[3], kIphi: 0 });
          ctx.update(); return null;
        },
      };
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'b', title: `(b) Inner loop: t<sub>r<sub>θ</sub></sub> = ${prob.trTh} s, ζ<sub>θ</sub> = ${prob.zetaTh}`,
          html: 'Use ω<sub>n</sub> = π/(2 t<sub>r</sub>√(1−ζ²)) as the C.8 solution does (p. 131).',
          inputs: { wn: 'ω<sub>n<sub>θ</sub></sub>', kPth: 'k<sub>P<sub>θ</sub></sub>', kDth: 'k<sub>D<sub>θ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { wn: s().wnTh, kPth: s().kPth, kDth: s().kDth }, { wn: 'ωnθ', kPth: 'kPθ', kDth: 'kDθ' }),
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
          id: 'd', title: `(d) Outer loop: t<sub>r<sub>φ</sub></sub> = ${prob.M} t<sub>r<sub>θ</sub></sub>, ζ<sub>φ</sub> = ${prob.zetaPhi}`,
          inputs: { kPphi: 'k<sub>P<sub>φ</sub></sub>', kDphi: 'k<sub>D<sub>φ</sub></sub>', kdc: 'k<sub>DC<sub>φ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { kPphi: s().kPphi, kDphi: s().kDphi, kdc: s().kDCphi }, { kPphi: 'kPφ', kDphi: 'kDφ', kdc: 'kDCφ' }),
          actions: [useGains],
          solution: () => [
            { tex: `\\omega_{n_\\phi} = ${tex(s().wnPhi)},\\quad \\begin{bmatrix}k_{P_\\phi}\\\\ k_{D_\\phi}\\end{bmatrix} = ${M.texMat(s().AA)}^{-1}${M.texMat(s().bb)} = \\begin{bmatrix}${tex(s().kPphi)}\\\\ ${tex(s().kDphi)}\\end{bmatrix}` },
            { tex: `k_{DC_\\phi} = \\frac{k\\,k_{P_\\phi}}{k + k\\,k_{P_\\phi}} = ${tex(s().kDCphi)}` },
            { html: 'Book: (0.834, 8.253) and 0.455 (p. 132). The k<sub>D<sub>φ</sub></sub> printed there differs from what its own formula gives; see ISSUES.md.' },
          ],
        },
        {
          id: 'e', title: '(e) Simulate the 15° square wave',
          html: 'Click <em>Use my gains</em> in (d) (it also uses your (b) gains). Saturation only enters in part (f): with t<sub>r<sub>θ</sub></sub> = 1 s the first sample demands about 37 N·m, so raise τ<sub>max</sub> in the left panel (up to 100) to run (e) unsaturated. Passes when the simulated design poles match the C.8 targets.',
          check: () => {
            if (ctx.S.mode !== 'work') return { ok: false, msg: 'Switch to Work mode so the simulation uses your gains.' };
            const p = ctx.pModel, g = ctx.gains, sp = s();
            const ok = M.polesMatch(lib().innerPoles(p, g), lib().innerPoles(p, sp), 0.02, 0.01) && M.polesMatch(lib().outerPoles(p, g), lib().outerPoles(p, sp), 0.02, 0.005);
            return ok ? { ok, msg: 'Both loops sit on their targets. Compare the φ trace with the dashed design model.' } : { ok, msg: 'At least one loop is off its target rings.' };
          },
        },
        {
          id: 'f', title: `(f) Fastest t<sub>r<sub>θ</sub></sub> (with t<sub>r<sub>φ</sub></sub> = ${prob.M} t<sub>r<sub>θ</sub></sub>) that does not saturate τ on a ${prob.satStepDeg}° step`,
          inputs: { tr: 't<sub>r<sub>θ</sub></sub> [s]' },
          html: `Checked by simulation: a ${prob.satStepDeg}° step on φ<sub>r</sub> from rest, ζ = 0.9 in both loops, feedforward on and |θ<sub>r</sub>| ≤ 30° as in ctrlPD.py. The peak demanded |τ| should be 95–100% of τ<sub>max</sub>.`,
          check: (v) => {
            const tr = PD().num(v.tr);
            if (tr === null || tr <= 0) return { ok: false, msg: 'Enter a positive rise time.' };
            const pk = peakFor(tr), pct = (100 * pk).toFixed(1);
            if (pk > 1.0005) return { ok: false, msg: `Peak demand is ${pct}% of τmax, so it saturates. Slow it down.` };
            if (pk < 0.95) return { ok: false, msg: `Peak demand is ${pct}% of τmax. You can go faster.` };
            return { ok: true, msg: `Peak demand is ${pct}% of τmax.` };
          },
          actions: [{
            label: 'Try it',
            run: (v) => {
              const tr = PD().num(v.tr);
              if (tr === null || tr <= 0) return { ok: false, msg: 'Enter a positive rise time.' };
              ctx.app.setMode('explore');
              Object.assign(ctx.st, { trTh: tr, zetaTh: prob.zetaTh, M: prob.M, zetaPhi: prob.zetaPhi, rule: prob.rule, ff: true, thetaMaxDeg: prob.thetaMaxDeg });
              Object.assign(ctx.S.sim, { type: 'step', amplitude: prob.satStepDeg, tStep: 0, y0: 0, tEnd: Math.max(ctx.S.sim.tEnd, 8 * prob.M * tr) });
              ctx.S.sim.init.phi0 = 0;
              ctx.update(); return null;
            },
          }],
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
              { html: `ctrlPD.py uses t<sub>r<sub>θ</sub></sub> = ${prob.repoTr} s ("tuned to not saturate the input"); with it the step demands ${(100 * peakFor(prob.repoTr)).toFixed(0)}% of τ<sub>max</sub>.` },
            ];
          },
        },
      ]);
    },
  };

  // ------------------------------------------------------------- C.9 --
  // Reference shapes on φ_r: step, ramp, parabola (amplitude = size, slope, coefficient).
  function shapedReference(ctx, base) {
    const S = ctx.S;
    const A = S.sim.amplitude * DEG, t0 = S.sim.tStep, y0 = (S.sim.init.phi0 ?? 0) * DEG;
    if (ctx.st.input === 'ramp') return (t) => (t < t0 ? y0 : y0 + A * (t - t0));
    if (ctx.st.input === 'parabola') return (t) => (t < t0 ? y0 : y0 + A * (t - t0) ** 2);
    return base;
  }

  CH.ch9 = {
    id: 'ch9', num: 9, tab: 'Ch 9', title: 'System type & integrators', pages: 'pp. 150–153',
    controller: (ctx) => makeCascade(ctx),
    reference: shapedReference,
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
        get: () => ctx.st.input, set: (v) => { ctx.st.input = v; ctx.update(); },
      });
      commonControls(sec, ctx, { ff: true });
      if (ctx.S.mode === 'work') {
        workSliders(sec, ctx, true);
        loadC8Button(sec, ctx);
        sec.append(el('p', { class: 'muted small', text: 'C.9 analyzes the C.8 loops; load them or use your own. The |θ_r| limit is opened up so ramps are not clipped.' }));
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
      const row = (l, v) => el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: l }), el('strong', { text: v }));
      const show = S.mode === 'explore' || ctx.app.isRevealed('C:ch9:type');
      const rows = [];
      if (show) {
        const pr = this.predicted(ctx);
        rows.push(row('inner loop vs. θ_r / vs. d', `type ${a.inner.type} / type ${a.inner.dType}`));
        rows.push(row('outer loop vs. φ_r / vs. d₂ (k_DCθ = 1)', `type ${a.outer.type} / type ${a.outer.dType}`));
        rows.push(row(`predicted φ_r − φ (${ctx.st.input}${ctx.st.ff ? ', feedforward' : ''}, d = ${fmt(S.sim.dist, 3)})`, isFinite(pr) ? `${fmt(pr * R2D, 3)}°` : '∞ (grows)'));
      } else {
        rows.push(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reveal the predicted types and errors', onclick: () => { ctx.app.reveal('C:ch9:type'); WB.ui.refreshAll(); } }));
      }
      rows.push(row('simulated φ_r − φ at t_end', isFinite(eEnd) ? `${fmt(eEnd, 3)}°` : '—'));
      box.replaceChildren(...rows);
    },

    splane(ctx) { return cascadeMarkers(ctx, { drag: false }); },

    math(ctx) {
      const a = this.analysis(ctx), g = ctx.gains;
      return [
        { title: 'Inner loop (Fig. 9-11)', page: 'p. 151',
          theory: 'P(s)C(s) = \\frac{k_D s + k_P}{(J_s+J_p)s^2}\\;(\\text{PD on the error, Fig. 9-11}),\\quad \\text{type} = \\text{free integrators in } PC',
          symbolic: '\\text{type 2},\\quad e_{step} = e_{ramp} = 0,\\; e_{parab} = \\frac{J_s+J_p}{k_P},\\quad \\text{input disturbance: type 0},\\; e = \\frac{1}{k_P}',
          numbers: `e_{parab} = ${tex(a.inner.parab)},\\quad e_{d} = \\frac{1}{k_{P_\\theta}} = ${tex(a.inner.dStep)},\\quad \\text{derivative on }\\theta\\text{ (ctrlPD.py): type 1},\\; e_{ramp} = \\frac{k_{D_\\theta}}{k_{P_\\theta}} = ${tex(g.kDth / g.kPth)}`, spoiler: true,
          note: 'The repo controllers differentiate θ, not the error (Fig. 7-2). That changes the reference type of this loop; see the solution.' },
        { title: 'Outer loop (Fig. 9-12)', page: 'p. 151–152',
          theory: 'P(s)C(s) = \\frac{\\frac{b}{J_p}s + \\frac{k}{J_p}}{s^2 + \\frac{b}{J_p}s + \\frac{k}{J_p}}\\cdot\\frac{k_Ds^2 + k_Ps + k_I}{s}',
          symbolic: 'P(0) = 1:\\quad k_I = 0 \\Rightarrow \\text{type 0},\\; e_{step} = \\frac{1}{1 + k_P};\\quad k_I > 0 \\Rightarrow \\text{type 1},\\; e_{ramp} = \\frac{1}{k_I}',
          numbers: a.hasI ? `e_{ramp} = \\frac{1}{k_I} = ${tex(a.outer.ramp)}` : `e_{step} = \\frac{1}{1 + k_P} = ${tex(a.outer.step)}`, spoiler: true },
        { title: 'Disturbance at the outer plant input', page: 'p. 152–153',
          theory: '\\lim_{t\\to\\infty} e = \\lim_{s\\to0} s\\frac{P}{1+PC}\\frac{1}{s^{q+1}}',
          symbolic: 'k_I = 0 \\Rightarrow \\frac{1}{1+k_P}\\;(q = 0),\\quad k_I \\ne 0 \\Rightarrow \\frac{1}{k_I}\\;(q = 1)', spoiler: true,
          note: 'In the simulation, d acts on the body torque; it reaches the outer loop through the inner loop\'s steady-state offset.' },
        { title: 'Final value theorem', page: 'p. 137',
          theory: '\\lim_{t\\to\\infty} e(t) = \\lim_{s\\to 0} sE(s),\\quad E = \\frac{1}{1 + PC}R' },
        { title: 'Feedforward', page: 'p. 133 · Fig. 8-20',
          theory: '\\theta_r = k_P(\\phi_r - \\phi) - k_D\\dot\\phi + \\phi_r',
          symbolic: '\\phi_{ss} = \\phi_r \\text{ for a step, even with } k_I = 0', spoiler: true,
          note: `The C.9 analysis has no feedforward, so it is off by default here. Current gains: kPφ = ${fmt(g.kPphi, 4)}, kIφ = ${fmt(g.kIphi, 4)}.` },
      ];
    },

    buildProblem(parent, ctx) {
      const a = () => this.analysis(ctx);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch9, [
        {
          id: 'a', title: '(a) Inner loop with PD',
          html: 'Answers use the current k<sub>P<sub>θ</sub></sub>, k<sub>D<sub>θ</sub></sub>. Give the reference type and the error for the lowest-order input with a finite nonzero error (unit input, R = 1/s<sup>q+1</sup> as in Table 9-1, rad), then the disturbance type and the error for a unit step d.',
          inputs: { type: 'type vs. θ<sub>r</sub>', err: 'e (finite, nonzero)', dtype: 'type vs. d', dstep: 'e from unit step d' },
          check: (v) => {
            const A = a(), g = ctx.gains;
            const t = PD().num(v.type);
            const d = PD().checkNumbers({ dtype: v.dtype, dstep: v.dstep }, { dtype: 0, dstep: A.inner.dStep }, { dtype: 'disturbance type', dstep: 'e from d' });
            if (!d.ok) return d;
            if (t === 2) {
              const r = PD().checkNumbers({ err: v.err }, { err: A.inner.parab }, { err: 'e_parab' });
              return r.ok ? { ok: true, msg: 'The book\'s answer, for PD on the error (Fig. 9-11). The repo differentiates θ instead, which makes it type 1 (see the solution).' } : r;
            }
            if (t === 1) {
              const r = PD().checkNumbers({ err: v.err }, { err: g.kDth / g.kPth }, { err: 'e_ramp' });
              return r.ok ? { ok: true, msg: 'Right for the implemented controller (derivative on θ, ctrlPD.py). The book answers type 2 for PD on the error (Fig. 9-11).' } : r;
            }
            return { ok: false, msg: 'Count the free integrators in the loop (and mind where the derivative acts).' };
          },
          solution: () => [
            { tex: `\\text{Fig. 9-11 (PD on the error)}:\\; PC = \\frac{k_Ds + k_P}{(J_s+J_p)s^2} \\Rightarrow \\text{type 2},\\; e_{parab} = \\frac{J_s+J_p}{k_{P_\\theta}} = ${tex(a().inner.parab)}` },
            { tex: `\\text{derivative on } \\theta \\text{ (ctrlPD.py, ctrlPID.py)}:\\; \\frac{E}{R} = \\frac{(J_s+J_p)s^2 + k_Ds}{(J_s+J_p)s^2 + k_Ds + k_P} \\Rightarrow \\text{type 1},\\; e_{ramp} = \\frac{k_{D_\\theta}}{k_{P_\\theta}} = ${tex(ctx.gains.kDth / ctx.gains.kPth)}` },
            { tex: `\\text{input disturbance (either form): type 0},\\; e = \\frac{1}{k_{P_\\theta}} = ${tex(a().inner.dStep)}` },
            { html: 'Book: p. 151. Its Notes (p. 153) say the type does not change when the derivative moves to the output; for this loop it does, because the plant has no damping of its own.' },
          ],
        },
        {
          id: 'b1', title: '(b) Outer loop with PD (k<sub>I<sub>φ</sub></sub> = 0)',
          inputs: { type: 'type', step: 'e<sub>step</sub>' },
          check: (v) => {
            if (a().hasI) return { ok: false, msg: 'Set kIφ = 0 first.' };
            return PD().checkNumbers(v, { type: 0, step: a().outer.step }, { step: 'e_step' });
          },
          solution: () => [{ tex: `\\text{type 0}:\\; e_{step} = \\frac{1}{1 + k_{P_\\phi}} = ${tex(1 / (1 + ctx.gains.kPphi))},\\; e_{ramp} = \\infty;\\; \\text{disturbance: type 0, } \\frac{1}{1 + k_{P_\\phi}}` }, { html: 'Book: p. 152. Try it: feedforward off, step input, k<sub>I<sub>φ</sub></sub> = 0.' }],
        },
        {
          id: 'b2', title: '(b) Outer loop with an integrator (k<sub>I<sub>φ</sub></sub> > 0)',
          inputs: { type: 'type', ramp: 'e<sub>ramp</sub>' },
          check: (v) => {
            if (!a().hasI) return { ok: false, msg: 'Set kIφ > 0 first.' };
            return PD().checkNumbers(v, { type: 1, ramp: a().outer.ramp }, { ramp: 'e_ramp' });
          },
          solution: () => [{ tex: `\\text{type 1}:\\; e_{step} = 0,\\; e_{ramp} = \\frac{1}{k_{I_\\phi}};\\; \\text{disturbance: type 1, ramp error } \\frac{1}{k_{I_\\phi}}` }, { html: 'Book: p. 152–153. Try a ramp input with k<sub>I<sub>φ</sub></sub> > 0.' }],
        },
      ]);
    },
  };

  // ---------------------------------------------------- Appendix C.P.6 --
  function rootLocus(den, num, kMax, steps = 300) {
    const branches = [];
    let prev = null;
    for (let i = 0; i <= steps; i++) {
      const k = kMax * Math.pow(i / steps, 2);
      let r = L.roots(L.polyAdd(den, L.polyScale(num, k)));
      if (prev) {
        const used = new Set(), ordered = [];
        for (const q0 of prev) {
          let best = -1, bd = Infinity;
          r.forEach((q, j) => { if (!used.has(j)) { const d = Math.hypot(q.re - q0.re, q.im - q0.im); if (d < bd) { bd = d; best = j; } } });
          used.add(best); ordered.push(r[best]);
        }
        r = ordered;
      } else r.forEach(() => branches.push([]));
      r.forEach((q, j) => branches[j].push(q));
      prev = r;
    }
    return branches;
  }

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
    linearLabel: 'outer design model (inner loop → 1)',
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
      slider(sec, { label: 'locus to', unit: '× kI,crit', min: 0.2, max: 5, step: 0.1, sig: 2, get: () => ctx.st.kMaxFactor, set: (v) => { ctx.st.kMaxFactor = v; ctx.update(); } });
      segmented(sec, {
        label: 's-plane view',
        options: [{ value: 'all', label: 'all poles' }, { value: 'outer', label: 'zoom on outer loop' }],
        get: () => ctx.st.zoom, set: (v) => { ctx.st.zoom = v; ctx.update(); },
      });
      sec.append(el('p', { class: 'muted small', text: 'Drag a closed-loop pole along the locus to set k_Iφ. The locus is for the book\'s outer design model; the circles are the full four-state loop with the same gains.' }));
      readout(sec, ctx, ['kPth', 'kDth', 'kPphi', 'kDphi', 'kIphi']);
    },

    splane(ctx) {
      const ev = this.evans(ctx);
      const kMax = Math.max(isFinite(ev.kCrit) ? ev.kCrit * ctx.st.kMaxFactor : 2 * ctx.st.kMaxFactor, (ev.g.kIphi || 0) * 1.2, 1e-3);
      const loci = rootLocus(ev.den, ev.num, kMax);
      const base = cascadeMarkers(ctx, { drag: false });
      const markers = base.markers.filter((m) => m.kind !== 'cl');
      L.roots(ev.den).forEach((q, i) => markers.push({ ...q, kind: 'ol', label: `pole of L(s) ${i + 1}` }));
      L.roots(ev.num).forEach((q) => markers.push({ ...q, kind: 'olzero', label: 'zero of L(s): −k/b' }));
      L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, ev.g.kIphi))).forEach((q, i) => markers.push({ ...q, kind: 'cl', label: `outer closed-loop pole at kIφ = ${fmt(ev.g.kIphi, 3)}`, dragId: i }));
      const fitR = ctx.st.zoom === 'outer' ? Math.max(...L.roots(ev.den).map((q) => Math.hypot(q.re, q.im)), Math.abs(ev.num[1] / ev.num[0])) * 1.4 : undefined;
      return { markers, loci, fitR, legendNames: { ...LEGEND, cl: 'outer closed loop (design model)', olzero: 'zero of L(s)', ol: 'open-loop / L(s) pole' } };
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
        outerCard(ctx, ev.g, { withI: true }),
        { title: 'Evans form', page: 'p. 472–473',
          theory: '\\Delta(s) = D(s) + k_I N(s) \\;\\Rightarrow\\; 1 + k_I\\frac{N(s)}{D(s)} = 0',
          symbolic: '1 + k_I\\frac{k_{DC}(bs + k)}{a_3s^3 + a_2s^2 + a_1s} = 0,\\quad a_3 = J_p + bk_{DC}k_D,\\; a_1 = k + kk_{DC}k_P,\\quad a_2 = b + bk_{DC}k_P + kk_{DC}k_D',
          numbers: `L(s) = \\frac{${T.polyTex(ev.num)}}{${T.polyTex(ev.den)}}`, spoiler: true,
          note: 'The book jumps from the closed-loop transfer function straight to the Matlab command (p. 473); this card fills in the step it leaves out.' },
        { title: 'Where the locus crosses into the RHP', page: 'Routh–Hurwitz (not in the book)',
          theory: 's^3 + d_2s^2 + (d_1 + k_Ic_1)s + k_Ic_0 \\text{ is stable iff}\\quad d_2(d_1 + k_Ic_1) > k_Ic_0 \\;(\\text{all } c_i, d_i > 0)',
          numbers: `k_{I,crit} = \\frac{d_2d_1}{c_0 - d_2c_1} = ${isFinite(ev.kCrit) ? tex(ev.kCrit) : '\\infty'}`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.p6;
      const ev = () => this.evans(ctx, this.specGains(ctx));
      const evNow = () => this.evans(ctx);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: 'Evans form: L(s) = (c<sub>1</sub>s + c<sub>0</sub>) / (s³ + d<sub>2</sub>s² + d<sub>1</sub>s)',
          html: 'For the C.8 PD gains (t<sub>r<sub>θ</sub></sub> = 1 s, M = 10, ζ = 0.9, π/(2t<sub>r</sub>√(1−ζ²))) and k<sub>DC<sub>θ</sub></sub> = 1.',
          inputs: { c1: 'c<sub>1</sub>', c0: 'c<sub>0</sub>', d2: 'd<sub>2</sub>', d1: 'd<sub>1</sub>' },
          check: (v) => { const e = ev(); return PD().checkNumbers(v, { c1: e.num[0], c0: e.num[1], d2: e.den[1], d1: e.den[2] }, {}); },
          solution: () => { const e = ev(); return [{ tex: `L(s) = \\frac{${T.polyTex(e.num)}}{${T.polyTex(e.den)}}` }, { html: 'Divide the Fig. 6-10 characteristic polynomial by its leading coefficient J<sub>p</sub> + bk<sub>DC</sub>k<sub>D<sub>φ</sub></sub> and collect the k<sub>I</sub> terms.' }]; },
        },
        {
          id: 'b', title: 'Largest stable k<sub>I<sub>φ</sub></sub> (design model)',
          inputs: { k: 'k<sub>I,crit</sub>' },
          check: (v) => PD().checkNumbers(v, { k: ev().kCrit }, { k: 'kI,crit' }),
          solution: () => [{ tex: `k_{I,crit} = ${isFinite(ev().kCrit) ? tex(ev().kCrit) : '\\infty'}` }],
        },
        {
          id: 'c', title: 'Pick k<sub>I<sub>φ</sub></sub> that barely moves the PD poles',
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
      ]);
    },
  };

  // ------------------------------------------------------------- C.10 --
  CH.ch10 = {
    id: 'ch10', num: 10, tab: 'Ch 10', title: 'Digital PID (successive loops)', pages: 'pp. 166–169',
    controller: (ctx) => makeCascade(ctx),
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
    linearLabel: 'outer design model (inner loop → 1)',
    linearSim: designOverlay,
    outputSeries(ctx, res, sc, oi) {
      if (oi !== 0 || !res.extras.thetaR) return [];
      return [{ label: 'θ_r (outer-loop output)', y: sc(res.extras.thetaR), color: '--series-2', dash: [4, 3], width: 1.5 }];
    },
    spec(ctx) { const pr = ctx.sys.problems.ch10; return designOf(ctx, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, kIx: pr.ki }); },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Digital PID, measured angles only', 'p. 166 · Listing 10.4');
      if (ctx.S.mode === 'work') { workSliders(sec, ctx, true); targetToggle(sec, ctx); }
      else { designSliders(sec, ctx, { withI: true }); readout(sec, ctx, ['kPth', 'kDth', 'kPphi', 'kDphi', 'kIphi']); }
      const imp = section(parent, 'Implementation', 'p. 157 · Eq. 10.3–10.4');
      commonControls(imp, ctx, { ff: true, deriv: true, aw: true });
      segmented(imp, {
        label: 'Extra plot',
        options: [{ value: 'int', label: 'integrator' }, { value: 'deriv', label: 'rate estimates' }],
        get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); },
      });
      const sep = section(parent, 'Bandwidth separation', 'p. 129');
      separationBox(sep, ctx, () => (ctx.st.deriv === 'dirty' ? ctx.st.sigma : null));
    },

    extraPlot(ctx, res) {
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
      const tg = ctx.S.mode === 'work' && ctx.st.showTargets ? [...lib().innerPoles(ctx.pModel, s), ...lib().outerPoles(ctx.pModel, { ...s, kIphi: 0 })] : null;
      return cascadeMarkers(ctx, { targets: tg, sigma: ctx.st.deriv === 'dirty' ? ctx.st.sigma : null });
    },
    onPoleDrag: onCascadeDrag,

    math(ctx) {
      const st = ctx.st, Ts = ctx.S.sim.Ts;
      const beta = (2 * st.sigma - Ts) / (2 * st.sigma + Ts), gamma = 2 / (2 * st.sigma + Ts);
      const g = ctx.gains, d = ctx.S.mode === 'work' ? this.spec(ctx) : designOf(ctx);
      return [
        innerCards(ctx, g), outerCard(ctx, { ...g, kIphi: 0 }), ...designCards(ctx, d),
        { title: 'Dirty derivative of the measured angles', page: 'p. 157 · Eq. 10.4',
          theory: '\\dot{\\hat y}[n] = \\frac{2\\sigma - T_s}{2\\sigma + T_s}\\dot{\\hat y}[n-1] + \\frac{2}{2\\sigma + T_s}\\big(y[n] - y[n-1]\\big)',
          numbers: `\\sigma = ${tex(st.sigma)},\\; T_s = ${tex(Ts)}:\\quad ${tex(beta)},\\quad ${tex(gamma)}`, spoiler: true },
        { title: 'Outer PID and anti-windup (Listing 10.4)', page: 'p. 167–168',
          theory: '\\theta_r = \\text{sat}\\big(k_{P_\\phi}e + k_{I_\\phi}u_I - k_{D_\\phi}\\dot{\\hat\\phi}\\big),\\quad u_I \\mathrel{+}= \\frac{T_s}{k_{I_\\phi}}(\\theta_r - \\theta_{r,unsat})',
          note: 'The listing uses t_rθ = 0.4 s with ω_n = 2.2/t_r and M = 15, not the C.8 values, and drops C.8\'s φ_r feedforward (the integrator removes the error instead).' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch10;
      const s = () => this.spec(ctx);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'c1', title: `(c) Gains with the listing's tuning: t<sub>r<sub>θ</sub></sub> = ${prob.trTh} s (ω<sub>n</sub> = 2.2/t<sub>r</sub>), ζ = ${prob.zetaTh}, M = ${prob.M}`,
          inputs: { kPth: 'k<sub>P<sub>θ</sub></sub>', kDth: 'k<sub>D<sub>θ</sub></sub>', kPphi: 'k<sub>P<sub>φ</sub></sub>', kDphi: 'k<sub>D<sub>φ</sub></sub>' },
          check: (v) => PD().checkNumbers(v, { kPth: s().kPth, kDth: s().kDth, kPphi: s().kPphi, kDphi: s().kDphi }, { kPth: 'kPθ', kDth: 'kDθ', kPphi: 'kPφ', kDphi: 'kDφ' }),
          actions: [{
            label: 'Use my gains',
            run: (v) => {
              const vals = ['kPth', 'kDth', 'kPphi', 'kDphi'].map((k) => PD().num(v[k]));
              if (vals.some((x) => x === null)) return { ok: false, msg: 'Enter all four gains first.' };
              ctx.app.setMode('work');
              Object.assign(ctx.st.w, { kPth: vals[0], kDth: vals[1], kPphi: vals[2], kDphi: vals[3] });
              ctx.update(); return null;
            },
          }],
          solution: () => [
            { tex: `k_{P_\\theta} = ${tex(s().kPth)},\\; k_{D_\\theta} = ${tex(s().kDth)},\\; k_{P_\\phi} = ${tex(s().kPphi)},\\; k_{D_\\phi} = ${tex(s().kDphi)}` },
            { html: 'Same C.8 procedure, with ctrlPID.py\'s tuning (Listing 10.4, p. 167).' },
          ],
        },
        {
          id: 'c2', title: '(c) Dirty-derivative coefficients for σ = 0.05, T<sub>s</sub> = 0.01',
          inputs: { a: '(2σ−T<sub>s</sub>)/(2σ+T<sub>s</sub>)', b: '2/(2σ+T<sub>s</sub>)' },
          check: (v) => PD().checkNumbers(v, { a: 0.09 / 0.11, b: 2 / 0.11 }, {}),
          solution: () => [{ tex: '\\frac{0.09}{0.11} = 0.8182,\\quad \\frac{2}{0.11} = 18.18' }],
        },
        {
          id: 'c3', title: '(c) Tune k<sub>I<sub>φ</sub></sub> to remove the steady-state error',
          html: 'Checks the current simulation, with the plant mismatch in the left panel. The book only says "tune the integrator"; the workbench asks for |φ<sub>r</sub> − φ| under 0.5° just before the first reference switch (15° step, 25 s later).',
          check: () => {
            const res = ctx.app.result(), S = ctx.S;
            const tSw = S.sim.type === 'square' ? S.sim.tStep + 0.5 / S.sim.frequency : S.sim.tEnd;
            const i = Math.min(res.t.length - 1, Math.round((tSw - 0.05) / S.sim.Ts));
            const e = Math.abs(res.rAll[0][i] - res.yAll[1][i]) * R2D;
            if (!(ctx.gains.kIphi > 0)) return { ok: false, msg: `kIφ = 0: error before the switch is ${fmt(e, 3)}°.` };
            return { ok: e < 0.5, msg: `Error before the switch: ${fmt(e, 3)}° (workbench limit 0.5°).` };
          },
          solution: () => [{ html: `The C.10 solution uses k<sub>I<sub>φ</sub></sub> = ${prob.ki} (Listing 10.4, p. 167). With the listing's PD gains and the left panel's mismatch that leaves about 0.23° just before the switch, inside the 0.5° limit; larger k<sub>I<sub>φ</sub></sub> settles faster but overshoots more.` }],
        },
      ]);
    },
  };

  WB.studies.C.cascade = { makeCascade, slc: (p, o) => lib().slcDesign(p, o) };
})();
