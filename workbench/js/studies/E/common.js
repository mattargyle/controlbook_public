// Study E shared pieces: successive-loop PD/PID design (Ch 8–10, 15–17), the
// nested PID controller, state-space design with the structured observer
// (Ch 11–14), and small UI helpers. Everything is on WB.E so the chapter files
// stay short. Derivations are in the chapter solution texts.
window.WB = window.WB || {};

(function () {
  const { el, slider, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { fmt } = M;
  const sysE = () => WB.systems.E;

  // s-plane legends: pages pass data.kindNames = {kind: label} (plot.js).

  // -------------------------------------------------- successive loop closure --
  // Design model (E.5(c), p. 386–387): P_in = b0/s², b0 = ℓ/(m2ℓ²/3 + m1 z_e²);
  // P_out = −g/s². Inner PD: Δ = s² + b0 kD s + b0 kP; its DC gain is 1. Outer PD
  // with the inner loop as k_DC: Δ = s² − g k_DC kD s − g k_DC kP (E.8, p. 387).
  function pdDesign(p, k) {
    const s = sysE();
    const z0 = s.ze(p), De = s.inertia(p, z0), b0 = p.ell / De;
    const rule = k.rule || '2.2';
    const wTh = WB.design.wnFromTr(k.trTh, k.zetaTh, rule);
    const kPth = wTh * wTh / b0, kDth = 2 * k.zetaTh * wTh / b0;
    const kDC = 1;  // b0 kP / (b0 kP): the inner loop has unity DC gain
    const trZ = k.M * k.trTh;
    const wZ = WB.design.wnFromTr(trZ, k.zetaZ, rule);
    const kPz = -wZ * wZ / (p.g * kDC), kDz = -2 * k.zetaZ * wZ / (p.g * kDC);
    return { ze: z0, De, b0, wTh, wZ, trZ, kDC, kPth, kDth, kPz, kDz };
  }
  const innerPoles = (b0, g) => M.roots2(b0 * g.kDth, b0 * g.kPth);
  const outerPoles = (p, g, kDC = 1) => M.roots2(-p.g * kDC * g.kDz, -p.g * kDC * g.kPz);
  // All four poles of the nested PD loop on a linear model (comp 'fl' = design model).
  // F̃ = kPθ(kPz(zr − z) − kDz ż − θ) − kDθ θ̇  ⇒  K = (kPθkPz, kPθ, kPθkDz, kDθ).
  function nestedPoles(p, g, comp = 'fl') {
    const { A, B } = sysE().linear(p, { comp });
    const K = [g.kPth * g.kPz, g.kPth, g.kPth * g.kDz, g.kDth];
    return L.eig(L.sub(A, L.mul(B, [K])));
  }

  // Nested PID: outer z-loop (PID, gains negative) commands θ_r; inner θ-loop (PD,
  // optional integrator) gives F̃; F = F_ff + F̃ with F_ff = F_fl(z) ('fl'), F_e ('eq')
  // or 0. Conventions follow the repo's ctrlPID.py: error_prev = 0, y_prev = first
  // sample, trapezoidal integrators, dirty derivative Eq. 10.4.
  //   opt.meas: 'state' (true ż, θ̇: E.8, E.9) | 'dirty' (measured z, θ: E.10)
  //   opt.antiwindup: 'gate' (integrate z only when |ż| < v̄, E.10(c)) | 'none'
  function nestedPID(ctx, g, opt = {}, { linear = false } = {}) {
    const { pModel, S } = ctx;
    const s = sysE();
    const Ts = S.sim.Ts, sigma = opt.sigma ?? 0.05;
    const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    const FeV = s.Fe(pModel);
    let zPrev = null, thPrev = 0, zd = 0, thd = 0, Iz = 0, ezPrev = 0, Ith = 0, ethPrev = 0;
    return {
      update(r, x, yMeas) {
        const dirty = opt.meas === 'dirty';
        const z = dirty ? yMeas[0] : x[0], th = dirty ? yMeas[1] : x[1];
        if (zPrev === null) { zPrev = z; thPrev = th; }
        if (dirty) {
          zd = beta * zd + gamma * (z - zPrev);
          thd = beta * thd + gamma * (th - thPrev);
        } else {
          zd = x[2]; thd = x[3];
        }
        const ez = r - z;
        if (g.kIz && (opt.antiwindup !== 'gate' || Math.abs(zd) < (opt.vbar ?? 0.05))) Iz += (Ts / 2) * (ez + ezPrev);
        const thetaR = g.kPz * ez + (g.kIz || 0) * Iz - g.kDz * zd;
        const eth = thetaR - th;
        if (g.kIth) Ith += (Ts / 2) * (eth + ethPrev);
        const Ft = g.kPth * eth + (g.kIth || 0) * Ith - g.kDth * thd;
        let ff = 0;
        if (!linear) ff = opt.comp === 'fl' ? s.Ffl(z, pModel) : opt.comp === 'eq' ? FeV : 0;
        zPrev = z; thPrev = th; ezPrev = ez; ethPrev = eth;
        return { u: ff + Ft, thetaR, ff, Ft, zdHat: zd, thdHat: thd, Iz };
      },
    };
  }

  // Linear design-model overlay (z̈ = −gθ, θ̈ = b0 F̃) for the nested loops.
  function nestedLinearSim(ctx, common, g, opt) {
    const s = sysE();
    const { A, B, C } = s.linear(ctx.pModel, { comp: 'fl' });
    const plant = WB.design.linearPlant(A, B, C, { xe: [s.ze(ctx.pModel), 0, 0, 0] });
    return WB.sim.simulate({ ...common, disturbance: null, noise: null, plant, controller: nestedPID(ctx, g, { ...opt, meas: 'state' }, { linear: true }) });
  }

  // ------------------------------------------------------------ state space --
  // Two second-order pairs: θ-like (fast) and z-like (slow), as in B.11 (p. 190).
  function ssPoles(k) {
    return [...WB.design.polesFromWnZeta(2.2 / k.trTh, k.zetaTh), ...WB.design.polesFromWnZeta(2.2 / k.trZ, k.zetaZ)];
  }
  const Cr = [[1, 0, 0, 0]];
  function augI(A, B) { return WB.design.augmentIntegrator(A, B, Cr); }
  function augD(A, B, C) { return WB.design.augmentDisturbance(A, B, C); }

  // Observer gain. 'decoupled': L cancels the two cross-couplings of A − LC (the −g
  // and −m1g/De entries), leaving a z-block s² + l1 s + l2 and a θ-block
  // s² + l3 s + l4 (or the 3rd-order θ/d block for the disturbance observer). Each
  // block is then one second-order (or third-order) placement. 'zonly' places all
  // poles from z alone (single-output Ackermann), ignoring the θ measurement.
  // poles: {z: [2 poles], th: [2 poles], d: pole?}
  function obsGain(A, B, C, poles, mode = 'decoupled') {
    const withD = poles.d !== undefined && poles.d !== null;
    const n = withD ? 5 : 4;
    if (mode === 'zonly') {
      const Aa = withD ? augD(A, B, C).A2 : A;
      const Cz = [new Array(n).fill(0)]; Cz[0][0] = 1;
      const all = [...poles.z, ...poles.th, ...(withD ? [{ re: poles.d, im: 0 }] : [])];
      const Lt = L.place(L.T(Aa), L.T(Cz), all);
      return Lt ? Lt[0].map((v) => [v, 0]) : null;
    }
    const az = L.polyFromRoots(poles.z);
    const Lg = Array.from({ length: n }, () => [0, 0]);
    Lg[0][0] = az[1]; Lg[2][0] = az[2];
    Lg[2][1] = A[2][1];             // cancels −g (z̈ from θ)
    Lg[3][0] = A[3][0];             // cancels −m1 g/De (θ̈ from z); 0 when F_fl is used
    if (!withD) {
      const at = L.polyFromRoots(poles.th);
      Lg[1][1] = at[1]; Lg[3][1] = at[2];
    } else {
      const at = L.polyFromRoots([...poles.th, { re: poles.d, im: 0 }]);
      Lg[1][1] = at[1]; Lg[3][1] = at[2]; Lg[4][1] = at[3] / B[3][0];  // s³ + a s² + b s + b0 c
    }
    return Lg;
  }

  // Gains for a level from tuning knobs k (trTh, zetaTh, trZ, zetaZ, pI, obsFactor,
  // zetaObs, pD, comp, obsMode). Returns poles too, for markers and math cards.
  function ssDesign(p, k, level, polesOverride) {
    const s = sysE();
    const lin = s.linear(p, { comp: k.comp === 'fl' ? 'fl' : 'eq' });
    const { A, B, C } = lin;
    const out = { lin, poles: polesOverride || ssPoles(k) };
    if (level === 'sf') {
      const Kp = L.place(A, B, out.poles);
      out.K = Kp ? Kp[0] : [NaN, NaN, NaN, NaN];
      out.kr = WB.design.refGain(A, B, Cr, out.K);
      return out;
    }
    const { A1, B1 } = augI(A, B);
    out.poles = [...out.poles, { re: k.pI, im: 0 }];
    const K1 = L.place(A1, B1, out.poles);
    out.K = K1 ? K1[0].slice(0, 4) : [NaN, NaN, NaN, NaN];
    out.ki = K1 ? K1[0][4] : NaN;
    if (level === 'obs' || level === 'dobs') {
      const f = k.obsFactor;
      out.obs = {
        z: WB.design.polesFromWnZeta(f * 2.2 / k.trZ, k.zetaObs),
        th: WB.design.polesFromWnZeta(f * 2.2 / k.trTh, k.zetaObs),
        d: level === 'dobs' ? k.pD : undefined,
      };
      out.obsPoles = [...out.obs.z, ...out.obs.th, ...(level === 'dobs' ? [{ re: k.pD, im: 0 }] : [])];
      out.L = obsGain(A, B, C, out.obs, k.obsMode);
    }
    return out;
  }

  // State-space controller (E.11–E.14). Linear model in deviation coordinates
  // x̃ = x − x_e, F̃ = F − F_ff with F_ff = F_e ('eq', Jacobian A) or F_fl(z) ('fl', A[3][0] = 0).
  //   sf:   F̃ = −K x̃ + k_r (z_r − z_e)                       (Eq. 11.38)
  //   sfi:  F̃ = −K x̃ − k_I ∫(z_r − z)                         (Eq. 12.x, p. 199)
  //   obs:  same with x̂ from ẋ̂ = A x̃̂ + B(u − F_ff(ẑ)) + L(ỹ − C x̃̂)   (p. 224)
  //   dobs: ... + B d̂ in the model, ḋ̂ = L_d(ỹ − C x̃̂), and F̃ −= d̂   (p. 241)
  // Anti-windup 'clamp': hold the integrator while the actuator saturates.
  function makeSS(ctx, g, level, opt = {}, { linear = false } = {}) {
    const { pModel, S } = ctx;
    const s = sysE();
    const lin = s.linear(pModel, { comp: opt.comp === 'fl' ? 'fl' : 'eq' });
    const { A, B, C } = lin;
    const z0 = lin.ze;
    const ff = (z) => (linear ? 0 : opt.comp === 'fl' ? s.Ffl(z, pModel) : lin.Fe);
    const Ts = S.sim.Ts, uLim = s.uLimit(pModel);
    const useObs = level === 'obs' || level === 'dobs';
    const useDO = level === 'dobs' && opt.dobs !== false;
    const sigma = 0.05, { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    const Lg = g.L || [];
    let I = 0, ePrev = 0, uPrev = null, yPrev = null, zd = 0, thd = 0;
    let xh = [opt.zhat0 || 0, 0, 0, 0], dh = 0;

    function fObs(v, yt, u) {
      const xx = v.slice(0, 4), d = v[4];
      const innov = [yt[0] - xx[0], yt[1] - xx[1]];
      const ut = u - ff(xx[0] + z0) + (useDO ? d : 0);
      const dx = A.map((row, i) => row.reduce((a, aij, j) => a + aij * xx[j], 0) + B[i][0] * ut + Lg[i][0] * innov[0] + Lg[i][1] * innov[1]);
      dx.push(level === 'dobs' ? Lg[4][0] * innov[0] + Lg[4][1] * innov[1] : 0);
      return dx;
    }

    return {
      update(r, x, yMeas) {
        let xt;
        if (useObs) {
          if (uPrev === null) uPrev = ff(z0);  // F̃ = 0 before the first sample
          const yt = [yMeas[0] - z0, yMeas[1]];
          const v = M.rk4Step((vv) => fObs(vv, yt, uPrev), [...xh, dh], 0, Ts);
          xh = v.slice(0, 4); dh = v[4];
          xt = xh;
        } else if (opt.est === 'dirty') {
          if (yPrev === null) yPrev = yMeas.slice();
          zd = beta * zd + gamma * (yMeas[0] - yPrev[0]);
          thd = beta * thd + gamma * (yMeas[1] - yPrev[1]);
          yPrev = yMeas.slice();
          xt = [yMeas[0] - z0, yMeas[1], zd, thd];
        } else {
          xt = [x[0] - z0, x[1], x[2], x[3]];
        }
        const z = xt[0] + z0;
        const Kx = g.K.reduce((a, k, i) => a + k * xt[i], 0);
        let u;
        if (level === 'sf') {
          u = ff(z) - Kx + g.kr * (r - z0);
        } else {
          const e = r - z;
          const dTerm = useDO ? dh : 0;
          const Itry = I + (Ts / 2) * (e + ePrev);
          const uTry = ff(z) - Kx - g.ki * Itry - dTerm;
          if (!(opt.antiwindup === 'clamp' && !linear && Math.abs(uTry) > uLim)) I = Itry;
          ePrev = e;
          u = ff(z) - Kx - g.ki * I - dTerm;
        }
        uPrev = linear ? u : M.saturate(u, uLim);
        return { u, zhat: xt[0] + z0, thhat: xt[1], zdhat: xt[2], thdhat: xt[3], dhat: dh, integrator: I };
      },
    };
  }

  // ---------------------------------------------------------------- UI bits --
  const readout = (parent, rows) => WB.ui.readout(parent, rows);

  // Grid of number boxes for matrix gains (K, L) in Work mode. obj[key] is a
  // number, or an array for matrix entries (key 'L' with idx).
  function numGrid(parent, items, onChange) {
    const grid = el('div', { class: 'part-inputs' });
    for (const it of items) {
      const inp = el('input', { type: 'number', step: 'any', class: 'num', 'aria-label': it.label.replace(/<[^>]+>/g, '') });
      inp.style.width = '92px';
      inp.addEventListener('change', () => { const v = parseFloat(inp.value); if (!isNaN(v)) { it.set(v); onChange(); } });
      const lab = el('label', { class: 'part-label' });
      lab.innerHTML = it.label;
      grid.append(el('div', { class: 'part-field' }, lab, inp));
      WB.ui.addRefresher(() => { if (document.activeElement !== inp) inp.value = Number(it.get().toPrecision(5)); });
    }
    parent.append(grid);
    return grid;
  }

  // Slider helper bound to ctx.st[key].
  function knob(parent, ctx, key, label, min, max, step, extra = {}) {
    return slider(parent, { label, min, max, step, sig: extra.sig || 4, unit: extra.unit, hint: extra.hint, log: extra.log, disabled: extra.disabled,
      ...bind(ctx, key) });
  }

  // Index of the sample just before the first reference switch (square wave) or t_end.
  function beforeSwitch(ctx, res) {
    const S = ctx.S;
    const tSw = WB.sim.switchTime(S);
    return WB.sim.indexBefore(S, res, tSw);
  }

  // Block stays on the (true) beam for the whole run: 0 ≤ z ≤ ℓ_true.
  function onBeam(ctx, res) {
    let zmin = Infinity, zmax = -Infinity;
    for (const z of res.yAll[0]) { if (!isFinite(z)) { zmin = -Infinity; break; } zmin = Math.min(zmin, z); zmax = Math.max(zmax, z); }
    const ell = ctx.pTrue ? ctx.pTrue.ell : ctx.pModel.ell;
    const ok = zmin >= 0 && zmax <= ell;
    return { ok, zmin, zmax, ell, msg: ok ? `on the beam (z in [${fmt(zmin, 3)}, ${fmt(zmax, 3)}] m)` : `block left the beam (z from ${fmt(zmin, 3)} to ${fmt(zmax, 3)} m, beam 0–${fmt(ell, 3)} m)` };
  }

  // Index just before the last reference switch (square wave), else before t_end.
  function beforeLastSwitch(ctx, res) {
    const S = ctx.S, half = 0.5 / S.sim.frequency;
    let tSw = S.sim.tEnd;
    if (S.sim.type === 'square') tSw = S.sim.tStep + Math.floor((S.sim.tEnd - S.sim.tStep) / half - 1e-9) * half;
    return WB.sim.indexBefore(S, res, tSw);
  }

  // Shared problem-panel accessors.
  const P = () => WB.pd;
  const answersOf = (ctx, probId) => WB.ui.store.get(`wb.${ctx.sys.id}.${probId}.answers`, {});
  const poleText = (ps) => ps.map((q) => M.fmtPole(q, 4)).join(', ');

  // Closed-loop bandwidth: the first frequency where |T| falls 3 dB below its
  // low-frequency value (as control.bandwidth).
  const bandwidth = (Tc, W) => WB.tf.bandwidth(Tc, W, WB.tf.mag(Tc, W[0]) * 10 ** (-3 / 20));

  WB.E = {
    pdDesign, innerPoles, outerPoles, nestedPoles, nestedPID, nestedLinearSim,
    ssPoles, augI, augD, obsGain, ssDesign, makeSS, Cr,
    readout, numGrid, knob, beforeSwitch, beforeLastSwitch, onBeam, P, answersOf, poleText, bandwidth, fmt,
  };
})();
