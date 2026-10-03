// Design Study B controller library. Each factory mirrors one of the repo's
// _B_pendulum/python/ctrl*.py classes line for line (same update order, same
// initial values, same internal saturation points). Controllers return the
// unsaturated force; the simulation saturates it, as pendulumDynamics.update does,
// so the demanded force is visible and the applied force matches the repo. The
// chapter pages and tools/regress_B.py check the JS against the Python to machine
// precision.
// The chapter modules only add the UI around these.
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.B = WB.studies.B || { chapters: {} };

(function () {
  const sat = (u, lim) => (Math.abs(u) > lim ? lim * Math.sign(u) : u);
  const PL = () => WB.studies.B.place;

  // ω_n from a rise-time spec: '2.2' → 2.2/t_r (Eq. 8.5, B.8); 'tp' → π/(2 t_r √(1−ζ²))
  // (the 0.5π/(t_r√(1−ζ²)) rule in the B.11–B.14 listings).
  function wnOf(tr, zeta, rule) {
    return rule === 'tp' ? 0.5 * Math.PI / (tr * Math.sqrt(Math.max(1e-12, 1 - zeta ** 2))) : 2.2 / tr;
  }
  function pair(wn, zeta) {
    if (zeta < 1) { const wd = wn * Math.sqrt(1 - zeta * zeta); return [{ re: -zeta * wn, im: wd }, { re: -zeta * wn, im: -wd }]; }
    const r = wn * Math.sqrt(zeta * zeta - 1);
    return [{ re: -zeta * wn + r, im: 0 }, { re: -zeta * wn - r, im: 0 }];
  }

  // ------------------------------------------------ successive loop closure --
  // Inner PD gains, inner DC gain, and outer PD gains (B.8 / B.10).
  //   formula 'book'    : Eqs. 8.12–8.13 (p. 127), as ctrlPID.py codes them
  //   formula 'listing' : Listing 8.3 / ctrlPD.py, whose k_Dz expression differs (ISSUES.md)
  function slcGains(p, { trTh, zetaTh, M, zetaZ, formula = 'book' }) {
    const J = p.m1 * (p.ell / 6.0) + p.m2 * (2.0 * p.ell / 3.0);
    const wnTh = 2.2 / trTh, trZ = M * trTh, wnZ = 2.2 / trZ;
    const out = { wnTh, wnZ, trZ };
    if (formula === 'listing') {
      // ctrlPD.py
      const b0 = 1.0 / J, a0 = (p.m1 + p.m2) * p.g / J;
      out.kPth = -(wnTh ** 2 + a0) / b0;
      out.kDth = -(2.0 * zetaTh * wnTh) / b0;
      out.kDC = b0 * out.kPth / (b0 * out.kPth + a0);
      const a = wnZ ** 2 * Math.sqrt(2.0 * p.ell / 3.0 / p.g) - 2.0 * zetaZ * wnZ;
      out.kDz = a / (a + Math.sqrt(3.0 * p.g / 2.0 / p.ell));
      out.kPz = -(wnZ ** 2) * Math.sqrt(2.0 * p.ell / 3.0 / p.g) * (1 + out.kDz);
    } else {
      // ctrlPID.py (the book's derivation)
      const b0 = -1.0 / J, a1 = 0.0, a0 = -(p.m1 + p.m2) * p.g / J;
      const alpha1 = 2.0 * zetaTh * wnTh, alpha0 = wnTh ** 2;
      out.kPth = (alpha0 - a0) / b0;
      out.kDth = (alpha1 - a1) / b0;
      out.kDC = out.kPth / ((p.m1 + p.m2) * p.g + out.kPth);
      const a = -(wnZ ** 2) * Math.sqrt(2.0 * p.ell / (3.0 * p.g));
      const b = (a - 2.0 * zetaZ * wnZ) * Math.sqrt(2.0 * p.ell / (3.0 * p.g));
      out.kDz = b / (1 - b);
      out.kPz = a * (1 + out.kDz);
      out.a = a; out.b = b;
    }
    return out;
  }
  const dcGain = (p, kPth) => kPth / ((p.m1 + p.m2) * p.g + kPth);

  // Zero-canceling low-pass filter −(3/(2ℓ k_DC)) / (s + √(3g/2ℓ)), integrated with
  // one Euler (RK1) step per sample, as zeroCancelingFilter does.
  function zcFilter(p, kDC, Ts) {
    const a = -3.0 / (2.0 * p.ell * kDC), b = Math.sqrt(3.0 * p.g / (2.0 * p.ell));
    let state = 0.0;
    return { a, b, update(input) { state = state + Ts * (-b * state + a * input); return state; } };
  }

  // ctrlPD.py: PD on both loops using the true state.
  function slcPD({ g, p, Ts, uLim, filter = true, linear = false }) {
    const filt = zcFilter(p, g.kDC, Ts);
    return {
      update(r, x) {
        const z = x[0], theta = x[1], zdot = x[2], thetadot = x[3];
        const tmp = g.kPz * (r - z) - g.kDz * zdot;
        const thetaR = filter ? filt.update(tmp) : tmp;
        const F = g.kPth * (thetaR - theta) - g.kDth * thetadot;
        return { u: F, thetaR };   // unsaturated: the plant saturates (pendulumDynamics.update)
      },
    };
  }

  // ctrlPID.py: outer PID / inner PD on the measured (z, θ), dirty derivatives,
  // integrator gated on |ż| < v̄, θ_r saturated at θ_max, then the zero-canceling filter.
  function slcPID({ g, p, Ts, uLim, sigma = 0.05, vbar = 0.07, gate = true, thetaMax = 30 * Math.PI / 180, filter = true, deriv = 'dirty', linear = false }) {
    const filt = zcFilter(p, g.kDC, Ts);
    const beta = (2.0 * sigma - Ts) / (2.0 * sigma + Ts), gamma = 2.0 / (2.0 * sigma + Ts);
    let integ = 0.0, ePrev = 0.0, zdot = 0.0, thdot = 0.0, zPrev = null, thPrev = null;
    return {
      update(r, x, y) {
        const z = y[0], theta = y[1];
        if (zPrev === null) { zPrev = x[0]; thPrev = x[1]; }   // P.z0, P.theta0
        const errZ = r - z;
        zdot = deriv === 'state' ? x[2] : beta * zdot + gamma * (z - zPrev);
        if (!gate || Math.abs(zdot) < vbar) integ = integ + (Ts / 2) * (errZ + ePrev);
        const thRunsat = g.kPz * errZ + g.kIz * integ - g.kDz * zdot;
        let thetaR = linear ? thRunsat : sat(thRunsat, thetaMax);
        thetaR = filter ? filt.update(thetaR) : thetaR;
        const errTh = thetaR - theta;
        thdot = deriv === 'state' ? x[3] : beta * thdot + gamma * (theta - thPrev);
        const Funsat = g.kPth * errTh - g.kDth * thdot;
        ePrev = errZ; zPrev = z; thPrev = theta;
        return { u: Funsat, thetaR, thetaRunsat: thRunsat, integrator: integ, zdotHat: zdot, thetadotHat: thdot };
      },
    };
  }

  // ----------------------------------------------------------- state space --
  // Desired poles: θ pair (t_r,θ, ζ_θ), z pair (t_r,z = M t_r,θ, ζ_z), optional p_I.
  function ssPoles({ trTh, zetaTh, M, zetaZ, rule, pI }) {
    const wnTh = wnOf(trTh, zetaTh, rule), wnZ = wnOf(trTh * M, zetaZ, rule);
    const poles = [...pair(wnZ, zetaZ), ...pair(wnTh, zetaTh)];
    if (pI !== undefined && pI !== null) poles.push({ re: pI, im: 0 });
    return { poles, wnTh, wnZ };
  }
  // Observer poles: each pair `factor` times faster in rise time (tr_obs = tr/factor).
  function obsPoles({ trTh, zetaTh, M, zetaZ, obsRule, obsFactor, obsZeta }, pD) {
    const zt = obsZeta ?? zetaTh, zz = obsZeta ?? zetaZ;
    const wnTh = wnOf(trTh / obsFactor, zt, obsRule), wnZ = wnOf(trTh * M / obsFactor, zz, obsRule);
    const poles = [...pair(wnZ, zz), ...pair(wnTh, zt)];
    if (pD !== undefined && pD !== null) poles.push({ re: pD, im: 0 });
    return { poles, wnTh, wnZ };
  }

  const matInv = (A) => WB.la.inv(A);
  function augI(ss) {
    const { A, B } = ss;
    return { A1: [...A.map((r) => [...r, 0]), [-1, 0, 0, 0, 0]], B1: [...B.map((r) => [r[0]]), [0]] };
  }
  function augD(ss) {
    const { A, B, C } = ss;
    return { A2: [...A.map((r, i) => [...r, B[i][0]]), [0, 0, 0, 0, 0]], C2: C.map((r) => [...r, 0]), B1: [...B.map((r) => [r[0]]), [0]] };
  }

  // Gains for level 'sf' (B.11), 'sfi' (B.12), 'obs' (B.13), 'dobs' (B.14).
  function ssDesign(ss, level, t) {
    const { A, B, C } = ss;
    const out = {};
    if (level === 'sf') {
      const d = ssPoles({ ...t, pI: null });
      Object.assign(out, d);
      const K = PL().placeYT(A, B, d.poles);
      out.K = K ? K[0] : [NaN, NaN, NaN, NaN];
      const Ai = matInv(WB.la.sub(A, WB.la.mul(B, [out.K])));
      out.kr = Ai ? -1.0 / WB.la.mul(WB.la.mul([[1, 0, 0, 0]], Ai), B)[0][0] : NaN;
      return out;
    }
    const d = ssPoles(t);
    Object.assign(out, d);
    const { A1, B1 } = augI(ss);
    const K1 = PL().placeYT(A1, B1, d.poles);
    out.K = K1 ? K1[0].slice(0, 4) : [NaN, NaN, NaN, NaN];
    out.ki = K1 ? K1[0][4] : NaN;
    if (level === 'obs') {
      const o = obsPoles(t);
      out.obsPoles = o.poles; out.wnThObs = o.wnTh; out.wnZObs = o.wnZ;
      out.L = PL().observerYT(A, C, o.poles) || A.map(() => [NaN, NaN]);
    }
    if (level === 'dobs') {
      const o = obsPoles(t, t.pD);
      out.obsPoles = o.poles; out.wnThObs = o.wnTh; out.wnZObs = o.wnZ;
      const { A2, C2 } = augD(ss);
      out.L2 = PL().observerYT(A2, C2, o.poles) || A2.map(() => [NaN, NaN]);
      // plain observer with the same four state poles (used when the disturbance observer is off)
      out.L = PL().observerYT(A, C, o.poles.slice(0, 4)) || A.map(() => [NaN, NaN]);
    }
    return out;
  }

  const dot = (k, x) => k.reduce((s, v, i) => s + v * x[i], 0);

  // ctrlStateFeedback.py: F = −K x + k_r z_r, true state.
  function sfCtrl({ K, kr, uLim, linear = false }) {
    return { update(r, x) { const F = -dot(K, x) + kr * r; return { u: F }; } };
  }

  // ctrlStateFeedbackIntegrator.py: trapezoidal integrator on z_r − z (error_d1 = 0),
  // F = −K x − k_I x_I. antiwindup 'clamp' holds the integrator while F saturates.
  function sfiCtrl({ K, ki, uLim, Ts, antiwindup = 'none', linear = false }) {
    let integ = 0.0, eD1 = 0.0;
    return {
      update(r, x) {
        const e = r - x[0];
        const next = integ + (Ts / 2.0) * (e + eD1);
        if (antiwindup === 'clamp' && !linear && Math.abs(-dot(K, x) - ki * next) > uLim) { /* hold */ } else integ = next;
        eD1 = e;
        const F = -dot(K, x) - ki * integ;
        return { u: F, integrator: integ };
      },
    };
  }

  // RK4 observer step with the input held (update_observer / obsv_f).
  function rk4(fn, x, Ts) {
    const add = (a, b, k) => a.map((ai, i) => ai + k * b[i]);
    const F1 = fn(x), F2 = fn(add(x, F1, Ts / 2)), F3 = fn(add(x, F2, Ts / 2)), F4 = fn(add(x, F3, Ts));
    return x.map((xi, i) => xi + Ts / 6 * (F1[i] + 2 * F2[i] + 2 * F3[i] + F4[i]));
  }

  // ctrlObserver.py (dist = false) and ctrlDisturbanceObserver.py (dist = true).
  // Aobs, Bobs, Cobs, L are the (augmented, for dist) model and gain.
  function obsCtrl({ Aobs, Bobs, Cobs, L, K, ki, Ts, uLim, dist = false, xhat0 = null, antiwindup = 'none', linear = false }) {
    const n = Aobs.length;
    let xh = xhat0 ? xhat0.slice() : new Array(n).fill(0.0);
    let Fd1 = 0.0, integ = 0.0, eD1 = 0.0;
    const fObs = (xx, y) => {
      const innov = Cobs.map((row, i) => y[i] - dot(row, xx));
      return Aobs.map((row, i) => dot(row, xx) + Bobs[i][0] * Fd1 + dot(L[i], innov));
    };
    return {
      update(r, x, y) {
        xh = rk4((xx) => fObs(xx, y), xh, Ts);
        const xhat = xh.slice(0, 4), dhat = dist ? xh[4] : 0;
        const e = r - xhat[0];
        const next = integ + (Ts / 2.0) * (e + eD1);
        if (antiwindup === 'clamp' && !linear && Math.abs(-dot(K, xhat) - ki * next - dhat) > uLim) { /* hold */ } else integ = next;
        eD1 = e;
        const Fu = -dot(K, xhat) - ki * integ - dhat;
        const F = linear ? Fu : sat(Fu, uLim);
        Fd1 = F;   // the repo keeps the saturated force for the observer
        return { u: Fu, zhat: xhat[0], thhat: xhat[1], zdhat: xhat[2], thdhat: xhat[3], dhat, integrator: integ };
      },
    };
  }

  // ------------------------------------------------------------ loopshaping --
  // loopshape_tools.py building blocks (the repo's lead has DC gain 1/√M).
  const T = () => WB.tf;
  const blocks = {
    prop: (k) => T().tf([k], [1]),
    lead: (w, M) => T().tf([Math.sqrt(M), w], [1.0, w * Math.sqrt(M)]),
    lag: (z, M) => T().tf([1, z], [1, z / M]),
    lpf: (p) => T().tf([p], [1, p]),
    integ: (ki) => T().tf([1, ki], [1, 0]),
  };

  // ctrlLoopshape.transferFunction: control canonical form, one RK4 step per
  // sample, output y = C x⁺ + D u from the updated state.
  function tfStateSpace(numIn, denIn, Ts) {
    let num = numIn.slice(), den = denIn.slice();
    if (den[0] !== 1) { const k = den[0]; num = num.map((v) => v / k); den = den.map((v) => v / k); }
    const n = den.length, m = num.length, ns = n - 1;
    const A = Array.from({ length: ns }, () => new Array(ns).fill(0));
    for (let i = 0; i < ns; i++) A[0][i] = -den[i + 1];
    for (let i = 1; i < ns; i++) A[i][i - 1] = 1.0;
    const B = new Array(ns).fill(0); if (n > 1) B[0] = 1.0;
    const padded = [...new Array(n - m).fill(0), ...num];   // general proper case
    const D = padded[0];
    const C = new Array(ns).fill(0);
    for (let i = 0; i < ns; i++) C[i] = m === n ? num[i + 1] - num[0] * den[i + 1] : padded[i + 1];
    let state = new Array(ns).fill(0.0);
    const f = (xx, u) => A.map((row, i) => dot(row, xx) + B[i] * u);
    return {
      update(u) {
        if (ns) state = rk4((xx) => f(xx, u), state, Ts);
        return dot(C, state) + D * u;
      },
    };
  }

  // Tustin (bilinear) discretization of num/den, then ctrlLoopshape.digitalFilter.
  function tustin(num, den, Ts) {
    const n = den.length - 1;
    const pad = [...new Array(den.length - num.length).fill(0), ...num];
    const L = WB.la;
    const k = 2 / Ts;
    // substitute s = k (z − 1)/(z + 1) and multiply by (z + 1)^n: coefficients in z (descending)
    const sub = (p) => {
      let acc = [0];
      p.forEach((c, i) => {
        const pw = n - i;  // power of s
        let term = [c * k ** pw];
        for (let a = 0; a < pw; a++) term = L.conv(term, [1, -1]);
        for (let a = 0; a < n - pw; a++) term = L.conv(term, [1, 1]);
        acc = L.polyAdd(acc, term);
      });
      while (acc.length < n + 1) acc.unshift(0);
      return acc;
    };
    const nd = sub(pad), dd = sub(den);
    const a0 = dd[0];
    return { num: nd.map((v) => v / a0), den: dd.map((v) => v / a0) };
  }
  function digitalFilter(numIn, denIn, Ts) {
    const { num, den } = tustin(numIn, denIn, Ts);
    let uHist = new Array(den.length).fill(0), yHist = new Array(num.length - 1).fill(0);
    return {
      num, den,
      update(u) {
        uHist = [u, ...uHist.slice(0, -1)];
        const y = dot(num, uHist) - dot(den.slice(1), yHist);
        yHist = [y, ...yHist.slice(0, -1)];
        return y;
      },
    };
  }

  // ctrlLoopshape.py: prefilter → outer C → inner C, F saturated.
  function loopshapeCtrl({ Cin, Cout, F, Ts, uLim, method = 'state_space', linear = false }) {
    const mk = method === 'digital_filter' ? digitalFilter : tfStateSpace;
    const ci = mk(Cin.num, Cin.den, Ts), co = mk(Cout.num, Cout.den, Ts), pf = mk(F.num, F.den, Ts);
    return {
      update(r, x, y) {
        const zrF = pf.update(r);
        const thetaR = co.update(zrF - y[0]);
        const Fu = ci.update(thetaR - y[1]);
        return { u: Fu, thetaR };
      },
    };
  }

  WB.studies.B.lib = {
    sat, wnOf, pair, slcGains, dcGain, zcFilter, slcPD, slcPID,
    ssPoles, obsPoles, augI, augD, ssDesign, sfCtrl, sfiCtrl, obsCtrl,
    blocks, tfStateSpace, tustin, digitalFilter, loopshapeCtrl,
  };
})();
