// Generic design building blocks for any study: second-order pole placement,
// a digital PID block, state-space gain design, and an RK4 observer. These follow
// the repo's Python conventions (error_prev = 0, y_prev = first sample, trapezoidal
// integrator, dirty derivative Eq. 10.4, RK4 at Ts), so a study's JS controller
// can match its ctrl*.py to machine precision.
window.WB = window.WB || {};

WB.design = (function () {
  const L = WB.la;
  const M = WB.math;

  // ----------------------------------------------------- second-order design --
  // ω_n from a rise-time spec: '2.2' → 2.2/t_r (Eq. 8.5); 'tp' → π/(2 t_r √(1−ζ²)).
  function wnFromTr(tr, zeta, rule = '2.2') {
    if (rule === 'tp') return Math.PI / (2 * tr * Math.sqrt(Math.max(1e-9, 1 - zeta * zeta)));
    return 2.2 / tr;
  }
  function polesFromWnZeta(wn, zeta) {
    if (zeta < 1) {
      const wd = wn * Math.sqrt(1 - zeta * zeta);
      return [{ re: -zeta * wn, im: wd }, { re: -zeta * wn, im: -wd }];
    }
    const r = wn * Math.sqrt(zeta * zeta - 1);
    return [{ re: -zeta * wn + r, im: 0 }, { re: -zeta * wn - r, im: 0 }];
  }
  // PD gains placing the poles of b0/(s² + a1 s + a0) under u = kP(r−y) − kD ẏ (Ch 7).
  function pdGains({ b0, a1, a0 }, poles) {
    const { alpha1, alpha0 } = M.polyFromPoles(poles[0], poles[1]);
    return { kP: (alpha0 - a0) / b0, kD: (alpha1 - a1) / b0 };
  }

  // ---------------------------------------------------------------- PID block --
  // Digital PID on one loop (Listing 10.1/10.2 conventions).
  //   deriv: 'y' (dirty derivative of the measurement; D term = −kD ẏ) or 'error'
  //          (dirty derivative of e; D term = +kD ė).
  //   antiwindup: 'none' | 'gate' (integrate only when |ẏ| < vbar) |
  //               'backcalc' (u_I += (u_sat − u_unsat)/kI) | 'clamp' (skip while saturated)
  //   limit: saturation of this block's output (Infinity = none). update() returns
  //          the saturated value when a limit is set (as the repo's PIDControl does).
  function pidBlock({ kP, kI = 0, kD = 0, sigma = 0.05, Ts, limit = Infinity, antiwindup = 'none', vbar = Infinity, deriv = 'y' }) {
    const beta = (2 * sigma - Ts) / (2 * sigma + Ts), gamma = 2 / (2 * sigma + Ts);
    const st = { I: 0, ePrev: 0, yPrev: null, ydot: 0, edot: 0, u: 0, uUnsat: 0, first: true };
    const sat = (u) => M.saturate(u, limit);
    return {
      state: st,
      // y: measurement; opts.ydot overrides the dirty derivative (e.g. use a true rate).
      update(r, y, opts = {}) {
        const e = r - y;
        if (st.first) { st.yPrev = y; st.first = false; }
        st.ydot = opts.ydot !== undefined ? opts.ydot : beta * st.ydot + gamma * (y - st.yPrev);
        st.edot = beta * st.edot + gamma * (e - st.ePrev);
        const gateOk = antiwindup !== 'gate' || Math.abs(st.ydot) < vbar;
        const Itry = st.I + (Ts / 2) * (e + st.ePrev);
        const dTerm = deriv === 'error' ? kD * st.edot : -kD * st.ydot;
        let uUnsat = kP * e + kI * (gateOk ? Itry : st.I) + dTerm;
        if (antiwindup === 'clamp' && Math.abs(uUnsat) > limit) {
          uUnsat = kP * e + kI * st.I + dTerm;  // hold the integrator
        } else if (gateOk) {
          st.I = Itry;
        }
        let u = sat(uUnsat);
        if (antiwindup === 'backcalc' && kI !== 0 && u !== uUnsat) st.I += (u - uUnsat) / kI;
        st.ePrev = e; st.yPrev = y; st.u = u; st.uUnsat = uUnsat;
        return u;
      },
    };
  }

  // ------------------------------------------------------------ state space --
  // K (1×n row as a flat array) placing eig(A − BK) at poles; null if uncontrollable.
  function place(A, B, poles) {
    const K = L.place(A, B, poles);
    return K ? K[0] : null;
  }
  // Reference gain k_r = −1 / (C (A − BK)^{-1} B)  (Eq. 11.35)
  function refGain(A, B, C, K) {
    const Ai = L.inv(L.sub(A, L.mul(B, [K])));
    return Ai ? -1 / L.mul(L.mul(C, Ai), B)[0][0] : NaN;
  }
  // [A 0; −Cr 0], [B; 0]  (Eq. 12.1)
  function augmentIntegrator(A, B, Cr) {
    const n = A.length;
    const A1 = A.map((row) => [...row, 0]);
    A1.push([...Cr[0].map((c) => -c), 0]);
    const B1 = [...B.map((r) => [r[0]]), [0]];
    return { A1, B1 };
  }
  // Input-disturbance model [A B; 0 0], [C 0]  (p. 240)
  function augmentDisturbance(A, B, C) {
    const n = A.length;
    const A2 = A.map((row, i) => [...row, B[i][0]]);
    A2.push(new Array(n + 1).fill(0));
    const C2 = C.map((row) => [...row, 0]);
    const B2 = [...B.map((r) => [r[0]]), [0]];
    return { A2, B2, C2 };
  }
  // L (n×p) so eig(A − LC) are the poles: place(Aᵀ, Cᵀ)ᵀ  (single output)
  function observerGain(A, C, poles) {
    const Lt = L.place(L.T(A), L.T(C), poles);
    return Lt ? Lt[0].map((v) => [v]) : null;
  }

  // RK4 observer over one Ts with the input held, matching update_observer().
  // rhs(xhat, y, u) defaults to A x̂ + B u + L (y − C x̂); pass your own to add
  // feedback-linearization or equilibrium terms. y and u may be numbers or arrays.
  function observer({ A, B, C, L: Lg, Ts, x0, rhs }) {
    let xh = (x0 || new Array(A.length).fill(0)).slice();
    const asArr = (v) => (Array.isArray(v) ? v : [v]);
    const f = rhs || ((x, y, u) => {
      const yv = asArr(y), uv = asArr(u);
      const innov = C.map((row, i) => yv[i] - row.reduce((s, c, j) => s + c * x[j], 0));
      return A.map((row, i) => row.reduce((s, a, j) => s + a * x[j], 0)
        + B[i].reduce((s, b, j) => s + b * uv[j], 0)
        + Lg[i].reduce((s, l, j) => s + l * innov[j], 0));
    });
    return {
      get xhat() { return xh; },
      set xhat(v) { xh = v.slice(); },
      update(y, u) {
        xh = M.rk4Step((x) => f(x, y, u), xh, 0, Ts);
        return xh;
      },
    };
  }

  // Multi-input / multi-output linear plant xdot = A x + B u for linear overlays.
  function linearPlant(A, B, C, { xe, ue, ye } = {}) {
    const n = A.length;
    xe = xe || new Array(n).fill(0);
    return {
      f: (x, u) => {
        const uv = Array.isArray(u) ? u : [u];
        const dx = x.map((v, i) => v - xe[i]);
        const du = uv.map((v, j) => v - ((ue || [])[j] || 0));
        return A.map((row, i) => row.reduce((s, a, j) => s + a * dx[j], 0) + B[i].reduce((s, b, j) => s + b * du[j], 0));
      },
      // y = C (x − x_e) + y_e, with y_e defaulting to C x_e (i.e. y = C x)
      h: (x) => {
        const y = C.map((row, i) => {
          const Cx = row.reduce((s, c, j) => s + c * x[j], 0);
          return ye ? Cx - row.reduce((s, c, j) => s + c * xe[j], 0) + ye[i] : Cx;
        });
        return y.length === 1 ? y[0] : y;
      },
      uLimit: Infinity,
    };
  }

  // Problem-panel kit (implemented in chapters/pd.js; resolved at call time).
  const problemPanel = (...a) => WB.pd.problemPanel(...a);
  const checkNumbers = (...a) => WB.pd.checkNumbers(...a);
  const num = (s) => WB.pd.num(s);

  return {
    wnFromTr, polesFromWnZeta, pdGains, pidBlock,
    place, refGain, augmentIntegrator, augmentDisturbance, observerGain, observer, linearPlant,
    problemPanel, checkNumbers, num,
  };
})();
