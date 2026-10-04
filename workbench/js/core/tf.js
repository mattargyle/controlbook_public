// SISO transfer functions G(s) = num(s)/den(s) for the frequency-domain chapters
// (Ch 15-18): products, feedback, frequency response, margins, and a state-space
// realization for simulating a compensator.
window.WB = window.WB || {};

WB.tf = (function () {
  const L = WB.la;
  const { C } = L;

  function tf(num, den) {
    num = L.trimLeading(num); den = L.trimLeading(den);
    const k = den[0];
    return { num: num.map((v) => v / k), den: den.map((v) => v / k) };
  }
  const mul = (...Gs) => Gs.reduce((a, b) => tf(L.conv(a.num, b.num), L.conv(a.den, b.den)));
  const add = (a, b) => tf(L.polyAdd(L.conv(a.num, b.den), L.conv(b.num, a.den)), L.conv(a.den, b.den));
  const gain = (k) => tf([k], [1]);
  // G / (1 + G H)
  function feedback(G, H = gain(1)) {
    return tf(L.conv(G.num, H.den), L.polyAdd(L.conv(G.den, H.den), L.conv(G.num, H.num)));
  }

  function at(G, w) {
    const s = C.of(0, w);
    return C.div(L.polyvalC(G.num, s), L.polyvalC(G.den, s));
  }

  // Magnitude (absolute) and unwrapped phase (deg) on a frequency grid.
  function bode(G, ws) {
    const mag = new Float64Array(ws.length), phase = new Float64Array(ws.length);
    let prev = null;
    for (let i = 0; i < ws.length; i++) {
      const g = at(G, ws[i]);
      mag[i] = C.abs(g);
      let ph = C.arg(g) * 180 / Math.PI;
      if (prev !== null) { while (ph - prev > 180) ph -= 360; while (ph - prev < -180) ph += 360; }
      else {
        // anchor the low-frequency phase to the system type: an integrator contributes -90 each
        const nInt = countOriginRoots(G.den) - countOriginRoots(G.num);
        const expected = -90 * nInt;
        while (ph - expected > 180) ph -= 360;
        while (ph - expected < -180) ph += 360;
      }
      phase[i] = ph; prev = ph;
    }
    return { mag, phase };
  }

  function countOriginRoots(p) {
    let n = 0;
    for (let i = p.length - 1; i > 0 && Math.abs(p[i]) < 1e-12; i--) n++;
    return n;
  }

  function logspace(a, b, n) {
    return Float64Array.from({ length: n }, (_, i) => Math.pow(10, a + (b - a) * i / (n - 1)));
  }

  // Gain/phase margins of a loop gain L(s), matching control.margin: PM at the
  // gain crossover (|L| = 1), GM at the phase crossover (phase = -180 deg).
  // gcs lists every gain crossover {w, pm} (a loop can cross |L| = 1 more than once).
  function margins(Lg, wlo = -4, whi = 5, n = 4000) {
    const ws = logspace(wlo, whi, n);
    const { mag, phase } = bode(Lg, ws);
    let wc = NaN, pm = Infinity, w180 = NaN, gm = Infinity;
    const crossings = [];  // every phase crossing of -180 (+k·360): conditionally stable loops have several
    const gcs = [];
    for (let i = 1; i < ws.length; i++) {
      if ((mag[i - 1] - 1) * (mag[i] - 1) <= 0 && mag[i - 1] !== mag[i]) {
        const t = Math.log(mag[i - 1]) / (Math.log(mag[i - 1]) - Math.log(mag[i]));
        const w = Math.exp(Math.log(ws[i - 1]) + t * (Math.log(ws[i]) - Math.log(ws[i - 1])));
        const ph = phase[i - 1] + t * (phase[i] - phase[i - 1]);
        gcs.push({ w, pm: ((180 + ph) % 360 + 540) % 360 - 180 });  // wrap into [-180, 180)
        if (isNaN(wc)) { wc = w; pm = gcs[0].pm; }
      }
      const a = (phase[i - 1] + 180) / 360, b = (phase[i] + 180) / 360;
      if (Math.floor(a) !== Math.floor(b)) {
        const target = 360 * Math.max(Math.floor(a), Math.floor(b)) - 180;
        const t = (target - phase[i - 1]) / (phase[i] - phase[i - 1]);
        const w = Math.exp(Math.log(ws[i - 1]) + t * (Math.log(ws[i]) - Math.log(ws[i - 1])));
        crossings.push({ w, gm: 1 / C.abs(at(Lg, w)) });
        if (isNaN(w180)) { w180 = w; gm = crossings[0].gm; }
      }
    }
    return { pm, wc, gm, w180, crossings, gcs };
  }

  const mag = (G, w) => C.abs(at(G, w));
  const db = (m) => 20 * Math.log10(m);

  // Downward crossings of |G(jω)| through `level` on the grid ws, each refined
  // by bisection in log ω.
  function crossDown(G, ws, level) {
    const out = [];
    let prev = mag(G, ws[0]);
    for (let i = 1; i < ws.length; i++) {
      const m = mag(G, ws[i]);
      if (prev >= level && m < level) {
        let lo = Math.log(ws[i - 1]), hi = Math.log(ws[i]);
        for (let k = 0; k < 60; k++) { const mid = 0.5 * (lo + hi); if (mag(G, Math.exp(mid)) < level) hi = mid; else lo = mid; }
        out.push(Math.exp(0.5 * (lo + hi)));
      }
      prev = m;
    }
    return out;
  }

  // Closed-loop bandwidth: the first frequency where |T| drops below `level`.
  // The default, 3 dB below the DC gain, is what control.bandwidth uses.
  function bandwidth(Tc, ws, level = Math.abs(dcgain(Tc)) * 10 ** (-3 / 20)) {
    const c = crossDown(Tc, ws, level);
    return c.length ? c[0] : NaN;
  }

  // Root-locus branches of den(s) + k num(s) = 0 for k in [0, kMax] (quadratic
  // spacing in k), each new root matched greedily to the nearest previous one.
  function rootLocus(den, num, kMax, steps = 300) {
    const branches = [];
    let prev = null;
    for (let i = 0; i <= steps; i++) {
      const k = kMax * Math.pow(i / steps, 2);
      let r = L.roots(L.polyAdd(den, L.polyScale(num, k)));
      if (prev) {
        const used = new Set(), ordered = [];
        for (const p of prev) {
          let best = -1, bd = Infinity;
          r.forEach((q, j) => { if (!used.has(j)) { const d = Math.hypot(q.re - p.re, q.im - p.im); if (d < bd) { bd = d; best = j; } } });
          used.add(best); ordered.push(r[best]);
        }
        r = ordered;
      } else {
        r.forEach(() => branches.push([]));
      }
      r.forEach((q, j) => branches[j].push(q));
      prev = r;
    }
    return branches;
  }

  // Compensators. PID with a dirty derivative (p. 313): k_P + k_I/s + k_D s/(σs + 1),
  // i.e. ((k_D + σk_P)s² + (k_P + σk_I)s + k_I) / (s(σs + 1)); the PD form when k_I = 0.
  function pid({ kP, kI = 0, kD = 0, sigma }) {
    return kI ? tf([kD + sigma * kP, kP + sigma * kI, kI], [sigma, 1, 0]) : tf([kD + sigma * kP, kP], [sigma, 1]);
  }
  // Loopshaping blocks: lead M(s + ω/√M)/(s + ω√M) (Eq. 18.2), lag (s + z)/(s + z/M)
  // (Eq. 18.1), low-pass p/(s + p), and PI (s + z)/s (§18.1).
  const lead = (M, w) => tf([M, M * w / Math.sqrt(M)], [1, w * Math.sqrt(M)]);
  const lag = (z, M) => tf([1, z], [1, z / M]);
  const lpf = (p) => tf([p], [1, p]);
  const pi = (z) => tf([1, z], [1, 0]);

  const poles = (G) => L.roots(G.den);
  const zeros = (G) => (G.num.length > 1 ? L.roots(G.num) : []);
  function dcgain(G) {
    const d = G.den[G.den.length - 1], n = G.num[G.num.length - 1];
    return d === 0 ? Infinity : n / d;
  }

  // Controllable canonical realization of a proper G: xdot = A x + B u, y = C x + D u.
  function ss(G) {
    const den = G.den, n = den.length - 1;
    let num = G.num.slice();
    if (num.length > den.length) throw new Error('improper transfer function');
    while (num.length < den.length) num.unshift(0);
    const D = num[0];
    const r = num.map((v, i) => v - D * den[i]).slice(1); // strictly proper remainder
    if (n === 0) return { A: [], B: [], C: [], D, n };
    const A = L.zeros(n, n);
    for (let i = 0; i < n - 1; i++) A[i][i + 1] = 1;
    for (let j = 0; j < n; j++) A[n - 1][j] = -den[n - j];
    const B = new Array(n).fill(0); B[n - 1] = 1;
    const Cv = new Array(n).fill(0);
    for (let j = 0; j < n; j++) Cv[j] = r[n - 1 - j];
    return { A, B, C: Cv, D, n };
  }

  // A stateful continuous-time filter integrated with RK4 at step Ts (input held).
  function filter(G, Ts) {
    const { A, B, C: Cv, D, n } = ss(G);
    let x = new Array(n).fill(0);
    const f = (xx, u) => xx.map((_, i) => A[i].reduce((s, a, j) => s + a * xx[j], 0) + B[i] * u);
    // RK4 is stable for |λ h| < ~2.8; substep so fast compensator poles stay stable.
    const fastest = n ? Math.max(...L.roots(G.den).map((r) => Math.hypot(r.re, r.im))) : 0;
    const nSub = Math.max(1, Math.ceil(fastest * Ts / 1.5));
    const h = Ts / nSub;
    return {
      // output for the current input, then advance the state by Ts
      step(u) {
        const y = Cv.reduce((s, c, j) => s + c * x[j], 0) + D * u;
        for (let k = 0; k < nSub && n; k++) x = WB.math.rk4Step(f, x, u, h);
        return y;
      },
    };
  }

  // The repo's transferFunction class (loopshape_tools.py): controllable canonical
  // form with the first state on top, one RK4 step per Ts, and the output taken
  // after the state update (with the repo's indexing bug for strictly proper
  // numerators fixed; see studies/C/ISSUES.md). Use this, not filter(), to match
  // a ctrlLoopshape.py.
  function repoFilter(G, Ts) {
    let num = G.num.slice(), den = G.den.slice();
    if (den[0] !== 1) { const k = den[0]; num = num.map((v) => v / k); den = den.map((v) => v / k); }
    const n = den.length - 1, pad = n + 1 - num.length;
    const A = L.zeros(n, n), B = new Array(n).fill(0), Cv = new Array(n).fill(0);
    for (let i = 0; i < n; i++) A[0][i] = -den[i + 1];
    for (let i = 1; i < n; i++) A[i][i - 1] = 1;
    if (n) B[0] = 1;
    const D = pad ? 0 : num[0];
    for (let i = 0; i < n; i++) Cv[i] = pad ? (i + 1 >= pad ? num[i + 1 - pad] : 0) : num[i + 1] - num[0] * den[i + 1];
    let x = new Array(n).fill(0);
    const f = (xx, u) => xx.map((_, i) => A[i].reduce((s, a, j) => s + a * xx[j], 0) + B[i] * u);
    return {
      update(u) {
        if (n) x = WB.math.rk4Step(f, x, u, Ts);
        return Cv.reduce((s, c, j) => s + c * x[j], 0) + D * u;
      },
    };
  }

  // Least squares A x = b by modified Gram-Schmidt on unit-norm columns, or null
  // when the columns are (nearly) dependent.
  function lsq(A, b) {
    const k = A[0].length, dot = (u, v) => u.reduce((s, x, i) => s + x * v[i], 0);
    const cols = Array.from({ length: k }, (_, j) => A.map((r) => r[j]));
    const nrm = cols.map((c) => Math.sqrt(dot(c, c)) || 1);
    const Q = cols.map((c, j) => c.map((v) => v / nrm[j]));
    const R = Array.from({ length: k }, () => new Array(k).fill(0));
    for (let j = 0; j < k; j++) {
      for (let i = 0; i < j; i++) { R[i][j] = dot(Q[i], Q[j]); Q[j] = Q[j].map((v, r) => v - R[i][j] * Q[i][r]); }
      R[j][j] = Math.sqrt(dot(Q[j], Q[j]));
      if (R[j][j] < 1e-12) return null;
      Q[j] = Q[j].map((v) => v / R[j][j]);
    }
    const z = Q.map((q) => dot(q, b));
    for (let j = k - 1; j >= 0; j--) { for (let i = j + 1; i < k; i++) z[j] -= R[j][i] * z[i]; z[j] /= R[j][j]; }
    return z.map((v, j) => v / nrm[j]);
  }

  // The lowest-order proper G(s) (denominator degree <= maxN, real coefficients)
  // that reproduces samples P at complex points sPts to 1e-6, or null. Levy's
  // linear least squares on num(s) - P den(s) = 0 with den monic, with P scaled to
  // unit size and solved by QR (fourth-order plants are too ill-conditioned for
  // the normal equations).
  function fit(sPts, P, maxN = 4) {
    const scale = Math.max(...P.map((v) => Math.hypot(v.re, v.im)), 1e-300);
    const Pn = P.map((v) => C.of(v.re / scale, v.im / scale));
    for (let n = 0; n <= maxN; n++) {
      for (let m = 0; m <= n; m++) {
        // unknowns: a_0..a_{n-1} (den), b_0..b_m (num); rows: real and imaginary parts
        const rows = [], rhs = [];
        sPts.forEach((s, i) => {
          const pw = [C.of(1)];
          for (let k = 1; k <= n; k++) pw.push(C.mul(pw[k - 1], s));
          const cols = [...pw.slice(0, n).map((w) => C.mul(Pn[i], w)), ...pw.slice(0, m + 1).map((w) => C.mul(C.of(-1), w))];
          const r = C.mul(C.of(-1), C.mul(Pn[i], pw[n]));
          rows.push(cols.map((c) => c.re), cols.map((c) => c.im));
          rhs.push(r.re, r.im);
        });
        const x = lsq(rows, rhs);
        if (!x || !x.every(Number.isFinite)) continue;
        const G = { num: x.slice(n).reverse().map((v) => v * scale), den: [1, ...x.slice(0, n).reverse()] };
        const ok = sPts.every((s, i) => {
          const g = C.div(L.polyvalC(G.num, s), L.polyvalC(G.den, s));
          return Math.hypot(g.re - P[i].re, g.im - P[i].im) <= 1e-6 * scale;
        });
        if (ok) return G;
      }
    }
    return null;
  }

  // LaTeX for a polynomial and a transfer function.
  function polyTex(p, v = 's', sig = 4) {
    const n = p.length - 1;
    const terms = [];
    p.forEach((c, i) => {
      if (Math.abs(c) < 1e-12) return;
      const pow = n - i;
      const mag = Math.abs(c);
      const coef = pow > 0 && Math.abs(mag - 1) < 1e-12 ? '' : WB.math.tex(mag, sig);
      const vv = pow === 0 ? '' : pow === 1 ? v : `${v}^{${pow}}`;
      terms.push({ neg: c < 0, body: `${coef}${coef && vv ? '\\,' : ''}${vv}` || '1' });
    });
    if (!terms.length) return '0';
    return terms.map((t, i) => (i === 0 ? (t.neg ? '-' : '') : t.neg ? ' - ' : ' + ') + t.body).join('');
  }
  const texTf = (G, sig = 4) => `\\frac{${polyTex(G.num, 's', sig)}}{${polyTex(G.den, 's', sig)}}`;

  return {
    tf, mul, add, gain, feedback, at, mag, db, bode, logspace, margins, crossDown, bandwidth, rootLocus,
    pid, lead, lag, lpf, pi, poles, zeros, dcgain, ss, filter, repoFilter, fit, polyTex, texTf,
  };
})();
