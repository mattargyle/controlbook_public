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
  function margins(Lg, wlo = -4, whi = 5) {
    const ws = logspace(wlo, whi, 4000);
    const { mag, phase } = bode(Lg, ws);
    let wc = NaN, pm = Infinity, w180 = NaN, gm = Infinity;
    const crossings = [];  // every phase crossing of -180 (+k·360): conditionally stable loops have several
    for (let i = 1; i < ws.length; i++) {
      if (isNaN(wc) && (mag[i - 1] - 1) * (mag[i] - 1) <= 0) {
        const t = Math.log(mag[i - 1]) / (Math.log(mag[i - 1]) - Math.log(mag[i]));
        wc = Math.exp(Math.log(ws[i - 1]) + t * (Math.log(ws[i]) - Math.log(ws[i - 1])));
        const ph = phase[i - 1] + t * (phase[i] - phase[i - 1]);
        pm = ((180 + ph) % 360 + 540) % 360 - 180;  // wrap into [-180, 180)
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
    return { pm, wc, gm, w180, crossings };
  }

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

  return { tf, mul, add, gain, feedback, at, bode, logspace, margins, poles, zeros, dcgain, ss, filter, polyTex, texTf };
})();
