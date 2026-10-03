// Numerical helpers shared by every system and chapter.
// Classic script (no modules) so the page works when opened from file://.
window.WB = window.WB || {};

WB.math = (function () {
  const DEG = Math.PI / 180;

  function saturate(u, limit) {
    return Math.max(-limit, Math.min(limit, u));
  }

  // One RK4 step of xdot = f(x, u) with u held constant (matches rk4_step in the repo).
  function rk4Step(f, x, u, Ts) {
    const add = (a, b, k) => a.map((ai, i) => ai + k * b[i]);
    const F1 = f(x, u);
    const F2 = f(add(x, F1, Ts / 2), u);
    const F3 = f(add(x, F2, Ts / 2), u);
    const F4 = f(add(x, F3, Ts), u);
    return x.map((xi, i) => xi + (Ts / 6) * (F1[i] + 2 * F2[i] + 2 * F3[i] + F4[i]));
  }

  // Roots of s^2 + a1 s + a0, returned as [{re, im}, {re, im}].
  function roots2(a1, a0) {
    const disc = (a1 * a1) / 4 - a0;
    if (disc >= 0) {
      const r = Math.sqrt(disc);
      return [{ re: -a1 / 2 + r, im: 0 }, { re: -a1 / 2 - r, im: 0 }];
    }
    const w = Math.sqrt(-disc);
    return [{ re: -a1 / 2, im: w }, { re: -a1 / 2, im: -w }];
  }

  // (s - p1)(s - p2) = s^2 + alpha1 s + alpha0 for a real pair or a conjugate pair.
  function polyFromPoles(p1, p2) {
    return { alpha1: -(p1.re + p2.re), alpha0: p1.re * p2.re - p1.im * p2.im };
  }

  // Natural frequency / damping of a pole pair from its characteristic polynomial.
  function wnZeta(alpha1, alpha0) {
    if (alpha0 <= 0) return { wn: NaN, zeta: NaN };
    const wn = Math.sqrt(alpha0);
    return { wn, zeta: alpha1 / (2 * wn) };
  }

  // Step-response metrics on y over the window starting at index i0.
  // y0 = value before the step, yf = commanded final value.
  function stepMetrics(t, y, i0, i1, y0, yf) {
    const span = yf - y0;
    const out = { tr: NaN, os: NaN, ts: NaN, ess: NaN, tPeak: NaN };
    if (Math.abs(span) < 1e-12 || i1 - i0 < 3) return out;
    const n = (k) => (y[k] - y0) / span; // normalized response, 0 -> 1
    let t10 = NaN, t90 = NaN, peak = -Infinity, iPeak = i0;
    for (let k = i0; k < i1; k++) {
      const v = n(k);
      if (isNaN(t10) && v >= 0.1) t10 = t[k];
      if (isNaN(t90) && v >= 0.9) t90 = t[k];
      if (v > peak) { peak = v; iPeak = k; }
    }
    out.tr = t90 - t10;
    out.t10 = t10; out.t90 = t90;
    out.os = Math.max(0, (peak - 1) * 100);
    out.tPeak = t[iPeak] - t[i0];
    out.iPeak = iPeak;
    let lastOut = i0;
    for (let k = i0; k < i1; k++) if (Math.abs(n(k) - 1) > 0.02) lastOut = k;
    out.ts = lastOut < i1 - 1 ? t[lastOut + 1] - t[i0] : NaN; // NaN = never settled in window
    out.ess = yf - y[i1 - 1];
    return out;
  }

  // Significant-figure formatting that never prints "-0".
  function fmt(x, sig = 4) {
    if (x === undefined || x === null || isNaN(x)) return '—';
    if (!isFinite(x)) return x > 0 ? '∞' : '−∞';
    if (Math.abs(x) < 1e-12) return '0';
    const s = Number(x.toPrecision(sig));
    const abs = Math.abs(s);
    let str = abs >= 1e5 || abs < 1e-3 ? s.toExponential(sig - 1) : String(s);
    return str.replace('-', '−');
  }

  // Same as fmt but uses an ASCII hyphen-minus (for KaTeX source).
  function tex(x, sig = 4) {
    return fmt(x, sig).replace('−', '-').replace(/e([+-]?)(\d+)/, (m, sgn, d) => `\\times 10^{${sgn === '-' ? '-' : ''}${d}}`);
  }

  function texPole(p, sig = 3) {
    if (Math.abs(p.im) < 1e-9) return tex(p.re, sig);
    return `${tex(p.re, sig)} ${p.im >= 0 ? '+' : '-'} ${tex(Math.abs(p.im), sig)}j`;
  }

  function fmtPole(p, sig = 3) {
    if (Math.abs(p.im) < 1e-9) return fmt(p.re, sig);
    return `${fmt(p.re, sig)} ${p.im >= 0 ? '+' : '−'} ${fmt(Math.abs(p.im), sig)}j`;
  }

  // Parse "−3", "-0.5+2j", "1 - 2.5i" into {re, im}; returns null if unparseable.
  function parseComplex(str) {
    if (str === undefined || str === null) return null;
    const s = String(str).replace(/\s+/g, '').replace(/−/g, '-').replace(/i/g, 'j');
    if (s === '') return null;
    const m = s.match(/^([+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?)?(?:([+-])((?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?)?\*?j)?$/i);
    if (m && (m[1] !== undefined || m[2] !== undefined)) {
      const re = m[1] !== undefined ? parseFloat(m[1]) : 0;
      const im = m[2] !== undefined ? (m[2] === '-' ? -1 : 1) * (m[3] !== undefined ? parseFloat(m[3]) : 1) : 0;
      return { re, im };
    }
    const pureIm = s.match(/^([+-]?(?:\d+\.?\d*|\.\d+)?)\*?j$/);
    if (pureIm) {
      const c = pureIm[1];
      return { re: 0, im: c === '' || c === '+' ? 1 : c === '-' ? -1 : parseFloat(c) };
    }
    return null;
  }

  // Relative-or-absolute closeness used by answer checking.
  function close(a, b, rel = 0.01, abs = 1e-3) {
    return Math.abs(a - b) <= Math.max(abs, rel * Math.abs(b));
  }

  // Order-insensitive match of two pole pairs.
  function polesMatch(given, truth, rel = 0.01, abs = 1e-3) {
    const ok = (p, q) => close(p.re, q.re, rel, abs) && close(p.im, q.im, rel, abs);
    return (ok(given[0], truth[0]) && ok(given[1], truth[1])) || (ok(given[0], truth[1]) && ok(given[1], truth[0]));
  }

  // LaTeX bmatrix from an array of rows (or a flat array as a column).
  function texMat(Mx, sig = 4) {
    const rows = Array.isArray(Mx[0]) ? Mx : Mx.map((v) => [v]);
    return `\\begin{bmatrix}${rows.map((r) => r.map((v) => tex(v, sig)).join(' & ')).join(' \\\\ ')}\\end{bmatrix}`;
  }

  return { DEG, saturate, parseComplex, texMat, close, polesMatch, rk4Step, roots2, polyFromPoles, wnZeta, stepMetrics, fmt, tex, texPole, fmtPole };
})();
