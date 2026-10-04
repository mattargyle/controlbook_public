// Small dense linear algebra, polynomials, and complex helpers for SISO design
// (Ch 11-18). Matrices are arrays of rows; polynomials are coefficient arrays in
// descending powers, like numpy (e.g. [1, 3, 2] = s^2 + 3s + 2).
window.WB = window.WB || {};

WB.la = (function () {
  // ------------------------------------------------------------ complex --
  const C = {
    of: (re, im = 0) => ({ re, im }),
    add: (a, b) => ({ re: a.re + b.re, im: a.im + b.im }),
    sub: (a, b) => ({ re: a.re - b.re, im: a.im - b.im }),
    mul: (a, b) => ({ re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re }),
    div: (a, b) => {
      const d = b.re * b.re + b.im * b.im;
      return { re: (a.re * b.re + a.im * b.im) / d, im: (a.im * b.re - a.re * b.im) / d };
    },
    abs: (a) => Math.hypot(a.re, a.im),
    arg: (a) => Math.atan2(a.im, a.re),
  };

  // ------------------------------------------------------------ polynomials --
  function conv(a, b) {
    const out = new Array(a.length + b.length - 1).fill(0);
    for (let i = 0; i < a.length; i++) for (let j = 0; j < b.length; j++) out[i + j] += a[i] * b[j];
    return out;
  }

  function polyAdd(a, b) {
    const n = Math.max(a.length, b.length);
    const pa = new Array(n - a.length).fill(0).concat(a);
    const pb = new Array(n - b.length).fill(0).concat(b);
    return trimLeading(pa.map((v, i) => v + pb[i]));
  }

  function polyScale(a, k) { return a.map((v) => v * k); }

  function trimLeading(p) {
    let i = 0;
    while (i < p.length - 1 && Math.abs(p[i]) < 1e-14) i++;
    return p.slice(i);
  }

  // Evaluate polynomial at complex s (Horner).
  function polyvalC(p, s) {
    let acc = C.of(0);
    for (const c of p) acc = C.add(C.mul(acc, s), C.of(c));
    return acc;
  }

  // Real-coefficient monic polynomial from roots (conjugates assumed paired).
  function polyFromRoots(roots) {
    let p = [C.of(1)];
    for (const r of roots) {
      const next = new Array(p.length + 1).fill(null).map(() => C.of(0));
      for (let i = 0; i < p.length; i++) {
        next[i] = C.add(next[i], p[i]);
        next[i + 1] = C.sub(next[i + 1], C.mul(p[i], r));
      }
      p = next;
    }
    return p.map((c) => c.re);
  }

  // All roots of a real polynomial (Durand-Kerner), returned as {re, im}.
  function roots(pIn) {
    const p = trimLeading(pIn);
    const n = p.length - 1;
    if (n < 1) return [];
    if (n === 1) return [C.of(-p[1] / p[0])];
    if (n === 2) {
      const [a, b, c] = p;
      const disc = b * b - 4 * a * c;
      if (disc >= 0) {
        const s = Math.sqrt(disc);
        return [C.of((-b + s) / (2 * a)), C.of((-b - s) / (2 * a))];
      }
      const w = Math.sqrt(-disc) / (2 * a);
      return [C.of(-b / (2 * a), w), C.of(-b / (2 * a), -w)];
    }
    const monic = p.map((c) => c / p[0]);
    const R = 1 + Math.max(...monic.slice(1).map(Math.abs));
    let z = Array.from({ length: n }, (_, k) => ({ re: R * Math.cos(2 * Math.PI * k / n + 0.4), im: R * Math.sin(2 * Math.PI * k / n + 0.4) }));
    for (let it = 0; it < 500; it++) {
      let delta = 0;
      z = z.map((zi, i) => {
        let den = C.of(1);
        for (let j = 0; j < n; j++) if (j !== i) den = C.mul(den, C.sub(zi, z[j]));
        const step = C.div(polyvalC(monic, zi), den);
        delta = Math.max(delta, C.abs(step));
        return C.sub(zi, step);
      });
      if (delta < 1e-13) break;
    }
    // clean tiny imaginary parts and sort: real parts descending, then imag
    return z.map((r) => (Math.abs(r.im) < 1e-9 * Math.max(1, Math.abs(r.re)) ? C.of(r.re) : r))
      .sort((a, b) => b.re - a.re || b.im - a.im);
  }

  // ------------------------------------------------------------ matrices --
  const zeros = (n, m) => Array.from({ length: n }, () => new Array(m).fill(0));
  const eye = (n) => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
  const T = (A) => A[0].map((_, j) => A.map((row) => row[j]));
  const mul = (A, B) => A.map((row) => B[0].map((_, j) => row.reduce((s, a, k) => s + a * B[k][j], 0)));
  const add = (A, B) => A.map((row, i) => row.map((v, j) => v + B[i][j]));
  const sub = (A, B) => A.map((row, i) => row.map((v, j) => v - B[i][j]));
  const scale = (A, k) => A.map((row) => row.map((v) => v * k));
  const col = (v) => v.map((x) => [x]);
  const hstack = (...Ms) => Ms[0].map((_, i) => Ms.flatMap((M) => M[i]));
  const vstack = (...Ms) => Ms.flat().map((r) => r.slice());

  function inv(A) {
    const n = A.length;
    const M = A.map((row, i) => row.concat(eye(n)[i]));
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
      if (Math.abs(M[piv][c]) < 1e-14) return null;
      [M[c], M[piv]] = [M[piv], M[c]];
      const d = M[c][c];
      for (let j = 0; j < 2 * n; j++) M[c][j] /= d;
      for (let r = 0; r < n; r++) {
        if (r === c) continue;
        const f = M[r][c];
        for (let j = 0; j < 2 * n; j++) M[r][j] -= f * M[c][j];
      }
    }
    return M.map((row) => row.slice(n));
  }

  // Determinant by LU with partial pivoting.
  function det(A) {
    const M = A.map((r) => r.slice()), n = M.length;
    let d = 1;
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
      if (M[piv][c] === 0) return 0;
      if (piv !== c) { [M[c], M[piv]] = [M[piv], M[c]]; d = -d; }
      d *= M[c][c];
      for (let r = c + 1; r < n; r++) {
        const f = M[r][c] / M[c][c];
        for (let j = c; j < n; j++) M[r][j] -= f * M[c][j];
      }
    }
    return d;
  }

  function rank(A, tol = 1e-9) {
    const M = A.map((r) => r.slice());
    const n = M.length, m = M[0].length;
    const scaleTol = tol * Math.max(1, ...M.flat().map(Math.abs));
    let r = 0;
    for (let c = 0; c < m && r < n; c++) {
      let piv = r;
      for (let i = r + 1; i < n; i++) if (Math.abs(M[i][c]) > Math.abs(M[piv][c])) piv = i;
      if (Math.abs(M[piv][c]) < scaleTol) continue;
      [M[r], M[piv]] = [M[piv], M[r]];
      for (let i = r + 1; i < n; i++) {
        const f = M[i][c] / M[r][c];
        for (let j = c; j < m; j++) M[i][j] -= f * M[r][j];
      }
      r++;
    }
    return r;
  }

  // Controllability matrix [B, AB, ..., A^{n-1}B] (Eq. 11.x) and observability
  // matrix [C; CA; ...; CA^{n-1}].
  function ctrb(A, B) {
    const blocks = [B];
    for (let i = 1; i < A.length; i++) blocks.push(mul(A, blocks[i - 1]));
    return hstack(...blocks);
  }
  function obsv(A, Cm) { return T(ctrb(T(A), T(Cm))); }

  // Characteristic polynomial det(sI - A) via Faddeev-LeVerrier.
  function charPoly(A) {
    const n = A.length;
    const c = [1];
    let M = zeros(n, n);
    for (let k = 1; k <= n; k++) {
      M = add(mul(A, M), scale(eye(n), c[k - 1]));
      const AM = mul(A, M);
      let tr = 0;
      for (let i = 0; i < n; i++) tr += AM[i][i];
      c.push(-tr / k);
    }
    return c;
  }

  const eig = (A) => roots(charPoly(A));

  // p(A) = p0 A^n + p1 A^{n-1} + ... + pn I
  function polyMat(p, A) {
    let R = zeros(A.length, A.length);
    for (const c of p) R = add(mul(R, A), scale(eye(A.length), c));
    return R;
  }

  // Ackermann's formula: K = [0 ... 0 1] C_AB^{-1} Delta_d(A). Returns a 1xn row,
  // or null if (A, B) is not controllable.
  function place(A, B, poles) {
    const n = A.length;
    const Cab = ctrb(A, B);
    if (rank(Cab) < n) return null;
    const Ci = inv(Cab);
    if (!Ci) return null;
    const en = [new Array(n).fill(0).map((_, i) => (i === n - 1 ? 1 : 0))];
    return mul(mul(en, Ci), polyMat(polyFromRoots(poles), A));
  }

  return {
    C, conv, polyAdd, polyScale, polyvalC, polyFromRoots, roots, trimLeading,
    zeros, eye, T, mul, add, sub, scale, col, hstack, vstack, inv, det, rank, ctrb, obsv, charPoly, eig, polyMat, place,
  };
})();
