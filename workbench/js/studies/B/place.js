// Pole placement for Design Study B, ported from scipy.signal.place_poles
// (method 'YT', rtol 1e-3, maxiter 30), which is what control.place calls.
//
// Why a port: the B.13 and B.14 observers have two measured outputs, so the
// observer gain L (4x2 or 5x2) is not unique. Any L that puts eig(A - LC) at the
// requested poles is "correct", but the repo's ctrlObserver.py and
// ctrlDisturbanceObserver.py get theirs from scipy's YT iteration. To match those
// controllers to machine precision the workbench has to land on the same L, which
// means reproducing the same iteration from the same starting point. The starting
// point comes from LAPACK Householder QR factorizations, so the QR below follows
// LAPACK's dgeqr2/dorg2r (zgeqr2/zung2r for complex poles) step by step.
//
// Single-input designs (B.11, B.12) have a unique gain; they go through the same
// code so every B gain is computed one way.
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.B = WB.studies.B || { chapters: {} };

(function () {
  // ------------------------------------------------------- complex scalars --
  const cx = (re, im = 0) => ({ re, im });
  const cadd = (a, b) => cx(a.re + b.re, a.im + b.im);
  const csub = (a, b) => cx(a.re - b.re, a.im - b.im);
  const cmul = (a, b) => cx(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
  const cconj = (a) => cx(a.re, -a.im);
  const cabs = (a) => Math.hypot(a.re, a.im);
  const cscale = (a, k) => cx(a.re * k, a.im * k);
  function cdiv(a, b) {
    // Smith's algorithm, as zladiv does, to avoid overflow
    if (Math.abs(b.im) <= Math.abs(b.re)) {
      const r = b.im / b.re, d = b.re + b.im * r;
      return cx((a.re + a.im * r) / d, (a.im - a.re * r) / d);
    }
    const r = b.re / b.im, d = b.im + b.re * r;
    return cx((a.re * r + a.im) / d, (a.im * r - a.re) / d);
  }
  const isReal = (p) => p.im === 0;

  function lapy3(x, y, z) {
    const w = Math.max(Math.abs(x), Math.abs(y), Math.abs(z));
    if (w === 0) return Math.abs(x) + Math.abs(y) + Math.abs(z);
    return w * Math.sqrt((x / w) ** 2 + (y / w) ** 2 + (z / w) ** 2);
  }
  function nrm2(v) {  // scaled 2-norm of a complex vector (dznrm2)
    let scale = 0, ssq = 1;
    for (const c of v) {
      for (const t of [c.re, c.im]) {
        if (t !== 0) {
          const a = Math.abs(t);
          if (scale < a) { ssq = 1 + ssq * (scale / a) ** 2; scale = a; } else ssq += (a / scale) ** 2;
        }
      }
    }
    return scale * Math.sqrt(ssq);
  }

  // ------------------------------------------------- LAPACK Householder QR --
  // Full QR of an m x n complex matrix A (rows of {re, im}); returns Q (m x m).
  // zgeqr2 builds H(i) = I - tau v v^H with zlarfg, applies H(i)^H to the trailing
  // columns, and zung2r accumulates Q = H(1) ... H(k) into the identity.
  function qrFull(Ain) {
    const m = Ain.length, n = Ain[0].length, k = Math.min(m, n);
    const A = Ain.map((r) => r.map((c) => cx(c.re, c.im)));
    const tau = [];
    // apply H = I - t v v^H (v[0] = 1 implicitly stored at A[i][i]) to columns c0.. of rows i..
    const applyLeft = (i, t, cols, M) => {
      const v = []; for (let r = i; r < m; r++) v.push(r === i ? cx(1) : M[r][i]);
      for (const j of cols) {
        let w = cx(0);  // w = v^H C(:, j)
        for (let r = i; r < m; r++) w = cadd(w, cmul(cconj(v[r - i]), M[r][j]));
        const tw = cmul(t, w);
        for (let r = i; r < m; r++) M[r][j] = csub(M[r][j], cmul(v[r - i], tw));
      }
    };
    for (let i = 0; i < k; i++) {
      // zlarfg
      const alpha = A[i][i];
      const x = []; for (let r = i + 1; r < m; r++) x.push(A[r][i]);
      const xnorm = nrm2(x);
      let t = cx(0);
      if (!(xnorm === 0 && alpha.im === 0)) {
        let beta = lapy3(alpha.re, alpha.im, xnorm);
        beta = alpha.re >= 0 ? -beta : beta;  // -SIGN(lapy3, alphr)
        t = cx((beta - alpha.re) / beta, -alpha.im / beta);
        const sc = cdiv(cx(1), csub(alpha, cx(beta)));
        for (let r = i + 1; r < m; r++) A[r][i] = cmul(A[r][i], sc);
        A[i][i] = cx(beta);
      }
      tau.push(t);
      if (i < n - 1) {
        const save = A[i][i]; A[i][i] = cx(1);
        const cols = []; for (let j = i + 1; j < n; j++) cols.push(j);
        applyLeft(i, cconj(t), cols, A);  // H(i)^H
        A[i][i] = save;
      }
    }
    // zung2r into an m x m matrix whose first n columns hold the reflectors
    const Q = Array.from({ length: m }, (_, r) => Array.from({ length: m }, (_, j) => (j < n ? A[r][j] : cx(0))));
    for (let j = k; j < m; j++) { for (let r = 0; r < m; r++) Q[r][j] = cx(0); Q[j][j] = cx(1); }
    for (let i = k - 1; i >= 0; i--) {
      if (i < m - 1) {
        Q[i][i] = cx(1);
        const cols = []; for (let j = i + 1; j < m; j++) cols.push(j);
        applyLeft(i, tau[i], cols, Q);
        for (let r = i + 1; r < m; r++) Q[r][i] = cmul(Q[r][i], cx(-tau[i].re, -tau[i].im));
      }
      Q[i][i] = csub(cx(1), tau[i]);
      for (let r = 0; r < i; r++) Q[r][i] = cx(0);
    }
    return { Q, R: A };
  }
  const toC = (M) => M.map((r) => r.map((v) => (typeof v === 'number' ? cx(v) : v)));
  const re = (M) => M.map((r) => r.map((c) => c.re));

  // ---------------------------------------------------- small dense helpers --
  // |det| by LU with partial pivoting (real)
  function absDet(Min) {
    const A = Min.map((r) => r.slice()), n = A.length;
    let d = 1;
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
      if (A[p][c] === 0) return 0;
      [A[c], A[p]] = [A[p], A[c]];
      d *= A[c][c];
      for (let r = c + 1; r < n; r++) {
        const l = A[r][c] / A[c][c];
        for (let j = c; j < n; j++) A[r][j] -= l * A[c][j];
      }
    }
    return Math.abs(d);
  }
  // Solve A X = B for complex A (n x n) and B (n x k), partial pivoting.
  function csolve(Ain, Bin) {
    const n = Ain.length;
    const A = toC(Ain).map((r) => r.slice()), B = toC(Bin).map((r) => r.slice());
    const cab1 = (c) => Math.abs(c.re) + Math.abs(c.im);
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (cab1(A[r][c]) > cab1(A[p][c])) p = r;
      if (cab1(A[p][c]) === 0) throw new Error('singular');
      [A[c], A[p]] = [A[p], A[c]]; [B[c], B[p]] = [B[p], B[c]];
      for (let r = c + 1; r < n; r++) {
        const l = cdiv(A[r][c], A[c][c]);
        for (let j = c; j < n; j++) A[r][j] = csub(A[r][j], cmul(l, A[c][j]));
        for (let j = 0; j < B[0].length; j++) B[r][j] = csub(B[r][j], cmul(l, B[c][j]));
      }
    }
    const X = B.map((r) => r.map(() => cx(0)));
    for (let j = 0; j < B[0].length; j++) {
      for (let r = n - 1; r >= 0; r--) {
        let s = B[r][j];
        for (let q = r + 1; q < n; q++) s = csub(s, cmul(A[r][q], X[q][j]));
        X[r][j] = cdiv(s, A[r][r]);
      }
    }
    return X;
  }
  const col = (M, j) => M.map((r) => r[j]);
  const setCol = (M, j, v) => v.forEach((x, i) => { M[i][j] = x; });
  const dropCols = (M, idx) => M.map((r) => r.filter((_, j) => !idx.includes(j)));
  const allClose0 = (v) => v.every((x) => Math.abs(typeof x === 'number' ? x : cabs(x)) <= 1e-8);
  const allClose = (a, b) => Math.abs(a - b) <= 1e-8 + 1e-5 * Math.abs(b);
  const norm = (v) => Math.sqrt(v.reduce((s, x) => s + (typeof x === 'number' ? x * x : x.re * x.re + x.im * x.im), 0));
  // K (n x q real) times K^T times vector v (real)
  function projReal(K, v) {
    const q = K[0].length, n = K.length;
    const t = new Array(q).fill(0);
    for (let j = 0; j < q; j++) for (let i = 0; i < n; i++) t[j] += K[i][j] * v[i];
    return K.map((row) => row.reduce((s, kij, j) => s + kij * t[j], 0));
  }

  // Order as scipy's _order_complex_poles: real poles sorted, then each complex
  // pole with negative imaginary part (sorted by real, then imaginary) followed by
  // its conjugate.
  function orderPoles(poles) {
    const real = poles.filter(isReal).map((p) => cx(p.re)).sort((a, b) => a.re - b.re);
    const neg = poles.filter((p) => p.im < 0).sort((a, b) => a.re - b.re || a.im - b.im);
    const out = real.slice();
    for (const p of neg) out.push(cx(p.re, p.im), cx(p.re, -p.im));
    if (out.length !== poles.length) throw new Error('complex poles must come in conjugate pairs');
    return out;
  }

  // ---------------------------------------------------------- YT updates --
  function knv0(ker, X, j) {
    const { Q } = qrFull(toC(dropCols(X, [j])));
    const q = re(Q).map((r) => r[r.length - 1]);
    const yj = projReal(ker[j], q);
    if (!allClose0(yj)) { const nn = norm(yj); setCol(X, j, yj.map((v) => v / nn)); }
  }

  // 2x2 real SVD: returns {s: [s1, s2], u: [u1, u2] (columns), v: [v1, v2] (columns)}
  function svd2(M) {
    const [[a, b], [c, d]] = M;
    // eigen-decomposition of M^T M
    const p = a * a + c * c, q = a * b + c * d, r = b * b + d * d;
    const tr = p + r, disc = Math.sqrt(Math.max(0, ((p - r) / 2) ** 2 + q * q));
    const l1 = tr / 2 + disc, l2 = Math.max(0, tr / 2 - disc);
    let v1 = Math.abs(q) > 1e-300 ? [q, l1 - p] : (p >= r ? [1, 0] : [0, 1]);
    const nv = Math.hypot(v1[0], v1[1]); v1 = [v1[0] / nv, v1[1] / nv];
    const v2 = [-v1[1], v1[0]];
    const s1 = Math.sqrt(l1), s2 = Math.sqrt(l2);
    const mv = (v) => [a * v[0] + b * v[1], c * v[0] + d * v[1]];
    const u1 = s1 > 0 ? mv(v1).map((x) => x / s1) : [1, 0];
    const u2 = s2 > 0 ? mv(v2).map((x) => x / s2) : [-u1[1], u1[0]];
    return { s: [s1, s2], u: [u1, u2], v: [v1, v2] };
  }

  function ytReal(ker, Q, X, i, j) {
    const n = X.length, last = Q[0].length;
    const u = Q.map((r) => r[last - 2].re), v = Q.map((r) => r[last - 1].re);
    // m = ker_i^T (u v^T - v u^T) ker_j
    const Ki = ker[i], Kj = ker[j];
    const q = Ki[0].length;
    const W = (r, c) => u[r] * v[c] - v[r] * u[c];
    const Mm = Array.from({ length: q }, (_, a) => Array.from({ length: q }, (_, b) => {
      let s = 0;
      for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) s += Ki[r][a] * W(r, c) * Kj[c][b];
      return s;
    }));
    const { s, u: U, v: V } = svd2(Mm);
    const t = [...col(X, i), ...col(X, j)];
    let kmn;  // 2n x (1 or 2)
    if (!allClose(s[0], s[1])) {
      const a = Ki.map((row) => row[0] * U[0][0] + row[1] * U[0][1]);
      const b = Kj.map((row) => row[0] * V[0][0] + row[1] * V[0][1]);
      kmn = [...a, ...b].map((x) => [x]);
    } else {
      const a1 = Ki.map((row) => row[0] * U[0][0] + row[1] * U[0][1]), a2 = Ki.map((row) => row[0] * U[1][0] + row[1] * U[1][1]);
      const b1 = Kj.map((row) => row[0] * V[0][0] + row[1] * V[0][1]), b2 = Kj.map((row) => row[0] * V[1][0] + row[1] * V[1][1]);
      kmn = [...a1.map((x, r) => [x, a2[r]]), ...b1.map((x, r) => [x, b2[r]])];
    }
    let tij = projReal(kmn, t);
    if (!allClose0(tij)) {
      const nn = norm(tij);
      tij = tij.map((x) => Math.SQRT2 * x / nn);
      setCol(X, i, tij.slice(0, n)); setCol(X, j, tij.slice(n));
    } else {
      setCol(X, i, kmn.slice(0, n).map((r) => r[0])); setCol(X, j, kmn.slice(n).map((r) => r[0]));
    }
  }

  // Eigen-decomposition of a 2x2 Hermitian matrix: eigenvalues ascending by |.|
  function eigHerm2(M) {
    const a = M[0][0].re, d = M[1][1].re, b = M[0][1];
    const mid = (a + d) / 2, rad = Math.sqrt(((a - d) / 2) ** 2 + cabs(b) ** 2);
    const vals = [mid - rad, mid + rad];
    const vecs = vals.map((l) => {
      let v = cabs(b) > 1e-300 ? [b, cx(l - a)] : (Math.abs(l - a) <= Math.abs(l - d) ? [cx(1), cx(0)] : [cx(0), cx(1)]);
      const nn = Math.sqrt(cabs(v[0]) ** 2 + cabs(v[1]) ** 2);
      return v.map((c) => cscale(c, 1 / nn));
    });
    const idx = [0, 1].sort((p, q) => Math.abs(vals[p]) - Math.abs(vals[q]));
    return { vals: idx.map((k) => vals[k]), vecs: idx.map((k) => vecs[k]) };
  }

  function ytComplex(ker, Q, X, i, j) {
    const n = X.length, last = Q[0].length;
    const ur = Q.map((r) => Math.SQRT2 * r[last - 2].re), ui = Q.map((r) => Math.SQRT2 * r[last - 1].re);
    const K = ker[i];  // n x q complex
    const q = K[0].length;
    // u u^H - conj(u) u^T = 2i (ui ur^T - ur ui^T)
    const W = (r, c) => cx(0, 2 * (ui[r] * ur[c] - ur[r] * ui[c]));
    const Mm = Array.from({ length: q }, (_, a) => Array.from({ length: q }, (_, b) => {
      let s = cx(0);
      for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) s = cadd(s, cmul(cmul(cconj(K[r][a]), W(r, c)), K[c][b]));
      return s;
    }));
    const { vals, vecs } = eigHerm2(Mm);
    const mu1 = vecs[1], mu2 = vecs[0];
    const t = col(X, i).map((x, r) => cx(x, X[r][j]));
    const Kmu = (mu) => K.map((row) => row.reduce((s, kk, c) => cadd(s, cmul(kk, mu[c])), cx(0)));
    const cols = !allClose(Math.abs(vals[1]), Math.abs(vals[0])) ? [Kmu(mu1)] : [Kmu(mu1), Kmu(mu2)];
    // P t with P = sum over columns k k^H
    let tij = new Array(n).fill(null).map(() => cx(0));
    for (const kv of cols) {
      let w = cx(0);
      for (let r = 0; r < n; r++) w = cadd(w, cmul(cconj(kv[r]), t[r]));
      tij = tij.map((x, r) => cadd(x, cmul(kv[r], w)));
    }
    if (!allClose0(tij)) {
      const nn = norm(tij);
      tij = tij.map((c) => cscale(c, 1 / nn));
      setCol(X, i, tij.map((c) => c.re)); setCol(X, j, tij.map((c) => c.im));
    } else {
      setCol(X, i, cols[0].map((c) => c.re)); setCol(X, j, cols[0].map((c) => c.im));
    }
  }

  function updateOrder(poles) {
    const nbReal = poles.filter(isReal).length, hnb = Math.floor(nbReal / 2);
    const o0 = [], o1 = [];
    const range = (a, b) => { const r = []; for (let x = a; x < b; x++) r.push(x); return r; };
    if (nbReal > 0) { o0.push(nbReal); o1.push(1); }
    const rComp = []; for (let x = nbReal + 1; x < poles.length + 1; x += 2) rComp.push(x);
    const addComp = () => { o0.push(...rComp); o1.push(...rComp.map((x) => x + 1)); };
    const knv = () => { if (hnb === 0 && isReal(poles[0])) { o0.push(1); o1.push(1); } };
    let rp = range(1, hnb + (nbReal % 2));
    o0.push(...rp.map((x) => 2 * x)); o1.push(...rp.map((x) => 2 * x + 1));
    addComp();
    rp = range(1, hnb + 1);
    o0.push(...rp.map((x) => 2 * x - 1)); o1.push(...rp.map((x) => 2 * x));
    knv(); addComp();
    for (const j of range(2, hnb + (nbReal % 2))) for (let i = 1; i < hnb + 1; i++) { o0.push(i); o1.push(i + j); }
    knv(); addComp();
    for (const j of range(2, hnb + (nbReal % 2))) {
      for (let i = hnb + 1; i < nbReal + 1; i++) { let idx = i + j; if (idx > nbReal) idx = i + j - nbReal; o0.push(i); o1.push(idx); }
    }
    knv(); addComp();
    for (let i = 1; i < hnb + 1; i++) { o0.push(i); o1.push(i + hnb); }
    knv(); addComp();
    return o0.map((x, k) => [x - 1, o1[k] - 1]);
  }

  function ytLoop(ker, X, poles, maxiter = 30, rtol = 1e-3) {
    const order = updateOrder(poles);
    const eps = Math.sqrt(Number.EPSILON);
    let stop = false, nb = 0;
    while (nb < maxiter && !stop) {
      const detB = absDet(X);
      for (const [i, j] of order) {
        if (i === j) knv0(ker, X, i);
        else {
          const { Q } = qrFull(toC(dropCols(X, [i, j])));
          if (isReal(poles[i])) ytReal(ker, Q, X, i, j); else ytComplex(ker, Q, X, i, j);
        }
      }
      const det = Math.max(eps, absDet(X));
      const cur = Math.abs((det - detB) / det);
      if (cur < rtol && det > eps) stop = true;
      nb++;
    }
    return nb;
  }

  // ----------------------------------------------------------- place_poles --
  // A (n x n), B (n x m) real arrays; poles [{re, im}]. Returns the gain K (m x n)
  // with eig(A - B K) = poles, or null if they can't be placed.
  function placeYT(A, B, polesIn) {
    const n = A.length, m = B[0].length;
    const poles = orderPoles(polesIn);
    const { Q: U, R } = qrFull(toC(B));
    const rankB = m;  // B has full column rank in every B design
    const u0 = re(U).map((r) => r.slice(0, rankB)), u1 = re(U).map((r) => r.slice(rankB));
    const z = re(R).slice(0, rankB);
    if (n === rankB) throw new Error('square B not used here');
    const ker = [];
    let X = null, skip = false;
    for (let j = 0; j < n; j++) {
      if (skip) { skip = false; continue; }
      const p = poles[j];
      // pole_space_j = (u1^T (A - p I))^T  : n x (n - rankB)
      const ps = Array.from({ length: n }, (_, r) => Array.from({ length: n - rankB }, (_, c) => {
        let s = cx(0);
        for (let k = 0; k < n; k++) {
          const a = cx(A[k][r] - (k === r ? p.re : 0), k === r ? -p.im : 0);
          s = cadd(s, cscale(a, u1[k][c]));
        }
        return s;
      }));
      const { Q } = qrFull(ps);
      const K = Q.map((r) => r.slice(n - rankB));
      let tj = K.map((r) => r.reduce((s, c) => cadd(s, c), cx(0)));
      const nn = norm(tj);
      tj = tj.map((c) => cscale(c, 1 / nn));
      let colsJ;
      if (!isReal(p)) {
        colsJ = [tj.map((c) => c.re), tj.map((c) => c.im)];
        ker.push(K, K);
        skip = true;
      } else {
        colsJ = [tj.map((c) => c.re)];
        ker.push(re(K));
      }
      if (!X) X = Array.from({ length: n }, () => []);
      for (const cc of colsJ) cc.forEach((v, r) => X[r].push(v));
    }
    if (rankB > 1) ytLoop(ker, X, poles);
    // back to complex eigenvectors and the gain
    const Xc = X.map((r) => r.map((v) => cx(v)));
    for (let idx = 0; idx < n - 1; idx++) {
      if (!isReal(poles[idx])) {
        for (let r = 0; r < n; r++) {
          const rel = X[r][idx], img = X[r][idx + 1];
          Xc[r][idx] = cx(rel, -img); Xc[r][idx + 1] = cx(rel, img);
        }
        idx++;
      }
    }
    try {
      const XT = Xc[0].map((_, j) => Xc.map((r) => r[j]));
      const DXT = XT.map((row, i) => row.map((c) => cmul(poles[i], c)));
      const Mt = csolve(XT, DXT);                       // m^T
      const Mm = Mt[0].map((_, j) => Mt.map((r) => r[j]));
      const rhs = u0[0].map((_, a) => Mm[0].map((__, c) => {  // u0^T (m - A)
        let s = cx(0);
        for (let k = 0; k < n; k++) s = cadd(s, cscale(csub(Mm[k][c], cx(A[k][c])), u0[k][a]));
        return s;
      }));
      const G = csolve(z, rhs);
      return G.map((r) => r.map((c) => -c.re));
    } catch (e) {
      return null;
    }
  }

  // Observer gain L (n x p) with eig(A - L C) = poles: place(A^T, C^T)^T.
  function observerYT(A, C, poles) {
    const AT = A[0].map((_, j) => A.map((r) => r[j]));
    const CT = C[0].map((_, j) => C.map((r) => r[j]));
    const Lt = placeYT(AT, CT, poles);
    return Lt ? Lt[0].map((_, i) => Lt.map((r) => r[i])) : null;
  }

  WB.studies.B.place = { placeYT, observerYT, qrFull, orderPoles };
})();
