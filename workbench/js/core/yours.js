// "Plot my answer" overlays for the modeling chapters (Ch 2-6).
//
// A Python part's "Plot my ..." button runs the student's code, solved or not, and
// the chapter draws what it gives (dotted, labelled "your ...") beside the
// workbench's own model: their K(t), their linear model's response and
// eigenvalues, their P(s) poles, the plant under their F_fl. Only the student's own
// result is drawn, so Work mode gives nothing away. Once plotted, the overlay is
// rerun whenever the chapter's state changes, so it follows the sliders.
//
// Usage in a chapter module:
//   actions: [{ label: 'Plot my A, B', run: (code) => WB.yours.plot(ctx, 'ch4.b', code, runFn, describe) }]
//   splane(ctx) { const y = WB.yours.data(ctx, 'ch4.b'); ... }
// runFn(ctx, code) resolves to overlay data, or to {ok: false, msg} on an error
// (lib.pyError(out) for Python errors). Data with a time series should carry
// n = number of samples, so stale data is kept only while it fits the time axis.
window.WB = window.WB || {};

WB.yours = (function () {
  const L = WB.la;
  const store = {};  // `${sysId}.${id}` -> { code, run, data, key, pending, failed }
  const STYLE = { color: '--series-3', dash: [2, 3], width: 2 };
  const stateKey = (ctx) => JSON.stringify([ctx.S.sysId, ctx.st, ctx.pModel, ctx.pTrue, ctx.S.sim]);
  const slot = (ctx, id) => `${ctx.S.sysId}.${id}`;

  // Button handler: run once now, keep the overlay, redraw, and describe it.
  async function plot(ctx, id, code, run, describe) {
    const key = stateKey(ctx);
    const data = await run(ctx, code);
    if (data.ok === false) return data;
    store[slot(ctx, id)] = { code, run, data, key };
    ctx.update();
    return { info: true, msg: typeof describe === 'function' ? describe(data) : describe };
  }

  // Overlay data for part `id`, or null. A stale overlay is rerun once per state;
  // meanwhile the old data is kept if it still fits the time axis.
  function data(ctx, id) {
    const y = store[slot(ctx, id)];
    if (!y) return null;
    const key = stateKey(ctx);
    if (y.key !== key && y.pending !== key && y.failed !== key) {
      y.pending = key;
      y.run(ctx, y.code).then((d) => {
        if (y.pending !== key) return;
        y.pending = null;
        if (d.ok === false) { y.failed = key; return; }
        y.data = d; y.key = key;
        ctx.update();
      }, () => { y.pending = null; y.failed = key; });
    }
    if (y.key === key) return y.data;
    return !y.data.n || y.data.n === ctx.app.result().t.length ? y.data : null;
  }

  // A dotted "your" trace; it joins the autoscale only while it stays in range
  // (|y| < lim), so a model that blows up doesn't flatten the plot.
  function series(label, y, { lim = 50, ...more } = {}) {
    const fit = Array.prototype.every.call(y, (v) => Number.isFinite(v) && Math.abs(v) < lim);
    return { label, y: Array.from(y), ...STYLE, fit, ...more };
  }

  // The student's value reshaped to r x c (nested or flat), or null.
  function asMat(v, r, c) {
    const flat = [v].flat(Infinity);
    if (flat.length !== r * c || !flat.every((x) => typeof x === 'number' && Number.isFinite(x))) return null;
    return Array.from({ length: r }, (_, i) => flat.slice(i * c, i * c + c));
  }

  // xdot = Ax + Bu, y = Cx + Du on the main simulation's time grid (RK4 at Ts,
  // input held over each step). A n x n, B n x m, C p x n, D p x m (arrays of rows);
  // input(t) -> number or array of m. Returns { y: [p arrays], x: [states] }.
  function linResponse(ctx, { A, B, C, D }, x0, input) {
    const res = ctx.app.result(), Ts = ctx.S.sim.Ts;
    const mv = (Mx, v) => Mx.map((row) => row.reduce((s, a, j) => s + a * v[j], 0));
    const f = (x, u) => mv(A, x).map((v, i) => v + mv(B, u)[i]);
    let x = x0.slice();
    const y = C.map(() => new Float64Array(res.t.length)), xs = [];
    for (let k = 0; k < res.t.length; k++) {
      const uv = input(res.t[k]);
      const u = Array.isArray(uv) ? uv : [uv];
      const yk = mv(C, x).map((v, i) => v + mv(D, u)[i]);
      yk.forEach((v, i) => { y[i][k] = v; });
      xs.push(x);
      if (k < res.t.length - 1) x = WB.math.rk4Step(f, x, u, Ts);
    }
    return { y, x: xs };
  }

  // s-plane markers for the student's eigenvalues / poles (kind 'obs': green ×).
  const eig = (A, label) => L.eig(A).map((p, i) => ({ ...p, kind: 'obs', label: `${label} ${i + 1}` }));
  const poles = (list, label) => list.map((p, i) => ({ ...p, kind: 'obs', label: `${label} ${i + 1}` }));
  const zeros = (list, label) => list.map((p, i) => ({ ...p, kind: 'zero', label: `${label} ${i + 1}` }));
  // Add the student's markers (and legend names, e.g. {obs: 'your A'}) to an s-plane result.
  function withMarkers(sp, markers, legend) {
    if (!markers.length) return sp;
    sp = sp || { markers: [] };
    return { ...sp, markers: [...sp.markers, ...markers], legendNames: { ...(sp.legendNames || {}), ...legend } };
  }

  // Fixed complex sample points for fitting a student's transfer function.
  const S_PTS = Array.from({ length: 16 }, (_, i) => ({ re: -3 + 5 * ((i * 0.618034) % 1), im: 0.3 + 5.7 * ((i * 0.381966 + 0.1) % 1) }));
  // Evaluate fn(s) (a student transfer function) at S_PTS and fit G = num/den.
  // extraArgs are appended after s. Resolves to {G} or {ok: false, msg}.
  async function fitTf(ctx, code, fn, { params = ctx.pModel, extraArgs = [], maxN = 4 } = {}) {
    const out = await WB.py.evaluate(code, [{ params, calls: S_PTS.map((s) => ({ name: fn, args: [s, ...extraArgs] })) }]);
    if (out.error) return pyError(out);
    const P = out.rows[0].calls.map((v) => (typeof v === 'number' ? { re: v, im: 0 } : v));
    if (!P.every((v) => v && Number.isFinite(v.re) && Number.isFinite(v.im))) return { ok: false, msg: `${fn} should return one (complex) number.` };
    const G = WB.tf.fit(S_PTS, P, maxN);
    if (!G) return { ok: false, msg: `Your ${fn} is not a proper ratio of polynomials in s of degree ${maxN} or less, so there is no model to plot.` };
    return { G };
  }
  // Response of a fitted G(s) from rest to input(t), as one array.
  function tfResponse(ctx, G, input) {
    const r = WB.tf.ss(G);
    const A = r.n ? r.A : [], B = r.n ? r.B.map((b) => [b]) : [], C = [r.n ? r.C : []], D = [[r.D]];
    return linResponse(ctx, { A, B, C, D }, new Array(r.n).fill(0), input).y[0];
  }

  const pyError = (out) => ({ ok: false, msg: out.timeout ? out.error : 'Python raised an error.', detail: [out.error, out.where, (out.stdout || '').trim()].filter(Boolean).join('\n') });

  return { plot, data, series, asMat, linResponse, eig, poles, zeros, withMarkers, fitTf, tfResponse, pyError, STYLE, S_PTS };
})();
