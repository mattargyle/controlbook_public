// Generic closed-loop simulation mirroring the repo's hwNN_*Sim.py loop:
//   r = reference(t); u = controller.update(r, y_m); x = plant.update(u + d)
// with the controller's output saturated (as the ctrl*.py classes do) and the
// plant input saturated again after the disturbance is added (as
// <sys>Dynamics.update does). y_m = h(x) + measurement noise.
//
// Signals may be scalars (single-input/single-output systems such as the arm)
// or arrays (multi-output / multi-input systems). Channel 0 of every signal is
// also stored in the scalar result fields (y, r, u, ...) so SISO code keeps working.
window.WB = window.WB || {};

WB.sim = (function () {
  const { rk4Step } = WB.math;
  const asArr = (v) => (Array.isArray(v) ? v : [v]);

  // Saturation for input i: limit is a number (±limit), an array of numbers, or
  // an array of [lo, hi] ranges.
  function sat(u, limit, i) {
    const L = Array.isArray(limit) ? limit[i] : limit;
    if (Array.isArray(L)) return Math.max(L[0], Math.min(L[1], u));
    return Math.max(-L, Math.min(L, u));
  }

  // plant: { f(x, u) -> xdot, h(x) -> y (number or array), uLimit }
  // controller: { update(r, x, yMeas, t) -> u | [u...] | { u, ...extras } }
  //   x is the true state; only full-state-feedback chapters may use it.
  //   Extra numeric fields returned are recorded in result.extras.
  function simulate({ plant, controller, reference, disturbance, noise, x0, Ts, tEnd }) {
    const N = Math.round(tEnd / Ts) + 1;
    const F = () => new Float64Array(N);
    const out = {
      t: F(), r: F(), y: F(), yMeas: F(), uDemand: F(), u: F(), uApplied: F(),
      rAll: [], yAll: [], yMeasAll: [], uDemandAll: [], uAll: [], uAppliedAll: [],
      x: [], extras: {},
    };
    const ensure = (arr, n) => { while (arr.length < n) arr.push(F()); };
    let x = x0.slice();
    let vectorU = false, vectorY = false, vectorR = false;
    for (let k = 0; k < N; k++) {
      const t = k * Ts;
      const yv = plant.h(x);
      vectorY = vectorY || Array.isArray(yv);
      const y = asArr(yv);
      const n = noise ? asArr(noise(k)) : [];
      const yMeas = y.map((v, i) => v + (n[i] || 0));
      const rv = reference(t);
      vectorR = vectorR || Array.isArray(rv);
      const r = asArr(rv);
      const ret = controller.update(vectorR ? r : r[0], x, vectorY ? yMeas : yMeas[0], t);
      let uRaw = ret;
      if (ret && typeof ret === 'object' && !Array.isArray(ret)) {
        uRaw = ret.u;
        for (const [key, v] of Object.entries(ret)) {
          if (key === 'u' || typeof v !== 'number') continue;
          (out.extras[key] = out.extras[key] || F())[k] = v;
        }
      }
      vectorU = vectorU || Array.isArray(uRaw);
      const uD = asArr(uRaw);
      const d = disturbance ? asArr(disturbance(t)) : [];
      const u = uD.map((v, i) => sat(v, plant.uLimit, i));
      const uA = u.map((v, i) => sat(v + (d[i] || 0), plant.uLimit, i));
      ensure(out.yAll, y.length); ensure(out.yMeasAll, y.length); ensure(out.rAll, r.length);
      ensure(out.uDemandAll, uD.length); ensure(out.uAll, uD.length); ensure(out.uAppliedAll, uD.length);
      out.t[k] = t;
      y.forEach((v, i) => { out.yAll[i][k] = v; out.yMeasAll[i][k] = yMeas[i]; });
      r.forEach((v, i) => { out.rAll[i][k] = v; });
      uD.forEach((v, i) => { out.uDemandAll[i][k] = v; out.uAll[i][k] = u[i]; out.uAppliedAll[i][k] = uA[i]; });
      out.x.push(x);
      if (k < N - 1) x = rk4Step((xx, uu) => plant.f(xx, uu), x, vectorU ? uA : uA[0], Ts);
    }
    out.y = out.yAll[0]; out.yMeas = out.yMeasAll[0]; out.r = out.rAll[0];
    out.uDemand = out.uDemandAll[0]; out.u = out.uAll[0]; out.uApplied = out.uAppliedAll[0];
    return out;
  }

  // Reference generators (match signalGenerator.py; the step/square start at tStep).
  function makeReference({ type, amplitude, offset = 0, frequency, tStep }) {
    if (type === 'square') {
      return (t) => {
        if (t < tStep) return offset;
        const tt = t - tStep;
        return (tt % (1 / frequency) <= 0.5 / frequency ? amplitude : -amplitude) + offset;
      };
    }
    if (type === 'sine') {
      return (t) => (t < tStep ? offset : amplitude * Math.sin(2 * Math.PI * frequency * (t - tStep)) + offset);
    }
    return (t) => (t >= tStep ? amplitude + offset : offset);
  }

  // Repeatable Gaussian noise: same seed -> same sequence, so the plots don't
  // flicker every time a slider moves. Returns noise(k) for sample index k.
  function makeNoise(sigma, seed = 1) {
    if (!(sigma > 0)) return null;
    let s = seed >>> 0 || 1;
    const rand = () => { // mulberry32
      s = (s + 0x6D2B79F5) >>> 0;
      let t = s;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    const cache = [];
    return (k) => {
      while (cache.length <= k) {
        const u1 = Math.max(rand(), 1e-12), u2 = rand();
        cache.push(sigma * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2));
      }
      return cache[k];
    };
  }

  // Fill the per-channel arrays of a result built by hand (custom simulate hooks
  // may return only the scalar fields).
  function normalize(res) {
    if (!res) return res;
    const pairs = [['y', 'yAll'], ['yMeas', 'yMeasAll'], ['r', 'rAll'], ['uDemand', 'uDemandAll'], ['u', 'uAll'], ['uApplied', 'uAppliedAll']];
    for (const [one, all] of pairs) {
      if (!res[all] || !res[all].length) res[all] = res[one] ? [res[one]] : [];
      if (!res[one] && res[all].length) res[one] = res[all][0];
    }
    if (!res.yMeas) { res.yMeas = res.y; res.yMeasAll = res.yAll; }
    res.extras = res.extras || {};
    return res;
  }

  // Time of the n-th switch of the primary square-wave reference (half a period
  // per switch), or t_end for other references; and the result index 0.05 s
  // before time t. Problems read tracking errors there.
  const switchTime = (S, n = 1) => (S.sim.type === 'square' ? S.sim.tStep + n * 0.5 / S.sim.frequency : S.sim.tEnd);
  const indexBefore = (S, res, t) => Math.max(0, Math.min(res.t.length - 1, Math.round((t - 0.05) / S.sim.Ts)));

  return { simulate, makeReference, makeNoise, sat, normalize, switchTime, indexBefore };
})();
