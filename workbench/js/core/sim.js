// Generic closed-loop simulation mirroring the repo's hwNN_*Sim.py loop:
//   r = reference(t); u = controller.update(r, y_m); x = plant.update(u + d)
// with the controller's output saturated (as the ctrl*.py classes do) and the
// plant input saturated again after the disturbance is added (as
// <sys>Dynamics.update does). y_m = h(x) + measurement noise.
window.WB = window.WB || {};

WB.sim = (function () {
  const { rk4Step, saturate } = WB.math;

  // plant: { f(x, u) -> xdot, h(x) -> y (scalar), uLimit }
  // controller: { update(r, x, yMeas, t) -> u  |  { u, ...extras } }
  //   x is the true state; only full-state-feedback chapters may use it.
  //   Any extra numeric fields returned (e.g. xhat0, dhat, integrator) are recorded.
  function simulate({ plant, controller, reference, disturbance, noise, x0, Ts, tEnd }) {
    const N = Math.round(tEnd / Ts) + 1;
    const out = {
      t: new Float64Array(N), r: new Float64Array(N), y: new Float64Array(N), yMeas: new Float64Array(N),
      uDemand: new Float64Array(N), u: new Float64Array(N), uApplied: new Float64Array(N),
      x: [], extras: {},
    };
    let x = x0.slice();
    for (let k = 0; k < N; k++) {
      const t = k * Ts;
      const y = plant.h(x);
      const yMeas = y + (noise ? noise(k) : 0);
      const r = reference(t);
      let ret = controller.update(r, x, yMeas, t);
      let uDemand = ret;
      if (typeof ret === 'object') {
        uDemand = ret.u;
        for (const [key, v] of Object.entries(ret)) {
          if (key === 'u') continue;
          (out.extras[key] = out.extras[key] || new Float64Array(N))[k] = v;
        }
      }
      const u = saturate(uDemand, plant.uLimit);
      const uApplied = saturate(u + (disturbance ? disturbance(t) : 0), plant.uLimit);
      out.t[k] = t; out.r[k] = r; out.y[k] = y; out.yMeas[k] = yMeas;
      out.uDemand[k] = uDemand; out.u[k] = u; out.uApplied[k] = uApplied;
      out.x.push(x);
      if (k < N - 1) x = rk4Step((xx, uu) => plant.f(xx, uu), x, uApplied, Ts);
    }
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

  return { simulate, makeReference, makeNoise };
})();
