// Generic closed-loop simulation mirroring the repo's hwNN_*Sim.py loop:
//   r = reference(t); u = controller.update(r, x); x = plant.update(u + d)
// with the controller's output saturated (as ctrlPD does) and the plant input
// saturated again after the disturbance is added (as <sys>Dynamics.update does).
window.WB = window.WB || {};

WB.sim = (function () {
  const { rk4Step, saturate } = WB.math;

  // plant: { f(x, u) -> xdot, h(x) -> y (scalar), uLimit }
  // controller: { update(r, x, y) -> unsaturated control }
  function simulate({ plant, controller, reference, disturbance, x0, Ts, tEnd }) {
    const N = Math.round(tEnd / Ts) + 1;
    const out = {
      t: new Float64Array(N), r: new Float64Array(N), y: new Float64Array(N),
      uDemand: new Float64Array(N), u: new Float64Array(N), uApplied: new Float64Array(N),
      x: [],
    };
    let x = x0.slice();
    let y = plant.h(x);
    for (let k = 0; k < N; k++) {
      const t = k * Ts;
      const r = reference(t);
      const uDemand = controller.update(r, x, y);
      const u = saturate(uDemand, plant.uLimit);
      const uApplied = saturate(u + disturbance(t), plant.uLimit);
      out.t[k] = t; out.r[k] = r; out.y[k] = y;
      out.uDemand[k] = uDemand; out.u[k] = u; out.uApplied[k] = uApplied;
      out.x.push(x);
      if (k < N - 1) {
        x = rk4Step(plant.f, x, uApplied, Ts);
        y = plant.h(x);
      }
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
    return (t) => (t >= tStep ? amplitude + offset : offset);
  }

  return { simulate, makeReference };
})();
