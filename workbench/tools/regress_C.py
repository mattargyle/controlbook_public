#!/usr/bin/env python3
"""Numerical regression: workbench JS vs the repo's _C_satellite/python controllers.

Run with the repo venv:  .venv/bin/python workbench/tools/regress_C.py
Simulates the same closed loop in Python (satelliteDynamics + ctrl*.py) and in
the workbench JS (WB.systems.C + the study C controllers), then compares samples
of θ, φ, τ and the observer estimates. Prints the max |py - js| per case and the
observer gains; exits 1 above 1e-9.

Repo bugs patched in memory only (see workbench/js/studies/C/ISSUES.md):
  * ctrlStateFeedbackIntegrator.update integrates θ (x[0]) instead of φ (x[1]).
  * ctrlLoopshape.transferFunction indexes num past its end for strictly proper
    numerators (crashes with IndexError for C_out), so method="state_space" never ran.
"""
import contextlib
import importlib
import io
import os
import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from js_eval import js_eval  # noqa: E402

REPO = HERE.parent.parent
SAT = REPO / "_C_satellite" / "python"
N = 8000                                   # 80 s at Ts = 0.01
KS = [50, 500, 2000, 4100, 6000, 7999]
MIS = {"Js": 5.0 * 0.88, "Jp": 1.0 * 1.09, "k": 0.1 * 1.15, "b": 0.05 * 0.93}   # the fixed alpha = 0.2 draw the pages use
DEG = np.pi / 180

CASES = {
    # name: (reference amplitude [deg], frequency [Hz], disturbance, mismatched plant)
    "pd":   (30, 0.015, 0.0, False),   # hw08: ctrlPD (t_rθ = 1.75), true state
    "pid":  (15, 0.02, 0.0, True),     # hw10: ctrlPID, measured outputs, alpha = 0.2
    "sf":   (15, 0.04, 0.0, False),    # hw11: ctrlStateFeedback
    "sfi":  (15, 0.01, 1.0, True),     # hw12: ctrlStateFeedbackIntegrator, d = 1, alpha = 0.2
    "obs":  (15, 0.03, 1.0, False),    # hw13 (part e): ctrlObserver, d = 1
    "dobs": (15, 0.03, 1.0, True),     # hw14 without noise: ctrlDisturbanceObserver
    "ls":   (15, 0.02, 0.5, True),     # hw18: ctrlLoopshape(method="state_space")
}

JS = r"""
const sys = WB.systems.C, C = WB.studies.C, nom = {Js: 5, Jp: 1, k: 0.1, b: 0.05, tau_max: 5};
const mis = %(mis)s, KS = %(ks)s, cases = %(cases)s, N = %(n)d, out = {};
const ctxOf = (st, gains) => ({ sys, pModel: nom, S: { sim: { Ts: 0.01, init: {} } }, st, gains });
function run(name, controller) {
  const [amp, freq, d, mm] = cases[name];
  const p = mm ? { ...nom, ...mis } : nom;
  const res = WB.sim.simulate({
    plant: { f: (x, u) => sys.f(x, u, p), h: sys.h, uLimit: 5 }, controller,
    reference: WB.sim.makeReference({ type: 'square', amplitude: amp * Math.PI / 180, frequency: freq, tStep: 0 }),
    disturbance: () => [d], x0: [0, 0, 0, 0], Ts: 0.01, tEnd: (N - 1) * 0.01 });
  const e = ['obs', 'dobs'].includes(name) ? res.extras : {};
  out[name] = KS.map((k) => [res.yAll[0][k], res.yAll[1][k], res.u[k], (e.phHat || [])[k] || 0, (e.phdHat || [])[k] || 0, (e.dhat || [])[k] || 0]);
}
const slc = (o) => C.lib.slcDesign(nom, o);
// C.8 ctrlPD: t_rθ = 1.75, π/2 rule, M = 10, feedforward, θ_r limit 30°
{ const g = slc({ trTh: 1.75, zetaTh: 0.9, M: 10, zetaPhi: 0.9, rule: 'tp' });
  run('pd', C.cascade.makeCascade(ctxOf({ ff: true, deriv: 'state', antiwindup: 'none', thetaMaxDeg: 30 }, g))); }
// C.10 ctrlPID
{ const g = slc({ trTh: 0.4, zetaTh: 0.9, M: 15, zetaPhi: 0.9, rule: '2.2', ki: 0.15 });
  run('pid', C.cascade.makeCascade(ctxOf({ ff: false, deriv: 'dirty', antiwindup: 'repo', sigma: 0.05, thetaMaxDeg: 30 }, g))); }
// C.11-C.14
const base = { trTh: 2, zetaTh: 0.9, M: 3, zetaPhi: 0.9, rule: 'tp', pI: -2, antiwindup: 'none', xhat0: 0 };
const obs = C.ss.obsDefaults({ ...sys.problems.ch13 });
const gains = {};
for (const [name, level, extra] of [['sf', 'sf', {}], ['sfi', 'sfi', {}], ['obs', 'obs', { obs }], ['dobs', 'dobs', { obs, pD: -10, dobs: true }]]) {
  const st = { ...base, ...extra }, g = C.ss.design(nom, st, level);
  gains[name] = { K: g.K, kr: g.kr, ki: g.ki, L: g.L, L2: g.L2 };
  run(name, C.ss.makeSS(ctxOf(st, g), level));
}
// C.18 loopshaping, repo listings
{ const st = C.freq.presetRepo(nom, C.freq.c10(sys, nom));
  run('ls', C.freq.loopshapeController(ctxOf(st, {}))); }
return { out, gains };
""" % {"mis": str(MIS).replace("'", '"'), "ks": KS, "cases": str({k: [v[0], v[1], v[2], v[3]] for k, v in CASES.items()}).replace("'", '"').replace("True", "true").replace("False", "false"), "n": N}


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def load_patched(modname, old, new):
    """Import a repo module with one source substitution (in memory only)."""
    src = (SAT / f"{modname}.py").read_text()
    assert old in src, f"patch target not found in {modname}.py"
    mod = types.ModuleType(modname)
    mod.__file__ = str(SAT / f"{modname}.py")
    quiet(exec, compile(src.replace(old, new), f"{modname}_patched", "exec"), mod.__dict__)
    return mod


def python_runs():
    os.chdir(SAT)
    sys.path.insert(0, str(SAT))
    import satelliteParam as P
    from satelliteDynamics import satelliteDynamics
    from signalGenerator import signalGenerator
    runs = {}

    def loop(name, make_ctrl, uses_state):
        amp, freq, d, mm = CASES[name]
        ref = signalGenerator(amplitude=amp * DEG, frequency=freq)
        sat = satelliteDynamics(alpha=0.0)
        if mm:
            sat.Js, sat.Jp, sat.k, sat.b = MIS["Js"], MIS["Jp"], MIS["k"], MIS["b"]
        ctrl = quiet(make_ctrl)
        y = sat.h()
        rec = {}
        for k in range(N):
            r = ref.square(k * P.Ts)
            out = ctrl.update(r, sat.state if uses_state else y)
            if isinstance(out, tuple):
                u = out[0]
                xh = out[1]
                dh = out[2] if len(out) > 2 else 0.0
                extra = (xh[1, 0], xh[3, 0], dh)
            else:
                u, extra = out, (0.0, 0.0, 0.0)
            rec[k] = (y[0, 0], y[1, 0], u) + extra
            y = sat.update(u + d)
        runs[name] = [rec[k] for k in KS]
        return ctrl

    from ctrlPD import ctrlPD
    from ctrlPID import ctrlPID
    from ctrlStateFeedback import ctrlStateFeedback
    from ctrlObserver import ctrlObserver
    from ctrlDisturbanceObserver import ctrlDisturbanceObserver
    sfi = load_patched("ctrlStateFeedbackIntegrator", "phi = x[0, 0]", "phi = x[1, 0]")
    ls = load_patched("ctrlLoopshape", "self.C[0, i] = num.item(i)", "self.C[0, i] = num.item(i - (n - m - 1))")

    ctrls = {}
    ctrls["pd"] = loop("pd", ctrlPD, True)
    ctrls["pid"] = loop("pid", ctrlPID, False)
    ctrls["sf"] = loop("sf", ctrlStateFeedback, True)
    ctrls["sfi"] = loop("sfi", sfi.ctrlStateFeedbackIntegrator, True)
    ctrls["obs"] = loop("obs", ctrlObserver, False)
    ctrls["dobs"] = loop("dobs", ctrlDisturbanceObserver, False)
    ctrls["ls"] = loop("ls", lambda: ls.ctrlLoopshape(method="state_space"), False)
    return runs, ctrls


def main():
    res = js_eval(JS, budget=60000)
    js, jg = res["out"], res["gains"]
    py, ctrls = python_runs()
    worst = 0.0
    print("closed-loop samples (θ, φ, τ, φ̂, φ̂dot, d̂) at k =", KS)
    for name in CASES:
        err = max(abs(py[name][i][j] - js[name][i][j]) for i in range(len(KS)) for j in range(6))
        scale = max(abs(v) for row in py[name] for v in row)
        worst = max(worst, err)
        print(f"  {name:5s} max |py - js| = {err:.3e}   (largest |value| {scale:.3g})")
    # gains, for the record
    ge = []
    ge.append(abs(np.array(jg["sf"]["K"]) - ctrls["sf"].K[0]).max())
    ge.append(abs(jg["sf"]["kr"] - ctrls["sf"].kr[0, 0]))
    ge.append(abs(np.array(jg["sfi"]["K"]) - ctrls["sfi"].K[0]).max())
    ge.append(abs(jg["sfi"]["ki"] - ctrls["sfi"].ki))
    ge.append(abs(np.array(jg["obs"]["L"]) - ctrls["obs"].L).max())
    ge.append(abs(np.array(jg["dobs"]["L2"]) - ctrls["dobs"].L).max())
    print(f"gains: max |py - js| over K, k_r, k_I, L (4x2, YT) and L2 (5x2, YT) = {max(ge):.3e}")
    return 0 if worst < 1e-9 else 1


if __name__ == "__main__":
    sys.exit(main())
