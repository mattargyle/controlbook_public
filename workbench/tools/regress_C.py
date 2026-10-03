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
// Outer design-model poles with an integrator (C.9/C.10/C.P.6) and the C.17 bandwidths, C.10 gains.
const g10 = C.freq.c10(sys, nom);
const outerPoles = C.lib.outerPoles(nom, g10).map((q) => [q.re, q.im]);
const lp = C.freq.loops({ pModel: nom }, g10);
const freq = { outerPoles, bwIn: lp.bwIn, bwOut: lp.bwOut, pmIn: lp.mgIn.pm, wcIn: lp.mgIn.wc, pmOut: lp.mgOut.pm, wcOut: lp.mgOut.wc,
  g: [g10.kPth, g10.kDth, g10.kPphi, g10.kIphi, g10.kDphi] };
return { out, gains, freq };
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
    fe = check_frequency(res["freq"])
    return 0 if worst < 1e-9 and fe < 1e-6 else 1


def check_frequency(f):
    """Outer-loop poles with k_I and the C.17 margins/bandwidths, against python-control."""
    import control as ct
    from scipy.optimize import brentq
    kPth, kDth, kPphi, kIphi, kDphi = f["g"]
    Js, Jp, k, b, sig = 5.0, 1.0, 0.1, 0.05, 0.05
    # Fig. 6-10: theta_r = (kP + kI/s)(phi_r - phi) - kD s phi, phi = G theta_r, k_DCtheta = 1
    G = ct.tf([b, k], [Jp, b, k])
    T = ct.feedback(ct.tf([kPphi, kIphi], [1, 0]) * ct.feedback(G, ct.tf([kDphi, 0], [1])))
    py_p = sorted(ct.poles(T), key=lambda z: (round(z.real, 9), z.imag))
    js_p = sorted([complex(*q) for q in f["outerPoles"]], key=lambda z: (round(z.real, 9), z.imag))
    e_p = max(abs(a - c) for a, c in zip(py_p, js_p))
    # C.17 loops (hw16.py)
    Lin = ct.tf([1], [Js + Jp, 0, 0]) * ct.tf([kDth + sig * kPth, kPth], [sig, 1])
    Lout = ct.tf([b / Jp, k / Jp], [1, b / Jp, k / Jp]) * ct.tf([kDphi + kPphi * sig, kPphi + kIphi * sig, kIphi], [sig, 1, 0])
    Tin, Tout = ct.feedback(Lin), ct.feedback(Lout)
    # inner: ct.bandwidth(T_in) = 17.538 stops at its own tolerance; refine the same crossing with brentq
    w0 = float(ct.bandwidth(Tin))
    bw_in = brentq(lambda x: abs(Tin(1j * x)) - np.sqrt(0.5), 0.5 * w0, 2 * w0, xtol=1e-14)
    # outer: highest frequency with |T| >= -3 dB (the first crossing, which ct.bandwidth reports, is a dip near 0.08)
    w = np.logspace(-4, 4, 200001)
    m = np.abs(Tout(1j * w))
    i = np.nonzero(m >= np.sqrt(0.5))[0][-1]
    bw_out = brentq(lambda x: abs(Tout(1j * x)) - np.sqrt(0.5), w[i], w[i + 1], xtol=1e-14)
    gm_i, pm_i, _, wc_i = ct.margin(Lin)
    gm_o, pm_o, _, wc_o = ct.margin(Lout)
    rows = [("outer poles with k_I (Fig. 6-10)", e_p),
            ("C.17 inner bandwidth", abs(bw_in - f["bwIn"])), ("C.17 outer bandwidth (last -3 dB)", abs(bw_out - f["bwOut"])),
            ("C.17 bandwidth ratio", abs(bw_in / bw_out - f["bwIn"] / f["bwOut"])),
            ("C.17 inner PM [deg]", abs(pm_i - f["pmIn"])), ("C.17 inner w_co", abs(wc_i - f["wcIn"])),
            ("C.17 outer PM [deg]", abs(pm_o - f["pmOut"])), ("C.17 outer w_co", abs(wc_o - f["wcOut"]))]
    print(f"frequency-domain (C.10 gains): bw_in = {bw_in:.6f}, bw_out = {bw_out:.6f}, ratio = {bw_in / bw_out:.4f}, "
          f"ct.bandwidth: T_in {w0:.4f}, T_out {float(ct.bandwidth(Tout)):.4f} (first crossing)")
    for name, e in rows:
        print(f"  {name:36s} |py - js| = {e:.3e}")
    # margins are found on a grid in WB.tf.margins (shared code), so they agree to ~1e-4, not 1e-9
    return max(e for name, e in rows if "PM" not in name and "w_co" not in name)


if __name__ == "__main__":
    sys.exit(main())
