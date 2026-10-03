#!/usr/bin/env python3
"""Numerical regression: workbench JS vs the repo's _B_pendulum/python controllers.

Run with the repo venv:  .venv/bin/python workbench/tools/regress_B.py

Runs the same closed-loop scenarios in Python (the repo's pendulumDynamics and
ctrl*.py, unmodified unless noted) and in the workbench JS (js/studies/B/lib.js),
then compares z, θ, F and the observer estimates at a handful of samples. Also
compares the dynamics f(x, u) and the designed gains (K, k_I, the two-output
observer gains L and L2, the loopshaping Tustin coefficients).

Prints the max |py - js| per case and exits 1 if any exceeds 1e-9.
"""
import contextlib
import io
import json
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from js_eval import js_eval  # noqa: E402

REPO = HERE.parent.parent
PEND = REPO / "_B_pendulum" / "python"
KS = [50, 200, 999, 1500, 2999]
MIS = {"m1": 14, "m2": -11, "ell": 7, "b": 16}      # the workbench's fixed "alpha = 0.2" draw
MIS18 = {"m1": 7, "m2": -6, "ell": 4, "b": 8}       # alpha = 0.1 (hw18)
TH0 = 5.0                                            # initial angle [deg] for the PD runs
XS = [[0.1, 0.2, -0.3, 0.4], [-0.5, 1.2, 0.7, -2.0], [2.0, -2.9, 0.0, 3.0], [0.0, 3.1, 1.5, 0.5]]

JS = r"""
const sys = WB.systems.B, lib = WB.studies.B.lib, P = WB.studies.B.place;
const p = {m1:.25, m2:1, ell:1, b:.05, F_max:5, g:9.8};
const mis = (m) => ({...p, m1: p.m1*(1+m.m1/100), m2: p.m2*(1+m.m2/100), ell: p.ell*(1+m.ell/100), b: p.b*(1+m.b/100)});
const ss = sys.stateSpace(p), Ts = 0.01;
const ref = WB.sim.makeReference({type:'square', amplitude:0.5, frequency:0.04, tStep:0});
const KS = %(KS)s, out = {runs: {}, gains: {}};
function run(name, controller, {dist = 0, ptrue = p, x0 = [0, 0, 0, 0]} = {}) {
  const res = WB.sim.simulate({plant: {f: (x, u) => sys.f(x, u, ptrue), h: sys.h, uLimit: 5}, controller, reference: ref,
    disturbance: () => dist, x0, Ts, tEnd: 29.99});
  out.runs[name] = KS.map((k) => [res.yAll[0][k], res.yAll[1][k], res.u[k], (res.extras.zhat || [])[k] || 0, (res.extras.thhat || [])[k] || 0, (res.extras.dhat || [])[k] || 0]);
}
out.f = %(XS)s.map((x) => sys.f(x, 1.7, p));
// Ch 8: ctrlPD as written (Listing 8.3), and with the book's outer gains (Eqs. 8.12-8.13)
const x0pd = [0, %(TH0)s * Math.PI / 180, 0, 0];
for (const formula of ['listing', 'book']) {
  const g = lib.slcGains(p, {trTh: 0.15, zetaTh: 0.707, M: 15, zetaZ: 0.707, formula});
  out.gains['pd_' + formula] = [g.kPth, g.kDth, g.kDC, g.kPz, g.kDz];
  run('pd_' + formula, lib.slcPD({g, p, Ts, uLim: 5}), {x0: x0pd});
}
// Ch 10: ctrlPID, mismatched plant
{ const g = lib.slcGains(p, {trTh: 0.2, zetaTh: 0.707, M: 10, zetaZ: 0.707, formula: 'book'}); g.kIz = -0.05;
  out.gains.pid = [g.kPth, g.kDth, g.kDC, g.kPz, g.kDz];
  run('pid', lib.slcPID({g, p, Ts, uLim: 5, sigma: 0.05, vbar: 0.07, gate: true, thetaMax: 30 * Math.PI / 180}), {ptrue: mis(%(MIS)s)}); }
// Ch 11-14
const repo = (M) => ({trTh: 0.5, zetaTh: 0.9, M, zetaZ: 0.9, rule: 'tp', pI: -2, obsFactor: 10});
{ const d = lib.ssDesign(ss, 'sf', repo(5)); out.gains.sf = [...d.K, d.kr]; run('sf', lib.sfCtrl({K: d.K, kr: d.kr, uLim: 5})); }
{ const d = lib.ssDesign(ss, 'sfi', repo(3)); out.gains.sfi = [...d.K, d.ki];
  run('sfi', lib.sfiCtrl({K: d.K, ki: d.ki, uLim: 5, Ts}), {dist: 0.5, ptrue: mis(%(MIS)s)}); }
{ const d = lib.ssDesign(ss, 'obs', {...repo(3), obsRule: 'tp'}); out.gains.obs = [...d.K, d.ki]; out.gains.L = d.L;
  run('obs', lib.obsCtrl({Aobs: ss.A, Bobs: ss.B, Cobs: ss.C, L: d.L, K: d.K, ki: d.ki, Ts, uLim: 5}), {dist: 0.05}); }
{ const d = lib.ssDesign(ss, 'dobs', {...repo(3), obsRule: '2.2', pD: -1}); out.gains.dobs = [...d.K, d.ki]; out.gains.L2 = d.L2;
  const {A2, C2, B1} = lib.augD(ss);
  run('dobs', lib.obsCtrl({Aobs: A2, Bobs: B1, Cobs: C2, L: d.L2, K: d.K, ki: d.ki, Ts, uLim: 5, dist: true}), {dist: 0.5, ptrue: mis(%(MIS)s)}); }
// Ch 18: ctrlLoopshape with loopShapingInner/Outer.py, both implementations
{ const T = WB.tf, bl = lib.blocks;
  const Cin = T.mul(bl.prop(-1), bl.prop(800), bl.lead(40, 15));
  const Cout = T.mul(bl.prop(0.1), bl.lead(1, 20), bl.lag(0.04, 10), bl.lpf(50));
  const F = bl.lpf(2);
  out.gains.tustin = [lib.tustin(Cin.num, Cin.den, Ts), lib.tustin(Cout.num, Cout.den, Ts), lib.tustin(F.num, F.den, Ts)];
  for (const method of ['state_space', 'digital_filter']) run('ls_' + method, lib.loopshapeCtrl({Cin, Cout, F, Ts, uLim: 5, method}), {dist: 0.5, ptrue: mis(%(MIS18)s)});
}
return out;
""" % {"KS": KS, "XS": json.dumps(XS), "TH0": TH0, "MIS": json.dumps(MIS), "MIS18": json.dumps(MIS18)}


def python_runs():
    os.chdir(PEND)
    sys.path.insert(0, str(PEND))
    import pendulumParam as P
    from pendulumDynamics import pendulumDynamics
    from signalGenerator import signalGenerator
    quiet = contextlib.redirect_stdout(io.StringIO())
    ref = signalGenerator(amplitude=0.5, frequency=0.04)
    out = {"runs": {}, "gains": {}}

    def mismatch(pend, m):
        pend.m1 = P.m1 * (1 + m["m1"] / 100)
        pend.m2 = P.m2 * (1 + m["m2"] / 100)
        pend.ell = P.ell * (1 + m["ell"] / 100)
        pend.b = P.b * (1 + m["b"] / 100)

    def loop(name, update, d=0.0, mis=None, th0=0.0):
        pend = pendulumDynamics(alpha=0.0)
        if mis:
            mismatch(pend, mis)
        pend.state[1, 0] = th0 * np.pi / 180
        y = pend.h()
        rec = {}
        for k in range(3000):
            u, zh, thh, dh = update(ref.square(k * P.Ts), pend, y)
            rec[k] = (y[0, 0], y[1, 0], u, zh, thh, dh)
            y = pend.update(u + d)
        out["runs"][name] = [rec[k] for k in KS]

    pend = pendulumDynamics(alpha=0.0)
    out["f"] = [pend.f(np.array(x, dtype=float).reshape(4, 1), 1.7)[:, 0].tolist() for x in XS]

    # Ch 8: ctrlPD as written, and with the book's Eq. 8.12-8.13 outer gains patched in
    from ctrlPD import ctrlPD
    with quiet:
        c = ctrlPD()
    out["gains"]["pd_listing"] = [c.kp_th, c.kd_th, None, c.kp_z, c.kd_z]
    loop("pd_listing", lambda r, pd, y: (c.update(r, pd.state), 0, 0, 0), th0=TH0)
    with quiet:
        c2 = ctrlPD()
    wn_z = 2.2 / (15 * 0.15)
    a = -(wn_z ** 2) * np.sqrt(2.0 * P.ell / (3.0 * P.g))
    b = (a - 2.0 * 0.707 * wn_z) * np.sqrt(2.0 * P.ell / (3.0 * P.g))
    c2.kd_z = b / (1 - b)
    c2.kp_z = a * (1 + c2.kd_z)
    out["gains"]["pd_book"] = [c2.kp_th, c2.kd_th, None, c2.kp_z, c2.kd_z]
    loop("pd_book", lambda r, pd, y: (c2.update(r, pd.state), 0, 0, 0), th0=TH0)

    # Ch 10: ctrlPID on a mismatched plant
    from ctrlPID import ctrlPID
    with quiet:
        cp = ctrlPID()
    out["gains"]["pid"] = [cp.kp_th, cp.kd_th, None, cp.kp_z, cp.kd_z]
    loop("pid", lambda r, pd, y: (cp.update(r, y), 0, 0, 0), mis=MIS)

    # Ch 11-14
    from ctrlStateFeedback import ctrlStateFeedback
    from ctrlStateFeedbackIntegrator import ctrlStateFeedbackIntegrator
    from ctrlObserver import ctrlObserver
    from ctrlDisturbanceObserver import ctrlDisturbanceObserver
    with quiet:
        sf, sfi, ob, do = ctrlStateFeedback(), ctrlStateFeedbackIntegrator(), ctrlObserver(), ctrlDisturbanceObserver()
    out["gains"]["sf"] = [*sf.K[0], sf.kr.item()]
    out["gains"]["sfi"] = [*sfi.K[0], sfi.ki]
    out["gains"]["obs"] = [*np.asarray(ob.K)[0], ob.ki]
    out["gains"]["L"] = ob.L.tolist()
    out["gains"]["dobs"] = [*np.asarray(do.K)[0], do.ki]
    out["gains"]["L2"] = do.L.tolist()
    loop("sf", lambda r, pd, y: (sf.update(r, pd.state), 0, 0, 0))
    loop("sfi", lambda r, pd, y: (sfi.update(r, pd.state), 0, 0, 0), d=0.5, mis=MIS)

    def obs_upd(r, pd, y):
        u, xh = ob.update(r, y)
        return u, xh[0, 0], xh[1, 0], 0.0
    loop("obs", obs_upd, d=0.05)

    def dobs_upd(r, pd, y):
        u, xh, dh = do.update(r, y)
        return u, xh[0, 0], xh[1, 0], dh
    loop("dobs", dobs_upd, d=0.5, mis=MIS)

    # Ch 18: ctrlLoopshape, both implementations
    with quiet:
        from ctrlLoopshape import ctrlLoopshape
    for method in ["state_space", "digital_filter"]:
        with quiet:
            cl = ctrlLoopshape(method=method)
        if method == "digital_filter":
            out["gains"]["tustin"] = [{"num": list(f.num_d), "den": list(f.den_d)} for f in (cl.control_in, cl.control_out, cl.prefilter_out)]
        loop("ls_" + method, lambda r, pd, y, cl=cl: (cl.update(r, y), 0, 0, 0), d=0.5, mis=MIS18)
    return out


def maxdiff(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    return float(np.max(np.abs(a - b)))


def main():
    js = js_eval(JS, budget=60000)
    py = python_runs()
    worst = 0.0
    print("dynamics f(x, u)       max |py - js| = %.3e" % maxdiff(py["f"], js["f"]))
    worst = max(worst, maxdiff(py["f"], js["f"]))
    print("gains (relative error)")
    for k in ["pd_listing", "pd_book", "pid", "sf", "sfi", "obs", "dobs", "L", "L2"]:
        pv = np.asarray([v for v in np.ravel(py["gains"][k]) if v is not None], dtype=float)
        jv = np.asarray([v for i, v in enumerate(np.ravel(js["gains"][k])) if np.ravel(py["gains"][k])[i] is not None], dtype=float)
        rel = float(np.max(np.abs(pv - jv) / np.maximum(1.0, np.abs(pv))))
        worst = max(worst, rel)
        print("  %-11s %.3e" % (k, rel))
    tr = max(maxdiff(p_["num"], j_["num"]) + maxdiff(p_["den"], j_["den"]) for p_, j_ in zip(py["gains"]["tustin"], js["gains"]["tustin"]))
    print("  %-11s %.3e   (python-control c2d vs direct bilinear substitution)" % ("tustin", tr))
    worst = max(worst, tr)
    print("closed loop, 30 s at Ts = 0.01 (z, theta, F, z_hat, theta_hat, d_hat)")
    for name in ["pd_listing", "pd_book", "pid", "sf", "sfi", "obs", "dobs", "ls_state_space", "ls_digital_filter"]:
        err = maxdiff(py["runs"][name], js["runs"][name])
        worst = max(worst, err)
        print("  %-18s max |py - js| = %.3e" % (name, err))
    print("worst = %.3e" % worst)
    return 0 if worst < 1e-9 else 1


if __name__ == "__main__":
    sys.exit(main())
