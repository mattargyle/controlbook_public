#!/usr/bin/env python3
"""Numerical regression: workbench JS vs the repo's _A_arm/python controllers.

Run with the repo venv:  .venv/bin/python workbench/tools/regress_A.py
Compares closed-loop samples (y, u, estimates) for PD, digital PID, observer and
disturbance observer. Prints the max |py - js| per case; exits 1 above 1e-9.
A template for checking other studies against their own ctrl*.py files.
"""
import contextlib
import io
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from js_eval import js_eval  # noqa: E402

REPO = HERE.parent.parent
ARM = REPO / "_A_arm" / "python"
KS = [50, 200, 999, 1500, 2999]

JS = r"""
const sys = WB.systems.A, p = {m:.5, ell:.3, b:.01, tau_max:1, g:9.8};
const ref = WB.sim.makeReference({type:'square', amplitude:30*Math.PI/180, frequency:0.05, tStep:0});
const KS = %s, out = {};
const base = {sys, pModel:p, ss:sys.stateSpace(p), model:sys.secondOrderModel(p), S:{sim:{Ts:0.01}}};
function run(name, controller, dist, ptrue) {
  const res = WB.sim.simulate({plant:{f:(x,u)=>sys.f(x,u,ptrue||p), h:sys.h, uLimit:1}, controller, reference:ref,
    disturbance:()=>dist, x0:[0,0], Ts:0.01, tEnd:30});
  out[name] = KS.map(k => [res.y[k], res.u[k], (res.extras.xhat1||[])[k]||0, (res.extras.dhat||[])[k]||0]);
}
// PD (Ch 8, tr = 0.8)
{ const c = {...base, st:{arch:'output', comp:'fl'}}; c.gains = WB.pd.gainsFromPoles(c.model, WB.pd.polesFromWnZeta(2.2/0.8, 0.707)); run('pd', WB.pd.makeController(c), 0); }
// digital PID (Ch 10), mismatched plant
{ const c = {...base, st:{arch:'output', comp:'fl', deriv:'dirty', antiwindup:'gate', vbar:0.08, sigma:0.05}};
  const wn = WB.pd.wnFromTr(.6, .9, 'tp'); c.gains = {...WB.pd.gainsFromPoles(c.model, WB.pd.polesFromWnZeta(wn, .9)), kI: 0.2};
  run('pid', WB.pid.makePID(c), 0, {...p, m:.56, ell:.273, b:.0115}); }
// observer (Ch 13) and disturbance observer (Ch 14)
for (const [name, level, st, d] of [
  ['obs', 'obs', {tr:.4, zeta:.707, rule:'2.2', pI:-9, wnObs:55, zetaObs:.707, comp:'fl', antiwindup:'none', xhat0:0}, 0.01],
  ['dobs', 'dobs', {tr:.4, zeta:.95, rule:'tp', pI:-9, wnObs:10, zetaObs:.707, pD:-5.5, comp:'fl', antiwindup:'none', dobs:true, xhat0:0}, 0.5]]) {
  const c = {...base, st, level}; c.gains = WB.ss.design(c, st, level); run(name, WB.ss.makeSS(c), d);
}
return out;
""" % KS


def python_runs():
    os.chdir(ARM)
    sys.path.insert(0, str(ARM))
    import armParam as P
    from armDynamics import armDynamics
    from signalGenerator import signalGenerator
    ref = signalGenerator(amplitude=30 * np.pi / 180, frequency=0.05)
    runs = {}

    def loop(name, update, d, setup=None):
        arm = armDynamics(alpha=0.0)
        if setup:
            setup(arm)
        y = arm.h()
        rec = {}
        for k in range(3000):
            out = update(ref.square(k * P.Ts), arm, y)
            u, xh, dh = out
            rec[k] = (y[0, 0], u, xh, dh)
            y = arm.update(u + d)
        runs[name] = [rec[k] for k in KS]

    # PD, tr = 0.8 (ctrlPD's part (a) tuning), derivative on the true state
    kp = (2.2 / 0.8) ** 2 * P.m * P.ell ** 2 / 3
    kd = P.m * P.ell ** 2 / 3 * (2 * 0.707 * 2.2 / 0.8 - 3 * P.b / (P.m * P.ell ** 2))

    def pd(r, arm, y):
        th, thd = arm.state[0, 0], arm.state[1, 0]
        u = np.clip(P.m * P.g * P.ell / 2 * np.cos(th) + kp * (r - th) - kd * thd, -1, 1)
        return u, 0.0, 0.0
    loop('pd', pd, 0)

    fixed = REPO / "_A_arm" / "python" / "ctrlPID.py"
    src = fixed.read_text().replace("abs(self.theta_dot < 0.08)", "abs(self.theta_dot) < 0.08")
    ns = {}
    exec(compile(src, "ctrlPID_fixed", "exec"), ns)
    with contextlib.redirect_stdout(io.StringIO()):
        cpid = ns["ctrlPID"]()

    def setup(arm):
        arm.m, arm.ell, arm.b = .56, .273, .0115
    loop('pid', lambda r, arm, y: (cpid.update(r, y), 0.0, 0.0), 0, setup)

    from ctrlObserver import ctrlObserver
    from ctrlDisturbanceObserver import ctrlDisturbanceObserver
    for name, Ctl, d in [('obs', ctrlObserver, 0.01), ('dobs', ctrlDisturbanceObserver, 0.5)]:
        with contextlib.redirect_stdout(io.StringIO()):
            c = Ctl()

        def upd(r, arm, y, c=c):
            out = c.update(r, y)
            return out[0], out[1][1, 0], (out[2] if len(out) > 2 else 0.0)
        loop(name, upd, d)
    return runs


def main():
    js = js_eval(JS)
    py = python_runs()
    worst = 0
    for name in ['pd', 'pid', 'obs', 'dobs']:
        err = max(abs(py[name][i][j] - js[name][i][j]) for i in range(len(KS)) for j in range(4))
        worst = max(worst, err)
        print(f"{name:5s} max |py - js| = {err:.3e}")
    return 0 if worst < 1e-9 else 1


if __name__ == "__main__":
    sys.exit(main())
