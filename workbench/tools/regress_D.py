#!/usr/bin/env python3
"""Numerical regression for Design Study D (mass-spring-damper).

Run with the repo venv:  .venv/bin/python workbench/tools/regress_D.py

D has no book solutions, so this script checks three things:
  1. The JS f(x, u) reproduces _D_mass/python/testDynamics.py exactly (|err| < 1e-14,
     the tolerance testDynamics.py itself uses).
  2. Every derived answer in WB.systems.D.answers (closed forms) matches an
     independent computation with numpy / python-control (place, ss2tf, bode,
     margin, ...), and the explore-mode numerical pole placement in JS agrees.
  3. Closed-loop simulations of the D.10 PID, D.12 integral state feedback and
     D.14 disturbance observer match a Python port of the repo's ctrl*.py
     conventions (dirty derivative, trapezoid integrator, RK4 observer at Ts)
     to machine precision.
Exits 1 on any failure.
"""
import json
import pathlib
import re
import sys

import control as ct
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from js_eval import js_eval  # noqa: E402

REPO = HERE.parent.parent
TEST = REPO / "_D_mass" / "python" / "testDynamics.py"
m, k, b, Fmax = 5.0, 3.0, 0.5, 6.0
P = dict(m=m, k=k, b=b, Fmax=Fmax)
Ts = 0.01
fails = []


def check(name, got, want, tol=1e-6, rel=True):
    got, want = np.atleast_1d(np.asarray(got, float)), np.atleast_1d(np.asarray(want, float))
    err = np.max(np.abs(got - want) / (np.maximum(1, np.abs(want)) if rel else 1))
    ok = err < tol
    print(f"{'ok  ' if ok else 'FAIL'} {name:46s} err {err:.2e}  js {np.round(got, 6).tolist()}")
    if not ok:
        fails.append(f"{name}: js {got} vs py {want}")


def test_vectors():
    src = TEST.read_text()
    grab = lambda name: np.array(eval(re.search(name + r" = np\.array\(\s*(\[.*?\])\s*(?:,\s*dtype=[^)]*)?\)", src, re.S).group(1)), float)  # noqa: E731
    return grab("x_tests").reshape(-1, 2), grab("u_tests"), grab("xdot_tests").reshape(-1, 2)


# --------------------------------------------------------------------- JS side --
REF = dict(tr=2.0, zeta=0.7, pI=-1.0, wnObs=11.0, zetaObs=0.707, pD=-5.0, kI=1.0, sigma=0.05)
JS = r"""
const sys = WB.systems.D, A = sys.answers, CH = WB.studies.D.chapters, D = WB.design;
const p = %(P)s, R = %(REF)s, X = %(X)s, U = %(U)s;
const out = {};
out.f = X.map((x, i) => sys.f(x, U[i], p));
const poles2 = D.polesFromWnZeta(2.2 / R.tr, R.zeta);
const obs2 = D.polesFromWnZeta(R.wnObs, R.zetaObs);
const c = (o) => JSON.parse(JSON.stringify(o));
out.tf = A.tf(p); out.ss = A.ss(p); out.ol = A.olPoles(p);
out.d7 = A.pdGains(p, [{re:-1, im:0}, {re:-1.5, im:0}]);
out.d8a = c(A.spec(p, R.tr, R.zeta));
out.d8b = A.satLimit(p, 1, R.zeta);
out.type0 = A.typeAnalysis(p, {kP: 3.05, kD: 7.2, kI: 0});
out.type1 = A.typeAnalysis(p, {kP: 3.05, kD: 7.2, kI: R.kI});
out.evans = A.evans(p, {kP: 3.05, kD: 7.2});
out.sf = A.stateFeedback(p, poles2);
out.sfi = A.integralFeedback(p, [...poles2, {re: R.pI, im: 0}]);
out.obs = A.observer(p, obs2);
out.dobs = A.disturbanceObserver(p, [...obs2, {re: R.pD, im: 0}]);
out.bode = A.bode(p);
// explore-mode numerical placement (Ackermann in WB.la) for the same poles
const ss = A.ss(p);
const {A1, B1} = D.augmentIntegrator(ss.A, ss.B, ss.C), {A2, C2} = D.augmentDisturbance(ss.A, ss.B, ss.C);
out.num = { K: D.place(ss.A, ss.B, poles2), kr: D.refGain(ss.A, ss.B, ss.C, D.place(ss.A, ss.B, poles2)),
  K1: D.place(A1, B1, [...poles2, {re: R.pI, im: 0}]), L: D.observerGain(ss.A, ss.C, obs2).map(r => r[0]),
  L2: D.observerGain(A2, C2, [...obs2, {re: R.pD, im: 0}]).map(r => r[0]) };
// frequency domain with the reference D.10 gains
const ctx0 = {sys, pModel: p, model: sys.secondOrderModel(p), ss, S: {mode: 'explore', sim: {Ts: 0.01}, ch: {}}, app: {isRevealed: () => true}};
const pid = {kP: out.d8a.kP, kD: out.d8a.kD, kI: R.kI, sigma: R.sigma, wdin: 0.1, wno: 100};
const sp = WB.studies.D.freq.specs({...ctx0, st: pid});
const lp = WB.studies.D.freq.loop({...ctx0, st: pid});
out.d15mag = [0.1, 1, 10].map(w => 20 * Math.log10(WB.la.C.abs(WB.tf.at(WB.studies.D.freq.plantTf(ctx0), w))));
out.d16 = {ramp: sp.ramp, gdin: sp.gdin, gdinExact: sp.gdinExact, gn: sp.gn, gnExact: sp.gnExact};
out.d17 = {pm: lp.mg.pm, wc: lp.mg.wc, bw: lp.bw, bwLast: lp.bwLast, ncross: lp.mg.crossings.length};
const lp2 = WB.studies.D.freq.loop({...ctx0, st: {kP: 2, kI: 0.5, kD: 4, sigma: 0.05}});  // placeholder gains: 3 gain crossovers
out.d17b = {pm: lp2.mg.pm, wc: lp2.mg.wc, n: lp2.mg.gcs.length};
const r18 = WB.studies.D.freq.presetRef(ctx0), st18 = {...CH.ch18.defaults(sys), ...r18};
const d18 = CH.ch18.design({...ctx0, st: st18}, st18);
out.d18 = {k: r18.k, pm: d18.mg.pm, wc: d18.mg.wc, low: d18.lowMin, high: d18.highMax, gm: d18.mg.crossings.map(q => [q.w, q.gm]), ok: d18.integOk && d18.lowOk && d18.highOk && d18.pmOk && d18.stable};
// closed-loop simulations through the chapter controllers
const KS = [50, 777, 1500, 2600, 3999];
const ptrue = {...p, m: p.m * 1.15, k: p.k * 0.82, b: p.b * 1.1};
function run(chId, st, dist, noise) {
  const ctx = {...ctx0, st};
  ctx.gains = CH[chId].gains(ctx);
  const res = WB.sim.simulate({plant: {f: (x, u) => sys.f(x, u, ptrue), h: sys.h, uLimit: p.Fmax}, controller: CH[chId].controller(ctx),
    reference: WB.sim.makeReference({type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0}),
    disturbance: () => dist, noise: noise ? WB.sim.makeNoise(noise, 7) : null, x0: [0, 0], Ts: 0.01, tEnd: 40});
  return {rows: KS.map(k => [res.y[k], res.uDemand[k], (res.extras.xhat0 || [])[k] || 0, (res.extras.dhat || [])[k] || 0]),
          noise: noise ? Array.from(res.yMeas, (v, i) => v - res.y[i]) : null};
}
out.sim = {
  pid: run('ch10', {...CH.ch10.defaults(sys), kIx: R.kI}, 0),
  sfi: run('ch12', {...CH.ch12.defaults(sys)}, 0.25),
  dobs: run('ch14', {...CH.ch14.defaults(sys)}, 0.25, 0.001),
};
return out;
"""


# ----------------------------------------------------------------- Python side --
def plant_f(x, u, p):
    return np.array([x[1], (u - p["b"] * x[1] - p["k"] * x[0]) / p["m"]])


def rk4(f, x, h):
    k1 = f(x); k2 = f(x + h / 2 * k1); k3 = f(x + h / 2 * k2); k4 = f(x + h * k3)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def square(t):
    return 0.5 if (t % 50.0) <= 25.0 else -0.5


def py_sim(controller, dist, noise=None, N=4001):
    pt = dict(m=m * 1.15, k=k * 0.82, b=b * 1.1)
    x = np.zeros(2)
    rows = {}
    for i in range(N):
        y = x[0]
        ym = y + (noise[i] if noise is not None else 0.0)
        u, xh0, dh = controller(square(i * Ts), ym)
        rows[i] = (y, u, xh0, dh)
        ua = np.clip(np.clip(u, -Fmax, Fmax) + dist, -Fmax, Fmax)
        x = rk4(lambda s: plant_f(s, ua, pt), x, Ts)
    return rows


class PID:  # D.10: ctrlPID conventions (Listing 10.2), F = k z_r + PID, gate anti-windup
    def __init__(s, kP, kI, kD, sigma=0.05, vbar=0.05):
        s.kP, s.kI, s.kD, s.vbar = kP, kI, kD, vbar
        s.beta, s.gamma = (2 * sigma - Ts) / (2 * sigma + Ts), 2 / (2 * sigma + Ts)
        s.I = s.e1 = s.zd = 0.0; s.z1 = None

    def __call__(s, r, y):
        e = r - y
        if s.z1 is None: s.z1 = y
        s.zd = s.beta * s.zd + s.gamma * (y - s.z1)
        if abs(s.zd) < s.vbar: s.I += Ts / 2 * (e + s.e1)
        u = k * r + s.kP * e + s.kI * s.I - s.kD * s.zd
        s.e1, s.z1 = e, y
        return u, 0.0, 0.0


class SFI:  # D.12: integral state feedback, dirty derivative for z', hold integrator while saturated
    def __init__(s, K, ki, sigma=0.05):
        s.K, s.ki = K, ki
        s.beta, s.gamma = (2 * sigma - Ts) / (2 * sigma + Ts), 2 / (2 * sigma + Ts)
        s.I = s.e1 = s.zd = 0.0; s.z1 = None

    def __call__(s, r, y):
        if s.z1 is None: s.z1 = y
        s.zd = s.beta * s.zd + s.gamma * (y - s.z1); s.z1 = y
        e = r - y
        It = s.I + Ts / 2 * (e + s.e1)
        if abs(-s.K[0] * y - s.K[1] * s.zd - s.ki * It) <= Fmax: s.I = It
        s.e1 = e
        return -s.K[0] * y - s.K[1] * s.zd - s.ki * s.I, y, 0.0


class DOBS:  # D.14: observer + disturbance observer, RK4 over Ts with the previous saturated input
    def __init__(s, K, ki, L, Ld, Am, Bm):
        s.K, s.ki, s.L, s.Ld, s.A, s.B = K, ki, L, Ld, Am, Bm
        s.z = np.zeros(3); s.I = s.e1 = s.u1 = 0.0

    def __call__(s, r, y):
        def f(z):
            inn = y - z[0]
            xd = s.A @ z[:2] + s.B[:, 0] * (s.u1 + z[2]) + s.L * inn
            return np.array([xd[0], xd[1], s.Ld * inn])
        s.z = rk4(f, s.z, Ts)
        xh, dh = s.z[:2], s.z[2]
        e = r - xh[0]
        It = s.I + Ts / 2 * (e + s.e1)
        if abs(-s.K @ xh - s.ki * It - dh) <= Fmax: s.I = It
        s.e1 = e
        u = -s.K @ xh - s.ki * s.I - dh
        s.u1 = np.clip(u, -Fmax, Fmax)
        return u, xh[0], dh


def main():
    X, U, XD = test_vectors()
    js = js_eval(JS % dict(P=json.dumps(P), REF=json.dumps(REF), X=X.tolist(), U=U.tolist()))

    print("-- 1. testDynamics.py vectors")
    err = np.abs(np.array(js["f"]) - XD)
    ok = bool((err < 1e-14).all())
    print(f"{'ok  ' if ok else 'FAIL'} {len(X)} vectors, max |f_js - expected| = {err.max():.1e} (tolerance 1e-14)")
    if not ok:
        fails.append("testDynamics vectors")

    print("-- 2. derived answers vs numpy / python-control")
    Am = np.array([[0, 1], [-k / m, -b / m]]); Bm = np.array([[0], [1 / m]]); Cm = np.array([[1, 0]])
    num, den = ct.ss2tf(Am, Bm, Cm, 0 * Cm[:, :1]).num[0][0], ct.ss2tf(Am, Bm, Cm, 0 * Cm[:, :1]).den[0][0]
    num = np.trim_zeros(np.round(num, 14), "f")
    check("D.5 P(s) vs ss2tf of D.6", [js["tf"]["b0"], js["tf"]["a1"], js["tf"]["a0"]], [num[-1] / den[0], den[1] / den[0], den[2] / den[0]])
    check("D.6 A, B", np.array(js["ss"]["A"]).ravel().tolist() + np.array(js["ss"]["B"]).ravel().tolist(), Am.ravel().tolist() + Bm.ravel().tolist())
    olp = np.roots([1, b / m, k / m])
    check("D.7(a) open-loop poles vs np.roots", sorted([q["im"] for q in js["ol"]]) + [js["ol"][0]["re"]], sorted(olp.imag.tolist()) + [olp.real[0]])
    K7 = ct.place(Am, Bm, [-1, -1.5])
    check("D.7(c) kP, kD vs control.place", [js["d7"]["kP"], js["d7"]["kD"]], K7.ravel())
    check("D.7(c) eig(A - B[kP kD])", sorted(np.linalg.eigvals(Am - Bm @ np.array([[js["d7"]["kP"], js["d7"]["kD"]]])).real), [-1.5, -1])
    wn = 2.2 / REF["tr"]; p8 = np.roots([1, 2 * REF["zeta"] * wn, wn ** 2])
    K8 = ct.place(Am, Bm, p8)
    check("D.8(a) kP, kD vs control.place", [js["d8a"]["kP"], js["d8a"]["kD"]], K8.ravel())
    check("D.8(a) alpha1, alpha0, wn", [js["d8a"]["alpha1"], js["d8a"]["alpha0"], js["d8a"]["wn"]], [2 * 0.7 * wn, wn ** 2, wn])
    # D.8(b): simulate the 1 m step with the saturation-limited gains; the peak demand must be Fmax (at t = 0+)
    g = js["d8b"]; x = np.zeros(2); peak = 0
    for i in range(1500):
        u = g["kP"] * (1 - x[0]) - g["kD"] * x[1]; peak = max(peak, abs(u))
        x = rk4(lambda s: plant_f(s, np.clip(u, -Fmax, Fmax), P), x, Ts)
    check("D.8(b) peak |F| (sim) = Fmax, F_e = 0", peak, Fmax)
    check("D.8(b) t_r = 2.2/sqrt((k+kP)/m)", g["tr"], 2.2 / np.sqrt((k + Fmax) / m))
    # D.9: limits evaluated numerically from the loop transfer functions
    s = ct.tf("s"); Pt = ct.tf([1 / m], [1, b / m, k / m])
    L0 = Pt * (7.2 * s + 3.05); L1 = Pt * (7.2 * s ** 2 + 3.05 * s + REF["kI"]) / s
    eps = 1e-7
    check("D.9 PD step error 1/(1+Mp)", js["type0"]["step"], abs((1 / (1 + L0))(eps)))
    check("D.9 PD input-disturbance gain P/(1+PC)", js["type0"]["dist"], abs((Pt / (1 + L0))(eps)))
    check("D.9 PID ramp error lim 1/(s PC)", js["type1"]["ramp"], abs(1 / (eps * L1(eps))), tol=1e-5)
    kc = js["evans"]["kCrit"]; rts = np.roots(np.polyadd(js["evans"]["den"], np.array(js["evans"]["num"]) * kc))
    check("D.P.6 kI,crit: roots on the jw axis", np.max(rts.real), 0.0, rel=False, tol=1e-6)
    Ksf = ct.place(Am, Bm, p8); kr = -1 / (Cm @ np.linalg.inv(Am - Bm @ Ksf) @ Bm)[0, 0]
    check("D.11 K vs control.place", js["sf"]["K"], Ksf.ravel())
    check("D.11 kr = -1/(C(A-BK)^-1 B)", js["sf"]["kr"], kr)
    check("D.11 numerical place in JS (explore)", js["num"]["K"] + [js["num"]["kr"]], Ksf.ravel().tolist() + [kr])
    A1 = np.block([[Am, np.zeros((2, 1))], [-Cm, np.zeros((1, 1))]]); B1 = np.vstack([Bm, [[0]]])
    K1 = ct.place(A1, B1, np.r_[p8, REF["pI"]])
    check("D.12 [K, kI] vs control.place(A1, B1)", js["sfi"]["K"] + [js["sfi"]["ki"]], K1.ravel())
    check("D.12 numerical place in JS (explore)", js["num"]["K1"], K1.ravel())
    q2 = np.roots([1, 2 * REF["zetaObs"] * REF["wnObs"], REF["wnObs"] ** 2])
    Lo = ct.place(Am.T, Cm.T, q2).T.ravel()
    check("D.13 L vs control.place(A^T, C^T)^T", js["obs"]["L"], Lo)
    check("D.13 numerical observerGain in JS", js["num"]["L"], Lo)
    check("D.13 rank O(A,C) = 2", np.linalg.matrix_rank(ct.obsv(Am, Cm)), 2)
    A2 = np.block([[Am, Bm], [np.zeros((1, 3))]]); C2 = np.hstack([Cm, [[0]]])
    L2 = ct.place(A2.T, C2.T, np.r_[q2, REF["pD"]]).T.ravel()
    check("D.14 [L, Ld] vs control.place(A2^T, C2^T)^T", js["dobs"]["L"] + [js["dobs"]["Ld"]], L2)
    check("D.14 numerical observerGain in JS", js["num"]["L2"], L2)
    bd = js["bode"]
    check("D.15 |P(j wn)| vs python-control", bd["peak"], float(np.abs(Pt(1j * np.sqrt(k / m)))))
    check("D.15 DC gain, wn, zeta", [bd["dc"], bd["wn"], bd["zeta"]], [Pt.dcgain(), ct.damp(Pt, doprint=False)[0][0], ct.damp(Pt, doprint=False)[1][0]])
    check("D.15 |P| at 0.1, 1, 10 rad/s [dB]", js["d15mag"], [20 * np.log10(np.abs(Pt(1j * ww))) for ww in (0.1, 1, 10)])
    kP, kD, kI, sg = js["d8a"]["kP"], js["d8a"]["kD"], REF["kI"], REF["sigma"]
    C = ct.tf([kD + sg * kP, kP + sg * kI, kI], [sg, 1, 0]); Lg = Pt * C
    check("D.16(a) ramp error = k/kI", js["d16"]["ramp"], abs(1 / (eps * Lg(eps))), tol=1e-5)
    check("D.16(b) 1/|C(j0.1)|, |P/(1+PC)(j0.1)|", [js["d16"]["gdin"], js["d16"]["gdinExact"]], [1 / abs(C(0.1j)), abs((Pt / (1 + Lg))(0.1j))])
    check("D.16(c) |PC(j100)|, |T(j100)|", [js["d16"]["gn"], js["d16"]["gnExact"]], [abs(Lg(100j)), abs((Lg / (1 + Lg))(100j))])
    gm, pm, wg, wc = ct.margin(Lg)
    check("D.17 PM, w_co vs control.margin", [js["d17"]["pm"], js["d17"]["wc"]], [pm, wc], tol=2e-3)
    gm2, pm2, wg2, wc2 = ct.margin(Pt * ct.tf([4 + 0.05 * 2, 2 + 0.05 * 0.5, 0.5], [0.05, 1, 0]))
    check("D.17 multi-crossover loop: min PM vs control.margin", [js["d17b"]["pm"], js["d17b"]["wc"], js["d17b"]["n"]], [pm2, wc2, 3], tol=2e-3)
    check("D.17 GM infinite (no phase crossover)", js["d17"]["ncross"], 0 if not np.isfinite(gm) else 1)
    Tc = ct.feedback(Lg, 1)
    check("D.17 first -3 dB crossing vs control.bandwidth", js["d17"]["bw"], ct.bandwidth(Tc), tol=2e-3)
    wf = np.logspace(-3, 4, 200001); below = np.abs(Tc(1j * wf)) < np.sqrt(0.5)
    down = np.where(~below[:-1] & below[1:])[0]
    check("D.17 final -3 dB roll-off (fine grid)", js["d17"]["bwLast"], wf[down[-1] + 1], tol=2e-3)
    r = js["d18"]
    C18 = r["k"] * (s + 0.5) / s * 30 * (s + 5 / np.sqrt(30)) / (s + 5 * np.sqrt(30)) * 50 / (s + 50)
    gm18, pm18, wg18, wc18 = ct.margin(Pt * C18)
    wl = np.logspace(-3, -1, 400); wh = np.logspace(np.log10(500), 4, 400)
    check("D.18 reference design PM, w_co", [r["pm"], r["wc"]], [pm18, wc18], tol=2e-3)
    check("D.18 GM (dB) and its frequency", [20 * np.log10(r["gm"][0][1]), r["gm"][0][0]], [20 * np.log10(gm18), wg18], tol=5e-3)
    check("D.18 min |PC| (w<=0.1), max |PC| (w>=500)", [r["low"], r["high"]], [np.abs((Pt * C18)(1j * wl)).min(), np.abs((Pt * C18)(1j * wh)).max()], tol=1e-3)
    check("D.18 all specs met (1 = yes)", float(r["ok"]), 1.0)
    check("D.18 closed loop stable", float(np.all(ct.poles(ct.feedback(Pt * C18, 1)).real < 0)), 1.0)

    print("-- 3. closed-loop simulations vs a Python port of the repo conventions")
    KS = [50, 777, 1500, 2600, 3999]
    runs = {
        "pid": py_sim(PID(kP, REF["kI"], kD), 0.0),
        "sfi": py_sim(SFI(np.array(js["sfi"]["K"]), js["sfi"]["ki"]), 0.25),
        "dobs": py_sim(DOBS(np.array(js["sfi"]["K"]), js["sfi"]["ki"], np.array(js["dobs"]["L"]), js["dobs"]["Ld"], Am, Bm), 0.25,
                       noise=np.array(js["sim"]["dobs"]["noise"])),
    }
    for name, rows in runs.items():
        py = np.array([rows[i] for i in KS]); jsr = np.array(js["sim"][name]["rows"])
        e = np.abs(py - jsr).max()
        ok = e < 1e-9
        print(f"{'ok  ' if ok else 'FAIL'} {name:5s} (y, F, ẑ, d̂ at {len(KS)} samples) max |py - js| = {e:.2e}")
        if not ok:
            fails.append(f"sim {name}")

    print(f"\n{'ALL PASS' if not fails else 'FAILURES: ' + '; '.join(fails)}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
