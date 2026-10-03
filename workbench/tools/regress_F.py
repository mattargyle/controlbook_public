#!/usr/bin/env python3
"""Numerical checks for Design Study F (planar VTOL), which has no book solutions.

Run with the repo venv:  .venv/bin/python workbench/tools/regress_F.py

1. Dynamics: runs the expected vectors of _F_planar_vtol/python/testDynamics.py
   through the JS f(x, u) and requires |error| < 1e-14 (testDynamics' tolerance).
   It also checks f against an Euler-Lagrange derivation done here with sympy.
2. Answers: every answer the workbench derives (gains, poles, observer gains,
   margins, ...) is recomputed independently in Python (python-control /
   numpy) and compared with the JS value (relative error < 1e-6).
Exits 1 on any failure.
"""
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from js_eval import js_eval  # noqa: E402

REPO = HERE.parent.parent
TEST = REPO / "_F_planar_vtol" / "python" / "testDynamics.py"
FAIL = []


def check(name, py, js, rel=1e-6, abs_=1e-9):
    py = np.atleast_1d(np.asarray(py, dtype=float)).ravel()
    js = np.atleast_1d(np.asarray(js, dtype=float)).ravel()
    if py.shape != js.shape:
        FAIL.append(name)
        print(f"FAIL {name}: shape {py.shape} vs {js.shape}")
        return
    err = np.max(np.abs(py - js) / np.maximum(abs_, np.abs(py)))
    ok = err < rel
    if not ok:
        FAIL.append(name)
    print(f"{'ok  ' if ok else 'FAIL'} {name:46s} max rel err {err:.2e}")


def test_vectors():
    """Parse x_tests, u_tests, xdot_tests from testDynamics.py without importing it
    (it imports the student's VTOLDynamics, which the template does not have)."""
    src = TEST.read_text()

    def grab(name):
        m = re.search(name + r"\s*=\s*np\.array\((\[.*?\]),\s*dtype", src, re.S)
        return np.array(eval(m.group(1)), dtype=float)
    return grab("x_tests"), grab("u_tests"), grab("xdot_tests")


def sympy_f():
    import sympy as sp
    t = sp.symbols("t")
    z, h, th = [sp.Function(n)(t) for n in ("z", "h", "th")]
    mc, mr, Jc, d, mu, g, fr, fl = sp.symbols("m_c m_r J_c d mu g f_r f_l")
    pr = sp.Matrix([z + d * sp.cos(th), h + d * sp.sin(th)])
    pl = sp.Matrix([z - d * sp.cos(th), h - d * sp.sin(th)])
    K = (sp.Rational(1, 2) * mc * (z.diff(t) ** 2 + h.diff(t) ** 2) + sp.Rational(1, 2) * Jc * th.diff(t) ** 2
         + sp.Rational(1, 2) * mr * (pr.diff(t).dot(pr.diff(t)) + pl.diff(t).dot(pl.diff(t))))
    P = (mc + 2 * mr) * g * h
    Lg = K - P
    F, tau = fr + fl, d * (fr - fl)
    Q = [-F * sp.sin(th) - mu * z.diff(t), F * sp.cos(th), tau]
    q = [z, h, th]
    eqs = [sp.diff(Lg.diff(qi.diff(t)), t) - Lg.diff(qi) - Qi for qi, Qi in zip(q, Q)]
    acc = sp.solve(eqs, [qi.diff(t, 2) for qi in q])
    X = sp.symbols("x0:6")
    sub = {z.diff(t): X[3], h.diff(t): X[4], th.diff(t): X[5]}
    sub2 = {z: X[0], h: X[1], th: X[2]}
    exprs = [acc[qi.diff(t, 2)].subs(sub).subs(sub2) for qi in q]
    par = {mc: 1.0, mr: 0.25, Jc: 0.0042, d: 0.3, mu: 0.1, g: 9.81}
    fn = sp.lambdify([X, fr, fl], [e.subs(par) for e in exprs])
    return lambda x, u: [x[3], x[4], x[5]] + [float(v) for v in fn(x, u[0], u[1])]


def dynamics():
    xs, us, xds = test_vectors()
    js = js_eval("""
      const s = WB.systems.F, p = {...Object.fromEntries(s.params.map(q => [q.key, q.value])), ...s.constants};
      return %s.map((x, i) => s.f(x, %s[i], p));
    """ % (xs.tolist(), us.tolist()))
    js = np.array(js)
    err = np.max(np.abs(js - xds))
    ok = err < 1e-14
    print(f"{'ok  ' if ok else 'FAIL'} testDynamics.py vectors (10 cases)            max |err| {err:.2e} (tolerance 1e-14)")
    if not ok:
        FAIL.append("testDynamics")
    f = sympy_f()
    py = np.array([f(x, u) for x, u in zip(xs, us)])
    check("f vs. sympy Euler-Lagrange derivation", py, js, rel=1e-12, abs_=1e-6)


def main():
    dynamics()
    answers()
    print(f"\n{len(FAIL)} failure(s)" + (": " + ", ".join(FAIL) if FAIL else ""))
    return 1 if FAIL else 0


JS_ANSWERS = r"""
const sys = WB.systems.F, Fx = WB.F;
const p = {...Object.fromEntries(sys.params.map(q => [q.key, q.value])), ...sys.constants};
function mkctx(chId, mode, over = {}) {
  const ch = WB.studies.F.chapters[chId];
  const st = ch.defaults(sys); Object.assign(st, over.st || {});
  const sd = ch.simDefaults(sys);
  const sim = {type:'step', amplitude:2, frequency:0.03, tStep:0, y0:0, dist:0, tDist:0, tEnd:50, Ts:0.01, noise:0,
    refs:[{type:'square', amplitude:2.5, frequency:0.08, tStep:0}], init:{z0:0, theta0:0}, dists:[0,0,0,0], noises:[0,0], ...sd};
  if (sd.refs) sim.refs = [{type:'square', amplitude:2.5, frequency:0.08, tStep:0, ...sd.refs[0]}];
  if (sd.dists) sim.dists = [0,0,0,0].map((d,i) => sd.dists[i] ?? d);
  Object.assign(sim, over.sim || {});
  const mm = over.mismatch !== undefined ? over.mismatch : (sd.mismatch || {});
  const pTrue = {...p}; for (const k of sys.uncertain) pTrue[k] = p[k] * (1 + (mm[k] || 0) / 100);
  const S = {mode, sim, ch: {}};
  const ctx = {sys, pModel: p, pTrue, S, st, chan: {dists: sys.disturbances, refs: sys.refs, initial: sys.initial, outputs: sys.outputs}, chapter: ch, app: {result: () => null}};
  for (const id of Object.keys(WB.studies.F.chapters)) S.ch[id] = WB.studies.F.chapters[id].defaults(sys);
  S.ch[chId] = st;
  ctx.gains = ch.gains(ctx);
  return ctx;
}
function runCtx(ctx) {
  const S = ctx.S;
  const gens = ctx.chan.refs.map((rf, i) => { const c = i === 0 ? S.sim : S.sim.refs[0]; const g = WB.sim.makeReference({type: c.type, amplitude: c.amplitude, frequency: c.frequency, tStep: c.tStep}); return (t) => g(t); });
  const reference = Fx.reference(ctx, (t) => gens.map((g) => g(t)));
  const x0 = sys.x0({h0: S.sim.y0, z0: S.sim.init.z0 || 0, theta0: 0});
  return Fx.simulate(ctx, {reference, noise: null, x0, Ts: S.sim.Ts, tEnd: S.sim.tEnd});
}
const out = {};
const m = sys.models(p); out.models = {M: m.M, J: m.J, a: m.a, Fe: m.Fe, lonA: m.lonSS.A, lonB: m.lonSS.B, latA: m.latSS.A, latB: m.latSS.B};
out.f7 = Fx.pdFromPoles(m.lon, [{re: -0.2, im: 0}, {re: -0.3, im: 0}]);
out.f8 = Fx.refF8(p);
const g8 = out.f8, A9 = (mdl, kP, kI, kD, arch) => Fx.pid.loopAnalysis(mdl, kP, kI, kD, arch);
const outerM = {b0: m.outer.b0 * Fx.kDCof(m, g8), a1: m.a, a0: 0};
out.f9 = {
  lonPD: A9(m.lon, g8.kPh, 0, g8.kDh, 'error'), lonPID: A9(m.lon, g8.kPh, 0.01, g8.kDh, 'error'),
  lonPDy: A9(m.lon, g8.kPh, 0, g8.kDh, 'output'), inner: A9(m.inner, g8.kPth, 0, g8.kDth, 'error'),
  outerPD: A9(outerM, g8.kPz, 0, g8.kDz, 'error'), outerPID: A9(outerM, g8.kPz, -0.0005, g8.kDz, 'error'),
};
const c6 = mkctx('p6', 'explore');
out.p6 = {lon: Fx.pid.evans(c6, 'lon', c6.gains).kCrit, outer: -Fx.pid.evans(c6, 'outer', c6.gains).kCrit,
  lonRange: Fx.pid.kIRange(Fx.pid.evans(c6, 'lon', c6.gains)), outerRange: -Fx.pid.kIRange(Fx.pid.evans(c6, 'outer', c6.gains))};
const c8 = mkctx('ch8', 'explore');
out.f8f = Fx.pid.fastestNoSat(c8);
const k = Fx.ss.knobs(sys);
out.f11 = Fx.ss.design(p, k, 'sf'); out.f12 = Fx.ss.design(p, k, 'sfi'); out.f13 = Fx.ss.design(p, k, 'obs'); out.f14 = Fx.ss.design(p, k, 'dobs');
const Lm = Fx.ss.latL(out.f13.Lz, out.f13.Lt);
out.f13.eigLat = WB.la.eig(WB.la.sub(m.latSS.A, WB.la.mul(Lm, m.latSS.C))).map(q => [q.re, q.im]);
const P = Fx.freq.plants(p), db = (x) => 20 * Math.log10(x);
out.f15 = ['lon', 'inner', 'outer'].map(w => db(WB.la.C.abs(WB.tf.at(P[w], 1))));
const c16 = mkctx('ch16', 'explore'); const s16 = Fx.freq.specs16(c16);
out.f16 = {ePar: s16.ePar, eParImpl: s16.eParImpl, gn: s16.gn, gnExact: s16.gnExact, gdin: s16.gdin, gdinExact: s16.gdinExact, wSensor: s16.wSensor, gr: s16.gr, grExact: s16.grExact, gout: s16.gout};
const c17 = mkctx('ch17', 'explore'); const r17 = Fx.freq.margins17(c17);
out.f17 = Object.fromEntries(['lon', 'inner', 'outer'].map(w => [w, [r17[w].mg.pm, r17[w].mg.wc, r17[w].bw]])); out.f17.sep = r17.sep;
const c18 = mkctx('ch18', 'explore'); const ref = Fx.freq.refState(c18); const d18 = Fx.freq.design18(c18, {...c18.st, ...ref});
out.f18 = {k: [ref.lon.k, ref.inner.k, ref.outer.k], pm: [d18.mgl.pm, d18.mgi.pm, d18.mgo.pm], wc: [d18.mgl.wc, d18.mgi.wc, d18.mgo.wc],
  specs: Fx.freq.specs18(c18, d18, {...c18.st, ...ref})};
// closed-loop F.10 (nested digital PID, measured outputs, mismatch) for a Python re-implementation
const c10 = mkctx('ch10', 'explore'); const r10 = runCtx(c10);
const KS = [100, 500, 1500, 3000, 6000, 7999];
out.f10 = {gains: c10.gains, pTrue: c10.pTrue, samples: KS.map(i => [r10.yAll[0][i], r10.yAll[1][i], r10.yAll[2][i], r10.uDemandAll[0][i], r10.uDemandAll[1][i]]), KS};
return out;
"""


def answers():
    import control as ct
    import sympy as sp
    js = js_eval(JS_ANSWERS, budget=120000)
    mc, mr, Jc, d, mu, g = 1.0, 0.25, 0.0042, 0.3, 0.1, 9.81
    M, J, a, Fe = mc + 2 * mr, Jc + 2 * mr * d ** 2, mu / (mc + 2 * mr), (mc + 2 * mr) * g
    jm = js["models"]
    check("F.4/F.5 M, J, mu/M, F_e", [M, J, a, Fe], [jm["M"], jm["J"], jm["a"], jm["Fe"]])
    Alon, Blon = np.array([[0, 1], [0, 0]]), np.array([[0], [1 / M]])
    Alat = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, -g, -a, 0], [0, 0, 0, 0]])
    Blat = np.array([[0], [0], [0], [1 / J]])
    Clat = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
    Cz = np.array([[1, 0, 0, 0]])
    # F.6 two ways: hand-written matrices vs. numerical Jacobian of the sympy EOM
    f = sympy_f()
    xe, ue = np.array([0, 0, 0, 0, 0, 0.0]), np.array([Fe / 2, Fe / 2])
    Ajac = np.zeros((6, 6))
    for i in range(6):
        dx = np.zeros(6); dx[i] = 1e-6
        Ajac[:, i] = (np.array(f(xe + dx, ue)) - np.array(f(xe - dx, ue))) / 2e-6
    check("F.6 A_lon vs Jacobian of EOM", Ajac[np.ix_([1, 4], [1, 4])], jm["lonA"], abs_=1e-6)
    check("F.6 A_lat vs Jacobian of EOM", Ajac[np.ix_([0, 2, 3, 5], [0, 2, 3, 5])], jm["latA"], abs_=1e-6)
    check("F.6 B_lon, B_lat", [Blon[1, 0], Blat[3, 0]], [jm["lonB"][1][0], jm["latB"][3][0]])
    # F.7
    K7 = ct.place(Alon, Blon, [-0.2, -0.3])[0]
    check("F.7 kP, kD (control.place)", K7, [js["f7"]["kP"], js["f7"]["kD"]])
    # F.8: place on the 2-state models; outer: (z, zdot) with input theta, B = (0, -g)
    z8 = 0.707
    def pp(wn, zeta): return np.roots([1, 2 * zeta * wn, wn ** 2])
    f8 = js["f8"]
    Kh = ct.place(Alon, Blon, pp(2.2 / 8, z8))[0]
    Kt = ct.place(Alon, np.array([[0], [1 / J]]), pp(2.2 / 0.8, z8))[0]
    Kz = ct.place(np.array([[0, 1], [0, -a]]), np.array([[0], [-g]]), pp(2.2 / 8, z8))[0]
    check("F.8 kP,h kD,h", Kh, [f8["kPh"], f8["kDh"]])
    check("F.8 kP,th kD,th", Kt, [f8["kPth"], f8["kDth"]])
    check("F.8 kP,z kD,z (k_DC = 1)", Kz, [f8["kPz"], f8["kDz"]])
    # F.9: final value theorem with sympy, unity feedback (book) and derivative on y
    s = sp.symbols("s")
    def ess(P, C, k, arch="error", Ck=None):
        if arch == "error":
            E = 1 / (1 + P * C)
        else:
            E = 1 - P * Ck / (1 + P * C)
        return float(sp.limit(s * E / s ** k, s, 0))
    kP, kD = f8["kPh"], f8["kDh"]
    Pl = (1 / sp.Float(M)) / s ** 2
    check("F.9a lon PD e_parab (book)", ess(Pl, kP + kD * s, 3), js["f9"]["lonPD"]["parab"])
    check("F.9a lon PD e_ramp (D on y)", ess(Pl, kP + kD * s, 2, "output", kP), js["f9"]["lonPDy"]["ramp"])
    check("F.9a lon d_in step error (PD)", float(sp.limit(s * Pl / (1 + Pl * (kP + kD * s)) / s, s, 0)), js["f9"]["lonPD"]["dStep"])
    Pi = (1 / sp.Float(J)) / s ** 2
    check("F.9b inner e_parab", ess(Pi, f8["kPth"] + f8["kDth"] * s, 3), js["f9"]["inner"]["parab"])
    Po = -g / (s * (s + a))
    check("F.9c outer PD e_ramp", ess(Po, f8["kPz"] + f8["kDz"] * s, 2), js["f9"]["outerPD"]["ramp"])
    check("F.9c outer PID e_parab", ess(Po, f8["kPz"] - 0.0005 / s + f8["kDz"] * s, 3), js["f9"]["outerPID"]["parab"])
    types = [js["f9"][k]["type"] for k in ("lonPD", "lonPID", "inner", "outerPD", "outerPID")]
    check("F.9 system types (2, 3, 2, 1, 2)", [2, 3, 2, 1, 2], types)
    # P.6: critical gains, Routh (JS) vs. the gain where a root crosses the jw axis
    def crit(den, num):
        lo, hi = 0.0, 1.0
        while max(np.roots(np.polyadd(den, hi * np.array(num))).real) < 0:
            hi *= 2
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if max(np.roots(np.polyadd(den, mid * np.array(num))).real) < 0:
                lo = mid
            else:
                hi = mid
        return lo
    kc_h = crit([1, kD / M, kP / M, 0], [1 / M])
    kc_z = -crit([1, a - g * f8["kDz"], -g * f8["kPz"], 0], [g])
    check("P.6 kI,h crit (root crossing)", kc_h, js["p6"]["lon"], rel=1e-6)
    check("P.6 kI,z crit (root crossing)", kc_z, js["p6"]["outer"], rel=1e-6)
    # F.8(f): the JS bisection answer is a saturation boundary in an independent Python sim
    trh, trz, trza = js["f8f"]["trh"], js["f8f"]["trz"], js["f8f"]["trzAlone"]
    for name, base, cases in (
            ("altitude alone", (trh, 8.0), [((1.001, 1), False), ((0.97, 1), True)]),
            ("lateral alone", (8.0, trza), [((1, 1.001), False), ((1, 0.97), True)]),
            ("trade-off corner", (trh, trz), [((1.001, 1.001), False), ((0.97, 1), True), ((1, 0.97), True)])):
        ok = all(f8_sat(base[0] * a_, base[1] * b_) == want for (a_, b_), want in cases)
        print(f"{'ok  ' if ok else 'FAIL'} F.8(f) {name}: t_r,h = {base[0]:.3f}, t_r,z = {base[1]:.3f} is a saturation boundary (Python sim)")
        if not ok:
            FAIL.append("F.8f " + name)
    # F.11 / F.12
    lonP, latP = pp(2.2 / 8, z8), np.concatenate([pp(2.2 / 8, z8), pp(2.2 / 0.8, z8)])
    K11h, K11z = ct.place(Alon, Blon, lonP)[0], ct.place(Alat, Blat, latP)[0]
    kr = lambda A, B, C, K: -1 / (C @ np.linalg.inv(A - B @ K[None, :]) @ B)[0, 0]
    f11 = js["f11"]
    check("F.11 K_h, k_r,h", list(K11h) + [kr(Alon, Blon, np.array([[1, 0]]), K11h)], f11["Kh"] + [f11["krh"]])
    check("F.11 K_z, k_r,z", list(K11z) + [kr(Alat, Blat, Cz, K11z)], f11["Kz"] + [f11["krz"]])
    pr12 = -0.4
    A1h = np.block([[Alon, np.zeros((2, 1))], [-np.array([[1, 0]]), np.zeros((1, 1))]]); B1h = np.vstack([Blon, [[0]]])
    A1z = np.block([[Alat, np.zeros((4, 1))], [-Cz, np.zeros((1, 1))]]); B1z = np.vstack([Blat, [[0]]])
    K12h = ct.place(A1h, B1h, np.append(lonP, pr12))[0]; K12z = ct.place(A1z, B1z, np.append(latP, pr12))[0]
    f12 = js["f12"]
    check("F.12 [K_h, k_I,h]", K12h, f12["Kh"] + [f12["kIh"]])
    check("F.12 [K_z, k_I,z]", K12z, f12["Kz"] + [f12["kIz"]])
    # F.13 / F.14 observers: per block, place(A^T, C^T)^T
    obs = lambda A, C, poles: ct.place(np.array(A).T, np.array(C).T, poles).T[:, 0]
    fo = 10
    Lh = obs(Alon, [[1, 0]], pp(fo * 2.2 / 8, 0.707))
    Lz = obs([[0, 1], [0, -a]], [[1, 0]], pp(fo * 2.2 / 8, 0.707))
    Lt = obs([[0, 1], [0, 0]], [[1, 0]], pp(fo * 2.2 / 0.8, 0.707))
    f13 = js["f13"]
    check("F.13 L_h", Lh, f13["Lh"])
    check("F.13 L_z, L_theta (block design)", list(Lz) + list(Lt), f13["Lz"] + f13["Lt"])
    Lfull = np.array([[Lz[0], 0], [0, Lt[0]], [Lz[1], 0], [0, Lt[1]]])
    want = np.sort_complex(np.concatenate([pp(fo * 2.2 / 8, 0.707), pp(fo * 2.2 / 0.8, 0.707)]))
    got = np.sort_complex(np.linalg.eigvals(Alat - Lfull @ Clat))
    check("F.13 eig(A_lat - L C) = desired (numpy)", np.r_[got.real, got.imag], np.r_[want.real, want.imag], abs_=1e-6)
    jsg = [complex(*q) for q in f13["eigLat"]]
    jsm = np.array([min(jsg, key=lambda q: abs(q - w)) for w in got])   # nearest-pole matching
    check("F.13 eig(A_lat - L C), JS vs numpy", np.r_[got.real, got.imag], np.r_[jsm.real, jsm.imag], rel=1e-6, abs_=1e-6)
    f14 = js["f14"]
    Lh3 = obs([[0, 1, 0], [0, 0, 1 / M], [0, 0, 0]], [[1, 0, 0]], np.append(pp(fo * 2.2 / 8, 0.707), -1))
    Lz3 = obs([[0, 1, 0], [0, -a, 1 / M], [0, 0, 0]], [[1, 0, 0]], np.append(pp(fo * 2.2 / 8, 0.707), -1))
    Lt3 = obs([[0, 1, 0], [0, 0, 1 / J], [0, 0, 0]], [[1, 0, 0]], np.append(pp(fo * 2.2 / 0.8, 0.707), -10))
    check("F.14 L_h, L_z, L_theta (with d states)", list(Lh3) + list(Lz3) + list(Lt3), f14["Lh"] + f14["Lz"] + f14["Lt"])
    # F.15
    Pl_, Pi_, Po_ = ct.tf([1 / M], [1, 0, 0]), ct.tf([1 / J], [1, 0, 0]), ct.tf([-g], [1, a, 0])
    check("F.15 |P(j1)| dB (lon, inner, outer)", [20 * np.log10(abs(G(1j))) for G in (Pl_, Pi_, Po_)], js["f15"])
    # F.16 / F.17 with the reference F.10 gains and the dirty derivative
    sg = 0.05
    def cpid(kP, kI, kD):
        return ct.tf([kD + sg * kP, kP + sg * kI, kI], [sg, 1, 0]) if kI else ct.tf([kD + sg * kP, kP], [sg, 1])
    kIh, kIz = 0.01, -0.0005
    Ll = Pl_ * cpid(kP, kIh, kD); Li = Pi_ * cpid(f8["kPth"], 0, f8["kDth"]); Lo = Po_ * cpid(f8["kPz"], kIz, f8["kDz"])
    Ti = ct.feedback(Li, 1)
    f16 = js["f16"]
    check("F.16a e_parab with D on h (5 kD/kI)", 5 * kD / kIh, f16["eParImpl"])
    check("F.16b |PC(j30)|", abs(Ll(30j)), f16["gn"])
    check("F.16c 1/k_P,th", 1 / f8["kPth"], f16["gdin"], rel=1e-4)
    from scipy.optimize import brentq
    w01 = brentq(lambda w: abs(Ti(1j * w)) - 0.1, 1, 1000)
    check("F.16d |T_in| = 0.1 frequency (brentq)", w01, f16["wSensor"], rel=1e-6)
    check("F.16e 1/|PC(j0.1)|", 1 / abs(Lo(0.1j)), f16["gr"])
    check("F.16e exact |S(j0.1)|", abs(1 / (1 + Lo(0.1j))), f16["grExact"])
    check("F.16f 1/|PC(j0.01)|", 1 / abs(Lo(0.01j)), f16["gout"])
    Lo_pd = Po_ * cpid(f8["kPz"], 0, f8["kDz"])
    for name, Lg in (("lon", Ll), ("inner", Li), ("outer", Lo_pd)):
        gm, pm, wg, wc = ct.margin(Lg)
        Tc = ct.feedback(Lg, 1)
        # |T| = 1/sqrt(2); control.bandwidth uses -3 dB exactly (0.1% higher level)
        bw = brentq(lambda w: abs(Tc(1j * w)) - 1 / np.sqrt(2), 1e-3, ct.bandwidth(Tc) * 1.5)
        check(f"F.17 {name}: PM, w_co", [pm, wc], js["f17"][name][:2], rel=2e-3)
        check(f"F.17 {name}: bandwidth (brentq)", bw, js["f17"][name][2], rel=1e-6)
    # F.18 reference loopshapes: k from crossover, margins from python-control
    s_ = ct.tf("s")
    lead = lambda w, Mm: Mm * (s_ + w / np.sqrt(Mm)) / (s_ + w * np.sqrt(Mm))
    lpf = lambda q: q / (s_ + q)
    C0l = (s_ + 0.23) / s_ * lead(1.5, 35) * lpf(70) * lpf(70)
    C0i = lead(10, 20) * lpf(100)
    kl, ki = 1 / abs((Pl_ * C0l)(1.5j)), 1 / abs((Pi_ * C0i)(10j))
    Pout = Po_ * ct.feedback(Pi_ * ki * C0i, 1)
    C0o = -(s_ + 0.1) / s_ * lead(1, 20) * lpf(30)
    ko = 1 / abs((Pout * C0o)(1j))
    check("F.18 gains k (lon, inner, outer)", [kl, ki, ko], js["f18"]["k"])
    pms = [ct.margin(L)[1] for L in (Pl_ * kl * C0l, Pi_ * ki * C0i, Pout * ko * C0o)]
    check("F.18 phase margins", pms, js["f18"]["pm"], rel=2e-3)
    allspec = all(all(v for kk, v in d.items() if isinstance(v, bool)) for d in js["f18"]["specs"].values())
    print(f"{'ok  ' if allspec else 'FAIL'} F.18 reference design meets every spec check")
    if not allspec:
        FAIL.append("F.18 specs")
    # F.10: closed loop, JS vs. an independent Python nested PID (B.10 listing pattern)
    py = f10_python(js["f10"])
    check("F.10 closed loop z, h, theta, f_r, f_l samples", py, js["f10"]["samples"], rel=1e-9, abs_=1e-9)


def f8_sat(trh, trz):
    """Nonlinear VTOL with the F.8 PD loops (true rates), h_r = 2 m step, z_r = 3 + 2.5 square
    at 0.08 Hz, 50 s. True if a rotor demand leaves [0, 10] N."""
    mc, mr, Jc, d, mu, g, fmax = 1.0, 0.25, 0.0042, 0.3, 0.1, 9.81, 10.0
    M, J, a = mc + 2 * mr, Jc + 2 * mr * d ** 2, mu / (mc + 2 * mr)
    z_ = 0.707
    wnh, wnt, wnz = 2.2 / trh, 2.2 / 0.8, 2.2 / trz
    kPh, kDh = M * wnh ** 2, M * 2 * z_ * wnh
    kPt, kDt = J * wnt ** 2, J * 2 * z_ * wnt
    kPz, kDz = -wnz ** 2 / g, (a - 2 * z_ * wnz) / g

    def f(x, u):
        F, tau = u[0] + u[1], d * (u[0] - u[1])
        return np.array([x[3], x[4], x[5], (-F * np.sin(x[2]) - mu * x[3]) / M, -g + F * np.cos(x[2]) / M, tau / J])
    x, Ts = np.zeros(6), 0.01
    for k in range(5001):
        t = k * Ts
        zr = 3 + (2.5 if (t % 12.5) <= 6.25 else -2.5)
        Fc = M * g + kPh * (2 - x[1]) - kDh * x[4]
        thd = kPz * (zr - x[0]) - kDz * x[3]
        tau = kPt * (thd - x[2]) - kDt * x[5]
        u = np.array([Fc / 2 + tau / (2 * d), Fc / 2 - tau / (2 * d)])
        if np.any(u > fmax + 1e-9) or np.any(u < -1e-9):
            return True
        u = np.clip(u, 0, fmax)
        k1 = f(x, u); k2 = f(x + Ts / 2 * k1, u); k3 = f(x + Ts / 2 * k2, u); k4 = f(x + Ts * k3, u)
        x = x + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return False


def f10_python(js):
    """Nested PID from measured outputs: dirty derivatives (sigma = 0.05), trapezoid
    integrators gated at |ydot| < 0.5, z_prev = first sample, error_prev = 0
    (B.10 listing, p. 163-165). True plant has the mismatch; controller uses nominal."""
    g_ = js["gains"]; pt = js["pTrue"]
    mc, mr, Jc, d, mu, g, fmax = 1.0, 0.25, 0.0042, 0.3, 0.1, 9.81, 10.0
    M = mc + 2 * mr
    Ts, sigma = 0.01, 0.05
    beta, gamma = (2 * sigma - Ts) / (2 * sigma + Ts), 2 / (2 * sigma + Ts)
    Mt, Jt = pt["mc"] + 2 * pt["mr"], pt["Jc"] + 2 * pt["mr"] * pt["d"] ** 2

    def f(x, u):
        F, tau = u[0] + u[1], pt["d"] * (u[0] - u[1])
        return np.array([x[3], x[4], x[5], (-F * np.sin(x[2]) - pt["mu"] * x[3]) / Mt, (-Mt * g + F * np.cos(x[2])) / Mt, tau / Jt])

    class PID:
        def __init__(s, kP, kI, kD, vbar):
            s.kP, s.kI, s.kD, s.vbar = kP, kI, kD, vbar
            s.I, s.eprev, s.yprev, s.ydot = 0.0, 0.0, None, 0.0

        def update(s, r, y):
            e = r - y
            if s.yprev is None:
                s.yprev = y
            s.ydot = beta * s.ydot + gamma * (y - s.yprev)
            if abs(s.ydot) < s.vbar:
                s.I += Ts / 2 * (e + s.eprev)
            u = s.kP * e + s.kI * s.I - s.kD * s.ydot
            s.eprev, s.yprev = e, y
            return u
    ph, pz, pth = PID(g_["kPh"], g_["kIh"], g_["kDh"], 0.5), PID(g_["kPz"], g_["kIz"], g_["kDz"], 0.5), PID(g_["kPth"], 0, g_["kDth"], np.inf)
    x, rec = np.zeros(6), {}
    for k in range(max(js["KS"]) + 1):
        t = k * Ts
        hr, zr = 2.0, 3 + 2.5
        F = M * g + ph.update(hr, x[1])
        thd = pz.update(zr, x[0])
        tau = pth.update(thd, x[2])
        u = np.array([F / 2 + tau / (2 * d), F / 2 - tau / (2 * d)])
        rec[k] = [x[0], x[1], x[2], u[0], u[1]]
        ua = np.clip(u, 0, fmax)
        k1 = f(x, ua); k2 = f(x + Ts / 2 * k1, ua); k3 = f(x + Ts / 2 * k2, ua); k4 = f(x + Ts * k3, ua)
        x = x + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return [rec[k] for k in js["KS"]]


if __name__ == "__main__":
    sys.exit(main())
