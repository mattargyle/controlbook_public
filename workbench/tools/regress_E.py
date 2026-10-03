#!/usr/bin/env python3
"""Numerical regression for Design Study E (block on beam).

Run with the repo venv:  .venv/bin/python workbench/tools/regress_E.py

The book has no worked solutions for E, so this checks the workbench JS three ways:
  1. f(x, u): the ten test vectors of _E_blockbeam/python/testDynamics.py, with the
     same |error| < 1e-14 test the script uses (it assumes g = 9.81).
  2. Designs: JS gains against independent Python (closed-form formulas,
     python-control place / margin / bandwidth, numpy eigenvalues, a Routh-free
     bisection for the critical integrator gain).
  3. Closed-loop simulations: JS against a Python re-implementation of the same
     controllers (nested PD, digital nested PID with mismatch, state feedback with
     integrator, disturbance observer), sample by sample.
Prints one line per check and exits 1 on any failure.
"""
import pathlib
import sys

import control as ct
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from js_eval import js_eval  # noqa: E402

REPO = HERE.parent.parent
TEST = REPO / "_E_blockbeam" / "python" / "testDynamics.py"
P = dict(m1=0.35, m2=2.0, ell=0.5, g=9.8, F_max=15.0)
MM = dict(m1=12, m2=-9, ell=15)                       # the pages' fixed α = 0.2 draw
KS = [100, 1000, 2500, 3999, 5000, 7500, 9999]
fails = []


def report(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'}  {name:34s} {detail}")
    if not ok:
        fails.append(name)


def test_vectors():
    src = TEST.read_text().split("num_tests")[0].replace("from blockbeamDynamics import blockbeamDynamics", "")
    ns = {"np": np}
    exec(src, ns)
    return ns["x_tests"].reshape(-1, 4), ns["u_tests"], ns["xdot_tests"].reshape(-1, 4)


# ---------------------------------------------------------------- python side --
def f(x, F, p):
    z, th, zd, thd = x
    m1, m2, l, g = p["m1"], p["m2"], p["ell"], p["g"]
    zdd = (1.0 / m1) * (m1 * z * thd ** 2 - m1 * g * np.sin(th))
    thdd = (1.0 / ((m2 * l ** 2) / 3.0 + m1 * z ** 2)) * (F * l * np.cos(th) - 2.0 * m1 * z * zd * thd
                                                          - m1 * g * z * np.cos(th) - m2 * g * l / 2.0 * np.cos(th))
    return np.array([zd, thd, zdd, thdd])


def rk4(x, u, Ts, p):
    k1 = f(x, u, p); k2 = f(x + Ts / 2 * k1, u, p); k3 = f(x + Ts / 2 * k2, u, p); k4 = f(x + Ts * k3, u, p)
    return x + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def sat(u, L):
    return max(-L, min(L, u))


def true_params():
    q = dict(P)
    for k, v in MM.items():
        q[k] = P[k] * (1 + v / 100)
    return q


def lin(p, comp="eq"):
    m1, m2, l, g = p["m1"], p["m2"], p["ell"], p["g"]
    ze = l / 2; De = m2 * l ** 2 / 3 + m1 * ze ** 2
    A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, -g, 0, 0], [(-m1 * g / De) if comp == "eq" else 0.0, 0, 0, 0]])
    B = np.array([[0], [0], [0], [l / De]]); C = np.array([[1., 0, 0, 0], [0, 1, 0, 0]])
    return A, B, C, ze, m1 * g * ze / l + m2 * g / 2, De


def pd_design(trth, zth, M, zz, p=P):
    _, B, _, ze, _, De = lin(p)
    b0 = p["ell"] / De
    wth, wz = 2.2 / trth, 2.2 / (M * trth)
    return dict(b0=b0, kPth=wth ** 2 / b0, kDth=2 * zth * wth / b0, kPz=-wz ** 2 / p["g"], kDz=-2 * zz * wz / p["g"])


def pair(wn, z):
    return np.roots([1, 2 * z * wn, wn ** 2])


def ss_poles(trth, zth, trz, zz):
    return np.concatenate([pair(2.2 / trth, zth), pair(2.2 / trz, zz)])


def ffl(z, p):
    return p["m1"] * p["g"] * z / p["ell"] + p["m2"] * p["g"] / 2


def square(t):
    return (0.15 if (t % 100) <= 50 else -0.15) + 0.25


def square05(t):
    return (0.15 if (t % 20) <= 10 else -0.15) + 0.25


def sim_nested(d, ref, tEnd, ptrue, dirty=False, kIz=0.0, vbar=0.05, sigma=0.05, Ts=0.01):
    beta, gam = (2 * sigma - Ts) / (2 * sigma + Ts), 2 / (2 * sigma + Ts)
    x = np.array([0.25, 0, 0, 0.]); out = []
    zp = None; thp = 0; zd = thd = Iz = ezp = 0.0
    for k in range(int(round(tEnd / Ts)) + 1):
        t = k * Ts; r = ref(t); z, th = x[0], x[1]
        if zp is None:
            zp, thp = z, th
        if dirty:
            zd = beta * zd + gam * (z - zp); thd = beta * thd + gam * (th - thp)
        else:
            zd, thd = x[2], x[3]
        ez = r - z
        if kIz and abs(zd) < vbar:
            Iz += Ts / 2 * (ez + ezp)
        thr = d["kPz"] * ez + kIz * Iz - d["kDz"] * zd
        F = ffl(z, P) + d["kPth"] * (thr - th) - d["kDth"] * thd
        out.append((x[0], x[1], F))
        zp, thp, ezp = z, th, ez
        x = rk4(x, sat(sat(F, P["F_max"]), P["F_max"]), Ts, ptrue)
    return out


def sim_ss(level, K, ki, Lg, ref, tEnd, ptrue, dist, Ts=0.01):
    A, B, C, ze, Fe, _ = lin(P)
    x = np.array([0.25, 0, 0, 0.]); out = []
    I = ep = 0.0; uprev = None; xh = np.zeros(4); dh = 0.0

    def fobs(v, yt, u):
        xx, dd = v[:4], v[4]
        innov = yt - xx[:2]
        uu = u - Fe + (dd if level == "dobs" else 0.0)
        dx = A @ xx + B[:, 0] * uu + Lg[:4] @ innov
        return np.concatenate([dx, [Lg[4] @ innov if level == "dobs" else 0.0]])
    for k in range(int(round(tEnd / Ts)) + 1):
        t = k * Ts; r = ref(t)
        if level == "dobs":
            if uprev is None:
                uprev = Fe
            yt = np.array([x[0] - ze, x[1]]); v = np.concatenate([xh, [dh]])
            k1 = fobs(v, yt, uprev); k2 = fobs(v + Ts / 2 * k1, yt, uprev); k3 = fobs(v + Ts / 2 * k2, yt, uprev); k4 = fobs(v + Ts * k3, yt, uprev)
            v = v + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4); xh, dh = v[:4], v[4]; xt = xh.copy()
        else:
            xt = x - np.array([ze, 0, 0, 0])
        z = xt[0] + ze; e = r - z
        dterm = dh if level == "dobs" else 0.0
        Itry = I + Ts / 2 * (e + ep)
        if abs(Fe - K @ xt - ki * Itry - dterm) <= P["F_max"]:
            I = Itry
        ep = e
        F = Fe - K @ xt - ki * I - dterm
        u = sat(F, P["F_max"]); uprev = u
        out.append((x[0], x[1], F))
        x = rk4(x, sat(u + dist, P["F_max"]), Ts, ptrue)
    return out


def structured_L(A, B, pz, pth, pD=None):
    az = np.poly(pz).real
    n = 5 if pD is not None else 4
    L = np.zeros((n, 2)); L[0, 0], L[2, 0] = az[1], az[2]; L[2, 1] = A[2, 1]; L[3, 0] = A[3, 0]
    if pD is None:
        at = np.poly(pth).real; L[1, 1], L[3, 1] = at[1], at[2]
    else:
        at = np.poly(np.concatenate([pth, [pD]])).real; L[1, 1], L[3, 1], L[4, 1] = at[1], at[2], at[3] / B[3, 0]
    return L


# -------------------------------------------------------------------- JS side --
JS = r"""
const sys = WB.systems.E, E = WB.E, CH = WB.studies.E.chapters;
const p = {m1: .35, m2: 2, ell: .5, F_max: 15, g: 9.8};
const out = {};
out.f = %(X)s.map((x, i) => sys.f(x, %(U)s[i], {...p, g: 9.81}));
const mm = %(MM)s, pt = {...p};
for (const k of Object.keys(mm)) pt[k] = p[k] * (1 + mm[k] / 100);
const S = {sim: {Ts: 0.01}};
const ctx = {sys, pModel: p, S};
const KS = %(KS)s;
const sq = (f) => WB.sim.makeReference({type: 'square', amplitude: 0.15, offset: 0.25, frequency: f, tStep: 0});
function run(controller, ref, tEnd, ptrue, dist) {
  const res = WB.sim.simulate({plant: {f: (x, u) => sys.f(x, u, ptrue), h: sys.h, uLimit: 15}, controller, reference: ref,
    disturbance: () => dist, noise: null, x0: [0.25, 0, 0, 0], Ts: 0.01, tEnd});
  return KS.map((k) => [res.yAll[0][k], res.yAll[1][k], res.uDemand[k]]);
}
// E.8 and E.10
const d8 = E.pdDesign(p, {trTh: 1, zetaTh: .707, M: 10, zetaZ: .707});
out.d8 = d8;
out.pd8 = run(E.nestedPID(ctx, d8, {comp: 'fl', meas: 'state'}), sq(0.01), 100, p, 0);
const d10 = {...E.pdDesign(p, {trTh: .4, zetaTh: .707, M: 10, zetaZ: .707}), kIz: -0.01};
out.pid10 = run(E.nestedPID(ctx, d10, {comp: 'fl', meas: 'dirty', sigma: .05, antiwindup: 'gate', vbar: .05}), sq(0.01), 100, pt, 0);
out.kIcrit = -p.g * d8.kDz * d8.kPz;
// E.11–E.14
const kn = {trTh: .5, zetaTh: .8, trZ: 1.5, zetaZ: .8, pI: -1, obsFactor: 5, zetaObs: .8, pD: -5, comp: 'eq', obsMode: 'decoupled'};
out.sf = E.ssDesign(p, kn, 'sf'); out.sfi = E.ssDesign(p, kn, 'sfi');
out.obs = E.ssDesign(p, kn, 'obs'); out.dobs = E.ssDesign(p, kn, 'dobs');
out.obsZ = E.ssDesign(p, {...kn, obsMode: 'zonly'}, 'obs'); out.dobsZ = E.ssDesign(p, {...kn, obsMode: 'zonly'}, 'dobs');
out.sfFl = E.ssDesign(p, {...kn, comp: 'fl'}, 'sf');
out.sim12 = run(E.makeSS(ctx, out.sfi, 'sfi', {comp: 'eq', antiwindup: 'clamp'}), sq(0.05), 40, pt, 1);
out.sim14 = run(E.makeSS(ctx, out.dobs, 'dobs', {comp: 'eq', antiwindup: 'clamp'}), sq(0.05), 40, pt, 0.5);
// E.16–E.18
const c16 = {sys, pModel: p, S, st: CH.ch16.defaults(sys)};
const s16 = CH.ch16.specs(c16);
out.e16 = {gr: s16.gr, gdin: s16.gdin, gdinEdge: s16.gdinEdge, gn: s16.gn, gdout: s16.gdout, gdoutExact: s16.gdoutExact, esin: s16.esin};
const c17 = {sys, pModel: p, S, st: CH.ch17.defaults(sys)};
const l17 = CH.ch17.loops(c17);
out.e17 = {pmIn: l17.mi.pm, wcIn: l17.mi.wc, bwIn: l17.bwIn, pmOut: l17.mo.pm, wcOut: l17.mo.wc, bwOut: l17.bwOut};
const c18 = {sys, pModel: p, S, st: {d: {in: E.freq.sampleInner(), out: E.freq.sampleOuter(), pf: {on: true, p: 0.7}}}};
const d18 = E.freq.lsDesign(c18);
out.e18 = {pmIn: d18.si.mg.pm, wcIn: d18.si.mg.wc, loIn: d18.si.lo, hiIn: d18.si.hi, pmOut: d18.so.mg.pm, wcOut: d18.so.mg.wc, loOut: d18.so.lo, hiOut: d18.so.hi,
  ok: d18.si.lowOk && d18.si.highOk && d18.si.pmOk && d18.so.lowOk && d18.so.highOk && d18.so.pmOk && d18.stableIn && d18.stableOut, peakFT: d18.peakFT};
// E.8(f) answer (bisection on the simulated peak force)
out.e8f = E.pid.fastestTrZ({...ctx, S: {sim: {Ts: 0.01}}}, {trTh: 1, zetaTh: .707, M: 10, zetaZ: .707, rule: '2.2', step: 0.25});
return out;
"""


def cplx(ps):
    return np.array([q["re"] + 1j * q["im"] for q in ps])


def same_set(a, b, tol=1e-6):
    a, b = sorted(a, key=lambda c: (c.real, c.imag)), sorted(b, key=lambda c: (c.real, c.imag))
    return max(abs(x - y) for x, y in zip(a, b)) / max(1, max(abs(x) for x in b)) < tol


def main():
    X, U, XD = test_vectors()
    js = js_eval(JS % dict(X=X.tolist(), U=U.tolist(), MM=MM, KS=KS))

    # 1. testDynamics vectors, same tolerance as the script
    err = np.abs(np.array(js["f"]) - XD)
    report("f(x,u) vs testDynamics.py", bool((err < 1e-14).all()), f"max |err| = {err.max():.2e} over {len(X)} vectors (g = 9.81)")
    errpy = np.abs(np.array([f(x, u, {**P, "g": 9.81}) for x, u in zip(X, U)]) - XD)
    report("python f (same formula) vs test", bool((errpy < 1e-14).all()), f"max |err| = {errpy.max():.2e}")

    # 2. designs
    d8 = pd_design(1, 0.707, 10, 0.707)
    e = max(abs(js["d8"][k] - d8[k]) / abs(d8[k]) for k in ["kPth", "kDth", "kPz", "kDz"])
    report("E.8 gains (closed form)", e < 1e-12, f"rel err {e:.1e}: kPθ={d8['kPth']:.6g} kDθ={d8['kDth']:.6g} kPz={d8['kPz']:.6g} kDz={d8['kDz']:.6g}")
    # 4th-order check: python-control closes both loops on the design model
    s = ct.tf("s"); g = P["g"]
    Lin = d8["b0"] / s ** 2 * (d8["kPth"] + d8["kDth"] * s)
    Tin = ct.feedback(Lin, 1)
    report("E.8(c) inner DC gain", abs(ct.dcgain(Tin) - 1) < 1e-12, f"k_DC = {ct.dcgain(Tin):.12g}")
    # E.P.6: critical kI by bisection on the closed-loop roots
    lo, hi = -1.0, 0.0
    for _ in range(200):
        mid = (lo + hi) / 2
        r = np.roots([1, -g * d8["kDz"], -g * d8["kPz"], -g * mid])
        lo, hi = (mid, hi) if max(r.real) >= 0 else (lo, mid)
    report("E.P.6 k_I,crit", abs(js["kIcrit"] - hi) < 1e-9, f"JS {js['kIcrit']:.8g}, bisection {hi:.8g}")

    A, B, C, ze, Fe, De = lin(P)
    poles = ss_poles(0.5, 0.8, 1.5, 0.8)
    K = ct.place(A, B, poles); Cr = np.array([[1, 0, 0, 0]])
    kr = -1 / (Cr @ np.linalg.inv(A - B @ K) @ B)[0, 0]
    e = max(np.max(np.abs(np.array(js["sf"]["K"]) - K[0]) / np.abs(K[0])), abs(js["sf"]["kr"] - kr) / abs(kr))
    report("E.11 K, k_r vs control.place", e < 1e-8, f"rel err {e:.1e}: K={np.round(K[0], 5)}, kr={kr:.6g}")
    report("E.11 k_r = K1 + m1 g/l (F_e)", abs(kr - (K[0, 0] + P["m1"] * g / P["ell"])) < 1e-9, f"{kr:.8g} vs {K[0, 0] + P['m1'] * g / P['ell']:.8g}")
    Afl, _, _, _, _, _ = lin(P, "fl")
    Kfl = ct.place(Afl, B, poles)
    report("E.11 k_r = K1 with F_fl", abs(js["sfFl"]["kr"] - Kfl[0, 0]) < 1e-8 * abs(Kfl[0, 0]), f"kr {js['sfFl']['kr']:.8g}, K1 {Kfl[0, 0]:.8g}")
    A1 = np.block([[A, np.zeros((4, 1))], [-Cr, np.zeros((1, 1))]]); B1 = np.vstack([B, [[0]]])
    K1 = ct.place(A1, B1, np.concatenate([poles, [-1]]))
    jk = np.array(js["sfi"]["K"] + [js["sfi"]["ki"]])
    e = np.max(np.abs(jk - K1[0]) / np.abs(K1[0]))
    report("E.12 K, k_I vs control.place", e < 1e-8, f"rel err {e:.1e}: K1={np.round(K1[0], 5)}")
    wo = 5 * 2.2
    pz, pth = pair(wo / 1.5, 0.8), pair(wo / 0.5, 0.8)
    for name, key, aug, pD in [("E.13 observer L", "obs", False, None), ("E.14 dist. observer L", "dobs", True, -5)]:
        Lj = np.array(js[key]["L"])
        Aa, Ca = (np.block([[A, B], [np.zeros((1, 5))]]), np.hstack([C, np.zeros((2, 1))])) if aug else (A, C)
        target = np.concatenate([pz, pth] + ([[pD]] if aug else []))
        ok = same_set(np.linalg.eigvals(Aa - Lj @ Ca), target) and np.allclose(Lj, structured_L(A, B, pz, pth, pD), rtol=1e-12)
        Lp = ct.place(Aa.T, Ca.T, target).T
        report(name + " (decoupled)", ok, f"eig(A-LC) = targets; |L|max {np.abs(Lj).max():.4g} vs place {np.abs(Lp).max():.4g}")
        Lz = np.array(js[key + "Z"]["L"])
        report(name + " (z only)", same_set(np.linalg.eigvals(Aa - Lz @ Ca), target, 1e-5), "eig(A-LC) = targets")

    s16 = js["e16"]
    kp, kd, sig = d8["kPth"], d8["kDth"], 0.05
    Cin = kp + kd * s / (sig * s + 1); Lin = d8["b0"] / s ** 2 * Cin
    Cout = d8["kPz"] - 1e-4 / s + d8["kDz"] * s / (sig * s + 1); Lout = -g / s ** 2 * Cout
    mg = lambda G, w: abs(G(1j * w))
    py16 = dict(gr=1 / mg(Lin, 1.0), gdin=1 / kp, gdinEdge=1 / mg(Cin, 0.8), gn=mg(Lin, 300), gdout=1 / mg(Lout, 0.1),
                gdoutExact=1 / abs(1 + Lout(0.1j)), esin=2 / abs(1 + Lout(0.6j)))
    e = max(abs(s16[k] - py16[k]) / abs(py16[k]) for k in py16)
    report("E.16 spec numbers vs python-control", e < 1e-9, " ".join(f"{k}={py16[k]:.4g}" for k in py16))
    s17 = js["e17"]
    gmi, pmi, _, wci = ct.margin(Lin); gmo, pmo, _, wco = ct.margin(Lout)
    bwi, bwo = ct.bandwidth(ct.feedback(Lin, 1)), ct.bandwidth(ct.feedback(Lout, 1))
    ok = abs(s17["pmIn"] - pmi) < 0.05 and abs(s17["pmOut"] - pmo) < 0.05 and abs(s17["wcIn"] / wci - 1) < 2e-3 and abs(s17["wcOut"] / wco - 1) < 2e-3 \
        and abs(s17["bwIn"] / bwi - 1) < 0.02 and abs(s17["bwOut"] / bwo - 1) < 0.02
    report("E.17 margins/bandwidth vs control", ok, f"PM {pmi:.2f}°@{wci:.3f} / {pmo:.2f}°@{wco:.3f}, bw {bwi:.3f} / {bwo:.3f} (JS {s17['bwIn']:.3f} / {s17['bwOut']:.3f}, grid)")
    s18 = js["e18"]
    lead = lambda w, M: M * (s + w / np.sqrt(M)) / (s + w * np.sqrt(M))
    Ci = 135 * lead(40, 20) * 400 / (s + 400); Li = d8["b0"] / s ** 2 * Ci; Ti = ct.feedback(Li, 1)
    Co = -0.0735 * (s + 0.2) / s * lead(2, 30) * 30 / (s + 30); Lo = -g / s ** 2 * Ti * Co
    _, pmi, _, wci = ct.margin(Li); _, pmo, _, wco = ct.margin(Lo)
    ok = s18["ok"] and abs(s18["pmIn"] - pmi) < 0.05 and abs(s18["pmOut"] - pmo) < 0.05
    report("E.18 sample design vs control", ok, f"PM in {pmi:.2f}° @ {wci:.2f}, out {pmo:.2f}° @ {wco:.3f}, all specs met: {s18['ok']}, peak |FT| {20 * np.log10(s18['peakFT']):.2f} dB")

    # E.8(f): python bisection on the same simulated peak
    def peak(trz):
        dd = pd_design(trz / 10, 0.707, 10, 0.707)
        x = np.array([0.25, 0, 0, 0.]); pk = 0
        for k in range(int(round(max(5, 5 * trz) / 0.01)) + 1):
            z, th, zd, thd = x
            F = ffl(z, P) + dd["kPth"] * (dd["kPz"] * (0.5 - z) - dd["kDz"] * zd - th) - dd["kDth"] * thd
            pk = max(pk, abs(F)); x = rk4(x, F, 0.01, P)
        return pk
    lo, hi = 0.1, 10
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if peak(mid) > 15 else (lo, mid)
    report("E.8(f) fastest t_rz", abs(js["e8f"] - hi) < 1e-6, f"JS {js['e8f']:.5f} s, python {hi:.5f} s")

    # 3. closed-loop simulations, sample by sample
    pt = true_params()
    cases = {
        "pd8": sim_nested(d8, square, 100, P),
        "pid10": sim_nested(pd_design(0.4, 0.707, 10, 0.707), square, 100, pt, dirty=True, kIz=-0.01),
        "sim12": sim_ss("sfi", K1[0, :4], K1[0, 4], None, square05, 40, pt, 1.0),
        "sim14": sim_ss("dobs", np.array(js["dobs"]["K"]), js["dobs"]["ki"], structured_L(A, B, pz, pth, -5), square05, 40, pt, 0.5),
    }
    labels = {"pd8": "E.8 nested PD (true state)", "pid10": "E.10 digital PID, 20% mismatch", "sim12": "E.12 SF + integrator, d = 1 N", "sim14": "E.14 dist. observer, d = 0.5 N"}
    for name, py in cases.items():
        pyk = np.array([py[k] for k in KS if k < len(py)])
        jk = np.array(js[name])[:len(pyk)]
        e = np.max(np.abs(pyk - jk))
        report(f"sim {labels[name]}", e < 1e-9, f"max |py - js| = {e:.2e} (z, θ, F at {len(pyk)} samples)")

    print(f"\n{len(fails)} failure(s)" if fails else "\nall checks passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
