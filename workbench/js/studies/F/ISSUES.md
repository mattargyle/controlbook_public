# Design Study F: issues found in the book and repo

As of 2026-10-03. This covers Design Study F (the planar VTOL): problems F.2–F.18 and F.P.6.

The book has **no worked solutions** for F. Every answer in the workbench was derived from the problem statements and Chapters 2–18. Each answer is cross-checked a second way in `workbench/tools/regress_F.py`, using python-control, sympy, an independent nonlinear Python simulation, or Brent root finding.

I found:
- 15 ambiguities or inconsistencies in the problem statements;
- 3 repo issues, including the mixing-sign mix-up the study notes warn about.

**Dynamics.** The JS `f(x, u)` reproduces every expected vector in `_F_planar_vtol/python/testDynamics.py` to 0 error (the tolerance there is 1e-14). It also matches a sympy Euler-Lagrange derivation from the book's figure to 2e-15.

Page numbers are PDF pages (book page + 8). Parameters are from p. 393: m_c = 1 kg, J_c = 0.0042 kg·m², m_r = m_ℓ = 0.25 kg, d = 0.3 m, μ = 0.1 kg/s, g = 9.81 m/s².

## Problem statements: ambiguities and inconsistencies

| Problem | PDF page | Issue | Workbench reading / effect |
| --- | --- | --- | --- |
| Intro | 393 | No initial conditions, simulation length, altitude reference, or sample time are given anywhere in F. The template `VTOLParam.py` leaves them blank. | h(0) = z(0) = θ(0) = 0, h_r = 2 m step, T_s = 0.01 s, t_end = 50–80 s. All of these are adjustable in the left panel. |
| Intro | 393 | m_ℓ is listed separately from m_r but has the same value. Every later formula (F.12 hint, F.14 snippet, `testDynamics.py`) uses m_c + 2m_r. | One slider m_r = m_ℓ. |
| F.8(e) | 397 | z_r is a square wave at 0.08 Hz, a 6.25 s half period. The design in (d) has t_r,z = 10·0.8 = 8 s. | z never reaches the commanded level before the reference switches. The rise time is longer than the half period, and settling takes about 2.5× longer still. The lateral-response readout shows "not settled". Later chapters use 0.04 Hz or steps where a check needs a settled response. |
| F.8(f) | 398 | "Tune t_r,h and t_r,z for the fastest response without saturation." Both reference steps start at t = 0, so the loops share the rotor headroom (f_max − F_e/2 = 2.64 N per rotor above hover). | There is no single answer, only a trade-off curve. The altitude limit alone is t_r,h ≈ 1.67 s; the lateral limit alone is t_r,z ≈ 1.6 s. The lateral limit means M ≈ 2, which breaks the 5–10× separation rule. The checker accepts any pair where neither loop can go 10% faster. |
| F.9, F.16(a) | 398, 402 | Table 9-1 (p. 141) assumes C(s) acts on the error (Fig. 9-1). The F loops use the derivative on the output (Fig. 7-2, which F.7 asks for). That removes the controller zero from the r → y transfer function. | Each loop loses one tracking type. Altitude PD is type 2 by the book but type 1 as implemented, with e_ramp = k_D/k_P. F.16(a): with the F.10 PID the book convention gives e_ss = 0 for a parabola, but the implemented loop gives 5k_D,h/k_I,h ≈ 292 m. Input-disturbance types are unaffected. The checker uses the book convention, and the readouts show both. |
| F.16(a) | 402 | "A parabola with curvature 5." A.16 says θ_r = 5t² instead. | Read as r̈ = 5, i.e. R(s) = 5/s³. The answer (0, type 3) does not depend on this reading. |
| F.16(c) | 402 | It asks for the "percentage of the input disturbance" in θ, which compares rad with N·m. With PD, \|C\| is smallest at DC, so the worst case in ω ≤ 2 rad/s is a constant torque, not ω = 2. | Uses 1/min\|C\| = 1/k_P,θ = 2.69 (269%), and also shows the exact max\|P/(1+PC)\|. |
| F.16(d) | 402 | "What sensor characteristics keep θ noise under 0.1°?" is open-ended. | Asks for the frequency above which 1° of sensor noise is attenuated below 0.1° (\|T_in\| = 0.1, at 27.34 rad/s), and explains the low-frequency requirement in the solution. |
| F.17(c) | 403 | F.17 says to use the F.10 gains, but (c) says the outer loop is "under PD control". F.10 and F.P.6(b) suggest giving the outer loop an integrator. | Defaults to PD (k_I,z = 0), with a toggle for PID. |
| F.18(b) | 404 | It asks for "closed loop bandwidth approximately ω_co = 10 rad/s", which treats bandwidth and crossover as the same thing. With PM ≈ 60° the closed-loop bandwidth is about 1.6 ω_co. | Checked as a crossover spec (within 20% of 10 rad/s). The bandwidth is reported alongside. |
| F.18(a) | 403 | Rejecting constant input disturbances needs an integrator in C. With the double-integrator plant, the loop then has three integrators. | Every such design is conditionally stable: the phase is below −180° at low frequency, and python-control reports a negative-dB gain margin. |
| F.18(c) | 404 | The outer loop also needs an integrator (constant input disturbances), and its plant already has a pole at the origin. | The reference outer design is conditionally stable too: GM = 0.0041 (−47.7 dB) at 0.035 rad/s, besides +16.9 dB at 5.2 rad/s. The margins card lists every crossing for the current design and says which loops are conditionally stable. |
| F.13(e), F.14(a) | 400–401 | The disturbance models change from problem to problem. F.12 adds a lateral force F_wind [N] to z̈. F.13(e) adds input force and torque disturbances. F.14's snippet adds a wind *speed* to ż (`zdot = zdot + wind`), drops the −μż drag from z̈, and adds `altitude_dist` straight to ḧ as an acceleration. | Every kind is a separate slider (d_F, d_τ, F_wind, wind w, d_h), and μż is always kept. A constant wind speed w is exactly a force μw in the observer's ż coordinates, so the F.14 lateral disturbance observer's d_z state absorbs it. |
| F.11(a) | 399 | The poles are specified only by inequalities (ζ ≥ ζ_spec, ω_n ≥ ω_n,spec), so K is not unique. | The checker accepts any K whose poles meet the inequalities, and requires k_r to give unity DC gain for *that* K. The reference uses the F.8 pole pairs. |
| F.13(c) | 400 | The lateral outputs are z and θ, so the 4×2 observer gain L is not unique. place() on the full (A, C) returns one of many. | Uses the block structure L = [[L_z1, 0], [0, L_θ1], [L_z2, 0], [0, L_θ2]], which makes A − LC block-triangular, so the design is two SISO problems. The eigenvalues are verified with numpy. |

Specs the workbench had to invent (the book leaves them to "tune"); the checks that test a fixed number use these:
- F.12(a): integrator poles p_I,h = p_I,z = −0.4 (with the F.11 poles).
- F.13(c): each observer pole pair 10× faster than its controller pair, ζ = 0.707.
- F.14(b): disturbance-observer poles p_d,h = −1, p_d,z = −1, p_d,θ = −10 (observers as in F.13).
- F.10: k_I,h = 0.01, k_I,z = −0.0005. These are inside the P.6 critical gains but outside P.6's "barely moves the PD poles" guideline (≤ 0.0037 and ≤ 0.00026 in magnitude); smaller gains leave a 1.6 m altitude error at 80 s under the α = 0.2 mismatch.
- F.18: the reference loopshapes (see the Ch 18 solutions).

Notes on the derived results (not errors):
- F.4: only the altitude channel is feedback-linearizable, with F = (F_e + F̃)/cos θ. The lateral channel is not, because θ is a state rather than an input.
- F.8(c): k_DC,θ = 1 (double-integrator inner plant). Compare B.8, where it is 1.89.
- F.8(d): k_P,z = −0.00771 and k_D,z = −0.0328. They are negative because Z/Θ = −g/(s(s + μ/M)).
- F.10: with α = 0.2, the soft altitude loop from F.8 (t_r = 8 s) and the fixed mismatch draw (m_c +14%) let the vehicle sag about 9 m before k_I,h recovers. The book's dynamics have no ground, so h goes negative. A tight anti-windup gate (v̄ = 0.1 m/s) keeps the integrator off during the sag and leaves a large error at 60 s. The reference uses v̄ = 0.5 m/s and t_end = 80 s.

## Repo issues

| File | Issue | Effect |
| --- | --- | --- |
| `VTOLParam.py` on branches `origin/2024Fall`, `origin/2024Winter`, `origin/add_LQG`, `origin/add_lqr` | `unmixing = np.array([[1.0, 1.0], [-d, d]])`, commented as "(fl, fr)". The `testCases.py` on the same branches feeds u = (f_r, f_ℓ): case 3, u = (100, −100), increases θ̇, i.e. τ = d(f_r − f_ℓ). | With that param file's `mixing` and (f_r, f_ℓ)-ordered dynamics, the torque sign is flipped. The current branch (and 2025Fall, 2025Winter, master, the fork) has the correct `[[1, 1], [d, −d]]` with (f_r, f_ℓ). |
| `_F_planar_vtol/python/VTOLParam.py` (current) | Template with blank values, but `unmixing = np.array([[1.0, 1.0], [d, -d]])` is evaluated at import. | The file is a syntax error until filled in, which is intended for a template. Older branches (2021_fall, 2022_winter) name the limit `fmax`; current ones use `F_max`. |
| `_F_planar_vtol/python/testDynamics.py` | Requires \|error\| < 1e-14 with g = 9.81 (study A uses 9.8) and imports `from VTOLDynamics import Dynamics`, a class name unlike A's `armDynamics`. ḣ has no drag term. | Students must match the expression closely. The workbench's `f` matches exactly. |

## How the workbench handles them

| Item | Workbench behavior |
| --- | --- |
| Mixing | `system.mix` = [[1, 1], [d, −d]]⁻¹ with the controller's nominal d. Input disturbances d_F and d_τ are mixed with the true d and then saturated together with u, as the repo's `dynamics.update(u + d)` does. |
| Reference offsets | F.8(e)'s 3 ± 2.5 m needs an offset the shared reference panel cannot set. Each closed-loop tab has a "Reference offsets" section. |
| F.8(f) | The checker simulates (t_r,h, t_r,z) and 10% faster in each loop. The solution gives both one-loop limits and a trade-off point. |
| F.9 / F.16(a) | Checks use the book's unity-feedback convention. The readouts show both conventions and the simulated error. |
| F.11 | K is checked against the inequality spec, and k_r against the entered K. |
| F.13 / F.14 | Block-structured lateral observer. The disturbance observer estimates d_F (altitude input), d_z (lateral force), and d_τ (input torque, cancelled in τ). |
| F.16 / F.17 | "My F.10 gains" (from the Ch 10 tab) or the reference design. Every check is computed from the selected gains. |
| F.18 | The outer loop is shaped on P_out·T_in with the current inner design. The reference designs meet every spec (PM 59.6° / 59.1° / 60.1°). |
