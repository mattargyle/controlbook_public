# Design Study E: issues found in the book and repo

As of 2026-10-03. This covers Design Study E (block on beam): problems E.2–E.6, E.8–E.18 and E.P.6 (pp. 385–392). The book has no worked solutions for E, and the repo has only templates (`_E_blockbeam/python`) plus `testDynamics.py`. So this list is mostly about ambiguous problem statements, design specs that turn out to be fragile, and the choices the workbench makes.

Page numbers are PDF pages (book page + 8). The numbers below use the book's parameters: m₁ = 0.35 kg, m₂ = 2 kg, ℓ = 0.5 m, g = 9.8 m/s², F_max = 15 N, z_e = ℓ/2.

**How the numbers were checked.** `tools/regress_E.py` checks three things:
- the JS f(x, u) against the `testDynamics.py` vectors;
- every design against independent Python (closed-form formulas, python-control `place` / `margin` / `bandwidth`, numpy eigenvalues);
- four closed-loop simulations against a Python re-implementation, sample by sample. These agree to about 1e-13.

## Book vs. repo

| Topic | Book | Repo | Workbench |
| --- | --- | --- | --- |
| Gravity | g = 9.8 m/s² (p. 385) | `testDynamics.py` expects g = 9.81. Its first vector gives θ̈ = −29.43 = −3g/(2ℓ) with g = 9.81. `blockbeamParam.py` leaves g blank. | Uses 9.8 everywhere. `regress_E.py` passes g = 9.81 only for the test vectors. The JS f then reproduces all ten vectors within the script's 1e-14 tolerance (max error 1.8e-15). |
| Order of operations in f | n/a | The 1e-14 tolerance is tight enough that the algebraic form matters. For example, computing z̈ as zθ̇² − g sin θ fails test 3 by 2.8e-14. | `system.js` writes f in the repo-solution order, `(1/m1)(m1 z θ̇² − m1 g sin θ)`, which passes. |
| Problem numbering | E.6 is followed by E.8, and the list on p. 385 also has no E.7 | n/a | No E.7 tab. |

## Ambiguous or underspecified problem statements

| Problem | Page | Issue | Workbench choice |
| --- | --- | --- | --- |
| E.5 → E.15, E.18 | 387, 390, 392 | E.18(a) says "P_in(s) … derived in HW E.5", and E.15(a) asks for "the inner loop transfer function from F̃ to θ̃". E.5 derives two models: the full (ℓ/J_e)s²/(s⁴ − m₁g²/J_e) in part (b), and the simplified b₀/s² in part (c). | Uses the simplified b₀/s². It is the model E.8 designs with, and F_fl(z) cancels exactly the term (c) drops. E.15 can overlay the full model. |
| E.8(f) | 388 | "a step of size 0.25 meter is placed on z̃^r" does not say where the block starts. | The block starts at rest at z_e and z_r goes to z_e + 0.25 = 0.5 m, the beam tip. That gives t_rz ≈ 1.173 s. Starting at z = 0 and stepping to 0.25 gives t_rz ≈ 1.040 s instead. |
| E.8(f) | 388 | "Using the rise time of the outer loop" does not say whether t_rθ stays at 1 s or keeps t_rθ = t_rz/10. | Keeps M = 10, so t_rθ = t_rz/10. With t_rθ fixed at 1 s, the outer loop cannot be pushed far before the separation is lost. |
| E.8(f) | 388 | The Eq. 8.8 bound (p. 119) does not decide this one. Right after the step, F̃(0⁺) = k_Pθ k_Pz z̃_r is negative (the beam tilts down first). On that side the room is F_fl + F_max ≈ 26.5 N. The binding limit comes in the braking phase near the beam tip, where F_fl(0.5) = 13.2 N leaves only 1.8 N. | The check and solution use a simulated peak with bisection. JS and Python agree to 1e-6 s. |
| E.9 | 388 | System type is asked for "PD control". The book computes it from the unity-feedback loop gain P·(k_P + k_D s) (pp. 145–150). The E.8 controllers differentiate the output, which keeps the type but can change the error constants (note on p. 145). | Follows the book's convention. The solution text says so. |
| E.10(c) | 389 | "the integrator gain will need to be negative which will cause problems for the anti-windup scheme" does not say which scheme. Back-calculation, u_I += (u_sat − u_unsat)/k_I, has the right sign for k_I < 0. The real obstacle is that saturation happens at F, inside the inner loop, so the outer integrator never sees it. | Implements the requested gate (integrate only while \|ż̂\| < v̄) and explains the inner-loop saturation point in the solution. |
| E.11 | 389 | It says to start from the E.10 files, which have α = 0.2, but E.11 has no integrator, so any mismatch leaves a steady-state error. Part (a) ("natural frequency greater than ω_nz and damping ratio greater than ζ_z") has no unique answer. | E.11 defaults to the exact model. Part (a) accepts any pole set that meets the rule. Part (d) is checked against `place` for the poles you entered in (a). |
| E.12(b) | 389 | The disturbance size is given ("1 Newtons"); the plant-variation draw is not. | The pages use one fixed α = 0.2 draw (m₁ +12%, m₂ −9%, ℓ +15%), so results are repeatable. "Randomize ±α" is available. |
| E.13(e) | 390 | It says a 0.5 N disturbance causes a steady-state error "even though there is an integrator". That is true only if the observer bias reaches ẑ. The bias is e_ss = −(A − LC)⁻¹Bd. d enters only θ̈, so an L that decouples A − LC into a z block and a θ block keeps ẑ unbiased, and z still tracks (θ̂ is biased). python's `place(Aᵀ, Cᵀ)` gives a coupled L, and there the book's claim holds. | Offers both a decoupled L and a z-only L. The (e) check reports z_r − z, z − ẑ and θ − θ̂, and the solution explains the difference. |
| E.14(a) | 390 | The noise is "standard deviation of 0.001" with no units. | Uses 0.001 m on z and 0.001 rad = 0.0573° on θ. |
| E.16(b) | 391 | For PD, \|C_in(jω)\| grows with ω, so the worst input-disturbance attenuation below ω_din is at low frequency: 1/k_Pθ = 54.8%. Reading the plot at ω_din = 0.8 (as A.16 does for a PID) gives 48.0%. | Accepts 1/k_Pθ. If you enter the band-edge value, it explains the difference. |
| E.16(d, e) | 391 | These use the book's \|e\| ≈ \|r\|/\|L\|, which needs \|L\| ≫ 1. At ω_dout = 0.1, \|L_out\| is only about 14 dB (1/\|L\| = 18.9% vs. exact \|S\| = 22.7%). At 0.6 rad/s (part e), \|L_out\| < 1, so A/\|L\| = 3.7 m is meaningless. The exact error is A/\|1 + L\| = 2.01 m: the output barely moves. | (d) accepts either value. (e) accepts only the exact A\|S\| and explains why. These numbers use the defaults: E.8 specs with k_Iz = −10⁻⁴. |
| E.16 / E.17 gains | 391 | E.16 asks for the inner gains from E.8 and the outer gains from E.10. E.17 asks for "the gains found in HW E.10" for both loops. As the next table shows, gains that pass E.10 are not the E.8 gains. | E.15–E.17 run from design knobs. The defaults are the E.8 specs plus k_Iz = −10⁻⁴ from E.P.6. Every check uses the current knobs. |
| E.17(b) | 392 | Three integrators put the outer loop's phase at −270° at low frequency, so the loop is conditionally stable (GM = −23.7 dB at 0.056 rad/s with the defaults). MATLAB's `margin` and python-control report different crossings (as in A.18). | All gain-margin crossings are listed. |

## Problems with the specified designs

| Topic | Page | Finding |
| --- | --- | --- |
| E.8 gains vs. E.10 uncertainty | 387–389 | With t_rθ = 1 s and M = 10, k_Pz = −0.0049 rad/m, so the outer loop can command at most about 0.0012 rad over the whole half-beam. An unmodeled force ΔF tilts the beam by ΔF/k_Pθ first. In the simulation, a 0.5% error in m₂ (0.05 N) makes the block run off the beam. Integrators don't help, outer or inner. E.10 asks for α = 0.2 with "the nested PID loops in Problems E.8". The loops have to be faster: t_rθ ≈ 0.4 s, M = 10, k_Iz ≈ −0.01 handles the default mismatch with under 1 mm of error. The steady-state force-to-position gain of the PD loop, −1/(k_Pθ k_Pz) ≈ 110 m/N, is shown in E.9. |
| E.8(e) with T_s = 0.01 | 388 | F_fl(z_k) is sampled and held for T_s while the gravity torque m₁gz changes continuously. The uncancelled force m₁g(z(t) − z_k)/ℓ averages to about (m₁gT_s/2ℓ)ż = 0.034ż N, against the designed outer damping term k_Pθ k_Dz ż = 0.058ż. The simulated overshoot is ~48% instead of the designed ~4% (20% at T_s = 0.005, 7% at T_s = 0.001). The linear model and python-control both give 4.3%. The workbench reproduces this in JS and Python and explains it in a math card. |
| E.8 with slow loops | 388 | During the E.8(f) bisection, a 0.25 m step from z_e diverges in the nonlinear simulation once t_rz ≳ 15 s (t_rθ ≳ 1.5 s, M = 10). The bisection therefore searches t_rz ∈ [0.1, 10] s. The mechanism was not investigated further; the slow loop leaves the beam tilted long enough for the cos θ / J(z) nonlinearities to matter. |
| E.P.6 | 388 | With the E.8 gains, k_I,crit = −g k_Dz k_Pz = −1.54×10⁻³. Only \|k_I\| ≲ 1.3×10⁻⁴ keeps the PD pair within 10%. The integrator pole is then at about −0.024 (a 40 s time constant), so the integrator is very slow. |

## How the workbench handles them

| Item | Workbench behavior |
| --- | --- |
| x_e, F_e, F_fl(z) | All three are kept distinct. PID chapters toggle between F_fl(z) (E.8e), the constant F_e at z_e, and none. State-space chapters use F = F_e + F̃ with the Jacobian A (as E.6 and E.11 ask), or F_fl(z) with A₄₁ = 0. Either way the model and the control law stay consistent. k_r differs between them: K₁ + m₁g/ℓ vs. K₁. |
| Reference | The reference is z̃_r on top of z_e = 0.25 m, so the E.8 square wave is z̃_r = ±0.15 m. The offset is fixed at the book's ℓ/2 even if ℓ is changed. |
| Two-output observer | The default L is decoupled: L₃₂ = a₃₂ and L₄₁ = a₄₁ cancel the cross-couplings, then each block is a second-order (or, for the disturbance observer, third-order) placement. It has the same eigenvalues as python's `place`, but a different, sparser matrix (two zero entries; largest entry 484 vs. 269 from `place` for E.13). A z-only Ackermann design is also offered. Answer checks for L test the eigenvalues, not the entries, because a two-output L is not unique. |
| SPlane legend | The s-plane names marker kinds with fixed strings. The successive-loop pages relabel them ("inner-loop pole (θ)", "outer-loop pole (z)") through a `kindNames` field, now supported by the shared s-plane plot. |
| E.18 sample design | Inner: C_in = 135 · lead(40 rad/s, M = 20) · 400/(s + 400). PM 59.1° at 39.8 rad/s, \|L(j1)\| = 51.2 dB, \|L(j1000)\| = −51.6 dB. Outer: C_out = −0.0735 (s + 0.2)/s · lead(2, M = 30) · 30/(s + 30). PM 59.7° at 2.0 rad/s, \|L(j0.1)\| = 44.5 dB, \|L(j100)\| = −71 dB. A prefilter 0.7/(s + 0.7) removes the 2 dB peak. It is loaded only from the problem panel's buttons, and python-control confirms every spec. |
