# Design Study D: issues found in the book and repo

As of 2026-10-03. This covers Design Study D (the mass-spring-damper): problems D.2–D.18 and D.P.6, controlbook.pdf pp. 377–383.

D has no worked solutions in the book, so there are no "wrong printed answers" to report. Most items are **ambiguous problem statements** that change a numerical answer, plus a few repo inconsistencies. Every number below was derived from m = 5 kg, k = 3 N/m, b = 0.5 N·s/m and F_max = 6 N. `tools/regress_D.py` cross-checks it against python-control.

Page numbers are PDF pages (book page + 8).

## Ambiguous or underspecified problem statements

| Problem | PDF page | Issue | Effect / workbench choice |
| --- | --- | --- | --- |
| D.7–D.8 (PD architecture) | 379 | Fig. 7-2 PD on this type-0 plant gives Z/Z_r = (k_P/m)/(s² + …), so a 1 m step settles at k_P/(k + k_P): 0.6 m with the D.7 gains and 0.504 m with D.8(a). D.10 then speaks of "the steady state error caused by the uncertain parameters," which only makes sense if the controller also supplies the equilibrium force F_e = k z_e with z_e = z_r. | The workbench offers a "spring compensation" toggle: `F = k z_r + F̃` or none (Fig. 7-2 exactly). It moves no poles. The default is none on D.7–D.P.6 and `F = k z_r + F̃` on D.10. The D.7(b) answer is the Fig. 7-2 transfer function, as asked. |
| D.8(a) "verify the step response" | 379 | Without F_e the response never reaches 90% of the 1 m step, so a rise time against the commanded step does not exist. | The verification check measures the 10–90% rise time against the final value: 1.93 s, with 4.6% overshoot, either way (2.2/ω_n is exact only near ζ = 0.707). The page's step-response panel still measures against the commanded step. |
| D.8(b) "just saturates" | 379 | The answer depends on the architecture. Without F_e: F(0⁺) = k_P·1 m ≤ 6 gives k_P = 6, ω_n = 1.342, t_r = 1.640 s (k_D = 8.89). With F_e = k z_r: F(0⁺) = 3 + k_P ≤ 6 gives k_P = 3, ω_n = 1.095, t_r = 2.008 s (k_D = 7.17). The D.8(a) design (k_P = 3.05) then demands 6.05 N, so it saturates very slightly. | The check simulates a 1 m step with the current compensation setting. It accepts a peak demand of 95–100% of F_max. The solution shows both cases. |
| D.P.6, D.10, D.11, D.13 "designed in D.8" | 380–381 | These don't say whether to use the D.8(a) gains (t_r = 2 s) or the D.8(b) gains. | The workbench uses the D.8(a) specs (t_r = 2 s, ζ = 0.7), which the problem gives, so no D.8(b) answer leaks into later tabs. The Explore knobs accept any t_r. |
| D.10(c) k_I | 380 | No value or criterion is given. | Workbench choice: the error before the first switch must be under 2% of the step, with a fixed α = 0.2 mismatch draw (Δm = +15%, Δk = −18%, Δb = +10%). With the D.8(a) gains, k_I = 0.5 gives 6.0 mm and k_I = 0.75 gives about 1.5 mm, against a 10 mm limit. D.P.6's own criterion (pair within 10% in \|p\|, also a workbench choice) allows k_I ≲ 0.8, so any k_I in 0.5–0.8 passes both checks. The reference solution uses k_I = 0.75. |
| D.12, D.13, D.14 poles | 381–382 | The integrator pole, the observer poles and the disturbance-observer pole are left to the student. | The checkers take your chosen poles as inputs and verify your gains against them. The reference solution uses p_I = −1, observer ω_n = 11 rad/s (10× the controller), ζ = 0.707, and p_d = −5. |
| D.12(a) anti-windup | 381 | It is asked for, but there is no method or threshold. | The workbench holds the integrator while the actuator is saturated, as in Study A. |
| D.13(e) | 382 | "Observe … steady state error" | With a 10×-faster observer, the bias is small: z − ẑ = −C(A − LC)⁻¹B·d = b₀d/β₀ = 0.2·0.25/121 = 0.41 mm. Use a slower observer to see it clearly. The solution shows the formula for the current L. |
| D.14(b) | 382 | "Verify that the steady state error in the estimator has been removed." | With the reference observer, the bias without the disturbance observer is only about 0.5 mm, against 1 mm measurement noise. The check averages z − ẑ over the 10 s before the first switch and requires < 0.2 mm (0.02 mm with it on, 0.51 mm with it off). |
| D.15 | 382 | "Force F̃ to position z̃" | The plant is already linear, so F̃ = F and z̃ = z for every equilibrium. |
| D.16(b) | 383 | The book's rule γ_din = 1/\|C(jω_din)\| (Eq. 16.8–16.9, p. 290–291) assumes \|PC\| ≫ 1 at ω_din. Here \|PC(j0.1)\| ≈ 3.3 (10.4 dB) with the D.10 gains. | 1/\|C(j0.1)\| = 10.2% vs. the exact \|P/(1+PC)\|(j0.1) = 9.1% (k_P = 3.05, k_D = 7.2, k_I = 1, σ = 0.05). The checker accepts either. |
| D.17 bandwidth | 383 | With D.8-style gains (k_D ≫ k_I), the PID zeros are complex (−0.21 ± 0.31j for k_I = 1) and make a notch in \|C\| near √(k_I/k_D) ≈ 0.37 rad/s. \|T\| dips to −4.8 dB around 0.3 rad/s, then recovers. | The "−3 dB bandwidth" is ambiguous: the first crossing is 0.190 rad/s (what `control.bandwidth` returns), while the final roll-off is 2.13 rad/s, just above ω_co = 1.78 rad/s. The checker accepts either; the solution explains the notch. |
| D.17 margins with other gains | 383 | — | The lightly damped resonance can push \|PC\| back above 0 dB. With k_P = 2, k_I = 0.5, k_D = 4 (the tab's placeholder gains), \|PC\| crosses 0 dB three times: PM 133° at 0.18, −171° at 0.43, and 70.7° at 1.28 rad/s. The workbench lists every crossover and reports the one with the smallest \|PM\|, as python-control's `margin` does (checked in `regress_D.py`). |
| D.18 | 383 | Saturation is not mentioned, but F_max = 6 N from D.8 is still in the simulation. The constant-input-disturbance size is not given. | The reference design (PI at 0.5, lead ω = 5 rad/s with M = 30, LPF 50, crossover 5 rad/s, prefilter p = 1) meets every loop-gain spec: PM 59.1°, GM 23.3 dB at 35.6 rad/s, min\|PC\| on ω ≤ 0.1 = 31.8 dB (needs 30.5), max\|PC\| on ω ≥ 500 = −85.5 dB. It still demands about 20 N for an instant on 1 m reference jumps. A prefilter with p ≈ 0.3 keeps the demand near 6 N, at the cost of an 8 s rise time. The workbench uses d = 0.25 N, as in D.12–D.14. |
| D.18 notation | 383 | ω_n means the noise frequency here but the natural frequency everywhere else (including D.11(a)). | Labels say "noise above 500 rad/s." |
| D.9(b) | 380 | "Inaccurate knowledge of the spring constant" as an input disturbance only fits if the controller uses k̂ (F_e = k̂ z_r or F = k̂ z + F̃). It is then (k − k̂)z, which is constant only once z is. | The solution states this. The tab defaults to "none" so the Fig. 7-2 analysis matches the simulation. |
| D.9 sign convention | 143 | Fig. 9-5 subtracts d_in at the plant input, so the book writes E = … + P/(1+PC)·D_in. | The workbench (like `<sys>Dynamics.update(u + d)`) adds d, which flips the sign. The card shows the book's form with a note, and the D.9(b) checker accepts ±1/(k + k_P). |
| D.9 with spring compensation | 380 | — | With F = k z_r + F̃ and exact k, the feedforward cancels a₀ in 1 − T and the tracking type goes up by one. PD then gives e_step = 0 and e_ramp = (a₁ + b₀k_D)/(a₀ + b₀k_P); PID gives e_ramp = 0 and e_parab = (a₁ + b₀k_D)/(b₀k_I). The type readout follows the toggle. The D.9 answers are for the Fig. 7-2 loop the problem asks about. |
| Fig. 7-1 vs 7-2 for D.9 | 380 | — | For this plant both architectures give the same error constants (P(0) is finite), so the type answers don't depend on the choice. Worth noting because they differ for the arm. |

## Book vs. repo code

| Topic | Book | Repo (`_D_mass/python`) |
| --- | --- | --- |
| Lead filter | Eq. 18.2 (p. 328): C_lead = M(s + ω/√M)/(s + ω√M). DC gain 1, high-frequency gain M. | `loopshape_tools.lead(w, M)` returns tf([√M, w], [1, w√M]) = √M(s + w/√M)/(s + w√M). DC gain 1/√M, high-frequency gain √M: a factor of √M below the book. `_A_arm`'s `get_control_lead` matches the book. |
| `loopshape_tools.py` | — | It is an older version than `_A_arm/python/loopshape_tools.py`: different function names (`lead` vs `get_control_lead`, …) and spec plots drawn as lines instead of points. Code from A will not run against it unchanged. |
| Parameters | m = 5 kg, k = 3 N/m, b = 0.5 N·s/m (p. 377). F_max = 6 N (p. 379). | `massParam.py` leaves them blank (template). The unit comments say "Kg/s^2" and "Kg/s," which equal N/m and N·s/m. `testDynamics.py` encodes m = 5, k = 3, b = 0.5: every vector satisfies z̈ = (F − 0.5ż − 3z)/5. |
| Animation | Fig. 19-1 shows a spring and a damper. | `massAnimation.py` draws the weight and spring only (no damper). The workbench animation draws both. |

## Repo bugs

None found in the D templates beyond the lead-filter scaling above. `testDynamics.py` compares at 1e-14 absolute. With `f = [ż, (F − b ż − k z)/m]`, all ten vectors match the JS model exactly (0.0 error).

## How the workbench handles them

| Item | Workbench behavior |
| --- | --- |
| Hidden answers | In Work mode, D-specific results are only in `symbolic`/`numbers` (Reveal), Check and Show solution. The s-plane hides the open-loop poles on D.5–D.7 until revealed, and D.8's ω_n circle and saturation region appear only in Explore. The D.15 Bode plot is behind a Reveal button. D.16–D.17 start from placeholder PID gains, with a "Load my D.10 gains" button. In Work mode, D.4 applies your own F_e or c, and D.P.6 uses your own PD gains. Notes that hint at answers appear only in Explore. |
| PD architecture | Spring-compensation toggle on D.7–D.10 (see above). The D.8(b) checker follows it. |
| Design choices | D.12–D.14 checkers take your poles as inputs. |
| D.16(b), D.17 | Accept both the book's approximation and the exact value / either bandwidth definition. |
