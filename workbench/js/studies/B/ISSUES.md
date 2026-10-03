# Design Study B: issues found in the book and repo

As of 2026-10-03. This covers Design Study B (the pendulum on a cart): problems B.2–B.6, B.8–B.18 and B.P.6.

I found these while building the workbench:
- 21 problems in the book's worked solutions;
- 6 places where the book and `_B_pendulum/python` disagree;
- 4 bugs in the repo code.

Three of the book items change a numerical answer:
- The B.11 and B.12 gains (and every matrix printed with them) are for ℓ = 0.5 m, not the stated ℓ = 1 m.
- Listing 8.3's outer-loop k_Dz formula is not the book's Eq. 8.13, so its outer loop has ζ ≈ 1.29 instead of 0.707.
- The Bode constants in B.15 (0.0659 and 0.174) are wrong.

**How the items were found.** I read the solution pages in `book_and_notes/controlbook.pdf` and re-ran the numbers. The workbench JS matches the repo's Python to 1e-10 or better (`workbench/tools/regress_B.py`). I recomputed the frequency-domain values from the book's parameters: m₁ = 0.25 kg, m₂ = 1 kg, ℓ = 1 m, b = 0.05 N·s/m, g = 9.8 m/s².

Page numbers are PDF pages (book page + 8).

## Errors in the book's worked solutions

| Problem | PDF page | Issue | Correct value / effect |
| --- | --- | --- | --- |
| B.6 | 90 | In the nonlinear derivation, the right-hand vector's second entry is printed m₁gℓ sin θ. | It is m₁g(ℓ/2) sin θ. The expanded line below it uses ℓ/2, so only the intermediate step is wrong. |
| B.8 (Listing 8.3) | 128–129 | The listing computes the outer gains as a = ω²√(2ℓ/3g) − 2ζω, k_Dz = a/(a + √(3g/2ℓ)). That is not Eq. 8.12–8.13 (p. 127). | The outer loop gets the right ωₙ but ζ = 1.29 for the listing's tuning (ζ_z = 0.707 asked). With Eq. 8.13 it is exactly 0.707. For t_r,θ = 0.15, M = 15: the listing gives k_Pz = −0.1447, k_Dz = −0.4196; Eq. 8.12–8.13 give −0.1749, −0.2986. `ctrlPID.py` (B.10) uses the correct formula. |
| B.8 | 124–129 | Part (b) asks for t_r,θ = 0.5 s, and the solution's numbers use it (k_Pθ = −26.0, k_Dθ = −4.41, k_DC = 1.89). The listing uses t_r,θ = 0.15 s. Part (d) never gives outer-loop specs, and the listing picks M = 15, ζ_z = 0.707. | The workbench checks (b)–(c) against t_r,θ = 0.5 s and uses M = 10, ζ_z = 0.707 for (d), stated on the page. |
| B.9 | 150 | The disturbance limit writes the outer plant as (2ℓs²/3 − g)/s², dropping the minus sign of Fig. 9-10. The outer analysis also drops k_DC and the zero-canceling filter. | The magnitude 1/k_P is unaffected. With k_DC and the filter, M_a = −q k_Pz instead of g k_Pz. The type is the same. |
| B.10 | 163–165 | The problem says to implement "the nested PID loops designed in B.8." The listing changes t_r,θ from 0.5 to 0.2 s and fixes M = 10. | The B.10 gains are k_Pθ = −97.96, k_Dθ = −11.02, k_DC = 1.143, k_Pz = −0.2121, k_Dz = −0.3280. |
| B.11 | 189–190 | Eq. 11.39 prints A₄₂ = 34.5882, A₄₃ = 0.1412, B₄ = −2.8235, with det 𝒞 = −6104.1 and K = (−1.5050, −24.9399, −1.9847, −3.5829). | These are exactly the ℓ = 0.5 m values. With ℓ = 1 m: A₄₂ = 17.294, A₄₃ = 0.0706, B₄ = −1.4118, det 𝒞 = −381.5, K = (−3.0101, −38.633, −3.9193, −8.4555), k_r = −3.0101. |
| B.11 | 189 vs 191 | Part (a) and the printed Δ^d use the B.8 poles (ζ = 0.707, ω_nθ = 4.4) with t_z = 3t_θ. Listing 11.2 uses ζ = 0.9, t_z = 5t_θ and ω_n = π/(2t_r√(1−ζ²)). | The workbench checks against part (a); a preset loads the listing's tuning. |
| B.12 | 207–208 | The same ℓ = 0.5 m matrices. The printed K₁ = (−5.3744, −32.1057, −4.5745, −5.1545, 3.0101) is the ℓ = 0.5 m answer. | With ℓ = 1 m: K = (−10.749, −55.544, −9.3037, −13.462), k_I = 6.0202. |
| B.12 | 207–208 | Step 2 says the B.11 poles are −1.4140 ± j1.4144 and −0.8 ± j0.6, with p_I = −10. The polynomial it then expands uses the B.8 poles (−1.0369 ± j1.0372, −3.1108 ± j3.1117) and (s + 2). | The listing uses p_I = −2. The workbench uses −2. |
| B.12 | 207–208 | 𝒞_{A1,B1} starts with "00.94118". a_A1 = (0.0471, −34.588, 1.3835, 0, 0) has +1.3835, while Δ_ol two lines above has −1.3835s². | Typos. |
| B.13 | 234 | The prose says the observer is updated on line 25, with RK4 on lines 40–48 and the observer equations on lines 50–56. | In Listing 13.2 these are around lines 105, 121–128 and 130–135. |
| B.15 | 277 | Eq. 15.17 gives the inner Bode constant as 0.0659, while the phase expression next to it uses 0.0816. | 1/((m₁+m₂)g) = 0.0816 (−21.8 dB). |
| B.15 | 278 | P_out's Bode form is written 0.174(jω/3.83 + 1)(−jω/3.83 + 1)/(jω)². 0.174 is (2/3)/3.83. | The constant is g = 9.8 (19.8 dB). Fig. 15-14 agrees with 9.8: 59.8 dB at ω = 0.1. |
| B.16 | 297–298 | The answers read 6.5 dB → 47% and −32.2 dB → 2.45% from Fig. 16-10, but the problem says to use the B.10 gains. | With the B.10 gains (hw16.py): B_r = 17.67 dB → 13.1%, |PC(j200)| = −39.0 dB → 1.12%. 6.5 dB is close to the t_r,θ = 0.5 s (B.8) gains (|L(j1)| = 6.23 dB), but the figure's −32.2 dB, read at ω = 100 rad/s although ω_no = 200, matches neither gain set. |
| B.16(c) | 297 | B_r = 154 dB, "|e(t)| ≤ 100e−08". | 100e−08 is 1·10⁻⁶. With the B.10 gains, B_r = 173.8 dB, γ_r = 2.04·10⁻⁹ and |e| ≤ 1.02·10⁻⁷. |
| B.17 | 316–317 | Fig. 17-12 reports GM −6.62, PM 32.88° with crossover ≈ 10 rad/s and bandwidth ≈ 19 rad/s. | With the B.10 gains (hw17.py): PM = 29.7° at 17.1 rad/s; the phase is −180° as ω → 0, so GM = 1/|L(0)| = −18.1 dB; bandwidth 27.59 rad/s (3 dB below |T(0)| = k_DC, as control.bandwidth defines it). The book's figure is close to the t_r,θ = 0.5 s gains but does not match them exactly either (PM 37.2° at 6.66 rad/s, GM −6.5 dB with σ = 0.05). |
| B.17(b) | 317–318 | The outer-loop figure shows a crossover near 1 rad/s with PM 72°. | With the B.10 gains, hw17's P_out·C_out never drops below +4.6 dB: P_out → −2ℓ/3 and C_out → (k_Dz + σk_Pz)/σ at high frequency, so |L| → 4.5. There is no crossover (python-control reports infinite margins). The loop the code actually closes, C_out · filter · inner closed loop · P_out, crosses over at 1.4 rad/s with PM 29.6° (gain margins −12.3 dB at 0.45 rad/s and +7.0 dB at 3.7 rad/s), bandwidth 4.136 rad/s, and a bandwidth separation of 6.67× from the inner loop (27.59/4.136). |
| B.18(a) | 349–350 | The text designs C_lead = −155.12 (s/10.72 + 1)/(s/120 + 1) and calls it a lead "centered at 40 rad/s with a ratio of 13.9." | A ratio-13.9 lead at 40 rad/s has its pole at 40·√13.9 = 149 rad/s, not 120. Listing 18.5 (and the repo) use −800·lead(ω = 40, M = 15) instead. |
| B.18(b) | 354–355 | The text's C_out = 0.469 (s + 0.194)/(s + 6.24) · (s + 0.0256)/(s + 0.0032) · 100/(s + 100) (lead at 1.1 rad/s, ratio 32, lag ratio 8, LPF 100). | Listing 18.6 and the repo use 0.1·lead(1, 20)·lag(0.04, 10)·lpf(50). Both designs are presets on the Ch 18 page. |
| B.18 text designs | 349–355 | The text's own designs do not meet the problem's specs. | Inner C_lead: PM 53.4° at 21.6 rad/s (asked: ≈ 60°, crossover ≈ 40 rad/s). Outer: |PC(j0.0032)| ≈ 98.7 dB, short of the 100 dB that γ_r = 10⁻⁵ needs. |
| B.P.6 | 471–472 | Fig. 6-9 closes the outer PID loop with k_DC but without the zero-canceling filter that B.8 adds. | With the B.8 or B.10 PD gains, that loop is already unstable at k_Iz = 0 (its cubic has a sign change; roots +2.15, −0.26, −7.48 for the B.8 gains). With the filter (as implemented), the locus starts stable and stays stable up to k_Iz = −0.235 (B.10 gains). The book stops at the rlocus command. |

Smaller notation issues:
- B.16(c) calls the outer reference y_r(t); it is z_r.
- B.17's Listing 17.2 sets `dB_flag` twice.
- The B.P.6 figure (Fig. 6-9) shares its number with the Chapter 6 figures.

## Book vs. repo code

| Topic | Book | Repo (`_B_pendulum/python`) |
| --- | --- | --- |
| B.8 tuning | Problem: t_r,θ = 0.5 s; outer specs open | `ctrlPD.py`: t_r,θ = 0.15 s, M = 15, ζ_z = 0.707, and the Listing 8.3 k_Dz formula |
| B.11–B.12 matrices and gains | Printed for ℓ = 0.5 m | `ctrlStateFeedback*.py` use `pendulumParam.ell = 1.0` (correct for the stated problem) |
| B.13–B.14 observer gains | Not printed | Two outputs make L non-unique. `control.place` returns the one from scipy's YT iteration; the workbench ports that iteration (`place.js`) and lands on the same L to 2e-11. |
| B.14 observer design | Text: observer 10× faster | `ctrlDisturbanceObserver.py` switches the observer to ω_n = 2.2/t_r (the controller keeps π/(2t_r√(1−ζ²))), and its disturbance pole at −1 is slower than every controller pole. |
| B.16–B.17 C_in | hw16.py: D on the error, ((k_D + σk_P)s + k_P)/(σs + 1) | `ctrlPID.py` differentiates θ. The loop gain is the same, but the inner closed loop θ_r → θ has numerator P k_Pθ, not P C_in. The Ch 17 "as implemented" outer loop uses the latter. |
| B.18 implementation | State-space form (Ch 18 text) | `hw18_pendulumSim.py` uses `method="digital_filter"` (Tustin) and α = 0.1. The workbench offers both; python-control's c2d gives Tustin coefficients that differ from direct bilinear substitution by 2e-13. |

## Bugs in the repo code

| File | Bug | Status |
| --- | --- | --- |
| `_B_pendulum/python/ctrlPD.py:46–48` | Outer k_Dz from the wrong expression (same as Listing 8.3), so ζ_z is not met (1.29 instead of 0.707) | Open |
| `_B_pendulum/python/ctrlLoopshape.py` `transferFunction` | For a strictly proper C with relative degree ≥ 2, `C[0, i] = num.item(i)` for i in range(n−m−1, n−1) indexes past the end of `num` (IndexError) or misplaces coefficients. It should be `num.item(i − (n−m−1))`. The repo's designs all have relative degree ≤ 1, so they are not affected. | Open |
| `_B_pendulum/python/pendulumDynamicsSympy.py` | `dill.load(open("eom_case_study_B", "rb"))` loads, but the first call to the loaded function segfaults under the repo's Python 3.14 `.venv`: the committed pickle of the lambdified EOM is from an older Python. | Open (regenerate with `hw03_pendulum_solving_for_state_variable_form.py`) |
| `_B_pendulum/python/hw13_pendulumSim.py` | The 0.05 N disturbance is always on (part e), although part (a) asks for none. | Open |

## How the workbench handles them

| Item | Workbench behavior |
| --- | --- |
| B.8 outer formula | Explore mode has a toggle between Eq. 8.12–8.13 and Listing 8.3, and a "Listing 8.3 tuning" preset. The math card notes the larger ζ. Problem checks use Eq. 8.12–8.13. |
| B.11–B.12 ℓ = 0.5 m numbers | Checks use the current parameters (ℓ = 1 m by default). Solutions quote the book's numbers and say why they differ. The book's printed K still balances the ℓ = 1 m pendulum (try it in Work mode), but it does not place the B.11(a) poles. |
| B.11/B.12 vs listings | Presets for the book text (B.8 poles, t_z = 3t_θ, 2.2/t_r) and for the listings (ζ = 0.9, π/(2t_r√(1−ζ²))). |
| B.13–B.14 observer gains | Designed in both modes from pole knobs. Problem parts ask for the observer pole frequencies, not the non-unique L. |
| B.15 Bode constants | The math cards and solutions use 0.0816 and 9.8 and flag the book's values. |
| B.16–B.17 numbers | Every readout is computed from the current gains (B.10 by default, button to restore). Solutions quote the book's values. |
| B.17 outer loop | A toggle between hw17's P_out·C_out and the loop the code closes. The default is the latter, since the former has no crossover with the B.10 gains. |
| B.P.6 model | A toggle between Fig. 6-9 (k_DC only) and the filtered loop. Checks use the filtered loop; the locus is drawn for k_Iz = −κ, since k_Pz and k_Dz are negative. |
| B.18 designs | Presets for the repo design and the text design, plus a stable first try. Implementation toggle: state space (RK4) or Tustin. |
