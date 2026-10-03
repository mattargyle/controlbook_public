# Design Study C: issues found in the book and repo

As of 2026-10-03. This covers Design Study C (satellite attitude control): problems C.2–C.6, C.8–C.18 and C.P.6.

I found these while building the workbench:
- 23 problems in the book's worked solutions (one of them, C.9/C.16's system type, also affects the repo);
- 6 places where the book and `_C_satellite/python` disagree;
- 9 bugs or robustness problems in the repo code.

Three items change what a student would see:
- The C.12 and C.14 simulations (`hw12`, `hw14`) can diverge.
- `ctrlLoopshape(method="state_space")` crashes.
- The C.18 outer-loop design uses a plant model whose DC gain is 10 instead of 1.

**How the items were found.** I read the solution pages in `book_and_notes/controlbook.pdf` and re-ran the numbers with python-control. `workbench/tools/regress_C.py` confirms that the workbench's JavaScript simulation matches the repo's Python closed loops to 1e-10 or better. The parameters are the book's (p. 15): Js = 5 kg·m², Jp = 1 kg·m², k = 0.1 N·m, b = 0.05 N·m·s, |τ| ≤ 5 N·m.

Page numbers are PDF pages (book page + 8).

## Errors in the book's worked solutions

| Problem | PDF page | Issue | Correct value / effect |
| --- | --- | --- | --- |
| C.8(d) | 132 | The solution prints (k_Pφ, k_Dφ) = (0.834, 8.253). | Its own matrix equation gives (0.8339, 8.2431). k_DCφ = 0.455 is right. |
| C.8(f) | 133 | The listing's t_rθ = 1.75 s is described as "tuned to not saturate the input." | A 30° step on φ_r then demands 6.85 N·m (137% of τ_max) at the first sample. The fastest non-saturating value with M = 10 is t_rθ ≈ 1.886 s. There, k_Pφ = −0.564, so θ_r(0) = (1 + k_Pφ)·30° and τ(0) = k_Pθ θ_r(0) = 5 N·m. |
| C.8 | 129–134 | The block-diagram analysis (Fig. 8-19) has no feedforward, but the listing adds φ_r to θ_r (Fig. 8-20). | Feedforward makes the overall DC gain 1. The C.9 type analysis assumes it is absent. |
| C.P.6 | 472–473 | The solution writes Φ = (b₂s² + b₁s + b₀)/(a₃s³ + …)Φ_r, then jumps straight to the Matlab command. The coefficients and the Evans form are never given. | 1 + k_I·k_DC(bs + k)/[(J_p + bk_DC k_D)s³ + (b + bk_DC k_P + kk_DC k_D)s² + (k + kk_DC k_P)s] = 0. This matches the Matlab line. |
| C.10 | 166–168 | The problem says to implement "the nested PID loops designed in C.8," but the listing does not use C.8's loops. | The listing uses t_rθ = 0.4 s with ω_n = 2.2/t_r and M = 15 (C.8: 1 s, π/(2t_r√(1−ζ²)), M = 10). It also drops C.8's φ_r feedforward. |
| C.10 | 167–168 | The back-calculation anti-windup in the listing differs from §10.1.1. | The listing uses u_I += (T_s/k_I)(θ_r − θ_r,unsat); §10.1.1 uses 1/k_I. Same issue as A.10. |
| C.11 | 193 | The printed A has −k/Js = −0.03, k/Jp = 0.15, and Δ_ol = s⁴ + 0.06s³ + 0.18s². | These correspond to k = 0.15. The book's k = 0.1 gives −0.02, 0.10 and 0.12s². |
| C.11 | 193 | det(𝒞_A,B) is printed as −36,000. | It is −3.6×10⁻⁵ for the printed A and −1.6×10⁻⁵ for the book's parameters. Either way it is nonzero, so the conclusion stands. |
| C.11 | 193–194 | Part (a) says to use the C.8 ω_n and ζ, but the solution uses other poles. | It uses ω_θ = 1.9848, ω_φ = 1.5 and ζ = 0.707; the listing uses t_rθ = 2 s, M = 3, ζ = 0.9. The printed K = (40.28, 255.17, 24.34, 366.18) and k_r = 295.46 are right for the printed (k = 0.15) A with those poles. With the true A they are K = (34.51, 408.67, 24.34, 487.58), k_r = 443.19. |
| C.11 | 194 | The text says k_r uses C_r = (0, 1, 0, 0); the listing uses C_r = [1, 0, 0, 0]. | Same value: at steady state the spring forces θ = φ. |
| C.12 | 211–212 | The printed A uses yet another set of numbers. | B₃ = 0.21 and row 4 = (0.14, −0.14, …). The step 2 poles (|p| = 1.5 and 0.99) match neither C.11's text (1.98, 1.5) nor its listing. p_I = −1 in the text, −2 in the listing. |
| C.12 | 212 | The printed values cannot be reproduced from the solution's own data. | Δ_ol = s⁵ + 0.0611s⁴ + 0.1657s³, the desired polynomial, and K₁ = (19.15, 43.41, 16.72, 111.63, −14.52) do not follow from its own A₁ and poles. Placing those poles on the printed A₁ gives (30.12, 196.76, 21.26, 287.40, −75.36). |
| C.12(a) | 211–214 | The problem asks for an integrator with anti-windup, but the listing has none. | `hw12` then winds up and diverges; see the repo table. The workbench offers "hold integrator while saturated." |
| C.9(a), C.16(a); Notes p. 153 | 151, 153, 299 | The inner-loop answers (type 2; parabola error (Js+Jp)/k_P; C.16's 2A/M_a) hold for PD on the error (Fig. 9-11). The book's own controllers (Listings 8.4, 10.4; ctrlPD.py, ctrlPID.py) differentiate θ instead. The Notes on p. 153 say moving the derivative does not change the type. | With the derivative on θ, E/R = ((Js+Jp)s² + k_Ds)/((Js+Jp)s² + k_Ds + k_P): type 1, ramp error k_D/k_P, and a parabola is not tracked at all. The plant has no damping, so the k_Ds term sits in the error numerator. The workbench accepts the book's answer, explains the difference, and also accepts type 1 with e_ramp = k_D/k_P. |
| C.13 | 234, 237 | The prose cites "line 25/26," "lines 40–48/41–49" and "lines 50–56/51–57" for the observer. | The code is at lines ~98, 113–121 and 123–128 of Listing 13.3. |
| C.16 | 298–301 | The problem says to use the C.10 gains, but the figures and numbers come from the C.8 loops. | The C.8 loops have k_Pθ = 77.9: B₂ = 22.3 dB, |C_in(j0.1)| = 38 dB, outer |PC(j10)| = −8.4 dB. With C.10's gains (k_Pθ = 181.5) these are 29.6 dB, 45.2 dB and −8.25 dB. Part (a)'s prose also says "B₂ = 20 dB," then uses 22.3. |
| C.16(a), Eq. 16.12 | 299 | It treats θ_r = 20t² as R(s) = 20/s³ and answers e = A/M_a = 1.53. | L{20t²} = 40/s³, so e = 2A/M_a = 3.08 for the C.8 loops (1.32 for C.10's). Same factor-2 error as A.16(b). |
| C.16(c) | 298 | The part says "using PI control," but the outer loop is C.10's PID. | — |
| C.17 | 321–322 | The problem says to use the C.10 gains, but the reported margins come from the C.8 loops. | Inner PM = 56.16° at 6.97 rad/s matches the C.8 loops exactly; C.10's gains give 48.2° at 10.4 rad/s. The outer PM 110.92° is near the 111.3° of either set. |
| C.18(a) | 362–363 | The text designs C_in = 45·8/(s + 8) on a rate-damped plant, but Listing 18.7 is a different design. | The text's plant is built from Θ/τ = (1/Js)/(s² + (b/Js)s + k/Js), which drops the panel coupling. Its rate gain is k_Dθ = 38.92, the C.8 loops' value, although the problem says C.10's (59.4): Figs. 18-29/18-30 (PM 84.93° at 1.15, 76.86° at 1.14 rad/s) reproduce exactly with 38.92. Listing 18.7 instead puts a lead at 0.41 rad/s with M = 15 on 1/(6s²), with no rate feedback. |
| C.18(b) | 366 | The derivation gives the right Φ/Θ_r′ numerator, but the plant is then written with the wrong one. | The derivation gives σbs² + (σk + b)s + k; P_out is written with σs + 1. |
| C.18(b) | 367–368 | Figs. 18-33/18-34 do not correspond to the printed listings. | They report PM = −36.4° before the gain, then 51.75° at 0.15 rad/s with GM 17.45 dB. They are reproduced closely (−36.47° and 51.78° at 0.149 rad/s, GM 17.43 dB) by the listing's outer model with the *book-text* inner loop and the C.8 k_Dφ = 8.243. The printed listings (lead inner loop, C.10 gains) give 55.5° at 0.149 rad/s with GM 14.1 dB, and the correctly derived plant gives about 83°. The workbench's *Book text* preset reproduces the figures. |
| C.18(b) | 367–368 | The lag is described inconsistently. | The text says "z = 2.0, M = 60," and the code sets z = 2.0 but calls get_control_lag(0.5, 60). The printed lag "(s + 0.5)/0.0083" is missing "s +" in the denominator. |

Smaller notation issues:
- C.6 measures y = (θ, φ − θ) (star tracker + strain gauge). Every later chapter and the repo use y = (θ, φ). Both are observable.
- Appendix P.6 numbers its figures 6-1 to 6-10. Its Figs. 6-1 to 6-5 collide with Figs. 6-1 to 6-5 of the Part III introduction (pp. 94–97), which are different figures.

## Book vs. repo code

| Topic | Book | Repo (`_C_satellite/python`) |
| --- | --- | --- |
| C.11–C.13 numbers | Printed for k = 0.15 and other poles | t_rθ = 2 s, M = 3, ζ = 0.9, π/(2t_r√(1−ζ²)), p_I = −2: K = [24.73, 33.82, 21.32, 183.39], k_r = 58.56; K₁ = [42.64, 425.34, 31.32, 680.04, −117.11] |
| C.13/C.14 observer gains | Not printed | L = place(Aᵀ, Cᵀ, q)ᵀ with two outputs is not unique. The repo's values are what scipy's YT algorithm converges to: Lᵀ = [[21.62, 5.30, 107.95, 1.31], [−5.21, 21.56, 0.69, 106.87]]. The workbench ports that algorithm (`system.js`, `placePoles`) and reproduces L and L₂ to 4×10⁻¹⁰. |
| C.16/C.17 gains | C.8 loops (see above) | `hw16.py` and `hw17.py` import ctrlPID, so they use the C.10 gains and print different numbers from the book |
| C.18 simulation | Not specified | `hw18_satelliteSim.py` uses `method="digital_filter"` (Tustin); `"state_space"` crashes (see below) |
| C.18 inner loop | Rate feedback + 45·8/(s + 8) | Lead only, no rate feedback (`ctrlLoopshape` computes θ̇ but never uses it) |
| C.18 outer plant | Derived correctly on p. 366 | `loopShapingOuter.py` uses (σs + 1)/(σJ_ps³ + (σb + J_p)s² + (σk + b + k_D)s + k): wrong numerator, and the b·k_D and k·k_D terms are missing. Its DC gain is 1/k = 10 instead of 1, so the design (and the book's Fig. 18-34 margins, PM 51.75° at 0.15 rad/s) are for the wrong plant. |

## Repo bugs and robustness problems

| File | Issue | Effect |
| --- | --- | --- |
| `ctrlStateFeedbackIntegrator.py` (and Listing 12.3) | `phi = x[0, 0]` reads θ, so the integrator acts on φ_r − θ while the gains were placed for φ_r − φ. | With d = 1 (hw12), the unpatched controller diverges even with α = 0 (|φ| reaches 94 rad in 100 s). Patched to x[1, 0], it is bounded for α = 0, but still diverges for some α = 0.2 draws: τ saturates and the integrator winds up. `regress_C.py` patches it in memory; the workbench integrates φ. |
| `ctrlDisturbanceObserver.py` / `hw14` | No anti-windup. With α = 0.2 the loop diverges for a sizable fraction of parameter draws, mostly those with the true Js above nominal. | With `np.random.seed(s)` then `satelliteDynamics(alpha=0.2)` for s = 0–39, d = 1, the hw14 reference, 50 s, and "diverged" meaning |φ| > 0.5 rad: 13 of 40 draws diverge without noise (seeds 4, 6, 8, 10, 13, 15, 18, 24, 25, 30, 32, 36, 37) and 15 of 40 with the book's noise (adds 29 and 39). With only Js changed, the onset is between +8% and +10%; with the other parameters also perturbed it can start lower. An independent check counted 14/40 without noise with a different horizon and criterion. The linearized loop stays stable (max Re λ ≈ −0.54 for Js +12%), and with no torque limit it converges. So the divergence is saturation-driven windup. The workbench pages use a draw with Js −12%, and offer anti-windup. |
| `ctrlLoopshape.py`, `transferFunction` | For a strictly proper numerator it sets `C[0, i] = num.item(i)` instead of `num.item(i − (n − m − 1))`. | Raises IndexError for C_out, so `method="state_space"` (the class default) never runs. `regress_C.py` patches it in memory; the workbench filter uses the correct indexing. |
| `ctrlStateFeedback.py` | k_r uses C_r = [1, 0, 0, 0] (θ) although the reference is φ. | Numerically the same, but misleading. |
| `ctrlPD.py` | t_rθ = 1.75 "tuned to not saturate the input" | Saturates; see C.8(f). |
| `ctrlDisturbanceObserver.py` | The observer-state comments say "z_hat, theta_hat" (copied from the ball-beam). | Cosmetic. |
| `hw14_satelliteSim.py` | `n = [noise_phi, noise_th]` adds "noise_phi" to θ and "noise_th" to φ. | Cosmetic: both have σ = 0.001. |
| `satelliteParam.py`, `hw15.py` | Header comments say "Inverted Pendulum Parameter File." | Cosmetic. |
| `.../python` initial-condition handling | ctrlPID initializes θ_prev, φ_prev from satelliteParam (θ0 = φ0 = 0), not from the first measurement. | The dirty derivatives kick if the simulation starts away from 0. The workbench uses the first measurement, which is identical for zero initial conditions. |
