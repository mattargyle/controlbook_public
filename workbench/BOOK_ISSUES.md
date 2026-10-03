# Design Study A: issues found in the book and repo

As of 2026-10-03. This covers Design Study A (the single link robot arm): problems A.2–A.18 and A.P.6.

I found these while building the workbench:
- 19 problems in the book's worked solutions;
- 6 places where the book and `_A_arm/python` disagree;
- 10 bugs in the repo code.

Two of the book items change a numerical answer:
- A.16(b) is off by a factor of 2.
- A.8(b)'s tuned rise time saturates the actuator it was tuned to avoid.

**How the items were found.** I read the solution pages in `book_and_notes/controlbook.pdf` and re-ran the numbers. The workbench's JavaScript simulation matches the repo's Python to machine precision. I recomputed the frequency-domain values from the book's parameters: m = 0.5 kg, ℓ = 0.3 m, b = 0.01 N·m·s.

Page numbers are PDF pages (book page + 8).

## Errors in the book's worked solutions

| Problem | PDF page | Issue | Correct value / effect |
| --- | --- | --- | --- |
| A.6 | 88 | The text says the Jacobian is evaluated at θe = 0, but the displayed A keeps the general entry (3g/2ℓ) sin θe. | A₂₁ = 0 at θe = 0. A.4 linearizes about a general θe, while A.6 and `hw06` use θe = 0. |
| A.8(b) | 122 | The listing's t_r = 0.37 s is described as "tuned for faster rise time before saturation," but it saturates on the first 0 → 50° step. | That step demands 120% of τ_max. The book's own bound (Eq. 8.8, Fig. 8-13, p. 119–121) with τ_fl(0) = 0.735 gives t_r ≥ 0.489 s. That is the value A.11 then uses. |
| §9.2 Fig. 9-8 | 146 | The PD case is drawn with the PID block (k_D s² + k_P s + k_I)/s, and the text then drops k_I. | Cosmetic. |
| A.10 | 161 | The problem says to use "the PID controller designed in A.8." The listing uses t_r = 0.6, ζ = 0.9 and ω_n = π/(2 t_r √(1−ζ²)). A.8 uses 2.2/t_r with ζ = 0.707. | These give different gains (k_P = 0.5411, k_D = 0.1522 vs. A.8's 0.1134, 0.0483). |
| A.10 | 162 | The anti-windup test is `if abs(self.theta_dot < 0.08):`, which is abs() of a boolean. | It integrates whenever θ̇ < 0.08, including large negative θ̇. It should be `abs(self.theta_dot) < 0.08`. |
| §10.1.1 vs Listing 10.1 | 158–160 | The back-calculation text uses u_I += (u_sat − u_unsat)/k_I. The listing uses T_s/k_I. | The two differ by a factor of T_s. |
| §10.1.3 | 160 | It mentions tuning the anti-windup parameter "v̄," which is never defined. | It is presumably the θ̇ threshold (0.08 in the listing). |
| A.11(e) | 186–189 | The problem asks for a digital differentiator to estimate θ̇. The listing feeds back the true state. | Omission. |
| A.12 | 204–205 | The printed gains K = (1.0370, 0.1817), k_I = −2.2687 correspond to t_r = 0.4. The A.12 listing and the repo use t_r = 0.489. | With t_r = 0.489 and p_I = −5: K = (0.7807, 0.1604), k_I = −1.5181. |
| A.12 | 204 | Step 2 cites the A.11 poles as −3.1191 ± j3.1200. | A.11's own polynomial s² + 6.3616s + 20.2408 gives −3.1808 ± j3.1817. |
| A.12 | 204 | It lists Δ_ol = s³ + 0.6174 s² and a_A1 = (0.6174, 0, 0). | The plant gives 3b/mℓ² = 0.6667. |
| A.12(a) | 203–206 | The problem asks for an integrator with anti-windup. The listing has none. Part (b)'s disturbance size is not given. | The repo uses d = 0.25. |
| A.13 | 230 | The prose cites "RK4 … line 48" and "lines 64–66." | The observer code is around lines 91–110 of Listing 13.1. |
| A.14 | 246–250 | The observer (ω_n,obs = 10, disturbance pole −5.5) is slower than the controller (ω_n = 12.6). Ch 14 also switches to ζ = 0.95 and the π/(2t_r√(1−ζ²)) rule. | This is the opposite of the usual "observer 5–10× faster" rule. "t_r = 0.4" means ω_n = 12.6 here but 5.5 in A.13. |
| A.15 | 275 | The plant is written as 44.44/(s(s + 0.4444)). | The book's parameters give 66.67/(s(s + 0.6667)), as Ch 7–8 and A.16–A.18 use. The Bode constant is 100 either way, but the corner is at 0.667 rad/s. |
| A.16 | 295–296 | The solution's C_PID = (0.179s² + 0.554s + 0.25)/(0.05s² + s) has k_I = 0.25. A.10's listing uses k_I = 0.2. | With k_I = 0.2: B_r = 43.5 dB (not 44.5), B₂ = B_din = 26.0 dB (not 28), so d_in → 5% (not 4%). |
| A.16(b), Eq. 16.12 | 293–296 | It treats θ_r = 5t² as R(s) = 5/s³ and answers e_ss = A/M_a = 0.2. | L{5t²} = 10/s³, so e_ss = 2A/M_a = 0.4 with k_I = 0.25 (0.5 with k_I = 0.2). |
| A.18 | 340–348 | There are several mismatches inside A.18. | Fig. 18-18 (without the extra lag): PM 59.69° at 14.24 rad/s. With the extra lag: PM 57.1°. |
| A.18 margins | 345 | Fig. 18-18 reports GM = +14.7 dB at 56 rad/s. | The loop is conditionally stable. The lag also drops the phase below −180° near 1.8 rad/s, where GM = −23 dB. MATLAB reports the upper crossing; python-control reports the lower one. |

The A.18 row's mismatches:
- The printed final C includes a (s + 0.7)/(s + 0.07) lag that the text never mentions, and the figures were made without it.
- The text claims PM = 64°.
- It says the lead "doesn't change the crossover," but ω_co moves from 10.8 to 14.2 rad/s.
- The prefilter is p = 3 in the text but p = 2 in the listing and Fig. 18-19.
- Listing 18.4 is a different design from the text.
- The Fig. 18-19 caption's "left/right/red" doesn't match the stacked green plots.
- Part (b) says "standard deviation σ² = 0.01."

Smaller notation issues:
- The A.P.6 figure labels the reference R̃_θ but the transfer function uses Θ̃^d (p. 470).
- The appendix figures are numbered 6-x, which collides with Chapter 6.
- Chapter 9's error constant M_p shares a symbol with overshoot.

## Book vs. repo code

| Topic | Book | Repo (`_A_arm/python`) |
| --- | --- | --- |
| A.12 gains | Printed for t_r = 0.4 (K = (1.037, 0.182), k_I = −2.269) | `ctrlStateFeedbackIntegrator.py`: t_r = 0.489, so K = [0.7807, 0.1604], k_I = −1.5181 |
| A.13 tuning | No numbers printed | `ctrlObserver.py`: t_r = 0.4, p_I = −9, observer t_r/10. Its comment says it is "similar to A.12, but [does] not match exactly." |
| A.16–A.17 PID | k_I = 0.25 | `ctrlPID.py`: k_I = 0.2 |
| A.16 C_PID | (`hw16.py`) Derivative on the error | `ctrlPID.py` differentiates θ. The loop gain is the same, but the closed-loop r → θ zeros differ. |
| A.18(c) implementation | State space, Euler with N = 10 substeps (Listing 18.2) | `hw18_armSim.py` uses `method="digital_filter"` (Tustin). The state-space option uses one RK4 step. `hw18` also uses α = 0.1. |
| A.5 transfer function | From the feedback-linearized Eq. 4.7 | `hw05_arm_transfer_function.py` imports `hw06` and uses the Jacobian about θe = 0. Same result. |

## Bugs in the repo code

| File | Bug | Status |
| --- | --- | --- |
| `_A_arm/python/ctrlPID.py:57` | `abs(self.theta_dot < 0.08)`: abs() of a boolean (same as the book listing) | Fixed on `fix/arm-pid-antiwindup`, PR mattargyle/controlbook_public#1 |
| `_A_arm/python/armDynamicsSympy.py` | Loads `"eom_case_study_A"` (no `.pkl`). The committed pickle's signature (x, u, m, ell, b) lacks g. `rk4_step` calls `self.eom(state, u)` with 2 args, so it raises TypeError. `f()` references an undefined `u`. | Open |
| `_A_arm/python/hw06_arm_linearization.py` | Uses `Matrix`, `simplify`, `cos` unqualified: NameError. `hw05` inherits the failure. | Open |
| `hw05` / `hw06` | Importing them re-runs `hw03`, which writes `eom_case_study_A.pkl` and `eom_generated.py` into the repo | Open |
| `_A_arm/python/loopshape_tools.py` | `add_spec_tracking_step` uses undefined `ones`/`size` when `dB_flag=False`. Spec helpers draw single points, not regions. | Open |
| `_A_arm/python/loopshape_tools.py` notch | Not a notch: zeros are a double real zero at √M·ω_s, so the dip is centered at √M·ω_s with depth 2√M/(M+1) (−4.8 dB for M = 10) | Open |
| `_A_arm/loopshape_tools_sliders.py` | The LPF and notch go into the prefilter F, which is then multiplied into the "open-loop" plot. Margins are computed without F. Specs default to dB while the Bode plots are absolute. It labels the phase-crossover frequency (Wcg) as "crossover." | Open |
| `_A_arm/python/ctrlDisturbanceObserver.py` | Dead code (`des_char_est`, `B2`). `tau_d1` is initialized twice. `self.B = B1` works only because it equals [B; 0]. | Open (harmless) |
| `_A_arm/python/hw13_armSim.py` | The 0.01 disturbance is already active (part e) although part (a) asks for none. `dhat` is plotted as 0. | Open |
| Cosmetic | `armParam.py` header says "Inverted Pendulum." `armDynamics.py` gives damping units as "Ns." `ctrlStateFeedback.py` has a stale "dirty derivatives" comment. Both book and repo say "observerable." | Open |

## How the workbench handles them

| Item | Workbench behavior |
| --- | --- |
| A.8(b) rise time | The checker requires the peak demanded torque to be 95–100% of τ_max on a 0 → 50° step. The solution shows the 0.489 s bound and explains why 0.37 s saturates. |
| A.10 anti-windup | Implements the corrected `abs(θ̇) < v̄` test, plus back-calculation (1/k_I) and "none" as options. The JS matches the fixed `ctrlPID.py` to 1e-16. |
| A.11(e) | Toggle between the true state and a dirty-derivative estimate. |
| A.12 gains, anti-windup | The checker uses t_r = 0.489 (listing/repo). The solution notes the book's t_r = 0.4 numbers. An anti-windup toggle (hold the integrator while saturated) is on by default. |
| A.14 tuning | Defaults to the book's tuning. The observer poles are draggable, and a math-card note points out that they are slower than the controller. |
| A.15 plant | Uses 66.67/(s(s + 0.6667)). A math-card note flags the 44.44 typo. |
| A.16 k_I | Buttons switch between k_I = 0.2 (repo) and 0.25 (book figures). Every check is computed from the current gains. |
| A.16(b) factor of 2 | Accepts 2A/M_a. If you enter the book's A/M_a, it says why that is off by 2. |
| A.18 design, margins | Presets for the book's text design (without the extra lag) and the repo's `loopShaping.py`. All gain-margin crossings are listed, so conditional stability shows. |
