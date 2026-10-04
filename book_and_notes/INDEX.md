# controlbook.pdf index

*Introduction to Feedback Control Using Design Studies*, Beard, McLain, Peterson, Killpack (revised Dec 19, 2025), 488 pages.

**All page numbers below are PDF page numbers** (what `Read` with `pages:` expects). PDF page = book page + 8 for the whole book; pages 1–8 are front matter.

## How to use

1. Find the topic here, or search `controlbook.txt` for a term. Every page in it starts with a marker like `===== PDF page 100 | book page 92 =====`, so a grep hit tells you which page to read:
   `grep -n "anti-windup" controlbook.txt`, then look upward for the nearest `PDF page` marker.
2. Read those pages of `controlbook.pdf` (max 20 per request). **Use the PDF for equations, figures, and numbers.** `controlbook.txt` comes from `pdftotext`, which scrambles math (fractions, subscripts, matrices). It is only for search.

Regenerate `controlbook.txt` after replacing the PDF: one `pdftotext -layout -f N -l N` call per page, with the page marker prepended.

## Code ↔ book

- `hwNN_*` files and Example X.NN map to Chapter NN (e.g., `hw13_armSim.py` → Ch 13, Design Study A).
- Worked examples in the chapters: A = arm, B = pendulum on cart, C = satellite.
- Homework problems in Part VI: D = mass-spring-damper, E = block on beam, F = planar VTOL. The `.P.6` problems are root-locus problems from Appendix P.6.
- Controller classes: `ctrlPD` → Ch 7–8, `ctrlPID` → Ch 9–10, `ctrlStateFeedback` → Ch 11, `ctrlStateFeedbackIntegrator` → Ch 12, `ctrlObserver` → Ch 13, `ctrlDisturbanceObserver` → Ch 14, `loopshape*`/`ctrlLoopshape` → Ch 15–18.
- The hummingbird lab is not covered in this book.

## Front matter and Chapter 1

| Content | PDF pages |
|---|---|
| Table of contents | 3–4 |
| Preface | 5–7 |
| Ch 1 Introduction (design process, Fig. 1-1) | 9–12 |
| 1.1 Design Studies A (arm, params), B (pendulum), C (satellite) | 13–15 |

## Part I: Simulation Models (part intro p. 16)

| Ch | Section | PDF pages |
|---|---|---|
| 2 | **Kinetic Energy of Mechanical Systems** | 19–40 |
| | 2.1 Theory | 19–25 |
| | 2.1.1 Example: mass spring; 2.1.2 Example: spinning dumbbell | 26 |
| | 2.2 Design Study A (arm) | 27–30 |
| | 2.3 Design Study B (pendulum on cart) | 31–34 |
| | 2.4 Design Study C (satellite) | 35–38 |
| 3 | **The Euler-Lagrange Equations** | 41–56 |
| | 3.1 Theory: 3.1.1 potential energy, 3.1.2 generalized coordinates/forces | 41–42 |
| | 3.1.3 damping forces, 3.1.4 Euler-Lagrange equations | 43 |
| | 3.2 Design Study A | 43–46 |
| | 3.3 Design Study B | 47–50 |
| | 3.4 Design Study C | 51–54 |

## Part II: Design Models (part intro p. 57)

| Ch | Section | PDF pages |
|---|---|---|
| 4 | **Equilibria and Linearization** | 59–68 |
| | 4.1.1 Jacobian linearization | 59–61 |
| | 4.1.2 Feedback linearization | 62 |
| | 4.2 Design Study A | 62–64 |
| | 4.3 Design Study B | 65–66 |
| | 4.4 Design Study C | 67 |
| 5 | **Transfer Function Models** | 69–80 |
| | 5.1 Theory | 69–70 |
| | 5.2 Design Study A | 71 |
| | 5.3 SIMO systems and cascade approximations | 72–74 |
| | 5.4 Design Study B | 75–76 |
| | 5.5 Design Study C | 77–79 |
| 6 | **State Space Models** | 81–93 |
| | 6.1 Theory; 6.1.1 Jacobian linearization revisited | 81–84 |
| | 6.1.2 Converting state space to transfer function | 85–86 |
| | 6.2 Design Study A | 87–88 |
| | 6.3 Design Study B | 89–90 |
| | 6.4 Design Study C | 91–92 |

## Part III: PID Control Design (part intro pp. 94–98, includes a PID overview)

| Ch | Section | PDF pages |
|---|---|---|
| 7 | **Pole Placement for Second-Order Systems** (PD control) | 99–106 |
| | 7.1 Theory | 99–100 |
| | 7.2 Design Study A | 101–104 |
| 8 | **Design Strategies for Second-Order Systems** | 107–136 |
| | 8.1.1 First-order response | 107–108 |
| | 8.1.2 Second-order response (ζ, ω_n, rise time) | 109–112 |
| | 8.1.3 Effect of a zero on the step response | 113–116 |
| | 8.1.4 Successive loop closure | 117 |
| | 8.1.5 Input saturation and limits of performance | 118–119 |
| | 8.2 Design Study A | 120–123 |
| | 8.3 Design Study B (successive loop closure) | 124–128 |
| | 8.4 Design Study C | 129–134 |
| 9 | **System Type and Integrators** | 137–154 |
| | 9.1.1 Final value theorem | 137 |
| | 9.1.2 System type for reference tracking | 138–142 |
| | 9.1.3 System type for input disturbances | 143–144 |
| | 9.2 Design Study A | 145–146 |
| | 9.3 Design Study B | 147–149 |
| | 9.4 Design Study C | 150–152 |
| 10 | **Digital Implementation of PID Controllers** | 155–169 |
| | 10.1 Theory (Tustin, dirty derivative) | 155–156 |
| | 10.1.1 Integrator anti-windup | 157–159 |
| | 10.1.2 Implementation | 158–159 |
| | 10.1.3 Gain selection for PID | 160 |
| | 10.2 Design Study A | 161–162 |
| | 10.3 Design Study B | 163–165 |
| | 10.4 Design Study C | 166–168 |

## Part IV: Observer Based Control Design (part intro pp. 170–172)

| Ch | Section | PDF pages |
|---|---|---|
| 11 | **Full State Feedback** | 173–196 |
| | 11.1.1 Control canonic form | 175–176 |
| | 11.1.2 Full state feedback using control canonic form | 177–178 |
| | 11.1.3 Full state feedback: general case (controllability, Ackermann) | 179–182 |
| | 11.1.4 State feedback for linearized systems | 183 |
| | 11.2 Summary of design process; 11.2.1 simple example | 184–185 |
| | 11.3 Design Study A | 186–188 |
| | 11.4 Design Study B | 189–191 |
| | 11.5 Design Study C | 192–194 |
| 12 | **Integrator with Full State Feedback** | 197–214 |
| | 12.1 Theory; 12.1.1 integral feedback for linearized systems | 197–199 |
| | 12.2 Summary of design process; 12.2.1 simple example | 200–202 |
| | 12.3 Design Study A | 203–205 |
| | 12.4 Design Study B | 206–209 |
| | 12.5 Design Study C | 210–213 |
| 13 | **Observers** | 215–238 |
| | 13.1.1 Observer canonic form | 216–219 |
| | 13.1.2 General state space (observability) | 220–221 |
| | 13.1.3 Separation principle | 222–223 |
| | 13.1.4 Observers for linearized systems; 13.2 design summary | 224–225 |
| | 13.2.1 Simple example | 226–227 |
| | 13.3 Design Study A | 228–230 |
| | 13.4 Design Study B | 231–233 |
| | 13.5 Design Study C | 234–237 |
| 14 | **Disturbance Observers** | 239–259 |
| | 14.1 Theory; 14.1.1 simple example | 239–245 |
| | 14.2 Design Study A | 246–249 |
| | 14.3 Design Study B | 250–253 |
| | 14.4 Design Study C | 254–258 |

## Part V: Loop-shaping Control Design (part intro p. 260)

| Ch | Section | PDF pages |
|---|---|---|
| 15 | **Frequency Response of LTI Systems** | 261–282 |
| | 15.1.1 Complex numbers | 261–262 |
| | 15.1.2 Frequency response of LTI systems | 263–265 |
| | 15.1.3 Straight-line Bode approximations | 266–271 |
| | 15.1.4–15.1.5 Examples | 272–274 |
| | 15.2 Design Study A | 275–276 |
| | 15.3 Design Study B | 277–279 |
| | 15.4 Design Study C | 280–281 |
| 16 | **Frequency Domain Specifications** | 283–301 |
| | 16.1 Theory | 283–284 |
| | 16.1.1 Tracking | 285–286 |
| | 16.1.2 Noise attenuation | 287 |
| | 16.1.3 Output disturbance rejection | 288–289 |
| | 16.1.4 Input disturbance rejection | 290 |
| | 16.1.5 Frequency response characterization of system type | 291–294 |
| | 16.2 Design Study A | 295–296 |
| | 16.3 Design Study B | 297 |
| | 16.4 Design Study C | 298–299 |
| 17 | **Stability and Robustness Margins** | 303–322 |
| | 17.1.1 Phase and gain margins | 303–305 |
| | 17.1.2 Open- and closed-loop frequency response | 306–309 |
| | 17.1.3 Bode phase-gain relationship | 310 |
| | 17.2 Design Study A | 311–313 |
| | 17.3 Design Study B | 314–317 |
| | 17.4 Design Study C | 318–321 |
| 18 | **Compensator Design** (loopshaping) | 323–374 |
| | 18.1.1–18.1.3 Building blocks: proportional, integral, low-pass | 324–325 |
| | 18.1.4 Phase-lag filter | 325–327 |
| | 18.1.5 Phase-lead filter | 328 |
| | 18.1.6 Simple example 1 | 329–333 |
| | 18.1.7 Controller implementation | 334–335 |
| | 18.1.8 Prefilter design | 336–337 |
| | 18.1.9 Successive loop closure design | 338–339 |
| | 18.2 Design Study A | 340–348 |
| | 18.3 Design Study B | 349–360 |
| | 18.4 Design Study C | 361–373 |

## Part VI: Homework Problems (part intro p. 375)

| Design study | Intro (system, parameters) | Problems |
|---|---|---|
| **D: Mass Spring Damper** | 377 | D.2–D.5: 378 · D.6–D.8: 379 · D.9, D.P.6, D.10: 380 · D.11–D.12: 380–381 · D.13: 381 · D.14–D.16: 382 · D.17–D.18: 383 |
| **E: Block on Beam** | 385 | E.2–E.5: 386 · E.6: 387 · E.8: 387 · E.9, E.P.6, E.10: 388 · E.11: 389 · E.12: 389 · E.13–E.14: 390 · E.15: 390 · E.16–E.17: 391 · E.18: 392 |
| **F: Planar VTOL** | 393–394 | F.2–F.3: 395 · F.4–F.5: 395–396 · F.6: 396 · F.7: 397 · F.8–F.9, F.P.6: 398 · F.10–F.11: 399 · F.12: 400 · F.13–F.14: 400–401 · F.15: 401 · F.16–F.17: 402 · F.18: 403 |

A problem can continue onto the next page, so read a page past the one listed.

## Appendices

| Appendix | PDF pages |
|---|---|
| P.1 Simulating control systems in Python (class structure used by this repo) | 407–420 |
| P.2 Simulating control systems in Matlab | 421–432 |
| P.3 Simulating control systems in Simulink | 433–444 |
| P.4 Numerical solutions to ODEs (RK4, etc.) | 445–456 |
| P.5 Review of ordinary differential equations | 457–464 |
| P.6 Root locus | 465–474 |
| P.7 Review of linear algebra | 475–482 |
| Bibliography | 483–484 |
| Index (book page numbers: add 8) | 485–488 |
