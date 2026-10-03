// Design Study C: satellite attitude control (controlbook.pdf p. 15).
// Physics mirrors _C_satellite/python/satelliteDynamics.py and satelliteParam.py:
// a rigid body (inertia Js, angle θ) and a solar panel (Jp, φ) joined by a
// torsional spring k and damper b. The thrusters apply a torque τ to the body.
//
// Besides the system object this file holds the study-wide helpers used by the
// chapter files (WB.studies.C.lib): the linear model, the successive-loop-closure
// design of C.8/C.10, a port of scipy.signal.place_poles (the YT algorithm that
// python-control's place() calls, needed because the two-output observers of
// C.13/C.14 have non-unique gains), and the s-plane legend relabeling.
window.WB = window.WB || {};
WB.systems = WB.systems || {};
WB.studies = WB.studies || {};
WB.studies.C = WB.studies.C || { chapters: {} };

(function () {
  const M = WB.math;
  const L = WB.la;
  const R2D = 180 / Math.PI;

  // ---------------------------------------------------------------- system --
  WB.systems.C = {
    id: 'C',
    name: 'Satellite Attitude Control',
    introPage: 15,
    // Nominal physical parameters (what the controller designer knows), p. 15.
    params: [
      { key: 'Js', label: 'J<sub>s</sub>', unit: 'kg·m²', value: 5.0, min: 1, max: 20, step: 0.05 },
      { key: 'Jp', label: 'J<sub>p</sub>', unit: 'kg·m²', value: 1.0, min: 0.1, max: 5, step: 0.01 },
      { key: 'k', label: 'k', unit: 'N·m', value: 0.1, min: 0.005, max: 1, step: 0.005 },
      { key: 'b', label: 'b', unit: 'N·m·s', value: 0.05, min: 0, max: 0.5, step: 0.005 },
      { key: 'tau_max', label: 'τ<sub>max</sub>', unit: 'N·m', value: 5.0, min: 0.5, max: 20, step: 0.1 },
    ],
    constants: {},
    // Parameters that satelliteDynamics(alpha) perturbs.
    uncertain: ['Js', 'Jp', 'k', 'b'],

    outputs: [
      { key: 'theta', label: 'θ', unit: '°', scale: R2D, minSpan: 2, noiseMax: 1 },
      { key: 'phi', label: 'φ', unit: '°', scale: R2D, minSpan: 2, noiseMax: 1 },
    ],
    inputs: [{ key: 'tau', label: 'τ', unit: 'N·m' }],
    // One reference, on the panel angle (the outer loop of C.8).
    refs: [{ output: 1, label: 'panel reference φ_r', unit: '°', scale: R2D, min: -60, max: 60 }],
    initial: [
      { key: 'theta0', label: 'θ(0)', unit: '°', scale: R2D, output: 0, value: 0, min: -90, max: 90 },
      { key: 'phi0', label: 'φ(0)', unit: '°', scale: R2D, output: 1, value: 0, min: -90, max: 90 },
    ],
    disturbances: [{ input: 0, label: 'd on τ', unit: 'N·m', min: -5, max: 5, step: 0.05 }],
    stateLabels: ['θ', 'φ', 'θ̇', 'φ̇'],

    x0(init) { return [init.theta0 || 0, init.phi0 || 0, 0, 0]; },

    // Eq. 3.3 (p. 53). The repo solves M qdd = c with inv(M) @ c; for this
    // diagonal M that is (1/Js)·c1 and (1/Jp)·c2, written the same way here so
    // the floating-point results match.
    f(x, tau, p) {
      const [th, ph, thd, phd] = x;
      const c1 = tau - p.b * (thd - phd) - p.k * (th - ph);
      const c2 = -p.b * (phd - thd) - p.k * (ph - th);
      return [thd, phd, (1 / p.Js) * c1, (1 / p.Jp) * c2];
    },
    h(x) { return [x[0], x[1]]; },
    uLimit(p) { return p.tau_max; },

    // Panel length and body width from satelliteParam.py (length 1, width 0.3).
    draw(ctx, w, h, s) {
      const { css } = WB.plot;
      const x = s.x, u = s.u;
      const phiR = s.rAll[0];
      const cx = w / 2, cy = h * 0.52;
      const S = Math.min(w * 0.36, h * 0.42);      // pixels per panel length
      const W = 0.3 * S;                            // body width
      // inertial reference line
      ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.setLineDash([3, 4]);
      ctx.beginPath(); ctx.moveTo(cx - 1.25 * S, cy); ctx.lineTo(cx + 1.25 * S, cy); ctx.stroke();
      ctx.setLineDash([]);

      // reference ghost for the panel (θ, φ are measured clockwise, Fig. 1-4)
      ctx.save(); ctx.translate(cx, cy); ctx.rotate(phiR);
      ctx.strokeStyle = css('--text-muted'); ctx.lineWidth = 1.5; ctx.setLineDash([6, 5]);
      ctx.beginPath(); ctx.moveTo(-1.12 * S, 0); ctx.lineTo(1.12 * S, 0); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = css('--text-muted'); ctx.font = '11px var(--font-sans, system-ui)';
      ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.fillText('φ_r', 1.14 * S, 0);
      ctx.restore();

      // solar panel: thin plate of half-length "length" (drawPanel, Listing 2.5)
      ctx.save(); ctx.translate(cx, cy); ctx.rotate(x[1]);
      ctx.fillStyle = css('--series-3'); ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.rect(-S, -W / 6, 2 * S, W / 3); ctx.fill(); ctx.stroke();
      ctx.restore();

      // body with its two thruster nubs (drawBase polygon, Listing 2.5)
      const pts = [[0.5, -0.5], [0.5, -1 / 6], [0.5 + 1 / 6, -1 / 6], [0.5 + 1 / 6, 1 / 6], [0.5, 1 / 6], [0.5, 0.5],
        [-0.5, 0.5], [-0.5, 1 / 6], [-0.5 - 1 / 6, 1 / 6], [-0.5 - 1 / 6, -1 / 6], [-0.5, -1 / 6], [-0.5, -0.5]];
      ctx.save(); ctx.translate(cx, cy); ctx.rotate(x[0]);
      ctx.fillStyle = css('--series-1'); ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 1.5;
      ctx.beginPath(); pts.forEach(([px, py], i) => (i ? ctx.lineTo(px * W, py * W) : ctx.moveTo(px * W, py * W)));
      ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.restore();

      // torque arc on the body, length proportional to |τ| / τmax
      const frac = Math.max(-1, Math.min(1, u / s.uLimit));
      if (Math.abs(frac) > 0.01) {
        const Rr = W * 1.05, sweep = frac * 1.5 * Math.PI, a0 = x[0] - Math.PI / 2;
        ctx.strokeStyle = css(s.saturated ? '--critical' : '--series-2'); ctx.lineWidth = 2.5;
        // positive τ turns θ clockwise on screen (canvas angles grow clockwise)
        ctx.beginPath(); ctx.arc(cx, cy, Rr, a0, a0 + sweep, sweep < 0); ctx.stroke();
        const ae = a0 + sweep, dir = sweep > 0 ? 1 : -1;
        const ex = cx + Rr * Math.cos(ae), ey = cy + Rr * Math.sin(ae);
        const tx = -Math.sin(ae) * dir, ty = Math.cos(ae) * dir;
        ctx.fillStyle = ctx.strokeStyle;
        ctx.beginPath();
        ctx.moveTo(ex + tx * 7, ey + ty * 7);
        ctx.lineTo(ex - tx * 2 + Math.cos(ae) * 5, ey - ty * 2 + Math.sin(ae) * 5);
        ctx.lineTo(ex - tx * 2 - Math.cos(ae) * 5, ey - ty * 2 - Math.sin(ae) * 5);
        ctx.fill();
      }
      ctx.fillStyle = css('--text-muted'); ctx.font = '11px var(--font-sans, system-ui)';
      ctx.textAlign = 'left'; ctx.textBaseline = 'top';
      ctx.fillText('θ, φ clockwise from the dashed line', 8, 8);
    },

    // Problem data. Statements are paraphrased; pages are controlbook.pdf pages.
    problems: {
      ch2: {
        id: 'C.2', page: 35,
        sim: { tEnd: 20, tStep: 0, amplitude: 0 },
        statement: [
          '(a) Using the configuration variables θ and φ, write an expression for the kinetic energy of the system.',
          '(b) Write a class that animates the satellite and display sinusoidal variations of q = (θ, φ)ᵀ.',
        ],
      },
      ch3: {
        id: 'C.3', page: 51,
        sim: { tEnd: 50, tStep: 0, amplitude: 0 },
        openLoop: { shape: 'sine', amp: 0.1, freq: 0.1 },   // hw03_satelliteSim.py
        statement: [
          '(a) Find the potential energy of the system.',
          '(b) Define the generalized coordinates.',
          '(c) Find the generalized forces and damping forces.',
          '(d) Derive the equations of motion with the Euler-Lagrange equations.',
          '(e) Implement the equations of motion and simulate with a variable torque on the body as the input.',
        ],
      },
      ch4: {
        id: 'C.4', page: 67,
        sim: { tEnd: 60, tStep: 0, amplitude: 0, y0: 20, init: { phi0: -10 } },
        statement: ['(a) Find the equilibria of the system.'],
      },
      ch5: {
        id: 'C.5', page: 77,
        sim: { tEnd: 40, tStep: 0, amplitude: 0 },
        statement: [
          '(a) Take the Laplace transform of the linearized equations of motion.',
          '(b) Find the full transfer matrix from τ(s) to Θ(s) and Φ(s).',
          '(c) From the transfer matrix, find the second-order transfer function from Θ(s) to Φ(s).',
          '(d) Assuming (J<sub>s</sub> + J<sub>p</sub>)/J<sub>s</sub> ≈ 1, find a second-order approximation of the transfer function from τ(s) to Θ(s).',
          '(e) Form the approximate cascade of the two transfer functions and justify why it makes sense physically.',
        ],
      },
      ch6: {
        id: 'C.6', page: 91,
        sim: { tEnd: 40, tStep: 0, amplitude: 0 },
        statement: [
          'A star tracker measures θ and a strain gauge approximates φ − θ. With x = (θ, φ, θ̇, φ̇)ᵀ, u = τ and measured output y = (θ, φ − θ)ᵀ, find A, B, C, D of the linear state-space model.',
        ],
      },
      ch8: {
        id: 'C.8', page: 129,
        trTh: 1, zetaTh: 0.9, M: 10, zetaPhi: 0.9, rule: 'tp', thetaMaxDeg: 30,
        repoTr: 1.75,      // ctrlPD.py, "tuned to not saturate the input"
        satStepDeg: 30,
        sim: { type: 'square', amplitude: 15, frequency: 0.015, tStep: 0, tEnd: 80 },
        statement: [
          '(a) Draw the successive-loop-closure block diagram with PD control in both loops: the outer loop maps φ<sub>r</sub> to the body reference θ<sub>r</sub>, the inner loop maps θ<sub>r</sub> to the torque τ.',
          '(b) Inner loop: find k<sub>P<sub>θ</sub></sub>, k<sub>D<sub>θ</sub></sub> for rise time t<sub>r<sub>θ</sub></sub> = 1 s and ζ<sub>θ</sub> = 0.9.',
          '(c) Find the DC gain k<sub>DC<sub>θ</sub></sub> of the inner loop.',
          '(d) Replacing the inner loop by its DC gain, find k<sub>P<sub>φ</sub></sub>, k<sub>D<sub>φ</sub></sub> so the outer rise time is t<sub>r<sub>φ</sub></sub> = 10 t<sub>r<sub>θ</sub></sub> with ζ<sub>φ</sub> = 0.9.',
          '(e) Simulate with φ<sub>r</sub> a 15° square wave at 0.015 Hz.',
          '(f) With |τ| ≤ 5 N·m, use the outer rise time as a tuning knob to get the fastest response without saturating τ for a 30° step on φ<sub>r</sub>.',
        ],
      },
      ch9: {
        id: 'C.9', page: 150,
        sim: { type: 'step', amplitude: 10, tStep: 0, tEnd: 120, dist: 0, tDist: 60 },
        statement: [
          '(a) With PD control on the inner loop, what is its system type with respect to the reference and to an input disturbance? Characterize the steady-state error for steps, ramps and parabolas in each.',
          '(b) With PD control on the outer loop, answer the same questions. How does this change if you add an integrator?',
        ],
      },
      p6: {
        id: 'C.P.6', page: 472,
        sim: { type: 'square', amplitude: 15, frequency: 0.015, tStep: 0, tEnd: 80 },
        statement: [
          'Add an integrator to the outer loop to get PID control. Put the closed-loop characteristic equation in Evans form, plot the root locus versus k<sub>I</sub>, and choose a k<sub>I</sub> that does not significantly change the other closed-loop poles.',
        ],
      },
      ch10: {
        id: 'C.10', page: 166,
        trTh: 0.4, zetaTh: 0.9, M: 15, zetaPhi: 0.9, ki: 0.15, sigma: 0.05, rule: '2.2', thetaMaxDeg: 30,
        sim: { type: 'square', amplitude: 15, frequency: 0.02, tStep: 0, tEnd: 60 },
        mismatch: { Js: -12, Jp: 9, k: 15, b: -7 },   // a fixed alpha = 0.2 draw so the page is repeatable (see ISSUES.md on Js > nominal)
        statement: [
          '(a) Let J<sub>s</sub>, J<sub>p</sub>, k and b vary by up to 20% (α = 0.2).',
          '(b) The controller may use only the measured angles θ, φ and the reference φ<sub>r</sub>.',
          '(c) Implement the nested PID loops designed in C.8 with dirty-derivative gain σ = 0.05, and tune the integrator to remove the steady-state error caused by the uncertain parameters.',
        ],
      },
      ch11: {
        id: 'C.11', page: 192,
        trTh: 2.0, zetaTh: 0.9, M: 3, zetaPhi: 0.9, rule: 'tp',
        sim: { type: 'square', amplitude: 15, frequency: 0.04, tStep: 0, tEnd: 30 },
        statement: [
          '(a) From ω<sub>n<sub>φ</sub></sub>, ζ<sub>φ</sub>, ω<sub>n<sub>θ</sub></sub>, ζ<sub>θ</sub> (C.8), find the desired closed-loop poles.',
          '(b) Add A, B, C, D from C.6.',
          '(c) Check controllability: rank(𝒞<sub>A,B</sub>) = n.',
          '(d) Find K so eig(A − BK) are the desired poles, and k<sub>r</sub> so the DC gain from φ<sub>r</sub> to φ is one.',
          '(e) Implement the state feedback and tune the poles. You should get a much faster response than with successive loop closure.',
        ],
      },
      ch12: {
        id: 'C.12', page: 210,
        trTh: 2.0, zetaTh: 0.9, M: 3, zetaPhi: 0.9, rule: 'tp', pI: -2,
        sim: { type: 'square', amplitude: 15, frequency: 0.01, tStep: 0, tEnd: 100, dist: 1, tDist: 0 },
        mismatch: { Js: -12, Jp: 9, k: 15, b: -7 },
        statement: [
          '(a) Add an integrator with anti-windup on φ to the C.11 state feedback.',
          '(b) Add a constant input disturbance of 1 N·m and let the parameters vary by up to 20%.',
          '(c) Tune the integrator pole (and other gains if needed) for good tracking.',
        ],
      },
      ch13: {
        id: 'C.13', page: 234,
        trTh: 2.0, zetaTh: 0.9, M: 3, zetaPhi: 0.9, rule: 'tp', pI: -2, obsFactor: 10,
        sim: { type: 'square', amplitude: 15, frequency: 0.03, tStep: 0, tEnd: 40, dist: 0, tDist: 0 },
        statement: [
          '(a) Use exact parameters (α = 0) and no input disturbance.',
          '(b) Check observability: rank(𝒪<sub>A,C</sub>) = n.',
          '(c) Add an observer and use x̂ in the C.12 controller. Tune the controller and observer poles.',
          '(d) Plot the states and their estimates together.',
          '(e) Add an input disturbance of 1.0 and observe the steady-state error, even with the integrator.',
        ],
      },
      ch14: {
        id: 'C.14', page: 254,
        trTh: 2.0, zetaTh: 0.9, M: 3, zetaPhi: 0.9, rule: 'tp', pI: -2, obsFactor: 10, pD: -10,
        sim: { type: 'square', amplitude: 15, frequency: 0.03, tStep: 0, tEnd: 40, dist: 1, tDist: 0, noise: 0.0573, noises: [0.0573] },
        mismatch: { Js: -12, Jp: 9, k: 15, b: -7 },
        statement: [
          '(a) Use α = 0.2, an input disturbance of 1.0 N·m, and noise with standard deviation 0.001 rad on both measured angles. Without a disturbance observer the controller is not robust to the large disturbance.',
          '(b) Add a disturbance observer, verify that the estimator\'s steady-state error is removed, and tune.',
        ],
      },
      ch15: {
        id: 'C.15', page: 280,
        sim: { tEnd: 60, tStep: 0, amplitude: 0 },
        statement: [
          '(a) Draw by hand the Bode plot of the inner-loop transfer function from τ to θ, then compare with the bode command.',
          '(b) Draw by hand the Bode plot of the outer-loop transfer function from θ to φ, then compare with the bode command.',
        ],
      },
      ch16: {
        id: 'C.16', page: 298,
        parabA: 20, wdin: 0.1, wno: 10,
        sim: { type: 'square', amplitude: 15, frequency: 0.02, tStep: 0, tEnd: 60 },
        statement: [
          'Inner loop: plot the Bode plots of the plant and of the plant under PD control with the C.10 gains.',
          '(a) To what error can the closed loop track θ<sub>r</sub>(t) = 20t²?',
          '(b) If the input disturbance has content below ω<sub>d<sub>in</sub></sub> = 0.1 rad/s, what percentage appears in the output?',
          'Outer loop: plot the plant and the plant under PID control with the C.10 gains.',
          '(c) If the noise has content above ω<sub>no</sub> = 10 rad/s, what percentage shows up in φ?',
        ],
      },
      ch17: {
        id: 'C.17', page: 318,
        sim: { type: 'square', amplitude: 15, frequency: 0.02, tStep: 0, tEnd: 60 },
        statement: [
          'Use the C.10 gains.',
          '(a) Find the phase and gain margins of the inner loop under PD control. Plot the open- and closed-loop Bode plots together. What is the closed-loop bandwidth, and how does it relate to the crossover frequency?',
          '(b) Answer the same for the outer loop under PID control.',
          '(c) What is the bandwidth separation between the inner and outer loops? Is successive loop closure justified?',
        ],
      },
      ch18: {
        id: 'C.18', page: 361,
        inner: { wr: 0.01, gr: 0.01, wn: 20, gn: 0.01, pm: 60 },
        outer: { wdin: 0.01, gdin: 0.1, wn: 10, gn: 1e-4, pm: 60 },
        sim: { type: 'square', amplitude: 15, frequency: 0.02, tStep: 0, tEnd: 100, dist: 0.5, tDist: 0 },
        mismatch: { Js: -6, Jp: 4, k: 8, b: -5 },
        statement: [
          '(a) Inner loop: add rate feedback τ = −k<sub>D<sub>θ</sub></sub>θ̇ + τ′ with k<sub>D<sub>θ</sub></sub> from C.10, then design C<sub>in</sub>(s) so θ tracks content below 0.01 rad/s to within γ<sub>r</sub> = 0.01, noise above 20 rad/s is attenuated by γ<sub>n</sub> = 0.01, and PM ≈ 60°.',
          '(b) Outer loop: with rate feedback using k<sub>D<sub>φ</sub></sub> from C.10, the plant is P = P<sub>out</sub>·P<sub>in</sub>C<sub>in</sub>/(1 + P<sub>in</sub>C<sub>in</sub>). Design C<sub>out</sub>: zero steady-state error to steps, input disturbances below 0.01 rad/s rejected by γ<sub>d<sub>in</sub></sub> = 0.1, noise above 10 rad/s attenuated by γ<sub>n</sub> = 10⁻⁴, PM ≈ 60°, and a prefilter to reduce peaking.',
        ],
      },
    },
  };

  // ------------------------------------------------------- study helpers --
  // Linear model (C.6, p. 92), with y = (θ, φ) as in the repo (C.13/C.14).
  function ss(p) {
    const { Js, Jp, k, b } = p;
    return {
      A: [[0, 0, 1, 0], [0, 0, 0, 1], [-k / Js, k / Js, -b / Js, b / Js], [k / Jp, -k / Jp, b / Jp, -b / Jp]],
      B: [[0], [0], [1 / Js], [0]],
      C: [[1, 0, 0, 0], [0, 1, 0, 0]],
      Cbook: [[1, 0, 0, 0], [-1, 1, 0, 0]],   // y = (θ, φ − θ), C.6
    };
  }

  // ω_n from a rise time: '2.2' → 2.2/t_r (Eq. 8.5); 'tp' → π/(2 t_r √(1−ζ²)) (C.8, p. 131).
  function wnRule(tr, zeta, rule) {
    if (rule === 'tp') return 0.5 * Math.PI / (tr * Math.sqrt(Math.max(1e-9, 1 - zeta * zeta)));
    return 2.2 / tr;
  }
  const pairPoles = (wn, zeta) => WB.design.polesFromWnZeta(wn, zeta);

  // Successive-loop-closure gains (C.8 pp. 130–132, ctrlPD.py / ctrlPID.py).
  // Inner loop on 1/((Js+Jp)s²); outer loop on the panel model with the inner
  // loop replaced by its DC gain (k_DCθ = 1).
  function slcDesign(p, { trTh, zetaTh, M: Msep, zetaPhi, rule, ki = 0 }) {
    const J = p.Js + p.Jp;
    const wnTh = wnRule(trTh, zetaTh, rule);
    const kPth = wnTh ** 2 * J, kDth = 2 * zetaTh * wnTh * J;
    const DC = 1;
    const trPhi = Msep * trTh;
    const wnPhi = wnRule(trPhi, zetaPhi, rule);
    const a11 = p.k * DC, a12 = -p.b * DC * wnPhi ** 2;
    const a21 = p.b * DC, a22 = p.k * DC - 2 * zetaPhi * wnPhi * p.b * DC;
    const b1 = -p.k + p.Jp * wnPhi ** 2, b2 = -p.b + 2 * p.Jp * zetaPhi * wnPhi;
    const det = a11 * a22 - a12 * a21;
    const kPphi = (a22 * b1 - a12 * b2) / det, kDphi = (-a21 * b1 + a11 * b2) / det;
    return { kPth, kDth, kPphi, kDphi, kIphi: ki, wnTh, wnPhi, trPhi, kDCth: DC, kDCphi: p.k * DC * kPphi / (p.k + p.k * DC * kPphi), AA: [[a11, a12], [a21, a22]], bb: [b1, b2] };
  }

  // Design-model poles of the two loops for given gains (p. 130 and Eq. 8.14).
  function innerPoles(p, g) {
    const J = p.Js + p.Jp;
    return M.roots2(g.kDth / J, g.kPth / J);
  }
  function outerCharPoly(p, g, kDC = 1) {
    const a2 = p.Jp + p.b * kDC * g.kDphi;
    const a1 = p.b + p.b * kDC * g.kPphi + p.k * kDC * g.kDphi;
    const a0 = p.k + p.k * kDC * g.kPphi;
    if (g.kIphi) return [a2, a1 + p.b * kDC * g.kIphi, a0, p.k * kDC * g.kIphi].map((c) => c / a2);  // P.6
    return [1, a1 / a2, a0 / a2];
  }
  const outerPoles = (p, g) => L.roots(outerCharPoly(p, g));

  // Closed-loop poles of the full four-state model under the cascade, to show
  // how well the bandwidth separation holds. With sigma, the dirty derivatives
  // (s/(σs+1)) are included as filter states; kI adds the outer integrator.
  function fullLoopPoles(p, g, { sigma = null } = {}) {
    const { A, B } = ss(p);
    const n = 4 + (g.kIphi ? 1 : 0) + (sigma ? 2 : 0);
    const Acl = L.zeros(n, n);
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) Acl[i][j] = A[i][j];
    // τ = kPθ(θr − θ) − kDθ θ̇̂,  θr = −kPφ φ − kDφ φ̇̂ + kIφ z   (reference set to 0)
    // Express τ as a row over the closed-loop state.
    const tau = new Array(n).fill(0), thr = new Array(n).fill(0);
    const iz = 4, iwTh = g.kIphi ? 5 : 4, iwPh = iwTh + 1;
    thr[1] -= g.kPphi;
    if (g.kIphi) { thr[iz] += g.kIphi; Acl[iz][1] = -1; }     // ż = r − φ
    if (sigma) {
      // φ̇̂ = (φ − w_φ)/σ with ẇ_φ = (φ − w_φ)/σ; the same for θ
      thr[1] -= g.kDphi / sigma; thr[iwPh] += g.kDphi / sigma;
      Acl[iwPh][1] = 1 / sigma; Acl[iwPh][iwPh] = -1 / sigma;
      Acl[iwTh][0] = 1 / sigma; Acl[iwTh][iwTh] = -1 / sigma;
      tau[0] -= g.kDth / sigma; tau[iwTh] += g.kDth / sigma;
    } else {
      thr[3] -= g.kDphi;
      tau[2] -= g.kDth;
    }
    for (let j = 0; j < n; j++) tau[j] += g.kPth * thr[j];
    tau[0] -= g.kPth;
    for (let j = 0; j < n; j++) Acl[2][j] += B[2][0] * tau[j];
    return L.eig(Acl);
  }

  // ------------------------------------- scipy.signal.place_poles (YT) port --
  // python-control's place(A, B, p) calls scipy.signal.place_poles(method='YT').
  // For a single input the gain is unique and any method agrees, but the
  // two-output observers of C.13/C.14 (place(Aᵀ, Cᵀ)) have a whole family of
  // valid gains; the repo's L is the one this algorithm converges to. This is a
  // line-by-line port (scipy 1.18 _ltisys.py), with Householder QR using the
  // LAPACK sign conventions so the kernel bases (and hence the starting point
  // of the iteration) are the same as numpy's.
  const cx = (re, im = 0) => ({ re, im });
  const cadd = (a, b) => cx(a.re + b.re, a.im + b.im);
  const csub = (a, b) => cx(a.re - b.re, a.im - b.im);
  const cmul = (a, b) => cx(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
  const cconj = (a) => cx(a.re, -a.im);
  const cdiv = (a, b) => { const d = b.re * b.re + b.im * b.im; return cx((a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d); };
  const cabs = (a) => Math.hypot(a.re, a.im);
  const toC = (Mx) => Mx.map((r) => r.map((v) => (typeof v === 'number' ? cx(v) : v)));

  // Full QR (Householder, as LAPACK zgeqr2 + zung2r). Am: m×n complex. Returns {Q (m×m), R (m×n)}.
  function qrFull(Am) {
    const m = Am.length, n = Am[0].length, k = Math.min(m, n);
    const A = Am.map((r) => r.map((z) => cx(z.re, z.im)));
    const refl = [];
    for (let i = 0; i < k; i++) {
      const alpha = A[i][i];
      const xs = [];
      for (let r = i + 1; r < m; r++) xs.push(A[r][i]);
      const xnorm = Math.sqrt(xs.reduce((s, z) => s + z.re * z.re + z.im * z.im, 0));
      let tau = cx(0), beta = alpha.re, v = [cx(1)];
      if (!(xnorm === 0 && alpha.im === 0)) {
        const hyp = Math.hypot(alpha.re, alpha.im, xnorm);
        beta = alpha.re >= 0 ? -hyp : hyp;
        tau = cx((beta - alpha.re) / beta, -alpha.im / beta);
        const sc = cdiv(cx(1), cx(alpha.re - beta, alpha.im));
        v = [cx(1), ...xs.map((z) => cmul(z, sc))];
        A[i][i] = cx(beta);
        for (let r = i + 1; r < m; r++) A[r][i] = cx(0);
      } else {
        v = [cx(1), ...xs.map(() => cx(0))];
      }
      // apply H^H = I − conj(τ) v v^H to the trailing columns
      const ct = cconj(tau);
      for (let j = i + 1; j < n; j++) {
        let w = cx(0);
        for (let r = 0; r < v.length; r++) w = cadd(w, cmul(cconj(v[r]), A[i + r][j]));
        const f = cmul(ct, w);
        for (let r = 0; r < v.length; r++) A[i + r][j] = csub(A[i + r][j], cmul(v[r], f));
      }
      refl.push({ tau, v });
    }
    // Q = H_1 H_2 ... H_k applied to the identity
    const Q = Array.from({ length: m }, (_, i) => Array.from({ length: m }, (_, j) => cx(i === j ? 1 : 0)));
    for (let i = k - 1; i >= 0; i--) {
      const { tau, v } = refl[i];
      for (let j = 0; j < m; j++) {
        let w = cx(0);
        for (let r = 0; r < v.length; r++) w = cadd(w, cmul(cconj(v[r]), Q[i + r][j]));
        const f = cmul(tau, w);
        for (let r = 0; r < v.length; r++) Q[i + r][j] = csub(Q[i + r][j], cmul(v[r], f));
      }
    }
    return { Q, R: A };
  }

  // Complex helpers on matrices (arrays of rows of {re, im}).
  const cmatmul = (X, Y) => X.map((row) => Y[0].map((_, j) => row.reduce((s, a, k) => cadd(s, cmul(a, Y[k][j])), cx(0))));
  const ccol = (X, j) => X.map((r) => r[j]);
  function cinv(X) {
    const n = X.length;
    const Mx = X.map((row, i) => [...row.map((z) => cx(z.re, z.im)), ...Array.from({ length: n }, (_, j) => cx(i === j ? 1 : 0))]);
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (cabs(Mx[r][c]) > cabs(Mx[piv][c])) piv = r;
      if (cabs(Mx[piv][c]) < 1e-300) return null;
      [Mx[c], Mx[piv]] = [Mx[piv], Mx[c]];
      const d = Mx[c][c];
      for (let j = 0; j < 2 * n; j++) Mx[c][j] = cdiv(Mx[c][j], d);
      for (let r = 0; r < n; r++) {
        if (r === c) continue;
        const f = Mx[r][c];
        if (f.re === 0 && f.im === 0) continue;
        for (let j = 0; j < 2 * n; j++) Mx[r][j] = csub(Mx[r][j], cmul(f, Mx[c][j]));
      }
    }
    return Mx.map((row) => row.slice(n));
  }
  function detReal(X) {
    const n = X.length, A = X.map((r) => r.slice());
    let d = 1;
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[piv][c])) piv = r;
      if (A[piv][c] === 0) return 0;
      if (piv !== c) { [A[c], A[piv]] = [A[piv], A[c]]; d = -d; }
      d *= A[c][c];
      for (let r = c + 1; r < n; r++) {
        const f = A[r][c] / A[c][c];
        for (let j = c; j < n; j++) A[r][j] -= f * A[c][j];
      }
    }
    return d;
  }
  const allclose = (a, b) => Math.abs(a - b) <= 1e-8 + 1e-5 * Math.abs(b);
  const allZero = (vec) => vec.every((z) => cabs(z) <= 1e-8);
  const cnorm = (vec) => Math.sqrt(vec.reduce((s, z) => s + z.re * z.re + z.im * z.im, 0));
  const isReal = (p) => p.im === 0;

  function orderComplexPoles(poles) {
    const lex = (a, b) => a.re - b.re || a.im - b.im;
    const real = poles.filter(isReal).sort(lex);
    const out = [...real];
    for (const p of poles.filter((q) => q.im < 0).sort(lex)) {
      if (poles.some((q) => q.re === p.re && q.im === -p.im)) out.push(p, cx(p.re, -p.im));
    }
    if (out.length !== poles.length) throw new Error('Complex poles must come with their conjugates');
    return out;
  }

  // 2×2 complex eigen-decomposition (eigenvectors normalized).
  function eig2(Mm) {
    const [[a, b], [c, d]] = Mm;
    const tr = cadd(a, d), det = csub(cmul(a, d), cmul(b, c));
    const disc = csub(cmul(tr, tr), cmul(cx(4), det));
    const r = cabs(disc), th = Math.atan2(disc.im, disc.re);
    const sq = cx(Math.sqrt(r) * Math.cos(th / 2), Math.sqrt(r) * Math.sin(th / 2));
    const lam = [cmul(cx(0.5), cadd(tr, sq)), cmul(cx(0.5), csub(tr, sq))];
    const vecs = lam.map((l) => {
      let v;
      if (cabs(b) >= cabs(c) && cabs(b) > 1e-300) v = [b, csub(l, a)];
      else if (cabs(c) > 1e-300) v = [csub(l, d), c];
      else v = cabs(csub(l, a)) < cabs(csub(l, d)) ? [cx(1), cx(0)] : [cx(0), cx(1)];
      const nv = cnorm(v);
      return v.map((z) => cmul(z, cx(1 / nv)));
    });
    return { lam, vecs };
  }
  // 2×2 real SVD: left/right singular vectors for the two singular values.
  function svd2(Mm) {
    const [[a, b], [c, d]] = Mm;
    const AtA = [[a * a + c * c, a * b + c * d], [a * b + c * d, b * b + d * d]];
    const tr = AtA[0][0] + AtA[1][1], det = AtA[0][0] * AtA[1][1] - AtA[0][1] ** 2;
    const disc = Math.sqrt(Math.max(0, tr * tr / 4 - det));
    const s2 = [tr / 2 + disc, Math.max(0, tr / 2 - disc)];
    const vs = s2.map((l) => {
      let v = Math.abs(AtA[0][1]) > 1e-300 ? [AtA[0][1], l - AtA[0][0]] : (AtA[0][0] >= AtA[1][1]) === (l === s2[0]) ? [1, 0] : [0, 1];
      const nv = Math.hypot(v[0], v[1]); return [v[0] / nv, v[1] / nv];
    });
    const sv = s2.map(Math.sqrt);
    const us = vs.map((v, i) => {
      const uu = [a * v[0] + b * v[1], c * v[0] + d * v[1]];
      const nu = Math.hypot(uu[0], uu[1]);
      return nu > 1e-300 ? [uu[0] / nu, uu[1] / nu] : (i === 0 ? [1, 0] : [-vs[0][1], vs[0][0]]);
    });
    return { s: sv, u: us, v: vs };
  }

  function placePoles(Ain, Bin, polesIn, { maxiter = 30, rtol = 1e-3 } = {}) {
    const n = Ain.length, m = Bin[0].length;
    const poles = orderComplexPoles(polesIn.map((p) => cx(p.re, p.im || 0)));
    const { Q: u, R: zR } = qrFull(toC(Bin));
    const rankB = Math.min(m, L.rank(Bin));
    const u0 = u.map((r) => r.slice(0, rankB)), u1 = u.map((r) => r.slice(rankB));
    const z = zR.slice(0, rankB);
    const Ac = toC(Ain);
    let gain;
    if (n === rankB) {
      // B square: place directly (not used in this study)
      const D = L.zeros(n, n);
      for (let i = 0; i < n; i++) {
        D[i][i] = poles[i].re;
        if (!isReal(poles[i])) { D[i][i + 1] = -poles[i].im; D[i + 1][i + 1] = poles[i].re; D[i + 1][i] = poles[i].im; i++; }
      }
      const Bi = L.inv(Bin);
      return L.mul(Bi, L.sub(D, Ain)).map((r) => r.map((v) => -v));
    }
    const ker = [];
    let X = null;
    const cols = [];
    for (let j = 0; j < n; j++) {
      const pj = poles[j];
      const ApI = Ac.map((r, i) => r.map((a, jj) => (i === jj ? csub(a, pj) : a)));
      // pole_space_j = (u1ᵀ (A − p_j I))ᵀ : n × (n − rankB)
      const u1T = u1[0].map((_, c) => u1.map((r) => r[c]));
      const ps = cmatmul(u1T, ApI);
      const psT = ps[0].map((_, c) => ps.map((r) => r[c]));
      const { Q } = qrFull(psT);
      const kj = Q.map((r) => r.slice(n - rankB));
      let t = kj.map((r) => r.reduce((s, zz) => cadd(s, zz), cx(0)));
      const nt = cnorm(t);
      t = t.map((zz) => cmul(zz, cx(1 / nt)));
      if (!isReal(pj)) {
        cols.push(t.map((zz) => zz.re), t.map((zz) => zz.im));
        ker.push(kj, kj);
        j++;
      } else {
        cols.push(t.map((zz) => zz.re));
        ker.push(kj);
      }
    }
    X = Array.from({ length: n }, (_, i) => cols.map((c) => c[i]));   // n × n real
    let nbIter = 0, curRtol = NaN;
    if (rankB > 1) {
      const order = ytOrder(poles);
      let stop = false;
      while (nbIter < maxiter && !stop) {
        const detb = Math.abs(detReal(X));
        for (const [i, j] of order) {
          if (i === j) {
            knv0(ker, X, j);
          } else {
            const keep = X.map((r) => r.filter((_, c) => c !== i && c !== j));
            const { Q } = qrFull(toC(keep));
            if (isReal(poles[i])) ytReal(ker, Q, X, i, j);
            else ytComplex(ker, Q, X, i, j);
          }
        }
        const det = Math.max(Math.sqrt(2.220446049250313e-16), Math.abs(detReal(X)));
        curRtol = Math.abs((det - detb) / det);
        if (curRtol < rtol && det > Math.sqrt(2.220446049250313e-16)) stop = true;
        nbIter++;
      }
    }
    // complex transfer matrix and the gain
    const Xc = toC(X);
    for (let idx = 0; idx < n - 1; idx++) {
      if (!isReal(poles[idx])) {
        for (let r = 0; r < n; r++) {
          const re = X[r][idx], im = X[r][idx + 1];
          Xc[r][idx] = cx(re, -im); Xc[r][idx + 1] = cx(re, im);
        }
        idx++;
      }
    }
    const Xi = cinv(Xc);
    if (!Xi) return null;
    const Lam = Xc.map((row) => row.map((zz, c) => cmul(zz, poles[c])));
    const Mm = cmatmul(Lam, Xi);                       // X Λ X⁻¹
    const u0T = u0[0].map((_, c) => u0.map((r) => r[c]));
    const rhs = cmatmul(u0T, Mm.map((r, i) => r.map((zz, j) => csub(zz, Ac[i][j]))));
    // back substitution with the upper-triangular z (rankB × m, m = rankB)
    gain = Array.from({ length: m }, () => new Array(n).fill(null));
    for (let c = 0; c < n; c++) {
      for (let i = m - 1; i >= 0; i--) {
        let s = rhs[i][c];
        for (let k2 = i + 1; k2 < m; k2++) s = csub(s, cmul(z[i][k2], gain[k2][c]));
        gain[i][c] = cdiv(s, z[i][i]);
      }
    }
    const K = gain.map((r) => r.map((zz) => -zz.re));
    K.nbIter = nbIter; K.rtol = curRtol;
    return K;
  }

  function knv0(ker, X, j) {
    const keep = X.map((r) => r.filter((_, c) => c !== j));
    const { Q } = qrFull(toC(keep));
    const q = ccol(Q, Q.length - 1);
    const K = ker[j];
    // mat_ker_pj = ker ker^T (plain transpose, as in scipy)
    const KtQ = K[0].map((_, c) => K.reduce((s, row, r) => cadd(s, cmul(row[c], q[r])), cx(0)));
    const y = K.map((row) => row.reduce((s, kk, c) => cadd(s, cmul(kk, KtQ[c])), cx(0)));
    if (!allZero(y)) {
      const ny = cnorm(y);
      y.forEach((zz, r) => { X[r][j] = zz.re / ny; });
    }
  }

  function ytReal(ker, Q, X, i, j) {
    const n = X.length, last = Q[0].length;
    const uu = ccol(Q, last - 2).map((zz) => zz.re), vv = ccol(Q, last - 1).map((zz) => zz.re);
    const Ki = ker[i].map((r) => r.map((zz) => zz.re)), Kj = ker[j].map((r) => r.map((zz) => zz.re));
    const W = uu.map((a, r) => vv.map((b, c) => a * b - vv[r] * uu[c]));    // u vᵀ − v uᵀ
    const m = L.mul(L.mul(L.T(Ki), W), Kj);
    const { s, u: U, v: V } = svd2(m);
    const xij = [...X.map((r) => r[i]), ...X.map((r) => r[j])];
    let kmn;   // 2n × q
    if (!allclose(s[0], s[1])) {
      const a = Ki.map((r) => r[0] * U[0][0] + r[1] * U[0][1]);
      const b2 = Kj.map((r) => r[0] * V[0][0] + r[1] * V[0][1]);
      kmn = [...a, ...b2].map((v) => [v]);
    } else {
      const a1 = Ki.map((r) => r[0] * U[0][0] + r[1] * U[0][1]), a2 = Ki.map((r) => r[0] * U[1][0] + r[1] * U[1][1]);
      const b1 = Kj.map((r) => r[0] * V[0][0] + r[1] * V[0][1]), b2 = Kj.map((r) => r[0] * V[1][0] + r[1] * V[1][1]);
      kmn = [...a1.map((v, r) => [v, a2[r]]), ...b1.map((v, r) => [v, b2[r]])];
    }
    const kt = L.mul(L.T(kmn), xij.map((v) => [v]));
    let t = L.mul(kmn, kt).map((r) => r[0]);
    if (!t.every((v) => Math.abs(v) <= 1e-8)) {
      const nt = Math.hypot(...t);
      t = t.map((v) => Math.SQRT2 * v / nt);
    } else {
      t = kmn.map((r) => r[0]);
    }
    for (let r = 0; r < n; r++) { X[r][i] = t[r]; X[r][j] = t[n + r]; }
  }

  function ytComplex(ker, Q, X, i, j) {
    const n = X.length, last = Q[0].length;
    const ur = ccol(Q, last - 2).map((zz) => Math.SQRT2 * zz.re), ui = ccol(Q, last - 1).map((zz) => Math.SQRT2 * zz.re);
    const uvec = ur.map((a, r) => cx(a, ui[r]));
    const K = ker[i];
    const q = K[0].length;
    // m = K^H (u u^H − conj(u) u^T) K
    const W = uvec.map((a) => uvec.map((b) => cmul(a, cconj(b))));
    for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) W[r][c] = csub(W[r][c], cmul(cconj(uvec[r]), uvec[c]));
    const KH = K[0].map((_, c) => K.map((row) => cconj(row[c])));
    const m = cmatmul(cmatmul(KH, W), K);
    const { lam, vecs } = eig2(m);
    const idx = cabs(lam[0]) >= cabs(lam[1]) ? [1, 0] : [0, 1];   // argsort by |λ|: [small, large]
    const mu1 = vecs[idx[1]], mu2 = vecs[idx[0]];
    const xij = X.map((r) => cx(r[i], r[j]));
    const Kmu = (mu) => K.map((row) => row.reduce((s, kk, c) => cadd(s, cmul(kk, mu[c])), cx(0)));
    const kmu = !allclose(cabs(lam[idx[1]]), cabs(lam[idx[0]])) ? [Kmu(mu1)] : [Kmu(mu1), Kmu(mu2)];
    void q;
    // t = kmu kmu^H xij
    let t = new Array(n).fill(null).map(() => cx(0));
    for (const col of kmu) {
      const proj = col.reduce((s, zz, r) => cadd(s, cmul(cconj(zz), xij[r])), cx(0));
      t = t.map((zz, r) => cadd(zz, cmul(col[r], proj)));
    }
    if (!allZero(t)) {
      const nt = cnorm(t);
      t.forEach((zz, r) => { X[r][i] = zz.re / nt; X[r][j] = zz.im / nt; });
    } else {
      kmu[0].forEach((zz, r) => { X[r][i] = zz.re; X[r][j] = zz.im; });
    }
  }

  // Update order of the YT loop (scipy _YT_loop), 0-based pairs.
  function ytOrder(poles) {
    const nbReal = poles.filter(isReal).length, hnb = Math.floor(nbReal / 2);
    const o0 = [], o1 = [];
    if (nbReal > 0) { o0.push(nbReal); o1.push(1); }
    const range = (a, b, s = 1) => { const r = []; for (let v = a; s > 0 ? v < b : v > b; v += s) r.push(v); return r; };
    const rComp = range(nbReal + 1, poles.length + 1, 2);
    let rP = range(1, hnb + nbReal % 2);
    o0.push(...rP.map((v) => 2 * v)); o1.push(...rP.map((v) => 2 * v + 1));
    o0.push(...rComp); o1.push(...rComp.map((v) => v + 1));
    rP = range(1, hnb + 1);
    o0.push(...rP.map((v) => 2 * v - 1)); o1.push(...rP.map((v) => 2 * v));
    const r0 = isReal(poles[0]);
    if (hnb === 0 && r0) { o0.push(1); o1.push(1); }
    o0.push(...rComp); o1.push(...rComp.map((v) => v + 1));
    let rJ = range(2, hnb + nbReal % 2);
    for (const jj of rJ) for (let ii = 1; ii <= hnb; ii++) { o0.push(ii); o1.push(ii + jj); }
    if (hnb === 0 && r0) { o0.push(1); o1.push(1); }
    o0.push(...rComp); o1.push(...rComp.map((v) => v + 1));
    rJ = range(2, hnb + nbReal % 2);
    for (const jj of rJ) for (let ii = hnb + 1; ii <= nbReal; ii++) { let i1 = ii + jj; if (i1 > nbReal) i1 = ii + jj - nbReal; o0.push(ii); o1.push(i1); }
    if (hnb === 0 && r0) { o0.push(1); o1.push(1); }
    o0.push(...rComp); o1.push(...rComp.map((v) => v + 1));
    for (let ii = 1; ii <= hnb; ii++) { o0.push(ii); o1.push(ii + hnb); }
    if (hnb === 0 && r0) { o0.push(1); o1.push(1); }
    o0.push(...rComp); o1.push(...rComp.map((v) => v + 1));
    return o0.map((v, k) => [v - 1, o1[k] - 1]);
  }

  // place(A, B, poles) as a flat row (single input) or matrix; observer gain
  // L = place(Aᵀ, Cᵀ, q)ᵀ as an n×p matrix.
  function place(A, B, poles) {
    try { const K = placePoles(A, B, poles); return K; } catch (e) { return null; }
  }
  function obsGain(A, C, poles) {
    const K = place(L.T(A), L.T(C), poles);
    return K ? L.T(K) : null;
  }

  // ------------------------------------------- s-plane legend relabeling --
  // The shared s-plane legend has fixed names per marker kind. Successive loop
  // closure needs "inner loop" / "outer loop" / "full model" markers, so the
  // chapters pass data.legendNames = {kind: label}; this wrapper renames the
  // legend entries after the shared setData has built them. Other studies
  // never set legendNames, so their legends are untouched.
  if (WB.plot && WB.plot.SPlane && !WB.plot.SPlane.prototype.__legendNamesC) {
    const orig = WB.plot.SPlane.prototype.setData;
    WB.plot.SPlane.prototype.setData = function (data) {
      orig.call(this, data);
      const names = data && data.legendNames;
      if (!names) return;
      for (const item of this.legend.querySelectorAll('.legend-item')) {
        const key = item.querySelector('.marker-key');
        const kind = key && [...key.classList].find((c) => c.startsWith('mk-'));
        if (kind && names[kind.slice(3)]) item.lastChild.textContent = names[kind.slice(3)];
      }
    };
    WB.plot.SPlane.prototype.__legendNamesC = true;
  }

  // Series helpers for result arrays.
  const deg = (arr) => Array.from(arr || [], (v) => v * R2D);

  WB.studies.C.lib = {
    R2D, ss, wnRule, pairPoles, slcDesign, innerPoles, outerPoles, outerCharPoly, fullLoopPoles,
    placePoles, place, obsGain, qrFull, deg,
  };
})();
