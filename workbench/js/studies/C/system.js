// Design Study C: satellite attitude control (controlbook.pdf p. 15).
// Physics mirrors _C_satellite/python/satelliteDynamics.py and satelliteParam.py:
// a rigid body (inertia Js, angle θ) and a solar panel (Jp, φ) joined by a
// torsional spring k and damper b. The thrusters apply a torque τ to the body.
//
// Besides the system object this file holds the study-wide helpers used by the
// chapter files (WB.studies.C.lib): the linear model and the successive-loop-closure
// design of C.8/C.10. The two-output observers of C.13/C.14 have non-unique gains,
// so they are placed with WB.yt (core/place_yt.js), the port of the scipy YT
// algorithm that python-control's place() calls.
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
      { key: 'Js', desc: 'moment of inertia of the satellite body', label: 'J<sub>s</sub>', unit: 'kg·m²', value: 5.0, min: 1, max: 20, step: 0.05 },
      { key: 'Jp', desc: 'moment of inertia of the panel', label: 'J<sub>p</sub>', unit: 'kg·m²', value: 1.0, min: 0.1, max: 5, step: 0.01 },
      { key: 'k', desc: 'spring constant of the flexible joint', label: 'k', unit: 'N·m', value: 0.1, min: 0.005, max: 1, step: 0.005 },
      { key: 'b', desc: 'damping coefficient of the flexible joint', label: 'b', unit: 'N·m·s', value: 0.05, min: 0, max: 0.5, step: 0.005 },
      { key: 'tau_max', desc: 'maximum torque', label: 'τ<sub>max</sub>', unit: 'N·m', value: 5.0, min: 0.5, max: 100, step: 0.1 },   // up to 100 so C.8(e) can run unsaturated
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
          '(b) Referring to Appendices P.1–P.3, write a Python or Matlab class (or a Matlab function) that creates an animation of the satellite. Simulate the animation to display sinusoidal variations on the configuration variables q = (θ, φ)ᵀ.',
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
          '(e) Referring to Appendices P.1–P.3, write a class or s-function that implements the equations of motion. Simulate the system using a variable torque on the body as an input. The output should connect to the animation function developed in homework C.2.',
        ],
      },
      ch4: {
        id: 'C.4', page: 67,
        sim: { tEnd: 60, tStep: 0, amplitude: 0, y0: 20, init: { phi0: -10 } },
        statement: ['For the satellite system:', '(a) Find the equilibria of the system.'],
      },
      ch5: {
        id: 'C.5', page: 77,
        sim: { tEnd: 40, tStep: 0, amplitude: 0 },
        statement: [
          '(a) Start with the linearized equations for the satellite attitude problem and use the Laplace transform to convert the equations of motion to the s-domain.',
          '(b) Find the full transfer matrix from the input τ(s) to the outputs Φ(s) and Θ(s).',
          '(c) From the transfer matrix, find the second-order transfer function from Θ(s) to Φ(s).',
          '(d) Under the assumption that the panel moment of inertia J<sub>p</sub> is significantly smaller than the spacecraft moment of inertia J<sub>s</sub> (specifically, (J<sub>s</sub> + J<sub>p</sub>)/J<sub>s</sub> ≈ 1), find the second-order approximation for the transfer function from τ(s) to Θ(s).',
          '(e) From your results on parts (c) and (d), form the approximate transfer function cascade for the satellite/panel system and justify why it makes sense physically.',
        ],
      },
      ch6: {
        id: 'C.6', page: 91,
        sim: { tEnd: 40, tStep: 0, amplitude: 0 },
        statement: [
          'Suppose that a star tracker is used to measure θ and a strain gauge is used to approximate φ − θ. Defining the states as x = (θ, φ, θ̇, φ̇)ᵀ, the input as u = τ, and the measured output as y = (θ, φ − θ)ᵀ, find the linear state space equations in the form ẋ = Ax + Bu, y = Cx + Du.',
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
          '(a) When the inner loop controller is PD control, what is the system type of the inner loop with respect to the reference input and with respect to the disturbance input? Characterize the steady-state error when the reference input is a step, a ramp, and a parabola, and when the input disturbance is a step, a ramp, and a parabola.',
          '(b) With PD control for the outer loop, what is the system type of the outer loop with respect to the reference input and with respect to the disturbance input? Characterize the steady-state error when the reference input and disturbance input is a step, a ramp, and a parabola. How does this change if you add an integrator?',
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
          '(a) Using the values for ω<sub>n<sub>φ</sub></sub>, ζ<sub>φ</sub>, ω<sub>n<sub>θ</sub></sub> and ζ<sub>θ</sub> selected in Homework C.8, find the desired closed-loop poles.',
          '(b) Add the state space matrices A, B, C, D derived in Homework C.6 to your param file.',
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
          '(d) Modify the simulation files so that the controller outputs both u and x̂. Add a plotting routine to plot both the state and the estimated state of the system on the same graph.',
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
          '(b) Find the phase and gain margins of the outer loop under PID control. Plot the open- and closed-loop Bode plots of the outer loop on the same plot as those of the inner loop. What is the closed-loop bandwidth, and how does it relate to the crossover frequency?',
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
  const wnRule = WB.design.wnFromTr;
  const pairPoles = WB.design.polesFromWnZeta;

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
    if (g.kIphi) return [a2, a1, a0 + p.b * kDC * g.kIphi, p.k * kDC * g.kIphi].map((c) => c / a2);  // P.6, Fig. 6-10
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

  // s-plane legends: chapters pass data.legendNames = {kind: label} (plot.js).

  // Series helpers for result arrays.
  const deg = (arr) => Array.from(arr || [], (v) => v * R2D);

  // Work-mode answer gating. Anything that answers part `key` ('C.8/b') is shown
  // in Explore mode, or in Work mode once that part is solved. Explore is tested
  // first, so contexts without app.isSolved (tools/regress_C.py) work in Explore.
  const shows = (ctx, ...keys) => ctx.S.mode === 'explore' || keys.every((k) => ctx.app.isSolved(k));
  // The satellite's open-loop poles (eig A) answer C.5(b) (poles of Θ/τ) and C.6.
  const showsOl = (ctx) => shows(ctx, 'C.5/b') || shows(ctx, 'C.6/a');

  WB.studies.C.lib = {
    R2D, ss, wnRule, pairPoles, slcDesign, innerPoles, outerPoles, outerCharPoly, fullLoopPoles,
    deg, shows, showsOl,
  };
})();
