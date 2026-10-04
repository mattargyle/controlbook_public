// Design Study A: single link robot arm (controlbook.pdf p. 13).
// Physics mirrors _A_arm/python/armDynamics.py and armParam.py.
//
// A system file provides everything chapter modules need, so adding studies
// D/E/F means writing one more file with the same fields (see workbench/README.md).
window.WB = window.WB || {};
WB.systems = WB.systems || {};

WB.systems.A = {
  id: 'A',
  name: 'Single Link Robot Arm',
  introPage: 13,
  sym: { y: '\\theta', u: '\\tau', yText: 'θ', uText: 'τ', ff: '\\tau_{fl}', ffText: 'τ_fl' },
  output: { unit: '°', scale: 180 / Math.PI, label: 'θ' },  // display in degrees
  input: { unit: 'N·m', label: 'τ' },

  // Nominal physical parameters (what the controller designer knows).
  params: [
    { key: 'm', desc: 'mass of the arm', label: 'm', unit: 'kg', value: 0.5, min: 0.1, max: 2.0, step: 0.01 },
    { key: 'ell', desc: 'length of the arm', label: 'ℓ', unit: 'm', value: 0.3, min: 0.1, max: 1.0, step: 0.01 },
    { key: 'b', desc: 'damping coefficient', label: 'b', unit: 'N·m·s', value: 0.01, min: 0, max: 0.2, step: 0.001 },
    { key: 'tau_max', desc: 'maximum torque', label: 'τ<sub>max</sub>', unit: 'N·m', value: 1.0, min: 0.2, max: 5, step: 0.05 },
  ],
  constants: { g: 9.8 },
  // Parameters that armDynamics(alpha) perturbs; g is "well known" and is not.
  uncertain: ['m', 'ell', 'b'],

  x0(y0) { return [y0, 0]; },
  stateLabels: ['θ', 'θ̇'],

  f(x, tau, p) {
    const [theta, thetadot] = x;
    const thetaddot = (3 / (p.m * p.ell ** 2)) * (tau - p.b * thetadot - p.m * p.g * (p.ell / 2) * Math.cos(theta));
    return [thetadot, thetaddot];
  },
  h(x) { return x[0]; },
  uLimit(p) { return p.tau_max; },

  // Linear design model P(s) = b0 / (s^2 + a1 s + a0) about theta_e = 0 with the
  // gravity torque cancelled (A.6 / A.7, p. 102).
  secondOrderModel(p) {
    const J = p.m * p.ell ** 2;
    return {
      b0: 3 / J, a1: 3 * p.b / J, a0: 0,
      tex: {
        b0: '\\frac{3}{m\\ell^2}', a1: '\\frac{3b}{m\\ell^2}', a0: '0',
      },
    };
  },
  // Feedback-linearized state space (A.6, Eq. 6.16, p. 88): x = (θ, θ̇), u = τ̃, y = θ.
  stateSpace(p) {
    const J = p.m * p.ell ** 2;
    return { A: [[0, 1], [0, -3 * p.b / J]], B: [[0], [3 / J]], C: [[1, 0]], D: [[0]] };
  },
  // Jacobian linearization about (θe, 0, τe) (A.4 Eq. 4.5, p. 64; A.6 p. 88).
  jacobian(p, yE) {
    const J = p.m * p.ell ** 2;
    return {
      A: [[0, 1], [(3 * p.g / (2 * p.ell)) * Math.sin(yE), -3 * p.b / J]],
      B: [[0], [3 / J]], C: [[1, 0]], D: [[0]],
      ue: p.m * p.g * (p.ell / 2) * Math.cos(yE),
    };
  },
  kinetic(x, p) { return 0.5 * (p.m * p.ell ** 2 / 3) * x[1] ** 2; },       // A.2, p. 29
  potential(x, p) { return p.m * p.g * (p.ell / 2) * Math.sin(x[0]); },    // A.3, p. 44 (P0 = 0)

  // Gravity-cancelling torque tau_fl = m g (l/2) cos(theta)  (Ch 7 Listing 7.1, p. 104)
  feedbackLinearization(x, p) { return p.m * p.g * (p.ell / 2) * Math.cos(x[0]); },
  equilibriumInput(yE, p) { return p.m * p.g * (p.ell / 2) * Math.cos(yE); },
  ffTex: 'm g \\tfrac{\\ell}{2}\\cos\\theta',

  draw(ctx, w, h, s) {
    const { css } = WB.plot;
    const { x, r, u, uLimit, saturated } = s;
    const cx = w / 2, cy = h * 0.55;
    const L = Math.min(w * 0.42, h * 0.48);
    const W = Math.max(8, L * 0.09);
    const tip = (ang, len) => [cx + len * Math.cos(ang), cy - len * Math.sin(ang)];

    // horizon
    ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.setLineDash([3, 4]);
    ctx.beginPath(); ctx.moveTo(cx - L * 1.1, cy); ctx.lineTo(cx + L * 1.1, cy); ctx.stroke();
    ctx.setLineDash([]);

    // reference ghost
    ctx.strokeStyle = css('--text-muted'); ctx.lineWidth = 1.5; ctx.setLineDash([6, 5]);
    const [rx, ry] = tip(r, L);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(rx, ry); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = css('--text-muted'); ctx.font = '11px var(--font-sans, system-ui)';
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillText('r', rx + 6 * Math.cos(r), ry - 6 * Math.sin(r));

    // arm body
    ctx.save();
    ctx.translate(cx, cy); ctx.rotate(-x[0]);
    ctx.fillStyle = css('--series-1');
    ctx.beginPath(); ctx.roundRect(-W / 2, -W / 2, L + W / 2, W, W / 2); ctx.fill();
    ctx.restore();

    // torque arc arrow, length proportional to |tau| / tau_max
    const frac = Math.max(-1, Math.min(1, u / uLimit));
    if (Math.abs(frac) > 0.01) {
      const R = W * 2.6, sweep = frac * 1.5 * Math.PI, a0 = -x[0];
      ctx.strokeStyle = css(saturated ? '--critical' : '--series-2'); ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.arc(cx, cy, R, a0, a0 - sweep, sweep > 0); ctx.stroke();
      const ae = a0 - sweep, dir = sweep > 0 ? -1 : 1;
      const ex = cx + R * Math.cos(ae), ey = cy + R * Math.sin(ae);
      const tx = -Math.sin(ae) * dir, ty = Math.cos(ae) * dir;
      ctx.fillStyle = ctx.strokeStyle;
      ctx.beginPath();
      ctx.moveTo(ex + tx * 7, ey + ty * 7);
      ctx.lineTo(ex - tx * 2 + Math.cos(ae) * 5, ey - ty * 2 + Math.sin(ae) * 5);
      ctx.lineTo(ex - tx * 2 - Math.cos(ae) * 5, ey - ty * 2 - Math.sin(ae) * 5);
      ctx.fill();
    }

    // pivot
    ctx.fillStyle = css('--surface'); ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(cx, cy, W * 0.55, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();

    // gravity cue
    ctx.strokeStyle = css('--text-muted'); ctx.lineWidth = 1;
    const gx = w - 18, gy = 14;
    ctx.beginPath(); ctx.moveTo(gx, gy); ctx.lineTo(gx, gy + 22); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(gx - 4, gy + 16); ctx.lineTo(gx, gy + 22); ctx.lineTo(gx + 4, gy + 16); ctx.stroke();
    ctx.fillStyle = css('--text-muted'); ctx.textAlign = 'right'; ctx.textBaseline = 'top';
    ctx.fillText('g', gx - 6, gy + 4);
  },

  // Problem data for the chapter modules. Statements are paraphrased; the page
  // numbers point at controlbook.pdf (PDF page, not book page).
  problems: {
    ch2: {
      id: 'A.2', page: 28,
      sim: { amplitude: 30, frequency: 0.25, tEnd: 8 },
      statement: [
        '(a) Using the configuration variable θ, write an expression for the kinetic energy of the system.',
        '(b) Write a class that animates the arm, and display a sinusoidal variation of θ.',
      ],
    },
    ch3: {
      id: 'A.3', page: 43,
      sim: { tEnd: 20, tStep: 0 },
      openLoop: { shape: 'square', amp: 0.2, freq: 0.05, offset: 0 },  // hw03_armSim.py
      statement: [
        '(a) Find the potential energy of the system.',
        '(b) Define the generalized coordinates.',
        '(c) Find the generalized forces and damping forces.',
        '(d) Derive the equations of motion with the Euler-Lagrange equations.',
        '(e) Implement the equations of motion and simulate with a variable torque input.',
      ],
    },
    ch4: {
      id: 'A.4', page: 62,
      sim: { tEnd: 6, tStep: 0 },
      statement: [
        '(a) Find the equilibria of the system.',
        '(b) Linearize about the equilibria using Jacobian linearization.',
        '(c) Linearize using feedback linearization.',
      ],
    },
    ch5: {
      id: 'A.5', page: 71,
      sim: { tEnd: 6, tStep: 0 },
      statement: ['Using the feedback-linearized model, find the transfer function from the torque τ̃ to the angle θ.'],
    },
    ch6: {
      id: 'A.6', page: 87,
      sim: { tEnd: 6, tStep: 0 },
      statement: ['With x = (θ, θ̇)ᵀ, ũ = τ̃ and y = θ, find A, B, C, D for the feedback-linearized equations (4.7).'],
    },
    ch9: {
      id: 'A.9', page: 145,
      sim: { amplitude: 30, tEnd: 12, dist: 0.1, tDist: 6 },
      statement: [
        '(a) With PD control, what is the system type? Characterize the steady-state error for a step, a ramp and a parabola. How does this change with an integrator?',
        '(b) A constant disturbance acts at the plant input (gravity, for example). What is the steady-state error with and without the integrator?',
      ],
    },
    ch10: {
      id: 'A.10', page: 161,
      tr: 0.6, zeta: 0.9, ki: 0.2, sigma: 0.05,
      sim: { type: 'square', amplitude: 30, frequency: 0.05, tStep: 0, tEnd: 20 },
      mismatch: { m: 12, ell: -9, b: 15 },  // a fixed "alpha = 0.2" draw so the page is repeatable
      statement: [
        '(a) Let the parameters vary by up to 20% (α = 0.2).',
        '(b) The controller may use only the measured angle θ and the reference θ<sub>r</sub>, not the state.',
        '(c) Implement the PID with dirty-derivative gain σ = 0.05, and tune the integrator to remove the steady-state error caused by the uncertain parameters.',
      ],
    },
    p6: {
      id: 'A.P.6', page: 470,
      sim: { amplitude: 30, tEnd: 12 },
      statement: [
        'Use the PD gains from A.8 and add an integrator to get PID control (derivative on the output). Put the closed-loop characteristic equation in Evans form, plot the root locus versus k<sub>I</sub>, and choose a k<sub>I</sub> that does not significantly move the other closed-loop poles.',
      ],
    },
    ch11: {
      id: 'A.11', page: 186,
      tr: 0.489, zeta: 0.707,
      sim: { type: 'square', amplitude: 30, frequency: 0.05, tStep: 0, tEnd: 10 },
      statement: [
        '(a) Choose the closed-loop poles from s² + 2ζω<sub>n</sub>s + ω<sub>n</sub>² with t<sub>r</sub> = 0.489 and ζ = 0.707.',
        '(b) Add the state-space matrices from A.6.',
        '(c) Check controllability: rank(𝒞<sub>A,B</sub>) = n.',
        '(d) Find K so eig(A − BK) are the desired poles, and k<sub>r</sub> so the DC gain from θ<sub>r</sub> to θ is one. If the poles match A.8, K = (k<sub>P</sub>, k<sub>D</sub>). Why?',
        '(e) Implement the state-feedback controller, using a digital differentiator to estimate θ̇.',
      ],
    },
    ch12: {
      id: 'A.12', page: 203,
      tr: 0.489, zeta: 0.707, pI: -5,
      sim: { type: 'square', amplitude: 30, frequency: 0.05, tStep: 0, tEnd: 20, dist: 0.25, tDist: 0 },
      mismatch: { m: 12, ell: -9, b: 15 },
      statement: [
        '(a) Add an integrator with anti-windup to the A.11 state-feedback controller.',
        '(b) Add a disturbance and let the parameters vary by up to 20%.',
        '(c) Tune the integrator pole (and other gains if needed) for good tracking.',
      ],
    },
    ch13: {
      id: 'A.13', page: 228,
      tr: 0.4, zeta: 0.707, pI: -9, trObsFactor: 10, zetaObs: 0.707,
      sim: { type: 'square', amplitude: 30, frequency: 0.05, tStep: 0, tEnd: 20, dist: 0, tDist: 0 },
      statement: [
        '(a) Use exact parameters (α = 0) and no input disturbance.',
        '(b) Check observability: rank(𝒪<sub>A,C</sub>) = n.',
        '(c) Add an observer that estimates x̂ and use x̂ in the controller. Tune the controller and observer poles.',
        '(d) Plot the state and the estimated state together.',
        '(e) Add an input disturbance of 0.01 and observe the steady-state error even with the integrator.',
      ],
    },
    ch14: {
      id: 'A.14', page: 246,
      tr: 0.4, zeta: 0.95, pI: -9, wnObs: 10, zetaObs: 0.707, pD: -5.5,
      sim: { type: 'square', amplitude: 30, frequency: 0.05, tStep: 0, tEnd: 20, dist: 0.5, tDist: 0, noise: 0.0573 },
      mismatch: { m: 12, ell: -9, b: 15 },
      statement: [
        '(a) Use α = 0.2, an input disturbance of 0.5 N·m, and output noise with standard deviation 0.001 rad. Without a disturbance observer, the controller cannot handle the large disturbance.',
        '(b) Add a disturbance observer, verify the estimator\'s steady-state error is removed, and tune.',
      ],
    },
    ch15: {
      id: 'A.15', page: 275,
      sim: { tEnd: 30, tStep: 0 },
      statement: ['Draw by hand the Bode plot from τ̃ to θ̃ with θ<sub>e</sub> = 0, then compare with the bode command.'],
    },
    ch16: {
      id: 'A.16', page: 295,
      wr: 0.4, parabolaA: 5, wdin: 0.01, wno: 100,
      sim: { type: 'square', amplitude: 30, frequency: 0.05, tStep: 0, tEnd: 20 },
      statement: [
        'Plot the Bode plots of the plant and of the plant under the A.10 PID control.',
        '(a) To what percent error can the closed loop track θ<sub>r</sub> if its frequency content is below ω<sub>r</sub> = 0.4 rad/s?',
        '(b) What is the steady-state tracking error to θ<sub>r</sub>(t) = 5t²?',
        '(c) If d<sub>in</sub> has content below 0.01 rad/s, what percent of it shows up in θ?',
        '(d) If the noise has content above 100 rad/s, what percent shows up in θ?',
      ],
    },
    ch17: {
      id: 'A.17', page: 311,
      sim: { type: 'square', amplitude: 30, frequency: 0.05, tStep: 0, tEnd: 20 },
      statement: [
        'Find the phase and gain margins under the A.10 PID control. Plot the open-loop and closed-loop Bode plots together. What is the closed-loop bandwidth, and how does it relate to the crossover frequency?',
      ],
    },
    ch18: {
      id: 'A.18', page: 340,
      wLow: 0.07, wHigh: 1000, factor: 10, pm: 60,
      sim: { type: 'square', amplitude: 30, frequency: 0.05, tStep: 0, tEnd: 20, dist: 0.1, tDist: 0, noise: 0.573 },
      mismatch: { m: 6, ell: -4, b: 8 },
      statement: [
        'C(s) = C<sub>pid</sub>(s)·C<sub>l</sub>(s), with C<sub>pid</sub> from A.10.',
        '(a) Design C<sub>l</sub>: improve tracking and disturbance rejection by 10× below 0.07 rad/s, improve noise attenuation by 10× above 1000 rad/s, and get PM ≈ 60°.',
        '(b) Add zero-mean Gaussian noise (σ = 0.01).',
        '(c) Implement C(s) in simulation using its state-space form.',
        '(d) Add a low-pass prefilter F(s) to flatten the closed-loop response and remove the overshoot.',
      ],
    },
    ch7: {
      id: 'A.7', page: 101,
      yE: 0,
      desiredPoles: [{ re: -3, im: 0 }, { re: -4, im: 0 }],
      sim: { amplitude: 30, tEnd: 4 },
      statement: [
        '(a) Using the transfer function from A.6, find the open-loop poles when θ<sub>e</sub> = 0.',
        '(b) With PD control on the error and derivative on the output (Fig. 7-2), find the closed-loop transfer function from θ<sub>r</sub> to θ and its characteristic polynomial in terms of k<sub>P</sub>, k<sub>D</sub>.',
        '(c) Pick k<sub>P</sub>, k<sub>D</sub> to place the closed-loop poles at −3 and −4.',
        '(d) Simulate the step response with your gains.',
      ],
    },
    ch8: {
      id: 'A.8', page: 121,
      tr: 0.8, zeta: 0.707,
      satStepDeg: 50,
      bookTr: 0.37,  // value in the book's / repo's ctrlPD.py (p. 122)
      sim: { amplitude: 50, tEnd: 4 },
      statement: [
        '(a) Rise time t<sub>r</sub> ≈ 0.8 s, damping ratio ζ = 0.707: find Δ<sup>d</sup><sub>cl</sub>(s), its poles, and k<sub>P</sub>, k<sub>D</sub>. Verify the step response.',
        '(b) With |τ| ≤ τ<sub>max</sub> = 1 N·m, use t<sub>r</sub> and ζ as tuning knobs so the input <em>just</em> saturates for a 50° step on θ<sub>r</sub>.',
      ],
    },
  },
};
