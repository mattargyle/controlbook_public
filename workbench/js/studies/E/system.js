// Design Study E: block on a beam (controlbook.pdf p. 385). Homework study: the
// book has no worked solutions for E, so every answer on these pages is derived
// here from the problem statements (see js/studies/E/ISSUES.md for ambiguities).
//
// State x = (z, θ, ż, θ̇), input F (force at the beam end), outputs y = (z, θ).
// f(x, F) reproduces _E_blockbeam/python/testDynamics.py to the last bit
// (tools/regress_E.py), operation for operation.
//
// Three different "equilibrium" ideas live in this study, and the pages keep them apart:
//   x_e  = (z_e, 0, 0, 0) with z_e = ℓ/2: the operating point for linearization (E.4, E.8a)
//   F_e  = m1 g z_e/ℓ + m2 g/2: the constant force that holds the beam level at z_e
//   F_fl(z) = m1 g z/ℓ + m2 g/2: feedback linearization with the measured z (E.8e)
window.WB = window.WB || {};
WB.systems = WB.systems || {};

(function () {
  const DEG = Math.PI / 180;

  // Operating point and linear-model helpers (E.4 p. 386, E.5 pp. 386–387, E.6 p. 387).
  const ze = (p) => p.ell / 2;
  // Beam + block inertia about the pivot with the block at z: m2ℓ²/3 + m1 z²
  const inertia = (p, z) => p.m2 * p.ell ** 2 / 3 + p.m1 * z ** 2;
  const Fe = (p, z = ze(p)) => p.m1 * p.g * z / p.ell + p.m2 * p.g / 2;
  const Ffl = (z, p) => p.m1 * p.g * z / p.ell + p.m2 * p.g / 2;

  // Jacobian linearization about (z_e, 0, 0, 0), F_e. With comp = 'fl' the
  // m1 g z̃ term is cancelled by F_fl(z) and A[3][0] = 0 (the E.5(c) simplification).
  function linear(p, { z = ze(p), comp = 'eq' } = {}) {
    const De = inertia(p, z);
    return {
      A: [[0, 0, 1, 0], [0, 0, 0, 1], [0, -p.g, 0, 0], [comp === 'fl' ? 0 : -p.m1 * p.g / De, 0, 0, 0]],
      B: [[0], [0], [0], [p.ell / De]],
      C: [[1, 0, 0, 0], [0, 1, 0, 0]],
      D: [[0], [0]],
      De, b0: p.ell / De, ze: z, Fe: Fe(p, z),
    };
  }

  WB.systems.E = {
    id: 'E',
    name: 'Block on Beam',
    introPage: 385,
    sym: { y: 'z', u: 'F', yText: 'z', uText: 'F' },

    params: [
      { key: 'm1', label: 'm<sub>1</sub>', unit: 'kg', value: 0.35, min: 0.05, max: 1.5, step: 0.005 },
      { key: 'm2', label: 'm<sub>2</sub>', unit: 'kg', value: 2, min: 0.5, max: 6, step: 0.01 },
      { key: 'ell', label: 'ℓ', unit: 'm', value: 0.5, min: 0.2, max: 1.5, step: 0.005 },
      { key: 'F_max', label: 'F<sub>max</sub>', unit: 'N', value: 15, min: 5, max: 60, step: 0.5 },
    ],
    constants: { g: 9.8 },
    uncertain: ['m1', 'm2', 'ell'],

    outputs: [
      { key: 'z', label: 'z', unit: 'm', scale: 1, minSpan: 0.05, noiseMax: 0.01 },
      { key: 'theta', label: 'θ', unit: '°', scale: 180 / Math.PI, minSpan: 2, noiseMax: 1 },
    ],
    inputs: [{ key: 'F', label: 'F', unit: 'N' }],
    // The reference is z̃_r = z_r − z_e; the plotted z_r adds z_e = ℓ/2 = 0.25 m (book ℓ).
    refs: [{
      output: 0, label: 'z̃ᵣ (zᵣ = 0.25 + z̃ᵣ)', unit: 'm', scale: 1, offset: 0.25, min: -0.25, max: 0.25, step: 0.005,
      defaults: { type: 'square', amplitude: 0.15, frequency: 0.05, tStep: 0 },
    }],
    initial: [
      { key: 'z0', label: 'z(0)', unit: 'm', scale: 1, output: 0, value: 0.25, min: -0.25, max: 0.75, step: 0.005 },
      { key: 'theta0', label: 'θ(0)', unit: '°', scale: 180 / Math.PI, output: 1, value: 0, min: -30, max: 30, step: 0.5 },
    ],
    x0(init) { return [init.z0, init.theta0, 0, 0]; },
    disturbances: [{ input: 0, label: 'd (input force)', unit: 'N', min: -3, max: 3, step: 0.05 }],
    simBase: { type: 'square', amplitude: 0.15, frequency: 0.05, tStep: 0, tEnd: 40, tDist: 0 },
    stateLabels: ['z', 'θ', 'ż', 'θ̇'],

    // Equations of motion (E.3, p. 386). Written in the same order as the repo's
    // solution so testDynamics.py's 1e-14 tolerance holds.
    f(x, F, p) {
      const z = x[0], theta = x[1], zdot = x[2], thetadot = x[3];
      const zddot = (1.0 / p.m1) * (p.m1 * z * thetadot ** 2 - p.m1 * p.g * Math.sin(theta));
      const thetaddot = (1.0 / ((p.m2 * p.ell ** 2) / 3.0 + p.m1 * z ** 2))
        * (F * p.ell * Math.cos(theta) - 2.0 * p.m1 * z * zdot * thetadot
          - p.m1 * p.g * z * Math.cos(theta) - p.m2 * p.g * p.ell / 2.0 * Math.cos(theta));
      return [zdot, thetadot, zddot, thetaddot];
    },
    h(x) { return [x[0], x[1]]; },
    uLimit(p) { return p.F_max; },

    // E.2: K = ½ m1 (ż² + z²θ̇²) + ½ (m2ℓ²/3) θ̇²; E.3: P = (m1 g z + m2 g ℓ/2) sin θ (P = 0 at θ = 0).
    kinetic(x, p) { return 0.5 * p.m1 * (x[2] ** 2 + x[0] ** 2 * x[3] ** 2) + 0.5 * (p.m2 * p.ell ** 2 / 3) * x[3] ** 2; },
    potential(x, p) { return (p.m1 * p.g * x[0] + p.m2 * p.g * p.ell / 2) * Math.sin(x[1]); },

    ze, inertia, Fe, Ffl, linear,
    // app.js reads stateSpace for ctx.ss: the Jacobian model of E.6 at z_e.
    stateSpace(p) { const l = linear(p); return { A: l.A, B: l.B, C: l.C, D: l.D }; },

    // Beam pivoted at the left, block at distance z along it, force F up at the tip.
    draw(ctx, w, h, s) {
      const { css } = WB.plot;
      const { x, p } = s;
      const ell = p.ell;
      const z = x[0], th = x[1];
      const F = s.uAll[0], sat = s.satAll[0];
      const Fmax = Math.max(Math.abs(s.ranges[0][0]), Math.abs(s.ranges[0][1]));
      const px = w * 0.16, py = h * 0.58;
      const k = Math.min(w * 0.66, h * 1.1) / ell;           // pixels per metre
      const along = (d, off = 0) => [px + d * k * Math.cos(th) - off * Math.sin(th), py - d * k * Math.sin(th) - off * Math.cos(th)];

      // level line and θ arc
      ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.setLineDash([3, 4]);
      ctx.beginPath(); ctx.moveTo(px - 20, py); ctx.lineTo(px + ell * k * 1.08, py); ctx.stroke(); ctx.setLineDash([]);
      ctx.font = '11px var(--font-sans, system-ui)';

      // reference position z_r: a tick under the beam
      const zr = s.rAll[0];
      if (isFinite(zr)) {
        const [rx, ry] = along(zr, -9);
        ctx.fillStyle = css('--text-muted');
        ctx.beginPath(); ctx.moveTo(rx, ry - 6); ctx.lineTo(rx - 5, ry + 3); ctx.lineTo(rx + 5, ry + 3); ctx.fill();
        ctx.textAlign = 'center'; ctx.textBaseline = 'top'; ctx.fillText('zᵣ', rx, ry + 5);
      }

      // beam
      const T = Math.max(4, k * 0.012);
      ctx.save(); ctx.translate(px, py); ctx.rotate(-th);
      ctx.fillStyle = css('--text-secondary');
      ctx.fillRect(0, -T / 2, ell * k, T);
      // block (point mass near the surface, drawn 5 cm wide like blockbeamParam's width)
      const bw = Math.max(14, 0.05 * k), bh = bw * 0.6;
      ctx.fillStyle = css('--series-1');
      ctx.fillRect(z * k - bw / 2, -T / 2 - bh, bw, bh);
      ctx.restore();

      // force arrow at the tip, length ∝ F / F_max
      const [tx, ty] = along(ell);
      const frac = Math.max(-1, Math.min(1, F / Fmax));
      if (Math.abs(frac) > 0.005) {
        const Lp = frac * h * 0.38;
        ctx.strokeStyle = css(sat ? '--critical' : '--series-2'); ctx.fillStyle = ctx.strokeStyle; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(tx, ty - Lp); ctx.stroke();
        const dir = Math.sign(Lp);
        ctx.beginPath(); ctx.moveTo(tx, ty - Lp - dir * 7); ctx.lineTo(tx - 5, ty - Lp); ctx.lineTo(tx + 5, ty - Lp); ctx.fill();
        ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
        ctx.fillText('F', tx + 8, ty - Lp / 2);
      }

      // pivot
      ctx.fillStyle = css('--surface'); ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(px, py, 5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(px - 8, py + 14); ctx.lineTo(px, py + 5); ctx.lineTo(px + 8, py + 14); ctx.closePath(); ctx.stroke();

      // gravity cue
      ctx.strokeStyle = css('--text-muted'); ctx.lineWidth = 1;
      const gx = w - 18, gy = 14;
      ctx.beginPath(); ctx.moveTo(gx, gy); ctx.lineTo(gx, gy + 22); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gx - 4, gy + 16); ctx.lineTo(gx, gy + 22); ctx.lineTo(gx + 4, gy + 16); ctx.stroke();
      ctx.fillStyle = css('--text-muted'); ctx.textAlign = 'right'; ctx.textBaseline = 'top';
      ctx.fillText('g', gx - 6, gy + 4);
    },

    // Problem data. Statements are paraphrased; pages are controlbook.pdf pages.
    // There is no E.7 in the book.
    problems: {
      ch2: {
        id: 'E.2', page: 386,
        sim: { tEnd: 10 },
        statement: [
          '(a) Using the configuration variables z and θ, write an expression for the kinetic energy of the system. Assume that the block is a point mass with mass concentrated near the surface of the beam (i.e. it is not a tall block).',
          '(b) Create an animation of the block on beam system. The inputs should be z and θ. Turn in a screen capture of the animation.',
        ],
      },
      ch3: {
        id: 'E.3', page: 386,
        sim: { tEnd: 3, y0: 0.25, tDist: 0 },
        statement: [
          '(a) Find the potential energy for the system.',
          '(b) Define the generalized coordinates.',
          '(c) Find the generalized forces and damping forces.',
          '(d) Derive the equations of motion using the Euler-Lagrange equations.',
          '(e) Referring to Appendices P.1, P.2, and P.3, write a class or s-function that implements the equations of motion. Simulate the system using a variable force input. The output should connect to the animation function developed in homework E.2.',
        ],
      },
      ch4: {
        id: 'E.4', page: 386,
        sim: { tEnd: 2, tDist: 0 },
        statement: [
          '(a) Find the equilibria of the system.',
          '(b) Linearize the system about the equilibria using Jacobian linearization.',
          '(c) If possible, linearize the system using feedback linearization.',
        ],
      },
      ch5: {
        id: 'E.5', page: 386,
        sim: { tEnd: 3, tDist: 0 },
        statement: [
          '(a) Start with the linearized equations and use the Laplace transform to convert the equations of motion to the s-domain.',
          '(b) Find the transfer functions from the input F̃(s) to the outputs Z̃(s) and Θ̃(s). Find the transfer function from the input Θ̃(s) to the output Z̃(s). (p. 387)',
          '(c) Assume that deviations in the gravity torque away from the equilibrium torque due to motions of the block along the beam are small compared to the torque required to hold up the beam. This will allow you to ignore the m<sub>1</sub>g z̃ term in the second equation of motion. How does this simplify your transfer functions?',
          '(d) Draw a block diagram of your simplified system transfer functions in a cascade from the input F̃(s) to the intermediate state Θ̃(s) and then to the output Z̃(s).',
        ],
      },
      ch6: {
        id: 'E.6', page: 387,
        sim: { tEnd: 3, tDist: 0 },
        statement: ['Defining the states as x̃ = (z̃, θ̃, z̃̇, θ̃̇)ᵀ, the input as ũ = F̃, and the measured output as ỹ = (z̃, θ̃)ᵀ, find the linear state space equations in the form x̃̇ = Ax̃ + Bũ, ỹ = Cx̃ + Dũ.'],
      },
      ch8: {
        id: 'E.8', page: 387,
        trTh: 1, zetaTh: 0.707, M: 10, zetaZ: 0.707, stepZ: 0.25,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.01, tStep: 0, tEnd: 100 },
        statement: [
          '(a) Draw a successive-loop-closure block diagram with PD control in both loops. For design, take z<sub>e</sub> = ℓ/2. The outer loop maps z<sub>r</sub> to θ<sub>r</sub>; the inner loop maps θ<sub>r</sub> to F̃.',
          '(b) Inner loop: find k<sub>P<sub>θ</sub></sub>, k<sub>D<sub>θ</sub></sub> for t<sub>r<sub>θ</sub></sub> = 1 s and ζ<sub>θ</sub> = 0.707.',
          '(c) Find the inner-loop DC gain k<sub>DC<sub>θ</sub></sub>.',
          '(d) Replace the inner loop by its DC gain; find k<sub>P<sub>z</sub></sub>, k<sub>D<sub>z</sub></sub> for t<sub>r<sub>z</sub></sub> = 10 t<sub>r<sub>θ</sub></sub> and ζ<sub>z</sub> = 0.707.',
          '(e) Simulate with z<sub>r</sub> a square wave of 0.25 ± 0.15 m at 0.01 Hz, using the actual block position in the feedback-linearizing equilibrium force. (p. 388)',
          '(f) With |F| ≤ F<sub>max</sub> = 15 N, use the outer rise time to get the fastest response that does not saturate for a 0.25 m step on z̃<sub>r</sub>.',
        ],
      },
      ch9: {
        id: 'E.9', page: 388,
        sim: { type: 'step', amplitude: 0.1, tStep: 1, tEnd: 60, dist: 0, tDist: 30 },
        statement: [
          '(a) Inner loop under PD: what is the system type? Characterize the steady-state error for a step, ramp and parabola on θ<sup>d</sup>. What is the type with respect to an input disturbance?',
          '(b) Outer loop under PD: the type, and the steady-state error for a step, ramp and parabola on z<sup>d</sup>. How does an integrator change this? What is the type for an input disturbance, for PD and for PID?',
        ],
      },
      p6: {
        id: 'E.P.6', page: 388,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.01, tStep: 0, tEnd: 100 },
        statement: [
          'Add an integrator to the outer loop (PID). Put the characteristic equation in Evans form and plot the root locus versus k<sub>I</sub>. Because k<sub>I</sub> will be negative, negate the Evans-form transfer function (rlocus plots only k<sub>I</sub> > 0). Choose a k<sub>I</sub> that does not move the other closed-loop poles much.',
        ],
      },
      ch10: {
        id: 'E.10', page: 388,
        sigma: 0.05,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.01, tStep: 0, tEnd: 100 },
        mismatch: { m1: 12, m2: -9, ell: 15 },  // a fixed α = 0.2 draw so the page is repeatable
        statement: [
          '(a) Let m<sub>1</sub>, m<sub>2</sub> and ℓ vary by up to 20% (α = 0.2).',
          '(b) The controller may use only the measured z, θ and z<sub>r</sub>. Implement the nested PID loops of E.8 with dirty-derivative gain σ = 0.05, and tune the integrators to remove the steady-state error. (p. 389)',
          '(c) The integrator gain will be negative. Replace the old anti-windup scheme with one that integrates only when |ż| is small.',
        ],
      },
      ch11: {
        id: 'E.11', page: 389,
        trTh: 0.5, zetaTh: 0.8, trZ: 1.5, zetaZ: 0.8,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.05, tStep: 0, tEnd: 40 },
        statement: [
          '(a) With ω<sub>n<sub>z</sub></sub> and ζ<sub>z</sub> from E.8, choose four closed-loop poles whose natural frequency exceeds ω<sub>n<sub>z</sub></sub> and whose damping ratio exceeds ζ<sub>z</sub>.',
          '(b) Use A, B, C, D from E.6. (c) Verify controllability: rank 𝒞<sub>A,B</sub> = n.',
          '(d) Find K so eig(A − BK) are the desired poles, and k<sub>r</sub> so the DC gain from z<sub>r</sub> to z is one.',
          '(e) Simulate and tune the poles; note how pole locations trade speed against control effort.',
        ],
      },
      ch12: {
        id: 'E.12', page: 389,
        trTh: 0.2, zetaTh: 0.8, trZ: 0.7, zetaZ: 0.85, pI: -2,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.05, tStep: 0, tEnd: 40, dist: 1, tDist: 0 },
        mismatch: { m1: 12, m2: -9, ell: 15 },
        statement: [
          '(a) Add an integrator with anti-windup on z to the E.11 state feedback.',
          '(b) Add a constant 1 N input disturbance and let the parameters vary by up to 20%.',
          '(c) Tune the integrator pole (and other gains if needed) for good tracking.',
        ],
      },
      ch13: {
        id: 'E.13', page: 390,
        trTh: 0.5, zetaTh: 0.8, trZ: 1.5, zetaZ: 0.8, pI: -1, obsFactor: 5, zetaObs: 0.8,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.05, tStep: 0, tEnd: 40, dist: 0, tDist: 0 },
        statement: [
          '(a) Use exact parameters (α = 0) and no input disturbance.',
          '(b) Verify observability: rank 𝒪<sub>A,C</sub> = n.',
          '(c) Add an observer for x̂ and use x̂ in the E.12 controller. Tune the controller and observer poles.',
          '(d) Output both u and x̂, and plot the state and its estimate together.',
          '(e) Add a 0.5 N input disturbance and observe the steady-state error even with the integrator, caused by the steady-state observation error.',
        ],
      },
      ch14: {
        id: 'E.14', page: 390,
        trTh: 0.5, zetaTh: 0.8, trZ: 1.5, zetaZ: 0.8, pI: -1, obsFactor: 5, zetaObs: 0.8, pD: -5,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.05, tStep: 0, tEnd: 40, dist: 0.5, tDist: 0, noise: 0.001, noises: [0.0573] },
        mismatch: { m1: 12, m2: -9, ell: 15 },
        statement: [
          '(a) Use α = 0.2, an input disturbance of 0.5, and noise with standard deviation 0.001 on z<sub>m</sub> and θ<sub>m</sub>.',
          '(b) Add a disturbance observer, verify that the steady-state estimation error is removed, and tune for a good response.',
        ],
      },
      ch15: {
        id: 'E.15', page: 390,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.01, tStep: 0, tEnd: 100 },
        statement: [
          '(a) Draw by hand the Bode plot of the inner-loop transfer function from F̃ to θ̃, then compare with bode.',
          '(b) Draw by hand the Bode plot of the outer-loop transfer function from θ̃ to z̃, then compare with bode. (p. 391)',
        ],
      },
      ch16: {
        id: 'E.16', page: 391,
        wr: 1.0, wdin: 0.8, wno: 300, wdout: 0.1, wsin: 0.6, Asin: 2,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.01, tStep: 0, tEnd: 100 },
        statement: [
          'Inner loop: Bode plots of the plant and of the plant under PD control with the E.8 gains (dirty derivative).',
          '(a) To what percent error can the inner loop track θ<sub>r</sub> if its frequency content is below ω<sub>r</sub> = 1.0 rad/s?',
          '(b) If the input disturbance has content below ω<sub>d<sub>in</sub></sub> = 0.8 rad/s, what percentage shows up in the output?',
          '(c) If the noise has content above ω<sub>no</sub> = 300 rad/s, what percentage shows up in θ?',
          'Outer loop: Bode plots of the plant and of the plant under PID control with the E.10 gains (dirty derivative).',
          '(d) If an output disturbance has content below ω<sub>d<sub>out</sub></sub> = 0.1 rad/s, what percentage remains in the output?',
          '(e) For y<sub>r</sub>(t) = 2 sin(0.6t), what is the output error?',
        ],
      },
      ch17: {
        id: 'E.17', page: 391,
        sim: { type: 'square', amplitude: 0.15, frequency: 0.01, tStep: 0, tEnd: 100 },
        statement: [
          'Use the E.10 gains.',
          '(a) Inner loop under PD: find the phase and gain margins with bode and margin. Plot the open- and closed-loop Bode plots together. What is the closed-loop bandwidth, and how does it relate to crossover?',
          '(b) Outer loop under PID: the same questions, on the same plot as the inner loop. (p. 392)',
          '(c) What is the bandwidth separation between the inner and outer loops? Is successive loop closure justified?',
        ],
      },
      ch18: {
        id: 'E.18', page: 392,
        inner: { wr: 1, gr: 0.0032, wn: 1000, gn: 0.0032, pm: 60 },
        outer: { wr: 0.1, gr: 0.01, wn: 100, gn: 0.001, pm: 60 },
        sim: { type: 'square', amplitude: 0.15, frequency: 0.05, tStep: 0, tEnd: 40, dist: 0, tDist: 20 },
        statement: [
          '(a) Inner loop: with P<sub>in</sub>(s) from E.5, design C<sub>in</sub>(s) for PM ≈ 60°, tracking error γ<sub>r</sub> = 0.0032 below ω<sub>r</sub> = 1 rad/s, and noise above 1000 rad/s attenuated by γ<sub>n</sub> = 0.0032.',
          '(b) Outer loop: the plant is P = P<sub>out</sub>·P<sub>in</sub>C<sub>in</sub>/(1 + P<sub>in</sub>C<sub>in</sub>). Design C<sub>out</sub> for PM ≈ 60°, rejection of constant input disturbances, tracking error γ<sub>r</sub> = 0.01 below 0.1 rad/s, and noise above 100 rad/s attenuated by γ<sub>n</sub> = 0.001. Add a prefilter F(s) to reduce closed-loop peaking.',
          '(c) Implement C(s) and F(s) in simulation using their state-space forms.',
        ],
      },
    },
  };

  WB.systems.E.DEG = DEG;
})();
