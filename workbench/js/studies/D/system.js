// Design Study D: mass-spring-damper (controlbook.pdf p. 377, homework D.2-D.18).
// The physics matches the f(x, u) that _D_mass/python/testDynamics.py expects:
//   m z'' = F - b z' - k z,   x = (z, z'),   u = F,   y = z.
//
// There are no book solutions for D. Everything under `answers` below was derived
// from the problem statements (pp. 377-383) and is cross-checked by
// tools/regress_D.py against python-control. The chapter files show these values
// only behind Check / Show solution / Reveal in Work mode.
window.WB = window.WB || {};
WB.systems = WB.systems || {};
WB.studies = WB.studies || {};
WB.studies.D = WB.studies.D || { chapters: {} };

(function () {
  const M = WB.math;
  const L = WB.la;

  // ------------------------------------------------------------ answer key --
  // Closed-form results in terms of the nominal parameters p = {m, k, b, Fmax}.
  // Kept free of UI code so tools/regress_D.py can call them directly.
  const answers = {
    // D.5: P(s) = Z(s)/F(s) = (1/m) / (s^2 + (b/m) s + k/m)
    tf(p) { return { b0: 1 / p.m, a1: p.b / p.m, a0: p.k / p.m }; },
    // D.6: x = (z, z'), u = F, y = z
    ss(p) { return { A: [[0, 1], [-p.k / p.m, -p.b / p.m]], B: [[0], [1 / p.m]], C: [[1, 0]], D: [[0]] }; },
    // D.7(a): roots of s^2 + (b/m) s + k/m
    olPoles(p) { const t = answers.tf(p); return M.roots2(t.a1, t.a0); },
    // D.7(c)/D.8: PD gains for any desired pole pair (Eq. 7.5 matched coefficient by coefficient)
    pdGains(p, poles) {
      const t = answers.tf(p);
      const { alpha1, alpha0 } = M.polyFromPoles(poles[0], poles[1]);
      return { kP: (alpha0 - t.a0) / t.b0, kD: (alpha1 - t.a1) / t.b0, alpha1, alpha0 };
    },
    // D.8(a): t_r = 2, zeta = 0.7, omega_n = 2.2 / t_r (Eq. 8.5)
    spec(p, tr, zeta) {
      const wn = 2.2 / tr;
      const poles = WB.design.polesFromWnZeta(wn, zeta);
      return { wn, poles, ...answers.pdGains(p, poles) };
    },
    // D.8(b): u = kP e - kD z' is largest right after the step (z' = 0, e = step),
    // so kP <= Fmax / step (Eq. 8.8). Then omega_n^2 = a0 + b0 kP and t_r = 2.2 / omega_n.
    satLimit(p, step, zeta) {
      const t = answers.tf(p);
      const kP = p.Fmax / step;
      const wn = Math.sqrt(t.a0 + t.b0 * kP);
      return { kP, wn, tr: 2.2 / wn, kD: (2 * zeta * wn - t.a1) / t.b0 };
    },
    // D.9: system type with C = kP + kI/s + kD s on the type-0 plant.
    // Fig. 7-1 (unity feedback) and Fig. 7-2 (D on output) give the same limits here.
    typeAnalysis(p, g) {
      if (g.kI > 0) {
        return { type: 1, Mp: Infinity, Mv: g.kI / p.k, step: 0, ramp: p.k / g.kI, parab: Infinity, distType: 1, dist: 0 };
      }
      const Mp = g.kP / p.k;
      return { type: 0, Mp, Mv: 0, step: 1 / (1 + Mp), ramp: Infinity, parab: Infinity, distType: 0, dist: 1 / (p.k + g.kP) };
    },
    // D.P.6: Delta_cl(s) = s^3 + (a1 + b0 kD) s^2 + (a0 + b0 kP) s + b0 kI  ->  1 + kI L(s) = 0
    evans(p, g) {
      const t = answers.tf(p);
      const den = [1, t.a1 + t.b0 * g.kD, t.a0 + t.b0 * g.kP, 0];
      return { den, num: [t.b0], kCrit: den[1] * den[2] / t.b0 };
    },
    // D.11: companion form, so K = (kP, kD) for the same poles; kr = -1/(C (A-BK)^-1 B) = alpha0/b0
    stateFeedback(p, poles) {
      const t = answers.tf(p);
      const { alpha1, alpha0 } = M.polyFromPoles(poles[0], poles[1]);
      return { K: [(alpha0 - t.a0) / t.b0, (alpha1 - t.a1) / t.b0], kr: alpha0 / t.b0 };
    },
    // D.12: det(sI - (A1 - B1 K1)) = s^3 + (a1 + b0 K2) s^2 + (a0 + b0 K1) s - b0 kI
    integralFeedback(p, poles3) {
      const t = answers.tf(p);
      const c = L.polyFromRoots(poles3);  // [1, c2, c1, c0]
      return { K: [(c[2] - t.a0) / t.b0, (c[1] - t.a1) / t.b0], ki: -c[3] / t.b0 };
    },
    // D.13: det(sI - (A - LC)) = s^2 + (a1 + L1) s + (a0 + a1 L1 + L2)
    observer(p, q2) {
      const t = answers.tf(p);
      const { alpha1: b1, alpha0: b0 } = M.polyFromPoles(q2[0], q2[1]);
      const L1 = b1 - t.a1;
      return { L: [L1, b0 - t.a0 - t.a1 * L1] };
    },
    // D.14: A2 = [A B; 0 0], C2 = [C 0]:
    // det(sI - (A2 - L C2)) = s^3 + (a1 + L1) s^2 + (a0 + a1 L1 + L2) s + b0 Ld
    disturbanceObserver(p, q3) {
      const t = answers.tf(p);
      const c = L.polyFromRoots(q3);
      const L1 = c[1] - t.a1;
      return { L: [L1, c[2] - t.a0 - t.a1 * L1], Ld: c[3] / t.b0 };
    },
    // D.15: P(jw) = (1/k) / (1 - (w/wn)^2 + j 2 zeta w/wn), wn = sqrt(k/m), zeta = b / (2 sqrt(k m))
    bode(p) {
      const wn = Math.sqrt(p.k / p.m), zeta = p.b / (2 * Math.sqrt(p.k * p.m));
      const dc = 1 / p.k;
      return { dc, dcDb: 20 * Math.log10(dc), wn, zeta, peak: dc / (2 * zeta), peakDb: 20 * Math.log10(dc / (2 * zeta)), slopeHi: -40 };
    },
  };

  WB.systems.D = {
    id: 'D',
    name: 'Mass Spring Damper',
    introPage: 377,
    answers,

    // Nominal physical parameters (p. 377); F_max from D.8(b), p. 379.
    params: [
      { key: 'm', label: 'm', unit: 'kg', value: 5, min: 0.5, max: 20, step: 0.05 },
      { key: 'k', label: 'k', unit: 'N/m', value: 3, min: 0, max: 20, step: 0.05 },
      { key: 'b', label: 'b', unit: 'N·s/m', value: 0.5, min: 0, max: 5, step: 0.01 },
      { key: 'Fmax', label: 'F<sub>max</sub>', unit: 'N', value: 6, min: 1, max: 50, step: 0.5 },
    ],
    constants: {},
    // massDynamics(alpha) perturbs m, k and b (D.10a, p. 380)
    uncertain: ['m', 'k', 'b'],

    outputs: [{ key: 'z', label: 'z', unit: 'm', scale: 1, minSpan: 0.2, noiseMax: 0.01 }],
    inputs: [{ key: 'F', label: 'F', unit: 'N' }],
    refs: [{ output: 0, label: 'reference z_r', unit: 'm', scale: 1, min: -2, max: 2, step: 0.01 }],
    initial: [
      { key: 'z0', label: 'z(0)', unit: 'm', scale: 1, output: 0, value: 0, min: -2, max: 2, step: 0.01 },
      { key: 'zd0', label: 'ż(0)', unit: 'm/s', scale: 1, value: 0, min: -2, max: 2, step: 0.01 },
    ],
    disturbances: [{ input: 0, label: 'd (input force)', unit: 'N', min: -2, max: 2, step: 0.01 }],
    simBase: { amplitude: 1, tStep: 0.5, tEnd: 20, frequency: 0.05, tDist: 10 },
    x0(init) { return [init.z0, init.zd0 || 0]; },
    stateLabels: ['z', 'ż'],

    f(x, F, p) {
      const z = x[0], zdot = x[1];
      const zddot = (F - p.b * zdot - p.k * z) / p.m;
      return [zdot, zddot];
    },
    h(x) { return x[0]; },
    uLimit(p) { return p.Fmax; },

    secondOrderModel(p) {
      const t = answers.tf(p);
      return { ...t, tex: { b0: '\\frac{1}{m}', a1: '\\frac{b}{m}', a0: '\\frac{k}{m}' } };
    },
    stateSpace(p) { return answers.ss(p); },
    kinetic(x, p) { return 0.5 * p.m * x[1] ** 2; },
    potential(x, p) { return 0.5 * p.k * x[0] ** 2; },

    // Animation: wall, spring and damper on the left, the mass at z, force arrow.
    draw(g, w, h, s) {
      const { css } = WB.plot;
      const z = s.x[0];
      const r = s.rAll ? s.rAll[0] : s.r;
      const F = s.uAll ? s.uAll[0] : s.u;
      const lim = s.ranges ? s.ranges[0][1] : s.uLimit;
      const sat = s.satAll ? s.satAll[0] : s.saturated;
      const ppm = Math.min(w / 6.2, h / 1.6);            // pixels per meter
      const x0 = w * 0.42;                               // screen x of z = 0 (left face of the mass)
      const ground = h * 0.74;
      const H = Math.min(0.55 * ppm, h * 0.36), W = Math.min(0.7 * ppm, w * 0.16);
      const wallX = Math.max(14, x0 - 1.35 * ppm);
      const X = (zz) => Math.max(wallX + 12, Math.min(w - W - 6, x0 + zz * ppm));
      g.lineCap = 'round';

      // ground with ticks every 0.5 m
      g.strokeStyle = css('--text-secondary'); g.lineWidth = 1.5;
      g.beginPath(); g.moveTo(wallX, ground); g.lineTo(w - 4, ground); g.stroke();
      g.fillStyle = css('--text-muted'); g.font = '10px system-ui, sans-serif';
      g.textAlign = 'center'; g.textBaseline = 'top';
      g.strokeStyle = css('--grid'); g.lineWidth = 1;
      for (let zz = -2; zz <= 6; zz += 0.5) {
        const px = x0 + zz * ppm;
        if (px < wallX + 6 || px > w - 8) continue;
        g.beginPath(); g.moveTo(px, ground); g.lineTo(px, ground + (zz % 1 === 0 ? 6 : 3)); g.stroke();
        if (zz % 1 === 0) g.fillText(`${zz}`, px, ground + 8);
      }
      g.textAlign = 'right'; g.fillText('z [m]', w - 6, ground + 20);

      // wall
      g.strokeStyle = css('--text-secondary'); g.lineWidth = 2;
      g.beginPath(); g.moveTo(wallX, ground); g.lineTo(wallX, ground - H * 1.5); g.stroke();
      g.lineWidth = 1;
      for (let yy = ground - H * 1.5; yy < ground; yy += 8) { g.beginPath(); g.moveTo(wallX, yy); g.lineTo(wallX - 7, yy + 7); g.stroke(); }

      // reference ghost
      const xr = X(r);
      g.strokeStyle = css('--text-muted'); g.lineWidth = 1.5; g.setLineDash([5, 4]);
      g.strokeRect(xr, ground - H, W, H); g.setLineDash([]);
      g.fillStyle = css('--text-muted'); g.textAlign = 'center'; g.textBaseline = 'bottom';
      g.fillText('z_r', xr + W / 2, ground - H - 3);

      const xm = X(z);
      // spring (zigzag), upper attachment
      const ys = ground - H * 0.7, n = 8, amp = Math.min(8, H * 0.12);
      const s0 = wallX, s1 = xm, lead = Math.min(10, (s1 - s0) * 0.12);
      g.strokeStyle = css('--series-3'); g.lineWidth = 2;
      g.beginPath(); g.moveTo(s0, ys); g.lineTo(s0 + lead, ys);
      for (let i = 1; i <= 2 * n; i++) {
        const px = s0 + lead + (s1 - s0 - 2 * lead) * i / (2 * n);
        g.lineTo(px, i === 2 * n ? ys : ys + (i % 2 ? -amp : amp));
      }
      g.lineTo(s1, ys); g.stroke();
      g.fillStyle = css('--text-secondary'); g.textAlign = 'center'; g.textBaseline = 'bottom';
      g.fillText('k', (s0 + s1) / 2, ys - amp - 2);

      // damper (dashpot), lower attachment: cylinder on the wall, piston on the mass
      const yd = ground - H * 0.28, cw = Math.min(14, H * 0.2);
      const cylL = s0 + (s1 - s0) * 0.3, cylR = s0 + (s1 - s0) * 0.62;
      g.strokeStyle = css('--text-secondary'); g.lineWidth = 2;
      g.beginPath(); g.moveTo(s0, yd); g.lineTo(cylL, yd);
      g.moveTo(cylR, yd - cw / 2); g.lineTo(cylL, yd - cw / 2); g.lineTo(cylL, yd + cw / 2); g.lineTo(cylR, yd + cw / 2);
      const pist = cylL + (cylR - cylL) * 0.55;
      g.moveTo(pist, yd - cw / 2 + 2); g.lineTo(pist, yd + cw / 2 - 2);
      g.moveTo(pist, yd); g.lineTo(s1, yd); g.stroke();
      g.fillText('b', (cylL + cylR) / 2, yd - cw / 2 - 2);

      // the mass
      g.fillStyle = css('--series-1');
      g.beginPath(); g.roundRect(xm, ground - H, W, H, 3); g.fill();
      g.fillStyle = css('--surface'); g.textBaseline = 'middle'; g.font = '12px system-ui, sans-serif';
      g.fillText('m', xm + W / 2, ground - H / 2);

      // force arrow, length proportional to |F| / Fmax
      const frac = Math.max(-1, Math.min(1, F / (lim || 1)));
      if (Math.abs(frac) > 0.01) {
        const len = frac * Math.min(0.9 * ppm, w * 0.2);
        const ya = ground - H * 0.5;
        const xa = frac > 0 ? xm + W : xm;
        const xe = xa + len;
        g.strokeStyle = g.fillStyle = css(sat ? '--critical' : '--series-2'); g.lineWidth = 3;
        g.beginPath(); g.moveTo(xa, ya); g.lineTo(xe, ya); g.stroke();
        const dir = Math.sign(len);
        g.beginPath(); g.moveTo(xe + dir * 7, ya); g.lineTo(xe - dir * 2, ya - 6); g.lineTo(xe - dir * 2, ya + 6); g.fill();
        g.font = '11px system-ui, sans-serif'; g.textBaseline = 'bottom';
        g.fillText('F', xe, ya - 7);
      }
    },

    // Problem data (statements paraphrased; pages are controlbook.pdf pages).
    problems: {
      ch2: {
        id: 'D.2', page: 378, sim: { tEnd: 10 },
        statement: [
          '(a) Using the configuration variable z, write an expression for the kinetic energy of the system.',
          '(b) Animate the mass-spring-damper with a variable input z (here: z(t) = A sin(2πft)).',
        ],
      },
      ch3: {
        id: 'D.3', page: 378, sim: { tEnd: 30, tStep: 0 },
        statement: [
          '(a) Find the potential energy of the system.',
          '(b) Define the generalized coordinates.',
          '(c) Find the generalized forces and damping forces.',
          '(d) Derive the equations of motion using the Euler-Lagrange equations.',
          '(e) Implement the equations of motion and simulate the system with a variable force input, connected to the D.2 animation.',
        ],
      },
      ch4: {
        id: 'D.4', page: 378, sim: { tEnd: 30, tStep: 0 },
        statement: ['(a) Find the equilibria of the system.', '(b) Linearize about the equilibria using Jacobian linearization.', '(c) Linearize the system using feedback linearization.'],
      },
      ch5: {
        id: 'D.5', page: 378, sim: { tEnd: 30, tStep: 0 },
        statement: ['(a) Use the Laplace transform to convert the equations of motion to the s-domain.', '(b) Find the transfer function from the input force F to the position z.', '(c) Draw the associated block diagram.'],
      },
      ch6: {
        id: 'D.6', page: 379, sim: { tEnd: 30, tStep: 0 },
        statement: ['With states x = (z, ż)ᵀ, input u = F and measured output y = z, find the linear state-space equations ẋ = Ax + Bu, y = Cx + Du.'],
      },
      ch7: {
        id: 'D.7', page: 379, desiredPoles: [{ re: -1, im: 0 }, { re: -1.5, im: 0 }],
        sim: { amplitude: 1, tStep: 0.5, tEnd: 12 },
        statement: [
          '(a) From the D.5 transfer function, find the open-loop poles when the equilibrium position is z<sub>e</sub> = 0.',
          '(b) With the PD architecture of Fig. 7-2 (p. 101), find the closed-loop transfer function from z<sub>r</sub> to z and the closed-loop poles as a function of k<sub>P</sub> and k<sub>D</sub>.',
          '(c) Select k<sub>P</sub> and k<sub>D</sub> to place the closed-loop poles at p<sub>1</sub> = −1 and p<sub>2</sub> = −1.5.',
          '(d) Implement the PD control with the gains from (c) and plot the response to a 1 m step input.',
        ],
      },
      ch8: {
        id: 'D.8', page: 379, tr: 2, zeta: 0.7, step: 1,
        sim: { amplitude: 1, tStep: 0.5, tEnd: 12 },
        statement: [
          '(a) The requirements are t<sub>r</sub> ≈ 2 s and ζ = 0.7. Find the desired closed-loop characteristic polynomial Δ<sup>d</sup><sub>cl</sub>(s), the pole locations, and k<sub>P</sub>, k<sub>D</sub>. Verify the step response in simulation.',
          '(b) The force is limited to F<sub>max</sub> = 6 N. Add saturation and, using t<sub>r</sub> as the tuning parameter, tune k<sub>P</sub>, k<sub>D</sub> so the input <em>just</em> saturates for a 1 m step on z<sub>r</sub>. Show the step response does not saturate with the new gains.',
        ],
      },
      ch9: {
        id: 'D.9', page: 380, sim: { amplitude: 1, tStep: 0.5, tEnd: 40, dist: 0.5, tDist: 20 },
        statement: [
          '(a) With PD control, what is the system type? Characterize the steady-state error for a step, a ramp and a parabola. How does this change if you add an integrator?',
          '(b) A constant disturbance acts at the plant input (for example, an inaccurate value of the spring constant). What is the steady-state error to a constant input disturbance without and with the integrator?',
        ],
      },
      p6: {
        id: 'D.P.6', page: 380, tr: 2, zeta: 0.7,
        sim: { amplitude: 1, tStep: 0.5, tEnd: 40 },
        statement: [
          'Add an integrator to obtain PID control. Put the characteristic equation in Evans form, plot the root locus versus the integrator gain k<sub>I</sub>, and select a k<sub>I</sub> that does not significantly change the other closed-loop poles. (Work mode: enter your D.8 gains. Explore starts from the D.8(a) specs t<sub>r</sub> = 2 s, ζ = 0.7.)',
        ],
      },
      ch10: {
        id: 'D.10', page: 380, tr: 2, zeta: 0.7, sigma: 0.05, kiRef: 0.75, tolPct: 2,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0, tEnd: 50 },
        mismatch: { m: 15, k: -18, b: 10 },  // one fixed draw with alpha = 0.2, so the page is repeatable
        statement: [
          '(a) Let m, k and b vary by up to 20% of their nominal values each run (α = 0.2).',
          '(b) The controller gets only the measured position z and the reference z<sub>r</sub>, not the state.',
          '(c) Implement the PID controller designed in D.8 with dirty-derivative gain σ = 0.05. Tune the integrator to remove the steady-state error caused by the uncertain parameters. (This tab uses the D.8(a) specs t<sub>r</sub> = 2 s, ζ = 0.7.)',
        ],
      },
      ch11: {
        id: 'D.11', page: 380, tr: 2, zeta: 0.7,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0, tEnd: 50 },
        statement: [
          '(a) Select the closed-loop poles as the roots of s² + 2ζω<sub>n</sub>s + ω<sub>n</sub>² with ω<sub>n</sub>, ζ from D.8 (here the D.8(a) specs).',
          '(b) Add the D.6 state-space matrices A, B, C, D to your parameter file.',
          '(c) Verify the system is controllable: rank(𝒞<sub>A,B</sub>) = n.',
          '(d) Find K so that eig(A − BK) are the desired poles, and k<sub>r</sub> so the DC gain from z<sub>r</sub> to z is one. Note that K = (k<sub>P</sub>, k<sub>D</sub>) from D.8. Why?',
          '(e) Implement the state-feedback controller, using a digital differentiator to estimate ż.',
        ],
      },
      ch12: {
        id: 'D.12', page: 381, tr: 2, zeta: 0.7, pIRef: -1,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0, tEnd: 50, dist: 0.25, tDist: 0 },
        mismatch: { m: 15, k: -18, b: 10 },
        statement: [
          '(a) Add an integrator with anti-windup to the D.11 state-feedback controller.',
          '(b) Add a constant input disturbance of 0.25 N and let the plant parameters vary by up to 20%.',
          '(c) Tune the integrator pole (and other gains if needed) for good tracking.',
        ],
      },
      ch13: {
        id: 'D.13', page: 381, tr: 2, zeta: 0.7, pIRef: -1, obsFactor: 10, zetaObs: 0.707,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0, tEnd: 50, dist: 0, tDist: 0 },
        statement: [
          '(a) Use exact parameters (α = 0) and no input disturbance.',
          '(b) Verify the system is observable: rank(𝒪<sub>A,C</sub>) = n.',
          '(c) Add an observer that estimates x̂, and use x̂ in the D.12 controller. Tune the controller and observer poles.',
          '(d) Plot the state and the estimated state together.',
          '(e) Add an input disturbance of 0.25 and observe the steady-state error even with the integrator. It comes from a steady-state error in the observation error.',
        ],
      },
      ch14: {
        id: 'D.14', page: 382, tr: 2, zeta: 0.7, pIRef: -1, obsFactor: 10, zetaObs: 0.707, pDRef: -5,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0, tEnd: 50, dist: 0.25, tDist: 0, noise: 0.001 },
        mismatch: { m: 15, k: -18, b: 10 },
        statement: [
          '(a) Use α = 0.2 (20% parameter error), an input disturbance of 0.25, and noise on z<sub>m</sub> with standard deviation 0.001.',
          '(b) Add a disturbance observer to the controller and verify that the steady-state error in the estimator is removed. Tune for a good response.',
        ],
      },
      ch15: {
        id: 'D.15', page: 382, sim: { tEnd: 80, tStep: 0 },
        statement: ['Draw by hand the Bode plot from force F̃ to position z̃, then compare with the <code>bode</code> command.'],
      },
      ch16: {
        id: 'D.16', page: 382, wdin: 0.1, wno: 100,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0, tEnd: 50 },
        statement: [
          'Plot the Bode plots of (1) the plant and (2) the plant under the D.10 PID control, with the dirty derivative in the D term.',
          '(a) What is the tracking error to a unit ramp under PID control?',
          '(b) If the input disturbance d<sub>in</sub> has frequency content below ω<sub>d<sub>in</sub></sub> = 0.1 rad/s, what percentage of it shows up in z under PID control?',
          '(c) If all the noise n(t) is above ω<sub>no</sub> = 100 rad/s, what percentage of it shows up in z? (p. 383)',
        ],
      },
      ch17: {
        id: 'D.17', page: 383,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0, tEnd: 50 },
        statement: ['Use <code>bode</code> and <code>margin</code> to find the phase and gain margins under the D.10 PID control. Plot the open-loop and closed-loop Bode plots on one graph. What is the closed-loop bandwidth, and how does it relate to the crossover frequency?'],
      },
      ch18: {
        id: 'D.18', page: 383, wr: 0.1, gr: 0.03, wn: 500, gn: 0.001, pm: 60,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0, tEnd: 50, dist: 0.25, tDist: 10, noise: 0.001 },
        mismatch: { m: 15, k: -18, b: 10 },
        statement: [
          'Loopshape a controller F = C(s)E(s) for Z(s) = (1/m)/(s² + (b/m)s + k/m) F(s), with e = z<sub>f</sub><sup>r</sup> − z and z<sub>f</sub><sup>r</sup> the prefiltered reference, so that:',
          '• constant input disturbances are rejected;',
          '• references below ω<sub>r</sub> = 0.1 rad/s are tracked to within γ<sub>r</sub> = 0.03;',
          '• noise on z above ω<sub>n</sub> = 500 rad/s is attenuated by γ<sub>n</sub> = 0.001;',
          '• the phase margin is close to 60°;',
          '• a prefilter F(s) reduces peaking in the closed-loop response;',
          '• C(s) and F(s) are implemented in simulation with their state-space (or digital filter) equivalents.',
        ],
      },
    },
  };

  // ------------------------------------------------------------ shared kit --
  // Small helpers every D chapter uses. Problem panels come from WB.design.
  const { el, slider, bind } = WB.ui;
  const lib = {
    panel: (...a) => WB.design.problemPanel(...a),
    check: (...a) => WB.design.checkNumbers(...a),
    num: (s) => WB.design.num(s),
    // Linear overlay: the same controller on the nominal model with no saturation,
    // disturbance or noise. D is linear, so any gap comes from those three.
    linearSim(ctx, common, makeController) {
      const p = ctx.pModel, sys = ctx.sys;
      return WB.sim.simulate({
        ...common, disturbance: null, noise: null,
        plant: { f: (x, u) => sys.f(x, u, p), h: sys.h, uLimit: Infinity },
        controller: makeController(ctx, { linear: true }),
      });
    },
    gainSlider(parent, ctx, key, label, max, { min = 0, step, obj } = {}) {
      return slider(parent, { label, min, max, step: step || (max - min) / 2000, sig: 4, ...bind(ctx, key, () => (obj ? ctx.st[obj] : ctx.st)) });
    },
    readout(parent, ctx, keys, vals) {
      WB.ui.readout(parent, () => { const v = vals(); return keys.map((k) => [k, v[k]]); });
    },
    note(parent, text) { parent.append(el('p', { class: 'muted small', text })); },
    // Error |r - y| just before the first square-wave switch (or at t_end).
    errorBeforeSwitch(ctx) {
      const res = ctx.app.result(), S = ctx.S;
      const tSw = WB.sim.switchTime(S);
      const i = WB.sim.indexBefore(S, res, tSw);
      return { e: Math.abs(res.r[i] - res.y[i]), t: res.t[i], amp: Math.abs(S.sim.amplitude) };
    },
    polesOf(list) { return list.map((p) => M.fmtPole(p)).join(', '); },
  };
  WB.studies.D.lib = lib;
})();
