// Design Study B: pendulum on a cart (controlbook.pdf p. 14).
// Physics mirrors _B_pendulum/python/pendulumDynamics.py and pendulumParam.py:
// state x = (z, θ, ż, θ̇), input F, outputs y = (z, θ). θ = 0 is upright and a
// positive θ tips the rod toward +z.
window.WB = window.WB || {};
WB.systems = WB.systems || {};
WB.studies = WB.studies || {};
WB.studies.B = WB.studies.B || { chapters: {} };

(function () {
  const DEG = Math.PI / 180;

  // Equation 3.2 solved for (z̈, θ̈), written exactly as pendulumDynamics.f does it
  // (M⁻¹C with M inverted by LU), so the JS and Python agree to rounding.
  function f(x, F, p) {
    const theta = x[1], zdot = x[2], thetadot = x[3];
    const c = Math.cos(theta), s = Math.sin(theta);
    const M00 = p.m1 + p.m2, M01 = p.m1 * (p.ell / 2.0) * c, M11 = p.m1 * (p.ell ** 2 / 3.0);
    const C0 = p.m1 * (p.ell / 2.0) * thetadot ** 2 * s + F - p.b * zdot;
    const C1 = p.m1 * p.g * (p.ell / 2.0) * s;
    // inv(M) by LU with partial pivoting (|M00| > |M01| for every θ)
    const l = M01 * (1 / M00), u11 = M11 - l * M01;
    const i11 = 1 / u11, i10 = -l / u11;
    const i01 = (0 - M01 * i11) / M00, i00 = (1 - M01 * i10) / M00;
    const zddot = i00 * C0 + i01 * C1;
    const thetaddot = i10 * C0 + i11 * C1;
    return [zdot, thetadot, zddot, thetaddot];
  }

  // ---------------------------------------------------------- linear models --
  // Inner-loop plant Θ̃/F̃ with b = 0 (B.5, p. 76): b0 / (s² + a0), both negative.
  function inner(p) {
    const J = p.m1 * p.ell / 6 + p.m2 * 2 * p.ell / 3;
    return { J, b0: -1 / J, a0: -(p.m1 + p.m2) * p.g / J };
  }
  // Outer-loop plant Z̃/Θ̃ = (−(2ℓ/3)s² + g)/s² (B.5, p. 76).
  function outerTf(p) { return { num: [-2 * p.ell / 3, 0, p.g], den: [1, 0, 0] }; }

  // Jacobian linearization about (z_e, θ_e, 0, 0) with F_e = 0 (B.4 / B.6). θ_e = 0
  // gives Eq. 6.17; θ_e = π (hanging) flips the sign of cos θ_e.
  function linearize(p, thetaE = 0) {
    const c = Math.cos(thetaE);
    const M00 = p.m1 + p.m2, M01 = p.m1 * p.ell / 2 * c, M11 = p.m1 * p.ell ** 2 / 3;
    const det = M00 * M11 - M01 * M01;
    const inv = [[M11 / det, -M01 / det], [-M01 / det, M00 / det]];
    // M q̈ = [−b ż + F̃, m1 g (ℓ/2) cos θe θ̃]
    const kTh = p.m1 * p.g * p.ell / 2 * c;
    const row = (i) => [0, inv[i][1] * kTh, inv[i][0] * -p.b, 0];
    const A = [[0, 0, 1, 0], [0, 0, 0, 1], row(0), row(1)];
    const B = [[0], [0], [inv[0][0]], [inv[1][0]]];
    return { A, B, C: [[1, 0, 0, 0], [0, 1, 0, 0]], D: [[0], [0]] };
  }
  // Eq. 6.17 written the way ctrlStateFeedback.py writes it (same rounding).
  function stateSpace(p) {
    const d = 0.25 * p.m1 + p.m2;
    return {
      A: [[0, 0, 1, 0], [0, 0, 0, 1],
        [0, -3 * p.m1 * p.g / 4 / d, -p.b / d, 0],
        [0, 3 * (p.m1 + p.m2) * p.g / 2 / d / p.ell, 3 * p.b / 2 / d / p.ell, 0]],
      B: [[0], [0], [1 / d], [-3.0 / 2 / d / p.ell]],
      C: [[1, 0, 0, 0], [0, 1, 0, 0]], D: [[0], [0]],
    };
  }

  // Coefficients of the linearized equations about θ_e = kπ (Eq. 4.12 for k even):
  //   M q̈̃ = (−b ż̃ + F̃, kTh θ̃),  c = cos θ_e.
  function linearEOM(p, thetaE = 0) {
    const c = Math.cos(thetaE);
    return { M: [[p.m1 + p.m2, p.m1 * p.ell / 2 * c], [p.m1 * p.ell / 2 * c, p.m1 * p.ell ** 2 / 3]], b: p.b, kTh: p.m1 * p.g * p.ell / 2 * c };
  }

  // Energies (B.2 Eq. 2.4, B.3 p. 47 with P₀ = 0)
  function kinetic(x, p) {
    const [, th, zd, thd] = x;
    return 0.5 * (p.m1 + p.m2) * zd ** 2 + 0.5 * p.m1 * p.ell ** 2 / 3 * thd ** 2 + p.m1 * p.ell / 2 * zd * thd * Math.cos(th);
  }
  function potential(x, p) { return p.m1 * p.g * p.ell / 2 * (Math.cos(x[1]) - 1); }

  // ---------------------------------------------------------------- drawing --
  function draw(ctx, w, h, s) {
    const { css } = WB.plot;
    const { x, p } = s;
    const z = x[0], th = x[1];
    const r = s.rAll && s.rAll.length ? s.rAll[0] : s.r;
    const ell = p.ell, cartW = 0.5, cartH = 0.15, gap = 0.005, rad = 0.06;
    const scale = Math.min(w / 5.2, (h * 0.66) / (ell + cartH + 0.3));
    const span = w / scale;
    let center = 0;
    const edge = span / 2 - 0.55;
    if (Math.abs(z) > edge) center = z - Math.sign(z) * edge;
    const gy = h * 0.74;
    const X = (v) => w / 2 + (v - center) * scale;
    const Y = (v) => gy - v * scale;

    // track with ticks every 0.5 m
    ctx.strokeStyle = css('--axis'); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, gy + 0.5); ctx.lineTo(w, gy + 0.5); ctx.stroke();
    ctx.fillStyle = css('--text-muted'); ctx.font = '10px var(--font-sans, system-ui)';
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    const lo = Math.ceil((center - span / 2) * 2) / 2, hi = center + span / 2;
    for (let v = lo; v <= hi; v += 0.5) {
      const xx = Math.round(X(v)) + 0.5, major = Math.abs(v - Math.round(v)) < 1e-9;
      ctx.beginPath(); ctx.moveTo(xx, gy); ctx.lineTo(xx, gy + (major ? 6 : 3)); ctx.stroke();
      if (major) ctx.fillText(`${Math.round(v)} m`, xx, gy + 7);
    }

    // reference marker on the track
    if (isFinite(r)) {
      const rx = X(r);
      ctx.strokeStyle = css('--ref'); ctx.lineWidth = 1.5; ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.moveTo(rx, gy); ctx.lineTo(rx, Y(cartH + 0.12)); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = css('--ref');
      ctx.beginPath(); ctx.moveTo(rx, gy + 1); ctx.lineTo(rx - 6, gy + 10); ctx.lineTo(rx + 6, gy + 10); ctx.closePath(); ctx.fill();
      ctx.textAlign = 'left'; ctx.textBaseline = 'top';
      ctx.fillText('z_r', rx + 8, gy + 2);
    }

    // cart
    const cx0 = X(z - cartW / 2), cy0 = Y(gap + cartH);
    ctx.fillStyle = css('--series-1');
    ctx.beginPath(); ctx.roundRect(cx0, cy0, cartW * scale, cartH * scale, 3); ctx.fill();

    // rod and bob (pendulumAnimation.py: rod from the top of the cart)
    const px = X(z), py = Y(gap + cartH);
    const tx = X(z + ell * Math.sin(th)), ty = Y(gap + cartH + ell * Math.cos(th));
    ctx.strokeStyle = css('--text-primary'); ctx.lineWidth = Math.max(2, scale * 0.025);
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(tx, ty); ctx.stroke();
    ctx.fillStyle = css('--series-3'); ctx.strokeStyle = css('--surface'); ctx.lineWidth = 1.5;
    const bx = X(z + (ell + rad) * Math.sin(th)), by = Y(gap + cartH + (ell + rad) * Math.cos(th));
    ctx.beginPath(); ctx.arc(bx, by, Math.max(4, rad * scale), 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.fillStyle = css('--surface'); ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(px, py, 3.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();

    // upright reference (θ = 0)
    ctx.strokeStyle = css('--grid'); ctx.setLineDash([2, 4]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, Y(gap + cartH + ell)); ctx.stroke(); ctx.setLineDash([]);

    // force arrow on the cart, length ∝ F / F_max
    const lim = s.uLimit || 1;
    const frac = Math.max(-1, Math.min(1, (s.u || 0) / lim));
    if (Math.abs(frac) > 0.01) {
      const L = frac * Math.min(1.2, span / 4) * scale;
      const ay = Y(gap + cartH / 2), ax = frac > 0 ? cx0 - 4 : cx0 + cartW * scale + 4;
      const x1 = frac > 0 ? ax - Math.abs(L) : ax + Math.abs(L);
      ctx.strokeStyle = css(s.saturated ? '--critical' : '--series-2'); ctx.fillStyle = ctx.strokeStyle; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.moveTo(x1, ay); ctx.lineTo(ax - Math.sign(frac) * 6, ay); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ax - Math.sign(frac) * 8, ay - 5); ctx.lineTo(ax - Math.sign(frac) * 8, ay + 5); ctx.closePath(); ctx.fill();
      ctx.font = '11px var(--font-sans, system-ui)'; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
      ctx.fillText('F', (x1 + ax) / 2, ay - 4);
    }

    // gravity cue
    ctx.strokeStyle = css('--text-muted'); ctx.lineWidth = 1;
    const gx = w - 18, g0 = 14;
    ctx.beginPath(); ctx.moveTo(gx, g0); ctx.lineTo(gx, g0 + 22); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(gx - 4, g0 + 16); ctx.lineTo(gx, g0 + 22); ctx.lineTo(gx + 4, g0 + 16); ctx.stroke();
    ctx.fillStyle = css('--text-muted'); ctx.font = '11px var(--font-sans, system-ui)'; ctx.textAlign = 'right'; ctx.textBaseline = 'top';
    ctx.fillText('g', gx - 6, g0 + 4);
  }

  WB.systems.B = {
    id: 'B',
    name: 'Pendulum on a Cart',
    introPage: 14,
    params: [
      { key: 'm1', label: 'm<sub>1</sub> (rod)', unit: 'kg', value: 0.25, min: 0.05, max: 1, step: 0.005 },
      { key: 'm2', label: 'm<sub>2</sub> (cart)', unit: 'kg', value: 1.0, min: 0.2, max: 3, step: 0.01 },
      { key: 'ell', label: 'ℓ', unit: 'm', value: 1.0, min: 0.2, max: 2, step: 0.01 },
      { key: 'b', label: 'b', unit: 'N·s/m', value: 0.05, min: 0, max: 1, step: 0.005 },
      { key: 'F_max', label: 'F<sub>max</sub>', unit: 'N', value: 5.0, min: 0.5, max: 30, step: 0.1 },
    ],
    constants: { g: 9.8 },
    // pendulumDynamics(alpha) perturbs m1, m2, ell, b; g is "well known".
    uncertain: ['m1', 'm2', 'ell', 'b'],

    f, h: (x) => [x[0], x[1]],
    uLimit: (p) => p.F_max,
    outputs: [
      { key: 'z', label: 'z', unit: 'm', scale: 1, minSpan: 0.2, noiseMax: 0.05 },
      { key: 'theta', label: 'θ', unit: '°', scale: 180 / Math.PI, minSpan: 2, noiseMax: 2 },
    ],
    inputs: [{ key: 'F', label: 'F', unit: 'N' }],
    refs: [{ output: 0, label: 'cart position z_r', unit: 'm', scale: 1, min: -2, max: 2, step: 0.01, defaults: { amplitude: 0.5 } }],
    initial: [
      { key: 'z0', label: 'z(0)', unit: 'm', scale: 1, output: 0, value: 0, min: -1.5, max: 1.5, step: 0.01 },
      { key: 'theta0', label: 'θ(0)', unit: '°', scale: 180 / Math.PI, output: 1, value: 0, min: -45, max: 45, step: 0.5 },
    ],
    x0(init) { return [init.z0, init.theta0, 0, 0]; },
    disturbances: [{ input: 0, label: 'd on F', unit: 'N', min: -3, max: 3, step: 0.01 }],
    simBase: { amplitude: 0.5, frequency: 0.04, Ts: 0.01 },
    stateLabels: ['z', 'θ', 'ż', 'θ̇'],

    inner, outerTf, linearize, linearEOM, stateSpace, kinetic, potential, draw,

    // Problem data for the chapter modules. Statements are paraphrased; page
    // numbers are controlbook.pdf pages (book page + 8).
    problems: {
      ch2: {
        id: 'B.2', page: 31,
        sim: { tEnd: 20, tStep: 0 },
        statement: [
          '(a) Using the configuration variables z and θ, write an expression for the kinetic energy of the system.',
          '(b) Write a class (App. P.1–P.3) that animates the inverted pendulum and display sinusoidal variations of q = (z, θ)ᵀ.',
        ],
      },
      ch3: {
        id: 'B.3', page: 47,
        sim: { tEnd: 10, tStep: 0 },
        openLoop: { shape: 'sine', amp: 1, freq: 1 },   // hw03_pendulumSim.py
        statement: [
          '(a) Find the potential energy of the system.',
          '(b) Define the generalized coordinates.',
          '(c) Find the generalized forces and damping forces.',
          '(d) Derive the equations of motion with the Euler-Lagrange equations.',
          '(e) Implement the equations of motion and simulate with a variable force on the cart, connected to the B.2 animation.',
        ],
      },
      ch4: {
        id: 'B.4', page: 65,
        sim: { tEnd: 3, tStep: 0 },
        statement: [
          '(a) Find the equilibria of the system.',
          '(b) Linearize about the equilibria using Jacobian linearization.',
          'The nonlinearities are not all in the channel of the force F, so the pendulum cannot be feedback linearized (p. 65).',
        ],
      },
      ch5: {
        id: 'B.5', page: 75,
        sim: { tEnd: 2, tStep: 0 },
        statement: [
          '(a) Starting from the linearized equations, use the Laplace transform to move to the s-domain.',
          '(b) Find the transfer functions from F̃(s) to Z̃(s) and Θ̃(s). Assume the damping is negligible. How does that simplify them? Is it reasonable?',
          '(c) From the simplified transfer functions, find Z̃(s)/Θ̃(s). Draw the block diagram as a cascade F̃ → Θ̃ → Z̃.',
          '(d) Explain the cascade physically: how does force move the angle, and how does the angle move the cart?',
        ],
      },
      ch6: {
        id: 'B.6', page: 89,
        sim: { tEnd: 2, tStep: 0 },
        statement: [
          'With x̃ = (z̃, θ̃, ż̃, θ̇̃)ᵀ, ũ = F̃ and measured output ỹ = (z̃, θ̃)ᵀ, find A, B, C, D of the linear state-space model.',
        ],
      },
      ch8: {
        id: 'B.8', page: 124,
        trTh: 0.5, zetaTh: 0.707, M: 10, zetaZ: 0.707,   // (b); (d) leaves the outer specs open
        listing: { trTh: 0.15, M: 15 },                   // Listing 8.3 / ctrlPD.py
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 25, init: { theta0: 10 } },   // (e): θ(0) = 10°
        statement: [
          '(a) Using successive loop closure, draw a block diagram with PD control on both loops. The outer controller takes the desired cart position r<sub>z</sub> and outputs the desired angle r<sub>θ</sub>; the inner controller takes r<sub>θ</sub> and outputs the force F.',
          '(b) For the inner loop, find k<sub>P<sub>θ</sub></sub> and k<sub>D<sub>θ</sub></sub> so that t<sub>r<sub>θ</sub></sub> = 0.5 s and ζ<sub>θ</sub> = 0.707.',
          '(c) Find the DC gain k<sub>DC<sub>θ</sub></sub> of the inner loop.',
          '(d) Replace the inner loop by its DC gain. Add a low-pass filter that cancels the left-half-plane zero, then design a PD controller that stabilizes the cart position.',
          '(e) Implement the design with |F| ≤ 5 N, start the pendulum at θ = 10°, and simulate 10 s to verify that the cart balances the pendulum.',
        ],
      },
      ch9: {
        id: 'B.9', page: 147,
        sim: { type: 'step', amplitude: 0.5, tStep: 1, tEnd: 30, tDist: 15 },
        statement: [
          '(a) With PD control on the inner loop, what is the system type for tracking r<sub>θ</sub>? Give the steady-state error for a step, a ramp and a parabola. What is the type with respect to an input disturbance?',
          '(b) With PD control on the outer loop, what is the system type for tracking r<sub>z</sub>? Give the steady-state error for a step, a ramp and a parabola. How does this change with an integrator? What is the type with respect to an input disturbance, for PD and for PID?',
        ],
      },
      ch10: {
        id: 'B.10', page: 163,
        trTh: 0.2, zetaTh: 0.707, M: 10, zetaZ: 0.707, ki: -0.05, sigma: 0.05, vbar: 0.07, thetaMax: 30,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 50 },
        mismatch: { m1: 14, m2: -11, ell: 7, b: 16 },   // a fixed "alpha = 0.2" draw so the page is repeatable
        statement: [
          '(a) Let m<sub>1</sub>, m<sub>2</sub>, ℓ and b vary by up to 20% of their nominal values (α = 0.2).',
          '(b) The controller may use only the measured outputs z and θ and the reference r<sub>z</sub>.',
          '(c) Implement the nested PID loops from B.8 with dirty-derivative gain σ = 0.05, and tune the integrator to remove the steady-state error caused by the uncertain parameters.',
        ],
      },
      p6: {
        id: 'B.P.6', page: 471,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 50 },
        statement: [
          'Add an integrator to the outer loop of the pendulum to get PID control. Put the closed-loop characteristic equation in Evans form, plot the root locus versus the integrator gain k<sub>I</sub>, and choose a k<sub>I</sub> that does not significantly move the other closed-loop poles.',
        ],
      },
      ch11: {
        id: 'B.11', page: 189,
        book: { trTh: 0.5, zetaTh: 0.707, M: 3, zetaZ: 0.707, rule: '2.2' },   // (a): B.8 values, t_z = 3 t_θ
        repo: { trTh: 0.5, zetaTh: 0.9, M: 5, zetaZ: 0.9, rule: 'tp' },        // Listing 11.2 / ctrlStateFeedback.py
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 25 },
        statement: [
          '(a) Find the desired closed-loop poles from ζ<sub>z</sub>, ω<sub>n<sub>θ</sub></sub> and ζ<sub>θ</sub> chosen in B.8, with ω<sub>n<sub>z</sub></sub> chosen so that t<sub>z</sub> = 3t<sub>θ</sub>.',
          '(b) Add the A, B, C, D matrices from B.6 to the parameter file.',
          '(c) Verify controllability: rank(𝒞) = n.',
          '(d) Find K so that eig(A − BK) are the desired poles, and k<sub>r</sub> so that the DC gain from z<sub>r</sub> to z is one.',
          '(e) Implement state feedback and tune the poles for a good response. State-space methods should give a much faster response.',
        ],
      },
      ch12: {
        id: 'B.12', page: 206,
        book: { trTh: 0.5, zetaTh: 0.707, M: 3, zetaZ: 0.707, rule: '2.2', pI: -2 },
        repo: { trTh: 0.5, zetaTh: 0.9, M: 3, zetaZ: 0.9, rule: 'tp', pI: -2 },
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 50, dist: 0.5, tDist: 0 },
        mismatch: { m1: 14, m2: -11, ell: 7, b: 16 },
        statement: [
          '(a) Add an integrator with anti-windup on the position to the B.11 state-feedback controller.',
          '(b) Add a constant input disturbance of 0.5 N and let the parameters vary by up to 20%.',
          '(c) Tune the integrator pole (and other gains if needed) for good tracking.',
        ],
      },
      ch13: {
        id: 'B.13', page: 231,
        repo: { trTh: 0.5, zetaTh: 0.9, M: 3, zetaZ: 0.9, rule: 'tp', pI: -2, obsFactor: 10, obsRule: 'tp' },
        sim: { type: 'square', amplitude: 0.5, frequency: 0.05, tStep: 0, tEnd: 30, dist: 0, tDist: 0 },
        statement: [
          '(a) Use exact parameters (α = 0) and no input disturbance.',
          '(b) Verify observability: rank(𝒪<sub>A,C</sub>) = n.',
          '(c) Add an observer that estimates x̂ and use x̂ in the controller. Tune the controller and observer poles.',
          '(d) Plot the states and their estimates together.',
          '(e) Add an input disturbance of 0.05 N: the response now has a steady-state error even with the integrator, because the observation error has one.',
        ],
      },
      ch14: {
        id: 'B.14', page: 250,
        repo: { trTh: 0.5, zetaTh: 0.9, M: 3, zetaZ: 0.9, rule: 'tp', pI: -2, obsFactor: 10, obsRule: '2.2', pD: -1 },
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 50, dist: 0.5, tDist: 0, noise: 0.001, noises: [0.0573] },
        mismatch: { m1: 14, m2: -11, ell: 7, b: 16 },
        statement: [
          '(a) Use α = 0.2, an input disturbance of 0.5 N, and output noise with standard deviation 0.001 on z<sub>m</sub> and θ<sub>m</sub>. Without a disturbance observer, the controller does not handle the large disturbance well.',
          '(b) Add a disturbance observer, verify that the estimator\'s steady-state error is gone, and tune.',
        ],
      },
      ch15: {
        id: 'B.15', page: 277,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 50 },
        statement: [
          '(a) Draw by hand the Bode plot of the inner-loop transfer function from F to θ, then compare with the bode command.',
          '(b) Draw by hand the Bode plot of the outer-loop transfer function from θ to z, then compare with the bode command.',
        ],
      },
      ch16: {
        id: 'B.16', page: 297,
        wrIn: 1.0, wnoIn: 200, wrOut: 0.001, rMax: 50,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 50 },
        statement: [
          'Inner loop: plot the Bode plots of the plant and of the plant under PD control, with the B.10 gains.',
          '(a) To what percent error can the closed loop track θ<sub>r</sub> if all its frequency content is below ω<sub>r</sub> = 1.0 rad/s?',
          '(b) If the sensor noise on the inner loop is all above ω<sub>no</sub> = 200 rad/s, what percent of it shows up in θ?',
          'Outer loop: plot the plant and the plant under PID control, with the B.10 gains.',
          '(c) If y<sub>r</sub>(t) has frequency content below ω<sub>r</sub> = 0.001 rad/s, what is the tracking error under PID control if |r(t)| ≤ 50?',
        ],
      },
      ch17: {
        id: 'B.17', page: 314,
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 50 },
        statement: [
          'Use the B.10 gains.',
          '(a) Inner loop under PD control: find the phase and gain margins, and plot the open- and closed-loop Bode plots together. What is the inner-loop bandwidth, and how does it compare with the crossover frequency?',
          '(b) Outer loop under PID control: the same questions.',
          '(c) What is the bandwidth separation between the inner (fast) and outer (slow) loops? Is successive loop closure justified?',
        ],
      },
      ch18: {
        id: 'B.18', page: 349,
        inner: { pm: 60, wno: 200, gn: 0.1, wbw: 40 },
        outer: { pm: 60, wr: 0.0032, gr: 1e-5, wno: 1000, gn: 1e-4, tr: 2 },
        sim: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0, tEnd: 50, dist: 0.5, tDist: 0, noise: 0.001, noises: [0.0573] },
        mismatch: { m1: 7, m2: -6, ell: 4, b: 8 },      // hw18_pendulumSim.py uses alpha = 0.1
        statement: [
          '(a) Inner loop: with proportional and phase-lead control, design C<sub>in</sub>(s) to stabilize P<sub>in</sub>(s) with a phase margin near 60°, rejecting noise above ω<sub>no</sub> = 200 rad/s by γ<sub>no</sub> = 0.1. The inner closed-loop bandwidth should be about 40 rad/s. P<sub>in</sub> has negative gain, so the proportional gain must be negative too.',
          '(b) Outer loop: the plant is P = P<sub>out</sub>·P<sub>in</sub>C<sub>in</sub>/(1 + P<sub>in</sub>C<sub>in</sub>). Design C<sub>out</sub> for PM ≈ 60°, tracking error γ<sub>r</sub> = 10⁻⁵ below ω<sub>r</sub> = 0.0032 rad/s, noise rejection γ<sub>no</sub> = 10⁻⁴ above 1000 rad/s, and a rise time of about 2 s.',
        ],
      },
    },
  };
  WB.systems.B.DEG = DEG;
})();
