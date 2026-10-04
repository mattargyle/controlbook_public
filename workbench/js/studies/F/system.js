// Design Study F: planar VTOL (controlbook.pdf pp. 393–394, homework pp. 395–404).
// The book gives no worked solutions for F. The physics matches the expected
// values in _F_planar_vtol/python/testDynamics.py exactly (tools/regress_F.py).
//
// State x = (z, h, θ, ż, ḣ, θ̇), input u = (f_r, f_ℓ), outputs (z, h, θ).
// Mixing (F.4, p. 395): F = f_r + f_ℓ, τ = d(f_r − f_ℓ), so
//   (f_r, f_ℓ) = [[1, 1], [d, −d]]⁻¹ (F, τ) = (F/2 + τ/2d, F/2 − τ/2d).
window.WB = window.WB || {};
WB.systems = WB.systems || {};

(function () {
  const RAD = 180 / Math.PI;
  const NO_EXT = { Fwind: 0, wind: 0, ah: 0 };

  // The plant for student controllers (see plantPy below). Same equations as f.
  const PLANT_PY = `import math

# disturbances (zero unless the run sets them), switched on at t >= wb_tDist
_D = {k: float(getattr(P, 'wb_' + k, 0.0)) for k in ('dF', 'dtau', 'Fwind', 'wind', 'ah')}
_tDist = float(getattr(P, 'wb_tDist', 0.0))
_M = P.mc + 2 * P.mr
_J = P.Jc + 2 * P.mr * P.d**2


def h(state):
    return [state[0][0], state[1][0], state[2][0]]


def f(state, u):
    zd, hd, th, thd = state[3][0], state[4][0], state[2][0], state[5][0]
    fr, fl = float(u[0][0]), float(u[1][0])
    Fw = wind = ah = 0.0
    if getattr(P, 't', 0.0) >= _tDist - 1e-9:   # P.t: time of the current step
        if _D['dF'] or _D['dtau']:
            fr = min(max(fr + _D['dF'] / 2 + _D['dtau'] / (2 * P.d), 0.0), P.f_max)
            fl = min(max(fl + _D['dF'] / 2 - _D['dtau'] / (2 * P.d), 0.0), P.f_max)
        Fw, wind, ah = _D['Fwind'], _D['wind'], _D['ah']
    F = fr + fl
    return np.array([[zd + wind], [hd], [thd],
                     [(-F * math.sin(th) - P.mu * zd + Fw) / _M],
                     [(-_M * P.g + F * math.cos(th)) / _M + ah],
                     [P.d * (fr - fl) / _J]])
`;

  // Total mass and roll inertia (rotors are point masses at ±d).
  const mass = (p) => p.mc + 2 * p.mr;
  const inertia = (p) => p.Jc + 2 * p.mr * p.d * p.d;

  WB.systems.F = {
    id: 'F',
    name: 'Planar VTOL',
    introPage: 393,

    // Nominal parameters (p. 393). m_ℓ = m_r = 0.25 kg, so one slider covers both rotors.
    params: [
      { key: 'mc', desc: 'mass of the center pod', label: 'm<sub>c</sub>', unit: 'kg', value: 1.0, min: 0.2, max: 3, step: 0.01 },
      { key: 'mr', desc: 'mass of each rotor (right and left)', label: 'm<sub>r</sub> = m<sub>ℓ</sub>', unit: 'kg', value: 0.25, min: 0.05, max: 1, step: 0.005 },
      { key: 'Jc', desc: 'moment of inertia of the center pod', label: 'J<sub>c</sub>', unit: 'kg·m²', value: 0.0042, min: 0.001, max: 0.02, step: 0.0001 },
      { key: 'd', desc: 'distance from the center to each rotor', label: 'd', unit: 'm', value: 0.3, min: 0.1, max: 0.6, step: 0.005 },
      { key: 'mu', desc: 'drag coefficient', label: 'μ', unit: 'kg/s', value: 0.1, min: 0, max: 0.5, step: 0.005 },
      { key: 'f_max', desc: 'maximum thrust of each rotor', label: 'f<sub>max</sub>', unit: 'N', value: 10, min: 2, max: 30, step: 0.1 },
    ],
    constants: { g: 9.81 },
    // F.10(a), p. 399: m_c, J_c, d and μ vary by up to 20%.
    uncertain: ['mc', 'Jc', 'd', 'mu'],

    mass, inertia,

    // Channels. Ref 0 is altitude (the first loop the book designs, F.7), so the
    // app's step-response metrics are for h; the z loop gets its own readout.
    outputs: [
      { key: 'z', label: 'z', unit: 'm', scale: 1, minSpan: 0.5, noiseMax: 0.2 },
      { key: 'h', label: 'h', unit: 'm', scale: 1, minSpan: 0.5, noiseMax: 0.2 },
      { key: 'theta', label: 'θ', unit: '°', scale: RAD, minSpan: 2, noiseMax: 2 },
    ],
    inputs: [
      { key: 'fr', label: 'f_r', unit: 'N' },
      { key: 'fl', label: 'f_ℓ', unit: 'N' },
    ],
    refs: [
      { output: 1, label: 'altitude h_r', unit: 'm', scale: 1, min: -5, max: 10, step: 0.1, defaults: { type: 'step', amplitude: 2, frequency: 0.03, tStep: 0 } },
      { output: 0, label: 'position z_r', unit: 'm', scale: 1, min: -5, max: 10, step: 0.1, defaults: { type: 'square', amplitude: 2.5, frequency: 0.08, tStep: 0 } },
    ],
    initial: [
      { key: 'h0', label: 'h(0)', unit: 'm', scale: 1, output: 1, value: 0, min: -5, max: 10, step: 0.1 },
      { key: 'z0', label: 'z(0)', unit: 'm', scale: 1, output: 0, value: 0, min: -5, max: 10, step: 0.1 },
      { key: 'theta0', label: 'θ(0)', unit: '°', scale: RAD, output: 2, value: 0, min: -30, max: 30, step: 0.5 },
    ],
    x0(init) { return [init.z0 || 0, init.h0 || 0, init.theta0 || 0, 0, 0, 0]; },
    stateLabels: ['z', 'h', 'θ', 'ż', 'ḣ', 'θ̇'],

    // Disturbances. The book uses several kinds, so the study's simulate hook
    // (common.js) applies them itself; `input` indices ≥ 2 are not plant inputs.
    //   d_F, d_τ   input force/torque disturbance (F.13e), mixed onto f_r, f_ℓ
    //   F_wind     lateral force in the z equation (F.12b hint, p. 400)
    //   wind       velocity added to ż in the z kinematics (F.14a snippet, p. 401)
    //   d_h        acceleration added to ḧ (F.14a snippet "altitude_dist")
    disturbances: [
      { input: 2, key: 'dF', label: 'thrust d<sub>F</sub>', unit: 'N', min: -3, max: 3, step: 0.01 },
      { input: 3, key: 'dtau', label: 'torque d<sub>τ</sub>', unit: 'N·m', min: -0.5, max: 0.5, step: 0.005 },
      { input: 4, key: 'Fwind', label: 'wind F<sub>wind</sub>', unit: 'N', min: -1, max: 1, step: 0.01 },
      { input: 5, key: 'wind', label: 'wind w (ż)', unit: 'm/s', min: -3, max: 3, step: 0.05 },
      { input: 6, key: 'ah', label: 'd<sub>h</sub> (on ḧ)', unit: 'm/s²', min: -3, max: 3, step: 0.05 },
    ],
    simBase: { tEnd: 50, tDist: 0 },

    // Equations of motion (F.3, derived with Euler-Lagrange; see models.js):
    //   (m_c + 2m_r) z̈ = −(f_r + f_ℓ) sin θ − μ ż
    //   (m_c + 2m_r) ḧ = −(m_c + 2m_r) g + (f_r + f_ℓ) cos θ
    //   (J_c + 2m_r d²) θ̈ = d (f_r − f_ℓ)
    // ext (optional) carries the non-input disturbances.
    f(x, u, p, ext) {
      const [, , th, zd, hd, thd] = x;
      const fr = u[0], fl = u[1];
      const e = ext || NO_EXT;
      const Mt = p.mc + 2 * p.mr;
      const zdd = (-(fr + fl) * Math.sin(th) - p.mu * zd + e.Fwind) / Mt;
      const hdd = (-Mt * p.g + (fr + fl) * Math.cos(th)) / Mt + e.ah;
      const thdd = p.d * (fr - fl) / (p.Jc + 2 * p.mr * p.d ** 2);
      return [zd + e.wind, hd, thd, zdd, hdd, thdd];
    },
    h(x) { return [x[0], x[1], x[2]]; },
    uLimit(p) { return [[0, p.f_max], [0, p.f_max]]; },

    // Student controllers (WB.myCtrl, Ch 7–18): the same plant as Python, u = (f_r, f_ℓ)
    // a 2×1 column. The disturbances come in as plant parameters (wb_dF, wb_dtau,
    // wb_Fwind, wb_wind, wb_ah, switched on when P.t ≥ wb_tDist; common.js
    // plantExtras). d_F, d_τ are mixed with the true d and saturated with u, as in
    // common.js simulate.
    plantPy: PLANT_PY,
    py: {
      r: 'r', rDoc: 'r = np.array([[h_r], [z_r]]): the altitude and position references',
      y: ['z', 'h', 'theta'], x: ['z', 'h', 'theta', 'zdot', 'hdot', 'thetadot'],
      u: 'u', uZero: 'np.array([0.0, 0.0])  # rotor forces [f_r, f_l]',
    },
    // Initial state as P.z0, ..., P.thetadot0 (as VTOLParam.py would define them).
    pyParams(x0) { return { z0: x0[0], h0: x0[1], theta0: x0[2], zdot0: x0[3], hdot0: x0[4], thetadot0: x0[5] }; },
    // Second parameter set for the controller checks: every loop's gains change by 25% or more.
    altParams(p) { return { ...p, mc: p.mc * 0.6, Jc: p.Jc * 1.5, d: p.d * 0.8, mu: p.mu * 1.5 }; },

    // (F, τ) → (f_r, f_ℓ) with the controller's (nominal) d.
    mix(F, tau, p) { return [F / 2 + tau / (2 * p.d), F / 2 - tau / (2 * p.d)]; },
    unmix(u, p) { return { F: u[0] + u[1], tau: p.d * (u[0] - u[1]) }; },
    equilibriumForce(p) { return mass(p) * p.g; },

    // Linearized design models about hover (θ_e = 0, F_e = (m_c + 2m_r) g), F.4–F.6.
    models(p) {
      const Mt = mass(p), J = inertia(p), a = p.mu / Mt;
      return {
        M: Mt, J, a, Fe: Mt * p.g,
        lon: { b0: 1 / Mt, a1: 0, a0: 0 },          // H/F̃ = (1/M)/s²
        inner: { b0: 1 / J, a1: 0, a0: 0 },         // Θ/τ̃ = (1/J)/s²
        outer: { b0: -p.g, a1: a, a0: 0 },          // Z/Θ = −g/(s² + (μ/M)s)
        // x_lon = (h, ḣ), u = F̃, y = h
        lonSS: { A: [[0, 1], [0, 0]], B: [[0], [1 / Mt]], C: [[1, 0]], D: [[0]] },
        // x_lat = (z, θ, ż, θ̇), u = τ̃, y = (z, θ)
        latSS: {
          A: [[0, 0, 1, 0], [0, 0, 0, 1], [0, -p.g, -a, 0], [0, 0, 0, 0]],
          B: [[0], [0], [0], [1 / J]],
          C: [[1, 0, 0, 0], [0, 1, 0, 0]], D: [[0], [0]],
        },
      };
    },

    // Kinetic and potential energy (F.2, F.3a).
    kinetic(x, p) { return 0.5 * mass(p) * (x[3] ** 2 + x[4] ** 2) + 0.5 * inertia(p) * x[5] ** 2; },
    potential(x, p) { return mass(p) * p.g * x[1]; },

    // Animation: world view following the vehicle, target box on the ground at z_r
    // (VTOLAnimation.py), dashed ghost at the commanded (z_r, h_r), rotor force arrows.
    draw(ctx, w, h, s) {
      const { css } = WB.plot;
      const x = s.x, p = s.p;
      const [z, alt, th] = x;
      // open-loop chapters pass h_r = NaN: no commanded-position ghost
      const hr = s.rAll && s.rAll.length > 0 ? s.rAll[0] : NaN;
      const zr = s.rAll && s.rAll.length > 1 && isFinite(s.rAll[1]) ? s.rAll[1] : 0;
      const ghost = isFinite(hr);
      // world window: fit the vehicle (drawn 2.5x, with its force arrows), the target
      // and the ghost in BOTH directions, at least 6 m x 3 m, so the vehicle is always
      // in view whatever the panel's aspect ratio; extra height goes above the ground
      const fin = (v, d) => (isFinite(v) ? v : d);
      const zv = fin(z, 0), av = fin(alt, 0);
      const R = 1.3;                                         // vehicle half-extent incl. arrows
      const xs = [zv - R, zv + R, zr - 0.5, zr + 0.5];
      const ys = [av - R, av + R];
      if (ghost) { xs.push(zr + 1.2); ys.push(hr + 0.4); }   // room for the "(z_r, h_r)" label
      if (Math.abs(av) < 20) ys.push(-0.4);                  // ground in view unless far off
      const minX = Math.min(...xs), maxX = Math.max(...xs);
      const minY = Math.min(...ys), maxY = Math.max(...ys);
      const sc = Math.min(w / Math.max(6, maxX - minX), h / Math.max(3, maxY - minY));
      const span = w / sc, visH = h / sc;
      const x0 = (maxX + minX) / 2 - span / 2;
      const y0 = minY - 0.15 * (visH - (maxY - minY));        // a little of the slack below
      const P = (wx, wy) => [(wx - x0) * sc, h - (wy - y0) * sc];

      // ground and 1 m ticks
      const [gx0, gy] = P(x0, 0);
      ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(gx0, gy); ctx.lineTo(w, gy); ctx.stroke();
      ctx.fillStyle = css('--text-muted'); ctx.font = '10px var(--font-sans, system-ui)';
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      for (let k = Math.ceil(x0); k <= x0 + span; k++) {
        const [tx] = P(k, 0);
        ctx.beginPath(); ctx.moveTo(tx, gy); ctx.lineTo(tx, gy + 4); ctx.stroke();
        if (gy + 16 < h && tx > 12 && tx < w - 12) ctx.fillText(`${k}`, tx, gy + 5);   // skip edge labels
      }

      // target on the ground at z_r
      const tw = 0.3, thh = 0.15;
      const [tx0, ty0] = P(zr - tw / 2, thh);
      ctx.fillStyle = css('--series-3');
      ctx.fillRect(tx0, ty0, tw * sc, thh * sc);

      // commanded position ghost
      if (ghost) {
        const [cx, cy] = P(zr, hr);
        ctx.strokeStyle = css('--text-muted'); ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.arc(cx, cy, 6, 0, 2 * Math.PI); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, gy); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = css('--text-muted'); ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
        ctx.fillText('(z_r, h_r)', cx + 9, cy);
      }

      // vehicle (Fig. 21-1): pod, arm to ±d, rotor discs; drawn 2.5x for visibility
      const k = 2.5;
      const c = Math.cos(th), sn = Math.sin(th);
      const body = (bx, by) => P(z + k * (c * bx - sn * by), alt + k * (sn * bx + c * by));
      const dd = p ? p.d : 0.3;
      const [lx, ly] = body(-dd, 0), [rx, ry] = body(dd, 0);
      ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(lx, ly); ctx.lineTo(rx, ry); ctx.stroke();
      ctx.fillStyle = css('--series-1'); ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 1;
      ctx.beginPath();
      [[0.08, 0.05], [0.08, -0.05], [-0.08, -0.05], [-0.08, 0.05]].forEach(([bx, by], i) => { const [px, py] = body(bx, by); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
      ctx.closePath(); ctx.fill(); ctx.stroke();
      for (const [ex, ey] of [[lx, ly], [rx, ry]]) {
        ctx.save(); ctx.translate(ex, ey); ctx.rotate(-th);
        ctx.beginPath(); ctx.ellipse(0, -3, 0.11 * k * sc, 3, 0, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        ctx.restore();
      }

      // rotor force arrows along the body normal, length ∝ f / f_max
      const fmax = p ? p.f_max : 10;
      const Lmax = 0.4 * k;
      [[dd, 0], [-dd, 1]].forEach(([bx, idx]) => {
        const f = s.uAll ? s.uAll[idx] : 0;
        const frac = Math.max(0, Math.min(1.2, f / fmax));
        const sat = s.satAll && s.satAll[idx];
        const [ax, ay] = body(bx, 0.03);
        const nx = -sn, ny = c;   // body normal in world coordinates
        const ex = ax + nx * frac * Lmax * sc, ey = ay - ny * frac * Lmax * sc;
        ctx.strokeStyle = css(sat ? '--critical' : '--series-2'); ctx.fillStyle = ctx.strokeStyle; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ex, ey); ctx.stroke();
        if (frac > 0.02) {
          const ux = nx, uy = -ny, px = -uy, py = ux;
          ctx.beginPath();
          ctx.moveTo(ex + ux * 6, ey + uy * 6);
          ctx.lineTo(ex + px * 4, ey + py * 4);
          ctx.lineTo(ex - px * 4, ey - py * 4);
          ctx.fill();
        }
      });

      // gravity cue
      ctx.strokeStyle = css('--text-muted'); ctx.lineWidth = 1;
      const gxx = w - 18, gyy = 14;
      ctx.beginPath(); ctx.moveTo(gxx, gyy); ctx.lineTo(gxx, gyy + 22); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gxx - 4, gyy + 16); ctx.lineTo(gxx, gyy + 22); ctx.lineTo(gxx + 4, gyy + 16); ctx.stroke();
      ctx.fillStyle = css('--text-muted'); ctx.textAlign = 'right'; ctx.textBaseline = 'top';
      ctx.fillText('g', gxx - 6, gyy + 4);
    },

    // Problem statements, paraphrased from controlbook.pdf (PDF page numbers).
    problems: {
      ch2: {
        id: 'F.2', page: 395,
        sim: { tEnd: 10, tStep: 0 },
        statement: [
          '(a) Using the configuration variables z<sub>v</sub>, h and θ, write an expression for the kinetic energy of the system.',
          '(b) Create an animation of the planar VTOL system. The inputs to the animation should be z<sub>v</sub>, z<sub>t</sub>, h and θ.',
        ],
      },
      ch3: {
        id: 'F.3', page: 395,
        sim: { tEnd: 4, tStep: 0 },
        statement: [
          '(a) Find the potential energy for the system.',
          '(b) Define the generalized coordinates and damping forces.',
          '(c) Find the generalized forces. The right and left forces are more easily modeled as a total force on the center of mass and a torque about the center of mass.',
          '(d) Derive the equations of motion for the planar VTOL using the Euler-Lagrange equations.',
          '(e) Referring to Appendices P.1–P.3, write a class that implements the equations of motion. Simulate the system using variable force inputs f<sub>r</sub> and f<sub>ℓ</sub>; the output connects to the F.2 animation.',
        ],
      },
      ch4: {
        id: 'F.4', page: 395,
        sim: { tEnd: 6, tStep: 0 },
        statement: [
          'Use F = f<sub>r</sub> + f<sub>ℓ</sub> and τ = d(f<sub>r</sub> − f<sub>ℓ</sub>) as the inputs; then f<sub>r</sub> = F/2 + τ/2d and f<sub>ℓ</sub> = F/2 − τ/2d (pp. 395–396).',
          '(a) Find the equilibria of the system.',
          '(b) Linearize the equations about the equilibria using Jacobian linearization.',
          '(c) If possible, linearize the system using feedback linearization.',
        ],
      },
      ch5: {
        id: 'F.5', page: 396,
        sim: { tEnd: 6, tStep: 0 },
        statement: [
          'The VTOL splits into longitudinal dynamics (F̃ in, h̃ out) and lateral dynamics (τ̃ in, z̃ out, with θ̃ in between).',
          '(a) Start with the linearized equations of motion and use the Laplace transform to convert them to the s-domain.',
          '(b) For the longitudinal dynamics, find the transfer function from F̃(s) to H̃(s).',
          '(c) For the lateral dynamics, find the transfer function from τ̃(s) to the intermediate state Θ̃(s) and to the output Z̃(s). Find the transfer function from Θ̃(s) to Z̃(s).',
          '(d) Draw a block diagram of the open-loop longitudinal and lateral systems.',
        ],
      },
      ch6: {
        id: 'F.6', page: 396,
        sim: { tEnd: 6, tStep: 0 },
        statement: [
          '(a) With x<sub>lon</sub> = (h̃, h̃̇)ᵀ, u<sub>lon</sub> = F̃ and y<sub>lon</sub> = h̃, find A, B, C, D.',
          '(b) With x<sub>lat</sub> = (z̃, θ̃, z̃̇, θ̃̇)ᵀ, u<sub>lat</sub> = τ̃ and y<sub>lat</sub> = (z̃, θ̃)ᵀ, find A, B, C, D.',
        ],
      },
      ch7: {
        id: 'F.7', page: 397,
        desiredPoles: [{ re: -0.2, im: 0 }, { re: -0.3, im: 0 }],
        sim: { tEnd: 40 },
        statement: [
          'Longitudinal (altitude) dynamics only.',
          '(a) From the F.5 transfer function F̃ → h̃, find the open-loop poles.',
          '(b) With PD control as in Fig. 7-2 (derivative on the output), find the closed-loop transfer function h̃<sub>r</sub> → h̃ and its poles in terms of k<sub>P</sub>, k<sub>D</sub>.',
          '(c) Pick k<sub>P</sub>, k<sub>D</sub> to put the closed-loop poles at −0.2 and −0.3.',
          '(d) Implement the PD altitude controller and plot the step response.',
        ],
      },
      ch8: {
        id: 'F.8', page: 397,
        trh: 8, zetah: 0.707, trth: 0.8, zetath: 0.707, Msep: 10, zetaz: 0.707,
        sim: { tEnd: 50 },
        statement: [
          '(a) Altitude: t<sub>r</sub> ≈ 8 s, ζ = 0.707. Find Δ<sup>d</sup><sub>cl</sub>(s), its poles, and k<sub>P<sub>h</sub></sub>, k<sub>D<sub>h</sub></sub>. Verify the step response.',
          '(b) Lateral, by successive loop closure: inner loop τ → θ. Find k<sub>P<sub>θ</sub></sub>, k<sub>D<sub>θ</sub></sub> for t<sub>r<sub>θ</sub></sub> = 0.8 s, ζ<sub>θ</sub> = 0.707.',
          '(c) Find the inner loop\'s DC gain k<sub>DC<sub>θ</sub></sub>.',
          '(d) Replace the inner loop by its DC gain. Find k<sub>P<sub>z</sub></sub>, k<sub>D<sub>z</sub></sub> for t<sub>r<sub>z</sub></sub> = 10 t<sub>r<sub>θ</sub></sub>, ζ<sub>z</sub> = 0.707.',
          '(e) Implement it with z<sub>r</sub> a square wave of 3 ± 2.5 m at 0.08 Hz.',
          '(f) With 0 ≤ f<sub>r</sub>, f<sub>ℓ</sub> ≤ f<sub>max</sub> = 10 N, tune the altitude rise time and the outer-loop rise time for the fastest response without saturation (p. 398).',
        ],
      },
      ch9: {
        id: 'F.9', page: 398,
        sim: { tEnd: 50 },
        statement: [
          '(a) Altitude loop with PD: system type? Steady-state error to a step, ramp and parabola in h<sub>r</sub>? With an integrator? Type with respect to an input disturbance, for PD and PID?',
          '(b) Inner lateral loop with PD: type, errors to a step, ramp and parabola in θ<sub>r</sub>, and type for an input disturbance.',
          '(c) Outer lateral loop with PD: type and errors in z<sub>r</sub>; with an integrator; type for an input disturbance, PD and PID.',
        ],
      },
      p6: {
        id: 'F.P.6', page: 398,
        sim: { tEnd: 60 },
        statement: [
          '(a) Add an integrator to the altitude loop (PID). Put the characteristic equation in Evans form, plot the root locus versus k<sub>I<sub>h</sub></sub>, and pick a k<sub>I<sub>h</sub></sub> that does not move the other closed-loop poles much.',
          '(b) Do the same for the outer lateral loop and k<sub>I<sub>z</sub></sub>.',
        ],
      },
      ch10: {
        id: 'F.10', page: 399,
        kIh: 0.01, kIz: -0.0005, sigma: 0.05,
        sim: { tEnd: 80 },
        mismatch: { mc: 14, Jc: -12, d: 9, mu: -17 },   // a fixed "α = 0.2" draw so the page is repeatable
        statement: [
          'Implement PID using only the measured outputs.',
          '(a) Let m<sub>c</sub>, J<sub>c</sub>, d and μ vary by up to 20% (α = 0.2).',
          '(b) The controller sees only z, θ, h, z<sub>r</sub> and h<sub>r</sub>. Implement the nested loops of F.8 with dirty derivatives (σ = 0.05), and tune the integrators so there is no steady-state error.',
        ],
      },
      ch11: {
        id: 'F.11', page: 399,
        sim: { tEnd: 50 },
        statement: [
          '(a) Using ω<sub>n<sub>h</sub></sub>, ζ<sub>h</sub>, ω<sub>n<sub>z</sub></sub>, ζ<sub>z</sub> from F.8, choose longitudinal poles with damping above ζ<sub>h</sub> and natural frequency above ω<sub>n<sub>h</sub></sub>, and lateral poles with damping above ζ<sub>z</sub> and natural frequency above ω<sub>n<sub>z</sub></sub>.',
          '(b) Use the F.6 state-space models. (c) Check controllability: rank 𝒞<sub>A,B</sub> = n.',
          '(d) Find K so eig(A − BK) are the desired poles, and k<sub>r<sub>h</sub></sub>, k<sub>r<sub>z</sub></sub> for unity DC gain from h<sub>r</sub> to h and from z<sub>r</sub> to z.',
          '(e) Tune. Which pole changes reduce the rise time? Which reduce the overshoot?',
        ],
      },
      ch12: {
        id: 'F.12', page: 400,
        pIh: -0.4, pIz: -0.4,
        sim: { tEnd: 60, tDist: 0 },
        dists: { Fwind: 0.1 },
        mismatch: { mc: 14, Jc: -12, d: 9, mu: -17 },
        statement: [
          '(a) Add an integrator with anti-windup to the altitude loop and to the position loop of F.11.',
          '(b) Let the parameters vary by up to 20% and add a constant 0.1 N wind force to the z dynamics: z̈ = (−(f<sub>r</sub> + f<sub>ℓ</sub>) sin θ − μż + F<sub>wind</sub>)/(m<sub>c</sub> + 2m<sub>r</sub>).',
          '(c) Tune the integrator poles (and other gains if needed) for good tracking.',
        ],
      },
      ch13: {
        id: 'F.13', page: 400,
        obsFactor: 10,
        sim: { tEnd: 60, tDist: 0 },
        statement: [
          '(a) Exact parameters (α = 0) and no input disturbance.',
          '(b) Check observability: rank 𝒪<sub>A,C</sub> = n.',
          '(c) Add an observer and use x̂ in the F.12 controller. Tune the controller and observer poles.',
          '(d) Plot the states and their estimates together.',
          '(e) Add an input force disturbance of 1.0 and an input torque disturbance of 0.1, and see the steady-state error that remains even with integrators.',
        ],
      },
      ch14: {
        id: 'F.14', page: 401,
        pDh: -1, pDz: -1, pDth: -10,
        sim: { tEnd: 60, tDist: 0 },
        dists: { wind: 1.0, ah: 1.0 },
        mismatch: { mc: 14, Jc: -12, d: 9, mu: -17 },
        statement: [
          '(a) α = 0.2, an altitude disturbance of 1.0 and a wind disturbance of 1.0 m/s (the book\'s snippet adds the wind to ż and 1.0 to ḧ). Without a disturbance observer the controller is not robust to them.',
          '(b) Add a disturbance observer to both controllers. Verify the estimator\'s steady-state error is gone and the disturbances are compensated. Tune.',
        ],
      },
      ch15: {
        id: 'F.15', page: 401,
        sim: { tEnd: 30, tStep: 0 },
        statement: [
          '(a) Draw by hand the Bode plot of the altitude transfer function from force F̃ to altitude h̃. Use the bode command and compare your results.',
          '(b) Draw by hand the Bode plot of the inner-loop transfer function for the lateral dynamics from torque τ to angle θ. Use the bode command and compare your results.',
          '(c) Draw by hand the Bode plot of the outer-loop transfer function for the lateral dynamics from angle θ to position z. Use the bode command and compare your results.',
        ],
      },
      ch16: {
        id: 'F.16', page: 402,
        parabola: 5, wno: 30, wdin: 2, thetaNoise: 0.1, wr: 0.1, wdout: 0.01,
        sim: { tEnd: 60 },
        statement: [
          'Altitude loop with the F.10 PID (dirty derivative): (a) tracking error to a parabola of curvature 5; (b) % of noise above ω<sub>no</sub> = 30 rad/s that shows up in h.',
          'Inner lateral loop with the F.8 PD (dirty derivative): (c) % of an input disturbance below ω<sub>d<sub>in</sub></sub> = 2 rad/s in the output; (d) what θ sensor (band and size) keeps the θ noise under 0.1°?',
          'Outer lateral loop with the F.10 PID: (e) tracking error (%) for z<sub>r</sub> content below ω<sub>r</sub> = 0.1 rad/s; (f) % of an output disturbance below ω<sub>d<sub>out</sub></sub> = 0.01 rad/s in the output.',
        ],
      },
      ch17: {
        id: 'F.17', page: 402,
        sim: { tEnd: 60 },
        statement: [
          'Use the F.10 gains (p. 402).',
          '(a) Altitude loop under PID: PM, GM, open- and closed-loop Bode, closed-loop bandwidth vs. crossover.',
          '(b) Inner lateral loop under PD: the same.',
          '(c) Outer lateral loop under PD: the same, on the same plot as the inner loop.',
          '(d) Bandwidth separation between the inner and outer loops: is successive loop closure justified?',
        ],
      },
      ch18: {
        id: 'F.18', page: 403,
        lon: { wr: 0.1, gr: 0.01, wn: 200, gn: 1e-4, pm: 60 },
        inner: { wco: 10, pm: 60 },
        outer: { wco: 1, wno: 100, gno: 1e-5, pm: 60 },
        sim: { tEnd: 40 },
        statement: [
          '(a) Altitude: F = F<sub>e</sub> + C<sub>lon</sub>(s)E<sub>h</sub>(s) with e<sub>h</sub> = h<sup>r</sup><sub>f</sub> − h<sub>m</sub>. Reject constant input disturbances; track content below ω<sub>r</sub> = 0.1 rad/s to γ<sub>r</sub> = 0.01; attenuate noise above ω<sub>n</sub> = 200 rad/s by γ<sub>n</sub> = 10⁻⁴; PM ≈ 60°; add a prefilter.',
          '(b) Inner lateral loop: C<sub>lat<sub>in</sub></sub>(s) with PM ≈ 60° and bandwidth ≈ ω<sub>co</sub> = 10 rad/s, plus a low-pass filter (p. 404).',
          '(c) Outer lateral loop: crossover ≈ 1 rad/s; reject constant input disturbances; attenuate noise above 100 rad/s by γ<sub>no</sub> = 10⁻⁵; PM ≈ 60°; prefilter to reduce overshoot.',
          '(d) Implement C(s) and F(s) in simulation (state space or digital filters).',
        ],
      },
    },
  };
})();
