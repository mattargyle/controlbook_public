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
    { key: 'm', label: 'm', unit: 'kg', value: 0.5, min: 0.1, max: 2.0, step: 0.01 },
    { key: 'ell', label: 'ℓ', unit: 'm', value: 0.3, min: 0.1, max: 1.0, step: 0.01 },
    { key: 'b', label: 'b', unit: 'N·m·s', value: 0.01, min: 0, max: 0.2, step: 0.001 },
    { key: 'tau_max', label: 'τ<sub>max</sub>', unit: 'N·m', value: 1.0, min: 0.2, max: 5, step: 0.05 },
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
