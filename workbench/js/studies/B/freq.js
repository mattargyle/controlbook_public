// Design Study B, Chapters 15–18: frequency response of the two loops, frequency
// specs and margins under the B.10 PID gains, and the loopshaping design of B.18.
//   P_in(s)  = −(1/J) / (s² − (m1+m2)g/J),   J = m1ℓ/6 + 2m2ℓ/3     (F → θ)
//   P_out(s) = (−(2ℓ/3)s² + g) / s²                                   (θ → z)
// The controllers are hw16.py's: C_in = ((k_Dθ + σk_Pθ)s + k_Pθ)/(σs + 1) and
// C_out = ((k_Dz + σk_Pz)s² + (k_Pz + σk_Iz)s + k_Iz)/(s(σs + 1)).
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const { tex, texPole, fmt } = M;
  const B = WB.studies.B;
  const lib = () => B.lib;
  const PD = () => WB.pd;
  const DEG = Math.PI / 180;
  const W = T.logspace(-4, 5, 900);
  const W15 = T.logspace(-2, 3, 600);   // the range of Figs. 15-13 and 15-14
  const { db } = T;

  const Pin = (p) => { const J = p.m1 * p.ell / 6.0 + p.m2 * 2 * p.ell / 3.0; return T.tf([-1 / J], [1, 0, -(p.m1 + p.m2) * p.g / J]); };
  const Pout = (p) => T.tf([-2 * p.ell / 3.0, 0, p.g], [1, 0, 0]);
  const Cin = (g) => T.pid({ kP: g.kPth, kD: g.kDth, sigma: g.sigma });
  const Cout = (g) => T.pid({ kP: g.kPz, kI: g.kIz, kD: g.kDz, sigma: g.sigma });

  // Bode with the low-frequency phase shifted into (−270°, 90°], so a negative
  // DC gain reads −180° (the usual place for margins).
  function bodeB(G, ws = W) {
    const b = T.bode(G, ws);
    let k = 0;
    while (b.phase[0] + 360 * k > 90) k--;
    while (b.phase[0] + 360 * k <= -270) k++;
    return { mag: b.mag, phase: b.phase.map((v) => v + 360 * k) };
  }
  // Every gain crossover with its PM, every −180° phase crossing with its GM,
  // and the ω → 0 case when the loop starts at −180° (negative DC gain).
  function marginsB(Lg) {
    const ws = T.logspace(-5, 5, 6000), { mag, phase } = bodeB(Lg, ws);
    const pms = [], gms = [];
    for (let i = 1; i < ws.length; i++) {
      if ((mag[i - 1] - 1) * (mag[i] - 1) < 0) {
        const t = Math.log(mag[i - 1]) / (Math.log(mag[i - 1]) - Math.log(mag[i]));
        const w = Math.exp(Math.log(ws[i - 1]) + t * (Math.log(ws[i]) - Math.log(ws[i - 1])));
        const ph = phase[i - 1] + t * (phase[i] - phase[i - 1]);
        pms.push({ w, pm: ((180 + ph) % 360 + 540) % 360 - 180 });
      }
      const a = Math.floor((phase[i - 1] + 180) / 360 + 1e-9), b2 = Math.floor((phase[i] + 180) / 360 + 1e-9);
      if (a !== b2 && i > 1) {
        const target = 360 * Math.max(a, b2) - 180;
        const t = (target - phase[i - 1]) / (phase[i] - phase[i - 1]);
        const w = Math.exp(Math.log(ws[i - 1]) + t * (Math.log(ws[i]) - Math.log(ws[i - 1])));
        gms.push({ w, gm: 1 / T.mag(Lg, w) });
      }
    }
    const w0 = ((phase[0] + 180) % 360 + 360) % 360;
    if (w0 < 0.5 || w0 > 359.5) {
      gms.unshift({ w: 0, gm: 1 / Math.abs(T.dcgain(Lg)) });
    }
    return { pms, gms, pm: pms.length ? pms[0].pm : Infinity, wc: pms.length ? pms[0].w : NaN };
  }
  // control.bandwidth's definition: 3 dB below |T(0)|.
  const bandwidth = (Tc) => T.bandwidth(Tc, W);
  const gmText = (gms) => (gms.length ? gms.map((c) => `${fmt(db(c.gm), 3)} dB at ${c.w === 0 ? 'ω → 0' : fmt(c.w, 3) + ' rad/s'}`).join('; ') : '∞');
  const pmText = (pms) => (pms.length ? pms.map((c) => `${fmt(c.pm, 3)}° at ${fmt(c.w, 3)} rad/s`).join('; ') : 'no crossover');

  // B.10 gains (Listing 10.3) on the book parameters.
  function b10(sys) {
    const p = { ...Object.fromEntries(sys.params.map((q) => [q.key, q.value])), ...sys.constants };
    const pr = sys.problems.ch10;
    const g = lib().slcGains(p, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaZ: pr.zetaZ, formula: 'book' });
    return { kPth: g.kPth, kDth: g.kDth, kPz: g.kPz, kDz: g.kDz, kIz: pr.ki, sigma: pr.sigma };
  }
  // The B.10 gains answer B.10(c), so Work mode starts from placeholder gains in
  // st.w (the Ch 10 tab's starting values, not the answer) and offers the student's
  // own Ch 10 gains; the workbench's B.10 gains only once B.10(c) is solved.
  const W_PID = { kPth: -60, kDth: -8, kPz: -0.1, kDz: -0.2, kIz: 0, sigma: 0.05 };
  const pidDefaults = (sys) => ({ ...b10(sys), loop: 'inner', w: { ...W_PID } });
  const shows = (ctx, key) => !ctx.S || ctx.S.mode === 'explore' || ctx.app.isSolved(key);
  // Gains in use: Explore (and tools/regress_B.py, which passes no S) reads st itself.
  const gainsOf = (ctx) => (!ctx.S || ctx.S.mode === 'explore' ? ctx.st : ctx.st.w);
  function gainButtons(sec, ctx) {
    const row = el('div', { class: 'btn-row' });
    if (ctx.S.mode === 'work') {
      row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Use my Ch 10 gains', title: 'Copy the Work-mode gains and σ from the Ch 10 tab', onclick: () => {
        const c = ctx.S.ch.ch10;
        if (c && c.w) Object.assign(ctx.st.w, { kPth: c.w.kPth, kDth: c.w.kDth, kPz: c.w.kPz, kDz: c.w.kDz, kIz: c.w.kIz ?? 0, sigma: c.sigma });
        ctx.update();
      } }));
    }
    if (shows(ctx, 'B.10/c1')) {
      row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'B.10 gains', onclick: () => {
        const g = b10(ctx.sys);
        // the listing's k_Iz is the B.10(c) tuning answer
        if (!shows(ctx, 'B.10/c')) delete g.kIz;
        Object.assign(gainsOf(ctx), g);
        ctx.update();
      } }));
    }
    sec.append(row);
  }
  function gainControls(parent, ctx) {
    const sec = section(parent, ctx.S.mode === 'work' ? 'Your B.10 gains' : 'B.10 gains', 'p. 163–164, p. 297');
    const obj = () => gainsOf(ctx);
    const sl = (k, label, min, max) => slider(sec, { label, min, max, step: (max - min) / 2000, sig: 4, ...bind(ctx, k, obj) });
    sl('kPth', 'k<sub>Pθ</sub>', -300, 0); sl('kDth', 'k<sub>Dθ</sub>', -40, 0);
    sl('kPz', 'k<sub>Pz</sub>', -1, 0.5); sl('kIz', 'k<sub>Iz</sub>', -0.5, 0.2); sl('kDz', 'k<sub>Dz</sub>', -1.5, 0.5);
    slider(sec, { label: 'σ', unit: 's', min: 0.005, max: 0.3, step: 0.001, sig: 3, ...bind(ctx, 'sigma', obj) });
    gainButtons(sec, ctx);
  }
  function loopToggle(parent, ctx, options) {
    segmented(parent, { label: 'Loop', options, ...bind(ctx, 'loop') });
  }
  // The time simulation runs the B.10 controller with these gains.
  function pidController(ctx, { linear = false } = {}) {
    const st = gainsOf(ctx), p = ctx.pModel;
    const g = { kPth: st.kPth, kDth: st.kDth, kPz: st.kPz, kDz: st.kDz, kIz: st.kIz, kDC: lib().dcGain(p, st.kPth) };
    return lib().slcPID({ g, p, Ts: ctx.S.sim.Ts, uLim: ctx.sys.uLimit(p), sigma: st.sigma, linear });
  }
  // The open-loop poles include the plant's, which answer B.5(b) (inner) and
  // B.5(c) (outer): Work mode shows them once that part is solved.
  function clMarkers(ctx, Lg, label) {
    const ol = shows(ctx, label === 'inner' ? 'B.5/b' : 'B.5/c');
    const mk = ol ? L.roots(Lg.den).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole ${i + 1}`, noFit: Math.hypot(p.re, p.im) > 60 })) : [];
    L.roots(L.polyAdd(Lg.den, Lg.num)).forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `${label} closed-loop pole ${i + 1}`, noFit: Math.hypot(p.re, p.im) > 60 }));
    return { markers: mk, fitR: 12 };
  }

  // ------------------------------------------------------------ Chapter 15 --
  B.chapters.ch15 = {
    id: 'ch15', num: 15, tab: 'Ch 15', title: 'Frequency response', pages: 'pp. 261–282',
    linear: false,
    defaults(sys) { return { ...pidDefaults(sys), asym: true }; },
    simDefaults(sys) { return sys.problems.ch15.sim; },
    controller: pidController,
    buildControls(parent, ctx) {
      const sec = section(parent, 'Plant transfer functions', 'p. 277–278 · Eq. 15.16, 15.18');
      loopToggle(sec, ctx, [{ value: 'inner', label: 'P_in: F → θ' }, { value: 'outer', label: 'P_out: θ → z' }]);
      segmented(sec, { label: 'Straight-line approximation', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'asym') });
      if (ctx.S.mode === 'work') {
        sec.append(el('p', { class: 'muted small', text: 'The time plots below run the B.10 controller with your gains (Use my Ch 10 gains), since the open-loop pendulum just falls over. Each Bode plot appears once you have drawn it by hand (problem panel).' }));
        gainButtons(sec, ctx);
      } else {
        sec.append(el('p', { class: 'muted small', text: 'The time plots below run the B.10 controller, since the open-loop pendulum just falls over. In Work mode each Bode plot appears once you have drawn it by hand (problem panel).' }));
      }
    },
    // B.15 is drawn by hand: in Work mode each loop's Bode plot (and its poles and
    // zeros) appears once that part is marked done.
    drawn(ctx) { return ctx.S.mode === 'explore' || ctx.app.isSolved(ctx.st.loop === 'inner' ? 'B.15/a' : 'B.15/b'); },
    bode(ctx) {
      if (!this.drawn(ctx)) return null;
      const p = ctx.pModel, inner = ctx.st.loop === 'inner';
      const G = inner ? Pin(p) : Pout(p), b = bodeB(G, W15);
      const lines = [{ label: inner ? 'P_in(jω)' : 'P_out(jω)', ...b, color: '--series-1' }];
      if (ctx.st.asym) {
        const mg = (p.m1 + p.m2) * p.g, J = p.m1 * p.ell / 6 + p.m2 * 2 * p.ell / 3;
        const wp = Math.sqrt(mg / J), q = Math.sqrt(3 * p.g / (2 * p.ell));
        const am = inner ? W15.map((w) => (w < wp ? 1 / mg : (1 / mg) * (wp / w) ** 2)) : W15.map((w) => (w < q ? p.g / (w * w) : 2 * p.ell / 3));
        lines.push({ label: 'straight-line approx.', mag: am, color: '--text-muted', dash: [5, 4], width: 1.5 });
      }
      return { title: inner ? 'Bode plot of P_in(s)' : 'Bode plot of P_out(s)', w: W15, lines };
    },
    splane(ctx) {
      if (!this.drawn(ctx)) return { markers: [] };
      const p = ctx.pModel, inner = ctx.st.loop === 'inner', G = inner ? Pin(p) : Pout(p);
      const mk = L.roots(G.den).map((x, i) => ({ ...x, kind: 'ol', label: `pole ${i + 1}` }));
      T.zeros(G).forEach((x, i) => mk.push({ ...x, kind: 'olzero', label: `zero ${i + 1}` }));
      return { markers: mk };
    },
    math(ctx) {
      const p = ctx.pModel, mg = (p.m1 + p.m2) * p.g, J = p.m1 * p.ell / 6 + p.m2 * 2 * p.ell / 3;
      const wp = Math.sqrt(mg / J), q = Math.sqrt(3 * p.g / (2 * p.ell));
      return [
        { title: 'Frequency response', page: 'p. 264 · Eq. 15.4',
          theory: 'u = A\\sin\\omega_0 t \\;\\Rightarrow\\; y_{ss} = A|P(j\\omega_0)|\\sin\\big(\\omega_0 t + \\angle P(j\\omega_0)\\big)' },
        { title: 'Bode canonical form', page: 'p. 266 · Eq. 15.5–15.7',
          theory: 'P(j\\omega) = K\\,\\frac{\\prod (1 + j\\omega/z_i)}{(j\\omega)^q \\prod (1 + j\\omega/p_i)}:\\quad 20\\log|P| = 20\\log K + \\textstyle\\sum 20\\log|1 + j\\omega/z_i| - 20q\\log\\omega - \\sum 20\\log|1 + j\\omega/p_i|' },
        { title: 'Building blocks', page: 'p. 266–270',
          theory: '\\frac{1}{(j\\omega)^2}: -40\\,\\text{dB/dec},\\; -180^\\circ;\\quad (1 \\pm j\\tfrac{\\omega}{a}): +20\\,\\text{dB/dec above } a,\\; \\pm\\tan^{-1}\\tfrac{\\omega}{a}' },
        { title: 'Bode form of P_in', page: 'p. 277 · Eq. 15.16–15.17', answers: 'B.15/a',
          theory: 'P_{in}(s) = \\frac{-1/J}{s^2 - (m_1+m_2)g/J},\\quad J = m_1\\tfrac{\\ell}{6} + m_2\\tfrac{2\\ell}{3}',
          symbolic: 'P_{in}(s) = \\frac{-1/J}{(s + \\omega_p)(s - \\omega_p)} \\;\\Rightarrow\\; P_{in}(j\\omega) = \\frac{\\frac{1}{(m_1+m_2)g}}{(1 + j\\frac{\\omega}{\\omega_p})(1 - j\\frac{\\omega}{\\omega_p})},\\quad \\omega_p = \\sqrt{(m_1+m_2)g/J}',
          numbers: `\\frac{1}{(m_1+m_2)g} = ${tex(1 / mg)}\\;(${tex(db(1 / mg))}\\,\\text{dB}),\\quad \\omega_p = ${tex(wp)}`,
          note: 'The book’s constant in Eq. 15.17 disagrees with the one in its phase expression (ISSUES.md).' },
        { title: 'Phase of P_in', page: 'p. 277–278 · Fig. 15-13', answers: 'B.15/a',
          theory: '\\angle P_{in}(j\\omega) = \\angle K - \\angle(1 + j\\tfrac{\\omega}{\\omega_p}) - \\angle(1 - j\\tfrac{\\omega}{\\omega_p})',
          symbolic: '(1 + j\\tfrac{\\omega}{\\omega_p})(1 - j\\tfrac{\\omega}{\\omega_p}) = 1 + \\tfrac{\\omega^2}{\\omega_p^2} > 0 \\;\\Rightarrow\\; \\angle P_{in} = 0^\\circ \\text{ for all } \\omega' },
        { title: 'Bode form of P_out', page: 'p. 277–278 · Eq. 15.18–15.19', answers: 'B.15/b',
          theory: 'P_{out}(s) = \\frac{-\\frac{2\\ell}{3}s^2 + g}{s^2}',
          symbolic: 'P_{out}(s) = -\\frac{2\\ell}{3}\\frac{(s - q)(s + q)}{s^2},\\; q = \\sqrt{\\tfrac{3g}{2\\ell}} \\;\\Rightarrow\\; P_{out}(j\\omega) = \\frac{g\\,(1 + j\\frac{\\omega}{q})(1 - j\\frac{\\omega}{q})}{(j\\omega)^2}',
          numbers: `g = ${tex(p.g)}\\;(${tex(db(p.g))}\\,\\text{dB}),\\quad q = ${tex(q)},\\quad |P_{out}| \\to \\tfrac{2\\ell}{3} = ${tex(2 * p.ell / 3)} \\text{ above } q`,
          note: 'The book’s Bode constant for P_out (0.174) is not the low-frequency gain (ISSUES.md).' },
      ];
    },
    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      const mg = () => (p().m1 + p().m2) * p().g, J = () => p().m1 * p().ell / 6 + p().m2 * 2 * p().ell / 3;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch15, [
        {
          id: 'a', title: '(a) Draw by hand the Bode plot of the inner loop, F to θ',
          html: 'On paper: put your B.5 transfer function from F̃ to Θ̃ (b = 0) in Bode canonical form and sketch the straight-line magnitude and phase. When you are done, click the button to see the comparison part and the workbench\'s Bode plot (Loop: P_in).',
          done: 'I\'ve drawn it: next',
          solution: () => [{ tex: 'P_{in}(j\\omega) = \\frac{\\frac{1}{(m_1+m_2)g}}{(1 + j\\frac{\\omega}{\\omega_p})(1 - j\\frac{\\omega}{\\omega_p})},\\quad \\omega_p = \\sqrt{\\frac{(m_1+m_2)g}{m_1\\frac{\\ell}{6} + m_2\\frac{2\\ell}{3}}}' }, { html: 'Flat, then −40 dB/dec above ω<sub>p</sub>; the phases of the RHP and LHP poles cancel, so the phase is 0° (Fig. 15-13, p. 278).' }],
        },
        {
          id: 'a2', title: '(a) Compare with the bode command: P<sub>in</sub>', after: 'a',
          inputs: { K: '|P<sub>in</sub>(0)|', wp: 'corner [rad/s]', s: 'slope above [dB/dec]', ph: 'phase [°]' },
          check: (v) => PD().checkNumbers(v, { K: 1 / mg(), wp: Math.sqrt(mg() / J()), s: -40, ph: 0 }, { K: '|Pin(0)|', wp: 'corner', s: 'slope', ph: 'phase' }),
          solution: () => [{ tex: `|P_{in}(0)| = \\frac{1}{(m_1+m_2)g} = ${tex(1 / mg())}\\;(${tex(db(1 / mg()))}\\,\\text{dB}),\\; \\omega_p = ${tex(Math.sqrt(mg() / J()))},\\; -40\\text{ dB/dec},\\; 0^\\circ` }, { html: 'An RHP pole and an LHP pole at ±ω<sub>p</sub>: their magnitudes add like two LHP poles, and their phases cancel (Fig. 15-13).' }],
        },
        {
          id: 'b', title: '(b) Draw by hand the Bode plot of the outer loop, θ to z',
          html: 'On paper: put your B.5 transfer function from Θ̃ to Z̃ in Bode canonical form and sketch it. Then click the button to see the comparison part and the Bode plot (Loop: P_out).',
          done: 'I\'ve drawn it: next',
          solution: () => [{ tex: 'P_{out}(j\\omega) = \\frac{g\\,(1 + j\\frac{\\omega}{q})(1 - j\\frac{\\omega}{q})}{(j\\omega)^2},\\quad q = \\sqrt{\\frac{3g}{2\\ell}}' }, { html: '−40 dB/dec below q, flat at 2ℓ/3 above it; the phase is −180° everywhere (Fig. 15-14, p. 279).' }],
        },
        {
          id: 'b2', title: '(b) Compare with the bode command: P<sub>out</sub>', after: 'b',
          inputs: { K: 'K (P ≈ K/(jω)²)', q: 'corner [rad/s]', hi: '|P<sub>out</sub>(j∞)|' },
          check: (v) => PD().checkNumbers(v, { K: p().g, q: Math.sqrt(3 * p().g / (2 * p().ell)), hi: 2 * p().ell / 3 }, { K: 'K', q: 'corner', hi: 'high-frequency gain' }),
          solution: () => [{ tex: `K = g = ${tex(p().g)},\\; q = \\sqrt{3g/2\\ell} = ${tex(Math.sqrt(3 * p().g / (2 * p().ell)))},\\; |P_{out}| \\to \\tfrac{2\\ell}{3} = ${tex(2 * p().ell / 3)}\\;(${tex(db(2 * p().ell / 3))}\\,\\text{dB})` }, { html: '−40 dB/dec below q, flat above it; the phase is ±180° everywhere (Fig. 15-14).' }],
        },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 16 --
  // B.16 answers are functions of the frequency and the gains (signs as the
  // pendulum's: all negative), checked at random arguments.
  const SIGMA = { label: 'σ', lo: 0.01, hi: 0.1 };
  const IN_ARGS = { kPth: { label: 'kPθ', lo: -150, hi: -20 }, kDth: { label: 'kDθ', lo: -15, hi: -1 }, sigma: SIGMA };
  const OUT_ARGS = { kPz: { label: 'kPz', lo: -0.5, hi: -0.02 }, kIz: { label: 'kIz', lo: -0.3, hi: -0.005 }, kDz: { label: 'kDz', lo: -0.8, hi: -0.05 }, sigma: SIGMA };
  // truth(|P_in C_in(jw)|) gives the expected value.
  const inCheck = (ctx, fn, w, truth) => (code) => WB.py.check(ctx, {
    args: { w, ...IN_ARGS },
    items: [{ fn, args: ['w', 'kPth', 'kDth', 'sigma'], truth: (p, a) => truth(T.mag(T.mul(Pin(p), Cin(a)), a.w)) }],
  }, code);
  const PY_IN = 'def loop_in(w, kP, kD, sigma):\n    s = 1j * w\n    J = P.m1 * P.ell / 6 + P.m2 * 2 * P.ell / 3\n    Pin = (-1 / J) / (s**2 - (P.m1 + P.m2) * P.g / J)\n    Cin = kP + kD * s / (sigma * s + 1)\n    return Pin * Cin\n';

  B.chapters.ch16 = {
    id: 'ch16', num: 16, tab: 'Ch 16', title: 'Frequency-domain specs', pages: 'pp. 283–301',
    linear: false,
    defaults(sys) { return pidDefaults(sys); },
    simDefaults(sys) { return sys.problems.ch16.sim; },
    controller: pidController,
    specs(ctx) {
      const p = ctx.pModel, pr = ctx.sys.problems.ch16, g = gainsOf(ctx);
      const Li = T.mul(Pin(p), Cin(g)), Lo = T.mul(Pout(p), Cout(g));
      const BrIn = db(T.mag(Li, pr.wrIn)), BnIn = -db(T.mag(Li, pr.wnoIn));
      const BrOut = db(T.mag(Lo, pr.wrOut));
      return { Li, Lo, BrIn, grIn: 10 ** (-BrIn / 20), BnIn, gnIn: 10 ** (-BnIn / 20), BrOut, grOut: 10 ** (-BrOut / 20), pr };
    },
    buildControls(parent, ctx) {
      gainControls(parent, ctx);
      const sec = section(parent, 'Specs', 'p. 297');
      loopToggle(sec, ctx, [{ value: 'inner', label: 'inner (PD)' }, { value: 'outer', label: 'outer (PID)' }]);
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const s = this.specs(ctx);
        const show = WB.ui.shown(ctx, 'B:ch16:specs');
        const row = WB.ui.metric;
        box.replaceChildren(...(show ? [
          row('inner tracking below 1 rad/s: B_r', `${fmt(s.BrIn, 3)} dB → ${fmt(100 * s.grIn, 3)} %`),
          row('inner noise above 200 rad/s: |PC|', `${fmt(-s.BnIn, 3)} dB → ${fmt(100 * s.gnIn, 3)} %`),
          row('outer tracking below 0.001 rad/s: B_r', `${fmt(s.BrOut, 4)} dB → γ_r = ${fmt(s.grOut, 3)}`),
        ] : [WB.ui.revealButton(ctx, 'B:ch16:specs', 'Reveal the spec readouts')]));
      });
    },
    bode(ctx) {
      const s = this.specs(ctx), p = ctx.pModel, inner = ctx.st.loop === 'inner';
      const show = WB.ui.shown(ctx, 'B:ch16:specs');
      if (inner) {
        return {
          title: 'Inner loop: P_in and P_in·C_in', w: W,
          // the plant's own Bode plot answers B.15(a), (b)
          lines: [...(shows(ctx, 'B.15/a') ? [{ label: 'P_in(jω)', ...bodeB(Pin(p)), color: '--text-muted', width: 1.5 }] : []), { label: 'P_in C_in(jω)', ...bodeB(s.Li), color: '--series-1' }],
          specs: show ? [{ w0: 1e-4, w1: s.pr.wrIn, db: s.BrIn, keep: 'above', color: '--series-3', label: `B_r = ${fmt(s.BrIn, 3)} dB` }, { w0: s.pr.wnoIn, w1: 1e5, db: -s.BnIn, keep: 'below', color: '--series-3', label: `${fmt(-s.BnIn, 3)} dB` }] : [],
          marks: [{ w: s.pr.wrIn, label: 'ω_r = 1' }, { w: s.pr.wnoIn, label: 'ω_no = 200' }],
        };
      }
      return {
        title: 'Outer loop: P_out and P_out·C_out', w: W,
        lines: [...(shows(ctx, 'B.15/b') ? [{ label: 'P_out(jω)', ...bodeB(Pout(p)), color: '--text-muted', width: 1.5 }] : []), { label: 'P_out C_out(jω)', ...bodeB(s.Lo), color: '--series-1' }],
        specs: show ? [{ w0: 1e-4, w1: s.pr.wrOut, db: s.BrOut, keep: 'above', color: '--series-3', label: `B_r = ${fmt(s.BrOut, 4)} dB` }] : [],
        marks: [{ w: s.pr.wrOut, label: 'ω_r = 0.001' }],
      };
    },
    splane(ctx) { const s = this.specs(ctx); return ctx.st.loop === 'inner' ? clMarkers(ctx, s.Li, 'inner') : clMarkers(ctx, s.Lo, 'outer'); },
    math(ctx) {
      const s = this.specs(ctx);
      return [
        // the dirty-derivative form is part of the B.10(c) implementation
        { title: 'Controllers (dirty derivative)', page: 'p. 297', answers: 'B.10/c',
          theory: 'C_{in}(s) = \\frac{(k_{D\\theta} + \\sigma k_{P\\theta})s + k_{P\\theta}}{\\sigma s + 1},\\quad C_{out}(s) = \\frac{(k_{Dz} + \\sigma k_{Pz})s^2 + (k_{Pz} + \\sigma k_{Iz})s + k_{Iz}}{s(\\sigma s + 1)}',
          numbers: `C_{in} = ${T.texTf(Cin(gainsOf(ctx)))},\\quad C_{out} = ${T.texTf(Cout(gainsOf(ctx)))}` },
        { title: 'Tracking', page: 'p. 286 · Eq. 16.4–16.5',
          theory: '|e| \\le \\gamma_r|r| \\text{ for } \\omega \\le \\omega_r \\text{ when } 20\\log|PC| \\ge B_r,\\quad \\gamma_r = 10^{-B_r/20}',
          numbers: `\\text{inner: } B_r = ${tex(s.BrIn)}\\,\\text{dB} \\Rightarrow ${tex(100 * s.grIn)}\\%,\\quad \\text{outer: } B_r = ${tex(s.BrOut)}\\,\\text{dB} \\Rightarrow \\gamma_r = ${tex(s.grOut)}`, spoiler: true,
          note: 'γ_r = 1/|PC| is the large-|PC| approximation of |1/(1 + PC)|. For the inner loop |PC| is not large and PC is negative, so the true low-frequency error ratio is |1/(1 + PC)| = |1 − k_DC|.' },
        { title: 'Noise', page: 'p. 287 · Eq. 16.6',
          theory: '20\\log|PC| \\le 20\\log\\gamma_n \\text{ for } \\omega \\ge \\omega_{no}',
          numbers: `|P_{in}C_{in}(j200)| = ${tex(-s.BnIn)}\\,\\text{dB} \\Rightarrow \\gamma_n = ${tex(s.gnIn)}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const s = () => this.specs(ctx);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch16, [
        { id: 'in', title: 'Inner loop: Bode plots of the plant and of the plant under PD control',
          html: 'Use the B.10 gains. In your code: bode(P_in) and bode(P_in·C_in) on one graph. Here: Loop = inner, and the gains on the right (in Work mode, Use my Ch 10 gains).' },
        { id: 'a', title: '(a) Inner-loop tracking error below 1 rad/s',
          html: 'Percent tracking error (γ<sub>r</sub>, p. 286) for r<sub>θ</sub> content at frequency w (rad/s), as a function of w and the inner gains with the dirty derivative σ. The check calls it at random arguments; w = 1 is the book\'s case.',
          code: { template: 'def track_pct_in(w, kP, kD, sigma):\n    return ...\n', check: inCheck(ctx, 'track_pct_in', { label: 'w', lo: 0.1, hi: 2 }, (Lg) => 100 / Lg) },
          solution: () => [{ tex: `B_r = ${tex(s().BrIn)}\\,\\text{dB} \\Rightarrow \\gamma_r = ${tex(100 * s().grIn)}\\%\;\\text{(current gains)}` }, { html: 'The book reads 6.5 dB → 47% from Fig. 16-10 (p. 297), which matches t<sub>r,θ</sub> = 0.5 s (B.8) gains, not the B.10 listing\'s 0.2 s.' },
            { code: `${PY_IN}\ndef track_pct_in(w, kP, kD, sigma):\n    return 100 / abs(loop_in(w, kP, kD, sigma))` }] },
        { id: 'b', title: '(b) Percent of inner-loop noise above 200 rad/s in θ',
          html: 'Percent of noise at frequency w (rad/s) that shows up in θ, as a function of w and the inner gains.',
          code: { template: 'def noise_pct_in(w, kP, kD, sigma):\n    return ...\n', check: inCheck(ctx, 'noise_pct_in', { label: 'w', lo: 100, hi: 1000 }, (Lg) => 100 * Lg) },
          solution: () => [{ tex: `|PC(j200)| = ${tex(-s().BnIn)}\\,\\text{dB} \\Rightarrow ${tex(100 * s().gnIn)}\\%\;\\text{(current gains)}` }, { html: 'Book: −32.2 dB → 2.45% (p. 297), again from different gains.' },
            { code: `${PY_IN}\ndef noise_pct_in(w, kP, kD, sigma):\n    return 100 * abs(loop_in(w, kP, kD, sigma))` }] },
        { id: 'out', title: 'Outer loop: Bode plots of the plant and of the plant under PID control',
          html: 'Same, with bode(P_out) and bode(P_out·C_out). Here: Loop = outer.' },
        { id: 'c', title: '(c) Outer-loop tracking error below 0.001 rad/s for |r| ≤ 50',
          html: 'Bound on |e| for |r| ≤ 50 with content at frequency w (rad/s), as a function of w and the outer PID gains with σ.',
          code: {
            template: 'def e_bound_out(w, kP, kI, kD, sigma):\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { w: { label: 'w', lo: 5e-4, hi: 2e-3 }, ...OUT_ARGS },
              items: [{ fn: 'e_bound_out', args: ['w', 'kPz', 'kIz', 'kDz', 'sigma'], truth: (p, a) => ctx.sys.problems.ch16.rMax / T.mag(T.mul(Pout(p), Cout(a)), a.w) }],
            }, code),
          },
          solution: () => [{ tex: `B_r = ${tex(s().BrOut)}\\,\\text{dB},\; |e| \\le 50\\gamma_r = ${tex(50 * s().grOut)}\;\\text{(current gains)}` }, { html: 'Book: 154 dB → γ<sub>r</sub> = 2·10⁻⁸, |e| ≤ 1·10⁻⁶ (p. 297; it prints "100e−08").' },
            { code: 'def e_bound_out(w, kP, kI, kD, sigma):\n    s = 1j * w\n    Pout = (-2 * P.ell / 3 * s**2 + P.g) / s**2\n    Cout = kP + kI / s + kD * s / (sigma * s + 1)\n    return 50 / abs(Pout * Cout)' }] },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 17 --
  B.chapters.ch17 = {
    id: 'ch17', num: 17, tab: 'Ch 17', title: 'Stability margins', pages: 'pp. 303–322',
    linear: false,
    // The outer-loop view defaults to the book's P_out·C_out (hw17.py).
    defaults(sys) { return { ...pidDefaults(sys), loop: 'both', outerModel: 'book' }; },
    simDefaults(sys) { return sys.problems.ch17.sim; },
    controller: pidController,
    // Outer loop gain. 'book': P_out·C_out as hw17.py. 'impl': what the B.10 code
    // closes, C_out · filter · (inner closed loop θ_r → θ) · P_out.
    // g: fixed gains (the B.17 checks), else the sliders; model: 'book' | 'impl', else the toggle.
    outerLoop(ctx, g = gainsOf(ctx), model = ctx.st.outerModel) {
      const p = ctx.pModel;
      if (model === 'book') return T.mul(Pout(p), Cout(g));
      const P = Pin(p), Ci = Cin(g);
      // θ/θ_r with k_Pθ on the error and the D term on θ: P k_Pθ / (1 + P C_in)
      const Tin = T.tf(L.conv(L.conv(P.num, [g.kPth]), Ci.den), L.polyAdd(L.conv(P.den, Ci.den), L.conv(P.num, Ci.num)));
      const zf = lib().zcFilter(p, lib().dcGain(p, g.kPth), 0.01);
      return T.mul(Cout(g), T.tf([zf.a], [1, zf.b]), Tin, Pout(p));
    },
    loops(ctx, g = gainsOf(ctx), model = ctx.st.outerModel) {
      const p = ctx.pModel;
      const Li = T.mul(Pin(p), Cin(g)), Lo = this.outerLoop(ctx, g, model);
      const Ti = T.feedback(Li), To = T.feedback(Lo);
      return { Li, Lo, Ti, To, mi: marginsB(Li), mo: marginsB(Lo), bwi: bandwidth(Ti), bwo: bandwidth(To) };
    },
    buildControls(parent, ctx) {
      gainControls(parent, ctx);
      const sec = section(parent, 'Margins and bandwidths', 'p. 314–317');
      loopToggle(sec, ctx, [{ value: 'inner', label: 'inner' }, { value: 'outer', label: 'outer' }, { value: 'both', label: 'both closed loops' }]);
      segmented(sec, {
        label: 'Outer-loop model',
        options: [{ value: 'impl', label: 'as implemented: C_out·F·T_in·P_out' }, { value: 'book', label: 'P_out·C_out' }],
        ...bind(ctx, 'outerModel'),
      });
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const l = this.loops(ctx);
        const show = WB.ui.shown(ctx, 'B:ch17:m');
        const row = WB.ui.metric;
        box.replaceChildren(...(show ? [
          row('inner PM', pmText(l.mi.pms)), row('inner GM', gmText(l.mi.gms)), row('inner bandwidth', `${fmt(l.bwi, 3)} rad/s`),
          row('outer PM', pmText(l.mo.pms)), row('outer GM', gmText(l.mo.gms)), row('outer bandwidth', `${fmt(l.bwo, 3)} rad/s`),
          row('separation ω_bw,in / ω_bw,out', `${fmt(l.bwi / l.bwo, 3)}×`),
        ] : [WB.ui.revealButton(ctx, 'B:ch17:m', 'Reveal the margins')]));
      });
    },
    bode(ctx) {
      const l = this.loops(ctx), v = ctx.st.loop;
      const show = WB.ui.shown(ctx, 'B:ch17:m');
      if (v === 'both') {
        const marks = show ? [{ w: l.bwi, label: `inner bw ${fmt(l.bwi, 3)}`, color: '--series-1' }, { w: l.bwo, label: `outer bw ${fmt(l.bwo, 3)}`, color: '--series-2' }] : [];
        return { title: 'Closed loops: inner T_in and outer T_out', w: W, lines: [{ label: 'inner T_in(jω)', ...bodeB(l.Ti), color: '--series-1' }, { label: 'outer T_out(jω)', ...bodeB(l.To), color: '--series-2' }], marks };
      }
      const inner = v === 'inner', Lg = inner ? l.Li : l.Lo, Tc = inner ? l.Ti : l.To, mg = inner ? l.mi : l.mo, bw = inner ? l.bwi : l.bwo;
      const marks = [];
      if (show) {
        for (const c of mg.pms) marks.push({ w: c.w, label: `ω_co ${fmt(c.w, 3)}, PM ${fmt(c.pm, 3)}°`, color: '--series-1' });
        if (isFinite(bw)) marks.push({ w: bw, label: `bandwidth ${fmt(bw, 3)}`, color: '--series-2' });
      }
      return { title: inner ? 'Inner loop: open loop P_in C_in and closed loop' : 'Outer loop: open loop P_out C_out and closed loop', w: W,
        lines: [{ label: 'open loop', ...bodeB(Lg), color: '--series-1' }, { label: 'closed loop', ...bodeB(Tc), color: '--series-2', width: 1.5 }], marks };
    },
    splane(ctx) { const l = this.loops(ctx); return ctx.st.loop === 'outer' ? clMarkers(ctx, l.Lo, 'outer') : clMarkers(ctx, l.Li, 'inner'); },
    math(ctx) {
      const l = this.loops(ctx);
      return [
        { title: 'Phase and gain margins', page: 'p. 303–306',
          theory: 'PM = 180^\\circ + \\angle PC(j\\omega_{co}),\\; |PC(j\\omega_{co})| = 1;\\quad GM = \\frac{1}{|PC(j\\omega_{180})|}',
          numbers: `\\text{inner: } PM = ${tex(l.mi.pm)}^\\circ \\text{ at } ${tex(l.mi.wc)};\\quad \\text{outer: } PM = ${tex(l.mo.pm)}^\\circ \\text{ at } ${tex(l.mo.wc)}`, spoiler: true, answers: ['B.17/a1', 'B.17/b1'],
          note: 'P_in C_in starts at −180° (negative DC gain), so the inner "gain margin" is a gain decrease: below 1/|L(0)| the loop cannot hold the pendulum up. The book prints GM −6.62 and PM 32.9° (Fig. 17-12), from other gains than B.10\'s.' },
        { title: 'Bandwidth vs. crossover', page: 'p. 316–317',
          theory: '\\omega_{bw}: \\text{first } \\omega \\text{ where } |T(j\\omega)| \\text{ is 3 dB below } |T(0)|,\\quad T = \\frac{PC}{1 + PC}',
          numbers: `\\omega_{bw,in} = ${tex(l.bwi)},\\; \\omega_{co,in} = ${tex(l.mi.wc)};\\quad \\omega_{bw,out} = ${tex(l.bwo)},\\; \\omega_{co,out} = ${tex(l.mo.wc)}`, spoiler: true, answers: ['B.17/a2', 'B.17/b2'],
          note: 'T_in(0) = k_DC ≠ 1, so the −3 dB point is measured from |T(0)|, not from 1 (as control.bandwidth does).' },
        { title: 'Successive loop closure', page: 'p. 317',
          theory: '\\text{10× faster inner loop} \\leftrightarrow \\text{inner bandwidth a decade above the outer}',
          numbers: `\\omega_{bw,in}/\\omega_{bw,out} = ${tex(l.bwi / l.bwo)}`, spoiler: true, answers: ['B.17/c1', 'B.17/c2'] },
      ];
    },
    buildProblem(parent, ctx) {
      // Checks use a fixed loop, the book's B.10 gains (Listing 10.3, nominal
      // parameters), never the sliders. The inner loop is P_in·C_in. For the outer
      // loop the book's P_out·C_out never crosses 0 dB with these gains (|L| → 4.5),
      // so it has no PM, GM or bandwidth: the outer answers use the loop the B.10
      // code closes, C_out·F·T_in·P_out ("as implemented").
      const g = b10(ctx.sys);
      const fixed = () => this.loops(ctx, g, 'impl');
      const gtxt = `k<sub>Pθ</sub> = ${fmt(g.kPth, 4)}, k<sub>Dθ</sub> = ${fmt(g.kDth, 4)}, k<sub>Pz</sub> = ${fmt(g.kPz, 4)}, k<sub>Iz</sub> = ${fmt(g.kIz, 3)}, k<sub>Dz</sub> = ${fmt(g.kDz, 4)}, σ = ${fmt(g.sigma, 3)}`;
      const with10 = `With the book's B.10 gains (${gtxt}), not the sliders.`;
      const outerNote = 'With these gains the book\'s P<sub>out</sub>·C<sub>out</sub> never drops below 0 dB (it tends to (k<sub>Dz</sub> + σk<sub>Pz</sub>)/σ · 2ℓ/3 ≈ 4.5), so it has no crossover; use the loop the B.10 code closes, C<sub>out</sub>·F·T<sub>in</sub>·P<sub>out</sub> (Outer-loop model: as implemented).';
      const gmIn = 'Enter GM in dB (any phase crossing; <code>inf</code> if the phase never reaches −180°). The inner loop starts at −180°, so its crossing is at ω → 0.';
      const SEP = 5;   // the book's bandwidth separation for successive loop closure: M ≈ 5–10 (p. 117–118)
      const ratio = () => { const x = fixed(); return x.bwi / x.bwo; };
      const gmTex = (gms) => (gms.length ? gms.map((c) => `${tex(db(c.gm))}\\,\\text{dB at } ${c.w === 0 ? '\\omega \\to 0' : tex(c.w)}`).join(';\\;') : '\\infty');
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch17, [
        { id: 'a1', title: '(a) Inner loop under PD control: phase and gain margins',
          html: `${with10} ${gmIn}`,
          inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub> [rad/s]', gm: 'GM [dB]' },
          check: (v) => { const x = fixed(); return WB.freq.marginCheck(v, { pm: x.mi.pm, wc: x.mi.wc }, { pm: 'PM', wc: 'ωco' }, { crossings: x.mi.gms }); },
          solution: () => { const x = fixed(); return [{ tex: `PM = ${tex(x.mi.pm)}^\\circ \\text{ at } \\omega_{co} = ${tex(x.mi.wc)}\\,\\text{rad/s},\\quad GM = ${gmTex(x.mi.gms)}` }, { html: 'P<sub>in</sub>C<sub>in</sub> starts at −180° (negative DC gain), so the gain margin is a gain <em>decrease</em>: below 1/|L(0)| the loop can no longer hold the rod up. Book: GM −6.62, PM 32.88° (Fig. 17-12), from other gains than B.10\'s (ISSUES.md).' }]; } },
        { id: 'a2', title: '(a) Inner-loop bandwidth, and how it relates to crossover',
          html: `${with10} Plot the open- and closed-loop Bode plots together; ω<sub>bw</sub> is where |T<sub>in</sub>| falls 3 dB below its DC value k<sub>DC</sub> (as control.bandwidth defines it).`,
          inputs: { bw: 'ω<sub>bw</sub> [rad/s]', ratio: 'ω<sub>bw</sub> / ω<sub>co</sub>' },
          check: (v) => { const x = fixed(); return PD().checkNumbers(v, { bw: x.bwi, ratio: x.bwi / x.mi.wc }, { bw: 'ωbw', ratio: 'ωbw/ωco' }); },
          solution: () => { const x = fixed(); return [{ tex: `\\omega_{bw} = ${tex(x.bwi)}\\,\\text{rad/s} = ${tex(x.bwi / x.mi.wc)}\\,\\omega_{co}` }, { html: 'The bandwidth sits a little above crossover. Above ω<sub>co</sub>, |PC| ≪ 1 and |T| ≈ |PC| falls with it; with a PM of only about 30° the closed loop peaks near ω<sub>co</sub>, which pushes the −3 dB point above it. Book: about 19 rad/s against a crossover of 10 rad/s (p. 317).' }]; } },
        { id: 'b1', title: '(b) Outer loop under PID control: phase and gain margins',
          html: `${with10} ${outerNote} Enter GM in dB (any phase crossing, or <code>inf</code>).`,
          inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub> [rad/s]', gm: 'GM [dB]' },
          check: (v) => { const x = fixed(); return WB.freq.marginCheck(v, { pm: x.mo.pm, wc: x.mo.wc }, { pm: 'PM', wc: 'ωco' }, { crossings: x.mo.gms }); },
          solution: () => { const x = fixed(); return [{ tex: `PM = ${tex(x.mo.pm)}^\\circ \\text{ at } \\omega_{co} = ${tex(x.mo.wc)}\\,\\text{rad/s},\\quad GM = ${gmTex(x.mo.gms)}` }, { html: 'Two phase crossings: lowering the gain below the first one, or raising it above the second, destabilizes the loop. Book: PM 72° near 1 rad/s (the plot title says −108.52; 180 − 108 = 72), from hw17\'s P<sub>out</sub>·C<sub>out</sub>, which has no crossover with the B.10 gains (ISSUES.md).' }]; } },
        { id: 'b2', title: '(b) Outer-loop bandwidth, and how it relates to crossover',
          html: `${with10} Same loop as above; ω<sub>bw</sub> is where |T<sub>out</sub>| falls 3 dB below its DC value.`,
          inputs: { bw: 'ω<sub>bw</sub> [rad/s]', ratio: 'ω<sub>bw</sub> / ω<sub>co</sub>' },
          check: (v) => { const x = fixed(); return PD().checkNumbers(v, { bw: x.bwo, ratio: x.bwo / x.mo.wc }, { bw: 'ωbw', ratio: 'ωbw/ωco' }); },
          solution: () => { const x = fixed(); return [{ tex: `\\omega_{bw} = ${tex(x.bwo)}\\,\\text{rad/s} = ${tex(x.bwo / x.mo.wc)}\\,\\omega_{co}` }, { html: 'Again above crossover, here by about 3×: with PM ≈ 30° the outer closed loop peaks strongly, so |T| stays within 3 dB well past ω<sub>co</sub>. Book: about 1.3 rad/s against a crossover of 1.0 rad/s (p. 317).' }]; } },
        { id: 'c1', title: '(c) Bandwidth separation between the inner and outer loops',
          html: `${with10} The ratio of the two closed-loop bandwidths from (a) and (b).`,
          inputs: { r: 'ω<sub>bw,in</sub> / ω<sub>bw,out</sub>' },
          check: (v) => PD().checkNumbers(v, { r: ratio() }, { r: 'ratio' }),
          solution: () => { const x = fixed(); return [{ tex: `\\frac{\\omega_{bw,in}}{\\omega_{bw,out}} = \\frac{${tex(x.bwi)}}{${tex(x.bwo)}} = ${tex(x.bwi / x.bwo)}` }]; } },
        { id: 'c2', title: '(c) For this design, is successive loop closure justified?',
          html: 'Answer <code>yes</code> or <code>no</code>.',
          inputs: { yn: 'yes / no' },
          check: (v) => {
            const t = String(v.yn || '').trim().toLowerCase();
            if (!/^(yes|no|y|n)$/.test(t)) return { ok: false, msg: 'Answer yes or no.' };
            const want = ratio() >= SEP;
            return (t[0] === 'y') === want ? { ok: true, msg: '' } : { ok: false, msg: 'Compare your (c) ratio with the separation successive loop closure needs (Ch 8).' };
          },
          solution: () => { const r = ratio(); return [{ html: `${r >= SEP ? 'Yes' : 'No'}. The inner loop's bandwidth is ${fmt(r, 3)}× the outer loop's. Successive loop closure treats the inner loop as its DC gain, which needs the inner loop about 5–10 times faster than the outer (M = t<sub>r,out</sub>/t<sub>r,in</sub> ≈ 5–10, p. 117–118); in the frequency domain that is an inner bandwidth 5–10 times higher, about a decade (p. 317). ${fmt(r, 3)}× is ${r >= 10 ? 'above that range' : r >= SEP ? 'inside that range, toward its low end' : 'below it'}. The book reads about 19/1.3 ≈ 15 from its own figures and calls it "about one decade."` }]; } },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 18 --
  // Presets: the repo's loopShapingInner/Outer.py, and the designs in the text.
  function presetRepo() {
    return { kin: -800, inLead: { on: true, w: 40, M: 15 }, kout: 0.1, outLead: { on: true, w: 1, M: 20 }, lag: { on: true, z: 0.04, M: 10 }, lpf: { on: true, p: 50 }, pf: { on: true, p: 2 } };
  }
  function presetBook() {
    // C_in = −155.12 (s/10.72 + 1)/(s/120 + 1);  C_out = 0.469 (s+0.194)/(s+6.24) (s+0.0256)/(s+0.0032) 100/(s+100)
    const Mi = 120 / 10.72, Mo = 6.24 / 0.194;
    return {
      kin: -155.12 * (120 / 10.72) / Math.sqrt(Mi), inLead: { on: true, w: Math.sqrt(10.72 * 120), M: Mi },
      kout: 0.469 / Math.sqrt(Mo), outLead: { on: true, w: Math.sqrt(0.194 * 6.24), M: Mo }, lag: { on: true, z: 0.0256, M: 8 }, lpf: { on: true, p: 100 }, pf: { on: true, p: 2 },
    };
  }
  function presetNone() {
    // a stable first try that meets neither part: proportional + one lead per loop
    return { kin: -200, inLead: { on: true, w: 15, M: 8 }, kout: 0.03, outLead: { on: true, w: 0.5, M: 15 }, lag: { on: false, z: 0.05, M: 10 }, lpf: { on: false, p: 100 }, pf: { on: false, p: 2 } };
  }

  // Solution for the B.18 implementation: the repo's transferFunction (controllable
  // canonical form, RK4), outer then inner as ctrlLoopshape.py, but with RK4
  // substeps: one step per T_s is too coarse for C_in's pole at 155 rad/s, and with
  // d = 0.5 N that version drops the rod (ISSUES.md).
  const LS_SOL = `class TransferFunction:
    def __init__(self, num, den):
        num = np.array(num, dtype=float) / den[0]
        den = np.array(den, dtype=float) / den[0]
        n = len(den) - 1
        num = np.concatenate([np.zeros(n + 1 - len(num)), num])   # pad to n + 1
        self.A = np.zeros((n, n))
        self.A[0, :] = -den[1:]
        self.A[1:, :-1] = np.eye(n - 1)
        self.B = np.zeros((n, 1))
        self.B[0, 0] = 1.0
        self.C = (num[1:] - num[0] * den[1:]).reshape(1, n)
        self.D = num[0]
        self.z = np.zeros((n, 1))

    def update(self, u):
        # output for this input, then advance the state over Ts (RK4, 10 substeps)
        y = (self.C @ self.z)[0, 0] + self.D * u
        f = lambda z: self.A @ z + self.B * u
        h = P.Ts / 10
        for _ in range(10):
            F1 = f(self.z); F2 = f(self.z + h / 2 * F1)
            F3 = f(self.z + h / 2 * F2); F4 = f(self.z + h * F3)
            self.z = self.z + h / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return y


class Controller:
    def __init__(self):
        self.C_in = TransferFunction(P.C_in_num, P.C_in_den)
        self.C_out = TransferFunction(P.C_out_num, P.C_out_den)

    def update(self, z_r, y):
        z = y[0, 0]
        theta = y[1, 0]
        theta_r = self.C_out.update(z_r - z)
        F = self.C_in.update(theta_r - theta)
        return max(-P.F_max, min(P.F_max, F))
`;
  // The implementation check's step [m]: small, though C_in's large high-frequency
  // gain still saturates F for a few samples.
  const LS_STEP = 0.1;

  B.chapters.ch18 = {
    id: 'ch18', num: 18, tab: 'Ch 18', title: 'Loopshaping', pages: 'pp. 323–374',
    // mineIn, mineOut: the student's C_in, C_out from (a) and (b) (Work mode).
    defaults() { return { ...presetNone(), loop: 'inner', method: 'state_space', showT: true, mineIn: null, mineOut: null }; },
    simDefaults(sys) { return { ...sys.problems.ch18.sim, mismatch: sys.problems.ch18.mismatch }; },
    linearLabel: 'linear model (no saturation, d, noise)',
    // Work mode simulates the student's controller, which gets the designed
    // compensators as P.C_in_num, P.C_in_den, P.C_out_num, P.C_out_den.
    implement: {
      feed: 'y', linear: false,
      params(ctx) {
        const d = B.chapters.ch18.design(ctx);
        return { C_in_num: d.Ci.num.slice(), C_in_den: d.Ci.den.slice(), C_out_num: d.Co.num.slice(), C_out_den: d.Co.den.slice() };
      },
    },

    // Work mode: C_in, C_out are the student's own ((a), (b), Python), or 1 until
    // they give them, and the prefilter is in their code; Explore: the block knobs.
    design(ctx) {
      const st = ctx.st, p = ctx.pModel, bl = lib().blocks, pr = ctx.sys.problems.ch18;
      const work = ctx.S && ctx.S.mode === 'work';
      let Ci, Co, F;
      if (work) {
        Ci = st.mineIn ? T.tf(st.mineIn.num, st.mineIn.den) : T.gain(1);
        Co = st.mineOut ? T.tf(st.mineOut.num, st.mineOut.den) : T.gain(1);
        F = T.gain(1);
      } else {
        Ci = bl.prop(st.kin);
        if (st.inLead.on) Ci = T.mul(Ci, bl.lead(st.inLead.w, st.inLead.M));
        Co = bl.prop(st.kout);
        if (st.outLead.on) Co = T.mul(Co, bl.lead(st.outLead.w, st.outLead.M));
        if (st.lag.on) Co = T.mul(Co, bl.lag(st.lag.z, st.lag.M));
        if (st.lpf.on) Co = T.mul(Co, bl.lpf(st.lpf.p));
        F = st.pf.on ? bl.lpf(st.pf.p) : T.gain(1);
      }
      const Li = T.mul(Pin(p), Ci), Ti = T.feedback(Li);
      const Pouter = T.mul(Pout(p), Ti), Lo = T.mul(Pouter, Co), To = T.feedback(Lo);
      const mi = marginsB(Li), mo = marginsB(Lo), bwi = bandwidth(Ti);
      const maxAbove = (G, w0) => { let m = 0; for (const w of W) if (w >= w0) m = Math.max(m, T.mag(G, w)); return m; };
      const minBelow = (G, w1) => { let m = Infinity; for (const w of W) if (w <= w1) m = Math.min(m, T.mag(G, w)); return m; };
      const inNoise = maxAbove(Li, pr.inner.wno), outNoise = maxAbove(Lo, pr.outer.wno), outTrack = minBelow(Lo, pr.outer.wr);
      const stableIn = L.roots(L.polyAdd(Li.den, Li.num)).every((r) => r.re < 0);
      const stableOut = L.roots(L.polyAdd(Lo.den, Lo.num)).every((r) => r.re < 0);
      return {
        Ci, Co, F, Li, Ti, Pouter, Lo, To, mi, mo, bwi, inNoise, outNoise, outTrack, stableIn, stableOut,
        ok: {
          inPM: stableIn && Math.abs(mi.pm - pr.inner.pm) <= 5, inNoise: inNoise <= pr.inner.gn * 1.001, inBw: isFinite(bwi) && Math.abs(bwi / pr.inner.wbw - 1) <= 0.3,
          outPM: stableOut && Math.abs(mo.pm - pr.outer.pm) <= 10, outTrack: outTrack >= 1 / pr.outer.gr * 0.999, outNoise: outNoise <= pr.outer.gn * 1.001,
        },
      };
    },

    controller(ctx, { linear = false } = {}) {
      const d = this.design(ctx);
      return lib().loopshapeCtrl({ Cin: d.Ci, Cout: d.Co, F: d.F, Ts: ctx.S.sim.Ts, uLim: ctx.sys.uLimit(ctx.pModel), method: ctx.st.method, linear });
    },
    linearSim(ctx, c) {
      const { A, B: Bm, C } = ctx.sys.stateSpace(ctx.pModel);
      return WB.sim.simulate({ ...c, disturbance: null, noise: null, plant: WB.design.linearPlant(A, Bm, C), controller: this.controller(ctx, { linear: true }) });
    },

    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') {
        // C_in and C_out come from the student's Python in (a) and (b): no block menus.
        WB.myCtrl.banner(section(parent, 'Your controller'), ctx, 'the B.18 implementation part');
        this.viewSection(parent, ctx);
        this.specSection(parent, ctx);
        return;
      }
      const pre = section(parent, 'Start from', 'p. 349–360');
      const row = el('div', { class: 'btn-row' },
        el('button', { type: 'button', class: 'btn', text: 'First try', onclick: () => { Object.assign(ctx.st, presetNone()); ctx.update(); } }));
      pre.append(row);
      // the repo and book designs answer B.18(a) and (b): Work mode offers them once both are solved
      if (shows(ctx, 'B.18/a') && shows(ctx, 'B.18/b')) {
        row.append(
          el('button', { type: 'button', class: 'btn', text: 'Repo (Listings 18.5–18.6)', title: 'C_in = −800·lead(40, 15); C_out = 0.1·lead(1, 20)·lag(0.04, 10)·lpf(50); F = lpf(2)', onclick: () => { Object.assign(ctx.st, presetRepo()); ctx.update(); } }),
          el('button', { type: 'button', class: 'btn', text: 'Book text', title: 'C_in = −155.12(s/10.72+1)/(s/120+1); C_out = 0.469 lead · lag · 100/(s+100); F = 2/(s+2)', onclick: () => { Object.assign(ctx.st, presetBook()); ctx.update(); } }));
      } else {
        pre.append(el('p', { class: 'muted small', text: 'The repo\'s and the book\'s designs appear here once (a) and (b) are solved.' }));
      }
      this.viewSection(parent, ctx);

      const onOff = (sec, key) => WB.ui.onOff(sec, ctx, () => ctx.st[key]);
      const inn = section(parent, 'Inner C_in = k_in · lead', 'p. 349–352');
      slider(inn, { label: 'k<sub>in</sub>', min: -3000, max: -1, step: 1, sig: 4, hint: 'negative: P_in has negative gain', ...bind(ctx, 'kin') });
      onOff(inn, 'inLead');
      slider(inn, { label: 'ω<sub>lead</sub>', unit: 'rad/s', min: 1, max: 500, log: true, sig: 3, ...bind(ctx, 'w', () => ctx.st.inLead), disabled: () => !ctx.st.inLead.on });
      slider(inn, { label: 'M', min: 1.01, max: 100, log: true, sig: 3, ...bind(ctx, 'M', () => ctx.st.inLead), disabled: () => !ctx.st.inLead.on });

      const out = section(parent, 'Outer C_out = k_out · lead · lag · LPF', 'p. 354–356');
      slider(out, { label: 'k<sub>out</sub>', min: 0.001, max: 10, log: true, sig: 4, ...bind(ctx, 'kout') });
      out.append(el('p', { class: 'muted small', text: 'Lead' })); onOff(out, 'outLead');
      slider(out, { label: 'ω<sub>lead</sub>', unit: 'rad/s', min: 0.05, max: 50, log: true, sig: 3, ...bind(ctx, 'w', () => ctx.st.outLead), disabled: () => !ctx.st.outLead.on });
      slider(out, { label: 'M', min: 1.01, max: 100, log: true, sig: 3, ...bind(ctx, 'M', () => ctx.st.outLead), disabled: () => !ctx.st.outLead.on });
      out.append(el('p', { class: 'muted small', text: 'Lag (s + z)/(s + z/M)' })); onOff(out, 'lag');
      slider(out, { label: 'z', unit: 'rad/s', min: 0.001, max: 5, log: true, sig: 3, ...bind(ctx, 'z', () => ctx.st.lag), disabled: () => !ctx.st.lag.on });
      slider(out, { label: 'M', min: 1.01, max: 100, log: true, sig: 3, ...bind(ctx, 'M', () => ctx.st.lag), disabled: () => !ctx.st.lag.on });
      out.append(el('p', { class: 'muted small', text: 'Low-pass p/(s + p)' })); onOff(out, 'lpf');
      slider(out, { label: 'p', unit: 'rad/s', min: 1, max: 2000, log: true, sig: 3, ...bind(ctx, 'p', () => ctx.st.lpf), disabled: () => !ctx.st.lpf.on });

      const pf = section(parent, 'Prefilter F(s) = p/(s + p)', 'p. 355');
      onOff(pf, 'pf');
      slider(pf, { label: 'p', unit: 'rad/s', min: 0.1, max: 50, log: true, sig: 3, ...bind(ctx, 'p', () => ctx.st.pf), disabled: () => !ctx.st.pf.on });
      segmented(pf, {
        label: 'Implementation',
        options: [{ value: 'state_space', label: 'state space, RK4' }, { value: 'digital_filter', label: 'Tustin filter (hw18 default)' }],
        ...bind(ctx, 'method'),
      });
      this.specSection(parent, ctx);
    },

    viewSection(parent, ctx) {
      const view = section(parent, 'Bode view');
      segmented(view, { options: [{ value: 'inner', label: 'inner: P_in C_in' }, { value: 'outer', label: 'outer: P C_out' }], ...bind(ctx, 'loop') });
      segmented(view, { label: 'Closed loop on the Bode plot', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'showT') });
    },

    specSection(parent, ctx) {
      const sp = section(parent, 'B.18 specs', 'p. 349');
      const box = el('div', { class: 'metrics' });
      sp.append(box);
      WB.ui.addRefresher(() => {
        const d = this.design(ctx), ok = d.ok;
        const row = WB.ui.specRow;
        box.replaceChildren(
          row('inner PM ≈ 60° (±5°), stable', ok.inPM, `${fmt(d.mi.pm, 3)}° at ${fmt(d.mi.wc, 3)} rad/s`),
          row('inner |PC| ≤ 0.1 above 200 rad/s', ok.inNoise, `${fmt(db(d.inNoise), 3)} dB`),
          row('inner bandwidth ≈ 40 rad/s (±30%)', ok.inBw, `${fmt(d.bwi, 3)} rad/s`),
          row('outer PM ≈ 60° (±10°), stable', ok.outPM, `${fmt(d.mo.pm, 3)}° at ${fmt(d.mo.wc, 3)} rad/s`),
          row('outer |PC| ≥ 10⁵ below 0.0032 rad/s', ok.outTrack, `${fmt(db(d.outTrack), 3)} dB`),
          row('outer |PC| ≤ 10⁻⁴ above 1000 rad/s', ok.outNoise, `${fmt(db(d.outNoise), 3)} dB`),
        );
      });
    },

    // Evaluate the student's C_<which>_num, C_<which>_den and keep them in the chapter state.
    async loadMine(ctx, code, which) {
      const names = [`C_${which}_num`, `C_${which}_den`];
      const out = await WB.py.evaluate(code, [{ params: ctx.pModel, vars: names }]);
      if (out.error) return WB.yours.pyError(out);
      const v = out.rows[0].vars;
      const num = L.trimLeading([v[names[0]]].flat(Infinity)), den = L.trimLeading([v[names[1]]].flat(Infinity));
      if (![...num, ...den].every((x) => typeof x === 'number' && Number.isFinite(x))) return { ok: false, msg: `${names[0]} and ${names[1]} must be lists of real numbers.` };
      if (!num.length || num.every((x) => x === 0)) return { ok: false, msg: `${names[0]} must not be zero.` };
      if (!den.length || den[0] === 0) return { ok: false, msg: `${names[1]} must have a nonzero leading coefficient.` };
      if (num.length > den.length) return { ok: false, msg: `C_${which}(s) must be proper: the numerator degree can be at most the denominator degree.` };
      ctx.st[which === 'in' ? 'mineIn' : 'mineOut'] = { num, den };
      ctx.update();
      return { ok: true };
    },

    bode(ctx) {
      const d = this.design(ctx), pr = ctx.sys.problems.ch18, p = ctx.pModel;
      if (ctx.st.loop === 'inner') {
        const lines = [...(shows(ctx, 'B.15/a') ? [{ label: 'P_in', ...bodeB(Pin(p)), color: '--text-muted', dash: [5, 4], width: 1.5 }] : []), { label: 'P_in C_in', ...bodeB(d.Li), color: '--series-1' }];
        if (ctx.st.showT) lines.push({ label: 'closed loop T_in', mag: T.bode(d.Ti, W).mag, color: '--series-3', width: 1.5 });
        return { title: 'Inner loop', w: W, lines, specs: [{ w0: pr.inner.wno, w1: 1e5, db: db(pr.inner.gn), keep: 'below', color: '--critical', label: 'noise spec' }],
          marks: d.mi.pms.map((c) => ({ w: c.w, label: `PM ${fmt(c.pm, 3)}° at ${fmt(c.w, 3)}`, color: '--series-1' })) };
      }
      const lines = [{ label: 'P = P_out T_in', ...bodeB(d.Pouter), color: '--text-muted', dash: [5, 4], width: 1.5 }, { label: 'P C_out', ...bodeB(d.Lo), color: '--series-1' }];
      if (ctx.st.showT) lines.push({ label: 'closed loop F·T_out', mag: T.bode(T.mul(d.F, d.To), W).mag, color: '--series-3', width: 1.5 });
      return { title: 'Outer loop', w: W, lines,
        specs: [{ w0: 1e-4, w1: pr.outer.wr, db: db(1 / pr.outer.gr), keep: 'above', color: '--critical', label: 'tracking spec' }, { w0: pr.outer.wno, w1: 1e5, db: db(pr.outer.gn), keep: 'below', color: '--critical', label: 'noise spec' }],
        marks: d.mo.pms.map((c) => ({ w: c.w, label: `PM ${fmt(c.pm, 3)}° at ${fmt(c.w, 3)}`, color: '--series-1' })) };
    },
    splane(ctx) { const d = this.design(ctx); return ctx.st.loop === 'inner' ? clMarkers(ctx, d.Li, 'inner') : clMarkers(ctx, d.Lo, 'outer'); },

    math(ctx) {
      const d = this.design(ctx);
      return [
        { title: 'Building blocks', page: 'p. 324–328',
          theory: '\\text{lead}(\\omega, M) = \\frac{\\sqrt M s + \\omega}{s + \\omega\\sqrt M},\\quad \\text{lag}(z, M) = \\frac{s + z}{s + z/M},\\quad \\text{lpf}(p) = \\frac{p}{s + p}',
          note: 'The repo\'s lead has DC gain 1/√M and high-frequency gain √M; the book\'s Eq. 18.2 lead has DC gain 1, so it is √M times larger.' },
        { title: 'Your controllers', page: 'p. 349, p. 355',
          theory: ctx.S.mode === 'work' && !ctx.st.mineIn && !ctx.st.mineOut ? '\\text{none yet: (a) and (b)}' : `C_{in}(s) = ${ctx.S.mode === 'work' && !ctx.st.mineIn ? '1' : T.texTf(d.Ci, 4)},\\quad C_{out}(s) = ${ctx.S.mode === 'work' && !ctx.st.mineOut ? '1' : T.texTf(d.Co, 4)}`, numbers: null },
        { title: 'Outer-loop plant', page: 'p. 349 · B.18(b)',
          theory: 'P = P_{out}\\frac{P_{in}C_{in}}{1 + P_{in}C_{in}}',
          note: 'The inner closed loop has DC gain P_in C_in(0)/(1 + P_in C_in(0)) ≠ 1, so P_out sees it as an extra gain.' },
        { title: 'Margins', page: 'p. 351, p. 356',
          theory: `\\text{inner } PM = ${tex(d.mi.pm)}^\\circ \\text{ at } ${tex(d.mi.wc)},\\quad \\text{outer } PM = ${tex(d.mo.pm)}^\\circ \\text{ at } ${tex(d.mo.wc)}` },
        { title: 'Implementation', page: 'p. 336 · Eq. 18.3–18.7', answers: 'B.18/c',
          theory: '\\tilde z_r = F(z_r),\\quad \\theta_r = C_{out}(\\tilde z_r - z),\\quad F = \\text{sat}\\big(C_{in}(\\theta_r - \\theta)\\big)',
          note: ctx.st.method === 'digital_filter' ? 'Tustin: s → (2/T_s)(z − 1)/(z + 1), then a difference equation (digitalFilter).' : 'Control-canonical state space, one RK4 step per sample (transferFunction).' },
      ];
    },

    buildProblem(parent, ctx) {
      const work = () => ctx.S.mode === 'work';
      const inMsg = (d) => `PM ${fmt(d.mi.pm, 3)}° ${d.ok.inPM ? '✓' : '✗'}, noise ${fmt(db(d.inNoise), 3)} dB ${d.ok.inNoise ? '✓' : '✗'}, bandwidth ${fmt(d.bwi, 3)} rad/s ${d.ok.inBw ? '✓' : '✗'}`;
      const outMsg = (d) => `PM ${fmt(d.mo.pm, 3)}° ${d.ok.outPM ? '✓' : '✗'}, tracking ${fmt(db(d.outTrack), 3)} dB ${d.ok.outTrack ? '✓' : '✗'}, noise ${fmt(db(d.outNoise), 3)} dB ${d.ok.outNoise ? '✓' : '✗'}`;
      const coeffs = 'Give it as coefficient lists, highest power of s first (<code>np.convolve</code> multiplies two factors).';
      const needBoth = () => (work() && (!ctx.st.mineIn || !ctx.st.mineOut) ? { ok: false, msg: 'Load your C_in in (a) and your C_out in (b) first (Use my C_in, Use my C_out).' } : null);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch18, [
        { id: 'a', title: '(a) Inner loop: design C<sub>in</sub>(s) to meet the specs',
          html: `${coeffs} <em>Use my C_in</em> draws P<sub>in</sub>C<sub>in</sub> in the Bode plot (Bode view: inner), the s-plane and the spec readouts. The check also requires a stable inner loop.`,
          code: {
            template: 'C_in_num = [1.0]\nC_in_den = [1.0]\n',
            check: async (code) => {
              const r = await this.loadMine(ctx, code, 'in');
              if (r.ok === false) return r;
              const d = this.design(ctx);
              return { ok: d.ok.inPM && d.ok.inNoise && d.ok.inBw, msg: inMsg(d) };
            },
            actions: [{ label: 'Use my C_in', run: async (code) => { const r = await this.loadMine(ctx, code, 'in'); return r.ok === false ? r : { info: true, msg: 'Your C_in is in the Bode plot, the s-plane and the spec readouts.' }; } }],
          },
          solution: () => [
            { code: '# the repo design (Listing 18.5): a negative proportional gain\n# and the repo\'s lead (sqrt(M) s + w) / (s + w sqrt(M)) at w = 40\nk = -800\nw, M = 40, 15\nC_in_num = k * np.array([np.sqrt(M), w])\nC_in_den = [1, w * np.sqrt(M)]\n' },
            { html: 'Repo (Listing 18.5): k = −1 × 800 to cross at 40 rad/s, then lead(ω = 40, M = 15). The text instead prints C<sub>lead</sub> = −155.12 (s/10.72 + 1)/(s/120 + 1) and calls it a ratio-13.9 lead at 40 rad/s; 10.72·13.9 is 149, not 120 (p. 349–350). Explore mode has both as presets.' },
          ] },
        { id: 'b', title: '(b) Outer loop: design C<sub>out</sub>(s) to meet the specs',
          html: `${coeffs} The outer plant P = P<sub>out</sub>·P<sub>in</sub>C<sub>in</sub>/(1 + P<sub>in</sub>C<sub>in</sub>) uses your C<sub>in</sub> from (a). <em>Use my C_out</em> draws P·C<sub>out</sub> (Bode view: outer) and the spec readouts. The check also requires a stable outer loop.`,
          code: {
            template: 'C_out_num = [1.0]\nC_out_den = [1.0]\n',
            check: async (code) => {
              if (work() && !ctx.st.mineIn) return { ok: false, msg: 'Load your C_in in (a) first (Use my C_in): the outer plant contains the inner closed loop.' };
              const r = await this.loadMine(ctx, code, 'out');
              if (r.ok === false) return r;
              const d = this.design(ctx);
              return { ok: d.ok.outPM && d.ok.outTrack && d.ok.outNoise, msg: outMsg(d) };
            },
            actions: [{ label: 'Use my C_out', run: async (code) => { const r = await this.loadMine(ctx, code, 'out'); return r.ok === false ? r : { info: true, msg: 'Your C_out is in the Bode plot (outer), the s-plane and the spec readouts.' }; } }],
          },
          solution: () => [
            { code: '# the repo design (Listing 18.6): gain, lead at 1 rad/s, lag, low-pass\nk = 0.1\nlead_num, lead_den = [np.sqrt(20), 1], [1, np.sqrt(20)]   # w = 1, M = 20\nlag_num, lag_den = [1, 0.04], [1, 0.04 / 10]             # z = 0.04, M = 10\nlpf_num, lpf_den = [50], [1, 50]\nC_out_num = k * np.convolve(np.convolve(lead_num, lag_num), lpf_num)\nC_out_den = np.convolve(np.convolve(lead_den, lag_den), lpf_den)\n' },
            { html: 'The text (p. 354–355): lead at 1.1 rad/s with ratio 32, gain 0.0146 for crossover at 1.1 rad/s, lag ratio 8, LPF at 100 rad/s, giving C<sub>out</sub> = 0.469 (s + 0.194)/(s + 6.24) · (s + 0.0256)/(s + 0.0032) · 100/(s + 100), and F = 2/(s + 2). Listing 18.6 and the repo use 0.1·lead(1, 20)·lag(0.04, 10)·lpf(50). Explore mode has both as presets.' },
          ] },
        WB.myCtrl.part(ctx, {
          id: 'c', title: 'Implement C<sub>in</sub>(s) and C<sub>out</sub>(s) in simulation using their state-space form',
          html: `Your designs are in <code>P.C_in_num</code>, <code>P.C_in_den</code>, <code>P.C_out_num</code>, <code>P.C_out_den</code> (coefficient lists, highest power of s first). Realize each as ż<sub>C</sub> = A<sub>C</sub>z<sub>C</sub> + B<sub>C</sub>e, u = C<sub>C</sub>z<sub>C</sub> + D<sub>C</sub>e (Eq. 18.3–18.4): the outer one turns z<sub>r</sub> − z into θ<sub>r</sub>, the inner one θ<sub>r</sub> − θ into F. The check steps z<sub>r</sub> by ${LS_STEP} m with exact parameters and compares z(t) with the workbench running the same C<sub>in</sub>, C<sub>out</sub> (within 3%). If the rod falls in the time plots once d = 0.5 N, look at how accurately you integrate C<sub>in</sub>'s fastest pole (|p|·T<sub>s</sub>).`,
          check: async (code) => {
            const nb = needBoth();
            if (nb) return nb;
            const sc = WB.myCtrl.scenario(ctx, { ref: { type: 'step', amplitude: LS_STEP, tStep: 0 }, tEnd: 5 });
            const mine = await WB.myCtrl.run(ctx, code, sc);
            if (mine.ok === false) return mine;
            const d = this.design(ctx);
            // The repo's transferFunction updates the state, then outputs; a filter that outputs first is fine too.
            const repo = lib().loopshapeCtrl({ Cin: d.Ci, Cout: d.Co, F: T.gain(1), Ts: sc.Ts });
            const fi = T.filter(d.Ci, sc.Ts), fo = T.filter(d.Co, sc.Ts);
            const first = { update: (r, x, y) => ({ u: fi.step(fo.step(r - y[0]) - y[1]) }) };
            const e = Math.min(WB.myCtrl.maxDiff(mine, WB.myCtrl.reference(ctx, sc, repo)).e, WB.myCtrl.maxDiff(mine, WB.myCtrl.reference(ctx, sc, first)).e);
            const tol = 0.03 * LS_STEP;
            return e <= tol ? { ok: true, msg: `Your z(t) matches C_in, C_out run by the workbench to within ${fmt(e * 1000, 3)} mm.` } : { ok: false, msg: `Your z(t) differs from C_in, C_out run by the workbench by ${fmt(e * 1000, 3)} mm.` };
          },
          solution: () => [{ code: LS_SOL }, { html: 'As the repo\'s ctrlLoopshape.py / loopshape_tools.py transferFunction (controllable canonical form, outer then inner), with RK4 substeps. The repo\'s single RK4 step per T<sub>s</sub> passes this check too, but with C<sub>in</sub>\'s pole at 155 rad/s (|p|T<sub>s</sub> = 1.55) it is inaccurate enough that the d = 0.5 N of the chapter\'s simulation drops the rod; hw18_pendulumSim.py uses the Tustin digitalFilter instead, which holds it.' }],
        }),
        WB.myCtrl.part(ctx, {
          id: 'd', title: 'Add a prefilter F(s) on z<sub>r</sub> for a good step response', seed: 'B.18/c',
          html: 'Filter z<sub>r</sub> before the outer error: e = F(z<sub>r</sub>) − z, with F implemented like C. The check runs the ±0.5 m square wave with exact parameters: the rod must stay within 30°, the first step may overshoot by at most 20%, and |z − z<sub>r</sub>| just before the first switch (t = 12.5 s) must be under 1 cm.',
          check: async (code) => {
            const nb = needBoth();
            if (nb) return nb;
            const sc = WB.myCtrl.scenario(ctx, { ref: { type: 'square', amplitude: 0.5, frequency: 0.04, tStep: 0 }, tEnd: 12.5 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const n = res.t.length;
            const z = res.yAll[0];
            const os = 100 * (Math.max(...z) - 0.5) / 0.5;
            const e = Math.abs(res.rAll[0][n - 6] - z[n - 6]);
            const th = res.x.reduce((m, x) => Math.max(m, Math.abs(x[1])), 0) / DEG;
            return { ok: th < 30 && os <= 20 && e < 0.01, msg: `Max |θ| ${fmt(th, 3)}°, overshoot ${fmt(os, 3)}%, |z − z_r| before the switch ${fmt(e * 100, 3)} cm.` };
          },
          solution: () => [
            { code: LS_SOL.replace('        self.C_out = TransferFunction(P.C_out_num, P.C_out_den)\n', '        self.C_out = TransferFunction(P.C_out_num, P.C_out_den)\n        p = 2.0   # prefilter pole\n        self.F = TransferFunction([p], [1, p])\n').replace('self.C_out.update(z_r - z)', 'self.C_out.update(self.F.update(z_r) - z)') },
            { html: 'The repo and the text both use F = 2/(s + 2) (p. 355, loopShapingOuter.py). Without it the 0.5 m steps saturate F and the rod falls with the repo design.' },
          ],
        }),
      ]);
    },
  };

  B.freq = { Pin, Pout, Cin, Cout, bodeB, marginsB, bandwidth };
})();
