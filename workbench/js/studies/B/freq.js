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
        if (!shows(ctx, 'B.10/c3')) delete g.kIz;
        Object.assign(gainsOf(ctx), g);
        ctx.update();
      } }));
    }
    sec.append(row);
  }
  function gainControls(parent, ctx) {
    const sec = section(parent, ctx.S.mode === 'work' ? 'Your B.10 gains (hw16.py transfer functions)' : 'B.10 gains (hw16.py transfer functions)', 'p. 163–164, p. 297');
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
        { title: 'Controllers (dirty derivative)', page: 'p. 297, hw16.py',
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
          html: 'Use the B.10 gains (hw16.py). In your code: bode(P_in) and bode(P_in·C_in) on one graph. Here: Loop = inner, and the gains on the right (in Work mode, Use my Ch 10 gains).' },
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
    defaults(sys) { return { ...pidDefaults(sys), loop: 'both', outerModel: 'impl' }; },
    simDefaults(sys) { return sys.problems.ch17.sim; },
    controller: pidController,
    // Outer loop gain. 'book': P_out·C_out as hw17.py. 'impl': what the B.10 code
    // closes, C_out · filter · (inner closed loop θ_r → θ) · P_out.
    outerLoop(ctx) {
      const p = ctx.pModel, g = gainsOf(ctx);
      if (ctx.st.outerModel === 'book') return T.mul(Pout(p), Cout(g));
      const P = Pin(p), Ci = Cin(g);
      // θ/θ_r with k_Pθ on the error and the D term on θ: P k_Pθ / (1 + P C_in)
      const Tin = T.tf(L.conv(L.conv(P.num, [g.kPth]), Ci.den), L.polyAdd(L.conv(P.den, Ci.den), L.conv(P.num, Ci.num)));
      const zf = lib().zcFilter(p, lib().dcGain(p, g.kPth), 0.01);
      return T.mul(Cout(g), T.tf([zf.a], [1, zf.b]), Tin, Pout(p));
    },
    loops(ctx) {
      const p = ctx.pModel, g = gainsOf(ctx);
      const Li = T.mul(Pin(p), Cin(g)), Lo = this.outerLoop(ctx);
      const Ti = T.feedback(Li), To = T.feedback(Lo);
      return { Li, Lo, Ti, To, mi: marginsB(Li), mo: marginsB(Lo), bwi: bandwidth(Ti), bwo: bandwidth(To) };
    },
    buildControls(parent, ctx) {
      gainControls(parent, ctx);
      const sec = section(parent, 'Margins and bandwidths', 'p. 314–317');
      loopToggle(sec, ctx, [{ value: 'inner', label: 'inner' }, { value: 'outer', label: 'outer' }, { value: 'both', label: 'both closed loops' }]);
      segmented(sec, {
        label: 'Outer-loop model',
        options: [{ value: 'impl', label: 'as implemented: C_out·F·T_in·P_out' }, { value: 'book', label: 'P_out·C_out (hw17.py)' }],
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
          numbers: `\\text{inner: } PM = ${tex(l.mi.pm)}^\\circ \\text{ at } ${tex(l.mi.wc)};\\quad \\text{outer: } PM = ${tex(l.mo.pm)}^\\circ \\text{ at } ${tex(l.mo.wc)}`, spoiler: true,
          note: 'P_in C_in starts at −180° (negative DC gain), so the inner "gain margin" is a gain decrease: below 1/|L(0)| the loop cannot hold the pendulum up. The book prints GM −6.62 and PM 32.9° (Fig. 17-12), from other gains than B.10\'s.' },
        { title: 'Bandwidth vs. crossover', page: 'p. 316–317',
          theory: '\\omega_{bw}: \\text{first } \\omega \\text{ where } |T(j\\omega)| \\text{ is 3 dB below } |T(0)|,\\quad T = \\frac{PC}{1 + PC}',
          numbers: `\\omega_{bw,in} = ${tex(l.bwi)},\\; \\omega_{co,in} = ${tex(l.mi.wc)};\\quad \\omega_{bw,out} = ${tex(l.bwo)},\\; \\omega_{co,out} = ${tex(l.mo.wc)}`, spoiler: true,
          note: 'T_in(0) = k_DC ≠ 1, so the −3 dB point is measured from |T(0)|, not from 1 (as control.bandwidth does).' },
        { title: 'Successive loop closure', page: 'p. 317',
          theory: '\\text{10× faster inner loop} \\leftrightarrow \\text{inner bandwidth a decade above the outer}',
          numbers: `\\omega_{bw,in}/\\omega_{bw,out} = ${tex(l.bwi / l.bwo)}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const l = () => this.loops(ctx);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch17, [
        { id: 'a', title: '(a) Inner loop: PM, crossover, bandwidth', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
          check: (v) => { const x = l(); return PD().checkNumbers(v, { pm: x.mi.pm, wc: x.mi.wc, bw: x.bwi }, { pm: 'PM', wc: 'ωco', bw: 'ωbw' }); },
          solution: () => { const x = l(); return [{ tex: `PM = ${tex(x.mi.pm)}^\\circ \\text{ at } ${tex(x.mi.wc)}\\,\\text{rad/s},\\quad \\omega_{bw} = ${tex(x.bwi)}` }, { html: `GM: ${gmText(x.mi.gms)}.` }, { html: 'Book: PM 32.88°, GM −6.62, crossover ≈ 10 rad/s, bandwidth ≈ 19 rad/s, slightly above crossover (p. 317).' }]; } },
        { id: 'b', title: '(b) Outer loop: PM, crossover, bandwidth', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
          check: (v) => { const x = l(); return PD().checkNumbers(v, { pm: x.mo.pm, wc: x.mo.wc, bw: x.bwo }, { pm: 'PM', wc: 'ωco', bw: 'ωbw' }); },
          solution: () => { const x = l(); return [{ tex: `PM = ${tex(x.mo.pm)}^\\circ \\text{ at } ${tex(x.mo.wc)}\\,\\text{rad/s},\\; \\omega_{bw} = ${tex(x.bwo)}` }, { html: 'Book: PM 72° (the plot title says −108.52; 180 − 108 = 72) at about 1 rad/s and bandwidth ≈ 1.3 rad/s (p. 317), from hw17\'s P<sub>out</sub>·C<sub>out</sub>. With the B.10 gains that loop never drops below 0 dB (its high-frequency gain is about 4.5), so this page defaults to the loop the code actually closes: C<sub>out</sub> · filter · inner closed loop · P<sub>out</sub>.' }]; } },
        { id: 'c', title: '(c) Bandwidth separation', inputs: { r: 'ω<sub>bw,in</sub>/ω<sub>bw,out</sub>' },
          check: (v) => { const x = l(); return PD().checkNumbers(v, { r: x.bwi / x.bwo }, { r: 'ratio' }); },
          solution: () => { const x = l(); return [{ tex: `\\frac{\\omega_{bw,in}}{\\omega_{bw,out}} = \\frac{${tex(x.bwi)}}{${tex(x.bwo)}} = ${tex(x.bwi / x.bwo)}` }, { html: 'About a decade, so successive loop closure is justified (p. 317).' }]; } },
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

  B.chapters.ch18 = {
    id: 'ch18', num: 18, tab: 'Ch 18', title: 'Loopshaping', pages: 'pp. 323–374',
    defaults() { return { ...presetNone(), loop: 'inner', method: 'state_space', showT: true }; },
    simDefaults(sys) { return { ...sys.problems.ch18.sim, mismatch: sys.problems.ch18.mismatch }; },
    linearLabel: 'linear model (no saturation, d, noise)',

    design(ctx) {
      const st = ctx.st, p = ctx.pModel, bl = lib().blocks, pr = ctx.sys.problems.ch18;
      let Ci = bl.prop(st.kin);
      if (st.inLead.on) Ci = T.mul(Ci, bl.lead(st.inLead.w, st.inLead.M));
      let Co = bl.prop(st.kout);
      if (st.outLead.on) Co = T.mul(Co, bl.lead(st.outLead.w, st.outLead.M));
      if (st.lag.on) Co = T.mul(Co, bl.lag(st.lag.z, st.lag.M));
      if (st.lpf.on) Co = T.mul(Co, bl.lpf(st.lpf.p));
      const F = st.pf.on ? bl.lpf(st.pf.p) : T.gain(1);
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
      const view = section(parent, 'Bode view');
      segmented(view, { options: [{ value: 'inner', label: 'inner: P_in C_in' }, { value: 'outer', label: 'outer: P C_out' }], ...bind(ctx, 'loop') });
      segmented(view, { label: 'Closed loop on the Bode plot', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'showT') });

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
        label: 'Implementation (ctrlLoopshape.py)',
        options: [{ value: 'state_space', label: 'state space, RK4' }, { value: 'digital_filter', label: 'Tustin filter (hw18 default)' }],
        ...bind(ctx, 'method'),
      });

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
        { title: 'Building blocks (loopshape_tools.py)', page: 'p. 324–328',
          theory: '\\text{lead}(\\omega, M) = \\frac{\\sqrt M s + \\omega}{s + \\omega\\sqrt M},\\quad \\text{lag}(z, M) = \\frac{s + z}{s + z/M},\\quad \\text{lpf}(p) = \\frac{p}{s + p}',
          note: 'The repo\'s lead has DC gain 1/√M and high-frequency gain √M; the book\'s Eq. 18.2 lead has DC gain 1, so it is √M times larger.' },
        { title: 'Your controllers', page: 'p. 349, p. 355',
          theory: `C_{in}(s) = ${T.texTf(d.Ci, 4)},\\quad C_{out}(s) = ${T.texTf(d.Co, 4)}`, numbers: null },
        { title: 'Outer-loop plant', page: 'p. 349 · B.18(b)',
          theory: 'P = P_{out}\\frac{P_{in}C_{in}}{1 + P_{in}C_{in}}',
          note: 'The inner closed loop has DC gain P_in C_in(0)/(1 + P_in C_in(0)) ≠ 1, so P_out sees it as an extra gain.' },
        { title: 'Margins', page: 'p. 351, p. 356',
          theory: `\\text{inner } PM = ${tex(d.mi.pm)}^\\circ \\text{ at } ${tex(d.mi.wc)},\\quad \\text{outer } PM = ${tex(d.mo.pm)}^\\circ \\text{ at } ${tex(d.mo.wc)}` },
        { title: 'Implementation', page: 'p. 336 · Eq. 18.3–18.7, ctrlLoopshape.py',
          theory: '\\tilde z_r = F(z_r),\\quad \\theta_r = C_{out}(\\tilde z_r - z),\\quad F = \\text{sat}\\big(C_{in}(\\theta_r - \\theta)\\big)',
          note: ctx.st.method === 'digital_filter' ? 'Tustin: s → (2/T_s)(z − 1)/(z + 1), then a difference equation (digitalFilter).' : 'Control-canonical state space, one RK4 step per sample (transferFunction).' },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch18, [
        { id: 'a', title: '(a) Inner loop meets its specs',
          check: () => { const { ok, mi, bwi, inNoise } = this.design(ctx); const good = ok.inPM && ok.inNoise && ok.inBw; return { ok: good, msg: `PM ${fmt(mi.pm, 3)}° ${ok.inPM ? '✓' : '✗'}, noise ${fmt(db(inNoise), 3)} dB ${ok.inNoise ? '✓' : '✗'}, bandwidth ${fmt(bwi, 3)} rad/s ${ok.inBw ? '✓' : '✗'}` }; },
          solution: () => [{ html: 'Repo (Listing 18.5): k = −1 × 800 to cross at 40 rad/s, then lead(ω = 40, M = 15). The text instead prints C<sub>lead</sub> = −155.12 (s/10.72 + 1)/(s/120 + 1) and calls it a ratio-13.9 lead at 40 rad/s; 10.72·13.9 is 149, not 120 (p. 349–350). Try both presets.' }] },
        { id: 'b', title: '(b) Outer loop meets its specs',
          check: () => { const { ok, mo, outTrack, outNoise } = this.design(ctx); const good = ok.outPM && ok.outTrack && ok.outNoise; return { ok: good, msg: `PM ${fmt(mo.pm, 3)}° ${ok.outPM ? '✓' : '✗'}, tracking ${fmt(db(outTrack), 3)} dB ${ok.outTrack ? '✓' : '✗'}, noise ${fmt(db(outNoise), 3)} dB ${ok.outNoise ? '✓' : '✗'}` }; },
          solution: () => [{ html: 'The text (p. 354–355): lead at 1.1 rad/s with ratio 32, gain 0.0146 for crossover at 1.1 rad/s, lag ratio 8, LPF at 100 rad/s, giving C<sub>out</sub> = 0.469 (s + 0.194)/(s + 6.24) · (s + 0.0256)/(s + 0.0032) · 100/(s + 100), and F = 2/(s + 2). Listing 18.6 and the repo use 0.1·lead(1, 20)·lag(0.04, 10)·lpf(50). Both presets are on this page.' }] },
      ]);
    },
  };

  B.freq = { Pin, Pout, Cin, Cout, bodeB, marginsB, bandwidth };
})();
