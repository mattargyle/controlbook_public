// Study C, chapters 15-18: frequency response of the two loop plants, frequency
// specifications and stability margins of the C.10 PD/PID loops, and
// loopshaping both loops of the successive-loop-closure design.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const { tex, fmt, fmtPole } = M;
  const PD = () => WB.pd;
  const lib = () => WB.studies.C.lib;
  const CH = WB.studies.C.chapters;
  const R2D = 180 / Math.PI;
  const W = T.logspace(-4, 4, 800);
  const { db, mag: magAt } = T;
  const gmText = (mg) => WB.freq.gmText(mg);

  // Loop plants (C.15, p. 280): inner τ → θ with (Js+Jp)/Js ≈ 1, outer θ → φ.
  const pIn = (p) => T.tf([1], [p.Js + p.Jp, 0, 0]);
  const pOut = (p) => T.tf([p.b / p.Jp, p.k / p.Jp], [1, p.b / p.Jp, p.k / p.Jp]);
  // Exact Θ/τ and Φ/τ (Eq. 5.6) for comparison.
  function exactTf(p) {
    const { Js, Jp, k, b } = p, J = Js + Jp;
    const den = [1, b * J / (Js * Jp), k * J / (Js * Jp), 0, 0];
    return { th: T.tf([1 / Js, b / (Js * Jp), k / (Js * Jp)], den), ph: T.tf([b / (Js * Jp), k / (Js * Jp)], den) };
  }
  // C_PD and C_PID with the dirty derivative (hw16.py):
  //   C_in = ((kD + σkP)s + kP)/(σs + 1),  C_out = ((kD + σkP)s² + (kP + σkI)s + kI)/(σs² + s)
  const cIn = (g, s) => T.pid({ kP: g.kPth, kD: g.kDth, sigma: s });
  const cOut = (g, s) => T.pid({ kP: g.kPphi, kI: g.kIphi, kD: g.kDphi, sigma: s });

  // C.10 gains (repo, hw16.py) and the gains the book's C.16/C.17 figures were made with (C.8 loops, k_I = 0.15).
  function c10(sys, p) {
    const pr = sys.problems.ch10;
    return { ...lib().slcDesign(p, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, ki: pr.ki }), sigma: pr.sigma };
  }
  function c8fig(sys, p) {
    const pr = sys.problems.ch8;
    return { ...lib().slcDesign(p, { trTh: pr.trTh, zetaTh: pr.zetaTh, M: pr.M, zetaPhi: pr.zetaPhi, rule: pr.rule, ki: 0.15 }), sigma: 0.05 };
  }
  const bookParams = (sys) => Object.fromEntries(sys.params.map((q) => [q.key, q.value]));
  const pick = (g) => ({ kPth: g.kPth, kDth: g.kDth, kPphi: g.kPphi, kDphi: g.kDphi, kIphi: g.kIphi, sigma: g.sigma });

  // Closed-loop bandwidth: the highest frequency where |T| is still above −3 dB
  // (the outer loop's |T| dips below −3 dB near 0.08 rad/s, below the panel resonance
  // at √(k/Jp) ≈ 0.32 rad/s, and recovers). −3 dB is relative to the DC gain, as
  // in control.bandwidth.
  function bandwidth(Tc) {
    const c = T.crossDown(Tc, W, Math.SQRT1_2 * magAt(Tc, 1e-6));
    return c.length ? c[c.length - 1] : NaN;
  }
  const marginMarks = (mg) => WB.freq.marginMarks(mg);

  // PID-cascade controls shared by C.16 and C.17.
  function gainControls(parent, ctx) {
    const st = ctx.st;
    const sec = section(parent, 'C.10 loops: PD inner, PID outer', 'p. 167');
    for (const [key, label, min, max] of [['kPth', 'k<sub>P<sub>θ</sub></sub>', 0, 400], ['kDth', 'k<sub>D<sub>θ</sub></sub>', 0, 150], ['kPphi', 'k<sub>P<sub>φ</sub></sub>', 0, 5], ['kIphi', 'k<sub>I<sub>φ</sub></sub>', 0, 2], ['kDphi', 'k<sub>D<sub>φ</sub></sub>', 0, 30]]) {
      slider(sec, { label, min, max, step: (max - min) / 4000, sig: 4, ...bind(ctx, key, () => st) });
    }
    slider(sec, { label: 'σ', unit: 's', min: 0.005, max: 0.3, step: 0.001, sig: 3, ...bind(ctx, 'sigma', () => st) });
    // "Load my C.10 gains" copies the Work-mode gains of the Ch 10 tab. The C.10 and
    // C.8 designs answer C.10(c) and C.8(b, d), so in Work mode their buttons appear
    // once those parts are solved.
    const row = el('div', { class: 'btn-row' },
      el('button', { type: 'button', class: 'btn btn-quiet', text: 'Load my C.10 gains', title: 'Copy the Work-mode gains and σ from the Ch 10 tab', onclick: () => {
        const c = ctx.S.ch.ch10;
        if (c && c.w) Object.assign(st, { kPth: c.w.kPth, kDth: c.w.kDth, kPphi: c.w.kPphi, kDphi: c.w.kDphi, kIphi: c.w.kIphi, sigma: c.sigma ?? st.sigma });
        ctx.update();
      } }));
    if (lib().shows(ctx, 'C.10/c1')) row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'C.10 gains (repo)', onclick: () => { Object.assign(st, pick(c10(ctx.sys, bookParams(ctx.sys)))); ctx.update(); } }));
    if (lib().shows(ctx, 'C.8/b', 'C.8/d')) row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'book figures (C.8 loops)', title: 't_rθ = 1 s, M = 10, ζ = 0.9, k_I = 0.15', onclick: () => { Object.assign(st, pick(c8fig(ctx.sys, bookParams(ctx.sys)))); ctx.update(); } }));
    sec.append(row);
    segmented(sec, {
      label: 'Bode plot',
      options: [{ value: 'inner', label: 'inner loop (θ)' }, { value: 'outer', label: 'outer loop (φ)' }],
      ...bind(ctx, 'view', () => st),
    });
    return sec;
  }
  const cascadeCtl = (ctx) => {
    const st = ctx.st;
    const fake = Object.create(ctx);
    fake.gains = { kPth: st.kPth, kDth: st.kDth, kPphi: st.kPphi, kDphi: st.kDphi, kIphi: st.kIphi };
    fake.st = { ff: false, deriv: 'dirty', antiwindup: 'repo', sigma: st.sigma, thetaMaxDeg: 30 };
    return WB.studies.C.cascade.makeCascade(fake);
  };
  function loops(ctx, st = ctx.st) {
    const p = ctx.pModel;
    const Lin = T.mul(pIn(p), cIn(st, st.sigma)), Lout = T.mul(pOut(p), cOut(st, st.sigma));
    const Tin = T.feedback(Lin), Tout = T.feedback(Lout);
    return { Lin, Lout, Tin, Tout, mgIn: T.margins(Lin), mgOut: T.margins(Lout), bwIn: bandwidth(Tin), bwOut: bandwidth(Tout) };
  }
  // showOl: whether the open-loop poles (which include the plant's) may be shown;
  // in Work mode they wait for the part that derives the plant.
  function clMarkers(ctx, Lg, title, showOl = true) {
    const poles = L.roots(L.polyAdd(Lg.den, Lg.num));
    const mk = showOl ? L.roots(Lg.den).map((q, i) => ({ ...q, kind: 'ol', label: `pole of ${title} ${i + 1}` })) : [];
    poles.forEach((q, i) => mk.push({ ...q, kind: 'cl', label: `closed-loop pole ${i + 1}`, noFit: Math.hypot(q.re, q.im) > 40 }));
    return { markers: mk };
  }

  // ------------------------------------------------------------- C.15 --
  CH.ch15 = {
    id: 'ch15', num: 15, tab: 'Ch 15', title: 'Frequency response', pages: 'pp. 280–282',
    openLoop: true, metrics: false, linear: false,
    defaults() { return { view: 'inner', w0: 0.3, A: 0.05, exact: true }; },
    simDefaults(sys) { return sys.problems.ch15.sim; },
    controller(ctx) { const { A, w0 } = ctx.st; return { update: (r, x, y, t) => A * Math.sin(w0 * t) }; },
    // Predicted steady sinusoids from the exact transfer functions, plus the
    // constant/ramp drift of the free double integrator (fitted over the last period).
    // The predicted sinusoid is the plant's frequency response at ω₀, so in Work mode
    // the θ prediction waits for part (a) and the φ prediction for part (b).
    outputSeries(ctx, res, sc, oi) {
      if (!lib().shows(ctx, oi === 0 ? 'C.15/a' : 'C.15/b')) return [];
      const ex = exactTf(ctx.pModel), G = oi === 0 ? ex.th : ex.ph;
      const g = T.at(G, ctx.st.w0), mag = L.C.abs(g), ph = L.C.arg(g), w = ctx.st.w0;
      const pred = Array.from(res.t, (t) => ctx.st.A * mag * Math.sin(w * t + ph));
      // drift a + b t from the initial transient (double integrator), least squares on the residual
      const y = res.yAll[oi], n = y.length;
      let st = 0, stt = 0, sy = 0, sty = 0;
      for (let k = 0; k < n; k++) { const t = res.t[k], r = y[k] - pred[k]; st += t; stt += t * t; sy += r; sty += t * r; }
      const bb = (n * sty - st * sy) / (n * stt - st * st), aa = (sy - bb * st) / n;
      return [{ label: 'A|G(jω₀)| sin(ω₀t + ∠G) + drift', y: sc(pred.map((v, k) => v + aa + bb * res.t[k])), color: '--series-3', dash: [2, 3], width: 2 }];
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Sinusoidal torque τ = A sin(ω₀t)', 'p. 264 · Eq. 15.4');
      slider(sec, { label: 'ω<sub>0</sub>', unit: 'rad/s', min: 0.02, max: 20, log: true, sig: 3, ...bind(ctx, 'w0') });
      slider(sec, { label: 'A', unit: 'N·m', min: 0, max: 1, step: 0.005, sig: 3, ...bind(ctx, 'A') });
      segmented(sec, {
        label: 'Bode plot',
        options: [{ value: 'inner', label: 'P_in: τ → θ' }, { value: 'outer', label: 'P_out: θ → φ' }],
        ...bind(ctx, 'view'),
      });
      segmented(sec, {
        label: 'Also show the exact transfer function (Eq. 5.6)',
        options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }],
        ...bind(ctx, 'exact'),
      });
      sec.append(el('p', { class: 'muted small', text: 'The double integrator makes θ drift after the start-up transient; the dotted prediction includes that drift. Sweep ω₀ to find the panel mode. In Work mode each loop\'s Bode plot and predicted sinusoid appear once you have drawn it (parts a and b).' }));
    },
    // C.15(a) and (b) are drawn by hand: in Work mode each loop's Bode plot (and
    // its poles and zeros) appears once that part is done.
    drawn: (ctx) => ctx.S.mode === 'explore' || ctx.app.isSolved(ctx.st.view === 'inner' ? 'C.15/a' : 'C.15/b'),
    bode(ctx) {
      if (!this.drawn(ctx)) return null;
      const p = ctx.pModel, ex = exactTf(p);
      const lines = [];
      if (ctx.st.view === 'inner') {
        lines.push({ label: 'P_in = 1/((Js+Jp)s²)', ...T.bode(pIn(p), W), color: '--series-1' });
        if (ctx.st.exact) lines.push({ label: 'exact Θ/τ', ...T.bode(ex.th, W), color: '--series-2', dash: [5, 4], width: 1.5 });
      } else {
        lines.push({ label: 'P_out = Φ/Θ', ...T.bode(pOut(p), W), color: '--series-1' });
        if (ctx.st.exact) lines.push({ label: 'exact (Φ/τ)/(Θ/τ)', ...T.bode(T.tf(ex.ph.num, ex.th.num), W), color: '--series-2', dash: [5, 4], width: 1.5 });
      }
      const G = ctx.st.view === 'inner' ? pIn(p) : pOut(p), g = T.at(G, ctx.st.w0);
      return { title: ctx.st.view === 'inner' ? 'Bode plot of P_in(s)' : 'Bode plot of P_out(s)', w: W, lines, marks: [{ w: ctx.st.w0, label: `ω₀: ${fmt(db(L.C.abs(g)), 3)} dB, ${fmt(L.C.arg(g) * R2D, 3)}°`, color: '--series-3' }] };
    },
    splane(ctx) {
      if (!this.drawn(ctx)) return { markers: [] };
      const G = ctx.st.view === 'inner' ? pIn(ctx.pModel) : pOut(ctx.pModel);
      const mk = L.roots(G.den).map((q, i) => ({ ...q, kind: 'ol', label: `pole ${i + 1}` }));
      if (G.num.length > 1) L.roots(G.num).forEach((q) => mk.push({ ...q, kind: 'olzero', label: 'zero' }));
      return { markers: mk };
    },
    params(ctx) {
      const p = ctx.pModel, wn = Math.sqrt(p.k / p.Jp);
      return { K: 1 / (p.Js + p.Jp), z: p.k / p.b, wn, zeta: p.b / (2 * p.Jp * wn), peak: db(magAt(pOut(p), wn)) };
    },
    math(ctx) {
      const q = this.params(ctx);
      return [
        { title: 'Frequency response', page: 'p. 264 · Eq. 15.4',
          theory: 'u = A\\sin\\omega_0 t \\;\\Rightarrow\\; y_{ss} = A|G(j\\omega_0)|\\sin\\big(\\omega_0 t + \\angle G(j\\omega_0)\\big)' },
        { title: 'Bode canonical form', page: 'p. 266 · Eq. 15.5–15.7',
          theory: 'P(j\\omega) = K\\,\\frac{\\prod (1 + j\\omega/z_i)}{(j\\omega)^q \\prod (1 + j\\omega/p_i)}:\\quad 20\\log|P| = 20\\log K + \\textstyle\\sum 20\\log|1 + j\\omega/z_i| - 20q\\log\\omega - \\sum 20\\log|1 + j\\omega/p_i|' },
        { title: 'Second-order pole', page: 'p. 271–273',
          theory: '\\Big(1 + 2\\zeta\\tfrac{j\\omega}{\\omega_n} + \\big(\\tfrac{j\\omega}{\\omega_n}\\big)^2\\Big)^{-1}:\\; 0\\text{ dB below } \\omega_n,\\; -40\\text{ dB/dec above},\\; \\text{peak} \\approx \\tfrac{1}{2\\zeta},\\; \\angle: 0^\\circ \\to -180^\\circ' },
        { title: 'Inner loop of the satellite in Bode form', page: 'p. 280 · Eq. 15.20–15.21', answers: 'C.15/a',
          theory: 'P_{in}(j\\omega) = \\frac{1}{J_s + J_p}\\frac{1}{(j\\omega)(j\\omega)},\\quad 20\\log|P_{in}| = 20\\log\\tfrac{1}{J_s+J_p} - 40\\log|\\omega|,\\quad \\angle P_{in} = -180^\\circ',
          numbers: `20\\log_{10}\\tfrac{1}{J_s+J_p} = ${tex(db(q.K))}\\,\\text{dB}` },
        { title: 'Outer loop of the satellite in Bode form', page: 'p. 280–281 · Eq. 15.22–15.23', answers: 'C.15/b',
          theory: 'P_{out}(j\\omega) = \\frac{1 + j\\omega/(k/b)}{1 + j\\omega\\frac{b}{k} + \\left(\\frac{j\\omega}{\\sqrt{k/J_p}}\\right)^2}:\\; \\text{zero at } \\tfrac{k}{b},\\; \\text{lightly damped pair at } \\omega_n = \\sqrt{k/J_p}',
          numbers: `\\tfrac{k}{b} = ${tex(q.z)},\\quad \\omega_n = ${tex(q.wn)},\\quad \\zeta = \\tfrac{b}{2J_p\\omega_n} = ${tex(q.zeta)},\\quad |P_{out}(j\\omega_n)| = ${tex(q.peak)}\\,\\text{dB}`,
          note: 'The C.15 solution prints (0.05s + 0.15)/(s² + 0.05s + 0.15), zero at 3 and ωn = 0.3873, which is k = 0.15 rather than the book\'s 0.1 (see ISSUES.md).' },
      ];
    },
    buildProblem(parent, ctx) {
      const q = () => this.params(ctx);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch15, [
        {
          id: 'a', title: '(a) Draw the inner-loop Bode plot by hand',
          html: 'On paper: take the transfer function from τ to θ (C.5), put it in Bode canonical form and sketch the straight-line magnitude and phase. When you are done, click the button to see the comparison part and the workbench\'s Bode plot (<em>P_in</em> view).',
          done: 'I\'ve drawn it: next',
          solution: () => [{ tex: `P_{in}(j\\omega) = \\frac{1}{J_s + J_p}\\frac{1}{(j\\omega)^2}:\\; ${tex(db(q().K))}\\,\\text{dB at } \\omega = 1,\\; -40\\,\\text{dB/dec},\\; -180^\\circ` }, { html: 'Book: Eq. 15.21 and Fig. 15-15 (p. 280–281).' }],
        },
        {
          id: 'a2', title: '(a) Compare with bode: straight-line pieces', after: 'a',
          inputs: { K: 'gain at ω = 1 [dB]', s: 'slope [dB/dec]', ph: 'phase [°]' },
          check: (v) => PD().checkNumbers(v, { K: db(q().K), s: -40, ph: -180 }, { K: 'gain', s: 'slope', ph: 'phase' }),
          solution: () => [{ tex: `20\\log\\tfrac{1}{J_s+J_p} = ${tex(db(q().K))}\\,\\text{dB at } \\omega = 1,\\; -40\\,\\text{dB/dec},\\; -180^\\circ` }, { html: 'Book: Eq. 15.21 and Fig. 15-15 (p. 280–281): the magnitude is the constant gain 1/6 (−15.6 dB) plus two −20 dB/dec lines through 0 dB at ω = 1.' }],
        },
        {
          id: 'b', title: '(b) Draw the outer-loop Bode plot by hand',
          html: 'On paper: take the transfer function from θ to φ (C.5), put it in Bode canonical form and sketch the straight-line magnitude and phase. When you are done, click the button to see the comparison part and the workbench\'s Bode plot (<em>P_out</em> view).',
          done: 'I\'ve drawn it: next',
          solution: () => [{ tex: `P_{out}(j\\omega) = \\frac{1 + j\\omega/${tex(q().z)}}{1 + 2(${tex(q().zeta)})\\frac{j\\omega}{${tex(q().wn)}} + \\left(\\frac{j\\omega}{${tex(q().wn)}}\\right)^2}` }, { html: 'Book: Eq. 15.23 and Fig. 15-16 (p. 281–282).' }],
        },
        {
          id: 'b2', title: '(b) Compare with bode: break frequencies and resonance', after: 'b',
          inputs: { z: 'zero [rad/s]', wn: 'ω<sub>n</sub> [rad/s]', zeta: 'ζ', pk: '|P<sub>out</sub>(jω<sub>n</sub>)| [dB]' },
          check: (v) => PD().checkNumbers(v, { z: q().z, wn: q().wn, zeta: q().zeta, pk: q().peak }, { z: 'zero', wn: 'ωn', zeta: 'ζ', pk: 'peak' }),
          solution: () => [{ tex: `\\text{zero at } ${tex(q().z)},\\quad \\omega_n = ${tex(q().wn)},\\quad \\zeta = ${tex(q().zeta)},\\quad \\text{peak} \\approx ${tex(q().peak)}\\,\\text{dB}` }, { html: 'Book: Eq. 15.23, Fig. 15-16 (p. 281–282).' }],
        },
      ]);
    },
  };

  // ------------------------------------------------------------- C.16 --
  // Work mode starts from placeholder gains; the buttons load the C.10 or book-figure gains.
  const pidDefaults = () => ({ kPth: 60, kDth: 30, kPphi: 0.5, kIphi: 0.05, kDphi: 5, sigma: 0.05, view: 'inner' });

  // Random arguments for the C.16 Python answers (functions of the loop gains).
  const PY = {
    kPin: { label: 'kP', lo: 10, hi: 200 }, kDin: { label: 'kD', lo: 5, hi: 80 },
    kPout: { label: 'kP', lo: 0.1, hi: 3 }, kIout: { label: 'kI', lo: 0.005, hi: 0.5 }, kDout: { label: 'kD', lo: 1, hi: 15 },
    sigma: { label: 'σ', lo: 0.01, hi: 0.1 },
  };
  const flat1 = (v) => (Array.isArray(v) ? v[0] : v);

  CH.ch16 = {
    id: 'ch16', num: 16, tab: 'Ch 16', title: 'Frequency-domain specs', pages: 'pp. 298–301',
    linear: false,
    defaults(sys) { const pr = sys.problems.ch16; return { ...pidDefaults(sys), A: pr.parabA, wdin: pr.wdin, wno: pr.wno }; },
    simDefaults(sys) { return sys.problems.ch16.sim; },
    controller: cascadeCtl,

    specs(ctx, st = ctx.st) {
      const p = ctx.pModel;
      const Ci = cIn(st, st.sigma), l = loops(ctx, st);
      const Ma = st.kPth / (p.Js + p.Jp);                 // lim s² P_in C_in
      const Bdin = db(magAt(Ci, st.wdin)), Bn = -db(magAt(l.Lout, st.wno));
      return { Ma, B2: db(Ma), eParab: 2 * st.A / Ma, eBook: st.A / Ma, Bdin, gdin: Math.pow(10, -Bdin / 20), Bn, gn: Math.pow(10, -Bn / 20), l };
    },

    buildControls(parent, ctx) {
      gainControls(parent, ctx);
      const sec = section(parent, 'Spec frequencies', 'p. 298');
      slider(sec, { label: 'parabola A (θ_r = At²)', unit: 'rad/s²', min: 0.1, max: 50, step: 0.1, sig: 3, ...bind(ctx, 'A') });
      slider(sec, { label: 'ω<sub>d,in</sub>', unit: 'rad/s', min: 0.001, max: 10, log: true, sig: 3, ...bind(ctx, 'wdin') });
      slider(sec, { label: 'ω<sub>no</sub>', unit: 'rad/s', min: 0.5, max: 1000, log: true, sig: 3, ...bind(ctx, 'wno') });
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const s = this.specs(ctx);
        const show = WB.ui.shown(ctx, 'C:ch16:specs');
        const row = WB.ui.metric;
        box.replaceChildren(...(show ? [
          row('inner: M_a = lim s²PC (type 2)', `${fmt(s.Ma, 4)} (${fmt(s.B2, 3)} dB)`),
          row('inner: e_ss for θ_r = At²', `${fmt(s.eParab, 4)} rad`),
          row('inner: d_in below ω_d,in, |C_in|', `${fmt(s.Bdin, 3)} dB → ${fmt(100 * s.gdin, 3)} %`),
          row('outer: noise above ω_no, |PC|', `${fmt(-s.Bn, 3)} dB → ${fmt(100 * s.gn, 3)} %`),
        ] : [WB.ui.revealButton(ctx, 'C:ch16:specs', 'Reveal the spec readouts')]));
      });
    },

    bode(ctx) {
      const p = ctx.pModel, s = this.specs(ctx), st = ctx.st;
      const show = WB.ui.shown(ctx, 'C:ch16:specs');
      if (st.view === 'inner') {
        const inv = T.tf([1], [1, 0, 0]);
        // P_in's Bode plot answers C.15(a): Work mode draws it once that part is done.
        const plant = lib().shows(ctx, 'C.15/a');
        return { title: 'Inner loop: P_in and P_in·C_in', w: W, lines: [
          ...(plant ? [{ label: 'P_in', ...T.bode(pIn(p), W), color: '--text-muted', width: 1.5 },
            { label: '1/s²', ...T.bode(inv, W), color: '--series-3', dash: [4, 4], width: 1.2 }] : []),
          { label: 'P_in·C_in', ...T.bode(s.l.Lin, W), color: '--series-1' },
        ], marks: [{ w: st.wdin, label: show ? `ω_d,in: |C_in| = ${fmt(s.Bdin, 3)} dB` : 'ω_d,in' }] };
      }
      return { title: 'Outer loop: P_out and P_out·C_out', w: W, lines: [
        ...(lib().shows(ctx, 'C.15/b') ? [{ label: 'P_out', ...T.bode(pOut(p), W), color: '--text-muted', width: 1.5 }] : []),   // answers C.15(b)
        { label: 'P_out·C_out', ...T.bode(s.l.Lout, W), color: '--series-1' },
      ], marks: [{ w: st.wno, label: show ? `ω_no: |PC| = ${fmt(-s.Bn, 3)} dB` : 'ω_no' }] };
    },
    splane(ctx) { const l = loops(ctx); return ctx.st.view === 'inner' ? clMarkers(ctx, l.Lin, 'P_in C_in', lib().shows(ctx, 'C.5/d')) : clMarkers(ctx, l.Lout, 'P_out C_out', lib().shows(ctx, 'C.5/c')); },

    math(ctx) {
      const s = this.specs(ctx), st = ctx.st;
      return [
        { title: 'Loop controllers with the dirty derivative', page: 'p. 313',
          theory: 'C_{in} = \\frac{(k_{D_\\theta} + \\sigma k_{P_\\theta})s + k_{P_\\theta}}{\\sigma s + 1},\\quad C_{out} = \\frac{(k_{D_\\phi} + \\sigma k_{P_\\phi})s^2 + (k_{P_\\phi} + \\sigma k_{I_\\phi})s + k_{I_\\phi}}{\\sigma s^2 + s}',
          numbers: `C_{in} = ${T.texTf(cIn(st, st.sigma))},\\quad C_{out} = ${T.texTf(cOut(st, st.sigma))}` },
        { title: 'Parabola tracking (type 2)', page: 'p. 293–295 · Eq. 16.12, p. 299',
          theory: 'M_a = \\lim_{s\\to0} s^2PC = 10^{B_2/20},\\quad \\theta_r = At^2 \\Rightarrow R = \\frac{2A}{s^3},\\; e_{ss} = \\frac{2A}{M_a}',
          numbers: `M_a = \\frac{k_{P_\\theta}}{J_s+J_p} = ${tex(s.Ma)}\\;(${tex(s.B2)}\\,\\text{dB}),\\quad e_{ss} = ${tex(s.eParab)}`, spoiler: true,
          note: 'The C.16(a) solution writes e ≤ A/M_a = 20·10^(−22.3/20) = 1.53, treating 20t² as R = 20/s³. Since L{t²} = 2/s³ the error is 2A/M_a.' },
        { title: 'Input disturbance', page: 'p. 290 · Eq. 16.8–16.9',
          theory: '|P| - |PC| \\text{ in dB} = -20\\log|C|:\\quad \\gamma_{d_{in}} = 10^{-B_{d_{in}}/20},\\; B_{d_{in}} = 20\\log|C_{in}(j\\omega_{d_{in}})|',
          numbers: `B_{d_{in}} = ${tex(s.Bdin)}\\,\\text{dB} \\Rightarrow \\gamma = ${tex(s.gdin)}`, spoiler: true },
        { title: 'Noise', page: 'p. 287 · Eq. 16.6',
          theory: '20\\log|PC| \\le 20\\log\\gamma_n \\text{ for } \\omega \\ge \\omega_{no}',
          numbers: `|P_{out}C_{out}(j\\omega_{no})| = ${tex(-s.Bn)}\\,\\text{dB} \\Rightarrow \\gamma_n = ${tex(s.gn)}`, spoiler: true },
      ];
    },

    buildProblem(parent, ctx) {
      const pr = ctx.sys.problems.ch16;
      const s = () => this.specs(ctx, { ...pick(c10(ctx.sys, ctx.pModel)), A: pr.parabA, wdin: pr.wdin, wno: pr.wno });
      PD().problemPanel(parent, ctx, pr, [
        { id: 'p1', title: 'Inner loop: Bode plots of the plant and of the plant under PD control',
          html: 'Use <code>bode</code> with the C.10 gains (<em>Load my C.10 gains</em> in the controls copies yours from the Ch 10 tab). The workbench\'s Bode plot (<em>inner loop</em> view) shows P<sub>in</sub>C<sub>in</sub>, and P<sub>in</sub> and 1/s² once C.15(a) is done.',
          check: () => {
            const g = pick(c10(ctx.sys, ctx.pModel)), st = ctx.st;
            const ok = ['kPth', 'kDth', 'kPphi', 'kDphi', 'kIphi', 'sigma'].every((k) => M.close(st[k], g[k], 1e-3, 1e-9));
            return ok ? { ok: true, msg: 'The sliders hold the C.10 gains.' } : { ok: false, msg: 'Set the sliders to the C.10 gains (Load my C.10 gains copies yours from the Ch 10 tab) to plot the loops the problem asks for.' };
          } },
        { id: 'a', title: '(a) Steady-state error to θ<sub>r</sub> = 20t²',
          html: 'Steady-state error (rad) as a function of the inner-loop gains, with the loop gain P<sub>in</sub>C<sub>in</sub> (PD on the error). The check calls it at random gains.',
          code: {
            template: 'def e_ss(kP, kD):\n    return ...\n',
            check: async (code) => {
              const r = await WB.py.check(ctx, {
                args: { kP: PY.kPin, kD: PY.kDin },
                items: [{ fn: 'e_ss', args: ['kP', 'kD'], truth: (p, a) => 2 * pr.parabA * (p.Js + p.Jp) / a.kP }],
                explain: (it, f) => (M.close(flat1(f.e.got), flat1(f.e.want) / 2, 1e-4) ? 'Half the expected value matches the book (A/M_a), but L{20t²} = 40/s³, so the error is 2A/M_a.' : ''),
              }, code);
              return r.ok ? { ...r, msg: `${r.msg} Right for the loop gain P_in C_in (PD on the error). The implemented controller differentiates θ, which makes the loop type 1, so it cannot track a parabola at all.` } : r;
            },
          },
          solution: () => [{ tex: `e_{ss} = \\frac{2A}{M_a} = \\frac{2A(J_s+J_p)}{k_{P_\\theta}};\\quad \\text{C.10 gains: } \\frac{${tex(2 * pr.parabA)}}{${tex(s().Ma)}} = ${tex(s().eParab)}\\,\\text{rad}` }, { html: 'Book: 1.53 with A/M<sub>a</sub> and the C.8 gains (M<sub>a</sub> at 22.3 dB, p. 299). Both treat the loop as P<sub>in</sub>C<sub>in</sub> with PD on the error. The repo\'s PID controller puts the derivative on θ, so its closed loop is type 1 (ramp error k<sub>D</sub>/k<sub>P</sub>) and the parabola error grows without bound.' },
            { code: `def e_ss(kP, kD):\n    Ma = kP / (P.Js + P.Jp)      # lim s^2 P_in C_in\n    return 2 * ${pr.parabA} / Ma          # θ_r = ${pr.parabA}t^2  ->  R = ${2 * pr.parabA}/s^3` }] },
        { id: 'b', title: '(b) % of d<sub>in</sub> below 0.1 rad/s in θ',
          html: 'Percent of an input disturbance at frequency w (rad/s) that shows up in θ, as a function of w and the inner-loop gains (dirty derivative σ).',
          code: {
            template: 'def din_pct(w, kP, kD, sigma):\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { w: { label: 'w', lo: 0.005, hi: 0.5 }, kP: PY.kPin, kD: PY.kDin, sigma: PY.sigma },
              items: [{ fn: 'din_pct', args: ['w', 'kP', 'kD', 'sigma'], truth: (p, a) => 100 / magAt(T.pid({ kP: a.kP, kD: a.kD, sigma: a.sigma }), a.w) }],
            }, code),
          },
          solution: () => [{ tex: `|C_{in}(j0.1)| = ${tex(s().Bdin)}\\,\\text{dB} \\Rightarrow ${tex(100 * s().gdin)}\\%\;\\text{(C.10 gains)}` }, { html: 'Book: 38 dB, 1.26% (C.8 gains).' },
            { code: 'def din_pct(w, kP, kD, sigma):\n    s = 1j * w\n    C_in = kP + kD * s / (sigma * s + 1)\n    return 100 / abs(C_in)      # |P/(1+PC)| ≈ 1/|C| where |PC| >> 1' }] },
        { id: 'p2', title: 'Outer loop: Bode plots of the plant and of the plant under PID control',
          html: 'Use <code>bode</code> with the C.10 gains. The workbench\'s Bode plot (<em>outer loop</em> view) shows P<sub>out</sub>C<sub>out</sub>, and P<sub>out</sub> once C.15(b) is done.' },
        { id: 'c', title: '(c) % of noise above 10 rad/s in φ',
          html: 'Percent of sensor noise at frequency w (rad/s) that shows up in φ, as a function of w and the outer-loop PID gains (dirty derivative σ).',
          code: {
            template: 'def noise_pct(w, kP, kI, kD, sigma):\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { w: { label: 'w', lo: 5, hi: 100 }, kP: PY.kPout, kI: PY.kIout, kD: PY.kDout, sigma: PY.sigma },
              items: [{ fn: 'noise_pct', args: ['w', 'kP', 'kI', 'kD', 'sigma'], truth: (p, a) => 100 * magAt(T.mul(pOut(p), T.pid({ kP: a.kP, kI: a.kI, kD: a.kD, sigma: a.sigma })), a.w) }],
            }, code),
          },
          solution: () => [{ tex: `|P_{out}C_{out}(j10)| = ${tex(-s().Bn)}\\,\\text{dB} \\Rightarrow ${tex(100 * s().gn)}\\%\;\\text{(C.10 gains)}` }, { html: 'Book: −8.4 dB, 38%. The part says "using PI control", but the loop is the PID of C.10.' },
            { code: 'def noise_pct(w, kP, kI, kD, sigma):\n    s = 1j * w\n    P_out = (P.b / P.Jp * s + P.k / P.Jp) / (s**2 + P.b / P.Jp * s + P.k / P.Jp)\n    C_out = kP + kI / s + kD * s / (sigma * s + 1)\n    return 100 * abs(P_out * C_out)   # |PC/(1+PC)| ≈ |PC| where |PC| << 1' }] },
      ]);
    },
  };

  // ------------------------------------------------------------- C.17 --
  CH.ch17 = {
    id: 'ch17', num: 17, tab: 'Ch 17', title: 'Stability margins', pages: 'pp. 318–322',
    linear: false,
    defaults(sys) { return pidDefaults(sys); },
    simDefaults(sys) { return sys.problems.ch17.sim; },
    controller: cascadeCtl,

    buildControls(parent, ctx) {
      gainControls(parent, ctx);
      const sec = section(parent, 'Margins and bandwidths', 'p. 304–306, p. 321');
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const l = loops(ctx);
        const show = WB.ui.shown(ctx, 'C:ch17:m');
        const row = WB.ui.metric;
        box.replaceChildren(...(show ? [
          row('inner PM', `${fmt(l.mgIn.pm, 3)}° at ω_co = ${fmt(l.mgIn.wc, 3)} rad/s`),
          row('inner GM', gmText(l.mgIn)),
          row('inner bandwidth', `${fmt(l.bwIn, 3)} rad/s`),
          row('outer PM', `${fmt(l.mgOut.pm, 3)}° at ω_co = ${fmt(l.mgOut.wc, 3)} rad/s`),
          row('outer GM', gmText(l.mgOut)),
          row('outer bandwidth', `${fmt(l.bwOut, 3)} rad/s`),
          row('separation ω_bw,in / ω_bw,out', `${fmt(l.bwIn / l.bwOut, 3)}×`),
        ] : [WB.ui.revealButton(ctx, 'C:ch17:m', 'Reveal the margins')]));
      });
    },

    bode(ctx) {
      const l = loops(ctx), inner = ctx.st.view === 'inner';
      const show = WB.ui.shown(ctx, 'C:ch17:m');
      const Lg = inner ? l.Lin : l.Lout, Tc = inner ? l.Tin : l.Tout, mg = inner ? l.mgIn : l.mgOut, bw = inner ? l.bwIn : l.bwOut;
      const marks = show ? marginMarks(mg) : [];
      if (show && isFinite(bw)) marks.push({ w: bw, label: `bandwidth ${fmt(bw, 3)}`, color: '--series-2' });
      return {
        title: inner ? 'Inner loop: P_in C_in and its closed loop' : 'Outer loop: P_out C_out and its closed loop', w: W,
        lines: [{ label: 'open loop PC', ...T.bode(Lg, W), color: '--series-1' }, { label: 'closed loop PC/(1+PC)', ...T.bode(Tc, W), color: '--series-2', width: 1.5 }],
        marks,
      };
    },
    splane(ctx) { const l = loops(ctx); return ctx.st.view === 'inner' ? clMarkers(ctx, l.Lin, 'P_in C_in', lib().shows(ctx, 'C.5/d')) : clMarkers(ctx, l.Lout, 'P_out C_out', lib().shows(ctx, 'C.5/c')); },

    math(ctx) {
      const l = loops(ctx);
      return [
        { title: 'Crossover and phase margin', page: 'p. 303–304',
          theory: '|PC(j\\omega_{co})| = 1,\\quad PM = \\angle PC(j\\omega_{co}) + 180^\\circ,\\quad GM = \\frac{1}{|PC(j\\omega_{180})|}',
          numbers: `\\text{inner: } PM = ${tex(l.mgIn.pm)}^\\circ \\text{ at } ${tex(l.mgIn.wc)},\\quad \\text{outer: } PM = ${tex(l.mgOut.pm)}^\\circ \\text{ at } ${tex(l.mgOut.wc)}`, spoiler: true },
        { title: 'Open vs. closed loop', page: 'p. 306–307',
          theory: 'T = \\frac{PC}{1+PC}:\\; |PC| \\gg 1 \\Rightarrow |T| \\approx 1,\\; |PC| \\ll 1 \\Rightarrow |T| \\approx |PC|',
          numbers: `\\omega_{bw,in} = ${tex(l.bwIn)},\\; \\omega_{bw,out} = ${tex(l.bwOut)},\\quad \\frac{\\omega_{bw,in}}{\\omega_{bw,out}} = ${tex(l.bwIn / l.bwOut)}`, spoiler: true,
          note: 'Bandwidth here is the highest frequency where |T| is still above −3 dB. The outer |T| dips below −3 dB near 0.08 rad/s (below the panel resonance at √(k/J_p) ≈ 0.32 rad/s) and comes back, so the first crossing (what control.bandwidth reports) would be misleading.' },
        { title: 'Successive loop closure, in frequency terms', page: 'p. 321',
          theory: '\\text{justified when } |T_{in}(j\\omega)| \\approx 1 \\text{ well past the outer crossover: } \\omega_{bw,in} \\gtrsim 5\\text{–}10\\,\\omega_{co,out}' },
      ];
    },

    buildProblem(parent, ctx) {
      const l = () => loops(ctx, pick(c10(ctx.sys, ctx.pModel)));
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch17, [
        { id: 'a', title: '(a) Inner loop', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
          html: 'Answers use the C.10 gains (nominal parameters), whatever the sliders are set to. ω<sub>bw</sub>: where |T| falls 3 dB below its DC value.',
          check: (v) => { const x = l(); return PD().checkNumbers(v, { pm: x.mgIn.pm, wc: x.mgIn.wc, bw: x.bwIn }, { pm: 'PM', wc: 'ωco', bw: 'ωbw' }); },
          solution: () => { const x = l(); return [{ tex: `PM = ${tex(x.mgIn.pm)}^\\circ \\text{ at } ${tex(x.mgIn.wc)},\\; GM = \\infty,\\; \\omega_{bw} = ${tex(x.bwIn)}` }, { html: 'Book: PM = 56.16°, crossover ≈ 7, bandwidth ≈ 11 rad/s (p. 321; made with the C.8 loops). The bandwidth is above crossover because the PM is below 90°.' }]; } },
        { id: 'b', title: '(b) Outer loop', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
          html: 'ω<sub>bw</sub>: the highest frequency where |T| is still above −3 dB (|T| dips near 0.08 rad/s and recovers).',
          check: (v) => { const x = l(); return PD().checkNumbers(v, { pm: x.mgOut.pm, wc: x.mgOut.wc, bw: x.bwOut }, { pm: 'PM', wc: 'ωco', bw: 'ωbw' }); },
          solution: () => { const x = l(); return [{ tex: `PM = ${tex(x.mgOut.pm)}^\\circ \\text{ at } ${tex(x.mgOut.wc)},\\; \\omega_{bw} = ${tex(x.bwOut)}` }, { html: 'Book: PM = 110.92°, bandwidth ≈ crossover ≈ 1 rad/s (Fig. 17-15).' }]; } },
        { id: 'c', title: '(c) Bandwidth separation', inputs: { r: 'ω<sub>bw,in</sub> / ω<sub>bw,out</sub>' },
          html: 'For the outer loop, use the highest frequency where |T| is still above −3 dB (|T| dips near 0.08 rad/s and recovers).',
          check: (v) => { const x = l(); return PD().checkNumbers(v, { r: x.bwIn / x.bwOut }, { r: 'ratio' }); },
          solution: () => { const x = l(); return [{ tex: `\\frac{${tex(x.bwIn)}}{${tex(x.bwOut)}} = ${tex(x.bwIn / x.bwOut)}` }, { html: 'Book: "close to a decade", so successive loop closure is justified.' }]; } },
      ]);
    },
  };

  // ------------------------------------------------------------- C.18 --
  // Compensator blocks (loopshape_tools.py).
  const blk = {
    lead: (b) => T.lead(b.M, b.w),
    lag: (b) => T.lag(b.z, b.M),
    lpf: (b) => T.lpf(b.p),
    int: (b) => T.pi(b.ki),
  };
  // Inner plant: τ → θ. Repo (loopShapingInner.py): 1/((Js+Jp)s²), no rate feedback.
  // Book text (p. 362): body only, with rate feedback −k_Dθ s/(σs+1).
  function innerPlant(p, st, g) {
    if (!st.inner.rate) return pIn(p);
    const s = g.sigma;
    return T.tf([s, 1], [s * p.Js, s * p.b + p.Js, s * p.k + p.b + g.kDth, p.k]);
  }
  // Outer plant with rate feedback −k_Dφ s/(σs+1), times the closed inner loop.
  // 'book' is the derivation on p. 366; 'repo' is loopShapingOuter.py.
  function outerRaw(p, st, g) {
    const s = g.sigma;
    if (st.model === 'repo') return T.tf([s, 1], [s * p.Jp, s * p.b + p.Jp, s * p.k + p.b + g.kDphi, p.k]);
    return T.tf([s * p.b, s * p.k + p.b, p.k], [s * p.Jp, s * p.b + p.Jp + p.b * g.kDphi, s * p.k + p.b + p.k * g.kDphi, p.k]);
  }
  function innerC(st) {
    const i = st.inner;
    let C = T.gain(i.k);
    if (i.lead.on) C = T.mul(C, blk.lead(i.lead));
    if (i.lpf.on) C = T.mul(C, blk.lpf(i.lpf));
    return C;
  }
  function outerC(st) {
    const o = st.outer;
    let C = T.gain(1);
    if (o.int.on) C = T.mul(C, blk.int(o.int));
    if (o.lead.on) C = T.mul(C, blk.lead(o.lead));
    if (o.lag.on) C = T.mul(C, blk.lag(o.lag));
    C = T.mul(C, T.gain(o.k));
    if (o.lpf1.on) C = T.mul(C, blk.lpf(o.lpf1));
    if (o.lpf2.on) C = T.mul(C, blk.lpf(o.lpf2));
    return C;
  }
  // Rate-feedback gains: C.10's (what the problem and the repo use) or the C.8
  // loops' (what the book's C.18 figures were made with; see ISSUES.md).
  const rateGains = (sys, p, st) => (st.kd === 'c8' ? c8fig(sys, p) : c10(sys, p));
  function presetRepo(p, g) {
    const st = { kd: 'c10', model: 'repo', inner: { rate: false, k: 1, lead: { on: true, w: 0.41, M: 15 }, lpf: { on: false, p: 8 } },
      outer: { k: 1, int: { on: true, ki: 0.1 }, lead: { on: true, w: 0.15, M: 120 }, lag: { on: true, z: 0.5, M: 60 }, lpf1: { on: true, p: 1.5 }, lpf2: { on: true, p: 1.8 }, pf: { on: true, p: 0.1 } } };
    // C_k = 1/|Plant·C_int·C_lead·C_lag(j0.15)| on the repo's outer plant (loopShapingOuter.py)
    const Pi = innerPlant(p, st, g), Ti = T.feedback(T.mul(Pi, innerC(st)));
    const Pl = T.mul(outerRaw(p, st, g), Ti);
    const C0 = T.mul(blk.int(st.outer.int), blk.lead(st.outer.lead), blk.lag(st.outer.lag));
    st.outer.k = 1 / magAt(T.mul(Pl, C0), st.outer.lead.w);
    return st;
  }
  function presetBook(p, g) {
    const st = presetRepo(p, g);
    st.kd = 'c8';
    st.inner = { rate: true, k: 45, lead: { on: false, w: 0.41, M: 15 }, lpf: { on: true, p: 8 } };
    st.outer.k = 0.0275;
    return st;
  }
  function presetNone() {
    return { kd: 'c10', model: 'book', inner: { rate: true, k: 1, lead: { on: false, w: 1, M: 10 }, lpf: { on: false, p: 20 } },
      outer: { k: 1, int: { on: false, ki: 0.1 }, lead: { on: false, w: 0.15, M: 10 }, lag: { on: false, z: 0.5, M: 10 }, lpf1: { on: false, p: 1.5 }, lpf2: { on: false, p: 1.8 }, pf: { on: false, p: 0.1 } } };
  }

  const tfFilter = T.repoFilter;   // ctrlLoopshape.transferFunction

  function loopshapeController(ctx) {
    const { sys, pModel, S } = ctx;
    const st = ctx.st, g = rateGains(sys, pModel, st), Ts = S.sim.Ts;
    const beta = (2 * g.sigma - Ts) / (2 * g.sigma + Ts);
    const Cin = tfFilter(innerC(st), Ts), Cout = tfFilter(outerC(st), Ts);
    const F = st.outer.pf.on ? tfFilter(blk.lpf(st.outer.pf), Ts) : { update: (u) => u };
    let thd = 0, phd = 0, thd1 = null, phd1 = null;
    return {
      update(r, x, y) {
        const th = y[0], ph = y[1];
        if (thd1 === null) { thd1 = th; phd1 = ph; }
        thd = beta * thd + (1 - beta) * ((th - thd1) / Ts); thd1 = th;
        phd = beta * phd + (1 - beta) * ((ph - phd1) / Ts); phd1 = ph;
        const rf = F.update(r);
        const eOut = rf - ph;
        const thr = -g.kDphi * phd + Cout.update(eOut);
        const eIn = thr - th;
        let tauU = Cin.update(eIn);
        if (st.inner.rate) tauU -= g.kDth * thd;
        return { u: tauU, thetaR: thr };   // unsaturated demand; the simulation clips it
      },
    };
  }

  CH.ch18 = {
    id: 'ch18', num: 18, tab: 'Ch 18', title: 'Loopshaping (both loops)', pages: 'pp. 361–374',
    linear: false,
    defaults(sys) { return { ...presetNone(), loop: 'inner', showT: true }; },
    simDefaults(sys) { return { ...sys.problems.ch18.sim, mismatch: sys.problems.ch18.mismatch }; },
    controller: loopshapeController,
    outputSeries(ctx, res, sc, oi) {
      if (oi !== 0 || !res.extras.thetaR) return [];
      return [{ label: 'θ_r (outer-loop output)', y: sc(res.extras.thetaR), color: '--series-2', dash: [4, 3], width: 1.5 }];
    },

    design(ctx) {
      const p = ctx.pModel, st = ctx.st, g = rateGains(ctx.sys, p, st), pr = ctx.sys.problems.ch18;
      const Pi = innerPlant(p, st, g), Ci = innerC(st), Li = T.mul(Pi, Ci), Ti = T.feedback(Li);
      const Po = T.mul(outerRaw(p, st, g), Ti), Co = outerC(st), Lo = T.mul(Po, Co);
      const F = st.outer.pf.on ? blk.lpf(st.outer.pf) : T.gain(1);
      const mgI = T.margins(Li), mgO = T.margins(Lo);
      const ip = pr.inner, op = pr.outer;
      const lowI = Math.min(...Array.from(W).filter((w) => w <= ip.wr).map((w) => magAt(Li, w)));
      const highI = Math.max(...Array.from(W).filter((w) => w >= ip.wn).map((w) => magAt(Li, w)));
      const dinO = Math.min(...Array.from(W).filter((w) => w <= op.wdin).map((w) => magAt(Lo, w) / magAt(Po, w)));
      const highO = Math.max(...Array.from(W).filter((w) => w >= op.wn).map((w) => magAt(Lo, w)));
      const typeO = T.tf(Lo.num, Lo.den).den.slice().reverse().findIndex((c) => Math.abs(c) > 1e-14);
      return {
        g, Pi, Ci, Li, Ti, Po, Co, Lo, F, mgI, mgO,
        inner: { track: lowI >= 1 / ip.gr * 0.999, noise: highI <= ip.gn * 1.001, pm: Math.abs(mgI.pm - ip.pm) <= 5, lowI, highI },
        outer: { type: typeO >= 1, din: dinO >= 1 / op.gdin * 0.999, noise: highO <= op.gn * 1.001, pm: Math.abs(mgO.pm - op.pm) <= 5, dinO, highO, typeO },
      };
    },

    buildControls(parent, ctx) {
      const st = ctx.st;
      const pre = section(parent, 'Start from', 'p. 362–369');
      const g0 = () => c10(ctx.sys, ctx.pModel);
      // The book's and the repo's compensators are worked answers to C.18(a) and (b):
      // Work mode offers them once both parts are solved.
      const presets = el('div', { class: 'btn-row' },
        el('button', { type: 'button', class: 'btn', text: 'Nothing', onclick: () => { Object.assign(st, presetNone()); ctx.update(); } }));
      if (lib().shows(ctx, 'C.18/a', 'C.18/b')) {
        presets.append(
          el('button', { type: 'button', class: 'btn', text: 'Book text', title: 'C_in = 45·8/(s+8) with rate feedback; C_out with k = 0.0275; C.8 rate gains and the listing\'s outer model, as in Figs. 18-29 to 18-35', onclick: () => { Object.assign(st, presetBook(ctx.pModel, g0())); ctx.update(); } }),
          el('button', { type: 'button', class: 'btn', text: 'Repo listings', title: 'The repo\'s inner and outer loop-shaping designs', onclick: () => { Object.assign(st, presetRepo(ctx.pModel, g0())); ctx.update(); } }));
      }
      pre.append(presets);
      if (ctx.S.mode === 'work') {
        pre.append(el('p', { class: 'muted small', text: 'In Work mode each loop\'s Bode plot, poles and spec readouts appear once you have derived its plant: (a) inner plant for C_in, (b) outer plant for C_out. The book and repo designs are offered here once (a) and (b) are solved.' }));
      }
      segmented(pre, {
        label: 'Rate-feedback gains k<sub>D<sub>θ</sub></sub>, k<sub>D<sub>φ</sub></sub>',
        options: [{ value: 'c10', label: 'C.10 (problem, repo)' }, { value: 'c8', label: 'C.8 loops (book figures)' }],
        ...bind(ctx, 'kd', () => st),
      });
      segmented(pre, {
        label: 'Edit and plot',
        options: [{ value: 'inner', label: 'inner loop C_in' }, { value: 'outer', label: 'outer loop C_out' }],
        ...bind(ctx, 'loop', () => st),
      });
      const onOff = (sec, obj) => WB.ui.onOff(sec, ctx, () => obj());
      const leadPhase = (sec, obj) => {
        const ptxt = el('p', { class: 'muted small' }); sec.append(ptxt);
        WB.ui.addRefresher(() => { const Mx = obj().M; ptxt.textContent = `max phase added: sin⁻¹((M−1)/(M+1)) = ${fmt(Math.asin((Mx - 1) / (Mx + 1)) * R2D, 3)}°`; });
      };
      if (st.loop === 'inner') {
        const s0 = section(parent, 'Inner plant', 'p. 362');
        segmented(s0, {
          label: 'Rate feedback τ = −k<sub>D<sub>θ</sub></sub>θ̇ + τ′',
          options: [{ value: true, label: 'on (book text)' }, { value: false, label: lib().shows(ctx, 'C.5/d') ? 'off: 1/((Js+Jp)s²) (repo)' : 'off (repo)' }],
          ...bind(ctx, 'rate', () => st.inner),
        });
        slider(s0, { label: 'k', min: 0.01, max: 1000, log: true, sig: 4, ...bind(ctx, 'k', () => st.inner) });
        const s1 = section(parent, 'Lead M(s + ω/√M)/(s + ω√M)', 'p. 328 · Eq. 18.2');
        onOff(s1, () => st.inner.lead);
        slider(s1, { label: 'ω<sub>lead</sub>', unit: 'rad/s', min: 0.01, max: 100, log: true, sig: 3, ...bind(ctx, 'w', () => st.inner.lead), disabled: () => !st.inner.lead.on });
        slider(s1, { label: 'M', min: 1.01, max: 200, log: true, sig: 3, ...bind(ctx, 'M', () => st.inner.lead), disabled: () => !st.inner.lead.on });
        leadPhase(s1, () => st.inner.lead);
        const s2 = section(parent, 'Low-pass p/(s + p)', 'p. 325');
        onOff(s2, () => st.inner.lpf);
        slider(s2, { label: 'p', unit: 'rad/s', min: 0.1, max: 1000, log: true, sig: 3, ...bind(ctx, 'p', () => st.inner.lpf), disabled: () => !st.inner.lpf.on });
      } else {
        const s0 = section(parent, 'Outer plant', 'p. 366');
        segmented(s0, {
          label: 'Model of θ_r′ → φ',
          options: [{ value: 'book', label: 'derived on p. 366' }, { value: 'repo', label: 'repo design' }],
          ...bind(ctx, 'model', () => st),
        });
        s0.append(el('p', { class: 'muted small', text: 'The listing\'s model has DC gain 1/k = 10 instead of 1; the book\'s figures were made with it. The simulation always uses the real satellite.' }));
        slider(s0, { label: 'k', min: 1e-4, max: 10, log: true, sig: 4, ...bind(ctx, 'k', () => st.outer) });
        const o = st.outer;
        const si = section(parent, 'Integral (s + k_I)/s', 'p. 362');
        onOff(si, () => o.int);
        slider(si, { label: 'k<sub>I</sub>', min: 0.001, max: 2, log: true, sig: 3, ...bind(ctx, 'ki', () => o.int), disabled: () => !o.int.on });
        const sl = section(parent, 'Lead M(s + ω/√M)/(s + ω√M)', 'p. 328 · Eq. 18.2');
        onOff(sl, () => o.lead);
        slider(sl, { label: 'ω<sub>lead</sub>', unit: 'rad/s', min: 0.005, max: 10, log: true, sig: 3, ...bind(ctx, 'w', () => o.lead), disabled: () => !o.lead.on });
        slider(sl, { label: 'M', min: 1.01, max: 300, log: true, sig: 3, ...bind(ctx, 'M', () => o.lead), disabled: () => !o.lead.on });
        leadPhase(sl, () => o.lead);
        const sg = section(parent, 'Lag (s + z)/(s + z/M)', 'p. 325 · Eq. 18.1');
        onOff(sg, () => o.lag);
        slider(sg, { label: 'z', unit: 'rad/s', min: 0.001, max: 10, log: true, sig: 3, ...bind(ctx, 'z', () => o.lag), disabled: () => !o.lag.on });
        slider(sg, { label: 'M', min: 1.01, max: 300, log: true, sig: 3, ...bind(ctx, 'M', () => o.lag), disabled: () => !o.lag.on });
        for (const key of ['lpf1', 'lpf2']) {
          const s = section(parent, `Low-pass ${key === 'lpf1' ? 1 : 2}: p/(s + p)`, 'p. 325');
          onOff(s, () => o[key]);
          slider(s, { label: 'p', unit: 'rad/s', min: 0.05, max: 100, log: true, sig: 3, ...bind(ctx, 'p', () => o[key]), disabled: () => !o[key].on });
        }
        const pf = section(parent, 'Prefilter F(s) = p/(s + p)', 'p. 336 · Eq. 18.5–18.7');
        onOff(pf, () => o.pf);
        slider(pf, { label: 'p', unit: 'rad/s', min: 0.01, max: 10, log: true, sig: 3, ...bind(ctx, 'p', () => o.pf), disabled: () => !o.pf.on });
        segmented(pf, { label: 'Bode: closed loop F·T', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'showT', () => st) });
      }
      const sp = section(parent, 'C.18 specs', 'p. 361–362');
      const box = el('div', { class: 'metrics' });
      sp.append(box);
      WB.ui.addRefresher(() => {
        const d = this.design(ctx);
        const row = WB.ui.specRow, metric = WB.ui.metric;
        // Each loop's readouts follow from its plant, so they wait for (a) / (b) in Work mode.
        const hold = (what) => metric(what, '— solve the plant part first');
        box.replaceChildren(...(this.plantShown(ctx, 'inner') ? [
          row('inner: |PC| ≥ 1/0.01 below 0.01 rad/s', d.inner.track, `${fmt(db(d.inner.lowI), 3)} dB`),
          row('inner: |PC| ≤ 0.01 above 20 rad/s', d.inner.noise, `${fmt(db(d.inner.highI), 3)} dB`),
          row('inner: PM ≈ 60°', d.inner.pm, `${fmt(d.mgI.pm, 3)}° at ${fmt(d.mgI.wc, 3)}`),
        ] : [hold('inner loop (C.18(a))')]), ...(this.plantShown(ctx, 'outer') ? [
          row('outer: type ≥ 1 (step error 0)', d.outer.type, `type ${d.outer.typeO}`),
          row('outer: |PC|/|P| ≥ 10 below 0.01 rad/s', d.outer.din, `${fmt(db(d.outer.dinO), 3)} dB`),
          row('outer: |PC| ≤ 10⁻⁴ above 10 rad/s', d.outer.noise, `${fmt(db(d.outer.highO), 3)} dB`),
          row('outer: PM ≈ 60°', d.outer.pm, `${fmt(d.mgO.pm, 3)}° at ${fmt(d.mgO.wc, 3)}`),
        ] : [hold('outer loop (C.18(b))')]));
      });
    },

    // The loop's Bode plot (with C = 1 it is the plant itself) answers the plant part
    // C.18(a1) / (b1), so Work mode draws each loop once its plant is derived.
    plantShown: (ctx, loop) => lib().shows(ctx, loop === 'inner' ? 'C.18/a1' : 'C.18/b1'),
    bode(ctx) {
      if (!this.plantShown(ctx, ctx.st.loop)) return null;
      const d = this.design(ctx), pr = ctx.sys.problems.ch18;
      if (ctx.st.loop === 'inner') {
        const ip = pr.inner;
        return { title: 'Inner loop: P_in and P_in·C_in', w: W, lines: [
          { label: 'P_in', ...T.bode(d.Pi, W), color: '--text-muted', dash: [5, 4], width: 1.5 },
          { label: 'P_in·C_in', ...T.bode(d.Li, W), color: '--series-1' },
        ], specs: [
          { w0: 1e-4, w1: ip.wr, db: db(1 / ip.gr), keep: 'above', color: '--critical', label: 'tracking spec' },
          { w0: ip.wn, w1: 1e4, db: db(ip.gn), keep: 'below', color: '--critical', label: 'noise spec' },
        ], marks: marginMarks(d.mgI) };
      }
      const op = pr.outer;
      const lines = [
        { label: 'P_out·T_in', ...T.bode(d.Po, W), color: '--text-muted', dash: [5, 4], width: 1.5 },
        { label: 'P·C_out', ...T.bode(d.Lo, W), color: '--series-1' },
      ];
      if (ctx.st.showT) lines.push({ label: 'closed loop F·T', mag: T.bode(T.mul(d.F, T.feedback(d.Lo)), W).mag, color: '--series-3', width: 1.5 });
      return { title: 'Outer loop: plant and P·C_out', w: W, lines, specs: [
        { w0: 1e-4, w1: op.wdin, db: db(magAt(d.Po, op.wdin) / op.gdin), keep: 'above', color: '--critical', label: 'd_in spec' },
        { w0: op.wn, w1: 1e4, db: db(op.gn), keep: 'below', color: '--critical', label: 'noise spec' },
      ], marks: marginMarks(d.mgO) };
    },
    splane(ctx) {
      if (!this.plantShown(ctx, ctx.st.loop)) return { markers: [] };
      const d = this.design(ctx);
      return ctx.st.loop === 'inner' ? clMarkers(ctx, d.Li, 'P_in C_in') : clMarkers(ctx, d.Lo, 'P C_out');
    },

    math(ctx) {
      const d = this.design(ctx), g = d.g, p = ctx.pModel;
      return [
        { title: 'Inner plant with rate feedback', page: 'p. 362',
          theory: '\\tau = -k_D\\frac{s}{\\sigma s + 1}\\Theta + \\tau\'',
          symbolic: 'P_{in} = \\frac{\\sigma s + 1}{\\sigma J_ss^3 + (\\sigma b + J_s)s^2 + (\\sigma k + b + k_D)s + k}\\;(\\text{from } \\Theta/\\tau = \\tfrac{1/J_s}{s^2 + (b/J_s)s + k/J_s})', answers: 'C.18/a1',
          note: `k_Dθ = ${fmt(g.kDth, 4)}, σ = ${fmt(g.sigma, 3)} (${ctx.st.kd === 'c8' ? 'C.8 loops' : 'C.10'}).` },
        { title: 'Outer plant with rate feedback', page: 'p. 366',
          theory: '\\theta_r = -k_D\\frac{s}{\\sigma s + 1}\\Phi + \\theta_r\'',
          symbolic: '\\frac{\\Phi}{\\Theta_r\'} = \\frac{\\sigma bs^2 + (\\sigma k + b)s + k}{\\sigma J_ps^3 + (\\sigma b + J_p + bk_D)s^2 + (\\sigma k + b + kk_D)s + k}', answers: 'C.18/b1',
          note: `k_Dφ = ${fmt(g.kDphi, 4)}. The book and the repo\'s design differ here; see ISSUES.md.` },
        { title: 'Your compensators', page: 'p. 362, p. 368',
          theory: `C_{in}(s) = ${T.texTf(d.Ci, 4)},\\quad C_{out}(s) = ${T.texTf(d.Co, 4)}` },
        // Each loop's margins follow from its plant (with C = 1 they are the plant's), so
        // Work mode shows them once C.18(a1) / (b1) is solved.
        { title: 'Margins', page: 'p. 304–306',
          theory: (this.plantShown(ctx, 'inner') ? `\\text{inner: } PM = ${tex(d.mgI.pm)}^\\circ \\text{ at } ${tex(d.mgI.wc)}` : '\\text{inner: after part (a), inner plant}') + ',\\quad ' + (this.plantShown(ctx, 'outer') ? `\\text{outer: } PM = ${tex(d.mgO.pm)}^\\circ \\text{ at } ${tex(d.mgO.wc)},\\; GM = ${d.mgO.crossings.length ? d.mgO.crossings.map((c) => tex(db(c.gm)) + '\\,\\text{dB}').join(',\\;') : '\\infty'}` : '\\text{outer: after part (b), outer plant}') },
        { title: 'Implementation', page: 'p. 335 · Eq. 18.3–18.4',
          theory: '\\theta_r = -k_{D_\\phi}\\dot{\\hat\\phi} + C_{out}\\big(F(\\phi_r) - \\phi\\big),\\quad \\tau = C_{in}(\\theta_r - \\theta)\\;[-\\,k_{D_\\theta}\\dot{\\hat\\theta}]' },
      ];
    },

    buildProblem(parent, ctx) {
      const cx = WB.py.cx;
      const args = {
        s: { label: 's', complex: true, re: [-2, 1], im: [0.5, 5] },
        sigma: { label: 'σ', lo: 0.01, hi: 0.2 },
      };
      // P/(1 + P·kD s/(σs + 1)): rate feedback around a plant P(s).
      const rateLoop = (P, s, kD, sg) => cx.div(P, cx.add(1, cx.mul(P, cx.div(cx.mul(kD, s), cx.poly([sg, 1], s)))));
      // The inner plant: C.5(d)'s 1/((Js+Jp)s²) (the problem), the book text's body-only
      // model, or the exact Θ/τ; any of them is accepted (see ISSUES.md).
      const innerModels = [
        { name: 'C.5(d)', P: (p, s) => cx.div(1, cx.mul(p.Js + p.Jp, cx.mul(s, s))) },
        { name: 'the book text', P: (p, s) => cx.div(1 / p.Js, cx.poly([1, p.b / p.Js, p.k / p.Js], s)) },
        { name: 'the exact Θ/τ (Eq. 5.6)', P: (p, s) => WB.studies.C.models.tfAt.thetaTau(p, s) },
      ];
      const checkInner = async (code) => {
        let first = null;
        for (const m of innerModels) {
          const r = await WB.py.check(ctx, {
            args: { ...args, kD: { label: 'kD', lo: 5, hi: 80 } },
            items: [{ fn: 'P_in', args: ['s', 'kD', 'sigma'], truth: (p, a) => rateLoop(m.P(p, a.s), a.s, a.kD, a.sigma) }],
          }, code);
          if (r.ok) return m.name === 'C.5(d)' ? r : { ...r, msg: `${r.msg} (With the inner plant of ${m.name}.)` };
          if (!first) first = r;
          if (/error|define|values? \(shape|has \d+ value/i.test(r.msg)) return r;
        }
        return first;
      };
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch18, [
        { id: 'a1', title: '(a) Inner plant with rate feedback',
          html: 'With the rate feedback τ = −k<sub>D</sub> s/(σs + 1) Θ + τ′ (the dirty derivative of θ), return the transfer function from τ′ to θ in terms of k<sub>D</sub> and σ, using P<sub>in</sub> from C.5. The check calls it at complex s.',
          code: { template: 'def P_in(s, kD, sigma):\n    # Theta(s)/tau_prime(s)\n    return ...\n', check: checkInner },
          solution: () => [
            { tex: '\\Theta = P(s)\\Big(\\tau\' - k_D\\frac{s}{\\sigma s + 1}\\Theta\\Big) \\;\\Rightarrow\\; \\frac{\\Theta}{\\tau\'} = \\frac{P(s)}{1 + k_D\\frac{s}{\\sigma s + 1}P(s)}' },
            { tex: 'P = \\frac{1}{(J_s + J_p)s^2}:\\quad \\frac{\\Theta}{\\tau\'} = \\frac{\\sigma s + 1}{\\sigma(J_s + J_p)s^3 + (J_s + J_p)s^2 + k_Ds}' },
            { code: 'def P_in(s, kD, sigma):\n    J = P.Js + P.Jp\n    return (sigma * s + 1) / (\n        sigma * J * s**3 + J * s**2 + kD * s)' },
            { html: 'Book: p. 362 uses Θ/τ = (1/J<sub>s</sub>)/(s² + (b/J<sub>s</sub>)s + k/J<sub>s</sub>), which gives (σs + 1)/(σJ<sub>s</sub>s³ + (σb + J<sub>s</sub>)s² + (σk + b + k<sub>D</sub>)s + k); that form is accepted too (see ISSUES.md).' },
          ] },
        { id: 'a', title: '(a) Inner-loop specs', after: 'a1',
          check: () => { const d = this.design(ctx), i = d.inner; return { ok: i.track && i.noise && i.pm, msg: `tracking ${i.track ? '✓' : '✗'}, noise ${i.noise ? '✓' : '✗'}, PM ${fmt(d.mgI.pm, 3)}° ${i.pm ? '✓' : '✗'}` }; },
          solution: () => [{ html: 'Book text (p. 362–363): proportional gain 45, then a low-pass at 8 rad/s, PM 76.9° (with the C.8 loops\' k<sub>D<sub>θ</sub></sub> = 38.9, not C.10\'s). The listing (Listing 18.7) is a different design: a lead at 0.41 rad/s with M = 15 on 1/(6s²), no rate feedback. Try both presets.' }] },
        { id: 'b1', title: '(b) Outer plant with rate feedback',
          html: 'With θ<sub>r</sub> = −k<sub>D</sub> s/(σs + 1) Φ + θ<sub>r</sub>′ and the inner loop taken as 1, return the transfer function from θ<sub>r</sub>′ to φ in terms of k<sub>D</sub> and σ, using P<sub>out</sub> from C.5.',
          code: {
            template: 'def P_out(s, kD, sigma):\n    # Phi(s)/Theta_r_prime(s)\n    return ...\n',
            check: (code) => WB.py.check(ctx, {
              args: { ...args, kD: { label: 'kD', lo: 1, hi: 20 } },
              items: [{ fn: 'P_out', args: ['s', 'kD', 'sigma'], truth: (p, a) => rateLoop(WB.studies.C.models.tfAt.phiTheta(p, a.s), a.s, a.kD, a.sigma) }],
            }, code),
          },
          solution: () => [
            { tex: '\\frac{\\Phi}{\\Theta_r\'} = \\frac{\\sigma bs^2 + (\\sigma k + b)s + k}{\\sigma J_ps^3 + (\\sigma b + J_p + bk_D)s^2 + (\\sigma k + b + kk_D)s + k}' },
            { code: 'def P_out(s, kD, sigma):\n    Jp, b, k = P.Jp, P.b, P.k\n    num = (b * s + k) * (sigma * s + 1)\n    den = ((Jp * s**2 + b * s + k)\n           * (sigma * s + 1)\n           + kD * s * (b * s + k))\n    return num / den' },
            { html: 'Book: p. 366. The repo\'s outer design uses a different model (see ISSUES.md).' },
          ] },
        { id: 'b', title: '(b) Outer-loop specs', after: 'b1',
          check: () => { const d = this.design(ctx), o = d.outer; return { ok: o.type && o.din && o.noise && o.pm, msg: `type ${o.type ? '✓' : '✗'}, d_in ${o.din ? '✓' : '✗'}, noise ${o.noise ? '✓' : '✗'}, PM ${fmt(d.mgO.pm, 3)}° ${o.pm ? '✓' : '✗'}` }; },
          solution: () => [{ html: 'Book (p. 367–368): integrator k<sub>I</sub> = 0.1, lead at 0.15 rad/s with M = 120, lag (s + 0.5)/(s + 0.0083), gain 0.0275, low-pass filters at 1.5 and 1.8 rad/s, prefilter 0.1/(s + 0.1). Its PM 51.75° at 0.15 rad/s (Fig. 18-34) is for the listing\'s outer model with the book-text inner loop and the C.8 rate gains (the <em>Book text</em> preset reproduces it). On the correctly derived plant the same C<sub>out</sub> has PM ≈ 83°.' }] },
      ]);
    },
  };

  WB.studies.C.freq = { pIn, pOut, cIn, cOut, c10, c8fig, loops, bandwidth, tfFilter, innerC, outerC, presetRepo, presetBook, loopshapeController };
})();
