// D.15-D.18: frequency response, frequency-domain specs, stability margins and
// loopshaping for P(s) = (1/m) / (s^2 + (b/m) s + k/m), a lightly damped
// second-order plant (zeta ≈ 0.065, resonance at sqrt(k/m) ≈ 0.775 rad/s).
//
// D.16 and D.17 are answered for whatever PID gains are in the sliders. They start
// from placeholder gains (not the D.10 answer); "Load my D.10 gains" copies the
// Work-mode gains from the D.10 tab.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const { tex, texPole, fmt } = M;
  const lib = WB.studies.D.lib;
  const ans = WB.systems.D.answers;
  const CH = WB.studies.D.chapters;
  const PID = () => WB.studies.D.pid;
  const W = T.logspace(-3, 4, 700);
  const { db, mag: absAt } = T;
  const gmText = (mg) => WB.freq.gmText(mg);

  const plantTf = (ctx) => T.tf([ctx.model.b0], [1, ctx.model.a1, ctx.model.a0]);
  // C_PID with dirty derivative (p. 313)
  const pidTf = (st) => T.pid(st);

  // Frequency-domain answers for a PID loop (D.16) and its margins (D.17).
  function specs(ctx, st = ctx.st) {
    const P = plantTf(ctx), C = pidTf(st), Lg = T.mul(P, C);
    const wdin = st.wdin, wno = st.wno;
    const Sdin = T.tf(L.conv(P.num, C.den), L.polyAdd(L.conv(P.den, C.den), L.conv(P.num, C.num)));  // P/(1+PC)
    return {
      P, C, Lg,
      Mv: st.kI / ctx.pModel.k, ramp: st.kI > 0 ? ctx.pModel.k / st.kI : Infinity,
      Bdin: db(absAt(C, wdin)), gdin: 1 / absAt(C, wdin),                // book: |PC| / |P| = |C| (Eq. 16.8-16.9)
      gdinExact: absAt(Sdin, wdin),
      gn: absAt(Lg, wno), gnExact: absAt(T.feedback(Lg), wno),             // book: |PC| (Eq. 16.6)
    };
  }
  // Margins with every gain crossover. The lightly damped plant can make |PC|
  // cross 0 dB several times; like python-control's stability_margins, report the
  // crossing whose phase margin is smallest in magnitude (T.margins alone reports
  // the first crossing). A crossing where the phase is above 0° gives PM < −180°
  // wrapped to (−180°, 0°); python-control reports it the same way.
  function margins(Lg) {
    const mg = T.margins(Lg, -4, 5, 6000);
    if (mg.gcs.length < 2) return mg;
    const worst = mg.gcs.reduce((a, c) => (Math.abs(c.pm) < Math.abs(a.pm) ? c : a));
    return { ...mg, pm: worst.pm, wc: worst.w };
  }

  // −3 dB crossings of |T|, refined in log ω. `first` is what python-control's
  // bandwidth() returns; `last` is the final roll-off. They differ when |T| dips
  // below −3 dB and recovers (the D.10 PID zeros put a notch in |C|; ISSUES.md).
  function bandwidth(Tc) {
    const cross = T.crossDown(Tc, W, Math.SQRT1_2);
    return { first: cross.length ? cross[0] : NaN, last: cross.length ? cross[cross.length - 1] : NaN, n: cross.length };
  }

  // The PID loop with the slider gains, or with fixed gains st ({kP, kI, kD, sigma}).
  function loop(ctx, st = ctx.st) {
    const Lg = T.mul(plantTf(ctx), pidTf(st)), Tc = T.feedback(Lg);
    const { mag } = T.bode(Tc, W);
    const bw = bandwidth(Tc);
    return { Lg, Tc, mg: margins(Lg), bw: bw.first, bwLast: bw.last, bwN: bw.n, peak: db(Math.max(...mag)) };
  }

  function pidControls(parent, ctx) {
    const sec = section(parent, 'C_PID (D.10 gains)', 'p. 161, p. 313');
    lib.gainSlider(sec, ctx, 'kP', 'k<sub>P</sub>', 20);
    lib.gainSlider(sec, ctx, 'kI', 'k<sub>I</sub>', 10);
    lib.gainSlider(sec, ctx, 'kD', 'k<sub>D</sub>', 30);
    slider(sec, { label: 'σ', unit: 's', min: 0.005, max: 0.3, step: 0.001, sig: 3, ...bind(ctx, 'sigma') });
    const row = el('div', { class: 'btn-row' },
      el('button', { type: 'button', class: 'btn btn-quiet', text: 'Load my D.10 gains', title: 'Copy the Work-mode kP, kI, kD from the D.10 tab', onclick: () => {
        const w = ctx.S.ch.ch10;
        if (w) Object.assign(ctx.st, { kP: w.kP, kI: w.kI, kD: w.kD, sigma: w.sigma });
        ctx.update();
      } }));
    if (ctx.S.mode === 'explore') {
      row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reference D.10 design', title: 't_r = 2 s, ζ = 0.7, k_I = 0.75, σ = 0.05', onclick: () => { Object.assign(ctx.st, refPid(ctx)); ctx.update(); } }));
    }
    sec.append(row);
    return sec;
  }
  // The reference D.10 PID: the D.8(a) design (t_r = 2 s, ζ = 0.7) with the D.10
  // solution's k_I and σ. D has no book listing; D.17's answers use these fixed gains.
  function refPid(ctx) {
    const pr = ctx.sys.problems.ch10;
    const d = ans.spec(ctx.pModel, pr.tr, pr.zeta);
    return { kP: d.kP, kD: d.kD, kI: pr.kiRef, sigma: pr.sigma };
  }
  // Placeholder gains (stable, not the D.10 answer).
  const pidDefaults = () => ({ kP: 2, kI: 0.5, kD: 4, sigma: 0.05, comp: 'eq', deriv: 'dirty', antiwindup: 'gate', vbar: 0.05 });
  const pidGains = (ctx) => ({ kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD });

  function clMarkers(ctx, Lg) {
    const poles = L.roots(L.polyAdd(Lg.den, Lg.num));
    // The plant poles answer D.7(a): hidden in Work mode until it is solved.
    const mk = lib.showOl(ctx) ? L.roots(plantTf(ctx).den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of P ${i + 1}` })) : [];
    poles.forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole ${i + 1}`, noFit: Math.hypot(p.re, p.im) > 12 }));
    return { markers: mk, fitR: 3 };
  }
  const marginMarks = (mg) => WB.freq.marginMarks(mg);

  // ---------------------------------------------------------------- D.15 --
  CH.ch15 = {
    id: 'ch15', num: 15, tab: 'Ch 15', title: 'Frequency response', pages: 'pp. 261–282, p. 382',
    openLoop: true, metrics: false,
    defaults() { return { comp: 'none', w0: 0.5, A: 1, asym: true, inp: { shape: 'sine', amp: 1, freq: 0.5 / (2 * Math.PI) } }; },
    simDefaults(sys) { return sys.problems.ch15.sim; },
    controller(ctx, o) {
      ctx.st.inp = { shape: 'sine', amp: ctx.st.A, freq: ctx.st.w0 / (2 * Math.PI), t0: 0 };
      return WB.studies.D.models.openLoop(ctx, o);
    },
    linearSim(ctx, c) { return lib.linearSim(ctx, c, (cx, o) => this.controller(cx, o)); },
    linearLabel: 'linear model (with transient)',
    // D.15 is drawn by hand: in Work mode the Bode plot, the predicted sinusoid and
    // the plant poles appear once the student says it is done.
    drawn: (ctx) => lib.shows(ctx, 'D.15/a'),
    outputSeries(ctx, res, sc) {
      if (!this.drawn(ctx)) return [];
      const g = T.at(plantTf(ctx), ctx.st.w0), mag = L.C.abs(g), ph = L.C.arg(g);
      return [{ label: 'A|P(jω₀)| sin(ω₀t + ∠P)', y: sc(Array.from(res.t, (t) => ctx.st.A * mag * Math.sin(ctx.st.w0 * t + ph))), color: '--series-3', dash: [2, 3], width: 2 }];
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Sinusoidal input F̃ = A sin(ω₀t)', 'p. 264 · Eq. 15.4');
      slider(sec, { label: 'ω<sub>0</sub>', unit: 'rad/s', min: 0.05, max: 20, log: true, sig: 3, ...bind(ctx, 'w0') });
      slider(sec, { label: 'A', unit: 'N', min: 0, max: 6, step: 0.01, sig: 3, ...bind(ctx, 'A') });
      segmented(sec, { label: 'Straight-line approximation', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'asym') });
      lib.note(sec, 'After the transient (it decays slowly for this plant) z is a sinusoid with gain |P(jω₀)| and phase ∠P(jω₀). Sweep ω₀ and watch the amplitude. In Work mode the Bode plot appears once you have drawn yours (problem part a).');
    },
    bode(ctx) {
      if (!this.drawn(ctx)) return null;
      const P = plantTf(ctx), { mag, phase } = T.bode(P, W);
      const lines = [{ label: 'P(jω)', mag, phase, color: '--series-1' }];
      const b = ans.bode(ctx.pModel);
      if (ctx.st.asym) {
        const am = W.map((w) => (w < b.wn ? b.dc : b.dc * (b.wn / w) ** 2));
        const ap = W.map((w) => (w < b.wn / 10 ? 0 : w > 10 * b.wn ? -180 : -90 * (Math.log10(w / b.wn) + 1)));
        lines.push({ label: 'straight-line approx.', mag: am, phase: ap, color: '--text-muted', dash: [5, 4], width: 1.5 });
      }
      const g = T.at(P, ctx.st.w0);
      return { title: 'Bode plot of P(s)', w: W, lines, marks: [{ w: ctx.st.w0, label: `ω₀: ${fmt(db(L.C.abs(g)), 3)} dB, ${fmt(L.C.arg(g) * 180 / Math.PI, 3)}°`, color: '--series-3' }] };
    },
    splane(ctx) {
      if (!this.drawn(ctx)) return { markers: [] };
      return { markers: L.roots(plantTf(ctx).den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of P ${i + 1}` })) };
    },
    math(ctx) {
      const b = ans.bode(ctx.pModel);
      return [
        { title: 'Frequency response', page: 'p. 264 · Eq. 15.4',
          theory: 'u = A\\sin\\omega_0 t \\;\\Rightarrow\\; y_{ss} = A|P(j\\omega_0)|\\sin\\big(\\omega_0 t + \\angle P(j\\omega_0)\\big)' },
        { title: 'Bode canonical form', page: 'p. 266 · Eq. 15.5–15.7',
          theory: 'P(j\\omega) = K\\,\\frac{\\prod (1 + j\\omega/z_i)}{(j\\omega)^q \\prod (1 + j\\omega/p_i)}:\\quad 20\\log|P| = 20\\log K + \\textstyle\\sum 20\\log|1 + j\\omega/z_i| - 20q\\log\\omega - \\sum 20\\log|1 + j\\omega/p_i|' },
        { title: 'Real pole', page: 'p. 269–270 · Eq. 15.9',
          theory: '\\frac{p}{s + p}:\\; -20\\text{ dB/dec above } p,\\; \\angle = -\\tan^{-1}(\\omega/p) \\text{ from } 0^\\circ \\text{ at } p/10 \\text{ to } -90^\\circ \\text{ at } 10p' },
        { title: 'Complex pole pair', page: 'p. 270–272 · Fig. 15-9',
          theory: '\\frac{1}{\\left(1 - \\frac{\\omega^2}{\\omega_n^2}\\right) + j2\\zeta\\frac{\\omega}{\\omega_n}}:\\quad \\omega \\ll \\omega_n: 0\\text{ dB}, 0^\\circ;\\quad \\omega \\gg \\omega_n: -40\\text{ dB/dec}, -180^\\circ;\\quad \\omega = \\omega_n: -20\\log_{10}|2\\zeta|,\\; -90^\\circ' },
        { title: 'Bode form of the mass-spring-damper', page: 'p. 266, p. 270–272', answers: 'D.15/a',
          theory: 'P(j\\omega) = \\frac{1}{k}\\,\\frac{1}{\\left(1 - \\frac{\\omega^2}{\\omega_n^2}\\right) + j2\\zeta\\frac{\\omega}{\\omega_n}},\\quad \\omega_n = \\sqrt{\\frac km},\\quad \\zeta = \\frac{b}{2\\sqrt{km}}',
          numbers: `K = ${tex(b.dc)}\\;(${tex(b.dcDb)}\\,\\text{dB}),\\quad \\omega_n = ${tex(b.wn)},\\quad \\zeta = ${tex(b.zeta)},\\quad |P(j\\omega_n)| = ${tex(b.peakDb)}\\,\\text{dB}`,
          note: 'ζ is small, so the resonant peak is tall and narrow, and the straight-line sketch misses it badly.' },
      ];
    },
    buildProblem(parent, ctx) {
      const b = () => ans.bode(ctx.pModel);
      const magDb = (w) => db(absAt(plantTf(ctx), w));
      lib.panel(parent, ctx, ctx.sys.problems.ch15, [
        {
          id: 'a', title: '(a) Draw the Bode plot by hand (F̃ to z̃)',
          html: 'On paper: put P(s) in Bode canonical form and sketch the straight-line magnitude and phase. When you are done, click the button to see part (b) and the workbench\'s Bode plot to compare against.',
          done: 'I\'ve drawn it: next',
          solution: () => [{ tex: `P(j\\omega) = \\frac{${tex(b().dc)}}{1 - (\\omega/${tex(b().wn)})^2 + j\\,2(${tex(b().zeta)})\\,\\omega/${tex(b().wn)}}:\\; ${tex(b().dcDb)}\\,\\text{dB flat, then } -40\\,\\text{dB/dec above } \\omega_n;\\; 0^\\circ \\to -180^\\circ` }],
        },
        {
          id: 'b', title: '(b) Compare with the bode command', after: 'a',
          html: 'Read these off the bode plot (your Python or the plot here). The resonance peak is what the straight-line sketch misses.',
          inputs: { m1: '|P(j0.1)| [dB]', m2: '|P(j1)| [dB]', m3: '|P(j10)| [dB]', peak: 'resonance peak [dB]' },
          check: (v) => lib.check(v, { m1: magDb(0.1), m2: magDb(1), m3: magDb(10), peak: b().peakDb }, {}),
          solution: () => [
            { tex: `${tex(magDb(0.1))},\\; ${tex(magDb(1))},\\; ${tex(magDb(10))}\\;\\text{dB}` },
            { tex: `\\zeta = \\frac{b}{2\\sqrt{km}} = ${tex(b().zeta)},\\quad |P(j\\omega_n)| = \\frac{1}{k\\,2\\zeta} = \\frac{1}{b\\,\\omega_n} = ${tex(b().peak)} = ${tex(b().peakDb)}\\,\\text{dB}` },
            { html: 'The true maximum (at ω<sub>n</sub>√(1 − 2ζ²)) is 0.02 dB higher; either is accepted. Checked against python-control.' },
          ],
        },
      ]);
    },
  };

  // D.16(b), (c) are functions of the frequency and the C_PID gains, checked at random
  // arguments against the book's approximation or the exact closed-loop value.
  // truth(|P|, {abs: |C|}, {din: |P/(1+PC)|, L: |PC|, T: |PC/(1+PC)|}) gives the percentage.
  const W_DIN = { label: 'w', lo: 0.005, hi: 0.2 }, W_NOISE = { label: 'w', lo: 50, hi: 2000 };
  function freqPy(ctx, fn, w, truths, bookRule) {
    const C = WB.py.cx;
    const spec = (truth) => ({
      args: { w, ...lib.gainArgs, sigma: { label: 'σ', lo: 0.01, hi: 0.1 } },
      items: [{ fn, args: ['w', 'kP', 'kI', 'kD', 'sigma'], truth: (p, a) => {
        const t = ans.tf(p), s = { re: 0, im: a.w };
        const Pj = C.div(t.b0, C.poly([1, t.a1, t.a0], s));
        const Cj = C.add(C.add(a.kP, C.div(a.kI, s)), C.div(C.mul(a.kD, s), C.add(C.mul(a.sigma, s), 1)));
        const Lj = C.mul(Pj, Cj), onePlus = C.add(1, Lj);
        const abs = (z) => Math.hypot(z.re, z.im);
        return truth(abs(Pj), { abs: abs(Cj) }, { din: abs(C.div(Pj, onePlus)), L: abs(Lj), T: abs(C.div(Lj, onePlus)) });
      } }],
    });
    return lib.pyEither(ctx, truths.map(spec), [`(${bookRule})`, '(exact closed-loop value)']);
  }
  const PY_LOOP = 'def loop(w, kP, kI, kD, sigma):\n    s = 1j * w\n    Pj = (1 / P.m) / (s**2 + P.b / P.m * s + P.k / P.m)\n    Cj = kP + kI / s + kD * s / (sigma * s + 1)\n    return Pj, Cj\n';

  // ---------------------------------------------------------------- D.16 --
  CH.ch16 = {
    id: 'ch16', num: 16, tab: 'Ch 16', title: 'Frequency-domain specs', pages: 'pp. 283–301, pp. 382–383',
    defaults(sys) { const p = sys.problems.ch16; return { ...pidDefaults(), wdin: p.wdin, wno: p.wno }; },
    simDefaults(sys) { return sys.problems.ch16.sim; },
    gains: pidGains,
    controller: (ctx, o) => PID().makePID(ctx, o),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, PID().makePID),

    buildControls(parent, ctx) {
      pidControls(parent, ctx);
      const sec = section(parent, 'Spec frequencies', 'p. 285–291');
      slider(sec, { label: 'ω<sub>d,in</sub>', unit: 'rad/s', min: 0.001, max: 1, log: true, sig: 3, ...bind(ctx, 'wdin') });
      slider(sec, { label: 'ω<sub>no</sub>', unit: 'rad/s', min: 10, max: 5000, log: true, sig: 3, ...bind(ctx, 'wno') });
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const s = specs(ctx);
        const row = WB.ui.metric;
        const show = WB.ui.shown(ctx, 'D:ch16:specs') || ['a', 'b', 'c'].every((k) => ctx.app.isSolved(`D.16/${k}`));
        box.replaceChildren(...(show ? [
          row('unit-ramp error k/k_I', `${fmt(s.ramp, 3)} m`),
          row('d_in below ω_d,in: 1/|C|', `${fmt(-s.Bdin, 3)} dB → ${fmt(100 * s.gdin, 3)} %`),
          row('exact |P/(1+PC)| at ω_d,in', `${fmt(100 * s.gdinExact, 3)} %`),
          row('noise above ω_no: |PC|', `${fmt(db(s.gn), 3)} dB → ${fmt(100 * s.gn, 3)} %`),
        ] : [WB.ui.revealButton(ctx, 'D:ch16:specs', 'Reveal the spec readouts')]));
      });
    },

    // The plot part asks for these curves: in Work mode they appear once it is done.
    bode(ctx) {
      if (!lib.shows(ctx, 'D.16/plot')) return null;
      const s = specs(ctx), st = ctx.st;
      return {
        title: 'Bode: plant and loop gain', w: W,
        lines: [{ label: 'P(jω)', ...T.bode(s.P, W), color: '--text-muted', width: 1.5 }, { label: 'P(jω)C(jω)', ...T.bode(s.Lg, W), color: '--series-1' }],
        marks: [{ w: st.wdin, label: 'ω_d,in', color: '--series-3' }, { w: st.wno, label: 'ω_no', color: '--series-3' }],
      };
    },
    splane(ctx) { return clMarkers(ctx, specs(ctx).Lg); },

    math(ctx) {
      const s = specs(ctx);
      return [
        { title: 'Error with every input', page: 'p. 284 · Eq. 16.2–16.3',
          theory: 'E = \\frac{1}{1+PC}R + \\frac{PC}{1+PC}N,\\quad + \\frac{1}{1+PC}D_{out} + \\frac{P}{1+PC}D_{in}' },
        { title: 'Type 1 from the Bode plot', page: 'p. 294 · Eq. 16.11',
          theory: 'M_v = \\lim_{\\omega\\to 0}|j\\omega\\,P(j\\omega)C(j\\omega)|,\\quad e_{ss} = \\frac{A}{M_v} \\text{ for a ramp of slope } A',
          numbers: `M_v = ${tex(s.Mv)} \\Rightarrow e_{ramp} = ${tex(s.ramp)}\\,\\text{m}`, answers: 'D.16/a' },
        { title: 'M_v of the mass-spring-damper under PID', page: 'p. 294 · Eq. 16.11', answers: 'D.16/a',
          theory: 'M_v = \\lim_{s\\to0} s\\,\\frac{1/m}{s^2 + \\frac bm s + \\frac km}\\,\\frac{k_I}{s} = \\frac{k_I}{k}' },
        { title: 'Input disturbance', page: 'p. 290 · Eq. 16.8, p. 291 · Eq. 16.9',
          theory: '20\\log|PC| - 20\\log|P| = 20\\log|C| \\ge B_{d_{in}} \\text{ for } \\omega \\le \\omega_{d_{in}},\\quad \\gamma_{d_{in}} = 10^{-B_{d_{in}}/20}',
          numbers: `|C(j\\omega_{d,in})| = ${tex(s.Bdin)}\\,\\text{dB} \\Rightarrow \\gamma_{d_{in}} = ${tex(s.gdin)}\\quad(\\text{exact } |P/(1+PC)| = ${tex(s.gdinExact)})`, answers: 'D.16/b',
          note: ctx.S.mode !== 'explore' ? 'The book\'s rule assumes |PC| ≫ 1 at ω_d,in. Check that assumption for your gains.' : `The book's rule assumes |PC| ≫ 1 at ω_d,in. Here |PC(j${fmt(ctx.st.wdin, 3)})| = ${fmt(absAt(s.Lg, ctx.st.wdin), 3)}, so the exact value differs from 1/|C|.` },
        { title: 'Noise', page: 'p. 287 · Eq. 16.6',
          theory: '20\\log|PC| \\le 20\\log\\gamma_n \\text{ for } \\omega \\ge \\omega_{no}',
          numbers: `|PC(j\\omega_{no})| = ${tex(db(s.gn))}\\,\\text{dB} \\Rightarrow \\gamma_n = ${tex(s.gn)}`, answers: 'D.16/c' },
        { title: 'C_PID with dirty derivative', page: 'p. 313',
          theory: 'C(s) = \\frac{(k_D + \\sigma k_P)s^2 + (k_P + \\sigma k_I)s + k_I}{s(\\sigma s + 1)}',
          numbers: `C(s) = ${T.texTf(s.C)}` },
      ];
    },

    buildProblem(parent, ctx) {
      const s = () => specs(ctx);
      lib.panel(parent, ctx, ctx.sys.problems.ch16, [
        { id: 'plot', title: 'Bode plots of the plant and of the plant under PID',
          html: 'With <code>bode</code> in your code: P(s), and P(s)C(s) with the D.10 gains and the dirty derivative in C (p. 313). When you have them, click the button: the Bode panel here then draws both for the gains in the sliders (<em>Load my D.10 gains</em> copies your Work-mode D.10 gains).',
          done: 'I\'ve plotted them' },
        { id: 'a', title: '(a) Tracking error to a unit ramp under PID',
          html: 'Steady-state error (m) as a function of the PID gains. The check calls it at random gains.',
          code: lib.pyPart(ctx, {
            args: lib.gainArgs,
            items: [{ fn: 'e_ss', args: ['kP', 'kI', 'kD'], truth: (p, a) => p.k / a.kI }],
          }, 'def e_ss(kP, kI, kD):\n    return ...\n'),
          solution: () => [{ tex: `\\text{type 1}:\; e_{ss} = \\frac{1}{M_v} = \\frac{k}{k_I} = ${tex(s().ramp)}\\,\\text{m}\;\\text{(current gains)}` }, { html: 'The dirty derivative and k<sub>D</sub> do not enter: only the integrator survives as s → 0.' },
            { code: 'def e_ss(kP, kI, kD):\n    return P.k / kI    # 1/M_v, M_v = lim s P C = kI/k' }] },
        { id: 'b', title: '(b) % of d<sub>in</sub> below 0.1 rad/s that shows up in z',
          html: 'Percent of an input disturbance at frequency w (rad/s) that shows up in z, as a function of w and the C<sub>PID</sub> gains (dirty derivative σ). The book\'s approximation and the exact closed-loop value are both accepted. ω<sub>d,in</sub> = 0.1 is the book\'s case.',
          code: { template: 'def din_pct(w, kP, kI, kD, sigma):\n    return ...\n', ...freqPy(ctx, 'din_pct', W_DIN, [(Pj, Cj) => 100 / Cj.abs, (Pj, Cj, S) => 100 * S.din], 'the book\'s 1/|C(jω)| rule') },
          solution: () => [{ tex: `\\gamma_{d_{in}} = \\frac{1}{|C(j0.1)|} = ${tex(100 * s().gdin)}\\%\\quad(\\text{exact: } ${tex(100 * s().gdinExact)}\\%)\;\\text{(current gains)}` }, { html: '|C| falls with ω below the PID zeros (k<sub>I</sub>/ω dominates), so the worst case on ω ≤ 0.1 is at 0.1 rad/s.' },
            { code: `${PY_LOOP}\ndef din_pct(w, kP, kI, kD, sigma):\n    Pj, Cj = loop(w, kP, kI, kD, sigma)\n    return 100 / abs(Cj)    # exact: 100 * abs(Pj / (1 + Pj * Cj))` }] },
        { id: 'c', title: '(c) % of noise above 100 rad/s that shows up in z',
          html: 'Percent of sensor noise at frequency w (rad/s) that shows up in z, as a function of w and the gains. The book\'s approximation and the exact value are both accepted.',
          code: { template: 'def noise_pct(w, kP, kI, kD, sigma):\n    return ...\n', ...freqPy(ctx, 'noise_pct', W_NOISE, [(Pj, Cj, S) => 100 * S.L, (Pj, Cj, S) => 100 * S.T], 'the book\'s |PC(jω)| rule') },
          solution: () => [{ tex: `\\gamma_n = |PC(j100)| = ${tex(db(s().gn))}\\,\\text{dB} = ${tex(100 * s().gn)}\\%\;\\text{(current gains)}` }, { html: 'At high frequency C → (k<sub>D</sub> + σk<sub>P</sub>)/σ (the dirty derivative caps it) and P ≈ 1/(mω²).' },
            { code: `${PY_LOOP}\ndef noise_pct(w, kP, kI, kD, sigma):\n    Pj, Cj = loop(w, kP, kI, kD, sigma)\n    return 100 * abs(Pj * Cj)    # exact: 100 * abs(Pj * Cj / (1 + Pj * Cj))` }] },
      ]);
    },
  };

  // ---------------------------------------------------------------- D.17 --
  CH.ch17 = {
    id: 'ch17', num: 17, tab: 'Ch 17', title: 'Stability margins', pages: 'pp. 303–322, p. 383',
    defaults() { return pidDefaults(); },
    simDefaults(sys) { return sys.problems.ch17.sim; },
    gains: pidGains,
    controller: (ctx, o) => PID().makePID(ctx, o),
    linearSim: (ctx, c) => lib.linearSim(ctx, c, PID().makePID),

    buildControls(parent, ctx) {
      pidControls(parent, ctx);
      const sec = section(parent, 'Margins', 'p. 303–306');
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const l = loop(ctx);
        const row = WB.ui.metric;
        box.replaceChildren(...(CH.ch17.marginsShown(ctx) ? [
          row('phase margin (smallest)', `${fmt(l.mg.pm, 3)}° at ω_co = ${fmt(l.mg.wc, 3)} rad/s`),
          ...(l.mg.gcs.length > 1 ? [row('all gain crossovers', l.mg.gcs.map((c) => `${fmt(c.pm, 3)}° at ${fmt(c.w, 3)}`).join(', '))] : []),
          row('gain margin(s)', gmText(l.mg)),
          row('closed-loop bandwidth (first −3 dB)', `${fmt(l.bw, 3)} rad/s`),
          row('final −3 dB roll-off of |T|', `${fmt(l.bwLast, 3)} rad/s`),
          row('closed-loop peak |T|', `${fmt(l.peak, 3)} dB`),
        ] : [WB.ui.revealButton(ctx, 'D:ch17:m', 'Reveal the margins')]));
      });
    },

    // Margins and bandwidth answer (a) and (c): shown once both are solved or revealed.
    marginsShown: (ctx) => WB.ui.shown(ctx, 'D:ch17:m') || (ctx.app.isSolved('D.17/a') && ctx.app.isSolved('D.17/c')),
    // Part (b) asks for these curves: in Work mode they appear once it is done.
    bode(ctx) {
      if (!lib.shows(ctx, 'D.17/b')) return null;
      const l = loop(ctx), show = CH.ch17.marginsShown(ctx);
      const marks = show ? marginMarks(l.mg) : [];
      if (show && isFinite(l.bw)) marks.push({ w: l.bw, label: `−3 dB ${fmt(l.bw, 3)}`, color: '--series-2' });
      if (show && l.bwN > 1) marks.push({ w: l.bwLast, label: `−3 dB ${fmt(l.bwLast, 3)}`, color: '--series-2' });
      return {
        title: 'Open loop PC and closed loop PC/(1+PC)', w: W,
        lines: [{ label: 'open loop P(jω)C(jω)', ...T.bode(l.Lg, W), color: '--series-1' }, { label: 'closed loop T(jω)', ...T.bode(l.Tc, W), color: '--series-2', width: 1.5 }],
        marks,
      };
    },
    splane(ctx) { return clMarkers(ctx, loop(ctx).Lg); },

    math(ctx) {
      const l = loop(ctx);
      return [
        { title: 'Crossover and phase margin', page: 'p. 303–304',
          theory: '|P(j\\omega_{co})C(j\\omega_{co})| = 1,\\quad PM = \\angle P(j\\omega_{co})C(j\\omega_{co}) + 180^\\circ',
          numbers: `\\omega_{co} = ${tex(l.mg.wc)},\\quad PM = ${tex(l.mg.pm)}^\\circ`, answers: 'D.17/a' },
        { title: 'Gain margin', page: 'p. 305',
          theory: 'GM = \\frac{1}{|PC(j\\omega_{180})|},\\quad \\angle PC(j\\omega_{180}) = -180^\\circ',
          numbers: l.mg.crossings.length ? `GM = ${l.mg.crossings.map((c) => tex(db(c.gm)) + '\\,\\text{dB at } ' + tex(c.w)).join(';\\;')}` : 'GM = \\infty \\;(\\text{phase never reaches } -180^\\circ)', answers: 'D.17/a' },
        { title: 'Open vs. closed loop', page: 'p. 306–307',
          theory: 'T = \\frac{PC}{1+PC}:\\; |PC| \\gg 1 \\Rightarrow |T| \\approx 1,\\; |PC| \\ll 1 \\Rightarrow |T| \\approx |PC|',
          numbers: `\\omega_{bw} = ${tex(l.bw)}${l.bwN > 1 ? `\\;(\\text{final roll-off } ${tex(l.bwLast)})` : ''}\\;\\text{vs.}\\;\\omega_{co} = ${tex(l.mg.wc)}`, answers: ['D.17/a', 'D.17/c'],
          note: 'Usually |T| rolls off a little above ω_co. With D.8-style gains (k_D ≫ k_I) the PID zeros are complex and put a notch in |C| near √(k_I/k_D), so |T| can dip below −3 dB at low frequency and recover.' },
        { title: 'C_PID with dirty derivative', page: 'p. 313',
          theory: 'C(s) = k_P + \\frac{k_I}{s} + \\frac{k_D s}{\\sigma s + 1}',
          numbers: `C(s) = ${T.texTf(pidTf(ctx.st))}` },
      ];
    },

    buildProblem(parent, ctx) {
      // Answers are for a fixed loop (the reference D.10 gains), not the sliders, so they
      // don't go stale when a slider moves.
      const g = refPid(ctx);
      const l = () => loop(ctx, g);
      const fixed = `With the reference D.10 gains (${WB.freq.gainsText(g)}: the D.8(a) design plus the D.10 k<sub>I</sub>), not the sliders.`;
      lib.panel(parent, ctx, ctx.sys.problems.ch17, [
        { id: 'a', title: 'Phase and gain margins under the D.10 PID control', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub> [rad/s]', gm: 'GM [dB]' },
          html: `${fixed} Enter GM in dB, or <code>inf</code> if the phase never reaches −180°.`,
          check: (v) => { const x = l(); return WB.freq.marginCheck(v, { pm: x.mg.pm, wc: x.mg.wc }, { pm: 'PM', wc: 'ωco' }, x.mg); },
          solution: () => {
            const x = l();
            return [
              { tex: `PM = ${tex(x.mg.pm)}^\\circ \\text{ at } \\omega_{co} = ${tex(x.mg.wc)},\\quad GM = ${x.mg.crossings.length ? tex(db(x.mg.gm)) + '\\,\\text{dB}' : '\\infty'}` },
              ...(x.mg.gcs.length > 1 ? [{ html: `|PC| crosses 0 dB ${x.mg.gcs.length} times (the plant resonance pokes back above 0 dB). The PM is the one smallest in magnitude, as python-control's margin reports: ${x.mg.gcs.map((c) => `${fmt(c.pm, 3)}° at ${fmt(c.w, 3)} rad/s`).join('; ')}.` }] : []),
              { html: x.mg.crossings.length ? 'GM is 1/|PC| where the phase crosses −180° (p. 305).' : 'The dirty derivative makes C(s) biproper, so PC rolls off at −40 dB/dec with the phase approaching −180° from above: no phase crossover, GM = ∞.' },
            ];
          } },
        { id: 'b', title: 'Open-loop and closed-loop Bode plots on one graph',
          html: 'In your code, plot P(s)C(s) and T(s) = PC/(1 + PC) together. When you have, click the button: the Bode panel here then shows both for the gains in the sliders.',
          done: 'I\'ve plotted them' },
        { id: 'c', title: 'Closed-loop bandwidth, and how it relates to the crossover frequency', inputs: { bw: 'ω<sub>bw</sub> [rad/s]', ratio: 'ω<sub>bw</sub> / ω<sub>co</sub>' },
          html: `${fixed} ω<sub>bw</sub> is where |T| falls 3 dB below its DC value. Either the first −3 dB crossing of |T| (what <code>bandwidth</code> returns) or its final roll-off is accepted, with the matching ratio.`,
          check: (v) => {
            const x = l(), bw = lib.num(v.bw), ratio = lib.num(v.ratio);
            if (bw === null || ratio === null) return { ok: false, msg: 'Enter ωbw and the ratio ωbw/ωco.' };
            const cand = [x.bw, x.bwLast].filter((w) => isFinite(w));
            const w = cand.find((c) => M.close(bw, c));
            if (w === undefined) return { ok: false, msg: 'Check ωbw.' };
            return M.close(ratio, w / x.mg.wc) ? { ok: true, msg: 'Within 1%.' } : { ok: false, msg: 'Check the ratio ωbw/ωco.' };
          },
          solution: () => {
            const x = l();
            return [
              { tex: `\\omega_{bw} = ${tex(x.bw)}${x.bwN > 1 ? ' \\text{ (first)},\; ' + tex(x.bwLast) + ' \\text{ (final roll-off)}' : ''},\\quad \\omega_{co} = ${tex(x.mg.wc)},\\quad \\frac{\\omega_{bw}}{\\omega_{co}} = ${tex(x.bw / x.mg.wc)}${x.bwN > 1 ? '\;\\text{or}\;' + tex(x.bwLast / x.mg.wc) : ''}` },
              { html: (x.bwN > 1 ? 'The −3 dB "bandwidth" is ambiguous for these gains: |T| dips below −3 dB around √(k<sub>I</sub>/k<sub>D</sub>), where the complex PID zeros make a notch in |C|, then recovers. ' : '') + 'The final roll-off sits a little above ω<sub>co</sub>: below crossover |PC| ≫ 1 so |T| ≈ 1, above it |PC| ≪ 1 so |T| ≈ |PC| falls off with the loop gain (p. 306). With PM ≈ ${fmt(x.mg.pm, 3)}° (well damped, Fig. 17-7) |T| does not peak, so the roll-off lands just above ω<sub>co</sub>: ratio ${fmt(x.bwLast / x.mg.wc, 3)}. The first crossing (ratio ${fmt(x.bw / x.mg.wc, 3)}) comes from the notch, not from crossover.' },
            ];
          } },
      ]);
    },
  };

  // ---------------------------------------------------------------- D.18 --
  const blocks = {
    pi: { label: 'PI (s + z_I)/s', page: 'p. 324 · §18.1.2', tf: (b) => T.pi(b.z) },
    lead: { label: 'Lead M(s + ω/√M)/(s + ω√M)', page: 'p. 328 · Eq. 18.2', tf: (b) => T.lead(b.M, b.w) },
    lag: { label: 'Lag (s + z)/(s + z/M)', page: 'p. 325 · Eq. 18.1', tf: (b) => T.lag(b.z, b.M) },
    lpf1: { label: 'Low-pass p/(s + p)', page: 'p. 325 · §18.1.3', tf: (b) => T.lpf(b.p) },
    lpf2: { label: 'Low-pass 2 p/(s + p)', page: 'p. 325 · §18.1.3', tf: (b) => T.lpf(b.p) },
  };
  const BLOCKS = ['pi', 'lead', 'lag', 'lpf1', 'lpf2'];
  // Work-mode start: a low-gain PI (stable, meets nothing but disturbance rejection).
  function presetStart() {
    return { k: 0.5, pi: { on: true, z: 0.5 }, lead: { on: false, w: 2, M: 5 }, lag: { on: false, z: 0.1, M: 5 }, lpf1: { on: false, p: 50 }, lpf2: { on: false, p: 200 }, pf: { on: false, p: 1 } };
  }
  // Reference design (derived for the solution, tools/regress_D.py verifies it):
  // PI zero 0.5, lead centred at 5 rad/s with M = 30, LPF at 50 rad/s, k for ω_co = 5,
  // prefilter p = 1.
  function presetRef(ctx) {
    const d = { k: 1, pi: { on: true, z: 0.5 }, lead: { on: true, w: 5, M: 30 }, lag: { on: false, z: 0.1, M: 5 }, lpf1: { on: true, p: 50 }, lpf2: { on: false, p: 200 }, pf: { on: true, p: 1 } };
    const C0 = T.mul(blocks.pi.tf(d.pi), blocks.lead.tf(d.lead), blocks.lpf1.tf(d.lpf1));
    d.k = 1 / absAt(T.mul(plantTf(ctx), C0), 5);
    return d;
  }

  // Solution for the implementation parts: the repo's transferFunction (controllable
  // canonical form, one RK4 step per Ts), used for C(s) and for the prefilter.
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
        self.x = np.zeros((n, 1))

    def update(self, u):
        f = lambda x: self.A @ x + self.B * u
        F1 = f(self.x); F2 = f(self.x + P.Ts / 2 * F1)
        F3 = f(self.x + P.Ts / 2 * F2); F4 = f(self.x + P.Ts * F3)
        self.x = self.x + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return (self.C @ self.x)[0, 0] + self.D * u


class Controller:
    def __init__(self):
        self.C = TransferFunction(P.C_num, P.C_den)

    def update(self, z_r, y):
        z = y[0, 0]
        e = z_r - z
        F = self.C.update(e)
        return max(-P.Fmax, min(P.Fmax, F))
`;
  const LS_PF_SOL = LS_SOL
    .replace('        self.C = TransferFunction(P.C_num, P.C_den)\n', '        self.C = TransferFunction(P.C_num, P.C_den)\n        p = 1.0   # prefilter pole\n        self.prefilter = TransferFunction([p], [1, p])\n')
    .replace('e = z_r - z', 'e = self.prefilter.update(z_r) - z');
  // The reference design (presetRef) as coefficient lists, for the D.18 design part.
  const DESIGN_SOL = `# one design that works (not unique): a PI zero at 0.5 rad/s, a lead centred
# at 5 rad/s (M = 30), a low-pass filter at 50 rad/s, and the gain k that puts
# the crossover at 5 rad/s
z_I, M, w, p = 0.5, 30, 5, 50
num = np.convolve(np.convolve([1, z_I], [M, M * w / np.sqrt(M)]), [p])
den = np.convolve(np.convolve([1, 0], [1, w * np.sqrt(M)]), [1, p])
s = 1j * w
P_jw = (1 / P.m) / (s**2 + P.b / P.m * s + P.k / P.m)
k = 1 / abs(P_jw * np.polyval(num, s) / np.polyval(den, s))   # |P C(j5)| = 1
C_num = k * num
C_den = den
`;
  const SQ18 = { type: 'square', amplitude: 0.5, frequency: 0.02, tStep: 0 };

  CH.ch18 = {
    id: 'ch18', num: 18, tab: 'Ch 18', title: 'Loopshaping', pages: 'pp. 323–374, p. 383',
    // mine: the student's C(s) from the design part (Work mode), as {num, den}.
    defaults() { return { ...presetStart(), showT: true, mine: null }; },
    simDefaults(sys) { return { ...sys.problems.ch18.sim, mismatch: sys.problems.ch18.mismatch }; },
    gains() { return {}; },
    linearLabel: 'linear loop (no saturation, d, noise)',
    // Work mode simulates the student's controller, which gets the designed C(s) as
    // P.C_num, P.C_den (coefficient lists, as loopshape_tools gives them).
    implement: {
      feed: 'y', linear: false,
      params(ctx) { const C = CH.ch18.design(ctx).C; return { C_num: C.num.slice(), C_den: C.den.slice() }; },
    },

    // Work mode: C is the student's own (Python, design part), or 1 until they give one,
    // and the prefilter is in their code; Explore: the block knobs.
    design(ctx, st = ctx.st) {
      const work = ctx.S.mode === 'work';
      let C = T.gain(work ? 1 : st.k);
      if (work) { if (st.mine) C = T.tf(st.mine.num, st.mine.den); } else for (const key of BLOCKS) if (st[key].on) C = T.mul(C, blocks[key].tf(st[key]));
      const P = plantTf(ctx), Lg = T.mul(P, C);
      const F = !work && st.pf.on ? T.lpf(st.pf.p) : T.gain(1);
      const pr = ctx.sys.problems.ch18;
      const { mag } = T.bode(Lg, W);
      let lowMin = Infinity, highMax = 0;
      for (let i = 0; i < W.length; i++) {
        if (W[i] <= pr.wr) lowMin = Math.min(lowMin, mag[i]);
        if (W[i] >= pr.wn) highMax = Math.max(highMax, mag[i]);
      }
      lowMin = Math.min(lowMin, absAt(Lg, pr.wr));   // the grid may straddle the spec frequencies
      highMax = Math.max(highMax, absAt(Lg, pr.wn));
      const mg = margins(Lg);
      const cl = L.roots(L.polyAdd(Lg.den, Lg.num));
      const stable = cl.every((p) => p.re < 0);
      const integ = Lg.den[Lg.den.length - 1] === 0 && Math.abs(Lg.num[Lg.num.length - 1]) > 0;
      return {
        C, Lg, F, P, mg, stable, lowMin, highMax,
        integOk: integ, lowOk: lowMin >= 1 / pr.gr, highOk: highMax <= pr.gn, pmOk: Math.abs(mg.pm - pr.pm) <= 5,
      };
    },

    controller(ctx) {
      const d = this.design(ctx), Ts = ctx.S.sim.Ts;
      const Cf = T.filter(d.C, Ts), Ff = T.filter(d.F, Ts);
      // F = C(s) E(s), e = F(s) z_r - z  (Listing 18.3 pattern, state-space form of C and F)
      return { update(r, x, yMeas) { const rf = Ff.step(r); return Cf.step(rf - yMeas); } };
    },
    linearSim(ctx, c) { return lib.linearSim(ctx, c, (cx) => this.controller(cx)); },

    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') {
        // C comes from the student's Python in the design part, so no block menu here.
        WB.myCtrl.banner(section(parent, 'Your controller'), ctx, 'the implementation parts');
        this.specSection(parent, ctx);
        return;
      }
      const pre = section(parent, 'Start from', 'p. 323–337');
      pre.append(el('div', { class: 'btn-row' },
        el('button', { type: 'button', class: 'btn', text: 'PI only', onclick: () => { Object.assign(ctx.st, presetStart()); ctx.update(); } })));
      lib.note(pre, 'C(s) = k · PI · lead · lag · LPF · LPF. The reference design is in the problem panel under Show solution.');

      const g = section(parent, 'Gain k', 'p. 324');
      slider(g, { label: 'k', min: 0.01, max: 1000, log: true, sig: 4, ...bind(ctx, 'k') });
      const onOff = (sec, key) => WB.ui.onOff(sec, ctx, () => ctx.st[key]);
      const off = (key) => () => !ctx.st[key].on;
      const pi = section(parent, blocks.pi.label, blocks.pi.page);
      onOff(pi, 'pi');
      slider(pi, { label: 'z<sub>I</sub>', unit: 'rad/s', min: 0.005, max: 20, log: true, sig: 3, ...bind(ctx, 'z', () => ctx.st.pi), disabled: off('pi') });
      const lead = section(parent, blocks.lead.label, blocks.lead.page);
      onOff(lead, 'lead');
      slider(lead, { label: 'ω<sub>lead</sub>', unit: 'rad/s', min: 0.1, max: 200, log: true, sig: 3, ...bind(ctx, 'w', () => ctx.st.lead), disabled: off('lead') });
      slider(lead, { label: 'M', min: 1.01, max: 100, log: true, sig: 3, ...bind(ctx, 'M', () => ctx.st.lead), disabled: off('lead') });
      const leadPhase = el('p', { class: 'muted small' });
      lead.append(leadPhase);
      WB.ui.addRefresher(() => { const Mx = ctx.st.lead.M; leadPhase.textContent = `max phase added: sin⁻¹((M−1)/(M+1)) = ${fmt(Math.asin((Mx - 1) / (Mx + 1)) * 180 / Math.PI, 3)}° at ω_lead`; });
      const lag = section(parent, blocks.lag.label, blocks.lag.page);
      onOff(lag, 'lag');
      slider(lag, { label: 'z', unit: 'rad/s', min: 0.005, max: 20, log: true, sig: 3, ...bind(ctx, 'z', () => ctx.st.lag), disabled: off('lag') });
      slider(lag, { label: 'M', min: 1.01, max: 200, log: true, sig: 3, ...bind(ctx, 'M', () => ctx.st.lag), disabled: off('lag') });
      for (const key of ['lpf1', 'lpf2']) {
        const s = section(parent, blocks[key].label, blocks[key].page);
        onOff(s, key);
        slider(s, { label: 'p', unit: 'rad/s', min: 1, max: 5000, log: true, sig: 3, ...bind(ctx, 'p', () => ctx.st[key]), disabled: off(key) });
      }
      const pf = section(parent, 'Prefilter F(s) = p/(s + p)', 'p. 336 · Eq. 18.5–18.7');
      onOff(pf, 'pf');
      slider(pf, { label: 'p', unit: 'rad/s', min: 0.05, max: 50, log: true, sig: 3, ...bind(ctx, 'p', () => ctx.st.pf), disabled: off('pf') });
      segmented(pf, { label: 'Bode: closed loop F·T', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'showT') });
      this.specSection(parent, ctx);
    },

    specSection(parent, ctx) {
      const sp = section(parent, 'D.18 specs', 'p. 383');
      const box = el('div', { class: 'metrics' });
      sp.append(box);
      WB.ui.addRefresher(() => {
        const d = this.design(ctx);
        const row = WB.ui.specRow;
        box.replaceChildren(
          row('constant input disturbance rejected', d.integOk, d.integOk ? 'yes' : 'no'),
          row('|PC| ≥ 1/0.03 for ω ≤ 0.1', d.lowOk, `min ${fmt(db(d.lowMin), 3)} dB (need ${fmt(db(1 / 0.03), 3)})`),
          row('|PC| ≤ 0.001 for ω ≥ 500', d.highOk, `max ${fmt(db(d.highMax), 3)} dB (need −60)`),
          row('PM ≈ 60° (±5°)', d.pmOk, `${fmt(d.mg.pm, 3)}° at ${fmt(d.mg.wc, 3)} rad/s${d.mg.gcs.length > 1 ? ` (${d.mg.gcs.length} crossovers)` : ''}`),
          row('closed loop stable', d.stable, d.stable ? 'yes' : 'no'),
          WB.ui.metric('gain margin(s)', gmText(d.mg)),
        );
      });
    },

    // Evaluate the student's C_num, C_den and keep them in the chapter state.
    async loadMine(ctx, code) {
      const out = await WB.py.evaluate(code, [{ params: ctx.pModel, vars: ['C_num', 'C_den'] }]);
      if (out.error) return WB.yours.pyError(out);
      const v = out.rows[0].vars;
      const num = L.trimLeading([v.C_num].flat(Infinity)), den = L.trimLeading([v.C_den].flat(Infinity));
      if (![...num, ...den].every((x) => typeof x === 'number' && Number.isFinite(x))) return { ok: false, msg: 'C_num and C_den must be lists of real numbers.' };
      if (!den.length || den[0] === 0) return { ok: false, msg: 'C_den must have a nonzero leading coefficient.' };
      if (num.length > den.length) return { ok: false, msg: 'C(s) must be proper: the numerator degree can be at most the denominator degree.' };
      ctx.st.mine = { num, den };
      ctx.update();
      return { ok: true };
    },
    // The implementation parts run on the designed C: in Work mode, the student's.
    needMine(ctx) {
      return ctx.S.mode === 'work' && !ctx.st.mine ? { ok: false, msg: 'Give your C(s) in the first part and press Use my C: it becomes P.C_num, P.C_den.' } : null;
    },

    bode(ctx) {
      const d = this.design(ctx), pr = ctx.sys.problems.ch18;
      const lines = [{ label: 'P', ...T.bode(d.P, W), color: '--text-muted', dash: [5, 4], width: 1.5 }, { label: 'P·C (loop gain)', ...T.bode(d.Lg, W), color: '--series-1' }];
      if (ctx.st.showT) lines.push({ label: 'closed loop F·T', mag: T.bode(T.mul(d.F, T.feedback(d.Lg)), W).mag, color: '--series-3', width: 1.5 });
      return {
        title: 'Loopshaping: P and P·C', w: W, lines,
        specs: [
          { w0: 1e-3, w1: pr.wr, db: db(1 / pr.gr), keep: 'above', color: '--critical', label: 'tracking spec' },
          { w0: pr.wn, w1: 1e4, db: db(pr.gn), keep: 'below', color: '--critical', label: 'noise spec' },
        ],
        marks: marginMarks(d.mg),
      };
    },
    splane(ctx) { return clMarkers(ctx, this.design(ctx).Lg); },

    math(ctx) {
      const d = this.design(ctx), st = ctx.st, pr = ctx.sys.problems.ch18;
      const parts = [tex(st.k)];
      for (const key of BLOCKS) if (st[key].on) parts.push(T.texTf(blocks[key].tf(st[key]), 4));
      return [
        { title: 'Specs as loop-gain constraints', page: 'p. 286 · Eq. 16.4, p. 287 · Eq. 16.6',
          theory: `|PC| \\ge \\tfrac{1}{\\gamma_r} = ${tex(1 / pr.gr)}\\;(${tex(db(1 / pr.gr))}\\,\\text{dB}) \\text{ for } \\omega \\le ${pr.wr},\\quad |PC| \\le \\gamma_n = ${pr.gn}\\;(-60\\,\\text{dB}) \\text{ for } \\omega \\ge ${pr.wn}` },
        { title: 'Building blocks multiply', page: 'p. 323',
          theory: '20\\log|PC| = 20\\log|P| + \\textstyle\\sum 20\\log|C_i|,\\quad \\angle PC = \\angle P + \\sum \\angle C_i' },
        { title: 'Lag and lead', page: 'p. 325–328 · Eq. 18.1–18.2',
          theory: 'C_{lag} = \\frac{s + z}{s + z/M},\\quad C_{lead} = M\\frac{s + \\omega_L/\\sqrt M}{s + \\omega_L\\sqrt M},\\; \\phi_{max} = \\sin^{-1}\\frac{M-1}{M+1}' },
        { title: 'Your C(s)', page: 'p. 323',
          theory: ctx.S.mode !== 'work' ? 'C(s) = ' + parts.join('\\cdot ') : st.mine ? `C(s) = ${T.texTf(d.C, 4)}` : 'C(s) = 1\\quad\\text{(none yet: first part)}' },
        { title: 'Margins', page: 'p. 303–305',
          theory: `PM = ${tex(d.mg.pm)}^\\circ \\text{ at } \\omega_{co} = ${tex(d.mg.wc)},\\quad GM = ${d.mg.crossings.length ? d.mg.crossings.map((c) => tex(db(c.gm)) + '\\,\\text{dB at } ' + tex(c.w)).join(';\\;') : '\\infty'}` },
        { title: 'Implementation', page: 'p. 334–335 · Eq. 18.3–18.4, p. 336 · Eq. 18.5–18.7',
          theory: '\\dot z_C = A_C z_C + B_C e,\\; F = C_C z_C + D_C e,\\quad \\dot z_F = -p z_F + z_r,\\; z^r_f = p z_F,\\quad e = z^r_f - z' },
      ];
    },

    buildProblem(parent, ctx) {
      const run = async (code, sc) => this.needMine(ctx) || WB.myCtrl.run(ctx, code, sc);
      lib.panel(parent, ctx, ctx.sys.problems.ch18, [
        { id: 'a', title: 'Meet the loop-gain specs',
          html: 'Give C(s) as coefficient lists, highest power of s first (<code>np.convolve</code> multiplies two factors). <em>Use my C</em> draws P·C in the Bode plot, the s-plane and the spec readouts. The check also requires a stable closed loop.',
          code: {
            template: 'C_num = [1.0]\nC_den = [1.0]\n',
            check: async (code) => {
              const r = await this.loadMine(ctx, code);
              if (r.ok === false) return r;
              const d = this.design(ctx);
              const ok = d.integOk && d.lowOk && d.highOk && d.pmOk && d.stable;
              return { ok, msg: `constant d_in ${d.integOk ? '✓' : '✗'}, tracking ${d.lowOk ? '✓' : '✗'}, noise ${d.highOk ? '✓' : '✗'}, PM ${fmt(d.mg.pm, 3)}° ${d.pmOk ? '✓' : '✗'}, stable ${d.stable ? '✓' : '✗'}` };
            },
            actions: [
              { label: 'Use my C', run: async (code) => { const r = await this.loadMine(ctx, code); return r.ok === false ? r : { info: true, msg: 'Your C(s) is in the Bode plot, the s-plane and the spec readouts.' }; } },
              { label: 'Load the reference design', run: () => {
                // Applying it answers this part: Work mode allows it once it is solved.
                if (ctx.S.mode === 'explore') { Object.assign(ctx.st, presetRef(ctx)); ctx.update(); return null; }
                if (!lib.shows(ctx, 'D.18/a')) return { ok: false, msg: 'Work mode loads the reference design once your own design passes. Explore mode has it now.' };
                const C = this.design({ ...ctx, S: { ...ctx.S, mode: 'explore' } }, { ...ctx.st, ...presetRef(ctx) }).C;
                ctx.st.mine = { num: C.num.slice(), den: C.den.slice() };
                ctx.update();
                return { info: true, msg: 'The reference C(s) is in the plots (and in P.C_num, P.C_den).' };
              } },
            ],
          },
          solution: () => {
            const r = presetRef(ctx), d = this.design({ ...ctx, S: { ...ctx.S, mode: 'explore' } }, { ...ctx.st, ...r });
            return [
              { html: 'One design that works (not unique). The plant is easy at high frequency (|P(j500)| ≈ −122 dB), so the binding specs are tracking (|PC| ≥ 30.5 dB up to 0.1 rad/s) and the phase margin:' },
              { tex: `C(s) = ${tex(r.k)}\\cdot\\frac{s + 0.5}{s}\\cdot 30\\,\\frac{s + 5/\\sqrt{30}}{s + 5\\sqrt{30}}\\cdot\\frac{50}{s + 50}` },
              { tex: `PM = ${tex(d.mg.pm)}^\\circ \\text{ at } ${tex(d.mg.wc)}\\,\\text{rad/s},\\quad \\min_{\\omega\\le0.1}|PC| = ${tex(db(d.lowMin))}\\,\\text{dB},\\quad \\max_{\\omega\\ge500}|PC| = ${tex(db(d.highMax))}\\,\\text{dB}` },
              { code: DESIGN_SOL },
              { html: 'The PI puts an integrator in C (constant d<sub>in</sub> rejected); crossing over at 5 rad/s with a lead of M = 30 centred there buys the phase and lifts the low-frequency gain above 1/γ<sub>r</sub>; the LPF at 50 rad/s rolls off the lead. Python-control gives the same margins.' },
            ];
          } },
        WB.myCtrl.part(ctx, {
          id: 'impl', title: 'Implement C(s) in simulation with its state-space equivalent',
          html: 'Your design is in <code>P.C_num</code>, <code>P.C_den</code> (coefficient lists, highest power of s first). Realize C(s) as ż<sub>C</sub> = A<sub>C</sub>z<sub>C</sub> + B<sub>C</sub>e, F = C<sub>C</sub>z<sub>C</sub> + D<sub>C</sub>e (Eq. 18.3–18.4) and use it on e = z<sub>r</sub> − z. The check runs the ±0.5 m square wave with exact parameters and compares z(t) with the workbench running the same C(s) (within 3% of the step).',
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQ18, tEnd: 20 });
            const mine = await run(code, sc);
            if (mine.ok === false) return mine;
            const Cd = this.design(ctx).C;
            // The repo's transferFunction updates the state, then outputs; a filter that outputs first is fine too.
            const refFor = (make) => {
              const f = make(Cd, sc.Ts);
              return WB.myCtrl.reference(ctx, sc, { update: (r, x, y) => (f.update ? f.update(r - y) : f.step(r - y)) });
            };
            const e = Math.min(WB.myCtrl.maxDiff(mine, refFor(T.repoFilter)).e, WB.myCtrl.maxDiff(mine, refFor(T.filter)).e);
            return e <= 0.03 * 0.5 ? { ok: true, msg: `Your z(t) matches C(s) run by the workbench to within ${fmt(1000 * e, 3)} mm.` } : { ok: false, msg: `Your z(t) differs from C(s) run by the workbench by ${fmt(1000 * e, 3)} mm.` };
          },
          solution: () => [{ code: LS_SOL }, { html: 'The repo\'s loopshape_tools.py transferFunction: controllable canonical form, one RK4 step per T<sub>s</sub>, output after the update.' }],
        }),
        WB.myCtrl.part(ctx, {
          id: 'pf', title: 'Add a prefilter F(s) that reduces the peaking', seed: 'D.18/impl',
          html: 'Filter z<sub>r</sub> before the error: e = z<sub>f</sub><sup>r</sup> − z with z<sub>f</sub><sup>r</sup> = F(s)z<sub>r</sub>, F implemented like C. The check runs the ±0.5 m square wave with exact parameters: the first step may overshoot by at most 5%, and |z<sub>r</sub> − z| just before the first switch (t = 25 s) must be under 5 mm.',
          check: async (code) => {
            const sc = WB.myCtrl.scenario(ctx, { ref: SQ18, tEnd: 25 });
            const res = await run(code, sc);
            if (res.ok === false) return res;
            const k = Math.round(24.95 / sc.Ts);
            const m = M.stepMetrics(res.t, res.y, 0, res.t.length, res.y[0], res.r[1]);
            const e = Math.abs(res.r[k] - res.y[k]);
            return { ok: m.os < 5 && e < 0.005, msg: `Overshoot ${fmt(m.os, 3)}%; error before the switch ${fmt(1000 * e, 3)} mm.` };
          },
          solution: () => [
            { code: LS_PF_SOL },
            { html: 'A first-order F = p/(s + p) with p well below crossover (p = 1 with the reference C) shapes the reference so the steps never excite the lead\'s peaking: no overshoot (about 12% without it), rise time about 2 s on the ±0.5 m square wave. The 1 m jumps still ask for more than F<sub>max</sub> = 6 N for a moment (the plant saturates; the PI has no anti-windup but recovers here). D.18 itself says nothing about F<sub>max</sub>.' },
          ],
        }),
      ]);
    },
  };

  WB.studies.D.freq = { presetRef, specs, loop, plantTf, pidTf };
})();
