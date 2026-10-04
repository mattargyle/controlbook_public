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

  function loop(ctx) {
    const Lg = T.mul(plantTf(ctx), pidTf(ctx.st)), Tc = T.feedback(Lg);
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
      row.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reference D.10 design', title: 't_r = 2 s, ζ = 0.7, k_I = 1, σ = 0.05', onclick: () => { Object.assign(ctx.st, refPid(ctx)); ctx.update(); } }));
    }
    sec.append(row);
    return sec;
  }
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
    const mk = L.roots(plantTf(ctx).den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of P ${i + 1}` }));
    poles.forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole ${i + 1}`, noFit: Math.hypot(p.re, p.im) > 12 }));
    return { markers: mk, fitR: 3 };
  }
  const marginMarks = (mg) => WB.freq.marginMarks(mg);

  // ---------------------------------------------------------------- D.15 --
  CH.ch15 = {
    id: 'ch15', num: 15, tab: 'D.15', title: 'Frequency response', pages: 'pp. 261–282, p. 382',
    openLoop: true, metrics: false,
    defaults() { return { comp: 'none', w0: 0.5, A: 1, asym: true, inp: { shape: 'sine', amp: 1, freq: 0.5 / (2 * Math.PI) } }; },
    simDefaults(sys) { return sys.problems.ch15.sim; },
    controller(ctx, o) {
      ctx.st.inp = { shape: 'sine', amp: ctx.st.A, freq: ctx.st.w0 / (2 * Math.PI), t0: 0 };
      return WB.studies.D.models.openLoop(ctx, o);
    },
    linearSim(ctx, c) { return lib.linearSim(ctx, c, (cx, o) => this.controller(cx, o)); },
    linearLabel: 'linear model (with transient)',
    outputSeries(ctx, res, sc) {
      if (!WB.ui.shown(ctx, 'D:ch15:bode')) return [];
      const g = T.at(plantTf(ctx), ctx.st.w0), mag = L.C.abs(g), ph = L.C.arg(g);
      return [{ label: 'A|P(jω₀)| sin(ω₀t + ∠P)', y: sc(Array.from(res.t, (t) => ctx.st.A * mag * Math.sin(ctx.st.w0 * t + ph))), color: '--series-3', dash: [2, 3], width: 2 }];
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Sinusoidal input F̃ = A sin(ω₀t)', 'p. 264 · Eq. 15.4');
      slider(sec, { label: 'ω<sub>0</sub>', unit: 'rad/s', min: 0.05, max: 20, log: true, sig: 3, ...bind(ctx, 'w0') });
      slider(sec, { label: 'A', unit: 'N', min: 0, max: 6, step: 0.01, sig: 3, ...bind(ctx, 'A') });
      segmented(sec, { label: 'Straight-line approximation', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'asym') });
      lib.note(sec, 'After the transient (it decays like e^(−ζωₙt), slowly for this plant) z is a sinusoid with gain |P(jω₀)| and phase ∠P(jω₀). Sweep ω₀ and watch the amplitude to find the resonance.');
      if (ctx.S.mode === 'work') {
        const b = WB.ui.revealButton(ctx, 'D:ch15:bode', 'Reveal the Bode plot (compare with your sketch)');
        WB.ui.addRefresher(() => { b.hidden = ctx.app.isRevealed('D:ch15:bode'); });
        sec.append(b);
      }
    },
    bode(ctx) {
      if (!WB.ui.shown(ctx, 'D:ch15:bode')) return null;
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
      if (!WB.ui.shown(ctx, 'D:ch15:bode')) return null;
      return { markers: L.roots(plantTf(ctx).den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of P ${i + 1}` })) };
    },
    math(ctx) {
      const b = ans.bode(ctx.pModel);
      return [
        { title: 'Frequency response', page: 'p. 264 · Eq. 15.4',
          theory: 'u = A\\sin\\omega_0 t \\;\\Rightarrow\\; y_{ss} = A|P(j\\omega_0)|\\sin\\big(\\omega_0 t + \\angle P(j\\omega_0)\\big)' },
        { title: 'Bode canonical form', page: 'p. 266 · Eq. 15.5–15.7',
          theory: 'P(j\\omega) = K\\,\\frac{1}{\\left(1 - \\frac{\\omega^2}{\\omega_n^2}\\right) + j2\\zeta\\frac{\\omega}{\\omega_n}}',
          symbolic: 'K = \\frac{1}{k},\\quad \\omega_n = \\sqrt{\\frac km},\\quad \\zeta = \\frac{b}{2\\sqrt{km}}',
          numbers: `K = ${tex(b.dc)}\\;(${tex(b.dcDb)}\\,\\text{dB}),\\quad \\omega_n = ${tex(b.wn)},\\quad \\zeta = ${tex(b.zeta)}`, spoiler: true },
        { title: 'Complex pole pair', page: 'p. 270–272 · Fig. 15-9',
          theory: '\\omega \\ll \\omega_n: 0\\text{ dB}, 0^\\circ;\\quad \\omega \\gg \\omega_n: -40\\text{ dB/dec}, -180^\\circ;\\quad \\omega = \\omega_n: -20\\log_{10}|2\\zeta|,\\; -90^\\circ',
          numbers: `-20\\log_{10}(2\\zeta) = ${tex(-db(2 * b.zeta))}\\,\\text{dB} \\Rightarrow |P(j\\omega_n)| = ${tex(b.peakDb)}\\,\\text{dB}`, spoiler: true,
          note: 'ζ is small, so the resonant peak is tall and narrow, and the straight-line sketch misses it badly.' },
      ];
    },
    buildProblem(parent, ctx) {
      const b = () => ans.bode(ctx.pModel);
      const magDb = (w) => db(absAt(plantTf(ctx), w));
      lib.panel(parent, ctx, ctx.sys.problems.ch15, [
        {
          id: 'a', title: 'Straight-line pieces',
          inputs: { K: 'low-frequency gain [dB]', wn: 'corner ω<sub>n</sub> [rad/s]', s2: 'slope above [dB/dec]', ph: 'phase above [°]' },
          check: (v) => lib.check(v, { K: b().dcDb, wn: b().wn, s2: -40, ph: -180 }, {}),
          solution: () => [{ tex: `P(j\\omega) = \\frac{${tex(b().dc)}}{1 - (\\omega/${tex(b().wn)})^2 + j\\,2(${tex(b().zeta)})\\,\\omega/${tex(b().wn)}}:\\; ${tex(b().dcDb)}\\,\\text{dB flat, then } -40\\,\\text{dB/dec}; \\; 0^\\circ \\to -180^\\circ` }],
        },
        {
          id: 'b', title: 'Resonance',
          inputs: { zeta: 'ζ', peak: '|P(jω<sub>n</sub>)| [dB]' },
          check: (v) => lib.check(v, { zeta: b().zeta, peak: b().peakDb }, {}),
          solution: () => [{ tex: `\\zeta = \\frac{b}{2\\sqrt{km}} = ${tex(b().zeta)},\\quad |P(j\\omega_n)| = \\frac{1}{k\\,2\\zeta} = \\frac{1}{b\\,\\omega_n} = ${tex(b().peak)} = ${tex(b().peakDb)}\\,\\text{dB}` }],
        },
        {
          id: 'c', title: 'Magnitudes from the bode command',
          inputs: { m1: '|P(j0.1)| [dB]', m2: '|P(j1)| [dB]', m3: '|P(j10)| [dB]' },
          check: (v) => lib.check(v, { m1: magDb(0.1), m2: magDb(1), m3: magDb(10) }, {}),
          solution: () => [{ tex: `${tex(magDb(0.1))},\\; ${tex(magDb(1))},\\; ${tex(magDb(10))}\\;\\text{dB}` }, { html: 'Checked against python-control in tools/regress_D.py.' }],
        },
      ]);
    },
  };

  // ---------------------------------------------------------------- D.16 --
  CH.ch16 = {
    id: 'ch16', num: 16, tab: 'D.16', title: 'Frequency-domain specs', pages: 'pp. 283–301, pp. 382–383',
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
        box.replaceChildren(...(WB.ui.shown(ctx, 'D:ch16:specs') ? [
          row('unit-ramp error k/k_I', `${fmt(s.ramp, 3)} m`),
          row('d_in below ω_d,in: 1/|C|', `${fmt(-s.Bdin, 3)} dB → ${fmt(100 * s.gdin, 3)} %`),
          row('exact |P/(1+PC)| at ω_d,in', `${fmt(100 * s.gdinExact, 3)} %`),
          row('noise above ω_no: |PC|', `${fmt(db(s.gn), 3)} dB → ${fmt(100 * s.gn, 3)} %`),
        ] : [WB.ui.revealButton(ctx, 'D:ch16:specs', 'Reveal the spec readouts')]));
      });
    },

    bode(ctx) {
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
          symbolic: 'M_v = \\lim_{s\\to0} s\\,\\frac{1/m}{s^2 + \\frac bm s + \\frac km}\\,\\frac{k_I}{s} = \\frac{k_I}{k}',
          numbers: `M_v = ${tex(s.Mv)} \\Rightarrow e_{ramp} = ${tex(s.ramp)}\\,\\text{m}`, spoiler: true },
        { title: 'Input disturbance', page: 'p. 290 · Eq. 16.8, p. 291 · Eq. 16.9',
          theory: '20\\log|PC| - 20\\log|P| = 20\\log|C| \\ge B_{d_{in}} \\text{ for } \\omega \\le \\omega_{d_{in}},\\quad \\gamma_{d_{in}} = 10^{-B_{d_{in}}/20}',
          numbers: `|C(j\\omega_{d,in})| = ${tex(s.Bdin)}\\,\\text{dB} \\Rightarrow \\gamma_{d_{in}} = ${tex(s.gdin)}\\quad(\\text{exact } |P/(1+PC)| = ${tex(s.gdinExact)})`, spoiler: true,
          note: ctx.S.mode !== 'explore' ? 'The book\'s rule assumes |PC| ≫ 1 at ω_d,in. Check that assumption for your gains.' : `The book's rule assumes |PC| ≫ 1 at ω_d,in. Here |PC(j${fmt(ctx.st.wdin, 3)})| = ${fmt(absAt(s.Lg, ctx.st.wdin), 3)}, so the exact value differs from 1/|C|.` },
        { title: 'Noise', page: 'p. 287 · Eq. 16.6',
          theory: '20\\log|PC| \\le 20\\log\\gamma_n \\text{ for } \\omega \\ge \\omega_{no}',
          numbers: `|PC(j\\omega_{no})| = ${tex(db(s.gn))}\\,\\text{dB} \\Rightarrow \\gamma_n = ${tex(s.gn)}`, spoiler: true },
        { title: 'C_PID with dirty derivative', page: 'p. 313',
          theory: 'C(s) = \\frac{(k_D + \\sigma k_P)s^2 + (k_P + \\sigma k_I)s + k_I}{s(\\sigma s + 1)}',
          numbers: `C(s) = ${T.texTf(s.C)}` },
      ];
    },

    buildProblem(parent, ctx) {
      const s = () => specs(ctx);
      const either = (v, a, b, what) => {
        const g = lib.num(v);
        if (g === null) return { ok: false, msg: 'Enter a number.' };
        if (M.close(g, a)) return { ok: true, msg: `Matches the book's ${what} rule.` };
        if (M.close(g, b)) return { ok: true, msg: 'Matches the exact closed-loop value.' };
        return { ok: false, msg: `Check ${what}.` };
      };
      lib.panel(parent, ctx, ctx.sys.problems.ch16, [
        { id: 'a', title: '(a) tracking error to a unit ramp', inputs: { v: 'e<sub>ss</sub> [m]' },
          html: 'For the PID gains in the sliders (load your D.10 gains first).',
          check: (v) => (ctx.st.kI > 0 ? lib.check(v, { v: s().ramp }, { v: 'e_ss' }) : { ok: false, msg: 'Set kI > 0.' }),
          solution: () => [{ tex: `\\text{type 1}:\\; e_{ss} = \\frac{1}{M_v} = \\frac{k}{k_I} = ${tex(s().ramp)}\\,\\text{m}` }, { html: 'The dirty derivative and k<sub>D</sub> do not enter: only the integrator survives as s → 0.' }] },
        { id: 'b', title: '(b) % of d<sub>in</sub> below 0.1 rad/s that shows up in z', inputs: { v: '%' },
          check: (v) => either(v.v, 100 * s().gdin, 100 * s().gdinExact, '1/|C(jω_d,in)|'),
          solution: () => [{ tex: `\\gamma_{d_{in}} = \\frac{1}{|C(j0.1)|} = ${tex(100 * s().gdin)}\\%\\quad(\\text{exact: } ${tex(100 * s().gdinExact)}\\%)` }, { html: '|C| falls with ω below the PID zeros (k<sub>I</sub>/ω dominates), so the worst case on ω ≤ 0.1 is at 0.1 rad/s.' }] },
        { id: 'c', title: '(c) % of noise above 100 rad/s that shows up in z', inputs: { v: '%' },
          check: (v) => either(v.v, 100 * s().gn, 100 * s().gnExact, '|PC(jω_no)|'),
          solution: () => [{ tex: `\\gamma_n = |PC(j100)| = ${tex(db(s().gn))}\\,\\text{dB} = ${tex(100 * s().gn)}\\%` }, { html: 'At high frequency C → (k<sub>D</sub> + σk<sub>P</sub>)/σ (the dirty derivative caps it) and P ≈ 1/(mω²).' }] },
      ]);
    },
  };

  // ---------------------------------------------------------------- D.17 --
  CH.ch17 = {
    id: 'ch17', num: 17, tab: 'D.17', title: 'Stability margins', pages: 'pp. 303–322, p. 383',
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
        box.replaceChildren(...(WB.ui.shown(ctx, 'D:ch17:m') ? [
          row('phase margin (smallest)', `${fmt(l.mg.pm, 3)}° at ω_co = ${fmt(l.mg.wc, 3)} rad/s`),
          ...(l.mg.gcs.length > 1 ? [row('all gain crossovers', l.mg.gcs.map((c) => `${fmt(c.pm, 3)}° at ${fmt(c.w, 3)}`).join(', '))] : []),
          row('gain margin(s)', gmText(l.mg)),
          row('closed-loop bandwidth (first −3 dB)', `${fmt(l.bw, 3)} rad/s`),
          row('final −3 dB roll-off of |T|', `${fmt(l.bwLast, 3)} rad/s`),
          row('closed-loop peak |T|', `${fmt(l.peak, 3)} dB`),
        ] : [WB.ui.revealButton(ctx, 'D:ch17:m', 'Reveal the margins')]));
      });
    },

    bode(ctx) {
      const l = loop(ctx), show = WB.ui.shown(ctx, 'D:ch17:m');
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
          numbers: `\\omega_{co} = ${tex(l.mg.wc)},\\quad PM = ${tex(l.mg.pm)}^\\circ`, spoiler: true },
        { title: 'Gain margin', page: 'p. 305',
          theory: 'GM = \\frac{1}{|PC(j\\omega_{180})|},\\quad \\angle PC(j\\omega_{180}) = -180^\\circ',
          numbers: l.mg.crossings.length ? `GM = ${l.mg.crossings.map((c) => tex(db(c.gm)) + '\\,\\text{dB at } ' + tex(c.w)).join(';\\;')}` : 'GM = \\infty \\;(\\text{phase never reaches } -180^\\circ)', spoiler: true },
        { title: 'Open vs. closed loop', page: 'p. 306–307',
          theory: 'T = \\frac{PC}{1+PC}:\\; |PC| \\gg 1 \\Rightarrow |T| \\approx 1,\\; |PC| \\ll 1 \\Rightarrow |T| \\approx |PC|',
          numbers: `\\omega_{bw} = ${tex(l.bw)}${l.bwN > 1 ? `\\;(\\text{final roll-off } ${tex(l.bwLast)})` : ''}\\;\\text{vs.}\\;\\omega_{co} = ${tex(l.mg.wc)}`, spoiler: true,
          note: 'Usually |T| rolls off a little above ω_co. With D.8-style gains (k_D ≫ k_I) the PID zeros are complex and put a notch in |C| near √(k_I/k_D), so |T| can dip below −3 dB at low frequency and recover.' },
        { title: 'C_PID with dirty derivative', page: 'p. 313',
          theory: 'C(s) = k_P + \\frac{k_I}{s} + \\frac{k_D s}{\\sigma s + 1}',
          numbers: `C(s) = ${T.texTf(pidTf(ctx.st))}` },
      ];
    },

    buildProblem(parent, ctx) {
      const l = () => loop(ctx);
      lib.panel(parent, ctx, ctx.sys.problems.ch17, [
        { id: 'a', title: 'Margins and bandwidth', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
          html: 'For the PID gains in the sliders (load your D.10 gains first). ω<sub>bw</sub>: either the first −3 dB crossing of |T| (what <code>bandwidth</code> returns) or its final roll-off is accepted.',
          check: (v) => {
            const x = l();
            const r = lib.check({ pm: v.pm, wc: v.wc }, { pm: x.mg.pm, wc: x.mg.wc }, { pm: 'PM', wc: 'ωco' });
            if (!r.ok) return r;
            const g = lib.num(v.bw);
            if (g === null) return { ok: false, msg: 'Enter ωbw.' };
            if (M.close(g, x.bw) || M.close(g, x.bwLast)) return { ok: true, msg: 'Within 1%.' };
            return { ok: false, msg: 'Check ωbw.' };
          },
          solution: () => {
            const x = l();
            return [
              { tex: `PM = ${tex(x.mg.pm)}^\\circ \\text{ at } \\omega_{co} = ${tex(x.mg.wc)},\\; GM = ${x.mg.crossings.length ? tex(db(x.mg.gm)) + '\\,dB' : '\\infty'},\\; \\omega_{bw} = ${tex(x.bw)}${x.bwN > 1 ? ' \\text{ (first)},\\; ' + tex(x.bwLast) + ' \\text{ (final roll-off)}' : ''}` },
              ...(x.mg.gcs.length > 1 ? [{ html: `|PC| crosses 0 dB ${x.mg.gcs.length} times (the plant resonance pokes back above 0 dB). The PM is the one smallest in magnitude, as python-control's margin reports: ${x.mg.gcs.map((c) => `${fmt(c.pm, 3)}° at ${fmt(c.w, 3)} rad/s`).join('; ')}.` }] : []),
              { html: (x.mg.crossings.length ? '' : 'The dirty derivative makes C(s) biproper, so PC rolls off at −40 dB/dec with the phase approaching −180° from above: no phase crossover, GM = ∞. ')
                + (x.bwN > 1 ? 'The −3 dB "bandwidth" is ambiguous for these gains: |T| dips below −3 dB around √(k<sub>I</sub>/k<sub>D</sub>), where the complex PID zeros make a notch in |C|, then recovers and finally rolls off a little above ω<sub>co</sub>. The final roll-off is the one that relates to crossover.' : 'The bandwidth sits a little above ω<sub>co</sub>, as expected when |PC| ≫ 1 below crossover and ≪ 1 above it (p. 306).') },
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

  CH.ch18 = {
    id: 'ch18', num: 18, tab: 'D.18', title: 'Loopshaping', pages: 'pp. 323–374, p. 383',
    defaults() { return { ...presetStart(), showT: true }; },
    simDefaults(sys) { return { ...sys.problems.ch18.sim, mismatch: sys.problems.ch18.mismatch }; },
    gains() { return {}; },
    linearLabel: 'linear loop (no saturation, d, noise)',

    design(ctx, st = ctx.st) {
      let C = T.gain(st.k);
      for (const key of BLOCKS) if (st[key].on) C = T.mul(C, blocks[key].tf(st[key]));
      const P = plantTf(ctx), Lg = T.mul(P, C);
      const F = st.pf.on ? T.lpf(st.pf.p) : T.gain(1);
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

      const sp = section(parent, 'D.18 specs', 'p. 383');
      const box = el('div', { class: 'metrics' });
      sp.append(box);
      WB.ui.addRefresher(() => {
        const d = this.design(ctx);
        const row = WB.ui.specRow;
        box.replaceChildren(
          row('integrator in C (rejects constant d_in)', d.integOk, d.integOk ? 'yes' : 'no'),
          row('|PC| ≥ 1/0.03 for ω ≤ 0.1', d.lowOk, `min ${fmt(db(d.lowMin), 3)} dB (need ${fmt(db(1 / 0.03), 3)})`),
          row('|PC| ≤ 0.001 for ω ≥ 500', d.highOk, `max ${fmt(db(d.highMax), 3)} dB (need −60)`),
          row('PM ≈ 60° (±5°)', d.pmOk, `${fmt(d.mg.pm, 3)}° at ${fmt(d.mg.wc, 3)} rad/s${d.mg.gcs.length > 1 ? ` (${d.mg.gcs.length} crossovers)` : ''}`),
          row('closed loop stable', d.stable, d.stable ? 'yes' : 'no'),
          WB.ui.metric('gain margin(s)', gmText(d.mg)),
        );
      });
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
        { title: 'Your C(s)', page: 'p. 323', theory: 'C(s) = ' + parts.join('\\cdot ') },
        { title: 'Margins', page: 'p. 303–305',
          theory: `PM = ${tex(d.mg.pm)}^\\circ \\text{ at } \\omega_{co} = ${tex(d.mg.wc)},\\quad GM = ${d.mg.crossings.length ? d.mg.crossings.map((c) => tex(db(c.gm)) + '\\,\\text{dB at } ' + tex(c.w)).join(';\\;') : '\\infty'}` },
        { title: 'Implementation', page: 'p. 334–335 · Eq. 18.3–18.4, p. 336 · Eq. 18.5–18.7',
          theory: '\\dot z_C = A_C z_C + B_C e,\\; F = C_C z_C + D_C e,\\quad \\dot z_F = -p z_F + z_r,\\; z^r_f = p z_F,\\quad e = z^r_f - z' },
      ];
    },

    buildProblem(parent, ctx) {
      lib.panel(parent, ctx, ctx.sys.problems.ch18, [
        { id: 'a', title: 'Meet the loop-gain specs',
          check: () => {
            const d = this.design(ctx);
            const ok = d.integOk && d.lowOk && d.highOk && d.pmOk && d.stable;
            return { ok, msg: `integrator ${d.integOk ? '✓' : '✗'}, tracking ${d.lowOk ? '✓' : '✗'}, noise ${d.highOk ? '✓' : '✗'}, PM ${fmt(d.mg.pm, 3)}° ${d.pmOk ? '✓' : '✗'}, stable ${d.stable ? '✓' : '✗'}` };
          },
          actions: [{ label: 'Load the reference design', run: () => { Object.assign(ctx.st, presetRef(ctx)); ctx.update(); return null; } }],
          solution: () => {
            const r = presetRef(ctx), d = this.design(ctx, { ...ctx.st, ...r });
            return [
              { html: 'One design that works (not unique). The plant is easy at high frequency (|P(j500)| ≈ −122 dB), so the binding specs are tracking (|PC| ≥ 30.5 dB up to 0.1 rad/s) and the phase margin:' },
              { tex: `C(s) = ${tex(r.k)}\\cdot\\frac{s + 0.5}{s}\\cdot 30\\,\\frac{s + 5/\\sqrt{30}}{s + 5\\sqrt{30}}\\cdot\\frac{50}{s + 50},\\quad F(s) = \\frac{1}{s + 1}` },
              { tex: `PM = ${tex(d.mg.pm)}^\\circ \\text{ at } ${tex(d.mg.wc)}\\,\\text{rad/s},\\quad \\min_{\\omega\\le0.1}|PC| = ${tex(db(d.lowMin))}\\,\\text{dB},\\quad \\max_{\\omega\\ge500}|PC| = ${tex(db(d.highMax))}\\,\\text{dB}` },
              { html: 'The PI puts the integrator in C (constant d<sub>in</sub> rejected); crossing over at 5 rad/s with a lead of M = 30 centred there buys the phase and lifts the low-frequency gain above 1/γ<sub>r</sub>; the LPF at 50 rad/s rolls off the lead. Python-control gives the same margins (tools/regress_D.py).' },
            ];
          } },
        { id: 'b', title: 'Prefilter reduces the peaking',
          html: 'Checks the first step of the current simulation: overshoot under 5%.',
          check: () => {
            const res = ctx.app.result(), S = ctx.S;
            const i0 = Math.round(S.sim.tStep / S.sim.Ts), i1 = S.sim.type === 'square' ? Math.round((S.sim.tStep + 0.5 / S.sim.frequency) / S.sim.Ts) : res.t.length;
            const m = M.stepMetrics(res.t, res.y, i0, Math.min(i1, res.t.length), res.y[i0], res.r[Math.min(i0 + 1, res.t.length - 1)]);
            return { ok: m.os < 5, msg: `overshoot ${fmt(m.os, 3)}%` };
          },
          solution: () => [{ html: 'A first-order F = p/(s + p) with p well below crossover (p = 1 in the reference design) shapes the reference so the steps never excite the lead\'s peaking: no overshoot, rise time about 2 s on the ±0.5 m square wave. It does not keep F inside F<sub>max</sub> = 6 N: the 1 m jumps demand about 20 N for a moment (the plant saturates; the PI has no anti-windup but recovers here). p ≈ 0.3 keeps the demand near 6 N at the cost of an 8 s rise time. D.18 itself says nothing about F<sub>max</sub>.' }] },
      ]);
    },
  };

  WB.studies.D.freq = { presetRef, specs, loop, plantTf, pidTf };
})();
