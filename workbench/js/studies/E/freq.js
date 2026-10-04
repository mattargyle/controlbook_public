// Study E, Chapters 15–18: Bode plots of the inner and outer plants (E.15),
// frequency-domain specs (E.16), margins and bandwidth separation (E.17), and
// loopshaping both loops (E.18).
//   P_in = b0/s² (E.5(c)),  P_out = −g/s²
//   C_in = kPθ + kDθ s/(σs + 1) (PD, dirty derivative)
//   C_out = kPz + kIz/s + kDz s/(σs + 1) (PID, dirty derivative)
// Loop gains follow the book's convention L = P·C (p. 313–317), whatever side
// the derivative sits on in the time-domain controller.
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.E = WB.studies.E || { chapters: {} };

(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const { tex, texPole, fmt } = M;
  const E = WB.E;
  const PD = () => WB.pd;
  const CH = WB.studies.E.chapters;
  const W = T.logspace(-4, 4, 800);
  const { db, mag } = T;
  const pct = (x) => `${fmt(100 * x, 3)} %`;

  const Pin = (ctx) => T.tf([ctx.sys.linear(ctx.pModel).b0], [1, 0, 0]);
  const Pout = (ctx) => T.tf([-ctx.pModel.g], [1, 0, 0]);
  const PinFull = (ctx) => { const l = ctx.sys.linear(ctx.pModel); return T.tf([l.b0, 0, 0], [1, 0, 0, 0, -ctx.pModel.m1 * ctx.pModel.g ** 2 / l.De]); };

  const KN = (st) => ({ trTh: st.trTh, zetaTh: st.zetaTh, M: st.M, zetaZ: st.zetaZ, rule: '2.2' });
  // Work mode keeps its own k_Iz (st.kIzW): the Explore default −10⁻⁴ is an answer to
  // E.P.6(c). Its starting value is E.P.6's starting k_I, which is not.
  const KIZ_W0 = -3e-4;
  const kIzKey = (ctx) => (ctx.S.mode === 'work' ? 'kIzW' : 'kIz');
  function loopsOf(ctx) {
    const st = ctx.st;
    const d = E.pdDesign(ctx.pModel, KN(st));
    const g = { ...d, kIz: ctx.S.mode === 'work' ? st.kIzW ?? KIZ_W0 : st.kIz, kIth: st.kIth || 0 };
    const Cin = T.pid({ kP: g.kPth, kI: g.kIth, kD: g.kDth, sigma: st.sigma }), Cout = T.pid({ kP: g.kPz, kI: g.kIz, kD: g.kDz, sigma: st.sigma });
    const Lin = T.mul(Pin(ctx), Cin), Lout = T.mul(Pout(ctx), Cout);
    return { g, Cin, Cout, Lin, Lout, Tin: T.feedback(Lin), Tout: T.feedback(Lout) };
  }

  function knobSection(parent, ctx, title, page) {
    const sec = section(parent, title, page);
    E.knob(sec, ctx, 'trTh', 't<sub>r<sub>θ</sub></sub>', 0.05, 3, 0.005, { unit: 's' });
    E.knob(sec, ctx, 'zetaTh', 'ζ<sub>θ</sub>', 0.3, 1.5, 0.005);
    E.knob(sec, ctx, 'M', 'M = t<sub>r<sub>z</sub></sub>/t<sub>r<sub>θ</sub></sub>', 1.5, 30, 0.1, { sig: 3 });
    E.knob(sec, ctx, 'zetaZ', 'ζ<sub>z</sub>', 0.3, 1.5, 0.005);
    const work = ctx.S.mode === 'work';
    if (work && ctx.st.kIzW === undefined) ctx.st.kIzW = KIZ_W0;
    E.knob(sec, ctx, kIzKey(ctx), 'k<sub>I<sub>z</sub></sub>', -0.1, 0, 0.00001, { sig: 3 });
    E.knob(sec, ctx, 'kIth', 'k<sub>I<sub>θ</sub></sub>', 0, 50, 0.01, { sig: 3 });
    E.knob(sec, ctx, 'sigma', 'σ', 0.005, 0.3, 0.001, { unit: 's', sig: 3 });
    // Work mode: the E.10 sample design is an answer to E.10(b), so its button waits for it.
    const e10 = work && !E.shows(ctx, `${ctx.sys.problems.ch10.id}/b2`);
    sec.append(el('p', { class: 'muted small', text: work
      ? `Defaults: the E.8 specs (t_rθ = 1 s, M = 10, ζ = 0.707), k_Iθ = 0 and σ = 0.05; k_Iz starts at E.P.6's starting value. Set k_Iz yourself (E.P.6(c)).${e10 ? ' The E.10 sample design is available once E.10(b) is solved.' : ''}`
      : 'Defaults: the E.8 specs (t_rθ = 1 s, M = 10, ζ = 0.707) with k_Iz = −10⁻⁴ from E.P.6, k_Iθ = 0 and σ = 0.05. These are not the gains that pass E.10 (see the E.10 solution).' }));
    const specs = work ? { trTh: 1, zetaTh: 0.707, M: 10, zetaZ: 0.707, kIth: 0 } : { trTh: 1, zetaTh: 0.707, M: 10, zetaZ: 0.707, kIz: -1e-4, kIth: 0 };
    sec.append(el('div', { class: 'btn-row' },
      el('button', { type: 'button', class: 'btn btn-quiet', text: 'E.8 specs', onclick: () => { Object.assign(ctx.st, specs); ctx.update(); } }),
      ...(e10 ? [] : [el('button', { type: 'button', class: 'btn btn-quiet', text: 'E.10 sample design', title: 'the robust design from the E.10 solution', onclick: () => { Object.assign(ctx.st, E10_SAMPLE, work ? { kIzW: E10_SAMPLE.kIz } : {}); ctx.update(); } })])));
    if (work) sec.append(el('p', { class: 'muted small', text: FL_NOTE }));
    const show = () => WB.ui.shown(ctx, `E:${ctx.S.chapter}:gains`);
    const box = el('div');
    sec.append(box);
    WB.ui.addRefresher(() => {
      if (show()) {
        const g = loopsOf(ctx).g;
        box.replaceChildren(el('div', { class: 'readout wrap' }, ...[['kPθ', g.kPth], ['kDθ', g.kDth], ['kPz', g.kPz], ['kDz', g.kDz], ['kIz', g.kIz], ['kIθ', g.kIth]].map(([k, v]) => el('div', {}, el('span', { class: 'ro-label', text: k }), el('strong', { text: fmt(v, 4) })))));
      } else {
        box.replaceChildren(WB.ui.revealButton(ctx, `E:${ctx.S.chapter}:gains`, 'Reveal the gains'));
      }
    });
    return sec;
  }
  // Closed-loop poles from the designed gains would give away E.8, so in Work mode
  // they appear only after "Reveal the gains".
  const gainsShown = (ctx) => WB.ui.shown(ctx, `E:${ctx.S.chapter}:gains`);
  function loopToggle(parent, ctx, label = 'Loop') {
    segmented(parent, {
      label, options: [{ value: 'in', label: 'inner (θ)' }, { value: 'out', label: 'outer (z)' }],
      ...bind(ctx, 'loop'),
    });
  }
  const FL_NOTE = 'Work mode: the time plots add F_fl(z) once E.4(c) is solved (Ch 4 tab); until then F = F̃.';
  const E10_SAMPLE = { trTh: 0.15, zetaTh: 0.707, M: 8, zetaZ: 0.707, kIz: -0.05, kIth: 20 };
  const freqDefaults = (extra = {}) => ({ comp: 'fl', trTh: 1, zetaTh: 0.707, M: 10, zetaZ: 0.707, kIz: -1e-4, kIth: 0, sigma: 0.05, loop: 'in', vbar: 0.05, ...extra });
  const nestedCtl = (ctx, o) => E.nestedPID(ctx, loopsOf(ctx).g, { comp: 'fl', meas: 'dirty', sigma: ctx.st.sigma, antiwindup: 'gate', vbar: ctx.st.vbar }, o);
  const metricRow = WB.ui.metric;
  function loopMarkers(ctx, Lg) {
    const mk = [{ re: 0, im: 0, kind: 'ol', label: 'double pole of the plant' }];
    // The dirty-derivative pole near −1/σ would squash everything else; leave it off-scale.
    L.roots(L.polyAdd(Lg.den, Lg.num)).forEach((q, i) => mk.push({ ...q, kind: 'cl', label: `closed-loop pole ${i + 1}`, noFit: Math.hypot(q.re, q.im) > 0.5 / ctx.st.sigma }));
    return { markers: mk };
  }
  const marginMarks = (mg, color, tag) => WB.freq.marginMarks(mg, { color, tag, gm: false });

  // ------------------------------------------------------------ Chapter 15 --
  // E.15(a) and (b) are drawn by hand: in Work mode each plant's Bode plot (and its
  // poles) appears once that part is done.
  const drawnKey = (ctx, inner) => `${ctx.sys.problems.ch15.id}/${inner ? 'a' : 'b'}`;
  const drawn = (ctx, inner) => ctx.S.mode === 'explore' || ctx.app.isSolved(drawnKey(ctx, inner));
  const fullShown = (ctx) => ctx.S.mode === 'explore' || ctx.app.isSolved(`${ctx.sys.problems.ch5.id}/b`);
  CH.ch15 = {
    id: 'ch15', num: 15, tab: 'Ch 15', title: 'Frequency response (inner & outer plants)', pages: 'pp. 261–282',
    defaults() { return freqDefaults({ w0: 2, full: true }); },
    simDefaults(sys) { return sys.problems.ch15.sim; },
    gains(ctx) { return loopsOf(ctx).g; },
    controller: nestedCtl,
    outputSeries: E.pid.thetaRSeries,

    buildControls(parent, ctx) {
      const sec = section(parent, 'Bode plot', 'p. 266 · §15.1.3');
      loopToggle(sec, ctx, 'Plant');
      slider(sec, { label: 'ω<sub>0</sub>', unit: 'rad/s', min: 0.05, max: 100, log: true, sig: 3, ...bind(ctx, 'w0') });
      segmented(sec, { label: 'Inner plant: also show the full E.5(b) model', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'full') });
      sec.append(el('p', { class: 'muted small', text: 'Sketch the straight-line approximation by hand first. In Work mode each plant’s Bode plot appears when you mark that part drawn. The time plots show the E.8 nested loop for reference.' }));
      if (ctx.S.mode === 'work') sec.append(el('p', { class: 'muted small', text: FL_NOTE }));
    },

    bode(ctx) {
      const inner = ctx.st.loop === 'in';
      if (!drawn(ctx, inner)) return null;
      const P = inner ? Pin(ctx) : Pout(ctx);
      const lines = [{ label: inner ? 'P_in(jω) (E.5c)' : 'P_out(jω)', ...T.bode(P, W), color: '--series-1' }];
      if (inner && ctx.st.full && fullShown(ctx)) lines.push({ label: 'full Θ̃/F̃ (E.5b)', ...T.bode(PinFull(ctx), W), color: '--text-muted', dash: [5, 4], width: 1.5 });
      const g = T.at(P, ctx.st.w0);
      const show = WB.ui.shown(ctx, 'E:ch15:w0');
      return { title: inner ? 'Bode: inner plant F̃ → θ̃' : 'Bode: outer plant θ̃ → z̃', w: W, lines,
        marks: [{ w: ctx.st.w0, label: show ? `ω₀: ${fmt(db(L.C.abs(g)), 3)} dB` : 'ω₀', color: '--series-3' }] };
    },
    splane(ctx) {
      const inner = ctx.st.loop === 'in';
      const mk = drawn(ctx, inner) ? [{ re: 0, im: 0, kind: 'ol', label: 'double pole at 0' }] : [];
      if (inner && ctx.st.full && fullShown(ctx)) L.roots(PinFull(ctx).den).forEach((q) => mk.push({ ...q, kind: 'olzero', label: 'pole of the full model' }));
      return { markers: mk, fitR: 1 };
    },

    math(ctx) {
      const b0 = ctx.sys.linear(ctx.pModel).b0, g = ctx.pModel.g, id = ctx.sys.problems.ch15.id;
      return [
        { title: 'Frequency response', page: 'p. 264 · Eq. 15.4',
          theory: 'u = A\\sin\\omega_0 t \\Rightarrow y_{ss} = A|P(j\\omega_0)|\\sin(\\omega_0 t + \\angle P(j\\omega_0))' },
        { title: 'Bode canonical form', page: 'p. 266 · Eq. 15.5–15.7',
          theory: 'P(j\\omega) = K\\,\\frac{\\prod (1 + j\\omega/z_i)}{(j\\omega)^q \\prod (1 + j\\omega/p_i)}:\\quad 20\\log|P| = 20\\log K + \\textstyle\\sum 20\\log|1 + j\\omega/z_i| - 20q\\log\\omega - \\sum 20\\log|1 + j\\omega/p_i|' },
        { title: 'Gain and integrators', page: 'p. 266–267',
          theory: '\\frac{k}{(j\\omega)^n}:\\; 20\\log |k| \\text{ at } \\omega = 1,\\; -20n\\,\\text{dB/dec},\\; \\angle = -90^\\circ n \\;(\\text{plus } 180^\\circ \\text{ if } k < 0)' },
        { title: 'Inner plant of the block and beam', page: 'p. 390 · E.15(a)', answers: `${id}/a`,
          theory: 'P_{in}(j\\omega) = \\frac{b_0}{(j\\omega)^2}:\\; 20\\log|P_{in}| = 20\\log b_0 - 40\\log\\omega,\\; \\angle P_{in} = -180^\\circ',
          numbers: `20\\log b_0 = ${tex(db(b0))}\\,\\text{dB at } \\omega = 1,\\quad 0\\,\\text{dB at } \\omega = \\sqrt{b_0} = ${tex(Math.sqrt(b0))}` },
        { title: 'Outer plant of the block and beam', page: 'p. 391 · E.15(b)', answers: `${id}/b`,
          theory: 'P_{out}(j\\omega) = \\frac{-g}{(j\\omega)^2} = \\frac{g}{\\omega^2}:\\; -40\\,\\text{dB/dec},\\; \\angle P_{out} = 0^\\circ \\;(\\equiv -360^\\circ;\\text{ the minus sign adds } \\pm180^\\circ)',
          numbers: `20\\log g = ${tex(db(g))}\\,\\text{dB at } \\omega = 1,\\quad 0\\,\\text{dB at } \\omega = \\sqrt g = ${tex(Math.sqrt(g))}` },
        { title: 'Full inner model (E.5b)', page: 'p. 387', answers: `${ctx.sys.problems.ch5.id}/b`,
          theory: '\\frac{\\tilde\\Theta}{\\tilde F} = \\frac{b s^2}{s^4 - a}:\\; +40\\,\\text{dB/dec below } \\lambda = a^{1/4},\\; \\to b/s^2 \\text{ above}',
          numbers: `\\lambda = ${tex(Math.pow(ctx.pModel.m1 * g * g / ctx.sys.linear(ctx.pModel).De, 0.25))}\\,\\text{rad/s}`,
          note: 'Dashed in the Bode plot of the inner plant.' },
      ];
    },

    buildProblem(parent, ctx) {
      const b0 = () => ctx.sys.linear(ctx.pModel).b0, g = () => ctx.pModel.g;
      const phaseOk = (v, truths) => { const x = PD().num(v); return x !== null && truths.some((t) => Math.abs(x - t) < 1); };
      const cmpInputs = { m1: '|P(j1)| [dB]', slope: 'slope [dB/dec]', ph: 'phase [°]', wc: '0 dB at ω [rad/s]' };
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch15, [
        {
          id: 'a', title: '(a) Draw by hand the Bode plot of the inner loop transfer function from F̃ to θ̃',
          html: 'On paper: use the transfer function from E.5 (the simplified one E.8 designs with), put it in Bode canonical form, and sketch the straight-line magnitude and phase. Then click the button to compare with the workbench’s Bode plot (Plant: inner).',
          done: 'I have drawn it: next',
          solution: () => [{ tex: `P_{in} = \\frac{${tex(b0())}}{s^2}:\\; ${tex(db(b0()))}\\,\\text{dB at 1 rad/s},\\; -40\\,\\text{dB/dec},\\; -180^\\circ` }],
        },
        {
          id: 'a2', title: '(a) Compare with bode', after: 'a',
          inputs: cmpInputs,
          check: (v) => {
            if (!phaseOk(v.ph, [-180, 180])) return { ok: false, msg: 'Check the phase.' };
            return PD().checkNumbers({ m1: v.m1, slope: v.slope, wc: v.wc }, { m1: db(b0()), slope: -40, wc: Math.sqrt(b0()) }, { m1: '|P(j1)|', slope: 'slope', wc: 'crossing' });
          },
          solution: () => [{ tex: `P_{in} = \\frac{${tex(b0())}}{s^2}:\\; ${tex(db(b0()))}\\,\\text{dB at 1 rad/s},\\; -40\\,\\text{dB/dec},\\; -180^\\circ,\\; 0\\,\\text{dB at } ${tex(Math.sqrt(b0()))}\\,\\text{rad/s}` }],
        },
        {
          id: 'b', title: '(b) Draw by hand the Bode plot of the outer loop transfer function from θ̃ to z̃',
          html: 'On paper, as in (a). Then click the button to compare (Plant: outer).',
          done: 'I have drawn it: next',
          solution: () => [{ tex: `P_{out} = \\frac{-${tex(g())}}{s^2}:\\; ${tex(db(g()))}\\,\\text{dB at 1 rad/s},\\; -40\\,\\text{dB/dec},\\; 0^\\circ\\,(\\text{or } -360^\\circ)` }],
        },
        {
          id: 'b2', title: '(b) Compare with bode', after: 'b',
          inputs: cmpInputs,
          check: (v) => {
            if (!phaseOk(v.ph, [0, -360, 360])) return { ok: false, msg: 'Check the phase: a negative gain over (jω)² is a positive real number at every ω.' };
            return PD().checkNumbers({ m1: v.m1, slope: v.slope, wc: v.wc }, { m1: db(g()), slope: -40, wc: Math.sqrt(g()) }, { m1: '|P(j1)|', slope: 'slope', wc: 'crossing' });
          },
          solution: () => [{ tex: `P_{out} = \\frac{-${tex(g())}}{s^2} = \\frac{${tex(g())}}{\\omega^2}\\big|_{s=j\\omega}:\\; ${tex(db(g()))}\\,\\text{dB},\\; -40\\,\\text{dB/dec},\\; 0^\\circ\\,(\\text{or } -360^\\circ),\\; 0\\,\\text{dB at } ${tex(Math.sqrt(g()))}` }],
        },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 16 --
  CH.ch16 = {
    id: 'ch16', num: 16, tab: 'Ch 16', title: 'Frequency-domain specs', pages: 'pp. 283–301',
    defaults(sys) { return freqDefaults({ ...sys.problems.ch16 }); },
    simDefaults(sys) { return sys.problems.ch16.sim; },
    gains(ctx) { return loopsOf(ctx).g; },
    controller: nestedCtl,
    outputSeries: E.pid.thetaRSeries,

    specs(ctx) {
      const l = loopsOf(ctx), st = ctx.st;
      const S = (Lg, w) => 1 / L.C.abs(L.C.add(L.C.of(1, 0), T.at(Lg, w)));
      return {
        ...l,
        gr: 1 / mag(l.Lin, st.wr), grExact: S(l.Lin, st.wr), Lsin: mag(l.Lout, st.wsin),
        gdin: Math.max(...Array.from(W.filter((w) => w <= st.wdin), (w) => 1 / mag(l.Cin, w))), gdinEdge: 1 / mag(l.Cin, st.wdin),
        gn: mag(l.Lin, st.wno),
        gdout: 1 / mag(l.Lout, st.wdout), gdoutExact: S(l.Lout, st.wdout),
        esin: st.Asin * S(l.Lout, st.wsin), esinApprox: st.Asin / mag(l.Lout, st.wsin),
      };
    },

    buildControls(parent, ctx) {
      knobSection(parent, ctx, 'Gains (default: E.8 specs)', 'p. 387–389');
      const sec = section(parent, 'Spec readouts', 'p. 284–290');
      loopToggle(sec, ctx, 'Bode of');
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const s = this.specs(ctx), st = ctx.st;
        box.replaceChildren(...(WB.ui.shown(ctx, 'E:ch16:specs') ? [
          metricRow(`inner tracking below ${st.wr} rad/s: 1/|L_in|`, pct(s.gr)),
          metricRow(`inner d_in below ${st.wdin} rad/s: 1/k_Pθ`, pct(s.gdin)),
          metricRow(`inner noise above ${st.wno} rad/s: |L_in|`, pct(s.gn)),
          metricRow(`outer d_out below ${st.wdout} rad/s: 1/|L_out|`, pct(s.gdout)),
          metricRow(`outer error to ${st.Asin} sin(${st.wsin}t): A|S|`, fmt(s.esin, 3)),
        ] : [WB.ui.revealButton(ctx, 'E:ch16:specs', 'Reveal the spec readouts')]));
      });
    },

    bode(ctx) {
      const s = this.specs(ctx), st = ctx.st, inner = st.loop === 'in';
      const P = inner ? Pin(ctx) : Pout(ctx), Lg = inner ? s.Lin : s.Lout;
      const show = WB.ui.shown(ctx, 'E:ch16:specs');
      const marks = inner
        ? [{ w: st.wr, label: 'ω_r' }, { w: st.wdin, label: 'ω_d,in' }, { w: st.wno, label: 'ω_no' }]
        : [{ w: st.wdout, label: 'ω_d,out' }, { w: st.wsin, label: '0.6 rad/s' }];
      return {
        title: inner ? 'Inner loop: P_in and P_in·C_in' : 'Outer loop: P_out and P_out·C_out', w: W,
        // The plant's Bode plot answers E.15(a)/(b): in Work mode only once drawn.
        lines: [...(drawn(ctx, inner) ? [{ label: inner ? 'P_in' : 'P_out', ...T.bode(P, W), color: '--text-muted', width: 1.5 }] : []), { label: inner ? 'P_in C_in (PD)' : 'P_out C_out (PID)', ...T.bode(Lg, W), color: '--series-1' }],
        specs: show && inner ? [{ w0: 1e-4, w1: st.wr, db: db(1 / s.gr), keep: 'above', color: '--series-3', label: `${fmt(db(1 / s.gr), 3)} dB` }] : [],
        marks,
      };
    },
    splane(ctx) { return gainsShown(ctx) ? loopMarkers(ctx, ctx.st.loop === 'in' ? loopsOf(ctx).Lin : loopsOf(ctx).Lout) : null; },

    math(ctx) {
      const s = this.specs(ctx), st = ctx.st, id = ctx.sys.problems.ch16.id, e8 = ctx.sys.problems.ch8.id;
      return [
        { title: 'Error with every input', page: 'p. 284 · Eq. 16.2–16.3',
          theory: 'E = \\frac{1}{1+PC}R + \\frac{PC}{1+PC}N + \\frac{1}{1+PC}D_{out} + \\frac{P}{1+PC}D_{in}' },
        { title: 'Specs from the loop gain', page: 'p. 287 · Eq. 16.5, p. 291 · Eq. 16.9, p. 287 · Eq. 16.6',
          theory: '|e| \\le \\gamma_r|r|:\\; \\gamma_r = \\frac{1}{\\min_{\\omega\\le\\omega_r}|PC|},\\quad \\gamma_{d_{in}} = \\max_{\\omega\\le\\omega_d}\\frac{1}{|C|},\\quad \\gamma_n = \\max_{\\omega\\ge\\omega_{no}}|PC|' },
        { title: 'Output disturbances and sinusoids', page: 'p. 289 · Eq. 16.7',
          theory: '\\gamma_{d_{out}} \\approx \\frac{1}{|L(j\\omega_{d})|},\\quad \\text{exact: } |e| = |r|\\,|S(j\\omega)| = \\frac{|r|}{|1 + L(j\\omega)|}' },
        { title: 'Inner loop of the block and beam under PD', page: 'p. 391 · E.16(a–c)', answers: [`${id}/a`, `${id}/b`, `${id}/c`],
          theory: '\\gamma_r = \\frac{1}{|L_{in}(j\\omega_r)|},\\quad \\gamma_{d_{in}} = \\frac{1}{k_{P_\\theta}}\\;(|C_{in}| \\text{ of a PD is smallest at low } \\omega),\\quad \\gamma_n = |L_{in}(j\\omega_{no})|',
          numbers: `\\gamma_r = ${tex(s.gr)}\\;(\\text{exact } |S| = ${tex(s.grExact)}),\\quad \\gamma_{d_{in}} = ${tex(s.gdin)}\\;(1/|C(j${st.wdin})| = ${tex(s.gdinEdge)}),\\quad \\gamma_n = ${tex(s.gn)}` },
        { title: 'Outer loop of the block and beam under PID', page: 'p. 391 · E.16(d, e)', answers: [`${id}/d`, `${id}/e`],
          theory: `\\gamma_{d_{out}} = ${tex(s.gdout)}\\;(\\text{exact } |S| = ${tex(s.gdoutExact)}),\\quad |e| = ${tex(s.esin)}\\;\\text{m}\\;(A/|L| = ${tex(s.esinApprox)},\\; |L(j${st.wsin})| = ${tex(s.Lsin, 2)})` },
        { title: 'Controllers with the dirty derivative', page: 'p. 313',
          theory: 'C_{in} = \\frac{(k_D + \\sigma k_P)s + k_P}{\\sigma s + 1},\\quad C_{out} = \\frac{(k_D + \\sigma k_P)s^2 + (k_P + \\sigma k_I)s + k_I}{s(\\sigma s + 1)}' },
        { title: 'Controllers from the gain knobs', page: 'p. 387 · E.8', answers: [`${e8}/b`, `${e8}/d`],
          theory: `C_{in} = ${T.texTf(s.Cin)},\\quad C_{out} = ${T.texTf(s.Cout)}` },
      ];
    },

    buildProblem(parent, ctx) {
      const s = () => this.specs(ctx);
      const pc = (v, truth, label) => PD().checkNumbers({ v }, { v: 100 * truth }, { v: label });
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch16, [
        { id: 'in', title: 'Inner loop: Bode plots of the plant and of the plant under PD',
          html: 'Select <em>Bode of: inner (θ)</em>. The gains come from the knobs (default: the E.8 specs), with the dirty derivative (σ). In your own code, build P<sub>in</sub> and P<sub>in</sub>C<sub>in</sub> with <code>control.tf</code> and plot both with <code>bode</code>.' },
        { id: 'a', title: '(a) Inner tracking error below 1 rad/s', inputs: { v: '%' },
          check: (v) => {
            const g = PD().num(v.v), x = s();
            if (g !== null && M.close(g, 100 * x.grExact, 0.01)) return { ok: true, msg: `Exact |S(j1)|. The book's method (1/|L|, Eq. 16.5) gives ${fmt(100 * x.gr, 3)}%.` };
            return pc(v.v, x.gr, 'percent');
          },
          solution: () => [{ tex: `|L_{in}(j1)| = ${tex(db(1 / s().gr))}\\,\\text{dB} \\Rightarrow \\gamma_r = ${tex(100 * s().gr)}\\%\\quad(\\text{exact } |S(j1)| = ${tex(100 * s().grExact)}\\%)` }, { html: 'Both are accepted: the book reads 1/|L| off the Bode plot (p. 287, and A.16/B.16 on pp. 295–297).' }] },
        { id: 'b', title: '(b) Input disturbance below 0.8 rad/s, % in θ', inputs: { v: '%' },
          check: (v) => {
            const g = PD().num(v.v), x = s();
            if (g !== null && M.close(g, 100 * x.gdinEdge, 0.02)) return { ok: false, msg: 'That is 1/|C_in| at 0.8 rad/s. For a PD, |C_in| is smallest at low frequency, so the worst case is 1/k_Pθ.' };
            return pc(v.v, x.gdin, 'percent');
          },
          solution: () => [{ tex: `\\frac{P_{in}}{1 + P_{in}C_{in}} \\approx \\frac{1}{C_{in}},\\; |C_{in}(j\\omega)| \\ge k_{P_\\theta} = ${tex(s().g.kPth)} \\Rightarrow \\gamma_{d_{in}} = ${tex(100 * s().gdin)}\\%` }, { html: `At the band edge 1/|C<sub>in</sub>(j0.8)| = ${fmt(100 * s().gdinEdge, 3)}%. See ISSUES.md: the problem does not say which one it wants.` }] },
        { id: 'c', title: '(c) Noise above 300 rad/s, % in θ', inputs: { v: '%' },
          check: (v) => pc(v.v, s().gn, 'percent'),
          solution: () => [{ tex: `|L_{in}(j300)| = ${tex(db(s().gn))}\\,\\text{dB} \\Rightarrow ${tex(100 * s().gn)}\\%` }, { html: 'Above 1/σ = 20 rad/s the dirty-derivative PD flattens at k<sub>P</sub> + k<sub>D</sub>/σ, so |L<sub>in</sub>| falls at −40 dB/dec.' }] },
        { id: 'out', title: 'Outer loop: Bode plots of the plant and of the plant under PID',
          html: 'Select <em>Bode of: outer (z)</em>. The book asks for the E.10 gains; the knobs default to the E.8 specs with a small k<sub>I<sub>z</sub></sub> (the buttons load the E.8 specs or the E.10 sample design). Dirty derivative as above.' },
        { id: 'd', title: '(d) Output disturbance below 0.1 rad/s, % in z', inputs: { v: '%' },
          check: (v) => {
            const g = PD().num(v.v), x = s();
            if (g !== null && M.close(g, 100 * x.gdoutExact, 0.01)) return { ok: true, msg: 'Exact |S(j0.1)|. The book\'s method (1/|L|) gives ' + fmt(100 * x.gdout, 3) + '%.' };
            return pc(v.v, x.gdout, 'percent');
          },
          solution: () => [{ tex: `\\gamma_{d_{out}} \\approx \\frac{1}{|L_{out}(j0.1)|} = ${tex(100 * s().gdout)}\\%\\quad(\\text{exact } |S(j0.1)| = ${tex(100 * s().gdoutExact)}\\%)` }, { html: 'Both are accepted. |L<sub>out</sub>(j0.1)| is only about 14 dB, so the approximation is rough.' }] },
        { id: 'e', title: '(e) Error amplitude for y<sub>r</sub> = 2 sin(0.6t)', inputs: { v: '|e| [m]' },
          check: (v) => {
            const g = PD().num(v.v), x = s();
            if (g !== null && M.close(g, x.esinApprox, 0.01)) return { ok: true, msg: `That is the book's A/|L| method. Here |L(j0.6)| = ${fmt(x.Lsin, 2)} < 1, so it is a poor approximation: the exact error is A/|1 + L| = ${fmt(x.esin, 3)} m.` };
            return PD().checkNumbers({ v: v.v }, { v: x.esin }, { v: '|e|' });
          },
          solution: () => [{ tex: `|e| = \\frac{2}{|1 + L_{out}(j0.6)|} = ${tex(s().esin)}\\;\\text{m}\\quad(\\text{book method } 2/|L| = ${tex(s().esinApprox)},\\; |L(j0.6)| = ${tex(s().Lsin, 2)})` }, { html: 'The outer bandwidth is about 0.45 rad/s, so a 0.6 rad/s reference is mostly not followed. (With the derivative on the output in the time domain, the actual closed loop has no zero from k<sub>D</sub>, which makes tracking a little worse still.)' }] },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 17 --
  CH.ch17 = {
    id: 'ch17', num: 17, tab: 'Ch 17', title: 'Margins & bandwidth separation', pages: 'pp. 303–322',
    defaults() { return freqDefaults({ loop: 'both' }); },
    simDefaults(sys) { return sys.problems.ch17.sim; },
    gains(ctx) { return loopsOf(ctx).g; },
    controller: nestedCtl,
    outputSeries: E.pid.thetaRSeries,

    loops(ctx) {
      const l = loopsOf(ctx);
      const mi = T.margins(l.Lin, -4, 5), mo = T.margins(l.Lout, -4, 5);
      return { ...l, mi, mo, bwIn: E.bandwidth(l.Tin, W), bwOut: E.bandwidth(l.Tout, W) };
    },

    buildControls(parent, ctx) {
      knobSection(parent, ctx, 'Gains (default: E.8 specs)', 'p. 391 · E.17');
      const sec = section(parent, 'Margins', 'p. 303–306');
      segmented(sec, { label: 'Bode shows', options: [{ value: 'both', label: 'both loops' }, { value: 'in', label: 'inner' }, { value: 'out', label: 'outer' }], ...bind(ctx, 'loop') });
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const l = this.loops(ctx);
        const gm = WB.freq.gmText;
        box.replaceChildren(...(WB.ui.shown(ctx, 'E:ch17:m') ? [
          metricRow('inner PM', `${fmt(l.mi.pm, 3)}° at ${fmt(l.mi.wc, 3)} rad/s`),
          metricRow('inner GM', gm(l.mi)),
          metricRow('inner bandwidth', `${fmt(l.bwIn, 3)} rad/s`),
          metricRow('outer PM', `${fmt(l.mo.pm, 3)}° at ${fmt(l.mo.wc, 3)} rad/s`),
          metricRow('outer GM', gm(l.mo)),
          metricRow('outer bandwidth', `${fmt(l.bwOut, 3)} rad/s`),
          metricRow('separation ω_bw,in / ω_bw,out', fmt(l.bwIn / l.bwOut, 3)),
        ] : [WB.ui.revealButton(ctx, 'E:ch17:m', 'Reveal the margins')]));
      });
    },

    bode(ctx) {
      const l = this.loops(ctx), which = ctx.st.loop;
      const show = WB.ui.shown(ctx, 'E:ch17:m');
      const lines = [], marks = [];
      if (which !== 'out') {
        lines.push({ label: 'L_in open', ...T.bode(l.Lin, W), color: '--series-1' }, { label: 'T_in closed', ...T.bode(l.Tin, W), color: '--series-1', dash: [5, 4], width: 1.5 });
        if (show) { marks.push(...marginMarks(l.mi, '--series-1', 'in: ')); if (isFinite(l.bwIn)) marks.push({ w: l.bwIn, label: `bw_in ${fmt(l.bwIn, 3)}`, color: '--series-1' }); }
      }
      if (which !== 'in') {
        lines.push({ label: 'L_out open', ...T.bode(l.Lout, W), color: '--series-3' }, { label: 'T_out closed', ...T.bode(l.Tout, W), color: '--series-3', dash: [5, 4], width: 1.5 });
        if (show) { marks.push(...marginMarks(l.mo, '--series-3', 'out: ')); if (isFinite(l.bwOut)) marks.push({ w: l.bwOut, label: `bw_out ${fmt(l.bwOut, 3)}`, color: '--series-3' }); }
      }
      return { title: 'Open and closed loop, inner and outer', w: W, lines, marks };
    },
    splane(ctx) { return gainsShown(ctx) ? loopMarkers(ctx, ctx.st.loop === 'out' ? loopsOf(ctx).Lout : loopsOf(ctx).Lin) : null; },

    math(ctx) {
      const l = this.loops(ctx), id = ctx.sys.problems.ch17.id;
      return [
        { title: 'Phase and gain margin', page: 'p. 303–306',
          theory: '|L(j\\omega_{co})| = 1,\\; PM = 180^\\circ + \\angle L(j\\omega_{co}),\\quad GM = 1/|L(j\\omega_{180})|' },
        { title: 'Closed-loop bandwidth', page: 'p. 306–307',
          theory: 'T = \\frac{L}{1 + L},\\quad \\omega_{bw}: |T(j\\omega_{bw})| = -3\\,\\text{dB} \\;(\\approx \\omega_{co} \\text{ for } PM \\approx 60^\\circ)' },
        { title: 'Bandwidth separation', page: 'pp. 315–317 (B.17c)',
          theory: '\\frac{\\omega_{bw,in}}{\\omega_{bw,out}} \\gtrsim 5\\text{–}10 \\;\\Rightarrow\\; \\text{the outer loop sees the inner loop as its DC gain}' },
        { title: 'Multiple phase crossings', page: 'p. 303–305',
          theory: '\\text{if the phase crosses } -180^\\circ \\text{ more than once, list every gain-margin crossing (cf. A.18, p. 345)}' },
        { title: 'Margins and bandwidth of both loops', page: 'p. 391–392 · E.17(a, b)', answers: [`${id}/a`, `${id}/b`],
          theory: `PM_{in} = ${tex(l.mi.pm)}^\\circ\\;(\\omega_{co} = ${tex(l.mi.wc)}),\\quad PM_{out} = ${tex(l.mo.pm)}^\\circ\\;(\\omega_{co} = ${tex(l.mo.wc)}),\\quad \\omega_{bw,in} = ${tex(l.bwIn)},\\quad \\omega_{bw,out} = ${tex(l.bwOut)}`,
          note: 'With k_I in the outer loop, L_out ~ −g k_I / s³ at low ω, so its phase heads to −270° and there is a low-frequency crossing with GM < 0 dB (conditionally stable).' },
        { title: 'Bandwidth separation of this design', page: 'p. 392 · E.17(c)', answers: `${id}/c`,
          theory: `\\frac{\\omega_{bw,in}}{\\omega_{bw,out}} = ${tex(l.bwIn / l.bwOut, 3)}` },
      ];
    },

    buildProblem(parent, ctx) {
      const l = () => this.loops(ctx);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch17, [
        { id: 'a', title: '(a) Inner loop margins and bandwidth', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
          check: (v) => { const x = l(); return PD().checkNumbers(v, { pm: x.mi.pm, wc: x.mi.wc, bw: x.bwIn }, { pm: 'PM', wc: 'ωco', bw: 'ωbw' }); },
          solution: () => { const x = l(); return [{ tex: `PM = ${tex(x.mi.pm)}^\\circ \\text{ at } ${tex(x.mi.wc)}\\,\\text{rad/s},\\; GM = \\infty,\\; \\omega_{bw} = ${tex(x.bwIn)}` }, { html: 'The phase of b₀(PD)/s² stays above −180° (the PD adds up to +90°, back to 0° above 1/σ), so GM is infinite. The bandwidth is about 1.5× crossover, as expected for PM ≈ 58° (Fig. 17-7).' }]; } },
        { id: 'b', title: '(b) Outer loop margins and bandwidth', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
          check: (v) => { const x = l(); return PD().checkNumbers(v, { pm: x.mo.pm, wc: x.mo.wc, bw: x.bwOut }, { pm: 'PM', wc: 'ωco', bw: 'ωbw' }); },
          solution: () => { const x = l(); return [{ tex: `PM = ${tex(x.mo.pm)}^\\circ \\text{ at } ${tex(x.mo.wc)}\\,\\text{rad/s},\\; \\omega_{bw} = ${tex(x.bwOut)}` }, { html: `Gain margin: ${x.mo.crossings.map((c) => `${fmt(db(c.gm), 3)} dB at ${fmt(c.w, 3)} rad/s`).join(', ') || '∞'} (conditionally stable; python-control's margin reports this lower crossing).` }]; } },
        { id: 'c', title: '(c) Bandwidth separation', inputs: { r: 'ω<sub>bw,in</sub>/ω<sub>bw,out</sub>' },
          check: (v) => PD().checkNumbers(v, { r: l().bwIn / l().bwOut }, { r: 'ratio' }),
          solution: () => [{ tex: `\\frac{${tex(l().bwIn)}}{${tex(l().bwOut)}} = ${tex(l().bwIn / l().bwOut, 3)}` }, { html: 'About a decade, matching M = 10 in the time domain, so successive loop closure is justified (same reasoning as B.17(c), p. 315).' }] },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 18 --
  const lead = (b) => T.lead(b.M, b.w);
  const lag = (b) => T.lag(b.z, b.M);
  const lpf = (b) => T.lpf(b.p);
  const blankLoop = () => ({ k: 1, pi: { on: false, z: 0.1 }, lead: { on: false, w: 10, M: 10 }, lag: { on: false, z: 1, M: 10 }, lpf: { on: false, p: 100 } });
  function startDesign() {
    return {
      in: { ...blankLoop(), k: 5, lead: { on: true, w: 5, M: 10 } },
      out: { ...blankLoop(), k: 0.01, pi: { on: true, z: 0.05 }, lead: { on: true, w: 0.5, M: 10 } },
      pf: { on: false, p: 1 },
    };
  }
  // A design that meets every E.18 spec (checked by tools/regress_E.py and python-control).
  function sampleInner() { return { ...blankLoop(), k: 135, lead: { on: true, w: 40, M: 20 }, lpf: { on: true, p: 400 } }; }
  function sampleOuter() { return { ...blankLoop(), k: 0.0735, pi: { on: true, z: 0.2 }, lead: { on: true, w: 2, M: 30 }, lpf: { on: true, p: 30 } }; }

  function buildC(b, sign = 1) {
    let C = T.gain(sign * b.k);
    if (b.pi.on) C = T.mul(C, T.pi(b.pi.z));
    if (b.lead.on) C = T.mul(C, lead(b.lead));
    if (b.lag.on) C = T.mul(C, lag(b.lag));
    if (b.lpf.on) C = T.mul(C, lpf(b.lpf));
    return C;
  }
  function lsDesign(ctx) {
    const st = ctx.st, pr = ctx.sys.problems.ch18;
    const Cin = buildC(st.d.in), Cout = buildC(st.d.out, -1);
    const Lin = T.mul(Pin(ctx), Cin), Tin = T.feedback(Lin);
    const Po = T.mul(Pout(ctx), Tin), Lout = T.mul(Po, Cout), Tout = T.feedback(Lout);
    const F = st.d.pf.on ? T.lpf(st.d.pf.p) : T.gain(1);
    const spec = (Lg, s) => {
      const m = T.bode(Lg, W).mag;
      let lo = Infinity, hi = 0;
      for (let i = 0; i < W.length; i++) { if (W[i] <= s.wr) lo = Math.min(lo, m[i]); if (W[i] >= s.wn) hi = Math.max(hi, m[i]); }
      const mg = T.margins(Lg, -4, 5);
      return { lo, hi, mg, lowOk: lo >= 1 / s.gr, highOk: hi <= s.gn, pmOk: Math.abs(mg.pm - s.pm) <= 5 };
    };
    const si = spec(Lin, pr.inner), so = spec(Lout, pr.outer);
    const stableIn = L.roots(L.polyAdd(Lin.den, Lin.num)).every((q) => q.re < 0);
    const stableOut = L.roots(L.polyAdd(Lout.den, Lout.num)).every((q) => q.re < 0);
    const peak = (G) => Math.max(...T.bode(G, W).mag);
    return { Cin, Cout, Lin, Tin, Po, Lout, Tout, F, si, so, stableIn, stableOut, intOk: st.d.out.pi.on, peakT: peak(Tout), peakFT: peak(T.mul(F, Tout)) };
  }

  // The sample designs answer E.18(a)/(b): in Work mode they load once that part is solved.
  const sampleOk = (ctx, part) => E.shows(ctx, `${ctx.sys.problems.ch18.id}/${part}`);
  const SAMPLE_LATER = { ok: false, msg: 'The sample design is an answer to this part: it loads once the part is solved (or in Explore mode).' };

  CH.ch18 = {
    id: 'ch18', num: 18, tab: 'Ch 18', title: 'Loopshaping both loops', pages: 'pp. 323–374',
    defaults() { return { loop: 'in', showT: true, d: startDesign() }; },
    simDefaults(sys) { return sys.problems.ch18.sim; },
    gains() { return {}; },
    linearLabel: 'linear design model',
    outputSeries: E.pid.thetaRSeries,

    // Listing 18.3 style for both loops: θ_r = C_out(F(z̃_r) − z̃), F̃ = C_in(θ_r − θ), F = F_fl(z) + F̃.
    controller(ctx, { linear = false } = {}) {
      const d = lsDesign(ctx), Ts = ctx.S.sim.Ts;
      const Ci = T.filter(d.Cin, Ts), Co = T.filter(d.Cout, Ts), Ff = T.filter(d.F, Ts);
      const { sys, pModel } = ctx, z0 = sys.ze(pModel);
      const ffl = E.ffOf(ctx, 'fl');  // Work mode: once E.4(c) is solved
      return {
        update(r, x, yMeas) {
          const z = yMeas[0], th = yMeas[1];
          const rf = Ff.step(r - z0);
          const thetaR = Co.step(rf - (z - z0));
          const Ft = Ci.step(thetaR - th);
          return { u: (linear ? 0 : ffl(z)) + Ft, thetaR };
        },
      };
    },
    linearSim(ctx, c) {
      const s = ctx.sys, { A, B, C } = s.linear(ctx.pModel, { comp: 'fl' });
      const plant = WB.design.linearPlant(A, B, C, { xe: [s.ze(ctx.pModel), 0, 0, 0] });
      return WB.sim.simulate({ ...c, disturbance: null, noise: null, plant, controller: this.controller(ctx, { linear: true }) });
    },

    buildControls(parent, ctx) {
      const st = ctx.st;
      const top = section(parent, 'Design', 'p. 338–339 · §18.1.9');
      loopToggle(top, ctx, 'Edit and plot');
      top.append(el('div', { class: 'btn-row' },
        el('button', { type: 'button', class: 'btn', text: 'Starting design', onclick: () => { st.d = startDesign(); ctx.update(); } }),
        el('button', { type: 'button', class: 'btn btn-quiet', text: 'Outer: plain PI + lead', onclick: () => { st.d.out = startDesign().out; ctx.update(); } })));
      const which = st.loop === 'out' ? 'out' : 'in';
      const b = st.d[which];
      top.append(el('p', { class: 'muted small', text: which === 'in'
        ? 'C_in = k · lead · lag · LPF on P_in from E.5.'
        : 'C_out = −k · (s + z_I)/s · lead · lag · LPF on P = P_out · T_in. The minus sign makes the loop gain positive (P_out has a negative gain).' }));
      const g = section(parent, which === 'in' ? 'Inner gain k' : 'Outer gain k (applied as −k)', 'p. 324');
      slider(g, { label: 'k', min: which === 'in' ? 0.1 : 1e-4, max: which === 'in' ? 1000 : 10, log: true, sig: 4, ...bind(ctx, 'k', () => b) });
      const onOff = (sec, blk) => WB.ui.onOff(sec, ctx, () => b[blk]);
      if (which === 'out') {
        const pi = section(parent, 'Integrator (s + z_I)/s', 'p. 324–325 · §18.1.2');
        onOff(pi, 'pi');
        slider(pi, { label: 'z<sub>I</sub>', unit: 'rad/s', min: 0.001, max: 10, log: true, sig: 3, ...bind(ctx, 'z', () => b.pi), disabled: () => !b.pi.on });
      }
      const ld = section(parent, 'Lead M(s + ω/√M)/(s + ω√M)', 'p. 328 · Eq. 18.2');
      onOff(ld, 'lead');
      slider(ld, { label: 'ω<sub>lead</sub>', unit: 'rad/s', min: 0.01, max: 1000, log: true, sig: 3, ...bind(ctx, 'w', () => b.lead), disabled: () => !b.lead.on });
      slider(ld, { label: 'M', min: 1.01, max: 200, log: true, sig: 3, ...bind(ctx, 'M', () => b.lead), disabled: () => !b.lead.on });
      const lp = el('p', { class: 'muted small' });
      ld.append(lp);
      WB.ui.addRefresher(() => { lp.textContent = `max phase added: ${fmt(Math.asin((b.lead.M - 1) / (b.lead.M + 1)) * 180 / Math.PI, 3)}° at ω_lead`; });
      const lg = section(parent, 'Lag (s + z)/(s + z/M)', 'p. 325 · Eq. 18.1');
      onOff(lg, 'lag');
      slider(lg, { label: 'z', unit: 'rad/s', min: 0.001, max: 100, log: true, sig: 3, ...bind(ctx, 'z', () => b.lag), disabled: () => !b.lag.on });
      slider(lg, { label: 'M', min: 1.01, max: 200, log: true, sig: 3, ...bind(ctx, 'M', () => b.lag), disabled: () => !b.lag.on });
      const lf = section(parent, 'Low-pass p/(s + p)', 'p. 325');
      onOff(lf, 'lpf');
      slider(lf, { label: 'p', unit: 'rad/s', min: 1, max: 5000, log: true, sig: 3, ...bind(ctx, 'p', () => b.lpf), disabled: () => !b.lpf.on });
      const pf = section(parent, 'Prefilter F(s) = p/(s + p) on z̃_r', 'p. 336 · Eq. 18.5–18.7');
      WB.ui.onOff(pf, ctx, () => st.d.pf);
      slider(pf, { label: 'p', unit: 'rad/s', min: 0.05, max: 50, log: true, sig: 3, ...bind(ctx, 'p', () => st.d.pf), disabled: () => !st.d.pf.on });
      segmented(pf, { label: 'Bode: closed loop', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], ...bind(ctx, 'showT', () => st) });

      const sp = section(parent, 'E.18 specs', 'p. 392');
      const box = el('div', { class: 'metrics' });
      sp.append(box);
      WB.ui.addRefresher(() => {
        const d = lsDesign(ctx), pr = ctx.sys.problems.ch18;
        const row = WB.ui.specRow;
        box.replaceChildren(
          row(`inner |L| ≥ ${fmt(db(1 / pr.inner.gr), 3)} dB below ${pr.inner.wr} rad/s`, d.si.lowOk, `${fmt(db(d.si.lo), 3)} dB`),
          row(`inner |L| ≤ ${fmt(db(pr.inner.gn), 3)} dB above ${pr.inner.wn} rad/s`, d.si.highOk, `${fmt(db(d.si.hi), 3)} dB`),
          row('inner PM ≈ 60° (±5°)', d.si.pmOk && d.stableIn, `${fmt(d.si.mg.pm, 3)}° at ${fmt(d.si.mg.wc, 3)}`),
          row(`outer |L| ≥ ${fmt(db(1 / pr.outer.gr), 3)} dB below ${pr.outer.wr} rad/s`, d.so.lowOk, `${fmt(db(d.so.lo), 3)} dB`),
          row(`outer |L| ≤ ${fmt(db(pr.outer.gn), 3)} dB above ${pr.outer.wn} rad/s`, d.so.highOk, `${fmt(db(d.so.hi), 3)} dB`),
          row('outer PM ≈ 60° (±5°)', d.so.pmOk && d.stableOut, `${fmt(d.so.mg.pm, 3)}° at ${fmt(d.so.mg.wc, 3)}`),
          row('integrator in C_out (input d)', d.intOk, d.intOk ? 'yes' : 'no'),
        );
      });
    },

    bode(ctx) {
      const d = lsDesign(ctx), pr = ctx.sys.problems.ch18, inner = ctx.st.loop !== 'out';
      const s = inner ? pr.inner : pr.outer, mg = inner ? d.si.mg : d.so.mg;
      const lines = [
        // The plant answers E.15(a)/(b): in Work mode only once that part is drawn.
        ...(drawn(ctx, inner) ? [{ label: inner ? 'P_in' : 'P = P_out·T_in', ...T.bode(inner ? Pin(ctx) : d.Po, W), color: '--text-muted', dash: [5, 4], width: 1.5 }] : []),
        { label: inner ? 'L_in = P_in C_in' : 'L_out = P C_out', ...T.bode(inner ? d.Lin : d.Lout, W), color: '--series-1' },
      ];
      if (ctx.st.showT) lines.push({ label: inner ? 'T_in' : 'F·T_out', mag: T.bode(inner ? d.Tin : T.mul(d.F, d.Tout), W).mag, color: '--series-3', width: 1.5 });
      return {
        title: inner ? 'Inner loop loopshaping' : 'Outer loop loopshaping', w: W, lines,
        specs: [
          { w0: 1e-4, w1: s.wr, db: db(1 / s.gr), keep: 'above', color: '--critical', label: 'tracking spec' },
          { w0: s.wn, w1: 1e4, db: db(s.gn), keep: 'below', color: '--critical', label: 'noise spec' },
        ],
        marks: marginMarks(mg),
      };
    },
    splane(ctx) {
      const d = lsDesign(ctx), Lg = ctx.st.loop === 'out' ? d.Lout : d.Lin;
      const cl = L.roots(L.polyAdd(Lg.den, Lg.num));
      const R = ctx.st.loop === 'out' ? 8 : 120;
      // The design model's poles at 0 answer E.5(c).
      const mk = E.shows(ctx, `${ctx.sys.problems.ch5.id}/c`) ? [{ re: 0, im: 0, kind: 'ol', label: 'plant poles at 0' }] : [];
      cl.forEach((q, i) => mk.push({ ...q, kind: 'cl', label: `closed-loop pole ${i + 1}`, noFit: Math.hypot(q.re, q.im) > R }));
      return { markers: mk, fitR: Math.min(R, Math.max(1, ...cl.filter((q) => Math.hypot(q.re, q.im) <= R).map((q) => Math.hypot(q.re, q.im)))) };
    },

    math(ctx) {
      const d = lsDesign(ctx);
      return [
        { title: 'Successive loop closure with loopshaping', page: 'p. 338–339 · §18.1.9',
          theory: 'T_{in} = \\frac{P_{in}C_{in}}{1 + P_{in}C_{in}},\\quad P = P_{out}T_{in},\\quad L_{out} = P\\,C_{out}' },
        { title: 'Specs as Bode bounds', page: 'p. 287 · Eq. 16.5, p. 287 · Eq. 16.6',
          theory: '|L(j\\omega)| \\ge \\frac{1}{\\gamma_r},\\; \\omega \\le \\omega_r;\\quad |L(j\\omega)| \\le \\gamma_n,\\; \\omega \\ge \\omega_n;\\quad \\gamma = 0.0032 \\leftrightarrow 49.9\\,\\text{dB}' },
        { title: 'Your controllers', page: 'p. 323',
          theory: `C_{in} = ${T.texTf(d.Cin, 3)}`,
          numbers: `C_{out} = ${T.texTf(d.Cout, 3)}` },
        { title: 'Closed-loop peaking and the prefilter', page: 'p. 336 · Eq. 18.5–18.7',
          theory: '\\max|T_{out}| \\text{ vs. } \\max|F\\,T_{out}|',
          numbers: `${tex(db(d.peakT))}\\,\\text{dB} \\to ${tex(db(d.peakFT))}\\,\\text{dB}` },
        { title: 'Implementation', page: 'p. 335 · Eq. 18.3–18.4',
          theory: '\\dot x_C = A_Cx_C + B_Ce,\\; u = C_Cx_C + D_Ce \\;(\\text{both loops, RK4 at } T_s),\\quad F = F_{fl}(z) + \\tilde F' },
      ];
    },

    buildProblem(parent, ctx) {
      const st = ctx.st;
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch18, [
        {
          id: 'a', title: '(a) Inner loop meets its specs',
          check: () => { const d = lsDesign(ctx), s = d.si; const ok = s.lowOk && s.highOk && s.pmOk && d.stableIn; return { ok, msg: `low ${s.lowOk ? '✓' : '✗'}, high ${s.highOk ? '✓' : '✗'}, PM ${fmt(s.mg.pm, 3)}° ${s.pmOk ? '✓' : '✗'}` }; },
          actions: [{ label: 'Load sample inner design', run: () => { if (!sampleOk(ctx, 'a')) return SAMPLE_LATER; st.d.in = sampleInner(); st.loop = 'in'; ctx.update(); return null; } }],
          solution: () => [
            { html: 'P<sub>in</sub> = b₀/s² has −40 dB/dec and −180° everywhere, so a gain alone gives PM = 0. To get +49.9 dB at 1 rad/s and −49.9 dB at 1000 rad/s, the loop needs about 100 dB over three decades: it can cross over near 40 rad/s with phase lead there, then roll off.' },
            { html: 'Sample: lead at ω = 40 rad/s with M = 20 (+64.8° max), a low-pass at 400 rad/s, and k = 135 so the loop crosses at 40 rad/s. PM ≈ 59°, |L(j1)| ≈ 51.1 dB, |L(j1000)| ≈ −51.6 dB, inner bandwidth ≈ 66 rad/s (python-control agrees).' },
          ],
        },
        {
          id: 'b', title: '(b) Outer loop meets its specs, prefilter reduces peaking',
          html: 'Passes when the three outer specs hold, C<sub>out</sub> has an integrator, and max|F·T<sub>out</sub>| ≤ 0.5 dB.',
          check: () => {
            const d = lsDesign(ctx), s = d.so;
            const ok = s.lowOk && s.highOk && s.pmOk && d.stableOut && d.intOk && db(d.peakFT) <= 0.5;
            return { ok, msg: `low ${s.lowOk ? '✓' : '✗'}, high ${s.highOk ? '✓' : '✗'}, PM ${fmt(s.mg.pm, 3)}°, integrator ${d.intOk ? '✓' : '✗'}, peak |FT| ${fmt(db(d.peakFT), 3)} dB` };
          },
          actions: [{ label: 'Load sample outer design', run: () => { if (!sampleOk(ctx, 'b')) return SAMPLE_LATER; st.d.in = sampleInner(); st.d.out = sampleOuter(); st.d.pf = { on: true, p: 0.7 }; st.loop = 'out'; ctx.update(); return null; } }],
          solution: () => [
            { html: 'An integrator (s + z<sub>I</sub>)/s rejects constant input disturbances. With −g/s² that is three integrators (−270°), so the loop needs strong lead at crossover. Sample: −k (s + 0.2)/s · lead(ω = 2, M = 30) · 30/(s + 30), k = 0.0735: PM ≈ 59.7° at 2 rad/s, |L(j0.1)| ≈ 44.5 dB, |L(j100)| ≈ −71 dB. Outer bandwidth ≈ 3.3 rad/s, about 20× below the inner loop.' },
            { html: 'Unfiltered, T<sub>out</sub> peaks at about 2 dB (21% overshoot). A prefilter F = 0.7/(s + 0.7) brings the peak to 0 dB, at the cost of a slower rise.' },
          ],
        },
        {
          id: 'c', title: '(c) Implemented in simulation',
          html: 'Passes when the simulated |z<sub>r</sub> − z| just before the first switch is under 2 mm and is under 5 mm again just before t<sub>end</sub>. Set d = 0.5 N (starting at 20 s) in the left panel to test the input-disturbance rejection too.',
          check: () => {
            const res = ctx.app.result(), i = E.beforeSwitch(ctx, res), n = res.t.length - 1 - Math.round(0.05 / ctx.S.sim.Ts);
            const e = Math.abs(res.rAll[0][i] - res.yAll[0][i]), eEnd = Math.abs(res.rAll[0][n] - res.yAll[0][n]);
            const ok = e < 0.002 && eEnd < 0.005 && isFinite(eEnd);
            if (!E.ffApplied(ctx, 'fl')) return { ok: false, msg: 'F_fl(z) is applied once E.4(c) is solved (Ch 4 tab).' };
            return { ok, msg: `error ${fmt(1000 * e, 3)} mm before the first switch, ${fmt(1000 * eEnd, 3)} mm just before t_end (d = ${fmt(ctx.S.sim.dist, 3)} N).` };
          },
          solution: () => [{ html: 'Both C(s) and F(s) run as state-space filters (controllable canonical form) integrated with RK4 at T<sub>s</sub>, with substeps for the fast inner-loop poles. The sample design tracks the square wave and rejects the 0.5 N step through the outer integrator.' }],
        },
      ]);
    },
  };

  WB.E.freq = { loopsOf, lsDesign, sampleInner, sampleOuter, Pin, Pout };
})();
