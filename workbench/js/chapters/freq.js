// Chapters 15-18: frequency response, frequency-domain specs, stability margins,
// and loopshaping, all on P(s) = b0 / (s^2 + a1 s + a0) with the A.10 PID.
window.WB = window.WB || {};
WB.chapters = WB.chapters || {};

(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const { tex, texPole, fmt } = M;
  const PD = () => WB.pd;
  const W = T.logspace(-3, 4, 700);
  const db = (m) => 20 * Math.log10(m);
  const gmText = (mg) => (mg.crossings && mg.crossings.length ? mg.crossings.map((c) => `${fmt(db(c.gm), 3)} dB at ${fmt(c.w, 3)}`).join(', ') : '∞');

  const plantTf = (ctx) => T.tf([ctx.model.b0], [1, ctx.model.a1, ctx.model.a0]);
  // C_PID with dirty derivative (p. 313): ((kD + σkP)s² + (kP + σkI)s + kI) / (s(σs + 1))
  const pidTf = ({ kP, kI, kD, sigma }) => T.tf([kD + sigma * kP, kP + sigma * kI, kI], [sigma, 1, 0]);

  // A.10 gains (t_r = 0.6, ζ = 0.9, ω_n = π/(2t_r√(1−ζ²)), k_I = 0.2, σ = 0.05) on the book parameters.
  function a10(sys) {
    const p = Object.fromEntries(sys.params.map((q) => [q.key, q.value]));
    const model = sys.secondOrderModel({ ...p, ...sys.constants });
    const pr = sys.problems.ch10;
    const wn = Math.PI / (2 * pr.tr * Math.sqrt(1 - pr.zeta ** 2));
    return { ...PD().gainsFromPoles(model, PD().polesFromWnZeta(wn, pr.zeta)), kI: pr.ki, sigma: pr.sigma };
  }

  function pidControls(parent, ctx) {
    const sec = section(parent, 'C_PID from A.10', 'p. 161, p. 313');
    slider(sec, { label: 'k<sub>P</sub>', min: 0, max: 2, step: 0.001, get: () => ctx.st.kP, set: (v) => { ctx.st.kP = v; ctx.update(); } });
    slider(sec, { label: 'k<sub>I</sub>', min: 0, max: 2, step: 0.001, get: () => ctx.st.kI, set: (v) => { ctx.st.kI = v; ctx.update(); } });
    slider(sec, { label: 'k<sub>D</sub>', min: 0, max: 0.5, step: 0.0005, get: () => ctx.st.kD, set: (v) => { ctx.st.kD = v; ctx.update(); } });
    slider(sec, { label: 'σ', unit: 's', min: 0.005, max: 0.3, step: 0.001, sig: 3, get: () => ctx.st.sigma, set: (v) => { ctx.st.sigma = v; ctx.update(); } });
    sec.append(el('div', { class: 'btn-row' },
      el('button', { type: 'button', class: 'btn btn-quiet', text: 'A.10 gains (k_I = 0.2, repo)', onclick: () => { Object.assign(ctx.st, a10(ctx.sys)); ctx.update(); } }),
      el('button', { type: 'button', class: 'btn btn-quiet', text: 'book figures (k_I = 0.25)', onclick: () => { Object.assign(ctx.st, a10(ctx.sys), { kI: 0.25 }); ctx.update(); } })));
    return sec;
  }

  function pidDefaults(sys) { return { ...a10(sys), arch: 'output', comp: 'fl', deriv: 'dirty', antiwindup: 'gate', vbar: 0.08 }; }

  function clMarkers(ctx, Lg) {
    const poles = L.roots(L.polyAdd(Lg.den, Lg.num));
    const mk = L.roots(plantTf(ctx).den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of P ${i + 1}` }));
    poles.forEach((p, i) => mk.push({ ...p, kind: 'cl', label: `closed-loop pole ${i + 1}`, noFit: Math.hypot(p.re, p.im) > 40 }));
    return { markers: mk, fitR: 15 };
  }

  function marginMarks(mg, phaseAt) {
    const marks = [];
    if (isFinite(mg.wc)) marks.push({ w: mg.wc, label: `ω_co = ${fmt(mg.wc, 3)}, PM = ${fmt(mg.pm, 3)}°`, phaseFrom: -180, phaseTo: -180 + mg.pm, inPhase: true, color: '--series-1' });
    for (const c of mg.crossings || []) marks.push({ w: c.w, label: `GM ${fmt(db(c.gm), 3)} dB`, dbFrom: 0, dbTo: -db(c.gm), color: '--critical' });
    return marks;
  }

  // ------------------------------------------------------------ Chapter 15 --
  WB.chapters.ch15 = {
    id: 'ch15', num: 15, tab: 'Ch 15', title: 'Frequency response', pages: 'pp. 261–282',
    openLoop: true, metrics: false,
    defaults() { return { comp: 'fl', w0: 2, A: 0.05, asym: true, inp: { shape: 'sine', amp: 0.05, freq: 2 / (2 * Math.PI) } }; },
    simDefaults(sys) { return sys.problems.ch15.sim; },
    controller(ctx, o) {
      ctx.st.inp = { shape: 'sine', amp: ctx.st.A, freq: ctx.st.w0 / (2 * Math.PI), t0: 0 };
      return WB.models.openLoop(ctx, o);
    },
    linearLabel: 'linear model (with transient)',
    outputSeries(ctx, res, deg) {
      const P = plantTf(ctx), g = T.at(P, ctx.st.w0);
      const mag = L.C.abs(g), ph = L.C.arg(g);
      const w = ctx.st.w0, n = res.t.length;
      const pred = Array.from(res.t, (t) => ctx.st.A * mag * Math.sin(w * t + ph));
      // integrator: θ keeps a constant offset set by the initial transient
      const per = Math.max(1, Math.round((2 * Math.PI / w) / ctx.S.sim.Ts));
      let off = 0;
      for (let k = Math.max(0, n - per); k < n; k++) off += res.y[k] - pred[k];
      off /= Math.min(per, n);
      return [{ label: 'A|P(jω₀)| sin(ω₀t + ∠P) + offset', y: deg(pred.map((v) => v + off)), color: '--series-3', dash: [2, 3], width: 2 }];
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Sinusoidal input τ̃ = A sin(ω₀t)', 'p. 264 · Eq. 15.4');
      slider(sec, { label: 'ω<sub>0</sub>', unit: 'rad/s', min: 0.05, max: 100, log: true, sig: 3, get: () => ctx.st.w0, set: (v) => { ctx.st.w0 = v; ctx.update(); } });
      slider(sec, { label: 'A', unit: 'N·m', min: 0, max: 0.3, step: 0.001, sig: 3, get: () => ctx.st.A, set: (v) => { ctx.st.A = v; ctx.update(); } });
      segmented(sec, { label: 'Straight-line approximation', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], get: () => ctx.st.asym, set: (v) => { ctx.st.asym = v; ctx.update(); } });
      sec.append(el('p', { class: 'muted small', text: 'Feedback linearization is on, so θ follows P(s). After the transient, θ is a sinusoid with gain |P(jω₀)| and phase ∠P(jω₀), plus a constant offset from the free integrator.' }));
    },
    bode(ctx) {
      const P = plantTf(ctx), { mag, phase } = T.bode(P, W);
      const lines = [{ label: 'P(jω)', mag, phase, color: '--series-1' }];
      if (ctx.st.asym) {
        const K = ctx.model.b0 / ctx.model.a1, p = ctx.model.a1;
        const am = W.map((w) => (w < p ? K / w : K * p / (w * w)));
        const ap = W.map((w) => (w < p / 10 ? -90 : w > 10 * p ? -180 : -90 - 45 * (Math.log10(w / p) + 1)));
        lines.push({ label: 'straight-line approx.', mag: am, phase: ap, color: '--text-muted', dash: [5, 4], width: 1.5 });
      }
      const g = T.at(P, ctx.st.w0);
      return { title: 'Bode plot of P(s)', w: W, lines, marks: [{ w: ctx.st.w0, label: `ω₀: ${fmt(db(L.C.abs(g)), 3)} dB, ${fmt(L.C.arg(g) * 180 / Math.PI, 3)}°`, color: '--series-3' }] };
    },
    splane(ctx) { return { markers: L.roots(plantTf(ctx).den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of P ${i + 1}` })) }; },
    math(ctx) {
      const m = ctx.model, K = m.b0 / m.a1;
      return [
        { title: 'Frequency response', page: 'p. 264 · Eq. 15.4',
          theory: 'u = A\\sin\\omega_0 t \\;\\Rightarrow\\; y_{ss} = A|P(j\\omega_0)|\\sin\\big(\\omega_0 t + \\angle P(j\\omega_0)\\big)' },
        { title: 'Bode canonical form', page: 'p. 266 · Eq. 15.5–15.7',
          theory: 'P(j\\omega) = \\frac{b_0/a_1}{j\\omega\\,(1 + j\\omega/a_1)},\\quad 20\\log|P| = 20\\log\\tfrac{b_0}{a_1} - 20\\log|j\\omega| - 20\\log|1 + j\\omega/a_1|',
          numbers: `P(j\\omega) = \\frac{${tex(K)}}{j\\omega\\,(1 + j\\omega/${tex(m.a1)})}`, spoiler: true,
          note: 'The A.15 solution (p. 275) writes 44.44/(s(s+0.4444)), which does not match the book\'s own parameters. With m = 0.5, ℓ = 0.3, b = 0.01 the plant is 66.67/(s(s+0.6667)). The Bode constant is 100 either way.' },
        { title: 'Pole building block', page: 'p. 269–270 · Eq. 15.9',
          theory: '\\frac{p}{s + p}:\\; -20\\text{ dB/dec above } p,\\; \\angle = -\\tan^{-1}(\\omega/p) \\text{ from } 0^\\circ \\text{ at } p/10 \\text{ to } -90^\\circ \\text{ at } 10p' },
        { title: 'Integrator', page: 'p. 266–267',
          theory: '\\frac{1}{j\\omega}:\\; -20\\text{ dB/dec through 0 dB at } \\omega = 1,\\; \\angle = -90^\\circ' },
      ];
    },
    buildProblem(parent, ctx) {
      const m = () => ctx.model, P = () => plantTf(ctx);
      const magDb = (w) => db(L.C.abs(T.at(P(), w)));
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch15, [
        {
          id: 'a', title: 'Straight-line pieces',
          inputs: { K: 'Bode constant', wp: 'corner [rad/s]', s1: 'slope below [dB/dec]', s2: 'slope above [dB/dec]' },
          check: (v) => PD().checkNumbers(v, { K: m().b0 / m().a1, wp: m().a1, s1: -20, s2: -40 }, {}),
          solution: () => [{ tex: `P(j\\omega) = \\frac{${tex(m().b0 / m().a1)}}{j\\omega(1 + j\\omega/${tex(m().a1)})}\\;\\Rightarrow\\; -20 \\text{ then } -40\\text{ dB/dec, phase } -90^\\circ \\to -180^\\circ` }],
        },
        {
          id: 'b', title: 'Magnitudes (hw15.py prints these)',
          inputs: { m1: '|P(j0.3)| [dB]', m2: '|P(j10)| [dB]', m3: '|P(j1000)| [dB]' },
          check: (v) => PD().checkNumbers(v, { m1: magDb(0.3), m2: magDb(10), m3: magDb(1000) }, {}),
          solution: () => [{ tex: `${tex(magDb(0.3))},\\; ${tex(magDb(10))},\\; ${tex(magDb(1000))}\\;\\text{dB}` }],
        },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 16 --
  WB.chapters.ch16 = {
    id: 'ch16', num: 16, tab: 'Ch 16', title: 'Frequency-domain specs', pages: 'pp. 283–301',
    defaults(sys) { const p = sys.problems.ch16; return { ...pidDefaults(sys), wr: p.wr, wdin: p.wdin, wno: p.wno, A: p.parabolaA }; },
    simDefaults(sys) { return sys.problems.ch16.sim; },
    gains(ctx) { return { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD }; },
    controller: (ctx, o) => WB.pid.makePID(ctx, o),

    specs(ctx) {
      const P = plantTf(ctx), C = pidTf(ctx.st), Lg = T.mul(P, C), st = ctx.st;
      const Br = db(L.C.abs(T.at(Lg, st.wr)));
      const Bdin = db(L.C.abs(T.at(C, st.wdin)));
      const Bn = -db(L.C.abs(T.at(Lg, st.wno)));
      const Ma = ctx.model.b0 * st.kI / ctx.model.a1;   // lim s^2 P C
      return { Br, gr: Math.pow(10, -Br / 20), Bdin, gdin: Math.pow(10, -Bdin / 20), Bn, gn: Math.pow(10, -Bn / 20), Ma, B2: db(Ma), eParab: 2 * st.A / Ma, eParabBook: st.A / Ma, Lg, P };
    },

    buildControls(parent, ctx) {
      pidControls(parent, ctx);
      const sec = section(parent, 'Spec frequencies', 'p. 284–285');
      slider(sec, { label: 'ω<sub>r</sub>', unit: 'rad/s', min: 0.01, max: 10, log: true, sig: 3, get: () => ctx.st.wr, set: (v) => { ctx.st.wr = v; ctx.update(); } });
      slider(sec, { label: 'ω<sub>d,in</sub>', unit: 'rad/s', min: 0.001, max: 1, log: true, sig: 3, get: () => ctx.st.wdin, set: (v) => { ctx.st.wdin = v; ctx.update(); } });
      slider(sec, { label: 'ω<sub>no</sub>', unit: 'rad/s', min: 10, max: 5000, log: true, sig: 3, get: () => ctx.st.wno, set: (v) => { ctx.st.wno = v; ctx.update(); } });
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const s = this.specs(ctx);
        const show = ctx.S.mode === 'explore' || ctx.app.isRevealed('ch16:specs');
        const row = (l, v) => el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: l }), el('strong', { text: v }));
        box.replaceChildren(...(show ? [
          row(`tracking below ω_r: B_r`, `${fmt(s.Br, 3)} dB → ${fmt(100 * s.gr, 3)} %`),
          row('d_in below ω_d,in: |C|', `${fmt(s.Bdin, 3)} dB → ${fmt(100 * s.gdin, 3)} %`),
          row('noise above ω_no: |PC|', `${fmt(-s.Bn, 3)} dB → ${fmt(100 * s.gn, 3)} %`),
          row('type 2: M_a = lim s²PC', `${fmt(s.Ma, 4)} (${fmt(s.B2, 3)} dB)`),
        ] : [el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reveal the spec readouts', onclick: () => { ctx.app.reveal('ch16:specs'); WB.ui.refreshAll(); } })]));
      });
    },

    bode(ctx) {
      const s = this.specs(ctx), st = ctx.st;
      const P = T.bode(s.P, W), Lb = T.bode(s.Lg, W);
      return {
        title: 'Bode: plant and loop gain', w: W,
        lines: [{ label: 'P(jω)', ...P, color: '--text-muted', width: 1.5 }, { label: 'P(jω)C(jω)', ...Lb, color: '--series-1' }],
        specs: [
          { w0: 1e-3, w1: st.wr, db: s.Br, keep: 'above', color: '--series-3', label: `B_r = ${fmt(s.Br, 3)} dB` },
          { w0: st.wno, w1: 1e4, db: -s.Bn, keep: 'below', color: '--series-3', label: `${fmt(-s.Bn, 3)} dB` },
        ],
        marks: [{ w: st.wdin, label: `ω_d,in: |C| = ${fmt(s.Bdin, 3)} dB` }],
      };
    },
    splane(ctx) { return clMarkers(ctx, T.mul(plantTf(ctx), pidTf(ctx.st))); },

    math(ctx) {
      const s = this.specs(ctx);
      return [
        { title: 'Error with every input', page: 'p. 284 · Eq. 16.2–16.3',
          theory: 'E = \\frac{1}{1+PC}R + \\frac{PC}{1+PC}N + \\frac{1}{1+PC}D_{out} + \\frac{P}{1+PC}D_{in}' },
        { title: 'Tracking', page: 'p. 286 · Eq. 16.4, p. 287 · Eq. 16.5',
          theory: '20\\log|PC| \\ge 20\\log\\tfrac{1}{\\gamma_r} \\text{ for } \\omega \\le \\omega_r,\\quad \\gamma_r = 10^{-B_r/20}',
          numbers: `B_r = ${tex(s.Br)}\\,\\text{dB} \\Rightarrow \\gamma_r = ${tex(s.gr)}`, spoiler: true },
        { title: 'Input disturbance', page: 'p. 290 · Eq. 16.8–16.9',
          theory: '20\\log|PC| - 20\\log|P| = 20\\log|C| \\ge 20\\log\\tfrac{1}{\\gamma_{d_{in}}}',
          numbers: `|C(j\\omega_{d,in})| = ${tex(s.Bdin)}\\,\\text{dB} \\Rightarrow \\gamma_{d_{in}} = ${tex(s.gdin)}`, spoiler: true },
        { title: 'Noise', page: 'p. 287 · Eq. 16.6',
          theory: '20\\log|PC| \\le 20\\log\\gamma_n \\text{ for } \\omega \\ge \\omega_{no}',
          numbers: `|PC(j\\omega_{no})| = ${tex(-s.Bn)}\\,\\text{dB} \\Rightarrow \\gamma_n = ${tex(s.gn)}`, spoiler: true },
        { title: 'Type 2 from the Bode plot', page: 'p. 293–295 · Eq. 16.12',
          theory: 'M_a = \\lim_{\\omega\\to0}|(j\\omega)^2 PC| = 10^{B_2/20},\\quad r = At^2 \\Rightarrow R = \\frac{2A}{s^3},\\; e_{ss} = \\frac{2A}{M_a}',
          numbers: `M_a = \\frac{b_0 k_I}{a_1} = ${tex(s.Ma)}\\;(${tex(s.B2)}\\,\\text{dB}),\\quad e_{ss} = ${tex(s.eParab)}`, spoiler: true,
          note: 'The A.16(b) solution uses A/M_a = 5·10^(−28/20) = 0.2, i.e. treats 5t² as R = 5/s³. Since L{t²} = 2/s³, the error is 2A/M_a.' },
      ];
    },

    buildProblem(parent, ctx) {
      const s = () => this.specs(ctx);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch16, [
        { id: 'a', title: '(a) tracking error below ω<sub>r</sub>', inputs: { v: '%' },
          check: (v) => PD().checkNumbers({ v: v.v }, { v: 100 * s().gr }, { v: 'percent' }),
          solution: () => [{ tex: `B_r = ${tex(s().Br)}\\,\\text{dB},\\; \\gamma_r = ${tex(100 * s().gr)}\\%` }, { html: 'Book (k<sub>I</sub> = 0.25): 44.5 dB, 0.60% (p. 295).' }] },
        { id: 'b', title: '(b) steady-state error to θ<sub>r</sub> = 5t²', inputs: { v: 'e<sub>ss</sub> [rad]' },
          check: (v) => {
            const g = PD().num(v.v), x = s();
            if (g !== null && M.close(g, x.eParabBook, 0.02)) return { ok: false, msg: 'That matches the book (A/M_a), but L{5t²} = 10/s³, so the error is 2A/M_a.' };
            return PD().checkNumbers({ v: v.v }, { v: x.eParab }, { v: 'e_ss' });
          },
          solution: () => [{ tex: `e_{ss} = \\frac{2A}{M_a} = \\frac{10}{${tex(s().Ma)}} = ${tex(s().eParab)}` }, { html: 'Book: 0.2 using A/M<sub>a</sub> with k<sub>I</sub> = 0.25 (p. 296).' }] },
        { id: 'c', title: '(c) % of d<sub>in</sub> below 0.01 rad/s in θ', inputs: { v: '%' },
          check: (v) => PD().checkNumbers({ v: v.v }, { v: 100 * s().gdin }, { v: 'percent' }),
          solution: () => [{ tex: `|C(j0.01)| = ${tex(s().Bdin)}\\,\\text{dB} \\Rightarrow ${tex(100 * s().gdin)}\\%` }, { html: 'Book: 28 dB, 4% (k<sub>I</sub> = 0.25).' }] },
        { id: 'd', title: '(d) % of noise above 100 rad/s in θ', inputs: { v: '%' },
          check: (v) => PD().checkNumbers({ v: v.v }, { v: 100 * s().gn }, { v: 'percent' }),
          solution: () => [{ tex: `|PC(j100)| = ${tex(-s().Bn)}\\,\\text{dB} \\Rightarrow ${tex(100 * s().gn)}\\%` }, { html: 'Book: −32.6 dB, 2.3%.' }] },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 17 --
  function bandwidth(Tc) {
    const { mag } = T.bode(Tc, W);
    for (let i = 0; i < W.length; i++) if (mag[i] < Math.SQRT1_2) return W[i];
    return NaN;
  }

  WB.chapters.ch17 = {
    id: 'ch17', num: 17, tab: 'Ch 17', title: 'Stability margins', pages: 'pp. 303–322',
    defaults(sys) { return pidDefaults(sys); },
    simDefaults(sys) { return sys.problems.ch17.sim; },
    gains(ctx) { return { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD }; },
    controller: (ctx, o) => WB.pid.makePID(ctx, o),

    loop(ctx) {
      const Lg = T.mul(plantTf(ctx), pidTf(ctx.st));
      const Tc = T.feedback(Lg);
      const mg = T.margins(Lg);
      const { mag } = T.bode(Tc, W);
      return { Lg, Tc, mg, bw: bandwidth(Tc), peak: db(Math.max(...mag)) };
    },

    buildControls(parent, ctx) {
      pidControls(parent, ctx);
      const sec = section(parent, 'Margins', 'p. 304–306');
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const l = this.loop(ctx);
        const show = ctx.S.mode === 'explore' || ctx.app.isRevealed('ch17:m');
        const row = (a, b) => el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: a }), el('strong', { text: b }));
        box.replaceChildren(...(show ? [
          row('phase margin', `${fmt(l.mg.pm, 3)}° at ω_co = ${fmt(l.mg.wc, 3)} rad/s`),
          row('gain margin(s)', gmText(l.mg)),
          row('closed-loop bandwidth (−3 dB)', `${fmt(l.bw, 3)} rad/s`),
          row('closed-loop peak |T|', `${fmt(l.peak, 3)} dB`),
        ] : [el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reveal the margins', onclick: () => { ctx.app.reveal('ch17:m'); WB.ui.refreshAll(); } })]));
      });
    },

    bode(ctx) {
      const l = this.loop(ctx);
      const show = ctx.S.mode === 'explore' || ctx.app.isRevealed('ch17:m');
      const marks = show ? marginMarks(l.mg) : [];
      if (show && isFinite(l.bw)) marks.push({ w: l.bw, label: `bandwidth ${fmt(l.bw, 3)}`, color: '--series-2' });
      return {
        title: 'Open loop PC and closed loop PC/(1+PC)', w: W,
        lines: [{ label: 'open loop P(jω)C(jω)', ...T.bode(l.Lg, W), color: '--series-1' }, { label: 'closed loop T(jω)', ...T.bode(l.Tc, W), color: '--series-2', width: 1.5 }],
        marks,
      };
    },
    splane(ctx) { return clMarkers(ctx, this.loop(ctx).Lg); },

    math(ctx) {
      const l = this.loop(ctx);
      return [
        { title: 'Crossover and phase margin', page: 'p. 303–304',
          theory: '|P(j\\omega_{co})C(j\\omega_{co})| = 1,\\quad PM = \\angle P(j\\omega_{co})C(j\\omega_{co}) + 180^\\circ',
          numbers: `\\omega_{co} = ${tex(l.mg.wc)},\\quad PM = ${tex(l.mg.pm)}^\\circ`, spoiler: true },
        { title: 'Gain margin', page: 'p. 306',
          theory: 'GM = \\frac{1}{|PC(j\\omega_{180})|},\\quad \\angle PC(j\\omega_{180}) = -180^\\circ',
          numbers: isFinite(l.mg.gm) ? `GM = ${tex(db(l.mg.gm))}\\,\\text{dB}` : 'GM = \\infty \\;(\\text{phase never reaches } -180^\\circ)', spoiler: true },
        { title: 'Open vs. closed loop', page: 'p. 306–307',
          theory: 'T = \\frac{PC}{1+PC}:\\; |PC| \\gg 1 \\Rightarrow |T| \\approx 1,\\; |PC| \\ll 1 \\Rightarrow |T| \\approx |PC|',
          numbers: `\\omega_{bw} = ${tex(l.bw)}\\;\\text{vs.}\\;\\omega_{co} = ${tex(l.mg.wc)}`, spoiler: true,
          note: 'PM ≈ 60° behaves like ζ ≈ 0.707 (Fig. 17-7). A smaller PM gives peaking in |T| and a bandwidth above ω_co.' },
        { title: 'C_PID with dirty derivative', page: 'p. 313',
          theory: 'C(s) = k_P + \\frac{k_I}{s} + \\frac{k_D s}{\\sigma s + 1} = \\frac{(k_D + \\sigma k_P)s^2 + (k_P + \\sigma k_I)s + k_I}{s(\\sigma s + 1)}',
          numbers: `C(s) = ${T.texTf(pidTf(ctx.st))}` },
        { title: 'Bode gain-phase relationship', page: 'p. 310 · Eq. 17.1',
          theory: '\\text{slope } -20\\text{ dB/dec} \\leftrightarrow -90^\\circ,\\; -40 \\leftrightarrow -180^\\circ:\\; \\text{cross over between } -20 \\text{ and } -40 \\text{ for } PM \\approx 60^\\circ' },
      ];
    },

    buildProblem(parent, ctx) {
      const l = () => this.loop(ctx);
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch17, [
        { id: 'a', title: 'Margins and bandwidth', inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
          check: (v) => { const x = l(); return PD().checkNumbers(v, { pm: x.mg.pm, wc: x.mg.wc, bw: x.bw }, { pm: 'PM', wc: 'ωco', bw: 'ωbw' }); },
          solution: () => { const x = l(); return [{ tex: `PM = ${tex(x.mg.pm)}^\\circ \\text{ at } ${tex(x.mg.wc)},\\; GM = ${isFinite(x.mg.gm) ? tex(db(x.mg.gm)) + '\\,dB' : '\\infty'},\\; \\omega_{bw} = ${tex(x.bw)}` }, { html: 'Book (k<sub>I</sub> = 0.25): PM 49.0° at 10.8 rad/s, GM ∞, bandwidth ≈ 18 rad/s, a bit above crossover because the PM is small (p. 313–314).' }]; } },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 18 --
  const blocks = {
    lead: { label: 'Lead', page: 'p. 328 · Eq. 18.2', tf: (b) => T.tf([b.M, b.M * b.w / Math.sqrt(b.M)], [1, b.w * Math.sqrt(b.M)]) },
    lag: { label: 'Lag', page: 'p. 325 · Eq. 18.1', tf: (b) => T.tf([1, b.z], [1, b.z / b.M]) },
    lpf1: { label: 'Low-pass 1', page: 'p. 325', tf: (b) => T.tf([b.p], [1, b.p]) },
    lpf2: { label: 'Low-pass 2', page: 'p. 325', tf: (b) => T.tf([b.p], [1, b.p]) },
  };

  function presetBook() {
    return { k: 1, lead: { on: true, w: 40, M: 10 }, lag: { on: true, z: 1.5, M: 40 }, lpf1: { on: true, p: 50 }, lpf2: { on: true, p: 150 }, pf: { on: true, p: 3 } };
  }
  function presetNone() {
    return { k: 1, lead: { on: false, w: 10, M: 10 }, lag: { on: false, z: 1, M: 10 }, lpf1: { on: false, p: 100 }, lpf2: { on: false, p: 300 }, pf: { on: false, p: 3 } };
  }
  function presetRepo(ctx) {
    const d = { k: 1, lead: { on: true, w: 10, M: 10 }, lag: { on: true, z: 5, M: 90 }, lpf1: { on: true, p: 90 }, lpf2: { on: true, p: 100 }, pf: { on: true, p: 2 } };
    const C0 = T.mul(pidTf(ctx.st), blocks.lpf1.tf(d.lpf1), blocks.lag.tf(d.lag), blocks.lead.tf(d.lead));
    d.k = 1 / L.C.abs(T.at(T.mul(plantTf(ctx), C0), 6.35));   // loopShaping.py: crossover at 6.35 rad/s
    return d;
  }

  WB.chapters.ch18 = {
    id: 'ch18', num: 18, tab: 'Ch 18', title: 'Loopshaping', pages: 'pp. 323–374',
    defaults(sys) { return { ...pidDefaults(sys), ...presetNone(), showT: true }; },
    simDefaults(sys) { return { ...sys.problems.ch18.sim, mismatch: sys.problems.ch18.mismatch }; },
    gains(ctx) { return { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD }; },
    linearLabel: 'linear loop (no saturation, d, noise)',

    design(ctx) {
      const st = ctx.st;
      let Cl = T.gain(st.k);
      for (const key of Object.keys(blocks)) if (st[key].on) Cl = T.mul(Cl, blocks[key].tf(st[key]));
      const Cp = pidTf(st), P = plantTf(ctx);
      const C = T.mul(Cp, Cl), Lg = T.mul(P, C), Lp = T.mul(P, Cp);
      const F = st.pf.on ? T.tf([st.pf.p], [1, st.pf.p]) : T.gain(1);
      const pr = ctx.sys.problems.ch18;
      const { mag: clMag } = T.bode(Cl, W);
      let lowMin = Infinity, highMax = 0;
      for (let i = 0; i < W.length; i++) {
        if (W[i] <= pr.wLow) lowMin = Math.min(lowMin, clMag[i]);
        if (W[i] >= pr.wHigh) highMax = Math.max(highMax, clMag[i]);
      }
      const mg = T.margins(Lg);
      return { Cl, C, Lg, Lp, F, P, mg, lowOk: lowMin >= pr.factor * 0.999, highOk: highMax <= 1.001 / pr.factor, pmOk: Math.abs(mg.pm - pr.pm) <= 5, lowMin, highMax };
    },

    controller(ctx, { linear = false } = {}) {
      const d = this.design(ctx);
      const Ts = ctx.S.sim.Ts;
      const Cf = T.filter(d.C, Ts), Ff = T.filter(d.F, Ts);
      const { sys, pModel } = ctx;
      return {
        // Listing 18.3 / ctrlLoopshape.py: e = F(r) − y, τ = τ_fl(y) + C(e)
        update(r, x, yMeas) {
          const rf = Ff.step(r);
          const ut = Cf.step(rf - yMeas);
          return linear ? ut : sys.feedbackLinearization([yMeas, 0], pModel) + ut;
        },
      };
    },

    buildControls(parent, ctx) {
      const pre = section(parent, 'Start from', 'p. 340–348');
      pre.append(el('div', { class: 'btn-row' },
        el('button', { type: 'button', class: 'btn', text: 'C_pid only', onclick: () => { Object.assign(ctx.st, presetNone()); ctx.update(); } }),
        el('button', { type: 'button', class: 'btn', text: 'Book text design', title: 'lag 1.5/40, lead 40/10, LPF 50 and 150, prefilter 3', onclick: () => { Object.assign(ctx.st, presetBook()); ctx.update(); } }),
        el('button', { type: 'button', class: 'btn', text: 'Repo loopShaping.py', title: 'LPF 90, lag 5/90, lead 10/10, gain to cross at 6.35, LPF 100, prefilter 2', onclick: () => { Object.assign(ctx.st, presetRepo(ctx)); ctx.update(); } })));
      pre.append(el('p', { class: 'muted small', text: 'C(s) = C_pid(s) · k · lead · lag · LPF · LPF. Both presets use C_pid from A.10 (k_I set in the PID section below).' }));

      const g = section(parent, 'Gain k', 'p. 324');
      slider(g, { label: 'k', min: 0.01, max: 100, log: true, sig: 4, get: () => ctx.st.k, set: (v) => { ctx.st.k = v; ctx.update(); } });

      const onOff = (sec, key) => segmented(sec, { options: [{ value: true, label: 'on' }, { value: false, label: 'off' }], get: () => ctx.st[key].on, set: (v) => { ctx.st[key].on = v; ctx.update(); } });
      const lead = section(parent, 'Lead M(s + ω/√M)/(s + ω√M)', blocks.lead.page);
      onOff(lead, 'lead');
      slider(lead, { label: 'ω<sub>lead</sub>', unit: 'rad/s', min: 0.1, max: 500, log: true, sig: 3, get: () => ctx.st.lead.w, set: (v) => { ctx.st.lead.w = v; ctx.update(); }, disabled: () => !ctx.st.lead.on });
      slider(lead, { label: 'M', min: 1.01, max: 100, log: true, sig: 3, hint: '', get: () => ctx.st.lead.M, set: (v) => { ctx.st.lead.M = v; ctx.update(); }, disabled: () => !ctx.st.lead.on });
      const leadPhase = el('p', { class: 'muted small' });
      lead.append(leadPhase);
      WB.ui.addRefresher(() => { const Mx = ctx.st.lead.M; leadPhase.textContent = `max phase added: sin⁻¹((M−1)/(M+1)) = ${fmt(Math.asin((Mx - 1) / (Mx + 1)) * 180 / Math.PI, 3)}° at ω_lead`; });
      const lag = section(parent, 'Lag (s + z)/(s + z/M)', blocks.lag.page);
      onOff(lag, 'lag');
      slider(lag, { label: 'z', unit: 'rad/s', min: 0.005, max: 50, log: true, sig: 3, get: () => ctx.st.lag.z, set: (v) => { ctx.st.lag.z = v; ctx.update(); }, disabled: () => !ctx.st.lag.on });
      slider(lag, { label: 'M', min: 1.01, max: 200, log: true, sig: 3, get: () => ctx.st.lag.M, set: (v) => { ctx.st.lag.M = v; ctx.update(); }, disabled: () => !ctx.st.lag.on });
      for (const key of ['lpf1', 'lpf2']) {
        const s = section(parent, `${blocks[key].label} p/(s + p)`, blocks[key].page);
        onOff(s, key);
        slider(s, { label: 'p', unit: 'rad/s', min: 1, max: 5000, log: true, sig: 3, get: () => ctx.st[key].p, set: (v) => { ctx.st[key].p = v; ctx.update(); }, disabled: () => !ctx.st[key].on });
      }
      const pf = section(parent, 'Prefilter F(s) = p/(s + p)', 'p. 336 · Eq. 18.5–18.7');
      onOff(pf, 'pf');
      slider(pf, { label: 'p', unit: 'rad/s', min: 0.1, max: 100, log: true, sig: 3, get: () => ctx.st.pf.p, set: (v) => { ctx.st.pf.p = v; ctx.update(); }, disabled: () => !ctx.st.pf.on });
      segmented(pf, { label: 'Bode: closed loop F·T', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], get: () => ctx.st.showT, set: (v) => { ctx.st.showT = v; ctx.update(); } });

      pidControls(parent, ctx);

      const sp = section(parent, 'A.18 specs', 'p. 340–341');
      const box = el('div', { class: 'metrics' });
      sp.append(box);
      WB.ui.addRefresher(() => {
        const d = this.design(ctx);
        const row = (label, ok, v) => el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: label }), el('strong', { text: v }),
          el('span', { class: 'status ' + (ok ? 'good' : 'bad') }, el('span', { class: 'status-icon', 'aria-hidden': 'true', text: ok ? '✓' : '✗' }), el('span', { text: ok ? 'met' : 'not met' })));
        box.replaceChildren(
          row('10× better below 0.07 rad/s (min |C_l|)', d.lowOk, `${fmt(db(d.lowMin), 3)} dB`),
          row('10× better above 1000 rad/s (max |C_l|)', d.highOk, `${fmt(db(d.highMax), 3)} dB`),
          row('PM ≈ 60° (±5°)', d.pmOk, `${fmt(d.mg.pm, 3)}° at ${fmt(d.mg.wc, 3)} rad/s`),
          el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: 'gain margin(s)' }), el('strong', { text: gmText(d.mg) })),
        );
      });
    },

    bode(ctx) {
      const d = this.design(ctx), pr = ctx.sys.problems.ch18;
      const lines = [
        { label: 'P·C_pid', ...T.bode(d.Lp, W), color: '--text-muted', dash: [5, 4], width: 1.5 },
        { label: 'P·C (loop gain)', ...T.bode(d.Lg, W), color: '--series-1' },
      ];
      if (ctx.st.showT) lines.push({ label: 'closed loop F·T', mag: T.bode(T.mul(d.F, T.feedback(d.Lg)), W).mag, color: '--series-3', width: 1.5 });
      const lowDb = db(L.C.abs(T.at(d.Lp, pr.wLow))) + 20 * Math.log10(pr.factor);
      const highDb = db(L.C.abs(T.at(d.Lp, pr.wHigh))) - 20 * Math.log10(pr.factor);
      return {
        title: 'Loopshaping: P·C_pid vs. P·C', w: W, lines,
        specs: [
          { w0: 1e-3, w1: pr.wLow, db: lowDb, keep: 'above', color: '--critical', label: 'tracking / d spec' },
          { w0: pr.wHigh, w1: 1e4, db: highDb, keep: 'below', color: '--critical', label: 'noise spec' },
        ],
        marks: marginMarks(d.mg),
      };
    },
    splane(ctx) { return clMarkers(ctx, this.design(ctx).Lg); },

    math(ctx) {
      const d = this.design(ctx), st = ctx.st;
      const parts = [`${tex(st.k)}`];
      if (st.lead.on) parts.push(T.texTf(blocks.lead.tf(st.lead), 4));
      if (st.lag.on) parts.push(T.texTf(blocks.lag.tf(st.lag), 4));
      if (st.lpf1.on) parts.push(T.texTf(blocks.lpf1.tf(st.lpf1), 4));
      if (st.lpf2.on) parts.push(T.texTf(blocks.lpf2.tf(st.lpf2), 4));
      return [
        { title: 'Building blocks multiply', page: 'p. 323',
          theory: 'C = C_1 C_2 \\cdots C_q:\\quad 20\\log|PC| = 20\\log|P| + \\textstyle\\sum 20\\log|C_i|,\\quad \\angle PC = \\angle P + \\sum \\angle C_i' },
        { title: 'Lag and lead', page: 'p. 325–328 · Eq. 18.1–18.2',
          theory: 'C_{lag} = \\frac{s + z}{s + z/M}\\;(20\\log M \\text{ low-frequency boost}),\\quad C_{lead} = M\\frac{s + \\omega_L/\\sqrt M}{s + \\omega_L\\sqrt M},\\; \\phi_{max} = \\sin^{-1}\\frac{M-1}{M+1}' },
        { title: 'Your C_l(s)', page: 'p. 340',
          theory: 'C_l(s) = ' + parts.join('\\cdot ') },
        { title: 'Margins', page: 'p. 304–306',
          theory: `PM = ${tex(d.mg.pm)}^\\circ \\text{ at } \\omega_{co} = ${tex(d.mg.wc)},\\quad GM = ${d.mg.crossings.length ? d.mg.crossings.map((c) => tex(db(c.gm)) + '\\,\\text{dB at } ' + tex(c.w)).join(';\\;') : '\\infty'}`,
          note: 'Two phase crossings mean conditional stability: the lag drops the phase below −180° at low frequency, so lowering the gain enough would destabilize the loop. MATLAB\'s margin reports the upper crossing (the book\'s +14.7 dB); python-control reports the first one.' },
        { title: 'Implementation', page: 'p. 335 · Eq. 18.3–18.4, p. 336 · Eq. 18.5–18.7',
          theory: '\\dot z_C = A_C z_C + B_C e,\\; u = C_C z_C + D_C e,\\quad \\dot z_F = -p z_F + r,\\; r_f = p z_F,\\quad e = r_f - y' },
      ];
    },

    buildProblem(parent, ctx) {
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch18, [
        { id: 'a', title: '(a) Meet all three specs',
          check: () => { const d = this.design(ctx); const ok = d.lowOk && d.highOk && d.pmOk; return { ok, msg: `low ${d.lowOk ? '✓' : '✗'}, high ${d.highOk ? '✓' : '✗'}, PM ${fmt(d.mg.pm, 3)}° ${d.pmOk ? '✓' : '✗'}` }; },
          solution: () => [{ html: 'Book text design (p. 341–345): lag z = 1.5, M = 40 (PM drops to 41°), lead at 40 rad/s with M = 10, then low-pass filters at 50 and 150 rad/s. Its figures give PM 59.7° at 14.2 rad/s. The printed final C also includes an unexplained (s+0.7)/(s+0.07); the figures were made without it. The repo (Listing 18.4) is a different design: try both presets.' }] },
        { id: 'd', title: '(d) Prefilter removes the overshoot',
          html: 'Checks the first step of the current simulation: overshoot under 5%.',
          check: () => {
            const res = ctx.app.result(), S = ctx.S;
            const i0 = Math.round(S.sim.tStep / S.sim.Ts), i1 = S.sim.type === 'square' ? Math.round((S.sim.tStep + 0.5 / S.sim.frequency) / S.sim.Ts) : res.t.length;
            const m = M.stepMetrics(res.t, res.y, i0, Math.min(i1, res.t.length), res.y[i0], res.r[Math.min(i0 + 1, res.t.length - 1)]);
            return { ok: m.os < 5, msg: `overshoot ${fmt(m.os, 3)}%` };
          },
          solution: () => [{ html: 'The text uses F = 3/(s+3); the listing and Fig. 18-19 use p = 2 (p. 346). Without the prefilter the book reports about 20% overshoot.' }] },
      ]);
    },
  };
})();
