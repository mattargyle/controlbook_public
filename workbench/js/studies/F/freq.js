// Study F, Chapters 15–18 (F.15–F.18): Bode plots of the three loop plants,
// frequency-domain specs and stability margins with the F.10 gains, and
// loopshaping of the altitude, inner (θ) and outer (z) loops.
//
// Plants: P_lon = (1/M)/s², P_in = (1/J)/s², P_out = −g/(s(s + μ/M)) (inner loop as
// its DC gain, as in F.8). Ch 18 designs the outer loop on P_out·T_in instead,
// with the current inner design, so the loops really are designed in succession.
(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const T = WB.tf;
  const F = WB.F;
  const { tex, texPole, fmt } = M;
  const PD = () => WB.pd;
  const W = T.logspace(-3, 4, 700);
  const db = (m) => 20 * Math.log10(m);
  const ALL = ['kPh', 'kDh', 'kIh', 'kPth', 'kDth', 'kPz', 'kDz', 'kIz'];
  const LOOPS = { lon: 'altitude', inner: 'inner θ', outer: 'outer z' };

  function plants(p) {
    const m = WB.systems.F.models(p);
    return {
      m,
      lon: T.tf([m.lon.b0], [1, 0, 0]),
      inner: T.tf([m.inner.b0], [1, 0, 0]),
      outer: T.tf([m.outer.b0], [1, m.a, 0]),
    };
  }
  // C_PID with dirty derivative (p. 313); kI = 0 gives PD.
  const pidTf = (kP, kI, kD, sigma) => (kI
    ? T.tf([kD + sigma * kP, kP + sigma * kI, kI], [sigma, 1, 0])
    : T.tf([kD + sigma * kP, kP], [sigma, 1]));
  const abs = (G, w) => L.C.abs(T.at(G, w));
  // Frequency where |G| falls through level, refined by bisection on log ω
  // between grid points (the 700-point grid alone is only good to ~2%).
  function crossDown(G, level, wa, wb) {
    let lo = Math.log(wa), hi = Math.log(wb);
    for (let k = 0; k < 60; k++) { const mid = 0.5 * (lo + hi); if (abs(G, Math.exp(mid)) >= level) lo = mid; else hi = mid; }
    return Math.exp(0.5 * (lo + hi));
  }
  // closed-loop bandwidth: first drop below −3 dB
  function bandwidth(Tc) {
    const { mag } = T.bode(Tc, W);
    for (let i = 1; i < W.length; i++) if (mag[i] < Math.SQRT1_2) return crossDown(Tc, Math.SQRT1_2, W[i - 1], W[i]);
    return NaN;
  }
  const gmText = (mg) => (mg.crossings && mg.crossings.length ? mg.crossings.map((c) => `${fmt(db(c.gm), 3)} dB at ${fmt(c.w, 3)}`).join(', ') : '∞');
  function marginMarks(mg, color = '--series-1') {
    const marks = [];
    if (isFinite(mg.wc)) marks.push({ w: mg.wc, label: `ω_co = ${fmt(mg.wc, 3)}, PM = ${fmt(mg.pm, 3)}°`, phaseFrom: -180, phaseTo: -180 + mg.pm, inPhase: true, color });
    for (const c of mg.crossings || []) marks.push({ w: c.w, label: `GM ${fmt(db(c.gm), 3)} dB`, dbFrom: 0, dbTo: -db(c.gm), color: '--critical' });
    return marks;
  }

  // ---------------------------------------------- gains for Ch 16 and 17 --
  // 'mine': your F.10 Work-mode gains (Ch 10 tab). 'ref': the reference F.10 design.
  function f10Gains(ctx) {
    if (ctx.st.src === 'ref' || ctx.S.mode === 'explore') return F.refF10(ctx.pModel);
    const w = ctx.S.ch.ch10 ? ctx.S.ch.ch10.w : F.W0;
    return Object.fromEntries(ALL.map((k) => [k, w[k] || 0]));
  }
  function srcControls(parent, ctx) {
    const sec = section(parent, 'Gains', 'F.10 p. 399');
    segmented(sec, {
      label: 'Use',
      options: [{ value: 'mine', label: 'my F.10 gains (Ch 10 tab)' }, { value: 'ref', label: 'reference F.10 design' }],
      get: () => (ctx.S.mode === 'explore' ? 'ref' : ctx.st.src), set: (v) => { ctx.st.src = v; ctx.update(); },
    });
    slider(sec, { label: 'σ', unit: 's', min: 0.005, max: 0.3, step: 0.001, sig: 3, get: () => ctx.st.sigma, set: (v) => { ctx.st.sigma = v; ctx.update(); } });
    F.readout(sec, ctx, ALL.map((k) => ({ key: k, label: k })), () => f10Gains(ctx));
    sec.append(el('p', { class: 'muted small', text: ctx.S.mode === 'work' ? 'Answers below are computed from the gains selected here, so you can check your own design. The reference design reveals the F.8/F.10 gains.' : 'Explore mode uses the reference F.10 design.' }));
    return sec;
  }
  // Loop gains with the dirty derivative. Outer: kIz included unless pdOuter.
  function loopsOf(ctx, g = f10Gains(ctx), { pdOuter = false } = {}) {
    const P = plants(ctx.pModel), sg = ctx.st.sigma;
    const kDC = F.kDCof(P.m, g);
    const Cl = pidTf(g.kPh, g.kIh, g.kDh, sg), Ci = pidTf(g.kPth, 0, g.kDth, sg), Co = pidTf(g.kPz, pdOuter ? 0 : g.kIz, g.kDz, sg);
    const Li = T.mul(P.inner, Ci);
    const Ti = T.feedback(Li);
    const Pout = ctx.st.innerDyn ? T.mul(P.outer, Ti) : T.mul(P.outer, T.gain(kDC));
    return { P, Cl, Ci, Co, Ll: T.mul(P.lon, Cl), Li, Ti, Lo: T.mul(Pout, Co), Pout };
  }
  const f10Defaults = (extra = {}) => ({ comp: 'eq', lat: 'on', view: 'lon', zOff: 3, hOff: 0, deriv: 'dirty', sigma: 0.05, antiwindup: 'gate', vbarH: 0.5, vbarZ: 0.5, src: 'mine', innerDyn: false, ...extra });

  // ------------------------------------------------------------ Chapter 15 --
  // Open-loop sinusoid around hover with the initial state set to the steady
  // sinusoid, so the double integrators do not drift.
  function sineSim(ctx, common) {
    const s = ctx.sys, p = ctx.pModel, st = ctx.st, m = s.models(p);
    const w = st.w0, A = st.loop === 'lon' ? st.AF : st.AT;
    const isLon = st.loop === 'lon';
    const x0 = common.x0.slice();
    x0[2] = 0; x0[3] = 0; x0[4] = 0; x0[5] = 0;
    if (isLon) x0[4] = -A / (m.M * w);
    else {
      x0[5] = -A / (m.J * w);
      // θ_ss = −a sin ωt, a = A/(Jω²); z_ss = Re{P_z(jω) · (−a) e^{jωt}} written as amplitude/phase
      const a = A / (m.J * w * w);
      const Pz = T.at(plants(p).outer, w);
      const mag = L.C.abs(Pz) * a, ph = L.C.arg(Pz) + Math.PI;
      x0[0] = (common.x0[0] || 0) + mag * Math.sin(ph);
      x0[3] = mag * w * Math.cos(ph);
    }
    const ctrl = {
      update(r, x, y, t) {
        const u = A * Math.sin(w * t);
        return { u: isLon ? s.mix(m.Fe + u, 0, p) : s.mix(m.Fe, u, p) };
      },
    };
    return F.simulate(ctx, { ...common, x0 }, null, ctrl);
  }

  F.register({
    id: 'ch15', num: 15, tab: 'Ch 15', title: 'Frequency response', pages: 'pp. 261–282, F.15 p. 401',
    openLoop: true, metrics: false, linear: false,
    defaults() { return { loop: 'lon', w0: 1, AF: 0.5, AT: 0.005, asym: true, zOff: 0, hOff: 0 }; },
    simDefaults(sys) { return { ...sys.problems.ch15.sim, y0: 2 }; },
    simulate(ctx, common) { ctx.chapter = this; return sineSim(ctx, common); },
    reference: () => () => [NaN, 0],
    controller() { return { update: () => [0, 0] }; },
    outputSeries(ctx, res, sc, oi) {
      const P = plants(ctx.pModel), w = ctx.st.w0, A = ctx.st.loop === 'lon' ? ctx.st.AF : ctx.st.AT, m = P.m;
      const mk = (G, amp, off, phase0 = 0) => {
        const g = T.at(G, w), mag = L.C.abs(g), ph = L.C.arg(g);
        return Array.from(res.t, (t) => off + amp * mag * Math.sin(w * t + ph + phase0));
      };
      if (ctx.st.loop === 'lon' && oi === 1) return [{ label: 'A|P(jω₀)| sin(ω₀t + ∠P)', y: sc(mk(P.lon, A, res.yAll[1][0])), color: '--series-3', dash: [2, 3], width: 2 }];
      if (ctx.st.loop !== 'lon' && oi === 2) return [{ label: 'A|P_in| sin(ω₀t + ∠P_in)', y: sc(mk(P.inner, A, 0)), color: '--series-3', dash: [2, 3], width: 2 }];
      if (ctx.st.loop === 'outer' && oi === 0) {
        const a = A / (m.J * w * w);
        return [{ label: 'a|P_out| sin(ω₀t + π + ∠P_out)', y: sc(mk(P.outer, a, res.yAll[0][0] - (() => { const g = T.at(P.outer, w); return L.C.abs(g) * a * Math.sin(L.C.arg(g) + Math.PI); })(), Math.PI)), color: '--series-3', dash: [2, 3], width: 2 }];
      }
      return [];
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Plant and sinusoid', 'p. 264 · Eq. 15.4');
      segmented(sec, { label: 'Transfer function', options: [{ value: 'lon', label: 'F̃ → h (a)' }, { value: 'inner', label: 'τ → θ (b)' }, { value: 'outer', label: 'θ → z (c)' }], get: () => ctx.st.loop, set: (v) => { ctx.st.loop = v; ctx.update(); } });
      slider(sec, { label: 'ω<sub>0</sub>', unit: 'rad/s', min: 0.05, max: 50, log: true, sig: 3, get: () => ctx.st.w0, set: (v) => { ctx.st.w0 = v; ctx.update(); } });
      slider(sec, { label: 'A (F̃)', unit: 'N', min: 0, max: 3, step: 0.01, sig: 3, get: () => ctx.st.AF, set: (v) => { ctx.st.AF = v; ctx.update(); }, disabled: () => ctx.st.loop !== 'lon' });
      slider(sec, { label: 'A (τ)', unit: 'N·m', min: 0, max: 0.05, step: 0.0001, sig: 3, get: () => ctx.st.AT, set: (v) => { ctx.st.AT = v; ctx.update(); }, disabled: () => ctx.st.loop === 'lon' });
      segmented(sec, { label: 'Straight-line approximation', options: [{ value: true, label: 'show' }, { value: false, label: 'hide' }], get: () => ctx.st.asym, set: (v) => { ctx.st.asym = v; ctx.update(); } });
      sec.append(el('p', { class: 'muted small', text: 'Open loop around hover, started on the steady-state sinusoid so the double integrators do not drift. θ → z is driven through τ: the θ sinusoid then moves z. Dotted traces: the Bode prediction.' }));
    },
    bode(ctx) {
      const P = plants(ctx.pModel), G = P[ctx.st.loop];
      const { mag, phase } = T.bode(G, W);
      const lines = [{ label: `P_${ctx.st.loop}(jω)`, mag, phase, color: '--series-1' }];
      if (ctx.st.asym) {
        if (ctx.st.loop === 'outer') {
          const a = P.m.a, Kb = Math.abs(G.num[0]) / a;
          lines.push({ label: 'straight-line approx.', mag: W.map((w) => (w < a ? Kb / w : Kb * a / (w * w))), phase: W.map((w) => 90 + (w < a / 10 ? 0 : w > 10 * a ? -90 : -45 * (Math.log10(w / a) + 1))), color: '--text-muted', dash: [5, 4], width: 1.5 });
        } else {
          const b = G.num[0];
          lines.push({ label: 'straight-line approx.', mag: W.map((w) => b / (w * w)), phase: W.map(() => -180), color: '--text-muted', dash: [5, 4], width: 1.5 });
        }
      }
      const g = T.at(G, ctx.st.w0);
      return { title: `Bode plot of ${{ lon: 'H̃/F̃', inner: 'Θ/τ', outer: 'Z/Θ' }[ctx.st.loop]}`, w: W, lines, marks: [{ w: ctx.st.w0, label: `ω₀: ${fmt(db(L.C.abs(g)), 3)} dB, ${fmt(L.C.arg(g) * 180 / Math.PI, 3)}°`, color: '--series-3' }] };
    },
    splane(ctx) { const P = plants(ctx.pModel); return { markers: L.roots(P[ctx.st.loop].den).map((q) => ({ ...q, kind: 'ol', label: 'pole' })), fitR: 0.2 }; },
    math(ctx) {
      const P = plants(ctx.pModel), m = P.m;
      return [
        { title: 'Frequency response', page: 'p. 264 · Eq. 15.4', theory: 'u = A\\sin\\omega_0 t \\Rightarrow y_{ss} = A|P(j\\omega_0)|\\sin(\\omega_0 t + \\angle P(j\\omega_0))' },
        { title: 'Altitude F̃ → h', page: 'F.15(a)', theory: 'P_{lon}(j\\omega) = \\frac{1/M}{(j\\omega)^2}',
          numbers: `20\\log|P_{lon}| = ${tex(db(m.lon.b0))} - 40\\log\\omega\\;\\text{dB},\\; \\angle = -180^\\circ`, spoiler: true, note: 'A double integrator: −40 dB/dec through 20 log(1/M) at ω = 1, phase −180° everywhere.' },
        { title: 'Inner τ → θ', page: 'F.15(b)', theory: 'P_{in}(j\\omega) = \\frac{1/J}{(j\\omega)^2}',
          numbers: `20\\log|P_{in}| = ${tex(db(m.inner.b0))} - 40\\log\\omega\\;\\text{dB},\\; \\angle = -180^\\circ,\\; 0\\text{ dB at } \\omega = ${tex(Math.sqrt(m.inner.b0))}`, spoiler: true },
        { title: 'Outer θ → z', page: 'F.15(c)', theory: 'P_{out}(j\\omega) = \\frac{-g}{j\\omega(j\\omega + \\mu/M)} = \\frac{-gM/\\mu}{j\\omega(1 + j\\omega M/\\mu)}',
          numbers: `\\text{Bode gain } \\frac{gM}{\\mu} = ${tex(-m.outer.b0 / m.a)}\\,(${tex(db(-m.outer.b0 / m.a))}\\text{ dB}),\\; \\text{corner } \\frac{\\mu}{M} = ${tex(m.a)}\\,\\text{rad/s}`, spoiler: true,
          note: 'The −1 adds 180°: the phase starts at +90° (−270°) and falls to 0° (−360°). The corner is very low, so above 1 rad/s it looks like −g/s².' },
      ];
    },
    buildProblem(parent, ctx) {
      const P = () => plants(ctx.pModel);
      const magDb = (G, w) => db(abs(G, w));
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch15, [
        {
          id: 'a', title: '(a) F̃ → h: |P(j1)| [dB] and slope [dB/dec]', inputs: { m: '|P(j1)| [dB]', s: 'slope' },
          check: (v) => PD().checkNumbers(v, { m: magDb(P().lon, 1), s: -40 }, {}),
          solution: () => [{ tex: `20\\log\\tfrac{1}{M} = ${tex(magDb(P().lon, 1))}\\,\\text{dB at } \\omega = 1,\\; -40\\,\\text{dB/dec},\\; -180^\\circ` }],
        },
        {
          id: 'b', title: '(b) τ → θ: |P(j1)| [dB] and 0 dB crossing [rad/s]', inputs: { m: '|P(j1)| [dB]', w: 'ω at 0 dB' },
          check: (v) => PD().checkNumbers(v, { m: magDb(P().inner, 1), w: Math.sqrt(P().m.inner.b0) }, {}),
          solution: () => [{ tex: `20\\log\\tfrac1J = ${tex(magDb(P().inner, 1))}\\,\\text{dB},\\; |P| = 1 \\text{ at } \\omega = \\sqrt{1/J} = ${tex(Math.sqrt(P().m.inner.b0))}` }],
        },
        {
          id: 'c', title: '(c) θ → z: Bode gain, corner, and |P(j1)| [dB]', inputs: { K: 'gM/μ', wc: 'corner [rad/s]', m: '|P(j1)| [dB]' },
          check: (v) => PD().checkNumbers(v, { K: -P().m.outer.b0 / P().m.a, wc: P().m.a, m: magDb(P().outer, 1) }, {}),
          solution: () => [{ tex: `P_{out} = \\frac{-${tex(-P().m.outer.b0 / P().m.a)}}{j\\omega(1 + j\\omega/${tex(P().m.a)})},\\; |P(j1)| = ${tex(magDb(P().outer, 1))}\\,\\text{dB}` }, { html: '−20 dB/dec below the corner, −40 above; phase from +90° to 0° (that is −270° to −360°).' }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 16 --
  function specs16(ctx) {
    const pr = ctx.sys.problems.ch16, lp = loopsOf(ctx), g = f10Gains(ctx), P = lp.P;
    // (a) parabola r̈ = A: book convention |PC| slope at low frequency
    const nIntL = 3;  // 1/s² plant × integrator
    const ePar = g.kIh ? 0 : pr.parabola * P.m.M / g.kPh;
    const eParImpl = g.kIh ? pr.parabola * g.kDh / g.kIh : Infinity;   // derivative on h, no dirty derivative
    const gn = abs(lp.Ll, pr.wno), gnExact = abs(T.feedback(lp.Ll), pr.wno);
    // (c) input disturbance below ω_din: book 1/min|C|; exact max |P/(1+PC)|
    let minC = Infinity, maxS = 0;
    const Sd = T.tf(L.conv(P.inner.num, lp.Ci.den), L.polyAdd(L.conv(P.inner.den, lp.Ci.den), L.conv(P.inner.num, lp.Ci.num)));
    for (const w of W) if (w <= pr.wdin) { minC = Math.min(minC, abs(lp.Ci, w)); maxS = Math.max(maxS, abs(Sd, w)); }
    // (d) frequency above which 1° of θ noise shows up as < 0.1°
    let wSensor = NaN;
    for (let i = W.length - 2; i >= 0; i--) if (abs(lp.Ti, W[i]) >= pr.thetaNoise) { wSensor = crossDown(lp.Ti, pr.thetaNoise, W[i], W[i + 1]); break; }
    const gr = 1 / abs(lp.Lo, pr.wr), grExact = abs(T.feedback(T.gain(1), lp.Lo), pr.wr);
    const gout = 1 / abs(lp.Lo, pr.wdout), goutExact = abs(T.feedback(T.gain(1), lp.Lo), pr.wdout);
    return { lp, g, ePar, eParImpl, nIntL, gn, gnExact, gdin: 1 / minC, gdinExact: maxS, wSensor, gr, grExact, gout, goutExact };
  }

  F.register({
    id: 'ch16', num: 16, tab: 'Ch 16', title: 'Frequency-domain specs', pages: 'pp. 283–301, F.16 p. 402',
    controller: (ctx, o) => F.makePID(ctx, o),
    defaults() { return f10Defaults({ view: 'lon' }); },
    simDefaults(sys) { return { ...sys.problems.ch16.sim, refs: [{ type: 'step', amplitude: 2.5 }] }; },
    gains(ctx) { return f10Gains(ctx); },
    buildControls(parent, ctx) {
      srcControls(parent, ctx);
      const sec = section(parent, 'Loop shown in the Bode plot', 'p. 284–290');
      segmented(sec, { options: [{ value: 'lon', label: 'altitude (a, b)' }, { value: 'inner', label: 'inner θ (c, d)' }, { value: 'outer', label: 'outer z (e, f)' }], get: () => ctx.st.view, set: (v) => { ctx.st.view = v; ctx.update(); } });
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const s = specs16(ctx), pr = ctx.sys.problems.ch16;
        const show = F.shown(ctx, 'F:ch16');
        box.replaceChildren(...(show ? [
          F.metricRow('(a) parabola r̈ = 5: e_ss (book / D on h)', `${fmt(s.ePar, 3)} / ${isFinite(s.eParImpl) ? fmt(s.eParImpl, 3) : '∞'} m`),
          F.metricRow(`(b) noise above ${pr.wno} rad/s: |PC|`, `${fmt(100 * s.gn, 3)} %`),
          F.metricRow(`(c) d_in below ${pr.wdin} rad/s: 1/min|C|`, `${fmt(100 * s.gdin, 3)} %`),
          F.metricRow('(d) 1° of θ noise → < 0.1° above', `${fmt(s.wSensor, 3)} rad/s`),
          F.metricRow(`(e) tracking below ${pr.wr} rad/s: 1/|PC|`, `${fmt(100 * s.gr, 3)} %`),
          F.metricRow(`(f) d_out below ${pr.wdout} rad/s: 1/|PC|`, `${fmt(100 * s.gout, 3)} %`),
        ] : [F.revealButton(ctx, 'F:ch16', 'Reveal the spec readouts')]));
      });
      segmented(sec, { label: 'Outer plant', options: [{ value: false, label: 'inner loop = k_DC (F.8)' }, { value: true, label: 'with inner dynamics' }], get: () => ctx.st.innerDyn, set: (v) => { ctx.st.innerDyn = v; ctx.update(); } });
    },
    bode(ctx) {
      const s = specs16(ctx), pr = ctx.sys.problems.ch16, lp = s.lp, v = ctx.st.view;
      if (v === 'lon') {
        return { title: 'Altitude: P and PC (F.10 PID)', w: W, lines: [{ label: 'P_lon', ...T.bode(lp.P.lon, W), color: '--text-muted', width: 1.5 }, { label: 'P·C_PID', ...T.bode(lp.Ll, W), color: '--series-1' }],
          specs: [{ w0: pr.wno, w1: 1e4, db: db(s.gn), keep: 'below', color: '--series-3', label: `${fmt(db(s.gn), 3)} dB` }] };
      }
      if (v === 'inner') {
        return { title: 'Inner loop: P and PC (F.8 PD)', w: W, lines: [{ label: 'P_in', ...T.bode(lp.P.inner, W), color: '--text-muted', width: 1.5 }, { label: 'P·C_PD', ...T.bode(lp.Li, W), color: '--series-1' }, { label: 'closed loop T_in', mag: T.bode(lp.Ti, W).mag, color: '--series-2', width: 1.5 }],
          marks: [{ w: pr.wdin, label: `ω_din: |C| = ${fmt(db(abs(lp.Ci, pr.wdin)), 3)} dB` }, ...(isFinite(s.wSensor) ? [{ w: s.wSensor, label: '|T| = −20 dB', color: '--series-3' }] : [])] };
      }
      return { title: 'Outer loop: P and PC (F.10 PID)', w: W, lines: [{ label: 'P_out', ...T.bode(lp.Pout, W), color: '--text-muted', width: 1.5 }, { label: 'P·C', ...T.bode(lp.Lo, W), color: '--series-1' }],
        specs: [{ w0: 1e-3, w1: pr.wr, db: -db(s.gr), keep: 'above', color: '--series-3', label: `B_r = ${fmt(-db(s.gr), 3)} dB` }],
        marks: [{ w: pr.wdout, label: `ω_dout: ${fmt(-db(s.gout), 3)} dB` }] };
    },
    splane(ctx) { return F.pidSplane(ctx, f10Gains(ctx), {}); },
    math(ctx) {
      const s = specs16(ctx), pr = ctx.sys.problems.ch16;
      return [
        { title: 'C_PID with dirty derivative', page: 'p. 313',
          theory: 'C(s) = k_P + \\frac{k_I}{s} + \\frac{k_Ds}{\\sigma s + 1}',
          numbers: `C_{lon} = ${T.texTf(s.lp.Cl)},\\quad C_{in} = ${T.texTf(s.lp.Ci)}`, spoiler: true },
        { title: 'Parabola tracking (a)', page: 'p. 293 · Eq. 16.12, F.16(a)',
          theory: 'PC_{lon} \\sim \\frac{k_I}{M s^3} \\;(\\text{type 3}) \\Rightarrow e_{ss} = 0 \\text{ for } R = \\frac{5}{s^3}',
          numbers: `e_{ss} = ${tex(s.ePar)}\\;(\\text{book}),\\quad \\text{with } D \\text{ on } h:\\; e_{ss} = \\frac{5k_{D_h}}{k_{I_h}} = ${isFinite(s.eParImpl) ? tex(s.eParImpl) : '\\infty'}`, spoiler: true,
          note: 'Table 9-1 and the Bode argument assume C acts on the error. The book\'s loops differentiate h instead, which costs one system type for tracking (see Ch 9).' },
        { title: 'Noise (b) and output disturbance / tracking (e, f)', page: 'p. 287 · Eq. 16.5–16.6',
          theory: '|y/n| = |T| \\approx |PC| \\;(\\omega \\ge \\omega_{no}),\\quad |e/r| = |S| \\approx \\frac{1}{|PC|} \\;(\\omega \\le \\omega_r)',
          numbers: `(b)\\; ${tex(100 * s.gn)}\\%\\;(|T| = ${tex(100 * s.gnExact)}\\%),\\quad (e)\\; ${tex(100 * s.gr)}\\%\\;(|S| = ${tex(100 * s.grExact)}\\%),\\quad (f)\\; ${tex(100 * s.gout)}\\%\\;(|S| = ${tex(100 * s.goutExact)}\\%)`, spoiler: true },
        { title: 'Input disturbance (c)', page: 'p. 290 · Eq. 16.8–16.9',
          theory: '\\Big|\\frac{y}{d_{in}}\\Big| = \\Big|\\frac{P}{1 + PC}\\Big| \\approx \\frac{1}{|C|}\\;(|PC| \\gg 1)',
          numbers: `\\frac{1}{\\min_{\\omega\\le${pr.wdin}}|C|} = \\frac{1}{k_{P_\\theta}} = ${tex(s.gdin)}\\;\\text{rad/(N·m)},\\quad \\max|P/(1+PC)| = ${tex(s.gdinExact)}`, spoiler: true,
          note: 'For PD, |C| is smallest at DC (= k_Pθ), so the worst case in the band is a constant torque. The "percentage" mixes units (rad per N·m).' },
        { title: 'θ sensor (d)', page: 'p. 287',
          theory: '\\theta_{noise} = |T_{in}(j\\omega)|\\,n(\\omega) < 0.1^\\circ',
          numbers: `|T_{in}| < 0.1 \\text{ above } \\omega \\approx ${tex(s.wSensor)}\\;\\text{rad/s}`, spoiler: true,
          note: 'So: noise below that frequency must already be under 0.1°; above it, 1° of noise is attenuated to under 0.1°, and higher still (−40 dB/dec) much more.' },
      ];
    },
    buildProblem(parent, ctx) {
      const s = () => specs16(ctx);
      const pc = (v) => ({ v: 100 * v });
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch16, [
        { id: 'a', title: '(a) Tracking error to a parabola of curvature 5 [m]', inputs: { v: 'e<sub>ss</sub>' },
          check: (v) => { const x = s(); const gv = PD().num(v.v); if (gv !== null && isFinite(x.eParImpl) && M.close(gv, x.eParImpl, 0.02)) return { ok: false, msg: 'That is the error of the implemented loop (derivative on h). Table 9-1 / the Bode plot treat C as acting on the error.' }; return PD().checkNumbers(v, { v: x.ePar }, { v: 'e_ss' }); },
          solution: () => [{ tex: `\\text{With } k_{I_h} > 0:\\; PC \\propto 1/s^3 \\Rightarrow \\text{type 3} \\Rightarrow e_{ss} = ${tex(s().ePar)}` }, { html: `"Curvature 5" is read as r̈ = 5, i.e. R = 5/s³. The implemented loop (derivative on h) instead has e<sub>ss</sub> = 5k<sub>D<sub>h</sub></sub>/k<sub>I<sub>h</sub></sub> = ${fmt(s().eParImpl, 3)} m.` }] },
        { id: 'b', title: '(b) % of noise above 30 rad/s in h', inputs: { v: '%' },
          check: (v) => PD().checkNumbers(v, pc(s().gn), { v: 'percent' }),
          solution: () => [{ tex: `|P C(j30)| = ${tex(db(s().gn))}\\,\\text{dB} \\Rightarrow ${tex(100 * s().gn)}\\%` }] },
        { id: 'c', title: '(c) % of an input disturbance below 2 rad/s in θ', inputs: { v: '%' },
          check: (v) => PD().checkNumbers(v, pc(s().gdin), { v: 'percent' }),
          solution: () => [{ tex: `\\gamma_{d_{in}} = \\frac{1}{\\min|C|} = \\frac{1}{k_{P_\\theta}} = ${tex(s().gdin)} \\Rightarrow ${tex(100 * s().gdin)}\\%` }] },
        { id: 'd', title: '(d) Above what frequency is 1° of θ sensor noise attenuated below 0.1°?', inputs: { v: 'ω [rad/s]' },
          check: (v) => { const x = s(), gv = PD().num(v.v); if (gv === null) return { ok: false, msg: 'Enter a frequency.' }; return { ok: Math.abs(gv / x.wSensor - 1) < 0.05, msg: Math.abs(gv / x.wSensor - 1) < 0.05 ? 'Within 5%.' : 'Where does |T_in| drop to −20 dB?' }; },
          solution: () => [{ tex: `|T_{in}(j\\omega)| = 0.1 \\text{ at } \\omega \\approx ${tex(s().wSensor)}` }, { html: 'Sensor answer: noise must be below 0.1° at low frequency (|T| ≈ 1 inside the bandwidth); noise of 1° is fine only above this frequency, and 10° only about a decade above (−40 dB/dec roll-off).' }] },
        { id: 'e', title: '(e) % tracking error for z<sub>r</sub> content below 0.1 rad/s', inputs: { v: '%' },
          check: (v) => PD().checkNumbers(v, pc(s().gr), { v: 'percent' }),
          solution: () => [{ tex: `\\gamma_r = \\frac{1}{|PC(j0.1)|} = ${tex(s().gr)} \\Rightarrow ${tex(100 * s().gr)}\\%\\;(\\text{exact } |S(j0.1)| = ${tex(100 * s().grExact)}\\%)` }, { html: 'The book approximation 1/|PC| is poor here because |PC(j0.1)| is not ≫ 1.' }] },
        { id: 'f', title: '(f) % of an output disturbance below 0.01 rad/s in z', inputs: { v: '%' },
          check: (v) => PD().checkNumbers(v, pc(s().gout), { v: 'percent' }),
          solution: () => [{ tex: `\\frac{1}{|PC(j0.01)|} = ${tex(s().gout)} \\Rightarrow ${tex(100 * s().gout)}\\%` }] },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 17 --
  function margins17(ctx) {
    const lp = loopsOf(ctx, f10Gains(ctx), { pdOuter: ctx.st.pdOuter });
    const res = {};
    for (const [k, Lg] of [['lon', lp.Ll], ['inner', lp.Li], ['outer', lp.Lo]]) {
      const Tc = T.feedback(Lg);
      res[k] = { Lg, Tc, mg: T.margins(Lg), bw: bandwidth(Tc) };
    }
    res.sep = res.inner.bw / res.outer.bw;
    res.lp = lp;
    return res;
  }

  F.register({
    id: 'ch17', num: 17, tab: 'Ch 17', title: 'Stability margins', pages: 'pp. 303–322, F.17 pp. 402–403',
    controller: (ctx, o) => F.makePID(ctx, o),
    defaults() { return f10Defaults({ view: 'lon', pdOuter: true }); },
    simDefaults(sys) { return { ...sys.problems.ch17.sim, refs: [{ type: 'step', amplitude: 2.5 }] }; },
    gains(ctx) { const g = f10Gains(ctx); return ctx.st.pdOuter ? { ...g, kIz: 0 } : g; },
    buildControls(parent, ctx) {
      srcControls(parent, ctx);
      const sec = section(parent, 'Margins', 'p. 304–306');
      segmented(sec, { label: 'Bode plot', options: [{ value: 'lon', label: 'altitude (a)' }, { value: 'lat', label: 'inner + outer (b–d)' }], get: () => ctx.st.view, set: (v) => { ctx.st.view = v; ctx.update(); } });
      segmented(sec, { label: 'Outer controller', options: [{ value: true, label: 'PD (as F.17c says)' }, { value: false, label: 'PID (with k_I,z)' }], get: () => ctx.st.pdOuter, set: (v) => { ctx.st.pdOuter = v; ctx.update(); } });
      segmented(sec, { label: 'Outer plant', options: [{ value: false, label: 'inner = k_DC' }, { value: true, label: 'with inner dynamics' }], get: () => ctx.st.innerDyn, set: (v) => { ctx.st.innerDyn = v; ctx.update(); } });
      const box = el('div', { class: 'metrics' });
      sec.append(box);
      WB.ui.addRefresher(() => {
        const r = margins17(ctx);
        const show = F.shown(ctx, 'F:ch17');
        const rows = [];
        if (show) {
          for (const k of ['lon', 'inner', 'outer']) {
            rows.push(F.metricRow(`${LOOPS[k]}: PM at ω_co`, `${fmt(r[k].mg.pm, 3)}° at ${fmt(r[k].mg.wc, 3)} rad/s`));
            rows.push(F.metricRow(`${LOOPS[k]}: GM; bandwidth`, `${gmText(r[k].mg)}; ${fmt(r[k].bw, 3)} rad/s`));
          }
          rows.push(F.metricRow('separation ω_bw,in / ω_bw,out', fmt(r.sep, 3)));
        } else rows.push(F.revealButton(ctx, 'F:ch17', 'Reveal the margins'));
        box.replaceChildren(...rows);
      });
    },
    bode(ctx) {
      const r = margins17(ctx), show = F.shown(ctx, 'F:ch17');
      if (ctx.st.view === 'lon') {
        const marks = show ? marginMarks(r.lon.mg) : [];
        if (show && isFinite(r.lon.bw)) marks.push({ w: r.lon.bw, label: `bandwidth ${fmt(r.lon.bw, 3)}`, color: '--series-2' });
        return { title: 'Altitude: open loop PC and closed loop', w: W, lines: [{ label: 'open loop PC', ...T.bode(r.lon.Lg, W), color: '--series-1' }, { label: 'closed loop T', ...T.bode(r.lon.Tc, W), color: '--series-2', width: 1.5 }], marks };
      }
      const marks = show ? [...marginMarks(r.inner.mg, '--series-1'), ...marginMarks(r.outer.mg, '--series-3')] : [];
      return {
        title: 'Lateral: inner and outer loops, open and closed', w: W,
        lines: [
          { label: 'inner PC', ...T.bode(r.inner.Lg, W), color: '--series-1' },
          { label: 'inner T', ...T.bode(r.inner.Tc, W), color: '--series-1', dash: [5, 4], width: 1.5 },
          { label: 'outer PC', ...T.bode(r.outer.Lg, W), color: '--series-3' },
          { label: 'outer T', ...T.bode(r.outer.Tc, W), color: '--series-3', dash: [5, 4], width: 1.5 },
        ],
        marks,
      };
    },
    splane(ctx) { return F.pidSplane({ ...ctx, st: { ...ctx.st, view: ctx.st.view === 'lon' ? 'lon' : 'lat' } }, ctx.gains, {}); },
    math(ctx) {
      const r = margins17(ctx);
      return [
        { title: 'Crossover and phase margin', page: 'p. 303–304',
          theory: '|PC(j\\omega_{co})| = 1,\\quad PM = 180^\\circ + \\angle PC(j\\omega_{co}),\\quad GM = \\frac{1}{|PC(j\\omega_{180})|}',
          numbers: ['lon', 'inner', 'outer'].map((k) => `\\text{${LOOPS[k]}}: PM = ${tex(r[k].mg.pm)}^\\circ \\text{ at } ${tex(r[k].mg.wc)}`).join(',\\quad '), spoiler: true },
        { title: 'Bandwidth vs. crossover', page: 'p. 306–307',
          theory: '\\omega_{bw}: |T(j\\omega_{bw})| = -3\\,\\text{dB};\\quad PM \\approx 60^\\circ \\Rightarrow \\omega_{bw} \\approx 1.3\\text{–}1.6\\,\\omega_{co}',
          numbers: ['lon', 'inner', 'outer'].map((k) => `\\text{${LOOPS[k]}}: \\omega_{bw} = ${tex(r[k].bw)}`).join(',\\quad '), spoiler: true },
        { title: 'Bandwidth separation (d)', page: 'p. 118, F.17(d)',
          theory: '\\frac{\\omega_{bw,in}}{\\omega_{bw,out}} \\gtrsim 5\\text{–}10 \\Rightarrow \\text{successive loop closure is justified}',
          numbers: `\\frac{\\omega_{bw,in}}{\\omega_{bw,out}} = ${tex(r.sep)}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const r = () => margins17(ctx);
      const part = (k, lab) => ({
        id: k, title: `${lab} ${LOOPS[k]} loop: PM, ω<sub>co</sub>, bandwidth`, inputs: { pm: 'PM [°]', wc: 'ω<sub>co</sub>', bw: 'ω<sub>bw</sub>' },
        check: (v) => { const x = r()[k]; return PD().checkNumbers(v, { pm: x.mg.pm, wc: x.mg.wc, bw: x.bw }, { pm: 'PM', wc: 'ωco', bw: 'ωbw' }); },
        solution: () => { const x = r()[k]; return [{ tex: `PM = ${tex(x.mg.pm)}^\\circ \\text{ at } \\omega_{co} = ${tex(x.mg.wc)},\\; GM = ${x.mg.crossings.length ? x.mg.crossings.map((c) => tex(db(c.gm)) + '\\,\\text{dB}').join(', ') : '\\infty'},\\; \\omega_{bw} = ${tex(x.bw)} = ${tex(x.bw / x.mg.wc)}\\,\\omega_{co}` }]; },
      });
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch17, [
        part('lon', '(a)'), part('inner', '(b)'), part('outer', '(c)'),
        { id: 'd', title: '(d) Bandwidth separation ω<sub>bw,in</sub>/ω<sub>bw,out</sub>', inputs: { v: 'ratio' },
          check: (v) => PD().checkNumbers(v, { v: r().sep }, { v: 'ratio' }),
          solution: () => [{ tex: `\\frac{${tex(r().inner.bw)}}{${tex(r().outer.bw)}} = ${tex(r().sep)}` }, { html: 'Above the 5–10 rule of thumb (p. 118), so treating the inner loop as its DC gain is justified for these gains.' }] },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 18 --
  // Compensator per loop: k·[(s+z_I)/s]·lead·lag·LPF·LPF, prefilter F = p/(s+p).
  // The outer loop's C carries a minus sign (P_out has −g).
  const BLK = {
    pi: (b) => T.tf([1, b.z], [1, 0]),
    lead: (b) => T.tf([b.M, b.M * b.w / Math.sqrt(b.M)], [1, b.w * Math.sqrt(b.M)]),
    lag: (b) => T.tf([1, b.z], [1, b.z / b.M]),
    lpf1: (b) => T.tf([b.p], [1, b.p]),
    lpf2: (b) => T.tf([b.p], [1, b.p]),
  };
  const blank = (o = {}) => ({ k: 1, pi: { on: false, z: 0.1 }, lead: { on: false, w: 1, M: 10 }, lag: { on: false, z: 0.1, M: 10 }, lpf1: { on: false, p: 50 }, lpf2: { on: false, p: 100 }, pf: { on: false, p: 1 }, wco: 1, ...o });
  // Reference designs (see ISSUES.md / Show solution); k is set for crossover at wco.
  const REF = {
    lon: () => blank({ pi: { on: true, z: 0.23 }, lead: { on: true, w: 1.5, M: 35 }, lpf1: { on: true, p: 70 }, lpf2: { on: true, p: 70 }, pf: { on: true, p: 0.5 }, wco: 1.5 }),
    inner: () => blank({ lead: { on: true, w: 10, M: 20 }, lpf1: { on: true, p: 100 }, wco: 10 }),
    outer: () => blank({ pi: { on: true, z: 0.1 }, lead: { on: true, w: 1, M: 20 }, lpf1: { on: true, p: 30 }, pf: { on: true, p: 0.3 }, wco: 1 }),
  };
  // Work-mode start: stable lead designs that miss the specs (not the answer).
  const START = {
    lon: () => blank({ lead: { on: true, w: 0.5, M: 10 }, wco: 0.5 }),
    inner: () => blank({ lead: { on: true, w: 5, M: 10 }, wco: 5 }),
    outer: () => blank({ lead: { on: true, w: 0.3, M: 10 }, wco: 0.3 }),
  };
  function compOf(c, sign = 1) {
    let C = T.gain(sign * c.k);
    for (const key of Object.keys(BLK)) if (c[key].on) C = T.mul(C, BLK[key](c[key]));
    return C;
  }
  function design18(ctx, st = ctx.st) {
    const P = plants(ctx.pModel);
    const Cl = compOf(st.lon), Ci = compOf(st.inner), Co = compOf(st.outer, -1);
    const Ll = T.mul(P.lon, Cl), Li = T.mul(P.inner, Ci);
    const Ti = T.feedback(Li);
    const Pout = T.mul(P.outer, Ti);
    const Lo = T.mul(Pout, Co);
    const pf = (c) => (c.pf.on ? T.tf([c.pf.p], [1, c.pf.p]) : T.gain(1));
    return { P, Cl, Ci, Co, Ll, Li, Ti, Pout, Lo, Fl: pf(st.lon), Fo: pf(st.outer), mgl: T.margins(Ll), mgi: T.margins(Li), mgo: T.margins(Lo) };
  }
  // k so that |L(j wco)| = 1 for one loop
  function autoK(ctx, which) {
    const st = ctx.st, c = st[which];
    const tmp = { ...st, [which]: { ...c, k: 1 } };
    const d = design18(ctx, tmp);
    const Lg = which === 'lon' ? d.Ll : which === 'inner' ? d.Li : d.Lo;
    return 1 / abs(Lg, c.wco);
  }
  function refState(ctx) {
    const st = { lon: REF.lon(), inner: REF.inner(), outer: REF.outer() };
    const c = { ...ctx, st: { ...ctx.st, ...st } };
    for (const w of ['inner', 'lon', 'outer']) { c.st[w].k = autoK(c, w); }
    return c.st;
  }
  function specs18(ctx, d = design18(ctx), st = ctx.st) {
    const pr = ctx.sys.problems.ch18;
    let lo = Infinity, hiL = 0, hiO = 0;
    for (const w of W) {
      if (w <= pr.lon.wr) lo = Math.min(lo, abs(d.Ll, w));
      if (w >= pr.lon.wn) hiL = Math.max(hiL, abs(d.Ll, w));
      if (w >= pr.outer.wno) hiO = Math.max(hiO, abs(d.Lo, w));
    }
    const bwi = bandwidth(d.Ti);
    return {
      lon: { integ: st.lon.pi.on, track: lo >= 1 / pr.lon.gr * 0.999, lo, noise: hiL <= pr.lon.gn * 1.001, hiL, pm: Math.abs(d.mgl.pm - pr.lon.pm) <= 5, pf: st.lon.pf.on },
      inner: { pm: Math.abs(d.mgi.pm - pr.inner.pm) <= 5, wco: Math.abs(d.mgi.wc / pr.inner.wco - 1) <= 0.2, bw: bwi, lpf: st.inner.lpf1.on || st.inner.lpf2.on },
      outer: { wco: Math.abs(d.mgo.wc / pr.outer.wco - 1) <= 0.2, integ: st.outer.pi.on, noise: hiO <= pr.outer.gno * 1.001, hiO, pm: Math.abs(d.mgo.pm - pr.outer.pm) <= 5, pf: st.outer.pf.on },
    };
  }

  F.register({
    id: 'ch18', num: 18, tab: 'Ch 18', title: 'Loopshaping', pages: 'pp. 323–374, F.18 pp. 403–404',
    linearLabel: 'linear loops (no saturation, d, noise)',
    defaults() { return { view: 'lat', loop: 'lon', zOff: 3, hOff: 0, showT: true, lon: START.lon(), inner: START.inner(), outer: START.outer(), init: false }; },
    simDefaults(sys) { return { ...sys.problems.ch18.sim, refs: [{ type: 'square', amplitude: 2.5, frequency: 0.04 }] }; },
    gains(ctx) {
      const st = ctx.st;
      // Explore mode starts from the reference design; Work mode from START (k set once).
      if (!st.init) {
        if (ctx.S.mode === 'explore') Object.assign(st, refState(ctx));
        else for (const w of ['inner', 'lon', 'outer']) st[w].k = autoK(ctx, w);
        st.init = true;
      }
      return {};
    },
    controller(ctx, { linear = false } = {}) {
      const d = design18(ctx), Ts = ctx.S.sim.Ts, s = ctx.sys, p = ctx.pModel, m = s.models(p);
      const Cl = T.filter(d.Cl, Ts), Ci = T.filter(d.Ci, Ts), Co = T.filter(d.Co, Ts), Fl = T.filter(d.Fl, Ts), Fo = T.filter(d.Fo, Ts);
      return {
        // Listing 18.3 pattern per loop: e = F(r) − y_m, u = C(e); F = F_e + C_lon(e_h)
        update(r, x, y) {
          const Ft = Cl.step(Fl.step(r[0]) - y[1]);
          const thD = Co.step(Fo.step(r[1]) - y[0]);
          const tau = Ci.step(thD - y[2]);
          return { u: s.mix((linear ? 0 : m.Fe) + Ft, tau, p), thetaD: thD };
        },
      };
    },
    buildControls(parent, ctx) {
      const st = ctx.st;
      const top = section(parent, 'Loop being shaped', 'F.18 p. 403');
      segmented(top, { options: Object.entries(LOOPS).map(([v, l]) => ({ value: v, label: l })), get: () => st.loop, set: (v) => { st.loop = v; ctx.update(); } });
      top.append(el('div', { class: 'btn-row' },
        el('button', { type: 'button', class: 'btn', text: 'Start over (lead only)', onclick: () => { st[st.loop] = START[st.loop](); st[st.loop].k = autoK(ctx, st.loop); ctx.update(); } }),
        el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reference design (reveals answer)', onclick: () => { Object.assign(st, refState(ctx)); ctx.update(); } })));
      top.append(el('p', { class: 'muted small', text: 'C = k·PI·lead·lag·LPF·LPF (outer loop: −k·…). The outer plant includes the current inner closed loop, so shape the inner loop first.' }));
      const c = () => st[st.loop];
      const set = (path, key) => (v) => { c()[path][key] = v; ctx.update(); };
      const onOff = (sec, key) => segmented(sec, { options: [{ value: true, label: 'on' }, { value: false, label: 'off' }], get: () => c()[key].on, set: (v) => { c()[key].on = v; ctx.update(); } });
      const g = section(parent, 'Gain k', 'p. 324');
      slider(g, { label: 'k', min: 1e-4, max: 100, log: true, sig: 4, get: () => c().k, set: (v) => { c().k = v; ctx.update(); } });
      slider(g, { label: 'target ω<sub>co</sub>', unit: 'rad/s', min: 0.05, max: 50, log: true, sig: 3, get: () => c().wco, set: (v) => { c().wco = v; ctx.update(); } });
      g.append(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Set k for crossover at target ω_co', onclick: () => { c().k = autoK(ctx, st.loop); ctx.update(); } }));
      const pi = section(parent, 'PI (s + z_I)/s', 'p. 325');
      onOff(pi, 'pi');
      slider(pi, { label: 'z<sub>I</sub>', unit: 'rad/s', min: 0.001, max: 10, log: true, sig: 3, get: () => c().pi.z, set: set('pi', 'z'), disabled: () => !c().pi.on });
      const ld = section(parent, 'Lead M(s + ω/√M)/(s + ω√M)', 'p. 328 · Eq. 18.2');
      onOff(ld, 'lead');
      slider(ld, { label: 'ω<sub>lead</sub>', unit: 'rad/s', min: 0.05, max: 200, log: true, sig: 3, get: () => c().lead.w, set: set('lead', 'w'), disabled: () => !c().lead.on });
      slider(ld, { label: 'M', min: 1.01, max: 100, log: true, sig: 3, get: () => c().lead.M, set: set('lead', 'M'), disabled: () => !c().lead.on });
      const lg = section(parent, 'Lag (s + z)/(s + z/M)', 'p. 325 · Eq. 18.1');
      onOff(lg, 'lag');
      slider(lg, { label: 'z', unit: 'rad/s', min: 0.001, max: 20, log: true, sig: 3, get: () => c().lag.z, set: set('lag', 'z'), disabled: () => !c().lag.on });
      slider(lg, { label: 'M', min: 1.01, max: 200, log: true, sig: 3, get: () => c().lag.M, set: set('lag', 'M'), disabled: () => !c().lag.on });
      for (const key of ['lpf1', 'lpf2']) {
        const sx = section(parent, `Low-pass ${key === 'lpf1' ? 1 : 2}: p/(s + p)`, 'p. 325');
        onOff(sx, key);
        slider(sx, { label: 'p', unit: 'rad/s', min: 0.5, max: 5000, log: true, sig: 3, get: () => c()[key].p, set: set(key, 'p'), disabled: () => !c()[key].on });
      }
      if (st.loop !== 'inner') {
        const pf = section(parent, 'Prefilter F(s) = p/(s + p)', 'p. 336 · Eq. 18.5–18.7');
        onOff(pf, 'pf');
        slider(pf, { label: 'p', unit: 'rad/s', min: 0.05, max: 50, log: true, sig: 3, get: () => c().pf.p, set: set('pf', 'p'), disabled: () => !c().pf.on });
      }
      const sp = section(parent, 'F.18 specs (all loops)', 'p. 403–404');
      const box = el('div', { class: 'metrics' });
      sp.append(box);
      WB.ui.addRefresher(() => {
        const d = design18(ctx), s = specs18(ctx, d);
        const row = (label, ok, v) => el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: label }), el('strong', { text: v || '' }),
          el('span', { class: 'status ' + (ok ? 'good' : 'bad') }, el('span', { class: 'status-icon', 'aria-hidden': 'true', text: ok ? '✓' : '✗' }), el('span', { text: ok ? 'met' : 'not met' })));
        box.replaceChildren(
          row('(a) integrator (input d)', s.lon.integ), row('(a) |L| ≥ 40 dB below 0.1', s.lon.track, `${fmt(db(s.lon.lo), 3)} dB`),
          row('(a) |L| ≤ −80 dB above 200', s.lon.noise, `${fmt(db(s.lon.hiL), 3)} dB`), row('(a) PM ≈ 60°', s.lon.pm, `${fmt(d.mgl.pm, 3)}°`), row('(a) prefilter', s.lon.pf),
          row('(b) PM ≈ 60°', s.inner.pm, `${fmt(d.mgi.pm, 3)}°`), row('(b) ω_co ≈ 10', s.inner.wco, `${fmt(d.mgi.wc, 3)} (bw ${fmt(s.inner.bw, 3)})`), row('(b) low-pass filter', s.inner.lpf),
          row('(c) ω_co ≈ 1', s.outer.wco, `${fmt(d.mgo.wc, 3)}`), row('(c) integrator', s.outer.integ), row('(c) |L| ≤ −100 dB above 100', s.outer.noise, `${fmt(db(s.outer.hiO), 3)} dB`),
          row('(c) PM ≈ 60°', s.outer.pm, `${fmt(d.mgo.pm, 3)}°`), row('(c) prefilter', s.outer.pf),
        );
      });
    },
    bode(ctx) {
      const d = design18(ctx), pr = ctx.sys.problems.ch18, w = ctx.st.loop;
      const Lg = w === 'lon' ? d.Ll : w === 'inner' ? d.Li : d.Lo;
      const Pp = w === 'lon' ? d.P.lon : w === 'inner' ? d.P.inner : d.Pout;
      const mg = w === 'lon' ? d.mgl : w === 'inner' ? d.mgi : d.mgo;
      const lines = [{ label: w === 'outer' ? 'P_out·T_in' : 'P', ...T.bode(Pp, W), color: '--text-muted', dash: [5, 4], width: 1.5 }, { label: 'loop gain P·C', ...T.bode(Lg, W), color: '--series-1' }];
      if (ctx.st.showT) {
        const Fp = w === 'lon' ? d.Fl : w === 'outer' ? d.Fo : T.gain(1);
        lines.push({ label: w === 'inner' ? 'closed loop T' : 'closed loop F·T', mag: T.bode(T.mul(Fp, T.feedback(Lg)), W).mag, color: '--series-3', width: 1.5 });
      }
      const specs = w === 'lon'
        ? [{ w0: 1e-3, w1: pr.lon.wr, db: 40, keep: 'above', color: '--critical', label: 'tracking' }, { w0: pr.lon.wn, w1: 1e4, db: -80, keep: 'below', color: '--critical', label: 'noise' }]
        : w === 'outer' ? [{ w0: pr.outer.wno, w1: 1e4, db: -100, keep: 'below', color: '--critical', label: 'noise' }] : [];
      return { title: `Loopshaping: ${LOOPS[w]} loop`, w: W, lines, specs, marks: marginMarks(mg) };
    },
    splane(ctx) {
      const d = design18(ctx), w = ctx.st.loop;
      const Lg = w === 'lon' ? d.Ll : w === 'inner' ? d.Li : d.Lo;
      const cl = L.roots(L.polyAdd(Lg.den, Lg.num));
      const mk = L.roots(Lg.den).map((q) => ({ ...q, kind: 'ol', label: 'pole of L', noFit: Math.hypot(q.re, q.im) > 30 }));
      cl.forEach((q) => mk.push({ ...q, kind: 'cl', label: 'closed-loop pole', noFit: Math.hypot(q.re, q.im) > 30 }));
      return { markers: mk, fitR: w === 'inner' ? 25 : 4 };
    },
    math(ctx) {
      const d = design18(ctx), st = ctx.st;
      const texC = (c, sign) => { const parts = [`${sign < 0 ? '-' : ''}${tex(c.k)}`]; for (const key of Object.keys(BLK)) if (c[key].on) parts.push(T.texTf(BLK[key](c[key]), 4)); return parts.join('\\cdot '); };
      return [
        { title: 'Loops', page: 'F.18 p. 403',
          theory: 'F = F_e + C_{lon}(s)\\,(F_h h_r - h_m),\\quad \\theta_d = C_{out}(s)\\,(F_z z_r - z_m),\\quad \\tau = C_{in}(s)\\,(\\theta_d - \\theta_m)' },
        { title: `Your C for the ${LOOPS[st.loop]} loop`, page: 'p. 323',
          theory: `C = ${texC(st[st.loop], st.loop === 'outer' ? -1 : 1)}` },
        { title: 'Margins (all loops)', page: 'p. 304–306',
          theory: `\\text{alt: } PM = ${tex(d.mgl.pm)}^\\circ @ ${tex(d.mgl.wc)},\\quad \\text{inner: } ${tex(d.mgi.pm)}^\\circ @ ${tex(d.mgi.wc)},\\quad \\text{outer: } ${tex(d.mgo.pm)}^\\circ @ ${tex(d.mgo.wc)}`,
          note: 'The altitude loop has three integrators (two in the plant, one in C), so it is conditionally stable: the phase is below −180° at low frequency and there is a gain margin below 1 as well (negative dB).' },
        { title: 'Lead and lag', page: 'p. 325–328 · Eq. 18.1–18.2',
          theory: '\\phi_{max} = \\sin^{-1}\\frac{M-1}{M+1} \\text{ at } \\omega_{lead},\\quad \\text{lag boosts low frequencies by } 20\\log M' },
        { title: 'Implementation (d)', page: 'p. 335 · Eq. 18.3–18.4',
          theory: '\\dot x_C = A_Cx_C + B_Ce,\\; u = C_Cx_C + D_Ce\\quad (\\text{controllable canonical form, RK4 at } T_s)' },
      ];
    },
    buildProblem(parent, ctx) {
      const s = () => specs18(ctx);
      const refTxt = () => {
        const r = refState(ctx);
        const dd = design18(ctx, { ...ctx.st, ...r });
        return { r, dd };
      };
      PD().problemPanel(parent, ctx, ctx.sys.problems.ch18, [
        { id: 'a', title: '(a) Altitude loop meets every spec',
          check: () => { const x = s().lon; const ok = x.integ && x.track && x.noise && x.pm && x.pf; return { ok, msg: `integrator ${x.integ ? '✓' : '✗'}, tracking ${x.track ? '✓' : '✗'}, noise ${x.noise ? '✓' : '✗'}, PM ${x.pm ? '✓' : '✗'}, prefilter ${x.pf ? '✓' : '✗'}` }; },
          solution: () => { const { r, dd } = refTxt(); return [{ html: `PI zero at 0.23, lead at 1.5 rad/s with M = 35, two low-pass filters at 70 rad/s, k = ${fmt(r.lon.k, 4)} (crossover 1.5 rad/s): PM ${fmt(dd.mgl.pm, 3)}°. Prefilter p = 0.5 rad/s removes most of the ~23% overshoot.` }]; } },
        { id: 'b', title: '(b) Inner loop: PM ≈ 60°, ω<sub>co</sub> ≈ 10 rad/s, low-pass filter',
          check: () => { const x = s().inner; const ok = x.pm && x.wco && x.lpf; return { ok, msg: `PM ${x.pm ? '✓' : '✗'}, crossover ${x.wco ? '✓' : '✗'} (bandwidth ${fmt(x.bw, 3)} rad/s), LPF ${x.lpf ? '✓' : '✗'}` }; },
          solution: () => { const { r, dd } = refTxt(); return [{ html: `Lead at 10 rad/s with M = 20 (≈ 65° max phase), LPF at 100 rad/s, k = ${fmt(r.inner.k, 4)}: PM ${fmt(dd.mgi.pm, 3)}° at ${fmt(dd.mgi.wc, 3)} rad/s. The closed-loop bandwidth is then about 1.7 ω<sub>co</sub>; read "bandwidth ≈ ω<sub>co</sub> = 10" as a crossover spec.` }]; } },
        { id: 'c', title: '(c) Outer loop meets every spec',
          check: () => { const x = s().outer; const ok = x.wco && x.integ && x.noise && x.pm && x.pf; return { ok, msg: `crossover ${x.wco ? '✓' : '✗'}, integrator ${x.integ ? '✓' : '✗'}, noise ${x.noise ? '✓' : '✗'}, PM ${x.pm ? '✓' : '✗'}, prefilter ${x.pf ? '✓' : '✗'}` }; },
          solution: () => { const { r, dd } = refTxt(); return [{ html: `On P<sub>out</sub>·T<sub>in</sub> with the reference inner loop: −k·(s+0.1)/s · lead at 1 rad/s with M = 20 · LPF at 30 rad/s, k = ${fmt(r.outer.k, 4)}: PM ${fmt(dd.mgo.pm, 3)}° at ${fmt(dd.mgo.wc, 3)} rad/s. The inner loop's roll-off already meets the −100 dB noise spec. Prefilter p = 0.3 rad/s.` }]; } },
        { id: 'd', title: '(d) Simulation tracks both references',
          html: 'Passes when |h<sub>r</sub> − h| and |z<sub>r</sub> − z| are under 5 cm just before the first z<sub>r</sub> switch, with no runaway.',
          check: () => {
            const S = ctx.S, zc = S.sim.refs[0];
            const tSw = zc.type === 'square' ? zc.tStep + 0.5 / zc.frequency : S.sim.tEnd;
            const eh = Math.abs(F.errorBefore(ctx, tSw, 1, 0)), ez = Math.abs(F.errorBefore(ctx, tSw, 0, 1));
            return { ok: eh < 0.05 && ez < 0.05, msg: `|e_h| = ${fmt(eh, 3)} m, |e_z| = ${fmt(ez, 3)} m at t = ${fmt(tSw - 0.05, 3)} s.` };
          } },
      ]);
    },
  });

  WB.F.freq = { plants, pidTf, loopsOf, specs16, margins17, design18, refState, specs18 };
})();
