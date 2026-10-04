// Chapter 9 (system type), Chapter 10 (digital PID), and Appendix P.6 (root locus
// versus k_I). All three use PID on the feedback-linearized second-order model.
window.WB = window.WB || {};
WB.chapters = WB.chapters || {};

(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texPole, fmt, fmtPole } = M;
  const PD = () => WB.pd;

  // PID law shared by Ch 9, Ch 10 and P.6.
  //   deriv 'state': use the true θ̇ (as in the Ch 7-9 code)
  //   deriv 'dirty': dirty derivative of the measured y, Eq. 10.4 (p. 157)
  //   antiwindup 'none' | 'gate' (integrate only when |ẏ| < vbar, Listing 10.2)
  //              | 'backcalc' (u_I += (u_sat - u_unsat)/k_I, p. 158)
  function makePID(ctx, { linear = false } = {}) {
    const { sys, pModel, S } = ctx;
    const st = ctx.st;
    const { kP, kI, kD } = ctx.gains;
    const Ts = S.sim.Ts, sigma = st.sigma ?? 0.05;
    const { beta, gamma } = WB.design.dirtyCoeffs(sigma, Ts);
    const uLim = sys.uLimit(pModel);
    let I = 0, ePrev = null, yPrev = null, ydot = 0, rPrev = null;
    return {
      update(r, x, yMeas) {
        const dirty = st.deriv === 'dirty';
        const y = dirty ? yMeas : x[0];
        const e = r - y;
        if (yPrev === null) { ePrev = 0; yPrev = y; rPrev = r; } // repo: error_prev = 0, theta_prev = θ0
        if (dirty) {
          ydot = beta * ydot + gamma * (y - yPrev);
        } else {
          ydot = x[1];
        }
        const rdot = (r - rPrev) / Ts;
        const integrate = st.antiwindup !== 'gate' || Math.abs(ydot) < (st.vbar ?? 0.08);
        if (kI !== 0 && integrate) I += (Ts / 2) * (e + ePrev);
        const dTerm = st.arch === 'error' ? kD * (rdot - ydot) : -kD * ydot;
        const uTilde = kP * e + kI * I + dTerm;
        let ff = 0;
        if (!linear) {
          if (st.comp === 'fl') ff = sys.feedbackLinearization([y, ydot], pModel);
          else if (st.comp === 'eq') ff = sys.equilibriumInput(0, pModel);
        }
        let u = uTilde + ff;
        if (!linear && st.antiwindup === 'backcalc' && kI !== 0) {
          const uSat = M.saturate(u, uLim);
          if (uSat !== u) { I += (uSat - u) / kI; u = uSat; } // unwind so u_unsat sits at the limit
        }
        ePrev = e; yPrev = y; rPrev = r;
        return { u, integrator: I, ydotHat: ydot };
      },
    };
  }

  // Closed-loop characteristic polynomial for PID with derivative on the output:
  // s^3 + (a1 + b0 kD) s^2 + (a0 + b0 kP) s + b0 kI  (A.P.6, p. 470)
  function pidCharPoly(model, { kP, kI, kD }) {
    return [1, model.a1 + model.b0 * kD, model.a0 + model.b0 * kP, model.b0 * kI];
  }

  function pidMarkers(ctx) {
    const { model } = ctx;
    const g = ctx.gains;
    const markers = M.roots2(model.a1, model.a0).map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole p${i + 1}` }));
    const cl = g.kI ? L.roots(pidCharPoly(model, g)) : PD().clPoles(model, g.kP, g.kD);
    cl.forEach((p, i) => markers.push({ ...p, kind: 'cl', label: `closed-loop pole p${i + 1}` }));
    if (g.kI && g.kP) markers.push({ re: -g.kI / g.kP, im: 0, kind: 'zero', label: 'closed-loop zero −kI/kP' });
    return markers;
  }

  // Reference shapes for Ch 9: step, ramp, parabola. amplitude is the step size,
  // ramp slope, or parabola coefficient (r = A t²) in display units; scale converts
  // them to SI (π/180 for the arm's degrees) and y0 is the starting value in SI.
  // Studies B–F use this too.
  function shapedReference(ctx, base, scale = 1, y0 = ctx.S.sim.y0 * scale) {
    const A = ctx.S.sim.amplitude * scale, t0 = ctx.S.sim.tStep;
    if (ctx.st.input === 'ramp') return (t) => (t < t0 ? y0 : y0 + A * (t - t0));
    if (ctx.st.input === 'parabola') return (t) => (t < t0 ? y0 : y0 + A * (t - t0) ** 2);
    return base;
  }

  function gainSliders(parent, ctx, ranges = {}) {
    slider(parent, { label: 'k<sub>P</sub>', min: 0, max: ranges.kP || 2, step: 0.001, get: () => ctx.st.kP, set: (v) => { ctx.st.kP = v; ctx.update(); } });
    slider(parent, { label: 'k<sub>I</sub>', min: 0, max: ranges.kI || 2, step: 0.001, get: () => ctx.st.kI, set: (v) => { ctx.st.kI = v; ctx.update(); } });
    slider(parent, { label: 'k<sub>D</sub>', min: 0, max: ranges.kD || 0.5, step: 0.0005, get: () => ctx.st.kD, set: (v) => { ctx.st.kD = v; ctx.update(); } });
  }

  function designSliders(parent, ctx, withRule) {
    slider(parent, { label: 't<sub>r</sub>', unit: 's', min: 0.1, max: 3, step: 0.005, get: () => ctx.st.tr, set: (v) => { ctx.st.tr = v; ctx.update(); } });
    slider(parent, { label: 'ζ', min: 0.1, max: 2, step: 0.005, get: () => ctx.st.zeta, set: (v) => { ctx.st.zeta = v; ctx.update(); } });
    if (withRule) {
      segmented(parent, {
        label: 'ω<sub>n</sub> from t<sub>r</sub>',
        options: [{ value: '2.2', label: '2.2 / t<sub>r</sub>' }, { value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))' }],
        get: () => ctx.st.rule, set: (v) => { ctx.st.rule = v; ctx.update(); },
      });
    }
    slider(parent, { label: 'k<sub>I</sub>', min: 0, max: 2, step: 0.001, get: () => ctx.st.kIx, set: (v) => { ctx.st.kIx = v; ctx.update(); } });
  }

  function designedGains(ctx) {
    const st = ctx.st;
    const wn = PD().wnFromTr(st.tr, st.zeta, st.rule || '2.2');
    return { ...PD().gainsFromPoles(ctx.model, PD().polesFromWnZeta(wn, st.zeta)), kI: st.kIx, wn };
  }

  function readout(parent, ctx) {
    const box = el('div', { class: 'readout' });
    parent.append(box);
    WB.ui.addRefresher(() => {
      const g = ctx.gains;
      box.replaceChildren(...['kP', 'kI', 'kD'].map((k) => el('div', {}, el('span', { class: 'ro-label', text: k }), el('strong', { text: fmt(g[k], 4) }))));
    });
  }

  // ------------------------------------------------------------- Chapter 9 --
  WB.chapters.ch9 = {
    id: 'ch9', num: 9, tab: 'Ch 9', title: 'System type & integrators', pages: 'pp. 137–154',

    defaults(sys) {
      return { arch: 'output', comp: 'fl', deriv: 'state', antiwindup: 'none', kP: 0.1134, kD: 0.0483, kI: 0, tr: 0.8, zeta: 0.707, kIx: 0.05, rule: '2.2', input: 'step' };
    },
    simDefaults(sys) { return sys.problems.ch9.sim; },
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD } : designedGains(ctx); },
    controller: (ctx, o) => makePID(ctx, o),
    reference: (ctx, base) => shapedReference(ctx, base, M.DEG),

    buildControls(parent, ctx) {
      const sec = section(parent, 'PID controller', 'p. 142');
      PD().sharedControls(sec, ctx);
      segmented(sec, {
        label: 'Reference shape (amplitude = size, slope, or coefficient)',
        options: [{ value: 'step', label: 'step' }, { value: 'ramp', label: 'ramp' }, { value: 'parabola', label: 'parabola' }],
        get: () => ctx.st.input, set: (v) => { ctx.st.input = v; ctx.update(); },
      });
      if (ctx.S.mode === 'work') gainSliders(sec, ctx);
      else { designSliders(sec, ctx, false); readout(sec, ctx); }
      const ty = section(parent, 'System type analysis', 'p. 141 · Table 9-1');
      const box = el('div', { class: 'metrics' });
      ty.append(box);
      WB.ui.addRefresher(() => this.renderType(box, ctx));
    },

    analysis(ctx) {
      const { model } = ctx;
      const g = ctx.gains;
      const hasI = g.kI > 0;
      const Mv = hasI ? Infinity : model.b0 * g.kP / model.a1;    // lim s P C
      const Ma = hasI ? model.b0 * g.kI / model.a1 : 0;           // lim s^2 P C
      return {
        type: hasI ? 2 : 1, Mv, Ma,
        stepErr: 0, rampErr: hasI ? 0 : 1 / Mv, parabErr: hasI ? 1 / Ma : Infinity,
        distType: hasI ? 1 : 0, distStepErr: hasI ? 0 : 1 / g.kP,  // per unit input torque
      };
    },

    renderType(box, ctx) {
      const a = this.analysis(ctx);
      const st = ctx.st, S = ctx.S;
      const A = S.sim.amplitude * M.DEG;
      const pred = st.input === 'step' ? 0 : st.input === 'ramp' ? A * a.rampErr : 2 * A * a.parabErr; // r = A t^2 -> R = 2A/s^3
      const distPred = Math.abs(S.sim.dist) * a.distStepErr;
      const res = ctx.app.result();
      const eEnd = res ? res.r[res.r.length - 1] - res.y[res.y.length - 1] : NaN;
      const row = (l, v) => el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: l }), el('strong', { text: v }));
      const show = S.mode === 'explore' || ctx.app.isRevealed('ch9:type');
      const rows = [];
      if (show) {
        rows.push(row('reference tracking type', `type ${a.type}`));
        rows.push(row(`predicted e_ss (${st.input}${S.sim.dist ? '' : ''})`, isFinite(pred) ? `${fmt(pred / M.DEG, 3)}°` : '∞'));
        rows.push(row('input-disturbance type', `type ${a.distType}`));
        rows.push(row(`predicted e from d = ${fmt(S.sim.dist, 3)} N·m`, `${fmt(distPred / M.DEG, 3)}°`));
      } else {
        rows.push(el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reveal the predicted type and errors', onclick: () => { ctx.app.reveal('ch9:type'); WB.ui.refreshAll(); } }));
      }
      rows.push(row('simulated error r − θ at t_end', isFinite(eEnd) ? `${fmt(eEnd / M.DEG, 3)}°` : '—'));
      box.replaceChildren(...rows);
    },

    splane(ctx) { return { markers: pidMarkers(ctx) }; },

    math(ctx) {
      const { model } = ctx;
      const g = ctx.gains;
      const a = this.analysis(ctx);
      return [
        PD().plantCard(ctx),
        { title: 'Final value theorem and tracking error', page: 'p. 137, p. 138',
          theory: '\\lim_{t\\to\\infty} e(t) = \\lim_{s\\to 0} sE(s),\\quad E(s) = \\frac{1}{1 + P(s)C(s)}R(s)' },
        { title: 'Error constants and system type', page: 'p. 141 · Table 9-1',
          theory: 'M_p = \\lim_{s\\to0} PC,\\quad M_v = \\lim_{s\\to0} sPC,\\quad M_a = \\lim_{s\\to0} s^2PC,\\quad e_{step} = \\tfrac{1}{1+M_p},\\; e_{ramp} = \\tfrac{1}{M_v},\\; e_{parab} = \\tfrac{1}{M_a}',
          numbers: g.kI > 0
            ? `M_a = \\frac{b_0 k_I}{a_1} = ${tex(a.Ma)} \\;(\\text{type 2}),\\quad e_{parab} = \\frac{a_1}{b_0 k_I} = \\frac{b}{k_I} = ${tex(1 / a.Ma)}`
            : `M_v = \\frac{b_0 k_P}{a_1} = \\frac{k_P}{b} = ${tex(a.Mv)} \\;(\\text{type 1}),\\quad e_{ramp} = ${tex(1 / a.Mv)}`,
          spoiler: true },
        { title: 'PID controller', page: 'p. 142',
          theory: 'C(s) = k_P + \\frac{k_I}{s} + k_D s = \\frac{k_D s^2 + k_P s + k_I}{s}',
          numbers: `\\Delta_{cl}(s) = ${WB.tf.polyTex(pidCharPoly(model, g))}` },
        { title: 'Input disturbance', page: 'p. 143 · Fig. 9-5, p. 144',
          theory: 'E(s) = \\frac{P}{1+PC}D_{in}(s),\\quad \\lim_{t\\to\\infty} e = \\lim_{s\\to 0}\\frac{P}{1+PC}\\frac{1}{s^q}',
          numbers: g.kI > 0 ? '\\text{with } k_I > 0:\\; \\lim_{s\\to0}\\frac{P}{1+PC} = 0 \\Rightarrow \\text{type 1 (step } d \\to e_{ss} = 0)'
            : `\\text{PD}:\\; \\lim_{s\\to0}\\frac{P}{1+PC} = \\frac{1}{k_P} = ${tex(1 / g.kP)}\\;\\text{rad}/(\\text{N}\\cdot\\text{m})`,
          spoiler: true,
          note: 'Disturbance type depends on the integrators in C(s) only (p. 145). With gravity compensation off, gravity is exactly such an input disturbance.' },
        PD().compensationCard(ctx),
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch9;
      const a = () => this.analysis(ctx);
      const g = () => ctx.gains;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a1', title: '(a) PD only: set k<sub>I</sub> = 0',
          html: 'Answers use the current k<sub>P</sub>, k<sub>D</sub>. Errors are for a unit input, in rad.',
          inputs: { type: 'type', step: 'e<sub>step</sub>', ramp: 'e<sub>ramp</sub>' },
          check: (v) => {
            if (g().kI > 0) return { ok: false, msg: 'Set kI = 0 first.' };
            return PD().checkNumbers(v, { type: 1, step: 0, ramp: 1 / a().Mv }, { ramp: 'e_ramp' });
          },
          solution: () => [
            { tex: `PC = \\frac{b_0(k_D s + k_P)}{s(s + a_1)}\\;\\Rightarrow\\;\\text{type 1},\\; M_v = \\frac{b_0 k_P}{a_1} = \\frac{k_P}{b}` },
            { html: 'Step error 0, ramp error b/k<sub>P</sub>, parabola error ∞ (p. 146).' },
          ],
        },
        {
          id: 'a2', title: '(a) With the integrator (k<sub>I</sub> > 0)',
          html: 'Unit parabola means R(s) = 1/s³.',
          inputs: { type: 'type', ramp: 'e<sub>ramp</sub>', parab: 'e<sub>parab</sub>' },
          check: (v) => {
            if (!(g().kI > 0)) return { ok: false, msg: 'Set kI > 0 first.' };
            return PD().checkNumbers(v, { type: 2, ramp: 0, parab: 1 / a().Ma }, { parab: 'e_parab' });
          },
          solution: () => [{ tex: `PC = \\frac{b_0(k_D s^2 + k_P s + k_I)}{s^2(s + a_1)}\\;\\Rightarrow\\;\\text{type 2},\\; e_{parab} = \\frac{b}{k_I}` }],
        },
        {
          id: 'b', title: '(b) Constant input disturbance of size d',
          html: 'Steady-state error magnitude per unit d (rad per N·m), without and with the integrator.',
          inputs: { pd: 'PD', pid: 'PID' },
          check: (v) => PD().checkNumbers(v, { pd: 1 / g().kP, pid: 0 }, { pd: 'PD', pid: 'PID' }),
          solution: () => [
            { tex: `\\text{PD}: \\lim_{s\\to0}\\frac{P}{1+PC} = \\frac{1}{k_P} = ${tex(1 / g().kP)},\\quad \\text{PID}: 0` },
            { html: 'Book: p. 147. Try it: set d ≠ 0 (left panel), then switch k<sub>I</sub> between 0 and a positive value.' },
          ],
        },
      ]);
    },
  };

  // ------------------------------------------------------------ Chapter 10 --
  WB.chapters.ch10 = {
    id: 'ch10', num: 10, tab: 'Ch 10', title: 'Digital PID', pages: 'pp. 155–169',

    defaults(sys) {
      const p = sys.problems.ch10;
      return { arch: 'output', comp: 'fl', deriv: 'dirty', antiwindup: 'gate', vbar: 0.08, sigma: p.sigma, kP: 0.3, kD: 0.08, kI: 0, tr: p.tr, zeta: p.zeta, rule: 'tp', kIx: p.ki, extra: 'deriv' };
    },
    simDefaults(sys) { return { ...sys.problems.ch10.sim, mismatch: sys.problems.ch10.mismatch }; },
    gains(ctx) { return ctx.S.mode === 'work' ? { kP: ctx.st.kP, kI: ctx.st.kI, kD: ctx.st.kD } : designedGains(ctx); },
    controller: (ctx, o) => makePID(ctx, o),

    buildControls(parent, ctx) {
      const sec = section(parent, 'Digital PID', 'p. 155 · Listing 10.2 p. 161');
      if (ctx.S.mode === 'work') gainSliders(sec, ctx);
      else { designSliders(sec, ctx, true); readout(sec, ctx); }
      const imp = section(parent, 'Implementation', 'p. 157 · Eq. 10.3–10.4');
      segmented(imp, {
        label: 'θ̇ for the D term',
        options: [{ value: 'dirty', label: 'dirty derivative of y' }, { value: 'state', label: 'true θ̇ (cheating)' }],
        get: () => ctx.st.deriv, set: (v) => { ctx.st.deriv = v; ctx.update(); },
      });
      slider(imp, { label: 'σ', unit: 's', min: 0.002, max: 0.5, step: 0.001, sig: 3, hint: 'dirty-derivative bandwidth is 1/σ rad/s', get: () => ctx.st.sigma, set: (v) => { ctx.st.sigma = v; ctx.update(); }, disabled: () => ctx.st.deriv !== 'dirty' });
      segmented(imp, {
        label: 'Anti-windup',
        options: [
          { value: 'gate', label: 'integrate when |θ̇| < v̄', title: 'Listing 10.2 (with the abs() bug fixed)' },
          { value: 'backcalc', label: 'back-calculation', title: 'u_I += (u_sat − u_unsat)/k_I, p. 158' },
          { value: 'none', label: 'none' },
        ],
        get: () => ctx.st.antiwindup, set: (v) => { ctx.st.antiwindup = v; ctx.update(); },
      });
      slider(imp, { label: 'v̄', unit: 'rad/s', min: 0.01, max: 2, step: 0.01, sig: 3, get: () => ctx.st.vbar, set: (v) => { ctx.st.vbar = v; ctx.update(); }, disabled: () => ctx.st.antiwindup !== 'gate' });
      PD().sharedControls(imp, ctx);
      segmented(imp, {
        label: 'Extra plot',
        options: [{ value: 'deriv', label: 'θ̇ estimate' }, { value: 'int', label: 'integrator' }],
        get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); },
      });
    },

    splane(ctx) { return { markers: pidMarkers(ctx) }; },
    targets(ctx) { return ctx.S.mode === 'explore' ? { tr: ctx.st.tr, zeta: ctx.st.zeta } : {}; },

    extraPlot(ctx, res) {
      const k = 180 / Math.PI;
      if (ctx.st.extra === 'int') {
        return {
          opts: { title: 'integrator u_I(t)', yLabel: '∫e dt [rad·s]', unit: 'rad·s' },
          data: { series: [{ label: 'integrator ∫e', y: Array.from(res.extras.integrator || [], (v) => v), color: '--series-1' }] },
        };
      }
      return {
        opts: { title: 'θ̇(t)', yLabel: 'θ̇ [°/s]', unit: '°/s' },
        data: {
          series: [
            { label: 'controller estimate', y: Array.from(res.extras.ydotHat || [], (v) => v * k), color: '--series-2', width: 1.5 },
            { label: 'true θ̇', y: res.x.map((x) => x[1] * k), color: '--series-1' },
          ],
          hlines: ctx.st.antiwindup === 'gate' ? [{ y: ctx.st.vbar * k, label: '+v̄', color: '--text-muted' }, { y: -ctx.st.vbar * k, label: '−v̄', color: '--text-muted', fit: false }] : [],
        },
      };
    },

    math(ctx) {
      const st = ctx.st, Ts = ctx.S.sim.Ts;
      const { beta, gamma } = WB.design.dirtyCoeffs(st.sigma, Ts);
      const dg = designedGains(ctx);
      return [
        { title: 'PID from measured output', page: 'p. 155',
          theory: 'u(t) = k_P e(t) + k_I \\int_{-\\infty}^{t} e(\\tau)\\,d\\tau - k_D \\dot{y}(t)' },
        { title: 'Trapezoidal integrator (Tustin)', page: 'p. 157 · Eq. 10.2–10.3',
          theory: 's \\mapsto \\frac{2}{T_s}\\frac{1 - z^{-1}}{1 + z^{-1}},\\quad u_I[n] = u_I[n-1] + \\frac{T_s}{2}\\big(e[n] + e[n-1]\\big)' },
        { title: 'Dirty derivative', page: 'p. 157 · Eq. 10.4',
          theory: 'U_D(s) = \\frac{s}{\\sigma s + 1}Y(s),\\quad \\dot{\\hat y}[n] = \\frac{2\\sigma - T_s}{2\\sigma + T_s}\\dot{\\hat y}[n-1] + \\frac{2}{2\\sigma + T_s}\\big(y[n] - y[n-1]\\big)',
          numbers: `\\sigma = ${tex(st.sigma)},\\; T_s = ${tex(Ts)}:\\quad \\frac{2\\sigma - T_s}{2\\sigma + T_s} = ${tex(beta)},\\quad \\frac{2}{2\\sigma + T_s} = ${tex(gamma)}`,
          spoiler: true },
        { title: 'Anti-windup', page: 'p. 157 · §10.1.1',
          theory: '\\text{(1) integrate only when } |\\dot y| < \\bar v,\\quad \\text{(2) } u_I^+ = u_I + \\frac{1}{k_I}\\big(u_{sat} - u_{unsat}\\big)' },
        { title: 'Gains from t_r, ζ (Listing 10.2)', page: 'p. 161',
          theory: (st.rule === 'tp' ? '\\omega_n = \\frac{\\pi}{2 t_r\\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}') + ',\\quad k_P = \\frac{\\omega_n^2 - a_0}{b_0},\\quad k_D = \\frac{2\\zeta\\omega_n - a_1}{b_0}',
          numbers: `\\omega_n = ${tex(dg.wn)},\\quad k_P = ${tex(dg.kP)},\\quad k_D = ${tex(dg.kD)},\\quad k_I = ${tex(dg.kI)}`,
          spoiler: true,
          note: 'The A.10 solution uses ω_n = π/(2 t_r √(1−ζ²)) with t_r = 0.6, ζ = 0.9, not the 2.2/t_r rule from A.8.' },
        { title: 'Gain-selection guidance', page: 'p. 160 · §10.1.3',
          theory: '\\text{pick } k_P, k_D \\text{ (Ch. 8)},\\; \\text{then raise } k_I \\text{ from 0 until the steady-state error is gone}' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch10;
      const ref = () => {
        const wn = Math.PI / (2 * prob.tr * Math.sqrt(1 - prob.zeta ** 2));
        return { wn, ...PD().gainsFromPoles(ctx.model, PD().polesFromWnZeta(wn, prob.zeta)) };
      };
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'c1', title: `(c) PD gains for t<sub>r</sub> = ${prob.tr} s, ζ = ${prob.zeta}`,
          html: 'Use the A.10 rule ω<sub>n</sub> = π / (2 t<sub>r</sub>√(1−ζ²)).',
          inputs: { wn: 'ω<sub>n</sub>', kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>' },
          check: (v) => PD().checkNumbers(v, ref(), { wn: 'ωn', kP: 'kP', kD: 'kD' }),
          actions: [{
            label: 'Use my gains',
            run: (v) => {
              const kP = PD().num(v.kP), kD = PD().num(v.kD);
              if (kP === null || kD === null) return { ok: false, msg: 'Enter kP and kD first.' };
              ctx.app.setMode('work'); Object.assign(ctx.st, { kP, kD }); ctx.update(); return null;
            },
          }],
          solution: () => { const r = ref(); return [{ tex: `\\omega_n = ${tex(r.wn)},\\quad k_P = ${tex(r.kP)},\\quad k_D = ${tex(r.kD)}` }]; },
        },
        {
          id: 'c2', title: '(c) Dirty-derivative coefficients for σ = 0.05, T<sub>s</sub> = 0.01',
          inputs: { a: '(2σ−T<sub>s</sub>)/(2σ+T<sub>s</sub>)', b: '2/(2σ+T<sub>s</sub>)' },
          check: (v) => PD().checkNumbers(v, { a: 0.09 / 0.11, b: 2 / 0.11 }, {}),
          solution: () => [{ tex: '\\frac{0.09}{0.11} = 0.8182,\\quad \\frac{2}{0.11} = 18.18' }],
        },
        {
          id: 'c3', title: '(c) Tune k<sub>I</sub> to remove the steady-state error',
          html: 'Checks the current simulation (with the plant mismatch in the left panel): the error just before the first reference switch must be under 0.1°, with no saturation-driven windup ringing.',
          check: () => {
            const res = ctx.app.result();
            const tSw = ctx.S.sim.type === 'square' ? ctx.S.sim.tStep + 0.5 / ctx.S.sim.frequency : ctx.S.sim.tEnd;
            const i = Math.min(res.t.length - 1, Math.round((tSw - 0.05) / ctx.S.sim.Ts));
            const e = Math.abs(res.r[i] - res.y[i]) / M.DEG;
            if (!(ctx.gains.kI > 0)) return { ok: false, msg: `kI = 0: error before the switch is ${fmt(e, 3)}°.` };
            return e < 0.1 ? { ok: true, msg: `Error before the switch: ${fmt(e, 3)}°.` } : { ok: false, msg: `Error before the switch: ${fmt(e, 3)}°.` };
          },
          solution: () => [{ html: `The A.10 solution uses k<sub>I</sub> = ${prob.ki} (Listing 10.2, p. 161).` }],
        },
      ]);
    },
  };

  // ---------------------------------------------------- Appendix P.6 (root locus) --
  WB.chapters.p6 = {
    id: 'p6', num: 10.5, tab: 'App. P.6', short: 'P.6', title: 'Root locus vs. k_I', pages: 'pp. 465–474',

    defaults() { return { arch: 'output', comp: 'fl', deriv: 'state', antiwindup: 'none', tr: 0.8, zeta: 0.707, rule: '2.2', kIx: 0.05, kMaxFactor: 2 }; },
    simDefaults(sys) { return sys.problems.p6.sim; },
    gains(ctx) { return designedGains(ctx); },
    controller: (ctx, o) => makePID(ctx, o),

    evans(ctx) {
      const { model } = ctx;
      const g = designedGains(ctx);
      const den = [1, model.a1 + model.b0 * g.kD, model.a0 + model.b0 * g.kP, 0];
      const kCrit = den[1] * den[2] / model.b0;  // Routh: c2 c1 > c0
      return { den, num: [model.b0], kCrit, g };
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'PD from A.8, then add k_I', 'p. 470');
      PD().sharedControls(sec, ctx);
      slider(sec, { label: 't<sub>r</sub>', unit: 's', min: 0.2, max: 3, step: 0.005, get: () => ctx.st.tr, set: (v) => { ctx.st.tr = v; ctx.update(); } });
      slider(sec, { label: 'ζ', min: 0.2, max: 1.5, step: 0.005, get: () => ctx.st.zeta, set: (v) => { ctx.st.zeta = v; ctx.update(); } });
      slider(sec, { label: 'k<sub>I</sub>', min: 0, max: 2, step: 0.001, get: () => ctx.st.kIx, set: (v) => { ctx.st.kIx = v; ctx.update(); } });
      slider(sec, { label: 'locus to', unit: '× kI,crit', min: 0.2, max: 5, step: 0.1, sig: 2, get: () => ctx.st.kMaxFactor, set: (v) => { ctx.st.kMaxFactor = v; ctx.update(); } });
      sec.append(el('p', { class: 'muted small', text: 'Drag a closed-loop pole along the locus to set k_I.' }));
      readout(sec, ctx);
    },

    splane(ctx) {
      const ev = this.evans(ctx);
      const kMax = Math.max(ev.kCrit * ctx.st.kMaxFactor, ctx.st.kIx * 1.2, 1e-3);
      const loci = WB.tf.rootLocus(ev.den, ev.num, kMax);
      const markers = L.roots(ev.den).map((p, i) => ({ ...p, kind: 'ol', label: `pole of L(s) ${i + 1}` }));
      L.roots(pidCharPoly(ctx.model, ev.g)).forEach((p, i) => markers.push({ ...p, kind: 'cl', label: `closed-loop pole at kI = ${fmt(ev.g.kI, 3)}`, dragId: i }));
      const fitR = Math.max(...L.roots(ev.den).map((p) => Math.hypot(p.re, p.im))) * 1.6;
      return { markers, loci, fitR };
    },

    // Dragging: pick the k_I whose root is closest to the pointer.
    onPoleDrag(ctx, id, re, im) {
      const ev = this.evans(ctx);
      const kMax = Math.max(ev.kCrit * ctx.st.kMaxFactor, 1e-3);
      let bestK = ctx.st.kIx, bd = Infinity;
      for (let i = 0; i <= 400; i++) {
        const k = kMax * i / 400;
        for (const q of L.roots(L.polyAdd(ev.den, L.polyScale(ev.num, k)))) {
          const d = Math.hypot(q.re - re, q.im - im);
          if (d < bd) { bd = d; bestK = k; }
        }
      }
      ctx.st.kIx = bestK;
      ctx.update();
    },

    math(ctx) {
      const ev = this.evans(ctx);
      return [
        { title: 'Closed loop with PID (derivative on output)', page: 'p. 470',
          theory: '\\Delta_{cl}(s) = s^3 + \\frac{3b + 3k_D}{m\\ell^2}s^2 + \\frac{3k_P}{m\\ell^2}s + \\frac{3k_I}{m\\ell^2}',
          numbers: `\\Delta_{cl}(s) = ${WB.tf.polyTex(pidCharPoly(ctx.model, ev.g))}`, spoiler: true },
        { title: 'Evans form', page: 'p. 466, p. 470',
          theory: '1 + k_I\\,L(s) = 0,\\quad L(s) = \\frac{3/m\\ell^2}{s^3 + \\frac{3b+3k_D}{m\\ell^2}s^2 + \\frac{3k_P}{m\\ell^2}s}',
          numbers: `L(s) = \\frac{${tex(ctx.model.b0)}}{${WB.tf.polyTex(ev.den)}},\\quad k_P = ${tex(ev.g.kP)},\\; k_D = ${tex(ev.g.kD)}`, spoiler: true },
        { title: 'Where the locus crosses into the RHP', page: 'Routh–Hurwitz (not in the book)',
          theory: 's^3 + c_2 s^2 + c_1 s + c_0 \\text{ is stable iff } c_2, c_1, c_0 > 0 \\text{ and } c_2 c_1 > c_0',
          numbers: `k_{I,crit} = \\frac{c_2 c_1}{b_0} = ${tex(ev.kCrit)}`, spoiler: true,
          note: 'The book only uses rlocus (p. 471). Larger k_I always pushes this second-order-plus-integrator loop unstable (p. 160).' },
      ];
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.p6;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: 'Evans form: L(s) = c / (s³ + d<sub>2</sub>s² + d<sub>1</sub>s)',
          html: 'Uses the PD gains from the current t<sub>r</sub>, ζ (A.8: 0.8 s, 0.707).',
          inputs: { c: 'c', d2: 'd<sub>2</sub>', d1: 'd<sub>1</sub>' },
          check: (v) => { const ev = this.evans(ctx); return PD().checkNumbers(v, { c: ctx.model.b0, d2: ev.den[1], d1: ev.den[2] }, {}); },
          solution: () => { const ev = this.evans(ctx); return [{ tex: `L(s) = \\frac{${tex(ctx.model.b0)}}{${WB.tf.polyTex(ev.den)}}` }]; },
        },
        {
          id: 'b', title: 'Largest stable k<sub>I</sub>',
          inputs: { k: 'k<sub>I,crit</sub>' },
          check: (v) => PD().checkNumbers(v, { k: this.evans(ctx).kCrit }, { k: 'kI,crit' }),
          solution: () => [{ tex: `k_{I,crit} = ${tex(this.evans(ctx).kCrit)}` }],
        },
        {
          id: 'c', title: 'Pick k<sub>I</sub> that barely moves the PD poles',
          html: 'Checks the current k<sub>I</sub>: the complex pair must stay within 10% (in |p|) of the PD-only poles, and the new real pole must be slower than them.',
          check: () => {
            const ev = this.evans(ctx);
            const pd = PD().clPoles(ctx.model, ev.g.kP, ev.g.kD);
            const cl = L.roots(pidCharPoly(ctx.model, ev.g));
            if (!(ev.g.kI > 0)) return { ok: false, msg: 'Set kI > 0.' };
            const cpx = cl.filter((p) => Math.abs(p.im) > 1e-6);
            const real = cl.filter((p) => Math.abs(p.im) <= 1e-6);
            if (cpx.length !== 2) return { ok: false, msg: 'The PD pair has split into real poles; kI is too large.' };
            const ratio = Math.hypot(cpx[0].re, cpx[0].im) / Math.hypot(pd[0].re, pd[0].im);
            const slow = real.length === 1 && Math.abs(real[0].re) < Math.abs(pd[0].re);
            const ok = Math.abs(ratio - 1) < 0.1 && slow;
            return { ok, msg: `|p| ratio ${fmt(ratio, 3)}, real pole ${real.length ? fmtPole(real[0]) : '—'}.` };
          },
          solution: () => [{ html: 'With the A.8 gains, k<sub>I</sub> ≈ 0.02–0.05 keeps the pair close (k<sub>I</sub> = 0.05 gives −1.65 ± 1.70j and −0.60). The book stops at the rlocus command (p. 471).' }],
        },
      ]);
    },
  };

  WB.pid = { makePID, pidCharPoly, shapedReference };
})();
