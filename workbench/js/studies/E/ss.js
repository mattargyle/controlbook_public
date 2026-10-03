// Study E, Chapters 11–14: full state feedback, integral augmentation, observers
// and disturbance observers on the 4-state linear model of E.6
//   x̃ = x − x_e,  F = F_ff + F̃,  ẋ̃ = A x̃ + B F̃,  ỹ = C x̃ = (z̃, θ̃)
// with F_ff = F_e (Jacobian A, as E.6 and E.11 ask) or F_fl(z) (then A₄₁ = 0).
window.WB = window.WB || {};
WB.studies = WB.studies || {};
WB.studies.E = WB.studies.E || { chapters: {} };

(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const E = WB.E;
  const PD = () => WB.pd;
  const CH = WB.studies.E.chapters;

  const knobsOf = (st) => ({ trTh: st.trTh, zetaTh: st.zetaTh, trZ: st.trZ, zetaZ: st.zetaZ, pI: st.pI, obsFactor: st.obsFactor, zetaObs: st.zetaObs, pD: st.pD, comp: st.comp, obsMode: st.obsMode });
  // Work-mode starting gains: a sluggish, underdamped design (ζ = 0.6 breaks the E.11(a) rule).
  const W0K = { trTh: 1.5, zetaTh: 0.6, trZ: 5, zetaZ: 0.6, pI: -0.4, obsFactor: 3, zetaObs: 0.7, pD: -1, comp: 'eq', obsMode: 'decoupled' };

  function det(Am) {
    const A = Am.map((r) => r.slice()), n = A.length;
    let d = 1;
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[piv][c])) piv = r;
      if (Math.abs(A[piv][c]) < 1e-300) return 0;
      if (piv !== c) { [A[c], A[piv]] = [A[piv], A[c]]; d = -d; }
      d *= A[c][c];
      for (let r = c + 1; r < n; r++) { const f = A[r][c] / A[c][c]; for (let j = c; j < n; j++) A[r][j] -= f * A[c][j]; }
    }
    return d;
  }

  function workDefaults(sys, level) {
    const p = { ...Object.fromEntries(sys.params.map((q) => [q.key, q.value])), ...sys.constants };
    const d = E.ssDesign(p, W0K, level === 'sf' ? 'sf' : level);
    return { K: d.K.slice(), kr: d.kr ?? 0, ki: d.ki ?? 0, L: d.L ? d.L.map((r) => r.slice()) : null };
  }

  function gainsFor(ctx, level) {
    if (ctx.S.mode === 'explore') return E.ssDesign(ctx.pModel, knobsOf(ctx.st), level);
    const w = ctx.st.w;
    return { K: w.K, kr: w.kr, ki: w.ki, L: w.L, lin: ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp }) };
  }
  function toWork(ctx, level) {
    const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), level);
    ctx.st.w = { K: d.K.slice(), kr: d.kr ?? ctx.st.w.kr, ki: d.ki ?? ctx.st.w.ki, L: d.L ? d.L.map((r) => r.slice()) : ctx.st.w.L };
  }

  // Closed-loop and observer eigenvalues for the current gains.
  function clPoles(ctx, level) {
    const { A, B, C } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
    const g = ctx.gains;
    if (level === 'sf') return L.eig(L.sub(A, L.mul(B, [g.K])));
    const { A1, B1 } = E.augI(A, B);
    return L.eig(L.sub(A1, L.mul(B1, [[...g.K, g.ki]])));
  }
  function obsPolesOf(ctx, level, Lg = ctx.gains.L) {
    if (!Lg) return [];
    const { A, B, C } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
    if (level === 'obs') return L.eig(L.sub(A, L.mul(Lg, C)));
    if (level === 'dobs') { const { A2, C2 } = E.augD(A, B, C); return L.eig(L.sub(A2, L.mul(Lg, C2))); }
    return [];
  }

  // ------------------------------------------------------------- controls --
  function compControl(parent, ctx) {
    segmented(parent, {
      label: 'Equilibrium force and model',
      options: [
        { value: 'eq', label: 'F = F<sub>e</sub> + F̃, Jacobian A', title: 'consistent with E.6 / E.11' },
        { value: 'fl', label: 'F = F<sub>fl</sub>(z) + F̃, A₄₁ = 0', title: 'feedback linearization; the m1 g z̃ coupling is cancelled' },
      ],
      get: () => ctx.st.comp, set: (v) => { ctx.st.comp = v; ctx.update(); },
    });
  }

  function poleKnobs(parent, ctx, { pI = false } = {}) {
    E.knob(parent, ctx, 'trTh', 't<sub>r,θ</sub> (fast pair)', 0.05, 3, 0.005, { unit: 's' });
    E.knob(parent, ctx, 'zetaTh', 'ζ<sub>θ</sub>', 0.3, 1.5, 0.005);
    E.knob(parent, ctx, 'trZ', 't<sub>r,z</sub> (slow pair)', 0.1, 15, 0.01, { unit: 's' });
    E.knob(parent, ctx, 'zetaZ', 'ζ<sub>z</sub>', 0.3, 1.5, 0.005);
    if (pI) E.knob(parent, ctx, 'pI', 'p<sub>I</sub>', -10, -0.01, 0.01, { sig: 3 });
  }
  function obsKnobs(parent, ctx, { pD = false } = {}) {
    E.knob(parent, ctx, 'obsFactor', 'observer / controller ω<sub>n</sub>', 1, 20, 0.1, { sig: 3 });
    E.knob(parent, ctx, 'zetaObs', 'ζ<sub>obs</sub>', 0.3, 1.5, 0.005);
    if (pD) E.knob(parent, ctx, 'pD', 'p<sub>d</sub>', -40, -0.1, 0.1, { sig: 3 });
    segmented(parent, {
      label: 'Observer gain structure',
      options: [
        { value: 'decoupled', label: 'decoupled (uses z and θ)', title: 'L cancels the A cross-couplings: a z block and a θ block' },
        { value: 'zonly', label: 'from z only', title: 'Ackermann on C = [1 0 0 0]; ignores the θ measurement' },
      ],
      get: () => ctx.st.obsMode, set: (v) => { ctx.st.obsMode = v; ctx.update(); },
    });
  }

  function workGrid(parent, ctx, { kr = false, ki = false, Lrows = 0 } = {}) {
    const st = ctx.st;
    const items = [0, 1, 2, 3].map((i) => ({ label: `K<sub>${i + 1}</sub>`, get: () => st.w.K[i], set: (v) => { st.w.K[i] = v; } }));
    if (kr) items.push({ label: 'k<sub>r</sub>', get: () => st.w.kr, set: (v) => { st.w.kr = v; } });
    if (ki) items.push({ label: 'k<sub>I</sub>', get: () => st.w.ki, set: (v) => { st.w.ki = v; } });
    E.numGrid(parent, items, () => ctx.update());
    if (Lrows) {
      parent.append(el('p', { class: 'muted small', text: 'Observer gain L (rows z̃, θ̃, z̃̇, θ̃̇' + (Lrows === 5 ? ', d' : '') + '; columns: z and θ innovations)' }));
      const li = [];
      for (let r = 0; r < Lrows; r++) for (let c = 0; c < 2; c++) li.push({ label: `L<sub>${r + 1}${c + 1}</sub>`, get: () => st.w.L[r][c], set: (v) => { st.w.L[r][c] = v; } });
      E.numGrid(parent, li, () => ctx.update());
    }
  }

  function gainsReadout(parent, ctx, level) {
    E.readout(parent, () => {
      const g = ctx.gains;
      const rows = g.K.map((k, i) => [`K${i + 1}`, fmt(k, 4)]);
      if (level === 'sf') rows.push(['kr', fmt(g.kr, 4)]); else rows.push(['kI', fmt(g.ki, 4)]);
      if (g.L) g.L.forEach((r, i) => rows.push([`L${i + 1}·`, `${fmt(r[0], 4)}, ${fmt(r[1], 4)}`]));
      return rows;
    });
  }

  // ------------------------------------------------------------- markers --
  function markers(ctx, level) {
    const { A } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
    const explore = ctx.S.mode === 'explore';
    const mk = L.eig(A).map((q, i) => ({ ...q, kind: 'ol', label: `open-loop pole ${i + 1}` }));
    const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), level);
    const near = (q, list) => list.reduce((b, c, j) => (Math.hypot(c.re - q.re, c.im - q.im) < Math.hypot(list[b].re - q.re, list[b].im - q.im) ? j : b), 0);
    clPoles(ctx, level).forEach((q, i) => {
      const j = near(q, d.poles);
      mk.push({ ...q, kind: 'cl', label: j < 2 ? 'controller pole (fast pair)' : j < 4 ? 'controller pole (slow pair)' : 'integrator pole', dragId: explore ? (j < 2 ? 0 : j < 4 ? 1 : 2) : undefined });
    });
    obsPolesOf(ctx, level).forEach((q) => mk.push({ ...q, kind: 'obs', label: 'observer pole', dragId: explore ? (Math.abs(q.im) > 1e-9 ? 10 : 12) : undefined, noFit: ctx.st.zoom === 'ctrl' }));
    if (!explore) for (const q of d.poles) mk.push({ ...q, kind: 'target', label: 'target pole (specs)' });
    return mk;
  }

  function onDrag(ctx, id, re, im) {
    const st = ctx.st;
    re = Math.min(-0.005, re);
    const wn = Math.hypot(re, im), zeta = Math.max(0.3, Math.min(1, -re / wn));
    if (id === 0) { st.trTh = 2.2 / wn; st.zetaTh = zeta; }
    else if (id === 1) { st.trZ = 2.2 / wn; st.zetaZ = zeta; }
    else if (id === 2) st.pI = re;
    else if (id === 10) { st.obsFactor = Math.max(1, wn / (2.2 / st.trTh)); st.zetaObs = zeta; }
    else if (id === 12) { if (st.pD !== undefined) st.pD = re; }
    ctx.update();
  }

  // ---------------------------------------------------------- math cards --
  function ssCard(ctx) {
    const { A, B } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
    return {
      title: `State-space model (${ctx.st.comp === 'fl' ? 'F_fl(z), A₄₁ = 0' : 'Jacobian, F_e'})`, page: 'p. 387 · E.6, p. 183',
      theory: '\\dot{\\tilde x} = A\\tilde x + B\\tilde F,\\quad \\tilde x = x - x_e,\\; x_e = (\\tfrac{\\ell}{2}, 0, 0, 0),\\quad \\tilde F = F - F_{ff}',
      numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)}`,
      spoiler: true,
    };
  }
  function ctrbCard(A, B, title, page) {
    const Cm = L.ctrb(A, B);
    return { title, page, theory: '\\mathcal{C}_{A,B} = \\begin{bmatrix} B & AB & A^2B & A^3B\\end{bmatrix},\\quad \\text{controllable} \\iff \\operatorname{rank}\\mathcal{C}_{A,B} = n',
      numbers: `\\operatorname{rank}\\mathcal{C} = ${L.rank(Cm)},\\quad \\det\\mathcal{C} = ${tex(det(Cm))}`, spoiler: true };
  }
  function polesCard(ctx, d, level) {
    return {
      title: 'Desired closed-loop poles', page: 'p. 190 (B.11), p. 113 · Eq. 8.5',
      theory: '\\Delta^d = (s^2 + 2\\zeta_\\theta\\omega_{n_\\theta}s + \\omega_{n_\\theta}^2)(s^2 + 2\\zeta_z\\omega_{n_z}s + \\omega_{n_z}^2)' + (level === 'sf' ? '' : '(s - p_I)') + ',\\quad \\omega_n = \\frac{2.2}{t_r}',
      numbers: `p = ${d.poles.map((q) => texPole(q)).join(',\\;')}`,
      spoiler: true,
      note: 'E.11(a): every pole needs ωₙ > ωₙ,z = 0.22 rad/s and ζ > 0.707 (the E.8 outer loop).',
    };
  }

  // ------------------------------------------------------------- problems --
  const ansKey = (ctx, id) => E.answersOf(ctx, id);
  // Poles typed into E.11(a), validated; null if missing.
  function userPoles(ctx) {
    const a = ansKey(ctx, 'E.11');
    const ps = ['p1', 'p2', 'p3', 'p4'].map((k) => M.parseComplex(a[`a.${k}`]));
    return ps.every(Boolean) ? ps : null;
  }
  function validatePoles(ps) {
    if (!ps || ps.some((q) => !q)) return { ok: false, msg: 'Enter four poles (e.g. -1+0.5j).' };
    for (const q of ps) {
      if (Math.abs(q.im) > 1e-9 && !ps.some((c) => M.close(c.re, q.re) && M.close(c.im, -q.im))) return { ok: false, msg: `${M.fmtPole(q)} needs its conjugate.` };
    }
    const wnz = 2.2 / 10, bad = [];
    for (const q of ps) {
      const wn = Math.hypot(q.re, q.im), zeta = -q.re / wn;
      if (!(q.re < 0)) bad.push(`${M.fmtPole(q)} is not stable`);
      else if (wn <= wnz) bad.push(`|${M.fmtPole(q)}| = ${fmt(wn, 3)} ≤ ωn,z = 0.22`);
      else if (zeta < 0.707 - 1e-3) bad.push(`${M.fmtPole(q)} has ζ = ${fmt(zeta, 3)} < 0.707`);
    }
    return bad.length ? { ok: false, msg: bad.join('; ') + '.' } : { ok: true, msg: 'Valid choice. Parts (d) and E.12 use these poles.' };
  }
  function errorBefore(ctx) {
    const res = ctx.app.result();
    const i = E.beforeSwitch(ctx, res);
    return { e: Math.abs(res.rAll[0][i] - res.yAll[0][i]), t: res.t[i], res };
  }
  function useAction(ctx, label, keys, apply) {
    return {
      label,
      run: (v) => {
        const vals = keys.map((k) => PD().num(v[k]));
        if (vals.some((x) => x === null)) return { ok: false, msg: `Enter ${keys.join(', ')} first.` };
        ctx.app.setMode('work'); apply(vals); ctx.update(); return null;
      },
    };
  }
  const Linputs = (rows) => Object.fromEntries(Array.from({ length: rows }, (_, r) => [0, 1].map((c) => [`L${r + 1}${c + 1}`, `L<sub>${r + 1}${c + 1}</sub>`])).flat());
  const readL = (v, rows) => {
    const Lg = [];
    for (let r = 0; r < rows; r++) {
      const row = [PD().num(v[`L${r + 1}1`]), PD().num(v[`L${r + 1}2`])];
      if (row.some((x) => x === null)) return null;
      Lg.push(row);
    }
    return Lg;
  };

  // --------------------------------------------------------- chapter base --
  function base(level, num, title, pages, extra) {
    return Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages, level,
      gains(ctx) { return gainsFor(ctx, level); },
      toWork(ctx) { toWork(ctx, level); },
      controller(ctx, o) { return E.makeSS(ctx, ctx.gains, level, { comp: ctx.st.comp, est: ctx.st.est, antiwindup: ctx.st.antiwindup, dobs: ctx.st.dobs, zhat0: ctx.st.zhat0 || 0 }, o); },
      splane(ctx) { return { markers: markers(ctx, level), kindNames: { cl: 'controller pole', obs: 'observer pole', ol: 'open-loop pole', target: 'target pole' } }; },
      onPoleDrag: onDrag,
      targets(ctx) { return level === 'sf' ? { tr: ctx.st.trZ } : {}; },
    }, extra);
  }
  function defaultsFor(sys, key, level, extra = {}) {
    const p = sys.problems[key];
    return { comp: 'eq', est: 'true', antiwindup: 'clamp', obsMode: 'decoupled', dobs: true, zhat0: 0, zoom: 'all',
      trTh: p.trTh, zetaTh: p.zetaTh, trZ: p.trZ, zetaZ: p.zetaZ, pI: p.pI ?? -1, obsFactor: p.obsFactor ?? 5, zetaObs: p.zetaObs ?? 0.8, pD: p.pD ?? -5,
      w: workDefaults(sys, level), ...extra };
  }
  function estimateSeries(ctx, res, sc, oi) {
    const key = oi === 0 ? 'zhat' : 'thhat';
    if (!res.extras[key]) return [];
    return [{ label: `${oi === 0 ? 'ẑ' : 'θ̂'} (observer)`, y: sc(res.extras[key]), color: '--series-3', dash: [3, 3], width: 2 }];
  }

  // ------------------------------------------------------------ Chapter 11 --
  CH.ch11 = base('sf', 11, 'Full state feedback', 'pp. 173–196', {
    defaults(sys) { return defaultsFor(sys, 'ch11', 'sf'); },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    linearSim(ctx, c) {
      const lin = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const plant = WB.design.linearPlant(lin.A, lin.B, lin.C, { xe: [lin.ze, 0, 0, 0] });
      return WB.sim.simulate({ ...c, disturbance: null, noise: null, plant, controller: E.makeSS(ctx, ctx.gains, 'sf', { comp: ctx.st.comp }, { linear: true }) });
    },
    linearLabel: 'linear model',
    buildControls(parent, ctx) {
      const sec = section(parent, 'F̃ = −K x̃ + k_r z̃_r', 'p. 183 · Eq. 11.38');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Where x comes from',
        options: [{ value: 'true', label: 'true state' }, { value: 'dirty', label: 'z, θ + dirty derivatives' }],
        get: () => ctx.st.est, set: (v) => { ctx.st.est = v; ctx.update(); },
      });
      if (ctx.S.mode === 'work') workGrid(sec, ctx, { kr: true });
      const spec = section(parent, ctx.S.mode === 'work' ? 'Specs (target rings)' : 'Pole knobs', 'p. 389 · E.11(a)');
      poleKnobs(spec, ctx);
      if (ctx.S.mode === 'explore') gainsReadout(spec, ctx, 'sf');
    },
    math(ctx) {
      const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'sf');
      const { A, B } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const g = ctx.gains;
      return [
        ssCard(ctx), ctrbCard(A, B, 'Controllability', 'p. 180 · Eq. 11.29'), polesCard(ctx, d, 'sf'),
        { title: 'Pole placement (Ackermann)', page: 'p. 182 · Eq. 11.32',
          theory: 'K = (\\alpha - a_A)\\,\\mathcal{A}_A^{-1}\\,\\mathcal{C}_{A,B}^{-1},\\quad \\Delta_{ol}(s) = \\det(sI - A)',
          numbers: `\\Delta_{ol} = ${WB.tf.polyTex(L.charPoly(A))},\\quad K = ${texMat([d.K])}`, spoiler: true },
        { title: 'Reference gain', page: 'p. 182 · Eq. 11.35',
          theory: 'k_r = \\frac{-1}{C_r(A - BK)^{-1}B},\\quad C_r = \\begin{bmatrix}1 & 0 & 0 & 0\\end{bmatrix}',
          numbers: `k_r = ${tex(d.kr)}`, spoiler: true },
        { title: 'Control law', page: 'p. 183 · Eq. 11.38',
          theory: 'F = F_{ff} - K(x - x_e) + k_r(z_r - z_e)',
          numbers: `F = F_{ff} - ${texMat([g.K])}\\tilde x + ${tex(g.kr)}\\,\\tilde z_r` },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch11;
      const jac = () => ctx.sys.linear(ctx.pModel, { comp: 'eq' });
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Four closed-loop poles',
          html: 'Complex poles in conjugate pairs, e.g. <code>-1.2+0.9j</code> and <code>-1.2-0.9j</code>. From E.8: ω<sub>n<sub>z</sub></sub> = 2.2/10 = 0.22 rad/s, ζ<sub>z</sub> = 0.707.',
          inputs: { p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>', p3: 'p<sub>3</sub>', p4: 'p<sub>4</sub>' },
          check: (v) => validatePoles(['p1', 'p2', 'p3', 'p4'].map((k) => M.parseComplex(v[k]))),
          solution: () => [{ html: 'Many answers work. One choice, in the spirit of B.11 (p. 190): a fast pair from t<sub>r</sub> = 0.5 s and a slow pair from t<sub>r</sub> = 1.5 s, both with ζ = 0.8: −3.52 ± 2.64j and −1.173 ± 0.88j (the Explore defaults). Faster poles need more force: watch the F plot as you move them.' }],
        },
        {
          id: 'c', title: '(c) Controllability (A, B from E.6)',
          inputs: { rank: 'rank 𝒞<sub>A,B</sub>', det: 'det 𝒞<sub>A,B</sub>' },
          check: (v) => { const Cm = L.ctrb(jac().A, jac().B); return PD().checkNumbers(v, { rank: L.rank(Cm), det: det(Cm) }, { det: 'det' }); },
          solution: () => {
            const { A, B, b0 } = jac(), Cm = L.ctrb(A, B);
            return [{ tex: `\\mathcal{C}_{A,B} = ${texMat(Cm)},\\quad \\det = -g^2b_0^4 = ${tex(det(Cm))} \\ne 0 \\Rightarrow \\text{rank } 4` }, { html: `b₀ = ℓ/J<sub>e</sub> = ${fmt(b0, 4)}. The sign of the determinant depends on column order; its magnitude is what matters.` }];
          },
        },
        {
          id: 'd', title: '(d) K and k<sub>r</sub> for your poles from (a) (Jacobian A, F = F<sub>e</sub> + F̃)',
          inputs: { K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', kr: 'k<sub>r</sub>' },
          check: (v) => {
            const ps = userPoles(ctx), ok = validatePoles(ps);
            if (!ok.ok) return { ok: false, msg: `Part (a) first: ${ok.msg}` };
            const d = E.ssDesign(ctx.pModel, { comp: 'eq' }, 'sf', ps);
            return PD().checkNumbers(v, { K1: d.K[0], K2: d.K[1], K3: d.K[2], K4: d.K[3], kr: d.kr }, {});
          },
          actions: [useAction(ctx, 'Use my gains', ['K1', 'K2', 'K3', 'K4', 'kr'], (x) => { ctx.st.comp = 'eq'; ctx.st.w.K = x.slice(0, 4); ctx.st.w.kr = x[4]; })],
          solution: () => {
            const ps = userPoles(ctx);
            const use = validatePoles(ps).ok ? ps : E.ssPoles({ trTh: 0.5, zetaTh: 0.8, trZ: 1.5, zetaZ: 0.8 });
            const d = E.ssDesign(ctx.pModel, { comp: 'eq' }, 'sf', use);
            return [
              { html: validatePoles(ps).ok ? 'For your poles from (a):' : 'Part (a) is not filled in yet, so this uses −3.52 ± 2.64j, −1.173 ± 0.88j:' },
              { tex: `\\Delta^d = ${WB.tf.polyTex(L.polyFromRoots(use))},\\quad \\Delta_{ol} = ${WB.tf.polyTex(L.charPoly(jac().A))}` },
              { tex: `K = ${texMat([d.K])},\\quad k_r = ${tex(d.kr)}` },
              { html: 'Cross-checked against python-control <code>place</code> (tools/regress_E.py).' },
            ];
          },
        },
        {
          id: 'e', title: '(e) Simulate',
          html: 'Passes when |z<sub>r</sub> − z| just before the first switch is under 2 mm and F never saturates.',
          check: () => {
            const { e, t, res } = errorBefore(ctx);
            let peak = 0; for (const u of res.uDemand) peak = Math.max(peak, Math.abs(u));
            const ok = e < 0.002 && peak <= ctx.sys.uLimit(ctx.pModel);
            return { ok, msg: `error ${fmt(1000 * e, 3)} mm at t = ${fmt(t, 3)} s, peak |F| = ${fmt(peak, 3)} N.` };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 12 --
  CH.ch12 = base('sfi', 12, 'Integrator with state feedback', 'pp. 197–214', {
    defaults(sys) { return defaultsFor(sys, 'ch12', 'sfi'); },
    simDefaults(sys) { return { ...sys.problems.ch12.sim, mismatch: sys.problems.ch12.mismatch }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'F̃ = −K x̃ − k_I ∫(z_r − z)', 'p. 199');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Anti-windup (E.12a)',
        options: [{ value: 'clamp', label: 'hold integrator while saturated' }, { value: 'none', label: 'none' }],
        get: () => ctx.st.antiwindup, set: (v) => { ctx.st.antiwindup = v; ctx.update(); },
      });
      if (ctx.S.mode === 'work') workGrid(sec, ctx, { ki: true });
      const spec = section(parent, ctx.S.mode === 'work' ? 'Specs (target rings)' : 'Pole knobs', 'p. 389 · E.12(c)');
      poleKnobs(spec, ctx, { pI: true });
      if (ctx.S.mode === 'explore') gainsReadout(spec, ctx, 'sfi');
    },
    extraPlot(ctx, res) {
      return { opts: { title: 'integrator x_I(t)', yLabel: 'x_I [m·s]', unit: 'm·s' }, data: { series: [{ label: 'x_I = ∫(z_r − z) dt', y: Array.from(res.extras.integrator || []), color: '--series-1' }] } };
    },
    math(ctx) {
      const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'sfi');
      const { A, B } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const { A1, B1 } = E.augI(A, B);
      return [
        ssCard(ctx),
        { title: 'Augmented system', page: 'p. 198 · Eq. 12.1',
          theory: '\\dot x_I = z_r - C_r x,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C_r & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\ 0\\end{bmatrix}',
          numbers: `A_1 = ${texMat(A1)}`, spoiler: true },
        ctrbCard(A1, B1, 'Controllability of (A₁, B₁)', 'p. 198'),
        polesCard(ctx, d, 'sfi'),
        { title: 'Gains', page: 'p. 199–201',
          theory: '\\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad F = F_{ff} - K\\tilde x - k_I\\textstyle\\int_0^t (z_r - z)\\,d\\tau',
          numbers: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch12;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Gains for your E.11(a) poles plus an integrator pole p<sub>I</sub> (Jacobian A)',
          inputs: { pI: 'p<sub>I</sub>', K1: 'K<sub>1</sub>', K2: 'K<sub>2</sub>', K3: 'K<sub>3</sub>', K4: 'K<sub>4</sub>', ki: 'k<sub>I</sub>' },
          check: (v) => {
            const ps = userPoles(ctx), ok = validatePoles(ps);
            if (!ok.ok) return { ok: false, msg: `Fill in E.11(a) first: ${ok.msg}` };
            const pI = PD().num(v.pI);
            if (pI === null || pI >= 0) return { ok: false, msg: 'Enter a negative pI.' };
            const d = E.ssDesign(ctx.pModel, { comp: 'eq', pI }, 'sfi', ps);
            return PD().checkNumbers(v, { K1: d.K[0], K2: d.K[1], K3: d.K[2], K4: d.K[3], ki: d.ki }, {});
          },
          actions: [useAction(ctx, 'Use my gains', ['K1', 'K2', 'K3', 'K4', 'ki'], (x) => { ctx.st.comp = 'eq'; ctx.st.w.K = x.slice(0, 4); ctx.st.w.ki = x[4]; })],
          solution: () => {
            const ps = userPoles(ctx);
            const use = validatePoles(ps).ok ? ps : E.ssPoles({ trTh: 0.5, zetaTh: 0.8, trZ: 1.5, zetaZ: 0.8 });
            const pI = PD().num(ansKey(ctx, prob.id)['a.pI']) ?? prob.pI;
            const d = E.ssDesign(ctx.pModel, { comp: 'eq', pI }, 'sfi', use);
            return [
              { tex: `p = ${use.map((q) => texPole(q)).join(',\\;')},\\; p_I = ${tex(pI)}` },
              { tex: `K = ${texMat([d.K])},\\quad k_I = ${tex(d.ki)}` },
              { html: 'k<sub>I</sub> comes out positive with this sign convention (F̃ = −K x̃ − k<sub>I</sub>∫(z<sub>r</sub> − z)) because positive F tilts the beam up and moves the block toward the pivot.' },
            ];
          },
        },
        {
          id: 'c', title: '(b, c) Tracking with d = 1 N and 20% parameter error',
          html: 'Passes when the block stays on the beam (0 ≤ z ≤ ℓ of the true plant) for the whole run and |z<sub>r</sub> − z| just before the first switch is under 2 mm, with the current disturbance and mismatch.',
          check: () => {
            const { e, t, res } = errorBefore(ctx), S = ctx.S;
            const ob = E.onBeam(ctx, res);
            return { ok: ob.ok && e < 0.002, msg: `${ob.msg}; error ${fmt(1000 * e, 3)} mm at t = ${fmt(t, 3)} s (d = ${fmt(S.sim.dist, 3)} N).` };
          },
          solution: () => [
            { html: 'The slower poles of E.11 (pairs from t<sub>r</sub> = 0.5 and 1.5 s, p<sub>I</sub> = −1) are not enough: with d = 1 N from t = 0 and the default mismatch, the block dips to about −0.16 m (off the pivot end) before the integrator catches up, and only 7 of 30 random α = 0.2 draws pass.' },
            { html: 'Faster poles fix it: a pair from t<sub>r</sub> = 0.2 s (ζ = 0.8), a pair from t<sub>r</sub> = 0.7 s (ζ = 0.85), and p<sub>I</sub> = −2 (the Explore defaults). Robustness, tested in Python and JS: with d = +1 N, 88 of 90 random draws pass (30/30, 29/30, 29/30 for three seeds). With d = −1 N, which adds to the load, 24 of 30 pass (checked in Python). The peak force stays under the 15 N limit for the default draw.' },
            { html: 'Without the integrator (E.11 gains), a constant d shifts the block by d/k<sub>r</sub> at steady state: with k<sub>r</sub> ≈ −1.6, 1 N moves it about 0.6 m, off the beam.' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 13 --
  CH.ch13 = base('obs', 13, 'Observers', 'pp. 215–238', {
    defaults(sys) { return defaultsFor(sys, 'ch13', 'obs'); },
    simDefaults(sys) { return sys.problems.ch13.sim; },
    outputSeries: estimateSeries,
    extraPlot(ctx, res) {
      return {
        opts: { title: 'ż(t) and estimate', yLabel: 'ż [m/s]', unit: 'm/s' },
        data: { series: [
          { label: 'ż̂ (observer)', y: Array.from(res.extras.zdhat || []), color: '--series-3', dash: [3, 3], width: 2 },
          { label: 'true ż', y: res.x.map((x) => x[2]), color: '--series-1' },
        ] },
      };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂)', 'p. 222 · Fig. 13-3');
      compControl(sec, ctx);
      if (ctx.S.mode === 'work') workGrid(sec, ctx, { ki: true, Lrows: 4 });
      const spec = section(parent, ctx.S.mode === 'work' ? 'Specs (target rings)' : 'Controller and observer knobs', 'p. 224 · §13.2');
      poleKnobs(spec, ctx, { pI: true });
      obsKnobs(spec, ctx);
      slider(spec, { label: 'ẑ(0) − z<sub>e</sub>', unit: 'm', min: -0.2, max: 0.2, step: 0.005, sig: 3, hint: 'initial estimate error', get: () => ctx.st.zhat0, set: (v) => { ctx.st.zhat0 = v; ctx.update(); } });
      if (ctx.S.mode === 'explore') gainsReadout(spec, ctx, 'obs');
    },
    math(ctx) {
      const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'obs');
      const { A, C } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const O = L.obsv(A, C);
      return [
        ssCard(ctx),
        { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 224',
          theory: '\\dot{\\hat x} = A\\hat x + B(u - F_{ff}) + L(\\tilde y - C\\hat x),\\quad \\dot e = (A - LC)e' },
        { title: 'Observability', page: 'p. 221',
          theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix} C \\\\ CA \\\\ CA^2 \\\\ CA^3\\end{bmatrix}\\;(8\\times4),\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = 4',
          numbers: `\\operatorname{rank}\\mathcal{O} = ${L.rank(O)}`, spoiler: true },
        { title: 'Decoupled observer gain (two outputs)', page: 'p. 225 (B.13 uses place(Aᵀ, Cᵀ)ᵀ)',
          theory: '\\text{with two outputs } L \\text{ is } 4\\times2 \\text{ and not unique: any } L \\text{ with the desired eig}(A - LC) \\text{ works}',
          symbolic: 'L = \\begin{bmatrix}\\beta_{z1} & 0\\\\ 0 & \\beta_{\\theta1}\\\\ \\beta_{z0} & a_{32}\\\\ a_{41} & \\beta_{\\theta0}\\end{bmatrix} \\Rightarrow A - LC = \\text{blockdiag}\\left(\\begin{bmatrix}-\\beta_{z1} & 1\\\\ -\\beta_{z0} & 0\\end{bmatrix}, \\begin{bmatrix}-\\beta_{\\theta1} & 1\\\\ -\\beta_{\\theta0} & 0\\end{bmatrix}\\right)',
          numbers: `q = ${d.obsPoles.map((q) => texPole(q)).join(',\\;')},\\quad L = ${texMat(d.L)}`, spoiler: true,
          note: 'With two outputs L is not unique: python\'s place gives a different, dense L with the same eigenvalues. Any L with the right eig(A − LC) answers (c).' },
        { title: 'Separation principle', page: 'p. 222–223',
          theory: '\\text{eig} = \\text{eig}(A_1 - B_1K_1) \\cup \\text{eig}(A - LC)',
          note: 'Holds for the linear model only; saturation, mismatch and the nonlinear plant break it.' },
        polesCard(ctx, d, 'obs'),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch13;
      const jac = () => ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'b', title: '(b) Observability',
          inputs: { rank: 'rank 𝒪<sub>A,C</sub>' },
          check: (v) => PD().checkNumbers(v, { rank: L.rank(L.obsv(jac().A, jac().C)) }, {}),
          solution: () => [{ html: 'C picks z̃ and θ̃; CA adds z̃̇ and θ̃̇, so the first four rows of 𝒪 are already the 4×4 identity: rank 4. (z alone would also do: z, ż, z̈ = −gθ, z⃛ = −gθ̇.)' }],
        },
        {
          id: 'c', title: '(c) Your observer gain L (4×2)',
          html: 'Passes when eig(A − LC) are stable and the slowest observer pole is at least 2× faster (real part) than the slowest controller pole of the current gains. Uses the model selected in the controls.',
          inputs: Linputs(4),
          check: (v) => {
            const Lg = readL(v, 4);
            if (!Lg) return { ok: false, msg: 'Enter all eight entries.' };
            const qo = obsPolesOf(ctx, 'obs', Lg), qc = clPoles(ctx, 'obs');
            const so = Math.max(...qo.map((q) => q.re)), sc = Math.max(...qc.map((q) => q.re));
            const ok = so < 0 && so <= 2 * sc;
            return { ok, msg: `eig(A − LC) = ${E.poleText(qo)}; slowest controller pole Re = ${fmt(sc, 3)}.` };
          },
          actions: [{ label: 'Use my L', run: (v) => { const Lg = readL(v, 4); if (!Lg) return { ok: false, msg: 'Enter all eight entries.' }; ctx.app.setMode('work'); ctx.st.w.L = Lg; ctx.update(); return null; } }],
          solution: () => {
            const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'obs');
            return [
              { html: `One answer: decoupled L for observer pairs ${ctx.st.obsFactor}× faster than the controller pairs (ζ = ${ctx.st.zetaObs}), from the current knobs:` },
              { tex: `L = ${texMat(d.L)},\\quad \\text{eig}(A - LC) = ${d.obsPoles.map((q) => texPole(q)).join(',\\;')}` },
              { html: 'L<sub>32</sub> = a<sub>32</sub> = −g and L<sub>41</sub> = a<sub>41</sub> cancel the cross-couplings; the remaining entries are the coefficients of each block\'s desired polynomial.' },
            ];
          },
        },
        {
          id: 'e', title: '(e) Add a 0.5 N input disturbance',
          html: 'Set d = 0.5 N in the left panel. Reports the steady-state tracking error and the estimate bias at t<sub>end</sub>. Then try the "from z only" observer structure.',
          check: () => {
            const res = ctx.app.result(), n = E.beforeLastSwitch(ctx, res);  // a settled sample, not on a switch
            if (!(Math.abs(ctx.S.sim.dist) > 0)) return { ok: false, msg: 'Set d ≠ 0 first.' };
            const ez = res.rAll[0][n] - res.yAll[0][n];
            const bz = res.yAll[0][n] - res.extras.zhat[n], bth = (res.yAll[1][n] - res.extras.thhat[n]) / M.DEG;
            return { ok: true, msg: `At t = ${fmt(res.t[n], 3)} s (just before the last switch): z_r − z = ${fmt(1000 * ez, 3)} mm, z − ẑ = ${fmt(1000 * bz, 3)} mm, θ − θ̂ = ${fmt(bth, 3)}°.` };
          },
          solution: () => [{ html: 'The observer error obeys ė = (A − LC)e + Bd, so it settles at e<sub>ss</sub> = −(A − LC)⁻¹Bd ≠ 0. The integrator drives z<sub>r</sub> − ẑ to zero, so z ends up off by the ẑ bias, as the book says. With the <em>decoupled</em> L the z̃ block does not see d (d enters only θ̈), so ẑ is unbiased, z still tracks, and the bias shows up in θ̂ only. With the z-only L (or python\'s place) the bias reaches ẑ and z misses the reference.' }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 14 --
  CH.ch14 = base('dobs', 14, 'Disturbance observers', 'pp. 239–259', {
    defaults(sys) { return defaultsFor(sys, 'ch14', 'dobs'); },
    simDefaults(sys) { return { ...sys.problems.ch14.sim, mismatch: sys.problems.ch14.mismatch }; },
    outputSeries: estimateSeries,
    extraPlot(ctx, res) {
      const S = ctx.S;
      return {
        opts: { title: 'disturbance d and estimate d̂', yLabel: 'd [N]', unit: 'N' },
        data: { series: [
          { label: 'd̂ (estimate)', y: Array.from(res.extras.dhat || []), color: '--series-3', width: 2 },
          { label: 'true d', y: Array.from(res.t, (t) => (t >= S.sim.tDist ? S.sim.dist : 0)), color: '--series-1', dash: [6, 4], width: 1.5 },
        ] },
      };
    },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller (uses x̂, subtracts d̂)', 'p. 241');
      compControl(sec, ctx);
      segmented(sec, {
        label: 'Disturbance observer',
        options: [{ value: true, label: 'on' }, { value: false, label: 'off (E.14a)' }],
        get: () => ctx.st.dobs, set: (v) => { ctx.st.dobs = v; ctx.update(); },
      });
      if (ctx.S.mode === 'work') workGrid(sec, ctx, { ki: true, Lrows: 5 });
      const spec = section(parent, ctx.S.mode === 'work' ? 'Specs (target rings)' : 'Controller and observer knobs', 'p. 241');
      poleKnobs(spec, ctx, { pI: true });
      obsKnobs(spec, ctx, { pD: true });
      if (ctx.S.mode === 'explore') gainsReadout(spec, ctx, 'dobs');
    },
    math(ctx) {
      const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'dobs');
      const { A, B, C } = ctx.sys.linear(ctx.pModel, { comp: ctx.st.comp });
      const { A2, C2 } = E.augD(A, B, C);
      return [
        { title: 'Why the plain observer is biased', page: 'p. 240 · Eq. 14.2–14.3',
          theory: '\\dot x = Ax + B(u + d),\\quad \\dot e = (A - LC)e + Bd \\Rightarrow e_{ss} \\ne 0' },
        { title: 'Augmented model (ḋ = 0)', page: 'p. 240',
          theory: 'A_2 = \\begin{bmatrix}A & B\\\\ 0 & 0\\end{bmatrix},\\quad C_2 = \\begin{bmatrix}C & 0\\end{bmatrix}',
          numbers: `\\operatorname{rank}\\mathcal{O}_{A_2,C_2} = ${L.rank(L.obsv(A2, C2))}` },
        { title: 'Disturbance observer', page: 'p. 241',
          theory: '\\dot{\\hat x} = A\\hat x + B(u - F_{ff} + \\hat d) + L(\\tilde y - C\\hat x),\\quad \\dot{\\hat d} = L_d(\\tilde y - C\\hat x),\\quad \\tilde F = -K\\hat x - k_I\\textstyle\\int e - \\hat d' },
        { title: 'Decoupled gains: a z block and a θ–d block', page: 'p. 241',
          theory: '\\text{decoupled design: a } z \\text{ block and a } \\theta\\text{–}d \\text{ block}',
          symbolic: '\\theta\\text{–}d \\text{ block: } s^3 + \\beta_2 s^2 + \\beta_1 s + b_0 L_d = (s^2 + 2\\zeta\\omega s + \\omega^2)(s - p_d)',
          numbers: `L_2 = ${texMat(d.L)}`, spoiler: true },
        polesCard(ctx, d, 'dobs'),
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch14;
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'b1', title: '(b) Your disturbance-observer gain L<sub>2</sub> (5×2; last row is L<sub>d</sub>)',
          html: 'Passes when eig(A<sub>2</sub> − L<sub>2</sub>C<sub>2</sub>) are all stable.',
          inputs: Linputs(5),
          check: (v) => {
            const Lg = readL(v, 5);
            if (!Lg) return { ok: false, msg: 'Enter all ten entries.' };
            const q = obsPolesOf(ctx, 'dobs', Lg);
            return { ok: q.every((x) => x.re < 0), msg: `eig(A₂ − L₂C₂) = ${E.poleText(q)}.` };
          },
          actions: [{ label: 'Use my L', run: (v) => { const Lg = readL(v, 5); if (!Lg) return { ok: false, msg: 'Enter all ten entries.' }; ctx.app.setMode('work'); ctx.st.w.L = Lg; ctx.update(); return null; } }],
          solution: () => {
            const d = E.ssDesign(ctx.pModel, knobsOf(ctx.st), 'dobs');
            return [{ tex: `L_2 = ${texMat(d.L)},\\quad \\text{eig} = ${d.obsPoles.map((q) => texPole(q)).join(',\\;')}` }, { html: 'From the current knobs (decoupled structure). The θ–d block is third order: θ̃, θ̃̇ and d all observed through θ.' }];
          },
        },
        {
          id: 'b2', title: '(b) Estimator bias removed',
          html: 'Passes when, averaged over the last second, |z − ẑ| < 1 mm and |θ − θ̂| < 0.05°. Compare with the disturbance observer switched off.',
          check: () => {
            const res = ctx.app.result(), S = ctx.S, n = res.t.length;
            if (!ctx.st.dobs) return { ok: false, msg: 'Turn the disturbance observer on.' };
            const k0 = Math.max(0, n - Math.round(1 / S.sim.Ts));
            let bz = 0, bt = 0;
            for (let k = k0; k < n; k++) { bz += res.yAll[0][k] - res.extras.zhat[k]; bt += res.yAll[1][k] - res.extras.thhat[k]; }
            bz /= n - k0; bt /= (n - k0) * M.DEG;
            const ok = Math.abs(bz) < 0.001 && Math.abs(bt) < 0.05;
            return { ok, msg: `d̂ = ${fmt(res.extras.dhat[n - 1], 3)} N (d = ${fmt(S.sim.dist, 3)} N); mean z − ẑ = ${fmt(1000 * bz, 3)} mm, θ − θ̂ = ${fmt(bt, 3)}°.` };
          },
          solution: () => [{ html: 'd̂ estimates the input disturbance plus every force the model gets wrong (the default mismatch changes the force needed to hold the beam by about 0.9 N), so it settles near d + ΔF, not d. That is fine: what matters is that x̂ is unbiased and F̃ cancels the total.' }],
        },
      ]);
    },
  });

  WB.E.ss = { det, knobsOf };
})();
