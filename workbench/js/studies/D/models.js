// D.2-D.6: modeling the mass-spring-damper. Open-loop experiments: kinetic energy
// with prescribed motion (D.2), Euler-Lagrange equations with an energy-balance
// check (D.3), equilibria and linearization (D.4), the transfer function (D.5) and
// the state-space model (D.6). Derived results stay in `symbolic`/`numbers`
// (hidden in Work mode); `theory` lines show only the book's general formulas.
(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const { tex, texMat, texPole, fmt } = M;
  const lib = WB.studies.D.lib;
  const ans = WB.systems.D.answers;
  const CH = WB.studies.D.chapters;
  const inputForce = (st, t) => WB.models.inputTorque(st, t);  // same shapes as Study A

  // F = compensation + F_in(t). comp: 'none' | 'eq' (F_e = k z_e) | 'fl' (F = k z + F~)
  function openLoop(ctx, { linear = false } = {}) {
    const st = ctx.st, p = ctx.pModel;
    return {
      update(r, x, yMeas, t) {
        const fin = inputForce(st, t);
        if (linear || !st.comp || st.comp === 'none') return fin;
        if (st.comp === 'eq') return p.k * (st.zE || 0) + fin;
        return p.k * x[0] + fin;
      },
    };
  }

  function inputControls(parent, ctx, { comps, label = 'Input force F<sub>in</sub>(t)' } = {}) {
    const names = { zero: 'none', const: 'constant', pulse: 'pulse', square: 'square', sine: 'sine' };
    segmented(parent, {
      label, options: Object.keys(names).map((s) => ({ value: s, label: names[s] })),
      get: () => ctx.st.inp.shape, set: (v) => { ctx.st.inp.shape = v; ctx.update(); },
    });
    slider(parent, { label: 'amplitude', unit: 'N', min: -6, max: 6, step: 0.01, sig: 3, get: () => ctx.st.inp.amp, set: (v) => { ctx.st.inp.amp = v; ctx.update(); }, disabled: () => ctx.st.inp.shape === 'zero' });
    slider(parent, { label: 'frequency', unit: 'Hz', min: 0.005, max: 1, step: 0.005, sig: 3, get: () => ctx.st.inp.freq, set: (v) => { ctx.st.inp.freq = v; ctx.update(); }, disabled: () => !['square', 'sine'].includes(ctx.st.inp.shape) });
    slider(parent, { label: 'width', unit: 's', min: 0.1, max: 10, step: 0.1, sig: 3, get: () => ctx.st.inp.width, set: (v) => { ctx.st.inp.width = v; ctx.update(); }, disabled: () => ctx.st.inp.shape !== 'pulse' });
    if (comps) {
      segmented(parent, { label: 'Added to F<sub>in</sub>', options: comps, get: () => ctx.st.comp, set: (v) => { ctx.st.comp = v; ctx.update(); } });
    }
  }

  const common = {
    openLoop: true, metrics: false,
    controller(ctx, o) { return openLoop(ctx, o); },
  };
  // Eigenvalue markers; hidden in Work mode because they answer D.7(a).
  function eigMarkers(ctx, A, label) {
    if (ctx.S.mode === 'work' && !ctx.app.isRevealed('D:poles')) return null;
    return { markers: L.eig(A).map((p, i) => ({ ...p, kind: 'ol', label: `${label} ${i + 1}` })) };
  }
  function revealPolesButton(parent, ctx) {
    if (ctx.S.mode !== 'work') return;
    const b = el('button', { type: 'button', class: 'btn btn-quiet', text: 'Reveal the s-plane (eigenvalues)', onclick: () => { ctx.app.reveal('D:poles'); ctx.update(); } });
    WB.ui.addRefresher(() => { b.hidden = ctx.app.isRevealed('D:poles'); });
    parent.append(b);
  }
  const zSeries = (res) => res.x.map((x) => x[1]);

  // ------------------------------------------------------------------ D.2 --
  // Prescribed motion z(t) = A sin(2πft), as in hw02; nothing is simulated.
  CH.ch2 = Object.assign({}, common, {
    id: 'ch2', num: 2, tab: 'D.2', title: 'Kinetic energy', pages: 'pp. 19–40, p. 378',
    linear: false,
    defaults() { return { A: 0.5, f: 0.2, inp: { shape: 'zero', amp: 0, freq: 0.1, width: 1 } }; },
    simDefaults(sys) { return sys.problems.ch2.sim; },

    simulate(ctx) {
      const { pModel, st, S } = ctx;
      const Ts = S.sim.Ts, N = Math.round(S.sim.tEnd / Ts) + 1;
      const w = 2 * Math.PI * st.f;
      const res = { t: new Float64Array(N), r: new Float64Array(N), y: new Float64Array(N), uDemand: new Float64Array(N), u: new Float64Array(N), uApplied: new Float64Array(N), x: [], extras: { K: new Float64Array(N) } };
      for (let k = 0; k < N; k++) {
        const t = k * Ts, z = st.A * Math.sin(w * t), zd = st.A * w * Math.cos(w * t);
        res.t[k] = t; res.r[k] = z; res.y[k] = z;
        res.x.push([z, zd]);
        res.extras.K[k] = ctx.sys.kinetic([z, zd], pModel);
      }
      return res;
    },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Prescribed motion z(t) = A sin(2πft)', 'p. 378 · D.2(b)');
      slider(sec, { label: 'A', unit: 'm', min: 0, max: 2, step: 0.01, sig: 3, get: () => ctx.st.A, set: (v) => { ctx.st.A = v; ctx.update(); } });
      slider(sec, { label: 'f', unit: 'Hz', min: 0.02, max: 2, step: 0.01, sig: 3, get: () => ctx.st.f, set: (v) => { ctx.st.f = v; ctx.update(); } });
      lib.note(sec, 'No dynamics here: the motion is imposed, as in a hw02 animation script. The plot below shows the kinetic energy along that motion.');
    },

    extraPlot(ctx, res) {
      const show = ctx.S.mode === 'explore' || ctx.app.isRevealed('D:ch2:K');
      if (!show) return { opts: { title: 'ż(t)', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [{ label: 'ż', y: zSeries(res), color: '--series-1' }] } };
      return { opts: { title: 'kinetic energy K(t)', yLabel: 'K [J]', unit: 'J' }, data: { series: [{ label: 'K', y: Array.from(res.extras.K), color: '--series-1' }] } };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Kinetic energy of a rigid body', page: 'p. 25 · Eq. 2.3',
          theory: 'K = \\tfrac12 m\\,\\mathbf v_{cm}^\\top\\mathbf v_{cm} + \\tfrac12\\boldsymbol\\omega^\\top J_{cm}\\boldsymbol\\omega' },
        { title: 'Point masses on a line', page: 'p. 26 · §2.1.1',
          theory: 'K = \\tfrac12 m_1\\dot z_1^2 + \\tfrac12 m_2\\dot z_2^2 \\quad\\text{(the two-mass example, Fig. 2-6)}' },
        { title: 'Mass-spring-damper', page: 'p. 377 · Fig. 19-1',
          theory: '\\mathbf p = (z, 0, 0)^\\top,\\quad \\boldsymbol\\omega = 0',
          symbolic: 'K = \\tfrac12 m\\dot z^2',
          numbers: `K = ${tex(p.m / 2)}\\,\\dot z^2`, spoiler: true,
          note: 'The block slides without rotating, so only the translational term remains.' },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      lib.panel(parent, ctx, ctx.sys.problems.ch2, [
        {
          id: 'a', title: '(a) K = c ż²',
          inputs: { c: 'c [kg]' },
          check: (v) => lib.check(v, { c: p().m / 2 }, {}),
          actions: [{ label: 'Plot K(t)', run: () => { ctx.app.reveal('D:ch2:K'); ctx.update(); return null; } }],
          solution: () => [
            { tex: `\\mathbf v = (\\dot z, 0, 0)^\\top,\\;\\boldsymbol\\omega = 0 \\Rightarrow K = \\tfrac12 m\\dot z^2 = ${tex(p().m / 2)}\\,\\dot z^2` },
            { html: 'Same form as the mass-spring example on p. 26. The plot peaks at ½mA²(2πf)² when the mass passes z = 0.' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------------ D.3 --
  CH.ch3 = Object.assign({}, common, {
    id: 'ch3', num: 3, tab: 'D.3', title: 'Euler-Lagrange equations', pages: 'pp. 41–56, p. 378',
    linear: false,
    defaults() { return { comp: 'none', inp: { shape: 'square', amp: 1, freq: 0.05, width: 1 } }; },
    simDefaults(sys) { return sys.problems.ch3.sim; },

    buildControls(parent, ctx) {
      const sec = section(parent, 'Open-loop simulation', 'p. 378 · D.3(e)');
      inputControls(sec, ctx);
      lib.note(sec, 'The energy plot checks the equations of motion: the change in K + P must equal the work done by F minus what the damper dissipates.');
    },

    extraPlot(ctx, res) {
      const { sys, pModel } = ctx;
      const n = res.t.length, Ts = ctx.S.sim.Ts;
      const E = new Float64Array(n), Wk = new Float64Array(n);
      const E0 = sys.kinetic(res.x[0], pModel) + sys.potential(res.x[0], pModel);
      let w = 0;
      // power into the mass from the force the model assumes (nominal b), trapezoid rule
      const pw = (xx, u) => (u - pModel.b * xx[1]) * xx[1];
      for (let k = 0; k < n; k++) {
        E[k] = sys.kinetic(res.x[k], pModel) + sys.potential(res.x[k], pModel) - E0;
        if (k > 0) w += 0.5 * Ts * (pw(res.x[k - 1], res.uApplied[k - 1]) + pw(res.x[k], res.uApplied[k - 1]));
        Wk[k] = w;
      }
      return {
        opts: { title: 'energy balance', yLabel: 'energy [J]', unit: 'J' },
        data: { series: [
          { label: '∫(F − bż)ż dt', y: Wk, color: '--series-2', dash: [5, 4], width: 2 },
          { label: 'E(t) − E(0) = ΔK + ΔP', y: E, color: '--series-1' },
        ] },
      };
    },

    math(ctx) {
      const p = ctx.pModel;
      return [
        { title: 'Euler-Lagrange equations', page: 'p. 18 · Eq. 1.8, p. 43 · §3.1.4',
          theory: 'L = K - P,\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot q} - \\frac{\\partial L}{\\partial q} = \\tau - B\\dot q' },
        { title: 'Spring potential energy', page: 'p. 42 · Fig. 3-2',
          theory: 'P = \\tfrac12 k z^2 \\;\\text{for a spring stretched by } z',
          numbers: `P = ${tex(p.k / 2)}\\,z^2`, spoiler: true },
        { title: 'Generalized coordinates, forces, damping', page: 'p. 42 · §3.1.2, p. 43 · §3.1.3',
          theory: 'q = \\text{minimum set of configuration variables},\\quad \\tau = \\text{applied nonconservative forces},\\quad -B\\dot q = \\text{damping forces}',
          symbolic: 'q = z,\\quad \\tau = F,\\quad -B\\dot q = -b\\dot z', spoiler: true },
        { title: 'Equation of motion', page: 'p. 43',
          theory: '\\frac{d}{dt}\\frac{\\partial L}{\\partial \\dot z} - \\frac{\\partial L}{\\partial z} = F - b\\dot z',
          symbolic: 'L = \\tfrac12 m\\dot z^2 - \\tfrac12 k z^2 \\;\\Rightarrow\\; m\\ddot z + k z = F - b\\dot z',
          numbers: `\\ddot z = ${tex(1 / p.m)}\\,F - ${tex(p.b / p.m)}\\,\\dot z - ${tex(p.k / p.m)}\\,z`, spoiler: true },
        { title: 'Energy balance (a check on the EOM)', page: 'follows from the EOM',
          theory: '\\frac{d}{dt}(K + P) = (F - b\\dot z)\\,\\dot z' },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel;
      lib.panel(parent, ctx, ctx.sys.problems.ch3, [
        {
          id: 'a', title: '(a) P = c z² (P = 0 at the unstretched length)',
          inputs: { c: 'c [N/m]' },
          check: (v) => lib.check(v, { c: p().k / 2 }, {}),
          solution: () => [{ tex: `P = \\tfrac12 k z^2 = ${tex(p().k / 2)}\\,z^2` }, { html: 'Fig. 3-2 on p. 42 is exactly this spring.' }],
        },
        {
          id: 'bc', title: '(b, c) Generalized coordinate, force and damping force',
          html: 'Write them down, then compare.',
          solution: () => [
            { tex: 'q = z,\\qquad \\tau = F,\\qquad -B\\dot q = -b\\dot z' },
            { html: 'One configuration variable is enough to place the mass (p. 42). F is the only applied force; the damper is the only damping force (p. 43).' },
          ],
        },
        {
          id: 'd', title: '(d) z̈ = c<sub>1</sub>F + c<sub>2</sub>ż + c<sub>3</sub>z',
          inputs: { c1: 'c<sub>1</sub>', c2: 'c<sub>2</sub>', c3: 'c<sub>3</sub>' },
          check: (v) => lib.check(v, { c1: 1 / p().m, c2: -p().b / p().m, c3: -p().k / p().m }, {}),
          solution: () => [
            { tex: 'L = \\tfrac12 m\\dot z^2 - \\tfrac12 kz^2,\\quad \\frac{d}{dt}\\frac{\\partial L}{\\partial\\dot z} = m\\ddot z,\\quad \\frac{\\partial L}{\\partial z} = -kz' },
            { tex: `m\\ddot z + kz = F - b\\dot z \\;\\Rightarrow\\; c_1 = \\tfrac1m = ${tex(1 / p().m)},\\; c_2 = -\\tfrac bm = ${tex(-p().b / p().m)},\\; c_3 = -\\tfrac km = ${tex(-p().k / p().m)}` },
            { html: 'This is the f(x, u) that <code>_D_mass/python/testDynamics.py</code> checks (tools/regress_D.py runs its ten vectors through this page\'s model).' },
          ],
        },
      ]);
    },
  });

  // ------------------------------------------------------------------ D.4 --
  CH.ch4 = Object.assign({}, common, {
    id: 'ch4', num: 4, tab: 'D.4', title: 'Equilibria & linearization', pages: 'pp. 59–68, p. 378',
    defaults() { return { zE: 0.5, dz0: 0.3, method: 'jacobian', comp: 'eq', inp: { shape: 'zero', amp: 0.5, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch4.sim; },
    controller(ctx, o) { ctx.st.comp = ctx.st.method === 'fl' ? 'fl' : 'eq'; return openLoop(ctx, o); },

    // Start at z_e + δz(0) regardless of the left panel, and hold z_r = z_e for the plot.
    simulate(ctx, c, plant) {
      return WB.sim.simulate({ ...c, x0: [ctx.st.zE + ctx.st.dz0, 0], plant, controller: this.controller(ctx), reference: () => ctx.st.zE });
    },
    // Linearized model in deviation coordinates, shifted back to absolute z for the plot.
    linearSim(ctx, c) {
      const p = ctx.pModel, t = ans.tf(p);
      const fl = ctx.st.method === 'fl';
      const A = fl ? [[0, 1], [0, -t.a1]] : ans.ss(p).A, B = [[0], [t.b0]];
      const off = fl ? 0 : ctx.st.zE;  // the FL model is in absolute z
      return WB.sim.simulate({
        ...c, disturbance: null, noise: null, reference: () => ctx.st.zE,
        x0: [ctx.st.zE + ctx.st.dz0 - off, 0],
        plant: { f: (x, u) => A.map((row, i) => row[0] * x[0] + row[1] * x[1] + B[i][0] * u), h: (x) => x[0] + off, uLimit: Infinity },
        controller: { update: (r, x, y, tt) => inputForce(ctx.st, tt) },
      });
    },
    linearLabel: 'linearized model',

    buildControls(parent, ctx) {
      const sec = section(parent, 'Operating point', 'p. 59–60');
      slider(sec, { label: 'z<sub>e</sub>', unit: 'm', min: -1, max: 2, step: 0.01, sig: 3, get: () => ctx.st.zE, set: (v) => { ctx.st.zE = v; ctx.update(); } });
      slider(sec, { label: 'δz(0)', unit: 'm', min: -1, max: 1, step: 0.01, sig: 3, hint: 'initial offset from zₑ', get: () => ctx.st.dz0, set: (v) => { ctx.st.dz0 = v; ctx.update(); } });
      segmented(sec, {
        label: 'Linearization',
        options: [{ value: 'jacobian', label: 'Jacobian: F = F<sub>e</sub> + F̃' }, { value: 'fl', label: 'feedback: F = F<sub>fl</sub>(z) + F̃' }],
        get: () => ctx.st.method, set: (v) => { ctx.st.method = v; ctx.update(); },
      });
      lib.note(sec, 'In Work mode the simulation still applies the right F_e or F_fl, so you can test an answer by watching whether the mass stays put.');
      const inp = section(parent, 'Input F̃(t)', 'p. 60');
      inputControls(inp, ctx, { label: 'Input F̃(t)' });
      revealPolesButton(sec, ctx);
    },

    outputSeries(ctx, res) {
      return [{ label: 'zₑ', y: Array.from(res.t, () => ctx.st.zE), color: '--ref', dash: [6, 4], width: 1.5 }];
    },

    splane(ctx) {
      const t = ans.tf(ctx.pModel);
      const A = ctx.st.method === 'fl' ? [[0, 1], [0, -t.a1]] : ans.ss(ctx.pModel).A;
      return eigMarkers(ctx, A, `eigenvalue of A (${ctx.st.method})`);
    },

    math(ctx) {
      const p = ctx.pModel, t = ans.tf(p);
      const Afl = [[0, 1], [0, -t.a1]];
      return [
        { title: 'Nonlinear model', page: 'p. 59',
          theory: '\\dot x = f(x, u),\\quad x = (z, \\dot z)^\\top,\\quad u = F' },
        { title: 'Equilibria', page: 'p. 59–60',
          theory: 'f(x_e, u_e) = 0',
          symbolic: '\\dot z_e = 0,\\quad z_e \\text{ arbitrary},\\quad F_e = k z_e',
          numbers: `F_e(${fmt(ctx.st.zE, 3)}\\,\\text{m}) = ${tex(p.k * ctx.st.zE)}\\;\\text{N}`, spoiler: true },
        { title: 'Jacobian linearization', page: 'p. 60 · Eq. 4.1',
          theory: '\\dot{\\tilde x} = \\frac{\\partial f}{\\partial x}\\Big|_e\\tilde x + \\frac{\\partial f}{\\partial u}\\Big|_e\\tilde u,\\quad \\tilde x = x - x_e,\\; \\tilde u = u - u_e',
          symbolic: 'm\\ddot{\\tilde z} + b\\dot{\\tilde z} + k\\tilde z = \\tilde F \\quad(\\text{identical for every } z_e)',
          numbers: `A = ${texMat(ans.ss(p).A)},\\quad B = ${texMat(ans.ss(p).B)},\\quad \\text{eig}(A) = ${L.eig(ans.ss(p).A).map((q) => texPole(q)).join(',\\;')}`, spoiler: true,
          note: 'The plant is already linear, so the linearization is exact: the dashed trace lies on the simulated one.' },
        { title: 'Feedback linearization', page: 'p. 62 · §4.1.2',
          theory: 'u = u_{fl}(x) + \\tilde u',
          symbolic: 'F = kz + \\tilde F \\;\\Rightarrow\\; m\\ddot z + b\\dot z = \\tilde F',
          numbers: `A_{fl} = ${texMat(Afl)},\\quad \\text{eig} = 0,\\; ${tex(-t.a1)}`, spoiler: true,
          note: 'u_fl cancels the unwanted terms exactly. Cancelling the spring leaves a free integrator: the mass then drifts after any force pulse.' },
      ];
    },

    buildProblem(parent, ctx) {
      const p = () => ctx.pModel, zE = () => ctx.st.zE;
      lib.panel(parent, ctx, ctx.sys.problems.ch4, [
        {
          id: 'a', title: '(a) Equilibrium force at the current z<sub>e</sub>',
          inputs: { Fe: 'F<sub>e</sub> [N]' },
          check: (v) => lib.check(v, { Fe: p().k * zE() }, {}),
          solution: () => [{ tex: `\\dot z = 0,\\; \\ddot z = 0 \\Rightarrow F_e = kz_e = ${tex(p().k * zE())}\\,\\text{N}` }, { html: 'Every z<sub>e</sub> is an equilibrium, with ż<sub>e</sub> = 0.' }],
        },
        {
          id: 'b', title: '(b) Jacobian: z̃̈ = a<sub>21</sub>z̃ + a<sub>22</sub>z̃̇ + b<sub>2</sub>F̃',
          inputs: { a21: 'a<sub>21</sub>', a22: 'a<sub>22</sub>', b2: 'b<sub>2</sub>' },
          check: (v) => lib.check(v, { a21: -p().k / p().m, a22: -p().b / p().m, b2: 1 / p().m }, {}),
          solution: () => [{ tex: `\\frac{\\partial \\ddot z}{\\partial z} = -\\tfrac km = ${tex(-p().k / p().m)},\\; \\frac{\\partial \\ddot z}{\\partial \\dot z} = -\\tfrac bm = ${tex(-p().b / p().m)},\\; \\frac{\\partial \\ddot z}{\\partial F} = \\tfrac1m = ${tex(1 / p().m)}` }, { html: 'None of these depend on z<sub>e</sub>: the system is linear.' }],
        },
        {
          id: 'c', title: '(c) Feedback linearization F = c·z + F̃',
          inputs: { c: 'c [N/m]' },
          check: (v) => lib.check(v, { c: p().k }, {}),
          solution: () => [{ tex: `F = kz + \\tilde F \\Rightarrow m\\ddot z + b\\dot z = \\tilde F,\\quad c = k = ${tex(p().k)}` }, { html: 'Also valid: F = kz + bż + m·v gives z̈ = v (a double integrator). The book\'s convention (Study A, p. 64) cancels only the static term.' }],
        },
      ]);
    },
  });

  // ------------------------------------------------------- D.5 and D.6 --
  const flInput = (parent, ctx, title, page) => {
    const inp = section(parent, title, page);
    inputControls(inp, ctx);
    lib.note(inp, 'The dashed trace is the linear model driven by the same force. The plant is linear, so the two coincide unless you add mismatch, saturation or a disturbance.');
    revealPolesButton(inp, ctx);
  };
  const olLinear = (ctx, c) => lib.linearSim(ctx, c, openLoop);

  CH.ch5 = Object.assign({}, common, {
    id: 'ch5', num: 5, tab: 'D.5', title: 'Transfer function', pages: 'pp. 69–80, p. 378',
    defaults() { return { comp: 'none', inp: { shape: 'pulse', amp: 1, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch5.sim; },
    linearSim: olLinear,
    linearLabel: 'P(s) response',
    buildControls(parent, ctx) { flInput(parent, ctx, 'Input F(t)', 'p. 378'); },
    splane(ctx) { return eigMarkers(ctx, ans.ss(ctx.pModel).A, 'pole of P(s)'); },
    math(ctx) {
      const t = ans.tf(ctx.pModel);
      const ol = ans.olPoles(ctx.pModel);
      return [
        { title: 'Laplace transform', page: 'p. 69–70',
          theory: '\\mathcal L\\{\\dot y\\} = sY(s) - y(0),\\quad \\mathcal L\\{\\ddot y\\} = s^2Y(s) - sy(0) - \\dot y(0)' },
        { title: 'Transfer function', page: 'p. 70',
          theory: 'P(s) = \\frac{Y(s)}{U(s)}\\Big|_{\\text{zero initial conditions}}',
          symbolic: '(ms^2 + bs + k)Z(s) = F(s) \\;\\Rightarrow\\; P(s) = \\frac{Z(s)}{F(s)} = \\frac{1/m}{s^2 + \\frac bm s + \\frac km}',
          numbers: `P(s) = \\frac{${tex(t.b0)}}{s^2 + ${tex(t.a1)}\\,s + ${tex(t.a0)}},\\quad \\text{poles } ${texPole(ol[0])},\\; ${texPole(ol[1])}`, spoiler: true },
        { title: 'Block diagram', page: 'p. 70',
          theory: 'F(s) \\longrightarrow \\boxed{P(s)} \\longrightarrow Z(s)',
          symbolic: 'F \\to \\boxed{\\tfrac1m} \\to \\ddot z \\to \\boxed{\\tfrac1s} \\to \\dot z \\to \\boxed{\\tfrac1s} \\to z,\\quad \\text{with } -b\\dot z - kz \\text{ fed back into the sum before } \\tfrac1m', spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const t = () => ans.tf(ctx.pModel);
      lib.panel(parent, ctx, ctx.sys.problems.ch5, [
        {
          id: 'a', title: '(a) Laplace transform of the EOM (zero initial conditions)',
          html: 'Write (c<sub>2</sub>s² + c<sub>1</sub>s + c<sub>0</sub>) Z(s) = F(s).',
          inputs: { c2: 'c<sub>2</sub>', c1: 'c<sub>1</sub>', c0: 'c<sub>0</sub>' },
          check: (v) => lib.check(v, { c2: ctx.pModel.m, c1: ctx.pModel.b, c0: ctx.pModel.k }, {}),
          solution: () => [{ tex: `m\\ddot z + b\\dot z + kz = F \\xrightarrow{\\mathcal L} (${tex(ctx.pModel.m)}s^2 + ${tex(ctx.pModel.b)}s + ${tex(ctx.pModel.k)})Z(s) = F(s)` }],
        },
        {
          id: 'b', title: '(b) P(s) = b<sub>0</sub> / (s² + a<sub>1</sub>s + a<sub>0</sub>)',
          inputs: { b0: 'b<sub>0</sub>', a1: 'a<sub>1</sub>', a0: 'a<sub>0</sub>' },
          check: (v) => lib.check(v, { b0: t().b0, a1: t().a1, a0: t().a0 }, {}),
          solution: () => [{ tex: `P(s) = \\frac{1/m}{s^2 + \\frac bm s + \\frac km} = \\frac{${tex(t().b0)}}{s^2 + ${tex(t().a1)}s + ${tex(t().a0)}}` }, { html: 'Cross-checked in tools/regress_D.py against C(sI − A)<sup>−1</sup>B from D.6 (python-control ss2tf).' }],
        },
        {
          id: 'c', title: '(c) Block diagram',
          solution: () => [{ html: 'F(s) → [1/(ms² + bs + k)] → Z(s). Expanded: a summing junction forms F − bż − kz, then 1/m gives z̈, and two integrators 1/s give ż and z, with gains b and k feeding ż and z back to the junction.' }],
        },
      ]);
    },
  });

  CH.ch6 = Object.assign({}, common, {
    id: 'ch6', num: 6, tab: 'D.6', title: 'State-space model', pages: 'pp. 81–93, p. 379',
    defaults() { return { comp: 'none', inp: { shape: 'pulse', amp: 1, freq: 0.05, width: 2 } }; },
    simDefaults(sys) { return sys.problems.ch6.sim; },
    linearSim: olLinear,
    linearLabel: 'ẋ = Ax + Bu response',
    buildControls(parent, ctx) { flInput(parent, ctx, 'Input u = F(t)', 'p. 379'); },
    splane(ctx) { return eigMarkers(ctx, ans.ss(ctx.pModel).A, 'eigenvalue of A'); },
    extraPlot(ctx, res) {
      return { opts: { title: 'x₂ = ż(t)', yLabel: 'ż [m/s]', unit: 'm/s' }, data: { series: [{ label: 'ż', y: zSeries(res), color: '--series-1' }] } };
    },
    math(ctx) {
      const { A, B, C } = ans.ss(ctx.pModel);
      return [
        { title: 'Jacobian linearization revisited', page: 'p. 83–84',
          theory: 'A = \\frac{\\partial f}{\\partial x}\\Big|_e,\\quad B = \\frac{\\partial f}{\\partial u}\\Big|_e,\\quad C = \\frac{\\partial h}{\\partial x}\\Big|_e,\\quad D = \\frac{\\partial h}{\\partial u}\\Big|_e' },
        { title: 'State-space model', page: 'p. 379 · D.6',
          theory: 'x = \\begin{bmatrix} z \\\\ \\dot z\\end{bmatrix},\\quad u = F,\\quad y = z',
          symbolic: 'A = \\begin{bmatrix}0 & 1\\\\ -\\frac km & -\\frac bm\\end{bmatrix},\\quad B = \\begin{bmatrix}0\\\\ \\frac1m\\end{bmatrix},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\quad D = 0',
          numbers: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = ${texMat(C)}`, spoiler: true },
        { title: 'Back to the transfer function', page: 'p. 85 · Eq. 6.14–6.15',
          theory: 'P(s) = C(sI - A)^{-1}B + D,\\quad \\text{poles: } \\det(sI - A) = 0',
          numbers: `\\det(sI - A) = ${WB.tf.polyTex(L.charPoly(A))}`, spoiler: true },
      ];
    },
    buildProblem(parent, ctx) {
      const ss = () => ans.ss(ctx.pModel);
      lib.panel(parent, ctx, ctx.sys.problems.ch6, [{
        id: 'a', title: 'A, B, C, D',
        inputs: { a11: 'A<sub>11</sub>', a12: 'A<sub>12</sub>', a21: 'A<sub>21</sub>', a22: 'A<sub>22</sub>', b1: 'B<sub>1</sub>', b2: 'B<sub>2</sub>', c1: 'C<sub>1</sub>', c2: 'C<sub>2</sub>', d: 'D' },
        check: (v) => { const { A, B } = ss(); return lib.check(v, { a11: A[0][0], a12: A[0][1], a21: A[1][0], a22: A[1][1], b1: B[0][0], b2: B[1][0], c1: 1, c2: 0, d: 0 }, {}); },
        solution: () => { const { A, B } = ss(); return [{ tex: `A = ${texMat(A)},\\quad B = ${texMat(B)},\\quad C = \\begin{bmatrix}1 & 0\\end{bmatrix},\\quad D = 0` }, { html: 'Check: C(sI − A)<sup>−1</sup>B reproduces the D.5 transfer function (tools/regress_D.py).' }]; },
      }]);
    },
  });

  WB.studies.D.models = { openLoop, inputControls };
})();
