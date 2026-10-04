// Chapters 7 and 8: PD control of a second-order plant P(s) = b0 / (s^2 + a1 s + a0).
// Both chapters share the controller and the closed-loop analysis; they differ in
// how the gains are designed (pole locations vs. rise time / damping ratio) and in
// the problem statement.
//
// Modes:
//   work    - you set kP, kD yourself; derivations that answer the problem stay
//             hidden until you reveal them; the problem panel checks your answers.
//   explore - gains are designed from the chapter's tuning knobs (drag the poles).
window.WB = window.WB || {};
WB.chapters = WB.chapters || {};

(function () {
  const { el, slider, segmented, bind } = WB.ui;
  const M = WB.math;
  const { tex, texPole, fmt, fmtPole } = M;

  // ---------------------------------------------------------------- shared --
  function clPoles(model, kP, kD) {
    return M.roots2(model.a1 + model.b0 * kD, model.a0 + model.b0 * kP);
  }

  // PD law from Listing 7.1 / 8.1. arch 'output' differentiates y only (Fig. 7-2);
  // 'error' differentiates e = r - y (Fig. 7-1) using a backward difference on r.
  // comp: 'fl' feedback linearization, 'eq' equilibrium torque at y_e, 'none'.
  function makeController(ctx, { linear = false } = {}) {
    const { sys, pModel, S } = ctx;
    const { kP, kD } = ctx.gains;
    const st = ctx.st;
    const Ts = S.sim.Ts;
    let rPrev = null;
    return {
      update(r, x) {
        const y = x[0], ydot = x[1];
        if (rPrev === null) rPrev = r;
        const rdot = (r - rPrev) / Ts;
        rPrev = r;
        const deriv = st.arch === 'error' ? kD * (rdot - ydot) : -kD * ydot;
        const uTilde = kP * (r - y) + deriv;
        if (linear) return uTilde;
        if (st.comp === 'fl') return uTilde + sys.feedbackLinearization(x, pModel);
        if (st.comp === 'eq') return uTilde + sys.equilibriumInput(0, pModel);
        return uTilde;
      },
    };
  }

  function sharedControls(parent, ctx) {
    const { sys } = ctx;
    segmented(parent, {
      label: 'Derivative acts on',
      options: [
        { value: 'output', label: `output ${sys.sym.yText} (Fig. 7-2)` },
        { value: 'error', label: 'error e (Fig. 7-1)' },
      ],
      ...bind(ctx, 'arch'),
    });
    segmented(parent, {
      label: 'Gravity compensation',
      options: [
        { value: 'fl', label: `feedback lin. ${sys.sym.ffText}(${sys.sym.yText})`, title: 'τ = τ_fl(θ) + τ̃ (Listing 7.1)' },
        { value: 'eq', label: `${sys.sym.uText}<sub>e</sub> at ${sys.sym.yText}<sub>e</sub>=0`, title: 'τ = τ_e + τ̃' },
        { value: 'none', label: 'none' },
      ],
      ...bind(ctx, 'comp'),
    });
  }

  function workGainSliders(parent, ctx) {
    slider(parent, {
      label: 'k<sub>P</sub>', min: 0, max: 2, step: 0.001, sig: 4,
      ...bind(ctx, 'kP'),
    });
    slider(parent, {
      label: 'k<sub>D</sub>', min: 0, max: 0.5, step: 0.0005, sig: 4,
      ...bind(ctx, 'kD'),
    });
  }

  const gainReadout = (parent, ctx) => WB.ui.readout(parent, () => [['kP', ctx.gains.kP], ['kD', ctx.gains.kD]], { wrap: false });

  function baseMarkers(ctx, draggable) {
    const { model } = ctx;
    const { kP, kD } = ctx.gains;
    const ol = M.roots2(model.a1, model.a0);
    const cl = clPoles(model, kP, kD);
    // In Work mode the open-loop poles answer A.7(a) until it is solved.
    const showOl = ctx.S.mode === 'explore' || ctx.app.isSolved(`${ctx.sys.problems.ch7.id}/a`);
    const markers = showOl ? ol.map((p, i) => ({ ...p, kind: 'ol', label: `open-loop pole p${i + 1}` })) : [];
    cl.forEach((p, i) => markers.push({ ...p, kind: 'cl', label: `closed-loop pole p${i + 1}`, dragId: draggable ? i : undefined }));
    if (ctx.st.arch === 'error' && kD > 1e-9) markers.push({ re: -kP / kD, im: 0, kind: 'zero', label: 'zero z = −kP/kD' });
    return markers;
  }

  function plantCard(ctx) {
    const { model, sys } = ctx;
    return {
      title: 'Plant (design model)', page: 'p. 99 · Eq. 7.1 · A.7 p. 102', answers: `${sys.problems.ch5.id}/a`,
      theory: `P(s) = \\frac{b_0}{s^2 + a_1 s + a_0},\\quad b_0 = ${model.tex.b0},\\; a_1 = ${model.tex.a1},\\; a_0 = ${model.tex.a0}`,
      numbers: `P(s) = \\frac{${tex(model.b0)}}{s^2 + ${tex(model.a1)}\\,s ${model.a0 ? '+ ' + tex(model.a0) : ''}}`,
      spoiler: true,
      note: `linearized about ${sys.sym.yText}ₑ = 0 with the gravity torque cancelled`,
    };
  }

  function olPolesCard(ctx) {
    const ol = M.roots2(ctx.model.a1, ctx.model.a0);
    return {
      title: 'Open-loop poles', page: 'p. 99',
      theory: 'p_{ol} = -\\frac{a_1}{2} \\pm \\sqrt{\\left(\\frac{a_1}{2}\\right)^2 - a_0}',
      numbers: `p_{ol} = ${texPole(ol[0])},\\; ${texPole(ol[1])}`,
      spoiler: true,
    };
  }

  function closedLoopCard(ctx) {
    const { model, sym } = { model: ctx.model, sym: ctx.sys.sym };
    const { kP, kD } = ctx.gains;
    const err = ctx.st.arch === 'error';
    const c1 = model.a1 + model.b0 * kD, c0 = model.a0 + model.b0 * kP;
    const num = err ? `${tex(model.b0 * kD)}\\,s + ${tex(model.b0 * kP)}` : tex(model.b0 * kP);
    const cl = clPoles(model, kP, kD);
    return {
      title: `Closed loop, derivative on ${err ? 'error (Fig. 7-1)' : 'output (Fig. 7-2)'}`,
      page: err ? 'p. 100 · Eq. 7.4' : 'p. 101 · Eq. 7.5',
      theory: err
        ? `\\frac{Y(s)}{R(s)} = \\frac{b_0 k_D s + b_0 k_P}{s^2 + (a_1 + b_0 k_D)s + (a_0 + b_0 k_P)}`
        : `\\frac{Y(s)}{R(s)} = \\frac{b_0 k_P}{s^2 + (a_1 + b_0 k_D)s + (a_0 + b_0 k_P)}`,
      symbolic: `\\Delta_{cl}(s) = s^2 + (${tex(model.a1)} + ${tex(model.b0)}\\,k_D)\\,s + (${model.a0 ? tex(model.a0) + ' + ' : ''}${tex(model.b0)}\\,k_P)`,
      numbers: `\\frac{${sym.y}(s)}{${sym.y}_r(s)} = \\frac{${num}}{s^2 + ${tex(c1)}\\,s + ${tex(c0)}},\\quad p_{cl} = ${texPole(cl[0])},\\; ${texPole(cl[1])}`
        + (err && kD > 0 ? `,\\quad z_{cl} = -\\tfrac{k_P}{k_D} = ${tex(-kP / kD)}` : ''),
      spoiler: true,
      answers: `${ctx.sys.problems.ch7.id}/b`,
    };
  }

  function compensationCard(ctx) {
    const { sys, pModel } = ctx;
    const comp = ctx.st.comp;
    const tauE = sys.equilibriumInput(0, pModel);
    return {
      title: 'Gravity compensation', page: 'p. 104 · Listing 7.1',
      answers: comp === 'none' ? undefined : `${sys.problems.ch4.id}/c`,
      theory: comp === 'fl'
        ? `${sys.sym.u} = ${sys.sym.ff} + \\tilde{${sys.sym.u}},\\quad ${sys.sym.ff} = ${sys.ffTex}`
        : comp === 'eq'
          ? `${sys.sym.u} = ${sys.sym.u}_e + \\tilde{${sys.sym.u}},\\quad ${sys.sym.u}_e = ${sys.ffTex}\\big|_{${sys.sym.y}_e=0} = ${tex(tauE)}`
          : `${sys.sym.u} = \\tilde{${sys.sym.u}}\\quad\\text{(gravity acts as an unmodeled disturbance)}`,
      numbers: null,
    };
  }

  function placementCard(ctx, desired, title, page) {
    const { model } = ctx;
    const { alpha1, alpha0 } = M.polyFromPoles(desired[0], desired[1]);
    return {
      title, page,
      theory: '\\Delta^d_{cl}(s) = (s - p_1)(s - p_2) = s^2 + \\alpha_1 s + \\alpha_0,\\quad \\text{set } \\Delta_{cl}(s) = \\Delta^d_{cl}(s) \\text{ and match coefficients}',
      symbolic: 'k_P = \\frac{\\alpha_0 - a_0}{b_0},\\quad k_D = \\frac{\\alpha_1 - a_1}{b_0}',
      numbers: `\\Delta_{cl}^d = s^2 + ${tex(alpha1)}\\,s + ${tex(alpha0)} \\quad \\Rightarrow k_P = ${tex((alpha0 - model.a0) / model.b0)},\\; k_D = ${tex((alpha1 - model.a1) / model.b0)}`,
      spoiler: true,
    };
  }

  const gainsFromPoles = WB.design.pdGains;

  function wnFromTr(tr, zeta, rule) {
    if (rule === 'tp') return Math.PI / (2 * tr * Math.sqrt(Math.max(1e-6, 1 - zeta * zeta)));
    return 2.2 / tr;
  }

  function polesFromWnZeta(wn, zeta) {
    if (zeta < 1) {
      const wd = wn * Math.sqrt(1 - zeta * zeta);
      return [{ re: -zeta * wn, im: wd }, { re: -zeta * wn, im: -wd }];
    }
    const r = wn * Math.sqrt(zeta * zeta - 1);
    return [{ re: -zeta * wn + r, im: 0 }, { re: -zeta * wn - r, im: 0 }];
  }

  // ----------------------------------------------------- problem-panel kit --
  // Each part: inputs (name -> label), check(values) -> {ok, msg}, solution() -> text.
  // A part with `code: {template, check(code) -> Promise<{ok, msg, detail?}>, actions?}`
  // gets a Python editor instead (WB.py). A part with neither inputs, code nor
  // check is a step that is done outside the workbench; it shows its html only,
  // plus a button that marks it done when it has `done: 'button label'` (e.g. a
  // plot drawn by hand). `after: 'a'` keeps a part hidden in Work mode until part
  // (a) is solved or done.
  // Passing a check marks the part solved, which unlocks live-math cards that
  // answer it (card.answers, see app.js).
  function problemPanel(parent, ctx, prob, parts) {
    const key = `wb.${ctx.sys.id}.${prob.id}.answers`;
    const saved = WB.ui.store.get(key, {});
    const title = el('strong');
    const head = el('div', { class: 'problem-head' }, title, WB.ui.pageChip(`p. ${prob.page}`));
    const stmt = el('div', { class: 'problem-stmt' });
    stmt.innerHTML = prob.statement.map((s) => `<p>${s}</p>`).join(''); // authored text
    WB.ui.linkifyNode(stmt);
    // Everything below the heading collapses with the problem; each part also collapses on its own.
    const outer = parent;
    parent = el('div', { class: 'problem-body' });
    const tools = el('div', { class: 'part-tools' });
    outer.append(head, parent);
    WB.ui.collapsible(title, `Problem ${prob.id}`, outer, parent, `wb.collapsed.problem.${ctx.sys.id}.${prob.id}`);
    parent.append(stmt);
    const note = el('p', { class: 'muted small', text: 'Answers are checked against the current nominal parameters (left panel).' });
    parent.append(note);
    const partToggles = [];
    if (parts.some((p) => p.code)) {
      const pn = el('p', { class: 'muted small' });
      pn.innerHTML = 'Python answers run in your browser (Python 3.14 + numpy, loaded on the first Check). <code>np</code> is imported, and <code>P</code> holds the parameters like <code>armParam.py</code>: '
        + Object.keys(ctx.pModel).map((k) => `<code>P.${k}</code>`).join(', ') + '. Write answers with these, not numbers: they are checked with other parameter values too.';
      parent.append(pn);
    }

    if (parts.length > 1) parent.append(tools);
    for (const part of parts) {
      const box = el('div', { class: 'part' });
      const title = el('div', { class: 'part-title' });
      const partBody = el('div', { class: 'part-body' });
      partToggles.push(WB.ui.collapsible(title, part.title, box, partBody, `wb.collapsed.part.${ctx.sys.id}.${prob.id}.${part.id}`));
      box.append(title);
      const inputs = {};
      const grid = el('div', { class: 'part-inputs' });
      for (const [name, label] of Object.entries(part.inputs || {})) {
        const inp = el('input', { type: 'text', inputmode: 'decimal', autocomplete: 'off', spellcheck: 'false', 'aria-label': label.replace(/<[^>]+>/g, '') });
        inp.value = saved[`${part.id}.${name}`] ?? '';
        inp.addEventListener('input', () => {
          saved[`${part.id}.${name}`] = inp.value;
          WB.ui.store.set(key, saved);
        });
        const l = el('label', { class: 'part-label' });
        l.innerHTML = label;
        grid.append(el('div', { class: 'part-field' }, l, inp));
        inputs[name] = inp;
      }
      if (part.inputs) box.append(grid);
      if (part.html) {
        const p = el('p', { class: 'muted small' });
        p.innerHTML = part.html;
        WB.ui.linkifyNode(p);
        box.append(p);
      }
      let editor = null;
      if (part.code) {
        editor = codeEditor(saved[`${part.id}.code`] ?? part.code.template, (v) => { saved[`${part.id}.code`] = v; WB.ui.store.set(key, saved); });
        box.append(editor);
      }
      const result = el('div', { class: 'part-result', 'aria-live': 'polite' });
      const sol = el('div', { class: 'part-solution', hidden: '' });
      const btns = el('div', { class: 'part-buttons' });
      const values = () => Object.fromEntries(Object.entries(inputs).map(([n, i]) => [n, i.value]));
      const solvedKey = `${prob.id}/${part.id}`;
      if (part.after) {
        const gate = `${prob.id}/${part.after}`;
        WB.ui.addRefresher(() => { box.hidden = ctx.S.mode === 'work' && !ctx.app.isSolved(gate); });
      }
      if (part.done) {
        const b = el('button', { type: 'button', class: 'btn', text: part.done });
        b.addEventListener('click', () => { ctx.app.markSolved(solvedKey); ctx.update(); });
        WB.ui.addRefresher(() => { b.disabled = ctx.app.isSolved(solvedKey); });
        btns.append(b);
      }
      // Results may be a Promise (Python); show progress meanwhile.
      const report = (r, button) => {
        if (!r || typeof r.then !== 'function') {
          if (!r) return;
          showResult(result, r);
          if (r.ok && button && button.dataset.check) ctx.app.markSolved(solvedKey);
          return;
        }
        const starting = WB.py && WB.py.status() !== 'ready';
        result.replaceChildren(el('span', { class: 'status-msg', text: starting ? 'Starting Python (the first time downloads about 15 MB)…' : 'Running…' }));
        if (button) button.disabled = true;
        r.then((res) => {
          showResult(result, res);
          if (res && res.ok && button && button.dataset.check) ctx.app.markSolved(solvedKey);
        }, (e) => showResult(result, { ok: false, msg: String(e.message || e) }))
          .finally(() => { if (button) button.disabled = false; });
      };
      if (part.check || (part.code && part.code.check)) {
        const b = el('button', { type: 'button', class: 'btn', text: 'Check', 'data-check': '1' });
        b.addEventListener('click', () => {
          if (part.code) report(part.code.check(editor.value()), b);
          else report(part.check(values()), b);
        });
        btns.append(b);
      }
      for (const extra of [...(part.actions || []), ...((part.code && part.code.actions) || [])]) {
        const b = el('button', { type: 'button', class: 'btn', text: extra.label });
        b.addEventListener('click', () => {
          const r = extra.run(part.code ? editor.value() : values());
          if (r) report(r, b);
        });
        btns.append(b);
      }
      if (part.code) {
        btns.append(el('button', {
          type: 'button', class: 'btn btn-quiet', text: 'Reset code',
          onclick: () => { if (window.confirm('Replace your code with the starting template?')) editor.setValue(part.code.template); },
        }));
      }
      if (part.solution) {
        btns.append(el('button', {
          type: 'button', class: 'btn btn-quiet', text: 'Show solution',
          onclick: (e) => {
            sol.hidden = !sol.hidden;
            e.target.textContent = sol.hidden ? 'Show solution' : 'Hide solution';
            if (!sol.hidden) renderSolution(sol, part.solution());
          },
        }));
        WB.ui.addRefresher(() => { if (!sol.hidden) renderSolution(sol, part.solution()); });
      }
      if (btns.children.length) box.append(btns);
      box.append(result, sol);
      partBody.append(...[...box.childNodes].slice(1));
      box.append(partBody);
      parent.append(box);
    }
    if (parts.length > 1) {
      tools.append(
        el('button', { type: 'button', class: 'btn btn-quiet btn-small', text: 'Collapse all parts', onclick: () => partToggles.forEach((t) => t.set(true)) }),
        el('button', { type: 'button', class: 'btn btn-quiet btn-small', text: 'Expand all parts', onclick: () => partToggles.forEach((t) => t.set(false)) }));
    }
  }

  // Plain textarea editor: Tab indents four spaces, Enter keeps the indent.
  function codeEditor(initial, onChange) {
    const ta = el('textarea', { class: 'code-editor', spellcheck: 'false', autocomplete: 'off', autocapitalize: 'off', 'aria-label': 'Python code' });
    ta.value = initial;
    const fit = () => { ta.rows = Math.max(4, ta.value.split('\n').length + 1); };
    const insert = (text) => {
      const { selectionStart: a, selectionEnd: b } = ta;
      ta.setRangeText(text, a, b, 'end');
      onChange(ta.value); fit();
    };
    ta.addEventListener('keydown', (e) => {
      if (e.key === 'Tab' && !e.shiftKey && !e.ctrlKey && !e.metaKey) { e.preventDefault(); insert('    '); }
      if (e.key === 'Enter' && !e.shiftKey) {
        const line = ta.value.slice(0, ta.selectionStart).split('\n').pop();
        let indent = line.match(/^\s*/)[0];
        if (/:\s*$/.test(line)) indent += '    ';
        e.preventDefault(); insert('\n' + indent);
      }
    });
    ta.addEventListener('input', () => { onChange(ta.value); fit(); });
    fit();
    const wrap = el('div', { class: 'code-wrap' }, ta);
    wrap.value = () => ta.value;
    wrap.setValue = (v) => { ta.value = v; onChange(v); fit(); };
    return wrap;
  }

  function renderSolution(node, s) {
    node.replaceChildren();
    for (const line of s) {
      if (line.tex) {
        const d = el('div', { class: 'sol-tex' });
        WB.ui.renderTex(d, line.tex, true);
        node.append(d);
      } else if (line.code) {
        node.append(el('pre', { class: 'sol-code', text: line.code }));
      } else {
        const p = el('p', {});
        p.innerHTML = line.html;
        WB.ui.linkifyNode(p);
        node.append(p);
      }
    }
  }

  function showResult(node, r) {
    node.replaceChildren(el('span', { class: r.ok ? 'status good' : 'status bad' },
      el('span', { class: 'status-icon', 'aria-hidden': 'true', text: r.ok ? '✓' : '✗' }),
      el('span', { text: r.ok ? 'Correct' : 'Not yet' })), el('span', { class: 'status-msg', text: r.msg || '' }));
    if (r.detail) node.append(el('pre', { class: 'py-detail', text: r.detail }));
  }

  const num = (s) => {
    const v = parseFloat(String(s).replace(/−/g, '-'));
    return isNaN(v) ? null : v;
  };

  function checkNumbers(vals, truth, labels) {
    const bad = [];
    for (const [k, v] of Object.entries(truth)) {
      const g = num(vals[k]);
      if (g === null) return { ok: false, msg: `Enter a number for ${labels[k] || k}.` };
      // 1% relative; the 1e-3 absolute floor applies only when the answer is 0, so
      // small gains (e.g. k_P = −0.0049) are not accepted within ±20%.
      if (!M.close(g, v, 0.01, Math.abs(v) < 1e-12 ? 1e-3 : 0)) bad.push(labels[k] || k);
    }
    return bad.length ? { ok: false, msg: `Check ${bad.join(', ')}.` } : { ok: true, msg: 'Within 1%.' };
  }

  const useGainsAction = (ctx) => WB.design.useGains(ctx, ['kP', 'kD'], { target: () => ctx.st, msg: 'Enter kP and kD first.' });

  // ------------------------------------------------------------- Chapter 7 --
  WB.chapters.ch7 = {
    id: 'ch7', num: 7, tab: 'Ch 7', title: 'Pole placement (PD)', pages: 'pp. 99–106',
    controller: (ctx, o) => makeController(ctx, o),

    defaults(sys) {
      const prob = sys.problems.ch7;
      return {
        arch: 'output', comp: 'fl',
        kP: 0.05, kD: 0.01,               // work-mode starting gains (not the answer)
        form: 'real',
        p1: prob.desiredPoles[0].re, p2: prob.desiredPoles[1].re,
        sigma: -3.5, wd: 2,
      };
    },
    simDefaults(sys) { return sys.problems.ch7.sim; },

    designPoles(ctx) {
      const st = ctx.st;
      return st.form === 'real'
        ? [{ re: st.p1, im: 0 }, { re: st.p2, im: 0 }]
        : [{ re: st.sigma, im: st.wd }, { re: st.sigma, im: -st.wd }];
    },

    gains(ctx) {
      if (ctx.S.mode === 'work') return { kP: ctx.st.kP, kD: ctx.st.kD };
      return gainsFromPoles(ctx.model, this.designPoles(ctx));
    },

    buildControls(parent, ctx) {
      const sec = WB.ui.section(parent, 'PD controller', 'p. 99');
      sharedControls(sec, ctx);
      if (ctx.S.mode === 'work') {
        workGainSliders(sec, ctx);
        sec.append(el('p', { class: 'muted small', text: 'Target poles from the problem are drawn as dashed rings in the s-plane.' }));
        return;
      }
      const des = WB.ui.section(parent, 'Desired closed-loop poles', 'p. 100');
      segmented(des, {
        label: 'Pole pair',
        options: [{ value: 'real', label: 'two real' }, { value: 'complex', label: 'complex pair' }],
        ...bind(ctx, 'form'),
      });
      const realOnly = () => ctx.st.form !== 'real';
      const cplxOnly = () => ctx.st.form !== 'complex';
      slider(des, { label: 'p<sub>1</sub>', min: -20, max: 0, step: 0.01, ...bind(ctx, 'p1'), disabled: realOnly });
      slider(des, { label: 'p<sub>2</sub>', min: -20, max: 0, step: 0.01, ...bind(ctx, 'p2'), disabled: realOnly });
      slider(des, { label: '−σ', min: -20, max: 0, step: 0.01, ...bind(ctx, 'sigma'), disabled: cplxOnly });
      slider(des, { label: 'ω<sub>d</sub>', unit: 'rad/s', min: 0, max: 20, step: 0.01, ...bind(ctx, 'wd'), disabled: cplxOnly });
      des.append(el('p', { class: 'muted small', text: 'Or drag a closed-loop pole in the s-plane. Drag off the real axis for a complex pair.' }));
      gainReadout(des, ctx);
    },

    splane(ctx) {
      const markers = baseMarkers(ctx, ctx.S.mode === 'explore');
      if (ctx.S.mode === 'work') {
        for (const p of ctx.sys.problems.ch7.desiredPoles) markers.push({ ...p, kind: 'target', label: 'target pole (problem)' });
      }
      return { markers };
    },

    onPoleDrag(ctx, id, re, im) {
      const st = ctx.st;
      re = Math.min(-0.01, re);
      if (Math.abs(im) > 0.15) {
        st.form = 'complex'; st.sigma = re; st.wd = Math.abs(im);
      } else {
        if (st.form === 'complex') { st.p1 = st.p2 = st.sigma; }
        st.form = 'real';
        if (id === 0) st.p1 = re; else st.p2 = re;
      }
      ctx.update();
    },

    math(ctx) {
      const cards = [plantCard(ctx), olPolesCard(ctx), closedLoopCard(ctx)];
      const desired = ctx.S.mode === 'work' ? ctx.sys.problems.ch7.desiredPoles : this.designPoles(ctx);
      cards.push(placementCard(ctx, desired, ctx.S.mode === 'work' ? 'Pole placement (problem targets)' : 'Pole placement (your design)', 'p. 100'));
      cards.push(compensationCard(ctx));
      return cards;
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch7;
      const model = () => ctx.model;
      problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Open-loop poles',
          inputs: { p1: 'p<sub>1</sub>', p2: 'p<sub>2</sub>' },
          html: 'Complex values are fine: <code>-1+2j</code>.',
          check: (v) => {
            const g = [M.parseComplex(v.p1), M.parseComplex(v.p2)];
            if (!g[0] || !g[1]) return { ok: false, msg: 'Enter both poles.' };
            return M.polesMatch(g, M.roots2(model().a1, model().a0)) ? { ok: true, msg: '' } : { ok: false, msg: 'The open-loop poles are the roots of the denominator of P(s).' };
          },
          solution: () => {
            const ol = M.roots2(model().a1, model().a0);
            return [{ tex: `\\Delta_{ol}(s) = s^2 + ${tex(model().a1)}\\,s + ${tex(model().a0)} \\Rightarrow p_{ol} = ${texPole(ol[0])},\\; ${texPole(ol[1])}` }];
          },
        },
        {
          id: 'b', title: '(b) Closed-loop transfer function and characteristic polynomial',
          html: 'Derivative on the output (Fig. 7-2). Write both in terms of k<sub>P</sub> and k<sub>D</sub>; the check calls them at complex s. Any nonzero multiple of Δ<sub>cl</sub> is accepted.',
          code: {
            template: 'def closed_loop(s, kP, kD):\n    # Theta(s)/Theta_r(s)\n    return ...\n\ndef char_poly(s, kP, kD):\n    # Delta_cl(s)\n    return ...\n',
            check: (code) => {
              const cx = WB.py.cx;
              const den = (p, a) => { const m = ctx.sys.secondOrderModel(p); return cx.poly([1, m.a1 + m.b0 * a.kD, m.a0 + m.b0 * a.kP], a.s); };
              return WB.py.check(ctx, {
                args: { s: { label: 's', complex: true, re: [-6, 3], im: [0.3, 12] }, kP: { label: 'kP', lo: 0.02, hi: 2 }, kD: { label: 'kD', lo: 0.01, hi: 0.5 } },
                items: [
                  { fn: 'closed_loop', args: ['s', 'kP', 'kD'], truth: (p, a) => cx.div(ctx.sys.secondOrderModel(p).b0 * a.kP, den(p, a)) },
                  { fn: 'char_poly', args: ['s', 'kP', 'kD'], compare: 'scale', truth: den },
                ],
              }, code);
            },
          },
          solution: () => [
            { tex: `\\frac{${ctx.sys.sym.y}(s)}{${ctx.sys.sym.y}_r(s)} = \\frac{b_0 k_P}{s^2 + (a_1 + b_0 k_D)s + (a_0 + b_0 k_P)}` },
            { tex: `\\Delta_{cl}(s) = s^2 + (${tex(model().a1)} + ${tex(model().b0)}\\,k_D)\\,s + (${tex(model().a0)} + ${tex(model().b0)}\\,k_P)` },
            { code: 'b0 = 3 / (P.m * P.ell**2)\na1 = 3 * P.b / (P.m * P.ell**2)\n\ndef char_poly(s, kP, kD):\n    return s**2 + (a1 + b0 * kD) * s + b0 * kP\n\ndef closed_loop(s, kP, kD):\n    return b0 * kP / char_poly(s, kP, kD)' },
            { html: 'Book: Eq. 7.5 (p. 101) and the A.7 solution (pp. 102–103).' },
          ],
        },
        {
          id: 'c', title: `(c) Gains for poles at ${prob.desiredPoles.map((p) => fmtPole(p)).join(' and ')}`,
          inputs: { kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>' },
          check: (v) => checkNumbers(v, gainsFromPoles(model(), prob.desiredPoles), { kP: 'kP', kD: 'kD' }),
          actions: [useGainsAction(ctx)],
          solution: () => {
            const g = gainsFromPoles(model(), prob.desiredPoles);
            const { alpha1, alpha0 } = M.polyFromPoles(...prob.desiredPoles);
            return [{ tex: `\\Delta^d_{cl} = s^2 + ${tex(alpha1)}s + ${tex(alpha0)} \\Rightarrow k_P = ${tex(g.kP)},\\; k_D = ${tex(g.kD)}` }];
          },
        },
        {
          id: 'd', title: '(d) Simulate',
          html: 'Click <em>Use my gains</em> in (c), then compare the closed-loop × with the dashed target rings and look at the step response. The dashed orange trace is the linear design model, so any gap between it and θ comes from saturation, gravity, or plant mismatch.',
          check: () => {
            const cl = clPoles(ctx.model, ctx.st.kP, ctx.st.kD);
            if (ctx.S.mode !== 'work') return { ok: false, msg: 'Switch to Work mode so the simulation uses your gains.' };
            return M.polesMatch(cl, prob.desiredPoles, 0.02, 0.02)
              ? { ok: true, msg: 'The simulated closed loop has the target poles.' }
              : { ok: false, msg: `Current closed-loop poles: ${cl.map((p) => fmtPole(p)).join(', ')}.` };
          },
        },
      ]);
    },
  };

  // ------------------------------------------------------------- Chapter 8 --
  WB.chapters.ch8 = {
    id: 'ch8', num: 8, tab: 'Ch 8', title: 'Second-order design', pages: 'pp. 107–136',
    controller: (ctx, o) => makeController(ctx, o),
    targets: (ctx) => ({ tr: ctx.st.tr, zeta: ctx.st.zeta }),

    defaults(sys) {
      const prob = sys.problems.ch8;
      return { arch: 'output', comp: 'fl', kP: 0.05, kD: 0.01, tr: prob.tr, zeta: prob.zeta, rule: '2.2' };
    },
    simDefaults(sys) { return sys.problems.ch8.sim; },

    wn(ctx) { return wnFromTr(ctx.st.tr, ctx.st.zeta, ctx.st.rule); },
    designPoles(ctx) { return polesFromWnZeta(this.wn(ctx), ctx.st.zeta); },

    gains(ctx) {
      if (ctx.S.mode === 'work') return { kP: ctx.st.kP, kD: ctx.st.kD };
      return gainsFromPoles(ctx.model, this.designPoles(ctx));
    },

    // Fig. 8-13: room left for u~ once the equilibrium/feedforward input is accounted for.
    satBound(ctx) {
      const { sys, pModel, S, model } = ctx;
      const y0 = S.sim.y0 * M.DEG;
      const ue = ctx.st.comp === 'none' ? 0 : ctx.st.comp === 'fl' ? sys.feedbackLinearization([y0, 0], pModel) : sys.equilibriumInput(0, pModel);
      const umax = sys.uLimit(pModel);
      const uTildeMax = Math.max(0, umax - Math.abs(ue));
      const eMax = Math.abs(S.sim.amplitude - S.sim.y0) * M.DEG;
      const wnMax = eMax > 0 ? Math.sqrt(model.a0 + model.b0 * uTildeMax / eMax) : Infinity;
      return { ue, umax, uTildeMax, eMax, wnMax, trMin: 2.2 / wnMax };
    },

    buildControls(parent, ctx) {
      const sec = WB.ui.section(parent, 'PD controller', 'p. 111 · Fig. 8-12');
      sharedControls(sec, ctx);
      if (ctx.S.mode === 'work') workGainSliders(sec, ctx);
      const spec = WB.ui.section(parent, ctx.S.mode === 'work' ? 'Specs (targets)' : 'Design knobs', 'p. 113 · Eq. 8.5');
      slider(spec, { label: 't<sub>r</sub>', unit: 's', min: 0.1, max: 3, step: 0.005, ...bind(ctx, 'tr') });
      slider(spec, { label: 'ζ', min: 0.1, max: 2, step: 0.005, ...bind(ctx, 'zeta') });
      segmented(spec, {
        label: 'ω<sub>n</sub> from t<sub>r</sub>',
        options: [
          { value: '2.2', label: '2.2 / t<sub>r</sub>', title: 'Eq. 8.5, exact for ζ = 0.707' },
          { value: 'tp', label: 'π / (2 t<sub>r</sub>√(1−ζ²))', title: 't_r ≈ t_p / 2 (p. 113); used in ctrlPID.py' },
        ],
        ...bind(ctx, 'rule'),
      });
      if (ctx.S.mode === 'explore') {
        spec.append(el('p', { class: 'muted small', text: 'Drag a closed-loop pole: its distance from the origin sets ωₙ and its angle sets ζ.' }));
        gainReadout(spec, ctx);
      }
    },

    splane(ctx) {
      const markers = baseMarkers(ctx, ctx.S.mode === 'explore');
      const wn = this.wn(ctx);
      if (ctx.S.mode === 'work') {
        for (const p of this.designPoles(ctx)) markers.push({ ...p, kind: 'target', label: 'target pole (spec)' });
      }
      const sb = this.satBound(ctx);
      return { markers, zetaRay: ctx.st.zeta, wnCircle: wn, wnMax: isFinite(sb.wnMax) ? sb.wnMax : null };
    },

    onPoleDrag(ctx, id, re, im) {
      const st = ctx.st;
      re = Math.min(-0.01, re);
      const wn = Math.hypot(re, im);
      st.zeta = Math.max(0.1, Math.min(1, -re / wn));
      st.tr = Math.max(0.05, st.rule === 'tp' ? Math.PI / (2 * wn * Math.sqrt(Math.max(1e-6, 1 - st.zeta ** 2))) : 2.2 / wn);
      ctx.update();
    },

    math(ctx) {
      const st = ctx.st;
      const wn = this.wn(ctx);
      const poles = this.designPoles(ctx);
      const sb = this.satBound(ctx);
      const sym = ctx.sys.sym;
      const cards = [plantCard(ctx), closedLoopCard(ctx)];
      cards.push({
        title: 'Spec → desired characteristic polynomial', page: 'p. 110 · Eq. 8.2, p. 113 · Eq. 8.5',
        theory: (st.rule === 'tp' ? '\\omega_n = \\frac{\\pi}{2 t_r \\sqrt{1-\\zeta^2}}' : '\\omega_n = \\frac{2.2}{t_r}')
          + ',\\quad \\Delta^d_{cl} = s^2 + 2\\zeta\\omega_n s + \\omega_n^2,\\quad p = -\\zeta\\omega_n \\pm j\\omega_n\\sqrt{1-\\zeta^2}',
        numbers: `\\omega_n = ${tex(wn)},\\quad \\Delta^d_{cl} = s^2 + ${tex(2 * st.zeta * wn)}\\,s + ${tex(wn * wn)},\\quad p = ${texPole(poles[0])},\\; ${texPole(poles[1])}`,
        spoiler: true,
      });
      cards.push(placementCard(ctx, poles, 'Gains from the spec', 'p. 100, A.8 p. 122'));
      cards.push({
        title: 'Saturation limits the rise time', page: 'p. 119 · Eq. 8.8, p. 120 · Fig. 8-13',
        theory: `k_P \\le \\frac{\\tilde{u}_{max}}{e_{max}},\\quad \\omega_n \\le \\sqrt{a_0 + b_0\\frac{\\tilde{u}_{max}}{e_{max}}},\\quad t_r \\ge \\frac{2.2}{\\omega_{n,max}},\\quad \\tilde{u}_{max} = u_{max} - |u_e|`,
        numbers: `u_e = ${tex(sb.ue)}\\;(${st.comp === 'fl' ? sym.ff + '\\text{ at } ' + sym.y + '_0' : st.comp === 'eq' ? sym.u + '_e' : '\\text{none}'}),\\quad \\tilde{u}_{max} = ${tex(sb.uTildeMax)},\\; e_{max} = ${tex(sb.eMax)}\\,\\text{rad}\\quad \\Rightarrow \\omega_{n,max} = ${tex(sb.wnMax)},\\; t_{r,min} = ${tex(sb.trMin)}\\,\\text{s}`,
        spoiler: true,
        note: 'u_e uses the feedforward torque at the initial angle; e_max is the size of the first step. The red region in the s-plane marks ωₙ > ωₙ,max.',
      });
      cards.push(compensationCard(ctx));
      return cards;
    },

    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch8;
      const self = this;
      const model = () => ctx.model;
      const specGains = () => {
        const wn = 2.2 / prob.tr;
        return { wn, alpha1: 2 * prob.zeta * wn, alpha0: wn * wn, ...gainsFromPoles(model(), polesFromWnZeta(wn, prob.zeta)) };
      };
      // Peak demanded input for a satStepDeg step from rest, using gains designed from tr.
      const peakFor = (tr) => {
        const wn = 2.2 / tr;
        const g = gainsFromPoles(model(), polesFromWnZeta(wn, prob.zeta));
        const fake = { ...ctx, gains: g, st: { ...ctx.st, comp: 'fl', arch: 'output' } };
        const p = ctx.pModel;
        const out = WB.sim.simulate({
          plant: { f: (x, u) => ctx.sys.f(x, u, p), h: ctx.sys.h, uLimit: ctx.sys.uLimit(p) },
          controller: makeController(fake),
          reference: WB.sim.makeReference({ type: 'step', amplitude: prob.satStepDeg * M.DEG, tStep: 0 }),
          disturbance: () => 0, x0: ctx.sys.x0(0), Ts: ctx.S.sim.Ts, tEnd: 3,
        });
        let peak = 0;
        for (const u of out.uDemand) peak = Math.max(peak, Math.abs(u));
        return peak / ctx.sys.uLimit(p);
      };
      problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) t<sub>r</sub> = ${prob.tr} s, ζ = ${prob.zeta}`,
          html: 'Δ<sup>d</sup><sub>cl</sub>(s) = s² + α<sub>1</sub>s + α<sub>0</sub>',
          inputs: { wn: 'ω<sub>n</sub>', alpha1: 'α<sub>1</sub>', alpha0: 'α<sub>0</sub>', kP: 'k<sub>P</sub>', kD: 'k<sub>D</sub>' },
          check: (v) => {
            const s = specGains();
            return checkNumbers(v, { wn: s.wn, alpha1: s.alpha1, alpha0: s.alpha0, kP: s.kP, kD: s.kD }, { wn: 'ωn', alpha1: 'α1', alpha0: 'α0' });
          },
          actions: [useGainsAction(ctx)],
          solution: () => {
            const s = specGains();
            return [
              { tex: `\\omega_n = 2.2/${prob.tr} = ${tex(s.wn)},\\quad \\Delta^d_{cl} = s^2 + ${tex(s.alpha1)}s + ${tex(s.alpha0)}` },
              { tex: `k_P = ${tex(s.kP)},\\quad k_D = ${tex(s.kD)}` },
              { html: 'Book: Eqs. 8.9–8.10 (pp. 121–122).' },
            ];
          },
        },
        {
          id: 'b', title: `(b) Fastest t<sub>r</sub> that just saturates on a ${prob.satStepDeg}° step`,
          inputs: { tr: 't<sub>r</sub> [s]' },
          html: `Checked by simulation: a ${prob.satStepDeg}° step from rest with ζ = ${prob.zeta}, ω<sub>n</sub> = 2.2/t<sub>r</sub>, feedback linearization on. The peak demanded |τ| should land between 95% and 100% of τ<sub>max</sub>.`,
          check: (v) => {
            const tr = num(v.tr);
            if (tr === null || tr <= 0) return { ok: false, msg: 'Enter a positive rise time.' };
            const pk = peakFor(tr);
            const pct = (100 * pk).toFixed(1);
            if (pk > 1.0005) return { ok: false, msg: `Peak demand is ${pct}% of τmax, so it saturates. Slow it down.` };
            if (pk < 0.95) return { ok: false, msg: `Peak demand is ${pct}% of τmax. You can go faster.` };
            return { ok: true, msg: `Peak demand is ${pct}% of τmax.` };
          },
          actions: [{
            label: 'Try it',
            run: (v) => {
              const tr = num(v.tr);
              if (tr === null || tr <= 0) return { ok: false, msg: 'Enter a positive rise time.' };
              ctx.app.setMode('explore');
              Object.assign(ctx.st, { tr, zeta: prob.zeta, rule: '2.2', comp: 'fl', arch: 'output' });
              Object.assign(ctx.S.sim, { type: 'step', amplitude: prob.satStepDeg, y0: 0 });
              ctx.update();
              return null;
            },
          }],
          solution: () => {
            const sb = self.satBound({ ...ctx, st: { ...ctx.st, comp: 'fl' }, S: { ...ctx.S, sim: { ...ctx.S.sim, y0: 0, amplitude: prob.satStepDeg } } });
            return [
              { html: 'The largest demand is right after the step, when θ̇ = 0 and θ = 0, so τ = τ<sub>fl</sub>(0) + k<sub>P</sub>e<sub>max</sub> ≤ τ<sub>max</sub> (Eq. 8.8 and Fig. 8-13):' },
              { tex: `k_P \\le \\frac{${tex(sb.umax)} - ${tex(sb.ue)}}{${tex(sb.eMax)}} \\Rightarrow \\omega_n \\le ${tex(sb.wnMax)} \\Rightarrow t_r \\ge ${tex(sb.trMin)}\\,\\text{s}` },
              { html: `The book's solution (and <code>_A_arm/python/ctrlPD.py</code>) uses t<sub>r</sub> = ${prob.bookTr} s, tuned on the ±50° square wave. With that value the first 0→50° step demands ${(100 * peakFor(prob.bookTr)).toFixed(0)}% of τ<sub>max</sub>, so it saturates briefly.` },
            ];
          },
        },
      ]);
    },
  };

  // Shared with later chapters (PID, root locus, frequency response).
  WB.pd = { makeController, clPoles, gainsFromPoles, polesFromWnZeta, wnFromTr, problemPanel, checkNumbers, num, sharedControls, plantCard, closedLoopCard, compensationCard, baseMarkers, showResult };
})();
