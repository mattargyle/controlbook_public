// Small DOM helpers: labeled sliders, selects, segmented toggles, KaTeX, storage.
// Every control reads its value through get() and writes through set(), so the
// app can re-sync all controls after a change made elsewhere (e.g. dragging a pole).
window.WB = window.WB || {};

WB.ui = (function () {
  const refreshers = new Set();

  function el(tag, attrs = {}, ...children) {
    const node = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
      if (k === 'class') node.className = v;
      else if (k === 'text') node.textContent = v;
      else if (k.startsWith('on')) node.addEventListener(k.slice(2), v);
      else node.setAttribute(k, v);
    }
    for (const c of children) if (c) node.append(c);
    return node;
  }

  function section(parent, title, pageRef) {
    const head = el('div', { class: 'section-head' }, el('h3', { text: title }));
    if (pageRef) head.append(pageChip(pageRef));
    const body = el('div', { class: 'section-body' });
    parent.append(el('section', { class: 'panel-section' }, head, body));
    return body;
  }

  function pageChip(ref) {
    return el('span', { class: 'page-ref', title: 'controlbook.pdf page', text: ref });
  }

  // Slider + number box. opts: label, unit, min, max, step, get, set, sig, log, hint
  function slider(parent, opts) {
    const sig = opts.sig || 4;
    const toSlider = (v) => (opts.log ? Math.log10(Math.max(v, 1e-12)) : v);
    const fromSlider = (s) => (opts.log ? Math.pow(10, s) : s);
    const range = el('input', {
      type: 'range',
      min: toSlider(opts.min), max: toSlider(opts.max),
      step: opts.log ? (toSlider(opts.max) - toSlider(opts.min)) / 400 : opts.step,
      'aria-label': opts.label,
    });
    const num = el('input', { type: 'number', step: 'any', class: 'num', 'aria-label': opts.label + ' value' });
    const label = el('label', { class: 'ctl-label' });
    label.innerHTML = opts.label; // labels are authored in this repo (may contain <sub>)
    const unit = el('span', { class: 'unit', text: opts.unit || '' });
    const row = el('div', { class: 'ctl slider-row' }, label, range, el('span', { class: 'num-wrap' }, num, unit));
    if (opts.hint) row.append(el('div', { class: 'ctl-hint', text: opts.hint }));
    parent.append(row);

    range.addEventListener('input', () => opts.set(fromSlider(parseFloat(range.value))));
    num.addEventListener('change', () => {
      const v = parseFloat(num.value);
      if (!isNaN(v)) opts.set(v);
    });

    const refresh = () => {
      const v = opts.get();
      const disabled = opts.disabled ? opts.disabled() : false;
      range.disabled = num.disabled = disabled;
      row.classList.toggle('is-disabled', disabled);
      range.value = toSlider(v);
      if (document.activeElement !== num) num.value = Number(v.toPrecision(sig));
    };
    refreshers.add(refresh);
    refresh();
    return { row, refresh };
  }

  // opts: label, options [{value, label}], get, set
  function segmented(parent, opts) {
    const wrap = el('div', { class: 'segmented', role: 'radiogroup', 'aria-label': opts.label });
    const buttons = opts.options.map((o) => {
      const b = el('button', { type: 'button', role: 'radio', title: o.title || '' });
      b.innerHTML = o.label;
      b.addEventListener('click', () => opts.set(o.value));
      wrap.append(b);
      return [o.value, b];
    });
    const row = el('div', { class: 'ctl seg-row' });
    if (opts.label) {
      const l = el('span', { class: 'ctl-label' });
      l.innerHTML = opts.label;
      row.append(l);
    }
    row.append(wrap);
    parent.append(row);
    const refresh = () => {
      const v = opts.get();
      for (const [val, b] of buttons) {
        b.setAttribute('aria-checked', String(val === v));
        b.classList.toggle('on', val === v);
      }
    };
    refreshers.add(refresh);
    refresh();
    return { row, refresh };
  }

  function refreshAll() {
    for (const r of refreshers) r();
  }

  function clearRefreshers() {
    refreshers.clear();
  }

  function addRefresher(fn) {
    refreshers.add(fn);
  }

  // Render TeX with KaTeX when it loaded (CDN); otherwise show the source.
  function renderTex(node, src, display = true) {
    if (window.katex) {
      try {
        window.katex.render(src, node, { displayMode: display, throwOnError: false });
        return;
      } catch (e) { /* fall through */ }
    }
    node.textContent = src;
    node.classList.add('tex-fallback');
  }

  const store = {
    get(key, fallback) {
      try {
        const v = localStorage.getItem(key);
        return v === null ? fallback : JSON.parse(v);
      } catch (e) {
        return fallback;
      }
    },
    set(key, value) {
      try { localStorage.setItem(key, JSON.stringify(value)); } catch (e) { /* storage unavailable */ }
    },
  };

  return { el, section, pageChip, slider, segmented, refreshAll, clearRefreshers, addRefresher, renderTex, store };
})();
