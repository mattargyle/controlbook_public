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
    const h3 = el('h3');
    const head = el('div', { class: 'section-head' }, h3);
    if (pageRef) head.append(pageChip(pageRef));
    const body = el('div', { class: 'section-body' });
    const box = el('section', { class: 'panel-section' }, head, body);
    collapsible(h3, title, box, body, `wb.collapsed.section.${title.replace(/<[^>]+>/g, '')}`);
    parent.append(box);
    return body;
  }

  // Turn `host` into a toggle button (chevron + label html) that collapses `body`
  // (an element or a list of them). `box` gets the class "collapsed". The state is
  // remembered under `key`. `labelHtml` may instead be an existing element, which
  // moves into the button (so code that retitles it keeps working).
  function collapsible(host, labelHtml, box, body, key) {
    let label = labelHtml;
    if (label instanceof Node) label.classList.add('collapse-label');
    else {
      label = el('span', { class: 'collapse-label' });
      label.innerHTML = labelHtml; // authored text (may contain <sub>)
    }
    const btn = el('button', { type: 'button', class: 'collapse-toggle', 'aria-expanded': 'true' },
      el('span', { class: 'chev', 'aria-hidden': 'true', text: '▾' }), label);
    host.prepend(btn);
    const set = (collapsed) => {
      box.classList.toggle('collapsed', collapsed);
      for (const b of [].concat(body)) b.hidden = collapsed;
      btn.setAttribute('aria-expanded', String(!collapsed));
      btn.classList.toggle('is-collapsed', collapsed);
    };
    set(!!store.get(key, false));
    btn.addEventListener('click', () => {
      const collapsed = !box.classList.contains('collapsed');
      set(collapsed);
      store.set(key, collapsed);
    });
    return { set: (c) => { set(c); store.set(key, c); }, collapsed: () => box.classList.contains('collapsed') };
  }

  // Page references are controlbook.pdf page numbers (not book page numbers).
  // The PDF is gitignored, so links only resolve where book_and_notes/ exists locally.
  const PDF_PATH = '../book_and_notes/controlbook.pdf';
  // "p. 101 · Eq. 7.5", "pp. 102–103", "A.7 p. 102": a page and the labels that follow it.
  const PAGE_RE = /(?:[A-F]\.\d+\s+)?pp?\.\s*(\d+)(?:[–-]\d+)?(?:\s*·\s*(?:Eqs?\.|Fig\.|Listing)\s*[\d.–-]+)*/g;

  // When served over http(s) (e.g. GitHub Pages) the local PDF isn't there: link
  // to the public copy from the repo README instead. It can't jump to a page, so
  // the tooltip names the page.
  const HOSTED = /^https?:$/.test(location.protocol);
  const PUBLIC_PDF = 'https://drive.google.com/file/d/1OH6oSsbbdsxkY2CTMMxnchkWnNy_16zy/view?usp=sharing';

  function pdfLink(page, text) {
    if (HOSTED) {
      return el('a', { class: 'pdf-link', href: PUBLIC_PDF, target: 'controlbook-pdf', rel: 'noopener', title: `PDF page ${page} (opens the public PDF; go to page ${page})`, text });
    }
    // A named target reuses one PDF tab instead of opening a new tab per click.
    const a = el('a', { class: 'pdf-link', href: `${PDF_PATH}#page=${page}`, target: 'controlbook-pdf', title: `Open controlbook.pdf at page ${page}`, text });
    // If that tab already shows the PDF, a change to #page alone is a same-document
    // navigation that the PDF viewer ignores. A fresh query string forces a real
    // load, so the viewer opens at the new page.
    a.addEventListener('click', () => { a.href = `${PDF_PATH}?v=${Date.now()}#page=${page}`; });
    return a;
  }

  // Split text into plain runs and PDF links; returns an array of nodes.
  function linkPages(text) {
    const out = [];
    let last = 0;
    for (const m of text.matchAll(PAGE_RE)) {
      if (m.index > last) out.push(document.createTextNode(text.slice(last, m.index)));
      out.push(pdfLink(m[1], m[0]));
      last = m.index + m[0].length;
    }
    if (last < text.length) out.push(document.createTextNode(text.slice(last)));
    return out;
  }

  // Rewrite page references inside already-rendered (authored) HTML.
  function linkifyNode(root) {
    const walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT);
    const nodes = [];
    while (walker.nextNode()) if (!walker.currentNode.parentElement.closest('a, .katex')) nodes.push(walker.currentNode);
    for (const n of nodes) {
      // search() ignores lastIndex; test() on a /g regex would leave it advanced,
      // and matchAll() in linkPages copies lastIndex, skipping the first match.
      if (n.nodeValue.search(PAGE_RE) >= 0) n.replaceWith(...linkPages(n.nodeValue));
    }
  }

  function pageChip(ref) {
    return el('span', { class: 'page-ref' }, ...linkPages(ref));
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

  // ------------------------------------------------- chapter-panel helpers --
  // get/set for a slider or segmented control bound to obj()[key] (default ctx.st);
  // every change re-renders through ctx.update().
  function bind(ctx, key, obj = () => ctx.st) {
    return { get: () => obj()[key], set: (v) => { obj()[key] = v; ctx.update(); } };
  }

  // One slider per key from spec = {key: [label, min, max]}, bound to obj() (default
  // the Work-mode gains ctx.st.w). steps sets the slider resolution.
  function gainSliders(parent, ctx, spec, keys, { obj = () => ctx.st.w, steps = 4000 } = {}) {
    return keys.map((k) => {
      const [label, min, max] = spec[k];
      return slider(parent, { label, min, max, step: (max - min) / steps, sig: 4, ...bind(ctx, k, obj) });
    });
  }

  // on/off segmented control for a compensator block (obj() returns the block).
  function onOff(parent, ctx, obj, label) {
    return segmented(parent, { label, options: [{ value: true, label: 'on' }, { value: false, label: 'off' }], ...bind(ctx, 'on', obj) });
  }

  // A label/value row; status = {ok, text} adds a ✓/✗ badge.
  function metric(label, value, status) {
    const r = el('div', { class: 'metric' }, el('span', { class: 'metric-label', text: label }), el('strong', { text: value ?? '' }));
    if (status) r.append(el('span', { class: 'status ' + (status.ok ? 'good' : 'bad') }, el('span', { class: 'status-icon', 'aria-hidden': 'true', text: status.ok ? '✓' : '✗' }), el('span', { text: status.text })));
    return r;
  }
  // A spec-check row: met / not met.
  const specRow = (label, ok, value) => metric(label, value, { ok, text: ok ? 'met' : 'not met' });

  // Live readout box; rows() returns [[label, value], ...] (numbers shown to 4 figures).
  function readout(parent, rows, { wrap = true } = {}) {
    const box = el('div', { class: wrap ? 'readout wrap' : 'readout' });
    parent.append(box);
    addRefresher(() => box.replaceChildren(...rows().map(([k, v]) =>
      el('div', {}, el('span', { class: 'ro-label', text: k }), el('strong', { text: typeof v === 'number' ? WB.math.fmt(v, 4) : v })))));
    return box;
  }

  // Work-mode spoilers: true in Explore mode or once `key` has been revealed.
  const shown = (ctx, key) => ctx.S.mode === 'explore' || ctx.app.isRevealed(key);
  // A Reveal button for `key`. hideWhenShown removes it once revealed (or in Explore).
  function revealButton(ctx, key, text, { hideWhenShown = false } = {}) {
    const b = el('button', { type: 'button', class: 'btn btn-quiet', text, onclick: () => { ctx.app.reveal(key); ctx.update(); } });
    if (hideWhenShown) addRefresher(() => { b.hidden = shown(ctx, key); });
    return b;
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

  return {
    el, section, collapsible, pageChip, linkPages, linkifyNode, slider, segmented, refreshAll, clearRefreshers, addRefresher, renderTex, store,
    bind, gainSliders, onOff, metric, specRow, readout, shown, revealButton,
  };
})();

// Load a study's scripts during page parse (classic scripts, works from file://).
WB.studies = WB.studies || {};
WB.loadStudy = function (id, files) {
  for (const f of files) document.write(`<script src="${f}"><\/script>`);
};

