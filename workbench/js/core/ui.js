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

  // Page references are controlbook.pdf page numbers (not book page numbers).
  // The PDF is gitignored, so links only resolve where book_and_notes/ exists locally.
  const PDF_PATH = '../book_and_notes/controlbook.pdf';
  // "p. 101 · Eq. 7.5", "pp. 102–103", "A.7 p. 102": a page and the labels that follow it.
  const PAGE_RE = /(?:[A-F]\.\d+\s+)?pp?\.\s*(\d+)(?:[–-]\d+)?(?:\s*·\s*(?:Eqs?\.|Fig\.|Listing)\s*[\d.–-]+)*/g;

  function pdfLink(page, text) {
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

  return { el, section, pageChip, linkPages, linkifyNode, slider, segmented, refreshAll, clearRefreshers, addRefresher, renderTex, store };
})();

// Load a study's scripts during page parse (classic scripts, works from file://).
WB.studies = WB.studies || {};
WB.loadStudy = function (id, files) {
  for (const f of files) document.write(`<script src="${f}"><\/script>`);
};

