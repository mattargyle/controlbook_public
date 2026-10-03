// Canvas plots: TimePlot (signals vs. time, with crosshair + tooltip) and
// SPlane (pole/zero map with optional draggable poles and design overlays).
// Colors come from CSS custom properties so light/dark themes work unchanged.
window.WB = window.WB || {};

WB.plot = (function () {
  const { el } = WB.ui;
  const { fmt } = WB.math;

  function css(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#888';
  }

  function niceTicks(lo, hi, target = 5) {
    const span = hi - lo;
    if (!(span > 0)) return [lo];
    const raw = span / target;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => span / s <= target + 1) || 10 * mag;
    const ticks = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-9; v += step) ticks.push(Math.abs(v) < step * 1e-9 ? 0 : v);
    return ticks;
  }

  function setupCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h };
  }

  function legendItem(s) {
    const key = el('span', { class: 'legend-key' + (s.dash ? ' dashed' : '') });
    key.style.setProperty('--key-color', `var(${s.color})`);
    return el('span', { class: 'legend-item' }, key, el('span', { text: s.label }));
  }

  // ---------------------------------------------------------------------------
  class TimePlot {
    constructor(wrap, opts = {}) {
      this.opts = opts;
      this.legend = el('div', { class: 'legend' });
      this.canvas = el('canvas', { class: 'plot-canvas', role: 'img', 'aria-label': opts.title || 'plot' });
      this.tip = el('div', { class: 'tooltip', hidden: '' });
      const box = el('div', { class: 'plot-box' }, this.canvas, this.tip);
      const head = el('div', { class: 'plot-head' }, el('span', { class: 'plot-title', text: opts.title || '' }), this.legend);
      wrap.append(head, box);
      this.box = box;
      this.data = null;
      this.cursor = null;
      this.onHover = null;
      new ResizeObserver(() => this.draw()).observe(box);
      box.addEventListener('pointermove', (e) => this._pointer(e));
      box.addEventListener('pointerleave', () => this.onHover && this.onHover(null));
    }

    setData(data) {
      this.data = data;
      this.legend.replaceChildren(...data.series.filter((s) => s.label).map(legendItem));
      this.draw();
    }

    setCursor(i) {
      this.cursor = i;
      this.draw();
    }

    _layout() {
      return { l: 48, r: 12, t: 20, b: 24 };
    }

    _pointer(e) {
      if (!this.data || !this.onHover) return;
      const rect = this.canvas.getBoundingClientRect();
      const m = this._layout();
      const { t } = this.data;
      const frac = (e.clientX - rect.left - m.l) / (rect.width - m.l - m.r);
      const tq = t[0] + Math.max(0, Math.min(1, frac)) * (t[t.length - 1] - t[0]);
      const i = Math.round(((tq - t[0]) / (t[t.length - 1] - t[0])) * (t.length - 1));
      this.onHover(Math.max(0, Math.min(t.length - 1, i)));
    }

    draw() {
      if (!this.data) return;
      const { ctx, w, h } = setupCanvas(this.canvas);
      const m = this._layout();
      const { t, series, hlines = [], bands = [], vmarks = [], points = [] } = this.data;
      const t0 = t[0], t1 = t[t.length - 1];

      let lo = Infinity, hi = -Infinity;
      for (const s of series) for (const v of s.y) if (isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
      for (const hl of hlines) if (hl.fit !== false) { lo = Math.min(lo, hl.y); hi = Math.max(hi, hl.y); }
      if (this.opts.minSpan && hi - lo < this.opts.minSpan) {
        const mid = (hi + lo) / 2; lo = mid - this.opts.minSpan / 2; hi = mid + this.opts.minSpan / 2;
      }
      if (!(hi > lo)) { lo -= 1; hi += 1; }
      const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;

      const X = (v) => m.l + ((v - t0) / (t1 - t0)) * (w - m.l - m.r);
      const Y = (v) => m.t + (1 - (v - lo) / (hi - lo)) * (h - m.t - m.b);
      this.X = X; this.Y = Y;

      // bands (e.g. saturation region) behind everything
      for (const b of bands) {
        ctx.fillStyle = css(b.color);
        ctx.globalAlpha = b.alpha || 0.12;
        const ya = Y(Math.min(hi, Math.max(lo, b.y1))), yb = Y(Math.max(lo, Math.min(hi, b.y0)));
        ctx.fillRect(m.l, ya, w - m.l - m.r, yb - ya);
        ctx.globalAlpha = 1;
      }

      // grid + axis labels
      ctx.font = '11px var(--font-sans, system-ui)';
      ctx.fillStyle = css('--text-muted');
      ctx.strokeStyle = css('--grid');
      ctx.lineWidth = 1;
      ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
      for (const v of niceTicks(lo, hi, 4)) {
        const y = Math.round(Y(v)) + 0.5;
        ctx.beginPath(); ctx.moveTo(m.l, y); ctx.lineTo(w - m.r, y); ctx.stroke();
        ctx.fillText(fmt(v, 3), m.l - 6, y);
      }
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      for (const v of niceTicks(t0, t1, 8)) ctx.fillText(fmt(v, 3), X(v), h - m.b + 6);
      ctx.save(); ctx.translate(11, (h - m.b + m.t) / 2); ctx.rotate(-Math.PI / 2);
      ctx.textBaseline = 'middle'; ctx.fillText(this.opts.yLabel || '', 0, 0); ctx.restore();

      ctx.save();
      ctx.beginPath(); ctx.rect(m.l, m.t, w - m.l - m.r, h - m.t - m.b); ctx.clip();

      for (const hl of hlines) {
        ctx.strokeStyle = css(hl.color || '--text-muted');
        ctx.setLineDash(hl.dash || [4, 4]); ctx.lineWidth = 1;
        const y = Math.round(Y(hl.y)) + 0.5;
        ctx.beginPath(); ctx.moveTo(m.l, y); ctx.lineTo(w - m.r, y); ctx.stroke();
        if (hl.label) {
          // label sits below the line unless that would run off the bottom
          const below = y + 14 < h - m.b;
          ctx.setLineDash([]); ctx.fillStyle = css('--text-muted'); ctx.textAlign = 'right'; ctx.textBaseline = below ? 'top' : 'bottom';
          ctx.fillText(hl.label, w - m.r - 4, below ? y + 2 : y - 2);
        }
      }
      ctx.setLineDash([]);

      for (const vm of vmarks) {
        ctx.strokeStyle = css(vm.color || '--text-muted'); ctx.lineWidth = 1; ctx.setLineDash([2, 3]);
        const x = Math.round(X(vm.t)) + 0.5;
        ctx.beginPath(); ctx.moveTo(x, m.t); ctx.lineTo(x, h - m.b); ctx.stroke();
        ctx.setLineDash([]);
        if (vm.label) {
          ctx.fillStyle = css('--text-muted'); ctx.textAlign = 'left'; ctx.textBaseline = 'top';
          ctx.fillText(vm.label, x + 3, m.t + 2);
        }
      }

      for (const s of series) {
        ctx.strokeStyle = css(s.color); ctx.lineWidth = s.width || 2;
        ctx.setLineDash(s.dash || []); ctx.lineJoin = 'round';
        ctx.beginPath();
        let pen = false;
        for (let k = 0; k < t.length; k++) {
          const v = s.y[k];
          if (!isFinite(v)) { pen = false; continue; }
          const x = X(t[k]), y = Y(v);
          if (pen) ctx.lineTo(x, y); else { ctx.moveTo(x, y); pen = true; }
        }
        ctx.stroke();
      }
      ctx.setLineDash([]);

      for (const p of points) {
        ctx.fillStyle = css(p.color || '--text-primary');
        ctx.strokeStyle = css('--surface');
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(X(p.t), Y(p.y), 4.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        if (p.label) {
          ctx.fillStyle = css('--text-secondary'); ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
          ctx.fillText(p.label, X(p.t) + 7, Y(p.y) - 3);
        }
      }
      ctx.restore();

      // crosshair + tooltip
      if (this.cursor !== null && this.cursor !== undefined) {
        const i = this.cursor, x = Math.round(X(t[i])) + 0.5;
        ctx.strokeStyle = css('--text-secondary'); ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(x, m.t); ctx.lineTo(x, h - m.b); ctx.stroke();
        for (const s of series) {
          if (!isFinite(s.y[i])) continue;
          ctx.fillStyle = css(s.color); ctx.strokeStyle = css('--surface'); ctx.lineWidth = 2;
          ctx.beginPath(); ctx.arc(x, Y(s.y[i]), 4, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        }
        this._tooltip(i, x, w);
      } else {
        this.tip.hidden = true;
      }
    }

    _tooltip(i, x, w) {
      const { t, series } = this.data;
      const rows = [el('div', { class: 'tip-time', text: `t = ${fmt(t[i], 4)} s` })];
      for (const s of series) {
        if (!s.label) continue;
        const key = el('span', { class: 'tip-key' + (s.dash ? ' dashed' : '') });
        key.style.setProperty('--key-color', `var(${s.color})`);
        rows.push(el('div', { class: 'tip-row' }, key,
          el('strong', { text: `${fmt(s.y[i], 4)}${this.opts.unit ? ' ' + this.opts.unit : ''}` }),
          el('span', { class: 'tip-label', text: s.label })));
      }
      this.tip.replaceChildren(...rows);
      this.tip.hidden = false;
      const tw = this.tip.offsetWidth;
      this.tip.style.left = (x + 12 + tw > w ? x - 12 - tw : x + 12) + 'px';
      this.tip.style.top = '8px';
    }
  }

  // ---------------------------------------------------------------------------
  // Pole-zero map with equal axis scaling so damping-ratio angles read true.
  class SPlane {
    constructor(wrap, opts = {}) {
      this.opts = opts;
      this.legend = el('div', { class: 'legend' });
      this.canvas = el('canvas', { class: 'plot-canvas', role: 'img', 'aria-label': 's-plane pole map' });
      this.tip = el('div', { class: 'tooltip', hidden: '' });
      const box = el('div', { class: 'plot-box splane-box' }, this.canvas, this.tip);
      wrap.append(el('div', { class: 'plot-head' }, el('span', { class: 'plot-title', text: opts.title || 's-plane' }), this.legend), box);
      this.box = box;
      this.data = null;
      this.drag = null;
      this.frozen = null;
      this.hover = null;
      this.onDrag = null;
      new ResizeObserver(() => this.draw()).observe(box);
      box.addEventListener('pointerdown', (e) => this._down(e));
      box.addEventListener('pointermove', (e) => this._move(e));
      box.addEventListener('pointerup', (e) => this._up(e));
      box.addEventListener('pointercancel', (e) => this._up(e));
      box.addEventListener('pointerleave', () => { if (!this.drag) { this.hover = null; this.draw(); } });
    }

    // data: { markers: [{re, im, kind, label, dragId}], zetaRay, wnCircle, wnMax }
    setData(data) {
      this.data = data;
      const kinds = [...new Set(data.markers.map((mk) => mk.kind))];
      const names = { ol: 'open-loop pole', cl: 'closed-loop pole', zero: 'closed-loop zero', target: 'target pole' };
      this.legend.replaceChildren(...kinds.map((k) => {
        const key = el('span', { class: `marker-key mk-${k}` });
        return el('span', { class: 'legend-item' }, key, el('span', { text: names[k] }));
      }));
      this.draw();
    }

    _bounds(w, h) {
      if (this.frozen) return this.frozen;
      const d = this.data;
      let R = 1;
      for (const mk of d.markers) R = Math.max(R, Math.abs(mk.re), Math.abs(mk.im));
      if (d.wnCircle) R = Math.max(R, d.wnCircle);
      if (d.wnMax && d.wnMax < 4 * R) R = Math.max(R, d.wnMax);
      R *= 1.25;
      const maxRe = Math.max(0, ...d.markers.map((mk) => mk.re));
      const reHi = Math.max(R * 0.25, maxRe * 1.3);
      const reLo = -R;
      // equal scale: pixels per unit is limited by the tighter direction
      const m = 12;
      const scale = Math.min((w - 2 * m) / (reHi - reLo), (h - 2 * m) / (2 * R));
      const cx = m + (-reLo) * scale + ((w - 2 * m) - (reHi - reLo) * scale) / 2;
      return { scale, cx, cy: h / 2 };
    }

    _toCanvas(b, re, im) { return [b.cx + re * b.scale, b.cy - im * b.scale]; }
    _fromCanvas(b, x, y) { return [(x - b.cx) / b.scale, (b.cy - y) / b.scale]; }

    _hit(e) {
      const rect = this.canvas.getBoundingClientRect();
      const x = e.clientX - rect.left, y = e.clientY - rect.top;
      const b = this._bounds(rect.width, rect.height);
      let best = null, bestD = 16; // hit radius larger than the 8px marker
      for (const mk of this.data.markers) {
        const [px, py] = this._toCanvas(b, mk.re, mk.im);
        const dd = Math.hypot(px - x, py - y);
        if (dd < bestD) { bestD = dd; best = mk; }
      }
      return { mk: best, x, y, b };
    }

    _down(e) {
      if (!this.data) return;
      const { mk, b } = this._hit(e);
      if (mk && mk.dragId !== undefined && this.onDrag) {
        this.drag = mk.dragId;
        this.frozen = b;
        this.box.setPointerCapture(e.pointerId);
        this.box.classList.add('dragging');
        e.preventDefault();
      }
    }

    _move(e) {
      if (!this.data) return;
      if (this.drag !== null) {
        const rect = this.canvas.getBoundingClientRect();
        const [re, im] = this._fromCanvas(this.frozen, e.clientX - rect.left, e.clientY - rect.top);
        this.onDrag(this.drag, re, im);
        return;
      }
      const { mk } = this._hit(e);
      this.hover = mk;
      this.box.classList.toggle('can-drag', !!(mk && mk.dragId !== undefined && this.onDrag));
      this.draw();
    }

    _up(e) {
      if (this.drag === null) return;
      this.drag = null;
      this.frozen = null;
      this.box.classList.remove('dragging');
      try { this.box.releasePointerCapture(e.pointerId); } catch (err) { /* already released */ }
      this.draw();
    }

    draw() {
      if (!this.data) return;
      const { ctx, w, h } = setupCanvas(this.canvas);
      const b = this._bounds(w, h);
      const P = (re, im) => this._toCanvas(b, re, im);
      const d = this.data;

      // saturation-limited region: everything outside the omega_n,max circle
      if (d.wnMax && isFinite(d.wnMax)) {
        const [ox, oy] = P(0, 0);
        ctx.save();
        ctx.beginPath(); ctx.rect(0, 0, w, h);
        ctx.arc(ox, oy, d.wnMax * b.scale, 0, 2 * Math.PI, true);
        ctx.fillStyle = css('--critical'); ctx.globalAlpha = 0.08; ctx.fill('evenodd');
        ctx.globalAlpha = 1;
        ctx.beginPath(); ctx.arc(ox, oy, d.wnMax * b.scale, 0, 2 * Math.PI);
        ctx.strokeStyle = css('--critical'); ctx.setLineDash([5, 4]); ctx.lineWidth = 1; ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = css('--critical'); ctx.font = '11px var(--font-sans, system-ui)';
        ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
        const lx = ox - d.wnMax * b.scale * 0.71, ly = oy - d.wnMax * b.scale * 0.71;
        ctx.fillText('ωₙ,max (saturation)', Math.max(4, lx - 40), Math.max(14, ly - 4));
        ctx.restore();
      }

      // axes + ticks
      ctx.strokeStyle = css('--axis'); ctx.lineWidth = 1;
      const [ox, oy] = P(0, 0);
      ctx.beginPath(); ctx.moveTo(0, Math.round(oy) + 0.5); ctx.lineTo(w, Math.round(oy) + 0.5);
      ctx.moveTo(Math.round(ox) + 0.5, 0); ctx.lineTo(Math.round(ox) + 0.5, h); ctx.stroke();
      ctx.fillStyle = css('--text-muted'); ctx.font = '10px var(--font-sans, system-ui)';
      const [reMin] = this._fromCanvas(b, 0, 0), [reMax] = this._fromCanvas(b, w, 0);
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      for (const v of niceTicks(reMin, reMax, 6)) {
        if (v === 0) continue;
        const [x] = P(v, 0);
        ctx.beginPath(); ctx.moveTo(x, oy - 3); ctx.lineTo(x, oy + 3); ctx.stroke();
        ctx.fillText(fmt(v, 3), x, oy + 5);
      }
      const imMax = this._fromCanvas(b, 0, 0)[1];
      ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      for (const v of niceTicks(-imMax, imMax, 6)) {
        if (v === 0) continue;
        const [, y] = P(0, v);
        ctx.beginPath(); ctx.moveTo(ox - 3, y); ctx.lineTo(ox + 3, y); ctx.stroke();
        ctx.fillText(fmt(v, 3) + 'j', ox + 5, y);
      }
      ctx.textAlign = 'right'; ctx.textBaseline = 'top';
      ctx.fillText('Re', w - 4, oy + 16);
      ctx.textAlign = 'left';
      ctx.fillText('Im', ox + 5, 4);

      // design overlays: constant-zeta rays and target omega_n circle
      ctx.strokeStyle = css('--text-muted'); ctx.setLineDash([3, 4]);
      if (d.zetaRay && d.zetaRay < 1) {
        const L = 2 * Math.max(w, h) / b.scale;
        const sw = Math.sqrt(1 - d.zetaRay ** 2);
        for (const sgn of [1, -1]) {
          const [x2, y2] = P(-d.zetaRay * L, sgn * sw * L);
          ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(x2, y2); ctx.stroke();
        }
        ctx.setLineDash([]); ctx.fillStyle = css('--text-muted'); ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
        const [lx, ly] = P(-d.zetaRay * (d.wnCircle || 1) * 1.15, sw * (d.wnCircle || 1) * 1.15);
        ctx.fillText(`ζ = ${fmt(d.zetaRay, 3)}`, lx + 4, ly);
        ctx.setLineDash([3, 4]);
      }
      if (d.wnCircle) {
        ctx.beginPath(); ctx.arc(ox, oy, d.wnCircle * b.scale, Math.PI / 2, 1.5 * Math.PI); ctx.stroke();
      }
      ctx.setLineDash([]);

      // markers
      for (const mk of d.markers) {
        const [x, y] = P(mk.re, mk.im);
        const hov = this.hover === mk || (this.drag !== null && mk.dragId === this.drag);
        const r = hov ? 7 : 5.5;
        ctx.lineWidth = mk.kind === 'target' ? 1.5 : 2.5;
        ctx.strokeStyle = css({ ol: '--text-muted', cl: '--series-1', zero: '--series-2', target: '--text-secondary' }[mk.kind]);
        if (mk.kind === 'zero') {
          ctx.fillStyle = css('--surface');
          ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        } else if (mk.kind === 'target') {
          ctx.setLineDash([2, 2]);
          ctx.beginPath(); ctx.arc(x, y, r + 3, 0, 2 * Math.PI); ctx.stroke();
          ctx.setLineDash([]);
        } else {
          ctx.beginPath(); ctx.moveTo(x - r, y - r); ctx.lineTo(x + r, y + r);
          ctx.moveTo(x + r, y - r); ctx.lineTo(x - r, y + r); ctx.stroke();
        }
        if (mk.dragId !== undefined && this.onDrag) {
          ctx.strokeStyle = css('--series-1'); ctx.globalAlpha = hov ? 0.5 : 0.22; ctx.lineWidth = 1;
          ctx.beginPath(); ctx.arc(x, y, 12, 0, 2 * Math.PI); ctx.stroke(); ctx.globalAlpha = 1;
        }
      }

      const show = this.drag !== null ? d.markers.find((mk) => mk.dragId === this.drag) : this.hover;
      if (show) {
        const [x, y] = P(show.re, show.im);
        const val = Math.abs(show.im) < 1e-9 ? fmt(show.re, 4) : `${fmt(show.re, 4)} ${show.im >= 0 ? '+' : '−'} ${fmt(Math.abs(show.im), 4)}j`;
        this.tip.replaceChildren(el('div', { class: 'tip-row' }, el('strong', { text: val }), el('span', { class: 'tip-label', text: show.label })));
        this.tip.hidden = false;
        const tw = this.tip.offsetWidth;
        this.tip.style.left = (x + 14 + tw > w ? x - 14 - tw : x + 14) + 'px';
        this.tip.style.top = Math.max(4, Math.min(h - 40, y - 14)) + 'px';
      } else {
        this.tip.hidden = true;
      }
    }
  }

  return { TimePlot, SPlane, css, setupCanvas };
})();
