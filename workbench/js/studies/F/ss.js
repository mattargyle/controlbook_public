// Study F, Chapters 11–14 (F.11–F.14): full state feedback on the decoupled
// longitudinal (h, ḣ; input F̃) and lateral (z, θ, ż, θ̇; input τ) models,
// integral augmentation, observers, and disturbance observers.
//
// Lateral observer: the outputs are z and θ, so L is 4×2 and not unique. The
// workbench uses the block structure L = [[L_z1, 0], [0, L_θ1], [L_z2, 0], [0, L_θ2]]
// (θ innovation feeds θ̂, z innovation feeds ẑ). Then A − LC is block triangular
// and eig(A − LC) = eig(z block) ∪ eig(θ block): two SISO designs.
(function () {
  const { el, slider, segmented, section } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const F = WB.F;
  const T = WB.tf;
  const { tex, texMat, texPole, fmt, fmtPole } = M;
  const PD = () => WB.pd;
  const D = () => WB.design;

  // ----------------------------------------------------------- models --
  function sub(p) {
    const m = WB.systems.F.models(p);
    return {
      m,
      lon: m.lonSS,
      lat: m.latSS,
      Cz: [[1, 0, 0, 0]],
      // observer sub-blocks
      zBlk: { A: [[0, 1], [0, -m.a]], C: [[1, 0]] },
      tBlk: { A: [[0, 1], [0, 0]], C: [[1, 0]] },
      // disturbance-observer blocks: (h, ḣ, d_F), (z, ż, d_z), (θ, θ̇, d_τ)
      hD: { A: [[0, 1, 0], [0, 0, 1 / m.M], [0, 0, 0]], C: [[1, 0, 0]] },
      zD: { A: [[0, 1, 0], [0, -m.a, 1 / m.M], [0, 0, 0]], C: [[1, 0, 0]] },
      tD: { A: [[0, 1, 0], [0, 0, 1 / m.J], [0, 0, 0]], C: [[1, 0, 0]] },
    };
  }
  const augI = (A, B, Cr) => D().augmentIntegrator(A, B, Cr);
  const kr = (A, B, C, K) => D().refGain(A, B, C, K);

  // Desired poles from the knobs: lon pair; lateral = outer pair ∪ inner pair (F.11a).
  function ctrlPoles(k) {
    const wnh = 2.2 / k.trh, wnt = 2.2 / k.trth, wnz = 2.2 / (k.Msep * k.trth);
    return { lon: F.polesWZ(wnh, k.zetah), inner: F.polesWZ(wnt, k.zetath), outer: F.polesWZ(wnz, k.zetaz), wnh, wnt, wnz };
  }

  // Gains for a level: 'sf' (F.11), 'sfi' (F.12), 'obs' (F.13), 'dobs' (F.14).
  function design(p, k, level) {
    const S = sub(p);
    const P = ctrlPoles(k);
    const out = { poles: P };
    const latP = [...P.outer, ...P.inner];
    if (level === 'sf') {
      out.Kh = D().place(S.lon.A, S.lon.B, P.lon) || [NaN, NaN];
      out.Kz = D().place(S.lat.A, S.lat.B, latP) || [NaN, NaN, NaN, NaN];
      out.krh = kr(S.lon.A, S.lon.B, S.lon.C, out.Kh);
      out.krz = kr(S.lat.A, S.lat.B, S.Cz, out.Kz);
    } else {
      const ah = augI(S.lon.A, S.lon.B, S.lon.C), az = augI(S.lat.A, S.lat.B, S.Cz);
      const K1 = D().place(ah.A1, ah.B1, [...P.lon, { re: k.pIh, im: 0 }]) || [NaN, NaN, NaN];
      const K2 = D().place(az.A1, az.B1, [...latP, { re: k.pIz, im: 0 }]) || [NaN, NaN, NaN, NaN, NaN];
      out.Kh = K1.slice(0, 2); out.kIh = K1[2];
      out.Kz = K2.slice(0, 4); out.kIz = K2[4];
    }
    if (level === 'obs' || level === 'dobs') {
      const f = k.obsFactor, zo = k.zetaObs;
      const oh = F.polesWZ(f * P.wnh, zo), oz = F.polesWZ(f * P.wnz, zo), ot = F.polesWZ(f * P.wnt, zo);
      out.obsPoles = { lon: oh, outer: oz, inner: ot };
      if (level === 'obs') {
        out.Lh = col(D().observerGain(S.lon.A, S.lon.C, oh), 2);
        out.Lz = col(D().observerGain(S.zBlk.A, S.zBlk.C, oz), 2);
        out.Lt = col(D().observerGain(S.tBlk.A, S.tBlk.C, ot), 2);
      } else {
        out.obsPoles = { lon: [...oh, { re: k.pDh, im: 0 }], outer: [...oz, { re: k.pDz, im: 0 }], inner: [...ot, { re: k.pDth, im: 0 }] };
        out.Lh = col(D().observerGain(S.hD.A, S.hD.C, out.obsPoles.lon), 3);
        out.Lz = col(D().observerGain(S.zD.A, S.zD.C, out.obsPoles.outer), 3);
        out.Lt = col(D().observerGain(S.tD.A, S.tD.C, out.obsPoles.inner), 3);
      }
    }
    return out;
  }
  const col = (Lc, n) => (Lc ? Lc.map((r) => r[0]) : new Array(n).fill(NaN));
  // Full lateral L (4×2) from the block gains
  const latL = (Lz, Lt) => [[Lz[0], 0], [0, Lt[0]], [Lz[1], 0], [0, Lt[1]]];

  // ------------------------------------------------------- controller --
  function makeSS(ctx, { linear = false } = {}) {
    const s = ctx.sys, p = ctx.pModel, st = ctx.st, g = ctx.gains, level = ctx.chapter.level, Ts = ctx.S.sim.Ts;
    const S = sub(p), m = S.m;
    const [lo, hi] = s.uLimit(p)[0];
    const useObs = level === 'obs' || level === 'dobs';
    const useDO = level === 'dobs' && st.dobs !== false;
    const integ = level !== 'sf';
    let Ih = 0, Iz = 0, ehPrev = 0, ezPrev = 0;
    let xh = [0, 0, 0], xl = [0, 0, 0, 0, 0, 0];   // (h, ḣ, d_F), (z, θ, ż, θ̇, d_z, d_τ)
    let uPrev = null;
    const Lh = g.Lh || [0, 0, 0], Lz = g.Lz || [0, 0, 0], Lt = g.Lt || [0, 0, 0];
    const dot = (a, b) => a.reduce((acc, v, i) => acc + v * b[i], 0);

    function obsStep(y) {
      // previous input as applied: saturated rotor forces → F̃, τ
      let Ft = 0, tau = 0;
      if (uPrev) {
        const u = linear ? uPrev : uPrev.map((v) => Math.max(lo, Math.min(hi, v)));
        Ft = u[0] + u[1] - (linear ? 0 : m.Fe);
        tau = p.d * (u[0] - u[1]);
      }
      const fh = (x) => {
        const e = y[1] - x[0];
        return [x[1] + Lh[0] * e, (Ft + (useDO ? x[2] : 0)) / m.M + Lh[1] * e, useDO ? Lh[2] * e : 0];
      };
      const fl = (x) => {
        const ez = y[0] - x[0], et = y[2] - x[1];
        return [
          x[2] + Lz[0] * ez,
          x[3] + Lt[0] * et,
          -p.g * x[1] - m.a * x[2] + (useDO ? x[4] / m.M : 0) + Lz[1] * ez,
          (tau + (useDO ? x[5] : 0)) / m.J + Lt[1] * et,
          useDO ? Lz[2] * ez : 0,
          useDO ? Lt[2] * et : 0,
        ];
      };
      xh = M.rk4Step(fh, xh, 0, Ts);
      xl = M.rk4Step(fl, xl, 0, Ts);
    }

    return {
      update(r, x, y) {
        let xlon, xlat;
        if (useObs) {
          obsStep(y);
          xlon = [xh[0], xh[1]]; xlat = xl.slice(0, 4);
        } else {
          xlon = [x[1], x[4]]; xlat = [x[0], x[2], x[3], x[5]];
        }
        const compute = (IhT, IzT) => {
          let Ft, tau;
          if (integ) {
            Ft = -dot(g.Kh, xlon) - g.kIh * IhT;
            tau = -dot(g.Kz, xlat) - g.kIz * IzT;
          } else {
            Ft = -dot(g.Kh, xlon) + g.krh * r[0];
            tau = -dot(g.Kz, xlat) + g.krz * r[1];
          }
          if (useDO) { Ft -= xh[2]; tau -= xl[5]; }
          const Fc = linear ? Ft : m.Fe + Ft;
          return { u: s.mix(Fc, tau, p), Ft, tau };
        };
        let out;
        if (integ) {
          const eh = r[0] - xlon[0], ez = r[1] - xlat[0];
          const IhT = Ih + (Ts / 2) * (eh + ehPrev), IzT = Iz + (Ts / 2) * (ez + ezPrev);
          out = compute(IhT, IzT);
          const sat = !linear && out.u.some((v) => v > hi || v < lo);
          if (st.antiwindup === 'clamp' && sat) out = compute(Ih, Iz);   // hold the integrators (F.12a)
          else { Ih = IhT; Iz = IzT; }
          ehPrev = eh; ezPrev = ez;
        } else {
          out = compute(0, 0);
        }
        uPrev = out.u;
        return {
          u: out.u, Ftilde: out.Ft, tau: out.tau, intH: Ih, intZ: Iz,
          ...(useObs ? { zhat: xl[0], hhat: xh[0], thhat: xl[1], zdhat: xl[2], hdhat: xh[1], thdhat: xl[3], dFhat: xh[2], dzhat: xl[4], dthat: xl[5] } : {}),
        };
      },
    };
  }

  // ------------------------------------------------------------ poles --
  function clPoles(ctx, level, g = ctx.gains) {
    const S = sub(ctx.pModel);
    if (level === 'sf') {
      return {
        lon: L.eig(L.sub(S.lon.A, L.mul(S.lon.B, [g.Kh]))),
        lat: L.eig(L.sub(S.lat.A, L.mul(S.lat.B, [g.Kz]))),
      };
    }
    const ah = augI(S.lon.A, S.lon.B, S.lon.C), az = augI(S.lat.A, S.lat.B, S.Cz);
    return {
      lon: L.eig(L.sub(ah.A1, L.mul(ah.B1, [[...g.Kh, g.kIh]]))),
      lat: L.eig(L.sub(az.A1, L.mul(az.B1, [[...g.Kz, g.kIz]]))),
    };
  }
  function obsPolesOf(ctx, level, g = ctx.gains) {
    const S = sub(ctx.pModel);
    if (level === 'obs') {
      return {
        lon: L.eig(L.sub(S.lon.A, L.mul(L.col(g.Lh.slice(0, 2)), S.lon.C))),
        lat: L.eig(L.sub(S.lat.A, L.mul(latL(g.Lz, g.Lt), S.lat.C))),
      };
    }
    if (level === 'dobs') {
      const blk = (B, Lv) => L.eig(L.sub(B.A, L.mul(L.col(Lv), B.C)));
      return { lon: blk(S.hD, g.Lh), lat: [...blk(S.zD, g.Lz), ...blk(S.tD, g.Lt)] };
    }
    return { lon: [], lat: [] };
  }

  // ----------------------------------------------------------- controls --
  const KEYS = {
    sf: { lon: ['Kh1', 'Kh2', 'krh'], lat: ['Kz1', 'Kz2', 'Kz3', 'Kz4', 'krz'] },
    sfi: { lon: ['Kh1', 'Kh2', 'kIh'], lat: ['Kz1', 'Kz2', 'Kz3', 'Kz4', 'kIz'] },
  };
  const SPEC = {
    Kh1: ['K<sub>h,1</sub> (h)', 0, 2], Kh2: ['K<sub>h,2</sub> (ḣ)', 0, 3], krh: ['k<sub>r,h</sub>', 0, 2], kIh: ['k<sub>I,h</sub>', -1, 0],
    Kz1: ['K<sub>z,1</sub> (z)', -0.5, 0.5], Kz2: ['K<sub>z,2</sub> (θ)', 0, 2], Kz3: ['K<sub>z,3</sub> (ż)', -1, 1], Kz4: ['K<sub>z,4</sub> (θ̇)', 0, 1], krz: ['k<sub>r,z</sub>', -0.5, 0.5], kIz: ['k<sub>I,z</sub>', -0.5, 0.5],
    Lh1: ['L<sub>h,1</sub>', 0, 20], Lh2: ['L<sub>h,2</sub>', 0, 100], Ldh: ['L<sub>d,h</sub>', 0, 200],
    Lz1: ['L<sub>z,1</sub>', 0, 20], Lz2: ['L<sub>z,2</sub>', -50, 50], Ldz: ['L<sub>d,z</sub>', -200, 200],
    Lt1: ['L<sub>θ,1</sub>', 0, 200], Lt2: ['L<sub>θ,2</sub>', 0, 5000], Ldt: ['L<sub>d,θ</sub>', 0, 2000],
  };
  function wSliders(parent, ctx, keys) {
    for (const key of keys) {
      const [label, min, max] = SPEC[key];
      slider(parent, { label, min, max, step: (max - min) / 4000, sig: 4, get: () => ctx.st.w[key], set: (v) => { ctx.st.w[key] = v; ctx.update(); } });
    }
  }
  // Work-mode gains (flat st.w) → gain object
  function fromW(w, level) {
    const g = { Kh: [w.Kh1, w.Kh2], Kz: [w.Kz1, w.Kz2, w.Kz3, w.Kz4], krh: w.krh, krz: w.krz, kIh: w.kIh, kIz: w.kIz };
    if (level === 'obs') { g.Lh = [w.Lh1, w.Lh2, 0]; g.Lz = [w.Lz1, w.Lz2, 0]; g.Lt = [w.Lt1, w.Lt2, 0]; }
    if (level === 'dobs') { g.Lh = [w.Lh1, w.Lh2, w.Ldh]; g.Lz = [w.Lz1, w.Lz2, w.Ldz]; g.Lt = [w.Lt1, w.Lt2, w.Ldt]; }
    return g;
  }
  function toW(g) {
    const w = { Kh1: g.Kh[0], Kh2: g.Kh[1], Kz1: g.Kz[0], Kz2: g.Kz[1], Kz3: g.Kz[2], Kz4: g.Kz[3] };
    for (const k of ['krh', 'krz', 'kIh', 'kIz']) if (g[k] !== undefined) w[k] = g[k];
    if (g.Lh) Object.assign(w, { Lh1: g.Lh[0], Lh2: g.Lh[1], Lz1: g.Lz[0], Lz2: g.Lz[1], Lt1: g.Lt[0], Lt2: g.Lt[1] });
    if (g.Lh && g.Lh.length > 2) Object.assign(w, { Ldh: g.Lh[2], Ldz: g.Lz[2], Ldt: g.Lt[2] });
    return w;
  }
  // Work-mode starting gains: a deliberately slow design (t_r,h = 16 s, M = 20,
  // t_r,θ = 1.2 s, ζ = 0.9, observers 3× faster): stable, but not the answer.
  function slowW(sys, level) {
    const p = { ...Object.fromEntries(sys.params.map((q) => [q.key, q.value])), ...sys.constants };
    const k = { trh: 16, zetah: 0.9, trth: 1.2, zetath: 0.9, Msep: 20, zetaz: 0.9, pIh: -0.05, pIz: -0.03, obsFactor: 3, zetaObs: 0.9, pDh: -0.5, pDz: -0.5, pDth: -3 };
    return { ...toW(design(p, k, level)), ...(level === 'sf' ? {} : { krh: 0, krz: 0 }) };
  }
  function knobs(sys, extra = {}) {
    const pr = sys.problems.ch8;
    return { trh: pr.trh, zetah: pr.zetah, trth: pr.trth, zetath: pr.zetath, Msep: pr.Msep, zetaz: pr.zetaz,
      pIh: sys.problems.ch12.pIh, pIz: sys.problems.ch12.pIz, obsFactor: sys.problems.ch13.obsFactor, zetaObs: 0.707,
      pDh: sys.problems.ch14.pDh, pDz: sys.problems.ch14.pDz, pDth: sys.problems.ch14.pDth, ...extra };
  }
  function knobControls(parent, ctx, level, title) {
    const k = ctx.st.k, set = (key) => (v) => { k[key] = v; ctx.update(); };
    const s1 = section(parent, `${title}: controller poles`, 'p. 113 · Eq. 8.5, F.11(a) p. 399');
    slider(s1, { label: 't<sub>r,h</sub>', unit: 's', min: 0.5, max: 20, step: 0.01, sig: 3, get: () => k.trh, set: set('trh') });
    slider(s1, { label: 'ζ<sub>h</sub>', min: 0.3, max: 1.5, step: 0.005, sig: 3, get: () => k.zetah, set: set('zetah') });
    slider(s1, { label: 't<sub>r,θ</sub>', unit: 's', min: 0.1, max: 3, step: 0.005, sig: 3, get: () => k.trth, set: set('trth') });
    slider(s1, { label: 'ζ<sub>θ</sub>', min: 0.3, max: 1.5, step: 0.005, sig: 3, get: () => k.zetath, set: set('zetath') });
    slider(s1, { label: 'M = t<sub>r,z</sub>/t<sub>r,θ</sub>', min: 1, max: 30, step: 0.1, sig: 3, get: () => k.Msep, set: set('Msep') });
    slider(s1, { label: 'ζ<sub>z</sub>', min: 0.3, max: 1.5, step: 0.005, sig: 3, get: () => k.zetaz, set: set('zetaz') });
    if (level !== 'sf') {
      slider(s1, { label: 'p<sub>I,h</sub>', min: -3, max: -0.01, step: 0.001, sig: 3, get: () => k.pIh, set: set('pIh') });
      slider(s1, { label: 'p<sub>I,z</sub>', min: -3, max: -0.01, step: 0.001, sig: 3, get: () => k.pIz, set: set('pIz') });
    }
    if (level === 'obs' || level === 'dobs') {
      const s2 = section(parent, `${title}: observer poles`, 'p. 222');
      slider(s2, { label: 'ω<sub>n,obs</sub> / ω<sub>n,ctrl</sub>', min: 1, max: 30, step: 0.1, sig: 3, get: () => k.obsFactor, set: set('obsFactor'), hint: 'each observer pair is this many times faster than its controller pair' });
      slider(s2, { label: 'ζ<sub>obs</sub>', min: 0.3, max: 1.5, step: 0.005, sig: 3, get: () => k.zetaObs, set: set('zetaObs') });
      if (level === 'dobs') {
        slider(s2, { label: 'p<sub>d,h</sub>', min: -20, max: -0.05, step: 0.01, sig: 3, get: () => k.pDh, set: set('pDh') });
        slider(s2, { label: 'p<sub>d,z</sub>', min: -20, max: -0.05, step: 0.01, sig: 3, get: () => k.pDz, set: set('pDz') });
        slider(s2, { label: 'p<sub>d,θ</sub>', min: -60, max: -0.1, step: 0.01, sig: 3, get: () => k.pDth, set: set('pDth') });
      }
    }
  }
  function gainReadout(parent, ctx, level) {
    const box = el('div', { class: 'readout wrap' });
    parent.append(box);
    WB.ui.addRefresher(() => {
      const w = toW(ctx.gains);
      const keys = [...KEYS[level === 'sf' ? 'sf' : 'sfi'].lon, ...KEYS[level === 'sf' ? 'sf' : 'sfi'].lat,
        ...(level === 'obs' || level === 'dobs' ? ['Lh1', 'Lh2', 'Lz1', 'Lz2', 'Lt1', 'Lt2'] : []), ...(level === 'dobs' ? ['Ldh', 'Ldz', 'Ldt'] : [])];
      box.replaceChildren(...keys.map((k) => el('div', {}, el('span', { class: 'ro-label', text: k }), el('strong', { text: fmt(w[k], 4) }))));
    });
  }

  // ------------------------------------------------------------- s-plane --
  function splane(ctx) {
    const view = ctx.st.view === 'lon' ? 'lon' : 'lat';
    const S = sub(ctx.pModel);
    const cl = clPoles(ctx, this.level), ob = obsPolesOf(ctx, this.level);
    const mk = [];
    const ol = view === 'lon' ? L.eig(S.lon.A) : L.eig(S.lat.A);
    ol.forEach((q) => mk.push({ ...q, kind: 'ol', label: 'open-loop pole' }));
    cl[view].forEach((q) => mk.push({ ...q, kind: 'cl', label: `controller pole (${view === 'lon' ? 'altitude' : 'lateral'})` }));
    ob[view].forEach((q) => mk.push({ ...q, kind: 'obs', label: 'observer pole', noFit: Math.hypot(q.re, q.im) > 8 }));
    if (ctx.S.mode === 'work') {
      const P = ctrlPoles(ctx.st.k);
      (view === 'lon' ? P.lon : [...P.outer, ...P.inner]).forEach((q) => mk.push({ ...q, kind: 'target', label: 'target pole (spec)' }));
    }
    return { markers: mk, zetaRay: view === 'lon' ? ctx.st.k.zetah : ctx.st.k.zetaz, minR: 0.05 };
  }

  // ----------------------------------------------------------- math cards --
  function ssCards(ctx) {
    const S = sub(ctx.pModel);
    return [
      { title: 'Decoupled models (F.6)', page: 'F.6 p. 396',
        theory: 'x_{lon} = (h, \\dot h),\\; u = \\tilde F;\\quad x_{lat} = (z, \\theta, \\dot z, \\dot\\theta),\\; u = \\tilde\\tau',
        numbers: `A_{lon} = ${texMat(S.lon.A)},\\; B_{lon} = ${texMat(S.lon.B)},\\quad A_{lat} = ${texMat(S.lat.A)},\\; B_{lat} = ${texMat(S.lat.B)}`, spoiler: true },
    ];
  }
  function ctrbCard(A, B, title, page) {
    const Cm = L.ctrb(A, B);
    return { title, page, theory: '\\mathcal{C}_{A,B} = \\begin{bmatrix}B & AB & \\cdots & A^{n-1}B\\end{bmatrix}',
      numbers: `\\mathcal{C} = ${texMat(Cm, 3)},\\quad \\operatorname{rank} = ${L.rank(Cm)}`, spoiler: true };
  }
  function polesCard(ctx, d, level) {
    const P = d.poles;
    const lat = [...P.outer, ...P.inner];
    return {
      title: 'Desired closed-loop poles', page: 'F.11(a) p. 399',
      theory: '\\Delta^d_{lon} = s^2 + 2\\zeta_h\\omega_{n_h}s + \\omega_{n_h}^2,\\quad \\Delta^d_{lat} = (s^2 + 2\\zeta_z\\omega_{n_z}s + \\omega_{n_z}^2)(s^2 + 2\\zeta_\\theta\\omega_{n_\\theta}s + \\omega_{n_\\theta}^2)' + (level === 'sf' ? '' : '\\,(s - p_I)'),
      numbers: `p_{lon} = ${P.lon.map((q) => texPole(q)).join(',\\;')},\\quad p_{lat} = ${lat.map((q) => texPole(q)).join(',\\;')}`, spoiler: true,
      note: 'The F.8 pairs: the outer pair is the slow lateral mode, the inner pair the fast roll mode.',
    };
  }

  // -------------------------------------------------------- chapter base --
  function base(level, num, title, pages, extra) {
    return F.register(Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages, level,
      controller(ctx, o) { return makeSS(ctx, o); },
      gains(ctx) { return ctx.S.mode === 'explore' ? design(ctx.pModel, ctx.st.k, level) : fromW(ctx.st.w, level); },
      splane,
      targets(ctx) { return { tr: ctx.st.k.trh }; },
    }, extra));
  }
  function viewSeg(parent, ctx) {
    segmented(parent, { label: 's-plane shows', options: [{ value: 'lon', label: 'altitude' }, { value: 'lat', label: 'lateral' }], get: () => ctx.st.view, set: (v) => { ctx.st.view = v; ctx.update(); } });
  }
  function workControls(parent, ctx, level) {
    const keys = KEYS[level === 'sf' ? 'sf' : 'sfi'];
    const s1 = section(parent, 'Altitude gains', 'p. 173');
    wSliders(s1, ctx, keys.lon);
    const s2 = section(parent, 'Lateral gains', 'p. 173');
    wSliders(s2, ctx, keys.lat);
    if (level === 'obs' || level === 'dobs') {
      const s3 = section(parent, 'Observer gains', 'p. 216 · Eq. 13.3');
      wSliders(s3, ctx, level === 'dobs' ? ['Lh1', 'Lh2', 'Ldh', 'Lz1', 'Lz2', 'Ldz', 'Lt1', 'Lt2', 'Ldt'] : ['Lh1', 'Lh2', 'Lz1', 'Lz2', 'Lt1', 'Lt2']);
      s3.append(el('p', { class: 'muted small', text: 'Lateral L = [[L_z1, 0], [0, L_θ1], [L_z2, 0], [0, L_θ2]]: the z innovation corrects ẑ and ż̂, the θ innovation corrects θ̂ and θ̇̂.' }));
    }
  }

  // Satisfies "ζ ≥ ζ_spec and ω_n ≥ ω_n,spec" (F.11a) for every pole.
  function meetsSpec(poles, wn, zeta) {
    return poles.every((q) => {
      const w = Math.hypot(q.re, q.im);
      const z = w > 0 ? -q.re / w : 1;
      return w >= wn * 0.99 && z >= zeta - 0.005;
    });
  }

  // ------------------------------------------------------------ Chapter 11 --
  base('sf', 11, 'Full state feedback', 'pp. 173–196, F.11 p. 399', {
    defaults(sys) { return { view: 'lat', zOff: 3, hOff: 0, k: knobs(sys), w: slowW(sys, 'sf') }; },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'u = −Kx + k_r r (per loop)', 'p. 183 · Eq. 11.38');
      viewSeg(sec, ctx);
      sec.append(el('p', { class: 'muted small', text: 'F = F_e − K_h(h, ḣ) + k_r,h h_r;  τ = −K_z(z, θ, ż, θ̇) + k_r,z z_r. The true state is fed back.' }));
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'sf'); knobControls(parent, ctx, 'sf', 'Specs (target rings)'); }
      else { knobControls(parent, ctx, 'sf', 'Design'); gainReadout(section(parent, 'Gains', 'p. 182'), ctx, 'sf'); }
    },
    math(ctx) {
      const S = sub(ctx.pModel), d = design(ctx.pModel, ctx.st.k, 'sf');
      return [
        ...ssCards(ctx),
        ctrbCard(S.lon.A, S.lon.B, 'Controllability (altitude)', 'p. 180, F.11(c)'),
        ctrbCard(S.lat.A, S.lat.B, 'Controllability (lateral)', 'p. 180, F.11(c)'),
        polesCard(ctx, d, 'sf'),
        { title: 'Gains (place / Ackermann)', page: 'p. 182 · Eq. 11.32, 11.35',
          theory: 'K = \\text{place}(A, B, p),\\quad k_r = \\frac{-1}{C_r(A - BK)^{-1}B}',
          numbers: `K_h = ${texMat([d.Kh])},\\; k_{r,h} = ${tex(d.krh)},\\quad K_z = ${texMat([d.Kz])},\\; k_{r,z} = ${tex(d.krz)}`, spoiler: true,
          symbolic: '\\text{each loop has a free integrator, so } k_{r,h} = K_{h,1},\\; k_{r,z} = K_{z,1}' },
        { title: 'Tuning (F.11e)', page: 'p. 110–113',
          theory: 't_r \\approx \\frac{2.2}{\\omega_n}:\\; \\text{move poles farther from the origin to speed up};\\quad M_p = e^{-\\zeta\\pi/\\sqrt{1-\\zeta^2}}:\\; \\text{raise } \\zeta \\text{ (smaller angle from the real axis) to cut overshoot}' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch11;
      const S = () => sub(ctx.pModel);
      const P = () => ctrlPoles(knobs(ctx.sys));
      const ref = () => design(ctx.pModel, knobs(ctx.sys), 'sf');
      const num = (v) => PD().num(v);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'c', title: '(c) Controllability ranks', inputs: { rl: 'rank 𝒞 (altitude)', rz: 'rank 𝒞 (lateral)' },
          check: (v) => PD().checkNumbers(v, { rl: L.rank(L.ctrb(S().lon.A, S().lon.B)), rz: L.rank(L.ctrb(S().lat.A, S().lat.B)) }, {}),
          solution: () => [{ tex: `\\mathcal{C}_{lon} = ${texMat(L.ctrb(S().lon.A, S().lon.B))},\\quad \\mathcal{C}_{lat} = ${texMat(L.ctrb(S().lat.A, S().lat.B), 3)}` }, { html: 'Both full rank (2 and 4): each loop is controllable from its own input.' }],
        },
        {
          id: 'dh', title: '(d) Altitude: K<sub>h</sub> and k<sub>r,h</sub>',
          html: 'Any K whose poles have ζ ≥ ζ<sub>h</sub> = 0.707 and ω<sub>n</sub> ≥ ω<sub>n<sub>h</sub></sub> = 0.275 rad/s passes; k<sub>r,h</sub> must give unity DC gain for your K.',
          inputs: { K1: 'K<sub>h,1</sub>', K2: 'K<sub>h,2</sub>', kr: 'k<sub>r,h</sub>' },
          check: (v) => {
            const K = [num(v.K1), num(v.K2)], k = num(v.kr);
            if (K.some((x) => x === null) || k === null) return { ok: false, msg: 'Fill in K and k_r.' };
            const poles = L.eig(L.sub(S().lon.A, L.mul(S().lon.B, [K])));
            if (!meetsSpec(poles, P().wnh, prob.zetah || 0.707)) return { ok: false, msg: `Poles ${poles.map((q) => fmtPole(q)).join(', ')} miss the ζ/ωn spec.` };
            const want = kr(S().lon.A, S().lon.B, S().lon.C, K);
            return M.close(k, want) ? { ok: true, msg: `Poles ${poles.map((q) => fmtPole(q)).join(', ')}.` } : { ok: false, msg: 'K is fine; check k_r.' };
          },
          actions: [{ label: 'Use my gains', run: (v) => { const vals = [num(v.K1), num(v.K2), num(v.kr)]; if (vals.some((x) => x === null)) return { ok: false, msg: 'Fill in all three.' }; ctx.app.setMode('work'); [ctx.st.w.Kh1, ctx.st.w.Kh2, ctx.st.w.krh] = vals; ctx.update(); return null; } }],
          solution: () => { const r = ref(); return [
            { tex: `p = ${P().lon.map((q) => texPole(q, 4)).join(',\\;')}\\;(\\text{the F.8 pair}):\\quad K_h = ${texMat([r.Kh])},\\quad k_{r,h} = ${tex(r.krh)}` },
            { html: 'With x = (h, ḣ), −K<sub>h</sub>x + k<sub>r,h</sub>h<sub>r</sub> is exactly the F.8 PD law with derivative on h: K<sub>h</sub> = (k<sub>P<sub>h</sub></sub>, k<sub>D<sub>h</sub></sub>).' },
          ]; },
        },
        {
          id: 'dz', title: '(d) Lateral: K<sub>z</sub> and k<sub>r,z</sub>',
          html: 'Any K whose four poles have ζ ≥ ζ<sub>z</sub> = 0.707 and ω<sub>n</sub> ≥ ω<sub>n<sub>z</sub></sub> = 0.275 rad/s passes.',
          inputs: { K1: 'K<sub>z,1</sub>', K2: 'K<sub>z,2</sub>', K3: 'K<sub>z,3</sub>', K4: 'K<sub>z,4</sub>', kr: 'k<sub>r,z</sub>' },
          check: (v) => {
            const K = [num(v.K1), num(v.K2), num(v.K3), num(v.K4)], k = num(v.kr);
            if (K.some((x) => x === null) || k === null) return { ok: false, msg: 'Fill in K and k_r.' };
            const poles = L.eig(L.sub(S().lat.A, L.mul(S().lat.B, [K])));
            if (!meetsSpec(poles, P().wnz, 0.707)) return { ok: false, msg: `Poles ${poles.map((q) => fmtPole(q)).join(', ')} miss the ζ/ωn spec.` };
            const want = kr(S().lat.A, S().lat.B, S().Cz, K);
            return M.close(k, want) ? { ok: true, msg: `Poles ${poles.map((q) => fmtPole(q)).join(', ')}.` } : { ok: false, msg: 'K is fine; check k_r.' };
          },
          actions: [{ label: 'Use my gains', run: (v) => { const vals = ['K1', 'K2', 'K3', 'K4', 'kr'].map((k) => num(v[k])); if (vals.some((x) => x === null)) return { ok: false, msg: 'Fill in all five.' }; ctx.app.setMode('work'); [ctx.st.w.Kz1, ctx.st.w.Kz2, ctx.st.w.Kz3, ctx.st.w.Kz4, ctx.st.w.krz] = vals; ctx.update(); return null; } }],
          solution: () => { const r = ref(); return [
            { tex: `p = ${[...P().outer, ...P().inner].map((q) => texPole(q, 4)).join(',\\;')}` },
            { tex: `\\Delta^d = ${T.polyTex(L.polyFromRoots([...P().outer, ...P().inner]))}` },
            { tex: `K_z = ${texMat([r.Kz])},\\quad k_{r,z} = ${tex(r.krz)}` },
            { html: 'One reasonable choice: the F.8 outer and inner pairs (the problem allows anything faster and better damped). K<sub>z,1</sub> and K<sub>z,3</sub> are negative for the same reason k<sub>P<sub>z</sub></sub>, k<sub>D<sub>z</sub></sub> are.' },
          ]; },
        },
        {
          id: 'e', title: '(e) Faster? Less overshoot?',
          html: 'Rise time: move the dominant poles farther from the origin (larger ω<sub>n</sub>), e.g. smaller t<sub>r</sub> knobs. Overshoot: increase ζ (move the poles toward the real axis). Try both in Explore mode and watch the rotor limits.',
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 12 --
  // Tracking errors at t_end, or just before the last z_r switch for a square wave.
  function trackCheck(ctx, tol = 0.02) {
    const res = ctx.app.result(), S = ctx.S, zc = S.sim.refs[0];
    let t = S.sim.tEnd;
    if (zc.type === 'square') {
      const half = 0.5 / zc.frequency;
      const k = Math.floor((S.sim.tEnd - zc.tStep) / half - 1e-9);
      if (k >= 1) t = zc.tStep + k * half - 0.05;
    }
    const i = Math.max(0, Math.min(res.t.length - 1, Math.round(t / S.sim.Ts)));
    const eh = Math.abs(res.rAll[0][i] - res.yAll[1][i]), ez = Math.abs(res.rAll[1][i] - res.yAll[0][i]);
    return { eh, ez, t: res.t[i], ok: eh < tol && ez < tol };
  }
  base('sfi', 12, 'Integrators with state feedback', 'pp. 197–214, F.12 p. 400', {
    defaults(sys) { return { view: 'lat', zOff: 3, hOff: 0, antiwindup: 'clamp', extra: 'int', k: knobs(sys), w: slowW(sys, 'sfi') }; },
    simDefaults(sys) { const pr = sys.problems.ch12; return { ...pr.sim, refs: [{ type: 'step', amplitude: 2.5 }], dists: [0, pr.dists.Fwind], mismatch: pr.mismatch }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'u = −Kx − k_I ∫(r − y)', 'p. 199');
      viewSeg(sec, ctx);
      segmented(sec, { label: 'Anti-windup (F.12a)', options: [{ value: 'clamp', label: 'hold integrators while a rotor saturates' }, { value: 'none', label: 'none' }], get: () => ctx.st.antiwindup, set: (v) => { ctx.st.antiwindup = v; ctx.update(); } });
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'sfi'); knobControls(parent, ctx, 'sfi', 'Specs (target rings)'); }
      else { knobControls(parent, ctx, 'sfi', 'Design'); gainReadout(section(parent, 'Gains', 'p. 199'), ctx, 'sfi'); }
    },
    extraPlot(ctx, res) {
      return { opts: { title: 'integrators', yLabel: 'x_I [m·s]', unit: 'm·s' }, data: { series: [
        { label: 'x_I,h = ∫(h_r − h)', y: Array.from(res.extras.intH || []), color: '--series-1' },
        { label: 'x_I,z = ∫(z_r − z)', y: Array.from(res.extras.intZ || []), color: '--series-3' },
      ] } };
    },
    math(ctx) {
      const S = sub(ctx.pModel), d = design(ctx.pModel, ctx.st.k, 'sfi');
      const ah = augI(S.lon.A, S.lon.B, S.lon.C), az = augI(S.lat.A, S.lat.B, S.Cz);
      return [
        { title: 'Augmented systems', page: 'p. 198 · Eq. 12.1',
          theory: '\\dot x_I = r - C_r x,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C_r & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\0\\end{bmatrix}',
          numbers: `A_{1,lon} = ${texMat(ah.A1)},\\quad A_{1,lat} = ${texMat(az.A1, 3)}`, spoiler: true },
        ctrbCard(az.A1, az.B1, 'Controllability of (A₁, B₁), lateral', 'p. 198'),
        polesCard(ctx, d, 'sfi'),
        { title: 'Gains', page: 'p. 199–201',
          theory: '\\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad u = -Kx - k_I x_I',
          numbers: `K_h = ${texMat([d.Kh])},\\; k_{I,h} = ${tex(d.kIh)},\\quad K_z = ${texMat([d.Kz])},\\; k_{I,z} = ${tex(d.kIz)}`, spoiler: true },
        { title: 'Wind as a lateral force', page: 'F.12(b) p. 400',
          theory: '\\ddot z = \\frac{-(f_r + f_\\ell)\\sin\\theta - \\mu\\dot z + F_{wind}}{m_c + 2m_r}', symbolic: '\\theta_{ss} = \\frac{F_{wind}}{F_e}', spoiler: true,
          note: 'The integrator on z finds the steady tilt that cancels the wind; without it z settles off target.' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch12;
      const ref = () => design(ctx.pModel, knobs(ctx.sys), 'sfi');
      const num = (v) => PD().num(v);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) Gains with the F.11 poles plus p<sub>I,h</sub> = ${prob.pIh}, p<sub>I,z</sub> = ${prob.pIz}`,
          html: 'Enter k<sub>I,h</sub> and k<sub>I,z</sub> (book sign: u = −Kx − k<sub>I</sub>x<sub>I</sub>).',
          inputs: { kIh: 'k<sub>I,h</sub>', kIz: 'k<sub>I,z</sub>' },
          check: (v) => PD().checkNumbers(v, { kIh: ref().kIh, kIz: ref().kIz }, { kIh: 'kI,h', kIz: 'kI,z' }),
          solution: () => { const r = ref(); return [{ tex: `K_h = ${texMat([r.Kh])},\\; k_{I,h} = ${tex(r.kIh)},\\quad K_z = ${texMat([r.Kz])},\\; k_{I,z} = ${tex(r.kIz)}` }]; },
        },
        {
          id: 'c', title: '(b, c) Tracking with 20% uncertainty and F<sub>wind</sub> = 0.1 N',
          html: 'Passes when |h<sub>r</sub> − h| and |z<sub>r</sub> − z| at t<sub>end</sub> are both under 2 cm with the current mismatch and wind.',
          check: () => { const r = trackCheck(ctx); return { ok: r.ok, msg: `|e_h| = ${fmt(r.eh, 3)} m, |e_z| = ${fmt(r.ez, 3)} m.` }; },
          solution: () => [{ html: `The reference design keeps the F.11 poles and adds p<sub>I,h</sub> = ${prob.pIh}, p<sub>I,z</sub> = ${prob.pIz}. Both integrators settle the loops with the wind and the parameter errors; making p<sub>I</sub> faster speeds up the recovery but adds overshoot.` }],
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 13 --
  function estCards(ctx, level) {
    const S = sub(ctx.pModel), d = design(ctx.pModel, ctx.st.k, level);
    const Ol = L.obsv(S.lon.A, S.lon.C), Oz = L.obsv(S.lat.A, S.lat.C);
    return [
      { title: 'Observability', page: 'p. 221, F.13(b)',
        theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix}C\\\\ CA\\\\ \\vdots\\\\ CA^{n-1}\\end{bmatrix}',
        numbers: `\\operatorname{rank}\\mathcal{O}_{lon} = ${L.rank(Ol)},\\quad \\operatorname{rank}\\mathcal{O}_{lat} = ${L.rank(Oz)}\\;(C_{lat} \\text{ is } 2\\times4)`, spoiler: true },
      { title: 'Lateral observer gain (block structure)', page: 'p. 222 · Eq. 13.16',
        theory: 'L_{lat} = \\begin{bmatrix}L_{z1} & 0\\\\ 0 & L_{\\theta1}\\\\ L_{z2} & 0\\\\ 0 & L_{\\theta2}\\end{bmatrix}',
        symbolic: '\\operatorname{eig}(A - LC) = \\operatorname{eig}\\begin{bmatrix}-L_{z1} & 1\\\\ -L_{z2} & -\\frac{\\mu}{M}\\end{bmatrix} \\cup \\operatorname{eig}\\begin{bmatrix}-L_{\\theta1} & 1\\\\ -L_{\\theta2} & 0\\end{bmatrix}',
        numbers: d.Lz ? `L_h = ${texMat(d.Lh)},\\quad (L_{z1}, L_{z2}) = (${tex(d.Lz[0])}, ${tex(d.Lz[1])}),\\quad (L_{\\theta1}, L_{\\theta2}) = (${tex(d.Lt[0])}, ${tex(d.Lt[1])})` : '', spoiler: true,
        note: 'With two outputs L is not unique; place() on the full (A, C) would return a different L with the same eigenvalues.' },
      { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 224',
        theory: '\\dot{\\hat x} = A\\hat x + B\\tilde u + L(y - C\\hat x),\\quad \\tilde u = (F_{sat} - F_e,\\; \\tau_{sat})' },
    ];
  }
  function obsExtraPlot(ctx, res) {
    if (ctx.st.extra === 'd') {
      const dv = F.distValues(ctx), S = ctx.S, on = (t) => t >= S.sim.tDist;
      return { opts: { title: 'disturbance estimates', yLabel: 'd̂', unit: '' }, data: { series: [
        { label: 'd̂_F [N]', y: Array.from(res.extras.dFhat || []), color: '--series-1' },
        { label: 'd̂_z [N]', y: Array.from(res.extras.dzhat || []), color: '--series-3' },
        { label: 'd̂_τ [N·m]', y: Array.from(res.extras.dthat || []), color: '--series-2' },
        // what d̂_F should converge to: d_F + M_true d_h plus the weight error (M − M_true) g of F_e
        { label: 'equivalent altitude d (incl. weight error)', y: Array.from(res.t, (t) => (on(t) ? (dv.dF || 0) + WB.systems.F.mass(ctx.pTrue) * (dv.ah || 0) : 0) + (WB.systems.F.mass(ctx.pModel) - WB.systems.F.mass(ctx.pTrue)) * ctx.pModel.g), color: '--text-muted', dash: [5, 4], width: 1.5 },
      ] } };
    }
    return { opts: { title: 'velocities and estimates', yLabel: 'velocity [m/s]', unit: 'm/s' }, data: { series: [
      { label: 'ż̂', y: Array.from(res.extras.zdhat || []), color: '--series-3', dash: [3, 3], width: 2 },
      { label: 'ż', y: res.x.map((x) => x[3]), color: '--series-1' },
      { label: 'ḣ̂', y: Array.from(res.extras.hdhat || []), color: '--series-2', dash: [3, 3], width: 2 },
      { label: 'ḣ', y: res.x.map((x) => x[4]), color: '--text-muted' },
    ] } };
  }

  base('obs', 13, 'Observers', 'pp. 215–238, F.13 p. 400', {
    defaults(sys) { return { view: 'lat', zOff: 3, hOff: 0, antiwindup: 'clamp', extra: 'v', k: knobs(sys), w: slowW(sys, 'obs') }; },
    simDefaults(sys) { return { ...sys.problems.ch13.sim, refs: [{ type: 'square', amplitude: 2.5, frequency: 0.04 }] }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller uses x̂', 'p. 222 · Fig. 13-3');
      viewSeg(sec, ctx);
      segmented(sec, { label: 'Extra plot', options: [{ value: 'v', label: 'velocities' }, { value: 'd', label: 'disturbances' }], get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); } });
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'obs'); knobControls(parent, ctx, 'obs', 'Specs (target rings)'); }
      else { knobControls(parent, ctx, 'obs', 'Design'); gainReadout(section(parent, 'Gains', 'p. 222'), ctx, 'obs'); }
    },
    extraPlot: obsExtraPlot,
    math(ctx) { return [...estCards(ctx, 'obs'), { title: 'Separation principle', page: 'p. 223', theory: '\\operatorname{eig} = \\operatorname{eig}(A - BK) \\cup \\operatorname{eig}(A - LC)', note: 'For the linear model. Saturation and the nonlinear plant break it.' }]; },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch13;
      const S = () => sub(ctx.pModel);
      const ref = () => design(ctx.pModel, knobs(ctx.sys), 'obs');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'b', title: '(b) Observability ranks', inputs: { rl: 'rank 𝒪 (altitude)', rz: 'rank 𝒪 (lateral)' },
          check: (v) => PD().checkNumbers(v, { rl: L.rank(L.obsv(S().lon.A, S().lon.C)), rz: L.rank(L.obsv(S().lat.A, S().lat.C)) }, {}),
          solution: () => [{ tex: `\\mathcal{O}_{lat} = ${texMat(L.obsv(S().lat.A, S().lat.C), 3)}` }, { html: 'Ranks 2 and 4. The lateral pair is observable from z and θ together (and even from z alone).' }],
        },
        {
          id: 'c', title: `(c) Observer gains, each pair ${prob.obsFactor}× faster than its controller pair (ζ = 0.707)`,
          html: 'Altitude L<sub>h</sub> (2×1); lateral in the block structure above.',
          inputs: { Lh1: 'L<sub>h,1</sub>', Lh2: 'L<sub>h,2</sub>', Lz1: 'L<sub>z,1</sub>', Lz2: 'L<sub>z,2</sub>', Lt1: 'L<sub>θ,1</sub>', Lt2: 'L<sub>θ,2</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { Lh1: r.Lh[0], Lh2: r.Lh[1], Lz1: r.Lz[0], Lz2: r.Lz[1], Lt1: r.Lt[0], Lt2: r.Lt[1] }, {}); },
          solution: () => { const r = ref(); return [
            { tex: `q_h = ${r.obsPoles.lon.map((q) => texPole(q)).join(',\\;')},\\; q_z = ${r.obsPoles.outer.map((q) => texPole(q)).join(',\\;')},\\; q_\\theta = ${r.obsPoles.inner.map((q) => texPole(q)).join(',\\;')}` },
            { tex: `L_h = ${texMat(r.Lh)},\\quad (L_{z1}, L_{z2}) = (${tex(r.Lz[0])}, ${tex(r.Lz[1])}),\\quad (L_{\\theta1}, L_{\\theta2}) = (${tex(r.Lt[0])}, ${tex(r.Lt[1])})` },
            { html: 'Each block is a SISO observer design: L = place(Aᵀ, Cᵀ, q)ᵀ (p. 222). For the z block, det(sI − A + LC) = s² + (L<sub>z1</sub> + μ/M)s + (μ/M·L<sub>z1</sub> + L<sub>z2</sub>).' },
          ]; },
        },
        {
          id: 'e', title: '(e) Add d<sub>F</sub> = 1.0 N and d<sub>τ</sub> = 0.1 N·m',
          html: 'Set the thrust and torque disturbances in the left panel. The integrators act on r − ŷ, and ŷ is biased by the unmodeled disturbances, so the outputs settle off target.',
          actions: [{ label: 'Set them', run: () => { ctx.S.sim.dist = 1.0; ctx.S.sim.dists[0] = 0.1; ctx.update(); return null; } }],
          check: () => {
            const res = ctx.app.result(), n = res.t.length - 1;
            const dv = F.distValues(ctx);
            if (!(dv.dF || dv.dtau)) return { ok: false, msg: 'Add the disturbances first.' };
            const bh = res.yAll[1][n] - res.extras.hhat[n], bz = res.yAll[0][n] - res.extras.zhat[n];
            return { ok: true, msg: `At t_end: h − ĥ = ${fmt(bh, 3)} m, z − ẑ = ${fmt(bz, 3)} m. F.14 removes this bias.` };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 14 --
  base('dobs', 14, 'Disturbance observers', 'pp. 239–259, F.14 p. 401', {
    defaults(sys) { return { view: 'lat', zOff: 3, hOff: 0, antiwindup: 'clamp', dobs: true, extra: 'd', k: knobs(sys), w: slowW(sys, 'dobs') }; },
    simDefaults(sys) { const pr = sys.problems.ch14; return { ...pr.sim, refs: [{ type: 'square', amplitude: 2.5, frequency: 0.04 }], dists: [0, 0, pr.dists.wind, pr.dists.ah], mismatch: pr.mismatch }; },
    buildControls(parent, ctx) {
      const sec = section(parent, 'Controller uses x̂, subtracts d̂', 'p. 241');
      viewSeg(sec, ctx);
      segmented(sec, { label: 'Disturbance observer', options: [{ value: true, label: 'on' }, { value: false, label: 'off (F.14a)' }], get: () => ctx.st.dobs, set: (v) => { ctx.st.dobs = v; ctx.update(); } });
      segmented(sec, { label: 'Extra plot', options: [{ value: 'd', label: 'disturbance estimates' }, { value: 'v', label: 'velocities' }], get: () => ctx.st.extra, set: (v) => { ctx.st.extra = v; ctx.update(); } });
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'dobs'); knobControls(parent, ctx, 'dobs', 'Specs (target rings)'); }
      else { knobControls(parent, ctx, 'dobs', 'Design'); gainReadout(section(parent, 'Gains', 'p. 241'), ctx, 'dobs'); }
    },
    extraPlot: obsExtraPlot,
    math(ctx) {
      const d = design(ctx.pModel, ctx.st.k, 'dobs');
      return [
        { title: 'Disturbance models', page: 'p. 240–241',
          theory: '\\text{altitude: } \\dot{\\hat d}_F = L_{d,h}(h - \\hat h)\\text{ at the input};\\quad \\text{lateral: } d_\\tau \\text{ at the input, } d_z \\text{ a force in } \\ddot z',
          note: 'A constant wind speed w added to ż (the F.14 snippet) is exactly a force μw in the observer\'s ż coordinates, so the d_z state absorbs it.' },
        { title: 'Augmented blocks', page: 'p. 240',
          theory: 'A_2 = \\begin{bmatrix}A & B_d\\\\ 0 & 0\\end{bmatrix},\\quad C_2 = \\begin{bmatrix}C & 0\\end{bmatrix}\\;(\\dot d = 0)', spoiler: true,
          symbolic: 'A_{h} = \\begin{bmatrix}0&1&0\\\\0&0&\\frac1M\\\\0&0&0\\end{bmatrix},\\; A_{z} = \\begin{bmatrix}0&1&0\\\\0&-\\frac{\\mu}{M}&\\frac1M\\\\0&0&0\\end{bmatrix},\\; A_{\\theta} = \\begin{bmatrix}0&1&0\\\\0&0&\\frac1J\\\\0&0&0\\end{bmatrix},\\; C = \\begin{bmatrix}1&0&0\\end{bmatrix}' },
        { title: 'Observer gains', page: 'p. 241',
          theory: 'L = \\text{place}(A^\\top, C^\\top, q)^\\top \\text{ per block}',
          numbers: `L_h = ${texMat(d.Lh)},\\; L_z = ${texMat(d.Lz)},\\; L_\\theta = ${texMat(d.Lt)}`, spoiler: true },
        { title: 'Control law', page: 'p. 241',
          theory: '\\tilde F = -K_h\\hat x_{lon} - k_{I,h}x_{I,h} - \\hat d_F,\\quad \\tau = -K_z\\hat x_{lat} - k_{I,z}x_{I,z} - \\hat d_\\tau',
          note: 'd_z is not matched to τ, so it is not cancelled directly; removing the estimator bias lets the z integrator do the rest.' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch14;
      const ref = () => design(ctx.pModel, knobs(ctx.sys), 'dobs');
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'g', title: `(b) Disturbance-observer gains (observers ${prob.obsFactor || 10}× faster, p<sub>d,h</sub> = ${prob.pDh}, p<sub>d,z</sub> = ${prob.pDz}, p<sub>d,θ</sub> = ${prob.pDth})`,
          inputs: { Ldh: 'L<sub>d,h</sub>', Ldz: 'L<sub>d,z</sub>', Ldt: 'L<sub>d,θ</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { Ldh: r.Lh[2], Ldz: r.Lz[2], Ldt: r.Lt[2] }, {}); },
          solution: () => { const r = ref(); return [{ tex: `L_h = ${texMat(r.Lh)},\\quad L_z = ${texMat(r.Lz)},\\quad L_\\theta = ${texMat(r.Lt)}` }, { html: 'Same block structure as F.13, each block augmented with its disturbance state.' }]; },
        },
        {
          id: 'b', title: '(b) Estimator bias removed',
          html: 'Passes when |h − ĥ| and |z − ẑ| at t<sub>end</sub> are under 1 cm with the observer on. Compare with it off.',
          check: () => {
            if (!ctx.st.dobs) return { ok: false, msg: 'Turn the disturbance observer on.' };
            const res = ctx.app.result(), n = res.t.length - 1;
            const bh = Math.abs(res.yAll[1][n] - res.extras.hhat[n]), bz = Math.abs(res.yAll[0][n] - res.extras.zhat[n]);
            const t = trackCheck(ctx, 0.05);
            return { ok: bh < 0.01 && bz < 0.01, msg: `|h − ĥ| = ${fmt(bh, 3)} m, |z − ẑ| = ${fmt(bz, 3)} m; tracking |e_h| = ${fmt(t.eh, 3)}, |e_z| = ${fmt(t.ez, 3)} m at t = ${fmt(t.t, 3)} s.` };
          },
          solution: () => [{ html: 'd̂<sub>F</sub> settles at the altitude disturbance (M·1.0 N) plus the weight error from the mass mismatch; d̂<sub>z</sub> settles near μ·w for the wind speed w.' }],
        },
      ]);
    },
  });

  WB.F.ss = { design, makeSS, sub, latL, knobs };
})();
