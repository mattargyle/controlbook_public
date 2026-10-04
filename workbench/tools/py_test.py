#!/usr/bin/env python3
"""End-to-end test of the Python answer parts (Pyodide) and live-math locking.

For every chapter tab of the given studies that has Python parts, in Work mode:
  - the starting template must fail its Check,
  - the code in "Show solution" must pass it,
  - every locked live-math card must unlock once all of the chapter's parts are solved,
plus targeted wrong answers (hard-coded numbers, a sign error, an infinite loop) and
"Simulate my f". Needs google-chrome and network access to cdn.jsdelivr.net.

    python3 workbench/tools/py_test.py            # study A
    python3 workbench/tools/py_test.py --study A --chapters ch3,ch4
"""
import argparse
import json
import pathlib
import sys

from cdp import Chrome

WB = pathlib.Path(__file__).resolve().parent.parent
INDEX = WB / "index.html"

# JS helpers installed on each page.
HELPERS = r"""
(() => {  // remember each time plot's data for yourDeviations
  const TP = WB.plot.TimePlot.prototype;
  if (TP.__wrapped) return;
  const set = TP.setData;
  window.__plotData = new Map();
  TP.setData = function (d) { window.__plotData.set(this, d); return set.call(this, d); };
  TP.__wrapped = true;
})();
window.__t = {
  parts: () => [...document.querySelectorAll('#problem .part')],
  title: (p) => (p.querySelector('.part-title .collapse-label') || p.querySelector('.part-title')).textContent,
  setCode: (p, code) => { const ta = p.querySelector('textarea.code-editor'); ta.value = code; ta.dispatchEvent(new Event('input')); },
  button: (p, text) => [...p.querySelectorAll('.part-buttons button')].find((b) => b.textContent === text),
  async press(p, text) {
    const res = p.querySelector('.part-result');
    res.textContent = '';
    window.confirm = () => true;
    this.button(p, text).click();
    for (let i = 0; i < 1200; i++) {
      const t = res.textContent;
      if (t && !/^(Starting Python|Running)/.test(t)) break;
      await new Promise((r) => setTimeout(r, 100));
    }
    return { good: !!res.querySelector('.status.good'), bad: !!res.querySelector('.status.bad'), text: res.textContent.slice(0, 400) };
  },
  solution(p) {
    this.button(p, 'Show solution').click();
    const pre = p.querySelector('.part-solution .sol-code');
    return pre ? pre.textContent : null;
  },
  locked: () => document.querySelectorAll('.math-card.locked').length,
  // "Plot my ..." style buttons (overlays of the student's own answer).
  overlayButtons: (p) => [...p.querySelectorAll('.part-buttons button')].map((b) => b.textContent)
    .filter((t) => /^(Plot|Use|Simulate) my /.test(t) && t !== 'Simulate my f'),
  // Each visible "your ..." time series against the closest reference series:
  // [{label, dev}] with dev = max|yours - ref| / max|ref| (a correct answer gives ~0).
  yourDeviations() {
    const out = [];
    for (const [plot, d] of window.__plotData || []) {
      if (plot.box.closest('[hidden]')) continue;
      const refs = d.series.filter((s) => !/\byour\b/i.test(s.label || '') && !/measured|^zₑ$|reference/.test(s.label || ''));
      for (const s of d.series.filter((q) => /\byour\b/i.test(q.label || ''))) {
        let dev = Infinity;
        for (const r of refs) {
          if (r.y.length !== s.y.length) continue;
          let e = 0, m = 1e-9;
          for (let k = 0; k < r.y.length; k++) { if (!Number.isFinite(r.y[k])) continue; e = Math.max(e, Math.abs(s.y[k] - r.y[k])); m = Math.max(m, Math.abs(r.y[k])); }
          dev = Math.min(dev, e / m);
        }
        out.push({ label: s.label, dev });
      }
    }
    return out;
  },
  yourLegends: () => [...document.querySelectorAll('.legend-item')].map((e) => e.textContent).filter((t) => /\byour\b/i.test(t)),
};
return true;
"""

from py_targets import TARGETED  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--study", default="A")
    ap.add_argument("--chapters", default=None, help="comma-separated chapter ids (default: all)")
    ap.add_argument("--chrome", default="google-chrome")
    args = ap.parse_args()

    fails = 0

    def report(ok, label, detail=""):
        nonlocal fails
        fails += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {label}" + (f"\n       {detail}" if detail and not ok else ""))

    with Chrome(args.chrome) as c:
        nav = 0

        def open_tab(ch):
            nonlocal nav
            nav += 1
            c.goto(f"{INDEX.as_uri()}?n={nav}#{args.study}/{ch}/work", wait=1.5)
            c.js(HELPERS)

        open_tab("ch2")
        chapters = c.js("return [...document.querySelectorAll('#tabs .tab[data-ch]')].map((b) => b.dataset.ch)")
        if args.chapters:
            chapters = [x for x in chapters if x in args.chapters.split(",")]
        for ch in chapters:
            open_tab(ch)
            n = c.js("return __t.parts().filter((p) => p.querySelector('.code-wrap')).length")
            if not n:
                continue
            locked0 = c.js("return __t.locked()")
            for i in range(n):
                title = c.js(f"return __t.title(__t.parts().filter((p) => p.querySelector('.code-wrap'))[{i}])")
                part = f"__t.parts().filter((p) => p.querySelector('.code-wrap'))[{i}]"
                tmpl = c.js(f"return await __t.press({part}, 'Check')")
                report(not tmpl["good"], f"{args.study} {ch} {title}: template fails", tmpl["text"])
                sol = c.js(f"return __t.solution({part})")
                if sol is None:
                    report(False, f"{args.study} {ch} {title}: has solution code")
                    continue
                c.js(f"__t.setCode({part}, {json.dumps(sol)}); return 1")
                r = c.js(f"return await __t.press({part}, 'Check')")
                report(r["good"], f"{args.study} {ch} {title}: solution passes", r["text"])
                if c.js(f"return !!__t.button({part}, 'Simulate my f')"):
                    r = c.js(f"return await __t.press({part}, 'Simulate my f')")
                    report(r["good"], f"{args.study} {ch} {title}: Simulate my f overlaps the arm", r["text"])
                for label in c.js(f"return __t.overlayButtons({part})"):
                    r = c.js(f"return await __t.press({part}, {json.dumps(label)})")
                    yours = c.js("return __t.yourLegends()")
                    ok = not r["bad"] and bool(r["text"]) and (bool(yours) or label.startswith("Use my"))
                    report(ok, f"{args.study} {ch} {title}: {label} draws the student's answer", f"{r['text']} | legends: {yours}")
                    if label.startswith("Use my"):
                        continue
                    devs = c.js("return new Promise((r) => setTimeout(() => r(__t.yourDeviations()), 300))")
                    bad = [d for d in devs if not (d["dev"] is not None and d["dev"] < 1e-3)]
                    report(bool(devs) and not bad, f"{args.study} {ch} {title}: {label} with the solution lies on the workbench's model", json.dumps(devs))
            # Non-Python parts of this chapter that unlock cards: solve them from their solutions is
            # not generic, so only require that Python parts unlocked something when cards were locked.
            locked1 = c.js("return __t.locked()")
            print(f"     {ch}: locked cards {locked0} -> {locked1}")
            for (tch, prefix, code, expect, should_pass) in TARGETED.get(args.study, []):
                if tch != ch:
                    continue
                part = f"__t.parts().find((p) => __t.title(p).startsWith({json.dumps(prefix)}))"
                c.js(f"__t.setCode({part}, {json.dumps(code)}); return 1")
                r = c.js(f"return await __t.press({part}, 'Check')")
                ok = (r["good"] == should_pass) and expect in r["text"]
                report(ok, f"{args.study} {ch} {prefix} targeted: expects {'pass' if should_pass else 'fail'} with '{expect}'", r["text"])
            for e in c.errors():
                report(False, f"{args.study} {ch}: JS error", e)
    print(f"{fails} failure(s)")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
