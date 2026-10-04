#!/usr/bin/env python3
"""Headless smoke test for the workbench.

For each study (or the ones given), opens every chapter tab in Work and Explore
mode, clicks every Check / Show solution / Reveal button, and reports JavaScript
errors. Optionally saves one screenshot per chapter (Explore mode).

    python3 workbench/tools/smoke_test.py                 # all studies
    python3 workbench/tools/smoke_test.py --study B       # one study
    python3 workbench/tools/smoke_test.py --study B --shots /path/to/dir

Exit status is 1 if any error was seen. Needs google-chrome (or --chrome PATH).
"""
import argparse
import html
import json
import os
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
WB = HERE.parent
INDEX = WB / "index.html"

HARNESS_JS = r"""
<pre id="smoke-out"></pre>
<script>
window.addEventListener('load', () => setTimeout(() => {
  const out = { tabs: [], errors: window.__smokeErrs };
  const study = %(study)s;
  const sel = document.getElementById('study');
  const studies = study ? [study] : [...sel.options].filter(o => !o.disabled).map(o => o.value);
  for (const sid of studies) {
    if (sel.value !== sid) { sel.value = sid; sel.dispatchEvent(new Event('change')); }
    const tabIds = [...document.querySelectorAll('#tabs .tab[data-ch]')].map(b => b.dataset.ch);
    for (const ch of tabIds) {
      for (const mode of ['Work it', 'Explore']) {
        const before = window.__smokeErrs.length;
        try {
          const b = document.querySelector(`#tabs .tab[data-ch="${ch}"]`); b.click();
          [...document.querySelectorAll('#mode button')].find(x => x.textContent === mode).click();
          // Python parts (.code-wrap) need the network; tools/py_test.py covers them.
          document.querySelectorAll('#problem .part-buttons button').forEach(x => { if ((x.textContent === 'Check' && !x.closest('.part').querySelector('.code-wrap')) || x.textContent === 'Show solution') x.click(); });
          document.querySelectorAll('.reveal').forEach(x => x.click());
          document.querySelectorAll('#right .btn').forEach(x => { if (/^Reveal/.test(x.textContent)) x.click(); });
        } catch (e) { window.__smokeErrs.push(`EXC ${sid}/${ch}/${mode}: ${e.message}`); }
        out.tabs.push({ study: sid, ch, mode, cards: document.querySelectorAll('.math-card').length,
          plots: document.querySelectorAll('#plot-y .card').length, newErrors: window.__smokeErrs.slice(before) });
      }
    }
  }
  document.getElementById('smoke-out').textContent = JSON.stringify(out);
}, 600));
</script>
"""

ERR_HOOK = ('<script>window.__smokeErrs=[];window.addEventListener("error",e=>window.__smokeErrs.push('
            '(e.message||"error")+" @"+((e.filename||"").split("/").slice(-2).join("/"))+":"+e.lineno));</script>')


def chrome(args, url, budget):
    cmd = [args.chrome, "--headless=new", "--disable-gpu", "--no-sandbox", "--allow-file-access-from-files",
           f"--virtual-time-budget={budget}"] + url
    return subprocess.run(cmd, capture_output=True, text=True, timeout=600)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--study", default=None)
    ap.add_argument("--shots", default=None, help="directory for one Explore-mode screenshot per chapter")
    ap.add_argument("--chrome", default="google-chrome")
    ap.add_argument("--budget", type=int, default=60000, help="virtual time budget in ms")
    args = ap.parse_args()

    src = INDEX.read_text()
    base = f'<base href="{WB.as_uri()}/">'
    src = src.replace("<head>", "<head>" + base + ERR_HOOK, 1)
    src = src.replace("</body>", HARNESS_JS % {"study": json.dumps(args.study)} + "</body>", 1)
    harness = WB / f".smoke_harness_{os.getpid()}.html"  # unique: runs may overlap
    harness.write_text(src)
    try:
        start = f"#{args.study}/ch2/work" if args.study else "#A/ch2/work"
        res = chrome(args, ["--dump-dom", harness.as_uri() + start], args.budget)
        m = re.search(r'<pre id="smoke-out">(.*?)</pre>', res.stdout, re.S)
        if not m or not m.group(1).strip():
            print("NO OUTPUT from the harness (page failed before load?)")
            print(res.stderr[-2000:])
            return 1
        data = json.loads(html.unescape(m.group(1)))
    finally:
        harness.unlink(missing_ok=True)

    bad = 0
    for t in data["tabs"]:
        flag = "ERR" if t["newErrors"] else "ok "
        bad += bool(t["newErrors"])
        print(f'{flag} {t["study"]} {t["ch"]:5s} {t["mode"]:8s} cards={t["cards"]} plots={t["plots"]}')
        for e in t["newErrors"]:
            print("      ", e)
    early = [e for e in data["errors"] if not any(e in t["newErrors"] for t in data["tabs"])]
    for e in early:
        print("ERR (before tabs)", e)
    print(f'{len(data["tabs"])} tab/mode combinations, {bad} with errors, {len(early)} load errors')

    if args.shots:
        out = pathlib.Path(args.shots)
        out.mkdir(parents=True, exist_ok=True)
        for t in data["tabs"]:
            if t["mode"] != "Explore":
                continue
            png = out / f'{t["study"]}_{t["ch"]}.png'
            chrome(args, ["--window-size=1600,1600", f"--screenshot={png}", INDEX.as_uri() + f'#{t["study"]}/{t["ch"]}/explore'], 8000)
        print(f"screenshots in {out}")
    return 1 if (bad or early) else 0


if __name__ == "__main__":
    sys.exit(main())
