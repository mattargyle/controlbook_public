#!/usr/bin/env python3
"""Run a JavaScript snippet with every workbench script loaded and return its JSON.

    from js_eval import js_eval
    data = js_eval("return WB.design.wnFromTr(0.6, 0.9, 'tp');")

The snippet is the body of a function; whatever it returns is JSON-encoded.
Scripts load exactly as index.html loads them (including study manifests), but
app.js is skipped, so no page UI is built. Needs google-chrome.
"""
import html
import json
import os
import pathlib
import re
import subprocess

WB = pathlib.Path(__file__).resolve().parent.parent


def js_eval(body, chrome="google-chrome", budget=20000):
    index = (WB / "index.html").read_text()
    scripts = re.findall(r'<script src="(js/[^"]+)"></script>', index)
    scripts = [s for s in scripts if not s.endswith("app.js")]
    tags = "".join(f'<script src="{s}"></script>' for s in scripts)
    page = f"""<!doctype html><html><head><meta charset="utf-8"><base href="{WB.as_uri()}/">
<script>window.__errs=[];addEventListener("error",e=>window.__errs.push(e.message+" @"+(e.filename||"").split("/").slice(-2).join("/")+":"+e.lineno));</script>
</head><body>{tags}<pre id="out"></pre>
<script>
try {{ const r = (function () {{ {body} }})(); document.getElementById('out').textContent = JSON.stringify({{ ok: true, value: r, errors: window.__errs }}); }}
catch (e) {{ document.getElementById('out').textContent = JSON.stringify({{ ok: false, error: e.message + ' ' + (e.stack || ''), errors: window.__errs }}); }}
</script></body></html>"""
    tmp = WB / f".smoke_eval_{os.getpid()}.html"  # unique: runs may overlap
    tmp.write_text(page)
    try:
        res = subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-sandbox", "--allow-file-access-from-files",
                              f"--virtual-time-budget={budget}", "--dump-dom", tmp.as_uri()], capture_output=True, text=True, timeout=300)
    finally:
        tmp.unlink(missing_ok=True)
    m = re.search(r'<pre id="out">(.*?)</pre>', res.stdout, re.S)
    if not m or not m.group(1).strip():
        raise RuntimeError("no output from chrome:\n" + res.stderr[-2000:])
    out = json.loads(html.unescape(m.group(1)))
    if not out["ok"]:
        raise RuntimeError(out["error"] + "\nerrors: " + "; ".join(out["errors"]))
    if out["errors"]:
        raise RuntimeError("script errors: " + "; ".join(out["errors"]))
    return out["value"]


if __name__ == "__main__":
    import sys
    print(json.dumps(js_eval(sys.stdin.read()), indent=1))
