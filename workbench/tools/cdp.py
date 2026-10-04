"""Minimal Chrome DevTools Protocol driver over --remote-debugging-pipe (stdlib only).

Used by py_test.py, which needs real time (Pyodide downloads and runs in a
worker), so the --dump-dom/--virtual-time approach of smoke_test.py can't wait for it.

    with Chrome() as c:
        c.goto(url)
        c.js('return document.title')        # body of an async function; returns JSON
        c.errors()                           # JS exceptions and console errors so far
"""
import json
import os
import shutil
import subprocess
import tempfile
import time


class Chrome:
    def __init__(self, chrome="google-chrome", size="1500,1400"):
        self.dir = tempfile.mkdtemp(prefix="wb-cdp-")
        to_chrome_r, self._w = os.pipe()
        self._r, from_chrome_w = os.pipe()

        def fds():  # Chrome reads commands on fd 3 and writes replies on fd 4
            for src, dst in ((to_chrome_r, 3), (from_chrome_w, 4)):
                if src == dst:
                    os.set_inheritable(dst, True)  # pipe fds are close-on-exec; dup2(fd, fd) won't clear it
                else:
                    os.dup2(src, dst)

        self.proc = subprocess.Popen(
            [chrome, "--headless=new", "--disable-gpu", "--no-sandbox", "--remote-debugging-pipe",
             f"--user-data-dir={self.dir}", "--allow-file-access-from-files", f"--window-size={size}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=fds, close_fds=False)
        os.close(to_chrome_r)
        os.close(from_chrome_w)
        self._buf = b""
        self._id = 0
        self.events = []
        self.session = None
        target = self.send("Target.createTarget", url="about:blank")["targetId"]
        self.session = self.send("Target.attachToTarget", targetId=target, flatten=True)["sessionId"]
        for m in ("Runtime.enable", "Page.enable", "Log.enable"):
            self.send(m)

    def _recv(self):
        while b"\0" not in self._buf:
            chunk = os.read(self._r, 1 << 16)
            if not chunk:
                raise RuntimeError("Chrome closed the pipe")
            self._buf += chunk
        msg, self._buf = self._buf.split(b"\0", 1)
        return json.loads(msg)

    def send(self, method, **params):
        self._id += 1
        msg = {"id": self._id, "method": method, "params": params}
        if self.session:
            msg["sessionId"] = self.session
        os.write(self._w, json.dumps(msg).encode() + b"\0")
        while True:
            m = self._recv()
            if m.get("id") == self._id:
                if "error" in m:
                    raise RuntimeError(f"{method}: {m['error']}")
                return m.get("result", {})
            self.events.append(m)

    def goto(self, url, wait=1.0):
        self.send("Page.navigate", url=url)
        time.sleep(wait)

    def js(self, body, timeout=180):
        r = self.send("Runtime.evaluate", expression=f"(async () => {{ {body} }})()", awaitPromise=True,
                      returnByValue=True, timeout=timeout * 1000)
        if "exceptionDetails" in r:
            d = r["exceptionDetails"]
            raise RuntimeError(d.get("exception", {}).get("description") or d.get("text"))
        return r["result"].get("value")

    def errors(self):
        out = []
        for e in self.events:
            m, p = e.get("method"), e.get("params", {})
            if m == "Runtime.exceptionThrown":
                d = p["exceptionDetails"]
                out.append(d.get("exception", {}).get("description") or d.get("text", ""))
            elif m == "Runtime.consoleAPICalled" and p.get("type") == "error":
                out.append("console: " + " ".join(str(a.get("value", a.get("description", ""))) for a in p["args"]))
        self.events = []
        return out

    def close(self):
        self.proc.kill()
        self.proc.wait()
        shutil.rmtree(self.dir, ignore_errors=True)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
