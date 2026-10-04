// Student Python, run with Pyodide (CPython + numpy compiled to WebAssembly).
//
// Security model (see README "Python answers"):
//   - Pyodide runs in a Web Worker built from a Blob, so student code has no
//     access to the page, its DOM, or localStorage.
//   - Every file Pyodide downloads is pinned: the loader, the core module, the
//     wasm binary and the stdlib are fetched with subresource-integrity hashes,
//     and the lock file (also pinned) carries the sha256 Pyodide checks numpy against.
//   - Once numpy is loaded, the worker deletes its network APIs (fetch, XHR,
//     WebSocket, importScripts, ...), so code can't reach the network.
//   - Each run has a time limit; on timeout the worker is terminated and the next
//     run starts a fresh one.
//   - Code only runs when the student clicks a button; nothing runs from the URL.
//
// Pyodide loads on first use (about 15 MB, cached by the browser afterwards).
window.WB = window.WB || {};

WB.py = (function () {
  const VERSION = '314.0.7';
  const BASE = `https://cdn.jsdelivr.net/pyodide/v${VERSION}/full/`;
  const HASH = {
    'pyodide.mjs': 'sha384-Lp79fMwxa4n2BLtugpUAoQMlbxIdg8iCUiP1r3h7nJLkQjwlXT1JzZTeHVL+f4Dg',
    'pyodide.asm.mjs': 'sha384-/2281iaC0Iimw0B3Kagy1DBfZcJG3W4KYW+EmOqhsdY3fR9BJ88FE2Zt6y0KQKpa',
    'pyodide.asm.wasm': 'sha384-y++qrQ72KPA1fTjLIuGMP+jppgGxjbLCgfdb0zMrnNnQVhxRUXTOTF6rTxNzllSu',
    'python_stdlib.zip': 'sha384-xW3A5jmenkrPXbsgrIaB1FCJh9OuLT7FRMagsJMgtlhVc19k/mSy3+7nMaF/d767',
    'pyodide-lock.json': 'sha384-o8SQQvpVlZnOl04+3dFTJipKxul9GbViVusA379wlOrfHSgxbzt0yPiqCTxOiYES',
  };

  // Python side: evaluate student code at sample points, or simulate with RK4.
  const HARNESS = String.raw`
import io, json, linecache, math, sys, traceback, types
import numpy as np

FILE = '<your code>'

def _out(v):
    if v is None:
        raise TypeError('returned None (missing return?)')
    if isinstance(v, (bool, np.bool_)):
        raise TypeError('returned True/False, not a number')
    if isinstance(v, (complex, np.complexfloating)):
        return {'re': float(v.real), 'im': float(v.imag)}
    if isinstance(v, (int, float, np.integer, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else str(f)  # JSON has no inf/nan: 'inf', '-inf', 'nan'
    if isinstance(v, np.ndarray):
        return _out(v.item()) if v.ndim == 0 else [_out(x) for x in v]
    if isinstance(v, (list, tuple)):
        return [_out(x) for x in v]
    try:
        return float(v)
    except Exception:
        raise TypeError(f'returned a {type(v).__name__}, not a number or numpy array')

def _arg(a):
    if isinstance(a, dict) and 're' in a:
        return complex(a['re'], a['im'])
    if isinstance(a, dict) and 'col' in a:
        return np.array(a['col'], dtype=float).reshape(-1, 1)
    return a

def _err(e, extra=''):
    frames = [f for f in traceback.extract_tb(e.__traceback__) if f.filename == FILE]
    where = f'line {frames[-1].lineno}: {(frames[-1].line or "").strip()}' if frames else ''
    if isinstance(e, SyntaxError) and e.filename == FILE:
        where = f'line {e.lineno}: {(e.text or "").strip()}'
        return {'error': f'SyntaxError: {e.msg}', 'where': where}
    return {'error': f'{type(e).__name__}: {e}' + extra, 'where': where}

def _compile(src):
    linecache.cache[FILE] = (len(src), None, src.splitlines(True), FILE)  # so tracebacks show the line
    return compile(src, FILE, 'exec')

def _namespace(code, params):
    ns = {'__name__': '__student__', 'np': np, 'math': math, 'P': types.SimpleNamespace(**params)}
    exec(code, ns)
    return ns

def _run(fn, payload):
    buf = io.StringIO()
    old = sys.stdout
    sys.stdout = buf
    try:
        out = fn(json.loads(payload))
    except Exception as e:
        out = _err(e)
    finally:
        sys.stdout = old
    out['stdout'] = buf.getvalue()[-4000:]
    return json.dumps(out)

def _evaluate(a):
    code = _compile(a['code'])
    rows = []
    for s in a['samples']:
        ns = _namespace(code, s['params'])
        row = {'vars': {}, 'calls': []}
        for name in s.get('vars', []):
            if name not in ns:
                return {'error': f'NameError: define {name}', 'where': ''}
            try:
                row['vars'][name] = _out(ns[name])
            except Exception as e:
                return _err(e, f' ({name})')
        for c in s.get('calls', []):
            fn = ns.get(c['name'])
            if not callable(fn):
                return {'error': f"NameError: define a function named {c['name']}", 'where': ''}
            try:
                row['calls'].append(_out(fn(*[_arg(x) for x in c['args']])))
            except Exception as e:
                return _err(e, f" (in {c['name']})")
        rows.append(row)
    return {'rows': rows}

# Optional: 'plant' is the workbench's own f(state, u) source (run in a separate
# namespace with 'plantParams', default 'params'). Once per step, at its start (a
# zero-order hold, as WB.sim does), 'ctrl' names a function of the state components
# whose value is added to u[k], then saturated at uLimit and disturbed by d[k];
# 'hold' names a function hold(state, u[k]) that returns the input held over the step
# (for vector inputs, e.g. a student's feedback law plus the workbench's mixing).
def _simulate(a):
    code = _compile(a['code'])
    ns = _namespace(code, a['params'])
    name = 'f' if a.get('plant') else a['fn']
    f = (_namespace(compile(a['plant'], '<workbench plant>', 'exec'), a.get('plantParams') or a['params']) if a.get('plant') else ns).get(name)
    if not callable(f):
        return {'error': f"NameError: define a function named {name}", 'where': ''}
    ctrl = ns.get(a['ctrl']) if a.get('ctrl') else None
    if a.get('ctrl') and not callable(ctrl):
        return {'error': f"NameError: define a function named {a['ctrl']}", 'where': ''}
    hold = ns.get(a['hold']) if a.get('hold') else None
    if a.get('hold') and not callable(hold):
        return {'error': f"NameError: define a function named {a['hold']}", 'where': ''}
    lim, d = a.get('uLimit'), a.get('d')
    sat = (lambda v: max(-lim, min(lim, v))) if lim else (lambda v: v)
    x = np.array(a['x0'], dtype=float).reshape(-1, 1)
    n, Ts = x.shape[0], a['Ts']
    def F(xx, u):
        v = np.asarray(f(xx, u), dtype=float)
        if v.size != n:
            raise ValueError(f'{name} returned shape {v.shape}; expected ({n}, 1)')
        return v.reshape(n, 1)
    xs = []
    for k, u in enumerate(a['u']):
        if isinstance(u, list):  # vector input: a column, as check passes it
            u = np.array(u, dtype=float).reshape(-1, 1)
        if ctrl:
            try:
                uc = np.asarray(ctrl(*x[:, 0].tolist()), dtype=float)
                if uc.size != 1:
                    raise ValueError(f'returned {uc.size} values; expected one number')
                u = sat(float(uc.item()) + u)
            except Exception as e:
                return _err(e, f" (in {a['ctrl']}, at t = {k * Ts:.3g} s)")
            if d:
                u = sat(u + d[k])
        if hold:
            try:
                u = np.asarray(hold(x, u), dtype=float)
            except Exception as e:
                return _err(e, f' (at t = {k * Ts:.3g} s)')
        xs.append(x[:, 0].tolist())
        if k == len(a['u']) - 1:
            break
        try:
            F1 = F(x, u); F2 = F(x + Ts / 2 * F1, u); F3 = F(x + Ts / 2 * F2, u); F4 = F(x + Ts * F3, u)
        except Exception as e:
            return _err(e, f' (at t = {k * Ts:.3g} s)')
        x = x + Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        if not np.all(np.isfinite(x)):
            return {'error': f'The state became inf/NaN at t = {(k + 1) * Ts:.3g} s', 'where': ''}
    return {'x': xs}

# Student controllers (Ch 7-18). The student's class runs with 'params' (the
# controller's model); the workbench's plant 'f' and output 'h' run in their own
# namespace with 'plantParams' (the true plant). Each step, as hwNN_*Sim.py:
# y_m = h(x) + noise[k], out = ctrl.update(r[k], y_m or x), u = sat(out),
# u_applied = sat(u + d[k]), then RK4 over Ts with u_applied held. update may
# return u, or a tuple (u, x_hat) or (u, x_hat, d_hat), which are recorded.
def _make_ctrl(a):
    code = _compile(a['code'])
    ns = _namespace(code, a['params'])
    name = a.get('cls', 'Controller')
    cls = ns.get(name)
    if not isinstance(cls, type):
        return None, {'error': f'NameError: define a class named {name}', 'where': ''}
    try:
        ctrl = cls()
    except Exception as e:
        return None, _err(e, f' (in {name}())')
    if not callable(getattr(ctrl, 'update', None)):
        return None, {'error': f'AttributeError: {name} has no update(self, r, y) method', 'where': ''}
    return ctrl, None

def _split(ret, m, t):
    # (u, x_hat, d_hat) from what update returned; raises with a message for the student
    extra = []
    if isinstance(ret, tuple):
        if not ret:
            raise TypeError('update returned an empty tuple')
        ret, extra = ret[0], list(ret[1:3])
    if ret is None:
        raise TypeError(f'update returned None at t = {t:.3g} s (missing return?)')
    try:
        u = np.asarray(ret, dtype=float).flatten()
    except Exception:
        raise TypeError(f'update returned a {type(ret).__name__}, not a number')
    if u.size != m:
        raise ValueError(f'update returned {u.size} input values at t = {t:.3g} s; expected {m}')
    if not np.all(np.isfinite(u)):
        raise ValueError(f'update returned inf/NaN at t = {t:.3g} s')
    vecs = []
    for v in extra:
        try:
            w = np.asarray(v, dtype=float).flatten()
        except Exception:
            raise TypeError(f'update returned a {type(v).__name__} after u, not a number or array')
        if not np.all(np.isfinite(w)):
            raise ValueError(f'update returned an inf/NaN estimate at t = {t:.3g} s')
        vecs.append(w.tolist())
    return u.tolist(), vecs

def _closed_loop(a):
    ctrl, e = _make_ctrl(a)
    if e:
        return e
    pns = _namespace(compile(a['plant'], '<workbench plant>', 'exec'), a['plantParams'])
    f, h = pns['f'], pns['h']
    lims, Ts, feed = a['uLimit'], a['Ts'], a.get('feed', 'y')
    m = len(lims)
    rs, ds, nz = a['r'], a.get('d'), a.get('noise')
    x = np.array(a['x0'], dtype=float).reshape(-1, 1)
    n = x.shape[0]
    out = {k: [] for k in ('x', 'y', 'ym', 'uD', 'u', 'ua', 'xhat', 'dhat')}
    def F(xx, u):
        return np.asarray(f(xx, u), dtype=float).reshape(n, 1)
    for k in range(len(rs)):
        t = k * Ts
        y = [float(v) for v in np.asarray(h(x), dtype=float).flatten()]
        ym = [v + (nz[k][i] if nz else 0.0) for i, v in enumerate(y)]
        r = rs[k] if not isinstance(rs[k], list) else np.array(rs[k], dtype=float).reshape(-1, 1)
        arg = x.copy() if feed == 'state' else np.array(ym, dtype=float).reshape(-1, 1)
        try:
            ret = ctrl.update(r, arg)
        except Exception as e:
            return _err(e, f' (in update, at t = {t:.3g} s)')
        try:
            uD, ext = _split(ret, m, t)
        except Exception as e:
            return {'error': f'{type(e).__name__}: {e}', 'where': ''}
        us = [min(max(v, lo), hi) for v, (lo, hi) in zip(uD, lims)]
        d = ds[k] if ds else [0.0] * m
        ua = [min(max(v + d[i], lims[i][0]), lims[i][1]) for i, v in enumerate(us)]
        out['x'].append(x[:, 0].tolist()); out['y'].append(y); out['ym'].append(ym)
        out['uD'].append(uD); out['u'].append(us); out['ua'].append(ua)
        out['xhat'].append(ext[0] if len(ext) > 0 else None)
        out['dhat'].append(ext[1] if len(ext) > 1 else None)
        if k == len(rs) - 1:
            break
        uh = ua[0] if m == 1 else np.array(ua, dtype=float).reshape(-1, 1)
        F1 = F(x, uh); F2 = F(x + Ts / 2 * F1, uh); F3 = F(x + Ts / 2 * F2, uh); F4 = F(x + Ts * F3, uh)
        x = x + Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        if not np.all(np.isfinite(x)):
            return {'error': f'The state became inf/NaN at t = {(k + 1) * Ts:.3g} s', 'where': ''}
    return out

# Open-loop probe: a fresh controller fed a fixed sequence of (r, y) pairs; returns u.
def _probe(a):
    ctrl, e = _make_ctrl(a)
    if e:
        return e
    us = []
    for k, (r, y) in enumerate(a['calls']):
        r = r if not isinstance(r, list) else np.array(r, dtype=float).reshape(-1, 1)
        try:
            u, _ = _split(ctrl.update(r, np.array(y, dtype=float).reshape(-1, 1)), a['m'], k * a['Ts'])
        except Exception as e:
            return _err(e, ' (in update)')
        us.append(u)
    return {'u': us}

def wb_evaluate(payload):
    return _run(_evaluate, payload)

def wb_simulate(payload):
    return _run(_simulate, payload)

def wb_closed_loop(payload):
    return _run(_closed_loop, payload)

def wb_probe(payload):
    return _run(_probe, payload)

# A small stand-in for python-control (python-control itself needs scipy):
# ctrb, obsv, and place. place is scipy.signal.place_poles (method 'YT'), which is
# what python-control's place calls, so multi-input and observer gains match the
# repo's ctrl*.py; one input reduces to the unique (Ackermann) gain. For an
# observer, place(A.T, C.T, poles).T as in the book.
def _install_control():
    def _m(A):
        return np.atleast_2d(np.asarray(A, dtype=float))
    def ctrb(A, B):
        A, B = _m(A), _m(B)
        if B.shape[0] != A.shape[0]:
            B = B.T
        cols = [B]
        for _ in range(A.shape[0] - 1):
            cols.append(A @ cols[-1])
        return np.hstack(cols)
    def obsv(A, C):
        return ctrb(_m(A).T, _m(C).T).T
    def qr_full(M):
        return np.linalg.qr(M, mode='complete')
    def order(poles):
        real = sorted(p.real for p in poles if p.imag == 0)
        neg = sorted((p for p in poles if p.imag < 0), key=lambda p: (p.real, p.imag))
        out = [complex(r, 0) for r in real]
        for p in neg:
            out += [p, p.conjugate()]
        if len(out) != len(poles):
            raise ValueError('place: complex poles must come in conjugate pairs')
        return np.array(out)
    def knv0(ker, X, j):
        Q, _ = qr_full(np.delete(X, j, axis=1))
        yj = ker[j] @ ker[j].T @ Q[:, -1]
        if not np.allclose(yj, 0):
            X[:, j] = yj / np.linalg.norm(yj)
    def yt_real(ker, Q, X, i, j):
        u, v = Q[:, -2, None], Q[:, -1, None]
        m = ker[i].T @ (u @ v.T - v @ u.T) @ ker[j]
        um, sm, vm = np.linalg.svd(m)
        mu1, mu2 = um.T[:2, :, None]
        nu1, nu2 = vm[:2, :, None]
        t = np.vstack((X[:, i, None], X[:, j, None]))
        if not np.allclose(sm[0], sm[1]):
            kmn = np.vstack((ker[i] @ mu1, ker[j] @ nu1))
        else:
            kij = np.vstack((np.hstack((ker[i], np.zeros(ker[i].shape))), np.hstack((np.zeros(ker[j].shape), ker[j]))))
            kmn = kij @ np.vstack((np.hstack((mu1, mu2)), np.hstack((nu1, nu2))))
        tij = kmn @ kmn.T @ t
        n = X.shape[0]
        if not np.allclose(tij, 0):
            tij = np.sqrt(2) * tij / np.linalg.norm(tij)
            X[:, i], X[:, j] = tij[:n, 0], tij[n:, 0]
        else:
            X[:, i], X[:, j] = kmn[:n, 0], kmn[n:, 0]
    def yt_complex(ker, Q, X, i, j):
        u = np.sqrt(2) * Q[:, -2, None] + 1j * np.sqrt(2) * Q[:, -1, None]
        K = ker[i]
        m = np.conj(K.T) @ (u @ np.conj(u).T - np.conj(u) @ u.T) @ K
        ev, evec = np.linalg.eig(m)
        idx = np.argsort(np.abs(ev))
        mu1, mu2 = evec[:, idx[-1], None], evec[:, idx[-2], None]
        t = X[:, i, None] + 1j * X[:, j, None]
        km = K @ mu1 if not np.allclose(np.abs(ev[idx[-1]]), np.abs(ev[idx[-2]])) else K @ np.hstack((mu1, mu2))
        tij = km @ np.conj(km.T) @ t
        if not np.allclose(tij, 0):
            tij = tij / np.linalg.norm(tij)
            X[:, i], X[:, j] = np.real(tij[:, 0]), np.imag(tij[:, 0])
        else:
            X[:, i], X[:, j] = np.real(km[:, 0]), np.imag(km[:, 0])
    def update_order(poles):
        nb_real = int(np.sum(np.isreal(poles))); hnb = nb_real // 2
        o0, o1 = [], []
        r_comp = list(range(nb_real + 1, len(poles) + 1, 2))
        def comp():
            o0.extend(r_comp); o1.extend([x + 1 for x in r_comp])
        def knv():
            if hnb == 0 and np.isreal(poles[0]):
                o0.append(1); o1.append(1)
        if nb_real > 0:
            o0.append(nb_real); o1.append(1)
        r_p = range(1, hnb + nb_real % 2)
        o0.extend(2 * x for x in r_p); o1.extend(2 * x + 1 for x in r_p)
        comp()
        r_p = range(1, hnb + 1)
        o0.extend(2 * x - 1 for x in r_p); o1.extend(2 * x for x in r_p)
        knv(); comp()
        for j in range(2, hnb + nb_real % 2):
            for i in range(1, hnb + 1):
                o0.append(i); o1.append(i + j)
        knv(); comp()
        for j in range(2, hnb + nb_real % 2):
            for i in range(hnb + 1, nb_real + 1):
                idx = i + j
                if idx > nb_real:
                    idx = i + j - nb_real
                o0.append(i); o1.append(idx)
        knv(); comp()
        for i in range(1, hnb + 1):
            o0.append(i); o1.append(i + hnb)
        knv(); comp()
        return [(a - 1, b - 1) for a, b in zip(o0, o1)]
    def yt_loop(ker, X, poles, maxiter=30, rtol=1e-3):
        eps = np.sqrt(np.spacing(1))
        for _ in range(maxiter):
            det_b = abs(np.linalg.det(X))
            for i, j in update_order(poles):
                if i == j:
                    knv0(ker, X, i)
                else:
                    Q, _ = qr_full(np.delete(X, (i, j), axis=1))
                    (yt_real if np.isreal(poles[i]) else yt_complex)(ker, Q, X, i, j)
            det = max(eps, abs(np.linalg.det(X)))
            if abs((det - det_b) / det) < rtol and det > eps:
                break
    def place(A, B, p):
        A, B = _m(A), _m(B)
        n = A.shape[0]
        if B.shape[0] != n:
            B = B.T
        p = np.atleast_1d(np.asarray(p, dtype=complex)).flatten()
        if p.size != n:
            raise ValueError(f'place needs {n} poles, got {p.size}')
        if np.linalg.matrix_rank(ctrb(A, B)) < n:
            raise ValueError('place: (A, B) is not controllable')
        rank_b = np.linalg.matrix_rank(B)
        if rank_b < B.shape[1]:
            raise ValueError('place: B must have full column rank')
        poles = order(p)
        if rank_b == n:
            D = np.zeros((n, n)); i = 0
            while i < n:
                D[i, i] = poles[i].real
                if poles[i].imag != 0:
                    D[i, i + 1], D[i + 1, i + 1], D[i + 1, i] = -poles[i].imag, poles[i].real, poles[i].imag
                    i += 1
                i += 1
            return np.linalg.solve(B, A - D)
        U, Z = qr_full(B)
        u0, u1, z = U[:, :rank_b], U[:, rank_b:], Z[:rank_b, :]
        ker, cols, skip = [], [], False
        for j in range(n):
            if skip:
                skip = False
                continue
            ps = (u1.T @ (A - poles[j] * np.eye(n))).T
            Q, _ = qr_full(ps)
            kj = Q[:, ps.shape[1]:]
            tj = np.sum(kj, axis=1)[:, None]
            tj = tj / np.linalg.norm(tj)
            if poles[j].imag != 0:
                cols += [np.real(tj), np.imag(tj)]
                ker += [kj, kj]
                skip = True
            else:
                cols.append(np.real(tj))
                ker.append(np.real(kj))
        X = np.hstack(cols)
        if rank_b > 1:
            yt_loop(ker, X, poles)
        Xc = X.astype(complex); i = 0
        while i < n - 1:
            if poles[i].imag != 0:
                rel, img = X[:, i].copy(), X[:, i + 1].copy()
                Xc[:, i], Xc[:, i + 1] = rel - 1j * img, rel + 1j * img
                i += 1
            i += 1
        mm = np.linalg.solve(Xc.T, np.diag(poles) @ Xc.T).T
        return -np.real(np.linalg.solve(z, u0.T @ (mm - A)))
    mod = types.ModuleType('control')
    mod.ctrb, mod.obsv, mod.place = ctrb, obsv, place
    sys.modules['control'] = mod

_install_control()
`;

  // Runs inside the worker (stringified; it can't see this file's closure).
  function workerMain(BASE, HASH, HARNESS) {
    // This is a classic worker (see start()). Pyodide refuses those, detecting
    // them by whether importScripts works; it needs only fetch and import(),
    // which classic workers have, so disable importScripts first.
    self.importScripts = () => { throw new TypeError('importScripts is disabled'); };
    const realFetch = self.fetch.bind(self);
    // The loader fetches the wasm and stdlib itself: add their pinned hashes.
    self.fetch = (url, opts = {}) => {
      const name = String(url && url.url ? url.url : url).replace(BASE, '');
      return realFetch(url, HASH[name] ? { ...opts, integrity: HASH[name] } : opts);
    };
    const pinned = async (name) => {
      const r = await realFetch(BASE + name, { integrity: HASH[name] });
      if (!r.ok) throw new Error(`${name}: HTTP ${r.status}`);
      return r;
    };
    function lockDown() {
      const names = ['fetch', 'XMLHttpRequest', 'WebSocket', 'WebSocketStream', 'EventSource', 'importScripts',
        'indexedDB', 'caches', 'BroadcastChannel', 'Worker', 'SharedWorker', 'WebTransport', 'RTCPeerConnection'];
      for (let o = self; o; o = Object.getPrototypeOf(o)) {
        for (const n of names) {
          try { if (Object.prototype.hasOwnProperty.call(o, n)) delete o[n]; } catch (e) { /* not configurable */ }
          try { if (Object.prototype.hasOwnProperty.call(o, n)) o[n] = undefined; } catch (e) { /* read-only */ }
        }
      }
    }
    let py = null;
    const ready = (async () => {
      // Modules are imported from Blob URLs of pinned text (import() can't take an integrity hash).
      const [lock, loader, asm] = await Promise.all(['pyodide-lock.json', 'pyodide.mjs', 'pyodide.asm.mjs'].map((n) => pinned(n)))
        .then(([a, b, c]) => Promise.all([a.json(), b.text(), c.text()]));
      const blobUrl = (text) => URL.createObjectURL(new Blob([text], { type: 'text/javascript' }));
      const { loadPyodide } = await import(blobUrl(loader));
      const createPyodideModule = (await import(blobUrl(asm))).default;
      py = await loadPyodide({ indexURL: BASE, packageBaseUrl: BASE, lockFileContents: lock, createPyodideModule, stdout: () => {}, stderr: () => {} });
      await py.loadPackage('numpy', { messageCallback: () => {}, errorCallback: () => {} });
      py.runPython(HARNESS);
      lockDown();
      self.postMessage({ type: 'ready' });
    })().catch((e) => self.postMessage({ type: 'failed', msg: String(e && e.message || e) }));
    self.onmessage = async (ev) => {
      const { id, op, payload } = ev.data;
      try {
        await ready;
        const out = py.globals.get(`wb_${op}`)(payload);
        self.postMessage({ id, out });
      } catch (e) {
        self.postMessage({ id, out: JSON.stringify({ error: String(e && e.message || e), where: '' }) });
      }
    };
  }

  let worker = null, readyP = null, state = 'idle', failMsg = '';
  let seq = 0;
  const pending = new Map();
  const listeners = new Set();
  const setState = (s) => { state = s; for (const fn of listeners) fn(s); };

  function start() {
    if (readyP) return readyP;
    setState('loading');
    readyP = (async () => {
      // A classic worker: Chrome won't start a module worker from a Blob URL on a
      // page opened from disk (file://), which is how the workbench is usually used.
      const src = `(${workerMain.toString()})(${JSON.stringify(BASE)}, ${JSON.stringify(HASH)}, ${JSON.stringify(HARNESS)});`;
      const url = URL.createObjectURL(new Blob([src], { type: 'text/javascript' }));
      worker = new Worker(url);
      await new Promise((resolve, reject) => {
        // A blocked download can leave Pyodide waiting forever.
        setTimeout(() => reject(new Error('timed out while loading')), 120000);
        worker.onmessage = (ev) => {
          const m = ev.data || {};
          if (m.type === 'ready') { resolve(); return; }
          if (m.type === 'failed') { reject(new Error(m.msg)); return; }
          const p = pending.get(m.id);
          if (p) { pending.delete(m.id); p.resolve(m.out); }
        };
        worker.onerror = (e) => reject(new Error(e.message || 'the worker failed to start'));
      });
      setState('ready');
    })().catch((e) => {
      failMsg = String(e.message || e);
      stop();
      setState('error');
      throw new Error(`Python could not start (${failMsg}). It downloads from cdn.jsdelivr.net, so check your connection.`);
    });
    return readyP;
  }

  function stop() {
    if (worker) worker.terminate();
    worker = null; readyP = null;
    for (const p of pending.values()) p.reject(new Error('stopped'));
    pending.clear();
  }

  // Send one request; resolves with the parsed JSON reply. The timer starts once
  // Pyodide is ready, so the first download doesn't count against the limit.
  async function call(op, payload, timeoutMs) {
    await start();
    const id = ++seq;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        pending.delete(id);
        stop();
        setState('idle');
        resolve({ error: `Stopped after ${timeoutMs / 1000} s. Is there an infinite loop?`, where: '', timeout: true });
      }, timeoutMs);
      pending.set(id, {
        resolve: (out) => { clearTimeout(timer); try { resolve(JSON.parse(out)); } catch (e) { reject(e); } },
        reject: (e) => { clearTimeout(timer); reject(e); },
      });
      worker.postMessage({ id, op, payload: JSON.stringify(payload) });
    });
  }

  const evaluate = (code, samples, timeoutMs = 5000) => call('evaluate', { code, samples }, timeoutMs);
  const simulate = (code, opts, timeoutMs = 15000) => call('simulate', { code, ...opts }, timeoutMs);
  // A student's class Controller in closed loop with the workbench's plant (see
  // _closed_loop above; WB.myCtrl builds the payload), and an open-loop probe.
  const closedLoop = (code, opts, timeoutMs = 30000) => call('closed_loop', { code, ...opts }, timeoutMs);
  const probe = (code, opts, timeoutMs = 10000) => call('probe', { code, ...opts }, timeoutMs);

  // ------------------------------------------------------------ checking --
  // Deterministic pseudo-random numbers, so a failing case repeats.
  function rng(seed) {
    let s = seed >>> 0;
    return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  }
  const flat = (v) => (Array.isArray(v) ? v.flatMap(flat) : [v]);
  // Non-finite values arrive as 'inf', '-inf' or 'nan' (see _out).
  const real = (v) => (typeof v === 'string' ? Number(v.replace('inf', 'Infinity').replace('nan', 'NaN')) : v);
  const cplx = (v) => (v && typeof v === 'object' ? [v.re, v.im] : [real(v), 0]);
  const shapeOf = (v) => (Array.isArray(v) ? [v.length, ...shapeOf(v[0])] : []);
  const fmtV = (v) => {
    const [re, im] = cplx(v);
    if (Math.abs(im) < 1e-12 * Math.max(1, Math.abs(re))) return WB.math.fmt(re, 4);
    return `${WB.math.fmt(re, 4)}${im < 0 ? '−' : '+'}${WB.math.fmt(Math.abs(im), 4)}j`;
  };
  const fmtArr = (v) => (Array.isArray(v) ? `[${v.map(fmtArr).join(', ')}]` : fmtV(v));

  // Parameter sets for a check: the nominal (left-panel) values first, then
  // `sets` random ones with each `vary` parameter scaled by 0.5–1.5.
  function paramSets(ctx, { sets = 4, vary, params, seed = 12345 } = {}) {
    const sys = ctx.sys, rand = rng(seed);
    vary = vary || [...(sys.uncertain || []), ...Object.keys(sys.constants || {})];
    const base = { ...ctx.pModel };
    const out = [{ p: base, nominal: true }];
    for (let i = 0; i < sets; i++) {
      const p = { ...base };
      for (const k of vary) p[k] = base[k] * (0.5 + rand());
      out.push({ p, nominal: false });
    }
    for (const s of out) if (params) s.p = params(s.p);
    out.vary = vary;
    out.text = (s) => (s.nominal ? 'the nominal parameters' : vary.map((k) => `${k} = ${WB.math.fmt(s.p[k], 3)}`).join(', '));
    return out;
  }

  // spec = {
  //   args: { name: {label, lo, hi} | {label, complex: true, re: [lo, hi], im: [lo, hi]} | {col: [names]} },
  //   vary: ['m', ...]       parameters to randomize (default: sys.uncertain + constants)
  //   params: (p) => p       optional: add derived entries to each parameter set
  //   cases: [{label, fix: {name: value}}]   argument groups (default: one random group)
  //   items: [{fn, args: [names], truth(p, a), compare?: 'equal'|'offset'|'scale', label?}
  //           | {var, truth(p), label?}]
  //   perCase: points per case (default 4), sets: random parameter sets (default 4)
  //   explain?(item, failure) -> string   extra hint appended to a mismatch message (failure.e = {got, want, a})
  // }
  // Resolves to {ok, msg, detail?}.
  async function check(ctx, spec, code) {
    if (!code.trim()) return { ok: false, msg: 'Write your code first.' };
    const rand = rng(777);
    const paramSetsList = paramSets(ctx, spec);
    const vary = paramSetsList.vary;
    const draw = (def) => {
      if (def.complex) return { re: def.re[0] + rand() * (def.re[1] - def.re[0]), im: def.im[0] + rand() * (def.im[1] - def.im[0]) };
      return def.lo + rand() * (def.hi - def.lo);
    };
    const cases = spec.cases || [{ label: '' }];
    const n = spec.perCase ?? 4;
    // points[s] = [{caseIdx, a: {name: value}}]
    const points = paramSetsList.map(() => {
      const pts = [];
      cases.forEach((c, ci) => {
        for (let k = 0; k < n; k++) {
          const a = {};
          for (const [name, def] of Object.entries(spec.args || {})) if (!def.col) a[name] = name in (c.fix || {}) ? c.fix[name] : draw(def);
          pts.push({ ci, a });
        }
      });
      return pts;
    });
    const fns = spec.items.filter((it) => it.fn), vars = spec.items.filter((it) => it.var);
    const argVal = (name, a) => {
      const def = spec.args[name];
      if (def && def.col) return { col: def.col.map((c) => a[c]) };
      return a[name];
    };
    const samples = paramSetsList.map((s, si) => ({
      params: s.p,
      vars: vars.map((it) => it.var),
      calls: fns.length ? points[si].flatMap(({ a }) => fns.map((it) => ({ name: it.fn, args: it.args.map((nm) => argVal(nm, a)) }))) : [],
    }));
    const out = await evaluate(code, samples);
    const detail = (out.stdout || '').trim() ? `print output:\n${out.stdout.trim()}` : '';
    if (out.error) return { ok: false, msg: out.timeout ? out.error : 'Python raised an error.', detail: [out.error, out.where].filter(Boolean).join('\n') + (detail ? `\n\n${detail}` : '') };

    const paramText = paramSetsList.text;
    const argText = (it, a) => it.args.map((nm) => {
      const def = spec.args[nm];
      if (def.col) return `${nm} = [${def.col.map((c) => `${spec.args[c].label || c} ${fmtV(a[c])}`).join(', ')}]`;
      return `${def.label || nm} = ${fmtV(a[nm])}`;
    }).join(', ');

    // Gather (yours, expected) per item per parameter set.
    const failures = [];
    spec.items.forEach((it) => {
      paramSetsList.forEach((s, si) => {
        const row = out.rows[si];
        let entries;
        if (it.var) entries = [{ got: row.vars[it.var], want: it.truth(s.p), a: null, ci: -1 }];
        else {
          const fi = fns.indexOf(it);
          entries = points[si].map((pt, k) => ({ got: row.calls[k * fns.length + fi], want: it.truth(s.p, pt.a), a: pt.a, ci: pt.ci }));
        }
        const wantShape = shapeOf(entries[0].want), gotShape = shapeOf(entries[0].got);
        const nW = flat(entries[0].want).length, nG = flat(entries[0].got).length;
        if (nW !== nG) {
          failures.push({ it, s, kind: 'shape', msg: `${it.label || it.fn || it.var} has ${nG} value${nG === 1 ? '' : 's'}${gotShape.length ? ` (shape ${gotShape.join('×')})` : ''}; expected ${nW}${wantShape.length ? ` (shape ${wantShape.join('×')})` : ''}.` });
          return;
        }
        const G = entries.map((e) => flat(e.got).map(cplx)), W = entries.map((e) => flat(e.want).map(cplx));
        let scale = 0;
        for (const w of W) for (const [re, im] of w) if (Number.isFinite(re) && Number.isFinite(im)) scale = Math.max(scale, Math.hypot(re, im));
        const tol = 1e-6 * Math.max(scale, 1e-9) + 1e-12;
        let c = [1, 0];
        const mode = it.compare || 'equal';
        if (mode === 'offset') {
          const g0 = G[0], w0 = W[0];
          for (let k = 0; k < G.length; k++) { G[k] = G[k].map(([re, im], j) => [re - g0[j][0], im - g0[j][1]]); W[k] = W[k].map(([re, im], j) => [re - w0[j][0], im - w0[j][1]]); }
        } else if (mode === 'scale') {
          // least-squares complex ratio c = <w, g> / <w, w>
          let nr = 0, ni = 0, dd = 0;
          for (let k = 0; k < G.length; k++) for (let j = 0; j < G[k].length; j++) {
            const [gr, gi] = G[k][j], [wr, wi] = W[k][j];
            if (![gr, gi, wr, wi].every(Number.isFinite)) continue;
            nr += wr * gr + wi * gi; ni += wr * gi - wi * gr; dd += wr * wr + wi * wi;
          }
          c = dd > 0 ? [nr / dd, ni / dd] : [0, 0];
          if (Math.hypot(c[0], c[1]) < 1e-9) c = [1, 0];
        }
        for (let k = 0; k < G.length; k++) {
          for (let j = 0; j < G[k].length; j++) {
            const [gr, gi] = G[k][j], [wr, wi] = W[k][j];
            // An infinite expected value (an unbounded error, say) must be matched exactly.
            if (![gr, gi, wr, wi].every(Number.isFinite)) {
              if (!(gr === wr && gi === wi)) { failures.push({ it, s, e: entries[k], j, kind: 'value' }); return; }
              continue;
            }
            const er = gr - (c[0] * wr - c[1] * wi), ei = gi - (c[0] * wi + c[1] * wr);
            if (!(Math.hypot(er, ei) <= tol * Math.max(1, Math.hypot(c[0], c[1])))) {
              failures.push({ it, s, e: entries[k], j, kind: 'value' });
              return;
            }
          }
        }
      });
    });
    if (!failures.length) {
      const nPts = spec.items.some((it) => it.fn) ? `${cases.length * n} points × ` : '';
      return { ok: true, msg: `Matches at ${nPts}${paramSetsList.length} parameter sets.`, detail };
    }
    const f = failures.find((x) => x.s.nominal) || failures[0];
    const nominalOk = !failures.some((x) => x.s.nominal);
    if (f.kind === 'shape') return { ok: false, msg: f.msg, detail };
    const name = f.it.label || f.it.fn || f.it.var;
    let msg;
    if (nominalOk && vary.length) {
      msg = `${name} matches at the nominal parameters but not with ${paramText(f.s)}. Write it with ${vary.map((k) => `P.${k}`).join(', ')} rather than numbers.`;
    } else {
      const where = f.e.a ? `${f.it.fn}(${argText(f.it, f.e.a)})` : name;
      const cs = f.e.ci >= 0 && cases[f.e.ci].label ? ` ${cases[f.e.ci].label}` : '';
      const one = flat(f.e.want).length === 1;
      const vals = one
        ? `yours ${fmtV(flat(f.e.got)[0])}, expected ${fmtV(flat(f.e.want)[0])}`
        : `yours ${fmtArr(f.e.got)}, expected ${fmtArr(f.e.want)}`;
      msg = `${where}${cs}, with ${paramText(f.s)}: ${vals}.`;
      if (f.it.compare === 'offset') msg += ' (Compared up to a constant.)';
      if (f.it.compare === 'scale') msg += ' (Any nonzero multiple is accepted.)';
      if (spec.explain) { const h = spec.explain(f.it, f); if (h) msg += ` ${h}`; }
    }
    return { ok: false, msg, detail };
  }

  // Complex arithmetic for truth functions of s ({re, im} objects; numbers are real).
  const cx = {
    of: (v) => (typeof v === 'number' ? { re: v, im: 0 } : v),
    add: (a, b) => { a = cx.of(a); b = cx.of(b); return { re: a.re + b.re, im: a.im + b.im }; },
    mul: (a, b) => { a = cx.of(a); b = cx.of(b); return { re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re }; },
    div: (a, b) => { a = cx.of(a); b = cx.of(b); const d = b.re * b.re + b.im * b.im; return { re: (a.re * b.re + a.im * b.im) / d, im: (a.im * b.re - a.re * b.im) / d }; },
    // coefficients highest power first, like numpy.polyval
    poly: (c, s) => c.reduce((acc, ci) => cx.add(cx.mul(acc, s), ci), { re: 0, im: 0 }),
  };

  return { start, stop, evaluate, simulate, closedLoop, probe, check, paramSets, cx, status: () => state, onStatus: (fn) => listeners.add(fn), BASE, VERSION };
})();
