#!/usr/bin/env python3
"""Lens bars: zoom and focus adjustment bars with the live camera image.

Runs next to tools/lens_ui (which owns the MCU console port and keeps the
latest camera frame); every motor command goes through lens_ui's
/api/do "raw" endpoint.

    python3 tools/lens_bars/lens_bars.py [--http 8091] [--lens-ui http://localhost:8090]

Positions are microsteps from the reverse stop of each axis (16 per full step).
"Record position" appends zoom, focus and sharpness to positions.json here.
"""
import argparse, json, os, re, threading, time, urllib.request
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

import cv2
import numpy as np
import sys

HERE0 = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE0, ".."))
import focus_cal as fc        # goto / frame / sharp / wait_idle (tools/focus_cal.py)

HERE = os.path.dirname(os.path.abspath(__file__))
LIVE = os.path.join(HERE, "..", "lens_ui", ".live.jpg")
REC = os.path.join(HERE, "positions.json")
LENS_UI = "http://localhost:8090"
LINE = re.compile(r"^(zoom|focus)\s+(\S+)\s+(\S+)\s+pos\s+(-?\d+)\s+\(.*?\)\s+(\S+)\s+(\S+)\s+lim\s+(\S+)")
_lock = threading.Lock()


def raw(cmd):
    req = urllib.request.Request(LENS_UI + "/api/do", json.dumps({"action": "raw", "cmd": cmd}).encode(),
                                 {"Content-Type": "application/json"})
    with _lock, urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r).get("msg", "")


def sharpness():
    """Mean Sobel energy on the centre 1/3 of the frame (same as focus_cal.py)."""
    try:
        if time.time() - os.path.getmtime(LIVE) > 3:
            return None
        im = cv2.imread(LIVE, cv2.IMREAD_GRAYSCALE)
    except OSError:
        return None
    if im is None:
        return None
    h, w = im.shape
    c = im[h // 3: 2 * h // 3, w // 3: 2 * w // 3].astype(np.float32)
    gx = cv2.Sobel(c, cv2.CV_32F, 1, 0); gy = cv2.Sobel(c, cv2.CV_32F, 0, 1)
    return float(np.mean(gx * gx + gy * gy))


def status():
    txt = raw("lens")
    out = {"locked": None}
    for line in txt.splitlines():
        m = LINE.match(line.strip())
        if m:
            ax = m.group(1)
            out[ax] = {"drv": m.group(2), "cfg": m.group(3), "pos": int(m.group(4)),
                       "moving": m.group(5) in ("MOVING", "HOMING"), "state": m.group(5),
                       "ref": m.group(6) == "ref", "lim": m.group(7)}
        if "LOCKED at" in line:
            out["locked"] = int(line.split("LOCKED at")[1].split()[0])
        elif "unlocked, fixed pos" in line:
            out["locked"] = None
    return out


def record(note):
    st = status()
    entry = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "zoom": st["zoom"]["pos"],
             "focus": st["focus"]["pos"], "sharpness": sharpness(), "note": note}
    allr = json.load(open(REC)) if os.path.exists(REC) else []
    allr.append(entry)
    json.dump(allr, open(REC, "w"), indent=1)
    return entry


# ---------------------------------------------------------------- autofocus --
class Abort(Exception):
    pass


AF = {"running": False, "mode": "", "phase": "", "pts": [], "best": None, "msg": "", "cancel": False}


def af_idle(timeout=180):
    """Wait until neither zoom nor focus is moving (zoom moves change the image)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if AF["cancel"]:
            raise Abort()
        st = status()
        if not st["zoom"]["moving"] and not st["focus"]["moving"]:
            return
        time.sleep(0.5)
    raise RuntimeError("motors still moving")


def af_goto(p):
    for attempt in range(3):
        try:
            return fc.goto(p)
        except RuntimeError as e:
            if "busy" not in str(e) or attempt == 2:
                raise
            af_idle()


def af_measure(p):
    if AF["cancel"]:
        raise Abort()
    if AF.get("zoom0") is not None and status()["zoom"]["pos"] != AF["zoom0"]:
        raise RuntimeError(f"zoom changed during autofocus (was {AF['zoom0']}, wanted unchanged)")
    t = af_goto(p)
    s = fc.sharp(fc.frame(t))
    AF["pts"].append([p, round(s, 1)])
    return s


def af_rehome():
    AF["phase"] = "homing focus (reverse stop, ~30 s)"
    fc.cmd("lens f home")
    fc.wait_idle(90)


def af_run(mode, fmax):
    """full: home, coarse 0..fmax/1000, re-home, fine +-1500/100.
       local: +-3000/500 around the current focus, then +-500/50.
       Every point is reached upward (long moves down lose steps)."""
    AF.update(running=True, mode=mode, cancel=False, pts=[], best=None, msg="", phase="waiting for motors to stop")
    try:
        af_idle()
        AF["zoom0"] = status()["zoom"]["pos"]
        if mode == "full":
            af_rehome()
            AF["phase"] = f"coarse 0..{fmax} every 1000"
            c = [(p, af_measure(p)) for p in range(0, fmax + 1, 1000)]
            pk = max(c, key=lambda r: r[1])[0]
            lo, hi, step = max(0, pk - 1500), min(fmax, pk + 1500), 100
            af_rehome()
        else:
            cur = fc.focus_state()[0]
            lo, hi = max(0, cur - 3000), min(fmax, cur + 3000)
            AF["phase"] = f"local coarse {lo}..{hi} every 500"
            c = [(p, af_measure(p)) for p in range(lo, hi + 1, 500)]
            pk = max(c, key=lambda r: r[1])[0]
            lo, hi, step = max(0, pk - 500), min(fmax, pk + 500), 50
        AF["phase"] = f"fine {lo}..{hi} every {step}"
        f = [(p, af_measure(p)) for p in range(lo, hi + 1, step)]
        best, bs = max(f, key=lambda r: r[1])
        AF["phase"] = f"going to best {best}"
        if mode == "full":
            af_rehome()
        t = af_goto(best)
        now = fc.sharp(fc.frame(t))
        AF["best"] = {"focus": best, "sharp": round(bs, 1), "check": round(now, 1)}
        AF["msg"] = f"best focus {best}: sharpness {bs:.0f} (re-measured {now:.0f})"
        AF["phase"] = "done"
    except Abort:
        fc.cmd("lens f stop")
        AF.update(phase="cancelled", msg="autofocus cancelled")
    except Exception as e:
        m = str(e)
        if "wanted" in m:
            m = f"stopped: the lens was moved by another command ({m}). Don't move zoom or focus during autofocus."
        AF.update(phase="error", msg=m)
    finally:
        AF["running"] = False


def handler():
    page = os.path.join(HERE, "index.html")

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, code, body, ctype="application/json"):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = urlparse(self.path).path
            try:
                if p in ("/", "/index.html"):
                    return self.send(200, open(page, "rb").read(), "text/html; charset=utf-8")
                if p == "/live.jpg":
                    return self.send(200, open(LIVE, "rb").read(), "image/jpeg")
                if p == "/api/state":
                    st = status()
                    st["sharp"] = sharpness()
                    st["af"] = {k: v for k, v in AF.items() if k != "cancel"}
                    return self.send(200, st)
                if p == "/api/records":
                    return self.send(200, json.load(open(REC)) if os.path.exists(REC) else [])
            except Exception as e:
                return self.send(500, {"error": str(e)})
            self.send(404, {"error": "not found"})

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                d = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self.send(400, {"ok": False, "msg": "bad json"})
            p = urlparse(self.path).path
            try:
                if p == "/api/cmd":
                    c = str(d.get("cmd", ""))
                    if not re.fullmatch(r"(lens( [a-z0-9 \-]+)?|ir cut (day|night))", c):
                        return self.send(400, {"ok": False, "msg": "only lens / ir cut commands"})
                    msg = raw(c)
                    return self.send(200, {"ok": "ERROR" not in msg, "msg": msg})
                if p == "/api/af/start":
                    if AF["running"]:
                        return self.send(409, {"ok": False, "msg": "autofocus already running"})
                    mode = "full" if d.get("mode") == "full" else "local"
                    fmax = max(1000, min(int(d.get("max", 32400)), 32400))
                    threading.Thread(target=af_run, args=(mode, fmax), daemon=True).start()
                    return self.send(200, {"ok": True, "msg": f"autofocus ({mode}) started"})
                if p == "/api/af/cancel":
                    AF["cancel"] = True
                    return self.send(200, {"ok": True, "msg": "cancelling"})
                if p == "/api/record":
                    return self.send(200, {"ok": True, "entry": record(str(d.get("note", ""))[:200])})
            except Exception as e:
                return self.send(500, {"ok": False, "msg": str(e)})
            self.send(404, {"error": "not found"})

    return H


def main():
    global LENS_UI
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--http", type=int, default=8091)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--lens-ui", default=LENS_UI)
    a = ap.parse_args()
    LENS_UI = a.lens_ui.rstrip("/")
    srv = ThreadingHTTPServer((a.bind, a.http), handler())
    print(f"lens bars on http://{a.bind}:{a.http}/  (via {LENS_UI})")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
