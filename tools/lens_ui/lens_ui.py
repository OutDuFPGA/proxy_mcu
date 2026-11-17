#!/usr/bin/env python3
"""Lens stage UI: go to zoom stages and set focus per stage over the MCU console (J10).

Every move follows the configurable movement rules in lens_ui.json:
  - zoom and focus moves are split into chunks of at most zoom_chunk / focus_chunk microsteps
  - zoom is approached from below (or above / any) after backing off zoom_backoff
  - focus ends with a downward (or upward / any) approach from focus_overshoot away
  - optional focus re-home before each stage
Each chunk is checked: if an axis stops short of the commanded position the move aborts.

    python3 tools/lens_ui/lens_ui.py [--serial /dev/ttyUSB1] [--http 8090] [--bind 0.0.0.0]
        [--rtsp rtsp://192.168.10.54:554/live1.sdp]

Then open http://<this-pc>:8090/ . Close other programs that use the console port first.
"""
import argparse, json, os, re, subprocess, threading, time, traceback
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(HERE, "lens_ui.json")
LIVE = os.path.join(HERE, ".live.jpg")

DEFAULT_CFG = {
    "rules": {
        "zoom_chunk": 100, "focus_chunk": 500,
        "zoom_approach": "below", "zoom_backoff": 300,
        "focus_approach": "down", "focus_overshoot": 300,
        "rehome_focus_before_stage": False,
        "zoom_min": 0, "zoom_max": 6200,
        "focus_min": 0, "focus_max": 44200,
        "settle_ms": 0,
    },
    "session": {
        "apply_on_connect": True,
        "focus_allow": True,
        "focus_limit": [0, 140000],
        "focus_speed": [100, 150, 300],
    },
    "stages": [
        {"name": "Stage 1", "zoom": 0, "focus": 30500},
        {"name": "Stage 2", "zoom": 886, "focus": 32198},
        {"name": "Stage 3", "zoom": 1771, "focus": 35993},
        {"name": "Stage 4", "zoom": 2657, "focus": 38892},
        {"name": "Stage 5", "zoom": 3543, "focus": 40690},
        {"name": "Stage 6", "zoom": 4429, "focus": 42237},
        {"name": "Stage 7", "zoom": 5314, "focus": 42643},
        {"name": "Stage 8", "zoom": 6200, "focus": 43750},
    ],
}


def load_cfg():
    cfg = json.loads(json.dumps(DEFAULT_CFG))
    if os.path.exists(CFG_PATH):
        with open(CFG_PATH) as f:
            saved = json.load(f)
        for k in ("rules", "session"):
            cfg[k].update(saved.get(k, {}))
        if saved.get("stages"):
            cfg["stages"] = saved["stages"]
    return cfg


def save_cfg(cfg):
    tmp = CFG_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cfg, f, indent=2)
    os.replace(tmp, CFG_PATH)


# ------------------------------------------------------------------ console --
class Console:
    STATUS_RE = re.compile(r"^(zoom|focus)\s+(\w+)\s+pos (-?\d+) \(.*?\)\s+(\w+).*?fault (\S+)\s+warn (\S+)", re.M)

    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.05)
        self.lock = threading.Lock()
        self.s.write(b"\r\n"); time.sleep(0.3); self.s.reset_input_buffer()

    def cmd(self, c, wait=0.6):
        with self.lock:
            self.s.reset_input_buffer(); self.s.write((c + "\r\n").encode())
            t0 = time.time(); out = b""
            while time.time() < t0 + wait:
                out += self.s.read(4096)
                if out.rstrip().endswith(b"MCU>") and time.time() > t0 + 0.15:
                    break
        return out.decode(errors="replace").replace("MCU>", "").strip()

    def status(self):
        r = self.cmd("lens status", 0.8)
        st = {}
        for m in self.STATUS_RE.finditer(r):
            st[m.group(1)] = {"state": m.group(2), "pos": int(m.group(3)), "motion": m.group(4),
                              "fault": m.group(5), "warn": m.group(6)}
        return st


# ------------------------------------------------------------------- camera --
class Camera:
    """ffmpeg keeps the latest 960x540 frame in LIVE; sharpness = mean Sobel gx^2+gy^2."""
    def __init__(self, rtsp):
        self.rtsp, self.proc, self.sharp, self.mtime = rtsp, None, None, 0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            import cv2, numpy as np
        except ImportError:
            cv2 = None
        while True:
            if self.proc is None or self.proc.poll() is not None:
                self.proc = subprocess.Popen(
                    ["ffmpeg", "-loglevel", "quiet", "-rtsp_transport", "tcp",
                     "-fflags", "nobuffer", "-flags", "low_delay",   # no backlog: frames lagged 2.3 s after 8 h
                     "-i", self.rtsp,
                     "-vf", "fps=4,scale=960:540", "-q:v", "4", "-f", "image2", "-update", "1",
                     "-atomic_writing", "1", "-y", LIVE],
                    stdin=subprocess.DEVNULL)
            try:
                mt = os.path.getmtime(LIVE)
                if cv2 is not None and mt != self.mtime:
                    self.mtime = mt
                    im = cv2.imread(LIVE, cv2.IMREAD_GRAYSCALE)
                    if im is not None:
                        im = im.astype(np.float32)
                        gx = cv2.Sobel(im, cv2.CV_32F, 1, 0); gy = cv2.Sobel(im, cv2.CV_32F, 0, 1)
                        self.sharp = float(np.mean(gx * gx + gy * gy))
            except FileNotFoundError:
                pass
            time.sleep(0.2)

    def fresh(self):
        return self.mtime and time.time() - self.mtime < 3


# -------------------------------------------------------------------- mover --
class Abort(Exception):
    pass


class Lens:
    NAME = {"z": "zoom", "f": "focus"}

    def __init__(self, con, cfg):
        self.con, self.cfg = con, cfg
        self.st = {}
        self.busy = None          # description of the running job
        self.stop_req = False
        self.log = []
        self.current_stage = None
        self.lock = threading.Lock()

    def L(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        self.log.append(line); del self.log[:-300]
        print(line, flush=True)

    # --- primitives
    def refresh(self):
        st = self.con.status()
        if st:
            self.st = st
        return st

    def pos(self, ax):
        return self.refresh()[self.NAME[ax]]["pos"]

    def wait_idle(self, timeout=120):
        t = time.time() + timeout
        while time.time() < t:
            st = self.refresh()
            if st and all(v["motion"] == "idle" and v["state"] != "HOMING" for v in st.values()):
                return st
            time.sleep(0.2)
        raise Abort("timeout waiting for the lens to stop")

    def check_stop(self):
        if self.stop_req:
            raise Abort("stopped by user")

    def chunked(self, ax, target):
        """Move ax to target in chunks no larger than the rule allows, checking each chunk."""
        r = self.cfg["rules"]; name = self.NAME[ax]
        k = max(1, int(r["zoom_chunk" if ax == "z" else "focus_chunk"]))
        cur = self.pos(ax)
        while cur != target:
            self.check_stop()
            nxt = cur + max(-k, min(k, target - cur))
            rep = self.con.cmd(f"lens {ax} goto {nxt}")
            if "REJECTED" in rep or "ERROR" in rep:
                raise Abort(rep.replace("\r", " ").replace("\n", " "))
            cur = self.wait_idle()[name]["pos"]
            if cur != nxt:
                raise Abort(f"{name} stopped at {cur}, wanted {nxt}")
        if r.get("settle_ms"):
            time.sleep(r["settle_ms"] / 1000)

    def clamp(self, ax, v):
        r = self.cfg["rules"]; lo, hi = (r["zoom_min"], r["zoom_max"]) if ax == "z" else (r["focus_min"], r["focus_max"])
        return max(lo, min(hi, int(v)))

    def move(self, ax, target):
        """Move with the approach rule for this axis."""
        r = self.cfg["rules"]; name = self.NAME[ax]
        target = self.clamp(ax, target)
        cur = self.pos(ax)
        if ax == "z":
            mode, off = r["zoom_approach"], int(r["zoom_backoff"])
            lo_dir, hi_dir = "below", "above"
        else:
            mode, off = r["focus_approach"], int(r["focus_overshoot"])
            lo_dir, hi_dir = "up", "down"     # "up" = final move upward (from below), "down" = from above
        lo, hi = (r["zoom_min"], r["zoom_max"]) if ax == "z" else (r["focus_min"], r["focus_max"])
        self.L(f"{name} {cur} -> {target} (approach {mode})")
        if mode == lo_dir and target < cur:
            pre = max(lo, target - off) if ax == "z" else max(lo, target - off)
            self.chunked(ax, pre)
        elif mode == hi_dir and target > cur:
            self.chunked(ax, min(hi, target + off) if ax == "f" else min(hi, target + off))
        self.chunked(ax, target)
        self.L(f"{name} at {self.pos(ax)}")

    def home(self, ax):
        name = self.NAME[ax]
        self.L(f"homing {name}: down to 0 in chunks, then reference")
        if self.pos(ax) > 0:
            self.chunked(ax, 0)
        self.check_stop()
        rep = self.con.cmd(f"lens {ax} home")
        if "REJECTED" in rep or "ERROR" in rep:
            raise Abort(rep)
        time.sleep(0.5)
        st = self.wait_idle(240)
        self.L(f"{name} homed: {st[name]['state']} pos {st[name]['pos']}")

    def apply_session(self):
        s = self.cfg["session"]
        cmds = []
        if s.get("focus_allow"):
            cmds.append("lens f allow on")
        lo, hi = s["focus_limit"]; cmds.append(f"lens f limit {lo} {hi}")
        a, b, c = s["focus_speed"]; cmds.append(f"lens f speed {a} {b} {c}")
        for c in cmds:
            rep = self.con.cmd(c)
            self.L(f"{c}: {rep.splitlines()[-1] if rep else '(no reply)'}")

    # --- jobs
    def run(self, desc, fn):
        with self.lock:
            if self.busy:
                return False, f"busy: {self.busy}"
            self.busy, self.stop_req = desc, False
        def work():
            try:
                fn()
                self.L(f"done: {desc}")
            except Abort as e:
                self.L(f"ABORTED {desc}: {e}")
            except Exception as e:
                self.L(f"ERROR {desc}: {e}"); traceback.print_exc()
            finally:
                self.busy = None
        threading.Thread(target=work, daemon=True).start()
        return True, "started"

    def go_stage(self, i):
        s = self.cfg["stages"][i]
        self.current_stage = None
        self.L(f"MOVING to {s['name']} (zoom {s['zoom']}, focus {s['focus']})")
        if self.cfg["rules"].get("rehome_focus_before_stage"):
            self.home("f")
        self.move("z", s["zoom"])
        self.move("f", s["focus"])
        self.current_stage = i
        self.L(f"AT {s['name']}")

    def stop(self):
        self.stop_req = True
        self.con.cmd("lens stop")
        self.L("stop requested")


# --------------------------------------------------------------------- http --
def make_handler(lens, cam, cfg):
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
            self.end_headers(); self.wfile.write(body)

        def do_GET(self):
            p = urlparse(self.path).path
            if p in ("/", "/index.html"):
                with open(page, "rb") as f:
                    return self.send(200, f.read(), "text/html; charset=utf-8")
            if p == "/live.jpg":
                try:
                    with open(LIVE, "rb") as f:
                        return self.send(200, f.read(), "image/jpeg")
                except FileNotFoundError:
                    return self.send(404, b"no frame", "text/plain")
            if p == "/api/state":
                if not lens.busy:
                    try:
                        lens.refresh()
                    except Exception:
                        pass
                return self.send(200, {
                    "status": lens.st, "busy": lens.busy, "stage": lens.current_stage,
                    "log": lens.log[-80:], "cfg": cfg,
                    "sharp": cam.sharp if cam.fresh() else None,
                })
            self.send(404, {"error": "not found"})

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                d = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self.send(400, {"ok": False, "msg": "bad json"})
            p = urlparse(self.path).path
            if p == "/api/stop":
                lens.stop(); return self.send(200, {"ok": True, "msg": "stopping"})
            if p == "/api/config":
                if lens.busy:
                    return self.send(409, {"ok": False, "msg": "wait for the move to finish"})
                if "rules" in d:
                    for k, v in d["rules"].items():
                        if k in cfg["rules"]:
                            cfg["rules"][k] = type(cfg["rules"][k])(v) if not isinstance(cfg["rules"][k], bool) else bool(v)
                if "session" in d:
                    cfg["session"].update(d["session"])
                if "stages" in d:
                    cfg["stages"] = [{"name": str(s["name"]), "zoom": int(s["zoom"]), "focus": int(s["focus"])} for s in d["stages"]]
                save_cfg(cfg); lens.L("settings saved")
                return self.send(200, {"ok": True, "msg": "saved"})
            if p == "/api/do":
                a = d.get("action")
                try:
                    if a == "stage":
                        i = int(d["index"]); name = cfg["stages"][i]["name"]
                        ok, msg = lens.run(f"go to {name}", lambda: lens.go_stage(i))
                    elif a == "goto":
                        ax, v = d["axis"], int(d["pos"])
                        ok, msg = lens.run(f"{Lens.NAME[ax]} to {v}", lambda: lens.move(ax, v))
                    elif a == "jog":
                        ax, dv = d["axis"], int(d["delta"])
                        ok, msg = lens.run(f"{Lens.NAME[ax]} {dv:+d}", lambda: lens.move(ax, lens.pos(ax) + dv))
                    elif a == "home":
                        axes = ["z", "f"] if d.get("axis") == "all" else [d["axis"]]
                        ok, msg = lens.run("home " + "+".join(Lens.NAME[x] for x in axes),
                                           lambda: [lens.home(x) for x in axes])
                    elif a == "save_focus":
                        i = int(d["index"]); st = lens.refresh()
                        cfg["stages"][i]["focus"] = st["focus"]["pos"]
                        if d.get("with_zoom"):
                            cfg["stages"][i]["zoom"] = st["zoom"]["pos"]
                        save_cfg(cfg)
                        s = cfg["stages"][i]
                        lens.L(f"saved {s['name']}: zoom {s['zoom']} focus {s['focus']}")
                        ok, msg = True, "saved"
                    elif a == "session":
                        ok, msg = lens.run("apply session settings", lens.apply_session)
                    elif a == "raw":
                        if lens.busy:
                            ok, msg = False, f"busy: {lens.busy}"
                        else:
                            rep = lens.con.cmd(str(d["cmd"]), 1.5)
                            lens.L(f"> {d['cmd']}"); [lens.L("  " + l) for l in rep.splitlines()[1:]]
                            ok, msg = True, rep
                    else:
                        ok, msg = False, f"unknown action {a}"
                except (KeyError, ValueError, IndexError) as e:
                    ok, msg = False, f"bad request: {e}"
                return self.send(200 if ok else 409, {"ok": ok, "msg": msg})
            self.send(404, {"error": "not found"})
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", default="/dev/ttyUSB1")
    ap.add_argument("--http", type=int, default=8090)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--rtsp", default="rtsp://192.168.10.54:554/live1.sdp")
    a = ap.parse_args()
    cfg = load_cfg()
    if not os.path.exists(CFG_PATH):
        save_cfg(cfg)
    lens = Lens(Console(a.serial), cfg)
    lens.refresh()
    if cfg["session"].get("apply_on_connect"):
        lens.apply_session()
    cam = Camera(a.rtsp)
    srv = ThreadingHTTPServer((a.bind, a.http), make_handler(lens, cam, cfg))
    lens.L(f"lens UI on http://{a.bind}:{a.http}/  (console {a.serial})")
    import signal
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if cam.proc:
            cam.proc.terminate()


if __name__ == "__main__":
    main()
