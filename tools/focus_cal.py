#!/usr/bin/env python3
"""Focus-only lens calibration (zoom stays locked by the firmware).

Talks to the MCU console through tools/lens_ui (it owns the serial port and
keeps the latest camera frame in tools/lens_ui/.live.jpg).

    python3 tools/focus_cal.py noise              # camera noise floor
    python3 tools/focus_cal.py farend [--max 100000] [--chunk 2000]
    python3 tools/focus_cal.py sweep --travel N   # coarse 1000 + fine 100
    python3 tools/focus_cal.py repeat --best N [--n 5]

Sharpness = mean Sobel energy on the centre 1/3 of the frame.
Every point is reached from below (the firmware does that for moves down).
Results are appended to tools/focus_cal/results.json.
"""
import argparse, json, os, sys, time
import cv2
import numpy as np
import urllib.request

API = "http://localhost:8090/api/do"
HERE = os.path.dirname(os.path.abspath(__file__))
LIVE = os.path.join(HERE, "lens_ui", ".live.jpg")
OUT = os.path.join(HERE, "focus_cal")
SETTLE = 1.5            # s after a move before a frame counts (RTSP latency)


def cmd(c, quiet=True):
    req = urllib.request.Request(API, json.dumps({"action": "raw", "cmd": c}).encode(),
                                 {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        msg = json.load(r)["msg"]
    if not quiet:
        print("  >", c, "|", msg.replace("\r\n", " | ")[:160])
    return msg


def focus_state():
    for line in cmd("lens f").splitlines():
        if line.startswith("focus"):
            t = line.split()
            return int(t[t.index("pos") + 1]), ("MOVING" in line or "HOMING" in line)
    raise RuntimeError("no focus status line")


def wait_idle(timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos, moving = focus_state()
        if not moving:
            return pos
        time.sleep(0.3)
    raise RuntimeError("focus did not stop")


def goto(pos):
    msg = cmd(f"lens f goto {pos}")
    if "ERROR" in msg:
        raise RuntimeError(msg.strip())
    got = wait_idle()
    if got != pos:
        raise RuntimeError(f"focus at {got}, wanted {pos}")
    return time.time()


def frame(after):
    """First camera frame written at least SETTLE s after `after`."""
    t_need = after + SETTLE
    t0 = time.time()
    while time.time() - t0 < 10:
        try:
            if os.path.getmtime(LIVE) >= t_need:
                img = cv2.imread(LIVE, cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    return img
        except OSError:
            pass
        time.sleep(0.1)
    raise RuntimeError("no fresh camera frame (is lens_ui's ffmpeg running?)")


def centre(img):
    h, w = img.shape
    return img[h // 3: 2 * h // 3, w // 3: 2 * w // 3]


def sharp(img):
    c = centre(img).astype(np.float32)
    gx = cv2.Sobel(c, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(c, cv2.CV_32F, 0, 1, ksize=3)
    return float(np.mean(gx * gx + gy * gy))


def change(a, b):
    """Image change between two frames: mean abs difference, small + blurred."""
    sa = cv2.GaussianBlur(cv2.resize(a, (240, 135)), (5, 5), 0).astype(np.float32)
    sb = cv2.GaussianBlur(cv2.resize(b, (240, 135)), (5, 5), 0).astype(np.float32)
    return float(np.mean(np.abs(sa - sb)))


def save(kind, data):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, "results.json")
    allr = json.load(open(p)) if os.path.exists(p) else []
    allr.append({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "kind": kind, **data})
    json.dump(allr, open(p, "w"), indent=1)


def noise():
    t = time.time() - SETTLE
    a = frame(t)
    time.sleep(1.0)
    b = frame(time.time() - SETTLE)
    n = {"change": change(a, b), "sharp_a": sharp(a), "sharp_b": sharp(b)}
    print(f"noise: change {n['change']:.3f}  sharp {n['sharp_a']:.1f} / {n['sharp_b']:.1f}")
    return n


def farend(maxpos, chunk):
    n = noise()
    thr = max(3 * n["change"], 0.3)
    print(f"far end search: +{chunk} per chunk up to {maxpos}, 'no change' below {thr:.3f}")
    t = goto(0)
    prev = frame(t)
    rows, still = [], 0
    pos = 0
    while pos + chunk <= maxpos:
        pos += chunk
        t = goto(pos)
        img = frame(t)
        ch, sh = change(prev, img), sharp(img)
        rows.append((pos, ch, sh))
        still = still + 1 if ch < thr else 0
        print(f"  {pos:6d}  change {ch:7.3f}  sharp {sh:8.1f}{'   (no change)' if ch < thr else ''}")
        prev = img
        if still >= 3:
            break
    end = rows[-1 - still][0] if still >= 3 else None
    print("far end:", f"image stopped changing after {end}" if end else "not found")
    save("farend", {"chunk": chunk, "threshold": thr, "end": end, "rows": rows})
    goto(0)
    return end


def scan(points):
    rows = []
    for p in points:
        t = goto(p)
        s = sharp(frame(t))
        rows.append((p, s))
        print(f"  {p:6d}  sharp {s:8.1f}")
    return rows


def sweep(travel, coarse=1000, fine=100, span=1500):
    print(f"coarse sweep 0..{travel} step {coarse}")
    c = scan(range(0, travel + 1, coarse))
    pk = max(c, key=lambda r: r[1])[0]
    lo, hi = max(0, pk - span), min(travel, pk + span)
    print(f"coarse peak at {pk}; fine sweep {lo}..{hi} step {fine}")
    goto(max(0, lo - 1000))                       # start the fine pass from below
    f = scan(range(lo, hi + 1, fine))
    best, bs = max(f, key=lambda r: r[1])
    print(f"BEST FOCUS {best}  (sharpness {bs:.1f})")
    t = goto(best)
    img = frame(t)
    os.makedirs(OUT, exist_ok=True)
    cv2.imwrite(os.path.join(OUT, f"best_{best}.jpg"), cv2.imread(LIVE))
    save("sweep", {"travel": travel, "coarse": c, "fine": f, "best": best, "best_sharp": bs})
    return best, bs


def repeat(best, n, ref=None):
    vals = []
    for i in range(n):
        cmd("lens f home")
        wait_idle(60)
        t = goto(best)
        s = sharp(frame(t))
        vals.append(s)
        print(f"  run {i + 1}: home -> {best}  sharp {s:.1f}")
    ref = ref or max(vals)
    dev = [abs(v - ref) / ref * 100 for v in vals]
    ok = max(dev) <= 5.0
    print(f"reference {ref:.1f}; max deviation {max(dev):.1f} %  -> {'PASS' if ok else 'FAIL'}")
    save("repeat", {"best": best, "ref": ref, "values": vals, "max_dev_pct": max(dev), "pass": ok})
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("noise")
    f = sub.add_parser("farend"); f.add_argument("--max", type=int, default=100000); f.add_argument("--chunk", type=int, default=2000)
    s = sub.add_parser("sweep"); s.add_argument("--travel", type=int, required=True)
    r = sub.add_parser("repeat"); r.add_argument("--best", type=int, required=True); r.add_argument("--n", type=int, default=5)
    r.add_argument("--ref", type=float)
    a = ap.parse_args()
    try:
        if a.cmd == "noise": noise()
        elif a.cmd == "farend": farend(a.max, a.chunk)
        elif a.cmd == "sweep": sweep(a.travel)
        elif a.cmd == "repeat": sys.exit(0 if repeat(a.best, a.n, a.ref) else 1)
    except KeyboardInterrupt:
        cmd("lens f stop")
        raise
