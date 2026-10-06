#!/usr/bin/env python3
"""IR LED current sweep with simultaneous video capture.

For each current level the script sets the IR LED current on the proxy MCU,
waits for the camera's auto-exposure to settle, records the IMX568 RTSP
stream for a fixed time while logging the MCU's FLASH/pulse status, then
burns the test scenario into every frame of an annotated copy.

  python3 tools/ir_sweep_test.py                      # 0/500/1000/2000/2990 mA, 30 s each
  python3 tools/ir_sweep_test.py --levels 0 1000 --duration 10
  python3 tools/ir_sweep_test.py --pulse 800 --levels 3500

Output (default captures/ir_sweep_<date>_<time>/ next to the repo):
  <level>mA_raw.mkv         recorded stream, untouched
  <level>mA_annotated.mp4   same, with the test scenario on every frame
  results.json              per-level MCU status, video stats, per-frame luma
  report.md                 summary table

Talking to the MCU: the UART console (/dev/ttyUSB1, 115200 8N1) is used when
it can be opened. If another program holds it (e.g. screen), the script
falls back to the ST-LINK: commands are written into the firmware's command
buffer and status is read from RAM, with addresses taken from the ELF that
is running on the board (--elf). Requires pyocd for that path; ffmpeg and
ffprobe always.
"""
import argparse
import json
import os
import re
import select
import statistics
import subprocess
import sys
import termios
import time
from datetime import datetime

DEFAULT_LEVELS = [0, 500, 1000, 2000, 2990]
RTSP_URL = "rtsp://192.168.10.54:554/live1.sdp"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
PULSE_THRESHOLD_MA = 3000
DAC_VREF_MV, DAC_MAX = 3300, 4095


# --------------------------------------------------------------------------
# MCU access
# --------------------------------------------------------------------------

class SerialConsole:
    """Raw termios access to the MCU console; parses 'ir status' replies."""

    def __init__(self, dev):
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0                                   # iflag
        attrs[1] = 0                                   # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                                   # lflag: raw
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.name = "UART " + dev

    def _read_for(self, seconds):
        out, end = b"", time.time() + seconds
        while time.time() < end:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if r:
                try:
                    out += os.read(self.fd, 4096)
                except BlockingIOError:
                    pass
        return out.decode(errors="replace")

    def command(self, cmd, wait=0.6):
        os.write(self.fd, ("\r" + cmd + "\r").encode())
        return self._read_for(wait)

    def status(self):
        txt = self.command("ir status", wait=1.2)
        st = {"source": "uart"}
        m = re.search(r"IR: (ENABLED|DISABLED)\s+pulse limit (\d+) us.*?pulses (\d+)", txt)
        if m:
            st.update(enabled=m.group(1) == "ENABLED", pulse_us=int(m.group(2)), pulses=int(m.group(3)))
        m = re.search(r"FLASH \(PC3, active low\): ([\d.]+) Hz\s+exposure (\d+) us\s+falls (\d+)", txt)
        if m:
            st.update(flash_hz=float(m.group(1)), exposure_us=int(m.group(2)), falls=int(m.group(3)))
        for ch, ma, mv, code in re.findall(r"CH(\d) \(J\d\):\s+(\d+) mA \(DAC_DIM\d (\d+) mV, code (\d+)\)", txt):
            st["ch%s" % ch] = {"ma": int(ma), "dac_mv": int(mv), "dac_code": int(code)}
        return st

    def close(self):
        os.close(self.fd)


class SwdConsole:
    """ST-LINK fallback: inject commands into cmd_buffer, read status from RAM."""

    def __init__(self, elf, target="stm32h563rgvx"):
        import logging
        logging.disable(logging.WARNING)
        from pyocd.core.helpers import ConnectHelper
        nm = subprocess.run(["arm-none-eabi-nm", elf], capture_output=True, text=True, check=True).stdout
        self.sym = {l.split()[2]: int(l.split()[0], 16) for l in nm.splitlines() if len(l.split()) == 3}
        for need in ("cmd_buffer", "cmd_index", "cmd_ready_flag", "s_ch", "s_fl_falls"):
            if need not in self.sym:
                raise SystemExit("ELF %s has no %s - is it the build running on the board?" % (elf, need))
        self.engine = "s_eng_on" in self.sym     # pulse-engine build vs strobe/gate build
        self.session = ConnectHelper.session_with_chosen_probe(
            target_override=target, options={"connect_mode": "attach"})
        self.session.open()
        self.t = self.session.target
        self.name = "ST-LINK (ELF %s)" % os.path.basename(elf)

    def command(self, cmd, wait=0.3):
        t, s = self.t, self.sym
        end = time.time() + 20
        while t.read8(s["cmd_ready_flag"]) or t.read16(s["cmd_index"]):
            if time.time() > end:
                raise SystemExit("console busy for 20 s (text half-typed in another terminal?)")
            time.sleep(0.05)
        t.write_memory_block8(s["cmd_buffer"], list(cmd.encode() + b"\0"))
        t.write8(s["cmd_ready_flag"], 1)
        while t.read8(s["cmd_ready_flag"]):
            time.sleep(0.01)
        time.sleep(wait)
        return ""

    def status(self):
        t, s, r = self.t, self.sym, self.t.read32
        cpu_mhz = 64
        per = r(s["s_fl_period_cyc"]) / (cpu_mhz * 1000.0)
        st = {"source": "swd",
              "falls": r(s["s_fl_falls"]),
              "flash_hz": round(1000.0 / per, 3) if per else 0.0,
              "exposure_us": int(r(s["s_fl_low_cyc"]) / cpu_mhz)}
        if self.engine:
            st.update(enabled=bool(t.read8(s["s_eng_on"])), pulse_us=r(s["s_eng_pulse_us"]),
                      pulses=r(s["s_eng_pulses"]), exposure_us=r(s["s_eng_exp_us"]) or st["exposure_us"])
        else:
            gate, strobe = r(s["s_gate_pins"]), r(s["s_dac_gate"])
            st.update(enabled=True, pulses=r(s.get("s_gate_count", s["s_fl_falls"])),
                      ch0_mode=("strobe" if strobe & 1 else "gate" if gate & 0x4000 else "follows FLASH only"))
        for ch in (0, 1):
            code = t.read16(s["s_ch"] + ch * 0x34 + 48)
            st["ch%d" % ch] = {"dac_code": code, "dac_mv": round(code * DAC_VREF_MV / DAC_MAX)}
        return st

    def close(self):
        self.session.close()


def open_console(args):
    try:
        return SerialConsole(args.uart)
    except OSError as e:
        print("UART %s not available (%s) - using the ST-LINK" % (args.uart, e.strerror))
        return SwdConsole(args.elf)


# --------------------------------------------------------------------------
# Video
# --------------------------------------------------------------------------

def record(url, seconds, path):
    cmd = ["ffmpeg", "-nostdin", "-loglevel", "error", "-rtsp_transport", "tcp",
           "-i", url, "-t", str(seconds), "-c", "copy", "-y", path]
    return subprocess.Popen(cmd)


def video_stats(path, skip_s=1.0):
    pts = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                          "packet=pts_time", "-of", "csv=p=0", path],
                         capture_output=True, text=True).stdout.split()
    p = sorted(float(x) for x in pts if x not in ("", "N/A"))
    q = [x for x in p if x >= skip_s]
    d = [(b - a) * 1000 for a, b in zip(q, q[1:])]
    med = statistics.median(d) if d else 0
    luma_txt = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-vf",
                               "format=gray,signalstats,metadata=print:key=lavfi.signalstats.YAVG:file=-",
                               "-f", "null", "-"], capture_output=True, text=True).stdout
    y = [float(v) for v in re.findall(r"YAVG=([\d.]+)", luma_txt)]
    roi_txt = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-vf",
                              "crop=iw/2:ih/2:iw/4:ih/4,format=gray,signalstats,"
                              "metadata=print:key=lavfi.signalstats.YAVG:file=-",
                              "-f", "null", "-"], capture_output=True, text=True).stdout
    yc = [float(v) for v in re.findall(r"YAVG=([\d.]+)", roi_txt)]
    ys = y[int(skip_s * 60):] or y
    diff = [abs(b - a) for a, b in zip(ys, ys[1:])]
    mu = statistics.mean(ys) if ys else 0
    ycs = yc[int(skip_s * 60):] or yc
    return {"frames": len(p),
            "centre_luma_mean": round(statistics.mean(ycs), 2) if ycs else 0,
            "fps": round((len(q) - 1) / (q[-1] - q[0]), 3) if len(q) > 1 else 0,
            "dropped": sum(1 for x in d if med and x > 1.5 * med),
            "luma_mean": round(mu, 2),
            "luma_std": round(statistics.pstdev(ys), 3) if ys else 0,
            "luma_min": round(min(ys), 2) if ys else 0,
            "luma_max": round(max(ys), 2) if ys else 0,
            "flicker_pct": round(100 * statistics.mean(diff) / mu, 3) if diff and mu else 0,
            "luma_per_frame": [round(v, 2) for v in y]}


def annotate(raw, out, level_ma, st, args):
    ch0 = st.get("ch0", {})
    if "ch0_mode" in st:
        mode = "CH0 mode: %s" % st["ch0_mode"]
    else:
        mode = ("PULSE %d us" % st.get("pulse_us", 0)) if level_ma > PULSE_THRESHOLD_MA else \
               ("full exposure" if level_ma else "off (0 mA)")
    l1 = "TEST  IR %s = %d mA   DAC %s mV (code %s)   %s   PWM_DIM synced to FLASH" % (
        args.channel.upper(), level_ma, ch0.get("dac_mv", "?"), ch0.get("dac_code", "?"), mode)
    l2 = "MCU   FLASH %.2f Hz   exposure %s us   gate pulses %s" % (
        st.get("flash_hz", 0), st.get("exposure_us", "?"), st.get("pulses", "?"))
    l3 = "VIDEO IMX568   %s   frame %%{frame_num}  t=%%{pts\\:hms}  Y=%%{metadata\\:lavfi.signalstats.YAVG}" % \
         datetime.now().strftime("%Y-%m-%d")
    esc = lambda s: s.replace(":", "\\:").replace("'", "")
    vf = ("signalstats,drawbox=x=0:y=0:w=iw:h=210:color=black@0.65:t=fill,"
          "drawtext=fontfile=%s:fontsize=44:fontcolor=yellow:x=24:y=18:text='%s',"
          "drawtext=fontfile=%s:fontsize=44:fontcolor=white:x=24:y=82:text='%s',"
          "drawtext=fontfile=%s:fontsize=44:fontcolor=cyan:x=24:y=146:text='%s'") % (
        FONT, esc(l1), FONT, esc(l2), FONT, l3)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", raw, "-vf", vf, "-c:v", "libx264",
                    "-preset", "veryfast", "-crf", "23", out], check=True)


# --------------------------------------------------------------------------

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--levels", type=int, nargs="+", default=DEFAULT_LEVELS, help="currents in mA")
    ap.add_argument("--duration", type=float, default=30, help="seconds of video per level")
    ap.add_argument("--settle", type=float, default=6, help="seconds to wait after a change")
    ap.add_argument("--channel", choices=["both", "ch0", "ch1"], default="both")
    ap.add_argument("--pulse", type=int, help="ir pulse <us> to set first (10..1500)")
    ap.add_argument("--rtsp", default=RTSP_URL)
    ap.add_argument("--uart", default="/dev/ttyUSB1")
    ap.add_argument("--elf", default=os.path.join(os.path.dirname(here), "STM32CubeIDE", "Secure",
                                                  "Debug_linux", "proxy-mcu_Secure.elf"),
                    help="ELF running on the board, for the ST-LINK fallback (default: tools/build.mk output)")
    ap.add_argument("--out", help="output directory")
    ap.add_argument("--final-ma", type=int, default=1000, help="current to leave set at the end")
    ap.add_argument("--no-annotate", action="store_true")
    ap.add_argument("--mode", choices=["gate", "strobe"], help="strobe/gate build: set ch0 mode first")
    args = ap.parse_args()

    out = args.out or os.path.join(os.path.dirname(os.path.dirname(here)), "captures",
                                   "ir_sweep_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    out = os.path.abspath(out)
    os.makedirs(out, exist_ok=True)
    prefix = "" if args.channel == "both" else args.channel + " "

    con = open_console(args)
    print("MCU via %s, output %s" % (con.name, out))
    results = {"started": datetime.now().isoformat(timespec="seconds"), "rtsp": args.rtsp,
               "channel": args.channel, "duration_s": args.duration, "console": con.name, "levels": []}
    try:
        engine = getattr(con, "engine", True)
        if engine:
            if args.pulse:
                con.command("ir pulse %d" % args.pulse)
            con.command("ir enable")
        elif args.mode:
            con.command("ir ch0 %s on" % args.mode)
        if not engine and args.channel == "both":
            prefix = "ch0 "          # strobe/gate build: 'ir ma' addresses ch0 only
        for ma in args.levels:
            print("--- %d mA" % ma)
            con.command("ir %sma %d" % (prefix, ma))
            time.sleep(args.settle)
            raw = os.path.join(out, "%04dmA_raw.mkv" % ma)
            rec = record(args.rtsp, args.duration, raw)
            polls, t_end = [], time.time() + args.duration
            while time.time() < t_end and rec.poll() is None:
                polls.append(con.status())
                time.sleep(2)
            rec.wait(timeout=args.duration + 30)
            st = polls[len(polls) // 2] if polls else con.status()
            vs = video_stats(raw)
            entry = {"level_ma": ma, "mode": ("pulse" if ma > PULSE_THRESHOLD_MA else ("full" if ma else "off")),
                     "mcu": st, "mcu_polls": polls, "video": {k: v for k, v in vs.items() if k != "luma_per_frame"},
                     "luma_per_frame": vs["luma_per_frame"], "raw_video": os.path.basename(raw)}
            if not args.no_annotate:
                ann = os.path.join(out, "%04dmA_annotated.mp4" % ma)
                annotate(raw, ann, ma, st, args)
                entry["annotated_video"] = os.path.basename(ann)
            results["levels"].append(entry)
            print("    %.2f fps, %d dropped, luma %.1f centre %.1f (std %.2f, flicker %.2f%%), FLASH %.2f Hz, exposure %s us" % (
                vs["fps"], vs["dropped"], vs["luma_mean"], vs["centre_luma_mean"], vs["luma_std"], vs["flicker_pct"],
                st.get("flash_hz", 0), st.get("exposure_us", "?")))
        con.command("ir %sma %d" % (prefix, args.final_ma))
    finally:
        con.close()

    # Side-by-side stills and amplified differences against the first level
    stills = []
    for e in results["levels"]:
        png = os.path.join(out, "still_%04dmA.png" % e["level_ma"])
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", str(args.duration / 2), "-i",
                        os.path.join(out, e["raw_video"]), "-frames:v", "1", "-vf", "scale=608:-1", png])
        stills.append((e["level_ma"], png))
    if stills:
        base = stills[0][1]
        for ma, png in stills[1:]:
            subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", png, "-i", base, "-filter_complex",
                            "[0:v][1:v]blend=all_mode=difference,lutyuv=y=val*6,format=gray",
                            os.path.join(out, "diff_%04dmA_vs_%04dmA_x6.png" % (ma, stills[0][0]))])
        lab = ["[%d:v]drawtext=fontfile=%s:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.6:x=10:y=10:"
               "text='%d mA  Y=%.1f'[v%d]" % (i, FONT, ma, e["video"]["luma_mean"], i)
               for i, ((ma, _), e) in enumerate(zip(stills, results["levels"]))]
        ins = sum([["-i", png] for _, png in stills], [])
        subprocess.run(["ffmpeg", "-v", "error", "-y"] + ins + ["-filter_complex",
                        ";".join(lab) + ";" + "".join("[v%d]" % i for i in range(len(stills))) +
                        "hstack=inputs=%d" % len(stills), os.path.join(out, "comparison_stills.png")])

    with open(os.path.join(out, "results.json"), "w") as f:
        json.dump(results, f, indent=1)
    with open(os.path.join(out, "report.md"), "w") as f:
        f.write("# IR LED current sweep - %s\n\n" % results["started"])
        f.write("Channel: %s, %g s per level, MCU via %s, stream %s\n\n" % (
            args.channel, args.duration, con.name, args.rtsp))
        f.write("| Current | Mode | DAC mV | FLASH Hz | Exposure us | Video fps | Dropped | Luma mean | vs first | Centre luma | vs first | Luma std | Flicker % | Video |\n")
        f.write("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
        b0 = results["levels"][0]["video"] if results["levels"] else {}
        for e in results["levels"]:
            v, m = e["video"], e["mcu"]
            f.write("| %d mA | %s | %s | %s | %s | %s | %s | %s | %+.1f | %s | %+.1f | %s | %s | %s |\n" % (
                e["level_ma"], m.get("ch0_mode", e["mode"]), m.get("ch0", {}).get("dac_mv", "?"),
                m.get("flash_hz", "?"), m.get("exposure_us", "?"), v["fps"], v["dropped"],
                v["luma_mean"], v["luma_mean"] - b0.get("luma_mean", 0), v["centre_luma_mean"],
                v["centre_luma_mean"] - b0.get("centre_luma_mean", 0), v["luma_std"],
                v["flicker_pct"], e.get("annotated_video", e["raw_video"])))
        f.write("\nStills: comparison_stills.png; differences vs the first level (x6): diff_*.png\n")
    print("done: %s" % out)


if __name__ == "__main__":
    main()
