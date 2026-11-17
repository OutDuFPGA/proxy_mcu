# Lens motor settings – zoom and focus

> **2026-11-17 ~14:30 IST – MSP430 zoom levels ported, verified (firmware committed).**
> Source: `MSP430_Varifocal/main.c` v1.5 – 6 zoom levels `{0,4000,…,20000}` from its zoom home, focus per level
> `{25600,20800,14200,9200,5400,1600}` (zoom-in) / `{24000,16400,12200,8400,4200,1600}` (zoom-out), both homed with
> a 108 000-step slam. Mapped onto this lens by measuring best focus vs zoom (`zoom_focus_curve`, `zoom_far_stop`):
> same 1/16 scale; MSP zoom home = our zoom **far stop (~33 300, telephoto end)**; MSP focus home = our focus reverse stop.
>
> | Level | Zoom | Focus (MSP430) | Focus (this camera, used) | Sharpness |
> |---|---|---|---|---|
> | 0 (tele) | 32 800 | 25 600 | **27 100** | 15 987 |
> | 1 | 29 300 | 20 800 | **16 500** | ~20 000–24 000 |
> | 2 | 25 300 | 14 200 | **11 000** | 34 404 |
> | 3 | 21 300 | 9 200 | **7 200** | 36 801 |
> | 4 | 17 300 | 5 400 | **4 400** | 36 656 |
> | 5 (wide) | 13 300 | 1 600 | **2 250** | 39 854 |
>
> Rules learned (all in `lens_ctrl.c`): one motor at a time (both at once at 3.3 V lost steps); full blind homes
> (shortened homes missed the stop); 1 s settle after each step; level 0 kept 500 short of the far stop (at the stop
> the zoom landed differently per approach); **zoom only homes with focus ≥ 10 000** (with focus at 0 the zoom stop
> moved – level 3 2 028 vs 36 335); moves down re-home that axis, then come up. Verified: 3 boots to level 0 (16 139–16 386),
> level changes 0↔5, 2, 3, 4 within ~5 % of calibration; level 1 ~20 000 (cal 24 192); one level 5 outlier (21 003),
> two repeats 39 799 / 40 884. Console `lens level [n]`, `lens zoomin|zoomout`, `lens table cal|msp`; SoC `L n`, `ZI`, `ZO`, `FB`.


> **2026-11-17 ~12:00 IST – firmware state (committed).** Zoom referenced to its **reverse stop** (home 150 pps, up to
> 4 500 fs), locked at **9 599** (Zoom0). Focus homes slowly (100 pps, 3 000 fs) to its **real reverse stop**; best focus
> **0** at zoom 9 599 (sharpness 914 / 911 over two boots – the peak is at the focus stop, so this view cannot get
> fully sharp). Both axes 100/150 pps; every move finishes upward.
> The far-zoom sharp view below (zoom "far stop − 3 394", focus 17 200 / 11 700) is **not reproducible**: homing
> 72 000 µsteps forward met no hard stop, so zoom crept ~68 600 further per boot. To keep that view, its zoom count
> must be found again from the reverse stop.


> **2026-11-17 ~11:30 IST – SHARP POSITION recorded, both axes referenced to mechanical stops (user set zoom by hand).**
> | Axis | Steps (µsteps, 16/fs) | Reference |
> |---|---|---|
> | Zoom (U2, `lens z`) | count **168 304** = **far (forward) stop − 3 394** (−212 fs) | far stop reached at count 171 698 by a long 100 pps jog; then 10 pps back / 3 pps forward |
> | Focus (U1, `lens f`) | **17 200** above the **reverse stop** (1 075 fs) | `lens f homecfg 100 3000` + `lens f home` (slow home reaches the real stop; 300 pps stalls ~800–1 600 short) |
>
> Focus sweep from the stop, day mode, centre-1/3 Sobel: coarse peak 17 000 (17 911), fine peak **17 200 = 20 535**
> (17 100: 20 407, 17 300: 20 355; half-height ~16 750..17 750). Home -> 17 200 three times: 20 479 / 20 419 / 20 405 (0.6 %).
> Long moves DOWN lose steps (~3 000 over a 23 500 move at 150 pps): always re-home focus, then approach from below.
> Frame: `tools/focus_cal/best_zoomfar-3394_f17200.jpg`; data: `tools/focus_cal/results.json` (`sweep_from_stop`).
> Not in the firmware yet (it still has zoom 9599 from the old home, focus best 0).


> **2026-11-17 ~10:40 IST – focus-only control, zoom locked, focus calibrated (`tools/focus_cal.py`).**
> Zoom locked at **9 599** (Zoom0 view); `lens z lock [pos]` / `unlock`. SoC `Z`/`ZR` -> `ERR LOCKED`.
> At boot the MCU homes both axes, zoom -> 9 599, focus -> best, then sends `READY` (moves before that: `ERR BUSY`).
> Focus speed 200/300 pps, acc 1000; every focus move finishes upward (moves down go 1000 below first).
> Day mode (`ir cut day`), centre-1/3 Sobel sharpness. Results: `tools/focus_cal/results.json`.
>
> | Item | Value |
> |---|---|
> | Best focus | **0** (homed position): sharpness 887, flat top (0/100/200 -> 887/881/868), falls steadily upward |
> | Repeatability | home -> 0, 5 runs: max deviation **0.6 %** (PASS); 5000 -> 0 from above, 3 runs: 882–885 |
> | Far end | image stops changing above ~34 000 (not confirmed by eye – may be full blur) -> travel **32 400** |
> | Homed 0 | a repeatable stall point, **not** the hard stop: the stop is ~800–1 600 lower; moves below 0 lose steps |
>
> Console: `lens f best`, SoC: `FB`. Firmware constants `FOCUS_BEST_DEFAULT` / `FOCUS_TRAVEL_DEFAULT` in lens_ctrl.c.


> **2026-11-17 ~09:50 IST – ZOOM0 re-recorded on the new firmware (user).** Zoom motor `lens z` (U2) **9 599** µsteps
> (599 fs), focus motor `lens f` (U1) **0** (its home). Both counted from `lens z home` / `lens f home`.
> Built into the firmware as preset `zoom0` (`Zoom0`, `lens zoom0`): homes both axes, then goes there. Verified.
> Replaces the earlier Zoom0 entries below, which were taken with the old axis names.


> **2026-11-17 ~09:40 IST – new firmware (lens_ctrl rewrite), both axes homed, user confirmed at home.**
> Names now follow the physical motor: `lens z` = zoom motor (U2 @0x02, PC7), `lens f` = focus motor (U1 @0x00, PB5).
> `lens z home` = 1130 fs reverse at 300 pps -> zoom 0, limits 0..16900. `lens f home` = 4000 fs reverse at 300 pps
> -> focus 0, no upper limit yet (travel not measured). Older entries below use the old names (`lens z` = U1).


> **2026-11-17 08:59 IST – **ZOOM0** (stored as "Zoom0"): sharp, clear image (user adjusted with zoom_run/focus_run).**
> Counts are relative to the Zoom Home / Reference (both 0); 16 µsteps per full step.
>
> | Motor (physical) | Firmware axis | µsteps | Full steps |
> |---|---|---|---|
> | Focus motor | `lens z` / zoom_run (U1 @0x00, PB5) | **−58 143** | **−3 634** |
> | Zoom motor | `lens f` / focus_run (U2 @0x02, PC7) | **9 526** | **595** |
>
> Camera sharpness 6 047 (reference ≈ 1 100). Frame: `test_images/pos_2026-11-17_zm9526_fo-58143.jpg`.
> Saved as UI stage "Zoom0". The focus motor really travelled about 58 000 µsteps in
> reverse from the reference, so the "reverse stop" assumed at the reference was not a stop. The counts are exact
> only if no steps were lost: the reverse runs at 1 024 µsteps/s from the reference may have slipped.

> **2026-11-17 ~08:55 IST – ZOOM HOME recorded (user).** The user confirmed the zoom lens is at **0x zoom** at the
> reference position. **Zoom Home = 0 steps** on the zoom motor (`lens f`, U2 @0x02, PC7; 0 µsteps / 0 full
> steps, the old counter 31 986 from the blind home). Focus motor (`lens z`, U1 @0x00) = 0 (its reverse stop).
> Saved as the UI stage "Zoom Home (0x zoom) / Reference". Counters are RAM only: after a reset, run `lens f zero`
> / `lens z zero` with the lens back at this position.

> **2026-11-17 ~08:45 IST – reference set and calibration sweep.** The recorded position above is now the
> reference: `lens z zero` (focus motor at its reverse stop) and `lens f zero` (zoom motor; was 31 986, so its
> home end is about −32 000). Sweep: zoom motor `lens f` −30 000 … +9 000 (step 3 000) × focus motor `lens z`
> 0 … 8 000 (step 800). At every zoom step the focus peak is at `lens z` 0–800 (sharpness about 1 100), and the
> image scale against the first frame is **1.00–1.01**. **The zoom motor does not change the field of view**, so
> no 1x … 3.5x positions can be assigned. Saved as the UI stage "Reference" (`lens z` 0, `lens f` 0).

> **2026-11-17 08:27 IST – recorded position (reverted 11-16-morning firmware + U2 @0x02, voltage mode, IRUN 31).**
> Cables crossed at the lens connector (user): `lens z`/zoom_run (U1 @0x00, PB5) = **focus motor**,
> `lens f`/focus_run (U2 @0x02, PC7) = **zoom motor**.
>
> | Motor (physical) | Firmware axis | Counter (µsteps, 16/fs) | Full steps | Reference |
> |---|---|---|---|---|
> | Focus motor | `lens z` (U1 @0x00) | **−22 588** | −1 411 | unhomed; `lens z zero` was set at the reverse stop, after which the motor was driven into that stop several times (slip) – physically at/near the **reverse stop** (≈ 0 … +200 fs); sharpest image of all sweeps |
> | Zoom motor | `lens f` (U2 @0x02) | **31 986** | 1 999 | homed (blind, 1130 fs reverse search) then jogged; counter is exact unless steps were lost; camera showed no zoom change over 0 … 32 000 |
>
> To return here: `lens z home` (or drive `lens z` into its reverse stop), `lens f goto 31986`.

> **2026-11-17: wiring error confirmed, names swapped.** TMC2300 U1 (PB5 STEP / PB4 DIR / PB3 EN / PC12 DIAG,
> TIM3_CH2, UART @0x00) drives the **FOCUS** motor; U2 (PC7 STEP / PC8 DIR / PA8 EN / PC9 DIAG, TIM8_CH2,
> UART @0x02) drives the **ZOOM** motor. Pin macros renamed to match (`FOCUS_*` = PB5/PB4/PB3/PC12,
> `ZOOM_*` = PC7/PC8/PA8/PC9, main.h + .ioc labels) and `LENS_SWAP_ZOOM_FOCUS` = 1, so
> **`lens f …` = U1 motor (0 … 32 550, SG 12, home 700 pps)** and **`lens z …` = U2 motor (0 … 6 000, blind
> home 300 pps)**. Hardware, timers, addresses and calibration unchanged. Everything below that says "zoom" for
> the U1/PB5 motor now means focus (`lens f`).

> **2026-11-17: speed limits set to the motor spec** (Foctek D14-0622IR: start ≤ 800 pps, slew ≤ 1200 pps
> at 4.0 V ±20 %). `UV_SPEED_CAP_PPS` 600 → 1200, zoom (U2) speed 200/300/600 → 200/1200/2000, focus (U1)
> stays 200/1200/2000. At ≥ 800 pps the focus motor reads SG 2–6 while running freely (same as a stall),
> so StallGuard is now ignored on moves faster than `STALL_SG_MAX_PPS` = 700 pps; homing (700 pps) is still
> stall-protected. Measured: focus 0 ↔ 32 000 in 2.3 s (was 3.8 s), no lost steps (camera round trip);
> zoom 0 ↔ 6 000 in 1.1 s.
>
> **Zoom-ratio calibration (1x … 3.5x) blocked:** with the camera (rtsp …/live1.sdp, 1920×1080), zoom moves
> over −16 000 … +6 000 counts change the image scale by < 0.5 % (1.000 ± 0.005) – the zoom group does not move.

> **2026-11-17 ~00:40: StallGuard re-calibration.** Focus (U1, `lens f`): stall-homed zero = reverse stop + 8 fs;
> far stop caught by StallGuard (700 pps, SG 12) at **32 531 / 32 681** (one false trip at 17 811) → firmware
> limit `U1_CAL_TRAVEL` **32 400** (was 32 550, which was past the 32 531 stop). Verified 0 ↔ 32 400 in 2.3 s.
> Zoom (U2, `lens z`): **StallGuard not usable** – SG_VALUE 0–78 (median 10–18) while running at 100–700 pps,
> no gap to a stall; and the camera shows **no field-of-view change** over −24 000 … +16 000 counts, even at
> 100 pps (SG cycles 2/8/38/4 per electrical period = rotor not turning freely). Zoom limits stay 0 … 6 000
> until the zoom motor drives the lens.

> **~21:00: swap reverted at the user's request.** `LENS_SWAP_ZOOM_FOCUS` = 0: zoom = PB5 / TIM3_CH2 /
> TMC2300 @0x00, focus = PC7 / TIM8_CH2 / TMC2300 @0x02 (as the schematic). The `U1_CAL_*` calibration
> (0 … 32 550, SG 12, home 700 pps) is back on `lens z …`. Verified: `lens z jog` toggles only PB5,
> `lens f jog` only PC7. Note: the motor on this driver changes sharpness only (user check), so the motor
> cables / lens labelling may be crossed – the swap note below is kept for history.
>
> **PC7 / TIM8 / @0x02 motor (`lens f …` in firmware) – Max Zoom (~21:20):** blind-homed (`lens f home`, no
> StallGuard, 2660 fs search at 300 pps, zero = 4 fs off where it ended), then `lens f goto 6000` → the user
> reports the lens at the **Max Zoom position at count 6 000**. Upper limit set to 6 000 (`lens f limit 0 6000`,
> test mode).
> **Firmware default since ~21:25:** `U2_CAL_TRAVEL 6000`, `U2_CAL_HOME_MAX_FS 402` (limits 0 … 6 000 at boot,
> blind homing at 300 pps). Verified after flashing: `lens f home` 1.5 s, `lens f goto 6000` 1.5 s.
> **~21:40 step-count record (PC7 / @0x02 axis):** after test-mode moves to 12 000 (limit raised to 0 … 12 000)
> and back, the user reports the lens at **Max Zoom at count 0**. Camera check right after: goto 0→3 000→6 000→
> 3 000→0 changed the picture by ≤ 1.3 (noise), sharpness 85–88 – the motor is not moving the lens in either
> direction now. So the Max Zoom count (first 6 000, now 0) is not reliable: this axis is losing all steps
> (pushed past Max Zoom to 12 000 without stall detection). Needs a hardware check before re-calibrating.

> **2026-11-16 ~20:40, board 03: zoom and focus swapped.** Moving this motor changes only sharpness, not the
> field of view, so it is the **focus** group. `LENS_SWAP_ZOOM_FOCUS` = 1 (lens_ctrl.h); the calibration below
> (`U1_CAL_*`, was `ZOOM_CAL_*`) now applies to **focus** (`lens f …`). Read "zoom" below as "focus":
> "Zoom 0" = focus 0 (lens inside the barrel), "Full Zoom" = focus 32 550 (far end).
> The zoom group is the U2 motor (TIM8, addr 2), still on provisional settings (limits 0 … 8 450) – it did
> not visibly move in earlier tests and needs its own calibration.
>
> **Zoom (U2, addr 2) – first observations (~20:50):** moves 0 → 8 000 → 0 (×2) at 200/300 pps, then
> limit raised to 40 000 (test mode) and `goto 32500` from the power-up position (7.0 s): the user reports the
> lens **completely at the zoom-out (wide) position** at count **32 500** (counted from the power-up position,
> unhomed, no stall protection – steps may have slipped at the end).
> **Correction:** after `goto 0` (7.0 s back) the user reports the lens is *still* at zoom-out. So the zoom
> group did not move between count 0 and 32 500 – the U2 motor is most likely not moving the lens at all
> (same as when it was called "focus"). Needs a hardware check (coils, connector, mechanism).

Measured and tuned on 2026-11-16 over the J10 console (`/dev/ttyUSB1`, 115200).
**Built into the firmware defaults** (`ZOOM_CAL_*` in `Secure/Core/Src/lens_ctrl.c`), so they apply at every boot.
`lens save` (MCU flash) still fails – see the last section. After a reset, just run `lens z home`.

> **Update 2026-11-16 ~19:55 – zoom zero re-set by eye.** The stall-homed "reverse stop" turned out to be a
> tight spot: the lens was still outside the barrel there. Zoom was jogged backward (stall off, ~312 pps)
> until the user confirmed "lens at Zoom 0", and `lens z zero` was set there (camera view: wide office scene).
> **Correction:** the far stop measured 32 659–32 755 from the stall-homed zero and 32 871 from the by-eye zero,
> so both zeros sit at the same reverse end stop (homing adds its 128-count backoff). There is no tight spot – the
> long backward jogs were the motor slipping at the stop. `lens z home` reproduces Zoom 0 (camera diff 4.0 vs the
> by-eye snapshot).

## Settings

| Setting | Value | Console command | Notes |
|---|---|---|---|
| Driver | TMC2300 U1, UART address 0 | – | STEP TIM3 (PB5), DIR PB4, EN PB3 |
| Microstepping | 16 µstep | – | set by `lens init` (CHOPCONF 0x14008001) |
| Run / hold current | IRUN 31 / IHOLD 16 | – | coil-limited (40 Ω coils at 3.3 V) |
| Test mode | off | – | not needed: the driver answers and is configured at boot |
| Speed start / max / accel | 200 / 1200 pps / 2000 pps² | `lens z speed 200 1200 2000` | effective top speed ≈ 600 pps (UV cap at VS < 3.5 V) = ~9600 counts/s |
| Stall threshold (SGTHRS) | 12 (stall when SG_VALUE ≤ 24) | `lens z sg 12` | 28 and 20 false-tripped (20: 1 in 14 at cruise); 12: 14/14 homes + 2/2 far-stop stalls correct |
| Homing config | 700 pps, max 2500 fs, backoff 8 fs | `lens z homecfg 700 2500 8` | StallGuard is armed only above 75 % of home pps (≈ 525 pps) |
| Zero (count 0) | reverse end stop (lens fully inside the barrel), 8 fs (128 counts) off the stop | `lens z home` (~1.8 s from mid travel) | homing stalls into the reverse stop at 700 pps, then backs off 8 fs |
| Direction | `dir 1` = away from the reverse stop (barrel out), `dir 0` = toward it | – | |
| **Full Zoom** | **10 484 counts** (≈ 655 fs) from the homed zero | `lens z goto 10484` | marked by the user 2026-11-16 (reached with two 10 s `zoom_run 1 500` from home) |
| Travel (end to end) | 32 660 – 32 692 counts (≈ 2 042 full steps) | – | 4 stall-stopped runs, spread 31 counts |
| **Far stop (step count)** | **32 563 – 32 755** counts from the homed zero (≈ 2 040 full steps); runs 32 565 / 32 595 / 32 563, re-calibrated later 32 755 / 32 659 / 32 659 | – | caught by StallGuard on `goto 40000` with the limit raised |
| Soft limits | 0 … 32 550 | (default) | far stop is at ~32 548 from the homed zero; 0 → 32 400 → 0 runs 3.5 s each way without a stall |

## StallGuard readings (zoom, ~600 pps)

| Condition | SG_VALUE |
|---|---|
| Free running | 66 – 200 |
| Pushing into an end stop | 0 – 52 (occasional 96) |
| Free running at ≤ 300 pps | 0 – 192 (unusable – that is why the arm threshold is ≈ 525 pps) |

Stall detection protects **only full-speed moves**; slow moves are unprotected.

Firmware stall gating (lens_ctrl.c, 2026-11-16):
- DIAG ignored for the first 64 full steps of a move (`STALL_BLANK_FS`, was 8).
- With StallGuard on, DIAG counts only at ≥ 97 % of max speed and not while decelerating (`STALL_CRUISE_FRAC`).
- Homing ramps from the start speed (200 pps) up to home pps instead of starting at 700 pps; same UV cap (~600 pps).

## Step counts (16 µstep, zero = homed reverse stop, 128 counts off it)

| Point | Counts | Full steps |
|---|---|---|
| Reverse stop (lens inside barrel) | −128 | −8 |
| Homed zero | 0 | 0 |
| **Full Zoom** | **10 484** | **≈ 655** |
| Upper soft limit = Full Zoom | 32 550 | 2 034 |
| Far stop | 32 563 – 32 755 | ≈ 2 040 |
| End-to-end travel | ≈ 32 690 – 32 720 | ≈ 2 044 |

## After a reset

```
lens z home          (stalls into the reverse stop, zero = 8 fs off it)
lens z goto <0..32400>
```

Homing (ramped, ~600 pps cruise) takes about distance/9600 counts·s⁻¹ + 0.3 s, e.g. 16000 → 2.0 s, 32400 → 3.8 s.
A *jog* that starts right at the stop overruns ~900 counts before StallGuard arms (it must first
accelerate past ~525 pps) – use `home`, not a jog, to find the end.

## Known issues

- `lens save` → `ERROR: save verify failed at 0x0C07C000`. The sector erases, but the quad-word
  program ends in a PGSERR in the **non-secure** FLASH_NSSR (0x00040000); SECSR stays 0, the SAU
  is disabled. Root cause not found yet, so the settings live in the firmware defaults instead.
- Driver reports open-load on coil A (DRV_STATUS bit 6) while moving; the motor still moves.
  Focus shows the same flag.
- Focus: stall detection not usable (SG 0–82 at all speeds), and focus motion could not be
  confirmed with the camera on this board – see the session notes.
