# Lens control – test report 2026-11-19

Firmware: branch `lens-rewrite`, image `firmware/proxy-mcu_Secure_2026-11-19_10levels.hex`
(built from this commit's `Secure/Core/Src/lens_ctrl.c`, flashed 19:22 and boot-tested).

Board: STM32H563 proxy MCU, TMC2300 drivers on 3.3 V (zoom = U2 @2, focus = U1 @0),
Foctek D14-0622IR 6–22 mm lens. Target: chart board with a checkerboard column, 17 m away.
Sharpness = Sobel energy on the checkerboard column, located in every frame by template
matching (960×540 RTSP frames). Values depend on the scene and camera aim – compare only
within one level and one camera position.

## Summary

| Area | Change |
|---|---|
| Levels | **10 levels** (was 6), zoom 10000…32500 about 2500 steps apart; all earlier tables discarded |
| Boot level | **L9** (`BOOT_LEVEL = NLEVELS - 1`) |
| Boot homing | **Full home on every boot** (`LENS_FAST_BOOT 0`) – the fast boot from the position log came up out of focus |
| Driver settle | `LENS_BOOT_SETTLE_MS` 1000 → **3000** ms before the first TMC2300 config |
| Late driver config | Boot config retries in the background after the 10 boot attempts; the boot (home → L9) starts as soon as both drivers answer |
| Driver reset watch | Once a second at rest: read GSTAT. `reset` set → re-configure, drop the reference, re-home. Silent driver → moves blocked until it answers |
| Lookup table | New table in flash (sector 62) and as the built-in default; `lens_ui.json` has 10 presets |

## Level table (flash and built-in)

| Level | Zoom | Focus | Checkerboard sharpness (18:20 view) |
|---|---|---|---|
| L0 | 10000 | 600 | 95498 |
| L1 | 12750 | 2000 | 91400 |
| L2 | 15250 | 3200 | 91121 |
| L3 | 17500 | 4400 | 81638 |
| L4 | 20000 | 6050 | 81399 |
| L5 | 22500 | 7950 | 77925 |
| L6 | 24750 | 10200 | 71559 |
| L7 | 27500 | 13400 | 63365 |
| L8 | 30000 | 17600 | 56095 |
| L9 | 32500 | 24550 | 45261 |

Chart: `test_images/levels_2026-11-19/final_10_levels/level_table_zoom_focus.png`;
images of every level: `final_10_levels/contact_sheet.jpg`.

Zoom below 10000 cannot focus at 17 m: the focus would have to go below the focus home stop
(zoom 8000: still blurred at focus 0; zoom 0: smeared image at every focus). The focus
mechanical end is about 33000 (soft limit 32400).

## How the table was made

1. 15:00 – 10 levels from zoom 10000, 2500 apart: re-home, levels in ascending order, focus
   scanned ±1200 around a prediction from the measured zoom/focus curve (200, then 100/50
   around the peak). No full-range sweeps – the earlier full 0–32400 sweeps lost focus steps
   on the long moves down and shifted the whole 6-level table by 500–1000 steps.
2. 18:20 – re-tune after the hardware rework (the camera had also been re-aimed): re-home,
   then per level focus ±600 and zoom ±250 with focus re-tuned; a zoom change kept only for
   > 3 % gain. Changes were small (L2 +9 %, L3 +4 %, the rest 0–2 %).

## Tests

### Level moves (after a re-home, 15:58, 150 pps)
- Up one level at a time L0 → L9: 96–106 % of the calibrated sharpness, 8–11 s per change.
- Down one level at a time L9 → L0: 93–98 %, one 85 % (L9 → L8).
- Jumps L0 → L5 → L9 → L0: 99 / 98 / 107 %.

### Step rate (L3 ↔ L9, the longest jump, 3 round trips each)
| Max rate | L9 → L3 | L3 → L9 | Time per jump |
|---|---|---|---|
| 150 pps (kept) | 101, 103, 102 % | 101, 101, 101 % | 25–26 s |
| 100 pps | 105, 103, 101 % | 98, 99, 103 % | 32–35 s |

Slower stepping did not improve focus on level moves, so the rate stays at 150 pps.

### Boot
| Case | Firmware | Result |
|---|---|---|
| Reset at L3, fast boot to L9 | 1 s settle, fast boot | L9 0 % (focus lost on the boot move) |
| Reset at L3, fast boot to L9 (×2) | 3 s settle, fast boot | 97 %, 81 % |
| Power cycle at L6, fast boot to L9 | 3 s settle, fast boot | 10 % – focus count off by > 2500 steps |
| Flash (MCU reset) | full home → L9 | **106 %**, drivers on 1st attempt |
| Hardware reset at L3 | full home → L9 | **102 %**, 137 s, drivers on 1st attempt |
| Flash 19:22 (final image) | full home → L9 | at L9, drivers on 1st attempt |

The fast boot trusted the position logged in flash; boot moves after a reset or power cycle
lost focus steps (normal level moves of the same length did not), and the error carried
into every level until a re-home. Full homing on every boot (~2 min) was chosen instead.
The position log is still written; `LENS_FAST_BOOT 1` restores the old behaviour.

## Hardware findings (schematic `SCH_PROXY_ BOARD_V1.pdf` p.4–5, TMC2300 datasheet rev 1.09)

- All three TMC2300 PDN_UART pins share VF_UART (PA9, 10 kΩ pull-up) through 1 kΩ each
  (R47 U1, R25 U2, R59 U5).
- VIO/nSTBY was tied to the motor rail (M1_PWR/M2_PWR) – no MCU control of driver
  standby/reset, and an unpowered driver clamps VF_UART low through its ESD diode
  (matches the "PA9 held at 0 V, nothing answers" episodes).
- Iris U5 is strapped to address 0 (schematic label says 10) – same as U1 (focus).
- M1_PWR can come from 5 V (L5) or 3.3 V (L6); neither is marked DNI. 5 V would also put VIO
  at 5 V (input high 3.5 V > the MCU's 3.3 V) unless VIO is split from VS.
- After any TMC2300 reset the registers return to defaults: 1/32 microsteps (firmware uses
  1/16) – every later move would cover half the distance with no visible lost step. The new
  reset watch catches this (GSTAT.reset).

Rework done 2026-11-19 (by the user): R55 removed (PA9–PA10 0 Ω link), VIO/nSTBY on 3.3 V,
iris driver disabled. After the rework the drivers answered on most power cycles; once
(18:59) VF_UART stayed low and neither driver answered. Suggested: remove R59 so the disabled
U5 is fully off the bus (a disabled TMC2300 back-powered through PDN_UART can hold the line).

## Open points
- One power cycle after the rework left the driver bus stuck low (see above) – hardware.
- Moves down are slightly less accurate than moves up (93–98 % vs 96–106 %).
- Boot takes ~2 min (full home), by choice.
- The sharpness reference values belong to the current camera aim.
