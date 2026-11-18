# Lens control – test report 2026-11-18

Firmware: commit `538d93e` (branch `lens-rewrite`), image `firmware/proxy-mcu_Secure_538d93e.hex`.
The board's flash was read back over SWD and compared with the build: every load segment
matches (0x0C000000 142232 B, 0x0C022B98 440 B, 0x0C07E000 32 B).

Board: STM32H563RGVX proxy MCU, TMC2300 drivers on 3.3 V (zoom = U2 @2, focus = U1 @0),
Foctek D14-0622IR 6–22 mm lens. Sharpness = mean Sobel energy on the centre 1/3 of a
960×540 RTSP frame (`tools/focus_cal.py`); values depend on the scene, so compare only
within one scene.

## What changed (commits 2beb79e..538d93e)

| Area | Change |
|---|---|
| Level table | Zoom far stop 34250. L0 zoom 33750 / focus 28100; L1–L5 focus re-tuned at the same zoom steps (see table) |
| Move speed | Level moves back to 100/150/500 pps (250 pps lost steps on 3.3 V) |
| Driver UART | TMC2300 UART at 115200 baud instead of 9600 – root cause of drivers not answering after a power cycle |
| Boot | 1 s settle after driver enable + up to 10 config attempts 200 ms apart |
| Level lookup table | Zoom, focus, sharpness per level + far stop, CRC-checked, in flash sector 62 (0x0C07C000); `lens lut …` console commands |
| Fast boot | Position log in flash sector 61 (0x0C07A000); power-up restores the resting position instead of homing |
| Flash writes | Use the HAL `*_NS` erase/program types – TrustZone is disabled on this chip (OPTSR2 TZEN = 0xC3) |

## Level table (stored in flash)

| Level | Zoom | Focus | Sharpness at calibration |
|---|---|---|---|
| 0 | 33750 | 28100 | 43225 |
| 1 | 29250 | 15450 | 53829 |
| 2 | 25250 | 10300 | 52480 |
| 3 | 21250 | 6700 | 49472 |
| 4 | 17250 | 4100 | 49788 |
| 5 | 13250 | 2000 | 54697 |

Focus peaks found with a ±600 scan (coarse 200, fine 100); zoom ±250/±500 with focus re-tuned
gave no clear gain (<3 %), so the zoom steps were kept.

## Tests

### Level moves (firmware `lens level`, focus-chart scene)
- L1 → L5 → L0, one level at a time: every stop 97–104 % of its tuned sharpness, 10–17 s per change.
- Direct jumps without reset: L3 → L0 103 %, L1 → L3 96 %, L3 → L0 94 %.

### Driver UART (cold-boot dropout)
- At 9600 baud: pulsing the driver EN pins low for 5 s left both drivers deaf; `lens init`
  failed for 10+ min; SYSRESETREQ and nRST MCU resets did not recover them.
- At 115200 baud: the same stuck drivers answered at once; an EN pulse no longer breaks them.
- Power cycles since the change: drivers configured after 1–2 attempts (~1.2 s).

### Level lookup table in flash
- First boot wrote the built-in table; after reset "lookup table loaded from flash".
- SWD read-back of 0x0C07C000: magic, 6 levels, far stop and CRC as expected.
- `lens lut set` → `save` → `default` → `load` round trip restores the saved values.
- Table survived a firmware reprogram (smart flash off).

### Fast boot (position log)
| Lens before reset / power cycle | Result | Time to L0 | L0 sharpness vs reference |
|---|---|---|---|
| L0 (MCU reset) | already at L0, no motion | 1.7 s | – |
| L0 (power cycle, 19:15) | already at L0, no motion | ~1.2 s | sharp (16378) |
| L0 (power cycle, 20:02) | already at L0, no motion | ~1.2 s | 104 % |
| L3 (MCU reset) | direct move | 23 s | 94 % / 100 % (two runs) |
| L5 (MCU reset) | direct move | 29 s | 100 % |
| L1 (MCU reset) | direct move | 16 s | 100 % |
| empty log (first boot) | full homing | 122 s | – |

## Known limits / open points
- From a level other than L0 the boot takes 16–29 s: moves run at 150 pps, the limit on the
  3.3 V motor supply. A 5 V supply would allow faster moves.
- A lens turned by hand while the board is off is not detected – run `lens rehome`
  (it returns to the *current* level).
- One blurred L0 after an MCU reset (19:38) was not reproducible; the L3 it started from
  (after several unchecked UI level clicks) was never checked.
- After the lens swap the driver bus once stayed at 0 V (drivers silent at every baud rate);
  it came back after the hardware was checked on the bench – not a firmware issue.
- After the lens swap only L0 was re-checked (sharp); L1–L5 use the 2026-11-18 calibration.
