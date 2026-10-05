# USHIO IR illuminator (LT3763) bring-up and test procedure

Proxy MCU board (STM32H563, secure partition) driving two USHIO IR LED driver
boards on J5 and J6.

References: `SCH_PROXY_BOARD_V1` sheets 3 and 5, `Ushio_IR_LED_DriverV2`
sheet 3, LT3763 datasheet Rev C. Firmware: `Secure/Core/Src/ir_led_driver.c`,
`dac.c`, `ir_led_cli.c`.

---

## Current operating mode (2026-10-05)

FLASH (PC3, pin 11) is a read-only input from the sensor, active low (low =
exposure). Boot: CH0 FLASH-gated at 1000 mA, CH1 PWM_DIM1 HIGH at 0 mA.

```
ir status                          FLASH, VSYNC/HSYNC inputs and both channels
ir [ch0|ch1] ma <mA>               LED current via DAC_DIMx (up to 4000 mA)
ir [ch0|ch1] mv <mV>               DAC_DIMx voltage directly
ir [ch0|ch1] strobe on|off         lit while FLASH low (PWM_DIM LOW + DAC setpoint),
                                   DAC 0 V while FLASH high
ir [ch0|ch1] gate on|off           PWM_DIMx LOW while FLASH low / fixed HIGH
ir [ch0|ch1] gate inv              PWM_DIMx HIGH while FLASH low (LED off in exposure)
ir [ch0|ch1] pwm high|low          PWM_DIMx fixed HIGH / LOW (LED forced on)
ir [ch0|ch1] pwm <hz> <duty%>      PWM on PWM_DIMx (TIM12, shared frequency)
ir [ch0|ch1] ramp <mA> <ms>        gradual current change (non-blocking)
ir [ch0|ch1] dacpwm <hz> <duty%>|off  chop DAC_DIMx between 0 V and the setpoint
ir [ch0|ch1] dactest <mV>|off      DAC bench test up to 1500 mV, BYPASSES the cap
```

PWM_DIMx LOW = LED on (Q2/Q3 invert). Only one channel can be gated/strobed,
and not while the other is held LOW or in PWM: FLASH is pulled high only
through a channel whose PWM_DIMx is HIGH. While FLASH is high the LT3763 is
enabled regardless of PWM_DIMx, so outside the exposure the LED sees CTRL1
only (0 V in strobe; the DAC runs unbuffered so 0 is a true 0 mV).

The FLASH-synchronised pulse engine (ir enable/pulse, both channels) is on
branch `ir-pulse-engine`.

Sections 3-11 below describe the earlier firmware and its `ir ch<0|1> ...`
commands, which have been removed. They are kept for the hardware notes.

---

## 0. Read this first

Three things about this hardware will mislead you if you do not know them.

**The gate is inverted.** `PWM_DIM0/1` (PB14/PB15) drive the gate of an
open-drain stage (Q3/Q2 MGSF1N02LT1G, 10k pull-up to 3V3 on the drain) whose
output is `IR_CTRLx`. So `PWM_DIMx` **LOW = LEDs ON**. Out of reset PB14/PB15
are inputs, the FETs are off, and `IR_CTRLx` is already pulled to 3V3 — the LED
boards are being commanded on. `MX_GPIO_Init()` closes that window by writing
the ODR high before switching the pins to outputs. **Do not power the 18V rail
before the MCU has booted.**

**`FLASH` (PC3, pin 11) is an input only — the MCU never drives it.** The
sensor's FLASH signal is wired to it. The firmware keeps PC3 in analog mode
from boot (ADC1 reads it for `mode flash`), and `ir flash mcu` is refused.
`FLASH` is still the source of both Q2/Q3 inverters, so while it is high
`PWM_DIMx` cannot force a channel dark — `ir disable` and a latched fault only
turn off the MCU-driven gate, not light that comes through `FLASH`.

**`0 mA` does not mean dark.** The LT3763 keeps switching at `CTRL1` = 0 V
(only its `PWM` pin stops it), and its ±3 mV current-amp offset is up to ~1.2 A
across the 2.5 mΩ sense resistor. A channel gated on at `ma 0` visibly lit the
scene on the camera stream. Use `mode off` / `pulse off` to make it dark.

**`flash` mode is blocked** (`IR_FLASH_SYNC_ENABLE` = 0 in
`ir_led_driver.h`): `ir chN mode flash` is refused and ADC1 never starts,
so PC3 stays in its high-Z analog state. Set it to 1 to re-enable.

**The firmware boots dark.** `IR_LED_Init()` leaves both channels `off` with
0 setpoints. Set `IR_BOOT_FLASH_SYNC` to 1 in `ir_led_driver.h` to boot in
`flash` mode at `IR_BOOT_CURRENT_MA` (each LED on while the sensor pulls
FLASH low, thresholds 600/300 mV), or set `IR_FRAME_PULSE_DEFAULT_WIDTH_US`
to boot in `fpulse`: one `IR_FRAME_PULSE_DEFAULT_WIDTH_US` pulse at every frame start, where frame
start is the `SN0_VS` (PC4) edge, or TIM7 at 30.000 Hz while none has
arrived for 100 ms (`sync=VSYNC` / `sync=free-run 30Hz` in `ir status`). Before
bench work that needs the LEDs dark, run `ir ch0 mode off` and
`ir ch1 mode off` - but since PC3 is input-only, a channel can still light
while the sensor holds FLASH high (it reaches IR_CTRLx through Q2/Q3).
Only removing the 18V supply, or a hardware change on the FLASH net,
guarantees dark.

**`FAULT` low does not mean something is broken.** The LT3763 pulls FAULT low
whenever the inductor current is below 10% of maximum, which includes every
startup, every strobe off-window, and the whole time the rail is down. The
firmware therefore confirms faults by polling with a 50 ms blanking window
after the gate goes on, not on the falling edge.

---

## 1. Board constants

| Quantity | Value | Where from |
|---|---|---|
| Sense resistor `R11` | 2.5 mΩ | USHIO sheet 3 |
| LT3763 full-scale sense voltage | 51 mV at CTRL = 1.5 V | datasheet Fig 5 |
| Unclamped max current | 51 mV / 2.5 mΩ = **20.4 A** | — |
| `CTRL2` divider | R16 45.3k / R22 10k off 2.0 V VREF | USHIO sheet 3 |
| `CTRL2` clamp voltage | **0.361 V** | 2.0 × 10/55.3 |
| **Achievable max current** | **4.909 A** | clamp / 1.5 × 51 mV / 2.5 mΩ |
| Output regulation | FB 1.206 V × (56.2k+10k)/10k = **7.99 V** | R15/R27 |
| Output OVP | FB 1.515 V → **10.0 V** | R15/R27 |
| `EN/UVLO` rising threshold | **6.85 V** on VIN | R9 374k / R7 124k |
| Switching frequency set by | `RT` = 182k | R13 |

Because `CTRL2` is a fixed divider and the LT3763 regulates on the *lower* of
CTRL1/CTRL2, **4.909 A is a hard ceiling no matter what the DAC does**, and
only DAC codes 0–448 of 4095 do anything. See section 8.

## 2. Connector map

Proxy `J5` (ch0) / `J6` (ch1) map pin-for-pin onto the USHIO `J3` "MCU-SIGNAL":

| Pin | Net | Goes to |
|---|---|---|
| 1 | `DAC_DIMx` | LT3763 `CTRL1` (pin 14) via R19 0R — sets current |
| 2 | `IR_FAULTx` | LT3763 `FAULT` (pin 9), open drain, 100k to 3V3 |
| 3 | `IR_CTRLx` | LT3763 `PWM` (pin 22) via R26 2R2 — gates the output |
| 4 | 3V3 | |
| 5 | GND | |

MCU pins: `DAC_DIM0`=PA4 (DAC1_OUT1), `DAC_DIM1`=PA5 (DAC1_OUT2),
`PWM_DIM0`=PB14, `PWM_DIM1`=PB15, `IR_FAULT0`=PB9, `IR_FAULT1`=PE0,
`FLASH`=PC3, `IR_ENB_MCU`=PA11 (18V rail request to the PDU, *not* on J5/J6).

USHIO test points: `TP1` IVINMON, `TP2` ISMON, `TP3` EN/UVLO, `TP4` SYNC,
`TP5` CTRL2. Proxy `J17`: pin 1 `IR_CTRL0`, pin 2 `IR_CTRL1`, pin 3 GND.

---

## 3. Dry checks — 18V rail OFF, MCU powered

Console is UART4. Everything here is done with the LED supply disconnected.

```
ir status
```
Expect: `IR master : disabled`, `FLASH net: mcu(low)`, `DAC1: ready`, both
channels `mode=off`, `set=0 mA`, `code 0`. Both fault lines will read
`ASSERTED` — correct, the boards are unpowered.

If `DAC1: NOT INITIALISED`, `IR_LED_Init()` did not run — stop here.

**Check the inversion at J17** (scope or DMM, referenced to J17 pin 3):

| Command | J17 pin 1 (`IR_CTRL0`) |
|---|---|
| `ir enable` then `ir ch0 mode cont` | **3.3 V** (LEDs commanded ON) |
| `ir ch0 mode off` | **0 V** |

If you see the opposite, `FLASH`/PC3 is not being held low — check PC3 is an
output and that the host SoC is not driving the net.

Confirm PC3 directly: it must sit at 0 V. If it floats or sits at 3.3 V,
`IR_CTRL0` will be stuck high and the LEDs will be on continuously the moment
the rail comes up.

```
ir disable
```
Leave it disabled before applying power.

## 4. First power-up

1. Set the bench supply to **18 V**, current limit **1 A** for now.
2. Confirm `ir status` still shows `disabled` and both modes `off`.
3. Apply the rail. `TP3` (EN/UVLO) should sit above 1.705 V — it is a divider
   off VIN, so the board enables itself at VIN ≈ 6.85 V. There is no MCU
   enable on J3.
4. Check `TP5` (`CTRL2`) = **0.361 V ± 5%**. This is the ceiling. If it reads
   anything else, stop and re-check R16/R22 before drawing current.
5. Check `J5` pin 1 (`CTRL1`) = **0 V**. The DAC is at code 0.
6. Supply current should be a few tens of mA — the LT3763 is enabled but
   `PWM` is low, so it is not switching.

## 5. DAC and current-setpoint check — still no illumination

With the channel still `off` (so `IR_CTRL0` is low and nothing switches), sweep
the DAC and verify `CTRL1` on `J5` pin 1 tracks it:

```
ir ch0 raw 0      ->  J5.1 ~= 0 mV
ir ch0 raw 124    ->  J5.1 ~= 100 mV
ir ch0 raw 248    ->  J5.1 ~= 200 mV
ir ch0 raw 365    ->  J5.1 ~= 294 mV   (this is the 4 A point)
ir ch0 raw 448    ->  J5.1 ~= 361 mV   (the CTRL2 clamp)
```

`ir limits` prints the same numbers the firmware is using. Then set it in
engineering units and check the mapping agrees:

```
ir ch0 ma 4000
```
Expect `SUCCESS: ch0 = 3998 mA (CTRL1 294 mV, code 365)`.

Reference points for the conversion (verified against the firmware's integer
math, worst round-trip error 7 mA over 0–4.9 A):

| Requested | DAC code | CTRL1 | Readback | V(SENSE+ − SENSE−) |
|---|---|---|---|---|
| 100 mA | 9 | 7 mV | 95 mA | 0.24 mV |
| 500 mA | 46 | 37 mV | 503 mA | 1.26 mV |
| 1000 mA | 92 | 74 mV | 1006 mA | 2.52 mV |
| 2000 mA | 182 | 147 mV | 1999 mA | 5.00 mV |
| 3000 mA | 274 | 221 mV | 3006 mA | 7.52 mV |
| **4000 mA** | **365** | **294 mV** | **3998 mA** | **10.00 mV** |
| 4900 mA | 447 | 360 mV | 4896 mA | 12.24 mV |
| >4909 mA | 448 | 361 mV | 4910 mA | clamped by CTRL2 |

Set it back to zero before illuminating:
```
ir ch0 ma 0
```

## 6. First illumination and the ramp to 4 A

Raise the supply current limit to **6 A** (4 A of LED current at ~8 V out from
18 V in is roughly 2 A of input current, plus margin — but leave headroom for
inrush into C18/C9).

Put a scope on `TP2` (`ISMON`). **ISMON reads 1.0 V per 50 mV of sense
voltage, i.e. 20 A per volt** — so 4 A should read **0.200 V**.

```
ir enable
ir ch0 mode cont
ir ch0 ramp 4000
```

The ramp walks the setpoint up in 100 mA steps with a 20 ms dwell (≈800 ms
total), running the fault check at every step. It prints `ABORTED` and stops
climbing if the channel faults or the rail drops on the way up.

At each stage verify:

| Check | Expected at 4 A |
|---|---|
| `TP2` ISMON | 0.200 V |
| LED string voltage (J2) | ~8 V |
| `TP1` IVINMON | 20 × 2.5 mΩ × I_in ≈ 0.10 V at 2 A input |
| Supply current | ~2.0–2.3 A from 18 V |
| `ir status` fault line | `clear`, `latched=no` |

Then repeat the whole of sections 5–6 for channel 1 on `J6` / `PE0`.

> Current is **hard-capped at 4000 mA** (`IR_LED_HARD_LIMIT_MA`, raised from
> 3000 on 2026-10-02), enforced on the DAC code itself: `ma` and `mv` stop at
> code 365 (294 mV on CTRL1, ~3998 mA). Changing it needs a rebuild.

## 7. Fault behaviour

Expected, and **not** a defect:

- `ir status` shows `line=ASSERTED` on any channel that is dark or unpowered.
- `edges` climbing while you enable/disable.

A real fault is `latched=YES`, which only happens if the line is still low
50 ms after the gate went on. Provoke one by disconnecting the LED string at
J2 with the channel illuminated (LED open → FB > 1.16 V): within ~50 ms the
channel gates off, the master enable drops, and `ir status` shows
`latched=YES`. Recover with:

```
ir ch0 clearfault
ir enable
```

Clearing the fault deliberately does **not** restore the rail on its own.

**Limitation to be aware of:** in `strobe` mode the gate goes off before the
50 ms blanking can complete, so faults are effectively unmonitored there.
Watch `edges` in `ir status` instead, or fault-test in `cont` mode.

## 8. Known hardware issue — the CTRL2 ceiling

`CTRL2` is tied to a fixed 45.3k/10k divider off VREF, giving 0.361 V. Since
the LT3763 regulates on the lower of CTRL1/CTRL2, the board is limited to
**4.909 A**, and the `CTRL1` DAC only does anything over its bottom ~11%
(codes 0–448 of 4095), costing ~3.4 bits of setpoint resolution — about 11 mA
per code.

The `J2` LED connector is annotated "8V/20AMPS" and the 2.5 mΩ sense resistor
is sized for 20.4 A, so the clamp looks unintended. **Raise with the hardware
team.** If it is intentional as a safety clamp, consider moving it to ~1.55 V
(e.g. R16 10k / R22 45.3k → 1.65 V) so CTRL1 keeps full resolution and the
clamp only backstops.

No firmware change is needed either way — update `IR_CTRL2_R_TOP_OHMS` and
`IR_CTRL2_R_BOT_OHMS` in `ir_led_driver.h` and the ceiling, the mA mapping and
the CLI output all follow. Setting `IR_CTRL2_R_TOP_OHMS` to 0 models "CTRL2
tied to VREF" and scales to the full 20.4 A.

`CTRL2` is the LT3763's designated *thermal* derating input. The proxy board
already has `LED0_TEMP_ADCIN` (PC11) and `LED1_TEMP_ADCIN` (PA15) coming back
from the `J7` IR TEMP connector, but those are MCU-side and cannot reach
`CTRL2`, which is on the LED board. Thermal derating therefore has to be done
in firmware by lowering the `CTRL1` setpoint — not yet implemented.

## 9. Frame-synchronised strobing

Only once continuous mode is solid at the target current.

```
ir sync
```
Run it twice. `frames` must climb and `lines` must wrap at the sensor's VMAX.
If `frames` stays at 0, `SN0_VS` (PC4) is not arriving and strobe mode will
leave the channel dark.

```
ir ch0 window 0 2160
ir ch0 mode strobe
```

`window <start> <end>` is in sensor lines counted from the `SN0_VS` edge.
Replace 2160 with the sensor's actual active line count. Scope `J17` pin 1
against `SN0_VS` to confirm the window lands where the exposure is.

Note `SN0_HS` fires once per line — ~65 kHz at 2160 lines and 30 fps, roughly
one interrupt per 980 CPU cycles at 64 MHz. `IR_LED_OnHSync()` is deliberately
short and writes BSRR directly for this reason. If the line rate goes higher,
move the line counter to a timer in external-clock mode instead of EXTI.

## 10. Host-driven strobe (optional)

`FLASH` is always an input, so to let it gate a channel directly:

```
ir ch0 mode host       # PWM_DIM0 high, so IR_CTRL0 follows FLASH
ir enable
```

`PWM_DIMx` alone can never force a channel dark while `FLASH` is high.
`ir disable` and any confirmed fault drop `host` channels to `off`, but they
cannot pull `FLASH` low.

---

## Quick reference

See "Current operating mode" at the top: `ir status`, `ir ma <mA>`,
`ir mv <mV>`. The commands used in sections 3-11 are no longer built.

## 11. DAC-generated PWM on DAC_DIM0 / DAC_DIM1

`ir ch<0|1> pwm <hz> <duty%>` chops the DAC output between 0V and whatever
level `mv`/`ma`/`raw` last set, instead of holding a DC level. TIM12 provides
the timebase: its update event raises the output, its capture/compare event
drops it (CH1 owns ch0, CH2 owns ch1). TIM12's outputs are never enabled, so
PB14/PB15 - the LED gates - are untouched.

Range 1Hz to 10kHz, duty 0-100. Duty 0 and 100 park the output at 0V or at
the setpoint and stop the timer rather than running it for a steady state.
Both channels share one frequency (one timer); duty is per channel.

Verified on hardware at 100Hz/90%: DAC_DIM0 toggling 0V <-> 500mV with the
DAC output register reading 92% high across 400 samples, and TIM12 at
PSC=9, ARR=63999 (64MHz / 10 / 64000 = 100.0Hz).

To see it on a scope, prefer a large slow signal - a high duty at a low
amplitude is a narrow notch on a near-flat line and is easy to miss:

```
ir ch0 mv 2000
ir ch0 pwm 10 50
```

2V p-p at 10Hz. Probe PA4 (MCU pin 20), or J5 pin 1, ground on J5 pin 5.
DC coupled, 500mV/div, 20ms/div, normal trigger on the rising edge.
2000mV is safe - below the 3V CTRL1 absolute maximum - but it is far above
the 361mV CTRL2 clamp, so it buys no extra LED current.

**What this does not do.** CTRL1 feeds the LT3763's average-current loop,
compensated by R23 44.2k + C16 8.2nF on VC with a further 33nF (C13) across
the sense inputs. That loop cannot slew as fast as the DAC steps, so the LED
*current* is a rounded, slewed version of the voltage waveform, and above
roughly a few hundred Hz the optical output approaches a steady average
rather than chopping. The pin waveform is clean; the light is not. For
genuine PWM dimming the LT3763 has a dedicated PWM pin (22), reached from
PWM_DIM0/1 - note the inverter there makes the duty complementary.
