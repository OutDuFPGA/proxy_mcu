# Lens zoom/focus control — design and guard rails

Board: proxy MCU (STM32H563), three TMC2300 STEP/DIR drivers on a shared
single-wire UART (USART1/PA9, 9600 Bd). Lens: NV-0622IR(5MP)-M14 6–22 mm,
motors D14-0622IR(5MP)-EE (2-phase PM stepper, 1:80.3 gearbox, 40 Ω coil,
rated 4.0 V ±20 %, max start 800 pps, max slew 1200 pps). No limit switches,
no encoder. Code: `Secure/Core/Src/lens_ctrl.c`, SoC link `host_link.c`.

## 1. Hardware facts that drive the design

| Fact | Source | Consequence |
|---|---|---|
| Driver U1 (TIM3/PB5, addr 0) turns the **focus** motor, U2 (TIM8/PC7, addr 1) the **zoom** motor | bench, 2026-11-13 | `LENS_SWAP_ZOOM_FOCUS 1` in `lens_ctrl.h` |
| U2 does not answer on the UART | `lens scan` | zoom runs only in test mode (driver reset settings) until fixed |
| GSTAT.u3v5 = 1 on every driver: VS < 3.5 V (live comparator, 3.15–3.5 V) | TMC2300 ds §5.1, el. char. | motors run at ~3.3 V vs 4.0 V rated → low torque. **Fix: feed M1/M2_PWR from 5 V (L5/L4)**. Firmware caps speed at 600 pps while u3v5 |
| Coil 40 Ω ⇒ current ≤ VS/40 (≈80 mA at 3.3 V, 125 mA at 5 V) | lens ds | current is coil-limited; IRUN ≥ 8 (ds: <8 not recommended); default 16 |
| Reset defaults: IRUN 31, IHOLD 8, MRES 32 (UART mode), CHOPCONF 0x13008001, PWMCONF 0xC40D1024 | ds §5 | an unconfigured driver still runs; position units differ (32 µstep) |
| DIAG is push-pull, high on driver error or (if enabled) stall | ds §2.2, §14.4 | polled every step in the STEP ISR |
| StallGuard4 only in StealthChop, only while TCOOLTHRS ≥ TSTEP, unreliable at low speed/low back-EMF | ds §10.4 | stall detection is an **optional extra layer**, never the only protection |

## 2. Axis state machine

```
            power-up / driver reset / stall / clr
                         │
                         ▼
   ┌────────────► UNHOMED ──home──► HOMING ──done──► READY
   │                 ▲                │ abort/stop      │
   │                 └────────────────┘                 │ stall → UNHOMED
   │                                                    │
   └──── clr ◄────────── FAULT ◄──── any fault (any state)
```

* **Position moves (absolute/relative, tracking, host jog) require READY.**
  Console calibration jogs work while UNHOMED but stay inside the soft limits,
  have a 30 s hard time limit and use the normal ramps.
* Zero is defined **only by homing**. Host `ZERO` was removed; console `zero`
  is test-mode only.

## 3. Homing (no switches)

1. Drive **reverse** at `home_pps` (default 300 full steps/s, below the 800 pps
   start limit) for at most `home_max_fs` full steps (default 2500 — must be
   larger than the whole travel). The reverse soft limit is ignored only in
   this phase.
2. The move ends at the mechanical stop either by StallGuard (if `sg` is tuned)
   or by running out the distance; at the stop the PM motor simply slips, with
   coil-limited current.
3. That point is `-home_backoff_fs` (default 8 full steps). The axis moves
   forward to 0 and becomes READY → event `homed`.
4. Any stop, fault or timeout during homing → `home-abort`, axis UNHOMED.

Homing also gives the TMC2300 its StealthChop auto-tuning conditions
(AT#1 standstill at IRUN, AT#2 movement) after power-up (ds §6.1).

## 4. Guard rails (all independent)

| # | Guard | Where | Action |
|---|---|---|---|
| 1 | Datasheet speed limits: start ≤ 800, max ≤ 1200 pps (full steps); refused above | config | — |
| 2 | Speed cap 600 pps while VS < 3.5 V | `axis_rates` | derate |
| 3 | Trapezoidal ramp on every move, including stop and retarget | STEP ISR | — |
| 4 | Soft limits `[0, max_pos]`, checked on every step (also jogs) | STEP ISR | stop at limit (`limit`) |
| 5 | Uncalibrated travel → `max_pos` = 2500 fs default, warning `TRAVEL?` | defaults | — |
| 6 | Requests out of range refused (`ERR RANGE`), never clamped silently | `move_to_raw` | — |
| 7 | Moves need READY (`ERR HOME`) | `gate()` | — |
| 8 | Unconfigured / unverified driver: no motion (`ERR NODRV`); config proven by IFCNT (+7 writes) and CHOPCONF read-back | `axis_configure` | — |
| 9 | DIAG high during a move (after 8 full-step blanking) | STEP ISR | immediate stop, classify |
| 10 | Classify DIAG: ot/t150 → `OT`, s2g/s2vs → `SHORT`, drv_err or stall-detection off → `DRV`, else stall → UNHOMED | `classify_diag` | fault / re-home |
| 11 | Driver health poll every 100 ms (round robin, GSTAT/DRV_STATUS): reset → re-config + UNHOMED; drv_err → `DRV`; ot → `OT`; shorts → `SHORT`; otpw/t120 → no new moves; open load (moving) → warning | `health_poll` | fault / warn |
| 12 | Lost UART to a driver (5 consecutive GSTAT failures) | `health_poll` | fault `COMM` |
| 13 | Move watchdog: deadline = distance at start speed + 1.5 s; overrun = firmware/hardware fault | `LENS_Task` | halt, fault `TIMEOUT` |
| 14 | Jog dead-man: host jog must be repeated within 600 ms; console jog max 30 s | `LENS_Task` | decelerated stop (`deadman`) |
| 15 | Motor duty bucket: fills while moving, drains at ½ rate; > 120 s → no new moves (`ERR HOT`), > 180 s → stop | `LENS_Task` | derate / stop |
| 16 | Faults latch; cleared only by `CLR` (re-configures driver, axis → UNHOMED) | `axis_fault` | — |
| 17 | Every stop/fault reported asynchronously: `DONE <axis> <pos> <reason>` | `LENS_Task` | — |
| 18 | Test mode (`lens <ax> allow on`, console only) bypasses NODRV/HOME but **not** limits, ramps, faults, watchdog, dead-man, heat | `gate()` | — |

## 5. Automatic zoom/focus

* SoC sends zoom targets (`Z`, `ZR`, `ZF`, dead-man `ZJ`). With tracking follow
  on (`TF 1`), focus is driven to the calibrated curve for the new zoom
  position at the current subject distance (`TD` sets it explicitly).
* Autofocus stays on the SoC (it owns the image statistics): it issues `F`/`FR`
  steps and waits for `DONE F`.
* Tracking curve: up to 16 points × focus at ∞/3 m/1 m/0.3 m, linear in zoom,
  linear in diopters between distances (see `lens track`).

## 6. Bring-up / calibration procedure

1. `lens clr` → check `lens status`: drivers `cfg`, no faults. Fix VS (5 V)
   and U2's UART first for a production setup.
2. `lens <ax> home` for zoom and focus → `homed`.
3. `lens <ax> jog 1` to the far end, `lens <ax> stop`, `lens <ax> setmax`
   (travel = position − backoff; homing distance = 9/8 travel + margin).
4. Optional stall detection: during a move, watch `lens <ax> reg 0x41`
   (SG_VALUE); set `lens <ax> sg <half the lowest value before stall>`.
5. Tracking table: `lens track cap <i> inf|3m|1m|0.3m` at ~9 zoom points.
6. `lens autohome on`, `lens save` (flash, bank 1 last sector, CRC-checked).

## 7. Known limits

* No absolute position sensor: if the gear slips without a stall being
  detected, position is wrong until the next homing. Home after every power-up
  (`autohome on` or SoC `H ALL`).
* Config sector write stalls flash bank 1 for a few ms: saved only on request
  and only with all axes idle.
* With VS ≈ 3.3 V and U2 not on the UART, the system cannot be made fully safe
  in firmware alone — both are hardware fixes.
