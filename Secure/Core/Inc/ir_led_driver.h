/**
  ******************************************************************************
  * @file    ir_led_driver.h
  * @brief   LT3763 IR LED driver control for the two USHIO illuminator boards.
  *
  * ===========================================================================
  * SIGNAL PATH (verified against SCH_PROXY_BOARD_V1 sheet 5 and
  *              Ushio_IR_LED_DriverV2 sheet 3)
  * ===========================================================================
  *
  * Proxy J5 (ch0) / J6 (ch1) are 5-pin and map pin-for-pin onto the USHIO
  * board's J3 "MCU-SIGNAL" header:
  *
  *   pin 1  DAC_DIMx   -> J3.1 -> R19 (0R)  -> LT3763 CTRL1 (pin 14)
  *   pin 2  IR_FAULTx  <- J3.2 <-            <- LT3763 FAULT (pin 9, open drain)
  *   pin 3  IR_CTRLx   -> J3.3 -> R26 (2R2) -> LT3763 PWM   (pin 22)
  *   pin 4  3V3
  *   pin 5  GND
  *
  * ---------------------------------------------------------------------------
  * 1. THE GATE IS INVERTED, AND ITS LOW LEVEL COMES FROM THE FLASH NET
  * ---------------------------------------------------------------------------
  * IR_CTRLx is NOT driven directly by the MCU. On proxy sheet 5 each channel
  * has an open-drain inverter:
  *
  *        VCC_3.3
  *           |
  *          R43/R41  10k
  *           |
  *           +---------------> IR_CTRLx  (to J5/J6 pin 3)
  *           |
  *          D|
  *   PWM_DIMx --R44/R42 10k-- G|  Q3/Q2  MGSF1N02LT1G
  *          S|
  *           |
  *         FLASH  (PC3, also wired to the host SoC connector on sheet 3)
  *
  * (R9/R8, the 0R drain-source bypass, are marked DNI and are not fitted.)
  *
  * So:
  *   PWM_DIMx = 0  ->  Q off  ->  IR_CTRLx = 3V3     -> LEDs ON
  *   PWM_DIMx = 1  ->  Q on   ->  IR_CTRLx = FLASH
  *
  * PC3 is an INPUT ONLY (the sensor's FLASH is wired to it) - the MCU never
  * drives it. So with PWM_DIMx = 1 the external FLASH level passes straight
  * through to IR_CTRLx: the MCU can force a channel ON (PWM_DIMx = 0) but
  * can only make it dark while FLASH is low.
  *
  * The practical consequence, and the reason this file exists in its current
  * form: PWM_DIMx LOW means LIGHT ON. Every gate write below goes through
  * gate_set()/IR_GATE_ON/IR_GATE_OFF so the inversion is stated in exactly
  * one place.
  *
  * Note the reset hazard this creates. Out of reset, before MX_GPIO_Init()
  * runs, PB14/PB15 are inputs, so Q is off and IR_CTRLx is pulled to 3V3 --
  * the LEDs are commanded ON. MX_GPIO_Init() therefore writes PWM_DIM0/1
  * HIGH before switching them to outputs. IR_ENB_MCU/IR_FBC_MCU (PA11/PA12)
  * are the IR-cut filter's DRV8210 EN/PH and stay low except during a pulse.
  *
  * ---------------------------------------------------------------------------
  * 2. CURRENT CONTROL, AND THE FIXED CTRL2 CEILING
  * ---------------------------------------------------------------------------
  * The LT3763 regulates the LED current so that the voltage across the sense
  * resistor follows whichever of CTRL1 / CTRL2 / SS / FBIN is LOWEST. Over
  * 0 to 1.5V the mapping is linear onto 0 to 51mV (datasheet Rev C, Fig 5 and
  * the "Output Current Regulation" section); above 1.5V nothing more happens.
  *
  * On the USHIO board the sense resistor is R11 = 2.5mR, so full scale is
  *     51mV / 2.5mR = 20.4A     (matches the "8V/20AMPS" LED connector note)
  *
  * REWORKED 2026-10-13: R16 is now 0R (user-confirmed), so CTRL2 = VREF =
  * 2.0V, above the 1.5V saturation point - no clamp, full 20.4A available.
  * IR_CTRL2_R_TOP_OHMS below is 0 accordingly. The original fit was:
  *
  * CTRL2 (pin 13) was tied to a fixed divider off the 2V VREF pin:
  *     R16 = 45.3k from VREF, R22 = 10k to GND
  *     V_CTRL2 = 2.0V * 10 / 55.3 = 0.3617V
  *
  * Since the lowest control pin wins, the board cannot exceed
  *     51mV * 0.3617/1.5 = 12.30mV  ->  12.30mV / 2.5mR = 4.92A
  *
  * no matter what the MCU puts on CTRL1. This driver therefore scales its
  * milliamp setpoint over 0..IR_LED_MAX_CURRENT_MA (~4920mA), which uses DAC
  * codes 0..~449 out of 4095 and gives roughly 11mA of resolution per step.
  *
  * >>> If a board revision changes R16/R22, update IR_CTRL2_R_TOP_OHMS and
  * >>> IR_CTRL2_R_BOT_OHMS below and everything else follows. Setting
  * >>> IR_CTRL2_R_TOP_OHMS to 0 models "CTRL2 tied to VREF", i.e. no clamp,
  * >>> and the driver will scale to the full 20.4A instead.
  *
  * ---------------------------------------------------------------------------
  * 3. FAULT IS A LIVE STATUS LINE, NOT AN EDGE
  * ---------------------------------------------------------------------------
  * LT3763 FAULT (pin 9) is open drain, pulled up to 3V3 through R24/R25 100k
  * on the LED board, so it is ACTIVE LOW. The datasheet condition is:
  *
  *   "Internal comparators pull down this pin when the FB pin voltage is
  *    lower than 0.25V or higher than 1.16V AND when the inductor current is
  *    less than ten percent of the maximum value."
  *
  * The inductor-current term is the important one. It means FAULT is asserted
  * during every normal startup, and during any interval where the converter
  * is not delivering current -- including the off part of a PWM strobe and
  * the entire time the 18V rail is down. Latching an error on the falling
  * edge, as a naive reading suggests, makes the driver fault instantly and
  * permanently on the first enable.
  *
  * So faults are confirmed by polling, not by edges: IR_LED_Task() latches a
  * fault only if the line is still low after the channel's gate has been
  * continuously ON for IR_FAULT_BLANKING_MS. The EXTI handlers only record
  * that an edge happened (for diagnostics and to prompt an early re-check).
  * A latched fault is sticky until IR_LED_ClearFault().
  *
  * ---------------------------------------------------------------------------
  * 4. WHAT IS NOT AVAILABLE
  * ---------------------------------------------------------------------------
  * ISMON (pin 19, output current monitor) and IVINMON (pin 8, input current
  * monitor) terminate at test points TP2 and TP1 on the LED board. They are
  * not on J3, so the MCU cannot read back actual LED current -- the setpoint
  * reported by IR_LED_GetCurrent_mA() is open loop. Likewise the LED board
  * has no enable input: its EN/UVLO is a divider off VIN (R9 374k / R7 124k,
  * rising threshold ~6.85V), so it powers up whenever the 18V rail is up.
  * IR_ENB_MCU (PA11) and IR_FBC_MCU (PA12) control the IR-cut filter
  * (DRV8210 EN/PH, user-confirmed 2026-10-05) - NOT an 18V rail request.
  ******************************************************************************
  */
#ifndef __IR_LED_DRIVER_H__
#define __IR_LED_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
   Board constants. Change these, not the code, when the hardware changes.
   --------------------------------------------------------------------------- */

/* USHIO board R11, the LED current sense resistor, in micro-ohms. */
#define IR_RSENSE_UOHM              2500U

/* LT3763 full-scale sense voltage, in microvolts, reached at CTRL = 1.5V. */
#define IR_LT3763_VSENSE_FS_UV      51000U

/* CTRL input voltage, in millivolts, at which the sense voltage saturates. */
#define IR_LT3763_VCTRL_FS_MV       1500U

/* USHIO board CTRL2 divider off the 2V VREF pin: R16 top, R22 bottom.
   Set IR_CTRL2_R_TOP_OHMS to 0 to model CTRL2 tied straight to VREF. */
#define IR_CTRL2_VREF_MV            2000U
#define IR_CTRL2_R_TOP_OHMS         0U        /* R16 reworked to 0R (was 45.3k) */
#define IR_CTRL2_R_BOT_OHMS         10000U

/* DAC reference, i.e. VDDA, in millivolts. */
#define IR_DAC_VREF_MV              3300U

/* LT3763 Absolute Maximum Rating for the CTRL1/CTRL2/FB/FBIN pins, in
   millivolts (datasheet Rev C, Absolute Maximum Ratings). DAC_DIMx reaches
   CTRL1 through R19, which is a 0 ohm link - there is nothing in series to
   drop the difference. Full-scale DAC output is IR_DAC_VREF_MV, which is
   ABOVE this, so a full-scale setpoint over-stresses the part unless J5/J6
   are unplugged. The driver still allows it (bench characterisation of the
   DAC itself is a legitimate thing to want) but the console warns. */
#define IR_CTRL_ABS_MAX_MV          3000U

/* Hard cap on the LED current, in milliamps. Enforced on the DAC code itself
   in IR_LED_SetCurrentRaw(), so no entry point - mA, mV, raw code, ramp or
   DAC PWM - can command more, and "ir limit" cannot raise the soft cap
   above it. Compile-time only. It caps the SETPOINT: with CTRL1 near 0V the
   LT3763's current-amp offset can still pass up to ~1.2A (see
   IR_LED_Init), which is below this cap. */
#define IR_LED_HARD_LIMIT_MA        4000U

/* High-current pulses (IR_LED_SetHighPulse) - the ONLY path above the hard
   cap. USHIO SMBB850DS-1200: IF = 4A DC, IFP = 20A. The datasheet's
   "Forward Current - Pulse Duration" curve gives, for repetition rate f,
   20A up to 1% duty and then a straight log-log line down to 4A at 100%
   duty, i.e.
       duty_max(I) = 1% * (20A / I) ^ (ln 100 / ln 5)      for 4A < I <= 20A
   At 30Hz: 20A <= 333us, 15A <= ~760us, 10A <= ~2.4ms, 6A <= ~10ms.
   IR_LED_HighPulseMaxWidth_us() applies this at IR_HP_DERATE_PCT of the
   curve, with f = IR_FRAME_PULSE_FPS; the pulse engine refuses to fire two
   pulses closer than IR_HP_MIN_SPACING_US so the rate cannot exceed f. */
#define IR_LED_PULSE_LIMIT_MA       20000U
#define IR_HP_DERATE_PCT            100U
#define IR_HP_MIN_SPACING_US        ((1000000U / IR_FRAME_PULSE_FPS) - 1000U)

/* Bench safety cap on the commanded current, in milliamps, applied on top of
   the hard cap above. Adjustable at runtime (up to IR_LED_HARD_LIMIT_MA)
   with IR_LED_SetCurrentLimit_mA() / "ir limit <mA>". */
#define IR_LED_DEFAULT_LIMIT_MA     IR_LED_HARD_LIMIT_MA

/* Default step size and dwell for IR_LED_RampCurrent_mA(). 100mA every 20ms
   takes a 0 -> 4A ramp about 800ms, which is slow enough to watch the ISMON
   test point (TP2) and the LED string settle at each step, and slow enough
   that IR_LED_Task() gets many chances to catch a fault on the way up. */
#define IR_LED_RAMP_STEP_MA         100U
#define IR_LED_RAMP_DWELL_MS        20U

/* How long a channel's gate must have been continuously ON before a low
   FAULT line is believed. Covers LT3763 soft-start (C14 = 4.7nF at 11uA
   reaches the 0.36V CTRL2 clamp in well under 1ms) plus output capacitor
   charging into the LED string, with a wide margin. */
#define IR_FAULT_BLANKING_MS        50U

/* ---------------------------------------------------------------------------
   FLASH sync (IR_MODE_FLASH_SYNC)
   ---------------------------------------------------------------------------
   The sensor's exposure signal arrives on PC3 at 1.8V, too low for the 3.3V
   digital input (VIH 2.31V guaranteed), so ADC1 samples it and its analog
   watchdogs detect the edges - see the FLASH sync section of
   ir_led_driver.c. While FLASH is active, every FLASH_SYNC channel's gate is
   ON at its current setpoint; otherwise OFF. */

/* Hysteresis thresholds on PC3, in millivolts: rising past HIGH counts as a
   high level, falling past LOW as a low level. Measured on the board
   (2026-10-01): the sensor's open collector pulls FLASH to 0-80mV; released,
   FLASH is pulled to ~2.3V only through Q2/Q3, i.e. only while the gates are
   off. While a gate is on, a released FLASH floats at ~0.9-1.4V. So "low"
   must mean "actively pulled" (< LOW) and anything above HIGH is released -
   thresholds above the float level latch the gate on (seen: 20ms max-on
   cut at 30Hz). Adjustable at runtime: "ir flash thresh". */
#define IR_FLASH_TH_HIGH_MV             600U
#define IR_FLASH_TH_LOW_MV              300U

/* Which level means "exposing". false = active LOW, i.e. nFLASH: TOUT0
   inverted by Q1 on the sensor board. Set true if TOUT0 is wired straight
   to PC3 (TOUT0 is high during exposure). Runtime: "ir flash polarity". */
#define IR_FLASH_ACTIVE_HIGH_DEFAULT    false

/* Safety: a FLASH that stays active longer than this (one 30fps frame plus
   margin) gates the LEDs off until FLASH next goes idle. */
#define IR_FLASH_MAX_ON_MS              40U

/* ---------------------------------------------------------------------------
   Frame pulse (IR_MODE_FRAME_PULSE) and boot defaults
   --------------------------------------------------------------------------- */

/* Image sensor on this board - selects how PC3 (pin 11, FLASH net) is used.
   IMX568: PC3 is the sensor's FLASH output, an MCU INPUT (FLASH monitor,
     gate, strobe, ADC sync); only "fhsync" borrows it as an output while it
     runs (user-approved 2026-10-14, sensor FLASH disabled).
   IMX678: PC3 is an MCU OUTPUT from boot, held LOW, driven by "fhsync"; the
     FLASH-input features are refused. (User instruction 2026-10-14.)
   Set by tools/build.mk (SENSOR=IMX568|IMX678); defaults to IMX568, so a
   build without the define (e.g. STM32CubeIDE) keeps the IMX568 wiring. */
#define IR_SENSOR_IMX568                568
#define IR_SENSOR_IMX678                678
#ifndef IR_SENSOR
#define IR_SENSOR                       IR_SENSOR_IMX568
#endif
#if (IR_SENSOR != IR_SENSOR_IMX568) && (IR_SENSOR != IR_SENSOR_IMX678)
#error "IR_SENSOR must be IR_SENSOR_IMX568 or IR_SENSOR_IMX678"
#endif
#define IR_PC3_OUTPUT                   (IR_SENSOR == IR_SENSOR_IMX678)

/* Sensor frame rate. The IMX568 is configured for 30 fps, so without a
   VSYNC edge the MCU generates its own frame start at this rate. */
#define IR_FRAME_PULSE_FPS              30U

/* A VSYNC edge older than this means sync is lost: TIM7 takes over and fires
   the pulses at IR_FRAME_PULSE_FPS until VSYNC edges return. */
#define IR_VSYNC_TIMEOUT_MS             100U

/* Longest pulse allowed: one frame period less a margin, so a pulse always
   ends before the next frame start can re-trigger it. */
#define IR_FRAME_PULSE_MAX_US           ((1000000U / IR_FRAME_PULSE_FPS) - 333U)

/* OPERATING MODE (2026-10-02): the sensor's FLASH gates the LED. Both
   channels stay IR_MODE_OFF. PWM_DIM0 is driven from the PC3 edge
   interrupts: LOW (LED forced on) while FLASH is low - the exposure - and
   HIGH otherwise (IR_LED_SetFlashGate, on at boot). While PWM_DIM0 is HIGH,
   Q2/Q3 pass FLASH straight to IR_CTRLx, so the LED is also on whenever
   FLASH is high. The MCU sets the current through DAC_DIM0 (CTRL1).
   Only CH0 has an LED driver fitted; CH1's DAC is left at 0.
   IR_LED_Init() sets CH0 to IR_BOOT_CURRENT_MA. */
#define IR_BOOT_CURRENT_MA              1000U

/* FLASH sync (IR_MODE_FLASH_SYNC) is BLOCKED while this is 0: SetMode
   refuses it and ADC1 is never started, so PC3 is never touched beyond its
   boot-time analog (high-Z) configuration. */
#define IR_FLASH_SYNC_ENABLE            0

/* FLASH input monitor: no PC3 edge for this long means "not toggling". */
#define IR_FLASH_IDLE_MS                200U

/* ---------------------------------------------------------------------------
   Derived limits. Computed at compile time from the constants above.
   --------------------------------------------------------------------------- */

/* CTRL2 clamp voltage in millivolts, or the full-scale CTRL voltage when no
   divider is fitted. */
#if (IR_CTRL2_R_TOP_OHMS == 0)
#define IR_CTRL_CEILING_MV          IR_LT3763_VCTRL_FS_MV
#else
#define IR_CTRL_CEILING_MV \
    ((uint32_t)((IR_CTRL2_VREF_MV * (uint64_t)IR_CTRL2_R_BOT_OHMS) / \
                (IR_CTRL2_R_TOP_OHMS + IR_CTRL2_R_BOT_OHMS)))
#endif

/* Highest LED current the board can actually reach, in milliamps.
   = VSENSE_FS * (CTRL_CEILING / VCTRL_FS) / RSENSE */
#define IR_LED_MAX_CURRENT_MA \
    ((uint32_t)(((uint64_t)IR_LT3763_VSENSE_FS_UV * IR_CTRL_CEILING_MV * 1000U) / \
                ((uint64_t)IR_LT3763_VCTRL_FS_MV * IR_RSENSE_UOHM)))

/* ---------------------------------------------------------------------------
   Types
   --------------------------------------------------------------------------- */

typedef enum {
    IR_CH0 = 0,          /* J5, DAC_DIM0/PA4, PWM_DIM0/PB14, IR_FAULT0/PB9  */
    IR_CH1 = 1,          /* J6, DAC_DIM1/PA5, PWM_DIM1/PB15, IR_FAULT1/PE0  */
    IR_CH_COUNT
} IR_Channel_t;

typedef enum {
    IR_MODE_OFF = 0,          /* Gate held off.                                  */
    IR_MODE_CONTINUOUS,       /* Gate held on. No sensor sync needed.            */
    IR_MODE_FRAME_STROBE,     /* Gated window per frame, counted in sensor lines */
    IR_MODE_HOST_FLASH,       /* Gate follows the host SoC's FLASH line          */
    IR_MODE_GATE_PWM,         /* TIM12 chops the gate: true PWM dimming          */
    IR_MODE_FRAME_PULSE,      /* One timed pulse per sensor frame (VSYNC/30Hz)   */
    IR_MODE_FLASH_SYNC        /* Gate follows sensor FLASH on PC3 via ADC watchdog */
} IR_LedMode_t;

typedef enum {
    /* PC3 driven low by this MCU. NOT SUPPORTED: PC3 is an input only (the
       sensor's FLASH is wired to it); IR_LED_SetFlashSource() refuses it. */
    IR_FLASH_SRC_MCU = 0,
    /* PC3 is an analog input; FLASH is driven externally. Always the case. */
    IR_FLASH_SRC_HOST
} IR_FlashSource_t;

/* ---------------------------------------------------------------------------
   Lifecycle
   --------------------------------------------------------------------------- */

/* Call once from main(), after MX_GPIO_Init(). Brings up DAC1, forces both
   gates off, leaves FLASH (PC3) an input - never driven, sets both current
   setpoints to 0 and leaves IR_ENB_MCU deasserted. It then applies the boot
   default: both channels IR_MODE_OFF (PWM_DIMx HIGH, LED follows FLASH)
   and CH0 at IR_BOOT_CURRENT_MA.
   Note that a 0 mA setpoint does NOT mean dark: the LT3763 keeps regulating
   at CTRL1 = 0V (its datasheet says only the PWM pin stops switching), and
   its +/-3mV current-amp offset alone is up to ~1.2A across R11. The gate is
   the only real off switch - confirmed on the camera stream, where a gated
   channel at "0 mA" visibly lit the scene. */
void IR_LED_Init(void);

/* Call from the main loop. Runs fault confirmation (see note 3 in the file
   header) and nothing else -- it does no I/O and returns immediately when
   there is nothing to do, so it is cheap to call every iteration. */
void IR_LED_Task(void);

/* ---------------------------------------------------------------------------
   Master enable
   --------------------------------------------------------------------------- */

/* Software output interlock. Forces both gates off while disabled and
   restores each channel's configured mode when re-enabled.

   NOTE: this is SOFTWARE ONLY. It does not touch IR_ENB_MCU (PA11), which
   is the IR-cut filter's DRV8210 EN and must only be pulsed (IR_LED_IrCut).

   It defaults to ENABLED at init, so a channel lights as soon as its mode is
   set; there is no separate "enable" step to forget. It is kept because a
   single command that kills both channels is worth having on a multi-amp
   illuminator, and because fault handling uses it to drop both channels. */
void IR_LED_Enable(bool enable);
bool IR_LED_IsEnabled(void);

/* ---------------------------------------------------------------------------
   Per-channel mode and strobe window
   --------------------------------------------------------------------------- */

/* IR_MODE_FRAME_STROBE needs the SN0_HS/SN0_VS EXTI lines running.
   IR_MODE_HOST_FLASH additionally needs IR_FLASH_SRC_HOST to be selected,
   otherwise the gate just follows the MCU's own (low) FLASH output and the
   channel stays dark; IR_LED_SetMode() returns false in that case. */
bool IR_LED_SetMode(IR_Channel_t ch, IR_LedMode_t mode);
IR_LedMode_t IR_LED_GetMode(IR_Channel_t ch);

/* Gate window in sensor lines, counted from the SN0_VS (frame start) edge.
   line_start = 0 asserts at frame start. Returns false if start >= end. */
bool IR_LED_SetWindow(IR_Channel_t ch, uint32_t line_start, uint32_t line_end);
void IR_LED_GetWindow(IR_Channel_t ch, uint32_t *line_start, uint32_t *line_end);

/* Lines counted since the last frame start, for checking that SN0_HS/SN0_VS
   are actually arriving and at the rate the sensor is configured for. */
uint32_t IR_LED_GetLineCount(void);
uint32_t IR_LED_GetFrameCount(void);

/* ---------------------------------------------------------------------------
   Current setpoint
   --------------------------------------------------------------------------- */

/* Sets the regulated LED current for one channel, in milliamps. Values above
   IR_LED_MAX_CURRENT_MA are clamped to it (the hardware would clamp them at
   CTRL2 anyway). Open loop -- there is no current readback on J5/J6. */
void IR_LED_SetCurrent_mA(IR_Channel_t ch, uint32_t milliamps);
uint32_t IR_LED_GetCurrent_mA(IR_Channel_t ch);

/* Raw DAC access, for bench work where you want to sweep CTRL1 directly and
   watch ISMON on TP2 rather than trust the transfer function above. */
void IR_LED_SetCurrentRaw(IR_Channel_t ch, uint16_t code_12bit);
uint16_t IR_LED_GetCurrentRaw(IR_Channel_t ch);

/* The CTRL1 voltage, in millivolts, that the current raw code produces. */
uint32_t IR_LED_GetCtrlVoltage_mV(IR_Channel_t ch);

/* Drives DAC_DIMx to a specific output voltage, in millivolts, rather than
   to a current. Clamped to the code for IR_LED_HARD_LIMIT_MA (~294mV at
   4A); not clamped to the soft current limit, because the point of this
   entry point is direct control of the pin - for metering PA4/PA5 or
   sweeping CTRL1 against ISMON on TP2. Callers that want a current should
   use IR_LED_SetCurrent_mA() instead.
   Returns the voltage actually applied after rounding and the hard cap. */
uint32_t IR_LED_SetCtrlVoltage_mV(IR_Channel_t ch, uint32_t millivolts);

/* Smallest voltage step the DAC can resolve, in microvolts. */
uint32_t IR_LED_GetCtrlStep_uV(void);

/* ---------------------------------------------------------------------------
   DAC-generated PWM on DAC_DIMx
   ---------------------------------------------------------------------------
   Chops the DAC output between 0V and the channel's present setpoint (as set
   by IR_LED_SetCurrent_mA / SetCtrlVoltage_mV / SetCurrentRaw), producing a
   square wave on PA4/PA5 rather than a DC level. TIM12 provides the
   timebase: its update event drives the output high and its capture/compare
   event drives it low, with CH1 owning IR_CH0 and CH2 owning IR_CH1. TIM12's
   outputs are never enabled, so PB14/PB15 (the LED gates) are untouched - it
   is used purely as an interrupt source.

   Both channels therefore SHARE one frequency: setting it on either channel
   changes it for both. Duty is per channel.

   IMPORTANT - what this does and does not achieve. CTRL1 feeds the LT3763's
   average-current control loop, compensated by R23 44.2k + C16 8.2nF on VC
   with a further 33nF (C13) across the sense inputs. That loop cannot slew
   anywhere near as fast as the DAC can step, so the LED current will round
   off the edges rather than switch cleanly, and above roughly a few hundred
   Hz the optical output approaches a steady average rather than a chopped
   waveform. For genuine PWM dimming the LT3763 provides a dedicated PWM pin
   (22), reached from PWM_DIM0/1 - see the gate description at the top of
   this file. This entry point exists because driving the dimming waveform
   from the DAC pins was asked for explicitly; it is not the chip's intended
   dimming path.

   freq_hz is clamped to IR_DAC_PWM_MIN_HZ..IR_DAC_PWM_MAX_HZ, duty_pct to
   0..100. Duty 0 parks the output at 0V, duty 100 parks it at the setpoint;
   both stop the timer for that channel. Returns false only for a bad
   channel. */
#define IR_DAC_PWM_MIN_HZ           1U
#define IR_DAC_PWM_MAX_HZ           10000U

/* Hardware gate PWM limits. The LT3763 switches at ~250kHz here (R13 = 182k,
   datasheet Table 4), and PWM dimming must stay well below the switching
   frequency for the converter to settle within each on-period. */
#define IR_GATE_PWM_MIN_HZ          10U
#define IR_GATE_PWM_MAX_HZ          20000U

bool IR_LED_SetDacPwm(IR_Channel_t ch, uint32_t freq_hz, uint32_t duty_pct);

/* Stops modulation and restores the plain DC setpoint on that channel. */
void IR_LED_StopDacPwm(IR_Channel_t ch);

/* True if this channel is currently being modulated; reports the live
   frequency and duty through the out-params when non-NULL. */
bool IR_LED_GetDacPwm(IR_Channel_t ch, uint32_t *freq_hz, uint32_t *duty_pct);

/* ---------------------------------------------------------------------------
   Hardware gate PWM - true PWM dimming through the LT3763's own PWM pin
   ---------------------------------------------------------------------------
   This is the chip's intended dimming path, and it is a different mechanism
   from IR_LED_SetDacPwm() above. PWM_DIM0/1 (PB14/PB15) reach LT3763 pin 22
   (PWM) through the Q3/Q2 inverter; when that pin goes low the controller
   terminates switching and drives PWM_OUT low, chopping the LED current with
   the dedicated FET (Q3, IRFH6200) in the LED return. The result is clean
   optical chopping with square edges at any frequency - unlike modulating
   CTRL1 from the DAC, which the average-current loop can only slew through.

   TIM12_CH1/CH2 drive PB14/PB15 directly in hardware, so there is no ISR and
   no jitter. The channel output is configured ACTIVE LOW so that the inverter
   on the proxy board cancels out: duty_pct is the fraction of time the LEDs
   are ON (the optical duty), not the fraction of time the MCU pin is high.
   Ask for 30% and you get 30% light.

   Calling this puts the channel into IR_MODE_GATE_PWM. Leaving that mode -
   via IR_LED_SetMode(), a fault, or IR_LED_Enable(false) - hands PB14/PB15
   back to plain GPIO and parks them at the OFF level.

   Both channels share TIM12, so they share a frequency; duty is per channel.
   freq_hz is clamped to IR_GATE_PWM_MIN_HZ..IR_GATE_PWM_MAX_HZ and duty_pct
   to 0..100. Duty 0 and 100 are steady states and fall back to plain GPIO
   (off, and continuous, respectively). */
bool IR_LED_SetGatePwm(IR_Channel_t ch, uint32_t freq_hz, uint32_t duty_pct);

/* Same hardware gate PWM, specified as time instead of frequency and duty:
   the LEDs are ON (IR_CTRLx high, PWM_DIMx low) for on_us, then OFF for
   off_us, repeating. Meant for strobe-like timings that do not come out as
   a whole number of Hz - 33000 on / 77000 off is a 110ms period (9.09Hz),
   reproduced exactly. The period is shared with the other channel (one
   timer), so setting it here changes the other channel's period too.
   Returns false for a bad channel or a period outside
   IR_GATE_PERIOD_MIN_US..IR_GATE_PERIOD_MAX_US. on_us = 0 is steady off and
   off_us = 0 is steady on, as with duty 0 / 100 above.
   The LT3763 needs each on-time to span at least two switching cycles
   (~8us at the board's ~250kHz) to reach the set current. */
#define IR_GATE_PERIOD_MIN_US       (1000000U / IR_GATE_PWM_MAX_HZ)
#define IR_GATE_PERIOD_MAX_US       10000000U
bool IR_LED_SetGatePulse(IR_Channel_t ch, uint32_t on_us, uint32_t off_us);

/* On/off times actually programmed after timer rounding, in microseconds.
   True while this channel is being chopped by TIM12. */
bool IR_LED_GetGatePulse(IR_Channel_t ch, uint32_t *on_us, uint32_t *off_us);

/* Puts the channel into IR_MODE_FRAME_PULSE: the LED goes ON for width_us at
   every frame start, then OFF until the next one. Frame start is the SN0_VS
   (PC4) edge; while none has arrived for IR_VSYNC_TIMEOUT_MS, TIM7 generates
   frame starts at IR_FRAME_PULSE_FPS instead (free-running, so it drifts
   slowly against the sensor's own frames). The pulse itself is TIM12 in
   one-pulse mode, so its width is exact to 1us with no ISR on the falling
   edge. The width is shared with the other channel (one timer); a channel of
   the other in IR_MODE_GATE_PWM is switched off, since both need TIM12.
   width_us is clamped to 1..IR_FRAME_PULSE_MAX_US; 0 turns the channel off. */
bool IR_LED_SetFramePulse(IR_Channel_t ch, uint32_t width_us);

/* HIGH-CURRENT PULSE (USHIO 20A test). One pulse of milliamps for width_us
   per frame tick (TIM7, IR_FRAME_PULSE_FPS; VSYNC if present), using the
   frame-pulse TIM12 one-pulse engine on PWM_DIMx PLUS the DAC: DAC_DIMx is
   raised to the pulse current immediately before the gate opens and dropped
   to 0 V in the TIM12 update interrupt the instant it closes. Outside the
   pulse the DAC is therefore always 0 V - essential, because Q2/Q3 pass a
   high FLASH to the LT3763 PWM pin whatever PWM_DIMx does, so a DAC left
   at 20A would light the LED for most of every frame.
   milliamps is clamped to IR_LED_PULSE_LIMIT_MA and width_us to
   IR_LED_HighPulseMaxWidth_us(milliamps). count = number of pulses to fire,
   0 = continuous. Ends the channel's FLASH gate / PWM / strobe / DAC PWM and
   sets its DC setpoint to 0. milliamps or width_us 0 stops it.
   Returns false for a bad channel. */
bool     IR_LED_SetHighPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t width_us, uint32_t count);
void     IR_LED_StopHighPulse(IR_Channel_t ch);
uint32_t IR_LED_HighPulseMaxWidth_us(uint32_t milliamps);
/* True while hpulse is set up on ch. Out-params (any may be NULL): current
   commanded (mA, after DAC rounding), width (us), pulses left (0 = endless
   or finished - see *fired), pulses fired since set up, pulses skipped by
   the spacing guard. */
bool     IR_LED_GetHighPulse(IR_Channel_t ch, uint32_t *ma, uint32_t *width_us,
                             uint32_t *left, uint32_t *fired, uint32_t *skipped);

/* HSYNC PULSE (USHIO 20A test). Per frame, for lines start_line ..
   start_line + lines - 1 counted from VSYNC (first HSYNC after VSYNC =
   line 1), DAC_DIMx AND PWM_DIMx follow SN0_HS (PB1): on each rising edge
   the DAC goes to milliamps and then the gate opens; on each falling edge
   the gate closes and then the DAC goes to 0 V. TIM12 (one-pulse, no pin
   output) starts on the first rising edge and cuts both at lines line
   periods + margin (line period = VSYNC period / lines per frame), so a
   missing HSYNC edge cannot leave the current up. That DAC-high time is held to
   IR_LED_HighPulseMaxWidth_us(milliamps): lines is clamped to fit, using the
   measured line period. count = frames to fire, 0 = continuous. One channel
   at a time; uses TIM12, so it ends hpulse / frame pulse / gate PWM and is
   ended by them. Returns false for a bad channel, no HSYNC, or a budget too
   short for one line. */
typedef struct {
    uint32_t ma;            /* pulse current after DAC rounding */
    uint32_t start_line;
    uint32_t lines;         /* after the derating clamp */
    uint32_t window_us;     /* TIM12 cut-off = max DAC-high time per frame */
    uint32_t left;          /* frames still to fire, 0 = continuous */
    uint32_t fired;         /* windows started */
    uint32_t timer_cuts;    /* windows ended by TIM12 rather than HSYNC */
    uint32_t hs_high_ns;    /* last measured HSYNC HIGH time (0 = not yet) */
    bool     vs_mode;       /* set up by IR_LED_SetVsyncPulse() */
    bool     solid;         /* set up by IR_LED_SetVsyncLinePulse() */
    uint32_t vs_low_ns;     /* vsync mode: last measured VSYNC LOW time */
    uint32_t skipped;       /* starts refused by the IR_HP_MIN_SPACING_US guard */
    bool     flash;         /* set up by IR_LED_SetHsyncFlashPulse() */
    bool     inv;           /* ... with inv: PC3 = NOT HSYNC */
} IR_HsyncPulseInfo_t;

bool IR_LED_SetHsyncPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                          uint32_t lines, uint32_t count);
void IR_LED_StopHsyncPulse(IR_Channel_t ch);
/* FLASH-gated HSYNC PULSE: as IR_LED_SetHsyncPulse() but the gate is the
   FLASH net - PC3 is switched to a push-pull output for the duration and
   driven HIGH while HSYNC is HIGH (Q2/Q3 light the LED), LOW otherwise;
   PWM_DIMx stays HIGH. ONLY with the sensor's FLASH output disabled (user
   approved 2026-10-14). Stop puts PC3 back to the boot input. Refused
   (false) while the FLASH ADC sync owns PC3. inv: PC3 = NOT HSYNC instead -
   HIGH (DAC up first) through the ~1.7 us LOW after each window line, LOW
   (then DAC 0) from the next rise; same window, clamp and cut-off. */
bool IR_LED_SetHsyncFlashPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                               uint32_t lines, uint32_t count, bool inv);
/* HSYNC LOW pulse on PWM_DIMx: as IR_LED_SetHsyncPulse() but the LED is on
   (DAC up, then PWM_DIMx LOW) through the ~1.7 us HSYNC LOW after each
   window line, off (PWM_DIMx HIGH, then DAC 0) from the next rise. */
bool IR_LED_SetHsyncLowPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                             uint32_t lines, uint32_t count);
/* VSYNC PULSE: same engine, LED on at milliamps while SN0_VS (PC4) is LOW -
   DAC up and PWM_DIMx low on the falling edge, both off on the rising edge.
   TIM12 cuts it at IR_LED_HighPulseMaxWidth_us(milliamps) if VSYNC stays
   low longer. Starts closer than IR_HP_MIN_SPACING_US are skipped. count =
   pulses, 0 = continuous. Stopped / reported with the hsync functions
   (info.vs_mode set). Returns false for a bad channel or no VSYNC. */
bool IR_LED_SetVsyncPulse(IR_Channel_t ch, uint32_t milliamps, uint32_t count);
/* VSYNC LINE PULSE: as IR_LED_SetHsyncPulse() (same line count from VSYNC,
   pre-line DAC step, derating clamp on lines, TIM12 cut-off) but the LED is
   held solid ON from line start_line to start_line + lines - 1 instead of
   following the HSYNC level - one clean pulse per frame. */
bool IR_LED_SetVsyncLinePulse(IR_Channel_t ch, uint32_t milliamps, uint32_t start_line,
                              uint32_t lines, uint32_t count);
bool IR_LED_GetHsyncPulse(IR_Channel_t ch, IR_HsyncPulseInfo_t *info);

/* Width in us; true while the channel is in IR_MODE_FRAME_PULSE. */
bool IR_LED_GetFramePulse(IR_Channel_t ch, uint32_t *width_us);

/* Frame-pulse diagnostics: pulses fired since boot, and whether the last one
   came from a real VSYNC edge (true) or the TIM7 fallback (false). */
uint32_t IR_LED_GetFramePulseCount(void);

/* FLASH sync. Enter it with IR_LED_SetMode(ch, IR_MODE_FLASH_SYNC): PC3 is
   released to analog and ADC1 starts; leaving the mode on every channel (or
   IR_LED_Enable(false)) stops the ADC and drives PC3 low again. */
void IR_LED_SetFlashPolarity(bool active_high);
bool IR_LED_GetFlashActiveHigh(void);
/* low_mv must be below high_mv. Returns false otherwise. */
bool IR_LED_SetFlashThresholds_mV(uint32_t high_mv, uint32_t low_mv);
void IR_LED_GetFlashThresholds_mV(uint32_t *high_mv, uint32_t *low_mv);
/* Live PC3 level, whether FLASH is active now, and edges since boot.
   Returns true while the ADC is running. */
bool IR_LED_GetFlashSyncStatus(uint32_t *level_mv, bool *active, uint32_t *edges);
bool     IR_LED_FramePulseIsSynced(void);

/* Stops the hardware PWM and returns the channel to IR_MODE_OFF. */
void IR_LED_StopGatePwm(IR_Channel_t ch);

/* True while this channel is being chopped by TIM12. */
bool IR_LED_GetGatePwm(IR_Channel_t ch, uint32_t *freq_hz, uint32_t *duty_pct);

/* Soft cap applied to every IR_LED_SetCurrent_mA() and ramp request, on top
   of the hardware ceiling. Values above IR_LED_MAX_CURRENT_MA are pointless
   and get clamped to it. Lowering the limit below a channel's present
   setpoint immediately re-applies the setpoint at the new cap, so turning the
   limit down always takes effect at once rather than at the next command. */
void     IR_LED_SetCurrentLimit_mA(uint32_t milliamps);
uint32_t IR_LED_GetCurrentLimit_mA(void);

/* Walks the setpoint from where it is now to target_ma in step_ma increments,
   pausing dwell_ms between steps and running the fault check at every step.
   Blocking: it is meant for bench bring-up from the console, not for a
   control loop. Passing 0 for step_ma or dwell_ms substitutes the
   IR_LED_RAMP_* defaults above.
   Returns false, leaving the setpoint wherever the ramp got to, if the
   channel faults or the master enable drops partway - so a ramp that trips
   the driver stops climbing instead of pushing on to the target. */
bool IR_LED_RampCurrent_mA(IR_Channel_t ch, uint32_t target_ma,
                           uint32_t step_ma, uint32_t dwell_ms);

/* ---------------------------------------------------------------------------
   FLASH net ownership
   --------------------------------------------------------------------------- */

/* PC3 is input-only: returns true for IR_FLASH_SRC_HOST (no change - it is
   always an input) and false for IR_FLASH_SRC_MCU, which is never allowed. */
bool IR_LED_SetFlashSource(IR_FlashSource_t src);

/* FLASH input monitor. PC3 is a digital input with EXTI on both edges (read
   only, never driven); edges are timed with the DWT cycle counter. */
typedef struct {
    bool     level_high;   /* PC3 right now */
    bool     toggling;     /* an edge within IR_FLASH_IDLE_MS */
    uint32_t falls;        /* falling edges since boot */
    uint32_t period_us;    /* last fall-to-fall */
    uint32_t low_us;       /* last fall-to-rise (low pulse width) */
    uint32_t idle_ms;      /* since the last edge */
} IR_FlashInput_t;
void IR_LED_GetFlashInput(IR_FlashInput_t *in);

/* Sensor sync monitor: rising edges of SN0_VS (PC4) and SN0_HS (PB1). */
typedef struct {
    bool     vs_active;     /* a VS edge within IR_FLASH_IDLE_MS */
    bool     hs_active;     /* HS count moved since the previous call */
    uint32_t vs_count;
    uint32_t vs_period_us;  /* last VS-to-VS */
    uint32_t hs_count;
    uint32_t hs_period_ns;  /* last HS-to-HS */
    uint32_t hs_per_frame;  /* HS edges between the last two VS edges */
} IR_SyncInput_t;
void IR_LED_GetSyncInput(IR_SyncInput_t *in);
void IR_LED_GetSyncDebug(uint32_t d[6]);   /* raw monitor values, for status */
/* fhsync only: PC3 write delay after SN0_HS interrupt entry, ns (rise
   min/avg/max, fall min/avg/max), last PC3 HIGH ns, PC3 pulses in the last
   window and min/max over windows, total rises, rises found pending at the
   end of a fall (late). False when not running. */
bool IR_LED_GetFlashSyncStats(uint32_t d[12]);
/* EXTI1 (SN0_HS) handler, first for each edge: rise or fall, late = a rise
   picked up after handling a fall in the same interrupt. */
void IR_LED_HsIrqFast(bool rise, bool late);
bool IR_LED_MeasureHsync(uint32_t d[7]);   /* see ir_led_driver.c; "ir hsmeas" */

/* FLASH gate, per channel: the PC3 edge interrupts drive PWM_DIMx LOW (LED
   on) while FLASH is low (the exposure) and HIGH again when it rises. With
   the gate off, PWM_DIMx stays HIGH and the LED follows FLASH only.
   IR_FLASH_GATE_AT_BOOT turns it on for CH0 at boot; CH1 starts off.
   Only ONE channel can be gated at a time (returns false otherwise): FLASH
   is pulled high only through a channel whose PWM_DIMx is HIGH, so with
   both gated FLASH never rises again and the LEDs latch on. */
#define IR_FLASH_GATE_AT_BOOT       (IR_PC3_OUTPUT ? 0 : 1)   /* no FLASH input on IMX678 */
bool IR_LED_SetFlashGate(IR_Channel_t ch, bool on);

/* Static PWM_DIMx level, per channel: HIGH = LED follows FLASH only, LOW =
   LED forced on. Either one cancels the FLASH gate on that channel. LOW is
   refused (false) while the other channel is gated, and gating is refused
   while the other channel is held LOW - same latch-up reason as above. */
typedef enum { IR_PWM_HIGH = 0, IR_PWM_LOW, IR_PWM_GATED, IR_PWM_DUTY } IR_PwmState_t;
bool IR_LED_SetPwmLevel(IR_Channel_t ch, bool high);

/* Free-running PWM on PWM_DIMx (TIM12): duty_pct = share of each period the
   LED is forced on (PWM_DIMx LOW). 0 % = fixed HIGH, 100 % = fixed LOW.
   freq_hz is clamped to IR_GATE_PWM_MIN_HZ..IR_GATE_PWM_MAX_HZ. TIM12 is
   shared, so both channels run at the most recently set frequency, each
   with its own duty. Refused while the other channel is FLASH-gated. */
bool IR_LED_SetPwmDuty(IR_Channel_t ch, uint32_t freq_hz, uint32_t duty_pct);

/* Gradual current change on DAC_DIMx: moves linearly from the present
   setpoint to target_ma over time_ms, one DAC update per millisecond from
   IR_LED_Task() - non-blocking, so the console and FLASH gate keep running.
   A new "ma"/"mv" or ramp on the channel replaces it. */
bool IR_LED_StartRamp_mA(IR_Channel_t ch, uint32_t target_ma, uint32_t time_ms);

/* Strobe: LED on while FLASH is low (exposure), off while it is high.
   FLASH gate on PWM_DIMx PLUS DAC_DIMx switched from the PC3 edge interrupt:
   setpoint while FLASH is low, 0 V while high. The board's Q2/Q3 pass FLASH
   high to IR_CTRLx regardless of PWM_DIMx, so outside the exposure the LT3763
   stays enabled at CTRL1 = 0 V: much dimmer, but its offset can still pass
   some current (worst case ~1.2 A). Same one-gate rule as the FLASH gate;
   gate/pwm/dacpwm on the channel end it. */
bool IR_LED_SetStrobe(IR_Channel_t ch, bool on);

/* DAC TEST - bench characterisation of DAC_DIMx with J5/J6 UNPLUGGED.
   Drives DAC_DIMx to 0..IR_DACTEST_MAX_MV, BYPASSING IR_LED_HARD_LIMIT_MA.
   With R16 reworked to 0R there is NO CTRL2 ceiling any more: on a
   connected driver 1500 mV is ~20 A DC, which destroys the USHIO LED, and
   anything above ~294 mV exceeds its 4 A DC rating. Stops ramp, strobe and
   DAC chop on the channel; any ma/mv/ramp (SetCurrentRaw) ends it. */
#define IR_DACTEST_MAX_MV           1500U
uint32_t IR_LED_SetDacTest_mV(IR_Channel_t ch, uint32_t millivolts);
void     IR_LED_ClearDacTest(IR_Channel_t ch);
bool     IR_LED_GetDacTest(IR_Channel_t ch, uint32_t *millivolts);
bool IR_LED_GetStrobe(IR_Channel_t ch);
void IR_LED_StopRamp(IR_Channel_t ch);
bool IR_LED_GetRamp(IR_Channel_t ch, uint32_t *to_ma, uint32_t *left_ms);
IR_PwmState_t IR_LED_GetPwmState(IR_Channel_t ch);
bool IR_LED_GetFlashGate(IR_Channel_t ch, uint32_t *count);

/* Inverted FLASH gate: PWM_DIMx HIGH while FLASH is low, LOW while FLASH is
   high. Through Q2/Q3 that makes IR_CTRLx = FLASH (low) during the exposure,
   so the LED is OFF during the exposure and ON outside it - the opposite of
   the normal gate. For testing the board's gate polarity. */
bool IR_LED_SetFlashGateInverted(IR_Channel_t ch);

/* IR-cut filter on PA11 (IR_ENB_MCU -> DRV8210 IN2/EN) and PA12
   (IR_FBC_MCU -> IN1/PH). A call pulses EN for the set time with PH chosen
   for day (filter in) or night (filter out); IR_LED_Task() ends the pulse.
   Which PH pulls the filter out depends on the coil wiring: IR_CUT_NIGHT_PH
   is the KL730 ircut.sh guess (0), swappable at runtime. Returns false if a
   pulse is already running. */
#define IR_CUT_PULSE_MS             150U
#define IR_CUT_PULSE_MIN_MS         10U
#define IR_CUT_PULSE_MAX_MS         1000U
#define IR_CUT_NIGHT_PH             0U
bool     IR_LED_IrCut(bool night);
uint32_t IR_LED_IrCutSetPulse_ms(uint32_t ms);
void     IR_LED_IrCutSwap(void);
void     IR_LED_IrCutStatus(int *last, uint32_t *pulse_ms, uint32_t *night_ph, bool *busy);

bool IR_LED_GetFlashGateInverted(IR_Channel_t ch);
IR_FlashSource_t IR_LED_GetFlashSource(void);

/* ---------------------------------------------------------------------------
   Faults
   --------------------------------------------------------------------------- */

/* Latched state, set by IR_LED_Task() once a low FAULT line has survived the
   blanking window. Sticky until IR_LED_ClearFault(). */
bool IR_LED_HasFault(IR_Channel_t ch);

/* Live level of the IR_FAULTx pin, ignoring blanking and latching. Expected
   to read asserted whenever the channel is dark or the rail is down -- see
   note 3 in the file header before reading anything into it. */
bool IR_LED_IsFaultLineAsserted(IR_Channel_t ch);

/* Number of FAULT edges seen on this channel since init. A value that climbs
   while the channel is steadily illuminated points at an intermittent LED
   open/short rather than a hard failure. */
uint32_t IR_LED_GetFaultEdgeCount(IR_Channel_t ch);

/* Clears the latch and re-arms the channel. Does not re-assert the master
   enable if a fault dropped it -- that stays an explicit IR_LED_Enable(true),
   so "a fault happened" and "I have re-armed the system" remain two separate
   deliberate steps. */
void IR_LED_ClearFault(IR_Channel_t ch);

/* ---------------------------------------------------------------------------
   EXTI entry points. Called from HAL_GPIO_EXTI_Callback() in this module;
   not intended for application code.
   --------------------------------------------------------------------------- */
void IR_LED_OnHSync(void);
void IR_LED_OnVSync(void);
void IR_LED_OnFaultEdge(IR_Channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __IR_LED_DRIVER_H__ */
