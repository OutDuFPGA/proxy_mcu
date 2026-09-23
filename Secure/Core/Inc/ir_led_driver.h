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
  * With FLASH held low by the MCU this is a plain inverter and the MCU owns
  * the strobe. With FLASH released (PC3 as input) and PWM_DIMx = 1, the host
  * SoC's FLASH signal passes straight through to the LED boards instead.
  * IR_FlashSource_t selects between those two; see IR_LED_SetFlashSource().
  *
  * The practical consequence, and the reason this file exists in its current
  * form: PWM_DIMx LOW means LIGHT ON. Every gate write below goes through
  * gate_set()/IR_GATE_ON/IR_GATE_OFF so the inversion is stated in exactly
  * one place.
  *
  * Note the reset hazard this creates. Out of reset, before MX_GPIO_Init()
  * runs, PB14/PB15 are inputs, so Q is off and IR_CTRLx is pulled to 3V3 --
  * the LEDs are commanded ON. MX_GPIO_Init() therefore writes PWM_DIM0/1
  * HIGH before switching them to outputs, and IR_ENB_MCU (the 18V rail
  * request to the PDU) stays low until the application explicitly enables it.
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
  * BUT CTRL2 (pin 13) is tied to a fixed divider off the 2V VREF pin:
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
  * IR_ENB_MCU (PA11) goes to the host/PDU connector on sheet 3, not to J5/J6,
  * and is treated here as the request for that rail.
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
#define IR_CTRL2_R_TOP_OHMS         45300U
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

/* Bench safety cap on the commanded current, in milliamps, applied on top of
   the hardware ceiling. Defaults to the 4A the illuminator is being brought
   up at, so a slipped digit on the console cannot command the board's full
   ~4.9A into the LED string. Adjustable at runtime with
   IR_LED_SetCurrentLimit_mA() / "ir limit <mA>". */
#define IR_LED_DEFAULT_LIMIT_MA     4000U

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
    IR_MODE_GATE_PWM          /* TIM12 chops the gate: true PWM dimming          */
} IR_LedMode_t;

typedef enum {
    /* PC3 driven low by this MCU. IR_CTRLx is then a clean inverter of
       PWM_DIMx and the MCU owns strobing. This is the default. The host SoC
       must leave its end of FLASH high-impedance. */
    IR_FLASH_SRC_MCU = 0,
    /* PC3 released to an input so the host SoC drives FLASH. Only meaningful
       together with IR_MODE_HOST_FLASH. */
    IR_FLASH_SRC_HOST
} IR_FlashSource_t;

/* ---------------------------------------------------------------------------
   Lifecycle
   --------------------------------------------------------------------------- */

/* Call once from main(), after MX_GPIO_Init(). Brings up DAC1, forces both
   gates off, takes ownership of FLASH (drives PC3 low), sets both current
   setpoints to 0 and leaves IR_ENB_MCU deasserted. Nothing illuminates until
   IR_LED_Enable(true) and a mode other than IR_MODE_OFF. */
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

   NOTE: this is SOFTWARE ONLY. IR_ENB_MCU (PA11) is left unconnected on this
   build - it reaches the sheet 3 board-to-board header and nothing consumes
   it - so it controls no rail. The pin is still driven in step with this
   flag in case it is ever wired, but nothing depends on it.

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
   to a current. Clamped to the DAC's own full scale (IR_DAC_VREF_MV); NOT
   clamped to IR_CTRL_ABS_MAX_MV or to the current limit, because the point
   of this entry point is direct control of the pin - for metering PA4/PA5,
   sweeping CTRL1 against ISMON on TP2, or checking the CTRL2 clamp. Callers
   that want a current should use IR_LED_SetCurrent_mA() instead.
   Returns the voltage actually applied after rounding to the nearest code. */
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

/* Reconfigures PC3. Switching to IR_FLASH_SRC_HOST while any channel is in a
   mode other than IR_MODE_HOST_FLASH is rejected (returns false), because
   releasing FLASH would leave those channels' gates floating.
   Note the asymmetry: with the host owning FLASH the MCU can no longer force
   a channel dark through PWM_DIMx alone, so IR_LED_Enable(false) and any
   confirmed fault both reclaim FLASH as an output low. */
bool IR_LED_SetFlashSource(IR_FlashSource_t src);
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
