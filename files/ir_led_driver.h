/**
  ******************************************************************************
  * @file    ir_led_driver.h
  * @brief   HSYNC/VSYNC-synchronized IR LED strobe control.
  *
  * Drives the two USHIO IR LED channels (LT3763 driver boards on J5/J6) in
  * sync with the IMX678 image sensor's line/frame sync pins:
  *   SN0_HS (PB1)  = sensor XHS, one pulse per line
  *   SN0_VS (PC4)  = sensor XVS, one pulse per frame
  *   PWM_DIM0 (PB14) = strobe gate into channel 0 driver board (J5, "IR_CTRL0")
  *   PWM_DIM1 (PB15) = strobe gate into channel 1 driver board (J6, "IR_CTRL1")
  *   DAC_DIM0 (PA4)  = analog current-level control, channel 0
  *   DAC_DIM1 (PA5)  = analog current-level control, channel 1
  *   IR_FAULT0 (PB9) = fault return, channel 0
  *   IR_FAULT1 (PE0) = fault return, channel 1
  *   IR_ENB_MCU (PA11) = master enable output to both driver boards
  *
  * See the accompanying design document for the full rationale: strobing is
  * gated once per frame from a line counter built out of SN0_HS/SN0_VS, not
  * toggled on every raw HSYNC edge (see the "design decision" section).
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

typedef enum {
    IR_CH0 = 0,
    IR_CH1 = 1,
    IR_CH_COUNT
} IR_Channel_t;

typedef enum {
    IR_MODE_CONTINUOUS = 0,   /* Mode A: gate held high, no sensor sync needed   */
    IR_MODE_FRAME_STROBE = 1, /* Mode B: gated window per frame (recommended)    */
    IR_MODE_OFF = 2           /* Gate held low, e.g. during fault                */
} IR_LedMode_t;

/* Call once from main(), after MX_GPIO_Init(). Leaves both channels off and
   the master enable (IR_ENB_MCU) low. */
void IR_LED_Init(void);

/* Master enable, common to both channels (drives IR_ENB_MCU). While disabled,
   both channel gates are forced off regardless of their configured mode -
   this is the "enable/disable" control, separate from per-channel mode
   selection below. Each channel's configured mode is preserved and resumes
   automatically when re-enabled (no need to call IR_LED_SetMode() again). */
void IR_LED_Enable(bool enable);
bool IR_LED_IsEnabled(void);

/* Select per-channel operating mode. IR_MODE_FRAME_STROBE requires the
   sensor's HSYNC/VSYNC EXTI interrupts to be running (see stm32h5xx_it.c
   additions). Takes effect immediately if the master enable is on. */
void IR_LED_SetMode(IR_Channel_t ch, IR_LedMode_t mode);
IR_LedMode_t IR_LED_GetMode(IR_Channel_t ch);

/* Configure the per-frame gate window in sensor lines, counted from the VSYNC
   (frame-start) edge. line_start = 0 means "assert immediately at frame
   start". line_end = total active lines per frame means "hold through the
   end of the active picture area" (the simple, robust full-frame choice
   described in the design doc). Recompute these whenever exposure/resolution/
   frame rate changes on the sensor. */
void IR_LED_SetWindow(IR_Channel_t ch, uint32_t line_start, uint32_t line_end);
void IR_LED_GetWindow(IR_Channel_t ch, uint32_t *line_start, uint32_t *line_end);

/* Analog current-level setpoint, 0-4095 (12-bit). Requires DAC1 to be enabled
   via CubeMX (MX_DAC1_Init()) — see the .c file for the TODO on this. The
   setpoint is recorded and returned by IR_LED_GetCurrentLevel() even before
   DAC1 is wired up, so the CLI/status path works already. */
void IR_LED_SetCurrentLevel(IR_Channel_t ch, uint16_t level_12bit);
uint16_t IR_LED_GetCurrentLevel(IR_Channel_t ch);

/* True after a fault has latched on this channel (IR_FAULTx asserted). The
   channel's gate is forced off immediately in the ISR; this flag requires an
   explicit call to IR_LED_ClearFault() to re-arm — never auto-cleared. */
bool IR_LED_HasFault(IR_Channel_t ch);
void IR_LED_ClearFault(IR_Channel_t ch);

/* Internal: called from HAL_GPIO_EXTI_Callback() in this module. Not intended
   to be called directly by application code. */
void IR_LED_OnHSync(void);
void IR_LED_OnVSync(void);
void IR_LED_OnFault(IR_Channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __IR_LED_DRIVER_H__ */
