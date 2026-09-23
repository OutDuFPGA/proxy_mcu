/**
  ******************************************************************************
  * @file    dac.h
  * @brief   DAC1 (PA4 = DAC_DIM0, PA5 = DAC_DIM1) analog dimming outputs.
  *
  * These two pins drive the CTRL1 pin of the LT3763 on each USHIO IR LED
  * board, via J5/J6 pin 1 -> board J3 pin 1 -> R19 (0R) -> LT3763 pin 14.
  * CTRL1 is what sets the regulated LED current; see ir_led_driver.h for the
  * transfer function and the board's fixed CTRL2 ceiling.
  *
  * Why this is register-level and not HAL:
  * this project's Drivers/ tree does not ship stm32h5xx_hal_dac.c/.h, and
  * HAL_DAC_MODULE_ENABLED is commented out in stm32h5xx_hal_conf.h. Rather
  * than require a CubeMX regeneration (which would also rewrite gpio.c and
  * main.c and lose the hand edits there), DAC1 is programmed directly. The
  * register set used here is tiny and entirely static:
  *   MCR   - channel mode: normal, buffer enabled, output to external pin
  *   CR    - channel enable, no trigger (TENx = 0 => DHR writes apply at once)
  *   DHR12Rx - 12-bit right-aligned data
  *
  * To move to HAL later: enable DAC1 in CubeMX (PA4 = DAC1_OUT1, PA5 =
  * DAC1_OUT2), which adds the HAL files and generates MX_DAC1_Init(); then
  * delete this module and repoint ir_led_driver.c's three DAC1_* calls at
  * HAL_DAC_Start()/HAL_DAC_SetValue().
  ******************************************************************************
  */
#ifndef __DAC_H__
#define __DAC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Channel selectors. These are the DAC's own channel numbers, not IR channel
   numbers - ir_led_driver.c maps IR_CH0 -> 1 (PA4) and IR_CH1 -> 2 (PA5). */
#define IR_DAC1_CH1   1U   /* PA4, DAC_DIM0 */
#define IR_DAC1_CH2   2U   /* PA5, DAC_DIM1 */

/* Full-scale DAC code. The output is code/DAC1_CODE_MAX * VDDA. */
#define DAC1_CODE_MAX    4095U

/* Enables the DAC1 clock and configures both channels for normal buffered
   output on their external pins, then enables them with a code of 0.
   Both outputs sit at ~0V (i.e. LED current setpoint = 0) when this returns.

   MX_GPIO_Init() must have run first: it already puts PA4/PA5 in analog mode,
   which is what the DAC needs. Safe to call more than once. */
void MX_DAC1_Init(void);

/* Writes a 12-bit code to one channel. Values above DAC1_CODE_MAX are
   clamped. Takes effect immediately (no trigger is configured). */
void DAC1_SetValue(uint32_t dac_channel, uint16_t code_12bit);

/* Last code written to this channel by DAC1_SetValue(). Reads back the
   holding register, not the analog output. */
uint16_t DAC1_GetValue(uint32_t dac_channel);

/* True once MX_DAC1_Init() has completed. ir_led_driver.c uses this so its
   status output can distinguish "current setpoint is 0" from "the DAC was
   never brought up, so CTRL1 is floating". */
bool DAC1_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __DAC_H__ */
