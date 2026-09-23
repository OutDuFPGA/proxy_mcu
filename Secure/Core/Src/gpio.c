/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
/* NOTE: the pin-mode changes below (SN0_HS_Pin/SN0_VS_Pin -> IT_RISING,
   PWM_DIM0_Pin/PWM_DIM1_Pin -> OUTPUT_PP, IR_FAULT0_Pin/IR_FAULT1_Pin ->
   IT_FALLING) are hand-edited directly in MX_GPIO_Init() below, together
   with the matching HAL_NVIC_EnableIRQ() calls for EXTI0/1/4/9. If this
   project's .ioc file is later regenerated from CubeMX, these edits will be
   overwritten unless the same pin modes are also set in the CubeMX Pinout
   view (PB1 = GPIO_EXTI1, PC4 = GPIO_EXTI4, PB9 = GPIO_EXTI9, PE0 =
   GPIO_EXTI0, PB14/PB15 = GPIO_Output) and the four EXTI lines are enabled
   in the NVIC view. */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN(OSC32_IN)   ------> RCC_OSC32_IN
     PH0-OSC_IN(PH0)   ------> RCC_OSC_IN
     PC0   ------> ADCx_INP10
     PA4   ------> COMP_DAC11_group
     PA5   ------> COMP_DAC12_group
     PA6   ------> USART11_TX
     PA7   ------> USART11_RX
     PB0   ------> TIM1_CH2N
     PB10   ------> I2C2_SCL
     PB11   ------> I2C2_SDA
     PB14   ------> S_TIM12_CH1
     PB15   ------> S_TIM12_CH2
     PA15(JTDI)   ------> GPXTI15
     PC11   ------> GPXTI11
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|HAPTIC_ENB_Pin|IRIS_DIR_Pin|FOCUS_DIR_Pin
                          |LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FOCUS_EN_Pin|IR_ENB_MCU_Pin|IR_FBC_MCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ZOOM_EN_Pin|ZOOM_DIR_Pin|LED_GREEN_Pin|LED_BLUE_Pin
                          |PROXY_HB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level : PWM_DIM0_Pin PWM_DIM1_Pin
     HIGH, not LOW. These gates are inverted by the open-drain stage on
     schematic sheet 5 (Q3/Q2 MGSF1N02LT1G, 10k pull-up to 3V3 on the drain),
     so PWM_DIMx LOW means IR_CTRLx HIGH means LEDs ON. Out of reset PB14/PB15
     are inputs, the FETs are off and IR_CTRLx is already pulled to 3V3 - i.e.
     the LED boards are being commanded on. Writing ODR here, before the pins
     become outputs just below, is what actually ends that window: the pin
     drives high the instant its mode changes, with no low-going glitch.
     Do not "tidy" this back into the RESET group above. */
  HAL_GPIO_WritePin(GPIOB, PWM_DIM0_Pin|PWM_DIM1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level : FLASH_Pin
     FLASH is the source of both inverters above, so it has to be a defined
     low for PWM_DIMx to gate anything. Parked low here and reconfigured as an
     output a few lines down; IR_LED_SetFlashSource() can hand it back to the
     host SoC later. */
  HAL_GPIO_WritePin(FLASH_GPIO_Port, FLASH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 HAPTIC_ENB_Pin IRIS_DIR_Pin FOCUS_DIR_Pin
                           LED_RED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|HAPTIC_ENB_Pin|IRIS_DIR_Pin|FOCUS_DIR_Pin
                          |LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ALCO_DIG_Pin PIR_INT_3V3_Pin */
  GPIO_InitStruct.Pin = ALCO_DIG_Pin|PIR_INT_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_Pin
     Output, not input. See the ODR write above: this net sets the low level
     of both IR_CTRL inverters, and left floating the IR gating does nothing
     repeatable. This assumes the host SoC leaves its end of FLASH
     high-impedance - it is a shared net on the sheet 3 board-to-board
     connector. IR_LED_SetFlashSource(IR_FLASH_SRC_HOST) returns it to an
     input when the host is meant to drive the strobe instead. */
  GPIO_InitStruct.Pin = FLASH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FLASH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SN0_VS_Pin (sensor XVS, frame start -> IR_LED_OnVSync) */
  GPIO_InitStruct.Pin = SN0_VS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SN0_VS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_DIM0_Pin DAC_DIM1_Pin */
  GPIO_InitStruct.Pin = DAC_DIM0_Pin|DAC_DIM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UART1_TXD_3V3_Pin UART1_RXD_3V3_Pin */
  GPIO_InitStruct.Pin = UART1_TXD_3V3_Pin|UART1_RXD_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART11;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SN0_HS_Pin (sensor XHS, one pulse per line -> IR_LED_OnHSync) */
  GPIO_InitStruct.Pin = SN0_HS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SN0_HS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_FAULT0_Pin (LT3763 board J5 fault, active low) */
  GPIO_InitStruct.Pin = IR_FAULT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_FAULT0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENS_TEMP_Pin */
  GPIO_InitStruct.Pin = SENS_TEMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENS_TEMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C0_SCL_3V3_Pin I2C0_SDA_3V3_Pin */
  GPIO_InitStruct.Pin = I2C0_SCL_3V3_Pin|I2C0_SDA_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PWM_DIM0_Pin PWM_DIM1_Pin
     IR strobe gates into the J5/J6 driver boards, ACTIVE LOW (see the ODR
     write near the top of this function). Plain outputs driven by
     ir_led_driver.c, not TIM12 PWM: the .ioc assigns PB14/PB15 to
     S_TIM12_CH1/CH2, but no MX_TIM12_Init() exists, so leaving them in AF
     mode left the gates floating and both IR channels dead. Switch back to
     GPIO_AF2_TIM12 only together with a real MX_TIM12_Init(), remembering
     that the inverter makes the duty cycle complementary - a 20%% optical
     duty needs an 80%% timer duty. */
  GPIO_InitStruct.Pin = PWM_DIM0_Pin|PWM_DIM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IRIS_EN_Pin */
  GPIO_InitStruct.Pin = IRIS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRIS_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FOCUS_EN_Pin IR_ENB_MCU_Pin IR_FBC_MCU_Pin */
  GPIO_InitStruct.Pin = FOCUS_EN_Pin|IR_ENB_MCU_Pin|IR_FBC_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ZOOM_EN_Pin ZOOM_DIR_Pin LED_GREEN_Pin LED_BLUE_Pin
                           PROXY_HB_Pin */
  GPIO_InitStruct.Pin = ZOOM_EN_Pin|ZOOM_DIR_Pin|LED_GREEN_Pin|LED_BLUE_Pin
                          |PROXY_HB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_FAULT1_Pin (LT3763 board J6 fault, active low) */
  GPIO_InitStruct.Pin = IR_FAULT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_FAULT1_GPIO_Port, &GPIO_InitStruct);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOH, GPIO_PIN_1, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOC, ALCO_ANA_Pin|LED0_TEMP_ADCIN_Pin, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(HAPTIC_PWM_GPIO_Port, HAPTIC_PWM_Pin, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(LED1_TEMP_ADCIN_GPIO_Port, LED1_TEMP_ADCIN_Pin, GPIO_PIN_NSEC);

  /* EXTI interrupt init - all four lines feed HAL_GPIO_EXTI_Callback(), which
     ir_led_driver.c implements and dispatches by pin.

     Priorities: HAL_Init() selects NVIC_PRIORITYGROUP_4, so all four bits are
     pre-emption priority and the sub-priority argument is ignored - the
     ordering below therefore has to live entirely in the first number.
     The IR fault lines share UART4's top priority (they cut the LED drive),
     then VSYNC, then HSYNC, so a burst of HSYNC edges can never delay a fault
     or the console RX.

     Note SN0_HS fires once per sensor line - at 2160 lines/frame and 30 fps
     that is ~65 kHz, i.e. one interrupt per ~980 CPU cycles at 64 MHz.
     IR_LED_OnHSync() is deliberately short for that reason; if the line rate
     goes higher, move the line counter onto a timer in external-clock mode
     instead of EXTI. */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);   /* IR_FAULT1, PE0 */
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI9_IRQn, 0, 0);   /* IR_FAULT0, PB9 */
  HAL_NVIC_EnableIRQ(EXTI9_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);   /* SN0_VS, PC4 - frame start */
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);   /* SN0_HS, PB1 - per line */
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
