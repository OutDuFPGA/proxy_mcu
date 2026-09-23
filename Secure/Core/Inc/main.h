/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined ( __ICCARM__ )
#  define CMSE_NS_CALL  __cmse_nonsecure_call
#  define CMSE_NS_ENTRY __cmse_nonsecure_entry
#else
#  define CMSE_NS_CALL  __attribute((cmse_nonsecure_call))
#  define CMSE_NS_ENTRY __attribute((cmse_nonsecure_entry))
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* Function pointer declaration in non-secure*/
#if defined ( __ICCARM__ )
typedef void (CMSE_NS_CALL *funcptr)(void);
#else
typedef void CMSE_NS_CALL (*funcptr)(void);
#endif

/* typedef for non-secure callback functions */
typedef funcptr funcptr_NS;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ALCO_ANA_Pin GPIO_PIN_0
#define ALCO_ANA_GPIO_Port GPIOC
#define ALCO_DIG_Pin GPIO_PIN_1
#define ALCO_DIG_GPIO_Port GPIOC
#define PIR_INT_3V3_Pin GPIO_PIN_2
#define PIR_INT_3V3_GPIO_Port GPIOC
#define FLASH_Pin GPIO_PIN_3
#define FLASH_GPIO_Port GPIOC
#define UART2_TXD_Pin GPIO_PIN_0
#define UART2_TXD_GPIO_Port GPIOA
#define UART2_RXD_Pin GPIO_PIN_1
#define UART2_RXD_GPIO_Port GPIOA
#define UART0_TXD_Pin GPIO_PIN_2
#define UART0_TXD_GPIO_Port GPIOA
#define UART0_RXD_Pin GPIO_PIN_3
#define UART0_RXD_GPIO_Port GPIOA
#define DAC_DIM0_Pin GPIO_PIN_4
#define DAC_DIM0_GPIO_Port GPIOA
#define DAC_DIM1_Pin GPIO_PIN_5
#define DAC_DIM1_GPIO_Port GPIOA
#define UART1_TXD_3V3_Pin GPIO_PIN_6
#define UART1_TXD_3V3_GPIO_Port GPIOA
#define UART1_RXD_3V3_Pin GPIO_PIN_7
#define UART1_RXD_3V3_GPIO_Port GPIOA
#define SN0_VS_Pin GPIO_PIN_4
#define SN0_VS_GPIO_Port GPIOC
#define HAPTIC_ENB_Pin GPIO_PIN_5
#define HAPTIC_ENB_GPIO_Port GPIOC
#define HAPTIC_PWM_Pin GPIO_PIN_0
#define HAPTIC_PWM_GPIO_Port GPIOB
#define SN0_HS_Pin GPIO_PIN_1
#define SN0_HS_GPIO_Port GPIOB
#define SENS_TEMP_Pin GPIO_PIN_2
#define SENS_TEMP_GPIO_Port GPIOB
#define I2C0_SCL_3V3_Pin GPIO_PIN_10
#define I2C0_SCL_3V3_GPIO_Port GPIOB
#define I2C0_SDA_3V3_Pin GPIO_PIN_11
#define I2C0_SDA_3V3_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_12
#define IMU_INT_GPIO_Port GPIOB
#define PWM_DIM0_Pin GPIO_PIN_14
#define PWM_DIM0_GPIO_Port GPIOB
#define PWM_DIM1_Pin GPIO_PIN_15
#define PWM_DIM1_GPIO_Port GPIOB
#define IRIS_EN_Pin GPIO_PIN_11
#define IRIS_EN_GPIO_Port GPIOD
#define IRIS_STEP_Pin GPIO_PIN_12
#define IRIS_STEP_GPIO_Port GPIOD
#define IRIS_DIR_Pin GPIO_PIN_6
#define IRIS_DIR_GPIO_Port GPIOC
#define FOCUS_STEP_Pin GPIO_PIN_7
#define FOCUS_STEP_GPIO_Port GPIOC
#define FOCUS_DIR_Pin GPIO_PIN_8
#define FOCUS_DIR_GPIO_Port GPIOC
#define FOCUS_EN_Pin GPIO_PIN_8
#define FOCUS_EN_GPIO_Port GPIOA
#define IR_ENB_MCU_Pin GPIO_PIN_11
#define IR_ENB_MCU_GPIO_Port GPIOA
#define IR_FBC_MCU_Pin GPIO_PIN_12
#define IR_FBC_MCU_GPIO_Port GPIOA
#define LED1_TEMP_ADCIN_Pin GPIO_PIN_15
#define LED1_TEMP_ADCIN_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_10
#define LED_RED_GPIO_Port GPIOC
#define LED0_TEMP_ADCIN_Pin GPIO_PIN_11
#define LED0_TEMP_ADCIN_GPIO_Port GPIOC
#define ZOOM_EN_Pin GPIO_PIN_3
#define ZOOM_EN_GPIO_Port GPIOB
#define ZOOM_DIR_Pin GPIO_PIN_4
#define ZOOM_DIR_GPIO_Port GPIOB
#define ZOOM_STEP_Pin GPIO_PIN_5
#define ZOOM_STEP_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_6
#define LED_GREEN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB
#define PROXY_HB_Pin GPIO_PIN_8
#define PROXY_HB_GPIO_Port GPIOB
#define IR_FAULT0_Pin GPIO_PIN_9
#define IR_FAULT0_GPIO_Port GPIOB
#define IR_FAULT1_Pin GPIO_PIN_0
#define IR_FAULT1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
