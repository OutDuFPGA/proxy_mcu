/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    icache.c
  * @brief   This file provides code for the configuration
  *          of the ICACHE instances.
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
#include "icache.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ICACHE init function */
void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */
  /* USER CODE BEGIN ICACHE_Init 2 */
  /* The generated init never switched the cache on, so every fetch waited
     out the 3 flash wait states (2026-10-14: the SN0_HS interrupt took ~180
     cycles to reach its first GPIO write). Default 2-way mode. */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ICACHE_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

