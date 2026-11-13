/**
  * @file    host_link.h
  * @brief   Line-based link to the M5S ISP SoC on UART4 (PA0/PA1, J12), 115200 8N1.
  *          RX by interrupt into a line buffer; TX through a ring drained from
  *          the main loop (same scheme as the console). Lines are handed to
  *          LENS_HostCommand(); see lens_ctrl.c for the protocol.
  */
#ifndef HOST_LINK_H
#define HOST_LINK_H

#include "main.h"

void HostLink_Init(void);
void HostLink_Task(void);                 /* main loop: run commands, drain TX */
void HostLink_Print(const char *s);       /* queue text to the SoC */
void HostLink_RxCplt(UART_HandleTypeDef *huart);   /* from HAL_UART_RxCpltCallback */
void HostLink_Error(UART_HandleTypeDef *huart);    /* from HAL_UART_ErrorCallback  */

#endif
