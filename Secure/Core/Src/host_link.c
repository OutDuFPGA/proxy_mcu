/**
  * @file    host_link.c
  * @brief   M5S SoC command link on UART4 / J12 - see host_link.h.
  */
#include "host_link.h"
#include "usart.h"
#include "lens_ctrl.h"
#include <string.h>

#define HL_LINE_MAX  48
#define HL_TX_SIZE   512

static uint8_t  s_rx_byte;
static char     s_line[HL_LINE_MAX];
static uint8_t  s_len;
static char     s_ready[HL_LINE_MAX];     /* completed line for the main loop */
static volatile uint8_t s_ready_flag;
static volatile uint8_t s_overrun;        /* line arrived while one was pending */

static volatile uint8_t  s_tx[HL_TX_SIZE];
static volatile uint16_t s_tx_head, s_tx_tail;

void HostLink_Print(const char *s)
{
  uint32_t pm = __get_PRIMASK();
  __disable_irq();
  while (*s) {
    uint16_t next = (uint16_t)((s_tx_head + 1u) % HL_TX_SIZE);
    if (next == s_tx_tail) break;          /* full: drop rather than block */
    s_tx[s_tx_head] = (uint8_t)*s++;
    s_tx_head = next;
  }
  __set_PRIMASK(pm);
}

void HostLink_Init(void)
{
  __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  (void)huart4.Instance->RDR;
  s_len = 0; s_ready_flag = 0;
  HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
}

void HostLink_RxCplt(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART4) return;
  uint8_t c = s_rx_byte;
  HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);

  if (c == '\r' || c == '\n') {
    if (s_len == 0) return;
    s_line[s_len] = 0;
    if (s_ready_flag) { s_overrun = 1; }
    else { memcpy(s_ready, s_line, (size_t)s_len + 1u); s_ready_flag = 1; }
    s_len = 0;
  } else if (c >= 0x20 && c < 0x7F) {
    if (s_len < HL_LINE_MAX - 1) s_line[s_len++] = (char)c;
  }
}

void HostLink_Error(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART4) return;
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
}

void HostLink_Task(void)
{
  if (s_ready_flag) {
    LENS_HostCommand(s_ready, HostLink_Print);
    s_ready_flag = 0;
  }
  if (s_overrun) { s_overrun = 0; HostLink_Print("ERR OVERRUN\n"); }

  /* If RX ever got disarmed (e.g. HAL busy at re-arm), re-arm it. */
  if (huart4.RxState == HAL_UART_STATE_READY) HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);

  while (s_tx_tail != s_tx_head) {
    if ((huart4.Instance->ISR & USART_ISR_TXE_TXFNF) == 0u) return;
    huart4.Instance->TDR = s_tx[s_tx_tail];
    s_tx_tail = (uint16_t)((s_tx_tail + 1u) % HL_TX_SIZE);
  }
}
