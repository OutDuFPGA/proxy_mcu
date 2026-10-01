/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "gtzc_s.h"
#include "icache.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TMC2300.h"
#include "ir_led_driver.h"
#include "ir_led_cli.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN VTOR_TABLE */
/* Non-secure Vector table to jump to (internal Flash Bank2 here)             */
/* Caution: address must correspond to non-secure internal Flash where is     */
/*          mapped in the non-secure vector table                             */
#define VTOR_TABLE_NS_START_ADDR  0x08080000U

/* This project currently ships a Secure image ONLY - see STM32CubeIDE/, which
   contains a "Secure" application and no NonSecure counterpart, and note that
   every peripheral this firmware uses (UART4, USART1/2, TIM3/4/8, ICACHE) is
   marked GTZC_TZSC_PERIPH_SEC in MX_GTZC_S_Init(). All the application logic -
   the UART4 console, the TMC2300 motor drivers and the IR LED driver - lives
   in this Secure image.

   NonSecure_Init() jumps to VTOR_TABLE_NS_START_ADDR and never returns, so
   calling it unconditionally left the console command loop at the bottom of
   main() permanently unreachable: the banner printed and the RX interrupt
   buffered whole commands, but cmd_ready_flag was never serviced and nothing
   ever executed. With no non-secure image programmed at 0x08080000 the jump
   also lands in blank flash.

   So the jump is off by default. Define USE_NONSECURE_APP (project symbol, or
   here) once a non-secure application actually exists at 0x08080000; at that
   point the console should move into the non-secure image, or reach the secure
   drivers through non-secure callable (CMSE_NS_ENTRY) veneers in
   secure_nsc.c. */
/* #define USE_NONSECURE_APP */
/* USER CODE END VTOR_TABLE*/

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define CMD_BUFFER_SIZE 64
static uint8_t rx_byte;
static char cmd_buffer[CMD_BUFFER_SIZE];
static uint16_t cmd_index = 0;
static volatile uint8_t cmd_ready_flag = 0;

/* Console TX ring buffer.
   Two producers write to this console: the main loop (command output) and
   the UART RX ISR (character echo). They used to each call the blocking
   HAL_UART_Transmit() directly, but that gates on huart->gState, so
   whichever one arrived second got HAL_BUSY back and silently dropped its
   bytes. The visible effect was a keypress during any command output losing
   its echo - the key looked ignored, so it got pressed again - and command
   output issued mid-echo being swallowed.
   Routing every byte through one buffer that only the main loop drains
   removes the contention: producers never block and never touch the
   peripheral, and there is exactly one writer to TDR. */
#define CLI_TX_BUF_SIZE 2048
static volatile uint8_t  cli_tx_buf[CLI_TX_BUF_SIZE];
static volatile uint16_t cli_tx_head = 0;   /* producers advance this */
static volatile uint16_t cli_tx_tail = 0;   /* drain advances this    */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
#ifdef USE_NONSECURE_APP
static void NonSecure_Init(void);
#endif
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void CLI_Print(const char *str);
static void CLI_TxDrain(void);
static void CLI_DisplayMenu(void);
static void CLI_Process_Command(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Appends one byte. Safe from both the main loop and the RX ISR: the
   head update is short enough that masking interrupts around it costs
   less than any lock-free scheme would. A full buffer drops the byte
   rather than blocking - stalling the ISR to wait for a console is never
   the right trade on this board. */
static void CLI_TxByte(uint8_t c)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint16_t next = (uint16_t)((cli_tx_head + 1u) % CLI_TX_BUF_SIZE);
  if (next != cli_tx_tail) {
    cli_tx_buf[cli_tx_head] = c;
    cli_tx_head = next;
  }
  __set_PRIMASK(primask);
}

static void CLI_Print(const char *str) {
  while (*str != '\0') {
    CLI_TxByte((uint8_t)*str++);
  }
}

/* Main loop only, so the tail has a single owner and needs no guarding.
   Writes TDR directly rather than going through the HAL, which keeps this
   independent of huart4.gState - the whole point of the exercise. Returns
   as soon as the FIFO is full so the loop keeps turning. */
static void CLI_TxDrain(void)
{
  while (cli_tx_tail != cli_tx_head) {
    if ((huart4.Instance->ISR & USART_ISR_TXE_TXFNF) == 0u) {
      return;
    }
    huart4.Instance->TDR = cli_tx_buf[cli_tx_tail];
    cli_tx_tail = (uint16_t)((cli_tx_tail + 1u) % CLI_TX_BUF_SIZE);
  }
}

static void CLI_DisplayMenu(void) {
  CLI_Print("\r\n=================================\r\n");
  CLI_Print("   HARDWARE PWM CONSOLE TEST     \r\n");
  CLI_Print("=================================\r\n");
  CLI_Print("  help                  - Display this menu\r\n");
  CLI_Print("  <color>_on / _off     - e.g. red_on, green_off\r\n");
  CLI_Print("  zoom_run <dir> <hz>   - Spin Zoom continuously\r\n");
  CLI_Print("  zoom_stop             - Stop Zoom\r\n");
  CLI_Print("  focus_run <dir> <hz>  - Spin Focus continuously\r\n");
  CLI_Print("  focus_stop            - Stop Focus\r\n");
  CLI_Print("  iris_run <dir> <hz>   - Spin Iris continuously\r\n");
  CLI_Print("  iris_stop             - Stop Iris\r\n");
  CLI_Print("  focus_dump            - Dump Focus Registers\r\n");
  CLI_Print("  ir help               - IR LED current (status, ma, mv)\r\n");
  CLI_Print("\r\nMCU> ");
}

static void CLI_Process_Command(char *cmd) {
  cmd[strcspn(cmd, "\r\n")] = 0;
  char resp_buf[64];

  if (strlen(cmd) == 0) { CLI_Print("MCU> "); return; }

  if (strcmp(cmd, "help") == 0) { CLI_DisplayMenu(); }
  else if (strcmp(cmd, "green_on") == 0)  { HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET); CLI_Print("SUCCESS: Green ON\r\nMCU> "); }
  else if (strcmp(cmd, "green_off") == 0) { HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET); CLI_Print("SUCCESS: Green OFF\r\nMCU> "); }
  else if (strcmp(cmd, "blue_on") == 0)   { HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET); CLI_Print("SUCCESS: Blue ON\r\nMCU> "); }
  else if (strcmp(cmd, "blue_off") == 0)  { HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET); CLI_Print("SUCCESS: Blue OFF\r\nMCU> "); }
  else if (strcmp(cmd, "red_on") == 0)    { HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); CLI_Print("SUCCESS: Red ON\r\nMCU> "); }
  else if (strcmp(cmd, "red_off") == 0)   { HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET); CLI_Print("SUCCESS: Red OFF\r\nMCU> "); }

  /* ZOOM */
  else if (strncmp(cmd, "zoom_run ", 9) == 0) {
    char *p = cmd + 9;
    long dir = strtol(p, &p, 10);
    long hz = strtol(p, NULL, 10);
    TMC2300_Run(MOTOR_ZOOM, (uint8_t)dir, (uint32_t)hz);
    snprintf(resp_buf, sizeof(resp_buf), "SUCCESS: Zoom spinning %ld Hz\r\nMCU> ", hz);
    CLI_Print(resp_buf);
  }
  else if (strcmp(cmd, "zoom_stop") == 0) { TMC2300_Stop(MOTOR_ZOOM); CLI_Print("SUCCESS: Zoom stopped\r\nMCU> "); }

  /* FOCUS */
  else if (strncmp(cmd, "focus_run ", 10) == 0) {
    char *p = cmd + 10;
    long dir = strtol(p, &p, 10);
    long hz = strtol(p, NULL, 10);
    TMC2300_Run(MOTOR_FOCUS, (uint8_t)dir, (uint32_t)hz);
    snprintf(resp_buf, sizeof(resp_buf), "SUCCESS: Focus spinning %ld Hz\r\nMCU> ", hz);
    CLI_Print(resp_buf);
  }
  else if (strcmp(cmd, "focus_stop") == 0) { TMC2300_Stop(MOTOR_FOCUS); CLI_Print("SUCCESS: Focus stopped\r\nMCU> "); }

  /* IRIS */
  else if (strncmp(cmd, "iris_run ", 9) == 0) {
    char *p = cmd + 9;
    long dir = strtol(p, &p, 10);
    long hz = strtol(p, NULL, 10);
    TMC2300_Run(MOTOR_IRIS, (uint8_t)dir, (uint32_t)hz);
    snprintf(resp_buf, sizeof(resp_buf), "SUCCESS: Iris spinning %ld Hz\r\nMCU> ", hz);
    CLI_Print(resp_buf);
  }
  else if (strcmp(cmd, "iris_stop") == 0) { TMC2300_Stop(MOTOR_IRIS); CLI_Print("SUCCESS: Iris stopped\r\nMCU> "); }

  /* DIAGNOSTICS */
  else if (strcmp(cmd, "focus_dump") == 0) { TMC2300_DumpRegisters(); CLI_Print("MCU> "); }

  /* IR LED driver console (enable/disable, per-channel mode, current, window,
     fault clear, status) — see ir_led_cli.c for the full command set ("ir help").
     Streams output line-by-line via CLI_Print directly (no large local buffer) -
     see ir_led_cli.h for why that matters on this Secure-partition stack. */
  else if (strncmp(cmd, "ir", 2) == 0 && (cmd[2] == '\0' || cmd[2] == ' ')) {
    IR_LED_CLI_ProcessCommand(cmd, CLI_Print);
    CLI_Print("MCU> ");
  }

  else { CLI_Print("ERROR: Unknown Command.\r\nMCU> "); }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* GTZC initialisation */
  MX_GTZC_S_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_ICACHE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 1. CRITICAL: Kill the unhandled LPUART1 interrupt */
  /* 1. CRITICAL: Kill the unhandled USART1 interrupt */
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    /* 2. Flush startup hardware noise */
    __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    volatile uint32_t tmpreg = huart4.Instance->RDR;
    (void)tmpreg;

    /* 3. Reset state */
    cmd_index = 0;
    cmd_ready_flag = 0;
    memset(cmd_buffer, 0, CMD_BUFFER_SIZE);

    /* 4. Initialize Motor Drivers */
    TMC2300_Init();

    /* 5. Initialize IR LED driver (LT3763 boards on J5/J6).
       Deliberately comes up DARK: master enable off, both channels
       IR_MODE_OFF, both CTRL1 setpoints at 0mA. The previous build booted
       straight into CONTINUOUS+enabled, which was already the wrong default
       for a 4A illuminator and became actively unsafe once the gate polarity
       was corrected - PWM_DIMx low is LEDs ON (see ir_led_driver.h).
       Output enable defaults ON (IR_ENB_MCU is unconnected, so gating on it
       only hid working channels), so a channel lights as soon as its mode is
       set. Bring one up from the console with:
           ir ch0 mv 180
           ir ch0 mode cont
       See the IR bring-up procedure in docs/ for the full sequence. */
    IR_LED_Init();
    IR_LED_SetWindow(IR_CH0, 0, 2160); /* TODO: replace 2160 with the sensor's
                                           actual active-lines-per-frame count
                                           (only used in IR_MODE_FRAME_STROBE) */
    IR_LED_SetWindow(IR_CH1, 0, 2160);

    /* 6. Display UI and start listening on UART4 */
    CLI_DisplayMenu();
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
  /* USER CODE END 2 */

#ifdef USE_NONSECURE_APP
  /*************** Setup and jump to non-secure *******************************/

  NonSecure_Init();

  /* Non-secure software does not return, this code is not executed */
#endif /* USE_NONSECURE_APP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (cmd_ready_flag)
    {
      CLI_Process_Command(cmd_buffer);
      memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
      cmd_index = 0;
      cmd_ready_flag = 0;
    }

    /* Stall detection for the three TMC2300 axes. Polled here rather than from
       the DIAG pins' EXTI lines because a stall only has to stop the step
       train "soon", and this keeps the sensor-sync EXTI budget free. */
    TMC2300_StallGuard_Poll();

    /* Confirms LT3763 faults with blanking. Must be polled: the FAULT line is
       legitimately asserted whenever a channel is dark, so an edge alone
       means nothing. See note 3 in ir_led_driver.h. */
    IR_LED_Task();

    /* Single owner of the UART TX register. Must run every iteration or
       console output simply never leaves the ring buffer. */
    CLI_TxDrain();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

#ifdef USE_NONSECURE_APP
/**
  * @brief  Non-secure call function
  *         This function is responsible for Non-secure initialization and switch
  *         to non-secure state
  * @retval None
  */
static void NonSecure_Init(void)
{
  funcptr_NS NonSecure_ResetHandler;

  SCB_NS->VTOR = VTOR_TABLE_NS_START_ADDR;

  /* Set non-secure main stack (MSP_NS) */
  __TZ_set_MSP_NS((*(uint32_t *)VTOR_TABLE_NS_START_ADDR));

  /* Get non-secure reset handler */
  NonSecure_ResetHandler = (funcptr_NS)(*((uint32_t *)((VTOR_TABLE_NS_START_ADDR) + 4U)));

  /* Start non-secure state software application */
  NonSecure_ResetHandler();
}
#endif /* USE_NONSECURE_APP */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_1);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    /* Swallows the second half of a CRLF pair. Terminals disagree about what
       Enter sends - CR, LF, or both - and without this a CRLF terminal gets
       two line submissions per keypress and so a doubled prompt. */
    static uint8_t last_was_cr = 0;

    uint8_t c = rx_byte;

    /* Re-arm FIRST. Everything below is bookkeeping that costs microseconds,
       but any byte arriving while reception is disarmed is lost outright -
       and the old order left RX deaf for the whole duration of a blocking
       echo. rx_byte is copied above because this call re-points the HAL at
       it immediately. */
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);

    /* Nothing is echoed up front any more. Each case below emits exactly the
       bytes that match what it did to cmd_buffer, so the screen can never
       disagree with what will actually be parsed. */
    if (c == '\r' || c == '\n') {
      if (c == '\n' && last_was_cr) {
        last_was_cr = 0;                 /* CRLF: the CR already submitted */
      } else {
        last_was_cr = (c == '\r');
        /* A full CRLF, not the bare CR the old code echoed. With only CR the
           cursor returned to column 0 and the command response overwrote the
           line the user had just typed. */
        CLI_Print("\r\n");
        if (cmd_index > 0) {
          cmd_buffer[cmd_index] = '\0';
          cmd_ready_flag = 1;            /* main loop prints the next prompt */
        } else {
          CLI_Print("MCU> ");
        }
      }
    }
    else if (c == '\b' || c == 0x7F) {   /* BS and DEL both erase */
      last_was_cr = 0;
      if (cmd_index > 0) {
        cmd_index--;
        cmd_buffer[cmd_index] = '\0';
        /* Back over the character, overwrite it with a space, back again.
           DEL (0x7F) must never be echoed raw - terminals render it as a
           stray glyph instead of erasing. */
        CLI_Print("\b \b");
      }
      /* Nothing emitted on an empty line, so the cursor cannot walk back
         over the "MCU> " prompt and start eating it. */
    }
    else if (c >= 0x20 && c < 0x7F) {     /* printable ASCII only */
      last_was_cr = 0;
      if (cmd_index < (CMD_BUFFER_SIZE - 1)) {
        cmd_buffer[cmd_index++] = c;
        CLI_TxByte(c);                    /* echo only what was stored */
      }
      /* Buffer full: drop silently. Echoing a character that was not stored
         would leave the display showing a command different from the one
         about to be parsed. */
    }
    else {
      /* Control characters and escape sequences - arrow keys, Ctrl-C, and
         so on - are discarded rather than inserted into the command. The old
         code stored them verbatim, so a single arrow key put three junk
         bytes ("\x1b[A") into the buffer and the command silently failed. */
      last_was_cr = 0;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4) {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region 0 and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08FFF000;
  MPU_InitStruct.LimitAddress = 0x08FFFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Attribute 0 and the memory to be protected
  */
  MPU_AttributesInit.Number = MPU_ATTRIBUTES_NUMBER0;
  MPU_AttributesInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
