/*
 * TMC2300.c
 */

#include "TMC2300.h"
#include <stdio.h>
#include <string.h>

/* --- UART Handles --- */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;

/* --- Timer Handles for Hardware PWM --- */
extern TIM_HandleTypeDef htim3; // Zoom (CH2)
extern TIM_HandleTypeDef htim8; // Focus (CH2)
extern TIM_HandleTypeDef htim4; // Iris (CH1)

/* --- Fallback Pin Definitions (in case CubeMX drops them) --- */
#ifndef IRIS_DIAG_Pin
  #define IRIS_DIAG_Pin GPIO_PIN_13
  #define IRIS_DIAG_GPIO_Port GPIOB
#endif
#ifndef FOCUS_DIAG_Pin
  #define FOCUS_DIAG_Pin GPIO_PIN_9
  #define FOCUS_DIAG_GPIO_Port GPIOC
#endif
#ifndef ZOOM_DIAG_Pin
  #define ZOOM_DIAG_Pin GPIO_PIN_12
  #define ZOOM_DIAG_GPIO_Port GPIOC
#endif

/* --- TMC2300 UART Addresses & Registers --- */
#define TMC2300_ADDR_FOCUS 0x00
#define REG_IOIN           0x06
#define REG_IHOLD_IRUN     0x10
#define REG_TCOOLTHRS      0x14  // <-- StallGuard Active Threshold
#define REG_SGTHRS         0x40  // <-- StallGuard Sensitivity
#define REG_CHOPCONF       0x6C
#define REG_PWMCONF        0x70

/* --- Motor State Tracking --- */
static volatile uint8_t is_zooming = 0;
static volatile uint8_t is_focusing = 0;
static volatile uint8_t is_irising = 0;

/* =========================================================================
 * PRIVATE HELPER FUNCTIONS
 * ========================================================================= */
static void Delay_us(uint32_t us) {
    uint32_t loops = us * 15;
    for(volatile uint32_t i = 0; i < loops; i++) { __NOP(); }
}

static void swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength) {
    uint8_t* crc = &datagram[datagramLength - 1];
    *crc = 0;
    for (int i = 0; i < (datagramLength - 1); i++) {
        uint8_t currentByte = datagram[i];
        for (int j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte & 0x01)) { *crc = (*crc << 1) ^ 0x07; }
            else { *crc = (*crc << 1); }
            currentByte = currentByte >> 1;
        }
    }
}

static void TMC2300_WriteRegister(uint8_t nodeAddress, uint8_t regAddress, uint32_t data) {
    uint8_t datagram[8];
    datagram[0] = 0x05;
    datagram[1] = nodeAddress;
    datagram[2] = regAddress | 0x80;
    datagram[3] = (data >> 24) & 0xFF;
    datagram[4] = (data >> 16) & 0xFF;
    datagram[5] = (data >> 8) & 0xFF;
    datagram[6] = data & 0xFF;

    swuart_calcCRC(datagram, 8);

    HAL_UART_Abort(&huart1);
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, datagram, 8, 100);
}

uint32_t TMC2300_ReadRegister(uint8_t nodeAddress, uint8_t regAddress) {
    uint8_t tx_datagram[4];
    uint8_t rx_datagram[8] = {0};

    tx_datagram[0] = 0x05;
    tx_datagram[1] = nodeAddress;
    tx_datagram[2] = regAddress & 0x7F;

    swuart_calcCRC(tx_datagram, 4);

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    HAL_UART_Abort(&huart1);
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, tx_datagram, 4, 100);
    HAL_HalfDuplex_EnableReceiver(&huart1);

    if (HAL_UART_Receive(&huart1, rx_datagram, 8, 100) == HAL_OK) {
        if (rx_datagram[0] == 0x05 && rx_datagram[1] == 0xFF) {
            return (rx_datagram[4] << 24) | (rx_datagram[5] << 16) | (rx_datagram[6] << 8) | rx_datagram[7];
        }
    }
    return 0xFFFFFFFF;
}

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */
void TMC2300_Init(void) {
  char msg[128];
  snprintf(msg, sizeof(msg), "\r\n[BOOT] TMC2300_Init Started...\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);

  __HAL_RCC_USART1_CLK_ENABLE();

  /* CRITICAL FIX 1: Disconnect PA10 so it doesn't fight PA9 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* CRITICAL FIX 2: Protect DIAG pins by forcing them to INPUT */
  GPIO_InitStruct.Pin = ZOOM_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZOOM_DIAG_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FOCUS_DIAG_Pin;
  HAL_GPIO_Init(FOCUS_DIAG_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IRIS_DIAG_Pin;
  HAL_GPIO_Init(IRIS_DIAG_GPIO_Port, &GPIO_InitStruct);

  /* Boot up the chips! */
  HAL_GPIO_WritePin(ZOOM_EN_GPIO_Port, ZOOM_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FOCUS_EN_GPIO_Port, FOCUS_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IRIS_EN_GPIO_Port, IRIS_EN_Pin, GPIO_PIN_SET);

  Delay_us(50000); // 50ms boot delay

  // Wake up UART
  uint8_t dummy = 0x00;
  HAL_UART_Abort(&huart1);
  HAL_HalfDuplex_EnableTransmitter(&huart1);
  HAL_UART_Transmit(&huart1, &dummy, 1, 100);
  Delay_us(2000);

  /* Setup Motor Profile */
  TMC2300_WriteRegister(TMC2300_ADDR_FOCUS, REG_IHOLD_IRUN, 0x00021F0F);
  TMC2300_WriteRegister(TMC2300_ADDR_FOCUS, REG_PWMCONF, 0x000000FF);
  TMC2300_WriteRegister(TMC2300_ADDR_FOCUS, REG_CHOPCONF, 0x10000002);

  /* ================= STALLGUARD CONFIG ================= */
  // 1. TCOOLTHRS: Enable StallGuard for speeds above this threshold (0xFFFFF = all speeds)
  TMC2300_WriteRegister(TMC2300_ADDR_FOCUS, REG_TCOOLTHRS, 0x000FFFFF);

  // 2. SGTHRS: Sensitivity (0 to 255).
  // IMPORTANT: Higher number = Stalls MORE easily. Lower number = Pushes harder before stalling.
  TMC2300_WriteRegister(TMC2300_ADDR_FOCUS, REG_SGTHRS, 0x00000080);
  /* ======================================================= */

  snprintf(msg, sizeof(msg), "[BOOT] TMC2300_Init Finished!\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
}

void TMC2300_SetEnable(MotorType_t motor, uint8_t state) {
  GPIO_PinState pinState = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
  if (motor == MOTOR_ZOOM) HAL_GPIO_WritePin(ZOOM_EN_GPIO_Port, ZOOM_EN_Pin, pinState);
  else if (motor == MOTOR_FOCUS) HAL_GPIO_WritePin(FOCUS_EN_GPIO_Port, FOCUS_EN_Pin, pinState);
  else if (motor == MOTOR_IRIS) HAL_GPIO_WritePin(IRIS_EN_GPIO_Port, IRIS_EN_Pin, pinState);
}

void TMC2300_Run(MotorType_t motor, uint8_t direction, uint32_t speed_hz) {
    if (speed_hz == 0) return;

    uint32_t timer_clock_hz = 1000000;
    uint32_t arr_value = (timer_clock_hz / speed_hz) - 1;
    uint32_t ccr_value = arr_value / 2;

    if (motor == MOTOR_ZOOM) {
        HAL_GPIO_WritePin(ZOOM_DIR_GPIO_Port, ZOOM_DIR_Pin, (direction) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ZOOM_EN_GPIO_Port, ZOOM_EN_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_AUTORELOAD(&htim3, arr_value);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr_value);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        is_zooming = 1;
    }
    else if (motor == MOTOR_FOCUS) {
        HAL_GPIO_WritePin(FOCUS_DIR_GPIO_Port, FOCUS_DIR_Pin, (direction) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FOCUS_EN_GPIO_Port, FOCUS_EN_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_AUTORELOAD(&htim8, arr_value);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ccr_value);
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
        is_focusing = 1;
    }
    else if (motor == MOTOR_IRIS) {
        HAL_GPIO_WritePin(IRIS_DIR_GPIO_Port, IRIS_DIR_Pin, (direction) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IRIS_EN_GPIO_Port, IRIS_EN_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_AUTORELOAD(&htim4, arr_value);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr_value);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        is_irising = 1;
    }
}

void TMC2300_Stop(MotorType_t motor) {
    if (motor == MOTOR_ZOOM) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        is_zooming = 0;
    }
    else if (motor == MOTOR_FOCUS) {
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
        is_focusing = 0;
    }
    else if (motor == MOTOR_IRIS) {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        is_irising = 0;
    }
}

uint8_t TMC2300_ReadDiag(MotorType_t motor) {
  if (motor == MOTOR_ZOOM) return (HAL_GPIO_ReadPin(ZOOM_DIAG_GPIO_Port, ZOOM_DIAG_Pin) == GPIO_PIN_SET) ? 1 : 0;
  if (motor == MOTOR_FOCUS) return (HAL_GPIO_ReadPin(FOCUS_DIAG_GPIO_Port, FOCUS_DIAG_Pin) == GPIO_PIN_SET) ? 1 : 0;
  if (motor == MOTOR_IRIS) return (HAL_GPIO_ReadPin(IRIS_DIAG_GPIO_Port, IRIS_DIAG_Pin) == GPIO_PIN_SET) ? 1 : 0;
  return 0;
}

/* Continuously checks the DIAG pins and instantly stops the timer if a stall is detected */
void TMC2300_StallGuard_Poll(void) {
    char msg[64];

    if (is_zooming && TMC2300_ReadDiag(MOTOR_ZOOM)) {
        TMC2300_Stop(MOTOR_ZOOM);
        snprintf(msg, sizeof(msg), "\r\n[ALARM] Zoom Stalled! Motor Stopped.\r\nMCU> ");
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
    }
    if (is_focusing && TMC2300_ReadDiag(MOTOR_FOCUS)) {
        TMC2300_Stop(MOTOR_FOCUS);
        snprintf(msg, sizeof(msg), "\r\n[ALARM] Focus Stalled! Motor Stopped.\r\nMCU> ");
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
    }
    if (is_irising && TMC2300_ReadDiag(MOTOR_IRIS)) {
        TMC2300_Stop(MOTOR_IRIS);
        snprintf(msg, sizeof(msg), "\r\n[ALARM] Iris Stalled! Motor Stopped.\r\nMCU> ");
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
    }
}
void TMC2300_DumpRegisters(void) {
    char buf[128];
    uint8_t target_address = 0xFF;

    snprintf(buf, sizeof(buf), "\r\n[DIAG] Scanning UART Bus for TMC2300...\r\n");
    HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);

    /* 1. Auto-Baud Scanner: Check addresses 0x00 through 0x03 */
    for (uint8_t addr = 0; addr <= 3; addr++) {
        uint32_t test_val = TMC2300_ReadRegister(addr, REG_IOIN);
        if (test_val != 0xFFFFFFFF) {
            target_address = addr;
            snprintf(buf, sizeof(buf), "[DIAG] SUCCESS! TMC2300 found at Address 0x%02X\r\n", target_address);
            HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);
            break;
        }
    }

    if (target_address == 0xFF) {
        snprintf(buf, sizeof(buf), "ERROR: No chips responded on Addresses 0x00-0x03.\r\nMCU> ");
        HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);
        return;
    }

    /* 2. Perform the actual dump on the confirmed address */
    typedef struct { uint8_t address; const char* name; } RegDef;
    const RegDef registers[] = {
        {0x00, "GCONF"},
        {0x01, "GSTAT"},
        {0x06, "IOIN"},
        {0x10, "IHOLD_IRUN"},
        {0x12, "TSTEP"},
        {0x14, "TCOOLTHRS"}, // <-- Added: Must be 0x0FFFFF
        {0x22, "VACTUAL"},
        {0x40, "SGTHRS"},    // <-- Added: Currently 0x00000040
        {0x41, "SG_VALUE"},  // <-- Added: Live mechanical load!
        {0x6C, "CHOPCONF"},
        {0x6F, "DRV_STATUS"},
        {0x70, "PWMCONF"},
        {0x71, "PWM_SCALE"},
        {0x72, "PWM_AUTO"}
    };

    snprintf(buf, sizeof(buf), "\r\n=== TMC2300 MOTOR (0x%02X) REGISTER DUMP ===\r\n", target_address);
    HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);

    for (uint8_t i = 0; i < (sizeof(registers)/sizeof(registers[0])); i++) {
        uint32_t val = TMC2300_ReadRegister(target_address, registers[i].address);
        if (val != 0xFFFFFFFF) snprintf(buf, sizeof(buf), "%-12s (0x%02X) : 0x%08lX\r\n", registers[i].name, registers[i].address, (unsigned long)val);
        else snprintf(buf, sizeof(buf), "%-12s (0x%02X) : READ FAILED\r\n", registers[i].name, registers[i].address);
        HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);
    }
    snprintf(buf, sizeof(buf), "=============================================\r\nMCU> ");
    HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);
}
