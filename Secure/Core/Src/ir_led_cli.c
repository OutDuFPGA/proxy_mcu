/**
  ******************************************************************************
  * @file    ir_led_cli.c
  * @brief   UART4 console command layer for the LT3763 IR LED driver.
  *
  * Operating mode: the sensor's FLASH (TOUT0) switches the LED through
  * Q2/Q3, with PWM_DIM0/1 held HIGH permanently. The only thing the MCU
  * controls is the CH0 current, through DAC_DIM0 (LT3763 CTRL1). Only CH0
  * has an LED driver fitted, so these commands address CH0 alone.
  ******************************************************************************
  */
#include "ir_led_cli.h"
#include "ir_led_driver.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static void print_help(IR_LED_CLI_PrintFn p)
{
    p("  ir status         - FLASH input, PWM_DIM0, DAC_DIM0 and FAULT0 state\r\n");
    p("  ir ma <mA>        - Set LED current (via DAC_DIM0)\r\n");
    p("  ir mv <mV>        - Set DAC_DIM0 voltage directly\r\n");
    p("  LED on/off follows the sensor FLASH pulse; PWM_DIM0 is fixed HIGH.\r\n");
}

static void warn_if_over_abs_max(uint32_t mv, IR_LED_CLI_PrintFn p)
{
    char l[100];
    if (mv <= IR_CTRL_ABS_MAX_MV) return;
    snprintf(l, sizeof(l), "WARNING: %lu mV exceeds the LT3763 CTRL1 abs max of %lu mV.\r\n",
             (unsigned long)mv, (unsigned long)IR_CTRL_ABS_MAX_MV);
    p(l);
}

static void print_status(IR_LED_CLI_PrintFn p)
{
    char l[110];
    IR_FlashInput_t fl;

    IR_LED_GetFlashInput(&fl);
    if (fl.toggling) {
        uint32_t mhz = fl.period_us ? (1000000000U / fl.period_us) : 0U;   /* milli-Hz */
        snprintf(l, sizeof(l), "FLASH (PC3 in):   %s  %lu.%02lu Hz  low %lu.%02lu ms  falls %lu\r\n",
                 fl.level_high ? "HIGH" : "LOW ",
                 (unsigned long)(mhz / 1000U), (unsigned long)((mhz % 1000U) / 10U),
                 (unsigned long)(fl.low_us / 1000U), (unsigned long)((fl.low_us % 1000U) / 10U),
                 (unsigned long)fl.falls);
    } else {
        snprintf(l, sizeof(l), "FLASH (PC3 in):   %s  NOT toggling (no edge for %lu ms, falls %lu)\r\n",
                 fl.level_high ? "HIGH" : "LOW ",
                 (unsigned long)fl.idle_ms, (unsigned long)fl.falls);
    }
    p(l);

    snprintf(l, sizeof(l), "PWM_DIM0 (PB14): %s (fixed - LED follows sensor FLASH)\r\n",
             HAL_GPIO_ReadPin(PWM_DIM0_GPIO_Port, PWM_DIM0_Pin) == GPIO_PIN_SET ? "HIGH" : "LOW");
    p(l);
    snprintf(l, sizeof(l), "DAC_DIM0 (PA4):   %lu mV (code %u) -> ~%lu mA   limit %lu mA\r\n",
             (unsigned long)IR_LED_GetCtrlVoltage_mV(IR_CH0),
             IR_LED_GetCurrentRaw(IR_CH0),
             (unsigned long)IR_LED_GetCurrent_mA(IR_CH0),
             (unsigned long)IR_LED_GetCurrentLimit_mA());
    p(l);
    snprintf(l, sizeof(l), "FAULT0 (PB9):     %s\r\n",
             IR_LED_IsFaultLineAsserted(IR_CH0) ? "low" : "high");
    p(l);
}

bool IR_LED_CLI_ProcessCommand(const char *cmd, IR_LED_CLI_PrintFn print_fn)
{
    char line[120];

    if (strncmp(cmd, "ir", 2) != 0) return false;
    if (cmd[2] != '\0' && cmd[2] != ' ') return false;   /* not "irrelevant" */

    const char *rest = (cmd[2] == '\0') ? "" : (cmd + 3);

    if (rest[0] == '\0' || strcmp(rest, "help") == 0) { print_help(print_fn);   return true; }
    if (strcmp(rest, "status") == 0)                  { print_status(print_fn); return true; }

    if (strncmp(rest, "ma ", 3) == 0) {
        long ma = strtol(rest + 3, NULL, 10);
        if (ma < 0) { print_fn("ERROR: milliamps must be >= 0\r\n"); return true; }
        if ((uint32_t)ma > IR_LED_GetCurrentLimit_mA()) {
            snprintf(line, sizeof(line), "NOTE: clamped to the current limit of %lu mA\r\n",
                     (unsigned long)IR_LED_GetCurrentLimit_mA());
            print_fn(line);
        }
        IR_LED_SetCurrent_mA(IR_CH0, (uint32_t)ma);
        snprintf(line, sizeof(line), "SUCCESS: %lu mA (DAC_DIM0 %lu mV, code %u)\r\n",
                 (unsigned long)IR_LED_GetCurrent_mA(IR_CH0),
                 (unsigned long)IR_LED_GetCtrlVoltage_mV(IR_CH0),
                 IR_LED_GetCurrentRaw(IR_CH0));
        print_fn(line);
        return true;
    }

    if (strncmp(rest, "mv ", 3) == 0) {
        long mv = strtol(rest + 3, NULL, 10);
        if (mv < 0) { print_fn("ERROR: millivolts must be >= 0\r\n"); return true; }
        uint32_t applied = IR_LED_SetCtrlVoltage_mV(IR_CH0, (uint32_t)mv);
        warn_if_over_abs_max(applied, print_fn);
        snprintf(line, sizeof(line), "SUCCESS: DAC_DIM0 = %lu mV (code %u, ~%lu mA)\r\n",
                 (unsigned long)applied, IR_LED_GetCurrentRaw(IR_CH0),
                 (unsigned long)IR_LED_GetCurrent_mA(IR_CH0));
        print_fn(line);
        return true;
    }

    print_fn("ERROR: unknown ir command - see 'ir help'\r\n");
    return true;
}
