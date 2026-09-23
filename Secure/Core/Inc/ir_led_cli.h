/**
  ******************************************************************************
  * @file    ir_led_cli.h
  * @brief   UART4 console command layer for the LT3763 IR LED driver.
  *
  * Output is streamed a line at a time through a caller-supplied print
  * function rather than composed into one buffer. This partition's secure
  * stack is small and main()'s CLI handler already has its own buffer; a
  * second multi-hundred-byte local here was not worth the risk.
  *
  * Parsing uses strncmp/strtol only, matching the zoom_run/focus_run/iris_run
  * handlers in main.c. sscanf is deliberately avoided: it is not used
  * anywhere else in this codebase, and it was the cause of a console hang
  * seen on an earlier build where every "ir" subcommand that used it hung
  * while the plain-strcmp ones worked.
  ******************************************************************************
  */
#ifndef __IR_LED_CLI_H__
#define __IR_LED_CLI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef void (*IR_LED_CLI_PrintFn)(const char *str);

/* Handles any command beginning with "ir". Returns false if cmd is not an IR
   command at all, so the caller can carry on with its own dispatch. */
bool IR_LED_CLI_ProcessCommand(const char *cmd, IR_LED_CLI_PrintFn print_fn);

#ifdef __cplusplus
}
#endif

#endif /* __IR_LED_CLI_H__ */
