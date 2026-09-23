/**
  ******************************************************************************
  * @file    ir_led_cli.h
  * @brief   UART4 console command layer for the IR LED driver (ir_led_driver.h).
  *
  * Kept separate from ir_led_driver.c so the driver module has no UART/CLI
  * dependency - this file only calls the public ir_led_driver.h API.
  ******************************************************************************
  */
#ifndef __IR_LED_CLI_H__
#define __IR_LED_CLI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>

/* Recognizes and executes any "ir ..." command. Writes a response string
   (always null-terminated, no trailing "MCU> " prompt - the caller adds
   that, matching the rest of the console's convention) into resp/resp_size.
   Returns true if cmd was an IR command (handled or a usage error - either
   way resp is filled in), false if cmd doesn't start with "ir" at all, so
   the caller's existing unknown-command handling still applies. */
bool IR_LED_CLI_ProcessCommand(const char *cmd, char *resp, size_t resp_size);

/* Returns the "ir help" submenu text (also shown for a bare "ir" with no
   subcommand, or an unrecognized ir sub-command). */
const char *IR_LED_CLI_HelpText(void);

#ifdef __cplusplus
}
#endif

#endif /* __IR_LED_CLI_H__ */
