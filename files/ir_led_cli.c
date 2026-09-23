/**
  ******************************************************************************
  * @file    ir_led_cli.c
  * @brief   UART4 console command layer for the IR LED driver.
  ******************************************************************************
  */
#include "ir_led_cli.h"
#include "ir_led_driver.h"
#include <stdio.h>
#include <string.h>

static const char *mode_name(IR_LedMode_t m)
{
    switch (m) {
        case IR_MODE_OFF:          return "off";
        case IR_MODE_CONTINUOUS:   return "cont";
        case IR_MODE_FRAME_STROBE: return "strobe";
        default:                   return "?";
    }
}

static bool parse_mode(const char *s, IR_LedMode_t *out)
{
    if      (strcmp(s, "off")    == 0) { *out = IR_MODE_OFF;          return true; }
    else if (strcmp(s, "cont")   == 0) { *out = IR_MODE_CONTINUOUS;   return true; }
    else if (strcmp(s, "strobe") == 0) { *out = IR_MODE_FRAME_STROBE; return true; }
    return false;
}

const char *IR_LED_CLI_HelpText(void)
{
    return
        "  ir status                    - Show enable/mode/fault/window/level for both channels\r\n"
        "  ir enable / ir disable        - Master enable for both IR driver boards (IR_ENB_MCU)\r\n"
        "  ir ch<0|1> mode off|cont|strobe - Per-channel operating mode\r\n"
        "  ir ch<0|1> current <0-4095>   - Per-channel analog current setpoint (needs DAC1, see TODO)\r\n"
        "  ir ch<0|1> window <start> <end> - Per-channel strobe window, in sensor lines from VSYNC\r\n"
        "  ir ch<0|1> clearfault         - Re-arm a latched fault on that channel\r\n";
}

bool IR_LED_CLI_ProcessCommand(const char *cmd, char *resp, size_t resp_size)
{
    if (strncmp(cmd, "ir", 2) != 0) {
        return false; /* not an IR command at all - let the caller handle it */
    }
    /* Must be "ir" alone or "ir " followed by something - not e.g. "irrelevant" */
    if (cmd[2] != '\0' && cmd[2] != ' ') {
        return false;
    }

    const char *rest = (cmd[2] == '\0') ? "" : (cmd + 3);

    if (rest[0] == '\0' || strcmp(rest, "help") == 0) {
        snprintf(resp, resp_size, "%s", IR_LED_CLI_HelpText());
        return true;
    }

    if (strcmp(rest, "status") == 0) {
        char ch0[128], ch1[128];
        uint32_t s0, e0, s1, e1;
        IR_LED_GetWindow(IR_CH0, &s0, &e0);
        IR_LED_GetWindow(IR_CH1, &s1, &e1);
        snprintf(ch0, sizeof(ch0), "CH0: mode=%s fault=%d window=[%lu,%lu] level=%u/4095\r\n",
                 mode_name(IR_LED_GetMode(IR_CH0)), IR_LED_HasFault(IR_CH0),
                 (unsigned long)s0, (unsigned long)e0, IR_LED_GetCurrentLevel(IR_CH0));
        snprintf(ch1, sizeof(ch1), "CH1: mode=%s fault=%d window=[%lu,%lu] level=%u/4095\r\n",
                 mode_name(IR_LED_GetMode(IR_CH1)), IR_LED_HasFault(IR_CH1),
                 (unsigned long)s1, (unsigned long)e1, IR_LED_GetCurrentLevel(IR_CH1));
        snprintf(resp, resp_size, "IR master: %s\r\n%s%s",
                 IR_LED_IsEnabled() ? "ENABLED" : "DISABLED", ch0, ch1);
        return true;
    }

    if (strcmp(rest, "enable") == 0) {
        IR_LED_Enable(true);
        snprintf(resp, resp_size, "SUCCESS: IR enabled (both channels resume their configured mode)\r\n");
        return true;
    }

    if (strcmp(rest, "disable") == 0) {
        IR_LED_Enable(false);
        snprintf(resp, resp_size, "SUCCESS: IR disabled (both channels forced off)\r\n");
        return true;
    }

    int ch_num;
    char sub[16];
    if (sscanf(rest, "ch%d mode %15s", &ch_num, sub) == 2) {
        if (ch_num != 0 && ch_num != 1) {
            snprintf(resp, resp_size, "ERROR: channel must be 0 or 1\r\n");
            return true;
        }
        IR_LedMode_t m;
        if (!parse_mode(sub, &m)) {
            snprintf(resp, resp_size, "ERROR: mode must be off|cont|strobe\r\n");
            return true;
        }
        IR_LED_SetMode((IR_Channel_t)ch_num, m);
        snprintf(resp, resp_size, "SUCCESS: ch%d mode = %s\r\n", ch_num, mode_name(m));
        return true;
    }

    unsigned int level;
    if (sscanf(rest, "ch%d current %u", &ch_num, &level) == 2) {
        if (ch_num != 0 && ch_num != 1) {
            snprintf(resp, resp_size, "ERROR: channel must be 0 or 1\r\n");
            return true;
        }
        if (level > 4095) {
            snprintf(resp, resp_size, "ERROR: current level must be 0-4095\r\n");
            return true;
        }
        IR_LED_SetCurrentLevel((IR_Channel_t)ch_num, (uint16_t)level);
        snprintf(resp, resp_size, "SUCCESS: ch%d current = %u/4095 (DAC1 not yet wired up - see TODO)\r\n",
                 ch_num, level);
        return true;
    }

    unsigned long w_start, w_end;
    if (sscanf(rest, "ch%d window %lu %lu", &ch_num, &w_start, &w_end) == 3) {
        if (ch_num != 0 && ch_num != 1) {
            snprintf(resp, resp_size, "ERROR: channel must be 0 or 1\r\n");
            return true;
        }
        if (w_start >= w_end) {
            snprintf(resp, resp_size, "ERROR: window start must be less than end\r\n");
            return true;
        }
        IR_LED_SetWindow((IR_Channel_t)ch_num, (uint32_t)w_start, (uint32_t)w_end);
        snprintf(resp, resp_size, "SUCCESS: ch%d window = [%lu,%lu]\r\n", ch_num, w_start, w_end);
        return true;
    }

    if (sscanf(rest, "ch%d clearfault", &ch_num) == 1) {
        if (ch_num != 0 && ch_num != 1) {
            snprintf(resp, resp_size, "ERROR: channel must be 0 or 1\r\n");
            return true;
        }
        IR_LED_ClearFault((IR_Channel_t)ch_num);
        snprintf(resp, resp_size,
                 "SUCCESS: ch%d fault cleared (master enable still off if a fault dropped it - "
                 "use 'ir enable' to re-arm)\r\n", ch_num);
        return true;
    }

    snprintf(resp, resp_size, "ERROR: unrecognized ir command. %s", IR_LED_CLI_HelpText());
    return true;
}
