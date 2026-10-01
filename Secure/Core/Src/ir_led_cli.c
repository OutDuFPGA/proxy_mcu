/**
  ******************************************************************************
  * @file    ir_led_cli.c
  * @brief   UART4 console command layer for the LT3763 IR LED driver.
  ******************************************************************************
  */
#include "ir_led_cli.h"
#include "ir_led_driver.h"
#include "dac.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *mode_name(IR_LedMode_t m)
{
    switch (m) {
        case IR_MODE_OFF:          return "off";
        case IR_MODE_CONTINUOUS:   return "cont";
        case IR_MODE_FRAME_STROBE: return "strobe";
        case IR_MODE_HOST_FLASH:   return "host";
        case IR_MODE_GATE_PWM:     return "dim";
        case IR_MODE_FRAME_PULSE:  return "fpulse";
        case IR_MODE_FLASH_SYNC:   return "flash";
        default:                   return "?";
    }
}

static bool parse_mode(const char *s, IR_LedMode_t *out)
{
    if      (strcmp(s, "off")    == 0) { *out = IR_MODE_OFF;          return true; }
    else if (strcmp(s, "cont")   == 0) { *out = IR_MODE_CONTINUOUS;   return true; }
    else if (strcmp(s, "strobe") == 0) { *out = IR_MODE_FRAME_STROBE; return true; }
    else if (strcmp(s, "host")   == 0) { *out = IR_MODE_HOST_FLASH;   return true; }
    else if (strcmp(s, "flash")  == 0) { *out = IR_MODE_FLASH_SYNC;   return true; }
    return false;
}

static void print_help(IR_LED_CLI_PrintFn p)
{
    p("  ir status                        - Full state, both channels\r\n");
    p("  ir enable | ir disable           - Software all-channels on/off (on by default)\r\n");
    p("  ir flash host                    - PC3 is input-only ('mcu' is refused)\r\n");
    p("  ir ch<0|1> mode off|cont|strobe|host\r\n");
    p("  ir ch<0|1> ma <milliamps>        - Current setpoint via CTRL1 DAC\r\n");
    p("  ir ch<0|1> ramp <mA> [step] [ms] - Walk the setpoint up/down in steps\r\n");
    p("  ir ch<0|1> mv <millivolts>       - Set DAC_DIMx output voltage directly\r\n");
    p("  ir ch<0|1> dim <hz> <duty%>      - TRUE PWM dimming via LT3763 PWM pin\r\n");
    p("  ir ch<0|1> dim off               - Stop dimming (channel goes off)\r\n");
    p("  ir ch<0|1> pulse <on_ms> <off_ms> - Gate LED on/off for exact times (TIM12)\r\n");
    p("  ir ch<0|1> fpulse <ms>|off       - LED pulse at every frame start (VSYNC / 30Hz)\r\n");
    p("  ir ch<0|1> mode flash            - LED on while sensor FLASH (PC3, ADC) is active\r\n");
    p("  ir flash level                   - PC3 level, FLASH state, edge count\r\n");
    p("  ir flash polarity high|low       - Which PC3 level means exposing\r\n");
    p("  ir flash thresh <hi_mV> <lo_mV>  - ADC watchdog hysteresis thresholds\r\n");
    p("  ir ch<0|1> pwm <hz> <duty%>      - Chop DAC_DIMx between 0V and setpoint\r\n");
    p("  ir ch<0|1> pwm off               - Back to a steady DC level\r\n");
    p("  ir ch<0|1> raw <0-4095>          - Raw DAC code, for CTRL1/ISMON sweeps\r\n");
    p("  ir dac                           - DAC output state, both channels\r\n");
    p("  ir limit [<mA>]                  - Show/set the bench current cap\r\n");
    p("  ir ch<0|1> window <start> <end>  - Strobe window, sensor lines from VSYNC\r\n");
    p("  ir ch<0|1> clearfault            - Re-arm a latched fault\r\n");
    p("  ir limits                        - Board current ceiling and how it arises\r\n");
    p("  ir sync                          - SN0_HS / SN0_VS activity counters\r\n");
}

static void print_limits(IR_LED_CLI_PrintFn p)
{
    char l[110];

    p("LT3763 / USHIO board current limits:\r\n");
    snprintf(l, sizeof(l), "  Rsense (R11)        : %lu uOhm\r\n",
             (unsigned long)IR_RSENSE_UOHM);
    p(l);
    snprintf(l, sizeof(l), "  Vsense full scale   : %lu uV at CTRL = %lu mV\r\n",
             (unsigned long)IR_LT3763_VSENSE_FS_UV,
             (unsigned long)IR_LT3763_VCTRL_FS_MV);
    p(l);
    snprintf(l, sizeof(l), "  Unclamped max       : %lu mA\r\n",
             (unsigned long)(((uint64_t)IR_LT3763_VSENSE_FS_UV * 1000U) / IR_RSENSE_UOHM));
    p(l);
#if (IR_CTRL2_R_TOP_OHMS != 0)
    snprintf(l, sizeof(l), "  CTRL2 divider       : %lu / %lu off %lu mV VREF\r\n",
             (unsigned long)IR_CTRL2_R_TOP_OHMS, (unsigned long)IR_CTRL2_R_BOT_OHMS,
             (unsigned long)IR_CTRL2_VREF_MV);
    p(l);
    snprintf(l, sizeof(l), "  CTRL2 clamp         : %lu mV -> board max %lu mA\r\n",
             (unsigned long)IR_CTRL_CEILING_MV, (unsigned long)IR_LED_MAX_CURRENT_MA);
    p(l);
    p("  NOTE: CTRL2 is a fixed divider, so this ceiling applies no matter\r\n");
    p("        what CTRL1 is driven to. Only the bottom ~11% of the DAC\r\n");
    p("        range does anything. Flagged for hardware review.\r\n");
#else
    snprintf(l, sizeof(l), "  CTRL2 unclamped     -> board max %lu mA\r\n",
             (unsigned long)IR_LED_MAX_CURRENT_MA);
    p(l);
#endif
    {
        /* Codes actually in play: 0 up to the one that puts CTRL1 on the
           clamp. Everything above is regulated away, so the usable step size
           is the ceiling current spread over just those codes. */
        uint32_t top_code = ((uint32_t)IR_CTRL_CEILING_MV * 4095U) / IR_DAC_VREF_MV;
        if (top_code > 4095U) top_code = 4095U;
        snprintf(l, sizeof(l), "  Usable DAC codes    : 0-%lu of 4095\r\n",
                 (unsigned long)top_code);
        p(l);
        if (top_code > 0U) {
            snprintf(l, sizeof(l), "  Setpoint resolution : ~%lu uA per code\r\n",
                     (unsigned long)(((uint64_t)IR_LED_MAX_CURRENT_MA * 1000U) / top_code));
            p(l);
        }
    }
}

static void print_status(IR_LED_CLI_PrintFn p)
{
    char l[120];

    snprintf(l, sizeof(l), "IR limit  : %lu mA (board ceiling %lu mA)\r\n",
             (unsigned long)IR_LED_GetCurrentLimit_mA(),
             (unsigned long)IR_LED_MAX_CURRENT_MA);
    p(l);
    snprintf(l, sizeof(l), "IR output : %s (software)   FLASH net: %s   DAC1: %s\r\n",
             IR_LED_IsEnabled() ? "ENABLED" : "DISABLED",
             IR_LED_GetFlashSource() == IR_FLASH_SRC_MCU ? "mcu(low)" : "input",
             DAC1_IsReady() ? "ready" : "NOT INITIALISED");
    p(l);

    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_Channel_t ch = (IR_Channel_t)i;
        uint32_t s, e;
        IR_LED_GetWindow(ch, &s, &e);

        snprintf(l, sizeof(l), "CH%d mode=%-6s  set=%lu mA (CTRL1 %lu mV, code %u)\r\n",
                 i, mode_name(IR_LED_GetMode(ch)),
                 (unsigned long)IR_LED_GetCurrent_mA(ch),
                 (unsigned long)IR_LED_GetCtrlVoltage_mV(ch),
                 IR_LED_GetCurrentRaw(ch));
        p(l);
        snprintf(l, sizeof(l), "    fault: latched=%s line=%s edges=%lu\r\n",
                 IR_LED_HasFault(ch) ? "YES" : "no",
                 IR_LED_IsFaultLineAsserted(ch) ? "ASSERTED" : "clear",
                 (unsigned long)IR_LED_GetFaultEdgeCount(ch));
        p(l);
        {
            uint32_t gf, gd;
            uint32_t on_us, off_us, fw;
            if (IR_LED_GetFramePulse(ch, &fw)) {
                snprintf(l, sizeof(l),
                         "    frame pulse: %lu.%03lu ms per frame, sync=%s, pulses=%lu\r\n",
                         (unsigned long)(fw / 1000U), (unsigned long)(fw % 1000U),
                         IR_LED_FramePulseIsSynced() ? "VSYNC" : "free-run 30Hz",
                         (unsigned long)IR_LED_GetFramePulseCount());
                p(l);
            }
            if (IR_LED_GetGatePwm(ch, &gf, &gd)) {
                snprintf(l, sizeof(l), "    dimming: %lu Hz, %lu%% optical duty (TIM12)\r\n",
                         (unsigned long)gf, (unsigned long)gd);
                p(l);
                IR_LED_GetGatePulse(ch, &on_us, &off_us);
                snprintf(l, sizeof(l), "    gate   : on %lu us / off %lu us\r\n",
                         (unsigned long)on_us, (unsigned long)off_us);
                p(l);
            }
        }
        snprintf(l, sizeof(l), "    strobe window: lines [%lu, %lu)\r\n",
                 (unsigned long)s, (unsigned long)e);
        p(l);
    }

    /* A fault line reading ASSERTED on a dark channel is normal, not a
       problem - the LT3763 pulls FAULT low whenever the inductor current is
       below ten percent of maximum. Say so, so nobody chases it. */
    if ((IR_LED_IsFaultLineAsserted(IR_CH0) && !IR_LED_HasFault(IR_CH0)) ||
        (IR_LED_IsFaultLineAsserted(IR_CH1) && !IR_LED_HasFault(IR_CH1))) {
        p("  (FAULT asserted while dark is expected: the LT3763 flags low\r\n");
        p("   inductor current. Only a latch after illumination is a fault.)\r\n");
    }
}

/* DAC_DIMx reaches the LT3763 CTRL1 pin through R19, a 0 ohm link, so any
   voltage set here lands on that pin unattenuated. Above the part's 3V
   absolute maximum that is over-stress, and the operator needs to know
   before they see smoke rather than after. */
static void warn_if_over_abs_max(uint32_t mv, IR_LED_CLI_PrintFn p)
{
    char l[100];
    if (mv <= IR_CTRL_ABS_MAX_MV) return;
    snprintf(l, sizeof(l), "WARNING: %lu mV exceeds the LT3763 CTRL1 abs max of %lu mV.\r\n",
             (unsigned long)mv, (unsigned long)IR_CTRL_ABS_MAX_MV);
    p(l);
    p("         R19 is 0 ohm - nothing is in series. Unplug J5/J6 before\r\n");
    p("         driving this high, or you are over-stressing both drivers.\r\n");
}

static void print_dac(IR_LED_CLI_PrintFn p)
{
    char l[110];
    snprintf(l, sizeof(l), "DAC1: %s   full scale %lu mV   step %lu uV/code\r\n",
             DAC1_IsReady() ? "ready" : "NOT INITIALISED",
             (unsigned long)IR_DAC_VREF_MV, (unsigned long)IR_LED_GetCtrlStep_uV());
    p(l);
    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_Channel_t ch = (IR_Channel_t)i;
        snprintf(l, sizeof(l), "  ch%d %s: code %4u  ->  %4lu mV  (~%lu mA)\r\n",
                 i, (i == 0) ? "PA4/DAC_DIM0" : "PA5/DAC_DIM1",
                 IR_LED_GetCurrentRaw(ch),
                 (unsigned long)IR_LED_GetCtrlVoltage_mV(ch),
                 (unsigned long)IR_LED_GetCurrent_mA(ch));
        p(l);
        {
            uint32_t f, d;
            if (IR_LED_GetDacPwm(ch, &f, &d)) {
                snprintf(l, sizeof(l), "       PWM: %lu Hz, %lu%% duty (0V <-> setpoint)\r\n",
                         (unsigned long)f, (unsigned long)d);
            } else {
                snprintf(l, sizeof(l), "       PWM: off (steady DC)\r\n");
            }
            p(l);
        }
    }
    snprintf(l, sizeof(l), "  CTRL2 clamp %lu mV -> nothing above it adds current\r\n",
             (unsigned long)IR_CTRL_CEILING_MV);
    p(l);
}

/* Parses a leading "ch0 "/"ch1 " prefix. Returns the text after it, or NULL. */
static const char *parse_channel_prefix(const char *rest, int *ch_num)
{
    if (rest[0] != 'c' || rest[1] != 'h') return NULL;
    if (rest[2] != '0' && rest[2] != '1') return NULL;
    if (rest[3] != ' ') return NULL;
    *ch_num = rest[2] - '0';
    return rest + 4;
}

bool IR_LED_CLI_ProcessCommand(const char *cmd, IR_LED_CLI_PrintFn print_fn)
{
    char line[120];

    if (strncmp(cmd, "ir", 2) != 0) return false;
    if (cmd[2] != '\0' && cmd[2] != ' ') return false;   /* not "irrelevant" */

    const char *rest = (cmd[2] == '\0') ? "" : (cmd + 3);

    if (rest[0] == '\0' || strcmp(rest, "help") == 0) { print_help(print_fn);  return true; }
    if (strcmp(rest, "status") == 0)                  { print_status(print_fn); return true; }
    if (strcmp(rest, "limits") == 0)                  { print_limits(print_fn); return true; }
    if (strcmp(rest, "dac") == 0)                     { print_dac(print_fn);    return true; }

    if (strcmp(rest, "sync") == 0) {
        snprintf(line, sizeof(line), "SN0_VS frames=%lu   SN0_HS lines this frame=%lu\r\n",
                 (unsigned long)IR_LED_GetFrameCount(),
                 (unsigned long)IR_LED_GetLineCount());
        print_fn(line);
        print_fn("(Run twice: frames must climb, lines must wrap at the sensor's VMAX.)\r\n");
        return true;
    }

    if (strcmp(rest, "limit") == 0 || strncmp(rest, "limit ", 6) == 0) {
        if (rest[5] == ' ') {
            long ma = strtol(rest + 6, NULL, 10);
            if (ma < 0) { print_fn("ERROR: limit must be >= 0\r\n"); return true; }
            IR_LED_SetCurrentLimit_mA((uint32_t)ma);
        }
        snprintf(line, sizeof(line), "IR current limit = %lu mA (board ceiling %lu mA)\r\n",
                 (unsigned long)IR_LED_GetCurrentLimit_mA(),
                 (unsigned long)IR_LED_MAX_CURRENT_MA);
        print_fn(line);
        return true;
    }

    if (strcmp(rest, "enable") == 0) {
        IR_LED_Enable(true);
        print_fn("SUCCESS: output enabled, channels resume their configured modes\r\n");
        return true;
    }

    if (strcmp(rest, "disable") == 0) {
        IR_LED_Enable(false);
        print_fn("SUCCESS: output disabled, both gates forced off\r\n");
        return true;
    }

    if (strcmp(rest, "flash level") == 0) {
        uint32_t mv, edges, hi, lo; bool act;
        bool run = IR_LED_GetFlashSyncStatus(&mv, &act, &edges);
        IR_LED_GetFlashThresholds_mV(&hi, &lo);
        snprintf(line, sizeof(line),
                 "FLASH sync: %s  PC3=%lu mV  %s  edges=%lu  polarity=active %s  thresh %lu/%lu mV\r\n",
                 run ? "running" : "stopped", (unsigned long)mv,
                 act ? "ACTIVE" : "idle", (unsigned long)edges,
                 IR_LED_GetFlashActiveHigh() ? "high" : "low",
                 (unsigned long)hi, (unsigned long)lo);
        print_fn(line);
        return true;
    }
    if (strncmp(rest, "flash polarity ", 15) == 0) {
        const char *pol = rest + 15;
        if      (strcmp(pol, "high") == 0) IR_LED_SetFlashPolarity(true);
        else if (strcmp(pol, "low")  == 0) IR_LED_SetFlashPolarity(false);
        else { print_fn("ERROR: flash polarity high|low\r\n"); return true; }
        snprintf(line, sizeof(line), "SUCCESS: FLASH active %s\r\n", pol);
        print_fn(line);
        return true;
    }
    if (strncmp(rest, "flash thresh ", 13) == 0) {
        char *pp = (char *)(rest + 13);
        long hi = strtol(pp, &pp, 10);
        long lo = strtol(pp, NULL, 10);
        if (hi <= 0 || lo < 0 || !IR_LED_SetFlashThresholds_mV((uint32_t)hi, (uint32_t)lo)) {
            print_fn("ERROR: flash thresh <hi_mV> <lo_mV>, lo < hi < 3300\r\n");
            return true;
        }
        snprintf(line, sizeof(line), "SUCCESS: FLASH thresholds %ld / %ld mV\r\n", hi, lo);
        print_fn(line);
        return true;
    }
    if (strncmp(rest, "flash ", 6) == 0) {
        const char *who = rest + 6;
        IR_FlashSource_t src;
        if      (strcmp(who, "mcu")  == 0) src = IR_FLASH_SRC_MCU;
        else if (strcmp(who, "host") == 0) src = IR_FLASH_SRC_HOST;
        else { print_fn("ERROR: flash must be mcu|host\r\n"); return true; }

        if (!IR_LED_SetFlashSource(src)) {
            print_fn("ERROR: PC3 (FLASH) is an input only - the MCU never drives it\r\n");
            return true;
        }
        snprintf(line, sizeof(line), "SUCCESS: FLASH net driven by %s\r\n", who);
        print_fn(line);
        return true;
    }

    int ch_num;
    const char *a = parse_channel_prefix(rest, &ch_num);
    if (a != NULL) {
        IR_Channel_t ch = (IR_Channel_t)ch_num;

        if (strncmp(a, "mode ", 5) == 0) {
            IR_LedMode_t m;
            if (!parse_mode(a + 5, &m)) {
                print_fn("ERROR: mode must be off|cont|strobe|host (use 'dim' for PWM)\r\n");
                return true;
            }
            if (!IR_LED_SetMode(ch, m)) {
                print_fn("ERROR: 'host' mode needs 'ir flash host' first, otherwise\r\n");
                print_fn("       the gate follows our own FLASH output (held low)\r\n");
                return true;
            }
            snprintf(line, sizeof(line), "SUCCESS: ch%d mode = %s\r\n", ch_num, mode_name(m));
            print_fn(line);
            return true;
        }

        if (strncmp(a, "ma ", 3) == 0) {
            long ma = strtol(a + 3, NULL, 10);
            if (ma < 0) { print_fn("ERROR: milliamps must be >= 0\r\n"); return true; }
            if ((uint32_t)ma > IR_LED_GetCurrentLimit_mA()) {
                snprintf(line, sizeof(line),
                         "NOTE: clamped to the current limit of %lu mA (hard cap %lu mA)\r\n",
                         (unsigned long)IR_LED_GetCurrentLimit_mA(),
                         (unsigned long)IR_LED_HARD_LIMIT_MA);
                print_fn(line);
            }
            IR_LED_SetCurrent_mA(ch, (uint32_t)ma);
            snprintf(line, sizeof(line), "SUCCESS: ch%d = %lu mA (CTRL1 %lu mV, code %u)\r\n",
                     ch_num, (unsigned long)IR_LED_GetCurrent_mA(ch),
                     (unsigned long)IR_LED_GetCtrlVoltage_mV(ch),
                     IR_LED_GetCurrentRaw(ch));
            print_fn(line);
            return true;
        }

        if (strncmp(a, "ramp ", 5) == 0) {
            char *p = (char *)(a + 5);
            long target = strtol(p, &p, 10);
            long step   = strtol(p, &p, 10);   /* 0 / absent -> driver default */
            long dwell  = strtol(p, NULL, 10);

            if (target < 0 || step < 0 || dwell < 0) {
                print_fn("ERROR: ramp <mA> [step_mA] [dwell_ms], all >= 0\r\n");
                return true;
            }
            if (!IR_LED_IsEnabled()) {
                print_fn("ERROR: run 'ir enable' first - a ramp with the rail down\r\n");
                print_fn("       only moves CTRL1, it does not light anything\r\n");
                return true;
            }
            snprintf(line, sizeof(line), "Ramping ch%d %lu -> %ld mA ...\r\n",
                     ch_num, (unsigned long)IR_LED_GetCurrent_mA(ch), target);
            print_fn(line);

            bool ok = IR_LED_RampCurrent_mA(ch, (uint32_t)target,
                                            (uint32_t)step, (uint32_t)dwell);
            snprintf(line, sizeof(line), "%s: ch%d now %lu mA (CTRL1 %lu mV, code %u)\r\n",
                     ok ? "SUCCESS" : "ABORTED",
                     ch_num, (unsigned long)IR_LED_GetCurrent_mA(ch),
                     (unsigned long)IR_LED_GetCtrlVoltage_mV(ch),
                     IR_LED_GetCurrentRaw(ch));
            print_fn(line);
            if (!ok) {
                print_fn("       ramp stopped early: channel faulted or the rail dropped\r\n");
            }
            return true;
        }

        if (strncmp(a, "mv ", 3) == 0) {
            long mv = strtol(a + 3, NULL, 10);
            if (mv < 0) { print_fn("ERROR: millivolts must be >= 0\r\n"); return true; }
            if ((uint32_t)mv > IR_DAC_VREF_MV) {
                snprintf(line, sizeof(line),
                         "NOTE: clamped to the DAC full scale of %lu mV\r\n",
                         (unsigned long)IR_DAC_VREF_MV);
                print_fn(line);
            }
            uint32_t applied = IR_LED_SetCtrlVoltage_mV(ch, (uint32_t)mv);
            warn_if_over_abs_max(applied, print_fn);
            snprintf(line, sizeof(line),
                     "SUCCESS: ch%d DAC = %lu mV (code %u, ~%lu mA)\r\n",
                     ch_num, (unsigned long)applied, IR_LED_GetCurrentRaw(ch),
                     (unsigned long)IR_LED_GetCurrent_mA(ch));
            print_fn(line);
            if (applied > IR_CTRL_CEILING_MV) {
                snprintf(line, sizeof(line),
                         "         (above the %lu mV CTRL2 clamp - no extra current)\r\n",
                         (unsigned long)IR_CTRL_CEILING_MV);
                print_fn(line);
            }
            return true;
        }

        if (strncmp(a, "dim ", 4) == 0) {
            const char *arg = a + 4;
            if (strcmp(arg, "off") == 0) {
                IR_LED_StopGatePwm(ch);
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d dimming off, channel now mode=off\r\n", ch_num);
                print_fn(line);
                return true;
            }
            char *pp = (char *)arg;
            long hz   = strtol(pp, &pp, 10);
            long duty = strtol(pp, NULL, 10);
            if (hz <= 0 || duty < 0 || duty > 100) {
                print_fn("ERROR: dim <hz> <duty 0-100>, or 'dim off'\r\n");
                return true;
            }
            IR_LED_SetGatePwm(ch, (uint32_t)hz, (uint32_t)duty);
            uint32_t f, d;
            if (IR_LED_GetGatePwm(ch, &f, &d)) {
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d dimming %lu Hz, %lu%% optical duty\r\n",
                         ch_num, (unsigned long)f, (unsigned long)d);
            } else {
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d %s (duty %ld%% is a steady state)\r\n",
                         ch_num, (duty == 0) ? "off" : "fully on", duty);
            }
            print_fn(line);
            if (IR_LED_GetCurrentRaw(ch) == 0) {
                print_fn("NOTE: current setpoint is 0 - set one, e.g. 'ir ch0 ma 1000'\r\n");
            }
            return true;
        }

        if (strncmp(a, "fpulse ", 7) == 0) {
            const char *arg = a + 7;
            uint32_t width_us = 0U;
            if (strcmp(arg, "off") != 0) {
                /* <ms>[.<fraction>], e.g. 10 or 2.5 */
                char *pp = (char *)arg;
                long whole = strtol(pp, &pp, 10);
                uint32_t frac = 0U, scale = 100U;
                if (*pp == '.') {
                    pp++;
                    while (*pp >= '0' && *pp <= '9' && scale > 0U) {
                        frac += (uint32_t)(*pp - '0') * scale;
                        scale /= 10U;
                        pp++;
                    }
                }
                if (whole < 0 || (whole == 0 && frac == 0U)) {
                    print_fn("ERROR: fpulse <width_ms> (e.g. 10 or 2.5), or 'fpulse off'\r\n");
                    return true;
                }
                width_us = (uint32_t)whole * 1000U + frac;
            }
            IR_LED_SetFramePulse(ch, width_us);
            uint32_t fw;
            if (IR_LED_GetFramePulse(ch, &fw)) {
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d LED pulse %lu.%03lu ms at every frame start\r\n",
                         ch_num, (unsigned long)(fw / 1000U), (unsigned long)(fw % 1000U));
            } else {
                snprintf(line, sizeof(line), "SUCCESS: ch%d frame pulse off, mode=off\r\n", ch_num);
            }
            print_fn(line);
            return true;
        }

        if (strncmp(a, "pulse ", 6) == 0) {
            const char *arg = a + 6;
            if (strcmp(arg, "off") == 0) {
                IR_LED_StopGatePwm(ch);
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d pulse off, channel now mode=off\r\n", ch_num);
                print_fn(line);
                return true;
            }
            char *pp = (char *)arg;
            long on_ms  = strtol(pp, &pp, 10);
            long off_ms = strtol(pp, NULL, 10);
            if (on_ms < 0 || off_ms < 0 || (on_ms + off_ms) <= 0 ||
                !IR_LED_SetGatePulse(ch, (uint32_t)on_ms * 1000U, (uint32_t)off_ms * 1000U)) {
                snprintf(line, sizeof(line),
                         "ERROR: pulse <on_ms> <off_ms> (period %lu us..%lu ms), or 'pulse off'\r\n",
                         (unsigned long)IR_GATE_PERIOD_MIN_US,
                         (unsigned long)(IR_GATE_PERIOD_MAX_US / 1000U));
                print_fn(line);
                return true;
            }
            uint32_t on_us, off_us;
            if (IR_LED_GetGatePulse(ch, &on_us, &off_us)) {
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d LED on %lu.%03lu ms / off %lu.%03lu ms "
                         "(IR_CTRL%d high while on, PWM_DIM%d low)\r\n",
                         ch_num,
                         (unsigned long)(on_us / 1000U),  (unsigned long)(on_us % 1000U),
                         (unsigned long)(off_us / 1000U), (unsigned long)(off_us % 1000U),
                         ch_num, ch_num);
            } else {
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d %s (steady state)\r\n",
                         ch_num, (on_ms == 0) ? "off" : "fully on");
            }
            print_fn(line);
            if (IR_LED_GetCurrentRaw(ch) == 0) {
                snprintf(line, sizeof(line),
                         "NOTE: setpoint is 0 mA, but the LED still conducts (LT3763 offset) - "
                         "set one, e.g. 'ir ch%d ma 1000'\r\n", ch_num);
                print_fn(line);
            }
            return true;
        }

        if (strncmp(a, "pwm ", 4) == 0) {
            const char *arg = a + 4;
            if (strcmp(arg, "off") == 0) {
                IR_LED_StopDacPwm(ch);
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d PWM off, DAC back to %lu mV DC\r\n",
                         ch_num, (unsigned long)IR_LED_GetCtrlVoltage_mV(ch));
                print_fn(line);
                return true;
            }
            char *pp = (char *)arg;
            long hz   = strtol(pp, &pp, 10);
            long duty = strtol(pp, NULL, 10);
            if (hz <= 0 || duty < 0 || duty > 100) {
                print_fn("ERROR: pwm <hz> <duty 0-100>, or 'pwm off'\r\n");
                return true;
            }
            if (IR_LED_GetCurrentRaw(ch) == 0) {
                print_fn("NOTE: setpoint is 0 - PWM chops between 0V and 0V.\r\n");
                print_fn("      Set a level first, e.g. 'ir ch0 mv 180'.\r\n");
            }
            IR_LED_SetDacPwm(ch, (uint32_t)hz, (uint32_t)duty);
            uint32_t f, d;
            if (IR_LED_GetDacPwm(ch, &f, &d)) {
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d PWM %lu Hz %lu%% (0V <-> %lu mV)\r\n",
                         ch_num, (unsigned long)f, (unsigned long)d,
                         (unsigned long)IR_LED_GetCtrlVoltage_mV(ch));
            } else {
                snprintf(line, sizeof(line),
                         "SUCCESS: ch%d parked at %s (duty %ld%% is a steady state)\r\n",
                         ch_num, (duty == 0) ? "0 mV" : "the setpoint", duty);
            }
            print_fn(line);
            print_fn("NOTE: CTRL1 drives the LT3763 average-current loop, which\r\n");
            print_fn("      slews rather than switches - LED current will not be a\r\n");
            print_fn("      clean square wave, especially above a few hundred Hz.\r\n");
            return true;
        }

        if (strncmp(a, "raw ", 4) == 0) {
            long code = strtol(a + 4, NULL, 10);
            if (code < 0 || code > 4095) {
                print_fn("ERROR: raw code must be 0-4095\r\n");
                return true;
            }
            IR_LED_SetCurrentRaw(ch, (uint16_t)code);
            warn_if_over_abs_max(IR_LED_GetCtrlVoltage_mV(ch), print_fn);
            snprintf(line, sizeof(line), "SUCCESS: ch%d code %u -> CTRL1 %lu mV, %lu mA\r\n",
                     ch_num, IR_LED_GetCurrentRaw(ch), (unsigned long)IR_LED_GetCtrlVoltage_mV(ch),
                     (unsigned long)IR_LED_GetCurrent_mA(ch));
            print_fn(line);
            return true;
        }

        if (strncmp(a, "window ", 7) == 0) {
            char *p = (char *)(a + 7);
            long w_start = strtol(p, &p, 10);
            long w_end   = strtol(p, NULL, 10);
            if (w_start < 0 || w_end < 0 ||
                !IR_LED_SetWindow(ch, (uint32_t)w_start, (uint32_t)w_end)) {
                print_fn("ERROR: window needs 0 <= start < end\r\n");
                return true;
            }
            snprintf(line, sizeof(line), "SUCCESS: ch%d window = lines [%ld, %ld)\r\n",
                     ch_num, w_start, w_end);
            print_fn(line);
            return true;
        }

        if (strcmp(a, "clearfault") == 0) {
            IR_LED_ClearFault(ch);
            snprintf(line, sizeof(line),
                     "SUCCESS: ch%d fault cleared (a fault also disabled output: 'ir enable')\r\n",
                     ch_num);
            print_fn(line);
            return true;
        }
    }

    print_fn("ERROR: unrecognized ir command\r\n");
    print_help(print_fn);
    return true;
}
