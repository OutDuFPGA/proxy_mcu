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
    p("  ir status               - FLASH, VSYNC/HSYNC inputs, both channels\r\n");
    p("  ir [ch0|ch1] ma <mA>    - Set LED current via DAC_DIMx (default ch0)\r\n");
    p("  ir [ch0|ch1] mv <mV>    - Set DAC_DIMx voltage directly\r\n");
    p("  ir [ch0|ch1] strobe on|off - LED on while FLASH low, DAC 0 V while FLASH high\r\n");
    p("  ir [ch0|ch1] ramp <mA> <ms>  - Change current gradually to <mA> over <ms> (non-blocking)\r\n");
    p("  ir [ch0|ch1] dacpwm <hz> <duty%>|off - Chop DAC_DIMx between 0 V and the setpoint\r\n");
    p("  ir [ch0|ch1] dactest <0..1500 mV>|off - DAC test, BYPASSES the 4 A cap (unplug J5/J6!)\r\n");
    p("  ir [ch0|ch1] hpulse <mA> <us> [n] - HIGH-CURRENT pulse (<=20 A), one per 30 Hz frame,\r\n");
    p("                            n pulses (default 0 = continuous); width clamped to USHIO derating\r\n");
    p("  ir [ch0|ch1] hpulse off - stop high-current pulses (DAC 0 V)\r\n");
    p("  ir hpulse table         - max pulse width per current at 30 Hz\r\n");
    p("  ir [ch0|ch1] hsync <mA> <line> <lines> [n] - DAC_DIMx + PWM_DIMx follow HSYNC HIGH for <lines> lines\r\n");
    p("                            from <line> after VSYNC, at <mA> (<=20 A); window held to the\r\n");
    p("                            USHIO derating (TIM12 cut-off), n frames (0 = continuous)\r\n");
    p("  ir [ch0|ch1] hsync <mA> <line> <lines> [n] inv - LED on (DAC up, PWM_DIMx LOW) during each\r\n");
    p("               ~1.7 us HSYNC LOW in the window instead of HSYNC HIGH\r\n");
    p("  ir [ch0|ch1] hsync on|off - repeat last hsync settings (default 20000 100 21) / stop\r\n");
    p("  ir [ch0|ch1] fhsync <mA> <line> <lines> [n] - as hsync, but PC3 (FLASH) is DRIVEN HIGH while\r\n");
    p("               HSYNC is HIGH (PWM_DIMx stays HIGH) - sensor FLASH output must be disabled\r\n");
    p("  ir [ch0|ch1] fhsync <mA> <line> <lines> [n] inv - PC3 = inverted HSYNC: HIGH (LED on) through\r\n");
    p("               each ~1.7 us HSYNC LOW in the window, DAC_DIMx up only then\r\n");
    p("  ir [ch0|ch1] fhsync on|off - repeat last fhsync settings / stop (PC3 back to input)\r\n");
    p("  ir [ch0|ch1] vsync <mA> [n] - LED on at <mA> (<=20 A) while VSYNC is LOW, cut off at\r\n");
    p("                            the USHIO derating width; n pulses (0 = continuous)\r\n");
    p("  ir [ch0|ch1] vsync <mA> <line> <lines> [n] - one solid pulse per frame from <line> after\r\n");
    p("                            VSYNC for <lines> lines (clamped to the USHIO derating)\r\n");
    p("  ir [ch0|ch1] vsync on|off - repeat last vsync settings (default 20000) / stop\r\n");
    p("  ir cut day|night        - pulse the IR-cut filter in (day) / out (night): PA11 EN, PA12 PH\r\n");
    p("  ir cut ms <10..1000>    - IR-cut pulse length (default 150 ms)\r\n");
    p("  ir cut swap             - swap PH polarity if day/night come out reversed\r\n");
    p("  ir [ch0|ch1] pwm high   - PWM_DIMx fixed HIGH (LED follows FLASH only)\r\n");
    p("  ir [ch0|ch1] pwm low    - PWM_DIMx fixed LOW (LED forced on)\r\n");
    p("  ir [ch0|ch1] pwm <hz> <duty%> - PWM on PWM_DIMx, duty = LED-on share (10..20000 Hz, shared freq)\r\n");
    p("  ir [ch0|ch1] gate on|off - PWM_DIMx LOW while FLASH is low (exposure) / fixed HIGH\r\n");
    p("  ir [ch0|ch1] gate inv   - PWM_DIMx HIGH while FLASH is low, LOW while high (inverted)\r\n");
    p("  Only one channel may be gated, and not while the other is held LOW.\r\n");
    p("  ch0 = J5: DAC_DIM0 PA4, PWM_DIM0 PB14, FAULT0 PB9\r\n");
    p("  ch1 = J6: DAC_DIM1 PA5, PWM_DIM1 PB15, FAULT1 PE0\r\n");
    p("  Note: while FLASH is high the LED is on regardless (Q2/Q3 pass FLASH).\r\n");
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

#if IR_PC3_OUTPUT
    (void)fl;
    snprintf(l, sizeof(l), "Sensor IMX678: FLASH (PC3 out) driven %s\r\n",
             (FLASH_GPIO_Port->ODR & FLASH_Pin) ? "HIGH" : "LOW");
    p(l);
#else
    IR_LED_GetFlashInput(&fl);
    p("Sensor IMX568: PC3 = FLASH input\r\n");
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
#endif

    IR_SyncInput_t sy;
    IR_LED_GetSyncInput(&sy);
    if (sy.vs_active && sy.vs_period_us) {
        uint32_t mhz = 1000000000U / sy.vs_period_us;
        snprintf(l, sizeof(l), "SN0_VS (PC4 in):  %lu.%02lu Hz  period %lu us  count %lu\r\n",
                 (unsigned long)(mhz / 1000U), (unsigned long)((mhz % 1000U) / 10U),
                 (unsigned long)sy.vs_period_us, (unsigned long)sy.vs_count);
    } else {
        snprintf(l, sizeof(l), "SN0_VS (PC4 in):  NOT toggling (count %lu)\r\n", (unsigned long)sy.vs_count);
    }
    p(l);
    if (!sy.vs_active || !sy.vs_period_us || !sy.hs_period_ns) {
        uint32_t d[6];
        IR_LED_GetSyncDebug(d);
        snprintf(l, sizeof(l), "  dbg: tick %lu vs_ms %lu vs_cyc %lu hs_cyc %lu sysclk %lu cyccnt %lu\r\n",
                 (unsigned long)d[0], (unsigned long)d[1], (unsigned long)d[2], (unsigned long)d[3],
                 (unsigned long)d[4], (unsigned long)d[5]);
        p(l);
    }
    if (sy.hs_active && sy.hs_period_ns) {
        uint32_t hz = (uint32_t)(1000000000ULL / sy.hs_period_ns);
        snprintf(l, sizeof(l), "SN0_HS (PB1 in):  %lu.%03lu kHz  period %lu ns  lines/frame %lu\r\n",
                 (unsigned long)(hz / 1000U), (unsigned long)(hz % 1000U),
                 (unsigned long)sy.hs_period_ns, (unsigned long)sy.hs_per_frame);
    } else {
        snprintf(l, sizeof(l), "SN0_HS (PB1 in):  NOT toggling (count %lu)\r\n", (unsigned long)sy.hs_count);
    }
    p(l);

    {
        int last; uint32_t cms, nph; bool busy;
        IR_LED_IrCutStatus(&last, &cms, &nph, &busy);
        snprintf(l, sizeof(l), "IR-cut (PA11 EN / PA12 PH): last %s, pulse %lu ms, night PH=%lu%s\r\n",
                 last < 0 ? "none since boot" : (last ? "NIGHT (filter out)" : "DAY (filter in)"),
                 (unsigned long)cms, (unsigned long)nph, busy ? ", PULSING now" : "");
        p(l);
    }

    for (int i = 0; i < IR_CH_COUNT; i++) {
        IR_Channel_t ch = (IR_Channel_t)i;
        uint32_t gates;
        GPIO_TypeDef *gp = (i == 0) ? PWM_DIM0_GPIO_Port : PWM_DIM1_GPIO_Port;
        uint16_t      gn = (i == 0) ? PWM_DIM0_Pin : PWM_DIM1_Pin;
        const char   *pin_pwm = (i == 0) ? "PB14" : "PB15";

        snprintf(l, sizeof(l), "CH%d %s\r\n", i, (i == 0) ? "(J5)" : "(J6)");
        p(l);
        IR_PwmState_t ps = IR_LED_GetPwmState(ch);
        (void)IR_LED_GetFlashGate(ch, &gates);
        if (ps == IR_PWM_GATED && IR_LED_GetStrobe(ch)) {
            snprintf(l, sizeof(l), "  STROBE: on during FLASH low (PWM_DIM%d LOW + DAC setpoint), DAC 0 V while high, now %s\r\n", i,
                     HAL_GPIO_ReadPin(gp, gn) == GPIO_PIN_SET ? "off" : "ON");
        } else if (ps == IR_PWM_GATED && IR_LED_GetFlashGateInverted(ch)) {
            snprintf(l, sizeof(l), "  PWM_DIM%d (%s): gated by FLASH INVERTED (HIGH during FLASH low), now %s, gates %lu\r\n", i, pin_pwm,
                     HAL_GPIO_ReadPin(gp, gn) == GPIO_PIN_SET ? "HIGH" : "LOW", (unsigned long)gates);
        } else if (ps == IR_PWM_GATED) {
            snprintf(l, sizeof(l), "  PWM_DIM%d (%s): gated by FLASH, now %s, gates %lu\r\n", i, pin_pwm,
                     HAL_GPIO_ReadPin(gp, gn) == GPIO_PIN_SET ? "HIGH" : "LOW", (unsigned long)gates);
        } else if (ps == IR_PWM_DUTY) {
            uint32_t hz, duty;
            (void)IR_LED_GetGatePwm(ch, &hz, &duty);
            snprintf(l, sizeof(l), "  PWM_DIM%d (%s): PWM %lu Hz, duty %lu%% (LED on share)\r\n", i, pin_pwm,
                     (unsigned long)hz, (unsigned long)duty);
        } else if (ps == IR_PWM_LOW) {
            snprintf(l, sizeof(l), "  PWM_DIM%d (%s): fixed LOW (LED forced on)\r\n", i, pin_pwm);
        } else {
            snprintf(l, sizeof(l), "  PWM_DIM%d (%s): fixed HIGH (LED follows FLASH only)\r\n", i, pin_pwm);
        }
        p(l);
        snprintf(l, sizeof(l), "  DAC_DIM%d (%s):  %lu mV (code %u) -> ~%lu mA   limit %lu mA\r\n", i,
                 (i == 0) ? "PA4" : "PA5",
                 (unsigned long)IR_LED_GetCtrlVoltage_mV(ch), IR_LED_GetCurrentRaw(ch),
                 (unsigned long)IR_LED_GetCurrent_mA(ch), (unsigned long)IR_LED_GetCurrentLimit_mA());
        p(l);
        uint32_t hma, hw, hleft, hfired, hskip;
        if (IR_LED_GetHighPulse(ch, &hma, &hw, &hleft, &hfired, &hskip)) {
            char cnt[24];
            if (hleft) snprintf(cnt, sizeof(cnt), "%lu left", (unsigned long)hleft);
            else       snprintf(cnt, sizeof(cnt), "continuous");
            snprintf(l, sizeof(l), "  HPULSE: %lu mA x %lu us per frame, %s, fired %lu, skipped %lu\r\n",
                     (unsigned long)hma, (unsigned long)hw, cnt, (unsigned long)hfired, (unsigned long)hskip);
            p(l);
        }
        IR_HsyncPulseInfo_t hs;
        if (IR_LED_GetHsyncPulse(ch, &hs) && hs.vs_mode) {
            char cnt[24];
            if (hs.left) snprintf(cnt, sizeof(cnt), "%lu left", (unsigned long)hs.left);
            else         snprintf(cnt, sizeof(cnt), "continuous");
            snprintf(l, sizeof(l), "  VSYNC PULSE: %lu mA while VSYNC LOW, cut-off %lu us, %s\r\n",
                     (unsigned long)hs.ma, (unsigned long)hs.window_us, cnt);
            p(l);
            snprintf(l, sizeof(l), "               fired %lu, timer cut-offs %lu, skipped %lu, VSYNC LOW %lu ns\r\n",
                     (unsigned long)hs.fired, (unsigned long)hs.timer_cuts, (unsigned long)hs.skipped,
                     (unsigned long)hs.vs_low_ns);
            p(l);
        } else if (IR_LED_GetHsyncPulse(ch, &hs)) {
            char cnt[24];
            if (hs.left) snprintf(cnt, sizeof(cnt), "%lu left", (unsigned long)hs.left);
            else         snprintf(cnt, sizeof(cnt), "continuous");
            snprintf(l, sizeof(l), "  %s: %lu mA, lines %lu..%lu, cut-off %lu us, %s\r\n",
                     hs.solid ? "VSYNC LINE PULSE (solid)" :
                     (hs.inv ? (hs.flash ? "HSYNC PULSE (PC3 = inverted HSYNC)" : "HSYNC PULSE (PWM_DIM on HSYNC LOW)")
                             : (hs.flash ? "HSYNC PULSE (FLASH/PC3 gate)" : "HSYNC PULSE")),
                     (unsigned long)hs.ma, (unsigned long)hs.start_line,
                     (unsigned long)(hs.start_line + hs.lines - 1U), (unsigned long)hs.window_us, cnt);
            p(l);
            if (hs.solid)
                snprintf(l, sizeof(l), "               fired %lu, timer cut-offs %lu\r\n",
                         (unsigned long)hs.fired, (unsigned long)hs.timer_cuts);
            else
                snprintf(l, sizeof(l), "               fired %lu, timer cut-offs %lu, HSYNC HIGH %lu ns\r\n",
                         (unsigned long)hs.fired, (unsigned long)hs.timer_cuts, (unsigned long)hs.hs_high_ns);
            p(l);
            uint32_t fx[12];
            if ((hs.flash || hs.inv) && IR_LED_GetFlashSyncStats(fx)) {
                snprintf(l, sizeof(l), hs.inv ? "  gate on after HSYNC fall %lu/%lu/%lu ns, gate off after HSYNC rise %lu/%lu/%lu ns\r\n"
                                              : "  PC3 vs HSYNC (from IRQ entry): rise %lu/%lu/%lu ns, fall %lu/%lu/%lu ns (min/avg/max)\r\n",
                         (unsigned long)fx[0], (unsigned long)fx[1], (unsigned long)fx[2],
                         (unsigned long)fx[3], (unsigned long)fx[4], (unsigned long)fx[5]);
                p(l);
                snprintf(l, sizeof(l), "               gate on %lu ns, pulses/window %lu (min %lu max %lu), rises %lu, late %lu\r\n",
                         (unsigned long)fx[6], (unsigned long)fx[7], (unsigned long)fx[8],
                         (unsigned long)fx[9], (unsigned long)fx[10], (unsigned long)fx[11]);
                p(l);
            }
        }
        uint32_t rto, rleft, dhz, dduty, tmv;
        if (IR_LED_GetDacTest(ch, &tmv)) {
            snprintf(l, sizeof(l), "  DAC TEST: DAC_DIM%d = %lu mV - current cap BYPASSED ('ir ch%d dactest off')\r\n",
                     i, (unsigned long)tmv, i);
            p(l);
        }
        if (IR_LED_GetRamp(ch, &rto, &rleft)) {
            snprintf(l, sizeof(l), "  ramp: to %lu mA, %lu ms left\r\n", (unsigned long)rto, (unsigned long)rleft);
            p(l);
        }
        if (IR_LED_GetDacPwm(ch, &dhz, &dduty)) {
            snprintf(l, sizeof(l), "  DAC PWM: %lu Hz, %lu%% at setpoint (rest at 0 V)\r\n", (unsigned long)dhz, (unsigned long)dduty);
            p(l);
        }
        snprintf(l, sizeof(l), "  FAULT%d (%s):     %s\r\n", i, (i == 0) ? "PB9" : "PE0",
                 IR_LED_IsFaultLineAsserted(ch) ? "low" : "high");
        p(l);
    }
}

bool IR_LED_CLI_ProcessCommand(const char *cmd, IR_LED_CLI_PrintFn print_fn)
{
    char line[120];

    if (strncmp(cmd, "ir", 2) != 0) return false;
    if (cmd[2] != '\0' && cmd[2] != ' ') return false;   /* not "irrelevant" */

    const char *rest = (cmd[2] == '\0') ? "" : (cmd + 3);

    if (rest[0] == '\0' || strcmp(rest, "help") == 0) { print_help(print_fn);   return true; }
    if (strcmp(rest, "status") == 0)                  { print_status(print_fn); return true; }
    if (strcmp(rest, "hsmeas") == 0) {
        uint32_t d[7];
        if (!IR_LED_MeasureHsync(d)) { print_fn("ERROR: stop hsync/vsync/fhsync first\r\n"); return true; }
        snprintf(line, sizeof(line), "SN0_HS polled 1 ms (IRQs off): HIGH %lu/%lu/%lu ns, LOW %lu/%lu/%lu ns (min/avg/max), %lu periods\r\n",
                 (unsigned long)d[0], (unsigned long)d[1], (unsigned long)d[2],
                 (unsigned long)d[3], (unsigned long)d[4], (unsigned long)d[5], (unsigned long)d[6]);
        print_fn(line); return true;
    }

    if (strcmp(rest, "cut day") == 0 || strcmp(rest, "cut night") == 0) {
        bool night = (rest[4] == 'n');
        if (!IR_LED_IrCut(night)) { print_fn("ERROR: IR-cut pulse already running, try again\r\n"); return true; }
        int last; uint32_t cms, nph; bool busy;
        IR_LED_IrCutStatus(&last, &cms, &nph, &busy);
        snprintf(line, sizeof(line), "SUCCESS: IR-cut %s - EN pulsed %lu ms, PH=%lu\r\n",
                 night ? "NIGHT (filter out)" : "DAY (filter in)", (unsigned long)cms,
                 (unsigned long)(night ? nph : 1U - nph));
        print_fn(line);
        return true;
    }
    if (strncmp(rest, "cut ms ", 7) == 0) {
        long ms = strtol(rest + 7, NULL, 10);
        if (ms < (long)IR_CUT_PULSE_MIN_MS || ms > (long)IR_CUT_PULSE_MAX_MS) {
            snprintf(line, sizeof(line), "ERROR: IR-cut pulse must be %lu..%lu ms\r\n",
                     (unsigned long)IR_CUT_PULSE_MIN_MS, (unsigned long)IR_CUT_PULSE_MAX_MS);
            print_fn(line); return true;
        }
        snprintf(line, sizeof(line), "SUCCESS: IR-cut pulse %lu ms\r\n", (unsigned long)IR_LED_IrCutSetPulse_ms((uint32_t)ms));
        print_fn(line);
        return true;
    }
    if (strcmp(rest, "cut swap") == 0) {
        IR_LED_IrCutSwap();
        uint32_t nph; IR_LED_IrCutStatus(NULL, NULL, &nph, NULL);
        snprintf(line, sizeof(line), "SUCCESS: IR-cut polarity swapped - night now PH=%lu, day PH=%lu\r\n",
                 (unsigned long)nph, (unsigned long)(1U - nph));
        print_fn(line);
        return true;
    }

    if (strcmp(rest, "hpulse table") == 0) {
        static const uint32_t tbl[] = { 4000U, 5000U, 6000U, 8000U, 10000U, 12000U, 15000U, 18000U, 20000U };
        snprintf(line, sizeof(line), "USHIO SMBB850DS-1200 max pulse width at %lu Hz (derate %lu%%):\r\n",
                 (unsigned long)IR_FRAME_PULSE_FPS, (unsigned long)IR_HP_DERATE_PCT);
        print_fn(line);
        for (unsigned i = 0; i < sizeof(tbl) / sizeof(tbl[0]); i++) {
            uint32_t w = IR_LED_HighPulseMaxWidth_us(tbl[i]);
            snprintf(line, sizeof(line), "  %5lu mA : %5lu us  (%lu.%02lu%% duty)\r\n",
                     (unsigned long)tbl[i], (unsigned long)w,
                     (unsigned long)(w * IR_FRAME_PULSE_FPS / 10000U),
                     (unsigned long)((w * IR_FRAME_PULSE_FPS / 100U) % 100U));
            print_fn(line);
        }
        return true;
    }

    /* Optional channel prefix; without one, commands address CH0. */
    IR_Channel_t ch = IR_CH0;
    if (strncmp(rest, "ch0 ", 4) == 0)      { rest += 4; }
    else if (strncmp(rest, "ch1 ", 4) == 0) { ch = IR_CH1; rest += 4; }
    int n = (int)ch;

    /* "hsync on" / "vsync on" repeat the last settings (20 A by default). */
    static char hs_last[40] = "20000 100 21";
    static char vs_last[40] = "20000";
    static char fhs_last[40] = "20000 100 21";
    if (strcmp(rest, "fhsync on") == 0) {
        char re[64];
        snprintf(re, sizeof(re), "ir ch%d fhsync %s", n, fhs_last);
        snprintf(line, sizeof(line), "NOTE: '%s'\r\n", re);
        print_fn(line);
        return IR_LED_CLI_ProcessCommand(re, print_fn);
    }
    if (strcmp(rest, "hsync on") == 0 || strcmp(rest, "vsync on") == 0) {
        char re[64];
        bool hsr = (rest[0] == 'h');
        snprintf(re, sizeof(re), "ir ch%d %s %s", n, hsr ? "hsync" : "vsync", hsr ? hs_last : vs_last);
        snprintf(line, sizeof(line), "NOTE: '%s'\r\n", re);
        print_fn(line);
        return IR_LED_CLI_ProcessCommand(re, print_fn);
    }

    if (strncmp(rest, "hsync ", 6) == 0 || strncmp(rest, "fhsync ", 7) == 0) {
        bool fl = (rest[0] == 'f');
        const char *a = rest + (fl ? 7 : 6);
        if (strcmp(a, "off") == 0) {
            IR_LED_StopHsyncPulse(ch);
            snprintf(line, sizeof(line), "SUCCESS: ch%d HSYNC pulse off (DAC_DIM%d 0 V, PWM_DIM%d HIGH%s)\r\n", n, n, n,
                     fl ? (IR_PC3_OUTPUT ? ", PC3 output LOW" : ", PC3 input") : "");
            print_fn(line); return true;
        }
        char *e1, *e2, *e3;
        long ma  = strtol(a, &e1, 10);
        long st  = strtol(e1, &e2, 10);
        long nl  = strtol(e2, &e3, 10);
        long cnt = strtol(e3, NULL, 10);
        if (e1 == a || e2 == e1 || e3 == e2 || ma <= 0 || st <= 0 || nl <= 0 || cnt < 0) {
            print_fn(fl ? "ERROR: usage 'ir [ch0|ch1] fhsync <mA> <start_line> <lines> [count]' or 'fhsync off'\r\n"
                        : "ERROR: usage 'ir [ch0|ch1] hsync <mA> <start_line> <lines> [count]' or 'hsync off'\r\n");
            return true;
        }
        if (ma > (long)IR_LED_PULSE_LIMIT_MA) {
            snprintf(line, sizeof(line), "NOTE: clamped to the %lu mA pulse limit\r\n", (unsigned long)IR_LED_PULSE_LIMIT_MA);
            print_fn(line);
        }
        bool inv = (strstr(a, "inv") != NULL);
        bool ok = fl ? IR_LED_SetHsyncFlashPulse(ch, (uint32_t)ma, (uint32_t)st, (uint32_t)nl, (uint32_t)cnt, inv)
                : inv ? IR_LED_SetHsyncLowPulse(ch, (uint32_t)ma, (uint32_t)st, (uint32_t)nl, (uint32_t)cnt)
                     : IR_LED_SetHsyncPulse(ch, (uint32_t)ma, (uint32_t)st, (uint32_t)nl, (uint32_t)cnt);
        if (!ok) {
            print_fn(fl ? "ERROR: HSYNC (SN0_HS, PB1) not toggling, derating budget shorter than 2 lines, or FLASH ADC sync running\r\n"
                        : "ERROR: HSYNC (SN0_HS, PB1) not toggling, or derating budget shorter than 2 lines\r\n");
            return true;
        }
        snprintf(fl ? fhs_last : hs_last, sizeof(hs_last), "%ld %ld %ld %ld%s", ma, st, nl, cnt, inv ? " inv" : "");
        IR_HsyncPulseInfo_t hs;
        (void)IR_LED_GetHsyncPulse(ch, &hs);
        if (hs.lines < (uint32_t)nl) {
            snprintf(line, sizeof(line), "NOTE: lines clamped to %lu - USHIO derating at this current and %lu Hz\r\n",
                     (unsigned long)hs.lines, (unsigned long)IR_FRAME_PULSE_FPS);
            print_fn(line);
        }
        snprintf(line, sizeof(line), "SUCCESS: ch%d HSYNC pulse%s %lu mA, lines %lu..%lu, cut-off %lu us, %s\r\n", n,
                 inv ? (fl ? " (PC3 = inverted HSYNC)" : " (PWM_DIM on HSYNC LOW)") : (fl ? " (FLASH/PC3 gate)" : ""),
                 (unsigned long)hs.ma, (unsigned long)hs.start_line,
                 (unsigned long)(hs.start_line + hs.lines - 1U), (unsigned long)hs.window_us,
                 cnt ? "burst" : "continuous");
        print_fn(line);
        snprintf(line, sizeof(line), "NOTE: scope TP2 (ISMON, %lu mV at full current), %s and SN0_HS\r\n",
                 (unsigned long)(hs.ma / 20U), fl ? "PC3 (FLASH)" : (ch == IR_CH0 ? "PWM_DIM0" : "PWM_DIM1"));
        print_fn(line);
        return true;
    }

    if (strncmp(rest, "vsync ", 6) == 0) {
        const char *a = rest + 6;
        if (strcmp(a, "off") == 0) {
            IR_LED_StopHsyncPulse(ch);
            snprintf(line, sizeof(line), "SUCCESS: ch%d VSYNC pulse off (DAC_DIM%d 0 V, PWM_DIM%d HIGH)\r\n", n, n, n);
            print_fn(line); return true;
        }
        char *e1, *e2, *e3, *e4;
        long ma  = strtol(a, &e1, 10);
        long cnt = strtol(e1, &e2, 10);
        long v3  = strtol(e2, &e3, 10);
        long v4  = strtol(e3, &e4, 10);
        if (e3 != e2) {                       /* <mA> <line> <lines> [count]: solid line pulse */
            long st = cnt, nl = v3, lc = (e4 != e3) ? v4 : 0;
            if (e1 == a || ma <= 0 || st <= 0 || nl <= 0 || lc < 0) {
                print_fn("ERROR: usage 'ir [ch0|ch1] vsync <mA> <line> <lines> [count]'\r\n"); return true;
            }
            if (ma > (long)IR_LED_PULSE_LIMIT_MA) {
                snprintf(line, sizeof(line), "NOTE: clamped to the %lu mA pulse limit\r\n", (unsigned long)IR_LED_PULSE_LIMIT_MA);
                print_fn(line);
            }
            if (!IR_LED_SetVsyncLinePulse(ch, (uint32_t)ma, (uint32_t)st, (uint32_t)nl, (uint32_t)lc)) {
                print_fn("ERROR: HSYNC (SN0_HS, PB1) not toggling, or derating budget shorter than 2 lines\r\n");
                return true;
            }
            snprintf(vs_last, sizeof(vs_last), "%ld %ld %ld %ld", ma, st, nl, lc);
            IR_HsyncPulseInfo_t hs;
            (void)IR_LED_GetHsyncPulse(ch, &hs);
            if (hs.lines < (uint32_t)nl) {
                snprintf(line, sizeof(line), "NOTE: lines clamped to %lu - USHIO derating at this current and %lu Hz\r\n",
                         (unsigned long)hs.lines, (unsigned long)IR_FRAME_PULSE_FPS);
                print_fn(line);
            }
            snprintf(line, sizeof(line), "SUCCESS: ch%d VSYNC line pulse %lu mA, solid lines %lu..%lu, cut-off %lu us, %s\r\n", n,
                     (unsigned long)hs.ma, (unsigned long)hs.start_line,
                     (unsigned long)(hs.start_line + hs.lines - 1U), (unsigned long)hs.window_us,
                     lc ? "burst" : "continuous");
            print_fn(line);
            snprintf(line, sizeof(line), "NOTE: scope TP2 (ISMON, %lu mV at full current), PWM_DIM%d and SN0_VS\r\n",
                     (unsigned long)(hs.ma / 20U), n);
            print_fn(line);
            return true;
        }
        if (e1 == a || ma <= 0 || cnt < 0) {
            print_fn("ERROR: usage 'ir [ch0|ch1] vsync <mA> [count]' or 'vsync <mA> <line> <lines> [count]'\r\n"); return true;
        }
        if (ma > (long)IR_LED_PULSE_LIMIT_MA) {
            snprintf(line, sizeof(line), "NOTE: clamped to the %lu mA pulse limit\r\n", (unsigned long)IR_LED_PULSE_LIMIT_MA);
            print_fn(line);
        }
        if (!IR_LED_SetVsyncPulse(ch, (uint32_t)ma, (uint32_t)cnt)) {
            print_fn("ERROR: VSYNC (SN0_VS, PC4) not toggling\r\n"); return true;
        }
        snprintf(vs_last, sizeof(vs_last), "%ld %ld", ma, cnt);
        IR_HsyncPulseInfo_t hs;
        (void)IR_LED_GetHsyncPulse(ch, &hs);
        snprintf(line, sizeof(line), "SUCCESS: ch%d VSYNC pulse %lu mA while VSYNC LOW, cut-off %lu us, %s\r\n", n,
                 (unsigned long)hs.ma, (unsigned long)hs.window_us, cnt ? "burst" : "continuous");
        print_fn(line);
        snprintf(line, sizeof(line), "NOTE: scope TP2 (ISMON, %lu mV at full current), PWM_DIM%d and SN0_VS\r\n",
                 (unsigned long)(hs.ma / 20U), n);
        print_fn(line);
        return true;
    }

    /* Any other per-channel command takes the channel back from hsync. */
    IR_LED_StopHsyncPulse(ch);

    if (strncmp(rest, "hpulse ", 7) == 0) {
        const char *a = rest + 7;
        if (strcmp(a, "off") == 0) {
            IR_LED_StopHighPulse(ch);
            snprintf(line, sizeof(line), "SUCCESS: ch%d high-current pulse off (DAC_DIM%d 0 V, PWM_DIM%d HIGH)\r\n", n, n, n);
            print_fn(line); return true;
        }
        char *e1, *e2;
        long ma = strtol(a, &e1, 10);
        long us = strtol(e1, &e2, 10);
        long cnt = strtol(e2, NULL, 10);
        if (e1 == a || e2 == e1 || ma <= 0 || us <= 0 || cnt < 0) {
            print_fn("ERROR: usage 'ir [ch0|ch1] hpulse <mA> <us> [count]' or 'hpulse off'\r\n"); return true;
        }
        if (ma > (long)IR_LED_PULSE_LIMIT_MA) {
            snprintf(line, sizeof(line), "NOTE: clamped to the %lu mA pulse limit\r\n", (unsigned long)IR_LED_PULSE_LIMIT_MA);
            print_fn(line);
        }
        uint32_t maxw = IR_LED_HighPulseMaxWidth_us((uint32_t)ma);
        if ((uint32_t)us > maxw) {
            snprintf(line, sizeof(line), "NOTE: width clamped to %lu us - USHIO derating at this current and %lu Hz\r\n",
                     (unsigned long)maxw, (unsigned long)IR_FRAME_PULSE_FPS);
            print_fn(line);
        }
        if (!IR_LED_SetHighPulse(ch, (uint32_t)ma, (uint32_t)us, (uint32_t)cnt)) {
            print_fn("ERROR: could not start high-current pulse\r\n"); return true;
        }
        uint32_t gma, gw;
        (void)IR_LED_GetHighPulse(ch, &gma, &gw, NULL, NULL, NULL);
        snprintf(line, sizeof(line), "SUCCESS: ch%d hpulse %lu mA x %lu us at %lu Hz, %s\r\n", n,
                 (unsigned long)gma, (unsigned long)gw, (unsigned long)IR_FRAME_PULSE_FPS,
                 cnt ? "burst" : "continuous");
        print_fn(line);
        snprintf(line, sizeof(line), "NOTE: scope TP2 (ISMON, 50 mV/A -> %lu mV) and PWM_DIM%d; DAC 0 V between pulses\r\n",
                 (unsigned long)(gma / 20U), n);
        print_fn(line);
        return true;
    }

    if (strncmp(rest, "ramp ", 5) == 0) {
        char *end;
        long ma = strtol(rest + 5, &end, 10);
        long ms = strtol(end, NULL, 10);
        if (end == rest + 5 || ma < 0 || ms < 0 || ms > 600000) {
            print_fn("ERROR: usage 'ir [ch0|ch1] ramp <mA> <0..600000 ms>'\r\n"); return true;
        }
        if ((uint32_t)ma > IR_LED_GetCurrentLimit_mA()) {
            snprintf(line, sizeof(line), "NOTE: target clamped to the current limit of %lu mA\r\n",
                     (unsigned long)IR_LED_GetCurrentLimit_mA());
            print_fn(line);
        }
        uint32_t from = IR_LED_GetCurrent_mA(ch);
        IR_LED_StartRamp_mA(ch, (uint32_t)ma, (uint32_t)ms);
        snprintf(line, sizeof(line), "SUCCESS: ch%d ramp %lu -> %lu mA over %ld ms\r\n", n,
                 (unsigned long)from, (unsigned long)((uint32_t)ma > IR_LED_GetCurrentLimit_mA() ? IR_LED_GetCurrentLimit_mA() : (uint32_t)ma), ms);
        print_fn(line);
        return true;
    }

#if IR_PC3_OUTPUT
    if (strcmp(rest, "strobe on") == 0 || strcmp(rest, "gate on") == 0 || strcmp(rest, "gate inv") == 0) {
        print_fn("ERROR: IMX678 build - PC3 (FLASH) is an MCU output, there is no FLASH input to gate/strobe on\r\n");
        return true;
    }
#endif
    if (strcmp(rest, "strobe on") == 0 || strcmp(rest, "strobe off") == 0) {
        bool on = (rest[8] == 'n');
        if (!IR_LED_SetStrobe(ch, on)) {
            snprintf(line, sizeof(line), "ERROR: ch%d is gated or held LOW/PWM - ch%d cannot strobe as well\r\n", 1 - n, n);
            print_fn(line); return true;
        }
        snprintf(line, sizeof(line), on ? "SUCCESS: ch%d strobe ON - lit during FLASH low, DAC 0 V while FLASH high\r\n"
                                        : "SUCCESS: ch%d strobe off - PWM_DIM HIGH, DAC at setpoint\r\n", n);
        print_fn(line);
        if (on) print_fn("NOTE: outside the exposure the LT3763 stays enabled at 0 V CTRL1 (offset current, dim not dark)\r\n");
        return true;
    }

    if (strncmp(rest, "dactest ", 8) == 0) {
        const char *a = rest + 8;
        if (strcmp(a, "off") == 0) {
            IR_LED_ClearDacTest(ch);
            snprintf(line, sizeof(line), "SUCCESS: ch%d DAC test off, DAC_DIM%d back to %lu mV setpoint (cap applies)\r\n",
                     n, n, (unsigned long)IR_LED_GetCtrlVoltage_mV(ch));
            print_fn(line); return true;
        }
        char *end;
        long mv = strtol(a, &end, 10);
        if (end == a || mv < 0 || mv > (long)IR_DACTEST_MAX_MV) {
            snprintf(line, sizeof(line), "ERROR: usage 'ir [ch0|ch1] dactest <0..%lu mV>' or 'dactest off'\r\n",
                     (unsigned long)IR_DACTEST_MAX_MV);
            print_fn(line); return true;
        }
        uint32_t got = IR_LED_SetDacTest_mV(ch, (uint32_t)mv);
        snprintf(line, sizeof(line), "SUCCESS: ch%d DAC TEST - DAC_DIM%d = %lu mV, current cap BYPASSED\r\n", n, n, (unsigned long)got);
        print_fn(line);
        print_fn("WARNING: for J5/J6 UNPLUGGED only. R16 is 0R now - NO CTRL2 ceiling: on a driver,\r\n");
        print_fn("         >294 mV is over the LED's 4 A DC max, 1500 mV is ~20 A DC. Strobe/ramp/dacpwm stopped.\r\n");
        print_fn("         'dactest off', ma, mv or ramp restores the capped setpoint.\r\n");
        return true;
    }

    if (strncmp(rest, "dacpwm ", 7) == 0) {
        const char *a = rest + 7;
        if (IR_LED_GetStrobe(ch)) (void)IR_LED_SetStrobe(ch, false);
        if (strcmp(a, "off") == 0) {
            IR_LED_SetDacPwm(ch, IR_DAC_PWM_MIN_HZ, 100U);    /* steady at the setpoint */
            snprintf(line, sizeof(line), "SUCCESS: ch%d DAC PWM off (steady at setpoint)\r\n", n);
            print_fn(line); return true;
        }
        char *end;
        long hz = strtol(a, &end, 10);
        long duty = strtol(end, NULL, 10);
        if (end == a || hz < (long)IR_DAC_PWM_MIN_HZ || hz > (long)IR_DAC_PWM_MAX_HZ || duty < 0 || duty > 100) {
            snprintf(line, sizeof(line), "ERROR: usage 'ir [ch0|ch1] dacpwm <%lu..%lu Hz> <0..100 %%>' or 'dacpwm off'\r\n",
                     (unsigned long)IR_DAC_PWM_MIN_HZ, (unsigned long)IR_DAC_PWM_MAX_HZ);
            print_fn(line); return true;
        }
        IR_LED_SetDacPwm(ch, (uint32_t)hz, (uint32_t)duty);
        snprintf(line, sizeof(line), "SUCCESS: ch%d DAC PWM %ld Hz, %ld%% at setpoint (rest at 0 V)\r\n", n, hz, duty);
        print_fn(line);
        if (duty < 100) print_fn("NOTE: 0 V on CTRL1 is not dark - the LT3763 offset can still pass up to ~1.2 A\r\n");
        return true;
    }

    if (strncmp(rest, "ma ", 3) == 0) {
        IR_LED_StopRamp(ch);
        long ma = strtol(rest + 3, NULL, 10);
        if (ma < 0) { print_fn("ERROR: milliamps must be >= 0\r\n"); return true; }
        if ((uint32_t)ma > IR_LED_GetCurrentLimit_mA()) {
            snprintf(line, sizeof(line), "NOTE: clamped to the current limit of %lu mA\r\n",
                     (unsigned long)IR_LED_GetCurrentLimit_mA());
            print_fn(line);
        }
        IR_LED_SetCurrent_mA(ch, (uint32_t)ma);
        snprintf(line, sizeof(line), "SUCCESS: ch%d %lu mA (DAC_DIM%d %lu mV, code %u)\r\n", n,
                 (unsigned long)IR_LED_GetCurrent_mA(ch), n,
                 (unsigned long)IR_LED_GetCtrlVoltage_mV(ch), IR_LED_GetCurrentRaw(ch));
        print_fn(line);
        return true;
    }

    if (strncmp(rest, "mv ", 3) == 0) {
        IR_LED_StopRamp(ch);
        long mv = strtol(rest + 3, NULL, 10);
        if (mv < 0) { print_fn("ERROR: millivolts must be >= 0\r\n"); return true; }
        uint32_t applied = IR_LED_SetCtrlVoltage_mV(ch, (uint32_t)mv);
        warn_if_over_abs_max(applied, print_fn);
        snprintf(line, sizeof(line), "SUCCESS: ch%d DAC_DIM%d = %lu mV (code %u, ~%lu mA)\r\n", n, n,
                 (unsigned long)applied, IR_LED_GetCurrentRaw(ch),
                 (unsigned long)IR_LED_GetCurrent_mA(ch));
        print_fn(line);
        return true;
    }

    if (strcmp(rest, "gate inv") == 0) {
        if (!IR_LED_SetFlashGateInverted(ch)) {
            snprintf(line, sizeof(line), "ERROR: ch%d is gated or held LOW/PWM - ch%d cannot be gated as well\r\n", 1 - n, n);
            print_fn(line); return true;
        }
        snprintf(line, sizeof(line), "SUCCESS: PWM_DIM%d gated by FLASH INVERTED (HIGH during FLASH low, LOW while high)\r\n", n);
        print_fn(line);
        print_fn("NOTE: via Q2/Q3 this puts the LED OFF during the exposure and ON outside it\r\n");
        return true;
    }

    if (strcmp(rest, "gate on") == 0 || strcmp(rest, "gate off") == 0) {
        bool on = (rest[6] == 'n');
        if (!IR_LED_SetFlashGate(ch, on)) {
            snprintf(line, sizeof(line), "ERROR: ch%d is gated or held LOW - ch%d cannot be gated as well\r\n", 1 - n, n);
            print_fn(line);
            print_fn("       (FLASH is pulled high only through a channel whose PWM_DIM is HIGH; otherwise it latches low)\r\n");
            return true;
        }
        snprintf(line, sizeof(line), on ? "SUCCESS: PWM_DIM%d gated by FLASH (LOW during exposure)\r\n"
                                        : "SUCCESS: PWM_DIM%d fixed HIGH (LED follows FLASH only)\r\n", n);
        print_fn(line);
        return true;
    }

    if (strncmp(rest, "pwm ", 4) == 0 && rest[4] >= '0' && rest[4] <= '9') {
        char *end;
        long hz = strtol(rest + 4, &end, 10);
        long duty = strtol(end, NULL, 10);
        if (hz < (long)IR_GATE_PWM_MIN_HZ || hz > (long)IR_GATE_PWM_MAX_HZ || duty < 0 || duty > 100 || end == rest + 4) {
            snprintf(line, sizeof(line), "ERROR: usage 'ir [ch0|ch1] pwm <%lu..%lu Hz> <0..100 %%>'\r\n",
                     (unsigned long)IR_GATE_PWM_MIN_HZ, (unsigned long)IR_GATE_PWM_MAX_HZ);
            print_fn(line); return true;
        }
        if (!IR_LED_SetPwmDuty(ch, (uint32_t)hz, (uint32_t)duty)) {
            snprintf(line, sizeof(line), "ERROR: ch%d is gated - turn its gate off before PWM on ch%d\r\n", 1 - n, n);
            print_fn(line); return true;
        }
        if (duty == 0 || duty == 100) {
            snprintf(line, sizeof(line), "SUCCESS: PWM_DIM%d fixed %s\r\n", n, duty ? "LOW (LED forced on)" : "HIGH");
        } else {
            uint32_t ahz, aduty; (void)IR_LED_GetGatePwm(ch, &ahz, &aduty);
            snprintf(line, sizeof(line), "SUCCESS: PWM_DIM%d PWM %lu Hz, duty %lu%% LED-on\r\n", n,
                     (unsigned long)ahz, (unsigned long)aduty);
        }
        print_fn(line);
        return true;
    }

    if (strcmp(rest, "pwm high") == 0 || strcmp(rest, "pwm low") == 0) {
        bool high = (rest[4] == 'h');
        if (!IR_LED_SetPwmLevel(ch, high)) {
            snprintf(line, sizeof(line), "ERROR: ch%d is gated - ch%d cannot be held LOW at the same time\r\n", 1 - n, n);
            print_fn(line);
            print_fn("       (the gate needs a channel with PWM_DIM HIGH to pull FLASH up; turn the gate off first)\r\n");
            return true;
        }
        snprintf(line, sizeof(line), high ? "SUCCESS: PWM_DIM%d fixed HIGH (LED follows FLASH only)\r\n"
                                          : "SUCCESS: PWM_DIM%d fixed LOW (LED forced on)\r\n", n);
        print_fn(line);
        return true;
    }

    print_fn("ERROR: unknown ir command - see 'ir help'\r\n");
    return true;
}
