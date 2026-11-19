/**
  ******************************************************************************
  * @file    lens_ctrl.c
  * @brief   Zoom / focus stepper control - see lens_ctrl.h.
  *
  * Layers, bottom up:
  *   1. TMC2300 UART   tmc_write / tmc_read / drv_config
  *   2. Step engine    one timer per axis, one STEP pulse per timer period,
  *                     trapezoidal ramp recomputed in the update interrupt
  *   3. Motion API     LENS_MoveTo / MoveRel / Jog / Stop / Halt / home / zero
  *   4. Front ends     console (LENS_CLI) and SoC line protocol (LENS_HostCommand)
  *
  * TMC2300 facts this relies on (docs/ and memory notes):
  *  - CHOPCONF bit0 enabledrv, MRES bits 27:24 (4 = 16 usteps). Reset 0x13008001.
  *  - Voltage mode PWMCONF 0xC40100FF (autoscale off, PWM_OFS 255): the 40 ohm
  *    coils at VS 3.3 V cannot reach a current-regulation target.
  *  - IFCNT (0x02) counts accepted writes - used to verify the config landed.
  *  - GSTAT u3v5 (bit 2) is a live VS < 3.5 V flag; on this board it is
  *    expected (VS = 3.3 V), and DRV_STATUS ola/olb read set in voltage mode.
  ******************************************************************************
  */
#include "lens_ctrl.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>
#include <stddef.h>
#include <arm_cmse.h>

/* ------------------------------------------------------ TMC2300 registers */
#define R_GSTAT       0x01
#define R_IFCNT       0x02
#define R_IHOLD_IRUN  0x10
#define R_TPOWERDOWN  0x11
#define R_TCOOLTHRS   0x14
#define R_SGTHRS      0x40
#define R_CHOPCONF    0x6C
#define R_DRV_STATUS  0x6F
#define R_PWMCONF     0x70

#define CHOPCONF_16US  ((0x13008001u & ~(0xFu << 24)) | (4u << 24))  /* MRES 16, enabled */
#define PWMCONF_VOLT   0xC40100FFu
#define MRES           16

/* ------------------------------------------------------------- tunables */
#define TICK_HZ          1000000.0f   /* TIM3/4/8 at 64 MHz / 64               */
#define ARR_MIN                 9u    /* STEP output capped at 100 kHz         */
#define ARR_MAX             65535u    /* slowest rate ~15 Hz                   */
#define MOTOR_IRQ_PRIO          3u
#define PPS_START_MAX         800u    /* lens datasheet                        */
#define PPS_SLEW_MAX         1200u
#define JOG_MAX_MS          30000u    /* console jog stops by itself           */
#define POS_FAR      (INT32_MAX / 4)

typedef enum { EVT_NONE = 0, EVT_DONE, EVT_STOP, EVT_LIMIT } Evt_t;

typedef struct {
  /* hardware */
  const char *drv;
  TIM_HandleTypeDef *htim; uint32_t ch; volatile uint32_t *ccr; IRQn_Type irq;
  GPIO_TypeDef *dir_port; uint16_t dir_pin;
  GPIO_TypeDef *en_port;  uint16_t en_pin;
  uint8_t addr;
  bool    fitted;             /* a motor is connected to this driver        */
  bool    present;            /* answered on the UART at the last config    */
  bool    configured;         /* all config writes counted by IFCNT         */
  /* settings */
  uint8_t  irun, ihold;
  uint32_t start_pps, max_pps, acc_pps2;
  uint32_t home_pps, home_fs;
  int8_t   home_dir;          /* -1: home = reverse stop, +1: home = forward stop */
  int32_t  travel;            /* soft-limit span from the home stop, 0 = none */
  /* reference / limits */
  bool     referenced;        /* pos 0 is the reverse stop (home or zero)   */
  bool     limits_on;
  int32_t  min_pos, max_pos;
  /* motion state, shared with the ISR */
  volatile int32_t pos, target;
  volatile int8_t  dir;
  volatile bool    running, jog, stop_req, nolimit;
  volatile uint8_t evt;
  float v, v_start, v_max, acc2;   /* usteps/s, 2*usteps/s^2 */
  /* task-level */
  uint32_t jog_until;         /* HAL tick, 0 = no time limit                */
  bool     timed_out;
  bool     homing;
} Axis_t;

const char *const LENS_AxisName[LENS_NAXIS] = { "zoom", "focus", "iris" };

static Axis_t      s_ax[LENS_NAXIS];
static LensEventFn s_evt_fn;
static char        s_buf[160];

static bool ax_ok(LensAxis_t ax) { return (unsigned)ax < LENS_NAXIS && s_ax[ax].fitted; }

/* ============================================================ TMC UART === */
static void tmc_crc(uint8_t *d, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i + 1u < len; i++) {
    uint8_t b = d[i];
    for (uint8_t j = 0; j < 8u; j++) {
      crc = ((crc >> 7) ^ (b & 1u)) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
      b >>= 1;
    }
  }
  d[len - 1u] = crc;
}

static void tmc_write(uint8_t addr, uint8_t reg, uint32_t val)
{
  uint8_t d[8] = { 0x05, addr, (uint8_t)(reg | 0x80u),
                   (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)val, 0 };
  tmc_crc(d, 8);
  HAL_Delay(2);                        /* idle gap between frames (U1 drops back-to-back writes) */
  HAL_UART_Abort(&huart1);
  HAL_HalfDuplex_EnableTransmitter(&huart1);
  HAL_UART_Transmit(&huart1, d, 8, 50);
}

/* Reply frame: 0x05, 0xFF, reg, data[31:0], CRC. */
static bool tmc_read(uint8_t addr, uint8_t reg, uint32_t *val)
{
  uint8_t tx[4] = { 0x05, addr, (uint8_t)(reg & 0x7Fu), 0 };
  uint8_t rx[8] = { 0 };
  tmc_crc(tx, 4);
  HAL_Delay(2);
  HAL_UART_Abort(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  HAL_HalfDuplex_EnableTransmitter(&huart1);
  HAL_UART_Transmit(&huart1, tx, 4, 50);
  HAL_HalfDuplex_EnableReceiver(&huart1);
  if (HAL_UART_Receive(&huart1, rx, 8, 30) != HAL_OK) return false;
  uint8_t crc = rx[7];
  tmc_crc(rx, 8);
  if (rx[0] != 0x05u || rx[1] != 0xFFu || rx[2] != (reg & 0x7Fu) || rx[7] != crc) return false;
  *val = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) | ((uint32_t)rx[5] << 8) | rx[6];
  return true;
}

static bool drv_config(Axis_t *a)
{
  uint32_t c0, c1, chop;
  a->configured = false;
  a->present = tmc_read(a->addr, R_IFCNT, &c0);
  if (!a->present) return false;
  tmc_write(a->addr, R_GSTAT, 0x7u);                              /* clear flags  */
  tmc_write(a->addr, R_IHOLD_IRUN, (4u << 16) | ((uint32_t)a->irun << 8) | a->ihold);
  tmc_write(a->addr, R_TPOWERDOWN, 20u);
  tmc_write(a->addr, R_PWMCONF, PWMCONF_VOLT);
  tmc_write(a->addr, R_TCOOLTHRS, 0u);                            /* no StallGuard */
  tmc_write(a->addr, R_SGTHRS, 0u);
  tmc_write(a->addr, R_CHOPCONF, CHOPCONF_16US);                  /* enables output */
  if (!tmc_read(a->addr, R_IFCNT, &c1) || !tmc_read(a->addr, R_CHOPCONF, &chop)) return false;
  a->configured = (((c1 - c0) & 0xFFu) == 7u) && chop == CHOPCONF_16US;
  return a->configured;
}

/* ========================================================= step engine === */
static inline uint32_t rate_to_arr(float v)
{
  float t = TICK_HZ / v;
  uint32_t arr = (t > (float)(ARR_MAX + 1u)) ? ARR_MAX : (uint32_t)t - 1u;
  return arr < ARR_MIN ? ARR_MIN : arr;
}

static inline void tim_load(Axis_t *a, uint32_t arr)
{
  a->htim->Instance->ARR = arr;
  *a->ccr = (arr + 1u) / 2u;          /* PWM2: STEP high in the second half */
}

/* ISR context or IRQ masked. */
static void axis_finish(Axis_t *a, Evt_t why)
{
  TIM_TypeDef *T = a->htim->Instance;
  T->CR1 &= ~TIM_CR1_CEN;
  if (T->CNT >= *a->ccr) a->pos += a->dir;   /* pulse of this period already went high */
  T->CNT = 0;
  a->running = false;
  a->stop_req = false;
  a->evt = (uint8_t)why;
}

static void poslog_moving(void);
static void boot_decide(LensPrintFn print);
static LensPrintFn s_print;           /* console print, for messages from LENS_Task */
static bool        s_cfg_pending;     /* boot config failed: keep retrying, then boot */
static void poslog_task(void);
static void axis_start(Axis_t *a, int dir, bool jog, int32_t target, uint32_t max_pps)
{
  poslog_moving();                     /* before any step: position unknown until idle again */
  uint32_t vmax = max_pps ? max_pps : a->max_pps;
  uint32_t vst  = a->start_pps < vmax ? a->start_pps : vmax;
  a->v_start = (float)vst * MRES;
  a->v_max   = (float)vmax * MRES;
  a->acc2    = 2.0f * (float)a->acc_pps2 * MRES;

  a->dir = (int8_t)(dir > 0 ? 1 : -1);
  HAL_GPIO_WritePin(a->dir_port, a->dir_pin, dir > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  for (volatile int i = 0; i < 20; i++) { __NOP(); }   /* DIR setup time */

  a->jog = jog;
  a->target = target;
  a->stop_req = false;
  a->evt = EVT_NONE;
  a->timed_out = false;
  a->v = a->v_start;

  TIM_TypeDef *T = a->htim->Instance;
  T->CR1 &= ~TIM_CR1_CEN;
  T->CNT = 0;
  tim_load(a, rate_to_arr(a->v));
  T->EGR = TIM_EGR_UG;                 /* URS = 1: no interrupt from this */
  T->SR = ~TIM_SR_UIF;
  a->running = true;
  T->CR1 |= TIM_CR1_CEN;
}

void LENS_TimerIRQ(LensAxis_t ax)
{
  if ((unsigned)ax >= LENS_NAXIS) return;
  Axis_t *a = &s_ax[ax];
  TIM_TypeDef *T = a->htim->Instance;
  if ((T->SR & TIM_SR_UIF) == 0u) return;
  T->SR = ~TIM_SR_UIF;
  if (!a->running) { T->CR1 &= ~TIM_CR1_CEN; return; }

  int32_t pos = a->pos + a->dir;       /* one complete STEP pulse just ended */
  a->pos = pos;

  int32_t to_lim = POS_FAR;
  if (a->limits_on && !a->nolimit) to_lim = a->dir > 0 ? a->max_pos - pos : pos - a->min_pos;
  int32_t rem = a->jog ? to_lim : (a->target - pos) * a->dir;
  if (rem > to_lim) rem = to_lim;

  if (rem <= 0) {
    axis_finish(a, (!a->jog && pos == a->target) ? EVT_DONE : EVT_LIMIT);
    return;
  }

  float v2 = a->v * a->v, vs2 = a->v_start * a->v_start, vm2 = a->v_max * a->v_max;
  bool decel = a->stop_req || (float)rem <= (v2 - vs2) / a->acc2 + 1.0f;
  if (decel) {
    v2 -= a->acc2;
    if (v2 <= vs2) {
      v2 = vs2;
      if (a->stop_req) { axis_finish(a, EVT_STOP); return; }
    }
  } else if (v2 < vm2) {
    v2 += a->acc2;
    if (v2 > vm2) v2 = vm2;
  }
  a->v = sqrtf(v2);
  tim_load(a, rate_to_arr(a->v));      /* preloaded: applies next period */
}

static void timer_init(Axis_t *a)
{
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM2;
  oc.Pulse = 500;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_Stop(a->htim, a->ch);
  HAL_TIM_PWM_ConfigChannel(a->htim, &oc, a->ch);   /* sets OCxPE (preload) */

  TIM_TypeDef *T = a->htim->Instance;
  T->CR1 &= ~TIM_CR1_CEN;
  T->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;
  T->CNT = 0;
  T->ARR = 999;
  *a->ccr = 500;
  T->EGR = TIM_EGR_UG;
  T->SR = 0;
  TIM_CCxChannelCmd(T, a->ch, TIM_CCx_ENABLE);
  if (IS_TIM_BREAK_INSTANCE(T)) __HAL_TIM_MOE_ENABLE(a->htim);
  __HAL_TIM_ENABLE_IT(a->htim, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(a->irq, MOTOR_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(a->irq);
}

/* ============================================================ motion ===== */
static const char *s_err = "";

static bool can_move(LensAxis_t ax)
{
  if (!ax_ok(ax))                { s_err = "no such axis"; return false; }
  if (s_ax[ax].running)          { s_err = "busy";         return false; }
  if (!s_ax[ax].configured)      { s_err = "driver not configured (lens init)"; return false; }
  return true;
}

/* ===================================================== zoom levels ====
   Ported from the MSP430 varifocal firmware (MSP430_Varifocal/main.c, v1.5):
   Zoom levels (NLEVELS, 10 since 2026-11-19) with a focus position each. Mapped onto this lens 2026-11-17
   (tools/focus_cal/results.json "zoom_focus_curve", "zoom_far_stop",
   "msp_level_verify*"):
   - both boards count 1/16 steps (1:1 scale fits the measured curve);
   - MSP zoom home (level 0, "fully zoomed out") = our zoom FAR stop, which is
     ~33300 above our zoom reverse stop; MSP zoom levels count back from it;
   - MSP focus home = our focus reverse stop (same direction).
   focus_cal: best focus measured on this camera, used by default.
   focus_msp: the MSP430 column is obsolete; it holds the same values, so
   "lens table msp" makes no difference.
     Calibrated 2026-11-19 15:33 (all earlier tables discarded): checkerboard
     on the chart board 17 m away, Sobel energy on the checkerboard column;
     re-home, then levels in ascending order, focus scanned +-1200 around a
     prediction (200, then 100/50 around the peak) - no full-range sweeps,
     which lost steps on the long moves down. Zoom below 10000 cannot focus at
     17 m (needs focus under the home stop). Focus mechanical end ~33000.
     Table: see s_levels_def.
   Zoom is referenced to its REVERSE stop = 0 (repeatable); the far stop is
   only used through the table. Focus 0 = its real reverse stop (home at
   100 pps; 300 pps stalls 800..1600 short). Long moves down lose steps, so a
   level change re-homes any axis that has to move down, then comes up. */
typedef struct { int32_t zoom, focus_msp, focus_cal; uint32_t sharp; } Level_t;
/* Built-in lookup table (calibrated 2026-11-19 15:33, 10 levels). L0 = zoom
   10000 (widest zoom that focuses at 17 m), levels 2500 zoom steps apart for
   shorter moves. "sharp" = Sobel energy on the checkerboard column of the
   chart board 17 m away (960x540 RTSP frames) - a reference, not used for
   motion; compare only within a level. The MSP430 focus column is obsolete
   and holds the same values. The live table (s_levels) is loaded from flash
   at boot; these values seed the flash when it holds no valid table. */
static const Level_t s_levels_def[] = {     /* re-tuned 2026-11-19 18:20 */
  { 10000,   600,   600, 95498 }, { 12750,  2000,  2000, 91400 }, { 15250,  3200,  3200, 91121 },
  { 17500,  4400,  4400, 81638 }, { 20000,  6050,  6050, 81399 }, { 22500,  7950,  7950, 77925 },
  { 24750, 10200, 10200, 71559 }, { 27500, 13400, 13400, 63365 }, { 30000, 17600, 17600, 56095 },
  { 32500, 24550, 24550, 45261 },
};
#define NLEVELS            ((int)(sizeof s_levels_def / sizeof s_levels_def[0]))
#define ZOOM_FAR_STOP_DEF  34250      /* zoom far stop found 2026-11-18 (image-change scan) */
static Level_t  s_levels[NLEVELS];    /* live table: copy of the flash LUT (or the defaults) */
static int32_t  s_zoom_far = ZOOM_FAR_STOP_DEF;
static const char *s_lut_src = "built-in";
#define LENS_FAST_BOOT     0          /* 1 = restore the logged position at boot (no homing) */
#define BOOT_LEVEL         (NLEVELS - 1)   /* power-up level: L9, the narrowest (user request 2026-11-19) */
#define ZOOM_TRAVEL        33750      /* 500 short of the far stop (34250, 2026-11-18) */
#define LENS_BOOT_SETTLE_MS   3000    /* driver power-up + UART settle before the first TMC config (was 1000; 3000 since 2026-11-19 rework) */
#define LENS_BOOT_CFG_TRIES   10      /* config attempts at boot ... */
#define LENS_BOOT_CFG_GAP_MS  200     /* ... this far apart */
#define LENS_TMC_BAUD         115200u /* TMC2300 UART (see LENS_Init) */
#define FOCUS_TRAVEL_DEFAULT  32400   /* forward soft limit after homing */
#define BACKLASH_PRE       1000       /* moves down go this far below, then up (both axes) */
static int8_t  s_level = -1;          /* current level, -1 = none / moved by hand */
static bool    s_use_msp;             /* focus column: false = focus_cal, true = focus_msp */
static bool    s_level_busy;          /* a level change is running */

/* Zoom lock: zoom moves only through levels (and home) unless unlocked. */
static bool    s_zoom_locked = true;

static int32_t level_focus(int l)
{
  if (l < 0 || l >= NLEVELS) return 0;
  return s_use_msp ? s_levels[l].focus_msp : s_levels[l].focus_cal;
}

static uint8_t s_seq_n, s_seq_i;      /* motion sequencer (see seq_plan) */

/* After-home target: home() then come up to this position. */
static bool    s_ah_valid[LENS_NAXIS];
static int32_t s_ah[LENS_NAXIS];

/* Second leg of a move that must finish from below. */
static bool    s_pend_valid[LENS_NAXIS];
static int32_t s_pend[LENS_NAXIS];

/* Boot sequence: 1 = homing both and going to BOOT_LEVEL, 0 = done. */
static uint8_t s_boot;

static bool zoom_lock_check(LensAxis_t ax)
{
  if (ax != LENS_ZOOM || !s_zoom_locked) return true;
  s_err = "zoom locked: use levels (lens level <n>) or lens z unlock";
  return false;
}

static bool boot_check(void)
{
  if (s_boot) { s_err = "busy: homing at boot"; return false; }
  return true;
}

static bool move_to(LensAxis_t ax, int32_t pos)
{
  if (!can_move(ax)) return false;
  Axis_t *a = &s_ax[ax];
  if (s_pend_valid[ax]) { s_err = "busy"; return false; }
  if (a->limits_on && (pos < a->min_pos || pos > a->max_pos)) { s_err = "outside soft limits"; return false; }
  if (pos == a->pos) { a->target = pos; a->evt = EVT_DONE; return true; }
  a->nolimit = false;
  int32_t first = pos;
  if (pos < a->pos) {                             /* backlash: finish upward */
    first = pos - BACKLASH_PRE;
    if (a->limits_on && first < a->min_pos) first = a->min_pos;
    if (first < pos) { s_pend[ax] = pos; s_pend_valid[ax] = true; }
    else first = pos;
  }
  axis_start(a, first > a->pos ? 1 : 0, false, first, 0);
  return true;
}

bool LENS_MoveTo(LensAxis_t ax, int32_t pos)
{
  if (!(boot_check() && zoom_lock_check(ax) && move_to(ax, pos))) return false;
  if (ax == LENS_ZOOM) s_level = -1;  /* focus trims keep the level */
  return true;
}

bool LENS_MoveRel(LensAxis_t ax, int32_t delta)
{
  if (!ax_ok(ax)) { s_err = "no such axis"; return false; }
  int32_t goal = s_pend_valid[ax] ? s_pend[ax] : s_ax[ax].pos;
  return LENS_MoveTo(ax, goal + delta);
}

static bool jog(LensAxis_t ax, int dir, uint32_t pps, uint32_t max_ms)
{
  if (!boot_check() || !zoom_lock_check(ax) || !can_move(ax)) return false;
  if (s_pend_valid[ax]) { s_err = "busy"; return false; }
  Axis_t *a = &s_ax[ax];
  if (pps == 0u || pps > a->max_pps) pps = a->max_pps;
  /* already at the limit in this direction: the first STEP would overshoot it */
  if (a->limits_on && (dir > 0 ? a->pos >= a->max_pos : a->pos <= a->min_pos)) {
    s_err = "already at the soft limit"; return false;
  }
  a->nolimit = false;
  a->jog_until = max_ms ? HAL_GetTick() + max_ms : 0u;
  if (a->jog_until == 0u && max_ms) a->jog_until = 1u;
  axis_start(a, dir, true, 0, pps);
  return true;
}

bool LENS_Jog(LensAxis_t ax, int dir, uint32_t pps) { return jog(ax, dir, pps, JOG_MAX_MS); }

void LENS_Stop(LensAxis_t ax)
{
  if ((unsigned)ax >= LENS_NAXIS) return;
  s_pend_valid[ax] = false;
  s_ah_valid[ax] = false;
  s_seq_i = s_seq_n;                   /* abort a level change / boot sequence */
  if (s_ax[ax].running) s_ax[ax].stop_req = true;
}

void LENS_Halt(LensAxis_t ax)
{
  if ((unsigned)ax >= LENS_NAXIS) return;
  Axis_t *a = &s_ax[ax];
  s_pend_valid[ax] = false;
  HAL_NVIC_DisableIRQ(a->irq);
  if (a->running) axis_finish(a, EVT_STOP);
  HAL_NVIC_EnableIRQ(a->irq);
}

static bool seq_active(void);
bool    LENS_IsMoving(LensAxis_t ax) { return (unsigned)ax < LENS_NAXIS && (s_ax[ax].running || s_pend_valid[ax] || s_ah_valid[ax] || seq_active()); }
int32_t LENS_GetPos(LensAxis_t ax)   { return (unsigned)ax < LENS_NAXIS ? s_ax[ax].pos : 0; }
void    LENS_SetEventHandler(LensEventFn fn) { s_evt_fn = fn; }

static void apply_limits(Axis_t *a)
{
  int32_t span = a->travel > 0 ? a->travel : POS_FAR;
  if (a->home_dir > 0) { a->min_pos = -span; a->max_pos = 0; }
  else                 { a->min_pos = 0;     a->max_pos = span; }
}

static void set_reference(Axis_t *a)
{
  a->pos = 0;
  a->referenced = true;
  a->limits_on = true;
  apply_limits(a);
}

static bool zero(LensAxis_t ax)
{
  if (!boot_check()) return false;
  if (!ax_ok(ax) || LENS_IsMoving(ax)) { s_err = "busy"; return false; }
  set_reference(&s_ax[ax]);
  poslog_moving();                     /* re-logged as IDLE on the next task pass */
  return true;
}

/* Blind home: run home_fs full steps toward the home stop (home_dir) at
   home_pps, ignoring the soft limits (a PM stepper at 40 ohm / 3.3 V just
   skips at the stop), then call that position 0. */
static bool home(LensAxis_t ax)
{
  if (!can_move(ax)) return false;
  if (s_pend_valid[ax]) { s_err = "busy"; return false; }
  Axis_t *a = &s_ax[ax];
  a->nolimit = true;
  a->homing = true;
  a->jog_until = 0;
  /* Always the full blind distance: a shortened home (position + 200 fs)
     did NOT reach the same zoom stop (focus peak moved 4400 -> <=1000 at
     zoom 17300); the full run repeats (4500 / 4500, 2026-11-17). */
  uint32_t fs = a->home_fs;
  uint32_t save = a->start_pps;
  a->start_pps = a->home_pps;          /* constant speed: no ramp into the stop */
  axis_start(a, a->home_dir > 0 ? 1 : 0, false, a->pos + a->home_dir * (int32_t)(fs * MRES), a->home_pps);
  a->start_pps = save;
  return true;
}

static bool home_cmd(LensAxis_t ax) { return boot_check() && home(ax); }

/* Motion sequencer: one motor at a time. Running zoom and focus together
   from the 3.3 V supply lost steps (2026-11-17: every level was blurred
   with both moving at once, sharp when the same moves ran one by one). */
enum { SEQ_HOME, SEQ_MOVE, SEQ_WAIT };
#define SETTLE_MS 1000                /* after a home: let the rotor settle off the stop */
static uint32_t s_seq_wait_until;
typedef struct { uint8_t ax, op; int32_t pos; } SeqStep_t;
static SeqStep_t s_seq[16];

static bool seq_active(void) { return s_seq_i < s_seq_n; }

/* Reliable move, queued: up = direct; down (or unreferenced) = home, then up. */
static void seq_add(LensAxis_t ax, uint8_t op, int32_t pos)
{
  s_seq[s_seq_n++] = (SeqStep_t){ (uint8_t)ax, op, pos };
  if (op != SEQ_WAIT) s_seq[s_seq_n++] = (SeqStep_t){ (uint8_t)ax, SEQ_WAIT, SETTLE_MS };
}

/* Zoom's reverse stop depends on the focus group: homing zoom with focus at
   0 put zoom ~thousands of steps off (level 3 sharpness 2028 vs 30924 /
   36335 with focus at 10000, 2026-11-17). So zoom only homes with focus at
   or above FOCUS_SAFE_ZOOM_HOME, and focus is homed after zoom is in place. */
#define FOCUS_SAFE_ZOOM_HOME  10000

/* full = re-reference both axes (boot, "lens rehome", or an axis without a
   reference); otherwise a level change moves each motor DIRECTLY to its new
   step count (user request 2026-11-17: no reset to 0 between levels). Moves
   down still overshoot BACKLASH_PRE and finish upward (move_to). */
static void seq_plan(int32_t zt, int32_t ft, bool full)
{
  int32_t fp = s_ax[LENS_FOCUS].pos;
  bool zr = !full && s_ax[LENS_ZOOM].referenced, fr = !full && s_ax[LENS_FOCUS].referenced;
  s_seq_n = s_seq_i = 0;
  if (zr && fr) {                                  /* direct: zoom, then focus */
    seq_add(LENS_ZOOM, SEQ_MOVE, zt);
    seq_add(LENS_FOCUS, SEQ_MOVE, ft);
    return;
  }
  if (!zr) {                                       /* zoom must home */
    if (!fr) { seq_add(LENS_FOCUS, SEQ_HOME, 0); fr = true; fp = 0; }
    if (fp < FOCUS_SAFE_ZOOM_HOME) { seq_add(LENS_FOCUS, SEQ_MOVE, FOCUS_SAFE_ZOOM_HOME); fp = FOCUS_SAFE_ZOOM_HOME; }
    seq_add(LENS_ZOOM, SEQ_HOME, 0);
  }
  seq_add(LENS_ZOOM, SEQ_MOVE, zt);
  if (!fr || full) seq_add(LENS_FOCUS, SEQ_HOME, 0);      /* focus zero taken at the new zoom */
  seq_add(LENS_FOCUS, SEQ_MOVE, ft);
}


static bool level_go(int l)
{
  if (!boot_check()) return false;
  if (l < 0 || l >= NLEVELS) { s_err = "level 0..9"; return false; }
  if (LENS_IsMoving(LENS_ZOOM) || LENS_IsMoving(LENS_FOCUS) || seq_active()) { s_err = "busy"; return false; }
  if (s_levels[l].zoom > s_ax[LENS_ZOOM].max_pos && s_ax[LENS_ZOOM].limits_on) { s_err = "outside soft limits"; return false; }
  seq_plan(s_levels[l].zoom, level_focus(l), false);
  s_level = (int8_t)l;
  s_level_busy = true;
  return true;
}

static bool focus_level_go(void)
{
  if (!boot_check()) return false;
  if (LENS_IsMoving(LENS_FOCUS) || LENS_IsMoving(LENS_ZOOM) || seq_active()) { s_err = "busy"; return false; }
  s_seq_n = s_seq_i = 0;
  seq_add(LENS_FOCUS, SEQ_MOVE, level_focus(s_level));   /* direct */
  return true;
}

static bool rehome_go(void)            /* re-reference both axes, back to the level */
{
  if (!boot_check()) return false;
  if (LENS_IsMoving(LENS_ZOOM) || LENS_IsMoving(LENS_FOCUS) || seq_active()) { s_err = "busy"; return false; }
  int l = s_level < 0 ? BOOT_LEVEL : s_level;
  seq_plan(s_levels[l].zoom, level_focus(l), true);
  s_level = (int8_t)l; s_level_busy = true;
  return true;
}

static void emit(int i, const char *reason)
{
  if (s_evt_fn) s_evt_fn((LensAxis_t)i, s_ax[i].pos, reason);
}

static bool all_idle(void)
{
  for (int i = LENS_ZOOM; i <= LENS_FOCUS; i++)
    if (s_ax[i].running || s_ax[i].homing || s_pend_valid[i] || s_ah_valid[i] || s_ax[i].evt != EVT_NONE) return false;
  return true;
}

static void boot_task(void)
{
  if (!all_idle()) return;
  if (seq_active()) {                  /* next step, only when both motors are idle */
    if (s_seq_wait_until && (int32_t)(HAL_GetTick() - s_seq_wait_until) < 0) return;
    s_seq_wait_until = 0;
    SeqStep_t *st = &s_seq[s_seq_i++];
    if (st->op == SEQ_WAIT) { s_seq_wait_until = HAL_GetTick() + (uint32_t)st->pos; if (!s_seq_wait_until) s_seq_wait_until = 1; return; }
    bool ok = st->op == SEQ_HOME ? home((LensAxis_t)st->ax) : move_to((LensAxis_t)st->ax, st->pos);
    if (!ok) s_seq_i = s_seq_n;        /* abort the rest */
    return;
  }
  if (s_level_busy) {
    s_level_busy = false;
    if (!s_boot) emit(LENS_ZOOM, "level");
  }
  if (s_boot) {
    s_boot = 0;
    emit(LENS_FOCUS, "ready");         /* main.c turns this into READY on the SoC link */
  }
}

/* Late driver config: the boot retries (LENS_BOOT_CFG_TRIES) ran out with a
   driver silent. Keep trying every 500 ms (every 2 s after the first 30 s)
   and run the normal boot (fast boot or homing) as soon as both answer -
   instead of leaving both axes unreferenced until someone types "lens init". */
static void cfg_retry_task(void)
{
  static uint32_t next, t0, n;
  if (!s_cfg_pending) return;
  uint32_t now = HAL_GetTick();
  if (!t0) t0 = now;
  if ((int32_t)(now - next) < 0) return;
  next = now + ((now - t0) < 30000u ? 500u : 2000u);
  Axis_t *z = &s_ax[LENS_ZOOM], *f = &s_ax[LENS_FOCUS];
  if (!z->configured || !f->configured) {
    uint8_t sync = 0;                  /* let the TMC auto-baud lock again */
    HAL_UART_Abort(&huart1);
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, &sync, 1, 10);
    HAL_Delay(2);
    if (!z->configured) drv_config(z);
    if (!f->configured) drv_config(f);
    n++;
  }
  if (z->configured && f->configured) {
    s_cfg_pending = false;
    if (s_print) {
      snprintf(s_buf, sizeof s_buf, "[LENS] driver config: ok after %lu more attempt(s), %lu ms after boot\r\n",
               (unsigned long)n, (unsigned long)(now - t0));
      s_print(s_buf);
    }
    boot_decide(s_print);
  }
}

/* Driver reset watch (idle only, once a second). A TMC2300 that browns out
   or resets comes back with its power-up defaults - 1/32 microsteps instead
   of 1/16, default currents - while the firmware keeps counting 1/16 steps:
   every later move covers half the distance and the lens ends up far from
   its counted position with no sign of a lost step. GSTAT.reset tells.
   On a reset: re-configure, drop the reference, re-home (back to the level).
   A driver that stops answering is marked not configured (moves refused)
   until it answers again. */
static void drv_watch_task(void)
{
  static uint32_t next;
  static uint8_t miss[2];
  static bool drv_err_said[2];
  if (s_cfg_pending || s_boot || seq_active() || s_level_busy || !all_idle()) return;
  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - next) < 0) return;
  next = now + 1000u;
  bool rehome = false;
  for (int i = LENS_ZOOM; i <= LENS_FOCUS; i++) {
    Axis_t *a = &s_ax[i];
    uint32_t g;
    if (!tmc_read(a->addr, R_GSTAT, &g)) {
      if (miss[i] < 255) miss[i]++;
      if (miss[i] == 3 && a->configured) {
        a->configured = false;
        if (s_print) { snprintf(s_buf, sizeof s_buf, "[LENS] %s driver stopped answering - moves blocked until it answers\r\n", LENS_AxisName[i]); s_print(s_buf); }
      }
      continue;
    }
    bool was_lost = miss[i] >= 3;
    miss[i] = 0;
    if (g & 0x1u) {                     /* reset since the last GSTAT clear */
      drv_config(a);
      if (a->referenced) { a->referenced = false; rehome = true; }
      poslog_moving();                  /* next boot homes unless re-referenced */
      if (s_print) { snprintf(s_buf, sizeof s_buf, "[LENS] %s driver RESET detected (supply dip?) - re-configured %s, position lost%s\r\n",
                              LENS_AxisName[i], a->configured ? "ok" : "FAILED", a->configured ? " - re-homing" : ""); s_print(s_buf); }
    } else if (!a->configured) {
      if (was_lost) a->configured = true;   /* registers kept (no reset): usable again */
      else drv_config(a);                    /* never finished its config */
      if (s_print) { snprintf(s_buf, sizeof s_buf, "[LENS] %s driver answers again (no reset), %s\r\n",
                              LENS_AxisName[i], a->configured ? "ok" : "config FAILED"); s_print(s_buf); }
    }
    if ((g & 0x2u) && !drv_err_said[i] && s_print) {
      drv_err_said[i] = true;
      snprintf(s_buf, sizeof s_buf, "[LENS] %s driver error flag (overtemp/short) - see lens status\r\n", LENS_AxisName[i]); s_print(s_buf);
    }
  }
  if (rehome && s_ax[LENS_ZOOM].configured && s_ax[LENS_FOCUS].configured) rehome_go();
}

void LENS_Task(void)
{
  cfg_retry_task();
  drv_watch_task();
  for (int i = 0; i < LENS_NAXIS; i++) {
    Axis_t *a = &s_ax[i];
    if (a->running && a->jog && a->jog_until && (int32_t)(HAL_GetTick() - a->jog_until) >= 0) {
      a->jog_until = 0;
      a->timed_out = true;
      a->stop_req = true;
    }
    uint8_t e = a->evt;
    if (e == EVT_NONE || a->running) continue;
    a->evt = EVT_NONE;
    if (a->homing) {
      a->homing = false;
      a->nolimit = false;
      if (e == EVT_DONE) {
        set_reference(a); emit(i, "homed");
        if (s_ah_valid[i]) { s_ah_valid[i] = false; move_to((LensAxis_t)i, s_ah[i]); }
      }
      else { s_ah_valid[i] = false; emit(i, "home-abort"); }
      continue;
    }
    if (s_pend_valid[i]) {             /* first leg done: now come up to the goal */
      s_pend_valid[i] = false;
      if (e == EVT_DONE && move_to((LensAxis_t)i, s_pend[i])) continue;
    }
    emit(i, a->timed_out ? "timeout" : e == EVT_DONE ? "done" : e == EVT_STOP ? "stop" : "limit");
    a->timed_out = false;
  }
  boot_task();
  poslog_task();
}

/* ============================================================== init ===== */
static void axis_hw(Axis_t *a, const char *drv, TIM_HandleTypeDef *h, uint32_t ch, volatile uint32_t *ccr,
                    IRQn_Type irq, GPIO_TypeDef *dp, uint16_t dpin, GPIO_TypeDef *ep, uint16_t epin, uint8_t addr)
{
  a->drv = drv; a->htim = h; a->ch = ch; a->ccr = ccr; a->irq = irq;
  a->dir_port = dp; a->dir_pin = dpin; a->en_port = ep; a->en_pin = epin; a->addr = addr;
  a->irun = 31; a->ihold = 0;
  a->start_pps = 200; a->max_pps = 300; a->acc_pps2 = 600;
  a->home_pps = 300; a->home_fs = 1200; a->home_dir = -1;
  a->travel = 0;
  a->min_pos = 0; a->max_pos = POS_FAR;
}

static void print_status(LensPrintFn print);

/* ===================================================== position log ===== */
/* Boot without homing (user request 2026-11-18: power-up to level 0 in ~5 s;
   blind homing alone takes ~2 min at the 150 pps the 3.3 V motors manage).
   The motors are unpowered at standstill (IHOLD 0) and the lens groups stay
   put mechanically, so the last resting position stays valid across a power
   cycle. Flash sector 61 (0x0C07A000, reserved by the linker script) is an
   append-only log of 16-byte records:
     MOVE  - written before any motion starts (axis_start, zero)
     IDLE  - zoom/focus position, written once both axes are idle, referenced
             and no sequence is running
   At boot a last record of IDLE restores both positions as referenced; a
   last record of MOVE (power lost mid-move), a blank or corrupt log, or a
   hand-moved lens (cannot be detected) -> full homing / "lens rehome".
   Erase only at rest, when fewer than 2 free slots remain (512 slots ->
   one erase per ~255 moves). */
#define PL_SECTOR   61u
#define PL_IDLE     0x454C4449u      /* "IDLE" */
#define PL_MOVE     0x45564F4Du      /* "MOVE" */
#define PL_SLOTS    (FLASH_SECTOR_SIZE / 16u)
typedef struct { uint32_t tag; int32_t z, f; uint32_t chk; } PosRec_t;
static int     s_pl_next = -1;       /* next free slot, -1 = not scanned */
static uint8_t s_pl_state;           /* 0 = nothing logged, 1 = IDLE last, 2 = MOVE last */
static bool    s_pl_enabled;         /* off until boot decided (no logging of boot homing glitches) */

static uint32_t pl_addr(void) { return FLASH_BASE + PL_SECTOR * FLASH_SECTOR_SIZE; }   /* 0x0C07A000 */
static uint32_t pl_chk(const PosRec_t *r) { return ~(r->tag ^ (uint32_t)r->z ^ ((uint32_t)r->f << 1) ^ 0x5A5A1234u); }

static void pl_scan(PosRec_t *last)
{
  const PosRec_t *r = (const PosRec_t *)pl_addr();
  int n = 0;
  memset(last, 0, sizeof *last);
  while (n < (int)PL_SLOTS && r[n].tag != 0xFFFFFFFFu) {
    if (r[n].chk == pl_chk(&r[n])) *last = r[n];
    n++;
  }
  s_pl_next = n;
}

static bool pl_flash(bool erase, const PosRec_t *rec)
{
  static PosRec_t q __attribute__((aligned(16)));
  bool ok = true;
  bool icache_on = (ICACHE->CR & ICACHE_CR_EN) != 0u;
  if (icache_on) HAL_ICACHE_Disable();
  HAL_FLASH_Unlock();
  FLASH->NSCCR = FLASH->NSSR & 0x00FF0000u;
  if (erase) {
    FLASH_EraseInitTypeDef e = { .TypeErase = FLASH_TYPEERASE_SECTORS_NS, .Banks = FLASH_BANK_1,
                                 .Sector = PL_SECTOR, .NbSectors = 1 };
    uint32_t serr = 0;
    ok = HAL_FLASHEx_Erase(&e, &serr) == HAL_OK;
    if (ok) s_pl_next = 0;
  }
  if (ok && rec) {
    q = *rec;
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD_NS, pl_addr() + 16u * (uint32_t)s_pl_next, (uint32_t)&q) == HAL_OK;
    if (ok) s_pl_next++;
  }
  HAL_FLASH_Lock();
  if (icache_on) { HAL_ICACHE_Invalidate(); HAL_ICACHE_Enable(); }
  return ok;
}

static void pl_append(uint32_t tag, int32_t z, int32_t f, bool may_erase)
{
  if (s_pl_next < 0) { PosRec_t d; pl_scan(&d); }
  bool erase = false;
  if (s_pl_next >= (int)PL_SLOTS - (may_erase ? 1 : 0)) {
    if (!may_erase) return;            /* never erase while a motor runs */
    erase = true;
  }
  PosRec_t r = { tag, z, f, 0 };
  r.chk = pl_chk(&r);
  if (pl_flash(erase || s_pl_next >= (int)PL_SLOTS, &r)) s_pl_state = (tag == PL_IDLE) ? 1 : 2;
}

static void poslog_moving(void)
{
  if (!s_pl_enabled || s_pl_state == 2) return;
  pl_append(PL_MOVE, 0, 0, false);
  s_pl_state = 2;                      /* even if the write failed: re-log IDLE later */
}

static bool all_idle(void);
static void poslog_task(void)
{
  if (!s_pl_enabled || s_pl_state != 2) return;
  if (!all_idle() || seq_active()) return;
  const Axis_t *z = &s_ax[LENS_ZOOM], *f = &s_ax[LENS_FOCUS];
  if (!z->referenced || !f->referenced) return;   /* stays MOVE: next boot homes */
  bool erase_now = s_pl_next >= (int)PL_SLOTS - 2;
  if (erase_now) pl_flash(true, NULL);
  pl_append(PL_IDLE, z->pos, f->pos, true);
}

/* Restore the logged resting position. true = both axes referenced again. */
static bool poslog_restore(int32_t *zp, int32_t *fp)
{
  PosRec_t last;
  pl_scan(&last);
  if (last.tag != PL_IDLE) return false;
  if (last.z < 0 || last.z > 40000 || last.f < 0 || last.f > 40000) return false;
  Axis_t *z = &s_ax[LENS_ZOOM], *f = &s_ax[LENS_FOCUS];
  z->pos = last.z; z->referenced = true; z->limits_on = true; apply_limits(z);
  f->pos = last.f; f->referenced = true; f->limits_on = true; apply_limits(f);
  s_pl_state = 1;
  *zp = last.z; *fp = last.f;
  return true;
}

/* ============================================= lookup table in flash ===== */
/* Zoom/focus level lookup table kept in flash bank 1 sector 62 (0x0C07C000,
   the linker script reserves it: FLASH = 496 KB). Sector 63 is FLASH_NSC
   (secure-gateway veneers) - never touch it.
   TrustZone is DISABLED on this chip (OPTSR2 TZEN = 0xC3; DAUTHSTATUS has no
   secure debug; the core runs non-secure). The HAL is built with -mcmse, so
   its plain erase/program types drive the secure SECCR register, which does
   not exist here (RAZ/WI): the data write then fails with PGSERR in NSSR.
   Always use the *_NS operation types (NSCR). Programming the
   firmware ELF does not erase sector 62, so the table survives reflashing.
   Loaded at boot (CRC-checked); if the sector holds no valid table, the
   built-in defaults are written to it. Changed only by "lens lut save" /
   "lens lut default" with both axes idle (erase stalls bank-1 code fetch
   for a few ms). */
#define LUT_SECTOR  62u
#define LUT_MAGIC   0x54554C4Cu      /* "LLUT" */
#define LUT_VER     1u
typedef struct {
  uint32_t magic, ver, size, nlevels;
  int32_t  zoom_far;
  struct { int32_t zoom, focus; uint32_t sharp; } lv[NLEVELS];
  uint32_t crc;
} LensLut_t;

static uint32_t crc32(const void *d, size_t n)
{
  const uint8_t *p = d; uint32_t c = 0xFFFFFFFFu;
  while (n--) { c ^= *p++; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
  return ~c;
}

static uint32_t lut_addr(void) { return FLASH_BASE + LUT_SECTOR * FLASH_SECTOR_SIZE; }   /* 0x0C07C000 */

static void lut_defaults(void)
{
  memcpy(s_levels, s_levels_def, sizeof s_levels);
  s_zoom_far = ZOOM_FAR_STOP_DEF;
}

static bool lut_load(void)
{
  const LensLut_t *t = (const LensLut_t *)lut_addr();
  if (t->magic != LUT_MAGIC || t->ver != LUT_VER || t->size != sizeof *t || t->nlevels != NLEVELS) return false;
  if (crc32(t, offsetof(LensLut_t, crc)) != t->crc) return false;
  for (int i = 0; i < NLEVELS; i++) {
    if (t->lv[i].zoom < 0 || t->lv[i].zoom > 40000 || t->lv[i].focus < 0 || t->lv[i].focus > 40000) return false;
  }
  for (int i = 0; i < NLEVELS; i++) {
    s_levels[i].zoom = t->lv[i].zoom; s_levels[i].focus_cal = t->lv[i].focus;
    s_levels[i].sharp = t->lv[i].sharp; s_levels[i].focus_msp = s_levels_def[i].focus_msp;
  }
  s_zoom_far = t->zoom_far;
  return true;
}

static const char *lut_save(void)
{
  for (int i = 0; i < LENS_NAXIS; i++) if (s_ax[i].running) return "axis moving";
  if (seq_active()) return "level change running";

  static LensLut_t t __attribute__((aligned(16)));
  memset(&t, 0, sizeof t);
  t.magic = LUT_MAGIC; t.ver = LUT_VER; t.size = sizeof t; t.nlevels = NLEVELS; t.zoom_far = s_zoom_far;
  for (int i = 0; i < NLEVELS; i++) {
    t.lv[i].zoom = s_levels[i].zoom; t.lv[i].focus = s_levels[i].focus_cal; t.lv[i].sharp = s_levels[i].sharp;
  }
  t.crc = crc32(&t, offsetof(LensLut_t, crc));

  FLASH_EraseInitTypeDef e = { .TypeErase = FLASH_TYPEERASE_SECTORS_NS, .Banks = FLASH_BANK_1,
                               .Sector = LUT_SECTOR, .NbSectors = 1 };
  uint32_t serr = 0;
  const char *res = NULL;
  static char why[144];
  /* ICACHE caches the flash code region: program/erase with it disabled,
     invalidate afterwards (RM0481 ICACHE / FLASH programming notes). */
  bool icache_on = (ICACHE->CR & ICACHE_CR_EN) != 0u;
  if (icache_on) HAL_ICACHE_Disable();
  HAL_FLASH_Unlock();
  FLASH->NSCCR = FLASH->NSSR & 0x00FF0000u;   /* clear stale error flags (e.g. left by the debugger) */
  if (HAL_FLASHEx_Erase(&e, &serr) != HAL_OK) {
    snprintf(why, sizeof why, "erase failed (err 0x%lX)", (unsigned long)HAL_FLASH_GetError()); res = why;
  }
  for (uint32_t off = 0; !res && off < sizeof t; off += 16u)
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD_NS, lut_addr() + off, (uint32_t)((uint8_t *)&t + off)) != HAL_OK) {
      snprintf(why, sizeof why, "program failed at 0x%08lX (err 0x%lX)", (unsigned long)(lut_addr() + off),
               (unsigned long)HAL_FLASH_GetError());
      res = why;
    }
  uint32_t nsr = FLASH->NSSR, cr = FLASH->NSCR, err = HAL_FLASH_GetError();
  HAL_FLASH_Lock();
  if (icache_on) { HAL_ICACHE_Invalidate(); HAL_ICACHE_Enable(); }
  if (!res) {
    const uint32_t *f = (const uint32_t *)lut_addr(), *r = (const uint32_t *)&t;
    for (uint32_t k = 0; k < sizeof t / 4u; k++) if (f[k] != r[k]) {
      snprintf(why, sizeof why, "verify failed @+%lu: %08lX want %08lX (NSSR %08lX NSCR %08lX err %lX)",
               (unsigned long)(4u * k), (unsigned long)f[k], (unsigned long)r[k],
               (unsigned long)nsr, (unsigned long)cr, (unsigned long)err);
      res = why; break;
    }
  }
  if (!res) s_lut_src = "flash";
  return res;
}

static void lut_print(LensPrintFn print)
{
  snprintf(s_buf, sizeof s_buf, "lookup table (%s @0x%08lX), zoom far stop %ld\r\n  lvl   zoom  focus  sharpness\r\n",
           s_lut_src, (unsigned long)lut_addr(), (long)s_zoom_far);
  print(s_buf);
  for (int i = 0; i < NLEVELS; i++) {
    snprintf(s_buf, sizeof s_buf, "  %d%s %6ld %6ld %10lu\r\n", i, i == s_level ? "*" : " ", (long)s_levels[i].zoom,
             (long)s_levels[i].focus_cal, (unsigned long)s_levels[i].sharp);
    print(s_buf);
  }
}

void LENS_Init(LensPrintFn print)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_10; g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);            /* PA10 (USART1_RX) must not load the PA9 bus */

  memset(s_ax, 0, sizeof s_ax);
  Axis_t *u1 = &s_ax[LENS_AXIS_U1], *u2 = &s_ax[LENS_AXIS_U2], *u5 = &s_ax[LENS_IRIS];
  axis_hw(u1, "U1", &htim3, TIM_CHANNEL_2, &TIM3->CCR2, TIM3_IRQn,
          ZOOM_DIR_GPIO_Port, ZOOM_DIR_Pin, ZOOM_EN_GPIO_Port, ZOOM_EN_Pin, 0);
  axis_hw(u2, "U2", &htim8, TIM_CHANNEL_2, &TIM8->CCR2, TIM8_UP_IRQn,
          FOCUS_DIR_GPIO_Port, FOCUS_DIR_Pin, FOCUS_EN_GPIO_Port, FOCUS_EN_Pin, 2);
  axis_hw(u5, "U5", &htim4, TIM_CHANNEL_1, &TIM4->CCR1, TIM4_IRQn,
          IRIS_DIR_GPIO_Port, IRIS_DIR_Pin, IRIS_EN_GPIO_Port, IRIS_EN_Pin, 3);
  u1->fitted = u2->fitted = true;      /* iris is fixed: U5 has no motor */

  /* Values tuned on the bench before the rewrite (2026-11-17). */
  Axis_t *zm = &s_ax[LENS_ZOOM], *fm = &s_ax[LENS_FOCUS];
  /* 2026-11-17 ~11:30: speeds used for the sharp-position calibration. */
  /* Level-move speed, tested 2026-11-17 on 3.3 V (12 level changes each,
     all levels >= 90 % of calibrated sharpness to pass): 100/200/500 pass,
     150/200/700 pass, 100/250/600 pass (99-110 %, mean 8.8 s per change),
     200/300/1000 FAIL (lost steps on every upward move). 250 is the
     fastest passing setting; little margin on 3.3 V - a 5 V motor supply
     (IRUN ~25) is the way to go faster. Homing speeds stay slow.
     2026-11-17 18:46 re-check: at 250/600 level 0 came out at 5 % sharpness
     and a re-home made it worse (0.5 %); back at 150/500 every level is at
     its focus peak again -> keep the calibration speed 150. */
  zm->start_pps = 100; zm->max_pps = 150; zm->acc_pps2 = 500;
  zm->home_pps = 150;  zm->home_fs = 4500; zm->home_dir = -1; zm->travel = ZOOM_TRAVEL; /* reverse stop, <=30 s */
  fm->start_pps = 100; fm->max_pps = 150; fm->acc_pps2 = 500;
  fm->home_pps = 100;  fm->home_fs = 3000; fm->home_dir = -1; fm->travel = FOCUS_TRAVEL_DEFAULT;

  HAL_GPIO_WritePin(u1->en_port, u1->en_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(u2->en_port, u2->en_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(u5->en_port, u5->en_pin, GPIO_PIN_RESET);
  /* 2026-11-18: after a power cycle one driver at a time missed its config
     (zoom NOCFG once, focus NO-UART once); "lens init" later always worked.
     The TMC2300s need time after power-up before they answer, so wait
     LENS_BOOT_SETTLE_MS, then retry the config until both are configured. */
  HAL_Delay(LENS_BOOT_SETTLE_MS);

  /* TMC UART baud: 115200, not CubeMX's 9600. 9600 is at the bottom of the
     TMC2300 auto-baud range (min 9000 baud at max internal clock, datasheet
     rev 1.09 4.1). After a cold power-up, or a glitch such as toggling EN,
     the drivers stayed deaf to 9600 (USART1 framing errors, "lens init"
     failing for 10+ minutes, MCU resets did not help); switching to 115200
     made the same stuck drivers answer at once (2026-11-18). */
  HAL_UART_Abort(&huart1);
  huart1.Init.BaudRate = LENS_TMC_BAUD;
  HAL_HalfDuplex_Init(&huart1);

  int tries = 0;
  for (; tries < LENS_BOOT_CFG_TRIES; tries++) {
    if (tries) HAL_Delay(LENS_BOOT_CFG_GAP_MS);
    uint8_t sync = 0;                  /* first byte lets the TMC auto-baud lock */
    HAL_UART_Abort(&huart1);
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, &sync, 1, 10);
    HAL_Delay(2);
    if (!u1->configured) drv_config(u1);
    if (!u2->configured) drv_config(u2);
    if (u1->configured && u2->configured) break;
  }
  timer_init(u1);
  timer_init(u2);
  timer_init(u5);

  print("\r\n[LENS] lens control (rewrite 2026-11-17) - \"lens help\"\r\n");
  if (lut_load()) {
    s_lut_src = "flash";
    print("[LENS] lookup table loaded from flash\r\n");
  } else {
    lut_defaults();
    const char *e = lut_save();
    snprintf(s_buf, sizeof s_buf, "[LENS] no valid lookup table in flash: built-in table %s%s\r\n",
             e ? "in use, flash write FAILED: " : "written to flash", e ? e : "");
    print(s_buf);
  }
  s_ax[LENS_ZOOM].travel = s_zoom_far - 500;
  s_print = print;
  bool ok = u1->configured && u2->configured;
  snprintf(s_buf, sizeof s_buf, "[LENS] driver config: %s after %d attempt(s), %d ms settle, %lu baud%s\r\n",
           ok ? "ok" : "FAILED",
           tries < LENS_BOOT_CFG_TRIES ? tries + 1 : tries, LENS_BOOT_SETTLE_MS, (unsigned long)huart1.Init.BaudRate,
           ok ? "" : " - retrying in the background, boot continues when both answer");
  print(s_buf);
  print_status(print);
  if (ok) boot_decide(print);
  else    s_cfg_pending = true;            /* LENS_Task keeps retrying (cfg_retry_task) */
}

/* Boot decision, once both drivers are configured: restore the logged
   position (fast boot) or home both. Called from LENS_Init, or later from
   cfg_retry_task when a driver answered only after the boot retries. */
static void boot_decide(LensPrintFn print)
{
  /* Boot: home both, then zoom -> fixed step, focus -> calibrated best. */
  int32_t rz, rf;
  /* LENS_FAST_BOOT 0 (user decision 2026-11-19): always full-home at boot.
     Fast boot trusted the logged position and came up out of focus after
     boot moves lost focus steps (L9 at 10 %, focus count off by >2500);
     the full blind home is the only reference that repeats. ~2 min. */
  if (LENS_FAST_BOOT && s_ax[LENS_ZOOM].configured && s_ax[LENS_FOCUS].configured && poslog_restore(&rz, &rf)) {
    /* Fast boot: position known from the log -> direct moves to BOOT_LEVEL
       (nothing to do if the lens rested there). */
    s_pl_enabled = true;
    bool at = rz == s_levels[BOOT_LEVEL].zoom && rf == level_focus(BOOT_LEVEL);
    if (!at) seq_plan(s_levels[BOOT_LEVEL].zoom, level_focus(BOOT_LEVEL), false);
    s_level = BOOT_LEVEL; s_level_busy = true;
    s_boot = 1;
    snprintf(s_buf, sizeof s_buf, "[LENS] boot: position restored (zoom %ld focus %ld), %s level %d - no homing (\"lens rehome\" to re-reference)\r\n",
             (long)rz, (long)rf, at ? "already at" : "direct move to", BOOT_LEVEL);
    print(s_buf);
  } else if (s_ax[LENS_ZOOM].configured && s_ax[LENS_FOCUS].configured) {
    /* One motor at a time; each axis is homed twice: the first sequence
       after an MCU reset landed zoom a few thousand steps short (level 0
       blurred), the same moves run later were fine (2026-11-17). */
    s_pl_enabled = true;
    poslog_moving();
    seq_plan(s_levels[BOOT_LEVEL].zoom, level_focus(BOOT_LEVEL), true);
    s_level = BOOT_LEVEL; s_level_busy = true;
    s_boot = 1;
    snprintf(s_buf, sizeof s_buf, "[LENS] boot: full home (every boot) - focus home + 10000, zoom home + level %d, focus home + level %d (~2 min)\r\n", BOOT_LEVEL, BOOT_LEVEL);
    print(s_buf);
  } else {
    print("[LENS] boot homing not started: driver not configured\r\n");
  }
}

/* =========================================================== console ===== */
static bool parse_long(const char *s, long *out)
{
  if (!s || !*s) return false;
  char *e;
  long v = strtol(s, &e, 0);
  if (*e) return false;
  *out = v;
  return true;
}

static int parse_axis(const char *s)
{
  if (!strcmp(s, "z") || !strcmp(s, "zoom"))  return LENS_ZOOM;
  if (!strcmp(s, "f") || !strcmp(s, "focus")) return LENS_FOCUS;
  if (!strcmp(s, "u1")) return LENS_AXIS_U1;
  if (!strcmp(s, "u2")) return LENS_AXIS_U2;
  return -1;
}

static void print_axis(int i, LensPrintFn print)
{
  Axis_t *a = &s_ax[i];
  uint32_t gstat = 0, drv = 0;
  bool rd = a->present && tmc_read(a->addr, R_GSTAT, &gstat) && tmc_read(a->addr, R_DRV_STATUS, &drv);
  char lim[32];
  if (a->limits_on) snprintf(lim, sizeof lim, "%ld..%ld", (long)a->min_pos,
                             a->max_pos >= POS_FAR ? -1L : (long)a->max_pos);
  else snprintf(lim, sizeof lim, "off");
  snprintf(s_buf, sizeof s_buf, "%-5s %s@%u %s pos %ld (%ld fs) %s  %s  lim %s\r\n",
           LENS_AxisName[i], a->drv, a->addr,
           !a->present ? "NO-UART" : a->configured ? "ok" : "NOCFG",
           (long)a->pos, (long)(a->pos / MRES), LENS_IsMoving((LensAxis_t)i) ? (a->homing ? "HOMING" : "MOVING") : "idle",
           a->referenced ? "ref" : "unref", lim);
  print(s_buf);
  if (i == LENS_ZOOM) {
    snprintf(s_buf, sizeof s_buf, "      %s %ld (level %d, focus table %s)\r\n", s_zoom_locked ? "LOCKED at" : "unlocked at",
             (long)a->pos, s_level, s_use_msp ? "msp" : "cal");
    print(s_buf);
  }
  snprintf(s_buf, sizeof s_buf, "      speed %lu/%lu pps acc %lu  irun %u ihold %u  home %lu pps %lu fs  travel %ld\r\n",
           (unsigned long)a->start_pps, (unsigned long)a->max_pps, (unsigned long)a->acc_pps2,
           a->irun, a->ihold, (unsigned long)a->home_pps, (unsigned long)a->home_fs, (long)a->travel);
  print(s_buf);
  if (rd) {
    snprintf(s_buf, sizeof s_buf, "      GSTAT %02lx%s%s%s  DRV_STATUS %08lx%s%s%s%s\r\n",
             (unsigned long)gstat, (gstat & 1u) ? " reset" : "", (gstat & 2u) ? " drv_err" : "",
             (gstat & 4u) ? " uv(VS<3.5V)" : "", (unsigned long)drv,
             (drv & 0x3u) ? " OT" : "", (drv & 0x3Cu) ? " SHORT" : "",
             (drv & 0xC0u) ? " openload" : "", (drv & (1u << 31)) ? " standstill" : "");
    print(s_buf);
  }
}

static void print_status(LensPrintFn print)
{
  print_axis(LENS_ZOOM, print);
  print_axis(LENS_FOCUS, print);
}

static void print_help(LensPrintFn print)
{
  print("lens commands  (<ax> = z|zoom f|focus u1 u2; pos in usteps, 16 per full step; speeds full steps/s)\r\n");
  print("  lens [status]                    positions and driver state\r\n");
  print("  lens init                        re-write both driver configs\r\n");
  print("  lens stop                        stop all axes\r\n");
  print("  lens <ax> goto <pos>             absolute move\r\n");
  print("  lens <ax> rel <+-usteps>         relative move\r\n");
  print("  lens <ax> jog <1|0> [pps]        run fwd/rev until stop, limit or 30 s\r\n");
  print("  lens <ax> stop | halt            ramped stop | immediate stop\r\n");
  print("  lens <ax> home                   run into the home stop (zoom: far, focus: reverse), pos := 0\r\n");
  print("  lens <ax> zero                   pos := 0 here (sets reference)\r\n");
  print("  lens <ax> limit <min> <max>|off  soft limits (max -1 = none)\r\n");
  print("  lens <ax> travel <usteps>        limit span from the home stop (0 = none)\r\n");
  print("  lens <ax> homecfg <pps> <fs>     homing speed and distance\r\n");
  print("  lens <ax> speed <start> <max> <acc>   start<=800, max<=1200\r\n");
  print("  lens <ax> cur <irun> [ihold]     0..31 (voltage mode: amplitude = VS*(irun+1)/32)\r\n");
  print("  lens z lock | unlock            zoom moves only through levels while locked (default)\r\n");
  print("  lens <ax> reg <r> [val]          raw TMC2300 register read/write\r\n");
  print("  lens level [n] | zoomin | zoomout  zoom level 0..9 (MSP430 table) with its focus\r\n");
  print("  lens table cal|msp               focus column: this camera's / MSP430 values\r\n");
  print("  lens lut                         level lookup table (zoom, focus, sharpness) in flash\r\n");
  print("  lens lut set <n> <z> <f> [sharp] | here <n> [sharp] | far <usteps>   edit in RAM\r\n");
  print("  lens lut save | load | default   write to / reload from flash, built-in values\r\n");
  print("  lens rehome                      re-reference both axes, return to the level\r\n");
  print("  lens f best                      focus back to the current level's focus\r\n");
  print("  zoom_run|focus_run <dir> <step_hz>, zoom_stop|focus_stop   old jog commands\r\n");
}

static void reply_move(bool ok, Axis_t *a, LensPrintFn print)
{
  if (ok) snprintf(s_buf, sizeof s_buf, "OK %s: pos %ld -> %ld\r\n", a == &s_ax[LENS_ZOOM] ? "zoom" : "focus",
                   (long)a->pos, (long)a->target);
  else snprintf(s_buf, sizeof s_buf, "ERROR: %s\r\n", s_err);
  print(s_buf);
}

static void cli_axis(LensAxis_t ax, char **tok, int nt, LensPrintFn print)
{
  Axis_t *a = &s_ax[ax];
  long v1, v2, v3;
  const char *op = nt > 0 ? tok[0] : "status";

  if (!strcmp(op, "status")) { print_axis(ax, print); return; }
  if (ax == LENS_FOCUS && !strcmp(op, "best")) {
    if (s_level < 0) { print("ERROR: no level selected (lens level <n>)\r\n"); return; }
    snprintf(s_buf, sizeof s_buf, "level %d focus %ld\r\n", s_level, (long)level_focus(s_level)); print(s_buf);
    bool ok = focus_level_go();
    print(ok ? "OK focus to level focus (re-homes first if moving down)\r\n" : "ERROR: busy\r\n"); return;
  }
  if (ax == LENS_ZOOM && !strcmp(op, "unlock")) { s_zoom_locked = false; print("OK zoom unlocked\r\n"); return; }
  if (ax == LENS_ZOOM && !strcmp(op, "lock")) { s_zoom_locked = true; print("OK zoom locked (levels only)\r\n"); return; }
  if (!strcmp(op, "goto") && nt >= 2 && parse_long(tok[1], &v1)) { reply_move(LENS_MoveTo(ax, (int32_t)v1), a, print); return; }
  if (!strcmp(op, "rel")  && nt >= 2 && parse_long(tok[1], &v1)) { reply_move(LENS_MoveRel(ax, (int32_t)v1), a, print); return; }
  if (!strcmp(op, "jog")  && nt >= 2 && parse_long(tok[1], &v1)) {
    v2 = 0;
    if (nt >= 3 && !parse_long(tok[2], &v2)) { print("ERROR: pps\r\n"); return; }
    bool ok = LENS_Jog(ax, v1 ? 1 : 0, (uint32_t)(v2 > 0 ? v2 : 0));
    if (ok) snprintf(s_buf, sizeof s_buf, "OK %s jog %s at %lu pps\r\n", LENS_AxisName[ax],
                     v1 ? "fwd" : "rev", (unsigned long)(a->v_max / MRES));
    else snprintf(s_buf, sizeof s_buf, "ERROR: %s\r\n", s_err);
    print(s_buf); return;
  }
  if (!strcmp(op, "stop")) { LENS_Stop(ax); print("OK stopping\r\n"); return; }
  if (!strcmp(op, "halt")) { LENS_Halt(ax); print("OK halted\r\n"); return; }
  if (!strcmp(op, "home")) {
    if (home_cmd(ax)) snprintf(s_buf, sizeof s_buf, "OK %s homing: %lu fs reverse at %lu pps\r\n",
                           LENS_AxisName[ax], (unsigned long)a->home_fs, (unsigned long)a->home_pps);
    else snprintf(s_buf, sizeof s_buf, "ERROR: %s\r\n", s_err);
    print(s_buf); return;
  }
  if (!strcmp(op, "zero")) { print(zero(ax) ? "OK pos 0, reference set\r\n" : "ERROR: busy\r\n"); return; }
  if (!strcmp(op, "limit") && nt >= 2) {
    if (!strcmp(tok[1], "off")) { a->limits_on = false; print("OK limits off\r\n"); return; }
    if (nt >= 3 && parse_long(tok[1], &v1) && parse_long(tok[2], &v2)) {
      a->min_pos = (int32_t)v1; a->max_pos = v2 < 0 ? POS_FAR : (int32_t)v2; a->limits_on = true;
      print("OK limits set\r\n"); return;
    }
  }
  if (!strcmp(op, "travel") && nt >= 2 && parse_long(tok[1], &v1) && v1 >= 0) {
    a->travel = (int32_t)v1;
    if (a->referenced) apply_limits(a);
    print("OK travel set\r\n"); return;
  }
  if (!strcmp(op, "homecfg") && nt >= 3 && parse_long(tok[1], &v1) && parse_long(tok[2], &v2) &&
      v1 >= 20 && v1 <= PPS_START_MAX && v2 > 0) {
    a->home_pps = (uint32_t)v1; a->home_fs = (uint32_t)v2; print("OK homecfg\r\n"); return;
  }
  if (!strcmp(op, "speed") && nt >= 4 && parse_long(tok[1], &v1) && parse_long(tok[2], &v2) && parse_long(tok[3], &v3)) {
    if (a->running) { print("ERROR: busy\r\n"); return; }
    if (v1 < 20 || v1 > PPS_START_MAX || v2 < v1 || v2 > PPS_SLEW_MAX || v3 < 50) {
      print("ERROR: need 20<=start<=800, start<=max<=1200, acc>=50\r\n"); return;
    }
    a->start_pps = (uint32_t)v1; a->max_pps = (uint32_t)v2; a->acc_pps2 = (uint32_t)v3;
    print("OK speed set\r\n"); return;
  }
  if (!strcmp(op, "cur") && nt >= 2 && parse_long(tok[1], &v1) && v1 >= 0 && v1 <= 31) {
    v2 = a->ihold;
    if (nt >= 3 && (!parse_long(tok[2], &v2) || v2 < 0 || v2 > 31)) { print("ERROR: ihold 0..31\r\n"); return; }
    a->irun = (uint8_t)v1; a->ihold = (uint8_t)v2;
    print(drv_config(a) ? "OK current set\r\n" : "ERROR: driver config failed\r\n"); return;
  }
  if (!strcmp(op, "reg") && nt >= 2 && parse_long(tok[1], &v1) && v1 >= 0 && v1 < 0x80) {
    uint32_t val;
    if (nt >= 3) {
      if (!parse_long(tok[2], &v2)) { print("ERROR: value\r\n"); return; }
      tmc_write(a->addr, (uint8_t)v1, (uint32_t)v2);
      print("OK written\r\n"); return;
    }
    if (tmc_read(a->addr, (uint8_t)v1, &val)) snprintf(s_buf, sizeof s_buf, "reg 0x%02lx = 0x%08lx\r\n", (unsigned long)v1, (unsigned long)val);
    else snprintf(s_buf, sizeof s_buf, "ERROR: no reply\r\n");
    print(s_buf); return;
  }
  print("ERROR: bad lens command (lens help)\r\n");
}

bool LENS_CLI(const char *cmd, LensPrintFn print)
{
  char line[96];
  strncpy(line, cmd, sizeof line - 1u);
  line[sizeof line - 1u] = '\0';
  for (char *p = line; *p; p++) *p = (char)tolower((unsigned char)*p);

  char *tok[8]; int nt = 0;
  for (char *t = strtok(line, " \t"); t && nt < 8; t = strtok(NULL, " \t")) tok[nt++] = t;
  if (nt == 0) return false;

  /* old jog commands: <axis>_run <dir> <step_hz> (STEP pulses/s), <axis>_stop */
  for (int i = LENS_ZOOM; i <= LENS_FOCUS; i++) {
    size_t n = strlen(LENS_AxisName[i]);
    if (strncmp(tok[0], LENS_AxisName[i], n) || tok[0][n] != '_') continue;
    const char *suf = tok[0] + n + 1;
    long dir, hz;
    if (!strcmp(suf, "stop")) { LENS_Stop((LensAxis_t)i); print("SUCCESS: stopping\r\n"); return true; }
    if (!strcmp(suf, "run")) {
      if (nt < 3 || !parse_long(tok[1], &dir) || !parse_long(tok[2], &hz) || hz <= 0) {
        print("ERROR: usage <axis>_run <0|1> <step_hz>, step_hz > 0\r\n"); return true;
      }
      uint32_t pps = (uint32_t)hz / MRES;
      if (pps < 20u) pps = 20u;
      if (LENS_Jog((LensAxis_t)i, dir ? 1 : 0, pps))
        snprintf(s_buf, sizeof s_buf, "SUCCESS: %s jog dir %ld at %lu pps (%lu step/s)\r\n", LENS_AxisName[i],
                 dir ? 1L : 0L, (unsigned long)(s_ax[i].v_max / MRES), (unsigned long)s_ax[i].v_max);
      else snprintf(s_buf, sizeof s_buf, "REJECTED: %s\r\n", s_err);
      print(s_buf); return true;
    }
    return false;
  }

  if (strcmp(tok[0], "lens")) return false;
  if (nt == 1 || !strcmp(tok[1], "status")) { print_status(print); return true; }
  if (!strcmp(tok[1], "help")) { print_help(print); return true; }
  if (!strcmp(tok[1], "stop")) { LENS_Stop(LENS_ZOOM); LENS_Stop(LENS_FOCUS); print("OK stopping all\r\n"); return true; }
  if (!strcmp(tok[1], "init")) {
    if (s_ax[LENS_ZOOM].running || s_ax[LENS_FOCUS].running) { print("ERROR: busy\r\n"); return true; }
    bool z = drv_config(&s_ax[LENS_ZOOM]), f = drv_config(&s_ax[LENS_FOCUS]);
    snprintf(s_buf, sizeof s_buf, "zoom %s, focus %s\r\n", z ? "ok" : "FAILED", f ? "ok" : "FAILED");
    print(s_buf); return true;
  }
  if (!strcmp(tok[1], "level") || !strcmp(tok[1], "zoomin") || !strcmp(tok[1], "zoomout")) {
    long l = s_level;
    if (!strcmp(tok[1], "zoomin")) l = s_level < 0 ? 0 : s_level + 1;
    else if (!strcmp(tok[1], "zoomout")) l = s_level - 1;
    else if (nt < 3) {
      snprintf(s_buf, sizeof s_buf, "level %d, focus table %s\r\n  lvl  zoom  focus_cal focus_msp\r\n", s_level, s_use_msp ? "msp" : "cal");
      print(s_buf);
      for (int i = 0; i < NLEVELS; i++) {
        snprintf(s_buf, sizeof s_buf, "  %d%s %6ld %8ld %8ld\r\n", i, i == s_level ? "*" : " ", (long)s_levels[i].zoom,
                 (long)s_levels[i].focus_cal, (long)s_levels[i].focus_msp);
        print(s_buf);
      }
      return true;
    } else if (!parse_long(tok[2], &l)) { print("ERROR: lens level <0..9>\r\n"); return true; }
    if (l < 0 || l >= NLEVELS) { print(l < 0 ? "Reached MIN level\r\n" : "Reached MAX level\r\n"); return true; }
    if (level_go((int)l)) snprintf(s_buf, sizeof s_buf, "OK level %ld: zoom %ld focus %ld\r\n", l,
                                   (long)s_levels[l].zoom, (long)level_focus((int)l));
    else snprintf(s_buf, sizeof s_buf, "ERROR: %s\r\n", s_err);
    print(s_buf); return true;
  }
  if (!strcmp(tok[1], "rehome")) {
    print(rehome_go() ? "OK re-homing both axes (~80 s), then back to the level\r\n" : "ERROR: busy\r\n");
    return true;
  }
  if (!strcmp(tok[1], "baud")) {              /* TMC UART baud rate (USART1, half duplex) */
    long v;
    if (nt >= 3 && parse_long(tok[2], &v) && v >= 9600 && v <= 500000) {
      HAL_UART_Abort(&huart1);
      huart1.Init.BaudRate = (uint32_t)v;
      bool ok = HAL_HalfDuplex_Init(&huart1) == HAL_OK;
      snprintf(s_buf, sizeof s_buf, ok ? "OK TMC UART %ld baud (\"lens init\" to re-config)\r\n" : "ERROR: UART init at %ld\r\n", v);
    } else snprintf(s_buf, sizeof s_buf, "TMC UART %lu baud - lens baud <9600..500000>\r\n", (unsigned long)huart1.Init.BaudRate);
    print(s_buf); return true;
  }
  if (!strcmp(tok[1], "lut")) {               /* level lookup table in flash */
    long n, z, f, v;
    if (nt < 3) { lut_print(print); return true; }
    if (!strcmp(tok[2], "save")) {
      const char *e = lut_save();
      if (e) snprintf(s_buf, sizeof s_buf, "ERROR: LUT save: %s\r\n", e);
      else snprintf(s_buf, sizeof s_buf, "OK lookup table saved to flash @0x%08lX\r\n", (unsigned long)lut_addr());
      print(s_buf); return true;
    }
    if (!strcmp(tok[2], "diag")) {
      uint32_t c0 = FLASH->SECCR, n0 = FLASH->NSCR;
      HAL_StatusTypeDef u = HAL_FLASH_Unlock();
      uint32_t c1 = FLASH->SECCR, n1 = FLASH->NSCR;
      HAL_FLASH_Lock();
      uint32_t c2 = FLASH->SECCR;
      snprintf(s_buf, sizeof s_buf, "SECCR %08lX->unlock(%d) %08lX->lock %08lX  NSCR %08lX->%08lX\r\n",
               (unsigned long)c0, (int)u, (unsigned long)c1, (unsigned long)c2, (unsigned long)n0, (unsigned long)n1);
      print(s_buf);
      snprintf(s_buf, sizeof s_buf, "CONTROL %08lX PRIVCFGR %08lX SECWM1R %08lX SECBB1R2 %08lX OPTSR2 %08lX SECSR %08lX\r\n",
               (unsigned long)__get_CONTROL(), (unsigned long)FLASH->PRIVCFGR, (unsigned long)FLASH->SECWM1R_CUR,
               (unsigned long)FLASH->SECBB1R2, (unsigned long)FLASH->OPTSR2_CUR, (unsigned long)FLASH->SECSR);
      print(s_buf);
      cmse_address_info_t r1 = cmse_TT((void *)&FLASH->SECCR), r2 = cmse_TT((void *)lut_addr()), r3 = cmse_TT((void *)&lut_save);
      snprintf(s_buf, sizeof s_buf, "SAU CTRL %08lX  TT SECCR %08lX  TT LUT %08lX  TT code %08lX\r\n",
               (unsigned long)SAU->CTRL, (unsigned long)r1.value, (unsigned long)r2.value, (unsigned long)r3.value);
      print(s_buf); return true;
    }
    if (!strcmp(tok[2], "load")) {
      print(lut_load() ? (s_lut_src = "flash", "OK lookup table reloaded from flash\r\n")
                       : "ERROR: no valid lookup table in flash\r\n");
      return true;
    }
    if (!strcmp(tok[2], "default")) {
      lut_defaults(); s_lut_src = "built-in (not saved)";
      print("OK built-in table in RAM - \"lens lut save\" to store it\r\n"); return true;
    }
    if (!strcmp(tok[2], "set") && nt >= 6 && parse_long(tok[3], &n) && parse_long(tok[4], &z) && parse_long(tok[5], &f)) {
      if (n < 0 || n >= NLEVELS || z < 0 || z > s_zoom_far || f < 0 || f > FOCUS_TRAVEL_DEFAULT) { print("ERROR: lens lut set <0..9> <zoom> <focus> [sharp]\r\n"); return true; }
      s_levels[n].zoom = z; s_levels[n].focus_cal = f;
      if (nt >= 7 && parse_long(tok[6], &v) && v >= 0) s_levels[n].sharp = (uint32_t)v;
      s_lut_src = "edited (not saved)";
      snprintf(s_buf, sizeof s_buf, "OK L%ld = zoom %ld focus %ld (\"lens lut save\" to store)\r\n", n, z, f); print(s_buf); return true;
    }
    if (!strcmp(tok[2], "here") && nt >= 4 && parse_long(tok[3], &n)) {
      if (n < 0 || n >= NLEVELS) { print("ERROR: level 0..9\r\n"); return true; }
      if (!s_ax[LENS_ZOOM].referenced || !s_ax[LENS_FOCUS].referenced) { print("ERROR: home first\r\n"); return true; }
      s_levels[n].zoom = s_ax[LENS_ZOOM].pos; s_levels[n].focus_cal = s_ax[LENS_FOCUS].pos;
      if (nt >= 5 && parse_long(tok[4], &v) && v >= 0) s_levels[n].sharp = (uint32_t)v;
      s_lut_src = "edited (not saved)";
      snprintf(s_buf, sizeof s_buf, "OK L%ld = zoom %ld focus %ld (\"lens lut save\" to store)\r\n", n,
               (long)s_levels[n].zoom, (long)s_levels[n].focus_cal); print(s_buf); return true;
    }
    if (!strcmp(tok[2], "far") && nt >= 4 && parse_long(tok[3], &v) && v > 0 && v <= 40000) {
      s_zoom_far = v; s_ax[LENS_ZOOM].travel = v - 500; s_lut_src = "edited (not saved)";
      print("OK zoom far stop set (applies to the zoom limit at the next home)\r\n"); return true;
    }
    print("ERROR: lens lut [save|load|default|set <n> <zoom> <focus> [sharp]|here <n> [sharp]|far <usteps>]\r\n");
    return true;
  }
  if (!strcmp(tok[1], "table") && nt >= 3) {
    s_use_msp = !strcmp(tok[2], "msp");
    print(s_use_msp ? "OK focus table: MSP430 values\r\n" : "OK focus table: calibrated on this camera\r\n");
    return true;
  }
  int ax = parse_axis(tok[1]);
  if (ax < 0) { print("ERROR: axis z|f|u1|u2 (lens help)\r\n"); return true; }
  cli_axis((LensAxis_t)ax, &tok[2], nt - 2, print);
  return true;
}

/* ====================================================== SoC host link ==== */
/* UART4 / J12, 115200 8N1, one command per line, case-insensitive.
   Positions in microsteps (16 per full step).
     Z <pos>   F <pos>      absolute move     -> OK Z <target>
     ZR <d>    FR <d>       relative move     -> OK Z <target>
     ZS  FS  S              ramped stop       -> OK STOP
     H Z|F|ALL              home              -> OK H, later DONE Z 0 homed
     ZERO Z|F|ALL           pos := 0 here     -> OK ZERO
     P  or  ?               positions         -> POS Z <pos> <moving> F <pos> <moving>
     FB                     focus to the level's focus -> OK F <pos>
     L <n> / L              zoom level 0..9 / query -> OK L <n> / LEVEL <n>
     ZI  ZO                 zoom level +1 / -1 -> OK L <n>; then DONE Z <pos> level
     V                      version           -> VER PROXY-LENS 3
   At boot the MCU homes both axes, moves zoom to its fixed step and focus to
   the calibrated best, then sends READY. Until then moves get ERR BUSY.
   Zoom is locked: Z / ZR get ERR LOCKED. Focus moves from above go 1000
   below the goal first and finish upward (gear backlash).
   Errors: ERR SYNTAX | ERR BUSY | ERR RANGE | ERR NODRV | ERR LOCKED
   Unsolicited on every stop: DONE Z|F <pos> <reason>                       */
static void host_err(LensPrintFn reply)
{
  if (!strncmp(s_err, "busy", 4)) reply("ERR BUSY\n");
  else if (!strncmp(s_err, "zoom locked", 11)) reply("ERR LOCKED\n");
  else if (!strcmp(s_err, "outside soft limits")) reply("ERR RANGE\n");
  else reply("ERR NODRV\n");
}

static int host_which(const char *w)    /* bit0 zoom, bit1 focus */
{
  if (!w) return 0;
  if (!strcmp(w, "Z")) return 1;
  if (!strcmp(w, "F")) return 2;
  if (!strcmp(w, "ALL")) return 3;
  return 0;
}

void LENS_HostCommand(const char *line, LensPrintFn reply)
{
  char b[64], out[80];
  strncpy(b, line, sizeof b - 1u);
  b[sizeof b - 1u] = '\0';
  for (char *p = b; *p; p++) *p = (char)toupper((unsigned char)*p);
  char *c = strtok(b, " \t"), *arg = strtok(NULL, " \t");
  if (!c) return;
  long v;

  if (!strcmp(c, "V")) { reply("VER PROXY-LENS 3\n"); return; }
  if (!strcmp(c, "P") || !strcmp(c, "?")) {
    snprintf(out, sizeof out, "POS Z %ld %d F %ld %d\n", (long)s_ax[LENS_ZOOM].pos, s_ax[LENS_ZOOM].running,
             (long)s_ax[LENS_FOCUS].pos, s_ax[LENS_FOCUS].running);
    reply(out); return;
  }
  if (!strcmp(c, "FB")) {
    if (s_level < 0) { reply("ERR RANGE\n"); return; }
    if (!focus_level_go()) { host_err(reply); return; }
    snprintf(out, sizeof out, "OK F %ld\n", (long)level_focus(s_level)); reply(out); return;
  }
  if (!strcmp(c, "L") || !strcmp(c, "ZI") || !strcmp(c, "ZO")) {
    long l;
    if (c[0] == 'L') {
      if (!arg) { snprintf(out, sizeof out, "LEVEL %d\n", s_level); reply(out); return; }
      if (!parse_long(arg, &l)) { reply("ERR SYNTAX\n"); return; }
    } else l = c[1] == 'I' ? (s_level < 0 ? 0 : s_level + 1) : s_level - 1;
    if (l < 0 || l >= NLEVELS) { reply("ERR RANGE\n"); return; }
    if (!level_go((int)l)) { host_err(reply); return; }
    snprintf(out, sizeof out, "OK L %ld\n", l); reply(out); return;
  }
  if (!strcmp(c, "S"))  { LENS_Stop(LENS_ZOOM); LENS_Stop(LENS_FOCUS); reply("OK STOP\n"); return; }
  if (!strcmp(c, "ZS")) { LENS_Stop(LENS_ZOOM);  reply("OK STOP\n"); return; }
  if (!strcmp(c, "FS")) { LENS_Stop(LENS_FOCUS); reply("OK STOP\n"); return; }

  if ((!strcmp(c, "Z") || !strcmp(c, "F") || !strcmp(c, "ZR") || !strcmp(c, "FR"))) {
    if (!parse_long(arg, &v)) { reply("ERR SYNTAX\n"); return; }
    LensAxis_t ax = c[0] == 'Z' ? LENS_ZOOM : LENS_FOCUS;
    bool ok = c[1] == 'R' ? LENS_MoveRel(ax, (int32_t)v) : LENS_MoveTo(ax, (int32_t)v);
    if (!ok) { host_err(reply); return; }
    snprintf(out, sizeof out, "OK %c %ld\n", c[0], (long)s_ax[ax].target);
    reply(out); return;
  }
  if (!strcmp(c, "H") || !strcmp(c, "ZERO")) {
    int w = host_which(arg);
    if (!w) { reply("ERR SYNTAX\n"); return; }
    bool is_home = c[0] == 'H';
    for (int i = 0; i < 2; i++) {
      if (!(w & (1 << i))) continue;
      LensAxis_t ax = i ? LENS_FOCUS : LENS_ZOOM;
      if (!(is_home ? home_cmd(ax) : zero(ax))) { host_err(reply); return; }
    }
    reply(is_home ? "OK H\n" : "OK ZERO\n"); return;
  }
  reply("ERR SYNTAX\n");
}
