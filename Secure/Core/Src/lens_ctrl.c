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
  int32_t  travel;            /* forward soft limit after referencing       */
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

static void axis_start(Axis_t *a, int dir, bool jog, int32_t target, uint32_t max_pps)
{
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

/* Zoom lock (2026-11-17, user: calibrate and control focus only, zoom stays
   at a fixed step). While locked, zoom move/jog requests are refused; homing
   is allowed and the axis then returns to the fixed position by itself. */
#define ZOOM_FIXED_DEFAULT  9599      /* Zoom0 field of view (user, 2026-11-17) */
static bool    s_zoom_locked = true;
static int32_t s_zoom_fixed  = ZOOM_FIXED_DEFAULT;
static char    s_lock_msg[48];

/* Focus calibration, tools/focus_cal.py, 2026-11-17 (zoom 9599, day mode,
   centre-1/3 Sobel sharpness, tools/focus_cal/results.json):
   - sharpness peaks at the homed 0 (887, flat top: 0/100/200 = 887/881/868)
     and falls steadily upward; home -> 0 five times: within 0.6 %.
   - the image stops changing above ~34000 (far end, or just full blur -
     not confirmed by eye); travel = 34000 - 1600.
   - the homed 0 is a repeatable stall point, not the hard stop: the stop is
     ~800..1600 lower, and moves below 0 lose steps. Keep the limit at 0. */
#define FOCUS_BEST_DEFAULT    0       /* best focus for the installed scene, usteps */
#define FOCUS_TRAVEL_DEFAULT  32400   /* forward soft limit after homing */
#define FOCUS_BACKLASH_PRE 1000       /* moves from above go this far below, then up */
static int32_t s_focus_best = FOCUS_BEST_DEFAULT;

/* Second leg of a move that must finish from below. */
static bool    s_pend_valid[LENS_NAXIS];
static int32_t s_pend[LENS_NAXIS];

/* Boot sequence: 1 = homing both, 2 = moving to zoom fixed / focus best, 0 = done. */
static uint8_t s_boot;

static bool zoom_lock_check(LensAxis_t ax)
{
  if (ax != LENS_ZOOM || !s_zoom_locked) return true;
  snprintf(s_lock_msg, sizeof s_lock_msg, "zoom locked at %ld (lens z unlock)", (long)s_zoom_fixed);
  s_err = s_lock_msg;
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
  if (ax == LENS_FOCUS && pos < a->pos) {          /* backlash: finish upward */
    first = pos - FOCUS_BACKLASH_PRE;
    if (a->limits_on && first < a->min_pos) first = a->min_pos;
    if (first < pos) { s_pend[ax] = pos; s_pend_valid[ax] = true; }
    else first = pos;
  }
  axis_start(a, first > a->pos ? 1 : 0, false, first, 0);
  return true;
}

bool LENS_MoveTo(LensAxis_t ax, int32_t pos)
{
  return boot_check() && zoom_lock_check(ax) && move_to(ax, pos);
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

bool    LENS_IsMoving(LensAxis_t ax) { return (unsigned)ax < LENS_NAXIS && (s_ax[ax].running || s_pend_valid[ax]); }
int32_t LENS_GetPos(LensAxis_t ax)   { return (unsigned)ax < LENS_NAXIS ? s_ax[ax].pos : 0; }
void    LENS_SetEventHandler(LensEventFn fn) { s_evt_fn = fn; }

static void set_reference(Axis_t *a)
{
  a->pos = 0;
  a->referenced = true;
  a->limits_on = true;
  a->min_pos = 0;
  a->max_pos = a->travel > 0 ? a->travel : POS_FAR;
}

static bool zero(LensAxis_t ax)
{
  if (!boot_check()) return false;
  if (!ax_ok(ax) || LENS_IsMoving(ax)) { s_err = "busy"; return false; }
  set_reference(&s_ax[ax]);
  return true;
}

/* Blind home: run home_fs full steps in reverse at home_pps, ignoring the
   soft limits, into the reverse hard stop (a PM stepper at 40 ohm / 3.3 V
   just skips there), then call that position 0. */
static bool home(LensAxis_t ax)
{
  if (!can_move(ax)) return false;
  if (s_pend_valid[ax]) { s_err = "busy"; return false; }
  Axis_t *a = &s_ax[ax];
  a->nolimit = true;
  a->homing = true;
  a->jog_until = 0;
  uint32_t save = a->start_pps;
  a->start_pps = a->home_pps;          /* constant speed: no ramp into the stop */
  axis_start(a, 0, false, a->pos - (int32_t)(a->home_fs * MRES), a->home_pps);
  a->start_pps = save;
  return true;
}

static bool home_cmd(LensAxis_t ax) { return boot_check() && home(ax); }

static void emit(int i, const char *reason)
{
  if (s_evt_fn) s_evt_fn((LensAxis_t)i, s_ax[i].pos, reason);
}

static bool all_idle(void)
{
  for (int i = LENS_ZOOM; i <= LENS_FOCUS; i++)
    if (s_ax[i].running || s_ax[i].homing || s_pend_valid[i] || s_ax[i].evt != EVT_NONE) return false;
  return true;
}

static void boot_task(void)
{
  if (!s_boot || !all_idle()) return;
  if (s_boot == 1) {
    s_boot = 2;
    if (s_ax[LENS_ZOOM].referenced && s_ax[LENS_FOCUS].referenced) {
      move_to(LENS_ZOOM, s_zoom_fixed);
      move_to(LENS_FOCUS, s_focus_best);
    }
    return;
  }
  s_boot = 0;
  emit(LENS_FOCUS, "ready");           /* main.c turns this into READY on the SoC link */
}

void LENS_Task(void)
{
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
        if (i == LENS_ZOOM && s_zoom_locked && !s_boot) move_to(LENS_ZOOM, s_zoom_fixed);
      }
      else emit(i, "home-abort");
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
}

/* ============================================================== init ===== */
static void axis_hw(Axis_t *a, const char *drv, TIM_HandleTypeDef *h, uint32_t ch, volatile uint32_t *ccr,
                    IRQn_Type irq, GPIO_TypeDef *dp, uint16_t dpin, GPIO_TypeDef *ep, uint16_t epin, uint8_t addr)
{
  a->drv = drv; a->htim = h; a->ch = ch; a->ccr = ccr; a->irq = irq;
  a->dir_port = dp; a->dir_pin = dpin; a->en_port = ep; a->en_pin = epin; a->addr = addr;
  a->irun = 31; a->ihold = 0;
  a->start_pps = 200; a->max_pps = 300; a->acc_pps2 = 600;
  a->home_pps = 300; a->home_fs = 1200;
  a->travel = 0;
  a->min_pos = 0; a->max_pos = POS_FAR;
}

static void print_status(LensPrintFn print);

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
  zm->start_pps = 200; zm->max_pps = 300; zm->acc_pps2 = 600;
  zm->home_pps = 300;  zm->home_fs = 1130; zm->travel = 16900;
  fm->start_pps = 200; fm->max_pps = 300; fm->acc_pps2 = 1000;  /* 3.3 V: slow, no lost steps */
  fm->home_pps = 300;  fm->home_fs = 4000; fm->travel = FOCUS_TRAVEL_DEFAULT;

  HAL_GPIO_WritePin(u1->en_port, u1->en_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(u2->en_port, u2->en_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(u5->en_port, u5->en_pin, GPIO_PIN_RESET);
  HAL_Delay(50);

  uint8_t sync = 0;                    /* first byte lets the TMC auto-baud lock */
  HAL_UART_Abort(&huart1);
  HAL_HalfDuplex_EnableTransmitter(&huart1);
  HAL_UART_Transmit(&huart1, &sync, 1, 10);
  HAL_Delay(2);

  drv_config(u1);
  drv_config(u2);
  timer_init(u1);
  timer_init(u2);
  timer_init(u5);

  print("\r\n[LENS] lens control (rewrite 2026-11-17) - \"lens help\"\r\n");
  print_status(print);

  /* Boot: home both, then zoom -> fixed step, focus -> calibrated best. */
  if (home(LENS_ZOOM) && home(LENS_FOCUS)) {
    s_boot = 1;
    print("[LENS] homing both axes (~15 s), then zoom fixed / focus best\r\n");
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
    snprintf(s_buf, sizeof s_buf, "      %s %ld\r\n", s_zoom_locked ? "LOCKED at" : "unlocked, fixed pos", (long)s_zoom_fixed);
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
  print("  lens <ax> home                   run home_fs reverse into the stop, pos := 0\r\n");
  print("  lens <ax> zero                   pos := 0 here (sets reference)\r\n");
  print("  lens <ax> limit <min> <max>|off  soft limits (max -1 = none)\r\n");
  print("  lens <ax> travel <usteps>        forward limit applied by home/zero (0 = none)\r\n");
  print("  lens <ax> homecfg <pps> <fs>     homing speed and distance\r\n");
  print("  lens <ax> speed <start> <max> <acc>   start<=800, max<=1200\r\n");
  print("  lens <ax> cur <irun> [ihold]     0..31 (voltage mode: amplitude = VS*(irun+1)/32)\r\n");
  print("  lens z lock [pos] | unlock      zoom fixed at pos (default: here); locked at boot\r\n");
  print("  lens <ax> reg <r> [val]          raw TMC2300 register read/write\r\n");
  print("  lens f best                      focus to the calibrated best position\r\n");
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
    snprintf(s_buf, sizeof s_buf, "best focus %ld\r\n", (long)s_focus_best); print(s_buf);
    reply_move(LENS_MoveTo(ax, s_focus_best), a, print); return;
  }
  if (ax == LENS_ZOOM && !strcmp(op, "unlock")) { s_zoom_locked = false; print("OK zoom unlocked\r\n"); return; }
  if (ax == LENS_ZOOM && !strcmp(op, "lock")) {
    long p = a->pos;
    if (nt >= 2 && !parse_long(tok[1], &p)) { print("ERROR: lock [pos]\r\n"); return; }
    if ((int32_t)p != a->pos && !(boot_check() && move_to(ax, (int32_t)p))) { snprintf(s_buf, sizeof s_buf, "ERROR: %s\r\n", s_err); print(s_buf); return; }
    s_zoom_fixed = (int32_t)p; s_zoom_locked = true;
    snprintf(s_buf, sizeof s_buf, "OK zoom locked at %ld\r\n", p); print(s_buf); return;
  }
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
    if (a->referenced) a->max_pos = a->travel > 0 ? a->travel : POS_FAR;
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
     FB                     focus to calibrated best -> OK F <pos>
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
    if (!LENS_MoveTo(LENS_FOCUS, s_focus_best)) { host_err(reply); return; }
    snprintf(out, sizeof out, "OK F %ld\n", (long)s_focus_best); reply(out); return;
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
