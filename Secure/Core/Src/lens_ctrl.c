/**
  ******************************************************************************
  * @file    lens_ctrl.c
  * @brief   Zoom / focus (/ iris) stepper control - see lens_ctrl.h.
  *
  * Replaces the bring-up TMC2300.c, which:
  *  - configured only UART node 0 (on the board that is the IRIS driver),
  *  - read registers one byte late (returned data[23:0]<<8 | CRC),
  *  - wrote CHOPCONF 0x10000002 (enabledrv = 0, MRES = 256) and
  *    PWMCONF 0xFF (autoscale off), IRUN = 31 (>10x the lens motor rating),
  *  - had no step counting, no ramp and no position.
  *
  * TMC2300 notes (verify against the datasheet when tuning; every value below
  * can be changed at run time with "lens <ax> reg/cur/mres/..."):
  *  - single-wire UART on PDN_UART, node address from AD0/AD1. Schematic:
  *    zoom U1 = 0, focus U2 = 1, iris U5 = 2 (assembly may differ - "lens scan").
  *  - IFCNT (0x02) counts successful writes - used to prove each write landed.
  *  - CHOPCONF (0x6C): bit0 enabledrv, MRES bits 27:24 (0=256 .. 4=16 .. 8=full),
  *    bit28 intpol. Reset default 0x13008001.
  *  - PWMCONF (0x70): reset default 0xC40D1024 (pwm_autoscale + autograd on),
  *    which makes StealthChop regulate to IRUN.
  *  - IHOLD_IRUN (0x10): IHOLD 4:0, IRUN 12:8, IHOLDDELAY 19:16. Write-only.
  *    Full scale ~ (CS+1)/32 * 0.32 V / Rsense; with the schematic's 150 mOhm
  *    that is ~47 mA RMS per CS step, so IRUN = 1 ~ 94 mA RMS ~ the motor's
  *    rated 100 mA. Re-check if the sense resistors are not fitted.
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

/* ---------------------------------------------------------------- pins --- */
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

/* ------------------------------------------------------ TMC2300 registers */
#define R_GCONF       0x00
#define R_GSTAT       0x01
#define R_IFCNT       0x02
#define R_IOIN        0x06
#define R_IHOLD_IRUN  0x10
#define R_TPOWERDOWN  0x11
#define R_TSTEP       0x12
#define R_TCOOLTHRS   0x14
#define R_VACTUAL     0x22
#define R_SGTHRS      0x40
#define R_SG_VALUE    0x41
#define R_CHOPCONF    0x6C
#define R_DRV_STATUS  0x6F
#define R_PWMCONF     0x70
#define R_PWM_SCALE   0x71
#define R_PWM_AUTO    0x72

#define CHOPCONF_DEFAULT  0x13008001u   /* TMC2300 reset value            */
#define PWMCONF_DEFAULT   0xC40D1024u   /* autoscale + autograd on        */

/* --------------------------------------------- lens motor limits (datasheet) */
#define LENS_START_PPS_MAX   800u    /* max starting frequency, full steps */
#define LENS_SLEW_PPS_MAX   1200u    /* max slewing frequency, full steps  */

#define DEF_START_PPS        400u
#define DEF_MAX_PPS         1000u
#define DEF_ACC_PPS2        2000u
#define DEF_MRES              16u
#define DEF_IRUN               1u
#define DEF_IHOLD              0u
#define DEF_IHOLDDELAY         4u

#define TICK_HZ          1000000.0f  /* TIM3/4/8: 64 MHz / (63+1)          */
#define ARR_MIN                9u    /* 100 kHz cap on the STEP output     */
#define ARR_MAX            65535u
#define MOTOR_IRQ_PRIO         3u    /* below IR (0..2) and console UART   */

#define POS_FAR   (INT32_MAX / 2)

/* -------------------------------------------------------------- types --- */
typedef enum { EVT_NONE = 0, EVT_DONE, EVT_STOP, EVT_LIMIT, EVT_STALL } LensEvt_t;
static const char *const s_evt_name[] = { "", "done", "stop", "limit", "stall" };

typedef struct {
  /* hardware */
  TIM_HandleTypeDef *htim;
  uint32_t           ch;
  volatile uint32_t *ccr;
  GPIO_TypeDef *dir_port;  uint16_t dir_pin;
  GPIO_TypeDef *en_port;   uint16_t en_pin;
  GPIO_TypeDef *diag_port; uint16_t diag_pin;
  /* configuration */
  uint8_t  addr;
  uint8_t  present;          /* answered on the TMC UART at last init     */
  uint8_t  configured;       /* every write acknowledged by IFCNT         */
  uint16_t mres;             /* microsteps per full step                  */
  uint8_t  irun, ihold, iholddelay;
  uint8_t  sg_thr;           /* 0 = stall detection off                   */
  uint32_t start_pps, max_pps, acc_pps2;
  int32_t  min_pos, max_pos;
  /* motion state (shared with ISR) */
  volatile int32_t  pos;
  volatile int32_t  target;
  volatile int8_t   dir;     /* +1 / -1 while running                     */
  volatile uint8_t  running;
  volatile uint8_t  jog;
  volatile uint8_t  stop_req;
  volatile uint8_t  evt;
  float    v, v_start, v_max, acc2;   /* microsteps/s, (microsteps/s)^2 *2 */
  /* queued move (issued while the axis had to stop/reverse first) */
  uint8_t  pend_valid;
  int32_t  pend_target;
} Axis_t;

const char *const LENS_AxisName[LENS_NAXIS] = { "zoom", "focus", "iris" };

static Axis_t s_ax[LENS_NAXIS];
static LensEventFn s_evt_fn;
static char s_buf[128];

/* ============================================================ TMC UART === */
static void tmc_crc(uint8_t *d, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len - 1u; i++) {
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
                   (uint8_t)(val >> 24), (uint8_t)(val >> 16),
                   (uint8_t)(val >> 8),  (uint8_t)val, 0 };
  tmc_crc(d, 8);
  HAL_UART_Abort(&huart1);
  HAL_HalfDuplex_EnableTransmitter(&huart1);
  HAL_UART_Transmit(&huart1, d, 8, 50);
}

/* Reply: sync 0x05, master 0xFF, reg, data[31:24..7:0], CRC. */
static bool tmc_read(uint8_t addr, uint8_t reg, uint32_t *val)
{
  uint8_t tx[4] = { 0x05, addr, (uint8_t)(reg & 0x7Fu), 0 };
  uint8_t rx[8] = { 0 };
  tmc_crc(tx, 4);
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

static uint8_t mres_code(uint16_t mres)
{
  switch (mres) {
    case 256: return 0; case 128: return 1; case 64: return 2; case 32: return 3;
    case 16:  return 4; case 8:   return 5; case 4:  return 6; case 2:  return 7;
    default:  return 8;   /* full step */
  }
}

static uint32_t axis_chopconf(const Axis_t *a)
{
  return (CHOPCONF_DEFAULT & ~(0xFu << 24)) | ((uint32_t)mres_code(a->mres) << 24) | 1u;
}

/* Writes the full profile and proves it landed: IFCNT must advance by the
   number of writes and CHOPCONF must read back. */
static bool axis_configure(Axis_t *a)
{
  uint32_t c0, c1, chop;
  a->present = tmc_read(a->addr, R_IFCNT, &c0);
  a->configured = 0;
  if (!a->present) return false;

  uint32_t ihr = ((uint32_t)a->iholddelay << 16) | ((uint32_t)a->irun << 8) | a->ihold;
  tmc_write(a->addr, R_GSTAT, 0x7u);                 /* clear reset/drv_err/uv */
  tmc_write(a->addr, R_IHOLD_IRUN, ihr);
  tmc_write(a->addr, R_TPOWERDOWN, 20u);
  tmc_write(a->addr, R_PWMCONF, PWMCONF_DEFAULT);
  tmc_write(a->addr, R_TCOOLTHRS, a->sg_thr ? 0xFFFFFu : 0u);
  tmc_write(a->addr, R_SGTHRS, a->sg_thr);
  tmc_write(a->addr, R_CHOPCONF, axis_chopconf(a));  /* last: enables the driver */
  const uint8_t n_writes = 7;

  if (!tmc_read(a->addr, R_IFCNT, &c1)) return false;
  if (!tmc_read(a->addr, R_CHOPCONF, &chop)) return false;
  a->configured = (((c1 - c0) & 0xFFu) == n_writes) && (chop == axis_chopconf(a));
  return a->configured;
}

/* ============================================================ motion ===== */
static void axis_rates(Axis_t *a)
{
  a->v_start = (float)a->start_pps * a->mres;
  a->v_max   = (float)a->max_pps   * a->mres;
  a->acc2    = 2.0f * (float)a->acc_pps2 * a->mres;
}

static inline uint32_t rate_to_arr(float v)
{
  float t = TICK_HZ / v;
  uint32_t arr = (t > (float)(ARR_MAX + 1u)) ? ARR_MAX : (uint32_t)t - 1u;
  return (arr < ARR_MIN) ? ARR_MIN : arr;
}

static inline void tim_load(Axis_t *a, uint32_t arr)
{
  a->htim->Instance->ARR = arr;
  *a->ccr = (arr + 1u) / 2u;          /* PWM2: STEP high for 2nd half */
}

/* ISR context (or IRQs masked). Stops the counter; if the pulse of the period
   that had just begun already went high, it counts as a step. */
static void axis_finish(Axis_t *a, LensEvt_t why)
{
  TIM_TypeDef *T = a->htim->Instance;
  T->CR1 &= ~TIM_CR1_CEN;
  if (T->CNT >= *a->ccr) a->pos += a->dir;
  T->CNT = 0;
  a->running = 0;
  a->stop_req = 0;
  a->evt = (uint8_t)why;
}

static void axis_start(Axis_t *a, int dir, bool jog, int32_t target)
{
  TIM_TypeDef *T = a->htim->Instance;
  a->dir = (int8_t)(dir > 0 ? 1 : -1);
  HAL_GPIO_WritePin(a->dir_port, a->dir_pin, dir > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(a->en_port, a->en_pin, GPIO_PIN_SET);
  for (volatile int i = 0; i < 20; i++) { __NOP(); }   /* DIR setup time */

  a->jog = jog ? 1u : 0u;
  a->target = target;
  a->stop_req = 0;
  a->evt = EVT_NONE;
  a->v = a->v_start;

  T->CR1 &= ~TIM_CR1_CEN;
  T->CNT = 0;
  tim_load(a, rate_to_arr(a->v));
  T->EGR = TIM_EGR_UG;                 /* URS=1: no update IRQ from this */
  T->SR = ~TIM_SR_UIF;
  a->running = 1;
  T->CR1 |= TIM_CR1_CEN;
}

void LENS_TimerIRQ(LensAxis_t ax)
{
  Axis_t *a = &s_ax[ax];
  TIM_TypeDef *T = a->htim->Instance;
  if ((T->SR & TIM_SR_UIF) == 0u) return;
  T->SR = ~TIM_SR_UIF;
  if (!a->running) { T->CR1 &= ~TIM_CR1_CEN; return; }

  /* one complete STEP pulse just ended */
  int32_t pos = a->pos + a->dir;
  a->pos = pos;

  /* distance left: to target (move) and to the soft limit (both modes) */
  int32_t to_lim = (a->dir > 0) ? (a->max_pos - pos) : (pos - a->min_pos);
  int32_t rem = a->jog ? to_lim : (a->target - pos) * a->dir;
  if (!a->jog && rem > to_lim) rem = to_lim;

  if (rem <= 0) {
    LensEvt_t why = (!a->jog && pos == a->target) ? EVT_DONE : EVT_LIMIT;
    axis_finish(a, why);
    return;
  }

  float v2 = a->v * a->v;
  float vs2 = a->v_start * a->v_start;
  float dec_steps = (v2 - vs2) / a->acc2;          /* steps needed to slow to v_start */

  if (a->stop_req || (float)rem <= dec_steps + 1.0f) {
    v2 -= a->acc2;
    if (v2 <= vs2) {
      v2 = vs2;
      if (a->stop_req) { axis_finish(a, EVT_STOP); return; }
    }
  } else if (a->v < a->v_max) {
    v2 += a->acc2;
    if (v2 > a->v_max * a->v_max) v2 = a->v_max * a->v_max;
  } else if (a->v > a->v_max) {                    /* speed lowered mid-move */
    v2 -= a->acc2;
    if (v2 < a->v_max * a->v_max) v2 = a->v_max * a->v_max;
  }
  a->v = sqrtf(v2);
  tim_load(a, rate_to_arr(a->v));                  /* applies from the period after next */
}

static bool ax_ok(LensAxis_t ax) { return (unsigned)ax < LENS_NAXIS; }

bool LENS_MoveTo(LensAxis_t ax, int32_t pos)
{
  if (!ax_ok(ax)) return false;
  Axis_t *a = &s_ax[ax];
  if (pos < a->min_pos || pos > a->max_pos) return false;

  if (a->running) {
    /* Same direction and still ahead of where a decelerated stop would land:
       retarget on the fly. Otherwise stop first and queue the move. */
    int32_t ahead = (pos - a->pos) * a->dir;
    float dec = (a->v * a->v - a->v_start * a->v_start) / a->acc2;
    if (!a->jog && ahead > (int32_t)dec + 2) {
      a->target = pos;
      return true;
    }
    a->pend_target = pos;
    a->pend_valid = 1;
    a->stop_req = 1;
    return true;
  }
  a->pend_valid = 0;
  if (pos == a->pos) { a->evt = EVT_DONE; return true; }
  axis_start(a, pos > a->pos ? 1 : -1, false, pos);
  return true;
}

bool LENS_MoveRel(LensAxis_t ax, int32_t delta)
{
  if (!ax_ok(ax)) return false;
  int32_t base = s_ax[ax].pend_valid ? s_ax[ax].pend_target :
                 (s_ax[ax].running && !s_ax[ax].jog) ? s_ax[ax].target : s_ax[ax].pos;
  return LENS_MoveTo(ax, base + delta);
}

bool LENS_Jog(LensAxis_t ax, int dir, uint32_t pps)
{
  if (!ax_ok(ax)) return false;
  Axis_t *a = &s_ax[ax];
  if (pps) {
    if (pps > LENS_SLEW_PPS_MAX) pps = LENS_SLEW_PPS_MAX;
    a->max_pps = pps;
    if (a->start_pps > pps) a->start_pps = pps;
    axis_rates(a);
  }
  int d = dir ? 1 : -1;
  if (a->running) {
    if (a->dir == d) { a->jog = 1; return true; }     /* same way: keep going */
    a->stop_req = 1;                                    /* reverse: stop first  */
    return false;
  }
  if ((d > 0 && a->pos >= a->max_pos) || (d < 0 && a->pos <= a->min_pos)) return false;
  a->pend_valid = 0;
  axis_start(a, d, true, 0);
  return true;
}

void LENS_Stop(LensAxis_t ax)
{
  if (!ax_ok(ax)) return;
  s_ax[ax].pend_valid = 0;
  if (s_ax[ax].running) s_ax[ax].stop_req = 1;
}

void LENS_Halt(LensAxis_t ax)
{
  if (!ax_ok(ax)) return;
  Axis_t *a = &s_ax[ax];
  a->pend_valid = 0;
  uint32_t pm = __get_PRIMASK();
  __disable_irq();
  if (a->running) axis_finish(a, EVT_STOP);
  __set_PRIMASK(pm);
}

bool    LENS_IsMoving(LensAxis_t ax) { return ax_ok(ax) && (s_ax[ax].running || s_ax[ax].pend_valid); }
int32_t LENS_GetPos(LensAxis_t ax)   { return ax_ok(ax) ? s_ax[ax].pos : 0; }
void    LENS_SetEventHandler(LensEventFn fn) { s_evt_fn = fn; }

void LENS_Task(void)
{
  for (int i = 0; i < LENS_NAXIS; i++) {
    Axis_t *a = &s_ax[i];

    if (a->running && a->sg_thr &&
        HAL_GPIO_ReadPin(a->diag_port, a->diag_pin) == GPIO_PIN_SET) {
      uint32_t pm = __get_PRIMASK();
      __disable_irq();
      if (a->running) axis_finish(a, EVT_STALL);
      __set_PRIMASK(pm);
      a->pend_valid = 0;
    }

    uint8_t evt = a->evt;
    if (evt != EVT_NONE && !a->running) {
      a->evt = EVT_NONE;
      if (a->pend_valid && evt == EVT_STOP) {          /* queued move: go now */
        a->pend_valid = 0;
        int32_t t = a->pend_target;
        if (t != a->pos) { axis_start(a, t > a->pos ? 1 : -1, false, t); continue; }
        evt = EVT_DONE;
      }
      if (s_evt_fn) s_evt_fn((LensAxis_t)i, a->pos, s_evt_name[evt]);
    }
  }
}

/* ============================================================ init ======= */
static void axis_timer_init(Axis_t *a, IRQn_Type irq)
{
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM2;
  oc.Pulse = 500;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_Stop(a->htim, a->ch);
  HAL_TIM_PWM_ConfigChannel(a->htim, &oc, a->ch);   /* also sets OCxPE (preload) */

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
  HAL_NVIC_SetPriority(irq, MOTOR_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(irq);
}

static void axis_defaults(Axis_t *a, uint8_t addr)
{
  a->addr = addr;
  a->mres = DEF_MRES;
  a->irun = DEF_IRUN; a->ihold = DEF_IHOLD; a->iholddelay = DEF_IHOLDDELAY;
  a->sg_thr = 0;
  a->start_pps = DEF_START_PPS; a->max_pps = DEF_MAX_PPS; a->acc_pps2 = DEF_ACC_PPS2;
  a->min_pos = 0;          /* power-up position = lens initial/zero position */
  a->max_pos = POS_FAR;    /* travel unknown until measured: "lens <ax> limit" */
  a->pos = 0;
  axis_rates(a);
}

static void scan_report(LensPrintFn print)
{
  for (uint8_t addr = 0; addr < 4u; addr++) {
    uint32_t io;
    if (tmc_read(addr, R_IOIN, &io)) {
      const char *who = "?";
      for (int i = 0; i < LENS_NAXIS; i++) if (s_ax[i].addr == addr) who = LENS_AxisName[i];
      snprintf(s_buf, sizeof s_buf, "[LENS] TMC2300 @%u answers: IOIN=0x%08lX ver=0x%02lX (mapped: %s)\r\n",
               addr, (unsigned long)io, (unsigned long)(io >> 24), who);
    } else {
      snprintf(s_buf, sizeof s_buf, "[LENS] TMC2300 @%u: no reply\r\n", addr);
    }
    print(s_buf);
  }
}

static void config_report(LensPrintFn print)
{
  for (int i = 0; i < LENS_NAXIS; i++) {
    Axis_t *a = &s_ax[i];
    axis_configure(a);
    snprintf(s_buf, sizeof s_buf, "[LENS] %-5s @%u: %s\r\n", LENS_AxisName[i], a->addr,
             !a->present ? "NO REPLY - driver not configured (check M1/M2_PWR, AD0/AD1)" :
             a->configured ? "configured" : "replied but config NOT confirmed");
    print(s_buf);
  }
}

void LENS_Init(LensPrintFn print)
{
  GPIO_InitTypeDef g = {0};

  /* PA10 (USART1_RX) must not fight the single-wire PA9 bus */
  g.Pin = GPIO_PIN_10; g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);
  g.Mode = GPIO_MODE_INPUT;
  g.Pin = ZOOM_DIAG_Pin;  HAL_GPIO_Init(ZOOM_DIAG_GPIO_Port, &g);
  g.Pin = FOCUS_DIAG_Pin; HAL_GPIO_Init(FOCUS_DIAG_GPIO_Port, &g);
  g.Pin = IRIS_DIAG_Pin;  HAL_GPIO_Init(IRIS_DIAG_GPIO_Port, &g);

  Axis_t *z = &s_ax[LENS_ZOOM], *f = &s_ax[LENS_FOCUS], *ir = &s_ax[LENS_IRIS];
  memset(s_ax, 0, sizeof s_ax);

  z->htim = &htim3; z->ch = TIM_CHANNEL_2; z->ccr = &TIM3->CCR2;
  z->dir_port = ZOOM_DIR_GPIO_Port; z->dir_pin = ZOOM_DIR_Pin;
  z->en_port = ZOOM_EN_GPIO_Port;   z->en_pin = ZOOM_EN_Pin;
  z->diag_port = ZOOM_DIAG_GPIO_Port; z->diag_pin = ZOOM_DIAG_Pin;

  f->htim = &htim8; f->ch = TIM_CHANNEL_2; f->ccr = &TIM8->CCR2;
  f->dir_port = FOCUS_DIR_GPIO_Port; f->dir_pin = FOCUS_DIR_Pin;
  f->en_port = FOCUS_EN_GPIO_Port;   f->en_pin = FOCUS_EN_Pin;
  f->diag_port = FOCUS_DIAG_GPIO_Port; f->diag_pin = FOCUS_DIAG_Pin;

  ir->htim = &htim4; ir->ch = TIM_CHANNEL_1; ir->ccr = &TIM4->CCR1;
  ir->dir_port = IRIS_DIR_GPIO_Port; ir->dir_pin = IRIS_DIR_Pin;
  ir->en_port = IRIS_EN_GPIO_Port;   ir->en_pin = IRIS_EN_Pin;
  ir->diag_port = IRIS_DIAG_GPIO_Port; ir->diag_pin = IRIS_DIAG_Pin;

  axis_defaults(z, 0);    /* schematic: zoom U1 ADDR 00  */
  axis_defaults(f, 1);    /*            focus U2 ADDR 01 */
  axis_defaults(ir, 2);   /*            iris U5 ADDR 10  */

  for (int i = 0; i < LENS_NAXIS; i++)
    HAL_GPIO_WritePin(s_ax[i].en_port, s_ax[i].en_pin, GPIO_PIN_SET);
  HAL_Delay(50);

  uint8_t dummy = 0;                   /* first byte syncs the TMC auto-baud */
  HAL_UART_Abort(&huart1);
  HAL_HalfDuplex_EnableTransmitter(&huart1);
  HAL_UART_Transmit(&huart1, &dummy, 1, 10);
  HAL_Delay(2);

  axis_timer_init(z, TIM3_IRQn);
  axis_timer_init(f, TIM8_UP_IRQn);
  axis_timer_init(ir, TIM4_IRQn);

  print("\r\n[LENS] NV-0622IR lens: 16 ustep, start 400 / max 1000 pps, IRUN 1\r\n");
  scan_report(print);
  config_report(print);
}

/* ============================================================ console ==== */
static int parse_axis(const char *s)
{
  if (!s) return -1;
  if (!strcmp(s, "z") || !strcmp(s, "zoom"))  return LENS_ZOOM;
  if (!strcmp(s, "f") || !strcmp(s, "focus")) return LENS_FOCUS;
  if (!strcmp(s, "i") || !strcmp(s, "iris"))  return LENS_IRIS;
  return -1;
}

static bool parse_long(const char *s, long *out)
{
  if (!s || !*s) return false;
  char *e;
  long v = strtol(s, &e, 0);
  if (*e) return false;
  *out = v;
  return true;
}

static void print_status(LensPrintFn print)
{
  for (int i = 0; i < LENS_NAXIS; i++) {
    Axis_t *a = &s_ax[i];
    snprintf(s_buf, sizeof s_buf,
             "%-5s pos %ld (%ld.%02ld fs) %s  lim %ld..%ld  @%u %s mres %u irun %u ihold %u\r\n",
             LENS_AxisName[i], (long)a->pos, (long)(a->pos / (int32_t)a->mres),
             (long)labs((a->pos % (int32_t)a->mres) * 100 / (int32_t)a->mres),
             a->running ? (a->jog ? "JOG" : "MOVING") : "idle",
             (long)a->min_pos, (long)(a->max_pos == POS_FAR ? -1 : a->max_pos),
             a->addr, a->configured ? "cfg" : (a->present ? "NOCFG" : "ABSENT"),
             a->mres, a->irun, a->ihold);
    print(s_buf);
    snprintf(s_buf, sizeof s_buf, "      speed start %lu max %lu pps, acc %lu pps2, stall %s (sgthrs %u)\r\n",
             (unsigned long)a->start_pps, (unsigned long)a->max_pps, (unsigned long)a->acc_pps2,
             a->sg_thr ? "on" : "off", a->sg_thr);
    print(s_buf);
  }
}

static void dump_axis(Axis_t *a, LensPrintFn print)
{
  static const struct { uint8_t r; const char *n; } regs[] = {
    {R_GCONF, "GCONF"}, {R_GSTAT, "GSTAT"}, {R_IFCNT, "IFCNT"}, {R_IOIN, "IOIN"},
    {R_TSTEP, "TSTEP"}, {R_SG_VALUE, "SG_VALUE"}, {R_CHOPCONF, "CHOPCONF"},
    {R_DRV_STATUS, "DRV_STATUS"}, {R_PWMCONF, "PWMCONF"}, {R_PWM_SCALE, "PWM_SCALE"},
    {R_PWM_AUTO, "PWM_AUTO"},
  };
  snprintf(s_buf, sizeof s_buf, "TMC2300 @%u (write-only regs IHOLD_IRUN/TCOOLTHRS/SGTHRS not readable)\r\n", a->addr);
  print(s_buf);
  for (unsigned i = 0; i < sizeof regs / sizeof regs[0]; i++) {
    uint32_t v;
    if (tmc_read(a->addr, regs[i].r, &v))
      snprintf(s_buf, sizeof s_buf, "  %-10s 0x%02X = 0x%08lX\r\n", regs[i].n, regs[i].r, (unsigned long)v);
    else
      snprintf(s_buf, sizeof s_buf, "  %-10s 0x%02X : no reply / CRC error\r\n", regs[i].n, regs[i].r);
    print(s_buf);
  }
}

static void print_help(LensPrintFn print)
{
  print("lens commands (<ax> = z|zoom f|focus i|iris; pos in microsteps, speeds in full steps/s):\r\n");
  print("  lens [status]                   positions, state, driver config\r\n");
  print("  lens scan | init                probe TMC2300 @0..3 / re-write driver config\r\n");
  print("  lens <ax> goto <pos>            absolute move (ramped)\r\n");
  print("  lens <ax> rel <+-steps>         relative move\r\n");
  print("  lens <ax> jog <1|0> [pps]       run fwd(1)/rev(0) until stop or soft limit\r\n");
  print("  lens <ax> stop | lens stop      decelerated stop (halt = immediate)\r\n");
  print("  lens <ax> zero                  current position := 0\r\n");
  print("  lens <ax> limit <min> <max>     soft limits (max -1 = none)\r\n");
  print("  lens <ax> speed <start> <max> <acc>  pps, pps, pps^2 (start<=800, max<=1200)\r\n");
  print("  lens <ax> cur <irun> [ihold]    0..31 (IRUN 1 ~ 94 mA RMS with 150 mOhm)\r\n");
  print("  lens <ax> mres <1..64>          microsteps per full step (rescales pos)\r\n");
  print("  lens <ax> addr <0..3>           TMC2300 UART node address\r\n");
  print("  lens <ax> sg <0..255>           StallGuard threshold, 0 = off\r\n");
  print("  lens <ax> dump | reg <r> [val]  driver registers\r\n");
  print("  old: zoom_run/focus_run/iris_run <dir> <step_hz>, *_stop, focus_dump\r\n");
}

static void cli_axis(Axis_t *a, LensAxis_t ax, char **tok, int nt, LensPrintFn print)
{
  const char *op = nt > 0 ? tok[0] : "status";
  long v1 = 0, v2 = 0, v3 = 0;
  bool ok = true;

  if (!strcmp(op, "goto") && nt >= 2 && parse_long(tok[1], &v1)) {
    ok = LENS_MoveTo(ax, (int32_t)v1);
  } else if (!strcmp(op, "rel") && nt >= 2 && parse_long(tok[1], &v1)) {
    ok = LENS_MoveRel(ax, (int32_t)v1);
  } else if (!strcmp(op, "jog") && nt >= 2 && parse_long(tok[1], &v1)) {
    if (nt >= 3 && !parse_long(tok[2], &v2)) { print("ERROR: pps\r\n"); return; }
    ok = LENS_Jog(ax, (int)v1, (uint32_t)v2);
  } else if (!strcmp(op, "stop")) {
    LENS_Stop(ax);
  } else if (!strcmp(op, "halt")) {
    LENS_Halt(ax);
  } else if (!strcmp(op, "zero")) {
    if (a->running) { print("ERROR: moving\r\n"); return; }
    a->pos = 0;
  } else if (!strcmp(op, "limit") && nt >= 3 && parse_long(tok[1], &v1) && parse_long(tok[2], &v2)) {
    a->min_pos = (int32_t)v1;
    a->max_pos = (v2 < 0) ? POS_FAR : (int32_t)v2;
  } else if (!strcmp(op, "speed") && nt >= 4 && parse_long(tok[1], &v1) && parse_long(tok[2], &v2) &&
             parse_long(tok[3], &v3)) {
    if (v1 < 20 || v1 > (long)LENS_START_PPS_MAX || v2 < v1 || v2 > (long)LENS_SLEW_PPS_MAX || v3 < 10) {
      print("ERROR: need 20<=start<=800, start<=max<=1200, acc>=10\r\n"); return;
    }
    a->start_pps = (uint32_t)v1; a->max_pps = (uint32_t)v2; a->acc_pps2 = (uint32_t)v3;
    axis_rates(a);
  } else if (!strcmp(op, "cur") && nt >= 2 && parse_long(tok[1], &v1)) {
    if (nt >= 3 && !parse_long(tok[2], &v2)) { print("ERROR: ihold\r\n"); return; }
    if (v1 < 0 || v1 > 31 || v2 < 0 || v2 > 31) { print("ERROR: 0..31\r\n"); return; }
    a->irun = (uint8_t)v1; a->ihold = (uint8_t)v2;
    ok = axis_configure(a);
  } else if (!strcmp(op, "mres") && nt >= 2 && parse_long(tok[1], &v1)) {
    if (a->running) { print("ERROR: moving\r\n"); return; }
    if (v1 != 1 && v1 != 2 && v1 != 4 && v1 != 8 && v1 != 16 && v1 != 32 && v1 != 64) {
      print("ERROR: 1,2,4,8,16,32,64\r\n"); return;
    }
    uint16_t old = a->mres;
    a->pos = (int32_t)((int64_t)a->pos * v1 / old);
    if (a->max_pos != POS_FAR) a->max_pos = (int32_t)((int64_t)a->max_pos * v1 / old);
    a->min_pos = (int32_t)((int64_t)a->min_pos * v1 / old);
    a->mres = (uint16_t)v1;
    axis_rates(a);
    ok = axis_configure(a);
  } else if (!strcmp(op, "addr") && nt >= 2 && parse_long(tok[1], &v1) && v1 >= 0 && v1 <= 3) {
    a->addr = (uint8_t)v1;
    ok = axis_configure(a);
  } else if (!strcmp(op, "sg") && nt >= 2 && parse_long(tok[1], &v1) && v1 >= 0 && v1 <= 255) {
    a->sg_thr = (uint8_t)v1;
    ok = axis_configure(a);
  } else if (!strcmp(op, "dump")) {
    dump_axis(a, print); return;
  } else if (!strcmp(op, "reg") && nt >= 2 && parse_long(tok[1], &v1)) {
    uint32_t r;
    if (nt >= 3 && parse_long(tok[2], &v2)) tmc_write(a->addr, (uint8_t)v1, (uint32_t)v2);
    if (tmc_read(a->addr, (uint8_t)v1, &r))
      snprintf(s_buf, sizeof s_buf, "reg 0x%02lX = 0x%08lX\r\n", (unsigned long)v1, (unsigned long)r);
    else
      snprintf(s_buf, sizeof s_buf, "reg 0x%02lX: no reply (write-only or absent)\r\n", (unsigned long)v1);
    print(s_buf); return;
  } else if (!strcmp(op, "status")) {
    /* fall through to the status line */
  } else {
    print("ERROR: lens help\r\n"); return;
  }

  snprintf(s_buf, sizeof s_buf, "%s %s: pos %ld%s%s\r\n", ok ? "OK" : "REJECTED", LENS_AxisName[ax],
           (long)a->pos, a->running ? " moving" : "",
           (!ok && !a->present) ? " (driver not answering)" : "");
  print(s_buf);
}

bool LENS_CLI(const char *cmd, LensPrintFn print)
{
  char line[96];
  strncpy(line, cmd, sizeof line - 1);
  line[sizeof line - 1] = 0;
  for (char *p = line; *p; p++) *p = (char)tolower((unsigned char)*p);

  char *tok[8]; int nt = 0;
  for (char *t = strtok(line, " \t"); t && nt < 8; t = strtok(NULL, " \t")) tok[nt++] = t;
  if (nt == 0) return false;

  /* legacy bring-up commands */
  const char *legacy[] = { "zoom", "focus", "iris" };
  for (int i = 0; i < LENS_NAXIS; i++) {
    size_t n = strlen(legacy[i]);
    if (!strncmp(tok[0], legacy[i], n) && tok[0][n] == '_') {
      const char *suf = tok[0] + n + 1;
      long dir, hz;
      if (!strcmp(suf, "run") && nt >= 3 && parse_long(tok[1], &dir) && parse_long(tok[2], &hz) && hz > 0) {
        /* old units: STEP pulses per second -> full steps per second */
        uint32_t pps = (uint32_t)hz / s_ax[i].mres;
        if (pps < 20u) pps = 20u;
        bool ok = LENS_Jog((LensAxis_t)i, (int)dir, pps);
        snprintf(s_buf, sizeof s_buf, "%s: %s jog dir %ld at %lu pps (%lu step/s)\r\n", ok ? "SUCCESS" : "REJECTED",
                 LENS_AxisName[i], dir, (unsigned long)s_ax[i].max_pps,
                 (unsigned long)(s_ax[i].max_pps * s_ax[i].mres));
        print(s_buf);
        return true;
      }
      if (!strcmp(suf, "stop")) {
        LENS_Stop((LensAxis_t)i);
        snprintf(s_buf, sizeof s_buf, "SUCCESS: %s stopping\r\n", LENS_AxisName[i]);
        print(s_buf);
        return true;
      }
      if (!strcmp(suf, "dump")) { scan_report(print); dump_axis(&s_ax[i], print); return true; }
      return false;
    }
  }

  if (strcmp(tok[0], "lens") != 0) return false;
  if (nt == 1 || !strcmp(tok[1], "status")) { print_status(print); return true; }
  if (!strcmp(tok[1], "help")) { print_help(print); return true; }
  if (!strcmp(tok[1], "scan")) { scan_report(print); return true; }
  if (!strcmp(tok[1], "init")) { config_report(print); return true; }
  if (!strcmp(tok[1], "stop")) {
    for (int i = 0; i < LENS_NAXIS; i++) LENS_Stop((LensAxis_t)i);
    print("OK stopping all\r\n"); return true;
  }
  int ax = parse_axis(tok[1]);
  if (ax < 0) { print("ERROR: axis z|f|i (lens help)\r\n"); return true; }
  cli_axis(&s_ax[ax], (LensAxis_t)ax, &tok[2], nt - 2, print);
  return true;
}

/* ====================================================== SOC host link ==== */
/* Line protocol on UART4 / J12, 115200 8N1, '\n' or '\r' terminated, ASCII,
   case-insensitive. Positions in microsteps (default 16 per full step).
     Z <pos>    F <pos>       absolute move        -> OK Z <target>
     ZR <d>     FR <d>        relative move        -> OK Z <target>
     ZS  FS  S                decelerated stop     -> OK STOP
     P  or  ?                 position query       -> POS Z <pos> <m> F <pos> <m>
     ZERO Z|F|ALL             set position to 0    -> OK ZERO
     V                        version              -> VER PROXY-LENS 1
   Errors: ERR SYNTAX | ERR RANGE | ERR BUSY | ERR NODRV
   Unsolicited on every stop:  DONE Z <pos> <done|stop|limit|stall>        */
void LENS_HostCommand(const char *line, LensPrintFn reply)
{
  char l[48];
  size_t n = 0;
  for (const char *p = line; *p && n < sizeof l - 1; p++) l[n++] = (char)toupper((unsigned char)*p);
  l[n] = 0;
  char *tok[3]; int nt = 0;
  for (char *t = strtok(l, " \t,"); t && nt < 3; t = strtok(NULL, " \t,")) tok[nt++] = t;
  if (nt == 0) return;

  char out[64];
  const char *c = tok[0];
  long v;

  if (!strcmp(c, "P") || !strcmp(c, "?")) {
    snprintf(out, sizeof out, "POS Z %ld %d F %ld %d\n",
             (long)s_ax[LENS_ZOOM].pos, LENS_IsMoving(LENS_ZOOM),
             (long)s_ax[LENS_FOCUS].pos, LENS_IsMoving(LENS_FOCUS));
    reply(out); return;
  }
  if (!strcmp(c, "V")) { reply("VER PROXY-LENS 1\n"); return; }
  if (!strcmp(c, "S")) { LENS_Stop(LENS_ZOOM); LENS_Stop(LENS_FOCUS); reply("OK STOP\n"); return; }
  if (!strcmp(c, "ZERO")) {
    const char *w = nt > 1 ? tok[1] : "ALL";
    if (LENS_IsMoving(LENS_ZOOM) || LENS_IsMoving(LENS_FOCUS)) { reply("ERR BUSY\n"); return; }
    if (!strcmp(w, "Z") || !strcmp(w, "ALL")) s_ax[LENS_ZOOM].pos = 0;
    if (!strcmp(w, "F") || !strcmp(w, "ALL")) s_ax[LENS_FOCUS].pos = 0;
    reply("OK ZERO\n"); return;
  }

  LensAxis_t ax;
  if (c[0] == 'Z') ax = LENS_ZOOM; else if (c[0] == 'F') ax = LENS_FOCUS;
  else { reply("ERR SYNTAX\n"); return; }
  const char *sub = c + 1;

  if (!strcmp(sub, "S")) { LENS_Stop(ax); reply("OK STOP\n"); return; }
  if ((strcmp(sub, "") && strcmp(sub, "R")) || nt < 2 || !parse_long(tok[1], &v)) {
    reply("ERR SYNTAX\n"); return;
  }
  if (!s_ax[ax].configured) { reply("ERR NODRV\n"); return; }
  bool ok = (*sub == 'R') ? LENS_MoveRel(ax, (int32_t)v) : LENS_MoveTo(ax, (int32_t)v);
  if (!ok) { reply("ERR RANGE\n"); return; }
  snprintf(out, sizeof out, "OK %c %ld\n", ax == LENS_ZOOM ? 'Z' : 'F',
           (long)(s_ax[ax].pend_valid ? s_ax[ax].pend_target :
                  s_ax[ax].running ? s_ax[ax].target : s_ax[ax].pos));
  reply(out);
}
