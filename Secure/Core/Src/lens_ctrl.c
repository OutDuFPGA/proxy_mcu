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
  *    I_RMS = (CS+1)/32 * 0.325 V / (Rsense + 30 mOhm) / sqrt2 (datasheet 9);
  *    150 mOhm -> ~40 mA RMS per CS step. Datasheet: IRUN < 8 not recommended.
  *    The 40 ohm lens coils cap the current at ~VS/40 anyway (~80 mA at 3.3 V,
  *    ~125 mA at 5 V), so IRUN 16 is coil-limited, not sense-limited.
  *  - GSTAT bit2 u3v5 is a LIVE comparator: VS < 3.5 V (3.15..3.5 V falling).
  *    The motor is rated 4.0 V +-20 %, so VS ~3.3 V gives marginal torque.
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
#define DEF_IRUN              16u   /* datasheet: IRUN < 8 not recommended */
#define DEF_IHOLD              0u
#define DEF_IHOLDDELAY         4u

#define TICK_HZ          1000000.0f  /* TIM3/4/8: 64 MHz / (63+1)          */
#define ARR_MIN                9u    /* 100 kHz cap on the STEP output     */
#define ARR_MAX            65535u
#define MOTOR_IRQ_PRIO         3u    /* below IR (0..2) and console UART   */

#define POS_FAR   (INT32_MAX / 2)

/* --------------------------------------------------- guard-rail tunables --
   See docs/lens-control-design.md. All of these are also run-time settable. */
#define DEF_HOME_PPS          300u    /* homing speed, full steps/s (<= start)    */
#define DEF_HOME_MAX_FS      2500u    /* max homing travel, full steps (> travel) */
#define DEF_HOME_BACKOFF_FS     8u    /* zero = this far off the hard stop        */
#define DEF_TRAVEL_FS        2500u    /* forward limit until travel is calibrated */
#define STALL_BLANK_FS          8u    /* ignore DIAG for the first n full steps   */
#define MOVE_TIMEOUT_SLACK_MS 1500u   /* added to the slowest-case move time      */
#define JOG_DEADMAN_HOST_MS    600u   /* host jog must be refreshed within this   */
#define JOG_MAX_CONSOLE_MS   30000u   /* console jog hard time limit              */
#define HEAT_LIMIT_MS       120000u   /* motor duty bucket: refuse motion above   */
#define HEAT_ABORT_MS       180000u   /* ...and stop a running move above this    */
#define HEALTH_PERIOD_MS       100u   /* one driver register poll per period      */
#define COMM_FAIL_LIMIT          5u   /* consecutive GSTAT read failures -> fault */
#define UV_SPEED_CAP_PPS       600u   /* max speed while VS < 3.5 V (u3v5)        */

/* Built-in zoom calibration, measured on the bench 2026-11-13 and accepted
   as the zoom home. Used whenever no valid calibration is saved in flash.
   Units are firmware microsteps (16/fs); zoom driver U2 currently runs at its
   reset 32 ustep because its UART is not answering - re-measure once fixed. */
#define ZOOM_CAL_TRAVEL      11656    /* forward limit, usteps from zero          */
#define ZOOM_CAL_HOME_PPS      300u
#define ZOOM_CAL_HOME_MAX_FS   813u   /* full search (position unknown)           */
#define ZOOM_CAL_BACKOFF_FS      8u   /* zero = 8 fs off the reverse stop         */
#define ZOOM_CAL_HOME_EXTRA   1232    /* re-home search past zero, usteps         */

/* -------------------------------------------------------------- types --- */
typedef enum { EVT_NONE = 0, EVT_DONE, EVT_STOP, EVT_LIMIT, EVT_STALL, EVT_DRVERR,
               EVT_TIMEOUT, EVT_DEADMAN } LensEvt_t;
static const char *const s_evt_name[] = { "", "done", "stop", "limit", "stall", "drverr",
                                          "timeout", "deadman" };

/* Axis state machine. Only READY accepts position moves. */
typedef enum { AX_UNHOMED = 0, AX_HOMING, AX_READY, AX_FAULT } AxState_t;
static const char *const s_state_name[] = { "UNHOMED", "HOMING", "READY", "FAULT" };

/* Latched faults (cleared only by "lens clr" / host CLR, which re-configures
   the driver and leaves the axis UNHOMED). */
#define F_NOCFG    0x0001u   /* driver did not accept/confirm configuration   */
#define F_DRV      0x0002u   /* DIAG high / GSTAT.drv_err: driver shut down   */
#define F_OT       0x0004u   /* DRV_STATUS ot / t150: overtemperature         */
#define F_SHORT    0x0008u   /* DRV_STATUS s2ga/s2gb/s2vsa/s2vsb              */
#define F_COMM     0x0010u   /* driver stopped answering on the UART          */
#define F_TIMEOUT  0x0020u   /* move exceeded its worst-case duration         */
static const char *const s_fault_name[] = { "NOCFG", "DRV", "OT", "SHORT", "COMM", "TIMEOUT" };
/* Live warnings (not latched). */
#define W_UV       0x01u     /* VS < 3.5 V: speed capped, torque marginal     */
#define W_OTPW     0x02u     /* driver >120 C pre-warning: no new moves       */
#define W_OL       0x04u     /* open-load seen while moving (informative)     */
#define W_HEAT     0x08u     /* motor duty bucket over limit: no new moves    */
#define W_TRAVEL   0x10u     /* travel not calibrated: default limit in use   */
static const char *const s_warn_name[] = { "UV", "OTPW", "OPENLOAD", "HEAT", "TRAVEL?" };

/* Motion request results (shared by console and host). */
typedef enum { E_OK = 0, E_NODRV, E_FAULT, E_HOME, E_HOT, E_RANGE, E_BUSY } LensErr_t;
static const char *const s_err_name[] = { "OK", "NODRV", "FAULT", "HOME", "HOT", "RANGE", "BUSY" };
static LensErr_t s_last_err;

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
  uint8_t  allow_nocfg;      /* user override: move even if not configured */
  const char *drv;           /* "U1" / "U2" / "U5" (schematic designator)  */
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
  /* ---- guard rails ---- */
  uint8_t  state;            /* AxState_t                                  */
  uint16_t fault;            /* F_* latched                                */
  uint8_t  warn;             /* W_* live                                   */
  uint8_t  travel_known;     /* max_pos came from calibration              */
  uint32_t home_pps, home_max_fs, home_backoff_fs;
  int32_t  home_extra;       /* re-home search margin past zero, usteps; <0 = auto */
  uint8_t  home_phase;       /* 0 idle, 1 seeking stop, 2 backing off      */
  volatile uint8_t homing;   /* ISR: ignore min_pos while seeking the stop */
  volatile uint32_t steps_run;   /* steps since start (stall blanking)     */
  uint32_t t_deadline;       /* HAL tick: move must be finished by then    */
  uint32_t jog_expire;       /* HAL tick: jog deadman                      */
  uint32_t heat_ms;          /* duty bucket                                */
  uint8_t  comm_fail;
  uint8_t  health_phase;
  volatile uint8_t evt_deadman;
} Axis_t;

const char *const LENS_AxisName[LENS_NAXIS] = { "zoom", "focus", "iris" };

static Axis_t s_ax[LENS_NAXIS];
static LensEventFn s_evt_fn;
static char s_buf[200];

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
  /* StallGuard/DIAG only above ~75 % of the homing speed: the datasheet
     (10.4) calls low-speed StallGuard unreliable. TSTEP is in 1/fCLK per
     1/256 microstep; stall output is enabled while TCOOLTHRS >= TSTEP.   */
  uint32_t tcool = 0;
  if (a->sg_thr) {
    uint32_t pps = a->home_pps * 3u / 4u;
    if (pps < 20u) pps = 20u;
    tcool = 12000000u / (256u * pps);
    if (tcool > 0xFFFFFu) tcool = 0xFFFFFu;
  }
  tmc_write(a->addr, R_TCOOLTHRS, tcool);
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
  uint32_t vmax = a->max_pps, vst = a->start_pps;
  if ((a->warn & W_UV) && vmax > UV_SPEED_CAP_PPS) vmax = UV_SPEED_CAP_PPS;   /* low VS: less torque */
  if (vst > vmax) vst = vmax;
  a->v_start = (float)vst * a->mres;
  a->v_max   = (float)vmax * a->mres;
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
  a->steps_run = 0;

  /* Worst case: the whole distance at the start speed, plus slack. A move
     still running after this is a firmware/hardware fault, not a slow move. */
  uint32_t now = HAL_GetTick();
  if (jog) {
    a->t_deadline = a->jog_expire + MOVE_TIMEOUT_SLACK_MS;
  } else {
    uint32_t dist = (uint32_t)abs(target - a->pos);
    a->t_deadline = now + (uint32_t)((float)dist * 1000.0f / a->v_start) + MOVE_TIMEOUT_SLACK_MS;
  }

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
  uint32_t n = ++a->steps_run;

  /* DIAG high = driver shut down (short/overtemp) or StallGuard stall.
     Blanked for the first few full steps (start-up transient). Classified
     afterwards from DRV_STATUS in LENS_Task. */
  if (n > STALL_BLANK_FS * a->mres && (a->diag_port->IDR & a->diag_pin)) {
    axis_finish(a, EVT_DRVERR);
    return;
  }

  /* distance left: to target (move) and to the soft limit (both modes).
     While homing, the reverse limit is ignored - the hard stop is the goal. */
  int32_t to_lim = (a->dir > 0) ? (a->max_pos - pos) :
                   a->homing ? INT32_MAX / 4 : (pos - a->min_pos);
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

/* ===================================================== zoom/focus tracking ==
   Calibration table: per point a zoom position and the in-focus focus
   position for objects at infinity, 3 m, 1 m and 0.3 m (any may be unset).
   Focus is interpolated linearly in zoom position between points and
   linearly in object vergence (diopters = 1/m) between the distance columns,
   which is how image-side focus shift scales (~f^2/u).
   Follow mode: a zoom move/jog also drives focus to the table value for the
   new zoom position, at the object distance implied by where focus sat
   before the zoom started (so a manually focused subject stays sharp). */
#define TRK_MAX     16
#define TRK_COLS     4
#define TRK_UNSET   INT32_MIN
static const float    s_col_dpt[TRK_COLS] = { 0.0f, 1.0f / 3.0f, 1.0f, 1.0f / 0.3f };
static const char *const s_col_name[TRK_COLS] = { "inf", "3m", "1m", "0.3m" };

typedef struct { uint8_t used; int32_t zoom; int32_t focus[TRK_COLS]; } TrkPt_t;
static TrkPt_t  s_trk[TRK_MAX];
static uint8_t  s_trk_follow;
static float    s_trk_dpt;          /* object vergence focus is held at (diopters) */

static bool move_to_raw(LensAxis_t ax, int32_t pos);

/* Sorted indices of points that have at least one focus column set. */
static int trk_sorted(uint8_t *idx)
{
  int n = 0;
  for (int i = 0; i < TRK_MAX; i++) {
    if (!s_trk[i].used) continue;
    bool any = false;
    for (int c = 0; c < TRK_COLS; c++) if (s_trk[i].focus[c] != TRK_UNSET) any = true;
    if (!any) continue;
    int j = n++;
    while (j > 0 && s_trk[idx[j - 1]].zoom > s_trk[i].zoom) { idx[j] = idx[j - 1]; j--; }
    idx[j] = (uint8_t)i;
  }
  return n;
}

/* Focus at one table point for vergence d: piecewise linear over the set
   columns, clamped at the nearest set column outside their span. */
static float trk_point_focus(const TrkPt_t *p, float d)
{
  int lo = -1, hi = -1;
  for (int c = 0; c < TRK_COLS; c++) {
    if (p->focus[c] == TRK_UNSET) continue;
    if (s_col_dpt[c] <= d) lo = c;
    if (s_col_dpt[c] >= d && hi < 0) hi = c;
  }
  if (lo < 0) return (float)p->focus[hi];
  if (hi < 0 || hi == lo) return (float)p->focus[lo];
  float t = (d - s_col_dpt[lo]) / (s_col_dpt[hi] - s_col_dpt[lo]);
  return (float)p->focus[lo] + t * (float)(p->focus[hi] - p->focus[lo]);
}

/* Focus for zoom position z at vergence d. false if the table is empty. */
static bool trk_focus(int32_t z, float d, int32_t *out)
{
  uint8_t idx[TRK_MAX];
  int n = trk_sorted(idx);
  if (n == 0) return false;
  int k = 0;
  while (k < n - 1 && s_trk[idx[k + 1]].zoom <= z) k++;
  const TrkPt_t *a = &s_trk[idx[k]];
  float fa = trk_point_focus(a, d), f;
  if (n == 1 || z <= a->zoom || k == n - 1) {
    f = fa;                                   /* clamp outside the table */
  } else {
    const TrkPt_t *b = &s_trk[idx[k + 1]];
    float t = (float)(z - a->zoom) / (float)(b->zoom - a->zoom);
    f = fa + t * (trk_point_focus(b, d) - fa);
  }
  *out = (int32_t)lroundf(f);
  return true;
}

/* Inverse: which object vergence puts focus f in focus at zoom z. */
static bool trk_dpt_from(int32_t z, int32_t f, float *d_out)
{
  float g[TRK_COLS];
  for (int c = 0; c < TRK_COLS; c++) {
    int32_t v;
    if (!trk_focus(z, s_col_dpt[c], &v)) return false;
    g[c] = (float)v;
  }
  float ff = (float)f;
  /* Pick the segment containing f; else clamp to the nearer end. */
  for (int c = 0; c < TRK_COLS - 1; c++) {
    float a = g[c], b = g[c + 1];
    if ((ff - a) * (ff - b) <= 0.0f) {
      float t = (b == a) ? 0.0f : (ff - a) / (b - a);
      *d_out = s_col_dpt[c] + t * (s_col_dpt[c + 1] - s_col_dpt[c]);
      return true;
    }
  }
  *d_out = (fabsf(ff - g[0]) <= fabsf(ff - g[TRK_COLS - 1])) ? s_col_dpt[0] : s_col_dpt[TRK_COLS - 1];
  return true;
}

static int32_t axis_goal(const Axis_t *a)
{
  return a->pend_valid ? a->pend_target : (a->running && !a->jog) ? a->target : a->pos;
}

/* Before a zoom motion starts: remember the subject distance focus is at. */
static void trk_capture(void)
{
  Axis_t *z = &s_ax[LENS_ZOOM], *f = &s_ax[LENS_FOCUS];
  if (!s_trk_follow || z->running || z->pend_valid || f->running || f->pend_valid) return;
  float d;
  if (trk_dpt_from(z->pos, f->pos, &d)) s_trk_dpt = d;
}

/* Drive focus to the table value for zoom position zpos. */
static void trk_focus_to(int32_t zpos)
{
  Axis_t *f = &s_ax[LENS_FOCUS];
  int32_t t;
  if (!s_trk_follow || !trk_focus(zpos, s_trk_dpt, &t)) return;
  if (t < f->min_pos) t = f->min_pos;
  if (t > f->max_pos) t = f->max_pos;
  if (t != axis_goal(f)) move_to_raw(LENS_FOCUS, t);
}

bool LENS_MoveTo(LensAxis_t ax, int32_t pos)
{
  if (ax == LENS_ZOOM) trk_capture();
  bool ok = move_to_raw(ax, pos);
  if (ok && ax == LENS_ZOOM) trk_focus_to(pos);
  return ok;
}

/* Track distance in mm (0 = infinity): hold focus there and refocus now. */
static void trk_set_distance(uint32_t mm)
{
  s_trk_dpt = mm ? 1000.0f / (float)mm : 0.0f;
  bool f = s_trk_follow;
  s_trk_follow = 1;
  trk_focus_to(axis_goal(&s_ax[LENS_ZOOM]));
  s_trk_follow = f;
}

/* ---------------------------------------------------------- motion gate --
   Every motion request passes here. need_home: position moves (absolute or
   relative) need a referenced axis; console calibration jogs do not.
   "lens <ax> allow on" (console test mode) bypasses NODRV and HOME only -
   faults, heat, soft limits, timeouts and the jog dead-man stay active.   */
static LensErr_t gate(const Axis_t *a, bool need_home)
{
  if (!a->configured && !a->allow_nocfg)                 return E_NODRV;
  if (a->fault)                                          return E_FAULT;
  if (a->state == AX_HOMING)                             return E_BUSY;
  if (need_home && a->state != AX_READY && !a->allow_nocfg) return E_HOME;
  if (a->warn & (W_HEAT | W_OTPW))                       return E_HOT;
  return E_OK;
}

static bool move_to_raw(LensAxis_t ax, int32_t pos)
{
  if (!ax_ok(ax)) { s_last_err = E_RANGE; return false; }
  Axis_t *a = &s_ax[ax];
  if ((s_last_err = gate(a, true)) != E_OK) return false;
  if (pos < a->min_pos || pos > a->max_pos) { s_last_err = E_RANGE; return false; }

  if (a->running) {
    /* Same direction and still ahead of where a decelerated stop would land:
       retarget on the fly. Otherwise stop first and queue the move. */
    int32_t ahead = (pos - a->pos) * a->dir;
    float dec = (a->v * a->v - a->v_start * a->v_start) / a->acc2;
    if (!a->jog && ahead > (int32_t)dec + 2) {
      a->target = pos;
      uint32_t dist = (uint32_t)abs(pos - a->pos);
      a->t_deadline = HAL_GetTick() + (uint32_t)((float)dist * 1000.0f / a->v_start) + MOVE_TIMEOUT_SLACK_MS;
      return true;
    }
    a->pend_target = pos;
    a->pend_valid = 1;
    a->stop_req = 1;
    return true;
  }
  a->pend_valid = 0;
  if (pos == a->pos) { a->evt = EVT_DONE; return true; }
  axis_rates(a);
  axis_start(a, pos > a->pos ? 1 : -1, false, pos);
  return true;
}

bool LENS_MoveRel(LensAxis_t ax, int32_t delta)
{
  if (!ax_ok(ax)) return false;
  return LENS_MoveTo(ax, axis_goal(&s_ax[ax]) + delta);
}

/* Jog: runs until stopped, a soft limit, or the dead-man expires.
   pps 0 = configured max speed; the jog speed never changes the profile. */
static bool jog_start(LensAxis_t ax, int dir, uint32_t pps, uint32_t deadman_ms, bool need_home)
{
  if (!ax_ok(ax)) { s_last_err = E_RANGE; return false; }
  Axis_t *a = &s_ax[ax];
  if ((s_last_err = gate(a, need_home)) != E_OK) return false;
  int d = dir ? 1 : -1;
  if (ax == LENS_ZOOM) trk_capture();
  if (a->running) {
    if (a->dir == d && a->jog) {                     /* refresh the dead-man */
      a->jog_expire = HAL_GetTick() + deadman_ms;
      a->t_deadline = a->jog_expire + MOVE_TIMEOUT_SLACK_MS;
      return true;
    }
    s_last_err = E_BUSY;
    return false;
  }
  if ((d > 0 && a->pos >= a->max_pos) || (d < 0 && a->pos <= a->min_pos)) { s_last_err = E_RANGE; return false; }
  axis_rates(a);
  if (pps) {
    if (pps > LENS_SLEW_PPS_MAX) pps = LENS_SLEW_PPS_MAX;
    float v = (float)pps * a->mres;
    if (v < a->v_max) a->v_max = v;
    if (a->v_start > a->v_max) a->v_start = a->v_max;
  }
  a->pend_valid = 0;
  a->jog_expire = HAL_GetTick() + deadman_ms;
  axis_start(a, d, true, 0);
  return true;
}

bool LENS_Jog(LensAxis_t ax, int dir, uint32_t pps)
{
  return jog_start(ax, dir, pps, JOG_MAX_CONSOLE_MS, false);
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

static void emit(int i, const char *reason)
{
  if (s_evt_fn) s_evt_fn((LensAxis_t)i, s_ax[i].pos, reason);
}

/* Latch a fault: stop now, drop the reference, notify. */
static void axis_fault(int i, uint16_t f)
{
  Axis_t *a = &s_ax[i];
  bool fresh = (a->fault & f) != f;
  LENS_Halt((LensAxis_t)i);
  a->fault |= f;
  a->state = AX_FAULT;
  a->homing = 0; a->home_phase = 0;
  if (fresh) {
    for (unsigned b = 0; b < 6; b++) if (f & (1u << b)) {
      char r[24]; snprintf(r, sizeof r, "fault-%s", s_fault_name[b]); emit(i, r);
    }
  }
}

/* Position can no longer be trusted (stall, driver reset): needs re-homing. */
static void axis_unref(int i, const char *why)
{
  Axis_t *a = &s_ax[i];
  if (a->state == AX_READY || a->state == AX_HOMING) a->state = AX_UNHOMED;
  a->homing = 0; a->home_phase = 0;
  emit(i, why);
}

/* --------------------------------------------------------------- homing --
   No limit switches: reference against the lens' reverse mechanical stop.
   Phase 1 drives reverse at a low speed for at most home_max_fs full steps
   (more than the whole travel), ending at the hard stop either by stall
   detection (if tuned) or by running out the distance - the motor just
   slips at the stop, current is coil-limited (~VS/40 ohm). Phase 2 backs
   off home_backoff_fs and defines that point as position 0.              */
/* Homing search distance tracks the travel: just over one full travel, so
   from anywhere inside the range the stop is reached, and from the stop the
   motor slips for at most ~1/16 travel + backoff. */
static void home_fit_travel(Axis_t *a)
{
  uint32_t t = (uint32_t)(a->max_pos / (int32_t)a->mres);
  a->home_max_fs = t + t / 16u + a->home_backoff_fs;
}

static void home_speed(Axis_t *a)
{
  axis_rates(a);
  a->v_start = a->v_max = (float)a->home_pps * a->mres;
}

static LensErr_t LENS_Home(LensAxis_t ax)
{
  if (!ax_ok(ax)) return E_RANGE;
  Axis_t *a = &s_ax[ax];
  if (!a->configured && !a->allow_nocfg) return E_NODRV;
  if (a->fault) return E_FAULT;
  if (a->running || a->state == AX_HOMING) return E_BUSY;
  if (a->warn & (W_HEAT | W_OTPW)) return E_HOT;
  /* Already referenced: the stop is pos + backoff away - search only that
     plus a margin, so re-homing barely loads the hard stop. Otherwise the
     full home_max_fs (position unknown). */
  uint32_t seek_fs = a->home_max_fs;
  if (a->state == AX_READY && a->pos >= 0) {
    uint32_t t = (uint32_t)(a->max_pos / (int32_t)a->mres);
    uint32_t extra_fs = (a->home_extra >= 0) ? ((uint32_t)a->home_extra + a->mres - 1u) / a->mres
                                             : 4u + t / 64u;
    uint32_t want = (uint32_t)(a->pos / (int32_t)a->mres) + a->home_backoff_fs + extra_fs;
    if (want < seek_fs) seek_fs = want;
  }
  a->pend_valid = 0;
  a->state = AX_HOMING;
  a->home_phase = 1;
  a->homing = 1;
  home_speed(a);
  axis_start(a, -1, false, a->pos - (int32_t)(seek_fs * a->mres));
  return E_OK;
}

static void home_step(int i, uint8_t evt, bool stalled)
{
  Axis_t *a = &s_ax[i];
  if (a->home_phase == 1 && (evt == EVT_DONE || stalled)) {
    a->homing = 0;
    a->pos = -(int32_t)(a->home_backoff_fs * a->mres);      /* at the hard stop */
    a->home_phase = 2;
    home_speed(a);
    axis_start(a, +1, false, 0);
    return;
  }
  if (a->home_phase == 2 && evt == EVT_DONE) {
    a->home_phase = 0;
    a->state = AX_READY;
    emit(i, "homed");
    return;
  }
  a->home_phase = 0; a->homing = 0;                          /* aborted */
  a->state = AX_UNHOMED;
  emit(i, "home-abort");
}

/* ------------------------------------------------------- health monitor --
   One UART register read per HEALTH_PERIOD_MS, round-robin over present
   drivers, alternating GSTAT and DRV_STATUS (datasheet ch. 14):
   - GSTAT.reset     driver lost its config -> reconfigure, UNHOMED
   - GSTAT.drv_err   driver shut down       -> fault DRV
   - GSTAT.u3v5      VS < 3.5 V (live)      -> warn UV, speed capped
   - DRV_STATUS ot/t150 -> fault OT; s2g/s2vs -> fault SHORT;
     otpw/t120 -> warn OTPW (no new moves); ola/olb while moving -> warn OL
   - COMM_FAIL_LIMIT consecutive GSTAT read failures -> fault COMM       */
static bool axis_configure(Axis_t *a);

static void health_poll(void)
{
  static uint32_t t_next;
  static uint8_t rr;
  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - t_next) < 0) return;
  t_next = now + HEALTH_PERIOD_MS;

  for (int k = 0; k < LENS_NAXIS; k++) {
    int i = (rr + k) % LENS_NAXIS;
    Axis_t *a = &s_ax[i];
    if (!a->configured) continue;
    rr = (uint8_t)((i + 1) % LENS_NAXIS);
    uint32_t v;
    if ((a->health_phase ^= 1u) != 0u) {
      if (!tmc_read(a->addr, R_GSTAT, &v)) {
        if (++a->comm_fail >= COMM_FAIL_LIMIT) axis_fault(i, F_COMM);
        return;
      }
      a->comm_fail = 0;
      if (v & 4u) a->warn |= W_UV; else a->warn &= (uint8_t)~W_UV;
      if (v & 2u) axis_fault(i, F_DRV);
      if (v & 1u) {                      /* driver reset: config is gone */
        LENS_Halt((LensAxis_t)i);
        axis_configure(a);
        if (!a->configured) axis_fault(i, F_NOCFG);
        else axis_unref(i, "driver-reset");
      }
    } else if (tmc_read(a->addr, R_DRV_STATUS, &v)) {
      if (v & ((1u << 9) | (1u << 1))) axis_fault(i, F_OT);
      if (v & 0x3Cu)                   axis_fault(i, F_SHORT);
      if (v & ((1u << 8) | 1u)) a->warn |= W_OTPW; else a->warn &= (uint8_t)~W_OTPW;
      if (a->running && (v & 0xC0u)) a->warn |= W_OL;
    }
    return;
  }
}

/* DIAG went high during a move: driver error, or a stall (StallGuard on). */
static bool classify_diag(int i)
{
  Axis_t *a = &s_ax[i];
  uint32_t ds = 0, gs = 0;
  bool have = tmc_read(a->addr, R_DRV_STATUS, &ds);
  (void)tmc_read(a->addr, R_GSTAT, &gs);
  if (have && (ds & ((1u << 9) | (1u << 1)))) { axis_fault(i, F_OT);    return false; }
  if (have && (ds & 0x3Cu))                   { axis_fault(i, F_SHORT); return false; }
  if ((gs & 2u) || a->sg_thr == 0u)           { axis_fault(i, F_DRV);   return false; }
  return true;                                   /* StallGuard stall */
}

void LENS_Task(void)
{
  uint32_t now = HAL_GetTick();
  static uint32_t t_prev;
  uint32_t dt = now - t_prev;
  t_prev = now;

  for (int i = 0; i < LENS_NAXIS; i++) {
    Axis_t *a = &s_ax[i];

    /* motor duty bucket: fills while running, drains at half rate idle */
    if (a->running) a->heat_ms += dt;
    else a->heat_ms = (a->heat_ms > dt / 2u) ? a->heat_ms - dt / 2u : 0u;
    if (a->heat_ms > HEAT_LIMIT_MS) a->warn |= W_HEAT;
    else if (a->heat_ms < HEAT_LIMIT_MS / 2u) a->warn &= (uint8_t)~W_HEAT;
    if (a->running && a->heat_ms > HEAT_ABORT_MS) LENS_Stop((LensAxis_t)i);

    if (a->running) {
      if (a->jog && (int32_t)(now - a->jog_expire) >= 0 && !a->stop_req) {
        a->stop_req = 1;                                /* dead-man released */
        a->evt_deadman = 1;
      }
      if ((int32_t)(now - a->t_deadline) >= 0) {         /* should never happen */
        LENS_Halt((LensAxis_t)i);
        a->evt = EVT_TIMEOUT;
        axis_fault(i, F_TIMEOUT);
      }
    }

    uint8_t evt = a->evt;
    if (evt == EVT_NONE || a->running) continue;
    a->evt = EVT_NONE;
    if (evt == EVT_STOP && a->evt_deadman) evt = EVT_DEADMAN;
    a->evt_deadman = 0;

    bool stalled = false;
    if (evt == EVT_DRVERR) {
      stalled = classify_diag(i);
      if (!stalled) continue;                           /* fault already raised */
      evt = EVT_STALL;
    }

    if (a->state == AX_HOMING) { home_step(i, evt, stalled); continue; }

    if (stalled) {                                      /* steps were lost */
      a->pend_valid = 0;
      if (s_evt_fn) s_evt_fn((LensAxis_t)i, a->pos, s_evt_name[EVT_STALL]);
      axis_unref(i, "unhomed");
      continue;
    }
    if (a->pend_valid && evt == EVT_STOP) {             /* queued move: go now */
      a->pend_valid = 0;
      int32_t t = a->pend_target;
      if (t != a->pos && gate(a, true) == E_OK) {
        axis_rates(a);
        axis_start(a, t > a->pos ? 1 : -1, false, t);
        continue;
      }
      evt = EVT_DONE;
    }
    if (i == LENS_ZOOM) trk_focus_to(a->pos);
    if (s_evt_fn) s_evt_fn((LensAxis_t)i, a->pos, s_evt_name[evt]);
  }

  health_poll();
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
  a->max_pos = (int32_t)(DEF_TRAVEL_FS * DEF_MRES);   /* until calibrated */
  a->travel_known = 0;
  a->warn = W_TRAVEL;
  a->home_pps = DEF_HOME_PPS; a->home_max_fs = DEF_HOME_MAX_FS; a->home_backoff_fs = DEF_HOME_BACKOFF_FS;
  a->home_extra = -1;
  a->state = AX_UNHOMED;
  a->pos = 0;
  axis_rates(a);
}

/* ============================================================ config ===== */
/* Calibration survives reset in flash bank 1 sector 62 (0x0C07C000, secure).
   Sector 63 (0x0C07E000) is FLASH_NSC (secure-gateway veneers) - never touch
   it. The linker script reserves sector 62 (FLASH = 496 KB). Written only on
   an explicit "save", only with every axis idle (erase stalls code fetch from
   bank 1 for a few ms), CRC-checked on load. */
#define CFG_SECTOR  62u
#define CFG_MAGIC   0x534E454Cu      /* "LENS" */
#define CFG_VER     2u
typedef struct {
  uint32_t magic, ver, size;
  struct {
    uint8_t  addr, irun, ihold, sg_thr;
    uint16_t mres; uint8_t travel_known, pad;
    uint32_t start_pps, max_pps, acc_pps2;
    int32_t  max_pos;
    uint32_t home_pps, home_max_fs, home_backoff_fs;
    int32_t  home_extra;
  } ax[LENS_NAXIS];
  uint8_t  autohome, trk_follow, pad[2];
  TrkPt_t  trk[TRK_MAX];
  uint32_t crc;
} LensCfg_t;

static uint8_t s_autohome;

static uint32_t crc32(const void *d, size_t n)
{
  const uint8_t *p = d; uint32_t c = 0xFFFFFFFFu;
  while (n--) { c ^= *p++; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
  return ~c;
}

static uint32_t cfg_addr(void)
{
  return FLASH_BASE + CFG_SECTOR * FLASH_SECTOR_SIZE;        /* 0x0C07C000 */
}

static void cfg_fill(LensCfg_t *c)
{
  memset(c, 0, sizeof *c);
  c->magic = CFG_MAGIC; c->ver = CFG_VER; c->size = sizeof *c;
  for (int i = 0; i < LENS_NAXIS; i++) {
    const Axis_t *a = &s_ax[i];
    c->ax[i].addr = a->addr; c->ax[i].irun = a->irun; c->ax[i].ihold = a->ihold; c->ax[i].sg_thr = a->sg_thr;
    c->ax[i].mres = a->mres; c->ax[i].travel_known = a->travel_known;
    c->ax[i].start_pps = a->start_pps; c->ax[i].max_pps = a->max_pps; c->ax[i].acc_pps2 = a->acc_pps2;
    c->ax[i].max_pos = a->max_pos;
    c->ax[i].home_pps = a->home_pps; c->ax[i].home_max_fs = a->home_max_fs;
    c->ax[i].home_backoff_fs = a->home_backoff_fs;
    c->ax[i].home_extra = a->home_extra;
  }
  c->autohome = s_autohome; c->trk_follow = s_trk_follow;
  memcpy(c->trk, s_trk, sizeof s_trk);
  c->crc = crc32(c, offsetof(LensCfg_t, crc));
}

static bool cfg_load(void)
{
  const LensCfg_t *c = (const LensCfg_t *)cfg_addr();
  if (c->magic != CFG_MAGIC || c->ver != CFG_VER || c->size != sizeof *c) return false;
  if (crc32(c, offsetof(LensCfg_t, crc)) != c->crc) return false;
  for (int i = 0; i < LENS_NAXIS; i++) {
    Axis_t *a = &s_ax[i];
    uint16_t m = c->ax[i].mres;
    if (m != 1 && m != 2 && m != 4 && m != 8 && m != 16 && m != 32 && m != 64) return false;
    a->addr = c->ax[i].addr & 3u; a->irun = c->ax[i].irun; a->ihold = c->ax[i].ihold;
    a->sg_thr = c->ax[i].sg_thr; a->mres = m;
    a->start_pps = c->ax[i].start_pps; a->max_pps = c->ax[i].max_pps; a->acc_pps2 = c->ax[i].acc_pps2;
    if (a->max_pps > LENS_SLEW_PPS_MAX) a->max_pps = LENS_SLEW_PPS_MAX;
    if (a->start_pps > LENS_START_PPS_MAX) a->start_pps = LENS_START_PPS_MAX;
    a->max_pos = c->ax[i].max_pos;
    a->travel_known = c->ax[i].travel_known;
    if (a->travel_known) a->warn &= (uint8_t)~W_TRAVEL;
    a->home_pps = c->ax[i].home_pps; a->home_max_fs = c->ax[i].home_max_fs;
    a->home_backoff_fs = c->ax[i].home_backoff_fs;
    a->home_extra = c->ax[i].home_extra;
    axis_rates(a);
  }
  s_autohome = c->autohome; s_trk_follow = c->trk_follow;
  memcpy(s_trk, c->trk, sizeof s_trk);
  return true;
}

static const char *cfg_save(void)
{
  for (int i = 0; i < LENS_NAXIS; i++) if (s_ax[i].running) return "axis moving";
  uint32_t wm = FLASH->SECWM1R_CUR;
  uint32_t sec = CFG_SECTOR;
  uint32_t st = (wm & FLASH_SECWMR_SECWM_STRT_Msk) >> FLASH_SECWMR_SECWM_STRT_Pos;
  uint32_t en = (wm & FLASH_SECWMR_SECWM_END_Msk) >> FLASH_SECWMR_SECWM_END_Pos;
  if (st > sec || en < sec) return "config sector is not in the secure flash area";

  static LensCfg_t c __attribute__((aligned(16)));
  cfg_fill(&c);
  FLASH_EraseInitTypeDef e = { .TypeErase = FLASH_TYPEERASE_SECTORS, .Banks = FLASH_BANK_1,
                               .Sector = sec, .NbSectors = 1 };
  uint32_t serr = 0;
  const char *res = NULL;
  static char why[96];
  /* ICACHE caches the flash code region: program/erase with it disabled,
     invalidate afterwards (RM0481 ICACHE / FLASH programming notes). */
  bool icache_on = (ICACHE->CR & ICACHE_CR_EN) != 0u;
  if (icache_on) HAL_ICACHE_Disable();
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&e, &serr) != HAL_OK) {
    snprintf(why, sizeof why, "erase failed (err 0x%lX)", (unsigned long)HAL_FLASH_GetError()); res = why;
  }
  for (uint32_t off = 0; !res && off < sizeof c; off += 16u)
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, cfg_addr() + off, (uint32_t)((uint8_t *)&c + off)) != HAL_OK) {
      snprintf(why, sizeof why, "program failed at 0x%08lX (err 0x%lX)", (unsigned long)(cfg_addr() + off),
               (unsigned long)HAL_FLASH_GetError());
      res = why;
    }
  uint32_t sr = FLASH->SECSR, cr = FLASH->SECCR;
  HAL_FLASH_Lock();
  if (icache_on) { HAL_ICACHE_Invalidate(); HAL_ICACHE_Enable(); }
  if (!res) {
    const uint8_t *f = (const uint8_t *)cfg_addr(), *r = (const uint8_t *)&c;
    for (uint32_t k = 0; k < sizeof c; k++) if (f[k] != r[k]) {
      snprintf(why, sizeof why, "verify failed at 0x%08lX (SECSR 0x%08lX SECCR 0x%08lX)",
               (unsigned long)(cfg_addr() + k), (unsigned long)sr, (unsigned long)cr);
      res = why; break;
    }
  }
  return res;
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
    if (a->present) {                      /* refresh the live VS warning */
      uint32_t g;
      if (tmc_read(a->addr, R_GSTAT, &g)) { if (g & 4u) a->warn |= W_UV; else a->warn &= (uint8_t)~W_UV; }
      axis_rates(a);
    }
    snprintf(s_buf, sizeof s_buf, "[LENS] %-5s %s@%u: %s\r\n", LENS_AxisName[i], a->drv, a->addr,
             !a->present ? "NO REPLY - driver not configured (check M1/M2_PWR, AD0/AD1)" :
             a->configured ? "configured" : "replied but config NOT confirmed");
    print(s_buf);
    uint32_t gs;
    if (a->present && tmc_read(a->addr, R_GSTAT, &gs) && (gs & 4u)) {
      snprintf(s_buf, sizeof s_buf, "[LENS] %-5s %s: WARNING VS < 3.5 V (GSTAT.u3v5) - motor rated 4 V, low torque\r\n",
               LENS_AxisName[i], a->drv);
      print(s_buf);
    }
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

  /* z = driver U1 (schematic ZOOM), f = driver U2 (schematic FOCUS); which
     logical axis each drives is set by LENS_SWAP_ZOOM_FOCUS (lens_ctrl.h). */
  Axis_t *z = &s_ax[LENS_AXIS_U1], *f = &s_ax[LENS_AXIS_U2], *ir = &s_ax[LENS_IRIS];
  memset(s_ax, 0, sizeof s_ax);
  z->drv = "U1"; f->drv = "U2"; ir->drv = "U5";

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

  axis_defaults(z, 0);    /* schematic: U1 ADDR 00 */
  axis_defaults(f, 1);    /*            U2 ADDR 01 */
  axis_defaults(ir, 2);   /*            iris U5 ADDR 10  */
  bool cfg_ok = cfg_load();
  if (!cfg_ok) {
    Axis_t *zc = &s_ax[LENS_ZOOM];
    zc->max_pos = ZOOM_CAL_TRAVEL;
    zc->travel_known = 1;
    zc->warn &= (uint8_t)~W_TRAVEL;
    zc->home_pps = ZOOM_CAL_HOME_PPS;
    zc->home_max_fs = ZOOM_CAL_HOME_MAX_FS;
    zc->home_backoff_fs = ZOOM_CAL_BACKOFF_FS;
    zc->home_extra = ZOOM_CAL_HOME_EXTRA;
  }

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

  print("\r\n[LENS] NV-0622IR lens - guarded motion control (lens help)\r\n");
  print(cfg_ok ? "[LENS] calibration loaded from flash\r\n"
               : "[LENS] no saved calibration - built-in zoom calibration (0..11656), focus uncalibrated\r\n");
  scan_report(print);
  config_report(print);
  if (s_autohome) {
    for (int i = 0; i < LENS_NAXIS; i++) {
      if (i == LENS_IRIS || !s_ax[i].configured) continue;   /* lens has no iris motor */
      LensErr_t e = LENS_Home((LensAxis_t)i);
      snprintf(s_buf, sizeof s_buf, "[LENS] autohome %s: %s\r\n", LENS_AxisName[i],
               e == E_OK ? "started" : s_err_name[e]);
      print(s_buf);
    }
  }
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

static void flags_str(char *o, size_t n, unsigned v, const char *const *names, unsigned cnt)
{
  o[0] = 0;
  if (!v) { snprintf(o, n, "-"); return; }
  for (unsigned b = 0; b < cnt; b++) if (v & (1u << b)) {
    size_t l = strlen(o);
    snprintf(o + l, n - l, "%s%s", l ? "," : "", names[b]);
  }
}

static void print_status(LensPrintFn print)
{
  char fs[48], ws[48];
  for (int i = 0; i < LENS_NAXIS; i++) {
    Axis_t *a = &s_ax[i];
    flags_str(fs, sizeof fs, a->fault, s_fault_name, 6);
    flags_str(ws, sizeof ws, a->warn, s_warn_name, 5);
    snprintf(s_buf, sizeof s_buf,
             "%-5s %-7s pos %ld (%ld fs) %s  lim %ld..%ld%s  fault %s  warn %s\r\n",
             LENS_AxisName[i], s_state_name[a->state], (long)a->pos, (long)(a->pos / (int32_t)a->mres),
             a->running ? (a->jog ? "JOG" : "MOVING") : "idle",
             (long)a->min_pos, (long)a->max_pos, a->travel_known ? "" : "(default)", fs, ws);
    print(s_buf);
    snprintf(s_buf, sizeof s_buf,
             "      %s@%u %s%s mres %u irun %u ihold %u | speed %lu/%lu pps acc %lu | home %lu pps max %lu fs off %lu fs extra %ld | heat %lus\r\n",
             a->drv, a->addr, a->configured ? "cfg" : (a->present ? "NOCFG" : "ABSENT"),
             a->allow_nocfg ? " TESTMODE" : "", a->mres, a->irun, a->ihold,
             (unsigned long)a->start_pps, (unsigned long)a->max_pps, (unsigned long)a->acc_pps2,
             (unsigned long)a->home_pps, (unsigned long)a->home_max_fs, (unsigned long)a->home_backoff_fs,
             (long)a->home_extra, (unsigned long)(a->heat_ms / 1000u));
    print(s_buf);
  }
  snprintf(s_buf, sizeof s_buf, "autohome %s, stall detection %s/%s/%s (sgthrs)\r\n", s_autohome ? "on" : "off",
           s_ax[0].sg_thr ? "on" : "off", s_ax[1].sg_thr ? "on" : "off", s_ax[2].sg_thr ? "on" : "off");
  print(s_buf);
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

static int trk_col(const char *t)
{
  for (int c = 0; c < TRK_COLS; c++) if (!strcmp(t, s_col_name[c])) return c;
  if (!strcmp(t, "0")) return 0;
  return -1;
}

static void trk_print_point(int i, LensPrintFn print, bool as_cmd)
{
  const TrkPt_t *p = &s_trk[i];
  char v[TRK_COLS][12];
  for (int c = 0; c < TRK_COLS; c++) {
    if (p->focus[c] == TRK_UNSET) strcpy(v[c], as_cmd ? "x" : "-");
    else snprintf(v[c], sizeof v[c], "%ld", (long)p->focus[c]);
  }
  if (as_cmd)
    snprintf(s_buf, sizeof s_buf, "lens track set %d %ld %s %s %s %s\r\n", i, (long)p->zoom, v[0], v[1], v[2], v[3]);
  else
    snprintf(s_buf, sizeof s_buf, "  %2d  zoom %8ld   focus inf %8s  3m %8s  1m %8s  0.3m %8s\r\n",
             i, (long)p->zoom, v[0], v[1], v[2], v[3]);
  print(s_buf);
}

static bool parse_focus(const char *t, int32_t *out)
{
  long v;
  if (!strcmp(t, "x") || !strcmp(t, "-")) { *out = TRK_UNSET; return true; }
  if (!parse_long(t, &v)) return false;
  *out = (int32_t)v;
  return true;
}

static void cli_track(char **tok, int nt, LensPrintFn print)
{
  const char *op = nt > 0 ? tok[0] : "show";
  long i = 0, v = 0;

  if (!strcmp(op, "show") || !strcmp(op, "dump")) {
    bool as_cmd = !strcmp(op, "dump");
    uint8_t idx[TRK_MAX];
    int n = trk_sorted(idx);
    if (!as_cmd) {
      snprintf(s_buf, sizeof s_buf, "tracking: %d point(s) usable, follow %s, distance %s",
               n, s_trk_follow ? "ON" : "off", s_trk_dpt <= 0.0f ? "inf" : "");
      print(s_buf);
      if (s_trk_dpt > 0.0f) {
        snprintf(s_buf, sizeof s_buf, "%ld mm", lroundf(1000.0f / s_trk_dpt)); print(s_buf);
      }
      print("\r\n");
    } else {
      print("lens track clear\r\n");
    }
    for (int k = 0; k < TRK_MAX; k++) if (s_trk[k].used) trk_print_point(k, print, as_cmd);
    if (!as_cmd && n > 0) {
      int32_t f;
      if (trk_focus(s_ax[LENS_ZOOM].pos, s_trk_dpt, &f)) {
        snprintf(s_buf, sizeof s_buf, "  at zoom %ld -> focus %ld (focus now %ld)\r\n",
                 (long)s_ax[LENS_ZOOM].pos, (long)f, (long)s_ax[LENS_FOCUS].pos);
        print(s_buf);
      }
    }
    return;
  }
  if (!strcmp(op, "clear")) {
    memset(s_trk, 0, sizeof s_trk); print("OK table cleared\r\n"); return;
  }
  if (!strcmp(op, "set") && nt >= 3 && parse_long(tok[1], &i) && i >= 0 && i < TRK_MAX &&
      parse_long(tok[2], &v)) {
    TrkPt_t p = { 1, (int32_t)v, { TRK_UNSET, TRK_UNSET, TRK_UNSET, TRK_UNSET } };
    for (int c = 0; c < TRK_COLS && 3 + c < nt; c++)
      if (!parse_focus(tok[3 + c], &p.focus[c])) { print("ERROR: focus value or x\r\n"); return; }
    s_trk[i] = p;
    trk_print_point((int)i, print, false); return;
  }
  if (!strcmp(op, "cap") && nt >= 3 && parse_long(tok[1], &i) && i >= 0 && i < TRK_MAX) {
    int c = trk_col(tok[2]);
    if (c < 0) { print("ERROR: column inf|3m|1m|0.3m\r\n"); return; }
    if (LENS_IsMoving(LENS_ZOOM) || LENS_IsMoving(LENS_FOCUS)) { print("ERROR: moving\r\n"); return; }
    TrkPt_t *p = &s_trk[i];
    if (!p->used || p->zoom != s_ax[LENS_ZOOM].pos) {
      if (p->used) print("note: zoom differs from stored point - other columns cleared\r\n");
      p->used = 1; p->zoom = s_ax[LENS_ZOOM].pos;
      for (int k = 0; k < TRK_COLS; k++) p->focus[k] = TRK_UNSET;
    }
    p->focus[c] = s_ax[LENS_FOCUS].pos;
    trk_print_point((int)i, print, false); return;
  }
  if (!strcmp(op, "del") && nt >= 2 && parse_long(tok[1], &i) && i >= 0 && i < TRK_MAX) {
    s_trk[i].used = 0; print("OK\r\n"); return;
  }
  if (!strcmp(op, "follow") && nt >= 2) {
    s_trk_follow = !strcmp(tok[1], "on");
    if (s_trk_follow) trk_capture();
    snprintf(s_buf, sizeof s_buf, "OK follow %s\r\n", s_trk_follow ? "on" : "off");
    print(s_buf); return;
  }
  if (!strcmp(op, "dist") && nt >= 2) {
    long mm = 0;
    if (strcmp(tok[1], "inf") && (!parse_long(tok[1], &mm) || mm < 0)) { print("ERROR: mm|inf\r\n"); return; }
    if (mm > 0 && mm < 100) mm = 100;
    uint8_t idx[TRK_MAX];
    if (trk_sorted(idx) == 0) { print("ERROR: table empty\r\n"); return; }
    trk_set_distance((uint32_t)mm);
    snprintf(s_buf, sizeof s_buf, "OK distance %s%ld, focus -> %ld\r\n", mm ? "" : "inf ", mm,
             (long)axis_goal(&s_ax[LENS_FOCUS]));
    print(s_buf); return;
  }
  if (!strcmp(op, "goto") && nt >= 2 && parse_long(tok[1], &v)) {
    long mm = -1;
    if (nt >= 3 && strcmp(tok[2], "inf") && (!parse_long(tok[2], &mm) || mm < 0)) { print("ERROR: mm\r\n"); return; }
    if (nt >= 3) s_trk_dpt = (mm > 0) ? 1000.0f / (float)(mm < 100 ? 100 : mm) : 0.0f;
    else trk_capture();
    uint8_t f = s_trk_follow;
    s_trk_follow = 1;                  /* one-shot follow for this move */
    bool ok = move_to_raw(LENS_ZOOM, (int32_t)v);
    if (ok) trk_focus_to((int32_t)v);
    s_trk_follow = f;
    snprintf(s_buf, sizeof s_buf, "%s zoom -> %ld, focus -> %ld\r\n", ok ? "OK" : "REJECTED", v,
             (long)axis_goal(&s_ax[LENS_FOCUS]));
    print(s_buf); return;
  }
  print("ERROR: lens track [show|dump|clear|set|cap|del|follow|dist|goto] - lens help\r\n");
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
  print("  lens <ax> cur <irun> [ihold]    0..31, IRUN >= 8 (coils limit to ~VS/40)\r\n");
  print("  lens <ax> mres <1..64>          microsteps per full step (rescales pos)\r\n");
  print("  lens <ax> addr <0..3>           TMC2300 UART node address\r\n");
  print("  lens <ax> sg <0..255>           StallGuard threshold, 0 = off\r\n");
  print("  lens <ax> dump | reg <r> [val]  driver registers\r\n");
  print("  --- guard rails (docs/lens-control-design.md) ---\r\n");
  print("  lens home [all] | lens <ax> home  reference against the reverse hard stop\r\n");
  print("  lens clr [all] | lens <ax> clr  clear faults, re-configure driver (-> UNHOMED)\r\n");
  print("  lens <ax> setmax                after homing + jog to far end: store travel\r\n");
  print("  lens <ax> travel <fs>           set travel in full steps directly\r\n");
  print("  lens <ax> homecfg <pps> <max_fs> <backoff_fs>\r\n");
  print("  lens <ax> homeextra <usteps>|auto  re-home search past zero (auto 4+travel/64 fs)\r\n");
  print("  lens save | lens autohome on|off  persist calibration / home at boot\r\n");
  print("  lens <ax> allow on|off          TEST MODE: no config/home needed (limits stay)\r\n");
  print("  --- zoom/focus tracking (focus columns: inf 3m 1m 0.3m, x = unset) ---\r\n");
  print("  lens track [show] | dump        table / table as re-loadable commands\r\n");
  print("  lens track set <i> <zoom> <f_inf> [f_3m] [f_1m] [f_0.3m]   point i (0..15)\r\n");
  print("  lens track cap <i> <inf|3m|1m|0.3m>  store current zoom+focus (calibration)\r\n");
  print("  lens track del <i> | clear\r\n");
  print("  lens track follow on|off        focus follows every zoom move/jog\r\n");
  print("  lens track dist <mm>|inf        hold focus at this object distance now\r\n");
  print("  lens track goto <zoom> [mm|inf] zoom with focus tracking (one-shot)\r\n");
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
    if (!a->allow_nocfg) { print("ERROR: zero is set by homing (lens <ax> home); test mode only\r\n"); return; }
    if (a->running) { print("ERROR: moving\r\n"); return; }
    a->pos = 0;
  } else if (!strcmp(op, "limit") && nt >= 3 && parse_long(tok[1], &v1) && parse_long(tok[2], &v2)) {
    if (!a->allow_nocfg) { print("ERROR: raw limits only in test mode - use travel/setmax\r\n"); return; }
    a->min_pos = (int32_t)v1;
    a->max_pos = (v2 < 0) ? (int32_t)(a->home_max_fs * a->mres) : (int32_t)v2;
    if (v2 > 0) { a->travel_known = 1; a->warn &= (uint8_t)~W_TRAVEL; home_fit_travel(a); }
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
  } else if (!strcmp(op, "home")) {
    LensErr_t e = LENS_Home(ax); ok = (e == E_OK); s_last_err = e;
  } else if (!strcmp(op, "clr")) {
    LENS_Halt(ax); a->fault = 0; a->comm_fail = 0;
    ok = axis_configure(a);
    a->state = AX_UNHOMED;
    if (!ok) s_last_err = E_NODRV;
  } else if (!strcmp(op, "travel") && nt >= 2 && parse_long(tok[1], &v1) && v1 > 0) {
    a->max_pos = (int32_t)(v1 * a->mres); a->travel_known = 1; a->warn &= (uint8_t)~W_TRAVEL;
    home_fit_travel(a);
  } else if (!strcmp(op, "setmax")) {
    if (a->state != AX_READY) { print("ERROR: home first (lens <ax> home)\r\n"); return; }
    if (a->running || a->pos <= 0) { print("ERROR: jog to the far end first, idle\r\n"); return; }
    a->max_pos = a->pos - (int32_t)(a->home_backoff_fs * a->mres);
    a->travel_known = 1; a->warn &= (uint8_t)~W_TRAVEL;
    home_fit_travel(a);
    snprintf(s_buf, sizeof s_buf, "travel %ld usteps (%ld fs), homing max %lu fs - 'lens save' to keep\r\n",
             (long)a->max_pos, (long)(a->max_pos / (int32_t)a->mres), (unsigned long)a->home_max_fs);
    print(s_buf);
  } else if (!strcmp(op, "homecfg") && nt >= 4 && parse_long(tok[1], &v1) && parse_long(tok[2], &v2) &&
             parse_long(tok[3], &v3)) {
    if (v1 < 50 || v1 > (long)LENS_START_PPS_MAX || v2 < 50 || v2 > 20000 || v3 < 0 || v3 > 500) {
      print("ERROR: pps 50..800, max_fs 50..20000, backoff_fs 0..500\r\n"); return;
    }
    a->home_pps = (uint32_t)v1; a->home_max_fs = (uint32_t)v2; a->home_backoff_fs = (uint32_t)v3;
  } else if (!strcmp(op, "homeextra") && nt >= 2) {
    if (!strcmp(tok[1], "auto")) a->home_extra = -1;
    else if (parse_long(tok[1], &v1) && v1 >= 0 && v1 <= 20000) a->home_extra = (int32_t)v1;
    else { print("ERROR: homeextra <usteps 0..20000>|auto\r\n"); return; }
  } else if (!strcmp(op, "allow") && nt >= 2) {
    a->allow_nocfg = !strcmp(tok[1], "on");
    if (a->allow_nocfg)
      print("TEST MODE: moves allowed without driver config and without homing. Soft limits,\r\n"
            "          faults, timeouts, heat and jog dead-man stay active. Unconfigured drivers\r\n"
            "          run at reset settings (IRUN 31, 32 ustep); 40 ohm coils cap current ~VS/40.\r\n");
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

  snprintf(s_buf, sizeof s_buf, "%s %s: pos %ld %s%s\r\n", ok ? "OK" : "REJECTED", LENS_AxisName[ax],
           (long)a->pos, a->running ? "moving " : "", ok ? "" : s_err_name[s_last_err]);
  print(s_buf);
}

bool LENS_CLI(const char *cmd, LensPrintFn print)
{
  char line[128];
  strncpy(line, cmd, sizeof line - 1);
  line[sizeof line - 1] = 0;
  for (char *p = line; *p; p++) *p = (char)tolower((unsigned char)*p);

  char *tok[10]; int nt = 0;
  for (char *t = strtok(line, " \t"); t && nt < 10; t = strtok(NULL, " \t")) tok[nt++] = t;
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
        snprintf(s_buf, sizeof s_buf, "%s: %s jog dir %ld at %lu pps (%lu step/s)%s%s\r\n", ok ? "SUCCESS" : "REJECTED",
                 LENS_AxisName[i], dir, (unsigned long)(s_ax[i].v_max / s_ax[i].mres),
                 (unsigned long)s_ax[i].v_max, ok ? "" : " - ", ok ? "" : s_err_name[s_last_err]);
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
  if (!strcmp(tok[1], "track")) { cli_track(&tok[2], nt - 2, print); return true; }
  if (!strcmp(tok[1], "home") && (nt == 2 || !strcmp(tok[2], "all"))) {
    for (int i = 0; i < LENS_NAXIS; i++) {
      if (i == LENS_IRIS) continue;
      LensErr_t e = LENS_Home((LensAxis_t)i);
      snprintf(s_buf, sizeof s_buf, "home %s: %s\r\n", LENS_AxisName[i], e == E_OK ? "started" : s_err_name[e]);
      print(s_buf);
    }
    return true;
  }
  if (!strcmp(tok[1], "clr") && (nt == 2 || !strcmp(tok[2], "all"))) {
    for (int i = 0; i < LENS_NAXIS; i++) {
      Axis_t *a = &s_ax[i];
      LENS_Halt((LensAxis_t)i); a->fault = 0; a->comm_fail = 0;
      axis_configure(a); a->state = AX_UNHOMED;
    }
    print_status(print); return true;
  }
  if (!strcmp(tok[1], "save")) {
    const char *e = cfg_save();
    print(e ? "ERROR: save " : "OK calibration saved to flash\r\n");
    if (e) { print(e); print("\r\n"); }
    return true;
  }
  if (!strcmp(tok[1], "autohome") && nt >= 3) {
    s_autohome = !strcmp(tok[2], "on");
    print(s_autohome ? "OK autohome on (lens save to keep)\r\n" : "OK autohome off\r\n");
    return true;
  }
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
     Tracking table (zoom/focus correlation; lost at reset - SoC reloads it):
     TC                       clear table          -> OK TC
     TS <i> <z> <fi> [f3 f1 f03]  set point i (x = unset) -> OK TS <i>
     TG <i>                   get point            -> TP <i> <z> <fi> <f3> <f1> <f03>
     TN                       points / follow      -> TN <n> <follow> <dist_mm|0=inf>
     TF 0|1                   focus follows zoom   -> OK TF <0|1>
     TD <mm>                  focus to distance now (0 = inf) -> OK TD <focus>
     ZF <z> [mm]              zoom + tracked focus -> OK ZF <z> <focus>
     Guard rails (docs/lens-control-design.md):
     H Z|F|ALL                home against the reverse stop -> OK H; then
                              DONE Z 0 homed | DONE Z <pos> home-abort
     ST                       -> ST Z <state> <pos> <fault> <warn> F ...
                              state UNHOMED|HOMING|READY|FAULT, fault/warn hex
     CLR Z|F|ALL              clear faults, re-configure (axis -> UNHOMED)
     ZJ <0|1> [pps] / FJ ...  dead-man jog: repeat within 600 ms or it stops
     LIM Z|F <fs>             set calibrated travel (full steps)
     SAVE                     persist calibration + tracking table to flash
   Moves need state READY (home first). Errors:
     ERR SYNTAX|RANGE|BUSY|NODRV|FAULT|HOME|HOT
   Unsolicited on every stop:  DONE Z <pos> <reason>
     reason: done stop limit stall deadman homed home-abort unhomed
             driver-reset fault-<NOCFG|DRV|OT|SHORT|COMM|TIMEOUT>          */
static void reply_err(LensPrintFn reply)
{
  char o[24]; snprintf(o, sizeof o, "ERR %s\n", s_err_name[s_last_err]); reply(o);
}

static int host_axis(const char *t)
{
  if (!strcmp(t, "Z")) return LENS_ZOOM;
  if (!strcmp(t, "F")) return LENS_FOCUS;
  if (!strcmp(t, "ALL")) return LENS_NAXIS;
  return -1;
}
void LENS_HostCommand(const char *line, LensPrintFn reply)
{
  char l[96];
  size_t n = 0;
  for (const char *p = line; *p && n < sizeof l - 1; p++) l[n++] = (char)toupper((unsigned char)*p);
  l[n] = 0;
  char *tok[8]; int nt = 0;
  for (char *t = strtok(l, " \t,"); t && nt < 8; t = strtok(NULL, " \t,")) tok[nt++] = t;
  if (nt == 0) return;

  char out[96];
  const char *c = tok[0];
  long v;

  if (!strcmp(c, "P") || !strcmp(c, "?")) {
    snprintf(out, sizeof out, "POS Z %ld %d F %ld %d\n",
             (long)s_ax[LENS_ZOOM].pos, LENS_IsMoving(LENS_ZOOM),
             (long)s_ax[LENS_FOCUS].pos, LENS_IsMoving(LENS_FOCUS));
    reply(out); return;
  }
  if (!strcmp(c, "V")) { reply("VER PROXY-LENS 2\n"); return; }

  /* ---- tracking table ---- */
  if (c[0] == 'T' && c[1] && !c[2]) {
    long i = 0, z = 0;
    uint8_t idx[TRK_MAX];
    switch (c[1]) {
      case 'C': memset(s_trk, 0, sizeof s_trk); reply("OK TC\n"); return;
      case 'S': {
        if (nt < 4 || !parse_long(tok[1], &i) || i < 0 || i >= TRK_MAX || !parse_long(tok[2], &z)) break;
        TrkPt_t p = { 1, (int32_t)z, { TRK_UNSET, TRK_UNSET, TRK_UNSET, TRK_UNSET } };
        for (int k = 0; k < TRK_COLS && 3 + k < nt; k++) {
          if (!strcmp(tok[3 + k], "X")) continue;
          long fv; if (!parse_long(tok[3 + k], &fv)) { reply("ERR SYNTAX\n"); return; }
          p.focus[k] = (int32_t)fv;
        }
        s_trk[i] = p;
        snprintf(out, sizeof out, "OK TS %ld\n", i); reply(out); return;
      }
      case 'G': {
        if (nt < 2 || !parse_long(tok[1], &i) || i < 0 || i >= TRK_MAX) break;
        if (!s_trk[i].used) { reply("ERR RANGE\n"); return; }
        char v[TRK_COLS][12];
        for (int k = 0; k < TRK_COLS; k++) {
          if (s_trk[i].focus[k] == TRK_UNSET) strcpy(v[k], "X");
          else snprintf(v[k], sizeof v[k], "%ld", (long)s_trk[i].focus[k]);
        }
        snprintf(out, sizeof out, "TP %ld %ld %s %s %s %s\n", i, (long)s_trk[i].zoom, v[0], v[1], v[2], v[3]);
        reply(out); return;
      }
      case 'N':
        snprintf(out, sizeof out, "TN %d %d %ld\n", trk_sorted(idx), s_trk_follow,
                 s_trk_dpt > 0.0f ? lroundf(1000.0f / s_trk_dpt) : 0L);
        reply(out); return;
      case 'F':
        if (nt < 2 || !parse_long(tok[1], &i)) break;
        s_trk_follow = i ? 1u : 0u;
        if (s_trk_follow) trk_capture();
        snprintf(out, sizeof out, "OK TF %d\n", s_trk_follow); reply(out); return;
      case 'D':
        if (nt < 2 || !parse_long(tok[1], &i) || i < 0) break;
        if (trk_sorted(idx) == 0) { reply("ERR RANGE\n"); return; }
        if ((s_last_err = gate(&s_ax[LENS_FOCUS], true)) != E_OK) { reply_err(reply); return; }
        trk_set_distance((uint32_t)(i > 0 && i < 100 ? 100 : i));
        snprintf(out, sizeof out, "OK TD %ld\n", (long)axis_goal(&s_ax[LENS_FOCUS])); reply(out); return;
      default: break;
    }
    reply("ERR SYNTAX\n"); return;
  }
  if (!strcmp(c, "ZF")) {
    long z, mm = -1;
    uint8_t idx[TRK_MAX];
    if (nt < 2 || !parse_long(tok[1], &z) || (nt >= 3 && (!parse_long(tok[2], &mm) || mm < 0))) {
      reply("ERR SYNTAX\n"); return;
    }
    if ((s_last_err = gate(&s_ax[LENS_ZOOM], true)) != E_OK ||
        (s_last_err = gate(&s_ax[LENS_FOCUS], true)) != E_OK) { reply_err(reply); return; }
    if (trk_sorted(idx) == 0) { reply("ERR RANGE\n"); return; }
    if (nt >= 3) s_trk_dpt = mm > 0 ? 1000.0f / (float)(mm < 100 ? 100 : mm) : 0.0f;
    else trk_capture();
    uint8_t f = s_trk_follow;
    s_trk_follow = 1;
    bool ok = move_to_raw(LENS_ZOOM, (int32_t)z);
    if (ok) trk_focus_to((int32_t)z);
    s_trk_follow = f;
    if (!ok) { reply_err(reply); return; }
    snprintf(out, sizeof out, "OK ZF %ld %ld\n", z, (long)axis_goal(&s_ax[LENS_FOCUS]));
    reply(out); return;
  }
  if (!strcmp(c, "S")) { LENS_Stop(LENS_ZOOM); LENS_Stop(LENS_FOCUS); reply("OK STOP\n"); return; }

  if (!strcmp(c, "ST")) {
    const Axis_t *z = &s_ax[LENS_ZOOM], *f = &s_ax[LENS_FOCUS];
    snprintf(out, sizeof out, "ST Z %s %ld %X %X F %s %ld %X %X\n",
             s_state_name[z->state], (long)z->pos, z->fault, z->warn,
             s_state_name[f->state], (long)f->pos, f->fault, f->warn);
    reply(out); return;
  }
  if (!strcmp(c, "H") || !strcmp(c, "CLR")) {
    int w = nt > 1 ? host_axis(tok[1]) : LENS_NAXIS;
    if (w < 0) { reply("ERR SYNTAX\n"); return; }
    LensErr_t res = E_OK;
    for (int i = 0; i < LENS_NAXIS - 1; i++) {          /* zoom, focus */
      if (w != LENS_NAXIS && w != i) continue;
      if (c[0] == 'H') {
        LensErr_t e = LENS_Home((LensAxis_t)i);
        if (e != E_OK) res = e;
      } else {
        Axis_t *a = &s_ax[i];
        LENS_Halt((LensAxis_t)i); a->fault = 0; a->comm_fail = 0;
        if (!axis_configure(a)) res = E_NODRV;
        a->state = AX_UNHOMED;
      }
    }
    if (res != E_OK) { s_last_err = res; reply_err(reply); return; }
    reply(c[0] == 'H' ? "OK H\n" : "OK CLR\n"); return;
  }
  if (!strcmp(c, "ZJ") || !strcmp(c, "FJ")) {
    long d, pps = 0;
    if (nt < 2 || !parse_long(tok[1], &d) || (nt >= 3 && !parse_long(tok[2], &pps)) || pps < 0) {
      reply("ERR SYNTAX\n"); return;
    }
    LensAxis_t jax = c[0] == 'Z' ? LENS_ZOOM : LENS_FOCUS;
    if (!jog_start(jax, (int)d, (uint32_t)pps, JOG_DEADMAN_HOST_MS, true)) { reply_err(reply); return; }
    reply("OK J\n"); return;
  }
  if (!strcmp(c, "LIM")) {
    long fs;
    int w = nt > 1 ? host_axis(tok[1]) : -1;
    if (w < 0 || w == LENS_NAXIS || nt < 3 || !parse_long(tok[2], &fs) || fs <= 0 || fs > 20000) {
      reply("ERR SYNTAX\n"); return;
    }
    Axis_t *a = &s_ax[w];
    a->max_pos = (int32_t)(fs * a->mres); a->travel_known = 1; a->warn &= (uint8_t)~W_TRAVEL;
    home_fit_travel(a);
    reply("OK LIM\n"); return;
  }
  if (!strcmp(c, "SAVE")) {
    const char *e = cfg_save();
    if (e) { snprintf(out, sizeof out, "ERR SAVE %s\n", e); reply(out); }
    else reply("OK SAVE\n");
    return;
  }
  if (!strcmp(c, "ZERO")) {
    reply("ERR SYNTAX\n"); return;   /* removed: zero is set by homing (H) only */
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
  bool ok = (*sub == 'R') ? LENS_MoveRel(ax, (int32_t)v) : LENS_MoveTo(ax, (int32_t)v);
  if (!ok) { reply_err(reply); return; }
  snprintf(out, sizeof out, "OK %c %ld\n", ax == LENS_ZOOM ? 'Z' : 'F',
           (long)(s_ax[ax].pend_valid ? s_ax[ax].pend_target :
                  s_ax[ax].running ? s_ax[ax].target : s_ax[ax].pos));
  reply(out);
}
