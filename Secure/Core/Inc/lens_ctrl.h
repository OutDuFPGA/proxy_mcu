/**
  ******************************************************************************
  * @file    lens_ctrl.h
  * @brief   Zoom / focus stepper control for the varifocal lens on J11
  *          (rewrite 2026-11-17; the previous version is in
  *          backup/2026-11-17_lens_ctrl_scrapped/).
  *
  * Lens  : Foctek D14-0622IR 6-22 mm, iris fixed. PM steppers, 40 ohm coils,
  *         rated 4 V, max start 800 pps, max slew 1200 pps (full steps).
  * Drivers: TMC2300 x2 in STEP/DIR mode, configured over single-wire UART
  *         (USART1 / PA9, 9600 baud):
  *           U1  TIM3 CH2 / PB5 STEP, PB4 DIR, PB3 EN, UART addr 0
  *           U2  TIM8 CH2 / PC7 STEP, PC8 DIR, PA8 EN, UART addr 2
  *         U5 (iris, TIM4 / PD12) is not fitted to a motor: held disabled.
  *
  * Units : position in MICROSTEPS (16 per full step), counted from power-up
  *         until "home" or "zero" sets a reference. Speeds in FULL steps/s.
  *         Direction 1 = forward (position increases), 0 = reverse.
  *
  * Kept deliberately small: no StallGuard, no flash storage, no tracking
  * table. Soft limits are only enforced once the axis has a reference.
  ******************************************************************************
  */
#ifndef LENS_CTRL_H
#define LENS_CTRL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum { LENS_ZOOM = 0, LENS_FOCUS = 1, LENS_IRIS = 2, LENS_NAXIS = 3 } LensAxis_t;

/* Which driver turns which motor. 2026-11-17 (docs/lens-zoom-settings.md):
   the cables are crossed at the lens connector, so U1 turns the FOCUS motor
   and U2 the ZOOM motor. Set to 0 if the wiring is corrected. */
#ifndef LENS_SWAP_ZOOM_FOCUS
#define LENS_SWAP_ZOOM_FOCUS  1
#endif
#define LENS_AXIS_U1  (LENS_SWAP_ZOOM_FOCUS ? LENS_FOCUS : LENS_ZOOM)   /* TIM3 */
#define LENS_AXIS_U2  (LENS_SWAP_ZOOM_FOCUS ? LENS_ZOOM : LENS_FOCUS)   /* TIM8 */

typedef void (*LensPrintFn)(const char *str);

void LENS_Init(LensPrintFn print);          /* drivers + timers; prints a boot report */
void LENS_Task(void);                       /* main loop: move-done events, jog timeout */

/* Motion (non-blocking). Return false if rejected. */
bool LENS_MoveTo(LensAxis_t ax, int32_t pos);
bool LENS_MoveRel(LensAxis_t ax, int32_t delta);
bool LENS_Jog(LensAxis_t ax, int dir, uint32_t pps);   /* until stop/limit/timeout; pps 0 = max */
void LENS_Stop(LensAxis_t ax);              /* decelerated stop */
void LENS_Halt(LensAxis_t ax);              /* immediate stop */
bool LENS_IsMoving(LensAxis_t ax);
int32_t LENS_GetPos(LensAxis_t ax);

/* Called from LENS_Task when an axis stops.
   reason: "done", "stop", "limit", "timeout", "homed". */
typedef void (*LensEventFn)(LensAxis_t ax, int32_t pos, const char *reason);
void LENS_SetEventHandler(LensEventFn fn);

/* Console: "lens ..." commands plus zoom_run/focus_run/... Returns true if handled. */
bool LENS_CLI(const char *cmd, LensPrintFn print);

/* SoC host protocol line handler (UART4 / J12). */
void LENS_HostCommand(const char *line, LensPrintFn reply);

/* Called from TIM3_IRQHandler / TIM8_UP_IRQHandler / TIM4_IRQHandler. */
void LENS_TimerIRQ(LensAxis_t ax);

extern const char *const LENS_AxisName[LENS_NAXIS];

#endif /* LENS_CTRL_H */
