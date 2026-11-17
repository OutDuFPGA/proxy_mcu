/**
  ******************************************************************************
  * @file    lens_ctrl.h
  * @brief   Zoom / focus (/ iris) stepper control for the varifocal lens on J11,
  *          driven by three TMC2300 STEP/DIR drivers.
  *
  * Lens  : NV-0622IR(5MP)-M14, 6-22 mm F1.6, iris FIXED.
  * Motors: D14-0622IR(5MP)-EE, 2-phase PM stepper, gearbox 1:80.3,
  *         coil 40 ohm, rated 4.0 V (~100 mA/phase),
  *         max start 800 pps, max slew 1200 pps (FULL steps, at 4 V),
  *         torque 2.0 gf-cm @ 300 pps.  Step angle / travel not specified.
  * J11   : 1 FOCUS A+  2 FOCUS A-  3 ZOOM A+  4 ZOOM A-
  *         5 ZOOM B+   6 ZOOM B-   7 FOCUS B- 8 FOCUS B+   (matches the lens)
  *
  * Units : position is in MICROSTEPS (MRES, default 16 per full step) counted
  *         from the power-up position, which the user defines as zero (the lens
  *         parks at its initial position).  Speeds/acceleration are in FULL
  *         steps per second (pps), the datasheet's unit.
  *         Direction 1 = forward (position increases), 0 = reverse.
  *
  * Step generation: one hardware timer per axis (zoom TIM3 CH2 / PB5,
  *         focus TIM8 CH2 / PC7, iris TIM4 CH1 / PD12) at 1 MHz in PWM2 mode,
  *         so each timer period ends with exactly one complete STEP pulse; the
  *         update interrupt counts it and recomputes the next period
  *         (trapezoidal ramp, preloaded ARR/CCR).
  ******************************************************************************
  */
#ifndef LENS_CTRL_H
#define LENS_CTRL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum { LENS_ZOOM = 0, LENS_FOCUS = 1, LENS_IRIS = 2, LENS_NAXIS = 3 } LensAxis_t;

/* Which TMC2300 drives which lens motor. 1 = crossed, 0 = as schematic.
   2026-11-14, checked on the camera image: driver U1 (schematic "ZOOM",
   TIM3/PB5, UART addr 0) changes the image scale (zoom group) and U2
   (schematic "FOCUS", TIM8/PC7, addr 1) only changes sharpness (focus
   group) - i.e. as the schematic. The 2026-11-13 bench guess (crossed) was
   wrong. */
#ifndef LENS_SWAP_ZOOM_FOCUS
#define LENS_SWAP_ZOOM_FOCUS  0
#endif
#define LENS_AXIS_U1  (LENS_SWAP_ZOOM_FOCUS ? LENS_FOCUS : LENS_ZOOM)   /* TIM3 */
#define LENS_AXIS_U2  (LENS_SWAP_ZOOM_FOCUS ? LENS_ZOOM : LENS_FOCUS)   /* TIM8 */

typedef void (*LensPrintFn)(const char *str);

void LENS_Init(LensPrintFn print);          /* drivers + timers; prints a boot report */
void LENS_Task(void);                       /* main loop: stall poll, move-done events */

/* Motion (non-blocking). Return false if rejected (busy limit, bad axis...). */
bool LENS_MoveTo(LensAxis_t ax, int32_t pos);
bool LENS_MoveRel(LensAxis_t ax, int32_t delta);
bool LENS_Jog(LensAxis_t ax, int dir, uint32_t pps);   /* until stop/limit; pps 0 = default */
void LENS_Stop(LensAxis_t ax);              /* decelerated stop */
void LENS_Halt(LensAxis_t ax);              /* immediate stop */
bool LENS_IsMoving(LensAxis_t ax);
int32_t LENS_GetPos(LensAxis_t ax);

/* Move-complete notification: called from LENS_Task when an axis stops.
   reason: "done", "stop", "limit", "stall". */
typedef void (*LensEventFn)(LensAxis_t ax, int32_t pos, const char *reason);
void LENS_SetEventHandler(LensEventFn fn);

/* Console: "lens ..." command set (also old zoom_run/focus_run/... aliases). */
bool LENS_CLI(const char *cmd, LensPrintFn print);

/* SOC host protocol line handler (UART4 / J12). */
void LENS_HostCommand(const char *line, LensPrintFn reply);

/* Called from TIM3_IRQHandler / TIM8_UP_IRQHandler / TIM4_IRQHandler. */
void LENS_TimerIRQ(LensAxis_t ax);

extern const char *const LENS_AxisName[LENS_NAXIS];

#endif /* LENS_CTRL_H */
