#ifndef __TMC2300_H
#define __TMC2300_H

#include "main.h"

typedef enum {
    MOTOR_ZOOM,
    MOTOR_FOCUS,
    MOTOR_IRIS
} MotorType_t;

void TMC2300_Init(void);
void TMC2300_SetEnable(MotorType_t motor, uint8_t state);

/* Hardware PWM Functions */
void TMC2300_Run(MotorType_t motor, uint8_t direction, uint32_t speed_hz);
void TMC2300_Stop(MotorType_t motor);

/* Diagnostics & Protection */
uint8_t TMC2300_ReadDiag(MotorType_t motor);
void TMC2300_DumpRegisters(void);
void TMC2300_StallGuard_Poll(void); // <-- NEW

#endif /* __TMC2300_H */
