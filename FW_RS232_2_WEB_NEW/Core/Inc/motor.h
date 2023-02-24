#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>
//uint32_t g_nSetPulse;
extern void MotorSetDir(int8_t nDir);
extern void MotorSetDuty(uint16_t nDuty);
extern void MotorInit(void);
extern uint16_t ConvertDegToPulse(uint16_t nDeg);
extern uint16_t ConvertPulseToDeg(uint16_t nPulse);
extern void MotorGetPulse(uint32_t *nPulse);
extern void MotorMovePos(void);
extern void MotorTuning(uint16_t nPos);

#endif
