#include "motor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "tim.h"
#include "pid.h"

/* PID */
PROCESS_t tProcess;
PID_CONTROL_t tPIDControl;
PROFILE_t tProfile;

int32_t g_nDutyCycle;
float g_dCmdVel;
uint32_t g_nActPulse;
uint32_t g_nCmdPulse;
uint16_t g_nIndex = 0;

void MotorSetDir(int8_t nDir)
{
	switch (nDir)
	{
	case 0: // CW
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		break;
	case 1: // CCW
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		break;
	default:
		break;
	}
}

void MotorSetDuty(uint16_t nDuty)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nDuty);
}

void MotorInit(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

	PIDReset(&tPIDControl);
	PIDInit(&tPIDControl, 1., 0., 0.00);
	MotorSetDir(1);
}

uint16_t ConvertDegToPulse(uint16_t nDeg)
{
	float dPulse = nDeg * 4 * 210 / 360;

	return (uint16_t)dPulse;
}

uint16_t ConvertPulseToDeg(uint16_t nPulse)
{
	float dDeg = nPulse * 360 / (4 *210);
	return (uint16_t)dDeg;
}

void MotorGetPulse(uint32_t *nPulse)
{
	*nPulse = __HAL_TIM_GetCounter(&htim4);
}

void MotorTuning(uint16_t nPos)
{
	uint32_t nPulse;
	MotorGetPulse(&nPulse);
	g_nActPulse = nPulse - 0;

	g_nCmdPulse = ConvertDegToPulse(nPos);
	//g_nSetPulse = g_nCmdPulse;
	if (g_nActPulse > 65000){
			g_nActPulse = 65535 - g_nActPulse;
		}
	
	g_nDutyCycle = (int16_t)PIDCompute(&tPIDControl, g_nCmdPulse, g_nActPulse, 0.01f);
	if (g_nDutyCycle >= 0)
	{
		MotorSetDir(1);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	else if (g_nDutyCycle < 0)
	{
		MotorSetDir(0);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	
	// Store data
	tPIDControl.nSampleTuningPID[g_nIndex] = g_nActPulse;
	g_nIndex++;
}

void MotorMovePos(void)
{
	uint32_t nPulse;
	MotorGetPulse(&nPulse);
	g_nActPulse = nPulse ;
	float dPosTemp = 0;
	
	// Profile Trapezoidal Speed
	if (tProfile.nTime <= tProfile.dMidStep1)
	{
		dPosTemp = (int32_t)(tProfile.dA1 * tProfile.nTime * tProfile.nTime);
		g_dCmdVel = 2 * tProfile.dA1 * tProfile.nTime;
	}
	else if (tProfile.nTime <= tProfile.dMidStep2)
	{
		dPosTemp = (int32_t)(tProfile.dA2 * tProfile.nTime + tProfile.dB2);
		g_dCmdVel = tProfile.dA2;
	}
	else if (tProfile.nTime <= tProfile.dMidStep3)
	{
		dPosTemp = (int32_t)(tProfile.dA3 * tProfile.nTime * tProfile.nTime + tProfile.dB3 * tProfile.nTime + tProfile.dC3);
		g_dCmdVel = 2 * tProfile.dA3 * tProfile.nTime + tProfile.dB3;
	}
	else
	{
		dPosTemp = tProfile.dPosMax;
	}

	// Control PID
	g_nCmdPulse = ConvertDegToPulse(dPosTemp);
	g_nDutyCycle = (int16_t)PIDCompute(&tPIDControl, g_nCmdPulse, g_nActPulse, 0.01f);
	if (g_nDutyCycle >= 0)
	{
		MotorSetDir(1);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	else if (g_nDutyCycle < 0)
	{
		MotorSetDir(0);
		MotorSetDuty(abs(g_nDutyCycle));
	}

	if (tProfile.nTime > tProfile.dMidStep3)
	{
		__HAL_TIM_SetCounter(&htim4, 0);
		dPosTemp = 0;
		g_nDutyCycle = 0;
		g_dCmdVel = 0;
		tProfile.nTime = 0;
		tProcess = NONE;
		MotorSetDuty(abs(g_nDutyCycle));
	}
	else
	{
		// Store data
		tPIDControl.nActPosSample[g_nIndex] = ConvertPulseToDeg(g_nActPulse);
		g_nIndex++;
	}

	// Compute timer count for control PID
	tProfile.nTime += 0.01;
}
