#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "tim.h"
#include "usart.h"
#include "pidspeedgraph.h"
#include "motor.h"
#include "serial.h"

 // extern bool flag;
	char speed_txBuffer[64] ; //tx
	uint8_t speed_rxBuffer[64]; //rx
	double speed, position;
	double Kp =0, Ki=0, Kd =0;
	int duty=0;
	double e, ep, p_e=0, pp_e=0, pPart=0 , iPart=0, dPart=0, udk, predPart = 0, pwm, intergralspeed, pre_e;
	float m_i;
	double setpoint;
	int  run=0, mode=0;
		float dErrorTerm1, dErrorTerm2;
	float alpha, beta, gamma, delta, dPIDResult, dPIDResultTerm; 
void xuat_PWM(void) 
{
//duty=(int)pwm;
		if (pwm> 9000)
  {	 duty=(int)pwm-10000;}
	else{duty=(int)pwm;}
	if (duty>=0)
		{

	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,duty);
		}
	else
		{
	
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 10000+duty);
		}
	if (mode==1)
	{
    sprintf(speed_txBuffer,"S,%ld,%ld,%ld,%ld\r\n",(long)(setpoint),(long)(speed*100),(long)(udk*100),(long)(e*1000));
	}
//	else
//	{
//		sprintf(speed_txBuffer,"S,%ld,%ld,%ld,%ld\r\n",(long)(setpoint),(long)(position*100),(long)(udk*100),(long)(ep*1000));
//	}
	HAL_UART_Transmit(&huart3,(uint8_t*)speed_txBuffer,strlen(speed_txBuffer),30);	
}

int32_t ENC0_GetSpeed(void)
{
    static int32_t p = 0, p_pre = 0, dp = 0;
//    p = (int32_t)TIM4->CNT;
	 p = (int32_t)__HAL_TIM_GetCounter(&htim4);
    dp = p - p_pre; //+/- (speed)
    if (dp > 32768)
        dp -= 65536;
    else if (dp < -32768)
        dp += 65536;
    p_pre = p;

    return dp;
	}

	
void PID_speed(void)
{  //int m = 10000;
	//bo ban dau
// e = setpoint - speed;
// ep=e*100/setpoint;
// if (ep>0) ep=ep;
// else ep=-ep;
// pPart = Kp * e;
//iPart += 0.5 * 0.2* Ki * e;
// dPart = Kd / 0.2* e - predPart;
// predPart = Kd / 0.2 * e;
// udk = pPart + iPart + dPart;
//		pwm = udk;
 /* Saturate */
// if (udk > m*9/10)
// pwm = m*9/10;
// else if (udk < -m*9/10)
// pwm = -m*9/10;
// else pwm = udk;
// udk=pwm*100.0/m;

	
	// bo luc sau
//	e = setpoint - speed;
//	pPart = Kp * e;
//	intergralspeed += e;
//	iPart = Ki * (0.01f) / 2 * intergralspeed;
//	dPart = Kd * (e - pre_e) / 0.01f;
//	udk = pPart + iPart + dPart;
//	pre_e = e;
//	pwm = udk;

   float dTs =0.2f;
////..............pid of hiengymer
e = setpoint - speed;
	alpha = 2*dTs*Kp + Ki*dTs*dTs + 2*Kd;
	beta = Ki*dTs*dTs - 4*Kd - 2*dTs*Kp;
    gamma = 2*Kd;
   delta = 2*dTs;
dPIDResult = (alpha*e + beta*dErrorTerm1 + gamma*dErrorTerm2 + delta*dPIDResultTerm)/delta;
dPIDResultTerm = dPIDResult;
dErrorTerm2 = dErrorTerm1;
dErrorTerm1 = e;

pwm = dPIDResult;
 /* Saturate */
// if (dPIDResult > m*9/10)
// pwm = m*9/10;
// else if (dPIDResult < -m*9/10)
// pwm = -m*9/10;
// else pwm = dPIDResult;
// dPIDResult=pwm*100.0/m;
}

void Receive_data_processing(void)
{ HAL_Delay(100);
	run=speed_rxBuffer[0];
	if (run == 0)
	{
		pwm = 0;
		setpoint = 0;
	};
	
	mode=speed_rxBuffer[1];
	setpoint=speed_rxBuffer[3]<<8|speed_rxBuffer[2];
	if (setpoint>32768) setpoint-=65536;
	int i=0;
	uint8_t nbytes[8] = {0,0,0,0,0,0,0};
	for(i=0;i<8;i++) {nbytes[i] = speed_rxBuffer[4+i];}
	   Kp = ConvertByte2Double(nbytes);

    for(i=0;i<8;i++) nbytes[i] = speed_rxBuffer[12+i];
    Ki = ConvertByte2Double(nbytes);

	for(i=0;i<8;i++) nbytes[i] = speed_rxBuffer[20+i];
	Kd = ConvertByte2Double(nbytes);
	    //flag = false;
}

double ConvertByte2Double(uint8_t *ptr)
{
  double a=0;
memcpy(&a, ptr, sizeof(double));
	return a;
}


float KalmanFilter( float inData )

{
static float prevData = 0;                                 //?????

static float p = 10, q = 0.001, r = 0.001, kGain = 0;      // q ???? r ??????

p = p + q;

kGain = p / ( p + r );                                      //???????

inData = prevData + ( kGain * ( inData - prevData ) );      //?????????

p = ( 1 - kGain ) * p;                                      //??????

prevData = inData;

return inData;                                             //?????
}

 unsigned long kalman_filter( unsigned long ADC_Value )

{
float LastData;

float NowData;

float kalman_adc;

static float kalman_adc_old = 0;

static float P1;

static float Q = 0.0003;

static float R = 5;

static float Kg = 0;

static float P = 1;

NowData = ADC_Value;

LastData = kalman_adc_old;

P = P1 + Q;

Kg = P / ( P + R );

kalman_adc = LastData + Kg * ( NowData - kalman_adc_old );

P1 = ( 1 - Kg ) * P;

P = P1;

kalman_adc_old = kalman_adc;

return ( unsigned long )( kalman_adc );
}
