//#ifndef __PIDSPEEDGRAPH_H
//#define __PIDSPEEDGRAPH_H
#ifndef __PIDSPEEDGRAPH_H
#define __PIDSPEEDGRAPH_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE BEGIN PFP */


	/* USER CODE END PFP */
  extern void xuat_PWM(void) ;
	extern void PID_speed(void);
	extern void Receive_data_processing(void);
	extern int32_t ENC0_GetSpeed(void);
	extern double ConvertByte2Double(uint8_t *ptr);
	float KalmanFilter( float inData );
	 unsigned long kalman_filter( unsigned long ADC_Value );
#endif	
