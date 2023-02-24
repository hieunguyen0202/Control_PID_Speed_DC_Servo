#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "pid.h"
#include "motor.h"
#include "serial.h"
#include "cJSON.h"
#include "pidspeedgraph.h"

//xu  ly json

unsigned  int i = 0;
float SETP = 0;
float SS = 0;
float ACT = 0;
unsigned int GTKP = 0;
unsigned int GTKI = 0;
unsigned int GTKD = 0;
char Str_set_point[100];
char Str_error_data[100];			
char Str_act_pos[100];
char Str_gtrKp[100];
char Str_gtrKi[100];
char Str_gtrKd[100];
char JSON[100];  

	
cJSON *str_json,*str_set_point, *str_error_data, *str_act_pos, *str_gtrKp , *str_gtrKi, *str_gtrKd, *str_Mode;
long last = 0;


//khai bao du lieu



 void chuyendoi(void);
void SendData(unsigned int SETP , unsigned int SS , unsigned int ACT , unsigned int GTKP , unsigned int GTKI , unsigned  int GTKD);
void SystemClock_Config(void);


//truy?n prinft
#ifdef __GNUC__
     #define PUTCHAR_PROTOTYPE int __io_putchar (int ch)
#else
		 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{// CONG PC

	HAL_UART_Transmit(&huart3, (uint8_t *)&ch,1,0xFFFF);
	
	return ch;}

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern PID_CONTROL_t tPIDControl;
extern PROFILE_t tProfile;
extern PROCESS_t tProcess;
extern bool flag;
extern bool g_bDataAvailable;
extern uint8_t nTxBuff[MAX_LEN];
extern uint8_t g_strCommand[4];
extern uint8_t g_nOption[3];
extern uint8_t g_nData[8];
extern uint8_t g_nRxBuff[MAX_LEN];
extern uint32_t g_nActPulse;
extern uint32_t g_nCmdPulse;
extern uint16_t g_nIndex;
extern float g_dPIDError;
extern double speed, position;
 extern int a;
//copy 
uint8_t Comand[4];
uint8_t option[3];
uint8_t CLEAR[64];
//extern char speed_txBuffer[64]; //tx
//extern uint8_t speed_rxBuffer[64]; //rx
extern double Kp , Ki, Kd;
extern int duty, run, mode;
extern double e, ep, p_e, pp_e, pPart , iPart, dPart, udk, predPart , pwm;
extern float m_i;

 extern int setpoint;
float sect;

bool temp = false;
 int stop=0;
//uint8_t rx_data;
uint8_t g_strTxCommand[4];
uint8_t g_nTxOption[3];
uint8_t g_nTxData[8];
uint16_t g_nTuningSampleCount = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
 {
  
  HAL_Init();


  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
	 //  MX_DMA_Init();
  /* USER CODE BEGIN 2 */
  SerialInit();
  MotorInit();
  tProcess = NONE;
   last = HAL_GetTick();
  while (1)
  {      
			    if(a!=2)
		{	SerialAcceptReceivespeed();}
		else
		{
			      if (flag == true)
	  	  	{
	  	  	Receive_data_processing();
						HAL_Delay(1000);
						 flag = false;			
					SerialAcceptReceivespeed();
		       }

	  	  	if (run==1)
	  	  	{
	  	  		m_i = ENC0_GetSpeed();
	  	  		if (mode==1)
	  	  		{
						speed=m_i*60.0/0.2/2120;
//							if (speed > 65000){
//			      speed = 65535 - speed;}
								
							kalman_filter(speed);
							
	  	  		//	PIDCompute(&tPIDControl, setpoint, speed, 0.01f);
							PID_speed();
	  	  		}
	  	  	
	  	  		xuat_PWM();
	  	  		HAL_Delay(200);
	  	  	}
	  	  	else
	  	  	{
	  	    	m_i=ENC0_GetSpeed();
	  	    	setpoint=0;
	  	    	pwm=0;
	  	  		if (mode==1)
	  	  		{   
	  	  			speed=m_i*60.0/0.2/2120;
//							if (speed > 65000){
//			      speed = 65535 - speed;}
					      kalman_filter(speed);
	  	  			//PIDCompute(&tPIDControl, setpoint, speed, 0.01f);
							PID_speed();
	  	  		}
	  	  		
	  	  		udk=0;
	  	  		xuat_PWM();
	  	  		HAL_Delay(100);
	  	  	}
	            
		}
		
		if (a !=2 )
		{	if (HAL_GetTick() - last >= 1500)
		  {  
		chuyendoi();
		SendData( SETP ,  SS ,  ACT ,  GTKP ,  GTKI ,  GTKD);
			last = HAL_GetTick();
	      	
		  }
		
		}
	

    if (g_bDataAvailable == true)
    {
      if (StrCompare(g_strCommand, (uint8_t *)"SPID", 4))
      {
        tProcess = SPID;
      }
      else if (StrCompare(g_strCommand, (uint8_t *)"CTUN", 4))
      {
        tProcess = CTUN_RES;
      }
			else if (StrCompare(g_strCommand, (uint8_t *)"SECT", 4))
      {
        tProcess = SECT;
      }
			else if (StrCompare(g_strCommand, (uint8_t *)"SPED", 4))
      {
        tProcess = SPED;
      }
			else if (StrCompare(g_strCommand, (uint8_t *)"SERV", 4))
      {
        tProcess = SERV;
      }
			else if (StrCompare(g_strCommand, (uint8_t *)"STOP", 4))
      {
        tProcess = STOP;
      }
      else if (StrCompare(g_strCommand, (uint8_t *)"GPID", 4))
      {
        tProcess = GPID;
      }
      else if (StrCompare(g_strCommand, (uint8_t *)"CSET", 4))
      {
        tProcess = CSET;
      }
      else if (StrCompare(g_strCommand, (uint8_t *)"CRUN", 4))
      {
        tProcess = CRUN_RES;
      }
      else if (StrCompare(g_strCommand, (uint8_t *)"GRMS", 4))
      {
        tProcess = GRMS;
      }
      else
      {
        tProcess = NONE;
      }
			g_bDataAvailable = false;
    }
    switch (tProcess)
    {
    case NONE:
      SerialAcceptReceive();
		if(a==2)
		{	SerialAcceptReceivespeed();}
      break;
		case SECT:
			 SerialWriteComm(g_strCommand, g_nOption, g_nData);
			sect = g_nData[0] + (float)g_nData[1] / pow(10,(double)g_nData[6]);
      tPIDControl.dSet = sect;
		tProcess = NONE;
      break;
		case SPED:
			 SerialWriteComm(g_strCommand, g_nOption, g_nData);
			       a = 2;

			 tProcess = NONE;

      break;
		case SERV:
			
	
	  	
    tProcess = NONE;
	

      break;
		case STOP:
			a = 5;
		
		SerialWriteComm(g_strCommand, g_nOption, g_nData);
		setpoint=0;
	  	    	pwm=0;
		  tProcess = NONE;
	
      break;
    case SPID:
      SerialWriteComm(g_strCommand, g_nOption, g_nData);
			g_nCmdPulse = 0;
			PIDReset(&tPIDControl);
			__HAL_TIM_SetCounter(&htim4, 0);
			g_nIndex = 0;

      /* Get PID params */
      	tPIDControl.dKp = g_nData[0] + (float)g_nData[1] / pow(10,(double)g_nData[6]);
			tPIDControl.dKi = g_nData[2] + (float)g_nData[3] / pow(10,(double)g_nData[6]);
			tPIDControl.dKd = g_nData[4] + (float)g_nData[5] / pow(10,(double)g_nData[6]);
     // tPIDControl.dSet = 
      tProcess = NONE;
      break;
    case CTUN_RES:
      SerialWriteComm(g_strCommand, g_nOption, g_nData);
      tProcess = CTUN;
      break;
    case CTUN:
      break;
    case GPID:
      for (int index = 0; index < (g_nIndex - 1); index++)
      {
				sprintf((char *)g_strTxCommand, "%s", g_strCommand);
				memset(g_nTxOption, '\0', 3);
				g_nTxData[6] = (tPIDControl.nSampleTuningPID[index]&0xFF00) >> 8;
				g_nTxData[7] = (tPIDControl.nSampleTuningPID[index]&0xFF);
				g_nTxData[4] = ((uint16_t)g_dPIDError&0xFF00) >> 8;
				g_nTxData[5] = ((uint16_t)g_dPIDError&0xFF);
				g_nTxData[2] = (index&0xFF00) >> 8;
				g_nTxData[3] = (index&0xFF);
				g_nTxData[0] = 0;
				g_nTxData[1] = 199;
				
        SerialWriteComm(g_strTxCommand, g_nTxOption, g_nTxData);
        memset(g_strTxCommand, '\0', 4);
        memset(g_nTxOption, '\0', 3);
        memset(g_nTxData, '\0', 8);
        HAL_Delay(40);
      }
      g_bDataAvailable = false;
      SerialAcceptReceive();
      tProcess = NONE;
      break;
    case CSET:
      SerialWriteComm(g_strCommand, g_nOption, g_nData);

      PIDReset(&tPIDControl);
      g_nActPulse = 0;
      g_nCmdPulse = 0;
		
      // Get Pmax, Vmax, Amax
			tProfile.dAccelMax = (float)((g_nData[2] >> 4) * 4096) + (float)((g_nData[2] & 0x0F) * 256) + (float)((g_nData[3] >> 4) * 16) + (float)((g_nData[3] & 0x0F) * 1);
      tProfile.dVelMax = (float)((g_nData[4] >> 4) * 4096) + (float)((g_nData[4] & 0x0F) * 256) + (float)((g_nData[5] >> 4) * 16) + (float)((g_nData[5] & 0x0F) * 1);
      tProfile.dPosMax = (float)((g_nData[6] >> 4) * 4096) + (float)((g_nData[6] & 0x0F) * 256) + (float)((g_nData[7] >> 4) * 16) + (float)((g_nData[7] & 0x0F) * 1);

			// Calculate params for trapezoidal speed
      tProfile.dA1 = 0.5f * tProfile.dAccelMax;
      tProfile.dA2 = tProfile.dVelMax;
      tProfile.dB2 = -0.5f * tProfile.dVelMax * tProfile.dVelMax / tProfile.dAccelMax;
      tProfile.dA3 = -0.5f * tProfile.dAccelMax;
      tProfile.dB3 = tProfile.dPosMax * tProfile.dAccelMax / tProfile.dVelMax + tProfile.dVelMax;
      tProfile.dC3 = -0.5f * tProfile.dPosMax * tProfile.dPosMax * tProfile.dAccelMax / (tProfile.dVelMax * tProfile.dVelMax) - 0.5f * tProfile.dVelMax * tProfile.dVelMax / tProfile.dAccelMax;
      tProfile.dMidStep1 = tProfile.dVelMax / tProfile.dAccelMax;
      tProfile.dMidStep2 = tProfile.dPosMax / tProfile.dVelMax;
      tProfile.dMidStep3 = tProfile.dMidStep1 + tProfile.dMidStep2;

      tProfile.nTime = 0;
      tProcess = NONE;
      break;
    case CRUN_RES:
      SerialWriteComm(g_strCommand, g_nOption, g_nData);
			g_nCmdPulse = 0;
			PIDReset(&tPIDControl);
			__HAL_TIM_SetCounter(&htim4, 0);
			g_nIndex = 0;
      tProcess = CRUN;
      break;
    case CRUN:
      g_bDataAvailable = false;
      SerialAcceptReceive();
      break;
    case GRMS:
			for (int index = 0; index < (g_nIndex - 1); index++)
      {
				sprintf((char *)g_strTxCommand, "%s", g_strCommand);
				memset(g_nTxOption, '\0', 3);
				g_nTxData[6] = (tPIDControl.nActPosSample[index]&0xFF00) >> 8;
				g_nTxData[7] = (tPIDControl.nActPosSample[index]&0xFF);
				g_nTxData[4] = ((uint16_t)g_dPIDError&0xFF00) >> 8;
				g_nTxData[5] = ((uint16_t)g_dPIDError&0xFF);
				g_nTxData[2] = (index&0xFF00) >> 8;
				g_nTxData[3] = (index&0xFF);
				g_nTxData[0] = ((g_nIndex - 2)&0xFF00) >> 8;
				g_nTxData[1] = ((g_nIndex - 2)&0xFF);
				
				
        SerialWriteComm(g_strTxCommand, g_nTxOption, g_nTxData);
        memset(g_strTxCommand, '\0', 4);
        memset(g_nTxOption, '\0', 3);
        memset(g_nTxData, '\0', 8);
        HAL_Delay(50);
      }
			
      g_bDataAvailable = false;
      SerialAcceptReceive();
      tProcess = NONE;
      break;
    }
  } //end while
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Timer 2: 10ms */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim2.Instance)
  {
    switch (tProcess)
    {
    case NONE:
      break;
    case SPID:
      break;
		case CTUN_RES:
			break;
		case SECT:
			break;
    case CTUN:
			if (g_nIndex <= 200)
			{
			//	MotorTuning(25);
			//	PID_CONTROL_t *PID_Set;
				MotorTuning(sect);
				//MotorTuning(25);
			}
      else
			{
				tProcess = NONE;
				MotorSetDuty(0);
			}
      break;
    case GPID:
      break;
    case CSET:
      break;
		case SPED:
			
      break;
		case SERV:
			break;
		case STOP:
			break;
		case CRUN_RES:
			break;
    case CRUN:
      MotorMovePos();
      break;
    case GRMS:
      break;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
 
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//	{ 
//		
//	}
	
	
void chuyendoi(void)
	{ 
		

		  SS = g_dPIDError; 
		SETP = g_nCmdPulse;
		ACT = g_nCmdPulse - g_dPIDError;
//		PID_CONTROL_t *PID_Ctrl;
		GTKP = tPIDControl.dKp;
		GTKI = tPIDControl.dKi;
		GTKD = tPIDControl.dKd;
//		else
//		{
////////			SS = e; 
////////		SETP = setpoint;
////////		ACT = speed;
////////		GTKP = Kp;
////////		GTKI = Ki;
////////		GTKD = Kd;
//		SS = 6; 
//		SETP = 8;
//		ACT = 7;
//		//PID_CONTROL_t *PID_Ctrl;
//		GTKP = 56;
//		GTKI = 89;
//		GTKD = 44;
//		}

	}



	void Xulyjson(char *Datajson)
		{ 
			str_json = cJSON_Parse(Datajson);
			if(!str_json)
				{ 
					printf(" JSON ERROR\r\n");
					return;
					
				}
				else
					{
						printf(" JSON ok\r\n");
						if (cJSON_GetObjectItem(str_json, ""))
						{ 
							uint8_t Setpoint_P = atoi(cJSON_GetObjectItem(str_json, "")->valuestring);
						}
						else if (cJSON_GetObjectItem(str_json, ""))
						{}
						cJSON_Delete(str_json);
					}
			
		}

	
	void SendData(unsigned int SETP , unsigned int SS , unsigned int ACT , unsigned int GTKP , unsigned int GTKI , unsigned  int GTKD)

  { // {"ND":DA":"gia tri do am"} 
	for(int i=0 ; i<100 ; i++)
	{ 
   JSON[i] = 0;
		Str_set_point[i] = 0;
		Str_error_data[i] = 0;
		Str_act_pos[i] = 0;
		Str_gtrKp[i] = 0;
		Str_gtrKi[i] = 0;
		Str_gtrKd[i] = 0;}
	// truyen du lieu vao char
	sprintf(Str_set_point, "%d", SETP);
	sprintf(Str_error_data, "%d", SS);
	sprintf(Str_act_pos, "%d", ACT);
	sprintf(Str_gtrKp, "%d", GTKP);
	sprintf(Str_gtrKi, "%d", GTKI);
	sprintf(Str_gtrKd, "%d", GTKD);
		//{"SETP":"123","SS":"456","ACT":"1","GTKP":"2","GTKI":"2","GTKD":"2"}
 //  strcat(JSON,"{\"ND\":\"");
		strcat(JSON, "{\"SETP\":\"");
   strcat(JSON, Str_set_point);
	 strcat(JSON,"\",\"SS\":\"");
   strcat(JSON, Str_error_data);
   strcat(JSON, "\",\"ACT\":\"");
   strcat(JSON,Str_act_pos);
		strcat(JSON, "\",\"GTKP\":\"");
		 strcat(JSON,Str_gtrKp);
		strcat(JSON, "\",\"GTKI\":\"");
		strcat(JSON,Str_gtrKi);
    strcat(JSON,"\",\"GTKD\":\"");
		strcat(JSON,Str_gtrKd);
		strcat(JSON,"\"}\n");
   printf("%s", JSON);
	 //send du lieu cho esp8266
	 HAL_UART_Transmit(&huart2,(uint8_t*)&JSON,strlen(JSON),1000);
	
	}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
