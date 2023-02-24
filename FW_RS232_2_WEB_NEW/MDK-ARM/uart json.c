#include "main.h"
#include "stdio.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "cJSON.h"
#include "uart json.h"
#include "pid.h"
#include "motor.h"
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
char rx_buffer[200];
	unsigned int rx_index = 0;
uint8_t rx_data;	
cJSON *str_json,*str_set_point, *str_error_data, *str_act_pos, *str_gtrKp , *str_gtrKi, *str_gtrKd, *str_Mode;
long last = 0;

void chuyendoi(void)
	{  SS = g_dPIDError; 
		SETP = g_nSetPulse;
		ACT = g_nSetPulse - g_dPIDError;
		PID_CONTROL_t *PID_Ctrl;
		GTKP = PID_Ctrl->dKp;
		GTKI = PID_Ctrl->dKi;
		GTKD = PID_Ctrl->dKd;
	}

void Received(void)
	{  if (rx_data != '\n')
      {  
				rx_buffer[rx_index++] = rx_data;
			}
   else			
		 { 
			 printf("Data nhan duoc: %s\n",rx_buffer);
		//	 Xulyjson(rx_buffer);
			 clearbufferEnd();
			 
		 }
	}

//	void Xulyjson(char *Datajson)
//		{ 
//			str_json = cJSON_Parse(Datajson);
//			if(!str_json)
//				{ 
//					printf(" JSON ERROR\r\n");
//					return;
//					
//				}
//				else
//					{
//						printf(" JSON ok\r\n");
//					}
//			
//		}
void clearbufferEnd(void)
	{ rx_index = 0;
		for (int i; i < 200; i++)
		{ rx_buffer[i] = 0;
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
	 strcat(JSON,",\"SS\":\"");
   strcat(JSON, Str_error_data);
   strcat(JSON, "\",\"ACT\":\"");
   strcat(JSON,Str_act_pos);
		strcat(JSON, "\",\"GTKP\":\"");
		 strcat(JSON,Str_gtrKp);
		strcat(JSON, "\",\"GTKI\":\"");
		strcat(JSON,Str_gtrKi);
    strcat(JSON,"\",\"GTKD\":\"");
		strcat(JSON,Str_gtrKd);
		strcat(JSON,"\"}");
   printf("%s\n", JSON);
	
	}
