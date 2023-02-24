#include "serial.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "main.h"
#include "cJSON.h"
uint8_t g_nRxBuff[MAX_LEN];

uint8_t g_strCommand[4];
uint8_t g_nOption[3];
uint8_t g_nData[8];
uint8_t rx_data;
char rx_buffer[200];
extern char speed_txBuffer[64]; //tx
extern uint8_t speed_rxBuffer[64]; //rx
extern bool temp;



int a = 0;
extern long last ;
//int flag=0;
	unsigned int rx_index = 0;
bool g_bDataAvailable = false;
bool flag = false;

uint8_t STX[] = {0x02U};
uint8_t ETX[] = {0x03U};
uint8_t ACK[] = {0x06U};
uint8_t SYN[] = {0x16U};

uint8_t *subString(uint8_t *pBuff, int nPos, int nIndex)
{
    uint8_t *t = &pBuff[nPos];
    pBuff[nPos - 1] = '\0';
    for (int i = nIndex; i < (strlen((char *)t) + 1); i++)
    {
        t[i] = '\0';
    }
    return t;
}

bool StrCompare(uint8_t *pBuff, uint8_t *pSample, uint8_t nSize)
{
    for (int i = 0; i < nSize; i++)
    {
        if (pBuff[i] != pSample[i])
        {
            return false;
        }
    }

    return true;
}


void SerialInit(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t *)g_nRxBuff, MAX_LEN);
	HAL_UART_Receive_IT(&huart3, (uint8_t *)speed_rxBuffer, 28);
		 HAL_UART_Receive_IT(&huart2,&rx_data,1); // giao tiep bluetooth
}

void SerialAcceptReceive(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t *)g_nRxBuff, MAX_LEN);
}
void SerialAcceptReceivespeed(void)
{
    HAL_UART_Receive_IT(&huart3, (uint8_t *)speed_rxBuffer, 28);
}

void SerialWriteComm(uint8_t *pStrCmd, uint8_t *pOpt, uint8_t *pData)
{
    uint8_t *pBuff;
    pBuff = (uint8_t *)malloc(18);
    uint8_t nIndex = 0;

    memcpy(pBuff + nIndex, STX, 1);
    nIndex += 1;
    memcpy(pBuff + nIndex, pStrCmd, 4);
    nIndex += 4;
    memcpy(pBuff + nIndex, pOpt, 3);
    nIndex += 3;
    memcpy(pBuff + nIndex, pData, 8);
    nIndex += 8;
    memcpy(pBuff + nIndex, ACK, 1);
    nIndex += 1;
    memcpy(pBuff + nIndex, ETX, 1);

    HAL_UART_Transmit(&huart1, pBuff, MAX_LEN, 1000);

    free(pBuff);
}

void SerialParse(uint8_t *pBuff)
{
    if ((pBuff[0] == STX[0]) && (pBuff[17] == ETX[0]))
    {
        memcpy(g_strCommand, subString(g_nRxBuff, 1, 4), 4);
        memcpy(g_nOption, subString(g_nRxBuff, 5, 3), 3);
        memcpy(g_nData, subString(g_nRxBuff, 8, 8), 8);
		}
		
//		     Comand[0] = g_nRxBuff[1]; 
//		     Comand[1] = g_nRxBuff[2];
//		     Comand[2] = g_nRxBuff[3];
//	       Comand[3] = g_nRxBuff[4];
//		     option[0] = g_nRxBuff[5]; 
//		     option[1] = g_nRxBuff[6];
//		     option[2] = g_nRxBuff[7];
//		     Data[0] = g_nRxBuff[8]; 
//		     Data[1] = g_nRxBuff[9];
//		     Data[2] = g_nRxBuff[10];
//		     Data[3] = g_nRxBuff[11]; 
//		     Data[4] = g_nRxBuff[12];
//		     Data[5] = g_nRxBuff[13];
//		     Data[6] = g_nRxBuff[14]; 
//		     Data[7] = g_nRxBuff[15];
	
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance)
    {    
    				
        g_bDataAvailable = true;
	
		    SerialParse(g_nRxBuff);
		}

			if (huart -> Instance == USART2)
		{  last = HAL_GetTick();
			   Received();
			
         HAL_UART_Receive_IT(&huart2,&rx_data,1);			
		}
				if (huart->Instance == huart3.Instance)
		{  
		         flag = true;
		}
	
}
void Received(void)
	{  if (rx_data != '\n')
      {  
				rx_buffer[rx_index++] = rx_data;
			}
   else			
		 { 
			 printf("Data nhan duoc: %s\n",rx_buffer);
     	 Xulyjson(rx_buffer);
			rx_index = 0;
		for (int i; i < 200; i++)
		 { rx_buffer[i] = 0;
		 }
	
		 }
	}
void Received2(void)
	{  if (rx_data != '\n')
      {  
				rx_buffer[rx_index++] = rx_data;
			}
   else			
		 { 
			// printf("Data nhan duoc: %s\n",rx_buffer);
     //	 Xulyjson(rx_buffer);
			rx_index = 0;
		for (int i; i < 200; i++)
		 { rx_buffer[i] = 0;
		 }
	
		 }
	}
	

