/******************************************************************************************************************
@File:  	DHT Sensor
@Author:  Khue Nguyen
@Website: khuenguyencreator.com
@Youtube: https://www.youtube.com/channel/UCt8cFnPOaHrQXWmVkk-lfvg
Huong dan su dung:
- Su dung thu vien HAL
- Khoi tao bien DHT : DHT_Name DHT1;
- Khoi tao chan DHT:
	DHT_Init(&DHT1, DHT11, &htim4, DHT_GPIO_Port, DHT_Pin);
- Su dung cac ham phai truyen dia chi cua DHT do: 
	DHT_ReadTempHum(&DHT1);
******************************************************************************************************************/
#include "dht.h"
#include <stdio.h>

#define MAX_TIMEOUT    1000 // us

static void DHT_DelayInit(DHT_Name* DHT)
{
	HAL_TIM_Base_Start(DHT->Timer);
}

static void DHT_DelayUs(DHT_Name* DHT, uint16_t time_us)
{
	__HAL_TIM_SET_COUNTER(DHT->Timer, 0);
	while (__HAL_TIM_GET_COUNTER(DHT->Timer) < time_us) {}
}

static void DHT_SetPinOut(DHT_Name* DHT)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT->Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_DeInit(DHT->Port, DHT->Pin);
	HAL_GPIO_Init(DHT->Port, &GPIO_InitStruct);
}

static void DHT_SetPinIn(DHT_Name* DHT)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT->Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_DeInit(DHT->Port, DHT->Pin);
	HAL_GPIO_Init(DHT->Port, &GPIO_InitStruct);
}

static void DHT_WritePin(DHT_Name* DHT, uint8_t Value)
{
	HAL_GPIO_WritePin(DHT->Port, DHT->Pin, Value);
}

static uint8_t DHT_ReadPin(DHT_Name* DHT)
{
	uint8_t Value = HAL_GPIO_ReadPin(DHT->Port, DHT->Pin);
	return Value;
}
//********************************* Middle level Layer ****************************************************//
static uint8_t DHT_Start(DHT_Name* DHT)
{
	uint8_t Response = 0;
	uint32_t micros_ticks = 0;

	DHT_SetPinOut(DHT);  
	DHT_WritePin(DHT, 0);
	DHT_DelayUs(DHT, DHT->Type);    
	DHT_SetPinIn(DHT);
	DHT_DelayUs(DHT, 40);

	if (!DHT_ReadPin(DHT))
	{
		DHT_DelayUs(DHT, 80); 
		if (DHT_ReadPin(DHT))
		{
			Response = DHT_STA_OK;
		}
		else 
		{
			Response = DHT_STA_FAIL;
		}
	}		
	while (DHT_ReadPin(DHT)) {
		if (micros_ticks++ > MAX_TIMEOUT) {
			return DHT_STA_TIMEOUT_ERROR;
		}
		DHT_DelayUs(DHT, 1); // 1us
	}
	return Response;
}

static uint8_t DHT_Read(DHT_Name* DHT)
{
	uint8_t value = 0;
	uint32_t micros_ticks = 0;

	DHT_SetPinIn(DHT);

	for (int i = 0; i < 8; i++)
	{
		while (!DHT_ReadPin(DHT)) {
			if (micros_ticks++ > MAX_TIMEOUT) {
				return DHT_STA_TIMEOUT_ERROR;
			}
			DHT_DelayUs(DHT, 1); // 1us
		}
		micros_ticks = 0;

		DHT_DelayUs(DHT, 40);

		if (!DHT_ReadPin(DHT)) {
			value &= ~(1 << (7-i));
		} else {
			value |= 1 << (7-i);
		}

		while (DHT_ReadPin(DHT)) {
			if (micros_ticks++ > MAX_TIMEOUT) {
				return DHT_STA_TIMEOUT_ERROR;
			}
			DHT_DelayUs(DHT, 1); // 1us
		}
	}
	return value;
}

//************************** High Level Layer ********************************************************//
void DHT_Init(DHT_Name* DHT, uint8_t DHT_Type, TIM_HandleTypeDef* Timer, GPIO_TypeDef* DH_PORT, uint16_t DH_Pin)
{
	if (DHT_Type == DHT11) {
		DHT->Type = DHT11_STARTTIME;
	}
	else if (DHT_Type == DHT22) {
		DHT->Type = DHT22_STARTTIME;
	}

	DHT->Port = DH_PORT;
	DHT->Pin = DH_Pin;
	DHT->Timer = Timer;
	DHT_DelayInit(DHT);
}

dht_status DHT_ReadTempHum(DHT_Name* DHT)
{
	dht_status status;
	uint8_t Temp1, Temp2, RH1, RH2, SUM;
	uint16_t Temp, Humi;

	DHT->Respone = DHT_Start(DHT);
	RH1 = DHT_Read(DHT);
	RH2 = DHT_Read(DHT);
	Temp1 = DHT_Read(DHT);
	Temp2 = DHT_Read(DHT);
	SUM = DHT_Read(DHT);

	if (SUM == (uint8_t)(RH1 + RH2 + Temp1 + Temp2)) {
		Temp = (Temp1 << 8) | Temp2;
		Humi = (RH1 << 8) | RH2;
		DHT->Temp = (float)(Temp / 10.0);
		DHT->Humi = (float)(Humi / 10.0);
		status = DHT_STA_OK;
    }
	else {
		DHT->Temp = -1;
		DHT->Humi = -1;
		status = DHT_STA_CRC_ERROR;
	}

//	char str[50];
//	snprintf(str, sizeof(str), "DHT_Type = %d, Temp = %0.1f, Humi = %0.1f\n", DHT->Type, DHT->Temp, DHT->Humi);
//	printf("%s", str);

	return status;
}
