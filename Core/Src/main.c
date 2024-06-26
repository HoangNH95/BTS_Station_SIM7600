/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "timer.h"
#include "dht.h"
#include "lm35.h"
#include "mq135.h"
#include "button.h"
#include "door_sensor.h"
#include "rainwater.h"
#include "fanctrl.h"
#include "irctrl.h"
#include "simcom.h"
#include "eventSerialCb.h"
#include "eventButtonCb.h"
#include "utilities.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PUBLISHER_NORMAL_TIMEOUT		30000 // ms
#define PUBLISHER_WARNING_TIMEOUT		15000 // ms
#define SUBSCRIBER_TIMEOUT		         5000 // ms
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
static char json_buffer[512];

static uint8_t publisherTimerEvt = NO_TIMER;
static uint8_t subscriberTimerEvt = NO_TIMER;

uint8_t tempSet_Room = 30;
uint8_t tempSet_AQ = 30;
uint8_t tempSet_Cabinet = 75;
uint8_t tempAirCondition = 30;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t adcMultiChannelGetValue(uint8_t channel);
static void publisherTimerCb(void *arg);
static void subscriberTimerCb(void *arg);
void subscriberHandlerCb(uint8_t *data, int32_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adc_buffer[ADC_CHANNEL_SIZE];
DHT_Name dht11_id0, dht11_id1;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  SIMCOM_Init(subscriberHandlerCb);

  /* The sensors are using ADC interface */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_CHANNEL_SIZE);
  LM35_Init(adcMultiChannelGetValue);
  MQ135_Init(adcMultiChannelGetValue);
  RainWater_Init(adcMultiChannelGetValue);

  /* The sensors are using LIN protocol */
  DHT_Init(&dht11_id0, DHT11, &htim1, DHT11_ID0_GPIO_Port, DHT11_ID0_Pin);
  DHT_Init(&dht11_id1, DHT11, &htim1, DHT11_ID1_GPIO_Port, DHT11_ID1_Pin);
  eventSerial_init(); // Get value temp of dht22 from rs232 interface

  EventButton_Init();

  publisherTimerEvt = TimerStart("publisherTimerCb", PUBLISHER_NORMAL_TIMEOUT, TIMER_REPEAT_FOREVER, publisherTimerCb, NULL);
//  subscriberTimerEvt = TimerStart("subscriberTimerCb", SUBSCRIBER_TIMEOUT, TIMER_REPEAT_FOREVER, subscriberTimerCb, NULL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  processTimerScheduler();
	  processSerialReceiver();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DHT11_ID0_Pin|DHT11_ID1_Pin|DHT22_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_NETWORK_Pin|LED_USER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WATER_POWER_Pin DHT11_ID0_Pin DHT11_ID1_Pin DHT22_Pin */
  GPIO_InitStruct.Pin = DHT11_ID0_Pin|DHT11_ID1_Pin|DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED_NETWORK_Pin LED_USER_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_NETWORK_Pin|LED_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_FAN_ON_Pin BTN_FAN_OFF_Pin BTN_IR_UP_Pin BTN_IR_DOWN_Pin
                           BTN_IR_ON_Pin BTN_IR_OFF_Pin */
  GPIO_InitStruct.Pin = BTN_FAN_ON_Pin|BTN_FAN_OFF_Pin|BTN_IR_UP_Pin|BTN_IR_DOWN_Pin
                          |BTN_IR_ON_Pin|BTN_IR_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_OUT_Pin */
  GPIO_InitStruct.Pin = FAN_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAN_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DOOR_SENSOR_Pin */
  GPIO_InitStruct.Pin = DOOR_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DOOR_SENSOR_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint16_t adcMultiChannelGetValue(uint8_t channel) {
    if (channel >= ADC_CHANNEL_SIZE) return 0;
    return adc_buffer[channel];
}

/**
 * @func   subscriberTimerCb
 * @brief
 * @param  None
 * @retval None
 */
static void subscriberTimerCb(void *arg) {
    SIMCOM_httpGet();
}

/**
 * @func   publisherTimerCb
 * @brief
 * @param  None
 * @retval None
 */
static void publisherTimerCb(void *arg) {
	uint8_t MSTT = 0;

	// Read data from all sensors
	dht_status status_id0 = DHT_ReadTempHum(&dht11_id0);
	DHT_ReadTempHum(&dht11_id1);
	float temp1 = rs232_GetTempDht22();
	float temp2 = LM35_GetTempCel(ADC_CHANNEL_LM35_ID0);
	float temp3 = LM35_GetTempCel(ADC_CHANNEL_LM35_ID1);
	door_status_e door = door_get_status();
	water_level_state waterSensor = RainWater_GetLevelState();
	float ppm = MQ135_GetPPM();
	uint8_t statusAirCondition = IRCtrl_GetState();
	uint8_t fanLevel = FanCtrl_GetLevel();

	if ((dht11_id0.Temp < tempSet_Room) && (dht11_id1.Temp < tempSet_Room) && \
		(temp2 < tempSet_Cabinet) && (temp3 < tempSet_AQ) && (waterSensor == WATER_LEVEL_LOW) && (ppm < 500) && (door == DOOR_STATUS_CLOSE))
	{
		MSTT = 0;
	}
	else if ((temp3 > tempSet_AQ) && (temp3 < 50) && \
		 (temp2 > tempSet_Cabinet) && (temp2 < 95) && \
		 (average(dht11_id0.Temp, dht11_id1.Temp)  > 30) && (waterSensor == WATER_LEVEL_LOW) && (ppm < 500) && (door == DOOR_STATUS_CLOSE))
	{
		MSTT = 2;
	}
//	else if () { // Them mat tin hieu nang luong AC
//		MSTT = 3;
//	}
//	else if () { // Them mat tin hieu nang luong DC
//		MSTT = 4;
//	}
	else if ((temp1 > 55) || (temp1 == -1))
	{
		MSTT = 5;
	}
	else if (((average(dht11_id0.Temp, dht11_id1.Temp) > 55) && (temp1 < 40)) || (status_id0 != DHT_STA_OK))
	{
		MSTT = 6;
	}
	else if (((temp3 < 30) && (temp2 < 75) && (average(dht11_id0.Temp, dht11_id1.Temp) < 50) && \
			(ppm > 500)) || (ppm == -1))
	{
		MSTT = 7;
	}
	else if ((statusAirCondition == 1) && (abs(average(dht11_id0.Temp, dht11_id1.Temp) - tempAirCondition) > 20))
	{
		MSTT = 8;
	}
	else if ((((temp2 > 75) && (average(dht11_id0.Temp, dht11_id1.Temp) < 30)) || \
			 ((temp2 > 95) && (average(dht11_id0.Temp, dht11_id1.Temp) > 30))) || (temp2 == -1))
	{
		MSTT = 9;
	}
	else if ((((temp3 > 30) && (average(dht11_id0.Temp, dht11_id1.Temp) < 30)) || \
			 ((temp3 > 50) && (average(dht11_id0.Temp, dht11_id1.Temp) > 30))) || (temp3 == -1))
	{
		MSTT = 10;
	}
	else if (waterSensor == WATER_LEVEL_UNKNOW)
	{
		MSTT = 11;
	}
	else if (waterSensor == WATER_LEVEL_HIGH) {
		MSTT = 12;
	}
	else if (door == DOOR_STATUS_OPEN)
	{
		MSTT = 13;
	}
	else if (((average(dht11_id0.Temp, dht11_id1.Temp) > 50) || (temp3 > 30) || (temp2 > 75)) && (ppm > 500))
	{
		MSTT = 14;
	}
//	else if () { // 2 trong 5 cam bien dht11_id0 va dht11_id1 va nhiet do temp3 va temp2 va ppm mat
//		MSTT = 15;
//	}

	// Construct the JSON string
	memset(json_buffer, 0, sizeof(json_buffer));
	snprintf(json_buffer, sizeof(json_buffer),
		"\"mac\":\"00:0A:F5:88:88:8F\","
		"\"MSDT\":\"1\","
		"\"MSTT\":\"%d\","
		"\"power\":\"%d\","
		"\"timesUpdate\":\"0\","
		"\"khoi\":\"%0.2f\","
		"\"door\":\"%s\","
		"\"waterSensor\":\"%s\","
		"\"temp1\":\"%.2f\","
		"\"temp2\":\"%.2f\","
		"\"temp3\":\"%.2f\","
		"\"humidity\":\"%.2f\","
		"\"status\":\"0\","
		"\"statusAirCondition\":\"%s\","
		"\"statusFan\":\"%s\","
		"\"tempAirCondition\":\"%.2f\","
		"\"tempTN1\":\"%.2f\","
		"\"tempTN2\":\"%.2f\","
		"\"tempSet\":\"%d\"",
		MSTT,
		0,
		ppm,
		(door == DOOR_STATUS_CLOSE) ? "close" : "open",
		(waterSensor == WATER_LEVEL_LOW) ? "low" : (waterSensor == WATER_LEVEL_MEDIUM ? "medium" : "high"),
		temp1,
		temp2,
		temp3,
		(dht11_id0.Humi + dht11_id0.Humi) / 2,
		(statusAirCondition == 0) ? "off" : "on",
		(fanLevel == 0) ? "off" : "on",
		32.0,
		dht11_id0.Temp,
		dht11_id1.Temp,
		tempSet_Room
	);

	// Send the JSON string via UART, save it to memory or further processing
	printf("%s\n", json_buffer);
	SIMCOM_httpPost(json_buffer);
}

// Function to parse JSON for AC control
void parseACControlJSON(const char *jsonString, int *id, int *actionAC) {
    sscanf(jsonString, "{\"ID\":\"%d\",\"actionAC\":\"%d\"", id, actionAC);
}

// Function to parse JSON for Fan control
void parseFanControlJSON(const char *jsonString, int *id, int *actionFan) {
	sscanf(jsonString, "{\"ID\":\"%d\",\"actionFan\":\"%d\"", id, actionFan);
}

// Function to parse JSON for AC temperature control
void parseACTempControlJSON(const char *jsonString, int *id, int *tempAC) {
	sscanf(jsonString, "{\"ID\":\"%d\",\"tempAC\":\"%d\"", id, tempAC);
}

// Function to parse JSON for setting default AC temperature
void parseACDefaultTempJSON(const char *jsonString, int *id, int *setTemp) {
	sscanf(jsonString, "{\"ID\":\"%d\",\"setTemp\":\"%d\"", id, setTemp);
}

// Callback function to handle received data
void subscriberHandlerCb(uint8_t *data, int32_t length) {
	if (data == NULL) return;

    // Create a copy of the data string to ensure null-termination
    char jsonString[length + 1];
    memcpy(jsonString, data, length);
    jsonString[length] = '\0'; // Ensure the string is null-terminated

    int id, actionAC, actionFan, tempAC, setTemp;

    // Determine the type of message based on JSON content
    if (strstr(jsonString, "actionAC") != NULL) {
        parseACControlJSON(jsonString, &id, &actionAC);
        printf("AC Control: ID: %d, Action: %d\n", id, actionAC);
    }
    else if (strstr(jsonString, "actionFan") != NULL) {
        parseFanControlJSON(jsonString, &id, &actionFan);
        printf("Fan Control: ID: %d, Action: %d\n", id, actionFan);
    }
    else if (strstr(jsonString, "tempAC") != NULL) {
        parseACTempControlJSON(jsonString, &id, &tempAC);
        printf("AC Temp Control: ID: %d, Temp: %d\n", id, tempAC);
    }
    else if (strstr(jsonString, "setTemp") != NULL) {
        parseACDefaultTempJSON(jsonString, &id, &setTemp);
        printf("AC Default Temp: ID: %d, Temp: %d\n", id, setTemp);
    }
    else {
        printf("Unknown JSON format: %s\n", jsonString);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
