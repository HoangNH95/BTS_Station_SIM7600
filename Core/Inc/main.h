/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
enum {
	ADC_CHANNEL_LM35_ID0,
	ADC_CHANNEL_LM35_ID1,
	ADC_CHANNEL_MQ135_SENSOR,
	ADC_CHANNEL_WATER_SENSOR,
	ADC_CHANNEL_SIZE
};

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef uint16_t (* adc_get_value_callback)(uint8_t channel);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define MQ135_SENSOR_Pin GPIO_PIN_0
#define MQ135_SENSOR_GPIO_Port GPIOC
#define WATER_LEVEL_Pin GPIO_PIN_1
#define WATER_LEVEL_GPIO_Port GPIOC
#define LM35_ID0_Pin GPIO_PIN_0
#define LM35_ID0_GPIO_Port GPIOA
#define LM35_ID1_Pin GPIO_PIN_1
#define LM35_ID1_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_2
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_3
#define DEBUG_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LED_NETWORK_Pin GPIO_PIN_6
#define LED_NETWORK_GPIO_Port GPIOA
#define LED_USER_Pin GPIO_PIN_7
#define LED_USER_GPIO_Port GPIOA
#define BTN_FAN_ON_Pin GPIO_PIN_0
#define BTN_FAN_ON_GPIO_Port GPIOB
#define BTN_FAN_OFF_Pin GPIO_PIN_1
#define BTN_FAN_OFF_GPIO_Port GPIOB
#define FAN_OUT_Pin GPIO_PIN_11
#define FAN_OUT_GPIO_Port GPIOC
#define BTN_IR_UP_Pin GPIO_PIN_12
#define BTN_IR_UP_GPIO_Port GPIOB
#define BTN_IR_DOWN_Pin GPIO_PIN_13
#define BTN_IR_DOWN_GPIO_Port GPIOB
#define BTN_IR_ON_Pin GPIO_PIN_14
#define BTN_IR_ON_GPIO_Port GPIOB
#define BTN_IR_OFF_Pin GPIO_PIN_15
#define BTN_IR_OFF_GPIO_Port GPIOB
#define DOOR_SENSOR_Pin GPIO_PIN_6
#define DOOR_SENSOR_GPIO_Port GPIOC
#define DHT11_ID0_Pin GPIO_PIN_7
#define DHT11_ID0_GPIO_Port GPIOC
#define DHT11_ID1_Pin GPIO_PIN_8
#define DHT11_ID1_GPIO_Port GPIOC
#define DHT22_Pin GPIO_PIN_9
#define DHT22_GPIO_Port GPIOC
#define IR_OUT_Pin GPIO_PIN_8
#define IR_OUT_GPIO_Port GPIOA
#define SERIAL_SIMCOM_TX_Pin GPIO_PIN_9
#define SERIAL_SIMCOM_TX_GPIO_Port GPIOA
#define SERIAL_SIMCOM_RX_Pin GPIO_PIN_10
#define SERIAL_SIMCOM_RX_GPIO_Port GPIOA
#define SERIAL_ENERGY_TX_Pin GPIO_PIN_11
#define SERIAL_ENERGY_TX_GPIO_Port GPIOA
#define SERIAL_ENERGY_RX_Pin GPIO_PIN_12
#define SERIAL_ENERGY_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
