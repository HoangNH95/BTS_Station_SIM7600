/*******************************************************************************
 *
 * Copyright (c) 2023
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description: Include file for application
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 1.2 $
 * Last Changed:     $Date: 22/03/2023 $
 *
 ******************************************************************************/
#ifndef _LED_H_
#define _LED_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include "main.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define LED_ID_NETWORK				0x00
#define LED_ID_USER  				0x01
#define LED_ALL_MASK				0x02

#define NUM_OF_LED                  2u
#define NUM_OF_COLOR                1u

#define BLINK_FOREVER               0xFFu

enum {
	LED_TYPE_BLUE
};

#define isTypeLED(x)                (x == LED_TYPE_BLUE)

/*! @brief Enum led blink color */
typedef enum __led_blink_type__ {
    BLINK_BLUE,
    BLINK_MAX
} led_blink_type_t;

/*! @brief Enum led color */
typedef enum __led_color__ {
	CLED_OFF,
    CLED_BLUE,
    CLED_MAX,
} led_color_t;

/*! @brief Enum led event */
typedef enum __led_event_indicator__ {
    LEDEV_IDLE,
	LEDEV_POWERUP,
    LEDEV_BT_PRESS,
	LEDEV_BT_PRESS_5TIMES,
	LEDEV_BT_HOLDED_1S,
	LEDEV_BT_HOLDED_3S,
    LEDEV_BT_HOLDED_5S,
	LEDEV_BT_HOLDED_15S,
    LEDEV_BT_RELEASE,
	LEDEV_NETWORK_NONE,
	LEDEV_NETWORK_JOINED,
	LEDEV_NETWORK_LEAVE,
	LEDEV_BTS_CONTROL_SUCCESS,
    LEDEV_BTS_CONTROL_ID_FAIL,
    LEDEV_OTA_PROCESS,
} led_event_t;
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func   led_init
 * @brief  Initializes module led
 * @param  None
 * @retval None
 */
void
led_init(void);

/**
 * @func   led_set_color
 * @brief  Set color for each led
 * @param  [byLedNumber] id number
 * @param  [byLedColor] type color
 * @retval None
 */
void
led_set_color(
    uint8_t byLedNumber,
	uint8_t byLedColor
);

/**
 * @func   led_event_show
 * @brief  Led show event
 * @param  event led
 * @retval None
 */
void
led_event_show(
	led_event_t ev
);

#endif

/* END FILE */
