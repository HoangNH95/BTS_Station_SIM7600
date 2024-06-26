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
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdbool.h>
#include "led.h"
#include "timer.h"
#include "utilities.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define TIME_DELAY_EVENT_LED_BLINKS    1 //ms

typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
} led_obj_t;

typedef struct {
	uint8_t id;
	uint8_t type;
	uint8_t times;
	uint16_t interval;
    uint8_t laststate;
} ledifo_t, *ledifo_p;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static const led_obj_t g_ledArray[NUM_OF_LED][NUM_OF_COLOR] = {
    {{ LED_NETWORK_GPIO_Port, LED_NETWORK_Pin }}, \
    {{ LED_USER_GPIO_Port,    LED_USER_Pin    }}
};

static ledifo_t ledUserBlink, ledNetworkBlink;
static bool g_bToggleColor = false;
static uint8_t ledUserBlinkTimerEvt = NO_TIMER;
static uint8_t ledNetworkBlinkTimerEvt = NO_TIMER;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void led_blink_stop(void);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func   led_init
 * @brief  Initializes module led
 * @param  None
 * @retval None
 */
void led_init(void) {
}

/**
 * @func   led_set_on
 * @brief  Set led on
 * @param  byLedNumber
 * @param  byLedType
 * @retval None
 */
static void
led_set_on(
    uint8_t byLedNumber,
    uint8_t byLedType
) {
    if ((byLedNumber >= NUM_OF_LED) || !isTypeLED(byLedType) )
        return;

    HAL_GPIO_WritePin(g_ledArray[byLedNumber][byLedType].port, \
    		g_ledArray[byLedNumber][byLedType].pin, GPIO_PIN_SET);
}

/**
 * @func   led_set_off
 * @brief  Set led off
 * @param  byLedNumber
 * @param  byLedType
 * @retval None
 */
static void
led_set_off(
    uint8_t byLedNumber,
    uint8_t byLedType
) {
    if ((byLedNumber >= NUM_OF_LED) || !isTypeLED(byLedType))
        return;

    HAL_GPIO_WritePin(g_ledArray[byLedNumber][byLedType].port, \
			g_ledArray[byLedNumber][byLedType].pin, GPIO_PIN_RESET);
}

/**
 * @func   led_set_toggle
 * @brief  Led toggle
 * @param  byLedNumber
 * @param  byLedType
 * @retval None
 */
static void
led_set_toggle(
	uint8_t byLedNumber,
	uint8_t byLedColor
) {
    if ((byLedNumber >= NUM_OF_LED) || (byLedColor >= CLED_MAX))
        return;

    if (g_bToggleColor == false) {
		led_set_color(byLedNumber, CLED_OFF);
	}
	else {
		led_set_color(byLedNumber, byLedColor);
	}

    g_bToggleColor = !g_bToggleColor;
}

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
) {
    switch (byLedColor) {
        case CLED_OFF:
            led_set_off(byLedNumber, LED_TYPE_BLUE);
            break;

        case CLED_BLUE:
            led_set_on(byLedNumber, LED_TYPE_BLUE);

        default:
            break;
    }
}

/**
 * @func   ledUserBlinkTimerCb
 * @brief  Callback handle blink led user
 * @param  None
 * @retval None
 */
static void ledUserBlinkTimerCb(void *arg) {
	if (ledUserBlink.times == 0) {
		led_blink_stop();

		TimerStop(ledUserBlinkTimerEvt);
		ledUserBlinkTimerEvt = NO_TIMER;
		return;
	}
	if ((ledUserBlink.times != BLINK_FOREVER) && (ledUserBlink.times != 0)) {
		ledUserBlink.times--;
	}

	switch (ledUserBlink.type) {
		case BLINK_BLUE:
			led_set_toggle(ledUserBlink.id, CLED_BLUE);
			break;

		default:
			break;
	}

	TimerChangePeriod(ledUserBlinkTimerEvt, ledUserBlink.interval);
}

/**
 * @func   led_blink_stop
 * @brief  Blink stop
 * @param  type:
 * @retval None
 */
static void led_blink_stop(void) {
    // Return state last
	led_set_color(ledUserBlink.id, ledUserBlink.laststate);

    // Reset variable to default
    ledUserBlink.times = 0;
}

/**
 * @func   led_user_blink
 * @brief  Delay blink led
 * @param  ledMask:
 * 		   type:
 * 		   timesBlink:
 * 		   timeDelay:
 * 		   lastState:
 * 		   delay:
 * @retval None
 */
static void led_user_blink(
	uint8_t ledMask,
	led_blink_type_t type,
	uint8_t timesBlink,
	uint16_t timeDelay,
	led_color_t lastState,
	uint16_t delay
) {
	g_bToggleColor = false;
	ledUserBlink.id = ledMask;
	ledUserBlink.type = (uint8_t)type;
	ledUserBlink.times = timesBlink;
	ledUserBlink.interval = timeDelay;
	ledUserBlink.laststate = (uint8_t)lastState;

	// Start timer blink
	if (ledUserBlinkTimerEvt) {
		TimerStop(ledUserBlinkTimerEvt);
		ledUserBlinkTimerEvt = NO_TIMER;
	}
	ledUserBlinkTimerEvt = TimerStart("ledUserBlinkTimerCb", \
			                          delay, \
									  TIMER_REPEAT_FOREVER, \
									  ledUserBlinkTimerCb, \
									  NULL);
}

/**
 * @func   ledNetworkBlinkTimerCb
 * @brief  Callback handle blink led network
 * @param  None
 * @retval None
 */
static void ledNetworkBlinkTimerCb(void *arg) {
	if (ledNetworkBlink.times == 0) {
		led_set_color(LED_ID_NETWORK, CLED_BLUE);
		TimerStop(ledNetworkBlinkTimerEvt);
		ledNetworkBlinkTimerEvt = NO_TIMER;
		return;
	}
	if ((ledNetworkBlink.times != BLINK_FOREVER) && (ledNetworkBlink.times != 0)) {
		ledNetworkBlink.times--;
	}

	led_set_toggle(LED_ID_NETWORK, CLED_BLUE);
	TimerChangePeriod(ledNetworkBlinkTimerEvt, 300);
}

static void led_network_blink(
	uint8_t numBlink,
	uint16_t interval,
	led_color_t lastState
) {
	ledNetworkBlink.times = numBlink;
	ledNetworkBlink.interval = interval;
	ledNetworkBlink.laststate = (uint8_t)lastState;

	// Start timer blink
	if (ledNetworkBlinkTimerEvt) {
		TimerStop(ledNetworkBlinkTimerEvt);
		ledNetworkBlinkTimerEvt = NO_TIMER;
	}
	ledNetworkBlinkTimerEvt = TimerStart("ledNetworkBlinkTimerCb", \
			                             TIME_DELAY_EVENT_LED_BLINKS, \
									     TIMER_REPEAT_FOREVER, \
										 ledNetworkBlinkTimerCb, \
									     NULL);
}

/**
 * @func   led_event_show
 * @brief  Led show event
 * @param  led_event_t ev
 * @retval None
 */
void led_event_show(
	led_event_t ev
) {
    switch (ev) {
		case LEDEV_POWERUP:
			break;

		case LEDEV_NETWORK_NONE:
			led_set_color(LED_ID_NETWORK, CLED_OFF);
			break;

		case LEDEV_NETWORK_JOINED:
			led_network_blink(6, 300, CLED_BLUE);
			break;

		case LEDEV_NETWORK_LEAVE:
			led_network_blink(10, 500, CLED_OFF);
			break;

		case LEDEV_BT_PRESS:
			led_user_blink(LED_ID_USER, BLINK_BLUE, 2, 100, CLED_OFF, TIME_DELAY_EVENT_LED_BLINKS);
			break;

		case LEDEV_BT_HOLDED_1S:
			break;

		case LEDEV_BT_HOLDED_3S:
			break;

		case LEDEV_BT_HOLDED_5S:
			break;

		case LEDEV_BTS_CONTROL_SUCCESS:
			led_user_blink(LED_ID_USER, BLINK_BLUE, 2, 300, CLED_OFF, TIME_DELAY_EVENT_LED_BLINKS);
			break;

		case LEDEV_BTS_CONTROL_ID_FAIL:
			led_user_blink(LED_ID_USER, BLINK_BLUE,  4, 300, CLED_OFF, TIME_DELAY_EVENT_LED_BLINKS);
			break;

		case LEDEV_OTA_PROCESS:
			break;

		default:
			break;
    }
}

/* END FILE */
