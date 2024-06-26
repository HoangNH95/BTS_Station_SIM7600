/*******************************************************************************
 *
 * Copyright (c) 2020
 * Lumi, JSC.
 * All Rights Reserved
 *
 * Description:
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 1.1 $
 * Last Changed:     $Date: 10/07/20 $
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "fanctrl.h"
#include "door_sensor.h"
#include "led.h"
#include "button.h"
#include "eventButtonCb.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void ButtonProcessIndicator(uint8_t button_id);
static void ButtonProcessEventEdge(uint8_t btnID, uint16_t edge);
static void ButtonProcessEventPress(uint8_t btnID, uint16_t time);
static void ButtonProcessEventHold(uint8_t btnID, uint16_t time);
static void ButtonProcessEventRelease(uint8_t btnID, uint16_t time);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func   EventButton_Init
 * @brief  Initialize events of button
 * @param  pFuncAppProcEvent: funtion callback
 * @retval None
 */
void
EventButton_Init(void) {
    /* Init callback functions */
	ButtonIndicateRegisterCallback(ButtonProcessIndicator);
	Button_RegisterEventCallback(BUTTON_EVENT_EDGE, ButtonProcessEventEdge);
	Button_RegisterEventCallback(BUTTON_EVENT_PRESS, ButtonProcessEventPress);
	Button_RegisterEventCallback(BUTTON_EVENT_HOLD, ButtonProcessEventHold);
	Button_RegisterEventCallback(BUTTON_EVENT_RELEASE, ButtonProcessEventRelease);

    Button_Init();
}

/**
 * @func   button_handle_indicator
 * @brief  Led indicate button press
 * @param  button_id
 * @retval None
 */
static void ButtonProcessIndicator(uint8_t button_id)
{
	// Display status led when press button configure
	led_event_show(LEDEV_BT_PRESS);
}

/**
 * @func   ButtonProcessEventEdge
 * @brief  Event edge pulse
 * @param  btnID
 * @param  edge
 * @retval None
 */
static void
ButtonProcessEventEdge(
    uint8_t btnID,
    uint16_t edge
) {
	if (btnID == DOOR_SENSOR_ID) {
		if (edge == BUTTON_EDGE_RISING)
		{
			door_set_status(DOOR_STATUS_CLOSE);
		}
		else /* (edge == BUTTON_EDGE_FAILING) */
		{
			door_set_status(DOOR_STATUS_OPEN);
		}
	}
}

/**
 * @func   InputProcessEventPress
 * @brief  Event logic level
 * @param  btnID
 * @param  time
 * @retval None
 */
static void 
ButtonProcessEventPress(
    uint8_t btnID,
    uint16_t time
) {
	if (time == 1)
	{
		switch (btnID) {
			case BUTTON_FAN_ON_ID:
				FanCtrl_SetLevel(100);
				break;

			case BUTTON_FAN_OFF_ID:
				FanCtrl_SetLevel(0);
				break;

			case BUTTON_IR_UP_ID:
				break;

			case BUTTON_IR_DOWN_ID:
				break;

			case BUTTON_IR_ON_ID:
				break;

			case BUTTON_IR_OFF_ID:
				break;

			default:
				break;
			}
	}
	else if (time == 2)
	{
	}
	else if (time == 5)
	{
	}
	else return;
}

/**
 * @func   InputProcessEventHold
 * @brief  Event hold
 * @param  butt
 * @param  time
 * @retval None
 */
static void 
ButtonProcessEventHold(
    uint8_t btnID,
    uint16_t time
) {
	if (time == TIMECNT_HOLD10S)
	{
	}
	else if (time == TIMECNT_HOLD5S)
	{
	}
	else if (time == TIMECNT_HOLD3S)
	{
	}
	else if (time == TIMECNT_HOLD1S)
	{
	}
	else return;
}

/**
 * @func   InputProcessEventRelease
 * @brief  Event release
 * @param  butt
 * @param  time
 * @retval None
 */
static void 
ButtonProcessEventRelease(
    uint8_t btnID,
    uint16_t time
) {
	if (time >= TIMECNT_HOLD10S)
	{
	}
	else if (time >= TIMECNT_HOLD5S)
	{
	}
	else if (time >= TIMECNT_HOLD3S)
	{
	}
	else if (time >= TIMECNT_HOLD1S)
	{
	}
	else return;
}

/* END FILE */
