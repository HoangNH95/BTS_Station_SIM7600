/*******************************************************************************
 *
 * Copyright (c) 2020
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 1.1 $
 * Last Changed:     $Date:  10/07/20 11:30 $
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <button.h>
#include "timer.h"
#include "utilities.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define PIN_ACTIVE                     1
#define PIN_UNACTIVE                   0

typedef struct _button_data_ {
    uint32_t time;
    uint32_t holdCnt;
    uint8_t pressCnt;
    uint8_t sold;
	uint8_t index;
	uint8_t mode;
	uint8_t countInactive;
	uint8_t countActive;
	void (*scanBtnFunc)(void *);
} buttondat_t, *buttondat_p;

typedef struct _key_pin_ {
	GPIO_TypeDef *port;
	uint16_t pin;
    uint8_t logicPress;
} buttonPin_t;

typedef enum _key_status_ {
    KEYST_IDLE = 0,
    KEYST_INPROCESS,
    KEYST_FINISH
} keyStatus;
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static buttonPin_t pinBtnPad[BUTTON_MAX]  = { \
    BUTTON_BOARD_PIN, BUTTON_FAN_ON_PIN, BUTTON_FAN_OFF_PIN, \
    BUTTON_IR_UP_PIN, BUTTON_IR_DOWN_PIN, BUTTON_IR_ON_PIN, \
	BUTTON_IR_OFF_PIN, DOOR_SENSOR_PIN
};
static buttondat_t kpad[BUTTON_MAX];
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void ButtonScan(void *arg);
static void ButtonScanTypeEdge(void *arg);
static button_indicate_callback pButtonIndicateCallbacks;
static button_event_callback pEventCallbacks[BUTTON_EVENT_MAXIMUM];
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func   Button_Init
 * @brief  Initialize module button
 * @param  None
 * @retval None
 */
void
Button_Init(void) {
    for (uint8_t i = 0; i < BUTTON_MAX; i++) {
		kpad[i].index = i;
		kpad[i].mode = BUTTON_TYPE_LOGIC;
		kpad[i].scanBtnFunc = ButtonScanTypeEdge;
		if (HAL_GPIO_ReadPin(pinBtnPad[i].port, pinBtnPad[i].pin) == pinBtnPad[i].logicPress) {
			kpad[i].sold = PIN_ACTIVE;
		}
	}

    kpad[DOOR_SENSOR_ID].mode = BUTTON_TYPE_EDGE;
    TimerStart("", KEY_TIME_SCAN, TIMER_REPEAT_FOREVER, ButtonScan, NULL);
}

/**
 * @func   ButtonIndicateRegisterCallback
 * @brief  None
 * @param  None
 * @retval None
 */
void
ButtonIndicateRegisterCallback(
	button_indicate_callback procButIndicate
) {
	pButtonIndicateCallbacks = procButIndicate;
}

/**
 * @func   ButtonSetMode
 * @brief  Set mode key
 * @param  None
 * @retval None
 */
void
Button_SetMode(
    uint8_t id,
    uint8_t mode
) {
    if (id >= BUTTON_MAX) return;
    kpad[id].mode = mode;
}

/**
 * @func   ButtonGetMode
 * @brief  Get mode key
 * @param  None
 * @retval None
 */
uint8_t
Button_GetMode(
    uint8_t id
) {
    if (id >= BUTTON_MAX) return 0;
    return kpad[id].mode;
}

/**
 * @func   Button_RegisterEventCallback
 * @brief  None
 * @param  None
 * @retval None
 */
void
Button_RegisterEventCallback(
    button_event_t buttonEvent,
    button_event_callback procbuttonEvent
) {
    pEventCallbacks[buttonEvent] = procbuttonEvent;
}

/**
 * @func   Button_GetLogicInputPin
 * @brief  Button get logic on input pin
 * @param  id: id of input pin
 * @retval logic pin
 */
uint8_t
Button_GetLogicInputPin(
	uint8_t id
) {
	if (id >= BUTTON_MAX) return 0xFE;

	return HAL_GPIO_ReadPin(pinBtnPad[id].port, pinBtnPad[id].pin);
}

/**
 * @func   ButtonScanTypeEdge
 * @brief  None
 * @param  None
 * @retval None
 */
void
ButtonScanTypeEdge(
    void *arg
) {
	buttondat_p bt = (buttondat_p)arg;
    uint8_t i = bt->index;
    
    if (HAL_GPIO_ReadPin(pinBtnPad[i].port, pinBtnPad[i].pin) == pinBtnPad[i].logicPress) {
        if (bt->countActive != 0xFFU) {
            bt->countActive++;
        }
    }
    else {
        if (bt->countInactive != 0xFFU) {
            bt->countInactive++;
        }
        
        bt->countActive = 0;
    }
    
    if (bt->countInactive >= KEY_COUNT_IS_PRESS) {
        if (bt->sold == PIN_ACTIVE) {
            bt->sold = PIN_UNACTIVE;
            
            if ((pEventCallbacks[BUTTON_EVENT_EDGE] != NULL) && (bt->mode & BUTTON_TYPE_EDGE)) {
                pEventCallbacks[BUTTON_EVENT_EDGE](i, BUTTON_EDGE_FALLING);
            }
            
            bt->time = bt->countInactive;
        } else {
            bt->time++;
            if (bt->time >= TIMECNT_BW2PRESS) {
                if (bt->holdCnt != 0) {
                    if ((pEventCallbacks[BUTTON_EVENT_RELEASE] != NULL) && (bt->mode & BUTTON_TYPE_LOGIC)) {
                        pEventCallbacks[BUTTON_EVENT_RELEASE](i, bt->holdCnt);
                    }
                }
                else if ((pEventCallbacks[BUTTON_EVENT_PRESS] != NULL) && (bt->mode & BUTTON_TYPE_LOGIC) \
                         && bt->pressCnt != 0) 
                {
                    pEventCallbacks[BUTTON_EVENT_PRESS](i, bt->pressCnt);
                }
                
                bt->holdCnt = 0;
                bt->pressCnt = 0;
            }
        }
    }
    
    if (bt->countActive >= KEY_COUNT_IS_PRESS) {
        /* reset count inactive */
        bt->countInactive = 0; 
        
        if (bt->sold == PIN_UNACTIVE) {
            bt->sold = PIN_ACTIVE;
            
            if ((pEventCallbacks[BUTTON_EVENT_EDGE] != NULL) && (bt->mode & BUTTON_TYPE_EDGE)) {
                pEventCallbacks[BUTTON_EVENT_EDGE](i, BUTTON_EDGE_RISING);
            }
            if (pButtonIndicateCallbacks != NULL) {
				pButtonIndicateCallbacks(i);
			}
            
            bt->time = bt->countActive;
            bt->pressCnt++;
        } else {
            bt->time++;
            if (bt->time >= TIMECNT_IS_HOLD) {
                bt->pressCnt = 0;
                bt->holdCnt = bt->time;
                if ((bt->holdCnt == TIMECNT_IS_HOLD) || (bt->holdCnt == TIMECNT_HOLD1S) || (bt->holdCnt == TIMECNT_HOLD3S) ||
                    (bt->holdCnt == TIMECNT_HOLD5S) || (bt->holdCnt == TIMECNT_HOLD10S)) 
                {
                    if (pEventCallbacks[BUTTON_EVENT_HOLD] != NULL && (bt->mode & BUTTON_TYPE_LOGIC)) {
                        pEventCallbacks[BUTTON_EVENT_HOLD](i, bt->holdCnt);
                    }
                }
            }
        }
    }
}

/**
 * @func   ButtonScan
 * @brief  Scan key
 * @param  None
 * @retval None
 */
void
ButtonScan(
    void *arg
) {
    for (uint8_t i = 0; i < BUTTON_MAX; i++) {
        kpad[i].scanBtnFunc(&kpad[i]);
    }
}

/* END FILE */
