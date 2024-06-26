/******************************************************************************
 *
 * Copyright (c) 2023
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description: Handler message of serial device
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: hoangnh $
 * Revision:         $Revision: 1.3 $
 * Last Changed:     $Date: 23/04/2023 $
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdio.h>
#include <string.h>

#include "eventSerialCb.h"
#include "rs232.h"
#include "timer.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define RS232_CONNECTION_TIMEOUT		5000 // ms
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static float dht22_Temp = -1;
static uint8_t rs232TimeoutConnectionTimerEvt = NO_TIMER;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void eventSerial_rs232ReceiverCallback(uint8_t lenPayload, uint8_t *payload);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

float rs232_GetTempDht22(void) {
	return dht22_Temp;
}

static void rs232TimeoutConnectionTimerCb(void *arg) {
	dht22_Temp = -1;
}

static void eventSerial_rs232ReceiverCallback(
	uint8_t lenPayload,
	uint8_t *payload
) {
	dht22_Temp = payload[0] + (payload[1] / 100.0);
    printf("RS232 length: %d, temp = %0.2f\n", lenPayload, dht22_Temp);

    if (rs232TimeoutConnectionTimerEvt != NO_TIMER) {
		TimerStop(rs232TimeoutConnectionTimerEvt);
		rs232TimeoutConnectionTimerEvt = NO_TIMER;
	}
    rs232TimeoutConnectionTimerEvt = TimerStart("rs232TimeoutConnection", \
    		                                    RS232_CONNECTION_TIMEOUT, \
												TIMER_REPEAT_FOREVER, \
												rs232TimeoutConnectionTimerCb, \
												NULL);
}

/**
 * @func   eventSerial_init
 * @brief  None
 * @param  None
 * @retval None
 */
void eventSerial_init(void) {
	RS232_Init(eventSerial_rs232ReceiverCallback);
}

void processSerialReceiver(void) {
	RS232_ReceiverProcedure();
}

/* END_FILE */
