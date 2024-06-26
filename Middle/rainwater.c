/*******************************************************************************
 *
 * Copyright (c) 2018
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
 * Last Changed:     $Date: 9/15/18 16:30 $
 *
 ******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "rainwater.h"
#include <stdio.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static adc_get_value_callback pRainWaterGetADCValueCallbacks;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

void RainWater_Init(adc_get_value_callback callback) {
	if (callback != NULL) {
		pRainWaterGetADCValueCallbacks = callback;
	}
}

uint16_t RainWater_GetCounterRaw(void) {
	uint16_t level = 0;

    for (uint8_t sample = 0; sample < WATER_SAMPLE_COUNTER_MAX; sample++) {
    	level += pRainWaterGetADCValueCallbacks(ADC_CHANNEL_WATER_SENSOR); // Read the analog value form sensor
    	HAL_Delay(1);
    }
    level /= WATER_SAMPLE_COUNTER_MAX;

    return level;
}

water_level_state RainWater_GetLevelState(void) {
	water_level_state state;
	uint16_t rawCounter = RainWater_GetCounterRaw();

	if (rawCounter <= WATER_LEVEL_LOWER_THRESHOLD) {
		state = WATER_LEVEL_LOW;
//		printf("Water Level: Low\n");

	}
	else if ((rawCounter > WATER_LEVEL_LOWER_THRESHOLD) && \
		(rawCounter <= WATER_LEVEL_UPPER_THRESHOLD))
	{
		state = WATER_LEVEL_MEDIUM;
//		printf("Water Level: Medium\n");
	}
	else if (rawCounter > WATER_LEVEL_UPPER_THRESHOLD) {
		state = WATER_LEVEL_HIGH;
//		printf("Water Level: High\n");

	}

	return state;
}

/* END FILE */
