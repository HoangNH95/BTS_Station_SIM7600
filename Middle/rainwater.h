/*******************************************************************************
 *
 * Copyright (c) 2018
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description: Include file for application
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 1.0 $
 * Last Changed:     $Date: 6/20/18 $
 *
 ******************************************************************************/
#ifndef _RAIN_WATER_H_
#define _RAIN_WATER_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include "main.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define WATER_SAMPLE_COUNTER_MAX	   20

#define WATER_LEVEL_LOWER_THRESHOLD    1200
#define WATER_LEVEL_UPPER_THRESHOLD    1700

typedef enum {
	WATER_LEVEL_LOW,
	WATER_LEVEL_MEDIUM,
	WATER_LEVEL_HIGH,
	WATER_LEVEL_UNKNOW,
} water_level_state;
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void RainWater_Init(adc_get_value_callback callback);
uint16_t RainWater_GetCounterRaw(void);
water_level_state RainWater_GetLevelState(void);

#endif

/* END FILE */
