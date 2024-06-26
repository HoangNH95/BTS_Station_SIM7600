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
#include "lm35.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static adc_get_value_callback pLM35GetADCValueCallbacks;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

void LM35_Init(adc_get_value_callback callback) {
	if (callback != NULL) {
		pLM35GetADCValueCallbacks = callback;
	}
}

float LM35_GetTempCel(uint8_t channel)
{
   float tempC = ((float)pLM35GetADCValueCallbacks(channel) * LM35_VREF * 100.0) / ADC_RESOLUTION;

   if ((tempC < LM35_TEMP_THRESHOLD_LOW) || (tempC > LM35_TEMP_THRESHOLD_HIGH)) {
	   return -1;
   }

   return tempC;
}

float LM35_GetTempFah(uint8_t channel)
{
   float tempF = ((((float)pLM35GetADCValueCallbacks(channel) * LM35_VREF * 100.0) / ADC_RESOLUTION) * 1.8) + 32;
   return tempF;
}

float LM35_GetTempKel(uint8_t channel)
{
   float tempK = ((float)pLM35GetADCValueCallbacks(channel) * LM35_VREF * 100.0 / ADC_RESOLUTION) + 273;
   return tempK;
}

/* END FILE */
