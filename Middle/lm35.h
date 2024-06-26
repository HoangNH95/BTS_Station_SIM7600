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
#ifndef _LM35_H_
#define _LM35_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include "main.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define LM35_VREF                    5.0  // Voltage
#define ADC_RESOLUTION               4095 // ADC 12 bit
#define LM35_TEMP_THRESHOLD_LOW      5    // oC
#define LM35_TEMP_THRESHOLD_HIGH     100  // oC

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void  LM35_Init(adc_get_value_callback callback);
float LM35_GetTempCel(uint8_t channel);
float LM35_GetTempFah(uint8_t channel);
float LM35_GetTempKel(uint8_t channel);

#endif

/* END FILE */
