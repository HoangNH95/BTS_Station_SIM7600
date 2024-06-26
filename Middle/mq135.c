/**************************************************************************/
/*!
@file     MQ135.cpp
@author   G.Krocker (Mad Frog Labs)
@license  GNU GPLv3

First version of an Arduino Library for the MQ135 gas sensor
TODO: Review the correction factor calculation. This currently relies on
the datasheet but the information there seems to be wrong.

@section  HISTORY

v1.0 - First release
*/
/**************************************************************************/

#include "mq135.h"

#include <math.h>

uint8_t _pin;
float _rload = 76.63; // The load resistance on the board in kOhm
float _rzero = 76.63; // Calibration resistance at atmospheric CO2 level

static adc_get_value_callback pMQ135GetADCValueCallbacks;

void MQ135_Init(adc_get_value_callback callback) {
	if (callback != NULL) {
		pMQ135GetADCValueCallbacks = callback;
	}
}

/**************************************************************************/
/*!
@brief  Get the correction factor to correct for temperature and humidity

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The calculated correction factor
*/
/**************************************************************************/
float MQ135_GetCorrectionFactor(float t, float h) {
    // Linearization of the temperature dependency curve under and above 20 degree C
    // below 20degC: fact = a * t * t - b * t - (h - 33) * d
    // above 20degC: fact = a * t + b * h + c
    // this assumes a linear dependency on humidity
    if(t < 20){
        return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
    } else {
        return CORE * t + CORF * h + CORG;
    }
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value
		Known issue: If the ADC resolution is not 12-bits, this will give
		back garbage values!

@return The sensor resistance in kOhm
*/
/**************************************************************************/
float MQ135_GetResistance() {
  int val = pMQ135GetADCValueCallbacks(ADC_CHANNEL_MQ135_SENSOR);
  return ((4095./(float)val) - 1.)*_rload;
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance kOhm
*/
/**************************************************************************/
float MQ135_GetCorrectedResistance(float t, float h) {
  return MQ135_GetResistance() / MQ135_GetCorrectionFactor(t, h);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135_GetPPM(void) {
  return PARA * pow((MQ135_GetResistance()/_rzero), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135_GetCorrectedPPM(float t, float h) {
  return PARA * pow((MQ135_GetCorrectedResistance(t, h)/_rzero), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the resistance RZero of the sensor for calibration purposes

@return The sensor resistance RZero in kOhm
*/
/**************************************************************************/
float MQ135_GetRZero(void) {
  return MQ135_GetResistance() * pow((ATMOCO2/PARA), (1./PARB));
}

/**************************************************************************/
/*!
@brief  Get the corrected resistance RZero of the sensor for calibration
        purposes

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance RZero in kOhm
*/
/**************************************************************************/
float MQ135_GetCorrectedRZero(float t, float h) {
  return MQ135_GetCorrectedResistance(t, h) * pow((ATMOCO2/PARA), (1./PARB));
}
