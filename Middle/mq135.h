/**************************************************************************/
/*!
@file     MQ135.h
@author   G.Krocker (Mad Frog Labs)
@license  GNU GPLv3

First version of an Arduino Library for the MQ135 gas sensor
TODO: Review the correction factor calculation. This currently relies on
the datasheet but the information there seems to be wrong.

@section  HISTORY

v1.0 - First release
*/
/**************************************************************************/
#ifndef MQ135_H
#define MQ135_H

#include "main.h"

/// For details about the parameters below, see:
/// http://davidegironi.blogspot.com/2014/01/cheap-co2-meter-using-mq135-sensor-with.html
/// https://hackaday.io/project/3475-sniffing-trinket/log/12363-mq135-arduino-library

/// Parameters for calculating ppm of CO2 from sensor resistance
#define PARA 116.6020682
#define PARB 2.769034857

/// Parameters to model temperature and humidity dependence
#define CORA .00035
#define CORB .02718
#define CORC 1.39538
#define CORD .0018
#define CORE -.003333333
#define CORF -.001923077
#define CORG 1.130128205

/// Atmospheric CO2 level for calibration purposes,
/// from "Globally averaged marine surface monthly mean data"
/// available at https://gml.noaa.gov/ccgg/trends/gl_data.html
#define ATMOCO2 415.58 // Global CO2 Aug 2022

void MQ135_Init(adc_get_value_callback callback);
float MQ135_GetCorrectionFactor(float t, float h);
float MQ135_GetResistance();
float MQ135_GetCorrectedResistance(float t, float h);
float MQ135_GetPPM();
float MQ135_GetCorrectedPPM(float t, float h);
float MQ135_GetRZero();
float MQ135_GetCorrectedRZero(float t, float h);

#endif
