/*
 * NTC_THERMISTOR.h
 *
 *  Created on: Sep 6, 2023
 *      Author: USER
 */

#ifndef INC_NTC_THERMISTOR_H_
#define INC_NTC_THERMISTOR_H_


#include "stm32f1xx_hal.h" /* needed for ADC */
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_def.h"

/**
* Defines
 */

/* Steinhart-Hart coefficients for a 10k thermistor*/
#define A (1.125308855e-03)
#define B (0.234711863e-03)
#define C (0.000085663e-03)
#define R1 10000.0

extern double R2, log_R2, T, raw;
/**
 * Sensor struct
 */
typedef struct{
  ADC_HandleTypeDef* adcHandle;

  float tempC; // temperature in degrees celcius

} NTC_Thermistor;

/**
 * Initialization
 */
void NTC_THERMISTOR_Initialize(NTC_Thermistor *, ADC_HandleTypeDef * );

/**
 * Data acquisation
 */
void NTC_THERMISTOR_ReadTemperature(NTC_Thermistor *);



#endif /* INC_NTC_THERMISTOR_H_ */
