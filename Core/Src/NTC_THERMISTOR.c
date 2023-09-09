#include "NTC_THERMISTOR.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_def.h"
#include <math.h>
#include <stdint.h>

double R2, log_R2, T, raw;

void NTC_THERMISTOR_Initialize(NTC_Thermistor *thermistor, ADC_HandleTypeDef* adcHandle){

  /* set sensor ADC handle */
  thermistor->adcHandle = adcHandle;

  /* initialize temperature to 0 */
  thermistor->tempC = 0.0f;

}

void NTC_THERMISTOR_ReadTemperature(NTC_Thermistor *thermistor){

  HAL_ADC_Start(thermistor->adcHandle);
  HAL_ADC_PollForConversion(thermistor->adcHandle, 100); /* wait for 10 ms before next intruction */

  raw = HAL_ADC_GetValue(thermistor->adcHandle);

  /* convert rawTemp to temperature in deg C */
  R2 = R1 * ( (4095 / raw  ) - 1.0);

  log_R2 = log(R2);

  T = 1 / (A + B * log_R2 + C * log_R2 * log_R2 * log_R2);

  T = T - 273.15;

  thermistor->tempC = T;

}
