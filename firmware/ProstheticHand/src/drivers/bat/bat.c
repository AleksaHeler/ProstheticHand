/**
 * @file btn.cpp
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Battery software component / driver
 *  This file deals only with sensing of the battery/input voltage for now.
 *
 * @version 0.1
 * @date 2023-10-15
 *
 * @copyright Copyright (c) 2023
 *
 */

/**************************************************************************
 * Includes
 **************************************************************************/

#include "bat_e.h"
#include "bat_i.h"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * Buffer for storing battery voltage
 *
 * @values Voltage, hopefully no higher than 16.8V (for 4 cell Li-ion battery)
 */
float32_t bat_g_BatVoltage_f32 = 0;

/**
 * @brief Analog to digital converter channel
 * 
 */
adc_channel_t bat_g_BatAdcCh_s;

/**
 * @brief Buffer for storing previous values of the batt voltage for filtering
 * 
 * @values same as bat values
 */
float32_t bat_g_PrevBatVoltages_f32[BAT_AVG_CNT];

/**************************************************************************
 * Functions
 **************************************************************************/

void bat_f_Init_v(void);
void bat_f_Handle_v(void);
float32_t bat_f_MapAdcToMillivolts_f32(uint16_t val);

#ifdef SERIAL_DEBUG
void bat_f_SerialDebug_v(void);
#endif

/**
 * @brief Initialize function to be called once on startup/boot
 *
 *  Set all the given pins as inputs
 *
 *  @return void
 */
void bat_f_Init_v(void)
{
  uint8_t i;

  /* Default configuration for all ADC channels */
  adc_oneshot_chan_cfg_t channel_config = {
      .atten = ADC_ATTEN_DB_11,
      .bitwidth = ADC_BITWIDTH_12};

  /* Now actually configure the ADC channel */
  ESP_ERROR_CHECK(adc_oneshot_io_to_channel(bat_s_BatSensConfig_s.pin_u16, &bat_s_BatSensConfig_s.adc_unit_s, &bat_g_BatAdcCh_s));
  if(bat_s_BatSensConfig_s.adc_unit_s == ADC_UNIT_1)
  {
    ESP_ERROR_CHECK(adc_oneshot_config_channel(main_g_AdcUnit1Handle_s, bat_g_BatAdcCh_s, &channel_config));
  }
  else // ADC_UNIT_2
  {
    ESP_ERROR_CHECK(adc_oneshot_config_channel(main_g_AdcUnit2Handle_s, bat_g_BatAdcCh_s, &channel_config));
  }

  /* Set default values of the low-pass filter */
  for (i = 0; i < BAT_AVG_CNT; i++)
  {
    bat_g_PrevBatVoltages_f32[i] = 0;
  }
}

/**
 * @brief Handle function to be called cyclically
 *
 * Read the defined adc channel and store scaled value in a buffer
 *
 * @return void
 */
void bat_f_Handle_v(void)
{
  uint8_t i;
  uint16_t adcAnalogRead = 0;
  float32_t sum = 0;

  /* Based on which ADC group this pin belongs to, read the corresponding group */
  if (bat_s_BatSensConfig_s.adc_unit_s == ADC_UNIT_1)
  {
    ESP_ERROR_CHECK(adc_oneshot_read(main_g_AdcUnit1Handle_s, bat_g_BatAdcCh_s, (int *)&adcAnalogRead));
  }
  else // ADC_UNIT_2
  {
    ESP_ERROR_CHECK(adc_oneshot_read(main_g_AdcUnit2Handle_s, bat_g_BatAdcCh_s, (int *)&adcAnalogRead));
  }

  /* Shift all values in buffer to the next place in the array */
  for (i = BAT_AVG_CNT - 1; i > 0; i--)
  { 
    bat_g_PrevBatVoltages_f32[i] = bat_g_PrevBatVoltages_f32[i - 1];
  }

  /* Set current value to first index (and scale the value to get exact voltage) */
  bat_g_PrevBatVoltages_f32[0] = bat_f_MapAdcToMillivolts_f32(adcAnalogRead) * bat_s_BatSensConfig_s.mult_f32 / (float)1000;

  /* Take the average of all those readings */
  sum = 0;
  for (i = 0; i < BAT_AVG_CNT; i++)
  {
    sum += bat_g_PrevBatVoltages_f32[i];
  }

  /* Finally get the average value */
  bat_g_BatVoltage_f32 = sum / (float32_t)BAT_AVG_CNT;
}

/**
 * @brief Scale given ADC int input value to corresponding float voltage input
 *
 * @param val
 * @return scaled float32_t value
 */
float32_t bat_f_MapAdcToMillivolts_f32(uint16_t val)
{
  return (float32_t)val * BAT_ADC_COUNT_TO_MILLIVOLT_MULT;
}

#ifdef SERIAL_DEBUG
void bat_f_SerialDebug_v(void)
{
  ESP_LOGD(BAT_TAG, "Battery/input voltage = %f", bat_g_BatVoltage_f32);
}
#endif