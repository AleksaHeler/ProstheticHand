/**
 * @file sns.cpp
 *
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 *
 * @brief Sensor software component / driver
 *
 * This module handles the reading of the EMG sensor connected to the board.
 *
 * @version 0.1
 * @date 2023-09-28
 *
 * @copyright Copyright (c) 2023
 *
 */

/**************************************************************************
 * Includes
 **************************************************************************/

/* Own header file */
#include "sns_e.h"
#include "sns_i.h"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Buffer for storing values of the
 * connected sensors for other modules to access
 *
 * @values See sns_g_SensorConfig_s in sns_i.h
 */
uint16_t sns_g_Values_u16[SNS_COUNT];

uint8_t sns_g_ActiveStatus_u8[SNS_COUNT];

/**
 * @brief Buffer for storing previous values of the potentiometers for filtering
 *
 * @valuessame as sensor values
 */
uint16_t sns_g_PrevValues_u16[SNS_COUNT][SNS_AVG_CNT];

/**
 * @brief Analog to digital converter channel
 *
 */
adc_channel_t sns_g_sensorChannel_t[SNS_COUNT];

/**************************************************************************
 * Functions
 **************************************************************************/

void sns_f_Init_v(void);
void sns_f_Handle_v(void);

#ifdef SERIAL_DEBUG
void sns_f_SerialDebug_v(void);
#endif

/**
 * @brief Initialise function to be called once on boot
 *
 * Configure the Analog to Digital converter unit and channel
 *
 * @return void
 */
void sns_f_Init_v(void)
{
  uint8_t i, j;

  /* Default ADC channel config for all inputs */
  adc_oneshot_chan_cfg_t channel_config = {
      .atten = ADC_ATTEN_DB_11,
      .bitwidth = ADC_BITWIDTH_12};
  /* Now actually go over all channels and initialize them */
  for (i = 0; i < SNS_COUNT; i++)
  {
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(sns_g_SensorConfig_s[i].pin_u16, &sns_g_SensorConfig_s[i].adc_unit_s, &sns_g_sensorChannel_t[i]));
    if (sns_g_SensorConfig_s[i].adc_unit_s == ADC_UNIT_1)
    {
      ESP_ERROR_CHECK(adc_oneshot_config_channel(main_g_AdcUnit1Handle_s, sns_g_sensorChannel_t[i], &channel_config));
    }
    else // ADC_UNIT_2
    {
      ESP_ERROR_CHECK(adc_oneshot_config_channel(main_g_AdcUnit2Handle_s, sns_g_sensorChannel_t[i], &channel_config));
    }

    /* And set all initial values for filter */
    for (j = 0; j < SNS_AVG_CNT; j++)
    {
      sns_g_PrevValues_u16[i][j] = 0;
    }
  }
}

/**
 * @brief Handle function to be called cyclically
 *
 * Read and store the sensor value
 *
 * @return void
 */
void sns_f_Handle_v(void)
{
  int i, j;
  uint32_t sum = 0;
  int readValue = 0;

  /* Go over all connected sensors */
  for (i = 0; i < SNS_COUNT; i++)
  {
    /* Based on pins ADC group, do the right reading: */
    if (sns_g_SensorConfig_s[i].adc_unit_s == ADC_UNIT_1)
    {
      ESP_ERROR_CHECK(adc_oneshot_read(main_g_AdcUnit1Handle_s, sns_g_sensorChannel_t[i], &readValue));
    }
    else // ADC_UNIT_2
    {
      ESP_ERROR_CHECK(adc_oneshot_read(main_g_AdcUnit2Handle_s, sns_g_sensorChannel_t[i], &readValue));
    }

    /* Shift all values in buffer to the next place in the array */
    for (j = SNS_AVG_CNT - 1; j > 0; j--)
    {
      sns_g_PrevValues_u16[i][j] = sns_g_PrevValues_u16[i][j - 1];
    }

    /* Set the current sensor value */
    sns_g_PrevValues_u16[i][0] = (uint16_t)readValue;

    /* Take the average of those readings */
    sum = 0;
    for (j = 0; j < SNS_AVG_CNT; j++)
    {
      sum += sns_g_PrevValues_u16[i][j];
    }

    /* Final sensor value assignment */
    sns_g_Values_u16[i] = (sum / SNS_AVG_CNT);

    /* Set sensor active if over threshold */
    sns_g_ActiveStatus_u8[i] = sns_g_Values_u16[i] > sns_g_SensorConfig_s[i].thresh_u16;
  }
}

#ifdef SERIAL_DEBUG
void sns_f_SerialDebug_v(void)
{
  uint16_t i;

  /* Go over all connected sensors */
  for (i = 0; i < SNS_COUNT; i++)
  {
    ESP_LOGD(SNS_TAG, "Sensor #%u reading = %u", i, sns_g_Values_u16[i]);
  }
}
#endif