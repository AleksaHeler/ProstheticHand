/**
 * @file pot.cpp
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Potentiometer software component / driver
 *
 * Here we have handling of the potentiometers connected to the board.
 * For now only a couple of pots are connected. We just read their values and store them in local buffers for easier later use.
 *
 * @version 0.1
 * @date 2023-09-18
 *
 * @copyright Copyright (c) 2023
 *
 */

/**************************************************************************
 * Includes
 **************************************************************************/

/* Own header file */
#include "pot_e.h"
#include "pot_i.h"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Buffer for storing values of the connected potentiometers
 *
 * @values See pot_g_PotConfig_s in pot_i.h
 */
float32_t pot_g_PotValues_f32[POT_COUNT];

/**
 * @brief Buffer for storing previous values of the potentiometers for filtering
 *
 * @values See pot_g_PotConfig_s in pot_i.h
 */
float32_t pot_g_PrevValues_f32[POT_COUNT][POT_AVG_CNT];

/**
 * @brief Analog to digital converter channel
 *
 */
adc_channel_t pot_g_channel_s[POT_COUNT];

adc_oneshot_unit_handle_t *pot_g_AdcHandle_s;

/**************************************************************************
 * Functions
 **************************************************************************/

void pot_f_Init_v(void);
void pot_f_Handle_v(void);

float32_t pot_f_AnalogRead_f32(uint16_t potIndex);

#ifdef SERIAL_DEBUG
void pot_f_SerialDebug_v(void);
#endif

/**
 * @brief Initialise function to be called once on boot
 *
 * Configure the Analog to Digital converter unit and channels
 *
 * @return void
 */
void pot_f_Init_v(void)
{
  uint16_t i, j;

  /* Default configuration for all channels */
  adc_oneshot_chan_cfg_t channel_config = {
      .atten = ADC_ATTEN_DB_11,
      .bitwidth = ADC_BITWIDTH_12};

  /* Now actually go over all channels and initialize them */
  for (i = 0; i < POT_COUNT; i++)
  {
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pot_g_PotConfig_s[i].pin_u16, &pot_g_PotConfig_s[i].adc_unit_s, &pot_g_channel_s[i]));
    if(pot_g_PotConfig_s[i].adc_unit_s == ADC_UNIT_1)
    {
      ESP_ERROR_CHECK(adc_oneshot_config_channel(main_g_AdcUnit1Handle_s, pot_g_channel_s[i], &channel_config));
    }
    else // ADC_UNIT_2
    {
      ESP_ERROR_CHECK(adc_oneshot_config_channel(main_g_AdcUnit2Handle_s, pot_g_channel_s[i], &channel_config));
    }
  }

  /* Set initial pot read value for filter */
  for (i = 0; i < POT_COUNT; i++)
  {
    for (j = 0; j < POT_AVG_CNT; j++)
    {
      pot_g_PrevValues_f32[i][j] = 0;
    }
  }
}

/**
 * @brief Handle function to be called cyclically
 *
 * Read all the defined potentiometers and store their scaled analog value as a percentage of max value in internal buffer
 *
 * @return void
 */
void pot_f_Handle_v(void)
{
  uint16_t i, j;
  float32_t sum = 0;

  /* Go over all the channels to be read */
  for (i = 0; i < POT_COUNT; i++)
  {
    /* Shift all values in buffer to the next place in the array */
    for (j = POT_AVG_CNT - 1; j > 0; j--)
    {
      pot_g_PrevValues_f32[i][j] = pot_g_PrevValues_f32[i][j - 1];
    }

    /* Read current pot value: */
    pot_g_PrevValues_f32[i][0] = pot_f_AnalogRead_f32(i);

    /* Take the average of those readings */
    sum = 0;
    for (j = 0; j < POT_AVG_CNT; j++)
    {
      sum += pot_g_PrevValues_f32[i][j];
    }
    pot_g_PotValues_f32[i] = sum / POT_AVG_CNT;
  }
}

#ifdef SERIAL_DEBUG
void pot_f_SerialDebug_v(void)
{
  uint16_t i;

  for (i = 0; i < POT_COUNT; i++)
  {
    ESP_LOGD(POT_TAG, "Pot #%u value = %f", i, pot_g_PotValues_f32[i]);
  }
}
#endif

/**
 * @brief Single shot analog read of given pot index
 *
 * Read ADC value of pot, and scale it as in config in pot_i.h
 *
 * @param potIndex
 * @return One time scaled analog reading of given pin index as float32_t
 */
float32_t pot_f_AnalogRead_f32(uint16_t potIndex)
{
  uint16_t adcAnalogRead = 0;

  /* Based on which ADC group this pin belongs to, read the corresponding group */
  if (pot_g_PotConfig_s[potIndex].adc_unit_s == ADC_UNIT_1)
  {
    ESP_ERROR_CHECK(adc_oneshot_read(main_g_AdcUnit1Handle_s, pot_g_channel_s[potIndex], (int *)&adcAnalogRead));
  }
  else // ADC_UNIT_2
  {
    ESP_ERROR_CHECK(adc_oneshot_read(main_g_AdcUnit2Handle_s, pot_g_channel_s[potIndex], (int *)&adcAnalogRead));
  }

  /* Return scaled value based on POT define */
  return pot_g_PotConfig_s[potIndex].offset_f32 + (float32_t)pot_f_MapFloat_f32(
                                                      adcAnalogRead,
                                                      0,
                                                      4095,
                                                      pot_g_PotConfig_s[potIndex].min_val_f32,
                                                      pot_g_PotConfig_s[potIndex].max_val_f32);
}

/**
 * @brief Scale given int input value to float output
 *
 * @param val
 * @param in_min
 * @param in_max
 * @param out_min
 * @param out_max
 * @return scaled float32_t value
 */
float32_t pot_f_MapFloat_f32(uint16_t val, uint16_t in_min, uint16_t in_max, float32_t out_min, float32_t out_max)
{
  return (float32_t)(val - in_min) * (out_max - out_min) / (float32_t)(in_max - in_min) + out_min;
}
