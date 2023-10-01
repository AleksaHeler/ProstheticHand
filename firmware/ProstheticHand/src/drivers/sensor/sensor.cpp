/**
 * @file sensor.cpp
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
#include "sensor_e.h"
#include "sensor_i.h"

/**************************************************************************
 * Global variables
 **************************************************************************/

uint16_t sensor_g_Value_u16;

/**************************************************************************
 * Functions
 **************************************************************************/

void sensor_f_Init_v(void);
void sensor_f_Handle_v(void);
void sensor_f_Deinit_v(void);
#ifdef SERIAL_DEBUG
void sensor_f_SerialDebug_v(void);
#endif



void sensor_f_Init_v(void)
{
    adc_unit_t adc_unit = ADC_UNIT_2;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &sensor_g_UnitHandle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12
    };
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(SENSOR_PIN, &adc_unit, &sensor_g_sensorChannel_t));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(sensor_g_UnitHandle, sensor_g_sensorChannel_t, &channel_config));
}

void sensor_f_Handle_v(void)
{
    ESP_ERROR_CHECK(adc_oneshot_read(sensor_g_UnitHandle, sensor_g_sensorChannel_t, (int*)&sensor_g_Value_u16));
}

void sensor_f_Deinit_v(void)
{
    ESP_ERROR_CHECK(adc_oneshot_del_unit(sensor_g_UnitHandle));
}

#ifdef SERIAL_DEBUG
void sensor_f_SerialDebug_v(void)
{
    ESP_LOGI(SENSOR_TAG, "Sensor reading: %u", sensor_g_Value_u16);
}
#endif