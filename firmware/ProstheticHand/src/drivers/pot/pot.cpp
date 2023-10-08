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
float32_t pot_g_PotPrevValues_f32[POT_COUNT];

/**
 * @brief Buffer for storing the sum of all readings to be averaged
 * 
 * @values See pot_g_PotConfig_s in pot_i.h 
 */
float32_t pot_g_PotReadingSum_f32[POT_COUNT];

/**
 * @brief Analog to Digital converter unit handle
 * 
 */
adc_oneshot_unit_handle_t pot_g_handle;

/**
 * @brief Analog to digital converter channel
 * 
 */
adc_channel_t pot_g_channel[POT_COUNT];

/**************************************************************************
 * Functions
 **************************************************************************/

void pot_f_Init_v(void);
void pot_f_Handle_v(void);
void pot_f_Deinit_v(void);
#ifdef SERIAL_DEBUG
void pot_f_SerialDebug_v(void);
#endif
float32_t pot_f_AnalogRead_f32(uint16_t potIndex);

/**
 * @brief Initialise function to be called once on boot
 * 
 * Configure the Analog to Digital converter unit and channels
 * 
 * @return void
 */
void pot_f_Init_v(void)
{
    uint16_t i;

    adc_unit_t adc_unit = ADC_UNIT_2;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
#if PROCESSOR_TYPE == ESP32
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
#endif
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &pot_g_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12
    };
    for(i = 0; i < POT_COUNT; i++){
        ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pot_g_PotConfig_s[i].pin_u16, &adc_unit, &pot_g_channel[i]));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(pot_g_handle, pot_g_channel[i], &channel_config));
    }

    /* Set initial pot read value for filter */
    for(i = 0; i < POT_COUNT; i++){
        pot_g_PotPrevValues_f32[i] = pot_f_AnalogRead_f32(i);
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

    /* Set sum to 0 first */
    for(i = 0; i < POT_COUNT; i++){
        pot_g_PotReadingSum_f32[i] = 0;
    }

    /* Go over all the channels to be read */
    for(i = 0; i < POT_COUNT; i++){
        /* Read each channel N times to be averaged */  // N equals the averageCount value in the PotConfig structure for each potentiometer
        for(j = 0; j < pot_g_PotConfig_s[i].averageCount_u16; j++){
                pot_g_PotReadingSum_f32[i] += pot_f_AnalogRead_f32(i);
        }

        /* Take the average of those readings */
        pot_g_PotValues_f32[i] = pot_g_PotReadingSum_f32[i] / pot_g_PotConfig_s[i].averageCount_u16;
    }

    /* Take % of last reading and % of current reading as resulting value */
    for(i = 0; i < POT_COUNT; i++){
        /* result = multipiler * prev + (1 - multiplier) * curr */
        pot_g_PotValues_f32[i] = pot_g_PotConfig_s[i].prevValMult_f32 * pot_g_PotPrevValues_f32[i] + (1.0 - pot_g_PotConfig_s[i].prevValMult_f32) * pot_g_PotValues_f32[i];
    }

    /* Keep track of the current value as it will become the previous value in the next cycle */
    for(i = 0; i < POT_COUNT; i++){
        pot_g_PotPrevValues_f32[i] = pot_g_PotValues_f32[i];
    }
}

/**
 * @brief Deinitialisation function
 * 
 * Delete the ADC unit
 * 
 * @return void
 */
void pot_f_Deinit_v(void)
{
    ESP_ERROR_CHECK(adc_oneshot_del_unit(pot_g_handle));
}

#ifdef SERIAL_DEBUG
void pot_f_SerialDebug_v(void)
{
    uint16_t i;

    for(i = 0; i < POT_COUNT; i++){
        ESP_LOGD(POT_TAG, "Pot%u: %f", i, pot_g_PotValues_f32[i]);
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
    uint16_t adcAnalogRead;

    ESP_ERROR_CHECK(adc_oneshot_read(pot_g_handle, pot_g_channel[potIndex], (int*)&adcAnalogRead));

    return pot_g_PotConfig_s[potIndex].offset_f32 + (float32_t)pot_f_MapFloat_f32(
    adcAnalogRead,
    0,
    4095,
    pot_g_PotConfig_s[potIndex].min_val_f32,
    pot_g_PotConfig_s[potIndex].max_val_f32
    );
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
