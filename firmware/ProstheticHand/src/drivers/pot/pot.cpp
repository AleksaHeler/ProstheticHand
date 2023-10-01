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

/**************************************************************************
 * Functions
 **************************************************************************/

void pot_f_Init_v(void);
void pot_f_Handle_v(void);
#ifdef SERIAL_DEBUG
void pot_f_SerialDebug_v(void);
#endif
float32_t pot_f_AnalogRead_f32(uint16_t potIndex);

/**
 * @brief Init function called once on boot
 * 
 * Set all the given pins as inputs
 * 
 * @return void
 */
void pot_f_Init_v(void)
{
    uint16_t i;

    /* Configure all given pins as inputs */
    gpio_config_t pot_pin_config = {
        pot_pin_config.pin_bit_mask = (1ULL << pot_g_PotConfig_s[0].pin_u16),
        pot_pin_config.mode         = GPIO_MODE_INPUT,
        pot_pin_config.pull_up_en   = GPIO_PULLUP_DISABLE,
        pot_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE,
        pot_pin_config.intr_type    = GPIO_INTR_ANYEDGE
    };

    for(i = 1; i < POT_COUNT; i++){
        pot_pin_config.pin_bit_mask |= (1ULL << pot_g_PotConfig_s[i].pin_u16);
    }

    ESP_ERROR_CHECK(gpio_config(&pot_pin_config));

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
    adc_oneshot_unit_handle_t adc2_handle_s;
    adc_unit_t unit = ADC_UNIT_2;

    adc_oneshot_unit_init_cfg_t pot_Config_s = {
        .unit_id = unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&pot_Config_s, &adc2_handle_s));

    adc_oneshot_chan_cfg_t adc_config_s = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12
    };

    adc_channel_t pot_Channel_e;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pot_g_PotConfig_s[potIndex].pin_u16, &unit, &pot_Channel_e));

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle_s, pot_Channel_e, &adc_config_s));

    uint16_t adcAnalogRead;

    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle_s, pot_Channel_e, (int*)&adcAnalogRead));

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle_s));

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
