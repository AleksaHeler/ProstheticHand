/**
 * @file pot_i.h
 * 
 * @author Aleksa Heler (aleksaheler@gmail.com)
 * 
 * @brief Header file for the corresponding pot.cpp
 * 
 * This file contains configurations needed by this module that should not be visible to other modules
 * 
 * 
 * @version 0.1
 * @date 2023-09-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef POT_I_H
#define POT_I_H


/**************************************************************************
 * Includes
 **************************************************************************/

#include "pot_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/

/**
 * @brief Configuration parameters of a potentiometer
 * Pot can be scaled automatically to any value by knowing its min/max values and offset
 * 
 */
typedef struct{
    /**
     * GPIO pin to which the potentiometer is connected
     * 
     * 
     * @values See which pins are usable in file "ESP32_Pins.xlsx
     */
    uint16_t pin_u16;

    /**
     * Minimum value returned when the pot is at its lowest
     * 
     * @values see float32_t define
     */
    float32_t min_val_f32;

    /**
     * Maximum value returned when the pot is at its lowest
     * 
     * @values see float32_t define
     */
    float32_t max_val_f32;

    /**
     * Add this offset when returning pot value
     * 
     * @values see float32_t define
     */
    float32_t offset_f32;

    /**
     * How many ADC readings to take at once to average reading
     * 
     * @values see float32_t define
     */
    uint16_t averageCount_u16;

    /**
     * How much the previous value should affect current reading
     * reading = prevValMult * prevReading + (1-prevValMult) * currReading
     * 
     * @values 0..1
     */
    float32_t prevValMult_f32;
} pot_s_PotConfig_t;

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Configures all connected potentiometers, the code does all the rest
 * 
 */
pot_s_PotConfig_t pot_g_PotConfig_s[POT_COUNT] = {
  /* pin min_val max_val offset averageCount prevValMult  */
  {   GPIO_NUM_25,      0,    180,     0,           2,        0.5, },  /* Normal 10k pot */ 
  {   GPIO_NUM_26,      0,    180,     0,           2,        0.8, },  /* 10k Trim pot   */ 
  {   GPIO_NUM_27,      0,    180,     0,           2,        0.8, }   /* 10k Trim pot   */
};

/**
 * @brief Buffer for storing previous values of the potentiometers for filtering
 * 
 * @values See pot_g_PotConfig_s above
 */
extern float32_t pot_g_PotPrevValues_f32[POT_COUNT];

/**
 * @brief Buffer for storing the sum of all readings to be averaged
 * 
 * @values See pot_g_PotConfig_s above 
 */
extern float32_t pot_g_PotReadingSum_f32[POT_COUNT];

/**
 * @brief Analog to Digital converter unit handle
 * 
 */
extern adc_oneshot_unit_handle_t pot_g_handle;

/**
 * @brief Analog to digital converter channel
 * 
 */
extern adc_channel_t pot_g_channel[POT_COUNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern float32_t pot_f_AnalogRead_f32(uint16_t potIndex);
extern float32_t pot_f_MapFloat_f32(uint16_t val, uint16_t in_min, uint16_t in_max, float32_t out_min, float32_t out_max);

#endif // POT_I_H