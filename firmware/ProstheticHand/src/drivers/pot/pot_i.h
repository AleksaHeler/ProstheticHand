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
 * @brief How many previous analog reads to take into account for averaging value 
 * 
 */
#define POT_AVG_CNT 50

/**
 * @brief Maximum output value of the potentiometer
 * 
 * @values 0..1
 */
const float32_t pot_c_PotMaxVal_f32 = 1.0;

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
     * To which ADC unit does the pin belong to? 1 or 2
     * 
     * 
     * @values ADC_UNIT_1, ADC_UNIT_2
     */
    adc_unit_t adc_unit_s;

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
} pot_s_PotConfig_t;

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Configures all connected potentiometers, the code does all the rest
 * 
 */
pot_s_PotConfig_t pot_g_PotConfig_s[POT_COUNT] = {
  /*  pin          adc_unit  min_val max_val offset */
  {   GPIO_NUM_10, ADC_UNIT_2,     0,    pot_c_PotMaxVal_f32,     0 },  /* Normal 5k pot  */ 
  {   GPIO_NUM_11, ADC_UNIT_2,     0,    pot_c_PotMaxVal_f32,     0 },  /* Normal 5k pot  */ 
  {   GPIO_NUM_12, ADC_UNIT_2,     0,    pot_c_PotMaxVal_f32,     0 }   /* 10k Trim pot   */
};


/**
 * @brief Buffer for storing previous values of the potentiometers for filtering
 * 
 * @values same as pot values
 */
extern float32_t pot_g_PrevValues_f32[POT_COUNT][POT_AVG_CNT];

/**
 * @brief Analog to digital converter channel
 * 
 */
extern adc_channel_t pot_g_channel_s[POT_COUNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern float32_t pot_f_AnalogRead_f32(uint16_t potIndex);
extern float32_t pot_f_MapFloat_f32(uint16_t val, uint16_t in_min, uint16_t in_max, float32_t out_min, float32_t out_max);

#endif // POT_I_H