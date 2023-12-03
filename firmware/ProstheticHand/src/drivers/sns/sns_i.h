/**
 * @file sensor_i.h
 * 
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 * 
 * @brief Header file for the corresponding pot.cpp
 * 
 * This file contains configurations needed by this module that should not be visible to other modules
 * 
 * @version 0.1
 * @date 2023-09-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SNS_I_H
#define SNS_I_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "sns_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/

/**
 * @brief How many previous analog reads to take into account for averaging value 
 * 
 */
#define SNS_AVG_CNT 50


/**
 * @brief Configuration parameters of a sensor
 */
typedef struct{
    /**
     * GPIO pin to which the sensor is connected
     * 
     * @values See which pins are usable in file "ESP32_Pins.xlsx
     */
    uint16_t pin_u16;

    /**
     * Which ADC unit does the pin belong to? 1 or 2
     * 
     * @values ADC_UNIT_1, ADC_UNIT_2
     */
    adc_unit_t adc_unit_s;

    /**
     * Minimum value for the sensor input 
     * (means muscle is 0% actuated - relaxed)
     * 
     * @values 0-4095
     */
    uint16_t min_val_u16;

    /**
     * Maximum value for the sensor input
     * (means muscle is 100% actuated - basically a cramp)
     * 
     * @values 0-4095
     */
    uint16_t max_val_u16;

    /**
     * Threshold value for servo activation
     * 
     * @values 0..4095
     */
    uint16_t thresh_u16;
} sns_s_SensorConfig_t;

/**************************************************************************
 * Global variables
 **************************************************************************/


/**
 * @brief Configures all connected sensors, the code does all the rest
 * 
 */
sns_s_SensorConfig_t sns_g_SensorConfig_s[SNS_COUNT] = {
  /*  pin          adc_unit  min_val max_val threshold */
  {   GPIO_NUM_18, ADC_UNIT_2,     0,   4095,     350 },  /* sensor 1   */
  {   GPIO_NUM_17, ADC_UNIT_2,     0,   4095,     350 }   /* sensor 2   */
};

/**
 * @brief Buffer for storing previous values of the potentiometers for filtering
 * 
 * @valuessame as sensor values
 */
extern uint16_t sns_g_PrevValues_u16[SNS_COUNT][SNS_AVG_CNT];

/**
 * @brief Analog to digital converter channel
 * 
 */
extern adc_channel_t sns_g_SensorChannel_t[SNS_COUNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

#endif // SNS_I_H