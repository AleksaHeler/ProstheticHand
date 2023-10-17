/**
 * @file bat_e.h
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Header file for the corresponding btn.cpp
 *
 * @version 0.1
 * @date 2023-10-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef BAT_I_H
#define BAT_I_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "bat_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/


/**
 * @brief Defines for voltage divider: Vout = (Vs * R2) / (R1 + R2)
 *        where: R1 is connected to battery and R2 is to GND
 *  
 *        Resistance is in Ohms
 */
#define BAT_SENSE_R1 4680
#define BAT_SENSE_R2 1200

/**
 * @brief Battery voltage sensing is of course done over a voltage divider,
 *        so here we define what is the multiplier for our case of the voltage divider
 *        
 *        mult = Vs/Vout = (R1 + R2) / R2
 */
#define BAT_SENSE_MULT (((float32_t)BAT_SENSE_R1) + ((float32_t)BAT_SENSE_R2)) / ((float32_t)BAT_SENSE_R2)

/**
 * @brief With what to multiply the ADC reading to get the actual voltage?
 *        0 - 4096 == 0 - 3.3V -> 1 count = 0.805664mV
 */
#define BAT_ADC_COUNT_TO_MILLIVOLT_MULT (float32_t)0.8056640625

/**
 * @brief How many previous analog reads to take into account for averaging value 
 * 
 */
#define BAT_AVG_CNT 10

/**
 * @brief Configuration parameters of a potentiometer
 * Pot can be scaled automatically to any value by knowing its min/max values and offset
 * 
 */
typedef struct{
    /**
     * GPIO pin to which the sensing voltage divider is connected
     * 
     * 
     * @values See which pins are usable in file "ESP32_Pins.xlsx"
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
     * Multiplier for getting the actual voltage from voltage divider
     * 
     * @values see float32_t define
     */
    float32_t mult_f32;
} bat_s_BatSensConfig_t;

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Configuration for the battery sens pin, the code does all the rest
 */
bat_s_BatSensConfig_t bat_s_BatSensConfig_s = 
  /*  pin          adc_unit    mult          */
  {   GPIO_NUM_14, ADC_UNIT_2, BAT_SENSE_MULT };


/**
 * @brief Analog to digital converter channel
 * 
 */
extern adc_channel_t bat_g_BatAdcCh_s;

/**
 * @brief Buffer for storing previous values of the batt voltage for filtering
 * 
 * @values same as bat values
 */
extern float32_t bat_g_PrevBatVoltages_f32[BAT_AVG_CNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

/**
 * @brief Scale given ADC int input value to corresponding float voltage input
 *
 * @param val
 * @return scaled float32_t value
 */
extern float32_t bat_f_MapAdcToMillivolts_f32(uint16_t val);

#endif // BAT_I_H