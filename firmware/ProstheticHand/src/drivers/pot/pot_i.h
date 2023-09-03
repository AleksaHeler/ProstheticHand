/** @file pot_i.h
 *  @brief Header file for the corresponding pot.cpp
 *
 *  This file contains configurations needed by this modules 
 *  that should not be visible to other modules.
 *  @todo Aleksa Heler: add comment for this file (what is located here?)
 *  @todo Aleksa Heler: add structure that defines pot input, so that it's modular
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */
#ifndef POT_I_H
#define POT_I_H


/********************************************************************************
 *** Includes
 *******************************************************************************/

#include "pot_e.h"


/********************************************************************************
 *** Defines
 *******************************************************************************/

/**
 * Define everything we need to know for configuration of a potentiometer.
 * Pot can be scaled automatically to any value by knowing its min/max values and an offset
 */
typedef struct {
  /**
   * GPIO pin to which the potentiometer is connected
   *
   * @values See which pins are usable in file "ESP32_Pins.xlsx"
   */
  uint16_t pin_u16;
  /**
   * GPIO pin to which the potentiometer is connected
   *
   * @values See which pins are usable in file "ESP32_Pins.xlsx"
   */
  uint16_t adcGroup_u16;
  /**
   * GPIO pin to which the potentiometer is connected
   *
   * @values See which pins are usable in file "ESP32_Pins.xlsx"
   */
  uint16_t adcChannel_u16;
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
  /**7
   * How many ADC readings to take at once to average reading
   *
   * @values see float32_t define
   */
  uint16_t averageCount_u16;
  /**
   * How much previous value should affect current reading
   * reading = prevValMult * prevReading + (1-prevValMult) * currReading
   *
   * @values 0..1
   */
  float32_t prevValMult_f32;
} pot_g_PotConfig_t;

/********************************************************************************
 *** Global variables
 *******************************************************************************/

/**
 * Configures all connected potentiometers, the code does all the rest
 */
pot_g_PotConfig_t pot_g_PotConfig_s[POT_COUNT] = {
  /* pin adc chn min_val max_val offset averageCount prevValMult  */
  {   25,  1,  8,      0,    180,     0,           2,        0.5, },  /* Normal 10k pot */ 
  {   26,  2,  9,      0,    180,     0,           2,        0.8, },  /* 10k Trim pot   */ 
  {   27,  2,  7,      0,    180,     0,           2,        0.8, }   /* 10k Trim pot   */
};


/**
 * Buffer for storing previous values of the potentiometers for filtering
 *
 * @values See pot_g_PotConfig_s above
 */
extern float32_t pot_g_PotPrevValues_f32[POT_COUNT];

/**
 * Buffer for storing previous values of the potentiometers for filtering
 *
 * @values See pot_g_PotConfig_s above
 */
extern float32_t pot_g_PotReadingSum_f32[POT_COUNT];

/********************************************************************************
 *** Functions
 *******************************************************************************/

extern float32_t pot_f_AnalogRead_f32( uint16_t potIndex );
extern float32_t pot_f_MapFloat_f32( uint16_t val, uint16_t in_min, uint16_t in_max, float32_t out_min, float32_t out_max );

#endif /* POT_I_H */