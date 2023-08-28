/** @file pot.cpp
 *  @brief Potentiometer software component / driver
 *
 *  Here we have handling of the potentiometers connected to the board.
 *  For now only couple of pots are connected, and we just read their values
 *  and store them in local buffers for easier later use.
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */


/********************************************************************************
 *** Includes
 *******************************************************************************/

/* Own header file */
#include "pot_e.h"
#include "pot_i.h"


/********************************************************************************
 *** Global variables
 *******************************************************************************/

/**
 * Buffer for storing values of the potentiometers connected
 *
 * @values See pot_PotConfig_u8 in pot_i.h
 */
float32_t pot_g_PotValues_f32[POT_COUNT];

/**
 * Buffer for storing previous values of the potentiometers for filtering
 *
 * @values See pot_PotConfig_u8 in pot_i.h
 */
float32_t pot_g_PotPrevValues_f32[POT_COUNT];

/**
 * Buffer for storing previous values of the potentiometers for filtering
 *
 * @values See pot_PotConfig_u8 in pot_i.h
 */
float32_t pot_g_PotReadingSum_f32[POT_COUNT];


/********************************************************************************
 *** Functions
 *******************************************************************************/

void pot_f_Init_v( void );
void pot_f_Handle_v( void );
float32_t pot_f_AnalogRead_f32( uint16_t potIndex );


/** @brief Init function called once on boot 
 *
 *  Set all the given pins as inputs
 *
 *  @return void
 */
void pot_f_Init_v( void )
{
  uint16_t i;

  #ifdef SERIAL_DEBUG
  Serial.println("POT: init");
  #endif

  /* Configure all given pins as inputs */
  for(i = 0; i < POT_COUNT; i++) {
    pinMode(pot_g_PotConfig_s[i].pin_u16, INPUT);
  }

  /* Set initial pot read value for filter */
  for(i = 0; i < POT_COUNT; i++) {
    pot_g_PotPrevValues_f32[i] = pot_f_AnalogRead_f32(i);
  }
}


/** @brief Handle function to be called cyclically
 *
 *  Read all the defined potentiometers and store their scaled analgo 
 *  value as a percentage of max value in internall buffer
 *
 *  @return void
 */
void pot_f_Handle_v( void )
{
  uint16_t i, j;

  #ifdef SERIAL_DEBUG
  Serial.print("POT: handle   ");
  Serial.print(pot_g_PotValues_f32[0]);
  Serial.print("   ");
  Serial.print(pot_g_PotValues_f32[1]);
  Serial.print("   ");
  Serial.println(pot_g_PotValues_f32[2]);
  #endif

  /* Set sum to 0 first */
  for(i = 0; i < POT_COUNT; i++) {
    pot_g_PotReadingSum_f32[i] = 0;
  }

  /* Go over all channels to be read */
  for(i = 0; i < POT_COUNT; i++) {
    /* Read each channel N times to be averaged */
    for(j = 0; j < pot_g_PotConfig_s[i].averageCount_u16; j++) {
      pot_g_PotReadingSum_f32[i] += pot_f_AnalogRead_f32(i);
    }
    /* Average all those readings */
    pot_g_PotValues_f32[i] = pot_g_PotReadingSum_f32[i] / pot_g_PotConfig_s[i].averageCount_u16;
  }
  
  /* Take % of last reading and % of current reading as resulting value */
  for(i = 0; i < POT_COUNT; i++) {
    /* result = mult * prev + (1 - mult) * curr */
    pot_g_PotValues_f32[i] = pot_g_PotConfig_s[i].prevValMult_f32 * pot_g_PotPrevValues_f32[i] + (1.0 - pot_g_PotConfig_s[i].prevValMult_f32) * pot_g_PotValues_f32[i];
  }

  /* Keep track of current value as it will become previous next time */
  for(i = 0; i < POT_COUNT; i++) {
    pot_g_PotPrevValues_f32[i] = pot_g_PotValues_f32[i];
  }
}

/** @brief Single shot analog read of given pot index
 *
 *  Read ADC value of pot, and scale it as in config in pot_i.h
 *
 *  @return One time scaled analog reading of given pin index as float32_t
 */
float32_t pot_f_AnalogRead_f32( uint16_t potIndex ){
  return pot_g_PotConfig_s[potIndex].offset_f32 + (float32_t)pot_f_MapFloat_f32(
    analogRead(pot_g_PotConfig_s[potIndex].pin_u16), 
    0,
    4096,
    pot_g_PotConfig_s[potIndex].min_val_f32,
    pot_g_PotConfig_s[potIndex].max_val_f32
  );
}

/** @brief Scale given int input value to float output
 *
 *  @return scaled float value
 */
float32_t pot_f_MapFloat_f32(uint16_t val, uint16_t in_min, uint16_t in_max, float32_t out_min, float32_t out_max) {
  return (float32_t)(val - in_min) * (out_max - out_min) / (float32_t)(in_max - in_min) + out_min;
}