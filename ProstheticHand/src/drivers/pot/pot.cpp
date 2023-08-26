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
#include "pot.h"


/********************************************************************************
 *** Global variables
 *******************************************************************************/

/**
 * Buffer for storing values of the potentiometers connected
 *
 * @values 0..100 (percents)
 */
uint16_t pot_PotStates_u8[3];


/********************************************************************************
 *** Functions
 *******************************************************************************/

void pot_Init_v( void );
void pot_Handle_v( void );


/** @brief Init function called once on boot 
 *
 *  Set all the given pins as inputs
 *
 *  @return void
 */
void pot_Init_v( void )
{
  Serial.println("POT: init");

  pinMode(POT_PIN0, INPUT);
  pinMode(POT_PIN1, INPUT);
  pinMode(POT_PIN2, INPUT);
}


/** @brief Handle function to be called cyclically
 *
 *  Read all the defined potentiometers and store their scaled analgo 
 *  value as a percentage of max value in internall buffer
 *
 *  @return void
 */
void pot_Handle_v( void )
{
  Serial.print("POT: handle    ");
  pot_PotStates_u8[0] = (uint16_t)map(analogRead(POT_PIN0), 0, 4096, 0, 100);
  pot_PotStates_u8[1] = (uint16_t)map(analogRead(POT_PIN1), 0, 4096, 0, 100);
  pot_PotStates_u8[2] = (uint16_t)map(analogRead(POT_PIN2), 0, 4096, 0, 100);
}