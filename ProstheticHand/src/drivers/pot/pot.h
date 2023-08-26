/** @file pot.h
 *  @brief Header file for the corresponding pot.cpp
 *
 *  @todo Aleksa Heler: add comment for this file (what is located here?)
 *  @todo Aleksa Heler: add structure that defines pot input, so that it's modular
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */
#ifndef POT_H
#define POT_H


/********************************************************************************
 *** Includes
 *******************************************************************************/

#include "config/project.h"


/********************************************************************************
 *** Defines
 *******************************************************************************/

#define POT_PIN0 25
#define POT_PIN1 26
#define POT_PIN2 27


/********************************************************************************
 *** Global variables
 *******************************************************************************/

/**
 * Buffer for storing values of the potentiometers connected
 *
 * @values 0..100 (percents)
 */
extern uint16_t pot_PotStates_u8[3];


/********************************************************************************
 *** Functions
 *******************************************************************************/

extern void pot_Init_v( void );
extern void pot_Handle_v( void );

#endif /* POT_H */