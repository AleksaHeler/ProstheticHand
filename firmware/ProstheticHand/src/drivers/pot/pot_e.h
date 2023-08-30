/** @file pot_e.h
 *  @brief Header file for the corresponding pot.cpp
 *
 *  This file contains everything needed by other modules in order
 *  to use this module.
 *  @todo Aleksa Heler: add comment for this file (what is located here?)
 *  @todo Aleksa Heler: add structure that defines pot input, so that it's modular
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */
#ifndef POT_E_H
#define POT_E_H


/********************************************************************************
 *** Includes
 *******************************************************************************/

#include "config/project.h"


/********************************************************************************
 *** Defines
 *******************************************************************************/

#define POT_COUNT 3

/********************************************************************************
 *** Global variables
 *******************************************************************************/

/**
 * Buffer for storing values of the potentiometers connected
 *
 * @values See pot_PotConfig_u8 below
 */
extern float32_t pot_g_PotValues_f32[POT_COUNT];

/********************************************************************************
 *** Functions
 *******************************************************************************/

extern void pot_f_Init_v( void );
extern void pot_f_Handle_v( void );
#ifdef SERIAL_DEBUG
extern void pot_f_SerialDebug_v( void );
#endif

#endif /* POT_E_H */