/**
 * @file pot_e.h
 * 
 * @author Aleksa Heler (aleksaheler@gmail.com)
 * 
 * @brief Header file for the corresponding pot.cpp
 * 
 * This file contains everything needed by other modules in order to use this module
 * 
 * @todo Aleksa Heler: add comment for this file (what is located here?)
 * @todo Aleksa Heler: add structure that defines pot input, so that it's modular
 * 
 * @version 0.1
 * @date 2023-09-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef POT_E_H
#define POT_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define POT_COUNT 3

#define POT_TAG "POT"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Buffer for storing values of the connected potentiometers 
 * 
 * @values See pot_g_PotConfig_s in pot_i.h
 */
extern float32_t pot_g_PotValues_f32[POT_COUNT];

/**************************************************************************
 * Functions
 **************************************************************************/

extern void pot_f_Init_v(void);
extern void pot_f_Handle_v(void);
#ifdef SERIAL_DEBUG
extern void pot_f_SerialDebug_v(void);
#endif


#endif // POT_E_H