/**
 * @file bat_e.h
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Header file for the corresponding bat.cpp
 *
 * This file contains everything needed by other modules in order to use this module
 *
 * @version 0.1
 * @date 2023-10-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef BAT_E_H
#define BAT_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define BAT_TAG "BTN"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * Buffer for storing battery voltage
 *
 * @values Voltage, hopefully no higher than 16.8V (for 4 cell Li-ion battery)
 */
extern float32_t bat_g_BatVoltage_f32;


/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void bat_f_Init_v(void);
extern void bat_f_Handle_v(void);

#ifdef SERIAL_DEBUG
extern void bat_f_SerialDebug_v(void);
#endif

#endif // BAT_E_H