/**
 * @file sensor_e.h
 * 
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 * 
 * @brief Header file for the corresponding sensor.cpp
 * 
 * This file contains everything needed by other modules in order to use this module
 * 
 * @version 0.1
 * @date 2023-09-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SENSOR_E_H
#define SENSOR_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"

/**************************************************************************
 * Defines
 **************************************************************************/

/**************************************************************************
 * Global variables
 **************************************************************************/

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void sensor_f_Init_v(void);
extern void sensor_f_Handle_v(void);
#ifdef SERIAL_DEBUG
extern void sensor_f_SerialDebug_v(void);
#endif


#endif // SENSOR_E_H