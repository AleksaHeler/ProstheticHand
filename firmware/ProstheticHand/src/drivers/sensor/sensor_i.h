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

#ifndef SENSOR_I_H
#define SENSOR_I_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "sensor_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/



/**************************************************************************
 * Global variables
 **************************************************************************/

adc_oneshot_unit_handle_t sensor_g_UnitHandle;
adc_channel_t sensor_g_sensorChannel_t;

/**************************************************************************
 * Function prototypes
 **************************************************************************/

#endif // SENSOR_I_H