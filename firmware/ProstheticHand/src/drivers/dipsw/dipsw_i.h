/**
 * @file dipsw_i.h
 * 
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 * 
 * @brief Header file for the corresponding btn.cpp
 * 
 * This file contains configurations needed by this module that should not be visible to other modules
 * 
 * @version 0.1
 * @date 2023-10-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef DIPSW_I_H
#define DIPSW_I_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "dipsw_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define DIP_0 GPIO_NUM_42
#define DIP_1 GPIO_NUM_41
#define DIP_2 GPIO_NUM_40
#define DIP_3 GPIO_NUM_39

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Stores the signal source configuration
 * 
 * Tells us whether we're using signal from the potentiometer,
 * or the signal from the sensor
 * 
 * @values 0..1 (SIG_SRC_POT/SIG_SRC_SENS)
 */
ESigSrc dipsw_g_SignalSrcConfig_e;

/**************************************************************************
 * Function prototypes
 **************************************************************************/


#endif // DIPSW_I_H