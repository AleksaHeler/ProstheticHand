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

#ifndef DSW_I_H
#define DSW_I_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "dsw_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/

typedef struct
{
  gpio_num_t gpio;
} dsw_PinConfig_s;

/**
 * @brief Stores the configuration of the dipswitch pins, with LSB first
 *
 * @values any available pin where the dipswitch is connected
 */
dsw_PinConfig_s dsw_g_PinConfigurations_s[DSW_COUNT] = 
{
  { GPIO_NUM_42 },
  { GPIO_NUM_41 },
  { GPIO_NUM_40 },
  { GPIO_NUM_39 }
};

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
dsw_HWRev_e dsw_g_HardwareRevision_e;

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void dsw_f_ReadConfig_v(void);

#endif // DSW_I_H