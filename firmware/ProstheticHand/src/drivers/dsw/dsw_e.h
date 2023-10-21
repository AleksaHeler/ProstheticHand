/**
 * @file dsw_e.h
 *
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 *
 * @brief Header file containing DIP switch configurations for the development board
 *
 * This file contains everything needed by other modules in order to use this module

 *
 * @version 0.1
 * @date 2023-10-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DSW_E_H
#define DSW_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define DSW_TAG "DSW"

#define DSW_COUNT 4

/**
 * @brief Enum that defines all possible HW revisions of our board
 *
 * Must have all values defined for 2^DSW_COUNT (for 4 dip switches that's total of 16 possible revisions)
 * @todo Document all wanted/possible/used variants, in docs, as well as here!
 */
typedef enum
{
  REV00 = 0, /* Servo angle controlled by a value from a potentiometer */
  REV01,     /* Servo angle controlled by a value from a sensor */
  REV02,     /* Servo controlled if a button is pressed (min/max angle controlled by servo position) */
  REV03,     /* Servo controlled if a sensor threshold is activated (min/max angle controlled by servo position) */
  REV04,     /* Servo PWM output from 0% duty to 100% duty (even if that is not a standard pot signal) based on potentiometer value */
  REV05,
  REV06,
  REV07,
  REV08,
  REV09,
  REV10,
  REV11,
  REV12,
  REV13,
  REV14,
  REV15
} dsw_HWRev_e;

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Stores the signal source configuration read from the DIP switch
 *
 * Tells us whether we're using signal from the potentiometer,
 * or the signal from the sensor
 *
 * @values 0..1 (SIG_SRC_POT/SIG_SRC_SENS)
 */
extern dsw_HWRev_e dsw_g_HardwareRevision_e;

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void dsw_f_Init_v(void);

#ifdef SERIAL_DEBUG
extern void dsw_f_SerialDebug_v(void);
#endif

#endif // DSW_E_H