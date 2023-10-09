/**
 * @file srv_e.h
 * 
 * @author Aleksa Heler (aleksaheler@gmail.com)
 * 
 * @brief Header file for the corresponding srv.cpp
 * 
 * @todo Aleksa Heler: add comment for this file (what is located here?)
 * 
 * @version 0.1
 * @date 2023-09-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SRV_E_H
#define SRV_E_H


/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"
#include "driver/ledc.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define SERVO_PIN0 GPIO_NUM_4
#define SERVO_PIN1 GPIO_NUM_6
#define SERVO_PIN3 GPIO_NUM_15

#define PWM_RESOLUTION LEDC_TIMER_13_BIT

#define BITS_TO_MAX_VAL(x) ((1<<x)-1)

#define SRV_TAG "SRV"


/**
 * Used when initializing servos, to know the min/max pulse times for 180 degrees
 * Normally servos take 1-2ms pulse for 0-1800 degrees, but that may vary
 * 
 * @values 500..2500 (microseconds)
 * 
 */
#define SERVO_MIN_WIDTH_US 500
#define SERVO_MAX_WIDTH_US 2500

/**
 * Min/max angles of the servo in degrees (So that the finger doesn't destroy itself)
 * 
 * @values 0..180 (degrees)
 */
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

/**
 * @brief Index of the potentiometer that controls the servos
 * 
 * @values 0..number of potentiometers (index)
 */
#define SERVO_CONTROL_POT_INDEX 0

/**************************************************************************
 * Global variables
 **************************************************************************/

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void srv_f_Init_v(void);
extern void srv_f_Handle_v(void);

#ifdef SERIAL_DEBUG
extern void srv_f_SerialDebug_v(void);
#endif

#endif // SRV_E_H