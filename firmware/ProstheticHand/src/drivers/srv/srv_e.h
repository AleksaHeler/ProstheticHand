/** @file srv_e.h
 *  @brief Header file for the corresponding srv.cpp
 *
 *  @todo Aleksa Heler: add comment for this file (what is located here?)
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */
#ifndef SRV_E_H
#define SRV_E_H


/********************************************************************************
 *** Includes
 *******************************************************************************/

#include "config/project.h"
#include "include/ESP32_Servo/ESP32_Servo.h"


/********************************************************************************
 *** Defines
 *******************************************************************************/

/**
 * Used when initializing servos, to know the min/max pulse times for 180 degrees
 * Normally servos take 1-2ms pulse for 0-180 degrees, but that may vary
 *
 * @values 500..2500 (microseconds)
 */
#define SERVO_PIN0 18
#define SERVO_PIN1 19

/**
 * Used when initializing servos, to know the min/max pulse times for 180 degrees
 * Normally servos take 1-2ms pulse for 0-180 degrees, but that may vary
 *
 * @values 500..2500 (microseconds)
 */
#define SERVO_MIN_ANGLE_US 600
#define SERVO_MAX_ANGLE_US 2400

/**
 * Min/max angles in degrees of the servo so the finger doesn't destroy itself
 *
 * @values 0..180 (degrees)
 */
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180


/**
 * Index of the potentiometer that controls the servos
 *
 * @values 0..number of potentiometers (index)
 */
#define SERVO_CONTROL_POT_INDEX 0

/********************************************************************************
 *** Global variables
 *******************************************************************************/


/********************************************************************************
 *** Functions
 *******************************************************************************/

extern void srv_Init_v( void );
extern void srv_Handle_v( void );
#ifdef SERIAL_DEBUG
extern void srv_SerialDebug_v( void );
#endif

#endif /* SRV_E_H */