/**
 * @file srv_i.h
 * 
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 * 
 * @brief Header file for the corresponding srv.cpp
 * 
 * This file contains configurations needed by this module that should not be visible to other modules
 * 
 * @version 0.1
 * @date 2023-10-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SRV_I_H
#define SRV_I_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "srv_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/

/**
 * @brief Converts the number of bits to the maximum value they can represent
 * 
 * The calculation equals 2^x
 */
#define BITS_TO_MAX_VAL(x) ((1<<x)-1)

/**
 * @brief Duty cycle resolution
 * 
 * The value of 12 bits gives us 4096 discrete levels of control over the signal width.
 * This means that the signal width will change in steps of roughly 0.74us.
 * 
 * Considering that the deadband of the servo we're using is 2us, this resolution is
 * more than adequate
 */
#define PWM_RESOLUTION LEDC_TIMER_12_BIT

/**
 * @brief Servo motor operating frequency
 * 
 * The frequency of 330Hz means the pulse will be triggered rougly every 3.03ms
 */
#define PWM_FREQUENCY 330  

/**
 * @brief Used when initializing servos, to know the min/max pulse times for 180 degrees
 * Normally servos take 1-2ms pulse for 0-180 degrees, but that may vary
 * 
 * @values 500..2500 (microseconds)
 */
#define SERVO_MIN_WIDTH_US 500
#define SERVO_MAX_WIDTH_US 2500

/**
 * @brief Minimum duty cycle that the servo responds to
 * 
 * Corresponds to 0 degrees
 */
#define SERVO_MIN_DUTY_CYCLE (BITS_TO_MAX_VAL(PWM_RESOLUTION)/((float32_t)(1000000/PWM_FREQUENCY)/SERVO_MIN_WIDTH_US))

/**
 * @brief Maximum duty cycle that the servo responds to
 * 
 * Corresponds to 180 degrees
 */
#define SERVO_MAX_DUTY_CYCLE (BITS_TO_MAX_VAL(PWM_RESOLUTION)/((float32_t)(1000000/PWM_FREQUENCY)/SERVO_MAX_WIDTH_US))



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
#define SERVO_CONTROL_POT_INDEX 1

/**
 * @brief The value of a single degree angle in duty cycle length
 * 
 */
const float32_t srv_c_OneDegreeAsDuty_f32 = (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE) / 180;

/**
 * @brief Duty cycle that corresponds to minimum angle set by SERVO_MIN_ANGLE
 * 
 */
const float32_t srv_c_minimumAllowedDuty_f32 = SERVO_MIN_DUTY_CYCLE + (SERVO_MIN_ANGLE * srv_c_OneDegreeAsDuty_f32);


/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Stores the servo's angle value as a duty cycle
 * 
 * @values srv_c_minimumAllowedDuty_f32..max allowed duty
 */
extern uint16_t srv_g_Angle_u16;

#endif