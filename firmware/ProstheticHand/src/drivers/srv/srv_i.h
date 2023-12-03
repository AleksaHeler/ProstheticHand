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
#define BITS_TO_MAX_VAL(x) ((1 << x) - 1)

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
#define PWM_FREQUENCY 50

/**
 * @brief Used when initializing servos, to know the min/max pulse times for 180 degrees
 * Normally servos take 1-2ms pulse for 0-180 degrees, but that may vary
 *
 * @values 500..2500 (microseconds)
 */
#define SERVO_MIN_WIDTH_US 600
#define SERVO_MAX_WIDTH_US 2400

/**
 * @brief Minimum duty cycle that the servo responds to
 *
 * Corresponds to 0 degrees
 */
#define SERVO_MIN_DUTY_CYCLE (BITS_TO_MAX_VAL(PWM_RESOLUTION) / ((float32_t)(1000000 / PWM_FREQUENCY) / SERVO_MIN_WIDTH_US))

/**
 * @brief Maximum duty cycle that the servo responds to
 *
 * Corresponds to 180 degrees
 */
#define SERVO_MAX_DUTY_CYCLE (BITS_TO_MAX_VAL(PWM_RESOLUTION) / ((float32_t)(1000000 / PWM_FREQUENCY) / SERVO_MAX_WIDTH_US))

/**
 * @brief Minimum duty cycle that the servo responds to
 *
 * Corresponds to 0 degrees
 */
#define SERVO_100_PERCENT_DUTY_CYCLE BITS_TO_MAX_VAL(PWM_RESOLUTION)

/**
 * @brief The value of a single degree angle in duty cycle length
 *
 */
const float32_t srv_c_OneDegreeAsDuty_f32 = (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE) / 180.0;

/**
 * @brief Configuration parameters of a servo motor
 */
typedef struct
{
  /**
   * GPIO pin to which the sensor is connected
   *
   * @values See which pins are usable in file "ESP32_Pins.xlsx
   */
  uint16_t pin_u16;

  /**
   * Timer channel. Using a different channels for different servos
   * makes it possible to give different PWM signals to each servo
   *
   * @values See which pins are usable in file "ESP32_Pins.xlsx
   */
  ledc_channel_t chn_s;

  /**
   * Minimum value for the sensor input
   * (means muscle is 0% actuated - relaxed)
   *
   * @values 0-4096
   */
  uint16_t min_angle_u16;

  /**
   * Maximum value for the sensor input
   * (means muscle is 100% actuated - basically a cramp)
   *
   * @values 0-4096
   */
  uint16_t max_angle_u16;
} srv_s_ServoConfig_t;

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Configures all connected sensors, the code does all the rest
 *
 */
srv_s_ServoConfig_t srv_s_ServoConfig_s[SRV_COUNT] = {
    /*  pin          channel       min_val max_val */
    {GPIO_NUM_4,  LEDC_CHANNEL_0,  30,     150 }, /* servo 1   */
    {GPIO_NUM_5,  LEDC_CHANNEL_0,  30,     150 }, /* servo 2   */
    {GPIO_NUM_6,  LEDC_CHANNEL_0,  30,     150 }  /* servo 3   */
};

/**
 * @brief Duty cycle that corresponds to minimum angle set by SERVO_MIN_ANGLE
 *
 * @todo delet
 */
extern float32_t srv_c_minimumAllowedDuty_f32[SRV_COUNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void srv_f_CalculateSrvAngleFromPot_f32(uint8_t servoIndex, uint8_t potIndex);
extern void srv_f_CalculateSrvAngleFromSensor_f32(uint8_t servoIndex, uint8_t sensorIndex);
extern void srv_f_CalculateSrvAngleFromSensorThreshold_f32(uint8_t servoIndex, uint8_t sensorIndex);
extern void srv_f_CalculateSrvAngleFromBtn_f32(uint8_t servoIndex, uint8_t btnIndex);

#endif // SRV_I_H