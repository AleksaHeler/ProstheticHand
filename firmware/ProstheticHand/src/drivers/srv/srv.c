/**
 * @file srv.cpp
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Servo software component / driver
 *
 * Here we have handling of the servos connected to the board.
 *
 * For now only a couple of servos are connected for debugging,
 * and we just set their angle to the potentiometer value.
 *
 * @version 0.1
 * @date 2023-09-19
 *
 * @bug Angle scaling to the duty cycle uses a fixed angle value of 180, needs to be fixed in a future update
 *
 * @copyright Copyright (c) 2023
 *
 */

/**************************************************************************
 * Includes
 **************************************************************************/

/* Own header file */
#include "srv_e.h"
#include "srv_i.h"

/* Other components used here */
#include "drivers/dsw/dsw_e.h"
#include "drivers/pot/pot_e.h"
#include "drivers/sns/sns_e.h"
#include "drivers/btn/btn_e.h"


/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Stores the servo's angle value as a duty cycle
 *
 * @values srv_c_minimumAllowedDuty_f32..max allowed duty
 */
uint16_t srv_g_Positions_u16[SRV_COUNT];

/**************************************************************************
 * Functions
 **************************************************************************/

void srv_f_Init_v(void);
void srv_f_Handle_v(void);

void srv_f_CalculateSrvAngleFromPot_f32(uint8_t servoIndex, uint8_t potIndex);
void srv_f_CalculateSrvAngleFromSensor_f32(uint8_t servoIndex, uint8_t sensorIndex);
void srv_f_CalculateSrvAngleFromSensorThreshold_f32(uint8_t servoIndex, uint8_t sensorIndex);
void srv_f_CalculateSrvAngleFromBtn_f32(uint8_t servoIndex, uint8_t btnIndex);
void srv_f_CalculatePWMFromPercentage_f32(uint8_t servoIndex, float32_t pwmDutyPercent);

#ifdef SERIAL_DEBUG
void srv_f_SerialDebug_v(void);
#endif

float32_t srv_c_minimumAllowedDuty_f32[SRV_COUNT];

/**
 * @brief Init function called once on boot
 *
 * Set timer and channel configuration for the PWM signal
 *
 * @return void
 */
void srv_f_Init_v(void)
{
  uint8_t i;

  ledc_timer_config_t srvPWM_TimerConfig = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = PWM_RESOLUTION,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = PWM_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&srvPWM_TimerConfig));

  /* Go over all servo pins and initialize the channel */
  for (i = 0; i < SRV_COUNT; i++)
  {
    ledc_channel_config_t srvPWM_ChannelConfig = {
        .gpio_num = srv_s_ServoConfig_s[i].pin_u16,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = srv_s_ServoConfig_s[i].chn_s,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {.output_invert = 0}};
    ESP_ERROR_CHECK(ledc_channel_config(&srvPWM_ChannelConfig));

    srv_c_minimumAllowedDuty_f32[i] = SERVO_MIN_DUTY_CYCLE + (srv_s_ServoConfig_s[i].min_angle_u16 * srv_c_OneDegreeAsDuty_f32);
  }
}

/**
 * @brief Handle function to be called cyclically
 *
 * Set the servos to an angle that relates to the chosen potentiometer/sensor
 *
 * @return void
 */
void srv_f_Handle_v(void)
{
  uint8_t i;

  for (i = 0; i < SRV_COUNT; i++)
  {
    /* Check how should the angle be calculated (by pot or sensor value?)*/
    if (dsw_g_HardwareRevision_e == REV00) /* POT controlled */
    {
      srv_f_CalculateSrvAngleFromPot_f32(i, SERVO_CONTROL_POT_INDEX);
    }
    else if (dsw_g_HardwareRevision_e == REV01) /* SNS controlled */
    {
      srv_f_CalculateSrvAngleFromSensor_f32(i, SERVO_CONTROL_SNS_INDEX);
    }
    else if (dsw_g_HardwareRevision_e == REV02){ /* BTN controlled */
      srv_f_CalculateSrvAngleFromBtn_f32(i, SERVO_CONTROL_BTN_INDEX);
    }
    else if (dsw_g_HardwareRevision_e == REV03){ /* SNS controlled (with threshold) */
      srv_f_CalculateSrvAngleFromSensorThreshold_f32(i, SERVO_CONTROL_SNS_INDEX);
    }
    else if (dsw_g_HardwareRevision_e == REV04) { /* Full PWM range control (through POT) */
      srv_f_CalculatePWMFromPercentage_f32(i, pot_g_PotValues_f32[SERVO_PWM_POT_INDEX]);
    }

    /* Finally set and update each servo PWM signal's duty cycle  */
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, srv_s_ServoConfig_s[i].chn_s, srv_g_Positions_u16[i]));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, srv_s_ServoConfig_s[i].chn_s));
  }
}

/**
 * @brief Calculates angle for given servo based on given potentiometer
 *
 */
void srv_f_CalculateSrvAngleFromPot_f32(uint8_t servoIndex, uint8_t potIndex)
{
  srv_g_Positions_u16[servoIndex] = srv_c_minimumAllowedDuty_f32[servoIndex] + (srv_s_ServoConfig_s[servoIndex].max_angle_u16 * srv_c_OneDegreeAsDuty_f32) * pot_g_PotValues_f32[SERVO_CONTROL_POT_INDEX];
}

/**
 * @brief Calculates angle for given servo based on given sensor
 *
 */
void srv_f_CalculateSrvAngleFromSensor_f32(uint8_t servoIndex, uint8_t sensorIndex)
{
  srv_g_Positions_u16[servoIndex] = srv_c_minimumAllowedDuty_f32[servoIndex] + (srv_s_ServoConfig_s[servoIndex].max_angle_u16 * srv_c_OneDegreeAsDuty_f32) * sns_g_Values_u16[0] / 4095;
}

/**
 * @brief Calculates angle for given servo based on given sensor
 * while taking into account the activation threshold of the sensor
 * 
 */
void srv_f_CalculateSrvAngleFromSensorThreshold_f32(uint8_t servoIndex, uint8_t sensorIndex)
{
  float32_t angle;

  if(sns_g_ActiveStatus_u8[sensorIndex] == 1){
    angle = pot_g_PotValues_f32[SERVO_ANGLE_MAX_POT_INDEX];
  }
  else {
    angle = pot_g_PotValues_f32[SERVO_ANGLE_MIN_POT_INDEX];
  }

  srv_g_Positions_u16[servoIndex] = srv_c_minimumAllowedDuty_f32[servoIndex] + (srv_s_ServoConfig_s[servoIndex].max_angle_u16 * srv_c_OneDegreeAsDuty_f32) * angle;
}

/**
 * @brief Calculates angle for given servo based on button press
 * 
 */
void srv_f_CalculateSrvAngleFromBtn_f32(uint8_t servoIndex, uint8_t btnIndex)
{
  float32_t angle;
  if(btn_g_BtnStates_u8[btnIndex] == 1){
    angle = pot_g_PotValues_f32[SERVO_ANGLE_MAX_POT_INDEX];
  }
  else {
    angle = pot_g_PotValues_f32[SERVO_ANGLE_MIN_POT_INDEX];
  }
  srv_g_Positions_u16[servoIndex] = srv_c_minimumAllowedDuty_f32[servoIndex] + (srv_s_ServoConfig_s[servoIndex].max_angle_u16 * srv_c_OneDegreeAsDuty_f32) * angle;
}

/**
 * @brief Writes PWM signal from 0% duty to 100% duty (always on) based on given value
 * 
 */
void srv_f_CalculatePWMFromPercentage_f32(uint8_t servoIndex, float32_t pwmDutyPercent)
{
  srv_g_Positions_u16[servoIndex] = SERVO_100_PERCENT_DUTY_CYCLE * pwmDutyPercent;
}

#ifdef SERIAL_DEBUG
void srv_f_SerialDebug_v(void)
{
  uint8_t i;

  /* Go over all servos */
  for (i = 0; i < SRV_COUNT; i++)
  {
    ESP_LOGD(SRV_TAG, "Servo #%d position = %d", i, srv_g_Positions_u16[i]);
  }
}
#endif