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
#include "drivers/pot/pot_e.h"
#include "drivers/sensor/sensor_e.h"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Stores the servo's angle value as a duty cycle
 * 
 * @values srv_c_minimumAllowedDuty_f32..max allowed duty
 */
uint16_t srv_g_Angle_u16;

/**************************************************************************
 * Functions
 **************************************************************************/

void srv_f_Init_v(void);
void srv_f_Handle_v(void);

#ifdef SERIAL_DEBUG
void srv_f_SerialDebug_v(void);
#endif

/**
 * @brief Init function called once on boot
 * 
 * Set timer and channel configuration for the PWM signal
 * 
 * @return void
 */
void srv_f_Init_v(void)
{
    ledc_timer_config_t srvPWM_TimerConfig = {
        .speed_mode         = LEDC_LOW_SPEED_MODE,
        .duty_resolution    = PWM_RESOLUTION,
        .timer_num          = LEDC_TIMER_0,
        .freq_hz            = PWM_FREQUENCY,
        .clk_cfg            = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&srvPWM_TimerConfig));

    /* @todo update later to make it more scalable for multiple servos */
    ledc_channel_config_t srvPWM0_ChannelConfig = {
        .gpio_num   = SERVO_PIN0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&srvPWM0_ChannelConfig));

    ledc_channel_config_t srvPWM1_ChannelConfig = {
        .gpio_num   = SERVO_PIN1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&srvPWM1_ChannelConfig));
}

/**
 * @brief Handle function to be called cyclically
 * 
 * Set the servos to an angle that relates to the chosen potentiometer
 * 
 * @return void
 */
void srv_f_Handle_v(void)
{
    /* Scale the angle to the duty cycle */
    srv_g_Angle_u16 = srv_c_minimumAllowedDuty_f32 + (SERVO_MAX_ANGLE * srv_c_OneDegreeAsDuty_f32) * pot_g_PotValues_f32[SERVO_CONTROL_POT_INDEX] / 100;
    //srv_g_Angle_u16 = srv_c_minimumAllowedDuty_f32 + (SERVO_MAX_ANGLE * srv_c_OneDegreeAsDuty_f32) * sensor_g_Value_u16 / 4095;

    /* Set and update the PWM signal's duty cycle */
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, srv_g_Angle_u16));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    /* Using a 2nd channel makes it possible to give different PWM signals to each servo */
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, srv_g_Angle_u16));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

#ifdef SERIAL_DEBUG
void srv_f_SerialDebug_v(void){
    ESP_LOGD(SRV_TAG, "MIN_CYCLE = %f, MAX_CYCLE = %f, duty = %u", SERVO_MIN_DUTY_CYCLE, SERVO_MAX_DUTY_CYCLE, srv_g_Angle_u16);
}
#endif