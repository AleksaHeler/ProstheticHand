/*
 * led.c
 *
 *  Created on: Jun 11, 2024
 *      Author: aleksaheler
 */

/* Own header files */
#include "led_i.h"
#include "led_e.h"

/* System header files */
#include "main.h"

/* External header files */


/**
 * @todo Description!
 */
void led_f_Init_v(void)
{

}

/**
 * @todo Description!
 * @note Called every 1s
 */
void led_f_Handle_v(void)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
}
