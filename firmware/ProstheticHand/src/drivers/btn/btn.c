/**
 * @file btn.cpp
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Button software component / driver
 *  This file deals with the handling fo the 4 buttons connected to the board.
 *  The button states are read and stored in local buffers for easier access.
 *
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

/**************************************************************************
 * Includes
 **************************************************************************/

#include "btn_e.h"
#include "btn_i.h"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * Buffer for storing button states
 *
 * @values 0..1 (HIGH/LOW, TRUE/FALSE...)
 */
uint8_t btn_g_BtnStates_u8[BTN_COUNT];

/**************************************************************************
 * Functions
 **************************************************************************/

void btn_f_Init_v(void);
void btn_f_Handle_v(void);

#ifdef SERIAL_DEBUG
void btn_f_SerialDebug_v(void);
#endif

/**
 * @brief Initialize function to be called once on startup/boot
 *
 *  Set all the given pins as inputs
 *
 *  @return void
 */
void btn_f_Init_v(void)
{
  uint8_t i;
  uint64_t btn_pin_mask = 0;

  /* Combine all button masks into one (set a bit to 1 on the correct pin index) */
  for (i = 0; i < BTN_COUNT; i++)
  {
    btn_pin_mask |= 1ULL << btn_g_BtnPins_s[i];
  }

  /* Now use that mask to configure the input ports */
  /* The pins used for btn don't have a pull-up/pull-down resistor in hardware,
  so we're using built-in ESP pullups here (and buttons connect the pin to ground) */
  gpio_config_t btn_pin_config = {
      .pin_bit_mask = btn_pin_mask,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&btn_pin_config));
}

/**
 * @brief Handle function to be called cyclically
 *
 * Read all the defined buttons and store them in an internal buffer
 *
 * @return void
 */
void btn_f_Handle_v(void)
{
  uint8_t i;

  /* Go over all buttons and store their inverse digital value, */
  /* since when the button is pressed it's connected to GND */
  for (i = 0; i < BTN_COUNT; i++)
  {
    btn_g_BtnStates_u8[i] = !gpio_get_level(btn_g_BtnPins_s[i]);
  }
}

#ifdef SERIAL_DEBUG
void btn_f_SerialDebug_v(void)
{
  uint16_t i;

  /* For all buttons, write their value to serial com */
  for (i = 0; i < BTN_COUNT; i++)
  {
    ESP_LOGD(BTN_TAG, "Button #%u state = %u", i, btn_g_BtnStates_u8[i]);
  }
}
#endif