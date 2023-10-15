/**
 * @file dipsw.cpp
 *
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 *
 * @brief This module handles the development board's configuration and mode of operation.
 *
 *
 * @bug The module lacks all the functionality and as such, isn't utilised in the main program yet
 *
 * @version 0.1
 * @date 2023-10-12
 *
 * @copyright Copyright (c) 2023
 *
 */

/**************************************************************************
 * Includes
 **************************************************************************/

#include "dsw_e.h"
#include "dsw_i.h"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**************************************************************************
 * Functions
 **************************************************************************/

void dsw_f_Init_v(void);
void dsw_f_ReadConfig_v(void);

/**
 * @brief Initialize function to be called once on startup/boot
 *
 *  Set all the given pins as inputs and call the ReadConfig function
 *
 *  @return void
 */
void dsw_f_Init_v(void)
{
  uint8_t i;
  uint64_t dsw_pin_mask = 0;

  /* Go over all pins and create a mask where bits corresponding to given pin are set to 1 */
  for(i = 0; i < DSW_COUNT; i++){
    dsw_pin_mask |= 1ULL << dsw_g_PinConfigurations_s[i].gpio;
  }
  
  /* Create a struct of the configuration (pullup, input, switches pull the pin to GND) */
  gpio_config_t dsw_pin_config = {
      .pin_bit_mask = dsw_pin_mask,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&dsw_pin_config));

  /* Actually read the digital inputs now! */
  dsw_f_ReadConfig_v();
}

/**
 * @brief Read all the configuration pins and store the values
 *
 * @return void
 */
void dsw_f_ReadConfig_v(void)
{
  uint8_t i;
  uint64_t dsw_values = 0;

  /* Read all pins, and shift them to their corresponding bits */
  for(i = 0; i < DSW_COUNT; i++){
    dsw_values |= (!gpio_get_level(dsw_g_PinConfigurations_s[i].gpio)) << i;
  }
  
  /* Store in globally accessible variable */
  dsw_g_HardwareRevision_e = (dsw_HWRev_e)dsw_values;
}

#ifdef SERIAL_DEBUG
void dsw_f_SerialDebug_v(void)
{
    ESP_LOGD(DSW_TAG, "Dipswitch reading = %u", dsw_g_HardwareRevision_e);
}
#endif
