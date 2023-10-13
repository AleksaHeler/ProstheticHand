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

#include "dipsw_e.h"
#include "dipsw_i.h"

/**************************************************************************
 * Global variables
 **************************************************************************/


/**************************************************************************
 * Functions
 **************************************************************************/

void dipsw_f_Init_v(void);
void dipsw_f_ReadConfig_v(void);

/**
 * @brief Initialize function to be called once on startup/boot
 *
 *  Set all the given pins as inputs and call the ReadConfig function
 *
 *  @return void
 */
void dipsw_f_Init_v(void)
{
  uint64_t dipsw_pin_mask = (1ULL << DIP_0) | (1ULL << DIP_1) | (1ULL << DIP_2) | (1ULL << DIP_3);

  gpio_config_t dipsw_pin_config = {
    .pin_bit_mask   = dipsw_pin_mask,
    .mode           = GPIO_MODE_INPUT,
    .pull_up_en     = GPIO_PULLUP_ENABLE,
    .pull_down_en   = GPIO_PULLDOWN_DISABLE,
    .intr_type      = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&dipsw_pin_config));

  dipsw_f_ReadConfig_v();
}

/**
 * @brief Read all the configuration pins and store the values
 * 
 * @return void
 */
void dipsw_f_ReadConfig_v(void)
{
  dipsw_g_SignalSrcConfig_e = (ESigSrc)!gpio_get_level(DIP_0);
}