/**
 * @file dipsw.cpp
 * 
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 * 
 * @brief This module handles the development board's configuration and mode of operation.
 * 
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

uint8_t dipsw_g_DIPConfig_u8;

/**************************************************************************
 * Functions
 **************************************************************************/

void dipsw_f_Init_v(void);


void dipsw_f_Init_v(void)
{
    uint8_t i;
    uint64_t dipsw_pin_mask = (1ULL << DIP_0) | (1ULL << DIP_1) | (1ULL << DIP_2) | (1ULL << DIP_3);

    gpio_config_t dipsw_pin_config = {
        .pin_bit_mask   = dipsw_pin_mask,
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&dipsw_pin_config));
}