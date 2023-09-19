/**
 * @file btn.cpp
 * 
 * @author your name (you@domain.com)
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

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * Buffer for storing button states
 * 
 * @values 0, 1 (HIGH/LOW, TRUE/FALSE...)
 */
uint8_t btn_BtnStates_u8[4];

/**************************************************************************
 * Functions
 **************************************************************************/

void btn_Init_v(void);
void btn_Handle_v( void );

#ifdef SERIAL_DEBUG
void btn_SerialDebug_v( void );
#endif

/**
 * @brief Initialize function to be called once on startup/boot
 *
 *  Set all the given pins as inputs
 *
 *  @return void
 */
void btn_Init_v(void)
{
    gpio_config_t btn_pin_config{
        (1ULL << BTN_0) | (1ULL << BTN_1) | (1ULL << BTN_2) | (1ULL << BTN_3),
        GPIO_MODE_INPUT,
        GPIO_PULLUP_ENABLE, // The pins used don't have a pull-up/pull-down resistor, so we've implemented our own in the PCB design
        GPIO_PULLDOWN_DISABLE,
        GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn_pin_config));
}


/**
 * @brief Handle function to be called cyclically
 * 
 * Read all the defined buttons and store them in an internal buffer
 *
 * @return void
 */
void btn_Handle_v( void )
{
    btn_BtnStates_u8[0] = !gpio_get_level(BTN_0);
    btn_BtnStates_u8[1] = !gpio_get_level(BTN_1);
    btn_BtnStates_u8[2] = !gpio_get_level(BTN_2);
    btn_BtnStates_u8[3] = !gpio_get_level(BTN_3);
}

#ifdef SERIAL_DEBUG
void btn_SerialDebug_v(void)
{
    uint16_t i;

    for(i = 0; i < 4; i++){
        ESP_LOGD(BTN_TAG, "Button %u: %u", i, btn_BtnStates_u8[i]);
    }
}
#endif