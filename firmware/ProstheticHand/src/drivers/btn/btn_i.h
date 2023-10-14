/**
 * @file btn_e.h
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Header file for the corresponding btn.cpp
 *
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef BTN_I_H
#define BTN_I_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "btn_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Configures all connected buttons, the code does all the rest
 */
gpio_num_t btn_g_BtnPins_s[BTN_COUNT] = {
    /* Pin          Comment        */
    GPIO_NUM_1, /* Normal button  */
    GPIO_NUM_2  /* Normal button  */
};

/**************************************************************************
 * Function prototypes
 **************************************************************************/

#endif // BTN_I_H