/**
 * @file btn_e.h
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Header file for the corresponding btn.cpp
 *
 * This file contains everything needed by other modules in order to use this module
 *
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef BTN_E_H
#define BTN_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define BTN_TAG "BTN"

#define BTN_COUNT 2

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * Buffer for storing if a button is pressed
 *
 * @values 0..1 (HIGH/LOW, TRUE/FALSE, PRESSED/RELEASED...)
 */
extern uint8_t btn_g_BtnStates_u8[BTN_COUNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void btn_f_Init_v(void);
extern void btn_f_Handle_v(void);
#ifdef SERIAL_DEBUG
extern void btn_f_SerialDebug_v(void);
#endif

#endif // BTN_E_H