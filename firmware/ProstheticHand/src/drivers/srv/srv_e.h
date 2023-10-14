/**
 * @file srv_e.h
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief Header file for the corresponding srv.cpp
 *
 * This file contains everything needed by other modules in order to use this module
 *
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef SRV_E_H
#define SRV_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"
#include "driver/ledc.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define SRV_TAG "SRV"

#define SRV_COUNT 3

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * @brief Stores the servo's angle value as a duty cycle
 *
 * @values srv_c_minimumAllowedDuty_f32..max allowed duty
 */
extern uint16_t srv_g_Positions_u16[SRV_COUNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void srv_f_Init_v(void);
extern void srv_f_Handle_v(void);

#ifdef SERIAL_DEBUG
extern void srv_f_SerialDebug_v(void);
#endif

#endif // SRV_E_H