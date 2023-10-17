/**
 * @file sensor_e.h
 * 
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 * 
 * @brief Header file for the corresponding sensor.cpp
 * 
 * This file contains everything needed by other modules in order to use this module
 * 
 * @version 0.1
 * @date 2023-09-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SNS_E_H
#define SNS_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define SNS_TAG "SNS"

#define SNS_COUNT 2

/**************************************************************************
 * Global variables
 **************************************************************************/

extern uint16_t sns_g_Values_u16[SNS_COUNT];

extern uint8_t sns_g_ActiveStatus_u8[SNS_COUNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void sns_f_Init_v(void);
extern void sns_f_Handle_v(void);

#ifdef SERIAL_DEBUG
extern void sns_f_SerialDebug_v(void);
#endif


#endif // SNS_E_H