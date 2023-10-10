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

#define SERVO_PIN0 GPIO_NUM_4
#define SERVO_PIN1 GPIO_NUM_6
#define SERVO_PIN3 GPIO_NUM_15


#define SRV_TAG "SRV"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void srv_f_Init_v(void);
extern void srv_f_Handle_v(void);

#ifdef SERIAL_DEBUG
extern void srv_f_SerialDebug_v(void);
#endif

#endif // SRV_E_H