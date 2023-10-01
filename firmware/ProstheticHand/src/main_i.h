/**
 * @file main_i.h
 * 
 * @author Aleksa Heler (aleksaheler@gmail.com)
 * 
 * @brief Header file for the corresponding main.cpp
 * 
 * @todo Aleksa Heler: add comment for this file (what is located here?)
 * 
 * @version 0.1
 * @date 2023-09-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MAIN_I_H
#define MAIN_I_H


/**************************************************************************
 * Includes
 **************************************************************************/

#include "main_e.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#ifdef SERIAL_DEBUG
/**
 * @brief How long to wait between writing serial data to console between writes
 * 
 * @values recommended more than 50 (milliseconds)
 */
#define MAIN_SERIAL_DEBUG_DELAY 1000/portTICK_PERIOD_MS
#endif

/**************************************************************************
 * Structures
 **************************************************************************/

/**************************************************************************
 * Global variables
 **************************************************************************/

extern uint64_t main_g_CurrMicros_u64;
extern uint64_t main_g_LastMicros_u64;

extern uint16_t main_g_CurrTaskIndex_u16;


extern TaskHandle_t main_g_SerialDebugTaskHandle_s;

/**************************************************************************
 * Functions
 **************************************************************************/

extern void main_f_Init_v(void);
extern void main_f_Handle_v(void);

#ifdef SERIAL_DEBUG
extern void main_f_SerialDebug_v(void *arg);
#endif

extern uint32_t main_f_StartRTM_v(void);
extern uint32_t main_f_StopRTM_v(uint32_t rtmStart);
extern void main_f_HandleRTMStats_v(uint16_t index);

#endif // MAIN_I_H