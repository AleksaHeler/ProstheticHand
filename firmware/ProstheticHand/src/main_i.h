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
#define MAIN_SERIAL_DEBUG_DELAY 1000 / portTICK_PERIOD_MS
#endif

/**
 * @brief Debug LED pin
 * 
 * Indicates that the board is functional
 *
 * @values GPIO pin
 */
#define MAIN_DEBUG_LED_01_PIN GPIO_NUM_37

/**
 * @brief How many main 10ms cycles to wait until changing debug LED state
 *
 * @values 0-250 = 0-2.5s
 */
#define MAIN_DEBUG_LED_01_CYCLE_COUNT 99

/**
 * @brief Debug LED pin
 * 
 * Indicates sensor threshold
 * 
 * @values GPIO pin
 */
#define MAIN_DEBUG_LED_02_PIN GPIO_NUM_38

/**************************************************************************
 * Structures
 **************************************************************************/

/**************************************************************************
 * Global variables
 **************************************************************************/

extern uint64_t main_g_CurrMicros_u64;
extern uint64_t main_g_LastMicros_u64;

extern uint16_t main_g_CurrTaskIndex_u16;

extern uint16_t main_g_DebugLED01Countdown_u16;
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
extern void main_f_ADCInit_v(void);
extern void main_f_DebugLEDInit_v(void);
extern void main_f_DebugLEDHandle_v(void);

#endif // MAIN_I_H