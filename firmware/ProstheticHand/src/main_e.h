/**
 * @file main_e.h
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

#ifndef MAIN_E_H
#define MAIN_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define MAIN_TAG "MAIN"

/**
 * @brief Length of the main cycle
 *
 * At what intervals will the main OS cycle be called (in milliseconds)
 * Recommended to be greater than 1ms as ESP32 is not really that fast,
 * but keep in mind it has to be somewhat frequent so we don't get stuttering
 *
 * @values no less than 1 ms
 */
#define MAIN_CYCLE_LENGTH_MS 10

/**
 * @brief Into how many cases is the main cycle divided? If this is for example 10,
 * and the main cycle is 10, then we will have 1ms containers
 *
 * @values recommended 1 - 10 (has to be able to divide MAIN_CYCLE_LENGTH_MS)
 */
#define MAIN_CYCLE_TASK_COUNT 10

/**
 * @brief Length of one main cycle sub-task (main cycle time / task count)
 *
 * @values in  microseconds
 */
const uint16_t main_c_CycleTaskLengthUs_u16 = MILLISEC_TO_MICROSEC * MAIN_CYCLE_LENGTH_MS / MAIN_CYCLE_TASK_COUNT;

/**************************************************************************
 * Structures
 **************************************************************************/

typedef struct
{
  uint32_t currentCycle_u32;
  uint32_t minCycle_u32;
  uint32_t maxCycle_u32;
} main_g_RuntimeMeasTyp_t;

/**************************************************************************
 * Global variables
 **************************************************************************/

extern main_g_RuntimeMeasTyp_t main_g_RuntimeMeas_s[MAIN_CYCLE_TASK_COUNT];

/**************************************************************************
 * Function prototypes
 **************************************************************************/

#endif // MAIN_E_H