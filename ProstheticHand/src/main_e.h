/** @file main_e.h
 *  @brief Header file for the corresponding main.cpp
 *
 *  @todo Aleksa Heler: add comment for this file (what is located here?)
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */
#ifndef MAIN_E_H
#define MAIN_E_H

/********************************************************************************
 *** Includes
 *******************************************************************************/

#include "config/project.h"


/********************************************************************************
 *** Defines
 *******************************************************************************/

#define MAIN_TIMER_CALLBACK_COUNT 2


/********************************************************************************
 *** Structures
 *******************************************************************************/

typedef struct {
  uint16_t currExecTime;
  uint16_t minExecTime;
  uint16_t maxExecTime;
  uint16_t stuff;
} main_g_RuntimeMeas_t;


/********************************************************************************
 *** Variables
 *******************************************************************************/

extern main_g_RuntimeMeas_t main_g_RuntimeMeas_s[MAIN_TIMER_CALLBACK_COUNT];

/********************************************************************************
 *** Functions
 *******************************************************************************/


#endif /* MAIN_E_H */