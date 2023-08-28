/** @file main.h
 *  @brief Header file for the corresponding main.cpp
 *
 *  @todo Aleksa Heler: add comment for this file (what is located here?)
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */
#ifndef MAIN_I_H
#define MAIN_I_H

/********************************************************************************
 *** Includes
 *******************************************************************************/

#include "main_e.h"


/********************************************************************************
 *** Defines
 *******************************************************************************/

#define MAIN_1MS_MEASURE_INDEX 0
#define MAIN_10MS_MEASURE_INDEX 1

/********************************************************************************
 *** Structures
 *******************************************************************************/


/********************************************************************************
 *** Variables
 *******************************************************************************/

hw_timer_t *main_g_1msTimer_s = NULL;
hw_timer_t *main_g_10msTimer_s = NULL;


/********************************************************************************
 *** Functions
 *******************************************************************************/

extern void main_f_ConfigTimers_v( void );
extern void IRAM_ATTR main_f_1msCallback_v( void );
extern void IRAM_ATTR main_f_10msCallback_v( void );

#endif /* MAIN_I_H */