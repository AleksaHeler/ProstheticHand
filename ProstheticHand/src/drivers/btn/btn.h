/** @file btn.h
 *  @brief Header file for the corresponding btn.cpp
 *
 *  @todo Aleksa Heler: add comment for this file (what is located here?)
 *  @todo Aleksa Heler: add structure that defines button input, so that it's modular
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */
#ifndef BTN_H
#define BTN_H


/********************************************************************************
 *** Includes
 *******************************************************************************/

#include "config/project.h"


/********************************************************************************
 *** Defines
 *******************************************************************************/

/**
 * Buffer for storing if a button is pressed
 *
 * @values 0..1 (HIGH/LOW, TRUE/FALSE...)
 */
#define BTN_PIN0 34
#define BTN_PIN1 35
#define BTN_PIN2 36 // VP
#define BTN_PIN3 39 // VN


/********************************************************************************
 *** Global variables
 *******************************************************************************/

/**
 * Buffer for storing if a button is pressed
 *
 * @values 0..1 (HIGH/LOW, TRUE/FALSE...)
 */
extern uint8_t btn_BtnStates_u8[4];


/********************************************************************************
 *** Function prototypes
 *******************************************************************************/

extern void btn_Init_v( void );
extern void btn_Handle_v( void );

#endif /* BTN_H */