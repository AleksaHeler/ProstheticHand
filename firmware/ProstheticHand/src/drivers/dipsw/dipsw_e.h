/**
 * @file dipsw_e.h
 * 
 * @author Milan Popđurđev (m.popdjurdjev@gmail.com)
 * 
 * @brief Header file containing DIP switch configurations for the development board
 * 
 * This file contains everything needed by other modules in order to use this module

 * 
 * @version 0.1
 * @date 2023-10-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef DIPSW_E_H
#define DIPSW_E_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include "config/project.h"

/**************************************************************************
 * Defines
 **************************************************************************/

#define DIP_0 GPIO_NUM_42
#define DIP_1 GPIO_NUM_41
#define DIP_2 GPIO_NUM_40
#define DIP_3 GPIO_NUM_39

/**************************************************************************
 * Function prototypes
 **************************************************************************/

extern void dipsw_f_Init_v(void);

#endif // DIPSW_E_H