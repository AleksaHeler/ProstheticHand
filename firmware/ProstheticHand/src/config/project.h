/**
 * @file project.h
 * 
 * @author your name (you@domain.com)
 * 
 * @brief Header file that unifies everything used in project for easier includes
 * 
 * @todo Document this file, and add everything that needs to be here 
 * @todo Milan Popđurđev: Migrate from driver/adc.h
 * 
 * @version 0.1
 * @date 2023-09-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef PROJECT_H
#define PROJECT_H

/**************************************************************************
 * Includes
 **************************************************************************/

/* ESP IDF stuff */

/* Integer types */
#include "stdint.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/adc.h" // WARNING: deprecated, migration needed

#include "include/GPIO_ADC_Mapping/gpio_adc_mapping.h"

/* All defines */
#include "defines.h"

/* Info headers */
#include "version.h"

#endif // PROJECT_H