/**
 * @file gpio_adc_mapping.h
 * 
 * @author your name (you@domain.com)
 * 
 * @brief Header file that contains mappings of ESP32's GPIO pins to its correspondent ADC channels
 * 
 * @version 0.1
 * @date 2023-09-19
 * 
 * @bug The mappings are unfinished, only containing the ones used in the ProstheticHand project so far
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef GPIO_ADC_MAPPING_H
#define GPIO_ADC_MAPPING_H

#include "driver/adc.h"

#define GPIO_PIN_TO_ADC_CHANNEL(x) \
    ( \
        (x == GPIO_NUM_25) ? ADC2_CHANNEL_8 : \
        (x == GPIO_NUM_26) ? ADC2_CHANNEL_9 : \
        (x == GPIO_NUM_27) ? ADC2_CHANNEL_7 : \
        ADC2_CHANNEL_MAX \
    )

#endif // GPIO_ADC_NAPPING_H