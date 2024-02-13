/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HW_REV_1_Pin GPIO_PIN_0
#define HW_REV_1_GPIO_Port GPIOA
#define HW_REV_2_Pin GPIO_PIN_1
#define HW_REV_2_GPIO_Port GPIOA
#define HW_REV_3_Pin GPIO_PIN_2
#define HW_REV_3_GPIO_Port GPIOA
#define HW_REV_3A3_Pin GPIO_PIN_3
#define HW_REV_3A3_GPIO_Port GPIOA
#define MOTOR_1_FAULT_Pin GPIO_PIN_4
#define MOTOR_1_FAULT_GPIO_Port GPIOA
#define MOTOR_2_FAULT_Pin GPIO_PIN_5
#define MOTOR_2_FAULT_GPIO_Port GPIOA
#define MOTOR_3_FAULT_Pin GPIO_PIN_6
#define MOTOR_3_FAULT_GPIO_Port GPIOA
#define HAPTIC_FAULT_Pin GPIO_PIN_7
#define HAPTIC_FAULT_GPIO_Port GPIOA
#define ADC_1_RDY_Pin GPIO_PIN_4
#define ADC_1_RDY_GPIO_Port GPIOC
#define ADC_2_RDY_Pin GPIO_PIN_5
#define ADC_2_RDY_GPIO_Port GPIOC
#define EMG_1_UC_Pin GPIO_PIN_0
#define EMG_1_UC_GPIO_Port GPIOB
#define EMG_2_UC_Pin GPIO_PIN_1
#define EMG_2_UC_GPIO_Port GPIOB
#define MOTOR_1_EN_Pin GPIO_PIN_12
#define MOTOR_1_EN_GPIO_Port GPIOB
#define MOTOR_2_EN_Pin GPIO_PIN_13
#define MOTOR_2_EN_GPIO_Port GPIOB
#define MOTOR_3_EN_Pin GPIO_PIN_14
#define MOTOR_3_EN_GPIO_Port GPIOB
#define HAPTIC_EN_Pin GPIO_PIN_15
#define HAPTIC_EN_GPIO_Port GPIOB
#define HAPTIC_1_PWM_Pin GPIO_PIN_6
#define HAPTIC_1_PWM_GPIO_Port GPIOC
#define HAPTIC_2_PWM_Pin GPIO_PIN_7
#define HAPTIC_2_PWM_GPIO_Port GPIOC
#define HAPTIC_1_DIR_Pin GPIO_PIN_8
#define HAPTIC_1_DIR_GPIO_Port GPIOC
#define HAPTIC_2_DIR_Pin GPIO_PIN_9
#define HAPTIC_2_DIR_GPIO_Port GPIOC
#define MOTOR_1_PWM_Pin GPIO_PIN_8
#define MOTOR_1_PWM_GPIO_Port GPIOA
#define MOTOR_2_PWM_Pin GPIO_PIN_9
#define MOTOR_2_PWM_GPIO_Port GPIOA
#define MOTOR_3_PWM_Pin GPIO_PIN_10
#define MOTOR_3_PWM_GPIO_Port GPIOA
#define MOTOR_1_DIR_Pin GPIO_PIN_11
#define MOTOR_1_DIR_GPIO_Port GPIOA
#define MOTOR_2_DIR_Pin GPIO_PIN_12
#define MOTOR_2_DIR_GPIO_Port GPIOA
#define MOTOR_3_DIR_Pin GPIO_PIN_10
#define MOTOR_3_DIR_GPIO_Port GPIOC
#define DEBUG_LED_1_Pin GPIO_PIN_11
#define DEBUG_LED_1_GPIO_Port GPIOC
#define DEBUG_LED_2_Pin GPIO_PIN_12
#define DEBUG_LED_2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
