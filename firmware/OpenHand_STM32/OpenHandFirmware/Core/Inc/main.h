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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_01_EN_Pin GPIO_PIN_5
#define MOTOR_01_EN_GPIO_Port GPIOC
#define MOTOR_02_EN_Pin GPIO_PIN_1
#define MOTOR_02_EN_GPIO_Port GPIOB
#define MOTOR_03_EN_Pin GPIO_PIN_10
#define MOTOR_03_EN_GPIO_Port GPIOB
#define MOTOR_01_DIR_Pin GPIO_PIN_7
#define MOTOR_01_DIR_GPIO_Port GPIOC
#define MOTOR_01_PWM_Pin GPIO_PIN_8
#define MOTOR_01_PWM_GPIO_Port GPIOC
#define MOTOR_02_DIR_Pin GPIO_PIN_9
#define MOTOR_02_DIR_GPIO_Port GPIOC
#define MOTOR_02_PWM_Pin GPIO_PIN_8
#define MOTOR_02_PWM_GPIO_Port GPIOA
#define MOTOR_03_DIR_Pin GPIO_PIN_9
#define MOTOR_03_DIR_GPIO_Port GPIOA
#define MOTOR_03_PWM_Pin GPIO_PIN_10
#define MOTOR_03_PWM_GPIO_Port GPIOA
#define LED_01_Pin GPIO_PIN_5
#define LED_01_GPIO_Port GPIOB
#define LED_02_Pin GPIO_PIN_6
#define LED_02_GPIO_Port GPIOB
#define BTN_Pin GPIO_PIN_7
#define BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
