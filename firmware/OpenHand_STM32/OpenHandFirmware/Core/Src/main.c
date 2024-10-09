/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led/led_e.h"
#include "btn/btn_e.h"
// #include "motor_e.h"...
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* Definitions for hand_control */
osThreadId_t hand_controlHandle;
const osThreadAttr_t hand_control_attributes = {
  .name = "hand_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for system_health */
osThreadId_t system_healthHandle;
const osThreadAttr_t system_health_attributes = {
  .name = "system_health",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

// ADC stuff
volatile uint16_t adcResultsDMA[64][8]; // 64 measurements per channel, 8 channels
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(uint16_t);
volatile int adcConversionComplete = 0; // Set by callback

// Mode handling stuff
uint32_t my_mode = 0; // 0 for button activation, 1 for EMG sensor activation

// Our ADC inputs
uint32_t my_pot_1 = 0;
uint32_t my_pot_2 = 0;
uint32_t my_emg_1 = 0;
uint32_t my_emg_2 = 0;
uint32_t my_batt = 0;
uint32_t my_hall_1 = 0;
uint32_t my_hall_2 = 0;
uint32_t my_hall_3 = 0;

double my_batt_volt = 0;
uint32_t my_emg_1_max = 0;

// Finger hall sensor calibration output:
uint16_t my_calibration[] = {
  2024, 1889,
  2026, 1888,
  2024, 1890,
  2024, 1887,
  2023, 1883,
  2022, 1878,
  2021, 1875,
  2019, 1868,
  2019, 1863,
  2019, 1857,
  2017, 1850,
  2016, 1840,
  2015, 1830,
  2014, 1818,
  2012, 1806,
  2010, 1794,
  2008, 1783,
  2007, 1768,
  2006, 1755,
  2005, 1739,
  2001, 1724,
  2000, 1708,
  1996, 1690,
  1995, 1675,
  1994, 1655,
  1992, 1635,
  1989, 1614,
  1987, 1592,
  1984, 1574,
  1982, 1550,
  1978, 1525,
  1975, 1502,
  1974, 1479,
  1971, 1453,
  1968, 1429,
  1966, 1407,
  1962, 1379,
  1959, 1354,
  1955, 1328,
  1952, 1307,
  1948, 1288,
  1945, 1269,
  1939, 1256,
  1936, 1248,
  1932, 1245,
  1928, 1246,
  1922, 1252,
  1918, 1266,
  1913, 1288,
  1910, 1317,
  1903, 1351,
  1898, 1391,
  1893, 1442,
  1886, 1498,
  1880, 1562,
  1873, 1627,
  1867, 1698,
  1861, 1776,
  1854, 1859,
  1846, 1937,
  1838, 2021,
  1829, 2104,
  1820, 2180,
  1812, 2254,
  1804, 2326,
  1792, 2395,
  1783, 2452,
  1773, 2505,
  1762, 2554,
  1751, 2596,
  1741, 2629,
  1725, 2658,
  1713, 2682,
  1700, 2695,
  1686, 2704,
  1671, 2711,
  1654, 2709,
  1639, 2706,
  1620, 2699,
  1604, 2688,
  1582, 2675,
  1562, 2658,
  1540, 2643,
  1520, 2624,
  1497, 2604,
  1471, 2587,
  1447, 2570,
  1423, 2549,
  1398, 2530,
  1373, 2513,
  1348, 2495,
  1318, 2475,
  1294, 2458,
  1270, 2440,
  1247, 2426,
  1224, 2409,
  1205, 2395,
  1192, 2380,
  1179, 2367,
  1172, 2355,
  1169, 2340,
  1171, 2326,
  1179, 2314,
  1199, 2300,
  1230, 2289,
  1268, 2277,
  1325, 2268,
  1394, 2257,
  1472, 2247,
  1552, 2238,
  1646, 2229,
  1749, 2222,
  1859, 2213,
  1963, 2206,
  2078, 2196,
  2190, 2190,
  2303, 2183,
  2405, 2179,
  2517, 2171,
  2626, 2165,
  2715, 2159,
  2806, 2153,
  2883, 2146,
  2948, 2142,
  2990, 2137,
  3019, 2133,
  3032, 2127,
  3037, 2123,
  3038, 2118,
  3034, 2116,
  3025, 2111,
  3010, 2106,
  2988, 2103,
  2958, 2100,
  2921, 2097,
  2883, 2094,
  2840, 2092,
  2800, 2088,
  2760, 2084,
  2717, 2083,
  2678, 2080,
  2639, 2076,
  2599, 2075,
  2565, 2072,
  2533, 2071,
  2501, 2067,
  2470, 2064,
  2446, 2063,
  2420, 2061,
  2397, 2061,
  2373, 2059,
  2354, 2055,
  2332, 2054,
  2313, 2052,
  2296, 2052,
  2279, 2049,
  2266, 2048,
  2253, 2048,
  2247, 2046,
  2245, 2046,
  2246, 2046,
  2247, 2046,
  2250, 2046,
  2253, 2046,
  2257, 2048,
  2259, 2047,
  2260, 2048,
  2261, 2048,
  2261, 2047
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void hand_control_function(void *argument);
void system_health_function(void *argument);

/* USER CODE BEGIN PFP */
void main_f_Init_v(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 *  @todo Description
 *  @note Called once during boot
 */
void main_f_Init_v(void)
{
  /* Start all PWM channels */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  /* Motor 1 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* Motor 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  /* Motor 3 */
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Read user button, if it's high, go to EMG mode
//  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
//  {
//    my_mode = 1;
//  }
//  else
//  {
//    my_mode = 0;
//  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* https://deepbluembedded.com/stm32-timer-encoder-mode-stm32-rotary-encoder-interfacing/ */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  main_f_Init_v();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of hand_control */
  hand_controlHandle = osThreadNew(hand_control_function, NULL, &hand_control_attributes);

  /* creation of system_health */
  system_healthHandle = osThreadNew(system_health_function, NULL, &system_health_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR_01_EN_Pin|MOTOR_01_DIR_Pin|MOTOR_02_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_02_EN_Pin|MOTOR_03_EN_Pin|LED_01_Pin|LED_02_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_03_DIR_GPIO_Port, MOTOR_03_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_01_EN_Pin MOTOR_01_DIR_Pin MOTOR_02_DIR_Pin */
  GPIO_InitStruct.Pin = MOTOR_01_EN_Pin|MOTOR_01_DIR_Pin|MOTOR_02_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_02_EN_Pin MOTOR_03_EN_Pin LED_01_Pin LED_02_Pin */
  GPIO_InitStruct.Pin = MOTOR_02_EN_Pin|MOTOR_03_EN_Pin|LED_01_Pin|LED_02_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_03_DIR_Pin */
  GPIO_InitStruct.Pin = MOTOR_03_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_03_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Called when buffer is completely filled
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//  adcConversionComplete = 1;
//}

uint32_t lastTime;
float Input, Output, Setpoint = 0;
float errSum, lastErr;
float kp = 0.2, ki = 0.00, kd = 0.001;

void handle_motor_pid()
{
  /* How long since we last calculated */
  uint32_t now = HAL_GetTick();
  float timeChange = (float)(now - lastTime);

  /* Compute all the working error variables */
  float error = Setpoint - Input;
  errSum += (error * timeChange);
  float dErr = (error - lastErr) / timeChange;

  /*Compute PID Output*/
  Output = kp * error + ki * errSum + kd * dErr;

  /*Remember some variables for next time*/
  lastErr = error;
  lastTime = now;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_hand_control_function */
/**
  * @brief  Function implementing the hand_control thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_hand_control_function */
void hand_control_function(void *argument)
{
  /* USER CODE BEGIN 5 */
  // char my_data[24];
  uint32_t last_10ms = 0;
  //uint32_t i = 0;
  //static uint8_t my_flag = 0;
  /* Infinite loop */
  for(;;)
  {
    /* Here we sould only go over all the driver handle functions... */
    // btn_f_Handle_v();
    // pot_f_Handle_v();
    // emg_f_Handle_v();
    // hall_f_Handle_v();
    // batt_f_Handle_v();
    // motor_f_Handle_v();

    /* All of this should be moved to separate drivers... */
    /* Firstly, ignore everything unless 10ms has passed, and it's our time to work */
    if(HAL_GetTick() - last_10ms > (10-1))
    {
      last_10ms = HAL_GetTick();

      ////////////////////////////////////////////////////////////////////////
      /// Initial handling of ADC inputs /////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////

      // Start ADC reading of all channels, to be stored via DMA - check if it should be done continuously
//      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, adcChannelCount);
      // Wait for that to finish - not necessary I think
      // while(adcConversionComplete == 0) {}
      // adcConversionComplete = 0;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);  // Motor 1 EN
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  // Motor 2 EN
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // Motor 3 EN

      // Store all ADC results in global vars
//      my_pot_1 = adcResultsDMA[0][0];
//      my_pot_2 = adcResultsDMA[0][1];
//      my_emg_1 = adcResultsDMA[0][2];
//      my_emg_2 = adcResultsDMA[0][3];
//      my_batt = adcResultsDMA[0][4];
//      my_hall_1 = adcResultsDMA[0][5];
//      my_hall_2 = adcResultsDMA[0][6];
//      my_hall_3 = adcResultsDMA[0][7];

//      my_batt_volt = ( ( (my_batt / 4096.0) * 3.3 ) * 5.7 ); // 47k and 10k voltage divider -> 5.7 factor

      // Find max of EMG signal
//      my_emg_1_max = 0;
//      for(i = 0; i < 64; i++)
//      {
//        if (adcResultsDMA[i][3] > my_emg_1_max)
//        {
//          my_emg_1_max = adcResultsDMA[i][3];
//        }
//      }
//      my_emg_1 = my_emg_1_max;

      ////////////////////////////////////////////////////////////////////////
      /// Move motors based on button ////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////


      /* If BTN pressed */
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
      {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); /* LED_01 */

        // Close hand
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);  // Motor 1 DIR
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);  // Motor 2 DIR
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);  // Motor 3 DIR
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

        TIM1->CCR1 = (uint16_t)0;  // Motor 2 PWM
        TIM1->CCR3 = (uint16_t)0;  // Motor 3 PWM
        TIM3->CCR3 = (uint16_t)0;  // Motor 1 PWM
        //TIM1->CCR2 = (uint16_t)my_pwm;
        //TIM3->CCR1 = (uint16_t)my_pwm;
        // Also, log current hand state
        //sprintf(my_data, "1\n");
        //CDC_Transmit_FS(my_data, strlen(my_data));
      }
      /* BTN released */
      else
      {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); /* LED_01 */

        // OpenHand™
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);  // Motor 1 DIR
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // Motor 2 DIR
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  // Motor 3 DIR
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

        TIM1->CCR1 = (uint16_t)19;  // Motor 2 PWM
        TIM1->CCR3 = (uint16_t)19;  // Motor 3 PWM
        TIM3->CCR3 = (uint16_t)19;  // Motor 1 PWM
        //TIM1->CCR2 = (uint16_t)my_pwm;
        //TIM3->CCR1 = (uint16_t)my_pwm;

        // Also, log current hand state
        //sprintf(my_data, "0\n");
        //CDC_Transmit_FS(my_data, strlen(my_data));
      }

      ////////////////////////////////////////////////////////////////////////
      /// Calibration procedure //////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////

      // my_hall = adcResultsDMA[0][5];
      // my_hall_2 = adcResultsDMA[0][6];

      // // If button is pressed, start calibration sequence
      // if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
      // {
      //   my_flag = 1;
      // }

      // if(my_flag == 0)
      // {
      //   // Log ready message
      //   sprintf(my_data, "Ready!\n");
      //   CDC_Transmit_FS(my_data, strlen(my_data));
      //   // Generally, first move all motors to open position
      //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
      //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
      //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
      //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);  // Motor 1 DIR
      //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
      //   my_pot = 1;
      //   my_pwm = 100.0 - (double)my_pot / 40.96;
      //   TIM1->CCR1 = (uint16_t)my_pwm;
      //   TIM1->CCR2 = (uint16_t)my_pwm;
      //   TIM1->CCR3 = (uint16_t)my_pwm;
      //   TIM3->CCR1 = (uint16_t)my_pwm;
      //   TIM3->CCR3 = (uint16_t)my_pwm;  // Motor 1 PWM
      // }
      // else
      // {
      //   // Log hall sensors...
      //   sprintf(my_data, "%d, %d\n", my_hall, my_hall_2);
      //   CDC_Transmit_FS(my_data, strlen(my_data));
      //   // ... and start moving motor to closed position
      //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
      //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
      //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
      //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);  // Motor 1 DIR
      //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
      //   my_pot = 1;
      //   my_pwm = (double)my_pot / 40.96;
      //   TIM1->CCR1 = (uint16_t)my_pwm;
      //   TIM1->CCR2 = (uint16_t)my_pwm;
      //   TIM1->CCR3 = (uint16_t)my_pwm;
      //   TIM3->CCR1 = (uint16_t)my_pwm;
      //   TIM3->CCR3 = (uint16_t)my_pwm;  // Motor 1 PWM
      // }

      ////////////////////////////////////////////////////////////////////////
      ///  End of main task //////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////
    }

    /* Wait 1ms to let other tasks do their thing! And for time to pass quicker while we wait for our 10ms! */
    osDelay(1);
  }
  /* In case we exit the infinite loop, terminate cleanly */
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_system_health_function */
/**
* @brief Function implementing the system_health thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_system_health_function */
void system_health_function(void *argument)
{
  /* USER CODE BEGIN system_health_function */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6); /* LED_02 */
    osDelay(1000);
  }
  /* USER CODE END system_health_function */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
