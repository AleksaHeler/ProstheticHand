/**
 * @file main.cpp
 * 
 * @author Aleksa Heler (aleksaheler@gmail.com)
 * 
 * @brief Main entry point of the Prosthetic Hand project's code
 * 
 * This is the main file and starting point of the prosthetic hand project.
 * Here we have two functions: setup (called once on boot) and loop (called in loop forever)
 * In 'setup' we firstly initialize all modules (software components), and then in 'loop'
 * we constantly call each ` function in constrained timing containers (1ms, 10ms etc.)
 * 
 * @version 0.1
 * @date 2023-09-21
 * 
 * @bug Pot and Sensor modules cannot be called together as they initialise the same ADC unit.
 * The code needs to be restructured so that the ADC unit initialisation is done directly in main.cpp
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/**************************************************************************
 * Includes
 **************************************************************************/

/* Include own headers first! */
#include "main_i.h"
#include "main_e.h"


/* Include all drivers/software components... */
#include "drivers/btn/btn_e.h"
#include "drivers/pot/pot_e.h"
#include "drivers/sensor/sensor_e.h"

/**************************************************************************
 * Global variables
 **************************************************************************/

/**
 * Keep track of current time in microseconds and the last time a task was performed
 * 
 * @values 0..UINT64_MAX
 */
uint64_t main_g_CurrMicros_u64 = 0;
uint64_t main_g_LastMicros_u64 = 0;

/**
 * Keep track of current task index
 * 
 * @values 0..MAIN_CYCLE_TASK_COUNT
 */
uint16_t main_g_CurrTaskIndex_u16 = 0;

/**
 * Counter to know when to turn on/off the debug LED
 * 
 * @values 0..MAIN_DEBUG_LED_CYCLE_COUNT
 */
uint16_t main_g_DebugLEDCountdown_u16 = 0;

/**
 * Buffer for runtime measurement statistics
 * 
 * @values see main_g_RuntimeMeasTyp_t define in main_e.h
 */
main_g_RuntimeMeasTyp_t main_g_RuntimeMeas_s[main_c_CycleTaskCount_u8];

#ifdef SERIAL_DEBUG
/**
 * @brief Handle for task running in parellel to the main OS for writing debug info
 * 
 */
TaskHandle_t main_g_SerialDebugTaskHandle_s = NULL;
#endif

/**************************************************************************
 * Functions
 **************************************************************************/

void main_f_Init_v(void);
void main_f_Handle_v(void);

#ifdef SERIAL_DEBUG
void main_f_SerialDebug_v(void *arg);
#endif

uint32_t main_f_StartRTM_v(void);
uint32_t main_f_StopRTM_v(uint32_t rtmStart);
void main_f_HandleRTMStats_v(uint16_t index);
void main_f_DebugLEDInit_v(void);
void main_f_DebugLEDHandle_v(void);

/**************************************************************************
 * Application entry point
 **************************************************************************/

extern "C" void app_main(void)
{
    ESP_LOGI(MAIN_TAG, "Application start");

    main_f_Init_v();

    while(true){
        main_f_Handle_v();
    }

    //pot_f_Deinit_v();
    //btn_f_Deinit_v();
    //sensor_f_Deinit_v();
}



/** @brief Init function called once on boot 
 *
 * This function only executes once and is as such used to set up the 
 * internal 'os' and then calls initializations of all other components.
 *
 * @return void
 */
void main_f_Init_v(void)
{
    uint16_t i;

    /* Prepare runtime measurement buffer */
    for(i = 0; i < main_c_CycleTaskCount_u8; i++) {
        main_g_RuntimeMeas_s[i].currentCycle_u32 = 0;
        main_g_RuntimeMeas_s[i].maxCycle_u32 = 0;
        main_g_RuntimeMeas_s[i].minCycle_u32 = 0;
    }

    /* Call all the initialization functions */
    main_f_DebugLEDInit_v();
    //btn_f_Init_v();
    //pot_f_Init_v();
    //sensor_f_Init_v();


    #ifdef SERIAL_DEBUG
    /* Create a separate parallel task on the other core (0) to run Serial debug interface (rest of the code runs on core 1) */
    xTaskCreatePinnedToCore(main_f_SerialDebug_v, "main_f_SerialDebug_v", 4096, NULL, 10, &main_g_SerialDebugTaskHandle_s, 0);
    #endif
}

/**
 * @brief Handle function called continuously in a loop
 * 
 * This function calls handle functions of all the other components
 * 
 */
void main_f_Handle_v(void)
{
  uint32_t l_rtmMeas_u32;

  /* Get current time */
  main_g_CurrMicros_u64 = esp_timer_get_time();

  if(main_g_CurrMicros_u64 - main_g_LastMicros_u64 >= main_c_CycleTaskLengthUs_u16){
    /* Keep track of the last task time */
    main_g_LastMicros_u64 = main_g_CurrMicros_u64;

    /* Start of runtime measurement */
    l_rtmMeas_u32 = main_f_StartRTM_v();

    /* Call the right handle functions for this task */
    switch(main_g_CurrTaskIndex_u16){
      case 0:
          main_f_DebugLEDHandle_v();
          break;
      case 1:
          //btn_f_Handle_v();
          break;
      case 2:
          //pot_f_Handle_v();
          break;
      case 3:
          //sensor_f_Handle_v();
          break;
      case 4:
          /* To be populated*/
          break;
      case 5:
          /* To be populated*/
          break;
      case 6:
          /* To be populated*/
          break;
      case 7:
          /* To be populated*/
          break;
      case 8:
          /* To be populated*/
          break;
      case 9:
          /* To be populated*/
          break;
      default:
          /* This should not happen */
          break;
    }

    /* Calculate current task execution time */
    main_g_RuntimeMeas_s[main_g_CurrTaskIndex_u16].currentCycle_u32 = main_f_StopRTM_v(l_rtmMeas_u32);

    /* Calculate the rest of the statistics for runtime measurement (min/max) */
    main_f_HandleRTMStats_v(main_g_CurrTaskIndex_u16);

    /* Keep track of which task we're in */
    main_g_CurrTaskIndex_u16++;
    if(main_g_CurrTaskIndex_u16 >= main_c_CycleTaskCount_u8){
        main_g_CurrTaskIndex_u16 = 0;
    }
  }
}

/**************************************************************************
 * Runtime measurements **************************************************************************/

/**
 * @brief Gets current time in microseconds
 * 
 */
uint32_t main_f_StartRTM_v(void)
{
    return esp_timer_get_time();
}

/**
 * @brief Returns time passed since the given parameter rtmStart
 * 
 */
uint32_t main_f_StopRTM_v(uint32_t rtmStart)
{
    return esp_timer_get_time() - rtmStart;
}

/**
 * @brief Calculate min/max values for RTM measurement for given task index
 * 
 */
void main_f_HandleRTMStats_v(uint16_t index)
{
    /* Keep track of max execution time */
    if(main_g_RuntimeMeas_s[index].currentCycle_u32 > main_g_RuntimeMeas_s[index].maxCycle_u32){
        main_g_RuntimeMeas_s[index].maxCycle_u32 = main_g_RuntimeMeas_s[index].currentCycle_u32;
    }

    /* Keep track of min execution time */
    if(main_g_RuntimeMeas_s[index].currentCycle_u32 < main_g_RuntimeMeas_s[index].minCycle_u32){
        main_g_RuntimeMeas_s[index].minCycle_u32 = main_g_RuntimeMeas_s[index].currentCycle_u32;
    }

    /* In case the execution time is 0 (should happen only on startup) */
    if(main_g_RuntimeMeas_s[index].minCycle_u32 == 0){
        main_g_RuntimeMeas_s[index].minCycle_u32 = main_g_RuntimeMeas_s[index].currentCycle_u32;
    }
}

/** @brief Configures the LED debug output pin accordingly
 */
void main_f_DebugLEDInit_v(void)
{
  /* Output pin without any pullup/pulldown */
  gpio_config_t dbg_pin_config{
    (1ULL << MAIN_DEBUG_LED_PIN),
    GPIO_MODE_INPUT_OUTPUT,
    GPIO_PULLUP_DISABLE,
    GPIO_PULLDOWN_DISABLE,
    GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&dbg_pin_config));
}


/** @brief Turns on/off debug LED when needed (cyclically)
 */
void main_f_DebugLEDHandle_v(void)
{
  /* If debug LED cycle counter greater than 0, decrement it */
  if(main_g_DebugLEDCountdown_u16 > 0){
    main_g_DebugLEDCountdown_u16--;
  }
  /* If countdown done: set LED to inverse of itself and reset the counter! */
  else{
    main_g_DebugLEDCountdown_u16 = MAIN_DEBUG_LED_CYCLE_COUNT;
    ESP_ERROR_CHECK(gpio_set_level(MAIN_DEBUG_LED_PIN, !gpio_get_level(MAIN_DEBUG_LED_PIN)));
    ESP_LOGI(MAIN_TAG, "LED level: %d", gpio_get_level(MAIN_DEBUG_LED_PIN));
  }
}

#ifdef SERIAL_DEBUG
/** @brief Write runtime data to Serial console and call right functions in components to do the same 
 */
void main_f_SerialDebug_v( void *arg ) {
  uint16_t i; 

  /* TODO: maybe make this output data as a JSON to Serial */
  while(true){
    ESP_LOGD(MAIN_TAG, "----------------------------------------");

    ESP_LOGD(MAIN_TAG, " > runtimes (in microseconds):");
    for(i = 0; i < MAIN_CYCLE_TASK_COUNT; i++) {
        ESP_LOGD(MAIN_TAG, "    ├─[%u] curr: %lu, min: %lu, max: %lu", i, main_g_RuntimeMeas_s[i].currentCycle_u32, main_g_RuntimeMeas_s[i].minCycle_u32, main_g_RuntimeMeas_s[i].maxCycle_u32);
    }

    /* Call all module debug functions! */
    //btn_f_SerialDebug_v();
    //pot_f_SerialDebug_v();
    //sensor_f_SerialDebug_v();

    vTaskDelay(MAIN_SERIAL_DEBUG_DELAY);
  }
}
#endif
