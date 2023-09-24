/**
 * @file main.cpp
 * 
 * @author your name (you@domain.com)
 * 
 * @brief Main entry point of the Prosthetic Hand project's code
 * 
 * This is the main file and starting point of the prosthetic hand project.
 * Here we have two functions: setup (called once on boot) and loop (called in loop forever)
 * In 'setup' we firstly initialize all modules (software components), and then in 'loop'
 * we constantly call each handle function in constrained timing containers (1ms, 10ms etc.)
 * 
 * @version 0.1
 * @date 2023-09-21
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
#include "drivers/srv/srv_e.h"

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
 * Buffer for runtime measurement statistics
 * 
 * @values see main_g_RuntimeMeasTyp_t define in main_e.h
 */
main_g_RuntimeMeasTyp_t main_g_RuntimeMeas_s[MAIN_CYCLE_TASK_COUNT];

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

/**************************************************************************
 * Application entry point
 **************************************************************************/

extern "C" void app_main(void)
{
  ESP_LOGI(MAIN_TAG, "Application start");
    main_f_Init_v();
    while(true){
      main_f_Handle_v();
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
    
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
    for(i = 0; i < MAIN_CYCLE_TASK_COUNT; i++) {
        main_g_RuntimeMeas_s[i].currentCycle_u32 = 0;
        main_g_RuntimeMeas_s[i].maxCycle_u32 = 0;
        main_g_RuntimeMeas_s[i].minCycle_u32 = 0;
    }

    /* Call all the initialization functions */
    btn_Init_v();
    pot_f_Init_v();
    srv_Init_v();


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

    if(main_g_CurrMicros_u64 - main_g_LastMicros_u64 >= main_g_CycleTaskLengthUs_u16){
        /* Keep track of the last task time */
        main_g_LastMicros_u64 = main_g_CurrMicros_u64;

        /* Start of runtime measurement */
        l_rtmMeas_u32 = main_f_StartRTM_v();

        /* Call the right handle functions for this task */
        switch(main_g_CurrTaskIndex_u16){
            case 0:
                btn_Handle_v();
                break;
            case 1:
                pot_f_Handle_v();
                break;
            case 2:
                srv_Handle_v();
                break;
            case 3:
                /* To be populated*/
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
        if(main_g_CurrTaskIndex_u16 >= MAIN_CYCLE_TASK_COUNT){
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

#ifdef SERIAL_DEBUG
/** @brief Write runtime data to Serial console and call right functions in components to do the same 
 */
void main_f_SerialDebug_v( void *arg ) {
  uint16_t i; 

  /* TODO: maybe make this output data as a JSON to Serial */
  while(1){
    ESP_LOGD(MAIN_TAG, "----------------------------------------");

    ESP_LOGD(MAIN_TAG, " > runtimes (in microseconds):");
    for(i = 0; i < MAIN_CYCLE_TASK_COUNT; i++) {
        ESP_LOGD(MAIN_TAG, "    ├─[%u] curr: %lu, min: %lu, max: %lu", i, main_g_RuntimeMeas_s[i].currentCycle_u32, main_g_RuntimeMeas_s[i].minCycle_u32, main_g_RuntimeMeas_s[i].maxCycle_u32);
    }

    btn_SerialDebug_v();
    pot_f_SerialDebug_v();
    srv_SerialDebug_v();

    vTaskDelay(MAIN_SERIAL_DEBUG_DELAY);
  }
}
#endif
 