/** @file main.cpp
 *  @brief Main entry point of the prosthetic hand code
 *
 *  This is the main file and starting point of the prosthetic hand project.
 *  Here we have two functions: setup (called once on boot) and loop (called in loop forever)
 *  In 'setup' we firstly initialize all modules (software components), and then in 'loop'
 *  we constantly call each handle function in constrained timing containers (1ms, 10ms etc.)
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */


/********************************************************************************
 *** Includes
 *******************************************************************************/

/* Include own headers first! */
#include "main_i.h"
#include "main_e.h"


/* Include all drivers/software components... */
#include "drivers/btn/btn_e.h"
#include "drivers/pot/pot_e.h"
#include "drivers/srv/srv_e.h"

/********************************************************************************
 *** Variables
 *******************************************************************************/

main_g_RuntimeMeas_t main_g_RuntimeMeas_s[MAIN_TIMER_CALLBACK_COUNT];


/********************************************************************************
 *** Functions
 *******************************************************************************/

void setup();
void loop();
void main_f_ConfigTimers_v( void );
void IRAM_ATTR main_f_1msCallback_v( void );
void IRAM_ATTR main_f_10msCallback_v( void );


/** @brief Function to configure the two timers of our project (1 and 10 ms)
 *
 *  This function sets up the 1ms and 10ms timer to call the right 
 *  functions once they count over. 
 *
 *  @return void
 */
void main_f_ConfigTimers_v( void ){
  /* Configure timer 0, with prescaler 80 (so we can use microseconds later), to count up (true) */
  main_g_1msTimer_s = timerBegin(0, 80, true);
  /* Attach callback function to the timer */
  timerAttachInterrupt(main_g_1msTimer_s, &main_f_1msCallback_v, true);
  /* Configure timer to trigger on 1000 microseconds (or 1ms) */
  timerAlarmWrite(main_g_1msTimer_s, 1000, true);
  /* Finally enable it! */
  timerAlarmEnable(main_g_1msTimer_s);
  
  /* Configure timer 1, with prescaler 80 (so we can use microseconds later), to count up (true) */
  main_g_10msTimer_s = timerBegin(0, 80, true);
  /* Attach callback function to the timer */
  timerAttachInterrupt(main_g_10msTimer_s, &main_f_10msCallback_v, true);
  /* Configure timer to trigger on 10000 microseconds (or 10ms) */
  timerAlarmWrite(main_g_10msTimer_s, 10000, true);
  /* Finally enable it! */
  timerAlarmEnable(main_g_10msTimer_s);
}

/** @brief Init function called once on boot 
 *
 *  This function only executes once and is as such used to set up the 
 *  internal 'os' and then calls initializations of all other components.
 *
 *  @return void
 */
void setup() {
  /* Internal setup first... */
  #ifdef SERIAL_DEBUG
  Serial.begin(9600);
  #endif

  /* Set up the timers */
  main_f_ConfigTimers_v();

  /* Call all the initialization functions */
  btn_Init_v();
  pot_f_Init_v();
  srv_Init_v();
}

/** @brief Handle function called in loop forever
 * 
 *  @return void
 */
void loop() { 
  main_f_1msCallback_v();
  main_f_10msCallback_v();
  delay(1);
}


/** @brief Callback function for 1ms timer
 *
 *  This function is called by a timer every 1ms
 *  Once the time is right, it then calls functions that handle these containers 
 *  (which then handle software components)
 * 
 *  @return void
 */
void IRAM_ATTR main_f_1msCallback_v( void ) {
  uint32_t l_StartMicros_u32;

  l_StartMicros_u32 = micros();

  /* Call all handle/update function */
  btn_Handle_v();
  pot_f_Handle_v();
  srv_Handle_v();

  /* Store execution time and check for min/max value */
  main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].currExecTime = micros() - l_StartMicros_u32;
  if (main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].currExecTime > main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].maxExecTime){
    main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].maxExecTime = main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].currExecTime;
  }
  if (main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].currExecTime < main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].minExecTime){
    main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].minExecTime = main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].currExecTime;
  }
}

/** @brief Callback function for 10ms timer
 *
 *  This function is called by a timer every 10ms
 * 
 *  @return void
 */
void IRAM_ATTR main_f_10msCallback_v( void ) {
  uint32_t l_StartMicros_u32;

  l_StartMicros_u32 = micros();

  #ifdef SERIAL_DEBUG
  Serial.println("cycle |  MIN   |  CURR  |  MAX (us)");

  Serial.print("1ms    ");
  Serial.print(main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].minExecTime);
  Serial.print("  ");
  Serial.print(main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].currExecTime);
  Serial.print("  ");
  Serial.println(main_g_RuntimeMeas_s[MAIN_1MS_MEASURE_INDEX].maxExecTime);
  
  Serial.print("10ms   ");
  Serial.print(main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].minExecTime);
  Serial.print("  ");
  Serial.print(main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].currExecTime);
  Serial.print("  ");
  Serial.println(main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].maxExecTime);
  #endif
  
  /* Store execution time and check for min/max value */
  main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].currExecTime = micros() - l_StartMicros_u32;
  if (main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].currExecTime > main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].maxExecTime){
    main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].maxExecTime = main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].currExecTime;
  }
  if (main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].currExecTime < main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].minExecTime){
    main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].minExecTime = main_g_RuntimeMeas_s[MAIN_10MS_MEASURE_INDEX].currExecTime;
  }
}
