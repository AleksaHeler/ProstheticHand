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


/********************************************************************************
 *** Functions
 *******************************************************************************/

void setup();
void loop();


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
  /* Call all handle/update function */
  btn_Handle_v();
  pot_f_Handle_v();
  srv_Handle_v();
}
