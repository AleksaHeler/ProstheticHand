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

/* Include own header first! */
#include "main.h"


/* Include all drivers/software components... */
#include "drivers/btn/btn.h"
#include "drivers/pot/pot.h"
#include "drivers/srv/srv.h"


/********************************************************************************
 *** Functions
 *******************************************************************************/

/** @brief Init function called once on boot 
 *
 *  This function only executes once and is as such used to set up the 
 *  internal 'os' and then calls initializations of all other components.
 *
 *  @return void
 */
void setup() 
{
  /* Internal setup first... */
  Serial.begin(9600);

  /* Call all the initialization functions */
  btn_Init_v();
  pot_Init_v();
  srv_Init_v();

}

/** @brief Handle function called in loop forever
 *
 *  This function checks the timers and keeps track of timing contaiers (for example 1ms, 10ms..) 
 *  Once the time is right, it then calls functions that handle these containers 
 *  (which then handle software components)
 * 
 *  @return void
 */
void loop() 
{
  /* Call all the handle functions */
  btn_Handle_v();
  pot_Handle_v();
  srv_Handle_v();

  /* For debug only */
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  /* Wait */
  delay(50);
}
