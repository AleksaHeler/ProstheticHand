/** @file srv.cpp
 *  @brief Servo software component / driver
 *
 *  Here we have handling of the servos connected to the board.
 *  For now only couple of servos are connected for debugging, 
 *  and we just set their angle to the potentiometer value.
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */


/********************************************************************************
 *** Includes
 *******************************************************************************/

/* Own header file */
#include "srv_e.h"

/* Other components used here */
#include "drivers/pot/pot_e.h"

/********************************************************************************
 *** Global variables
 *******************************************************************************/

Servo srv_Servo1_s;
Servo srv_Servo2_s;

/********************************************************************************
 *** Functions
 *******************************************************************************/

void srv_Init_v( void );
void srv_Handle_v( void );


/** @brief Init function called once on boot 
 *
 *  Set all the given pins as inputs
 *
 *  @return void
 */
void srv_Init_v( void )
{
  #ifdef SERIAL_DEBUG
  Serial.println("SRV: init");
  #endif

  pinMode(SERVO_PIN0, OUTPUT);
  pinMode(SERVO_PIN1, OUTPUT);

  srv_Servo1_s.attach(SERVO_PIN0, SERVO_MIN_ANGLE_US, SERVO_MAX_ANGLE_US);
  srv_Servo2_s.attach(SERVO_PIN1, SERVO_MIN_ANGLE_US, SERVO_MAX_ANGLE_US);
}


/** @brief Handle function to be called cyclically
 *
 *  Set the potentiometers to an angle that relates to a potentiometer
 *
 *  @return void
 */
void srv_Handle_v( void )
{
  #ifdef SERIAL_DEBUG
  Serial.println("SRV: handle");
  #endif

  uint16_t angle = (uint16_t)pot_g_PotValues_f32[SERVO_CONTROL_POT_INDEX];

  srv_Servo1_s.write(angle);
  srv_Servo2_s.write(angle);
}
