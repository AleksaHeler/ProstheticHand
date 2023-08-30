/** @file btn.cpp
 *  @brief Button software component / driver
 *
 *  Here we have handling of the buttons connected to the board.
 *  For now only couple of buttons are connected, and we just read their values
 *  and store them in local buffers for easier later use.
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */


/********************************************************************************
 *** Includes
 *******************************************************************************/

/* Own header file */
#include "btn_e.h"


/********************************************************************************
 *** Global variables
 *******************************************************************************/

/**
 * Buffer for storing if a button is pressed
 *
 * @values 0..1 (HIGH/LOW, TRUE/FALSE...)
 */
uint8_t btn_BtnStates_u8[4];

/********************************************************************************
 *** Functions
 *******************************************************************************/

void btn_Init_v( void );
void btn_Handle_v( void );

#ifdef SERIAL_DEBUG
void btn_SerialDebug_v( void );
#endif


/** @brief Initialize function to be called once on startup/boot
 *
 *  Set all the given pins as inputs
 *
 *  @return void
 */
void btn_Init_v( void )
{
  pinMode(BTN_PIN0, INPUT);
  pinMode(BTN_PIN1, INPUT);
  pinMode(BTN_PIN2, INPUT);
  pinMode(BTN_PIN3, INPUT);
}


/** @brief Handle function to be called cyclically
 *
 *  Read all the defined buttons and store them in internall buffer
 *
 *  @return void
 */
void btn_Handle_v( void )
{
  btn_BtnStates_u8[0] = !digitalRead(BTN_PIN0);
  btn_BtnStates_u8[1] = !digitalRead(BTN_PIN1);
  btn_BtnStates_u8[2] = !digitalRead(BTN_PIN2);
  btn_BtnStates_u8[3] = !digitalRead(BTN_PIN3);
}


#ifdef SERIAL_DEBUG
void btn_SerialDebug_v( void ) {
  uint16_t i;

  Serial.print("btns:  ");
  for(i = 0; i < 4; i++) {
    Serial.print(btn_BtnStates_u8[i]);
    Serial.print("  ");
  }
  Serial.println(" ");
}
#endif