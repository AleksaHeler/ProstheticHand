/* Basic example of FreeRTOS on ESP32, flashing LED using task instead of in the loop */

#include <Arduino.h>

#define LED_BUILTIN 2

/* Define function/task */
void myTask( void * parameter )
{
  /* loop forever */
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  /* Delete the task when finished */
  vTaskDelete( NULL );
}


void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  /* we create a new task here */
  xTaskCreate(myTask,    /* Task function. */
              "task",    /* Task name */
              10000,     /* Task stack size */
              NULL,      /* Task parameter */
              1,         /* Task priority */
              NULL);     /* Task handle to keep track of created task */
}

void loop() {
}

