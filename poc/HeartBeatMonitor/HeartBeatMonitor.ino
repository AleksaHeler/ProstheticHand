/******************************************************************************
Heart_Rate_Display.ino
Publisher: https://www.circuitschools.com
Example from: https://www.circuitschools.com/ecg-monitoring-system-using-ad8232-with-arduino-or-esp32-iot-based/

ESP & sensor wiring:
  Sensor        ESP
   GND           GND
   3.3v          3.3v
   out           34  (analog)
   LO-           25
   LO+           26
   SDN           not conn.

Body wiring:
  Red: RA (right arm or right side of chest)
  Yellow: LA (left arm or left side of chest)
  Green: RL (right leg)

******************************************************************************/

#include <Arduino.h>

const int analogInputPin = 34;
const int loNegPin = 34;
const int loPosPin = 34;

// Global Variables for highpass filter
int sensorValue = 0;  //initialization of sensor variable, equivalent to EMA Y
float EMA_a = 0.3;    //initialization of EMA alpha
int EMA_S = 0;        //initialization of EMA S
int highpass = 0;

int detectionTreshold = 350; // 500 also works good

void setup() {
  // Initialize the serial communication
  Serial.begin(115200);
  
  pinMode(loNegPin, INPUT); // Setup for leads off detection LO +
  pinMode(loPosPin, INPUT); // Setup for leads off detection LO -

  EMA_S = analogRead(analogInputPin);     //set EMA S for t=1
}
 
void loop() {
  // If leads are not connected
  if((digitalRead(loNegPin) == 1)||(digitalRead(loPosPin) == 1)){
    //Serial.println('!');
  }
  else{
    sensorValue = analogRead(analogInputPin);              //read the sensor value using ADC
    EMA_S = (EMA_a*sensorValue) + ((1-EMA_a)*EMA_S);  //run the EMA
    highpass = sensorValue - EMA_S;                   //calculate the high-pass signal 
    //Serial.println(highpass);

    // If muscle is activated send 1, else 0
    if(abs(highpass) > detectionTreshold){
      Serial.println(1);
    }else{
      Serial.println(0);
    }
  }
  
  // Wait for a bit to keep serial data from saturating
  delay(10);
}
