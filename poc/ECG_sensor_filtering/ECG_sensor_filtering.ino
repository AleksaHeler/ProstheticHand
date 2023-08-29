/******************************************************************************
Parts of this code are from:
  -> circuitschools.com (sensor interface): https://www.circuitschools.com/ecg-monitoring-system-using-ad8232-with-arduino-or-esp32-iot-based/
  -> norwegiancreations.com (highpass filter): https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/

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

  Example (Bicep):
    Green: back of elbow
    Yellow: upper end of muscle
    Red: lower end of muscle

Signal processing path:
  Analog signal from sensor -> highpass filter -> threshold activation -> lowpass filter

******************************************************************************/

#include <Arduino.h>
#include <ESP32Servo.h>

// Pins
const int sensorInputPin = 34;
const int loNegPin = 25;
const int loPosPin = 26;
const int servoPin = 33;

// Config constants
const int analogInputThreshold = 350;                 // Value above which the muscleActive is set to TRUE
const float lowpassFilterActivationThreshold = 0.05;  // After lowpass, how to check if muscle is active
const int sensorReadDelay = 10;                      // How long to wait between sensor readings
const float EMA_a = 0.3;                             // Initialization of EMA alpha
const float lowpassWeight = 0.1;                     // How much the values are smoothed
#define DEBOUNCE_DELAY 600
#define FINGER_ACTIVE_ANGLE 140
#define FINGER_NOT_ACTIVE_ANGLE 40

// Variables for highpass filter

// Other variables
int sensorValue = 0;                            // Initialization of sensor variable, equivalent to EMA Y
int highpassOutput = 0;                         // Output from high pass filter stage
bool highpassBool = false;
float lowpassValue = 0.0; 
float lowpassOutput = 0;                        // Output from low pass filter stage
int EMA_S = 0;                                  // Highpass filter initialization of EMA S
bool muscleActive = false;                      // Output from thresholding stage
bool fingerActive = false;
int lastMusleActive = 0;
Servo myservo;  // create servo object to control a servo

void setup() 
{
  // Initialize the serial communication
  Serial.begin(115200);

  // Setup pins/ports
  pinMode(loNegPin, INPUT); // Setup for leads off detection LO +
  pinMode(loPosPin, INPUT); // Setup for leads off detection LO -

  fingerActive = false;
  myservo.attach(servoPin);  // attaches the servo on pin 13 to the servo object
  
  // Reset variables
  highpassFilterReset();
}



void loop()
{
  // If leads are not connected
  //if((digitalRead(loNegPin) == 1)||(digitalRead(loPosPin) == 1)){
  //  Serial.println('!');
  //  return;
  //}
  
  sensorValue = readSensor();                                                        // Read the sensor value using ADC
  highpassOutput = highpassFilter(sensorValue);                                      // Calculate the high-pass signal
  highpassBool = thresholdSignal(highpassOutput, analogInputThreshold);
  lowpassOutput = lowpassFilter(highpassBool);                                       // Calculate the low-pass signal (and convert int signal to bool)
  muscleActive = thresholdSignal(lowpassOutput, lowpassFilterActivationThreshold);
  
  Serial.println(muscleActive);

  if (muscleActive && ( millis() - lastMusleActive > DEBOUNCE_DELAY)) {
    fingerActive = !fingerActive;
    lastMusleActive = millis();
  }

  if (fingerActive) {
    myservo.write(FINGER_ACTIVE_ANGLE);
  }
  else {
    myservo.write(FINGER_NOT_ACTIVE_ANGLE);
  }

  // Wait for a bit
  delay(sensorReadDelay);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Read the sensor value using ADC
int readSensor(){
  return analogRead(sensorInputPin);
}

// Set EMA_S for t=1
void highpassFilterReset(){
  EMA_S = analogRead(sensorInputPin);
}

// Filter input val
int highpassFilter(int val){
  EMA_S = (EMA_a*val) + ((1-EMA_a)*EMA_S);        //run the EMA
  return val - EMA_S;                             //calculate the high-pass signal 
}

// Return true if signal is greater than 'contractionDetectionThreshold'
bool thresholdSignal(float val, float threshold){
  if(abs(val) > threshold){
    return true;
  }
  return false;
}

float lowpassFilter(bool val){
  lowpassValue = (1.0-lowpassWeight) * lowpassValue + lowpassWeight * (float)val;
  return lowpassValue;
}
