#include <Arduino.h>
#include <PID_v1.h>
#include <Ultrasonic.h>

#include "TB6612MotorShield.h"
#include "FotoTransistorSensor.h"
#include "BatonMechanism.h"

#define BAUD_RATE 9600

/*
* this function is called once when the arduino starts
*
*/
void setup() {

  Serial.begin(BAUD_RATE);
  Serial.println("setup");

  Serial.println("setup finished");
}


/*
* this function is running in a loop while the arduino is running
*
*/
void loop() {

  int sensorValue = analogRead(A7);

  Serial.println((String) "sensorValue: "+sensorValue);


}