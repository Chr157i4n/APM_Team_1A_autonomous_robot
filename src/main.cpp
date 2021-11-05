#include <Arduino.h>
#include <PID_v1.h>

#include "TB6612MotorShield.h"
#include "LineSensor.h"

#define BAUD_RATE 9600


TB6612MotorShield motor;

/*
* this function is called once when the arduino starts
*
*/
void setup() {

  Serial.begin(BAUD_RATE);
  Serial.println("setup");


  motor.setSpeeds(0,0);                         // Ruckbewegung der Motoren am Anfang fällt hiermit weg.
  motor.setBreak(true);

  Serial.println("setup finished");
}


/*
* this function is running in a loop while the arduino is running
*
*/
void loop() {
  
 

}