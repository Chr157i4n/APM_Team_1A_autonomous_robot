#include <Arduino.h>
#include <PID_v1.h>
#include <Ultrasonic.h>

#include "TB6612MotorShield.h"
#include "FotoTransistorSensor.h"
#include "BatonMechanism.h"

#define BAUD_RATE 9600


TB6612MotorShield motor;

int baseSpeed=300;


/*
* this function inverts the motor speed of the second motor
* so that both wheels drive in the same direction when beeing set with the same value
*/
void setMotorSpeeds(int m1Speed, int m2Speed){
  
  motor.setSpeeds(-m1Speed, m2Speed);

}


/*
* this function is called once when the arduino starts
*
*/
void setup() {

  Serial.begin(BAUD_RATE);
  Serial.println("setup");

  motor.setBreak(false);

  Serial.println("setup finished");
}


/*
* this function is running in a loop while the arduino is running
*
*/
void loop() {

    setMotorSpeeds(baseSpeed, baseSpeed); // set the actual motor speed

}