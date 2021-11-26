#include <Arduino.h>
#include <PID_v1.h>
#include <Ultrasonic.h>
#include <Servo.h>

#include "TB6612MotorShield.h"
#include "FotoTransistorSensor.h"
#include "BatonMechanism.h"

#define BAUD_RATE 9600


#define PIN_SERVO 9 //todo: needs to be changed


Servo servo;


void setup() {

  Serial.begin(BAUD_RATE);
  Serial.println("setup");
git fetch
  servo.attach(PIN_SERVO);

  Serial.println("setup finished");
}


/*
* this function is running in a loop while the arduino is running
*
*/
void loop() {

  Serial.println((String) "Driving Servo to: 0°");
  servo.write(0);

  delay(2000);

  Serial.println((String) "Driving Servo to: 180°");
  servo.write(180);

  delay(2000);
}