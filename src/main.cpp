#include <Arduino.h>
#include "TB6612MotorShield.h"
#include "LineSensor.h"

//TB6612MotorShield motor;
LineSensor lineSensor(A7);

void setup() {

  //motor.setSpeeds(100,100);

  Serial.begin(115200);
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);

}

void loop() {

  Serial.println(lineSensor.getValue());

}