#include <Arduino.h>
#include "TB6612MotorShield.h"
#include "LineSensor.h"

TB6612MotorShield motor;
LineSensor lineSensor(A7);

void setup() {

  Serial.begin(9600);
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);

  motor.setSpeeds(100,100);
  delay(2000);
  motor.setSpeeds(-100,-100);
  delay(2000);
  motor.setSpeeds(0,0);

}

void loop() {

  //Serial.println("test");
  Serial.println(lineSensor.getValue());

}