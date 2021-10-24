#include <Arduino.h>
#include "TB6612MotorShield.h"
#include "LineSensor.h"

TB6612MotorShield motor;
LineSensor lineSensor(A7);

int baseSpeed = 200;
int sensorValue = 0;
int normalizedsensorValue = 0;

void setMotorSpeeds(int m1Speed, int m2Speed){
  
  motor.setSpeeds(m1Speed, -m2Speed);
}

void setup() {

  Serial.begin(9600);
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);

  motor.setSpeeds(0,0);           //Ruckbewegung der Motoren am Anfang fällt hiermit weg.
  delay(1000);


}

void loop() {

  //Serial.println("test");

  sensorValue = lineSensor.getValue();
  normalizedsensorValue = (sensorValue - 512) * 0.1;

  Serial.print(" raw: ");
  Serial.print(sensorValue);

  Serial.print(" norm: ");
  Serial.print(normalizedsensorValue);

  Serial.print(" lS: ");
  Serial.print(baseSpeed+normalizedsensorValue);

  Serial.print(" rS: ");
  Serial.println(baseSpeed-normalizedsensorValue);

  setMotorSpeeds(baseSpeed+normalizedsensorValue, baseSpeed-normalizedsensorValue); 

}