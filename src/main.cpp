#include <Arduino.h>
#include "test.h"
#include <Ultrasonic.h>

Ultrasonic ultrasonic1(12, 13);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Neustart");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("LED AN");
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("LED AUS");
  delay(1000);
}