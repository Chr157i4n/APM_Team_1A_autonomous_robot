#include "TB6612MotorShield.h"

TB6612MotorShield _ms;


void setup() {
   
  Serial.begin(9600); 
  _ms.setSpeeds(-100,0);
  _ms.setBreak(false);
  delay(3000);
  _ms.setBreak(true);
  _ms.setSpeeds(0,-100);
  _ms.setBreak(false);
  delay(3000);
  _ms.setBreak(true);
}

void loop() {
 
  
}
