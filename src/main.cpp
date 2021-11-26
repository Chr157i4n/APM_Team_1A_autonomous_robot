#include <Arduino.h>
#include <PID_v1.h>
#include <Ultrasonic.h>
#include <Servo.h>

#include "TB6612MotorShield.h"
#include "FotoTransistorSensor.h"
#include "BatonMechanism.h"

#define BAUD_RATE 9600


#define PIN_SERVO 9 //todo: needs to be changed

int currentServoAngle = 0;


Servo servo;


void setup() {

  Serial.begin(BAUD_RATE);
  Serial.println("setup");
  servo.attach(PIN_SERVO);

  servo.write(20);
  currentServoAngle = 20;

  Serial.println("setup finished");
}

void driveServo(int targetAngle, float speed){

  if(targetAngle>currentServoAngle){
  
    while(currentServoAngle<=targetAngle){
      currentServoAngle++;
      servo.write(currentServoAngle);
      delay(10/speed);
    }

  }else{

    while(currentServoAngle>=targetAngle){
      currentServoAngle--;
      servo.write(currentServoAngle);
      delay(10/speed);
    }

  }
}


/*
* this function is running in a loop while the arduino is running
*
*/
void loop() {

  Serial.println((String) "Driving Servo to: 20°");
  driveServo(20,1);

  delay(2000);

  Serial.println((String) "Driving Servo to: 180°");
  driveServo(180,1);

  delay(2000);
}