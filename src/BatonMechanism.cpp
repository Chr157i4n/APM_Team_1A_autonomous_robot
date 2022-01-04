#include "BatonMechanism.h"


BatonMechanism::BatonMechanism(int pin_servo){
   
    _pin_servo = pin_servo;
    pinMode(_pin_servo, OUTPUT);

}

void BatonMechanism::init(){

    //_servo.attach(_pin_servo);

    //_servo.write(ANGLE_UP);

    analogWrite(_pin_servo, ANGLE_UP);
    delay(500);

    _current_angle = ANGLE_UP;

}

void BatonMechanism::driveServo(int targetAngle, float speed){

  if(targetAngle>_current_angle){
  
    while(_current_angle<=targetAngle){
      _current_angle++;
      //_servo.write(_current_angle);
      analogWrite(_pin_servo,_current_angle);
      delay(10/speed);
    }

  }else{

    while(_current_angle>=targetAngle){
      _current_angle--;
      //_servo.write(_current_angle);
      analogWrite(_pin_servo,_current_angle);
      delay(10/speed);
    }

  }
}

void BatonMechanism::unload(){
    Serial.println("Baton unloading");
    tiltDown();
}

void BatonMechanism::tiltDown(){
    driveServo(ANGLE_DOWN, 1);
}

void BatonMechanism::tiltUp(){
    driveServo(ANGLE_UP, 1);
}

void BatonMechanism::tilt(int angle){
    driveServo(angle, 1);
}
