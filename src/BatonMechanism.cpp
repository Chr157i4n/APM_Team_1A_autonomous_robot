#include "BatonMechanism.h"


BatonMechanism::BatonMechanism(int pin_servo){
    _pin_servo = pin_servo;

    _servo.attach(_pin_servo);

    _servo.write(_up_angle);
    _current_angle = _up_angle;


}

void BatonMechanism::driveServo(int targetAngle, float speed){

  if(targetAngle>_current_angle){
  
    while(_current_angle<=targetAngle){
      _current_angle++;
      _servo.write(_current_angle);
      delay(10/speed);
    }

  }else{

    while(_current_angle>=targetAngle){
      _current_angle--;
      _servo.write(_current_angle);
      delay(10/speed);
    }

  }
}

void BatonMechanism::unload(){
    Serial.println("Baton unloading");
    tiltDown();
}

void BatonMechanism::tiltDown(){
    driveServo(_down_angle, 1);
}

void BatonMechanism::tiltUp(){
    driveServo(_up_angle, 1);
}

void BatonMechanism::tilt(int angle){
    driveServo(angle, 1);
}
