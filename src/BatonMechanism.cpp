#include "BatonMechanism.h"


BatonMechanism::BatonMechanism(int pin_servo){
    _pin_servo = pin_servo;

    _servo.attach(_pin_servo);
}

void BatonMechanism::unload(){
    tiltUp();
}

void BatonMechanism::tiltDown(){
    _servo.write(_down_angle);
}

void BatonMechanism::tiltUp(){
    _servo.write(_up_angle);
}

void BatonMechanism::tilt(int angle){
    _servo.write(angle);
}
