#include "LineSensor.h"


int LineSensor::_pin_sense = A0;

LineSensor::LineSensor(int pin_sense){
    _pin_sense = pin_sense;
}

int LineSensor::getValue(){
    return analogRead(_pin_sense);
}