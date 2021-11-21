#include "LineSensor.h"


LineSensor::LineSensor(int pin_sense){
    _pin_sense = pin_sense;
}

int LineSensor::getValue(){
    return analogRead(_pin_sense);
}