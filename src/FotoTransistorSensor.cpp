#include "FotoTransistorSensor.h"


FotoTransistorSensor::FotoTransistorSensor(int pin_sense){
    _pin_sense = pin_sense;
}

int FotoTransistorSensor::getValue(){
    return analogRead(_pin_sense);
}