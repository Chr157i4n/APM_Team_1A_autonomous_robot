#include "FotoTransistorSensor.h"


FotoTransistorSensor::FotoTransistorSensor(int pin_sense){
    _pin_sense = pin_sense;

    for(int i=0; i<MAX_VALUE_COUNT; i++){
        valueLog[i] = 0;
    }
}

int FotoTransistorSensor::getValue(){
    return analogRead(_pin_sense);
}

int FotoTransistorSensor::logValue(){
    unsigned short currentValue = getValue();

    valueLog[valueLogIterator] = currentValue;
    valueLogIterator++;
    if(valueLogIterator>=MAX_VALUE_COUNT) 
    {
        valueLogIterator = 0;
    }

    return currentValue;
}

int FotoTransistorSensor::getLoggedMean(){
    unsigned long sum = 0;

    for(int i=0; i<MAX_VALUE_COUNT; i++){
        sum += valueLog[i];
    }

    return sum/MAX_VALUE_COUNT;
}