#include <Arduino.h>

#define MAX_VALUE_COUNT 20

class FotoTransistorSensor 
{
  public:

    /*
    * constructor of the class
    * the number of the pin is needed as parameter
    */
    FotoTransistorSensor(int pin_sense);


    /*
    * this function returns the current measured value of the sensor
    *
    */
    int getValue();


    /*
    * this function returns the current measured value of the sensor
    * but it also save the current measured value in an array
    */
    int logValue();


    /*
    * this function calculates the mean value of all logged values
    * and returns the mean value
    */
    int getLoggedMean();
    
    
  private:
    int _pin_sense = A0;

    unsigned short valueLog[MAX_VALUE_COUNT];
    unsigned short valueLogIterator = 0;
};