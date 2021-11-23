#include <Arduino.h>

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
    
    
  private:
    int _pin_sense = A0;
};