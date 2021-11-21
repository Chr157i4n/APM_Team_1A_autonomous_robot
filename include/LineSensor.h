#include <Arduino.h>

class LineSensor 
{
  public:

    /*
    * constructor of the class
    * the number of the pin is needed as parameter
    */
    LineSensor(int pin_sense);


    /*
    * this function returns the current measured value of the sensor
    *
    */
    int getValue();
    
    
  private:
    int _pin_sense = A0;
};