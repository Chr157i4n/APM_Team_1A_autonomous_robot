#include <Arduino.h>

class LineSensor 
{
  public:
    LineSensor(int pin_sense);
    static int getValue();
    
    
  private:
    static int _pin_sense;
};