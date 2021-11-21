#include <Arduino.h>
#include <Servo.h>

class BatonMechanism 
{
  public:

    /*
    * constructor of the class
    * the number of the pin is needed as parameter
    */
    BatonMechanism(int pin_servo);


    /*
    * this function unloads the baton
    *
    */
    void unload();

    void tiltDown();

    void tiltUp();

    void tilt(int angle);
    
    
  private:
    int _pin_servo = -1;
    int _up_angle = 0, _down_angle = 180;
    Servo _servo;
};