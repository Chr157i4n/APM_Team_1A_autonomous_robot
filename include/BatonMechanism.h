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

    void driveServo(int targetAngle, float speed);

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
    int _up_angle = 20, _down_angle = 180;
    int _current_angle = 0;
    Servo _servo;
};