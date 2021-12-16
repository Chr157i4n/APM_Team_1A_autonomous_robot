#include <Arduino.h>
//#include <Servo.h>

#define ANGLE_UP 240
#define ANGLE_DOWN 20

class BatonMechanism 
{
  public:

    /*
    * constructor of the class
    * the number of the pin is needed as parameter
    */
    BatonMechanism(int pin_servo);

    void init();

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
    int _current_angle = 0;
    //Servo _servo;
};