#ifndef TB6612MotorShield_h
#define TB6612MotorShield_h

#include <Arduino.h>

class TB6612MotorShield 
{
  public:
    TB6612MotorShield();
    static void setM1Speed(int speed);
    static void setM2Speed(int speed);
    static void setSpeeds(int m1Speed, int m2Speed);
    static void setBreak(boolean brk);
    
    
  private:
    static void initPins();
    const static unsigned char _M1aDIR;
	  const static unsigned char _M1bDIR;
    const static unsigned char _M2aDIR;
	  const static unsigned char _M2bDIR;
    const static unsigned char _M1PWM;
    const static unsigned char _M2PWM;
	  const static unsigned char _STBY;
    const static unsigned char _maxSpeed;
       
    static inline void init()
    {
      static boolean initialized = false;
      if (!initialized)
      {
        initialized = true;
        initPins();
      }
    }
};
#endif
