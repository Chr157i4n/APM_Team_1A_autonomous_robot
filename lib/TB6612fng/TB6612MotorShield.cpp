#include "TB6612MotorShield.h"

const unsigned char TB6612MotorShield::_M1aDIR = 6;
const unsigned char TB6612MotorShield::_M1bDIR = 7;
const unsigned char TB6612MotorShield::_M2aDIR = 5;
const unsigned char TB6612MotorShield::_M2bDIR = 4;
const unsigned char TB6612MotorShield::_M1PWM = 11;
const unsigned char TB6612MotorShield::_M2PWM = 10;
const unsigned char TB6612MotorShield::_STBY = 13;
const unsigned char TB6612MotorShield::_maxSpeed =180; //0..255
TB6612MotorShield::TB6612MotorShield()
{  
	init();
  Serial.begin(9600);
}

void TB6612MotorShield::initPins()
{
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high, 
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.
  digitalWrite(_M1PWM, LOW);
  pinMode(_M1PWM, OUTPUT);
  digitalWrite(_M1PWM, LOW);
  digitalWrite(_M2PWM, LOW);
  pinMode(_M2PWM, OUTPUT);
  digitalWrite(_M2PWM, LOW);
  
  digitalWrite(_M1aDIR, LOW);
  pinMode(_M1aDIR, OUTPUT);
  digitalWrite(_M1bDIR, LOW);
  pinMode(_M1bDIR, OUTPUT);
  
  digitalWrite(_M2aDIR, LOW);
  pinMode(_M2aDIR, OUTPUT);
  digitalWrite(_M2bDIR, LOW);
  pinMode(_M2bDIR, OUTPUT);
  
  digitalWrite(_STBY, LOW);
  pinMode(_STBY, OUTPUT);  
  
}

// speed should be a number between -400 and 400
void TB6612MotorShield::setM1Speed(int speed)
{
  init(); // initialize if necessary
    
  boolean reverse = 0;
 
  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400)  // max 
    speed = 400;
    
  if (reverse) 
  {
    digitalWrite(_M1aDIR, HIGH);
	  digitalWrite(_M1bDIR, LOW);
  }
  else
  {
    digitalWrite(_M1aDIR, LOW);
	  digitalWrite(_M1bDIR, HIGH);
  };
  analogWrite(_M1PWM,1.0*_maxSpeed/400.0*speed);  
}

// speed should be a number between -400 and 400
void TB6612MotorShield::setM2Speed(int speed)
{
  init(); // initialize if necessary
    
  boolean reverse = 0;
 
  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400)  // max 
    speed = 400;
    
  if (reverse) 
  {
    digitalWrite(_M2aDIR, HIGH);
    digitalWrite(_M2bDIR, LOW);
  }
  else
  {
    digitalWrite(_M2aDIR, LOW);
  digitalWrite(_M2bDIR, HIGH);
  };
  analogWrite(_M2PWM,1.0*_maxSpeed/400.0*speed);  
}


// set speed for both motors
// speed should be a number between -400 and 400
void TB6612MotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

void TB6612MotorShield::setBreak(boolean brk)
{
	if (brk)
  {
	digitalWrite(_STBY,LOW);
    digitalWrite(_M1aDIR, LOW);
    digitalWrite(_M1bDIR, LOW);
    digitalWrite(_M2aDIR, LOW);
    digitalWrite(_M2bDIR, LOW);
  }
	else
		digitalWrite(_STBY,HIGH);
}

