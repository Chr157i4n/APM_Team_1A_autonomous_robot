#include <Arduino.h>
#include <PID_v1.h>
#include <Ultrasonic.h>

#include "TB6612MotorShield.h"
#include "LineSensor.h"

#define BAUD_RATE 9600
#define PIN_LINESENSOR_SENSE A7
#define PIN_LINESENSOR_POWER 12
#define DURATION_INITIAL_WAIT 1000 //ms
#define DURATION_DRIVE 20000 //ms
#define PRINT_DEBUG 0


TB6612MotorShield motor;
LineSensor lineSensor(PIN_LINESENSOR_SENSE);

unsigned long timeStart = 0, timeCurrent = 0;


// Sensor values in T1137
// lights out - black ground  24
// lights on - black ground 25-26

// lights out - malerkrepp 43-44
// lights on - malerkrepp 49

// lights out - white electrical tape 79-80
// lights on - white electrical tape 88

// lights out - black electrical tape 16
// lights on - black electrical tape 18

//variables for driving
int baseSpeed = 100;
double lineSensorMax = 80, lineSensorMin = 24;  // black ground - white eletrical tape
//double lineSensorMax = 44, lineSensorMin = 24;  // black ground - malerkrepp
double Setpoint = 512, lineSensorValue = 0, lineSensorPIDValue = 0;

// PID for steering
// Kp is the proportional factor the PID controller. it the main value to correct an offset of the measured value.
// Ki is the integral factor the PID controller. Ki is used to correct systematic errors that would cause a constant offset, like a slower motor on one side.
// Kd is the differential factor the PID controller. With Kd the controller can react to fast changes like a sharp curve.
// this needs calibration
//double Kp=0.5, Ki=0.1, Kd=0.2; //worked
double Kp=0.6, Ki=0.1, Kd=0.02; //worked
PID lineSensorPID(&lineSensorValue, &lineSensorPIDValue, &Setpoint, Kp, Ki, Kd, DIRECT);


/*
* this function inverts the motor speed of the second motor
* so that both wheels drive in the same direction when beeing set with the same value
*/
void setMotorSpeeds(int m1Speed, int m2Speed){
  
  motor.setSpeeds(-m1Speed, m2Speed);

}


/*
* this function is called once when the arduino starts
*
*/
void setup() {

  Serial.begin(BAUD_RATE);
  Serial.println("setup");

  // the Setpoint is the value where the measured input value of the sensor should be
  // and this should be the middle of the max (white) and the min (black) value of the sensor.
  // we still need to calibrate this 
  Setpoint = (lineSensorMax + lineSensorMin)*0.5;

  motor.setSpeeds(0,0);                         // Ruckbewegung der Motoren am Anfang fällt hiermit weg.
  motor.setBreak(true);

  lineSensorPID.SetOutputLimits(-50,50);      // standard of the limits is (0, 255) and we need negative values
  lineSensorPID.SetMode(AUTOMATIC);

  pinMode(PIN_LINESENSOR_POWER, OUTPUT);
  digitalWrite(PIN_LINESENSOR_POWER, HIGH);

  delay(DURATION_INITIAL_WAIT);                 // Wait a couple of seconds to start

  motor.setBreak(false);
  timeStart = millis();                         // save the current time as starting time

  Serial.println("setup finished");
}


/*
* this function is running in a loop while the arduino is running
*
*/
void loop() {
  
  timeCurrent = millis();                                     // get the current time
  if(timeCurrent < timeStart + DURATION_DRIVE){               // only drive for a defined amount of time
    
    lineSensorValue = lineSensor.getValue();                    // reading the line sensor (phototransistor) value
    //normalizedsensorValue = (sensorValue - 512) * 0.1;

    lineSensorPID.Compute();                                    // compute the output value for the steering based on the line sensor value (part of the PID libary)

#if PRINT_DEBUG == 1
    Serial.println((String)"set: "+Setpoint+" raw: "+lineSensorValue+" pid: "+lineSensorPIDValue+" lS: "+(baseSpeed+lineSensorPIDValue)+" rS: "+(baseSpeed-lineSensorPIDValue));
#endif


    // if you comment this out, the motor does not start to move
    setMotorSpeeds(baseSpeed+lineSensorPIDValue, baseSpeed-lineSensorPIDValue);     // set the actual motor speed

  } else {
    
    setMotorSpeeds(0, 0);
    motor.setBreak(true);
    //Serial.println("Destination reached");

    lineSensorValue = lineSensor.getValue();                    // reading the line sensor (phototransistor) value

    lineSensorPID.Compute();                                    // compute the output value for the steering based on the line sensor value (part of the PID libary)

#if PRINT_DEBUG == 1
    //still outputing the pid values after finished driving for testing purposes
    Serial.println((String)"set: "+Setpoint+" raw: "+lineSensorValue+" pid: "+lineSensorPIDValue+" lS: "+(baseSpeed+lineSensorPIDValue)+" rS: "+(baseSpeed-lineSensorPIDValue));
#endif

    
  }

}