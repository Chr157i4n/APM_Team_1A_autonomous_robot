#include <Arduino.h>
#include <PID_v1.h>

#include "TB6612MotorShield.h"
#include "LineSensor.h"

#define BAUD_RATE 9600
#define PIN_LINESENSOR_SENSE A7
#define PIN_LINESENSOR_POWER 12
#define DURATION_INITIAL_WAIT 1000 //ms
#define DURATION_DRIVE 20000 //ms


TB6612MotorShield motor;
LineSensor lineSensor(PIN_LINESENSOR_SENSE);

unsigned long timeStart = 0, timeCurrent = 0;

double lineSensorMax = 560, lineSensorMin = 450;

//variables for driving
int baseSpeed = 100;
double Setpoint = 512, lineSensorValue = 0, lineSensorPIDValue = 0;

// PID for steering
double Kp=0.6, Ki=0.1, Kd=0.2;
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

  Setpoint = (lineSensorMax + lineSensorMin)*0.5;

  motor.setSpeeds(0,0);                         // Ruckbewegung der Motoren am Anfang fällt hiermit weg.
  motor.setBreak(true);

  lineSensorPID.SetOutputLimits(-50,50);      // standard of the limits is (0, 255) and we need negative values
  lineSensorPID.SetMode(AUTOMATIC);

  Serial.begin(BAUD_RATE);
  pinMode(PIN_LINESENSOR_POWER, OUTPUT);
  digitalWrite(PIN_LINESENSOR_POWER, HIGH);

  delay(DURATION_INITIAL_WAIT);                 // Wait a couple of seconds to start

  motor.setBreak(false);
  timeStart = millis();                         // save the current time as starting time
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
    Serial.print(" set: ");
    Serial.print(Setpoint);
    Serial.print(" raw: ");
    Serial.print(lineSensorValue);
    Serial.print(" pid: ");
    Serial.print(lineSensorPIDValue);
    Serial.print(" lS: ");
    Serial.print(baseSpeed+lineSensorPIDValue);
    Serial.print(" rS: ");
    Serial.println(baseSpeed-lineSensorPIDValue);


    setMotorSpeeds(baseSpeed+lineSensorPIDValue, baseSpeed-lineSensorPIDValue);     // set the actual motor speed

  } else {
    
    setMotorSpeeds(0, 0);
    motor.setBreak(true);
    //Serial.println("Destination reached");

    lineSensorValue = lineSensor.getValue();                    // reading the line sensor (phototransistor) value
    //normalizedsensorValue = (sensorValue - 512) * 0.1;

    lineSensorPID.Compute();                                    // compute the output value for the steering based on the line sensor value (part of the PID libary)
    Serial.print(" set: ");
    Serial.print(Setpoint);
    Serial.print(" raw: ");
    Serial.print(lineSensorValue);
    Serial.print(" pid: ");
    Serial.print(lineSensorPIDValue);
    Serial.print(" lS: ");
    Serial.print(baseSpeed+lineSensorPIDValue);
    Serial.print(" rS: ");
    Serial.println(baseSpeed-lineSensorPIDValue);
    
  }

}