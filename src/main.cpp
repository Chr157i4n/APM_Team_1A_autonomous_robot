#include <Arduino.h>
#include <PID_v1.h>

#include "TB6612MotorShield.h"
#include "LineSensor.h"

#define BAUD_RATE 9600
#define PIN_LINESENSOR_SENSE A7
#define PIN_LINESENSOR_POWER 12
#define DURATION_INITIAL_WAIT 5000 //ms
#define DURATION_DRIVE 20000 //ms


TB6612MotorShield motor;
LineSensor lineSensor(PIN_LINESENSOR_SENSE);

unsigned int timeStart = 0, timeCurrent = 0;

//variables for driving
int baseSpeed = 200;
double Setpoint = 512, lineSensorValue = 0, lineSensorPIDValue = 0;

// PID for steering
double Kp=0.1, Ki=0, Kd=0;
PID lineSensorPID(&lineSensorValue, &lineSensorPIDValue, &Setpoint, Kp, Ki, Kd, DIRECT);


void setMotorSpeeds(int m1Speed, int m2Speed){
  
  motor.setSpeeds(m1Speed, -m2Speed);

}

void setup() {

  lineSensorPID.SetOutputLimits(-100,100);
  lineSensorPID.SetMode(AUTOMATIC);

  Serial.begin(BAUD_RATE);
  pinMode(PIN_LINESENSOR_POWER, OUTPUT);
  digitalWrite(PIN_LINESENSOR_POWER, HIGH);

  motor.setSpeeds(0,0);           //Ruckbewegung der Motoren am Anfang fällt hiermit weg.
  delay(DURATION_INITIAL_WAIT);

  timeStart = millis();
}

void loop() {
  
  timeCurrent = millis();

  if(timeCurrent < timeStart + DURATION_DRIVE){
  lineSensorValue = lineSensor.getValue();
  //normalizedsensorValue = (sensorValue - 512) * 0.1;

  lineSensorPID.Compute();

  Serial.print(" raw: ");
  Serial.print(lineSensorValue);

  Serial.print(" pid: ");
  Serial.print(lineSensorPIDValue);

  Serial.print(" lS: ");
  Serial.print(baseSpeed+lineSensorPIDValue);

  Serial.print(" rS: ");
  Serial.println(baseSpeed-lineSensorPIDValue);

  setMotorSpeeds(baseSpeed+lineSensorPIDValue, baseSpeed-lineSensorPIDValue);

  } else {
    setMotorSpeeds(0, 0);
    Serial.println("Destination reached");
  }

}