#include <Arduino.h>
#include <PID_v1.h>

#include "TB6612MotorShield.h"
#include "LineSensor.h"

#define PIN_LINESENSOR A7

TB6612MotorShield motor;
LineSensor lineSensor(PIN_LINESENSOR);


int baseSpeed = 200;
double Setpoint = 512, lineSensorValue, lineSensorPIDValue;

//Specify the links and initial tuning parameters
double Kp=0.1, Ki=0, Kd=0;
PID lineSensorPID(&lineSensorValue, &lineSensorPIDValue, &Setpoint, Kp, Ki, Kd, DIRECT);


void setMotorSpeeds(int m1Speed, int m2Speed){
  
  motor.setSpeeds(m1Speed, -m2Speed);

}

void setup() {

  lineSensorPID.SetOutputLimits(-100,100);
  lineSensorPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);

  motor.setSpeeds(0,0);           //Ruckbewegung der Motoren am Anfang fällt hiermit weg.
  delay(1000);


}

void loop() {
  
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

}