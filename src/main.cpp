#include <Arduino.h>
#include <PID_v1.h>
#include <Ultrasonic.h>

#include "TB6612MotorShield.h"
#include "FotoTransistorSensor.h"
#include "BatonMechanism.h"

#define BAUD_RATE 9600

#define PIN_LINESENSOR_SENSE A6
#define PIN_BATONSENSOR_SENSE A7 // todo: needs to be changed
#define PIN_ULTRASONIC_SENSOR_TRIGGER 12
#define PIN_ULTRASONIC_SENSOR_ECHO 8
#define PIN_SERVO 9 //Servo has to be on Pin 9, because Pin 9 is able to do PWM

#define DURATION_INITIAL_WAIT 1000 //ms
#define DURATION_INITIAL_WAIT_AFTER_BATON_DETECTED 2000 //ms
#define DURATION_DRIVE_TIMEOUT 20000 //ms

#define THRESHOLD_BATONSENSOR_DETECT 50 // todo: needs to be changed

#define PRINT_DEBUG 1


TB6612MotorShield motor;
FotoTransistorSensor lineSensor(PIN_LINESENSOR_SENSE);
FotoTransistorSensor batonSensor(PIN_BATONSENSOR_SENSE);
Ultrasonic ultrasonic(PIN_ULTRASONIC_SENSOR_TRIGGER, PIN_ULTRASONIC_SENSOR_ECHO);
BatonMechanism batonMechanism(PIN_SERVO);

unsigned long timeStart = 0, timeCurrent = 0, timeElasped = 0;
int distanceSecondRobot = 0, batonSensorValue = 0;

unsigned short state = 0;
// state 0 = wait for baton
// state 1 = drive
// state 2 = finished, stop driving


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

  batonMechanism.tiltUp();

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

  timeCurrent = millis(); // get the current time
  timeElasped = timeCurrent - timeStart;

  if(state == 0){
    batonSensorValue = batonSensor.getValue(); //todo: change to second sensor
#if PRINT_DEBUG == 1
    Serial.println((String)"batonSensorValue: "+batonSensorValue);
#endif

    if(batonSensorValue < THRESHOLD_BATONSENSOR_DETECT){ 
      //if the brightness is below the THRESHOLD_BATONSENSOR_DETECT the robot goes into state 1
      state = 1;
      Serial.println("robot should start driving in 2 seconds");
      delay(DURATION_INITIAL_WAIT_AFTER_BATON_DETECTED);
      timeStart = timeCurrent;
    }

  } else if(state == 1){
    distanceSecondRobot = ultrasonic.read();
    Serial.println("Distance: "+distanceSecondRobot);

    if(distanceSecondRobot < 20 || timeElasped > DURATION_DRIVE_TIMEOUT){
      // if the measured distance of the ultrasonic sensor is below 20cm the robot should go into state 2 (stop)
      state = 2;
      Serial.println("destination or timeout reached");
    } 

    lineSensorValue = lineSensor.getValue();  // reading the line sensor (phototransistor) value
    //normalizedsensorValue = (sensorValue - 512) * 0.1;

    lineSensorPID.Compute();  // compute the output value for the steering based on the line sensor value (part of the PID libary)

#if PRINT_DEBUG == 1
    Serial.println((String)"set: "+Setpoint+" raw: "+lineSensorValue+" pid: "+lineSensorPIDValue+" lS: "+(baseSpeed+lineSensorPIDValue)+" rS: "+(baseSpeed-lineSensorPIDValue));
#endif

    // if you comment this out, the motor does not start to move
    setMotorSpeeds(baseSpeed+lineSensorPIDValue, baseSpeed-lineSensorPIDValue); // set the actual motor speed

  } else if(state == 2) {
    //second robot reached

    setMotorSpeeds(0, 0);
    motor.setBreak(true);
    batonMechanism.unload();
    state = 3;

  } else {

#if PRINT_DEBUG == 1
    lineSensorValue = lineSensor.getValue();                    // reading the line sensor (phototransistor) value

    lineSensorPID.Compute();                                    // compute the output value for the steering based on the line sensor value (part of the PID libary)

    //still outputing the pid values after finished driving for testing purposes
    Serial.println((String)"set: "+Setpoint+" raw: "+lineSensorValue+" pid: "+lineSensorPIDValue+" lS: "+(baseSpeed+lineSensorPIDValue)+" rS: "+(baseSpeed-lineSensorPIDValue));
#endif

  }

}