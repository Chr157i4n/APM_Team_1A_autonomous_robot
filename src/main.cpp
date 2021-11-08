#include <Arduino.h>
#include <PID_v1.h>
#include <Ultrasonic.h>

#include "TB6612MotorShield.h"
#include "LineSensor.h"

#define BAUD_RATE 9600

#define ULTRASONIC_SENSOR_TRIGGER_PIN 12
#define ULTRASONIC_SENSOR_ECHO_PIN 9


TB6612MotorShield motor;
Ultrasonic ultrasonic(ULTRASONIC_SENSOR_TRIGGER_PIN, ULTRASONIC_SENSOR_ECHO_PIN);

/*
* this function is called once when the arduino starts
*
*/
void setup() {

  Serial.begin(BAUD_RATE);
  Serial.println("setup");


  motor.setSpeeds(0,0);                         // Ruckbewegung der Motoren am Anfang fällt hiermit weg.
  motor.setBreak(true);

  Serial.println("setup finished");
}


/*
* this function is running in a loop while the arduino is running
*
*/
void loop() {
  
  int time1 = millis();

  int distance = ultrasonic.read();

  int time2 = millis();

  Serial.println((String) distance+" cm. it took "+(time2-time1)+" ms");

}