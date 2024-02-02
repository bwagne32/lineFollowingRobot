/* References
  QTR sensor: https://github.com/pololu/qtr-sensors-arduino/blob/master/examples/QTRAExample/QTRAExample.ino


*/

#define core0 0
#define core1 1

#include "var.hpp"
#include "comms.hpp"
#include "PID.hpp"

#include <QTRSensors.h>
#include <ESP32Servo.h>

// Motors //////////////////////////////////////////////////////////////////////////////////////////////////
const short leftWheelPin1 = 7;
const short leftWheelPin2 = 6;
const short leftPWMpin = 5;

const short rightWheelPin1 = 4;
const short rightWheelPin2 = 3;
const short rightPWMpin = 2;

VAR_HPP_::motor left(leftWheelPin1, leftWheelPin2, leftPWMpin);
VAR_HPP_::motor right(rightWheelPin1,rightWheelPin2,rightPWMpin);

// Other pins //////////////////////////////////////////////////////////////////////////////////////////////////
const short fanPin = 12;
const short fanEnablePin = 8;

// Global non constant variables ////////////////////////////////////////////////////////////////////////////////
short output = 0;
bool stop = 0;


void setup() {
  // Motors 
  pinMode(leftWheelPin1, OUTPUT);
  pinMode(leftWheelPin2, OUTPUT);
  pinMode(leftPWMpin, OUTPUT);

  pinMode(rightWheelPin1, OUTPUT);
  pinMode(rightWheelPin2, OUTPUT);
  pinMode(rightPWMpin, OUTPUT);

  pinMode(fanPin, OUTPUT);
  pinMode(fanEnablePin, OUTPUT);


  // QTR sensors 
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){  A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode



  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 200; i++)
  {
    left.drive(50);
    right.drive(-50);
    qtr.calibrate();
    left.drive(-50);
    right.drive(50);
    qtr.calibrate();
  }

  left.drive(0);
  right.drive(0);

  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration


/*
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
*/


 /*
  xTaskCreatePinnedToCore(
    TaskBlink, "Task Blink"  // A name just for humans
    ,
    2048  // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,
    (void *)&blink_delay  // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,
    2  // Priority
    ,
    NULL  // Task handle is not used here - simply pass NULL
    ,
    CORE_0  // Core on which the task will run
  );
*/
  digitalWrite(fanEnablePin, HIGH);
  xTaskCreatePinnedToCore(comms, "Motor control", 2048, nullptr, 2, NULL, core1);
  xTaskCreatePinnedToCore(car, "PID control", 2048, nullptr, 2, NULL, core0);
}


void loop() { // unused since each core will be operating indepently
}


// Core loops //////////////////////////////////////////////////////////////////////////////////////////////////
void car(void *pvParameters){ // reads inputs, calculates PD control, and sends motor signals
  PID_HPP_::loopPID(stop, left, right);
}


void comms(void *pvParameters) { // sends communication info over BLE
    COMMS_HPP_::loopComms(stop);
}

// This function will probably need reworked
void fan(short pwm){// set fan speed
  analogWrite(fanPin, pwm);
}










