/* References
  QTR sensor: https://github.com/pololu/qtr-sensors-arduino/blob/master/examples/QTRAExample/QTRAExample.ino


*/

#define core0 0
#define core1 1

#include "var.hpp"
#include "comms.hpp"
#include "PID.hpp"

#include <QTRSensors.h>





void setup() {
  // Motors 
  pinMode(leftWheelPin1, OUTPUT);
  pinMode(rightWheelPin1, OUTPUT);
  pinMode(leftWheelPin2, OUTPUT);
  pinMode(rightWheelPin2, OUTPUT);

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
    left(128,0);
    right(0,128);
    qtr.calibrate();
    left(0,128);
    right(128,0);
    qtr.calibrate();
  }
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

  xTaskCreatePinnedToCore(car, "PID control", 2048, nullptr, 2, NULL, core0);
  xTaskCreatePinnedToCore(comms, "Motor control", 2048, nullptr, 2, NULL, core1);
}


void loop() { // unused since each core will be operating indepently
}


// Core loops //////////////////////////////////////////////////////////////////////////////////////////////////
void car(void *pvParameters){ // reads inputs, calculates PD control, and sends motor signals
  while(1){


  }
}


void comms(void *pvParameters) { // sends communication info over BLE
  while(1){




  }
}












