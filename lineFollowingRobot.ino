/* References
  QTR sensor: https://github.com/pololu/qtr-sensors-arduino/blob/master/examples/QTRAExample/QTRAExample.ino


*/

#define core0 0
#define core1 1

#include "motorclass.hpp"
#include "comms.hpp"
#include "PID.hpp"
#include "DShotRMT.h"

#include <QTRSensors.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>



// Motors //////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t leftWheelPin1 = 7;
const uint8_t leftWheelPin2 = 6;
const uint8_t leftPWMpin = 5;

const uint8_t rightWheelPin1 = 4;
const uint8_t rightWheelPin2 = 3;
const uint8_t rightPWMpin = 2;

uint8_t maxMotorSpeed = 90; //60
motorclass_h::motor left(leftWheelPin1, leftWheelPin2, leftPWMpin, maxMotorSpeed);
motorclass_h::motor right(rightWheelPin1, rightWheelPin2, rightPWMpin, maxMotorSpeed);

// Dshot //////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t fanPin = 12;
const uint8_t fanEnablePin = 8;
const auto DSHOT_MODE = DSHOT300;
const auto FAILSAFE_THROTTLE = 999;
const auto INITIAL_THROTTLE = 48;

// Initialize a DShotRMT object for the motor
//DShotRMT motor01(fanPin, RMT_CHANNEL_0);

// Global non constant variables ////////////////////////////////////////////////////////////////////////////////
//short output = 0;
bool stop = false;


// BLE stuff ////////////////////////////////////////////////////////////////////////////////
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        //Serial.println("*********");
        //Serial.print("New value: ");
        //for (int i = 0; i < value.length(); i++)
          //Serial.print(value[i]);
        if(value == "kill"){
          stop = true;
          //Serial.println(stop);
          //Serial.println("\n\n\nded\n\n\n");
        }
      }
    }
};

void setup() {
  //motorclass_h::motor_calibration leftCalibrate(leftWheelPin1, leftWheelPin2, leftPWMpin);
  //motorclass_h::motor_calibration rightCalibrate(rightWheelPin1, rightWheelPin2, rightPWMpin);

  //Serial.begin(9600);
  // BLE ////////////////////////////////////////////////////////////////////////////////
  
  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  // Motors ////////////////////////////////////////////////////////////////////////////////
  pinMode(leftWheelPin1, OUTPUT);
  pinMode(leftWheelPin2, OUTPUT);
  pinMode(leftPWMpin, OUTPUT);

  pinMode(rightWheelPin1, OUTPUT);
  pinMode(rightWheelPin2, OUTPUT);
  pinMode(rightPWMpin, OUTPUT);

  pinMode(fanPin, OUTPUT);
  pinMode(fanEnablePin, OUTPUT);


  // QTR sensors ////////////////////////////////////////////////////////////////////////////////
  using PID_HPP_::qtr;
  const uint8_t SensorCount = 8;
  uint16_t sensorValues[SensorCount];
  uint16_t position;  // 0-7000

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){  A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);
  qtr.setEmitterPin(2);

  delay(10000); // 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode



  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  
  /*for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }*/
  for (uint16_t i = 0; i < 200; i++)
  {
    //leftCalibrate.drive(50);
    //rightCalibrate.drive(-50);
    qtr.calibrate();
    //leftCalibrate.drive(-50);
    //rightCalibrate.drive(50);
    qtr.calibrate();
  }

  //leftCalibrate.drive(0);
  //rightCalibrate.drive(0);

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

  // DSHOT
  //digitalWrite(fanEnablePin, HIGH);
  //motor01.begin(DSHOT_MODE);



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
  //Serial.print("running on core ");
  //Serial.println(xPortGetCoreID());
  //Serial.println("ready");
  xTaskCreatePinnedToCore(car, "PID control", 2048, nullptr, 2, NULL, core0);
  xTaskCreatePinnedToCore(comms, "BLE?", 2048, (void *)pCharacteristic, 2, NULL, core1);
}


void loop() { // unused since each core will be operating indepently
}


// Core loops //////////////////////////////////////////////////////////////////////////////////////////////////
void car(void *pvParameters){ // reads inputs, calculates PD control, and sends motor signals
  PID_HPP_::loopPID(stop, left, right, maxMotorSpeed);
}


void comms(void *pvParameters) { // sends communication info over BLE
    COMMS_HPP_::loopComms(stop, /*motor01,*/ (BLECharacteristic*)pvParameters);
}






