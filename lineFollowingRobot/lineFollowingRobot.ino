#include <QTRSensors.h>

// QT sensors setup //////////////////////////////////////////////////////////////////////////////////////////////////
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t position;

// Motors //////////////////////////////////////////////////////////////////////////////////////////////////
const short leftWheelPin1 = 4;
const short leftWheelPin2 = 5;

const short rightWheelPin1 = 6;
const short rightWheelPin2 = 7;

void left(short,short);
void right(short,short);

// PID //////////////////////////////////////////////////////////////////////////////////////////////////
const short kp = 50;
const short kd = 70; // documenntation recommeds having kd higher than kp







void setup() {
  pinMode(leftWheelPin1, OUTPUT);
  pinMode(rightWheelPin1, OUTPUT);
  pinMode(leftWheelPin2, OUTPUT);
  pinMode(rightWheelPin2, OUTPUT);

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){  9, 10, 11, 12,14,15,16,17 }, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 200; i++) {
    left(128,0);
    right(0,128);
    qtr.calibrate();
    left(0,128);
    right(128,0);
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);  // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}


void loop() { //////////////////////////////////////////////////////////////////////////////////////////////////
  position = qtr.readLineBlack(sensorValues);


}













// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

/*
DRV8833 Dual H-Bridge Motor Driver
IN1    IN2    FUNCTION
0      0      Coast, fast decay

PWM    0      Forward
1      PWM    Forward, slow decay

0      PWM    Reverse, fast decay
PWM    1      Reverse, slow decay


*/

void left(short in1, short in2){
  analogWrite(leftWheelPin1, in1);
  analogWrite(leftWheelPin2, in2);
}

void right(short out1, short out2){
  analogWrite(rightWheelPin1, in1);
  analogWrite(rightWheelPin2, in2);
}

