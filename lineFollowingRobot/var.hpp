// PID loop controls
#ifndef VAR_HPP_
#define VAR_HPP_
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

// PID //////////////////////////////////////////////////////////////////////////////////////////////////
const short kp = 50;
const short kd = 70; // documenntation recommeds having kd higher than kp





#endif // VAR_HPP_