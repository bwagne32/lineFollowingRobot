#include <stdint.h>
// PID loop controls
#ifndef PID_HPP_
#define PID_HPP_

#include "comms.hpp"
#include "motorclass.hpp"
#include <QTRSensors.h>
#include "motorclass.hpp"

// QT sensors setup //////////////////////////////////////////////////////////////////////////////////////////////////
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t position;  // 0-7000


// PID SETUP //////////////////////////////////////////////////////////////////////////////////////////////////


int output;   // The output value of the controller to be converted to a ratio for turning
short error;  // Setpoint minus measured value
// *******************************************
const float Kp = .2;  // Proportional constant
const float Ki = .09;   // Integral constant
const float Kd = .09;   // Derivative constant
// *******************************************
bool clamp = 0;     // = 0 if we are not clamping and = 1 if we are clamping
bool iClamp;        // Prevents integral windup.  If 0 then continue to add to integral
bool signsEqual;    // = 1 if error and output have the same sign
float iError = 0.;  // Integral of error
float dError = 0;
float prevError1 = 0;
float prevError2 = 0;

int currentTime;
int previousTime = 0;
int timeDiff;

// Line follower Motor Control **************************************************************************************

int motorNominalSpeed = 40;  // 0 to 255
int calculatedTurnSpeed;

float sensingRatio;
float turnRatio;

int setpoint = 3500;  // sets target position for controller. Theoretically middle of bot

// ******************************************************************************************************************

void updateTime(); // 1st
void calcPID(); // 2nd (includes output)
void calcError(); // inside PID
void calcIntegral(); // inside PID
void calcDerivative(); // inside PID
void updateOutput(motorclass_h::motor &left, motorclass_h::motor &right); // 3rd


void killSwitch(bool &stop, motor &left, motor &right) {
  while (stop) {  // There's some infinite loop protection so I have to fight it like this
    left.brake();
    right.brake();
    right.outputToDrive();
    left.outputToDrive();
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("dead");
    delay(100000000);
  }
}

void loopPID(bool &stop, motorclass_h::motor &left, motorclass_h::motor &right) {
  Serial.println("car");
  left.direction(true);
  right.direction(true);

  while (true) {
    updateTime();
    //if(timeDiff > 100){
    calcPID();
    ////Serial.print("output: ");
    ////Serial.println(output);
    //delay(250);
    //Serial.print("Position: ");
    //Serial.println(position);
    //Serial.print("Left: ");
    //left.printOut();
    //Serial.print("Right: ");
    //right.printOut();
    Serial.println(output);
    updateOutput(left,right);
    //delay(100);
  
    
    if (stop)
      killSwitch(stop, left, right);
  }
}


// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

void calcPID(){ // Runs through P code, I code, and D code and generates an output signal
  calcError();
  //Serial.println(error);
  calcIntegral();
  calcDerivative();
  ///// CALC. OUTPUT /////////////////////////////////////////////////////////////////////////////////////////////////

    output = Kp * error + Ki * iError + Kd * dError;  // Calculate Output Command

    // Clamp output from 0 to 255 // For use with integrator clamping
    if (output > 500 || output < -500) {
      clamp = 1;

    } else {

      clamp = 0;
    }
} 

void calcError(){
  // Error calc ///////////////////////////////////////////////////////////////////
    //uint16_t posArray[5];
    for(int i = 0; i < 5; i++)
      position += qtr.readLineBlack(sensorValues);
    position = position / 5;

      //posArray[i] = qtr.readLineBlack(sensorValues);
    
    //position = ;
    error = setpoint - position;
} // inside PID

void calcIntegral(){
  ///// INTEGRAL ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Stop adding error for integral when output is clamped signs match for current error and accumulated error
    if (clamp == 1 && ((error >= 0 && output >= 0) || (error <= 0 && output <= 0))) {

      iClamp = 1;  // variable to tell if we need to stop adding to iError

    } else {
      iClamp = 0;
    }

    // Evaluate Integral of PI Controller unless we need to clamp it
    if (iClamp == 0) {

      iError += error * ( timeDiff * .01);

    } else {

      //stop adding error
    }

} // inside PID

void calcDerivative(){
  ///// DERIVATIVE ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculate Derivative

    dError = (error - prevError2) / (2 * timeDiff * .01);
    prevError2 = prevError1;
    prevError1 = error;
} // inside PID

void updateTime(){
  previousTime = currentTime;
  currentTime = millis();
  timeDiff = currentTime - previousTime;
}

void updateOutput(motorclass_h::motor &left, motorclass_h::motor &right){
  ///// ROBOT TURNING //////////////////////////////////////////////////////////////////////////////////////////////////
    //  NEW CODE FOR LINE FOLLOWER ************************************************************************

    // set motor direction and nominal speed (must be 8 bit integers)
    right.speed(motorNominalSpeed);
    left.speed(motorNominalSpeed);


    if (output < 0) {
      // if robot is left of line

      // Calculate sensing ratio
      sensingRatio = (-1 * output) / 1000.;

      // calculate turn speed , calculate turn ratio, and truncate data to 8 bit integer
      turnRatio = 1. - sensingRatio;
      calculatedTurnSpeed = motorNominalSpeed * turnRatio;

      // set speed of right motor to execute turn
      right.speed(calculatedTurnSpeed);

    } else if (output > 0) {
      // if robot is right of line

      // Calculate sensing ratio
      sensingRatio = (1 * output) / 1000.;

      // calculate turn speed , calculate turn ratio, and truncate data to 8 bit integer
      turnRatio = 1. - sensingRatio;
      calculatedTurnSpeed = motorNominalSpeed * turnRatio;

      // set speed of left motor to execute turn
      left.speed(calculatedTurnSpeed);
    }


    // Output to drive

    right.outputToDrive();
    left.outputToDrive();
}


#endif  // PID_HPP_