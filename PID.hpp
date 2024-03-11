#include <stdint.h>
#include "Arduino.h"
#include <cmath>
//#include <stdint.h>
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


int output = 0;   // The output value of the controller to be converted to a ratio for turning
short error = 0;  // Setpoint minus measured value
// *******************************************
const float Kp = 1.;  //  1 //Proportional constant
const float Ki = .001;  //.001 // Integral constant
const float Kd = 7. ;  // 6.5// Derivative constant
// *******************************************
bool clamp = 1;     // = 0 if we are not clamping and = 1 if we are clamping
bool iClamp = 1;        // Prevents integral windup.  If 0 then continue to add to integral
bool signsEqual;    // = 1 if error and output have the same sign
float iError = 0.;  // Integral of error
float dError = 0;
float prevError1 = 0;
float prevError2 = 0;

int currentTime = 0;
int previousTime = 0;
float timeDiff = 2;

// Line follower Motor Control **************************************************************************************

int motorNominalSpeed;  // 0 to 255
int calculatedTurnSpeed;

float sensingRatio;
float turnRatio;

int setpoint = 3500;  // sets target position for controller. Theoretically middle of bot

// Functions prototypes //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PID ////////////////////////////////////////////////////////////////
void calcPID(motorclass_h::motor &left, motorclass_h::motor &right); // Takes in error and calculates P,I,and D values for desire output
void calcError(motorclass_h::motor &left, motorclass_h::motor &right);  // Calculates difference from setpoint
                                                                        // Includes protection from losing line
                                                                        // inside calcPID
void calcIntegral();  // PID integral calculation
                      // Inside calcPID
void calcDerivative(); // PID Derivative calculation
                       // Inside calcPID
void updateOutput(motorclass_h::motor &left, motorclass_h::motor &right); // Calculates motor ratio for turning and sends output to motors

// Helper functions ////////////////////////////////////////////////////////////////
//void updateTime(); // Updates controller time for calcIntegral and calcDerivative
bool checkIfLost(); // checks sensor values to see if line is lost
                    // Runs inside of calcError();
float turnCurve(const float& ratio, const uint8_t& speed); // output turn curve
// Misc ////////////////////////////////////////////////////////////////
void killSwitch(bool&, motor&, motor&); // Locks program into infinite loop to let us retrieve the bot








// Main loop ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loopPID(bool &stop, motorclass_h::motor &left, motorclass_h::motor &right, uint8_t &maxMotorSpeed) {
  ////Serial.println("car");
  
  motorNominalSpeed = maxMotorSpeed;

  left.direction(true);
  right.direction(true);

  while (true) {
    //updateTime();
    ////Serial.println(timeDiff);
    currentTime = millis();
    if(currentTime - previousTime >= timeDiff){
      calcPID(left,right);
      //Serial.println(output);
      ////Serial.println("After PID");
      updateOutput(left,right);
      //Serial.println("After out");
      previousTime = currentTime;
      //Serial.flush();
    }
    delay(1);
  
    if (stop)
      killSwitch(stop, left, right);
  }
}


// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

void calcPID(motorclass_h::motor &left, motorclass_h::motor &right){ // Runs through P code, I code, and D code and generates an output signal
  calcError(left, right);
  //Serial.println(error);
  //calcIntegral();
  calcDerivative();
  ///// CALC. OUTPUT /////////////////////////////////////////////////////////////////////////////////////////////////

    output = Kp * error + Ki * iError + Kd * dError;  // Calculate Output Command

    // Clamp output from 0 to 255 // For use with integrator clamping
    if (output > 300 || output < -300) {
      clamp = 1;

    } else {

      clamp = 0;
    }
} 




void calcError(motorclass_h::motor &left, motorclass_h::motor &right){
  // Error calc ///////////////////////////////////////////////////////////////////
    //Serial.println("error");
    position = qtr.readLineBlack(sensorValues);

/*
    while(checkIfLost()){ // Finding line again if lost
      //Serial.println("lost line");
      if(prevError1 > 0){       //Turn left if the line was to the left before
        left.speed(0);
        right.speed(right.maxSpeed() >> 1); // multiply by 3 shift right 1 (divide by 2)
      }
      else{// turn right
        right.speed(0);
        left.speed(left.maxSpeed() >> 1);
      }
      left.outputToDrive();
      right.outputToDrive();
      position = qtr.readLineBlack(sensorValues);
    }*/
    error = setpoint - position;
} // inside PID

void calcIntegral(){
  ///// INTEGRAL ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Stop adding error for integral when output is clamped signs match for current error and accumulated error
    //Serial.println("integral");
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
  //Serial.println("Derivative");
  ///// DERIVATIVE ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculate Derivative
    dError = (error - prevError2) / (2 * timeDiff * .01);

    if(dError > 1000)
      dError = 1000;
    else if(dError < -1000)
      dError = -1000;

    prevError2 = prevError1;
    prevError1 = error;
} // inside PID

/*
void updateTime(){
  previousTime = currentTime;
  currentTime = millis();
  timeDiff = ceil(currentTime - previousTime);
}
*/

void updateOutput(motorclass_h::motor &left, motorclass_h::motor &right){
  ///// ROBOT TURNING //////////////////////////////////////////////////////////////////////////////////////////////////
    //  NEW CODE FOR LINE FOLLOWER ************************************************************************
    //Serial.println("Output");
    // set motor direction and nominal speed (must be 8 bit integers)
    right.speed(motorNominalSpeed);
    left.speed(motorNominalSpeed);


    if (output < 0) {
      // if robot is left of line

      // Calculate sensing ratio
      sensingRatio = (-1 * output) / 1000.;

      // calculate turn speed , calculate turn ratio, and truncate data to 8 bit integer
      turnRatio = 1. - sensingRatio; // 0.-1.
      
      if(turnRatio > .6){
        right.speed(int(turnCurve(turnRatio, motorNominalSpeed)));
        left.speed(int(turnCurve(turnRatio, motorNominalSpeed) + .3 * motorNominalSpeed));
        
        //right.speed(round(right.maxSpeed() * 1 / 2) * turnRatio);
        //left.speed(left.maxSpeed() * 1 / 2);
      }
      /*else if(turnRatio > .7){
        right.speed(round(right.maxSpeed() * 2 / 3) * turnRatio);
        left.speed(left.maxSpeed() * 2 / 3);
      }
      else if(turnRatio > .6){
        right.speed(round(right.maxSpeed() * 3 / 4) * turnRatio);
        left.speed(left.maxSpeed() * 3 / 4);
      }*/
      else{
       calculatedTurnSpeed = motorNominalSpeed * turnRatio;
        // set speed of right motor to execute turn
        right.speed(calculatedTurnSpeed);
        }

    } else if (output > 0) {
      // if robot is right of line

      // Calculate sensing ratio
      sensingRatio = (1 * output) / 1000.;

      // calculate turn speed , calculate turn ratio, and truncate data to 8 bit integer
      turnRatio = 1. - sensingRatio;
      

      if(turnRatio > .6){
        left.speed(int(turnCurve(turnRatio, motorNominalSpeed)));
        right.speed(int(turnCurve(turnRatio, motorNominalSpeed) + .3 * motorNominalSpeed));
        
        //left.speed(round(left.maxSpeed() * 1 / 2) * turnRatio);
        //right.speed(right.maxSpeed() * 1 / 2);
      }
      /*else if(turnRatio > .7){
        left.speed(round(left.maxSpeed() * 2 / 3) * turnRatio);
        right.speed(right.maxSpeed() * 2 / 3);
      }
      else if(turnRatio > .6){
        left.speed(round(left.maxSpeed() * 3 / 4) * turnRatio);
        right.speed(right.maxSpeed() * 3 / 4);
      }*/
      else{
        calculatedTurnSpeed = motorNominalSpeed * turnRatio;
        //Serial.println(turnRatio);
        // set speed of left motor to execute turn
        left.speed(calculatedTurnSpeed);
      }
    } else{
      left.speed(motorNominalSpeed * .7);
      right.speed(motorNominalSpeed * .7);
    }


    // Output to drive

    right.outputToDrive();
    left.outputToDrive();
}

bool checkIfLost(){
  return ((sensorValues[0]>=980) && 
          (sensorValues[1]>=980) && 
          (sensorValues[2]>=980) && 
          (sensorValues[3]>=980) && 
          (sensorValues[4]>=980) && 
          (sensorValues[5]>=980) && 
          (sensorValues[6]>=980) && 
          (sensorValues[7]>=980)); };

float turnCurve(const float& ratio, const uint8_t& speed){
  return .9 * speed * sin(5 * ratio - 1) / (2 * ratio) + .1 * speed;
}


void killSwitch(bool &stop, motor &left, motor &right) {
  while (stop) {  // There's some infinite loop protection so I have to fight it like this
    left.brake();
    right.brake();
    right.outputToDrive();
    left.outputToDrive();
    digitalWrite(LED_BUILTIN, HIGH);
    ////Serial.print("dead");
    delay(100000000);
  }
}

#endif  // PID_HPP_