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
<<<<<<< Updated upstream
uint16_t position;                            // 0-7000
=======
uint16_t position;  // 0-5000
>>>>>>> Stashed changes


// PID SETUP //////////////////////////////////////////////////////////////////////////////////////////////////


<<<<<<< Updated upstream
int output;                                   // The output value of the controller to be converted to a ratio for turning
float error;                                  // Setpoint minus measured value
=======
int output;          // The PWM output value
float output2 = 0.;  // Temporary value
float error;         // Setpoint minus measured value
>>>>>>> Stashed changes
// *******************************************
const float Kp = 50.;  // Proportional constant
const float Ki = .5;   // Integral constant
const float Kd = 5.;   // Derivative constant
// *******************************************
<<<<<<< Updated upstream
bool clamp = 0;                               // = 0 if we are not clamping and = 1 if we are clamping
bool iClamp;                                  // Prevents integral windup.  If 0 then continue to add to integral
bool signsEqual;                              // = 1 if error and output have the same sign
float iError = 0.;                            // Integral of error
=======
long unclampedOutput;  // Output prior to clamping between 0 and 255 inclusively
bool clamp = 0;        // = 0 if we are not clamping and = 1 if we are clamping
bool iClamp;           // Prevents integral windup.  If 0 then continue to add to integral
bool signsEqual;       // = 1 if error and output2 have the same sign
float iError = 0.;     // Integral of error
>>>>>>> Stashed changes
float dError = 0;
float prevError1 = 0;
float prevError2 = 0;

int currentTime = 0;
int previousTime = 0;
short timeDiff;


// Line follower Motor Control **************************************************************************************

int motorNominalSpeed = 190;  // 0 to 255
int calculatedTurnSpeed;

short float sensingRatio;
short float turnRatio;

<<<<<<< Updated upstream
int setpoint = 3500;                  // sets target position for controller. Theoretically middle of bot
=======
int setpoint = 2500;  // sets target position for controller. Theoretically middle of bot
>>>>>>> Stashed changes

// ******************************************************************************************************************

void killSwitch(bool &stop, motor &left, motor &right) {
  while (stop) {  // There's some infinite loop protection so I have to fight it like this
    left.brake();
    right.brake();
    delay(100000000);
  }
}

void errorCalc(); 
void outputCalc();
    
<<<<<<< Updated upstream
    ///// INTEGRAL ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Stop adding error for integral when output is clamped signs match for current error and accumulated error
  if (clamp == 1 && ( (error >= 0 && output >= 0)||(error <= 0 && output <= 0))){
=======

void loopPID(bool &stop, motor &left, motor &right) {
  while (true) {
>>>>>>> Stashed changes
    
    errorCalc(); 

    previousTime = currentTime;
    currentTime = millis();
    timeDiff = currentTime - previousTime;

    outputCalc(); // 
    //  NEW CODE FOR LINE FOLLOWER ************************************************************************

<<<<<<< Updated upstream
  }else {

    //stop adding error
    
  }

    ///// DERIVATIVE ///////////////////////////////////////////////////////////////////////////////////////////////////
  // Calculate Derivative

  dError= (error - prevError2) / (2*runControllerTime * .01);
  prevError2 = prevError1;
  prevError1 = error;

    ///// CALC. OUTPUT /////////////////////////////////////////////////////////////////////////////////////////////////
 
  output = Kp*error+ 0.5 + Ki*iError + Kd*dError; // Calculate Output Command

  // Clamp output from 0 to 255 // For use with integrator clamping
    if(output > 1000 || output < -1000){
    clamp = 1;

  }else{

    clamp = 0;
  }

    ///// ROBOT TURNING ////////////////////////////////////////////////////////////////////////////////////////////////// 
//  NEW CODE FOR LINE FOLLOWER ************************************************************************

  // set motor direction and nominal speed (must be 8 bit integers)
=======
    // set motor direction and nominal speed (must be 8 bit integers)
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

    }else if (output > 0){
=======
    }

    if (output > 0) {
>>>>>>> Stashed changes
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


    if (stop)
      killSwitch(stop, left, right);
  }
}


void errorCalc(){ // Checks sensors against setpoint
  position = qtr.readLineBlack(sensorValues);
  error = setpoint - position;
}


void outputCalc(){ // Calculates the PID output and scales output to -1000 to 1000
  unclampedOutput = Kp * error + Ki * iError;

  dError = (error - prevError2) / (2 * timeDiff * .01);
  prevError2 = prevError1;
  prevError1 = error;


  signsEqual = (((error) < 0) == ((output2) < 0));
  clamp = (output2 > 1000) || (output2 < -1000);
  iClamp = clamp && signsEqual;
  
  iError = iError + error * timeDiff * !iClamp; 
  output2 = Kp * error + Ki * iError + Kd * dError;


  output = output2 / 1000;
  output = (output > 1000) ? 1000 : ((output < -1000) ? -1000 : output); // Out of bounds protection


/*



<<<<<<< Updated upstream
// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors ////////////////////////////////////
// WTF is this> -cole
void read(){
  position = qtr.readLineBlack(sensorValues);
=======
    // Clamp output2 from 0 to 255 // For use with integrator clamping
    if (output2 > 1000 || output2 < -1000) {
      clamp = 1;

    } else {

      clamp = 0;
    }

    // Master clamp / limit for PWM output so that we cannot output an unachievable command
    if (output2 > 1000) output2 = 1000;
    if (output2 < -1000) output2 = -1000;
    output = output2;
    */
>>>>>>> Stashed changes
}




#endif  // PID_HPP_