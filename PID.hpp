#include <stdint.h>
// PID loop controls
#ifndef PID_HPP_
#define PID_HPP_

#include "comms.hpp"
#include "motorclass.hpp"
#include <QTRSensors.h>
#include <motorclass.h>

// QT sensors setup //////////////////////////////////////////////////////////////////////////////////////////////////
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t position;                            // 0-7000


// PID SETUP //////////////////////////////////////////////////////////////////////////////////////////////////


int output;                                   // The output value of the controller to be converted to a ratio for turning
float error;                                  // Setpoint minus measured value
// *******************************************
const float Kp = 50.;                         // Proportional constant 
const float Ki = .5;                          // Integral constant 
const float Kd = 5.;                          // Derivative constant
// *******************************************
bool clamp = 0;                               // = 0 if we are not clamping and = 1 if we are clamping
bool iClamp;                                  // Prevents integral windup.  If 0 then continue to add to integral
bool signsEqual;                              // = 1 if error and output have the same sign
float iError = 0.;                            // Integral of error
float dError = 0;
float prevError1 = 0;
float prevError2 = 0;


// Line follower Motor Control **************************************************************************************

int motorNominalSpeed = 190 ;         // 0 to 255
int calculatedTurnSpeed;

short float sensingRatio;
short float turnRatio;

int setpoint = 3500;                  // sets target position for controller. Theoretically middle of bot

// ******************************************************************************************************************

void killSwitch(bool &stop, motor &left, motor &right){
  while(stop){// There's some infinite loop protection so I have to fight it like this
      left.brake();
      right.brake();
      delay(100000000);
    } 
}

void loopPID(bool &stop, motor &left, motor &right){
  while(true){
    // Do your PID loop in here



    // Controller code here
 // Run controller

 // Calculate Error

  position = qtr.readLineBlack(sensorValues);

  error=setpoint-position;

    //Serial.println(clamp); // Used for Troubleshooting
    //delay(5000);
    
    ///// INTEGRAL ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Stop adding error for integral when output is clamped signs match for current error and accumulated error
  if (clamp == 1 && ( (error >= 0 && output >= 0)||(error <= 0 && output <= 0))){
    
    iClamp = 1; // variable to tell if we need to stop adding to iError
    
  }else {
    iClamp = 0;
  }

  // Evaluate Integral of PI Controller unless we need to clamp it
  if(iClamp == 0){

    iError+= error * (runControllerTime * .01);

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
    right.speed(motorNominalSpeed);
    left.speed(motorNominalSpeed);
  

    if (output < 0){
      // if robot is left of line

      // Calculate sensing ratio
      sensingRatio = (-1 * output) / 1000. ;

      // calculate turn speed , calculate turn ratio, and truncate data to 8 bit integer
      turnRatio = 1. - sensingRatio;
      calculatedTurnSpeed = motorNominalSpeed * turnRatio;

      // set speed of right motor to execute turn
      right.speed(calculatedTurnSpeed);

    }else if (output > 0){
      // if robot is right of line

      // Calculate sensing ratio
      sensingRatio = (1 * output) / 1000. ;

      // calculate turn speed , calculate turn ratio, and truncate data to 8 bit integer
      turnRatio = 1. - sensingRatio;
      calculatedTurnSpeed = motorNominalSpeed * turnRatio;

      // set speed of left motor to execute turn
      left.speed(calculatedTurnSpeed);

    }


    // Output to drive

    right.outputToDrive();
    left.outputToDrive();


    if(stop)
      killSwitch(stop, left, right);
  }

}


/*
void PID(){// Cole stuff


//AHHHHHH PLEASE HELP MEEEE
}

*/



// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors ////////////////////////////////////
// WTF is this> -cole
void read(){
  position = qtr.readLineBlack(sensorValues);
}




#endif // PID_HPP_