// PID loop controls
#ifndef PID_HPP_
#define PID_HPP_

#include "comms.hpp"

void PID(){//

}
















// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors ////////////////////////////////////

void read(){
  position = qtr.readLineBlack(sensorValues);
}

// Motors /////////////////////////////////////
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

void right(short in1, short in2){
  analogWrite(rightWheelPin1, in1);
  analogWrite(rightWheelPin2, in2);
}


#endif // PID_HPP_