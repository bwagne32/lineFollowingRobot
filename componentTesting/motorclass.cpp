#include "arduino.h"
#include "motorclass.hpp"


// Cole's motor class //////////////////////////////////////////////////////////////////////////////////////////////////
motor::motor(const uint8_t& in1, const uint8_t& in2, const uint8_t& pwm, const uint8_t speed){ 
  _IN1_pin = &in1;
  _IN2_pin = &in2;
  _PWM_pin = &pwm;

  _IN1_value = 0;
  _IN2_value = 0;
  _PWM_value = 0;

  maxSpeed_ = speed;
}


void motor::coast(){
// function commands drive to let motor coast
  _IN1_value = 0;
  _IN2_value = 0;
}

void motor::brake(){
// function commands drive to brake motor:
  _IN1_value = 1;
  _IN2_value = 1;
}

void motor::direction(bool fwdRev){
// function sets direction of motor
// False= Forward. True = Reverse

  if(fwdRev){
    //motor Reverse

    _IN1_value = 1;
    _IN2_value = 0;
    
  }else{
    //motor Forward

    _IN1_value = 0;
    _IN2_value = 1;
  }
}

void motor::speed(int desiredSpeed){
  // funciton sets the desired PWM value

  uint8_t cappedSpeed; // Value within arduino PWM range;

  if(desiredSpeed > maxSpeed_){
    // check for overflow

    cappedSpeed = maxSpeed_;

  }else if (desiredSpeed < 0){
    // check for negative

    cappedSpeed = 0;

  }else{
    // good input

    cappedSpeed = desiredSpeed;
  }

  if(desiredSpeed < 0) // reverse
    direction(0);
  else
    direction(1);
    
  _PWM_value = cappedSpeed;

}

void motor::outputToDrive(){
  // outputs values to output pins connected to motor drive for specific motor

  digitalWrite(*_IN1_pin , _IN1_value);
  digitalWrite(*_IN2_pin , _IN2_value);
  analogWrite(*_PWM_pin , _PWM_value);


}

