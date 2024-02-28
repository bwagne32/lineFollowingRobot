#include "arduino.h"
#include "motorclass.h"

motor::motor(const uint8_t* IN1_pin, const uint8_t* IN2_pin, const uint8_t* PWM_pin){

  _IN1_pin= IN1_pin;
  _IN2_pin= IN2_pin;
  _PWM_pin= PWM_pin;

  pinmode(*_IN1_pin, OUTPUT);
  pinmode(*_IN2_pin, OUTPUT);
  pinmode(*_PWM_pin, OUTPUT);

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

void motor::direction(bool fwdRev);{
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

void speed(int desiredSpeed){
  // funciton sets the desired PWM value

  int cappedSpeed; // Value within arduino PWM range;

  if(desiredSpeed > 255){
    // check for overflow

    cappedSpeed = 255;

  }else if (desiredSpeed < 0){
    // check for negative

    cappedSpeed = 0;

  }else{
    // good input

    cappedSpeed = desiredSpeed;
  }

  _PWM_value = cappedSpeed;

}

void outputToDrive(){
  // outputs values to output pins connected to motor drive for specific motor

  digitalWrite(*_IN1_pin , _IN1_value);
  digitalWrite(*_IN2_pin , _IN2_value);
  analogWrite(*_PWM_pin , _PWM_value);


}