#include <stdint.h>
#include <cmath>
#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
// PID loop controls
#ifndef motorclass_h
#define motorclass_h

// Motor class //////////////////////////////////////////////////////////////////////////////////////////////////
/*
PWM   in1    in2
0     0       0     Coast
0     1       1     brake
1     0       0     Coast
1     1       0     Forward
1     0       1     Reverse
1     1       1     brake

*/

class motor {
  // class for control of each motor
  public:

    motor(const uint8_t* IN1_pin, const uint8_t* IN2_pin, const uint8_t* PWM_pin);

    void coast();
    void brake();
    void direction(bool fwdRev);
    void speed(int desiredSpeed);
    Void outputToDrive();
  private:

    // pins
    uint8_t* _IN1_pin;
    uint8_t* _IN2_pin;
    uint8_t* _PWM_pin;

    // values for pins
    uint8_t _IN1_value;
    uint8_t _IN2_value;
    uint8_t _PWM_value;

}
/*
class motor {// I'm too lazy to make an actual cpp file for this so it'll be somewhat cramped
public:
  motor (const uint8_t& in1, const uint8_t& in2, const uint8_t& pwm){ in1_ = &in1; in2_ = &in2; pwm_ = &pwm; };
  
  void brake(){
    manual(0,true,true);
  }

  void drive(int signal){// receives input -255 to 255 for motor speed (0 for coast, negative for reverse, and positive for forward)
    if (signal < 0) // reverse
      manual(signal, false, true);
    else // forward
      manual(signal, true, false); 
  };

  void manual(short signal, bool in1, bool in2){
    short output;
    signal = std::abs(signal);
    if(signal > 255)
      output = 255;
    else
      output = signal;
    analogWrite(*pwm_, output);
    digitalWrite(*in1_, in1);
    digitalWrite(*in2_, in2);
  }

private: // pointers to const pin references
  const uint8_t *pwm_;
  const uint8_t *in1_;
  const uint8_t *in2_;
};


*/
#endif // VAR_HPP_