#include <stdint.h>
//#include <cmath>
//#include "esp32-hal-gpio.h"
//#include "esp32-hal.h"
// PID loop controls
#ifndef VAR_HPP_
#define VAR_HPP_

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
class motor {// I'm too lazy to make an actual cpp file for this so it'll be somewhat cramped
public:
  motor (const uint8_t& in1, const uint8_t& in2, const uint8_t& pwm){ in1_ = &in1; in2_ = &in2; pwm_ = &pwm; };
  
  void brake(){
    manual(0,true,true);
    Serial.println("BRAKE");
  }

  void drive(int signal){// receives input -255 to 255 for motor speed (0 for coast, negative for reverse, and positive for forward)
    if (signal < 0){ // reverse
      manual(signal, false, true);
      Serial.println("Reverse");
    }
    else{ // forward
      manual(signal, true, false); 
      Serial.println("Forward");
    }
  };

  void manual(short signal, bool in1, bool in2){
    short output;
    signal = abs(signal);
    if(signal > 255)
      output = 255;
    else
      output = signal;
    
    Serial.print("OUT:");
    Serial.println(output);

    analogWrite(*pwm_, output);
    digitalWrite(*in1_, in1);
    digitalWrite(*in2_, in2);
  }


private: // pointers to const pin references
  const uint8_t *pwm_;
  const uint8_t *in1_;
  const uint8_t *in2_;
};



#endif // VAR_HPP_