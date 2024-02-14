// Bluetooth function controls
#ifndef COMMS_HPP_
#define COMMS_HPP_

#include "PID.hpp"
#include "DShotRMT.h"

// https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial

void loopComms(bool& stop, DShotRMT& motor01){
  motor01.sendThrottleValue(50); // idk what to set this to. Will figure out with testing

  while(1){

    //main::stop = 1;

    
  }
}


#endif // COMMS_HPP_