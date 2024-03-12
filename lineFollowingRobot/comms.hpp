// Bluetooth function controls
#ifndef COMMS_HPP_
#define COMMS_HPP_

#include "PID.hpp"
#include "DShotRMT.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

void killSwitch(bool &stop/*, DShotRMT& motor01*/){
  while(stop){// There's some infinite loop protection so I have to fight it like this
      //motor01.sendThrottleValue(0);
      delay(100000000);
    } 
}

void loopComms(bool& stop, /*DShotRMT& motor01,*/ BLECharacteristic *pCharacteristic){
  //motor01.sendThrottleValue(50); // idk what to set this to. Will figure out with testing
  //Serial.println("comms");
  long idk = 0;
  while(1){
    idk++;
    idk--;

    
  }
}


#endif // COMMS_HPP_