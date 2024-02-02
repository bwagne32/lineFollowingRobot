// PID loop controls
#ifndef PID_HPP_
#define PID_HPP_

#include "comms.hpp"
#include "var.hpp"
#include <QTRSensors.h>

// QT sensors setup //////////////////////////////////////////////////////////////////////////////////////////////////
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t position;


// PID //////////////////////////////////////////////////////////////////////////////////////////////////
const short kp = 0;
const short kd = 0; 


using VAR_HPP_::motor;
/* Motor info
      Use drive(int signal)
      // receives input -255 to 255 for motor speed (0 for coast, negative for reverse, and positive for forward)
*/

void loopPID(bool &stop, motor &left, motor &right){
  while(!stop){ // bluetooth communication controls stop
    // Do your PID loop in here
    


  }

}



void PID(){// Cole stuff


//AHHHHHH PLEASE HELP MEEEE
}












// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors ////////////////////////////////////

void read(){
  position = qtr.readLineBlack(sensorValues);
}



#endif // PID_HPP_