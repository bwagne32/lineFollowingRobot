// PID loop controls
#ifndef PID_HPP_
#define PID_HPP_

#include "comms.hpp"
#include <QTRSensors.h>

// QT sensors setup //////////////////////////////////////////////////////////////////////////////////////////////////
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t position;


// PID //////////////////////////////////////////////////////////////////////////////////////////////////
const short kp = 0;
const short kd = 0; 

/* Motor info
      Use drive(int signal)
      // receives input -100 to 100 for motor speed (0 for coast, negative for reverse, and positive for forward)
*/

void loop(){
  while(1){ // Do your PID loop in here



  }

}



void PID(){// Cole stuff

}
















// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

// Sensors ////////////////////////////////////

void read(){
  position = qtr.readLineBlack(sensorValues);
}



#endif // PID_HPP_