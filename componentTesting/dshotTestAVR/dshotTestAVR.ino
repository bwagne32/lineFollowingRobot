#include "DShotRMT.h"

/*

redefine DSHOT_PORT if you want to change the default PORT

Defaults
UNO: PORTD, available pins 0-7 (D0-D7)
Leonardo: PORTB, available pins 4-7 (D8-D11)

e.g.
#define DSHOT_PORT PORTD
*/
DShot esc1(DShot::Mode::DSHOT150);

uint16_t throttle = 1500;
uint16_t target = 600;
const uint8_t escEnable = 5;
const uint8_t escPin = 7;
//const uint8_t enableControl = 8;
//bool enabled = 0;

void setup() {
  //Serial.begin(9600);
  pinMode(escEnable, OUTPUT);
  digitalWrite(escEnable, LOW);
  delay(5000);
  digitalWrite(escEnable, HIGH);
  // Notice, all pins must be connected to same PORT
  esc1.attach(escPin);  
  esc1.setThrottle(throttle);
}

void loop() {
    //delay(20);

  esc1.setThrottle(throttle);
  
/*

  if (Serial.available()>0){
    target = Serial.parseInt();
    if (target>2047)
      target = 2047;
    Serial.print(target, HEX);
    Serial.print("\t");
  }
  if (throttle<48){
    throttle = 48;
  }
  if (target<=48){
    esc1.setThrottle(target);
  }else{
    if (target>throttle){
      throttle ++;
      esc1.setThrottle(throttle);
    }else if (target<throttle){
      throttle --;
      esc1.setThrottle(throttle);
    }
  }*/
  delay(100);
}