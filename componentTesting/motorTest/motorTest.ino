#include "var.hpp"

// Motors //////////////////////////////////////////////////////////////////////////////////////////////////
/*
const uint8_t leftWheelPin1 = 7;
const uint8_t leftWheelPin2 = 6;
const uint8_t leftPWMpin = 5;

const uint8_t rightWheelPin1 = 4;
const uint8_t rightWheelPin2 = 3;
const uint8_t rightPWMpin = 2;
*/
const uint8_t leftWheelPin1 = 4;
const uint8_t leftWheelPin2 = 5;
const uint8_t leftPWMpin = 6;

const uint8_t rightWheelPin1 = 8;
const uint8_t rightWheelPin2 = 9;
const uint8_t rightPWMpin = 10;

VAR_HPP_::motor left(leftWheelPin1, leftWheelPin2, leftPWMpin);
VAR_HPP_::motor right(rightWheelPin1,rightWheelPin2,rightPWMpin);


short out = 0;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
uint16_t inputTime = 1000;

void setup() {
  Serial.begin(9600);
  Serial.println("Ready");
  
  // Motors ////////////////////////////////////////////////////////////////////////////////
  pinMode(leftWheelPin1, OUTPUT);
  pinMode(leftWheelPin2, OUTPUT);
  pinMode(leftPWMpin, OUTPUT);

  pinMode(rightWheelPin1, OUTPUT);
  pinMode(rightWheelPin2, OUTPUT);
  pinMode(rightPWMpin, OUTPUT);

}

void loop() {
  if(currentMillis - previousMillis >= inputTime){
      if(Serial.available())
        out = readInput();
      Serial.print("OUT: ");
      Serial.println(out);
      if(out == 999){
        left.brake();
        right.brake();
      }
      else{
      left.drive(out);
      right.drive(out);
      }
      previousMillis = currentMillis;
  }
  
  currentMillis = millis();
}

short readInput(){
  return Serial.readStringUntil('\n').toInt();
}
