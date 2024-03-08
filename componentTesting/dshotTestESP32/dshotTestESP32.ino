#include "DShotRMT.h"

// Dshot //////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t fanPin = 12;
const uint8_t fanEnablePin = 8;
const auto DSHOT_MODE = DSHOT300;
const auto FAILSAFE_THROTTLE = 999;
const auto INITIAL_THROTTLE = 48;

short out = 0;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
uint16_t inputTime = 1000;

// Initialize a DShotRMT object for the motor
DShotRMT motor01(fanPin, RMT_CHANNEL_0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Ready");
  pinMode(fanPin, OUTPUT);
  pinMode(fanEnablePin, OUTPUT);


  digitalWrite(fanEnablePin, HIGH);
  motor01.begin(DSHOT_MODE);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(currentMillis - previousMillis >= inputTime){
      if(Serial.available())
        out = readThrottle();
      Serial.print("OUT: ");
      Serial.println(out);
      motor01.sendThrottleValue(out);
      previousMillis = currentMillis;
  }

  currentMillis = millis();
}

uint16_t readThrottle(){
        return Serial.readStringUntil('\n').toInt();
}