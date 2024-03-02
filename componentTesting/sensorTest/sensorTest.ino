/* References
  QTR sensor: https://github.com/pololu/qtr-sensors-arduino/blob/master/examples/QTRAExample/QTRAExample.ino


*/

#include <QTRSensors.h>


QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t position;


unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
uint16_t inputTime = 500; 


void setup() {
  Serial.begin(115200);
  Serial.println("Ready");
  
  // QTR sensors ////////////////////////////////////////////////////////////////////////////////
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){  A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode



  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration



  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

}

void loop() {
  if(currentMillis - previousMillis >= inputTime){
      position = qtr.readLineBlack(sensorValues); // Range 0-7000
      Serial.print("Position: ");
      Serial.println(position);
      previousMillis = currentMillis;
  }
  
  currentMillis = millis();
}
