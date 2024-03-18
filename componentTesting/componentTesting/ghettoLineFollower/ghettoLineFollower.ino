/*
Our esp32 died :(
  this is emergency thrown together to test out PID
*/


#include <QTRSensors.h>


// Motors //////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t leftWheelPin1 = 4;
const uint8_t leftWheelPin2 = 5;
const uint8_t leftPWMpin = 6;

const uint8_t rightWheelPin1 = 8;
const uint8_t rightWheelPin2 = 9;
const uint8_t rightPWMpin = 10;

class motor {
  // class for control of each motor
  public:
    motor(const uint8_t&, const uint8_t&, const uint8_t&);
    void coast();
    void brake();
    void direction(bool fwdRev);
    void speed(int desiredSpeed);
    void outputToDrive();
    //void printOut(){//Serial.print("PWM: "); //Serial.println(_PWM_value);}
  private:
    // pins
    const uint8_t *_IN1_pin;
    const uint8_t *_IN2_pin;
    const uint8_t *_PWM_pin;
    // values for pins
    bool _IN1_value;
    bool _IN2_value;
    uint8_t _PWM_value;
};
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
uint16_t position;  // 0-7000


// PID SETUP //////////////////////////////////////////////////////////////////////////////////////////////////


int output;   // The output value of the controller to be converted to a ratio for turning
short error;  // Setpoint minus measured value
// *******************************************
const float Kp = .4;  // Proportional constant
const float Ki = 0.;   // Integral constant
const float Kd = 0.;   // Derivative constant
// *******************************************
bool clamp = 0;     // = 0 if we are not clamping and = 1 if we are clamping
bool iClamp;        // Prevents integral windup.  If 0 then continue to add to integral
bool signsEqual;    // = 1 if error and output have the same sign
float iError = 0.;  // Integral of error
float dError = 0;
float prevError1 = 0;
float prevError2 = 0;

int currentTime;
int previousTime = 0;
int timeDiff;

// Line follower Motor Control **************************************************************************************

int motorNominalSpeed = 40;  // 0 to 255
int calculatedTurnSpeed;

float sensingRatio;
float turnRatio;

int setpoint = 3000;  // sets target position for controller. Theoretically middle of bot


motor left(leftWheelPin1, leftWheelPin2, leftPWMpin);
motor right(rightWheelPin1, rightWheelPin2, rightPWMpin);

void setup(){
  //Serial.begin(9600);
  
  // Motors ////////////////////////////////////////////////////////////////////////////////
  pinMode(leftWheelPin1, OUTPUT);
  pinMode(leftWheelPin2, OUTPUT);
  pinMode(leftPWMpin, OUTPUT);

  pinMode(rightWheelPin1, OUTPUT);
  pinMode(rightWheelPin2, OUTPUT);
  pinMode(rightPWMpin, OUTPUT);

  left.direction(false);
  right.direction(false);
  

// QTR sensors ////////////////////////////////////////////////////////////////////////////////
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){  A0, A1, A2, A3, A4, A5 }, SensorCount);
  qtr.setEmitterPin(2);

  //delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

// analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  //Serial.println("ready");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration


  for (uint8_t i = 0; i < SensorCount; i++)
  {
    //Serial.print(qtr.calibrationOn.minimum[i]);
    //Serial.print(' ');
  }
  //Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    //Serial.print(qtr.calibrationOn.maximum[i]);
    //Serial.print(' ');
  }
  //Serial.println();
  //Serial.println();
}

void loop(){
  updateTime();
  //if(timeDiff > 100){
    calcPID();
    ////Serial.print("output: ");
    ////Serial.println(output);
    //delay(250);
    //Serial.print("Position: ");
    //Serial.println(position);
    //Serial.print("Left: ");
    //left.printOut();
    //Serial.print("Right: ");
    //right.printOut();
    
    updateOutput();
    delay(100);
    
 // }
}


// Functions //////////////////////////////////////////////////////////////////////////////////////////////////

void calcPID(){ // Runs through P code, I code, and D code and generates an output signal
  calcError();
  //Serial.println(error);
  //calcIntegral();
  //calcDerivative();
  ///// CALC. OUTPUT /////////////////////////////////////////////////////////////////////////////////////////////////

    output = Kp * error + 0.5 + Ki * iError + Kd * dError;  // Calculate Output Command

    // Clamp output from 0 to 255 // For use with integrator clamping
    if (output > 1000 || output < -1000) {
      clamp = 1;

    } else {

      clamp = 0;
    }
} 

void calcError(){
  // Error calc ///////////////////////////////////////////////////////////////////
    //uint16_t posArray[5];
    for(int i = 0; i < 5; i++)
      position += qtr.readLineBlack(sensorValues);
    position = position / 5;
      //posArray[i] = qtr.readLineBlack(sensorValues);
    
    //position = ;
    error = setpoint - position;
} // inside PID

void calcIntegral(){
  ///// INTEGRAL ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Stop adding error for integral when output is clamped signs match for current error and accumulated error
    if (clamp == 1 && ((error >= 0 && output >= 0) || (error <= 0 && output <= 0))) {

      iClamp = 1;  // variable to tell if we need to stop adding to iError

    } else {
      iClamp = 0;
    }

    // Evaluate Integral of PI Controller unless we need to clamp it
    if (iClamp == 0) {

      iError += error * ( timeDiff * .01);

    } else {

      //stop adding error
    }

} // inside PID

void calcDerivative(){
  ///// DERIVATIVE ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculate Derivative

    dError = (error - prevError2) / (2 * timeDiff * .01);
    prevError2 = prevError1;
    prevError1 = error;
} // inside PID

void updateTime(){
  previousTime = currentTime;
  currentTime = millis();
  timeDiff = currentTime - previousTime;
}

void updateOutput(){
  ///// ROBOT TURNING //////////////////////////////////////////////////////////////////////////////////////////////////
    //  NEW CODE FOR LINE FOLLOWER ************************************************************************

    // set motor direction and nominal speed (must be 8 bit integers)
    right.speed(motorNominalSpeed);
    left.speed(motorNominalSpeed);


    if (output < 0) {
      // if robot is left of line

      // Calculate sensing ratio
      sensingRatio = (-1 * output) / 1000.;

      // calculate turn speed , calculate turn ratio, and truncate data to 8 bit integer
      turnRatio = 1. - sensingRatio;
      calculatedTurnSpeed = motorNominalSpeed * turnRatio;

      // set speed of right motor to execute turn
      right.speed(calculatedTurnSpeed);

    } else if (output > 0) {
      // if robot is right of line

      // Calculate sensing ratio
      sensingRatio = (1 * output) / 1000.;

      // calculate turn speed , calculate turn ratio, and truncate data to 8 bit integer
      turnRatio = 1. - sensingRatio;
      calculatedTurnSpeed = motorNominalSpeed * turnRatio;

      // set speed of left motor to execute turn
      left.speed(calculatedTurnSpeed);
    }


    // Output to drive

    right.outputToDrive();
    left.outputToDrive();
}


motor::motor(const uint8_t& in1, const uint8_t& in2, const uint8_t& pwm){ 
  _IN1_pin = &in1;
  _IN2_pin = &in2;
  _PWM_pin = &pwm;

  _IN1_value = 0;
  _IN2_value = 0;
  _PWM_value = 0;
}


void motor::coast(){
// function commands drive to let motor coast
  _IN1_value = 0;
  _IN2_value = 0;
}

void motor::brake(){
// function commands drive to brake motor:
  _IN1_value = 1;
  _IN2_value = 1;
}

void motor::direction(bool fwdRev){
// function sets direction of motor
// False= Forward. True = Reverse

  if(fwdRev){
    //motor Reverse

    _IN1_value = 1;
    _IN2_value = 0;
    
  }else{
    //motor Forward

    _IN1_value = 0;
    _IN2_value = 1;
  }
}

void motor::speed(int desiredSpeed){
  // funciton sets the desired PWM value

  uint8_t cappedSpeed; // Value within arduino PWM range;

  if(desiredSpeed > 255){
    // check for overflow

    cappedSpeed = 255;

  }else if (desiredSpeed < 0){
    // check for negative

    cappedSpeed = 0;

  }else{
    // good input

    cappedSpeed = desiredSpeed;
  }

  _PWM_value = cappedSpeed;

}

void motor::outputToDrive(){
  // outputs values to output pins connected to motor drive for specific motor
  
  digitalWrite(*_IN1_pin , _IN1_value);
  digitalWrite(*_IN2_pin , _IN2_value);
  analogWrite(*_PWM_pin , _PWM_value);


}
