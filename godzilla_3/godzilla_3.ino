// ========================================================================
// Defines and includes
#include <Servo.h>

// System coordinates and orientation
/* |      ____=____
 * |  [1]-|       |-[2]       ^ X
 * |      o   ^   |       Y   |
 * |      |   |   |       <---+ 
 * |      o   |   |        Each motor has matching coordinate system
 * |  [3]-|_______|-[4]    Positive angular velocity counter-clockwise
 * 
 */

// TODO - implement error for GetServoAngle function
// TODO - change fanServoPin
// TODO - change fanPin
// TODO - Implement deltaTime
// TODO - reset state transition conditions every loop - e.g. startFireFighting = false and fireFound = false at the start of each loop... 
//        or maybe this wont work because the command to startFireFighting might be set to true after the firefighting state is called? idk.
//        Might need to reset it once the state transition happens. Or maybe we want to keep them on until the state is finished? Like 
//        having the fireFound to be true until the fire is put out
// TODO - Add DriveToFire algorithm

// ======================== Enums =========================
enum FINISHED_SM {
  NO_ACTION_FINISHED,
  FINISHED  
};

enum DRIVING_SM {
  NO_ACTION_DRIVING,
  DRIVING
};

enum EXTINGUISHING_SM {
  NO_ACTION_EXTINGUISHING,
  ALIGNING,
  EXTINGUISHING
};

enum SCANNING_SM {
  NO_ACTION_SCANNING,
  SCANNING,
  REPOSITION
};

// =================== Pin assignments ====================
//Motors
const byte motor1Pin = 46;
const byte motor3Pin = 47;
const byte motor4Pin = 50;
const byte motor2Pin = 51;
const byte fanServoPin = 52; 
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
Servo fanServo;

//Fan
const byte fanPin = 53; // Really unsure


// ====================== Variables ======================= 

// Controller gains
float kP_servoAngle = 0.001f;
float kI_servoAngle = 0.0f;
float servoAngle_windup = 10.0f;

// Motors
float omegaToPulse = 21.3f;
float L1 = 7.5f; //distance from centre to front axe
float L2 = 8.5f; //distance from centre to left/right wheen centres
float Rw = 2.25f; //wheel radius in cm
int maxPulseValue = 250;
int minPulseValue = 90; 
int minServoPulseValue = 10;
int maxServoPulseValue = 600;

// State machines
FINISHED_SM finishedState = NO_ACTION_FINISHED;
DRIVING_SM drivingState = NO_ACTION_DRIVING;
EXTINGUISHING_SM extinguishingState = NO_ACTION_EXTINGUISHING;
SCANNING_SM scanningState = NO_ACTION_SCANNING;

// IR Sensors
float irFront = 0;
float irBack = 0; 


// Track related
int firesPutOut = 0;
bool fireFound = false;
bool startSearching = true; 

// Phototransistors
float photoAverage;

// Fan
int toggle = 0; 
unsigned int startTime;

// Gyro
int currentAngle = 0;

// Rotation
float kP_Wz2 = 0.02;

// ================== Arduino functions ===================
void setup() {
}

void loop() {
  // Running the state machines
  FinishedRun();
  DrivingRun();
  ExtinguishRun();
  ScanningRun();
}

// =================== State machines =========================
void FinishedRun() {
  switch (finishedState) {
    case NO_ACTION_FINISHED:
      if (firesPutOut == 2)
        finishedState = FINISHED;
      break;
    case FINISHED:
      MotorWrite(0, 0, 0);
      break;
  }
}

void DrivingRun() {
  switch (drivingState) {
    case NO_ACTION_DRIVING:
      if (fireFound) {
        drivingState = DRIVING;
      }
      break;
    case DRIVING:
      drivingState = DriveToFire();
      break;
  }
}

void ExtinguishRun() {
    unsigned int deltaTime = 1;
    // TODO

    switch (extinguishingState) {
    case NO_ACTION_EXTINGUISHING:
      if (photoAverage >= 1000) {
        // Arbitruary value of 1000
        // change later
        startSearching = false; 
        extinguishingState = ALIGNING; 
      }
      break;
    case ALIGNING:
      ServoWrite(GetServoPulse(deltaTime)); 
      if (GetServoPulse(deltaTime) <= 0.5) {
        // Arbitruary value of 0.5
        // change later
        extinguishingState = EXTINGUISHING; 
      } else {
        extinguishingState = ALIGNING; 
      }
      break;
    case EXTINGUISHING:
      extinguishingState = RunFan(); 
      break;
  }
}

EXTINGUISHING_SM RunFan() {
  // Unsure whether this is how you turn on the fan
  digitalWrite(fanPin, HIGH); 
  extinguishingState = EXTINGUISHING; 

  // Reference time is when extinguishing state first starts
  if (toggle == 0) {
    toggle = 1; 
    startTime = millis(); 
  }

  // Stops fan once it has been turned on for 10s
  if (millis() - startTime >= 10000) {
    digitalWrite(fanPin, LOW);
    startSearching = true; 
    toggle = 0; 
    firesPutOut++; 
    extinguishingState = NO_ACTION_EXTINGUISHING; 
  }

  return extinguishingState; 
}

void ScanningRun() {
  unsigned int deltaTime = 1;
  // TODO

  switch (scanningState) {
    case NO_ACTION_SCANNING:
      if (startSearching) {
        scanningState = SCANNING; 
      }
      break;
    case SCANNING:
      scanningState = Scanning(deltaTime); 
      break;
    case REPOSITION:
      startSearching = false; 
      MotorWrite(0, 150, 0);
      if (irFront <= 20) {
        startSearching = true; 
        scanningState = SCANNING; 
      } 
      if (fireFound) {
        scanningState = NO_ACTION_SCANNING; 
      }
      break;
  }
}

SCANNING_SM Scanning(int deltaTime) {
  float desiredAngle = 420; 
  float error = desiredAngle - currentAngle;

  float Wz = (kP_Wz2 * error);
  MotorWrite(0, 0, Wz);

  // Need to read phototransistors
  // Arbuitary value
  if (photoAverage >= 1000) {
    fireFound = true;
    return NO_ACTION_SCANNING; 
  }

  if (abs(error) < 0.5 && Wz < 5) {
    return REPOSITION;
  } else {
    return SCANNING;
  }
}

// =================== State functions ========================
DRIVING_SM DriveToFire() {
  // Implement driving algorithm plz
  return DRIVING;
}

// =================== Controllers ============================
// Servo angle controller
float GetServoPulse(int deltaTime) {
  static float I_servoAngle;
  float error = 0;

  // TODO
  // Need to decide on phototransistor arrangement on servo motor 
  // and how the error would be decided
  // Idea: Error is most likely minimised when the front facing phototransistors have similar values
  // Eg.: error = phototransistor1 - phototransistor4

  if(abs(I_servoAngle) < servoAngle_windup){
    I_servoAngle += error * deltaTime/1000;
  }
  return (kP_servoAngle * error) + (kI_servoAngle * I_servoAngle);
}

// ================== Sensor functions =======================

void ReadGyro(int deltaTime) {
  currentAngle = 0;
}

// ================== Actuation functions =======================
void EnableMotors() {
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
  fanServo.attach(fanServoPin);
}

void MotorWrite(float Vx, float Vy, float Wz) {
  float _Vx = Sat2(Vx, 20, 0);
  float _Vy = Sat2(Vy, 20, 0);
  float _Wz = Sat2(Wz, 20, 0);

  // Kinematic equations to convert velocites to motor pulses
  int motor1Pulse = omegaToPulse * KinematicCalc(_Vx,  _Vy, -_Wz);
  int motor2Pulse = omegaToPulse * KinematicCalc(_Vx, -_Vy,  _Wz);
  int motor3Pulse = omegaToPulse * KinematicCalc(_Vx, -_Vy, -_Wz);
  int motor4Pulse = omegaToPulse * KinematicCalc(_Vx,  _Vy,  _Wz);

  motor1.writeMicroseconds(1500 + Sat2(motor1Pulse, maxPulseValue, minPulseValue));
  motor2.writeMicroseconds(1500 - Sat2(motor2Pulse, maxPulseValue, minPulseValue));
  motor3.writeMicroseconds(1500 + Sat2(motor3Pulse, maxPulseValue, minPulseValue));
  motor4.writeMicroseconds(1500 - Sat2(motor4Pulse, maxPulseValue, minPulseValue));
}

void ServoWrite(int servoPulse) {
  fanServo.writeMicroseconds(1500 + Sat2(servoPulse, maxServoPulseValue, minServoPulseValue));
}

// ====================== Helper functions ===========================
int Sat2(int value, int maxValue, int minValue) {
  // Will clamp the value if the value is outside the maximum ranges
  if (value < -maxValue)
    return -maxValue;
  else if (value > maxValue)
    return maxValue;
  else
    return value;

  // Will clamp if the value is too close to zero
  if (value < minValue)
    return 0;
  else if (value > -minValue) 
    return 0;
  else 
    return value; 
}

int KinematicCalc(int Vx, int Vy, int Wz) {
  return (int)((Vx + Vy + Wz*(L1 + L2)) / Rw);
}
