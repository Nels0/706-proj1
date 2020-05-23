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
// TODO - change the value for minServoPulseValue
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
int minServoPulseValue = 0;
int maxServoPulseValue = 600;

// State machines
FINISHED_SM finishedState = NO_ACTION_FINISHED;
DRIVING_SM drivingState = NO_ACTION_DRIVING;
EXTINGUISHING_SM extinguishingState = NO_ACTION_EXTINGUISHING;
SCANNING_SM scanningState = NO_ACTION_SCANNING;

// Track related
int firesPutOut = 0;
bool fireFound = false;

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
    switch (extinguishingState) {
    case NO_ACTION_EXTINGUISHING:
      // TODO
      break;
    case ALIGNING:
      // TODO
      break;
    case EXTINGUISHING:
      // TODO
      break;
  }
}

void ScanningRun() {
  switch (scanningState) {
    case NO_ACTION_SCANNING:
      // TODO
      break;
    case ALIGNING:
      // TODO
      break;
    case REPOSITION:
      // TODO
      break;
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
  if(abs(I_servoAngle) < servoAngle_windup){
    I_servoAngle += error * deltaTime/1000;
  }
  return (kP_servoAngle * error) + (kI_servoAngle * I_servoAngle);
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
