// ========================================================================
// Defines and includes
#include <Servo.h>
#define BUFFERLENGTH 20

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
// TODO - Add DriveToFire algorithm
// TODO - Add fire detection to the repositioning state in the scanning sm

// ======================== Enums =========================
enum RUNNING_SM {
  INITIALISING,
  RUNNING,
  STOPPED
};

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
const byte fanPin = 53; // TODO - Change these
// IR
const byte irFrontLeftPin = 54; // TODO - Change these
const byte irFrontRightPin = 55;
const byte irSideLeftPin = 56;
const byte irSideRightPin = 57;
// Sonar
const byte sonarTrigPin = 17; // TODO - change these
const byte sonarEchoPin = 18;

// ====================== Variables ======================= 

float loopTime = 10; // Time for each loop in ms
HardwareSerial *SerialCom;

// Controller gains
float kP_servoAngle = 0.001f;

// Motors
float omegaToPulse = 21.3f;
float L1 = 7.5f; // Distance from centre to front axe
float L2 = 8.5f; // Distance from centre to left/right wheen centres
float Rw = 2.25f; // Wheel radius in cm
int maxPulseValue = 250;
int minPulseValue = 90; 
int minServoPulseValue = 10;
int maxServoPulseValue = 600;

// State machines
RUNNING_SM runningState = INITIALISING;
FINISHED_SM finishedState = NO_ACTION_FINISHED;
DRIVING_SM drivingState = NO_ACTION_DRIVING;
EXTINGUISHING_SM extinguishingState = NO_ACTION_EXTINGUISHING;
SCANNING_SM scanningState = NO_ACTION_SCANNING;

// IR Sensors
float irFrontLeft = 0;
float irFrontRight = 0;
float irSideLeft = 0;
float irSideRight = 0;
float irFrontLeftBuffer[BUFFERLENGTH];
float irFrontRightBuffer[BUFFERLENGTH];
float irSideLeftBuffer[BUFFERLENGTH];
float irSideRightBuffer[BUFFERLENGTH];
int irFrontLeftidx = 0;
int irFrontRightidx = 0;
int irSideLeftidx = 0;
int irSideRightidx = 0;

// Track related
int firesPutOut = 0;
bool fireFound = false;
bool startSearching = true; 
bool startFireFighting = false;
float repositionSpeed = 150;
float searchDistanceThreshold = 20;

// Phototransistors
float photoAverage; // TODO: actually read these lol, are we actually going to use photoaverage? We are going to use sonar for detecting distance to fire

// Fan
bool fanStartingTimeMeasured = false; // Used to take a timestamp of the time the fan is first turned on
unsigned int startTime;
float fanOnTime = 10000; // ms
float fanAngleThreshold = 0.5;

// Gyro
int currentAngle = 0;

// Sonar
int lastPing = 0;
int sonaridx = 0;
float sonarBuffer[BUFFERLENGTH];
float sonarDistance = 999;
long sonarRiseMicros;

// Rotation
float kP_Wz2 = 0.02;

// ================== Arduino functions ===================
void setup() {
  SerialCom = &Serial;
  SerialCom->begin(115200);
}

void loop() {
  switch (runningState) {
  case INITIALISING:
    runningState = Initialising();
    break;
  case RUNNING:   
    runningState = Running();
    break;
  case STOPPED:
    runningState =  Stopped();
    break;
  }
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
        fireFound = false;
      }
      break;
    case DRIVING:
      drivingState = DriveToFire();
      break;
  }
}

void ExtinguishRun(float deltaTime) {
    switch (extinguishingState) {
    case NO_ACTION_EXTINGUISHING:
      if (startFireFighting)
        extinguishingState = ALIGNING;
        startFireFighting = false;
      break;
    case ALIGNING:
      extinguishingState = AlignFan(deltaTime);
      break;
    case EXTINGUISHING:
      extinguishingState = RunFan();
      break;
  }
}

void ScanningRun(float deltaTime) {
  switch (scanningState) {
    case NO_ACTION_SCANNING:
      if (startSearching)
        scanningState = SCANNING;
        startSearching = false;
      break;
    case SCANNING:
      scanningState = Scanning(deltaTime); 
      break;
    case REPOSITION:
      scanningState = Repositioning();
      break;
  }
}

// =================== State functions ========================
RUNNING_SM Initialising() {
  EnableMotors();
  // TODO
  SetupIR(irFrontLeftPin, irFrontLeftBuffer);  
  SetupIR(irFrontRightPin, irFrontRightBuffer);
  SetupIR(irSideLeftPin, irSideLeftBuffer);  
  SetupIR(irSideRightPin, irSideRightBuffer);
  // GyroSetup();
  SetupSonar();
  return RUNNING;
}

RUNNING_SM Running() {
  // Running the state machines every loopTime
  static unsigned long lastMillis;  
  unsigned int deltaTime = millis() - lastMillis;
  if (deltaTime >= loopTime) {
    lastMillis = millis();
    FinishedRun();
    DrivingRun();
    ExtinguishRun(deltaTime);
    ScanningRun(deltaTime);
  }
  if (!is_battery_voltage_OK()) return STOPPED;
  return RUNNING;  
}

RUNNING_SM Stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  //make motors not move
  MotorWrite(0, 0, 0);
  if (millis() - previous_millis > 500) { // Print message every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 2 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 2) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        SerialCom->println("Lipo OK returning to RUN STATE");
        return INITIALISING;
      }
    } else
      counter_lipo_voltage_ok = 0;
  }
  return STOPPED;
}

DRIVING_SM DriveToFire() {
  // move forward
  
  if (photoAverage > 40) { // not close enough to fire
    if ((irFrontRight < 20) || (irFrontLeft < 20)) {  // front sensors detect obstacle
      if (irFrontRight > irFrontLeft) {
         // move right for 10ccm
      } else {
        // move left for 10cm
      }
    } else {
      return DRIVING;
    }
  }
  else {
    startFireFighting = true;
  }
  
}

EXTINGUISHING_SM RunFan() {
  digitalWrite(fanPin, HIGH);
  // Reference time is when extinguishing state first starts
  if (!fanStartingTimeMeasured) {
    fanStartingTimeMeasured = true; 
    startTime = millis(); 
  }
  // Stops fan once it has been turned on for 10s
  if (millis() - startTime >= fanOnTime) {
    digitalWrite(fanPin, LOW);
    startSearching = true; 
    fanStartingTimeMeasured = false; 
    firesPutOut++;
    return NO_ACTION_EXTINGUISHING; 
  }
  return EXTINGUISHING; 
}

EXTINGUISHING_SM AlignFan(float deltaTime) {
  float controllerOutput = FanAlignController(deltaTime);
  ServoWrite(controllerOutput); 
  if (controllerOutput <= fanAngleThreshold) {
    return EXTINGUISHING; 
  }
  return ALIGNING; 
}

SCANNING_SM Repositioning() {
  MotorWrite(0, repositionSpeed, 0);
  if (1 <= searchDistanceThreshold) { // TODO - replace "1" with sonar distance
    scanningState = SCANNING; 
  }
}

SCANNING_SM Scanning(int deltaTime) {
  float desiredAngle = 270; // 360 - 90 as our sensors cover ~90 deg
  float error = desiredAngle - currentAngle;
  float Wz = (kP_Wz2 * error);
  MotorWrite(0, 0, Wz);
  // 1000 Arbuitary value
  if (photoAverage >= 1000) { // probably shouldn't be an average but a peak, given the fire won't be seen on all sensors
    fireFound = true;
    return NO_ACTION_SCANNING; 
  }
  if (abs(error) < 0.5 && Wz < 5) { // has completed a full scan
    return REPOSITION;
  }
  return SCANNING;
}

// =================== Controllers ============================
// Servo angle controller
float FanAlignController(int deltaTime) {
  float error = 0;

  // TODO
  // Need to decide on phototransistor arrangement on servo motor 
  // and how the error would be decided
  // Idea: Error is most likely minimised when the front facing phototransistors have similar values
  // Eg.: error = phototransistor1 - phototransistor4

  return kP_servoAngle * error;
}

// ================== Sensor functions =======================
void SetupIR(byte irPin, float irBuffer[]){
  pinMode(irPin, INPUT); 
  for (int i = 0; i < BUFFERLENGTH - 1; i++) { // Filter part
    irBuffer[i] = 0;
  }
}

void SetupSonar(){
  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sonarEchoPin), echoRead, CHANGE); // Attach interrupt to echo pin
  for (int i = 0; i < BUFFERLENGTH - 1; i++) { // Filter part
    sonarBuffer[i] = 999;
  }
  PingSonar();
}

void ReadSensors(int deltaTime){
  ReadGyro(deltaTime); // TODO
  ReadIR(irFrontLeftPin, irFrontLeft, irFrontLeftBuffer, irFrontLeftidx);
  ReadIR(irFrontRightPin, irFrontRight, irFrontRightBuffer, irFrontRightidx);
  ReadIR(irSideLeftPin, irSideLeft, irSideLeftBuffer, irSideLeftidx);
  ReadIR(irSideRightPin, irSideRight, irSideRightBuffer, irSideRightidx);
  PingSonar();
  // TODO - add phototransistor reading
}

void ReadIR(int irPin, float &value, float irBuffer[], int &idx){
  // NOTE: All "values" in buffer are divided by buffer length  
  float newValue = 448.35f * pow(analogRead(irPin), -0.593f) / BUFFERLENGTH;
  // n.b first 5 values will be off due to zero-initialisation
  value -= irBuffer[idx]; // Subtract last number from average
  value += newValue; // Add new number to average
  irBuffer[idx] = newValue; // Overwrite new number in buffer
  if (idx < BUFFERLENGTH - 1){ // Increment buffer index
    idx++;
  } else {
    idx = 0;
  }
}

void ReadGyro(int deltaTime) {
  currentAngle = 0;
}

void PingSonar(){
  // Gets HC-SR04 to seng sonar ping at 40Hz
  if (millis()-lastPing > 25){ // Runs at 40hz (25ms)
    lastPing = millis();
    // Trigger HIGH pulse of 10us
    digitalWrite(sonarTrigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(sonarTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sonarTrigPin, LOW);
  }
}

// Sonar readings (attached to interrupt)
void echoRead(){
  //Reads how long the echo pulse is
  if(digitalRead(sonarEchoPin) == HIGH){
    //measure the start of the pulse
    sonarRiseMicros = micros();
  } else {
    //measure the end of the pulse 
    long signalDuration = micros() - sonarRiseMicros;
    //Calculate distance
    if(signalDuration > 0 ){
      float newValue = ((signalDuration/2.0)*0.0343 + 7.5) / BUFFERLENGTH;
      // Convert to distance by multiplying by speed of sound, 
      // accounting for returned wave by division of 2
      // offset by 7.5cm to account for sensor positioning on robot
      // Filter:
      sonarDistance -= sonarBuffer[sonaridx]; // Subtract last number from average
      sonarDistance += newValue; // Add new number to average
      sonarBuffer[sonaridx] = newValue; // Overwrite new number in buffer
      if (sonaridx < BUFFERLENGTH - 1){ // Increment buffer index
        sonaridx++;
      } else {
        sonaridx = 0;
      }
    } 
  }
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

// ======================= Battery ========================
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 
  //3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  //Serial.println(raw_lipo * 5 / 1023);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
