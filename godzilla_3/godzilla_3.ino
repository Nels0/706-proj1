 // ========================================================================
// Defines and includes
#include <Servo.h>
#define BUFFERLENGTH 20

// System coordinates and orientation
/*        _o__=__o_
 *    [1]-|   #   |-[2]       ^ X
 *        o   ^   o       Y   |
 *        |   |   |       <---+ 
 *        |   |   |        Each motor has matching coordinate system
 *    [3]-|_______|-[4]    Positive angular velocity counter-clockwise
 * 
 *  o - IR, # - fan/servo/phototransistors, = - sonar
 */ 

// ======================== Enums =========================
// Running FSM: Used for setup and running other FSMs
enum RUNNING_SM { 
  INITIALISING,
  RUNNING,
  STOPPED
};

// Finished FSM: Used to stop robot when 2 fires are put out
enum FINISHED_SM { 
  NO_ACTION_FINISHED,
  FINISHED  
};

// Driving FSM: Used to move the robot towards the fire
enum DRIVING_SM {
  NO_ACTION_DRIVING,
  DRIVING
};

// Extinguishing FSM: used to operate the fan and servo
enum EXTINGUISHING_SM {
  NO_ACTION_EXTINGUISHING,
  ALIGNING,
  EXTINGUISHING,
  REALIGNING
};

// Scanning FSM: Used to locate a fire
enum SCANNING_SM {
  NO_ACTION_SCANNING,
  SCANNING,
  REPOSITION
};

// =================== Pin assignments ====================
// Motors
const byte motor1Pin = 46;
const byte motor3Pin = 47;
const byte motor4Pin = 50;
const byte motor2Pin = 51;
const byte fanServoPin = 34; 
const byte fanMosfetPin = 36;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
Servo fanServo;
// IR
const byte irFrontLeftPin = A8;
const byte irFrontRightPin = A9;
const byte irSideLeftPin = A10;
const byte irSideRightPin = A11;
// Sonar
const byte sonarTrigPin = 17;
const byte sonarEchoPin = 18;
// Gyro
const byte gyroPin = A2;
// Phototransistors
const byte photoTransistor1 = A4;
const byte photoTransistor2 = A5;
const byte photoTransistor3 = A6;
const byte photoTransistor4 = A7;

// ====================== Variables ======================= 

float loopTime = 10; // Time for each loop in ms
HardwareSerial *SerialCom;

// Controller gains
float kP_servoAngle = 0.001f;
float kP_Wz = 0.02;
float kI_Wz = 0.001f;
float Wz_windup = 20;
float kP_Vy = 0.1;
float kI_Vy = 0.001f;
float Vy_windup = 10;
float kP_Vx = 0.1;
float kI_Vx = 0.001f;
float Vx_windup = 10;

// Robot features
float omegaToPulse = 21.3f;
float L1 = 7.5f; // Distance from centre to front axe
float L2 = 8.5f; // Distance from centre to left/right wheen centres
float Rw = 2.25f; // Wheel radius in cm
int maxPulseValue = 250;
int minPulseValue = 90; 
int minServoPulseValue = 10;
int maxServoPulseValue = 600;

// Initial states
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
float irFrontLeftBuffer[BUFFERLENGTH]; // Used for average filtering
float irFrontRightBuffer[BUFFERLENGTH];
float irSideLeftBuffer[BUFFERLENGTH];
float irSideRightBuffer[BUFFERLENGTH];
int irFrontLeftidx = 0;
int irFrontRightidx = 0;
int irSideLeftidx = 0;
int irSideRightidx = 0;

// Track related
int firesPutOut = 0; // Used to transition in the Finished FSM
bool fireFound = false; // Pulse signal: used to transition in the Driving FSM
bool startSearching = true; // Pulse signal: used to transition in the Scanning FSM
bool startFireFighting = false; // Pulse signal: used to transition in the Extinguishing FSM
float repositionSpeed = 150;
float searchDistanceThreshold = 20;

// Phototransistors
float photoTransistorDistance1;
float photoTransistorDistance2;
float photoTransistorDistance3;
float photoTransistorDistance4; 
float photoBuffer1[BUFFERLENGTH];
float photoBuffer2[BUFFERLENGTH];
float photoBuffer3[BUFFERLENGTH];
float photoBuffer4[BUFFERLENGTH];
int photoIdx1 = 0;
int photoIdx2 = 0;
int photoIdx3 = 0;
int photoIdx4 = 0;

// Fan
bool fanStartingTimeMeasured = false; // Used to take a timestamp of the time the fan is first turned on
unsigned int startTime;
float fanOnTime = 10000; // ms
float fanAngleThreshold = 0.5;
float resetServoTime = 0; // Used to realign the fan for 2 seconds

// Gyrosope 
float gyroSupplyVoltage = 5;
float gyroZeroVoltage = 0;
float gyroSensitivity = 0.0070;
float rotationThreshold = 1;
float currentAngle = 0;
float desiredAngle;

// Sonar
int lastPing = 0;
int sonaridx = 0;
float sonarBuffer[BUFFERLENGTH]; // Used for average filtering
float sonarDistance = 999;
long sonarRiseMicros;

// ================== Arduino functions ===================
void setup() {
  SerialCom = &Serial;
  SerialCom->begin(115200);
}

void loop() {
  switch (runningState) {
  case INITIALISING: // Used to setup sensors and motors
    runningState = Initialising();
    break;
  case RUNNING: // Used to operate FSMs
    runningState = Running();
    break;
  case STOPPED: // Used to turn off motors and for battery safety
    runningState =  Stopped();
    break;
  }
}

// =================== State machines =========================
void FinishedRun() {
  switch (finishedState) {
    case NO_ACTION_FINISHED:
      if (firesPutOut == 2)
        finishedState = FINISHED; // State transition: NO_ACTION_FINISHED -> FINISHED
      break;
    case FINISHED:
      MotorWrite(0, 0, 0);
      break;
  }
}

void DrivingRun(float deltaTime) {
  switch (drivingState) {
    case NO_ACTION_DRIVING:
      if (fireFound) {
        drivingState = DRIVING; // State transition: NO_ACTION_DRIVING -> DRIVING
        fireFound = false;
      }
      break;
    case DRIVING:
      drivingState = DriveToFire(deltaTime);
      break;
  }
}

void ExtinguishRun(float deltaTime) {
    switch (extinguishingState) {
    case NO_ACTION_EXTINGUISHING:
      if (startFireFighting)
        extinguishingState = ALIGNING; // State transition: NO_ACTION_EXTINGUISHING -> ALIGNING
        startFireFighting = false;
      break;
    case ALIGNING:
      extinguishingState = AlignFan(deltaTime);
      break;
    case EXTINGUISHING:
      extinguishingState = RunFan();
      break;
    case REALIGNING:
      extinguishingState = ResetFanServo();
      break;
  }
}

void ScanningRun(float deltaTime) {
  switch (scanningState) {
    case NO_ACTION_SCANNING:
      if (startSearching)
        scanningState = SCANNING; // State transition: NO_ACTION_SCANNING -> SCANNING
        desiredAngle = currentAngle + 270; // 360 - 90 as our sensors cover ~90 deg
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
  SetupIR(irFrontLeftPin, irFrontLeftBuffer);  
  SetupIR(irFrontRightPin, irFrontRightBuffer);
  SetupIR(irSideLeftPin, irSideLeftBuffer);  
  SetupIR(irSideRightPin, irSideRightBuffer);
  SetupPhotoTransistor(photoTransistor1, photoBuffer1);
  SetupPhotoTransistor(photoTransistor2, photoBuffer2);
  SetupPhotoTransistor(photoTransistor3, photoBuffer3);
  SetupPhotoTransistor(photoTransistor4, photoBuffer4);
  SetupGyro();
  SetupSonar();
  return RUNNING;
}

RUNNING_SM Running() {
  // Running the state machines every loopTime
  static unsigned long lastMillis;  
  unsigned int deltaTime = millis() - lastMillis;
  if (deltaTime >= loopTime) {
    lastMillis = millis();
    // Running all the state machines
    FinishedRun();
    DrivingRun(deltaTime);
    ExtinguishRun(deltaTime);
    ScanningRun(deltaTime);
  }
  if (!IsBatteryVoltageOK()) return STOPPED;
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
    if (IsBatteryVoltageOK()) {
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

DRIVING_SM DriveToFire(float deltaTime) {
  static float I_Wz;
  static float I_Vx;
  static float I_Vy;
  float Vy = 0;
  float Vx = 0;
  // Align to the fire
  float photoError = photoTransistorDistance2 - photoTransistorDistance3;
  if(abs(I_Wz) < Wz_windup) // Integral component of controller
    I_Wz += photoError * deltaTime/1000;
  float Wz = kP_Wz * photoError + kI_Wz * I_Wz;

  if (PhotoMaxDistance() > 40) { // Not close enough to fire
    float sonarError = 9.5 - sonarDistance;
    float leftIRError = 9.5 - irFrontLeft;
    float rightIRError = 9.5 - irFrontRight;
    // Using the smallest error to determine control to prevent corner clipping
    float xError = min(sonarError, min(leftIRError, rightIRError));

    if (abs(I_Vx) < Vx_windup) // Integral part of controller
      I_Vx += xError * deltaTime/1000;
    Vx = kP_Vx * xError + kI_Vx * I_Vx;

    if ((irFrontRight < 20) || (irFrontLeft < 20)) {  // Front sensors detect obstacle
      // Vy is mainly controlled by the front sensors, strafing right and left. If the robot comes close
      // to the wall, the side sensors will contribute to control. Otherwise, they are insignificant
      float yError = (irFrontRight - irFrontLeft) + 1/pow(irSideLeft,2) - 1/pow(irSideRight,2);

      if(abs(I_Vy) < Vy_windup) // Integral component of controller
        I_Vy += yError * deltaTime/1000;
      Vy = kP_Wy * yError + kI_Wy * I_Wy;
    } 
  }
  else { // Close to fire
    startFireFighting = true; // Send startFireFighting pulse
    return NO_ACTION_DRIVING; // State transition: DRIVING -> NO_ACTION_DRIVING
  }
  MotorWrite(Vx,Vy,Wz);
  return DRIVING;
}

EXTINGUISHING_SM RunFan() {
  digitalWrite(fanMosfetPin, HIGH);
  // Reference time is when extinguishing state first starts
  if (!fanStartingTimeMeasured) {
    fanStartingTimeMeasured = true; 
    startTime = millis(); 
  }
  // Stops fan once it has been turned on for 10s
  if (millis() - startTime >= fanOnTime) {
    digitalWrite(fanMosfetPin, LOW);
    startSearching = true; // Send startSearching pulse
    fanStartingTimeMeasured = false; 
    firesPutOut++;
    resetServoTime = millis();
    return REALIGNING; // State transition: EXTINGUISHING -> REALIGNING
  }
  return EXTINGUISHING; 
}

EXTINGUISHING_SM ResetFanServo() {
  while (millis () < resetServoTime + 2000) { // servo will be set to zero angle for 2 seconds
    ServoWrite(0);
    return REALIGNING;
  }
  return NO_ACTION_EXTINGUISHING; // State transition: REALIGNING -> NO_ACTION_EXTINGUISHING
}

EXTINGUISHING_SM AlignFan(float deltaTime) {
  float controllerOutput = FanAlignController(deltaTime);
  ServoWrite(controllerOutput); 
  if (controllerOutput <= fanAngleThreshold) {
    return EXTINGUISHING; // State transition: ALIGNING -> EXTINGUISHING
  }
  return ALIGNING; 
}

SCANNING_SM Repositioning() {
  MotorWrite(repositionSpeed, 0, 0);
  // The robot will drive forward to reposition, unless an obstacle is detected
  if (sonarDistance <= searchDistanceThreshold) {
    scanningState = SCANNING; 
  }
}

SCANNING_SM Scanning(int deltaTime) {
  float error = desiredAngle - currentAngle;
  // If the angle is greater than 180, remove 360 to make it between 0 and -180
  // If the angle is less than -180, add 360 to make it between 0 and 180
  if (error > 180)
    error = error - 360;
  else if (error < -180)
    error = error + 360;

  float Wz = (kP_Wz * error);
  MotorWrite(0, 0, Wz);

  if (PhotoMinDistance() <= 60) {
    fireFound = true;
    return NO_ACTION_SCANNING; 
  }
  if (abs(error) < 0.5 && Wz < 5) { // Has completed a full scan
    return REPOSITION;
  }
  return SCANNING;
}

// =================== Controllers ============================
// Servo angle controller
float FanAlignController(int deltaTime) {

  // Weighted error, more sensitive to phototransistor 2 and 3 misallignment to fire
  float error = photoTransistorDistance1 - photoTransistorDistance4 + 2*photoTransistorDistance2 - 2*photoTransistorDistance3; 

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

void SetupPhotoTransistor(byte photoPin, float photoBuffer[]) {
  pinMode(photoPin, INPUT);
  for (int i = 0; i < BUFFERLENGTH - 1; i++) {
    photoBuffer[i] = 0;
  }
}

void SetupGyro() {
  int i;
  int gyroValue;
  float sum = 0;    
  pinMode(gyroPin,INPUT);
  Serial.println("Getting the gyro zero voltage...");
  // Read 100 values of voltage when gyro is at still, to calculate the zero-drift
  for (i=0; i<100; i++) {  
    gyroValue = analogRead(gyroPin);  
    sum += gyroValue;  
    delay(5);
  }
  gyroZeroVoltage = sum/100;
  Serial.println("Gyro Calibrated!");
}

void ReadSensors(int deltaTime){
  ReadGyro(deltaTime); 
  ReadIR(irFrontLeftPin, irFrontLeft, irFrontLeftBuffer, irFrontLeftidx);
  ReadIR(irFrontRightPin, irFrontRight, irFrontRightBuffer, irFrontRightidx);
  ReadIR(irSideLeftPin, irSideLeft, irSideLeftBuffer, irSideLeftidx);
  ReadIR(irSideRightPin, irSideRight, irSideRightBuffer, irSideRightidx);
  ReadPhotoTransistor(photoTransistor1, photoTransistorDistance1, photoBuffer1, photoIdx1);
  ReadPhotoTransistor(photoTransistor2, photoTransistorDistance2, photoBuffer2, photoIdx2);
  ReadPhotoTransistor(photoTransistor3, photoTransistorDistance3, photoBuffer3, photoIdx3);
  ReadPhotoTransistor(photoTransistor4, photoTransistorDistance4, photoBuffer4, photoIdx4);
  PingSonar();
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
    // Order of operations to preserve precision
  float angularVelocity = (analogRead(gyroPin) - gyroZeroVoltage);
  angularVelocity *= gyroSupplyVoltage / gyroSensitivity / 1023;
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T. one second will run (1000/T).  
    float angleChange = angularVelocity * deltaTime / 1000.0f; 
    currentAngle -= angleChange; // Negative due to right handed coordinate system
  }
  // keep the angle between 0-360
  if (currentAngle < 0)    
    currentAngle += 360;
  else if (currentAngle > 359)
    currentAngle -= 360;
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

void ReadPhotoTransistor(int photoPin, float &value, float irBuffer[], int &idx) {
  // NOTE: All "values" in buffer are divided by buffer length 
  float v = analogRead(photoPin);
  float newValue = (-26.3*(pow(v,3)) + 295.3*(pow(v, 2)) - 1103.4*v + 1413.9) / BUFFERLENGTH;
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

float PhotoMinDistance() {
  return min(min(photoTransistorDistance1, photoTransistorDistance2), min(photoTransistorDistance2, photoTransistorDistance4));
}

float PhotoMaxDistance() {
  return max(max(photoTransistorDistance1, photoTransistorDistance2), max(photoTransistorDistance2, photoTransistorDistance4));
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
boolean IsBatteryVoltageOK()
{
  static byte lowVoltageCounter;
  static unsigned long previousMillis;

  int lipoLevelCal;
  int rawLipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 
  //3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  rawLipo = analogRead(A0);
  //Serial.println(raw_lipo * 5 / 1023);
  lipoLevelCal = (rawLipo - 717);
  lipoLevelCal = lipoLevelCal * 100;
  lipoLevelCal = lipoLevelCal / 143;

  if (lipoLevelCal > 0 && lipoLevelCal < 160) {
    previousMillis = millis();
    lowVoltageCounter = 0;
    return true;
  } else {
    if (lipoLevelCal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (lipoLevelCal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(lipoLevelCal);
      SerialCom->println("%");
    }

    lowVoltageCounter++;
    if (lowVoltageCounter > 5)
      return false;
    else
      return true;
  }
}
