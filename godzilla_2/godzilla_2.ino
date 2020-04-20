// ========================================================================
// Defines and includes
//#define NO_BATTERY_V_OK
//#define GYRO_DEBUG
//#define IR_DEBUG 
//#define MOTOR_DEBUG
//#define SONAR_DEBUG
//#define ROTATION_CONTROL_DEBUG
#include <Servo.h>

/* |      ____=____
 * |  [1]-|       |-[2]       ^ X
 * |      o   ^   |       Y   |
 * |      |   |   |       <---+ 
 * |      o   |   |        Each motor has matching coordinate system
 * |  [3]-|_______|-[4]    Positive angular velocity counter-clockwise
 * 
 */

// ========= TODO =========
/* better debugging?
 * drive forward
 * gyro reading/calibrations
 * sonar reading
 * turning state
 * integral windup
 * still state
 * 
 */

// ======================Enums===================================================
 

enum RUNNING_STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};


enum ACTION_STATE {
  MOVING_FORWARD,
  MOVING_TURNING,
  STILL
};

enum DEBUG {
  NONE,
  FAULT,
  WARNING,
  VERBOSE
};


// ====================Pin assignments================================================
//Sensors
const byte gyroPin = A2;
//IR
const byte irFrontPin = A4;
const byte irBackPin = A5;
//Sonar
const byte sonarTrigPin = 17;
const byte sonarEchoPin = 16;
//Motors
const byte motor1Pin = 46;
const byte motor3Pin = 47;
const byte motor4Pin = 50;
const byte motor2Pin = 51;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


// ======================Variables===================================================
// 

// STATE
int turnCount = 0;
float desiredAngle = 90;

// SENSORS;
float sonarDistance = 999;

float irFront;
float irFrontBuffer[5];
float irBack;
float irBackBuffer[5];

float gyroSupplyVoltage = 5;
float gyroZeroVoltage = 0;
float gyroSensitivity = 0.0070;
float rotationThreshold = 1;
float currentAngle = 0;


// GAINS
float kP_Wz = 1.5f;
float kI_Wz = 0.0f;
float kP_Vy = 1.0f;
float kI_Vy = 0.0f;

// Motors
float omegaToPulse = 21.3;
float L1 = 7.5; //distance from centre to front axe
float L2 = 8.5; //distance from centre to left/right wheen centres
float Rw = 2.25; //wheel radius in cm
int maxSpeedValue = 250;
int minSpeedValue = 0; 

int loopTime = 10; // Time for each loop in ms
DEBUG debug_level = NONE;
HardwareSerial *SerialCom;


void setup() {
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("  ____ ____   ___  _   _ ____    _ ");
  SerialCom->println(" / ___|  _ \\ / _ \\| | | |  _ \\  / |");
  SerialCom->println("| |  _| |_) | | | | | | | |_) | | |");
  SerialCom->println("| |_| |  _ <| |_| | |_| |  __/  | |");
  SerialCom->println(" \\____|_| \\_\\\\___/ \\___/|_|     |_|");
  delay(1000);
  SerialCom->println("Setup....");
  delay(1000);

}

void loop() {

  static RUNNING_STATE machineState = INITIALISING;
  
  switch (machineState) {
    case INITIALISING:
      machineState = Initialising();
      break;
    case RUNNING:   
      machineState = Running();
      break;
    case STOPPED:
      machineState =  Stopped();
      break;
  }
}

// ==================State machine functions=====================================


// ========== MACHINE STATES=======
RUNNING_STATE Initialising() {
  SerialCom->println("INITIALISING...");
  delay(500);
  EnableMotors();
  IrSetup();
  GyroSetup();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}



RUNNING_STATE Running() {
  static ACTION_STATE actionState = STILL;
  
  static unsigned long lastMillis;  
  unsigned int deltaTime = millis() - lastMillis;

  if (deltaTime >= loopTime) {
    
    lastMillis = millis();
    
    ReadSensors(deltaTime);
    
    switch (actionState) {
      case MOVING_FORWARD:
        actionState = MoveForward(deltaTime);
      break;
      
      case MOVING_TURNING:
        actionState = Rotate(deltaTime);
        break;
      
      case STILL:
        break;
    }
  }
  
  if (!is_battery_voltage_OK()) return STOPPED;
  return RUNNING;  
}

//TODO: fix variable naming
RUNNING_STATE Stopped() {
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
        //enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return INITIALISING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
  }
  return STOPPED;
}


// ============= ACTION STATES ===========
ACTION_STATE MoveForward(int deltaTime){

  float Wz = GetWz(deltaTime);
  float Vy = GetVy(deltaTime);
  float Vx = 10.0f;//get_Vx(deltaTime); 
  
  MotorWrite(Vx, Vy, Wz);


  if(sonarDistance < 15){
    if(turnCount < 3){
      return STILL;
    } else {
      desiredAngle = currentAngle + 90;
      return MOVING_TURNING;
    }
  } else {
    return MOVING_FORWARD;
  }
}

// TODO - review by nelson
// TODO - reset integral when new control thing is happening?
ACTION_STATE Rotate(int deltaTime) {
  float error = desiredAngle - currentAngle;
  // If the angle is greater than 180, remove 360 to make it between 0 and -180
  // If the angle is less than -180, add 360 to make it between 0 and 180
  error = error + (error > 180) ? -360 : (error < -180) ? 360 : 0;

  // TODO - add integral
  float Wz = (kP_Wz * error);

  MotorWrite(0, 0, Wz);

  if (error < 0.5 && Wz < 5) { // TODO - change these values
    turnCount++;
    return MOVING_FORWARD;
  } else {
    return MOVING_TURNING;
  }
}

ACTION_STATE Still () {
  MotorWrite(0, 0, 0);

  return STILL;

}

float GetWz(int deltaTime) {
  static float I_Wz;
  float error = irFront - irBack;
  
  I_Wz += error *  deltaTime/1000;
  return (kP_Wz * error) + (kI_Wz * I_Wz);
}

float GetVy(int deltaTime) {
  static float I_Vy;
  float error = 15 - ((irFront + irBack)/2);

  I_Vy += error *  deltaTime/1000;
  return (kP_Vy * error) + (kI_Vy * I_Vy);
}

void EnableMotors(){
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
}

void MotorWrite(float Vx, float Vy, float Wz){
  float _Vx = Sat2(Vx, 20, 0);
  float _Vy = Sat2(Vy, 20, 0);
  float _Wz = Sat2(Wz, 20, 0);

  int motor1Speed = omegaToPulse * KinematicCalc(_Vx,  _Vy, -_Wz);
  int motor2Speed = omegaToPulse * KinematicCalc(_Vx, -_Vy,  _Wz);
  int motor3Speed = omegaToPulse * KinematicCalc(_Vx, -_Vy, -_Wz);
  int motor4Speed = omegaToPulse * KinematicCalc(_Vx,  _Vy,  _Wz);

  motor1.writeMicroseconds(1500 + Sat2(motor1Speed, maxSpeedValue, minSpeedValue));
  motor2.writeMicroseconds(1500 - Sat2(motor2Speed, maxSpeedValue, minSpeedValue));
  motor3.writeMicroseconds(1500 + Sat2(motor3Speed, maxSpeedValue, minSpeedValue));
  motor4.writeMicroseconds(1500 - Sat2(motor4Speed, maxSpeedValue, minSpeedValue));
}

int KinematicCalc(int Vx, int Vy, int Wz) {
    return (int)((Vx + Vy + Wz*(L1 + L2)) / Rw);
}

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

//========== SENSOR SETUP =============
void IrSetup(){
  pinMode(irFrontPin,INPUT); 
  pinMode(irBackPin,INPUT);

  //Filter part
}

void SonarSetup(){
  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
}

void GyroSetup() {
  int i;
  int gyroValue;
  float sum = 0;    
  pinMode(gyroPin,INPUT);
  Serial.println("Getting the gyro zero voltage...");
  //  read 100 values of voltage when gyro is at still, to calculate the zero-drift
  for (i=0; i<100; i++) {  
    gyroValue = analogRead(gyroPin);  
    sum += gyroValue;  
    delay(5);
  }
  gyroZeroVoltage = sum/100;
  Serial.println("Gyro Calibrated!");
}



//========== SENSOR READINGs===========
void ReadSensors(int deltaTime){
  
  ReadGyro(deltaTime);
  ReadIR(irFrontPin, irFront, irFrontBuffer);
  ReadIR(irBackPin, irBack, irBackBuffer);
  ReadSonar();
}

void ReadIR(int irPin, float &value, float irBuffer[]){
  value = 448.35f * pow(analogRead(irPin), -0.593f);
  #ifdef IR_DEBUG
    Serial.print(irPin);
    Serial.print(": ");
    Serial.println(value);
  #endif
}

void ReadGyro(int deltaTime) {
  //OoO to preserve precision
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

  #ifdef GYRO_DEBUG
    Serial.print(" Angular Velocity: ");
    Serial.print(angularVelocity);
    Serial.print(" Current Angle: ");
    Serial.println(currentAngle);
   #endif
}

void ReadSonar(){
  // Trigger HIGH pulse of 10us
  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrigPin, LOW);

  // Read duration signal
  pinMode(sonarEchoPin, INPUT);
  float signalDuration = pulseIn(sonarEchoPin, HIGH);
  // Convert to distance by multiplying by speed of sound, 
  // accounting for returned wave by division of 2
  // offset by 12.3cm to account for sensor positioning on robot
  sonarDistance = (signalDuration/2.0)*0.0343 + 12.3;
  //sonarDistance = 200;

  #ifdef SONAR_DEBUG
    Serial.print(" Sonar Distance: ");
    Serial.println(sonarDistance);
  #endif
}


// ============= BATTERY =========

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
