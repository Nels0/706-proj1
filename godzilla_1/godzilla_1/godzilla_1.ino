// ========================================================================
// Defines and includes
//#define NO_BATTERY_V_OK
#define GYRO_DEBUG
#define IR_DEBUG 
//#define LR_IR_DEBUG
//#define SR_IR_DEBUG
//#define MOTOR_DEBUG
#include <Servo.h>

// =========================================================================
// Enums

enum RUNNING_STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};
//coment

enum ACTION_STATE {
  // TODO
};

enum DEBUG {
  NONE,
  FAULT,
  WARNING,
  VERBOSE
};

// =========================================================================
// Pin assignments
int gyroPin = A2;
int srIRFrontPin = A4;
int srIRBackPin = A5;
int lrIRPin = A6;
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// =========================================================================
// Variables

DEBUG debug_level = NONE;

// Gyro variables
int gyroValue = 0;
float gyroSupplyVoltage = 5;
float gyroZeroVoltage = 0;
float gyroSensitivity = 0.007;
float rotationThreshold = 1.5;
float gyroRate = 0;
float currentAngle = 0;

// Short range IR
float srIRFrontDistance;
float srIRBackDistance;

// Long range IR
float lrIRDistance;

//Sonar
int sonarTimeUS;

// Other
HardwareSerial *SerialCom;
int loopTime = 10; // Time for each loop in ms
Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;
int speed_val = 100;

// =========================================================================
// Setup function
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

  GYRO_setup();
  IR_setup();
}

// =========================================================================
// Main loop function
void loop() {
  static RUNNING_STATE machine_state = INITIALISING;
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING:
      machine_state =  running();
      break;
    case STOPPED:
      machine_state =  stopped();
      break;
  };
}

// =========================================================================
// State machine functions
RUNNING_STATE initialising() {
  SerialCom->println("INITIALISING...");
  delay(1000);
  SerialCom->println("Enabling motors...");
  enable_motors();
  delay(1000);
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

RUNNING_STATE running() {
  static unsigned long running_previous_millis;
  unsigned int deltaTime = millis() - running_previous_millis;
  if (deltaTime >= loopTime) {
    GYRO_reading(deltaTime);
    SR_IR_front_reading();
    SR_IR_back_reading();
    //LR_IR_reading(); 
    running_previous_millis = millis();
  }
  if (!is_battery_voltage_OK()) return STOPPED;
  return RUNNING;
}

RUNNING_STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
  }
  return STOPPED;
}

// =========================================================================
// IR functions
void IR_setup() {
  pinMode(srIRFrontPin,INPUT); 
  pinMode(srIRBackPin,INPUT); 
  pinMode(lrIRPin,INPUT); 
  Serial.println("...");

  // TODO: Callibration for all IR sensors, consecutively 

  Serial.println("IR Calibrated!");
}

void SR_IR_front_reading() {
  srIRFrontDistance = 448.35f * pow(analogRead(srIRFrontPin), -0.593f);

//  srIRFrontDistanceFiltered = IR_Moving_Average(srIRFrontDistance, 0);

  #ifdef IR_DEBUG
    Serial.print(" SR Front Distance: ");
    Serial.print(srIRFrontDistance);
  #endif
}

void SR_IR_back_reading() {
  srIRBackDistance = 448.35f * pow(analogRead(srIRBackPin), -0.593f);

//  srIRBackDistanceFiltered = IR_Moving_Average(srIRBackDistance, 1);

   #ifdef IR_DEBUG
    Serial.print(" SR Back Distance: ");
    Serial.println(srIRBackDistance);
   #endif
}

void LR_IR_reading() {
  lrIRDistance = 1/analogRead(lrIRPin);

//  lrIRDistanceFiltered = IR_Moving_Average(lrIRDistance, 2);

   #ifdef IR_DEBUG
    Serial.print(" LR Distance: ");
    Serial.print(lrIRDistance);
   #endif
}

//void IR_Moving_Average(float Distance, enum IRSensor) {
//
//  switch 
//    case SRIRFront:
//    
//    break;
//
//    case SRIRBack:
//    break;
//
//    case LRIR:
//    break;
//      
//  
//}

//==================================================================
// Sonar Functions


void sonar_setup(){
  
}

void sonar_reading(){

  
}

// =========================================================================
// Motor functions
// TODO
void disable_motors()
{
  left_front_motor.detach();
  left_rear_motor.detach();
  right_rear_motor.detach();
  right_front_motor.detach();

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_front_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_rear_motor.attach(right_rear);
  right_front_motor.attach(right_front);
}

void stop()
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void move_forward()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void move_backward()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void move_counter_clockwise()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void move_clockwise()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void move_sideways_left()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void move_sideways_right()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}


// =========================================================================
//Gyro Functions
void GYRO_setup() {
  int i;
  float sum = 0;    
  pinMode(gyroPin,INPUT);
  Serial.println("Please keep the sensor still for calibration...");
  delay(1000);
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

void GYRO_reading(int currentLoopTime) {
  gyroRate = (analogRead(gyroPin)*gyroSupplyVoltage)/1023;
  gyroRate -= (gyroZeroVoltage/1023*5);
  float angularVelocity = gyroRate/ gyroSensitivity;
  
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T. one second will run (1000/T).  
    float angleChange = angularVelocity/(1000/currentLoopTime); 
    currentAngle += angleChange;
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

// =========================================================================
// Other functions
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
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
