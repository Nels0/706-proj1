// ========================================================================
// Defines and includes
//#define NO_BATTERY_V_OK
//#define GYRO_DEBUG
//#define IR_DEBUG 
//#define MOTOR_DEBUG
//#define SONAR_DEBUG
//#define ROTATION_CONTROL_DEBUG
//#define KALMAN_DEBUG
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
int sonarTrigPin = 17;
int sonarEchoPin = 16;

// =========================================================================
// Variables

DEBUG debug_level = NONE;

// Gyro variables
int gyroValue = 0;
float gyroSupplyVoltage = 5;
float gyroZeroVoltage = 0;
float gyroSensitivity = 0.007;
float rotationThreshold = 0.5;
float gyroRate = 0;
float currentAngle = 0;
float desiredAngle = 90;

// Short range IR
  // Front
int numReadingsFront = 5; 
float srIRFrontBuffer[5];
int readIndexFront = 0;
float sumFront = 0;              
float srIRFrontFiltered = 0;   

  // Back
int numReadingsBack = 5; 
float srIRBackBuffer[5];
int readIndexBack = 0;
float sumBack = 0;              
float srIRBackFiltered = 0; 

//Sonar
float sonar_distance = 0.0f;
float signal_duration = 0.0f;

//Kalman Filter
double process_noise = 1;
double sensor_noise = 1;
float prev_Gyro = 0;


// Speed
int L1 = 1;
int L2 = 0;
int Rw = 1;
int maxSpeedValue = 200;
int minSpeedValue = 30; 

// Gains
float rotationGain_P = 30.0f;
float rotationGain_I = 0.0f;
float sidewaysGain_P = 40.0f;

float sidewaysGain_I = 0.0f;
float forwardGain_P = 40.0f;
float forwardGain_I = 0.0f;

// Other
HardwareSerial *SerialCom;
static RUNNING_STATE machine_state;
int loopTime = 10; // Time for each loop in ms
Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;
int turn_count = 0; 
int speed_val = 100;
float angleError; 
float Wz; 

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
      machine_state = running();
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
  static ACTION_STATE action_state = MOVING_FORWARD;
  unsigned int deltaTime = millis() - running_previous_millis;
  
  if (deltaTime >= loopTime) {
    GYRO_reading(deltaTime);
    SR_IR_front_reading();
    SR_IR_back_reading();
    sonar_reading();
    running_previous_millis = millis();
  }

  switch (action_state) {
    case MOVING_FORWARD:
      move_forward(deltaTime);
      // If the distance from the sonar to wall is less than ~15 cm, robot will stop.
      if (sonar_distance < 14.5) action_state = STILL;
      break;
      
    case MOVING_TURNING:
      move_turning();
      // Error and angular velocity transition condition
      // These margins are estimates...
      if (angleError < 0.5 && Wz < 5) {
        turn_count++;
        action_state = MOVING_FORWARD;
      }
      break;
      
    case STILL:
      still();
      // When the robot reaches the finish, it will have turned three times.
      // The program completes at this point. Otherwise, it will continue to rotate.
      if (turn_count == 3) machine_state = STOPPED; 
      else action_state = MOVING_TURNING;
      break;
  }
  
  if (!is_battery_voltage_OK()) return STOPPED;
  return RUNNING;
}

RUNNING_STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;

  stop();
  
  if (millis() - previous_millis > 500) { // Print message every 500ms
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
        return INITIALISING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
  }
  return STOPPED;
}

void still() {
  stop();
}

void move_turning() {

  //error
  angleError = desiredAngle - currentAngle;
   
  if(angleError > 180){
    angleError = angleError - 360;
  } else if (angleError < -180){
    angleError = angleError + 360;
  }
  
  #ifdef ROTATION_CONTROL_DEBUG
    Serial.print("Desired: ");
    Serial.print(desiredAngle);
    Serial.print(" Current: ");
    Serial.print(currentAngle);
    Serial.print(" Error: ");
    Serial.println(angleError);
  #endif

  //P controller that is usually saturated (maybe PD?)
  Wz = clamp(angleError * rotationGain_P, 100, 0);

  // linear velocities are zero, angular velocity is determined by controller
  setSpeeds(0, 0, Wz);
}

void move_forward(int deltaTime) {
  
  /* |     ____=____
   * |  []-|       |-[]    ^ X
   * |     o   ^   |       |
   * |     |   |   |       +---> Y
   * |     o   |   |       
   * |  []-|_______|-[]    Positive angular velocity clockwise
   */   
  
  float Wz = get_Wz(deltaTime);
  float Vy = 0.0f;//get_Vy(deltaTime); // Negative to align itself with the LEFT wall
  float Vx = 0.0f;//get_Vx(deltaTime); 
  setSpeeds(Vx, Vy, Wz);
  // if transition condition met
  float desiredAngle = currentAngle - 90;
}

// Rotational velocity controller
float get_Wz(int deltaTime) {
   float r = 0;
   float IRdifference = srIRFrontFiltered - srIRBackFiltered;
   float error = r - IRdifference;
   float controlDifference = (error * rotationGain_P) + (error * rotationGain_I * deltaTime);
   return controlDifference;
}

// Lateral velocity controller
float get_Vy(int deltaTime) {
   float r = 15;
   float meanDistance = ((srIRFrontFiltered + srIRBackFiltered) / 2);
   float error = r - meanDistance;
   float controlDifference = (error * sidewaysGain_P) + (error * sidewaysGain_I * deltaTime);
   return controlDifference;
}

// Linear velocity controller
float get_Vx(int deltaTime) {
   float r = 15;
   float error = r - sonar_distance;
   float controlDifference = (error * forwardGain_P) + (error * forwardGain_I * deltaTime);
   return controlDifference;
}

int clamp(int value, int maxValue, int minValue) {
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

// =========================================================================
// IR functions
void IR_setup() {
  pinMode(srIRFrontPin,INPUT); 
  pinMode(srIRBackPin,INPUT); 
  pinMode(lrIRPin,INPUT); 
  Serial.println("...");

  // Initialises all buffer values to zero 
  for (int thisReading = 0; thisReading < numReadingsFront; thisReading++) {
    srIRFrontBuffer[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadingsBack; thisReading++) {
    srIRBackBuffer[thisReading] = 0;
  }
}

void SR_IR_front_reading() {

  // subtract the last reading
  sumFront = sumFront - srIRFrontBuffer[readIndexFront];

  srIRFrontBuffer[readIndexFront] = 448.35f * pow(analogRead(srIRFrontPin), -0.593f);
  // add the reading to the total
  
  sumFront = sumFront + srIRFrontBuffer[readIndexFront];
  // advance to the next position in the array
  readIndexFront = readIndexFront + 1;
  // Reset index to zero when at end of array
  if (readIndexFront >= numReadingsFront) {
    readIndexFront = 0;
  }

  srIRFrontFiltered = sumFront / numReadingsFront;

  #ifdef IR_DEBUG
    Serial.print(" SR Front Distance: ");
    Serial.print(srIRFrontBuffer[readIndexFront]);
    Serial.print(" SR Front Filtered Distance: ");
    Serial.print(srIRFrontFiltered);
  #endif
}

void SR_IR_back_reading() {

  // subtract the last reading
  sumBack = sumBack - srIRBackBuffer[readIndexBack];
  
  // read from the sensor
  srIRBackBuffer[readIndexBack] = 448.35f * pow(analogRead(srIRBackPin), -0.593f);
  // add the reading to the total
  sumBack = sumBack + srIRBackBuffer[readIndexBack];
  // advance to the next position in the array
  readIndexBack = readIndexBack + 1;
  // Reset index to zero when at end of array
  if (readIndexBack >= numReadingsBack) {
    readIndexBack = 0;
  }
  srIRBackFiltered = sumBack / numReadingsBack;

  #ifdef IR_DEBUG
    Serial.print(" SR Back Distance: ");
    Serial.print(srIRBackBuffer[readIndexBack]);
    Serial.print(" SR Back Filtered Distance: ");
    Serial.println(srIRBackFiltered);
  #endif
}

//==================================================================
// Sonar Functions


void sonar_setup(){
  Serial.begin(9600);
  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
}

void sonar_reading(){
  // Trigger HIGH pulse of 10
  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrigPin, LOW);

  // Read duration signal
  pinMode(sonarEchoPin, INPUT);
  signal_duration = pulseIn(sonarEchoPin, HIGH);
  // Convert to distance by multiplying by speed of sound, 
  // accounting for returned wave by division of 2
  // offset by 7cm to account for sensor positioning on robot
  //sonar_distance = (signal_duration/2.0)*0.0343 - 7.0;
  sonar_distance = 200;

  #ifdef SONAR_DEBUG
    Serial.print(" Sonar Distance: ");
    Serial.println(sonar_distance);
  #endif
}

// =========================================================================
// Kalman Filter Function

double kalman_filter(double raw_reading, double prev_est)
{
  double pri_est, pri_var, post_est, post_var, gain;

  pri_est = prev_est;
  pri_var = process_noise;

  gain  = pri_var/(pri_var+sensor_noise);
  post_est = pri_est + gain*(raw_reading-pri_est);
  post_var = (1 - gain)*pri_var;
 
  #ifdef KALMAN_DEBUG
    Serial.print(" Gain: ");
    Serial.print(gain);
    Serial.print(" Post Variance: ");
    Serial.print(post_var);
    Serial.print(" Post Estimate: ");
    Serial.println(post_est);
  #endif
  
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

int kinematic_calc(int Vx, int Vy, int Wz) {
    return (int)((Vx + Vy + Wz*(L1 + L2)) / Rw);
}

void write_to_motors(int motor1, int motor2, int motor3, int motor4) {
  left_front_motor.writeMicroseconds(1500 - clamp(motor1, maxSpeedValue, minSpeedValue));
  right_front_motor.writeMicroseconds(1500 + clamp(motor2, maxSpeedValue, minSpeedValue));
  left_rear_motor.writeMicroseconds(1500 - clamp(motor3, maxSpeedValue, minSpeedValue));
  right_rear_motor.writeMicroseconds(1500 + clamp(motor4, maxSpeedValue, minSpeedValue));
}

void setSpeeds(int Vx, int Vy, int Wz){
  /* |      ____=____
   * |  [1]-|       |-[2]   ^ Y
   * |      o   ^   |       |
   * |      |   |   |       +---> X
   * |      o   |   |        Each motor has matching coordinate system
   * |  [3]-|_______|-[4]    Positive angular velocity counter-clockwise
   * 
   * From slides:
   * S1 = (Vx + Vy - (L1 + L2)*Wz)/Rw
   * S2 = (Vx - Vy + (L1 + L2)*Wz)/Rw
   * S3 = (Vx - Vy - (L1 + L2)*Wz)/Rw
   * S4 = (Vx + Vy + (L1 + L2)*Wz)/Rw
   */   
   
  int motor_speed_1 = kinematic_calc(Vx, Vy, -Wz);
  int motor_speed_2 = kinematic_calc(Vx, -Vy, Wz);
  int motor_speed_3 = kinematic_calc(Vx, -Vy, -Wz);
  int motor_speed_4 = kinematic_calc(Vx, Vy, Wz);
  
  write_to_motors(motor_speed_1, motor_speed_2, motor_speed_3, motor_speed_4);
}

void stop()
{
  write_to_motors(0, 0, 0, 0);
  //insure device has stopped with consideration to inertial effects 
  delay(500); 
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
    currentAngle -= angleChange; // Negative due to right handed coordinate system
  }

  // keep the angle between 0-360
  if (currentAngle < 0)    
    currentAngle += 360;
  else if (currentAngle > 359)
    currentAngle -= 360;

   currentAngle = kalman_filter(currentAngle, prev_Gyro);
   prev_Gyro = currentAngle;


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
