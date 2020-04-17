// ========================================================================
// Defines and includes
//#define NO_BATTERY_V_OK
//#define GYRO_DEBUG
//#define IR_DEBUG 
//#define MOTOR_DEBUG
//#define SONAR_DEBUG
//#define ROTATION_CONTROL_DEBUG
#include <Servo.h>

// ======================Enums===================================================
// 

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


// ====================Pin assignments================================================
//Sensors
int gyroPin = A2;
//IR
int srIRFrontPin = A4;
int srIRBackPin = A5;
//Sonar
const byte sonarTrigPin = 17;
const byte sonarEchoPin = 16;
//Motors
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;



// ======================Variables===================================================
// 


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
