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

// ======================== Enums =========================

// =================== Pin assignments ====================

// const byte fanServoPin = ;  

Servo fanServo;

// ====================== Variables ======================= 

// Fan Servo Motor
int minFanServoPulseValue = 900;
int maxFanServoPulseValue = 2100; 
float kP_Fan = 0.001f;


// ================== Arduino functions ===================
void setup() {
}

void loop() {
}

void EnableMotors(){
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
  fanServo.attach(fanServoPin);
}

void fanServoMotorWrite() {
    float error = photo1 - photo4; // use phototransistors to allign servo
    float controlEffort = kP_Fan * error; 
    float fanServoPulse = Sat2(controlEffort, 400, -400);

    if (fanServoPulse >= 20) {
        fanServo.writeMicroseconds(1500 + Sat2(fanServoPulse, maxFanServoPulseValue, minFanServoPulseValue));
    } else
    {
        // enter state transition variable here
    }
    
}

