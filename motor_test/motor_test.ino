#include <Servo.h>

HardwareSerial *SerialCom;
const byte numChars = 5;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;



const byte motor1Pin = 46;
const byte motor3Pin = 47;
const byte motor4Pin = 50;
const byte motor2Pin = 51;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

int speedMS = 120;
float w = 0.0005;

float omegaToPulse = 21.3;
int maxSpeedValue = 250;
int minSpeedValue = 0; 
float L1 = 7.5; //distance from centre to front axe
float L2 = 8.5; //distance from centre to left/right wheen centres
float Rw = 2.25; //wheel radius in cm

void setup() {
  SerialCom = &Serial;
  SerialCom->begin(115200);
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
}

void loop() {
recvWithEndMarker();
read_serial_command();

  float A = 20;


  float Vx = A * cos( w * millis());
  float Vy = A * sin( w * millis());

  MotorWrite(Vx, Vy, 0);
  
/*
  if (is_battery_voltage_OK()){ 
    motor1.writeMicroseconds(1500 + speedMS);
    motor2.writeMicroseconds(1500 - speedMS);
    motor3.writeMicroseconds(1500 + speedMS);
    motor4.writeMicroseconds(1500 - speedMS);
  } else {
    motor1.writeMicroseconds(1500);
    motor2.writeMicroseconds(1500);
    motor3.writeMicroseconds(1500);
    motor4.writeMicroseconds(1500);
  }
  */
}
//Serial command pasing
void read_serial_command()
{
  if (newData) {
    if(receivedChars[0] == 's'){
      SerialCom->println("stopping");    
      speedMS = 0;
    } else {
      SerialCom->print("starting: ");
      delay(2000);
      w = atoi(receivedChars);
      SerialCom->println(speedMS);
      
    }
    newData = false;
  }
}


void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
   
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
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


    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
