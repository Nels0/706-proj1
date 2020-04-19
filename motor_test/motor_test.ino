#include <Servo.h>

const byte motor1Pin = 46;
const byte motor3Pin = 47;
const byte motor4Pin = 50;
const byte motor2Pin = 51;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

int speedMS = 100;

void setup() {
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
}

void loop() {

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
