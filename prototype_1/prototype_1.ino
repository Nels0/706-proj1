

//Uncomment of BATTERY_V_OK if you do not care about battery damage.
//#define NO_BATTERY_V_OK

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Serial Pointer
HardwareSerial *SerialCom;


// GYRO PARAMETERS
int gyroPin = A2;                 //define the pin that gyro is connected
int T = 100;                        // T is the time of one loop
int gyroValue = 0;                // read out value of sensor
float gyroSupplyVoltage = 5;        // supply voltage for gyro
float gyroZeroVoltage = 0;          // the value of voltage when gyro is zero 
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity less than this value will not be ignored
float gyroRate = 0;                 // read out value of sensor in voltage
float currentAngle = 0;             // current angle calculated by angular velocity integral on
byte serialRead = 0;                // for serial print control



void setup() {
  // put your setup code here, to run once:

  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");
    delay(1000); //settling time but no really needed



    gyro_setup();


}

void loop() {
  // put your main code here, to run repeatedly:
  
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };

}

STATE initialising() {
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("RUNNING STATE...");
  return RUNNING;

}

STATE running() {
  #ifndef NO_READ_GYRO
    GYRO_reading();
  #endif

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
  #endif

  return RUNNING;
}

STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        //enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

#ifndef NO_BATTERY_V_OK
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
    //SerialCom->print("Lipo level:");
    //SerialCom->print(Lipo_level_cal);
    //SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    //SerialCom->println("");
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
#endif


void gyro_setup() {
  // this section is initialize the sensor, find the the value of voltage when gyro is zero
  int i;
  float sum = 0;    
  pinMode(gyroPin,INPUT);
  
  Serial.println("please keep the sensor still for calibration");
  delay(1000);
  Serial.println("get the gyro zero voltage");

  //  read 100 values of voltage when gyro is at still, to calculate the zero-drift
  for (i=0; i<100; i++) {  
    gyroValue = analogRead(gyroPin);  
    sum += gyroValue;  
    delay(5);
  }
  gyroZeroVoltage = sum/100;    // average the sum as the zero drifting
  Serial.println("Gyro Calibrated");
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{

  gyroRate = (analogRead(gyroPin)*gyroSupplyVoltage)/1023;

  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage/1023*5);

  // read out voltage divided the gyro sensitivity to calculate the angular velocity 
  float angularVelocity = gyroRate/ gyroSensitivity;
  
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T. one second will run (1000/T).  
    float angleChange = angularVelocity/(1000/T); 
    currentAngle += angleChange;   
  }

  // keep the angle between 0-360
  if (currentAngle < 0)    
  {
    currentAngle += 360;
  }  
  else if (currentAngle > 359)
  {
    currentAngle -= 360;
  }
  
  Serial.print("Angular Velocity: ");
  Serial.print(angularVelocity);
  Serial.print(" Current Angle: ");
  Serial.println(currentAngle);// control the time per loopdelay (T);
  
}
#endif
