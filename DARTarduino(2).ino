//DART low-level control application
/*
   This application provides the basic functions of sensor reading and motor control. It
   is one part of the DART system, meant to interface with the onboard Raspberry Pi running
   a Python script.
   The program:
     - reads all connected sensors (IMU, sonar, current sensors, FSR)
     - controls the connected motor drivers( 4 motor circuits available)
     - controls the piezo speaker
     - bidirectionally transmits data serially to the Pi ( sensor data out, motor commands in)

   The complex stuff, such as kalman filtering, PID, mapping, etc as well as the GPS receiver
   and bluetooth connection, will be handled by the Pi.

   Thanks to the writers of the Adafruit sensor librarie and the BNO055 library, as well as the NewPing library Author, for
   the framework provided with the example codes.
*/
/* Board layout:
           +----------+
           |         *| RST   PITCH  ROLL  HEADING(YAW)
       ADR |*        *| SCL
       INT |*        *| SDA     ^            /->
       PS1 |*        *| GND     |            |
       PS0 |*        *| 3VO     Y    Z-->    \-X
           |         *| VIN
           +----------+
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NewPing.h>
#include <EEPROM.h>
//define sonar
#define SONAR_NUM 4      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define TRIG1 MISO
#define TRIG2 A4
#define TRIG3 16

#define ECHO1 MISO
#define ECHO2 A4
#define ECHO3 16

#define TRIG4 SCK
#define ECHO4 A5

//define current sensor pins
#define CURRENT1 A3
#define CURRENT2 A2
#define CURRENT3 A1
//battery voltage ADC
//#define BAT A5

//define FSR pin
#define FSR A0

//define Piezo speaker pin
#define PIEZO 17

//define motor pins
#define PWM1 6
#define PWM2 5
#define PWM3 10
#define PWM4 11
#define DIR1 9
#define DIR2 12
#define DIR3 7
#define DIR4 8

#define CAL1 5
#define CAL2 0
#define CAL3 0
#define CAL4 0


NewPing sonar[SONAR_NUM] = {   // Sensor object array - set pins and max distance of sensors

  NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE),
  NewPing(TRIG4, ECHO4, MAX_DISTANCE),
};

Adafruit_BNO055 bno = Adafruit_BNO055(55);



//global variables

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
int incomingByte;
const long interval = 100;
const long interval2 = 3000;
float systime;
float Current[3];
int sonarMeas[SONAR_NUM];
int FSRconvert;
bool flag = false;
int BATadc = 0;
float voltage = 0;
int motor1State = 0;
int motor2State = 0;
int motor3State = 0;
int motor4State = 0;
imu::Vector<3> orient;
imu::Vector<3> compass;
//imu::Vector<3> grav;
imu::Vector<3> linAccel;

char motorin[21];
int motorTempArray[4];
int cnt = 0;
bool readflag = false;

int set_motor(int motornum, int sp) { //Motor set function - sets given motor to given speed.
  // pass in the motor number (1 - 4) and the motor speed (-255 - 255) to set motor to appropriate speeds.
  //negative numbers for speed indicates opposite direction
  //1 PWM and 1 DIR pin is connected for each motor.

  if (motornum == 1) {
    motor1State = sp;
    if (sp < 0) {
      sp = -sp;
      digitalWrite(DIR1, LOW);
    }
    else {
      digitalWrite(DIR1, HIGH);
    }
    sp = sp - CAL1;
    if (sp < 0) {
      sp = 0;
    }
    analogWrite(PWM1, sp);
    return (sp);
  }
  else if (motornum == 2) {
    motor2State = sp;
    if (sp < 0) {
      sp = -sp;
      digitalWrite(DIR2, LOW);
    }
    else {
      digitalWrite(DIR2, HIGH);
    }
    sp = sp - CAL2;
    if (sp < 0) {
      sp = 0;
    }
    analogWrite(PWM2, sp);
    return (sp);
  }
  else if (motornum == 3) {
    motor3State = sp;
    if (sp < 0) {
      sp = -sp;
      digitalWrite(DIR3, LOW);
    }
    else {
      digitalWrite(DIR3, HIGH);

    }
    sp = sp - CAL3;
    if (sp < 0) {
      sp = 0;
    }
    analogWrite(PWM3, sp);
    return (sp);
  }
  else if (motornum == 4) {
    motor4State = sp;
    if (sp < 0) {
      sp = -sp;
      digitalWrite(DIR4, LOW);
    }
    else {
      digitalWrite(DIR4, HIGH);
    }
    sp = sp - CAL4;
    if (sp < 0) {
      sp = 0;
    }
    analogWrite(PWM4, sp);
    return (sp);
  }
  else {
    return (0);
  }

}

void motorParse(char motorcom[21]) { // Motor Parsing Function
    //This function is called whenever a full line of data is received from the Pi (denoted by the newline char).
    //When called, it parses the char array and decides which motor channel to power and at what speed/direction.
    //Input data from the Pi looks like : M1125N0231P1254Q1115 where MNPQ denote which channel. The first number is a bool
    //that represents direction (0 for backward, 1 for forward). The next three numbers are the speed of the motor, from
    //0 to 255. The input data is an array of char due to the nature of the serial input, and must first be translated from
    //ASCII code to the proper number by subtracting 48., then combining into an int type by multiplying the decimal places
    //by the appropriate factor.
  for (int i = 0; i < 21; i++) {
    int tmpval = 0;
    if (motorcom[i] == 'M') {
        motorTempArray[0] = motorcom[i+1] - 48;
        motorTempArray[1] = motorcom[i+2] - 48;
        motorTempArray[2] = motorcom[i+3] -48;
        motorTempArray[3] = motorcom[i+4] -48;
        (motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]);
        if (motorTempArray[0] == 0){
             set_motor(1, -(motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]));
        }
        else{
             set_motor(1, (motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]));
        }
    }
    if (motorcom[i] == 'N') {
        motorTempArray[0] = motorcom[i+1] - 48;
        motorTempArray[1] = motorcom[i+2] - 48;
        motorTempArray[2] = motorcom[i+3] -48;
        motorTempArray[3] = motorcom[i+4] -48;
        (motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]);
        if (motorTempArray[0] == 0){
             set_motor(2, -(motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]));
        }
        else{
             set_motor(2, (motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]));
        }
    }
    if (motorcom[i] == 'P') {
        motorTempArray[0] = motorcom[i+1] - 48;
        motorTempArray[1] = motorcom[i+2] - 48;
        motorTempArray[2] = motorcom[i+3] -48;
        motorTempArray[3] = motorcom[i+4] - 48;
        if (motorTempArray[0] == 0){
             set_motor(3, -(motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]));
        }
        else{
             set_motor(3, (motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]));
        }
    }
    if (motorcom[i] == 'Q') {
        motorTempArray[0] = motorcom[i+1] - 48;
        motorTempArray[1] = motorcom[i+2] - 48;
        motorTempArray[2] = motorcom[i+3] -48;
        motorTempArray[3] = motorcom[i+4] -48;
        
        if (motorTempArray[0] == 0){
             set_motor(4, -(motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]));
        }
        else{
             set_motor(4, (motorTempArray[1]*100 +motorTempArray[2]*10 + motorTempArray[3]));
        }
    }
  }
}

void displaySensorDetails(void) { //Sensor Details for IMU, if necessary.
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void setup(void) {

  Serial.begin(9600);
  pinMode(FSR, INPUT);
  //pinMode(BAT, INPUT);
  pinMode(CURRENT1, INPUT);
  pinMode(CURRENT2, INPUT);
  pinMode(CURRENT3, INPUT);
  pinMode(ECHO1, OUTPUT);
  pinMode(ECHO2, OUTPUT);
  pinMode(ECHO3, OUTPUT);
  pinMode(ECHO4, INPUT);
  pinMode(PIEZO, OUTPUT);

  pinMode(PWM2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(DIR2, OUTPUT);

  if (!bno.begin())
  {

    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) {
      Serial.println("no IMU");
      delay(1000);
    }
  }
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;
  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  bno.getSensor(&sensor);
  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, calibrationData);
  bno.setSensorOffsets(calibrationData);
  
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  /* Display some basic information on this sensor */
  //displaySensorDetails();

}

void loop(void) {

  unsigned long currentMillis = millis();

  //Here will be where motor commands will be received and set. for now, it contains a placeholder which
  //reads serial data if it is available in the buffer, and sets it to incomingByte
  //Two methods of implementation exist for this: either motor commands will be sent as 4 sets of bytes (byte is
  //8-bit e.g. 256 values, which is good for motor commands because they work using 8 bit numbers for PWM
  //The transmission would look like:
  //01111101 01111101 01111101 01111101
  // (this sets all motors to 125)
  //to parity check, you can do a size check (should be 4 bytes, 32 bits)
  //or you can hash the numbers before sending, send the hash with the motors (adding 1-2 bytes)
  //and then hash the motor numbers again when the arrive, then compare the two hashes

  //option 2 for motor command transmission: as strings
  //Serial.readString() (with timeout) or Serial.readStringUntil() (using a termination character that is also
  //transmitted by the Pi.)
  // This method may be simpler, just look if there is stuff to read, then readStringUntil() the termination char
  // if there is. you can even have a character for each motor, e.g. A B C D denotes the beginning or end of a motor command
  // lots of ways to do it this way, but need to look into Python/Arduino string compatibility too. (sometimes weird chars
  // are sent)
  while (Serial.available()) {
    char c = Serial.read();
    motorin[cnt++] = c;
    if ((c == '\n') || (cnt == sizeof(motorin) - 1))
    {
      motorin[cnt] = '\0';
      cnt = 0;
      readflag = true;
    }
  }
  if (readflag == true) {
    //here the parsing function will read the input and set motor values from it. should be in a null-terminated char buffer
    //Serial.print(motorin);
    readflag = false;
    motorParse(motorin);
  }


  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //ACQUIRING MEASUREMENT DATA

    //get system time
    systime = float(currentMillis) / 1000;
    //get currents
    int Cadc1 = analogRead(CURRENT1);
    int Cadc2 = analogRead(CURRENT2);
    int Cadc3 = analogRead(CURRENT3);
    //get FSR reading
    int FSRadc0 = analogRead(FSR);
    //get battery voltage reading
    //BATadc = analogRead(BAT);
    

    //Convert ADC value to actual
    FSRconvert = exp(FSRadc0 * 0.0047) * 0.15;
    Current[0] = Cadc3 * 0.0382 - 3.94; //sensor 1
    Current[1] = Cadc1 * 0.0494 - 25.3; //sensor 5
    // float Current2 = Cadc3 * 0.0496 - 25.3; //sensor 4
    Current[2] = Cadc2 * 0.0385 - 3.99; //sensor 2
    //voltage = ((BATadc/1023)*5)/(.277);

    //get sonar readings

    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
      sonarMeas[i] = sonar[i].ping_cm();
    }
    //get orientation, acceleration, and gravity vectors
    linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    //grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    orient = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    compass = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);


    ///
    ///SERIAL TRANSMISSION SECTION
    ///
    ///
    Serial.print("T");
    Serial.print(systime);
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      Serial.print("S");
      if (i == 0) {
        Serial.print("a");
      }
      if (i == 1) {
        Serial.print("b");
      }
      if (i == 2) {
        Serial.print("c");
      }
      if (i == 3) {
        Serial.print("d");
      }
      Serial.print(sonarMeas[i]);
    }
    for (uint8_t i = 0; i < 3; i++) {
      Serial.print("C");
      if (i == 0) {
        Serial.print("a");
      }
      if (i == 1) {
        Serial.print("b");
      }
      if (i == 2) {
        Serial.print("c");
      }

      Serial.print(Current[i]);
    }
    

    Serial.print("Ox");
    Serial.print(orient.x());
    Serial.print("Oy");
    Serial.print(orient.y());
    Serial.print("Oz");
    Serial.print(orient.z());

    Serial.print("Ax");
    Serial.print(linAccel.x());
    Serial.print("Ay");
    Serial.print(linAccel.y());
    Serial.print("Az");
    Serial.print(linAccel.z());

    Serial.print("Mx");
    Serial.print(compass.x());
    Serial.print("My");
    Serial.print(compass.y());
    Serial.print("Mz");
    Serial.print(compass.z());

    Serial.print("FSR");
    Serial.print(FSRconvert);
    

    Serial.print("M");
    Serial.print(motor1State);
    Serial.print("N");
    Serial.print(motor2State);
    Serial.print("P");
    Serial.print(motor3State);
    Serial.print("Q");
    Serial.print(motor4State);
    //Serial.print("V");
    //Serial.print(voltage);
    Serial.println();
  }
  //need functions to test straight and turning operations, and add calibration function
  //this section of code flips the motor on, then off, then reverse, then off. 3 sec intervals
  /**
  
    if (currentMillis - previousMillis2 >= interval2) {
      previousMillis2 = currentMillis;

      
      
      //if motor is on, turn it off
      
      if ((motor1State == 255) || (motor1State == -255)) {
        motor1State = set_motor(1, 0);
        motor2State = set_motor(2, 0);
        motor3State = set_motor(3, 0);
        motor4State = set_motor(4, 0);
      }
      else { //else if motor is off, pick a direction to turn and activate
        if (flag) {
          motor1State = set_motor(1, 255);
          motor2State = set_motor(2, 255);
          motor3State = set_motor(3, 255);
          motor4State = set_motor(4, 255);

          flag = false;
        }
        else {
          motor1State = set_motor(1, -255 );
          motor2State = set_motor(2, -255 );
          motor3State = set_motor(3, -255 );
          motor4State = set_motor(4, -255 );

          flag = true;
        }
    }
    }
    **/
  

}

