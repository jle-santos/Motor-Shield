// Temperature Sensor libraries 
#include <OneWire.h>
#include <DallasTemperature.h>

// Tilt Sensor libraries 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  

// Additional libraries
#include <math.h>

#define TEMP_PIN 8  // Data wire is conntect to the Arduino digital pin 7
#define LSM9DS1_SCL A5
#define LSM9DS1_MISO 12
#define LSM9DS1_SDA A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
#define O2_SENSOR_PIN A0
#define VREF  5
#define G_CONVERT 0.101971621

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_PIN);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Global variables that store the sensor information
int tilt;   // given in degrees 
int temperature; // given in degrees Celcius 
int north;       // given in degrees   
int oxygen;        // given as levels: 0, 1, 2 with 0 being GOOD

//--MOTOR SHIELD--//
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *MOTOR_A = AFMS.getMotor(4);
Adafruit_DCMotor *MOTOR_B = AFMS.getMotor(2);
Adafruit_DCMotor *MOTOR_C = AFMS.getMotor(1);
Adafruit_DCMotor *MOTOR_D = AFMS.getMotor(3);

#define A_PIN 2
#define B_PIN 3
#define EN_PIN 4
#define LED_PIN 13

#define BIT0 5
#define BIT1 6
#define BIT2 7

int PIN_A = 0;
int PIN_B = 0;
int PIN_EN = 0;

int BIT0VAL = 0;
int BIT1VAL = 0;
int BIT2VAL = 0;

// Set up the tilt sensor
void setupTiltSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void setup() {
  // put your setup code here, to run once:  
  
  // Start serial communication for debugging purposes
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    //Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  //Serial.println("Found LSM9DS1 9DOF");

  setupTiltSensor();
  // Start up the library
  sensors.begin();

  //--MOTOR-SHIELD SETUP--//
  DDRD = B00000000;
  /*
  pinMode(A_PIN, INPUT);
  pinMode(B_PIN, INPUT);
  pinMode(EN_PIN, INPUT);
  
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(BIT0, INPUT);
  pinMode(BIT1, INPUT);
  pinMode(BIT2, INPUT);
  */
  
  digitalWrite(LED_PIN, LOW);

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  MOTOR_A->setSpeed(255);
  MOTOR_B->setSpeed(255);
  MOTOR_C->setSpeed(255);
  MOTOR_D->setSpeed(255);
  // turn on motor
  MOTOR_A->run(RELEASE);
  MOTOR_B->run(RELEASE);
  MOTOR_C->run(RELEASE);
  MOTOR_D->run(RELEASE);
  
}

void loop() {
  //--MOTOR-SHIELD--//
  PIN_A = (PIND & B00000100) >> 2;//digitalRead(A_PIN);
  PIN_B = (PIND & B00001000) >> 3;//digitalRead(B_PIN);
  PIN_EN = (PIND & B00010000) >> 4; //digitalRead(EN_PIN);
  //Serial.print(PIN_EN);

  BIT0VAL = (PIND & B00100000) >> 5;
  BIT1VAL = (PIND & B01000000) >> 6;
  BIT2VAL = (PIND & B10000000) >> 7;

  byte i = (getSpeed(BIT0VAL, BIT1VAL, BIT2VAL))*32;
  
  MOTOR_A->setSpeed(i);
  MOTOR_B->setSpeed(i);
  MOTOR_C->setSpeed(i);
  MOTOR_D->setSpeed(i); 
  
  checkDir(PIN_A, PIN_B, PIN_EN);

  // put your main code here, to run repeatly:
  lsm.read();  /* ask it to read in the tilt sensor data */ 

   // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures(); 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  int direction;
  float accel_x = a.acceleration.x*G_CONVERT;
  float accel_y = a.acceleration.y*G_CONVERT; 
  float accel_z = a.acceleration.z*G_CONVERT; 

  float check; // Used for tilt testing 

  int level;
 
  // Calculate north direction (assuming flat surface) 
  if (m.magnetic.y > 0) {
    direction = 90 - atan(m.magnetic.x/m.magnetic.y)*(180/PI);
  }
  else if (m.magnetic.y < 0) {
    direction = 270 - atan(m.magnetic.x/m.magnetic.y)*(180/PI);
  }
  else if (m.magnetic.y == 0 && m.magnetic.x < 0) {
    direction = 180;
  }
  else if (m.magnetic.y == 0 && m.magnetic.x > 0) {
    direction = 0;
  }

//  When static, check should be near 1g 
//  check = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));
//  Calculate the tilt 
  tilt = atan(accel_x/accel_z)*180/PI;

// Measure the temperature 
  temperature = sensors.getTempCByIndex(0);

// Measure the oxygen  
  oxygen = ((analogRead(O2_SENSOR_PIN) * (VREF / 1023.0)) - 0.06) * 11.0097;

  if (oxygen >= 19.6)
    level = 0;
  else if (oxygen >= 16.0 && oxygen < 19.6)
    level = 1;
  else if (oxygen < 16.0)
    level = 2;

  Serial.print(direction); Serial.print(" "); 
  Serial.print(tilt); Serial.print(" ");
  Serial.print(temperature); Serial.print(" ");
  Serial.print(level); Serial.print(" ");
  Serial.println();
 
}

/*
float readO2Vout(uint8_t analogPin) {
  // Vout samples are with reference to 5V
  float MeasuredVout = (analogRead(analogPin) * (VREF / 1023.0)) - 0.06;
  return MeasuredVout;
}

float readConcentration(uint8_t analogPin) {
  float o2_concentrationInPercent;

  // Equation derived from linear relationship between O2 and voltage out 
  o2_concentrationInPercent = 

  return o2_concentrationInPercent; 
}
*/

byte getSpeed(int A, int B, int C) {
  byte speedControl = 0x00;

  bitWrite(speedControl, 0, A);
  bitWrite(speedControl, 1, B); 
  bitWrite(speedControl, 2, C);  
  
  return speedControl;
}

void checkDir(int A, int B, int EN) {
  if(EN == HIGH){
    //digitalWrite(LED_PIN, HIGH);
    //Backward
    if(A == HIGH && B == HIGH){
      MOTOR_A->run(BACKWARD);
      MOTOR_B->run(FORWARD);
      MOTOR_C->run(BACKWARD);
      MOTOR_D->run(FORWARD);
    } //FORWARD
    else if(A == LOW && B == LOW) {
      MOTOR_A->run(FORWARD);
      MOTOR_B->run(BACKWARD);
      MOTOR_C->run(FORWARD);
      MOTOR_D->run(BACKWARD);
    } //CLOCKWISE
    else if(A == LOW && B == HIGH) {
      MOTOR_A->run(BACKWARD);
      MOTOR_B->run(BACKWARD);
      MOTOR_C->run(BACKWARD);
      MOTOR_D->run(BACKWARD);
    } //COUNTER CLOCKWISE
    else if(A == HIGH && B == LOW) { 
      MOTOR_A->run(FORWARD);
      MOTOR_B->run(FORWARD);
      MOTOR_C->run(FORWARD);
      MOTOR_D->run(FORWARD);
    }
  }
  else {
      MOTOR_A->run(RELEASE);
      MOTOR_B->run(RELEASE);
      MOTOR_C->run(RELEASE);
      MOTOR_D->run(RELEASE);

  //digitalWrite(LED_PIN, LOW);
  }
}
