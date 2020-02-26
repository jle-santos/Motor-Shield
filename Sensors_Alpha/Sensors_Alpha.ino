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

#define ONE_WIRE_BUS 8  // Data wire is conntect to the Arduino digital pin 7
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
#define O2_SENSOR_PIN A0
#define VREF  5
#define G_CONVERT 0.101971621

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Global variables that store the sensor information
int tilt;   // given in degrees 
int temperature; // given in degrees Celcius 
int north;       // given in degrees   
int oxygen;        // given as levels: 0, 1, 2 with 0 being GOOD


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
}

void loop() {

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
  oxygen = readConcentration(O2_SENSOR_PIN);

  if (oxygen >= 19.6)
    level = 0;
  else if (oxygen >= 14.0 && oxygen < 19.6)
    level = 1;
  else if (oxygen < 14.0)
    level = 2;

  Serial.print(direction); Serial.print(" "); 
  Serial.print(tilt); Serial.print(" ");
  Serial.print(temperature); Serial.print(" ");
  Serial.print(level); Serial.print(" ");
  Serial.println();
  
//   Serial.print("\nNorth: "); Serial.print(direction); Serial.print("°");
//   Serial.println();
//   Serial.print("Tilt: "); Serial.print(tilt); Serial.print("°");
//   Serial.println();
//   Serial.print("Temperature: "); Serial.print(temperature); Serial.print("°C");
//   Serial.println();
//   Serial.print("Oxygen: "); Serial.print(oxygen); Serial.print("%");
//   Serial.println(); 
}

float readO2Vout(uint8_t analogPin) {
  // Vout samples are with reference to 5V
  float MeasuredVout = (analogRead(analogPin) * (VREF / 1023.0)) - 0.06;
  return MeasuredVout;
}

float readConcentration(uint8_t analogPin) {
  float o2_concentrationInPercent;

  // Equation derived from linear relationship between O2 and voltage out 
  o2_concentrationInPercent = readO2Vout(analogPin) * 11.0097;

  return o2_concentrationInPercent; 
}
