
#include <AFMotor.h>                                 // the Adafruit Motor Shield Library ver. 1 https://learn.adafruit.com/adafruit-motor-shield/library-install
#include "Wire.h"                                                 // Used by I2C and HMC5883L compass#include "I2Cdev.h"                                             // I2C Communications Library (used for compass)
#include "HMC5883L.h"                                             // Library for the compass - Download from Github @ https://github.com/jarzebski/Arduino-HMC5883L
#include <Servo.h>                                                // Servo library to control Servo arm for metal detector
#include <SoftwareSerial.h>                                       // Software Serial for Serial Communications - not used
#include <TinyGPS++.h>                                            // Tiny GPS Plus Library - Download from http://arduiniana.org/libraries/tinygpsplus/
                                                                  // TinyGPS++ object uses Hardware Serial 2 and assumes that you have a
                                                                  // 9600-baud serial GPS device hooked up on pins 16(tx) and 17(rx).                                                                  

//******************************************************************************************************                                                                  
// GPS Variables & Setup

int GPS_Course;                                                    // variable to hold the gps's determined course to destination
int Number_of_SATS;                                                // variable to hold the number of satellites acquired
TinyGPSPlus gps;                                                   // gps = instance of TinyGPS 
                                                                   // pin 17 (blue)   is connected to the TX on the GPS
                                                                   // pin 16 (yellow) is connected to the RX on the GPS

//******************************************************************************************************
// Setup Drive Motors using the Adafruit Motor Controller version 1.0 Library

AF_DCMotor motor1(1, MOTOR12_64KHZ);                               // create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ);                               // create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR12_64KHZ);                               // create motor #3, 64KHz pwm
AF_DCMotor motor4(4, MOTOR12_64KHZ);                               // create motor #4, 64KHz pwm

int turn_Speed = 175;                                              // motor speed when using the compass to turn left and right
int mtr_Spd = 250;                                                 // motor speed when moving forward and reverse

//******************************************************************************************************
// Compass Variables & Setup

HMC5883L compass;                                                  // HMC5883L compass(HMC5883L)
int16_t mx, my, mz;                                                // variables to store x,y,z axis from compass (HMC5883L)
int desired_heading;                                               // initialize variable - stores value for the new desired heading
int compass_heading;                                               // initialize variable - stores value calculated from compass readings
int compass_dev = 5;                                               // the amount of deviation that is allowed in the compass heading - Adjust as Needed
                                                                   // setting this variable too low will cause the robot to continuously pivot left and right
                                                                   // setting this variable too high will cause the robot to veer off course

int Heading_A;                                                     // variable to store compass heading
int Heading_B;                                                     // variable to store compass heading in Opposite direction
int pass = 0;                                                      // variable to store which pass the robot is on

//******************************************************************************************************
// Servo Control

Servo myservo;                                                     // create servo object to control a servo 
int pos = 0;                                                       // variable to store the servo position

//******************************************************************************************************
 //Ping Sensor for Collision Avoidance

boolean pingOn = false;                                            // Turn Collision detection On or Off

int trigPin = 41;                                                  // Trig - Orange
int echoPin = 40;                                                  // Echo - Yellow
long duration, inches;
int Ping_distance;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;                                  // Store last time Ping was updated
const long interval = 200;                                         // Ping the Distance every X miliseconds
 
//******************************************************************************************************
// Bluetooth Variables & Setup

String str;                                                        // raw string received from android to arduino
int blueToothVal;                                                  // stores the last value sent over via bluetooth
int bt_Pin = (19, 18);                                                   
int ledPin1 = 30;
int ledPin2 = 31;
  
//*****************************************************************************************************
// GPS Locations

unsigned long Distance_To_Home;                                    // variable for storing the distance to destination

int ac =0;                                                         // GPS array counter
int wpCount = 0;                                                   // GPS waypoint counter
double Home_LATarray[50];                                          // variable for storing the destination Latitude - Only Programmed for 5 waypoint
double Home_LONarray[50];                                          // variable for storing the destination Longitude - up to 50 waypoints


int increment = 0;

void setup() 
{  
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer
  Serial1.begin(9600);                                             // Serial 1 is for Bluetooth communication - DO NOT MODIFY - JY-MCU HC-06 v1.40
  Serial2.begin(9600);                                             // Serial 2 is for GPS communication at 9600 baud - DO NOT MODIFY - Ublox Neo 6m 
  myservo.attach(51);                                               // attaches the servo to pin 9 (Servo 0 on the Adafruit Motor Control Board)
  pinMode(A3, OUTPUT);                                             // define pin 36 as an output for an LED indicator - Not Used
  pinMode(bt_Pin, INPUT);                                          // This pin(34) is used to check the status of the Bluetooth connection - Not Used

  // Ping Sensor
  pinMode(trigPin, OUTPUT);                                        // Ping Sensor
  pinMode(echoPin, INPUT);   
  // bt
    pinMode(ledPin1, OUTPUT);    
     pinMode(ledPin2, OUTPUT);  

  // Compass
  Wire.begin();                                                    // Join I2C bus used for the HMC5883L compass
  compass.begin();                                                 // initialize the compass (HMC5883L)
  compass.setRange(HMC5883L_RANGE_1_3GA);                          // Set measurement range  
  compass.setMeasurementMode(HMC5883L_CONTINOUS);                  // Set measurement mode  
  compass.setDataRate(HMC5883L_DATARATE_30HZ);                     // Set data rate  
  compass.setSamples(HMC5883L_SAMPLES_8);                          // Set number of samples averaged  
  compass.setOffset(-255,14,260);                                          // Set calibration offset 

  Startup();                                                       // Run the Startup procedure on power-up one time
}

//********************************************************************************************************
// Main Loop

void loop()
{ 
  
  bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT                                                    
  getGPS();                                                        // Update the GPS location
  getCompass();                                                    // Update the Compass Heading
  Ping();                                                          // Use at your own discretion, this is not fully tested

}
