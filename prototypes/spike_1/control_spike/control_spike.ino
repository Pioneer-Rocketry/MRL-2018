#define ARDUINO_MICRO

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "kalmanfilter.h"
#include <math.h>
#include <Servo.h>
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>

//#include "dimensionals.h"
//#include "quatops.h"
#include "sensorhub.h"

#ifndef ARDUINO_MICRO
#include "AbstractServo.h"
#include "controller.h"
#endif

#define P_TUNING 1
#define I_TUNING 0
#define D_TUNING 0.5

#define SERVO_ONE_PIN 4
#define SERVO_TWO_PIN 5

#define MAX_SERVO 2000
#define MIN_SERVO 1000
#define MID_SERVO 1500

#define SERVO_RANGE 

//AbstractServo servo(1100, 1940, 4, -45.0f, 45.0f);
//AbstractServo servo2(1100, 1940, 5, -45.0f, 45.0f);

//AbstractServo controlServo(1000, 2000, 3, -250.0f, 250.0f);
//AbstractServo controlServoTwo(1000, 2000, 4, -250.0f, 250.0f);

enum states{PRELAUNCH, PREBURNOUT, BURNOUT, POSTAPOGEE, LANDING};

states launchState = PRELAUNCH;

Servo servo1 = Servo();
Servo servo2 = Servo();

File dataFile;

const int chipSelect = 3;
const float accLimitFreeFall = .1;
const float accLimitLaunch = 1;
const int preBurnoutLength = 2500;


unsigned long preLaunchStartTime = 0;

float power;

String dataString;

bool impactDetected = false;

void setup() {
  // put your setup code here, to run once:

  SensorHub::init();
  Serial.begin(57600);
  Serial1.begin(57600);

  servo1.attach(SERVO_ONE_PIN);
  servo2.attach(SERVO_TWO_PIN);

  Serial.print("Initializing SD card...");

//  pinMode(chipSelect, OUTPUT);
//  digitalWrite(chipSelect, HIGH);
  if (!SD.begin()) {
    Serial.println("Card failed, or not present");

  }
  else
  {
    Serial.println("card initialized.");

    dataFile = SD.open("datalog1.csv", FILE_WRITE);

    dataFile.print("time (ms), Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz, Altitude\n");
    dataFile.flush();
    dataFile.close();
  }

  
}

void loop() {

  SensorHub::update();

  
  switch(launchState){
    case PRELAUNCH:
      if( SensorHub::getAccel().z > accLimitLaunch ){
        launchState = PREBURNOUT;
        preLaunchStartTime = millis();
      }
      break;
    case PREBURNOUT:

      if( millis() - preLaunchStartTime >= preBurnoutLength ){
        launchState = BURNOUT;
      }
    case BURNOUT:

      power = sin((float)millis()/1000.0f) * 166.6f + MID_SERVO;

      servo1.writeMicroseconds((int)power);
      servo2.writeMicroseconds((int)power);

      if( SensorHub::getAccel().z > accLimitFreeFall ){
        launchState = POSTAPOGEE;
      }
    case POSTAPOGEE:
      if( impactDetected == true ){
        launchState = LANDING;
      }
    default:
      launchState = LANDING; /* Was not sure what to do for the default but this 
                                this made sense to me */
    
  }
  
    dataFile.print("time (ms), Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz, Altitude\n");
    dataString += millis();
    dataString += " , ";

    dataString += SensorHub::getGyro().x;
    dataString += " , ";

    dataString += SensorHub::getGyro().y;
    dataString += " , ";

    dataString += SensorHub::getGyro().z;
    dataString += " , ";


    dataString += SensorHub::getAccel().x;
    dataString += " , ";

    dataString += SensorHub::getAccel().y;
    dataString += " , ";

    dataString += SensorHub::getAccel().z;
    dataString += " , ";

    
    dataString += SensorHub::getMag().x;
    dataString += " , ";

    dataString += SensorHub::getMag().y;
    dataString += " , ";

    dataString += SensorHub::getMag().z;
    dataString += " , ";

    dataString += SensorHub::getAltitude();

    dataString += "\n";

    Serial1.print(dataString);

    dataFile = SD.open("datalog1.csv", FILE_WRITE);
  
    if (dataFile) {
      Serial.println("Written");
      dataFile.print(dataString);
      dataFile.flush();
      dataFile.close();
    }

}


