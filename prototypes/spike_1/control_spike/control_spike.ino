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

#include "dimensionals.h"
#include "quatops.h"
#include "sensorhub.h"
#include "AbstractServo.h"
#include "controller.h"

#define P_TUNING 1
#define I_TUNING 0
#define D_TUNING 0.5


AbstractServo servo(1100, 1940, 2, -45.0f, 45.0f);

AbstractServo controlServo(1000, 2000, 3, -250.0f, 250.0f);
AbstractServo controlServoTwo(1000, 2000, 4, -250.0f, 250.0f);

enum states{PRELAUNCH, PREBURNOUT, BURNOUT, POSTAPOGEE, LANDING};

states launchState = PRELAUNCH;

Controller * control = new Controller();

File dataFile;

const int chipSelect = 53;
const float accLimitFreeFall = .1;
const float accLimitLaunch = 1;
const int currentTime = 0;
const int preLaunchStartTime = 0;

void setup() {
  // put your setup code here, to run once:

  SensorHub::init();
  Serial.begin(57600);
  Serial1.begin(57600);

  servo.setIs3DMotor(true);

  servo.enable();

  controlServo.enable();

  controlServo.setIs3DMotor(true);

  controlServoTwo.enable();

  controlServoTwo.setIs3DMotor(true);

  control->init(P_TUNING, I_TUNING, D_TUNING);

  control->setSetpoint(0);

  control->setCurrentValue(0);

  control->applySetpointLimits(20.0f, -20.0f);

  servo.setPercentLimits(-20, 20);


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

    dataFile.print("time (ms), Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz\n");
    dataFile.flush();
    dataFile.close();
  }

  
}

void loop() {

  SensorHub::update();

  //Example for getting Gyro.
  SensorHub::Gyro().x;

  //Example for getting Accel.
  SensorHub::Accel().x;

  float power = sin((float)millis()/1000.0f) * 45.0f;

  servo.setPower(power);

  Serial.println(power);

  servo.update();
  
  switch(launchState){
    case PRELAUNCH:
      if( SensorHub::Accel().z > accLimitLaunch ){
        launchState = PREBURNOUT;
        preLaunchStartTime = millis;
      }
      break;
    case PREBURNOUT:
      currentTime = millis;
      if( currentTime - preLaunchStartTime >= preBurnoutLength ){
        launchState = BURNOUT;
      }
    case BURNOUT:
      if( SensorHub::Accel.z > accLimitFreeFall ){
        launchState = POSTAPOGEE;
      }
    case POSTAPOGEE:
      if( impactDetected == true ){
        launchState = LANDING;
      }
    case default:
      launchState = LANDING; /* Was not sure what to do for the default but this 
                                this made sense to me */
  }
  

  
}


