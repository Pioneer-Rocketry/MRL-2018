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


Controller * control = new Controller();

File dataFile;

const int chipSelect = 53;

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

//  SensorHub::update();



/*  servo.update();
  
//  servo.setPower(sin( millis()/(float)500 ) * 20 );

  

  control->setP(P_TUNING);

  control->setI(I_TUNING);

  control->setD(D_TUNING);

  control->setSetpoint(0);

  control->setCurrentValue(SensorHub::getGyro().z);

  control->update();

  servo.setAcceleration(-control->getOutput());

  //servo.setPower(-control->getOutput());

//  Serial1.printf("%f, %f, %f, %f", control->getPropMult(), control->getDirMult(), control->getIntiMult(), control->getOutput());*/

  // String dataString = "";

//  float controlValue = SensorHub::g
  //Serial.println(controlValue);
//etGyro().z + 3;

  

//  servo.setPower(15);
  float power = sin((float)millis()/1000.0f) * 45.0f;


  servo.setPower(power);

  Serial.println(power);

  servo.update();

/*  delay(500);

  servo.setPower(-15);

  servo.update();

  delay(500);*/


//  controlServoTwo.setPower(-controlValue);

//  controlServo.update();

//  controlServoTwo.update();

/*  dataString += millis();

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

  dataString += "\n";

  Serial.print(dataString);*/

/*  dataFile = SD.open("datalog1.csv", FILE_WRITE);
  
  if (dataFile) {
    //Serial.println("Written");
    dataFile.print(dataString);
    dataFile.flush();
    dataFile.close();
  }*/

/*  if(millis() / 2000 <= 1000)
  {
    servo.setPower(20);
  }
  else
  {

    servo.setPower(0);
    
  }*/
  
}


