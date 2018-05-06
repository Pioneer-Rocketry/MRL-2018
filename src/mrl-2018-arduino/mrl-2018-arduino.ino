//#include <i2c_t3.h>
#include <Adafruit_BNO055.h>

#include <Servo.h>

#include <SD.h>

//#include <SparkFunLSM9DS1.h>

#include "controller.h"
#include "kalmanfilter.h"
#include "quatops.h"
#include "sensorhub.h"

#define SERVO_ONE_PIN 2
#define SERVO_TWO_PIN 3
#define SERVO_THREE_PIN 4

#define SERVO_HIGH 2000
#define SERVO_LOW  1000
#define MID_SERVO  1500

#define SERVO_OFFSET_1 0
#define SERVO_OFFSET_2 0
#define SERVO_OFFSET_3 0

#define MAGNITUDE 250.0f

#define MULTIS  50

#define WRITE_RATE 5 //In milliseconds.

// 1/frequency. Run control every x milliseconds.
#define CONTROL_DELAY 10

const int sd_chipSelect = BUILTIN_SDCARD;

const float accLimitFreeFall = .1;
const float accLimitLaunch = 2;
const int preBurnoutLength = 2700;

enum states {PRELAUNCH, PREBURNOUT, BURNOUT, POSTAPOGEE, LANDING};

states launchState = PRELAUNCH;

unsigned long preLaunchStartTime = 0;

float power;

String stateString = "";

String dataString;

bool impactDetected = false;

Adafruit_BNO055 bno = Adafruit_BNO055(); //orientation sensor

Servo servo1;
Servo servo2;
Servo servo3;

bool controlled = false;

long micros_last;

File dataFile;


void get_write_data() {
  char temp[10];
  dataString = "";

  dataString += millis();
  dataString += " , ";

  sprintf(temp, "%.4f , %.4f , %.4f , ", SensorHub::getGyro().x,SensorHub::getGyro().y,SensorHub::getGyro().z);
  dataString += temp;


  sprintf(temp, "%.4f , %.4f , %.4f , ", SensorHub::getAccel().x,SensorHub::getAccel().y,SensorHub::getAccel().z);
  dataString += temp;

  sprintf(temp, "%.4f , %.4f , %.4f , ", SensorHub::getMag().x,SensorHub::getMag().y,SensorHub::getMag().z);
  dataString += temp;

  sprintf(temp, "%.4f , %.4f , %.1f , ", SensorHub::getAltitude(),SensorHub::getSpeed(),power);
  dataString += temp;

  dataString += stateString;

  dataString += "\n";

  Serial.print(dataString);

  Serial1.print(dataString + '\r');

  if (dataFile) {
    dataFile.print(dataString);
    dataFile.flush();
  }
}// end function get_write_data



void setup() {

  //Attach to servos
  servo1.attach(SERVO_ONE_PIN);
  servo2.attach(SERVO_TWO_PIN);
  servo3.attach(SERVO_THREE_PIN);

  //Zero out servos. Doing this first incase we reboot mid-flight.
  // servo1.writeMicroseconds((SERVO_HIGH - SERVO_LOW)/2 + SERVO_LOW);
  // servo2.writeMicroseconds((SERVO_HIGH - SERVO_LOW)/2 + SERVO_LOW);
  // servo3.writeMicroseconds((SERVO_HIGH - SERVO_LOW)/2 + SERVO_LOW);
  servo1.writeMicroseconds( MID_SERVO + SERVO_OFFSET_1);
  servo2.writeMicroseconds( MID_SERVO + SERVO_OFFSET_2);
  servo3.writeMicroseconds( MID_SERVO + SERVO_OFFSET_3);

  Serial.begin(57600);

  //Starting XBee serial.
  Serial1.begin(57600);

  //Initialize all sensors
  //SensorHub::init();

  Serial.println("Pioneer Rocketry XBee Initialized");

  Serial1.println("Pioneer Rocketry XBee Initialized");

  if (!SD.begin(sd_chipSelect))
  {
    Serial.println("SD failed!");
  }
  else
  {
    Serial.println("SD success!");
  }

  micros_last = micros();

  int count = 0;

  char name_string[20];

  do {

    sprintf(name_string, "MRLData%d.csv", count);

    Serial.println(name_string);

    count++;

  } while (SD.exists(name_string));



  dataFile = SD.open(name_string, FILE_WRITE);

  dataFile.print("sd start");
  dataFile.flush();

  Serial.println("SD started");

  SensorHub::init();

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
}// end setup



long last_read = 0;

long last_write = 0;

float RADS_TO_DEGREES;
long currentTime;
int currentState = 1;
int oneDegree = 50; //millisec per degree
int stepLength = 500; // millisecs
long lastStep = 0;

void loop() {
  
  // control execution code
  imu::Vector<3> euler;

  switch (launchState) {
    case PRELAUNCH:
      stateString = "PRELAUNCH";

      if ( SensorHub::getAccel().z > accLimitLaunch ) {
        launchState = PREBURNOUT;
        preLaunchStartTime = millis();
      }
      break;
    case PREBURNOUT:
    {
      stateString = "PREBURNOUT";
      if ( millis() - preLaunchStartTime >= preBurnoutLength ) {
        launchState = BURNOUT;
      }
    }break;
    case BURNOUT:
      stateString = "BURNOUT";

      // control execution code
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);


      //end control section

      if ( SensorHub::getAccel().z > accLimitFreeFall ) {
        launchState = POSTAPOGEE;
      }
      break;
    case POSTAPOGEE:
    {
      stateString = "POSTAPOGEE";
      if ( impactDetected == true ) {
        launchState = LANDING;
      }
    }break;
    default:
      stateString = "LANDING";
      launchState = LANDING;

  }

  if (launchState != BURNOUT) {
    power = MID_SERVO;
    servo1.writeMicroseconds(MID_SERVO);
    servo2.writeMicroseconds(MID_SERVO);
    servo3.writeMicroseconds(MID_SERVO);
  }

  // write data at constant time intervals
  if (millis() - last_write >= WRITE_RATE) {
    SensorHub::update();
    get_write_data();
    last_write = millis();

  }

  // change fins at set times ( step response )



}// end of main loop







