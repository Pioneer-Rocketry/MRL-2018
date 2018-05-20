#include <i2c_t3.h>
#include <Adafruit_BNO055.h>

#include <Servo.h>

#include <SD.h>

//#include <SparkFunLSM9DS1.h>

#include "controller.h"
#include "kalmanfilter.h"
#include "quatops.h"
#include "sensorhub.h"

#include "configManager.h"

#include "nonLinearController.h"

#define SERVO_ONE_PIN 2
#define SERVO_TWO_PIN 4
#define SERVO_THREE_PIN 3

#define SERVO_HIGH 2000
#define SERVO_LOW  1000
#define MID_SERVO  1500

#define SERVO_OFFSET_1 0
#define SERVO_OFFSET_2 0
#define SERVO_OFFSET_3 0

#define MAGNITUDE 250.0f

#define MULTIS  50

#define WRITE_RATE 10 //In milliseconds.

// 1/frequency. Run control every x milliseconds.
#define CONTROL_DELAY 10

#define LED_RIGHT 21
#define LED_LEFT 20

#define MAX_ANGLE 10
#define MAX_ANGLE_MICROS 111 //(1000 / 90) * 10

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

bool have_control_lock = false;

int current_control_num = 0;

long current_elapsed_control_time = 0;

long current_control_time;

float current_control_point;

float control_tolerance = 5;

bool have_control_point = false;

double control_efforts[3];

//Adafruit_BNO055 bno = Adafruit_BNO055(); //orientation sensor

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

  //Serial.print(dataString);

  Serial1.print(dataString + '\r');

  if (dataFile) {
    dataFile.print(dataString);
    dataFile.flush();
  }
}// end function get_write_data



void setup() {

  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);

  pinMode(LED_LEFT, OUTPUT);

  digitalWrite(LED_LEFT, LOW);

  pinMode(LED_RIGHT, OUTPUT);

  digitalWrite(LED_RIGHT, LOW);

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

  File configFile = SD.open("config.dat", FILE_READ);

  ConfigManager::loadConfig(configFile);

  //Time to get the next one!
  
  current_control_num = 0;

  if(ConfigManager::getConfigLength() > current_control_num)
  {
    current_control_time =  ConfigManager::getControlTime(current_control_num);
    current_control_point = ConfigManager::getControlPoint(current_control_num);
    have_control_lock = false;
    current_elapsed_control_time = 0;
  }

//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
//  bno.setExtCrystalUse(true);
}// end setup



long last_read = 0;

long last_write = 0;

float RADS_TO_DEGREES;
long currentTime;
int currentState = 1;
int oneDegree = 50; //millisec per degree
int stepLength = 500; // millisecs
long lastStep = 0;

long burnout_time = 0;

void loop() {
  
  // control execution code

  switch (launchState) {
    case PRELAUNCH:
      stateString = "PRELAUNCH";

      if ( SensorHub::getAccel().z > accLimitLaunch ) {
        launchState = PREBURNOUT;
        preLaunchStartTime = millis();
      }
      break;
    case PREBURNOUT:
    
      stateString = "PREBURNOUT";
      if ( millis() - preLaunchStartTime >= ConfigManager::getBurntime() ) {
        launchState = BURNOUT;
      }
    break;
    case BURNOUT:
      stateString = "BURNOUT";

      point cur_eulers = SensorHub::getEuler();

      if ( cur_eulers.x > 45 || cur_eulers.y > 45 ) {
        launchState = POSTAPOGEE;
      }

      burnout_time = millis();

      break;
    case POSTAPOGEE:
    
      stateString = "POSTAPOGEE";
      if ( impactDetected == true ) {
        launchState = LANDING;
      }
    break;
    default:
      stateString = "LANDING";
      launchState = LANDING;

  }

/*  if (launchState != BURNOUT) {
    power = MID_SERVO;
    servo1.writeMicroseconds(MID_SERVO);
    servo2.writeMicroseconds(MID_SERVO);
    servo3.writeMicroseconds(MID_SERVO);
  }*/

  // write data at constant time intervals
  if (millis() - last_write >= WRITE_RATE){// && (launchState == BURNOUT || launchState == POSTAPOGEE) && millis() - burnout_time > ConfigManager::getWaitAfterBurn())  {

    point eulers = SensorHub::getEuler();

    point gyro = SensorHub::getGyro();

    //We need to take into account the orientation of the craft relative to the rocket first. Eulers already did this.

    float temp = gyro.z;
    gyro.z = gyro.y;
    gyro.y = -temp;
    gyro.x = - gyro.x;

    Serial.printf("Diff: %f", abs(eulers.z - current_control_point));

    if(!ConfigManager::isFlipAndBurn())
    {

      if(abs(eulers.z - current_control_point) <  control_tolerance) 
      {
        have_control_lock = true;
      }

      if(have_control_lock)
      {
        current_elapsed_control_time += millis() - last_write;
      }

      if(current_elapsed_control_time > current_control_time)
      {
        //Time to get the next one!
        
        current_control_num++;

        if(ConfigManager::getConfigLength() > current_control_num)
        {
          current_control_time =  ConfigManager::getControlTime(current_control_num);
          current_control_point = ConfigManager::getControlPoint(current_control_num);
          have_control_lock = false;
          current_elapsed_control_time = 0;
        }
      }
    }
    else{

      if(!have_control_point)
      {
        //This is the first time through.
        current_control_point = eulers.z + 180;
        have_control_point = true;
      }

    }

    Serial.printf("config length: %d", ConfigManager::getConfigLength());

    nonLinearController::controlEffert(eulers.x , eulers.y, eulers.z, 
                  gyro.x, gyro.y, gyro.z,
                  current_control_point, (1000.0f/(float)WRITE_RATE), SensorHub::getSpeed(), (double *)&control_efforts);

    Serial.printf("euler: %f, %f, %f -- gyro: %f, %f, %f", 
                  SensorHub::getEuler().x, SensorHub::getEuler().y, SensorHub::getEuler().z,
                  gyro.x, gyro.y, gyro.z);

    Serial.printf("Control: %f, %f, %f\n", control_efforts[0], control_efforts[1], control_efforts[2]);

    Serial.printf("Current Point: %f", current_control_point);

    //Put the fuckings shit in the shit.
    float final_out_0 = min(max(control_efforts[0], -MAX_ANGLE), MAX_ANGLE);
    float final_out_1 = min(max(control_efforts[1], -MAX_ANGLE), MAX_ANGLE);
    float final_out_2 = min(max(control_efforts[2], -MAX_ANGLE), MAX_ANGLE);

    //"Oh hey it won't wind up at all"

    //k

    Serial.printf("servo 1: %f, servo 2: %f, servo 3: %f", 
    (final_out_0 / (float)MAX_ANGLE) * MAX_ANGLE_MICROS + MID_SERVO,
    (final_out_1 / (float)MAX_ANGLE) * MAX_ANGLE_MICROS + MID_SERVO,
    (final_out_2 / (float)MAX_ANGLE) * MAX_ANGLE_MICROS + MID_SERVO);

    servo1.writeMicroseconds((final_out_0 / (float)MAX_ANGLE) * MAX_ANGLE_MICROS + MID_SERVO);
    servo2.writeMicroseconds((final_out_1 / (float)MAX_ANGLE) * MAX_ANGLE_MICROS + MID_SERVO);
    servo3.writeMicroseconds((final_out_2 / (float)MAX_ANGLE) * MAX_ANGLE_MICROS + MID_SERVO);

    if(current_control_point <= eulers.z)
    {
      digitalWrite(LED_LEFT, HIGH);
      digitalWrite(LED_RIGHT, LOW);
    }
    else
    {
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, HIGH);
    }

    get_write_data();
    last_write = millis();
  }

  SensorHub::update();

  // change fins at set times ( step response )



}// end of main loop







