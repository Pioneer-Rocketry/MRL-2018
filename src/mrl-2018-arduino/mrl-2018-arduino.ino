#include <i2c_t3.h>


#include <Servo.h>

#include <SD.h>

//#include <SparkFunLSM9DS1.h>

#include "controller.h"
#include "kalmanfilter.h"
#include "quatops.h"
#include "sensorhub.h"

#define SERVO_ONE_PIN 1         
#define SERVO_TWO_PIN 2
#define SERVO_THREE_PIN 3

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
const int preBurnoutLength = 2500;

enum states{PRELAUNCH, PREBURNOUT, BURNOUT, POSTAPOGEE, LANDING};

states launchState = PRELAUNCH;

unsigned long preLaunchStartTime = 0;

float power;

String stateString = "";

String dataString;

bool impactDetected = false;



Servo servo1;
Servo servo2;
Servo servo3;

bool controlled = false;

long micros_last;

File dataFile;


void get_write_data(){
    dataString = "";
  
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
    dataString += " , ";

    dataString += SensorHub::getSpeed();
    dataString += " , ";
  
    dataString += power;
    dataString += " , ";
  
    dataString += stateString;
  
    dataString += "\n";

    Serial.print(dataString);
  
    if (dataFile) {
      dataFile.print(dataString);
      dataFile.flush();
      Serial1.print(dataString);
    }
}// end function get_write_data



void setup() {

  //Attach to servos
  servo1.attach(SERVO_ONE_PIN);
  servo2.attach(SERVO_TWO_PIN);
  servo3.attach(SERVO_THREE_PIN);

  //Zero out servos. Doing this first incase we reboot mid-flight.
 // servo1.writeMicroseconds((SERVO_HIGH - SERVO_LOW)/2 + SERVO_LOW);
  //servo2.writeMicroseconds((SERVO_HIGH - SERVO_LOW)/2 + SERVO_LOW);
 // servo3.writeMicroseconds((SERVO_HIGH - SERVO_LOW)/2 + SERVO_LOW);
  servo1.writeMicroseconds( MID_SERVO + SERVO_OFFSET_1);
  servo2.writeMicroseconds( MID_SERVO + SERVO_OFFSET_2);
  servo3.writeMicroseconds( MID_SERVO + SERVO_OFFSET_3);

  Serial.begin(57600);

  //Starting XBee serial.
  Serial1.begin(57600);


  while(!Serial) ;

  //Initialize all sensors
  //SensorHub::init();

  Serial.println("Pioneer Rocketry XBee Initialized");

  Serial1.println("Pioneer Rocketry XBee Initialized");

  if(!SD.begin(sd_chipSelect))
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

  do
  {

    sprintf(name_string, "MRL-Data-%d.csv", count);

    Serial.println(name_string);

    count++;

  }while(SD.exists(name_string));

  dataFile = SD.open(name_string, FILE_WRITE);

  SensorHub::init();

}

long last_read = 0;

long last_write = 0;

float RADS_TO_DEGREES;
long currentTime;
float currentState = 1.0;
int oneDegree = 5; //millsec per degree
int stepLength = 500; // millsecs

void loop() {

  switch(launchState){
    case PRELAUNCH:
      stateString = "PRELAUNCH";
      if( SensorHub::getAccel().z > accLimitLaunch ){
        launchState = PREBURNOUT;
        preLaunchStartTime = millis();
      }
      break;
    case PREBURNOUT:
      stateString = "PREBURNOUT";
      if( millis() - preLaunchStartTime >= preBurnoutLength ){
        launchState = BURNOUT;
      }
      break;
    case BURNOUT:
      stateString = "BURNOUT";
      //power = sin(((float)millis()/1000.0f) * 2 * PI) * MAGNITUDE + MID_SERVO;
  
      //servo1.writeMicroseconds( MID_SERVO + SERVO_OFFSET_1);
      //servo2.writeMicroseconds( MID_SERVO + SERVO_OFFSET_2);
      //servo3.writeMicroseconds( MID_SERVO + SERVO_OFFSET_3);
  
      if( SensorHub::getAccel().z > accLimitFreeFall ){
        launchState = POSTAPOGEE;
      }
      break;
    case POSTAPOGEE:
      stateString = "POSTAPOGEE";
      if( impactDetected == true ){
        launchState = LANDING;
      }
      break;
    default:
      stateString = "LANDING";
      launchState = LANDING; 
    
  }
  
  if(launchState != BURNOUT){
    power = MID_SERVO;
    servo1.writeMicroseconds(MID_SERVO);
    servo2.writeMicroseconds(MID_SERVO);
    servo3.writeMicroseconds(MID_SERVO);  
  }

  
// write data at constant time intervals 
  if(millis() - last_write >= WRITE_RATE){
    SensorHub::update();
    get_write_data();
    last_write = millis();

  }

// change fins at set times ( step response )
  currentTime = mills();
  if( currentTime - lastStep >= stepLength && launchState == BURNOUT){
    if(currentState > 0){
      currentState = -currentState;
      servo1.writeMicroseconds( currentState*oneDegree + MID_SERVO + SERVO_OFFSET_1);
      servo1.writeMicroseconds( currentState*oneDegree + MID_SERVO + SERVO_OFFSET_2);
      servo1.writeMicroseconds( currentState*oneDegree + MID_SERVO + SERVO_OFFSET_3);
    }
    lastStep = currentTime;
  }


}// end of main loop







