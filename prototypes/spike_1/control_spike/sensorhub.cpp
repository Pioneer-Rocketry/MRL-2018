/**
*	sensorhub.c
*	Implementation of logic that takes in sensor readings and calculates orientations
*	Author: Grant Oberhauser
*/

#include "sensorhub.h"
//#include <SparkFunLSM9DS1.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

#include <Wire.h>

//#include <i2c_t3.h>   

#include <MPU6050.h>


//Including Wire.h for the 
#include "Wire.h"

#define MAX_16_BITS 0xFFFF
#define MAX_GYRO    250
#define MAX_ACCEL   2
#define MAX_MAG     1200

#define BARO_READ_RATE 1000 // Every 1000 milliseconds we can read from the barometer.

//#define INVERSE_ACCEL_Y
//#define INVERSE_ACCEL_Z
//#define INVERSE_Z
#define INVERSE_X
#define INVERSE_Y
//#define FLIP_X_Y

//Always have the orientation of the device level
//#define ALL_LEVEL 
// begin() returns a 16-bit value which includes both the gyro 
// and accelerometers WHO_AM_I response. You can check this to
// make sure communication was successful.
//LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

point SensorHub::accel;
point SensorHub::mag;
point SensorHub::gyro;

float SensorHub::altitude;

quaternion SensorHub::orient;

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
  

uint16_t SensorHub::lastUpdate = 0;    
uint16_t SensorHub::now = 0;           

float SensorHub::deltat = 0.0;

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

unsigned long last_baro_read = 0;

//Initialize sensors and sensor readings.
void SensorHub::init()
{

  orient.a = 1.0f;
  orient.b = 0.0f;
  orient.c = 0.0f;
  orient.d = 0.0f;
  

  Wire.begin();

  //Wire1.begin();

  Serial.begin(38400);

  mpu.initialize();
  
}

//Update sensors and filterss
void SensorHub::update()
{

  now = millis();
  
  deltat = (now - lastUpdate) / 1000.0f;

  lastUpdate = now;
  
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  gyro.x = (float)gx/(float)MAX_16_BITS * MAX_GYRO * 2;
  gyro.y = (float)gy/(float)MAX_16_BITS * MAX_GYRO * 2;
  gyro.z = (float)gz/(float)MAX_16_BITS * MAX_GYRO * 2;

  accel.x = (float)ax/(float)MAX_16_BITS * MAX_ACCEL * 2;
  accel.y = (float)ay/(float)MAX_16_BITS * MAX_ACCEL * 2;
  accel.z = (float)az/(float)MAX_16_BITS * MAX_ACCEL * 2;

  mag.x = (float)mx/(float)MAX_16_BITS * MAX_MAG * 2;
  mag.y = (float)my/(float)MAX_16_BITS * MAX_MAG * 2;
  mag.z = (float)mz/(float)MAX_16_BITS * MAX_MAG * 2;

  //The madgwick function works in radians. So the gyro readings need to be converted quick. 
  point radGyro;
  radGyro.x = gyro.x*PI/180.0f;
  radGyro.y = gyro.y*PI/180.0f;
  radGyro.z = gyro.z*PI/180.0f;

  //orient = KalmanFilter::MadgwickQuaternionUpdate(accel, radGyro, mag, orient, deltat);
  
  if(millis() - last_baro_read > BARO_READ_RATE)
  {
    altitude = baro.getAltitude();
    last_baro_read = millis();
  }
}

quaternion SensorHub::filteredOrientation()
{
	return orient;
}

/*point SensorHub::localToGlobal(point p)
{
	quaternion pointQ;

	pointQ.a = 0;
	pointQ.b = p.x;
	pointQ.c = p.y;
	pointQ.d = p.z;

	quaternion tempPoint = QuatOps::hProd(QuatOps::hProd(orient, pointQ),QuatOps::conj(orient));

	point finalPoint;

	finalPoint.x = tempPoint.b;
	finalPoint.y = tempPoint.c;
	finalPoint.z = tempPoint.d;

	return finalPoint;


}

point SensorHub::globalToLocal(point p)
{
	quaternion pointQ;

	pointQ.a = 0;
	pointQ.b = p.x;
	pointQ.c = p.y;
	pointQ.d = p.z;

	quaternion tempPoint = QuatOps::hProd(QuatOps::hProd(QuatOps::conj(orient), pointQ),orient);

	point finalPoint;

	finalPoint.x = tempPoint.b;
	finalPoint.y = tempPoint.c;
	finalPoint.z = tempPoint.d; 

	return finalPoint;

}*/

point SensorHub::getAccel()
{
  return accel;
}

point SensorHub::getMag()
{
  return mag;
}

point SensorHub::getGyro()
{
  return gyro;
}

float SensorHub::getAltitude()
{
  return altitude;
}


