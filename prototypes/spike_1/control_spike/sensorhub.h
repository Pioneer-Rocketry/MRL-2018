/**
*	SensorHub.h
*	Header file that takes in sensor readings and calculates orientations
*	Author: Grant Oberhauser
*/
#ifndef SENSORHUB_H
#define SENSORHUB_H

#include "dimensionals.h"
//#include "quatops.h"
//#include "kalmanfilter.h"
//#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
//#include <mpu9250.h>

#include "Adafruit_MPL3115A2.h"

// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

//This interupt pins are used to obtain data only when it's ready.
//This helps with better filtering.
//They are not used currently.
#define INT1XM = 2; // INT1XM tells us when accel data is ready
#define INT2XM = 3; // INT2XM tells us when mag data is ready
#define DRDYG  = 4; // DRDYG  tells us when gyro data is ready

class SensorHub
{

public: 

	static void init();

	static void update();

/*	static point localToGlobal(point p);

	static point globalToLocal(point p);*/

	static point getAccel();

	static point getMag();

	static point getGyro();

  static float getAltitude();

	static quaternion filteredOrientation();

  static float getDeltaT(){return deltat;}

protected:


	static point accel;
	static point mag;
	static point gyro;

	static quaternion orient;

  static float altitude;

	static uint16_t lastUpdate;    
	static uint16_t now;           

	static float deltat;


};

#endif
