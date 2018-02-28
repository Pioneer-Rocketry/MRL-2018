/**
*	AbstractServo.h
*	Abstracts servo concepts to make controlling them more intuitive. 
*	Author: Grant Oberhauser
*/

#include <Servo.h>
#include "dimensionals.h"

#ifndef ABSTRACTSERVO_H
#define ABSTRACTSERVO_H

class AbstractServo
{

public:

	enum rotDirection { CW = 1, CCW = -1};

  AbstractServo();
  
	/*Initialize the servo, giving maximum and minimum operational values, and more simple external values. Along with the pin the servo is on*/
	AbstractServo(int low, int high, int pin, float rangeMin, float rangeMax);

  void update();

	/*Allow the servo to take commands*/
	void enable();

	/*Set to 0 and disable the servo*/
	void disable();

	/*Set the power using a float between the min and max values.*/
	void setPower(float p);

  void setAcceleration(float a);

	/*Set the location relative to the craft*/
	void setLocation(point p);

	point getLocation();

  void setPercentLimits(float min, float max);

	void setRotationDirection(rotDirection d){direction = d;}

  void setIs3DMotor(bool is3D);

	AbstractServo::rotDirection getRotationDirection(){return direction;}

private:

	int lowMicroseconds, highMicroseconds;

	float lowRange, highRange;

  float maxPercent, minPercent;

	int motorPin;

  float accelRate;

  bool isAccelerating;

	bool enabled;

	bool arming;

	int armTime;

	float currentPower;

	Servo servoObj;

	bool started;

	point location;

	long lastTime;

	rotDirection direction;

  int oldPower = 0;

  bool newVal = true;

  bool is3DMotor = false;

  bool rangeLimitSet = false;


};

#endif
