/**
*	AbstractServo.c
*	Abstracts servo concepts to make controlling them more intuitive. 
*	Author: Grant Oberhauser
*/

#include "AbstractServo.h"
#include <Servo.h>

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif



/*Initialize the servo, giving maximum and minimum operational values, and more simple external values. Along with the pin the servo is on*/
AbstractServo::AbstractServo(int low, int high, int pin, float rangeMin, float rangeMax)
{

	lowMicroseconds = low;

	highMicroseconds = high;

	motorPin = pin;

	lowRange = rangeMin;

	highRange = rangeMax;

	enabled = false;

  started = false;

  servoObj = Servo();

  arming = false;

  armTime = 0;

  currentPower = 0;

  isAccelerating = false;

  accelRate = 0.0f;

  maxPercent = 0;

  minPercent = 0;

  rangeLimitSet = false;
}

void AbstractServo::setIs3DMotor(bool is3D)
{

  is3DMotor = is3D;
  
}

void AbstractServo::update()
{


  arming = false;
    
  int range = highMicroseconds - lowMicroseconds;

  int totPower = 0;

  noInterrupts();
  if(enabled && !isAccelerating)
  {

    if(is3DMotor)
    {
      totPower = (int)(lowMicroseconds + (float)range/(float)2 + ((float)range/(float)2 * currentPower/(float)highRange));
    }
    else
    {
      totPower = (int)(lowMicroseconds + range * ((float)currentPower/(float)highRange));
      Serial.print("New Power:");
      Serial.println(totPower);
      Serial.print("low Range:"); 
      Serial.println(lowRange);
      Serial.print("high Range:");
      Serial.println(highRange);
      
    }
     
    if(newVal && totPower != oldPower)
    {
      servoObj.writeMicroseconds(totPower);
      oldPower = totPower;
      newVal = false;
    }
    
  }
  else if(enabled && isAccelerating)
  {
    long delta = millis() - lastTime;

    currentPower += accelRate * (delta/1000.0f);

    if(currentPower > maxPercent)
      currentPower = maxPercent;
    else if(currentPower < minPercent)
      currentPower = minPercent;
  
    if(is3DMotor)
    {
      totPower = (int)(lowMicroseconds + (float)range/(float)2 + ((float)range/(float)2 * currentPower/(float)highRange));
    }
    else
    {
      totPower = (int)(lowMicroseconds + range * ((float)currentPower/(float)highRange));
    }

    if(totPower > highMicroseconds)
      totPower = highMicroseconds;
    else if(totPower < lowMicroseconds)
      totPower = lowMicroseconds;

    servoObj.writeMicroseconds(totPower);
    oldPower = totPower;
    newVal = false;
   
  }
  else
  {
   //Do nothing. 
  }
  
  interrupts();
  
  lastTime = millis();
}

void AbstractServo::setPercentLimits(float min, float max)
{

  rangeLimitSet = true;
  maxPercent = max;
  minPercent = min;
  
}


/*Allow the servo to take commands*/
void AbstractServo::enable()
{
 
  servoObj.attach(motorPin);
  servoObj.writeMicroseconds(lowMicroseconds);

  arming = true;

  armTime = millis();

  enabled = true;

}

/*Set to 0 and disable the servo*/
void AbstractServo::disable()
{

  servoObj.writeMicroseconds(0);
  servoObj.detach();
  enabled = false;

}

/*Set the power using a float between the min and max values.*/
void AbstractServo::setPower(float p)
{

  	/*int range = highMicroseconds - lowMicroseconds;
   
    Serial.println((int)(lowMicroseconds + range * ((float)p/(float)highRange)));
    Serial.println(servoObj->attached());
    Serial.println(motorPin);*/

    //lastTime = millis();
	  //servoObj->writeMicroseconds((int)(lowMicroseconds + range * ((float)p/(float)highRange)));
    if(p < lowRange)
      p = lowRange;
    else if(p > highRange)
      p = highRange;
    
    currentPower = p;
    started = true;
    newVal = true;
    //Serial.println(currentPower);
 
  
}

void AbstractServo::setAcceleration(float a)
{

  isAccelerating = true;
  accelRate = a;  
  
}

/*Write the raw milliseconds value to the servo*/
/*void AbstractServo::writeMicroseconds(int m)
{
  started = true;
	servoObj->writeMicroseconds(m);

}*/


void AbstractServo::setLocation(point p)
{


	location = p;

}

point AbstractServo::getLocation()
{

	return location;

}
