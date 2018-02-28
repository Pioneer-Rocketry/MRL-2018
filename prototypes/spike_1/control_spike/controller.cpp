/**
*	controller.c
*	Implementation of a generic controller
*	Author: Grant Oberhauser
*/

#include "controller.h"


#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

Controller::Controller()
{
	Pval = 0; 
	Ival = 0; 
	Dval = 0;
  curProp = 0;
  lastError = 0;


}

/*Basic setup*/
void Controller::init(float PParam, float IParam, float DParam)
{

	Pval = anotherVal = PParam;
	Dval = DParam;
	Ival = IParam;

  setpointLimitsSet = false;

}

/*Setup that caps the setpoints to avoid runaway errors due to unrealistic requests.*/
void Controller::init(float PParam, float IParam, float DParam, float maxSetpoint, float minSetpoint)
{

	Pval = PParam;
	Dval = DParam;
	Ival = IParam;

	maxSet = maxSetpoint;
	minSet = minSetpoint;

  setpointLimitsSet = true;
}

/*Run a pass through the control loop*/
void Controller::update()
{

	lastError = curProp;

	curProp = curSetpoint - curReading;

	curDir = curProp - lastError;

	curInti += curProp;

	if(setpointLimitsSet && curInti > maxSet)
	{
		curInti = maxSet;
	}
	else if(setpointLimitsSet && curInti < minSet)
	{
		curInti = minSet;
	}

}

/*Set a new setpoint to control to.*/
void Controller::setSetpoint(float p)
{

	if(setpointLimitsSet && p < minSet)
		p = minSet;
	else if(setpointLimitsSet && p > maxSet)
		p = maxSet;

	Controller::curSetpoint = p;

}

/*Sets the maximum and minimum setpoints available.*/
void Controller::applySetpointLimits(float max, float min)
{

	maxSet = max;

	minSet = min;

	setpointLimitsSet = true;

}

/*Sets the proportional tuning parameter*/
void Controller::setP(float prop)
{
  Pval = prop;
}

/*Sets the integral tuning parameter*/
void Controller::setI(float inti)
{
  Ival = inti;
}
  
/*Sets the derivative tuning parameter*/
void Controller::setD(float dir)
{
  Dval = dir;
}



/*Get the output of the control loop*/
float Controller::getOutput()
{

  float rawOut = curProp * Pval + curInti * Ival + curDir * Dval;

  Serial1.println(rawOut);

  if(rawOut > maxSet && setpointLimitsSet)
    rawOut = maxSet;
	else if(rawOut < minSet && setpointLimitsSet)
    rawOut = minSet;
    
	return rawOut;

}
