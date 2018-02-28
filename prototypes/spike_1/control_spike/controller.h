/**
*	controller.h
*	The definition of a generic PID controller class
*	Author: Grant Oberhauser
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller
{
	
public:

	Controller();

	/*Basic setup*/
	void init(float PParam, float IParam, float DParam);

	/*Setup that caps the setpoints to avoid runaway errors due to unrealistic requests.*/
	void init(float PParam, float IParam, float DParam, float maxSetpoint, float minSetpoint);

	/*Run a pass through the control loop*/
	void update();

	/*Set a new setpoint to control to.*/
	void setSetpoint(float p);

	/*Set the current value of the control loop.*/
	void setCurrentValue(float v){curReading = v;}

	/*Sets the maximum and minimum setpoints available.*/
	void applySetpointLimits(float max, float min);

	/*Lifts any limits on setpoints that may exist*/
	void disableSetpointLimits(){setpointLimitsSet = false;}

	/*Sets the proportional tuning parameter*/
	void setP(float prop);

	/*Sets the integral tuning parameter*/
	void setI(float inti);
	
	/*Sets the derivative tuning parameter*/
	void setD(float dir);

	/*Preload a value into the actual proportional error value*/
	void preloadP(float prop){curProp = prop;}

	/*Preload a value into the actual integral value*/
	void preloadI(float inti){curInti = inti;}

	/*Preload a value into the actual derivative value*/
	void preloadD(float dir){curDir = dir;}

	/*Get the output of the control loop*/
  float getOutput();

  /*Get the current prop. (No multiplication)*/
  float getProp(){return curProp;}

  /*Get the current integral (No multiplication)*/
  float getInti(){return curInti;}

  /*Get the current derivative (No multiplication)*/
  float getDir(){return curDir;}

  /*Get the current prop. control input (With multiplication)*/
  float getPropMult(){return curProp * Pval;}

  /*Get the current integral (With multiplication)*/
  float getIntiMult(){return curInti * Ival;}

  /*Get the current derivative (With multiplication)*/
  float getDirMult(){return curDir * Dval;}
  
private:

	float Pval;
	float Ival;
	float Dval;

  float anotherVal;

	float curProp, curInti, curDir;

	float lastError;

	float maxSet;

	float minSet;

	float curSetpoint;

	float curReading;

	bool setpointLimitsSet = false;

};

#endif
