/**
*	kalman.h
*	Defines the kalman filter operations
*	Author: Grant Oberhauser
*/

#include "dimensionals.h"
#include <math.h>

#ifndef KALMAN_H
#define KALMAN_H

#define PI 3.141592653589793238

#define GyroMeasError PI * (90.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

#define sampleFreqDef 20.0f // sample frequency in Hz

class KalmanFilter
{
	
public:

	/*	Slightly modified version of the madgwick quaternion update function
	  	that allows the orientation to be input via the quaternion struct
	*/
	static quaternion MadgwickQuaternionUpdate(point a, point g, point m,
   quaternion orientation, float deltat);
   
   static quaternion updateIMU(float gx, float gy, float gz, float ax, float ay, float az,  quaternion orientation, float deltat);

        static float invSqrt(float x);
      
private:
    static float invSampleFreq;
};

#endif
