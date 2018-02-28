/**
*	quaternion.h
*	Defines the comon operations of quaternions
* 	Author: Grant Oberhauser
*/
#include "dimensionals.h"

#ifndef QUATOPS_H
#define QUATOPS_H

class QuatOps
{
	
public:

	//Performs a hamilton product
	static quaternion hProd(quaternion q1, quaternion q2);

	//Conjugate (inverse) the quaternion
	static quaternion conj(quaternion q1);

	//Converts a vector to a quaternion representation
	static quaternion vectorToQuaternion(point p);

private:


};

#endif
