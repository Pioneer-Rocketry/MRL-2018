/*
*	quaternion.cpp
*	Impelmentation of quaternion operations
*	Author: Grant Oberhauser
*/

#include "quatops.h"

//Performs a hamilton product
quaternion QuatOps::hProd(quaternion q1, quaternion q2)
{

  float a1 = q1.a;
  float b1 = q1.b; 
  float c1 = q1.c;
  float d1 = q1.d;

  float a2 = q2.a;
  float b2 = q2.b;
  float c2 = q2.c;
  float d2 = q2.d ;

  quaternion finalQuat;

  finalQuat.a = a1 * a2 - b1 * b2 - c1 * c2 - d1* d2;

  finalQuat.b = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;

  finalQuat.c = a1 * c2 - b1*d2 + c1 * a2 + d1 * b2;

  finalQuat.d = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;

  return finalQuat;
  
}

//Conjugate (inverse) the quaternion
quaternion QuatOps::conj(quaternion q1)
{

  float a = q1.a;//real
  float b = q1.b;//imaginary i
  float c = q1.c;//imaginary j
  float d = q1.d;//imaginary k

  quaternion finalQuat;

  finalQuat.a = a; 

  finalQuat.b = b * -1;

  finalQuat.c = c * -1;

  finalQuat.d = d * -1;

  return finalQuat;


}

//Converts a vector to a quaternion representation
quaternion QuatOps::vectorToQuaternion(point p)
{

  quaternion pointQ;

  pointQ.a = 0;
  pointQ.b = p.x;
  pointQ.c = p.y;
  pointQ.d = p.z;

  return pointQ;

}
