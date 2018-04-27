/**
*	dimensionals.h
*	This header file defines data structures to organize three dimensional concepts
*	Author: Grant Oberhauser
*/

#ifndef DIMENSIONALS_H
#define DIMENSIONALS_H

struct quaternion
{
  float a;
  float b;
  float c;
  float d;
};

struct point
{
  float x;
  float y;
  float z;
};

#endif