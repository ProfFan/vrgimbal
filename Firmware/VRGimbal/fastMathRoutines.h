// Org ATAN2 ~200us, fastAtan2 ~128us, ultraFastAtan2 ~92us

#ifndef _FASTMATHROUTINES_H_
#define _FASTMATHROUTINES_H_

#include "wirish.h"

// Fast arctan2
float ultraFastAtan2(float y, float x);

float fastAtan2(float y, float x); // in deg

//inline
int16_t _atan2(float y, float x);

int8_t sgn(int val);


inline int32_t constrain_int32(int32_t x , int32_t l, int32_t h) {
  if (x <= l) {
    return l;
  } else if (x >= h) {
    return h;
  } else {
    return x;
  }
}

//***************************************************************
// Efficient approximations for the arctangent functions,
// Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
//***************************************************************
inline float Rajan_FastArcTan(float x);

// atan2 for all quadrants by A. Hahn
inline float Rajan_FastArcTan2(float y, float x);

//// atan2 returnig degrees * 100
//int16_t Rajan_FastArcTan2_deg100(float y, float x);
// atan2 returnig degrees * 1000
int32_t Rajan_FastArcTan2_deg1000(float y, float x);

/************************/
/* LP Filter            */
/************************/
inline void utilLP_float(float * q, float i, float coeff) {
  *q += (i-*q)*coeff;
}

#endif // _FASTMATHROUTINES_H_
