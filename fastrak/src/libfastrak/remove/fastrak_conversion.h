#include <math.h>

//Macros to convert numbers
#ifndef PI
#define PI            3.141592653589793
#endif

#ifndef PI2
#define PI2           6.283185307179586
#endif

#ifndef PIH
#define PIH           1.57079632679
#endif

#ifndef PIQ
#define PIQ           0.785398163397
#endif

#ifndef PISQ
#define PISQ          9.869604401089358
#endif

#ifndef SQRT2
#define SQRT2         1.41421356237
#endif

#ifndef SQRT2_INV
#define SQRT2_INV     0.707106781187
#endif

#ifndef RAD
#define RAD(x)        ((x)*0.017453292519943)
#endif

#ifndef DEG
#define DEG(x)        ((x)*57.295779513082323)
#endif

#ifndef MIN
#define MIN(x,y)      (((x) > (y)) ? (y) : (x))
#endif

#ifndef MAX
#define MAX(x,y)      (((x) < (y)) ? (y) : (x))
#endif

#ifndef ABS
#define ABS(x)        (((x) > 0.0) ? (x) : -(x))
#endif

#define ROUND(x)      ((int)rint((double)(x)))
#define CEIL(x)       ((int)ceil((double)(x)))
#define FLOOR(x)      ((int)floor((double)(x)))


//Misc
#define YES   1
#define NO    0
#define ON    1
#define OFF   0

//=============================================================================
inline double NORM_RAD(double angle)
{
	while(angle > M_PI)
	  angle -= 2*M_PI;
	while(angle <= -M_PI)
	  angle += 2*M_PI;

	return angle;
}
//=============================================================================
inline double NORM_DEG(double angle)
{
	while(angle > 180.0)
	  angle -= 360.0;
	while(angle <= -180.0)
	  angle += 360.0;

	return angle;
}
//=============================================================================
