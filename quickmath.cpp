#include "quickmath.h"

#include <cmath>

#ifndef SE_NON_IEEE_STANDARD_FLOATS

/*
 * Fast inverse square-root
 * See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */

float quick::invSqrt(float x) {
    float halfx = 0.5f * (float)x;
    float y = (float)x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return fabs(y);
}

#else

float quick::invSqrt(float x) {
    return 1.0f / sqrt(x);
}

#endif

#define PI__2     1.5707963268
#define _4__PI    1.2732395447
#define _4__PIPI  0.4052847346

float quick::fast_sine(float x) {
    //always wrap input angle to -PI..PI
    while (x < -3.14159265) { x += 6.28318531; }
    while (x > 3.14159265) { x -= 6.28318531; }
    
    float sine;
    if (x < 0)
        sine = x * (_4__PI + _4__PIPI * x);
    else
        sine = x * (_4__PI - _4__PIPI * x);

    if (sine < 0)
        sine = sine*(-0.225 * (sine + 1) + 1);
    else
        sine = sine*( 0.225 * (sine - 1) + 1);
    
    return sine;
}

float quick::fast_cosine(float x) {
    return fast_sine( x + PI__2 );
}

