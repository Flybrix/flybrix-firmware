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
