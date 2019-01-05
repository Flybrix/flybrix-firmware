/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

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
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return fabs(y);
}

#else

float quick::invSqrt(float x) {
    return 1.0f / sqrt(x);
}

#endif

namespace {
constexpr float PI = 4 * std::atan(1.0);
constexpr float TWO_PI = 2 * PI;
constexpr float PI_HALF = PI / 2;
constexpr float FOUR_OVER_PI = 4 / PI;
constexpr float FOUR_OVER_PI_SQUARED = 4 / PI / PI;
}  // namespace

float quick::sin(float x) {
    // always wrap input angle to -PI..PI
    while (x < -PI) {
        x += TWO_PI;
    }
    while (x > PI) {
        x -= TWO_PI;
    }

    float helper = x * (FOUR_OVER_PI - FOUR_OVER_PI_SQUARED * std::fabs(x));
    return helper * (0.225 * (std::fabs(helper) - 1) + 1);
}

float quick::cos(float x) {
    return quick::sin(x + PI_HALF);
}
