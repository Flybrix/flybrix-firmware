#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>

#include "vector3.h"

template <typename Number>
struct Quaternion {
    Quaternion() : Quaternion(1, 0, 0, 0) {
    }
    Quaternion(Number w, Number x, Number y, Number z) : w{w}, x{x}, y{y}, z{z} {
    }

    Number pitch() const;
    Number roll() const;
    Number yaw() const;

    Number w;
    Number x;
    Number y;
    Number z;
};

template <typename Number>
Number Quaternion<Number>::pitch() const {
    float r11 = 2 * (y * z + x * w);
    float r12 = x * x + y * y - z * z - w * w;
    return -atan2(r11, r12);
}

template <typename Number>
Number Quaternion<Number>::roll() const {
    float r21 = -2 * (y * w - x * z);
    return asin(r21);
}

template <typename Number>
Number Quaternion<Number>::yaw() const {
    float r31 = 2 * (z * w + x * y);
    float r32 = x * x - y * y - z * z + w * w;
    return -atan2(r31, r32);
}

#endif  // QUATERNION_H
