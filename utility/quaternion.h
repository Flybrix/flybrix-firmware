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

    // Rotation around X in the Z1-Y2-X3 frame
    Number pitch() const;
    // Rotation around Y in the Z1-Y2-X3 frame
    Number roll() const;
    // Rotation around Z in the Z1-Y2-X3 frame
    Number yaw() const;

    Number w;
    Number x;
    Number y;
    Number z;
};

template <typename Number>
Quaternion<Number> operator*(const Quaternion<Number>& p, const Quaternion<Number>& q) {
    return {
        p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,  // W
        p.x * q.w + p.w * q.x - p.z * q.y + p.y * q.z,  // X
        p.y * q.w + p.z * q.x + p.w * q.y - p.x * q.z,  // Y
        p.z * q.w - p.y * q.x + p.x * q.y + p.w * q.z   // Z
    };
}

// Axis directions:
//
// X - Right
// Y - Forward
// Z - Up
//
// Rotation directions:
//
// Pitch - Nose up
// Roll - Left wing up
// Yaw - Turn left

template <typename Number>
Number Quaternion<Number>::pitch() const {
    return std::atan2(w * x + y * z, 0.5 - x * x - y * y);
}

template <typename Number>
Number Quaternion<Number>::roll() const {
    return std::asin(2 * (w * y - x * z));
}

template <typename Number>
Number Quaternion<Number>::yaw() const {
    return std::atan2(w * z + x * y, 0.5 - y * y - z * z);
}

#endif  // QUATERNION_H
