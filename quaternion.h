/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>

#include "rotation.h"
#include "vector3.h"

template<typename Number>
struct Quaternion {
    Quaternion();

    Quaternion(Number w, Number x, Number y, Number z);

    // Rotation around X in the Z1-Y2-X3 frame
    Number pitch() const;
    // Rotation around Y in the Z1-Y2-X3 frame
    Number roll() const;
    // Rotation around Z in the Z1-Y2-X3 frame
    Number yaw() const;

    bool isZero() const;
    Number lengthSq() const;
    Quaternion<Number> conj() const;
    Quaternion<Number>& operator*=(Number v);
    Quaternion<Number>& operator+=(const Quaternion<Number>& q);
    Quaternion<Number>& operator-=(const Quaternion<Number>& q);
    RotationMatrix<Number> toRotation() const;

    Number w;
    Number x;
    Number y;
    Number z;
};

template<typename Number>
Quaternion<Number> operator*(const Quaternion<Number>& p, const Quaternion<Number>& q);
template<typename Number>
Quaternion<Number> operator*(const Quaternion<Number>& p, Number v);
template<typename Number>
Quaternion<Number> operator+(const Quaternion<Number>& p, const Quaternion<Number>& q);
template<typename Number>
Quaternion<Number> operator-(const Quaternion<Number>& p, const Quaternion<Number>& q);
template<typename Number>
Quaternion<Number> operator*(const Quaternion<Number>& p, const Vector3<Number>& q);

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

#include "quaternion_impl.h"

#endif  // QUATERNION_H
