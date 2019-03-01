/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef ROTATION_H
#define ROTATION_H

#include <cmath>
#include "vector3.h"

template<typename Number>
class RotationMatrix;

template<typename Number>
struct __attribute__((packed)) RotationAngles final {
    RotationAngles();
    RotationAngles(Number pitch, Number roll, Number yaw);
    RotationAngles(const RotationMatrix<Number>& matrix);

    Number pitch;
    Number roll;
    Number yaw;
};

static_assert(sizeof(RotationAngles<float>) == 12, "RotationAngles data is not packed");

template<typename Number>
class RotationMatrix final {
public:
    RotationMatrix();
    RotationMatrix(const RotationAngles<Number>& angles);
    RotationMatrix(Number pitch, Number roll, Number yaw);

    const Number& operator()(size_t i, size_t j) const;
    Number& operator()(size_t i, size_t j);
    Vector3<Number> operator*(const Vector3<Number>& vector) const;

private:
    Number fields_[3][3];
};

#include "rotation_impl.h"

#endif  // ROTATION_H
