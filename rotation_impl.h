/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef ROTATION_IMPL_H
#define ROTATION_IMPL_H

#include "rotation.h"

template<typename Number>
RotationAngles<Number>::RotationAngles() : RotationAngles{0, 0, 0} {}

template<typename Number>
RotationAngles<Number>::RotationAngles(Number pitch, Number roll, Number yaw)
        : pitch{pitch}, roll{roll}, yaw{yaw} {}

template<typename Number>
RotationAngles<Number>::RotationAngles(const RotationMatrix<Number>& m)
        : pitch{std::atan2(m(2, 1), m(2, 2))},
          roll{std::atan2(-m(2, 0), std::sqrt(m(2, 1) * m(2, 1) + m(2, 2) * m(2, 2)))},
          yaw{std::atan2(m(1, 0), m(0, 0))} {}

template<typename Number>
RotationMatrix<Number>::RotationMatrix()
        : fields_{{1, 0, 0},
                  {0, 1, 0},
                  {0, 0, 1}} {
}

template<typename Number>
RotationMatrix<Number>::RotationMatrix(const RotationAngles<Number>& angles)
        : RotationMatrix(angles.pitch, angles.roll, angles.yaw) {}

template<typename Number>
RotationMatrix<Number>::RotationMatrix(Number pitch, Number roll, Number yaw) {
    Number cx{std::cos(pitch)};
    Number sx{std::sin(pitch)};
    Number cy{std::cos(roll)};
    Number sy{std::sin(roll)};
    Number cz{std::cos(yaw)};
    Number sz{std::sin(yaw)};

    fields_[0][0] = cz * cy;
    fields_[0][1] = cz * sy * sx - sz * cx;
    fields_[0][2] = cz * sy * cx + sz * sx;

    fields_[1][0] = sz * cy;
    fields_[1][1] = sz * sy * sx + cz * cx;
    fields_[1][2] = sz * sy * cx - cz * sx;

    fields_[2][0] = -sy;
    fields_[2][1] = cy * sx;
    fields_[2][2] = cy * cx;
}

template<typename Number>
const Number& RotationMatrix<Number>::operator()(size_t i, size_t j) const {
    return fields_[i][j];
}

template<typename Number>
Number& RotationMatrix<Number>::operator()(size_t i, size_t j) {
    return fields_[i][j];
}

template<typename Number>
Vector3<Number> RotationMatrix<Number>::operator*(const Vector3<Number>& vector) const {
    return {
            fields_[0][0] * vector.x + fields_[0][1] * vector.y + fields_[0][2] * vector.z,
            fields_[1][0] * vector.x + fields_[1][1] * vector.y + fields_[1][2] * vector.z,
            fields_[2][0] * vector.x + fields_[2][1] * vector.y + fields_[2][2] * vector.z
    };
}

#endif  // ROTATION_IMPL_H
