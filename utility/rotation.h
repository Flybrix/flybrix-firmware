#ifndef ROTATION_H
#define ROTATION_H

#include <cmath>
#include "vector3.h"

template <typename Number>
class RotationMatrix final {
   public:
    RotationMatrix() : fields_{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {
    }

    RotationMatrix(Number pitch, Number roll, Number yaw);

    const Number& operator()(size_t i, size_t j) const {
        return fields_[i][j];
    }

    Number& operator()(size_t i, size_t j) {
        return fields_[i][j];
    }

    Vector3<Number> operator*(const Vector3<Number>& vector) const {
        return {

            fields_[0][0] * vector.x + fields_[0][1] * vector.y + fields_[0][2] * vector.z,

            fields_[1][0] * vector.x + fields_[1][1] * vector.y + fields_[1][2] * vector.z,

            fields_[2][0] * vector.x + fields_[2][1] * vector.y + fields_[2][2] * vector.z

        };
    }

    Vector3<Number> pry() const {
        return {
            std::atan2(fields_[2][1], fields_[2][2]),                                                              // Pitch
            std::atan2(-fields_[2][0], std::sqrt(fields_[2][1] * fields_[2][1] + fields_[2][2] * fields_[2][2])),  // Roll
            std::atan2(fields_[1][0], fields_[0][0])                                                               // Yaw
        };
    }

   private:
    Number fields_[3][3];
};

template <typename Number>
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

#endif  // ROTATION_H
