#ifndef ROTATION_H
#define ROTATION_H

#include "vector3.h"

template <typename Number>
class RotationMatrix final {
   public:
    RotationMatrix() : fields_{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {
    }

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

   private:
    Number fields_[3][3];
};

#endif  // ROTATION_H
