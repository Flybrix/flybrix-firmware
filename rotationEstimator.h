#ifndef ROTATION_ESTIMATOR_H
#define ROTATION_ESTIMATOR_H

#include <array>
#include "utility/rotation.h"
#include "utility/vector3.h"

class RotationEstimator final {
   public:
    enum class Pose : uint8_t {
        Flat = 0,
        PitchForward = 1,
        PitchBack = 2,
        RollRight = 3,
        RollLeft = 4,
    };

    void updateGravity(Pose pose, const Vector3<float>& value);
    RotationMatrix<float> estimate() const;

   private:
    struct State {
        Vector3<float> measurement;
        bool handled{false};
    };
    std::array<State, 5> states_;
};

#endif  // ROTATION_ESTIMATOR_H
