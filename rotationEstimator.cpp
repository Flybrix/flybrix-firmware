#include "rotationEstimator.h"

#include "debug.h"
#include "quickmath.h"

void RotationEstimator::updateGravity(Pose pose, const Vector3<float>& value) {
    states_[(uint8_t)pose].measurement = value;
    states_[(uint8_t)pose].handled = true;
}

void RotationEstimator::clear() {
    for (RotationEstimator::State& state : states_) {
        state.handled = false;
    }
}

RotationMatrix<float> RotationEstimator::estimate() const {
    RotationMatrix<float> answer;
    for (const RotationEstimator::State& state : states_) {
        if (!state.handled) {
            DebugPrint("Calibration requires all five poses to be handled");
            return answer;
        }
    }
    Vector3<float> up = states_[(uint8_t)RotationEstimator::Pose::Flat].measurement;
    quick::normalize(up);

    Vector3<float> right = states_[(uint8_t)RotationEstimator::Pose::RollRight].measurement.projectOntoPlane(up);
    Vector3<float> left = states_[(uint8_t)RotationEstimator::Pose::RollLeft].measurement.projectOntoPlane(up);

    if (dot(right, left) > 0.0f) {
        DebugPrint("Right and left roll need to be performed in opposite directions");
        return answer;
    }

    quick::normalize(left);

    Vector3<float> forward = states_[(uint8_t)RotationEstimator::Pose::PitchForward].measurement.projectOntoPlane(up).projectOntoPlane(left);
    Vector3<float> back = states_[(uint8_t)RotationEstimator::Pose::PitchBack].measurement.projectOntoPlane(up).projectOntoPlane(left);

    if (dot(forward, back) > 0.0f) {
        DebugPrint("Forward and back pitch need to be performed in opposite directions");
        return answer;
    }

    quick::normalize(back);

    if (dot(cross(left, back), up) <= 0.0f) {
        DebugPrint("Roll and pitch directions were not properly lined up");
        return answer;
    }

    answer(0, 0) = left.x;
    answer(1, 0) = left.y;
    answer(2, 0) = left.z;

    answer(0, 1) = back.x;
    answer(1, 1) = back.y;
    answer(2, 1) = back.z;

    answer(0, 2) = up.x;
    answer(1, 2) = up.y;
    answer(2, 2) = up.z;

    return answer;
}
