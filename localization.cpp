/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "localization.h"

#include <algorithm>
#include <cmath>

#define SE_ACC_VARIANCE 0.01f

#define SE_STATE_P_Z 0
#define SE_STATE_V_Z 1
#define SE_STATE_A_Z 2

namespace {
float powForPT(float x) {
    // gives satisfactory results for x = 0.5 .. 1.5
    return ((0.0464624f * x - 0.216406f) * x + 0.483648f) * x + 0.686296f;
}

float calculateElevation(float p_sl, float p, float t) {
    return (powForPT(p_sl / p) - 1.0f) * (t + 273.15f) / 0.0065f;
}
}  // namespace

Localization::Localization(float deltaTime, Ahrs::Type ahrsType, const float* ahrsParameters, float elevation_variance)
    : Localization(1.0f, 0.0f, 0.0f, 0.0f, deltaTime, ahrsType, ahrsParameters, elevation_variance) {
}

Localization::Localization(float q0, float q1, float q2, float q3, float deltaTime, Ahrs::Type ahrsType, const float* ahrsParameters, float elevation_variance)
    : max_delta_time_{deltaTime * 4}, ahrsParameters(ahrsParameters), elevation_variance_(elevation_variance) {
    ahrs_.pose() = Quaternion<float>(q0, q1, q2, q3);
    ahrs_.setType(ahrsType).setParameters(ahrsParameters[0], ahrsParameters[1]).setMaxDeltaTime(max_delta_time_);
    setTime(0);
    setGravityEstimate(9.81f);
}

void Localization::ProcessMeasurementElevation(float elevation) {
    h_bar_ = UKF::Measurement(elevation, elevation_variance_);
    has_measurement_ = true;
}

void Localization::ProcessMeasurementPT(float p_sl, float p, float t) {
    ProcessMeasurementElevation(calculateElevation(p_sl, p, t));
}

uint8_t stage{0};

void Localization::updateFilter(uint32_t time) {
    if (!has_measurement_) {
        return;
    }
    if (time <= timeNow) {
        timeNow = time;
        return;
    }
    float dt{(time - timeNow) / 1000000.0f};
    if (dt > max_delta_time_) {
        dt = max_delta_time_;
    }
    timeNow = time;

    if (stage == 0) {
        ukf_.predict(dt);
        stage++;
    }
    else if (stage == 1) {
        ukf_.update(vu_, vv_, d_tof_, h_bar_, ahrs_.pose().pitch(), ahrs_.pose().roll());
        stage = 0;
    }
    
}

void Localization::ProcessMeasurementIMU(uint32_t time, const Vector3<float>& gyroscope, const Vector3<float>& accelerometer) {
    ahrs_.setParameters(ahrsParameters[0], ahrsParameters[1]);

    ahrs_.setGyroscope(gyroscope - gyro_drift_);
    ahrs_.setAccelerometer(accelerometer * 9.81f);

    ahrs_.update(time);
}

void Localization::ProcessMeasurementMagnetometer(const Vector3<float>& magnetometer) {
    ahrs_.setMagnetometer(magnetometer);
}

void Localization::setTime(uint32_t time) {
    timeNow = time;
    ahrs_.setTimestamp(time);
}

void Localization::setGravityEstimate(float gravity) {
    gravity_force_ = gravity;
}

void Localization::setGyroDriftEstimate(float x, float y, float z) {
    gyro_drift_ = Vector3<float>(x, y, z);
}

const Quaternion<float>& Localization::getAhrsQuaternion() const {
    return ahrs_.pose();
}

float Localization::getElevation() const {
    return ukf_.elevation();
}

Vector3<float> Localization::getVelocity() const {
    return {ukf_.vx(), ukf_.vy(), ukf_.vz()};
}
