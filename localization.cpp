#include "localization.h"

#include <algorithm>
#include <cmath>
#include "kalman.h"

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
}

Localization::Localization(float deltaTime, Ahrs::Type ahrsType, const float* ahrsParameters, float elevationVariance)
    : Localization(1.0f, 0.0f, 0.0f, 0.0f, deltaTime, ahrsType, ahrsParameters, elevationVariance) {
}

Localization::Localization(float q0, float q1, float q2, float q3, float deltaTime, Ahrs::Type ahrsType, const float* ahrsParameters, float elevationVariance)
    : z{0.0, 0.0, 0.0}, zCovar{1e30f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f}, ahrsParameters(ahrsParameters), elevationVariance(elevationVariance) {
    ahrs_.pose() = Quaternion<float>(q0, q1, q2, q3);
    ahrs_.setType(ahrsType).setParameters(ahrsParameters[0], ahrsParameters[1]).setMaxDeltaTime(deltaTime);
    setTime(0);
    setGravityEstimate(9.81f);
}

void Localization::ProcessMeasurementElevation(unsigned int time, float elevation) {
    se_kalman_predict((time - timeNow) / 1000000.0f, z, zCovar);
    timeNow = time;
    se_kalman_correct(z, zCovar, SE_STATE_P_Z, elevation, elevationVariance);
}

void Localization::ProcessMeasurementPT(unsigned int time, float p_sl, float p, float t) {
    ProcessMeasurementElevation(time, calculateElevation(p_sl, p, t));
}

void Localization::ProcessMeasurementIMU(unsigned int time, const Vector3<float>& gyroscope, const Vector3<float>& accelerometer) {
    ahrs_.setParameters(ahrsParameters[0], ahrsParameters[1]);

    ahrs_.setGyroscope(gyroscope - gyro_drift_);
    ahrs_.setAccelerometer(accelerometer * 9.81f);

    ahrs_.update(time);
}

void Localization::ProcessMeasurementMagnetometer(const Vector3<float>& magnetometer) {
    ahrs_.setMagnetometer(magnetometer);
}

void Localization::setTime(unsigned int time) {
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
    return z[0];
}
