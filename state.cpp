/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "debug.h"
#include "state.h"

#include <Arduino.h>

#include "systems.h"

// DEFAULT FILTER SETTINGS

#define STATE_EXPECTED_TIME_STEP 0.002f  // 500Hz
#define STATE_BARO_VARIANCE 1e-3f
#define STATE_GRAVITY_IIR_PER_SEC 0.01f
#define STATE_T_SCALE 0.01f
#define STATE_P_SCALE 0.000039063f

State::Parameters::Parameters() : state_estimation{1.0, 0.01}, enable{0.001, 30.0} {
}

State::State(Systems* sys) : localization(0.0f, 1.0f, 0.0f, 0.0f, STATE_EXPECTED_TIME_STEP, Ahrs::Type::Madgwick, parameters.state_estimation, STATE_BARO_VARIANCE), sys_{sys} {
}

bool State::stable(void) const {
    Vector3<float> variance = accel_filter_sq - accel_filter.squared();
    float max_variance = std::max(std::max(variance.x, variance.y), variance.z);
    return (max_variance < parameters.enable[0]);
}

float State::fast_cosine(float x_deg) {
    return 1.0f + x_deg * (-0.000275817445684765f - 0.00013858051199801900f * x_deg);
}

bool State::upright(void) const {
    // cos(angle) = (a dot g) / |a| / |g| = -a.z
    // cos(angle)^2 = a.z*a.z / (a dot a)
    float cos_test_angle = fast_cosine(parameters.enable[1]);
    return (accel_filter.z * accel_filter.z > accel_filter.lengthSq() * cos_test_angle * cos_test_angle);
}

void State::resetState() {
    localization.setTime(0.0f);
    sys_->kinematics = Kinematics();
    sys_->bmp.p0 = sys_->bmp.pressure;  // reset filter to current value
    localization = Localization(0.0f, 1.0f, 0.0f, 0.0f, STATE_EXPECTED_TIME_STEP, Ahrs::Type::Madgwick, parameters.state_estimation, STATE_BARO_VARIANCE);
}

float State::mixRadians(float w1, float a1, float a2) {
    float correction = 0.0f;
    if ((a2 - a1) > PI)
        correction = TWO_PI;
    else if ((a2 - a1) < -PI)
        correction = -TWO_PI;
    return (1.0f - w1) * a2 + w1 * (a1 + correction);
}

void State::updateStateIMU(uint32_t currentTime, const Vector3<float>& accel, const Vector3<float>& gyro) {
    // update IIRs (@500Hz)
    gyro_filter = gyro * 0.1 + gyro_filter * 0.9;
    accel_filter = accel * 0.1 + accel_filter * 0.9;
    accel_filter_sq = accel.squared() * 0.1 + accel_filter_sq * 0.9;

    Vector3<float> rate_scaled = gyro * DEG2RAD;

    sys_->kinematics.rate.pitch = rate_scaled.x;
    sys_->kinematics.rate.roll = rate_scaled.y;
    sys_->kinematics.rate.yaw = rate_scaled.z;

    localization.ProcessMeasurementIMU(currentTime, rate_scaled, accel);

    q = localization.getAhrsQuaternion();
    float r11 = 2.0f * (q.y * q.z + q.x * q.w);
    float r12 = q.x * q.x + q.y * q.y - q.z * q.z - q.w * q.w;
    float r21 = -2.0f * (q.y * q.w - q.x * q.z);
    float r31 = 2.0f * (q.z * q.w + q.x * q.y);
    float r32 = q.x * q.x - q.y * q.y - q.z * q.z + q.w * q.w;
    sys_->kinematics.angle.pitch = -atan2(r11, r12);
    sys_->kinematics.angle.roll = asin(r21);
    sys_->kinematics.angle.yaw = -atan2(r31, r32);
}

void State::updateStatePT(uint32_t currentTime) {
    localization.ProcessMeasurementPT(currentTime, STATE_P_SCALE * sys_->bmp.p0, STATE_P_SCALE * sys_->bmp.pressure, STATE_T_SCALE * sys_->bmp.temperature);
    sys_->kinematics.altitude = localization.getElevation();
}

void State::updateStateMag(const Vector3<float>& data) {
    localization.ProcessMeasurementMagnetometer(data);
}
