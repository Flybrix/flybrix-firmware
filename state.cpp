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

State::State(Systems* sys) : localization(0.0f, 1.0f, 0.0f, 0.0f, STATE_EXPECTED_TIME_STEP, FilterType::Madgwick, parameters.state_estimation, STATE_BARO_VARIANCE), sys_{sys} {
}

bool State::stable(void) const {
    float max_variance = 0.0f;
    for (int i = 0; i < 3; i++) {
        float variance = accel_filter_sq[i] - accel_filter[i] * accel_filter[i];
        if (variance > max_variance) {
            max_variance = variance;
        }
    }
    return (max_variance < parameters.enable[0]);
}

float State::fast_cosine(float x_deg) {
    return 1.0f + x_deg * (-0.000275817445684765f - 0.00013858051199801900f * x_deg);
}

bool State::upright(void) const {
    // cos(angle) = (a dot g) / |a| / |g| = -a[2]
    // cos(angle)^2 = a[2]*a[2] / (a dot a)
    float cos_test_angle = fast_cosine(parameters.enable[1]);
    float a_dot_a = 0.0f;
    for (uint8_t i = 0; i < 3; i++) {
        a_dot_a += accel_filter[i] * accel_filter[i];
    }
    return (accel_filter[2] * accel_filter[2] > a_dot_a * cos_test_angle * cos_test_angle);
}

void State::resetState() {
    localization.setTime(0.0f);
    sys_->kinematics = Kinematics();
    sys_->bmp.p0 = sys_->bmp.pressure;  // reset filter to current value
    localization = Localization(0.0f, 1.0f, 0.0f, 0.0f, STATE_EXPECTED_TIME_STEP, FilterType::Madgwick, parameters.state_estimation, STATE_BARO_VARIANCE);
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
    gyro_filter[0] = 0.1 * gyro.x + 0.9 * gyro_filter[0];
    gyro_filter[1] = 0.1 * gyro.y + 0.9 * gyro_filter[1];
    gyro_filter[2] = 0.1 * gyro.z + 0.9 * gyro_filter[2];
    accel_filter[0] = 0.1 * accel.x + 0.9 * accel_filter[0];
    accel_filter[1] = 0.1 * accel.y + 0.9 * accel_filter[1];
    accel_filter[2] = 0.1 * accel.z + 0.9 * accel_filter[2];
    accel_filter_sq[0] = 0.1 * accel.x * accel.x + 0.9 * accel_filter_sq[0];
    accel_filter_sq[1] = 0.1 * accel.y * accel.y + 0.9 * accel_filter_sq[1];
    accel_filter_sq[2] = 0.1 * accel.z * accel.z + 0.9 * accel_filter_sq[2];

    Vector3<float> rate_scaled{gyro.x * DEG2RAD, gyro.y * DEG2RAD, gyro.z * DEG2RAD};

    sys_->kinematics.rate.pitch = rate_scaled.x;
    sys_->kinematics.rate.roll = rate_scaled.y;
    sys_->kinematics.rate.yaw = rate_scaled.z;

    localization.ProcessMeasurementIMU(currentTime, rate_scaled, accel);

    const float* q = localization.getAhrsQuaternion();
    float r11 = 2.0f * (q[2] * q[3] + q[1] * q[0]);
    float r12 = q[1] * q[1] + q[2] * q[2] - q[3] * q[3] - q[0] * q[0];
    float r21 = -2.0f * (q[2] * q[0] - q[1] * q[3]);
    float r31 = 2.0f * (q[3] * q[0] + q[1] * q[2]);
    float r32 = q[1] * q[1] - q[2] * q[2] - q[3] * q[3] + q[0] * q[0];
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
