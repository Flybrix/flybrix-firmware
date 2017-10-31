/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "debug.h"
#include "state.h"

#include <Arduino.h>

#include "systems.h"
#include "utility/quaternion.h"

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

void State::resetState() {
    localization.setTime(0.0f);
    kinematics = Kinematics();
    sys_->bmp.p0 = sys_->bmp.pressure;  // reset filter to current value
    localization = Localization(0.0f, 1.0f, 0.0f, 0.0f, STATE_EXPECTED_TIME_STEP, Ahrs::Type::Madgwick, parameters.state_estimation, STATE_BARO_VARIANCE);
}

void State::updateLocalization(uint32_t currentTime, const Vector3<float>& accel, const Vector3<float>& rate_scaled) {
    kinematics.rate.pitch = rate_scaled.x;
    kinematics.rate.roll = rate_scaled.y;
    kinematics.rate.yaw = rate_scaled.z;

    localization.ProcessMeasurementIMU(currentTime, rate_scaled, accel);

    Quaternion<float> q = localization.getAhrsQuaternion();
    kinematics.angle.pitch = q.pitch();
    kinematics.angle.roll = q.roll();
    kinematics.angle.yaw = q.yaw();
}

void State::readStatePT() {
    localization.ProcessMeasurementPT(STATE_P_SCALE * sys_->bmp.p0, STATE_P_SCALE * sys_->bmp.pressure, STATE_T_SCALE * sys_->bmp.temperature);
}

void State::updateFilter(uint32_t time) {
    localization.updateFilter(time);
    kinematics.altitude = localization.getElevation();
}

void State::updateStateMag(const Vector3<float>& data) {
    localization.ProcessMeasurementMagnetometer(data);
}
