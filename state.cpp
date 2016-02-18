/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "state.h"

// DEFAULT FILTER SETTINGS

#define STATE_EXPECTED_TIME_STEP 0.002f  // 500Hz
#define STATE_BARO_VARIANCE 1e-3f
#define STATE_GRAVITY_IIR_PER_SEC 0.01f
#define STATE_T_SCALE 0.01f
#define STATE_P_SCALE 0.000039063f

State::State() : localization(0.0f, 1.0f, 0.0f, 0.0f, STATE_EXPECTED_TIME_STEP, FilterType::Madgwick, CONFIG.data.stateEstimationParameters, STATE_BARO_VARIANCE) {
}

boolean State::stable(void) {
    float max_variance = 0.0f;
    for (int i = 0; i < 3; i++) {
        float variance = accel_filter_sq[i] - accel_filter[i] * accel_filter[i];
        if (variance > max_variance) {
            max_variance = variance;
        }
    }
    return (max_variance < CONFIG.data.enableParameters[0]);
}

float State::fast_cosine(float x_deg) {
    return 1.0f + x_deg * (-0.000275817445684765f - 0.00013858051199801900f * x_deg);
}

boolean State::upright(void) {
    // cos(angle) = (a dot g) / |a| / |g| = -a[2]
    return (-accel_filter[2] > fast_cosine(CONFIG.data.enableParameters[1]));
}

void State::processMotorEnablingIteration(void) {
    if (is(STATUS_ENABLED)) {  // lazy GUI calls...
        // ERROR: ("DEBUG: extra call to processMotorEnablingIteration()!");
    } else if (is(STATUS_IDLE)) {  // first call
        clear(STATUS_IDLE);
        set(STATUS_ENABLING);
        set(STATUS_CLEAR_MPU_BIAS);  // our filters will start filling with fresh values!
        enableAttempts = 0;
    } else if (is(STATUS_ENABLING)) {
        enableAttempts++;  // we call this routine from "command" at 40Hz
        if (!upright()) {
            clear(STATUS_ENABLING);
            set(STATUS_FAIL_ANGLE);
        }
        // wait ~1 seconds for the IIR filters to adjust to their bias free values
        if (enableAttempts == 40) {
            if (!stable()) {
                clear(STATUS_ENABLING);
                set(STATUS_FAIL_STABILITY);
            } else {
                set(STATUS_SET_MPU_BIAS);  // now our filters will start filling with accurate
            }

        } else if (enableAttempts == 41) {  // reset the filter to start letting state reconverge with bias corrected mpu data
            resetState();
        }
        // wait ~1 seconds for the state filter to converge
        else if (enableAttempts > 80) {  // check one more time to see if we were stable
            if (!stable()) {
                clear(STATUS_ENABLING);
                set(STATUS_FAIL_STABILITY);

            } else {
                clear(STATUS_ENABLING);
                set(STATUS_ENABLED);
            }
        }
    }
}

void State::disableMotors(void) {
    clear(STATUS_ENABLED);
    clear(STATUS_FAIL_STABILITY);
    clear(STATUS_FAIL_ANGLE);
    set(STATUS_IDLE);
}

void State::set(const uint16_t status_code) {
    status |= status_code;
}

void State::clear(const uint16_t status_code) {
    status &= ~(status_code);
}

boolean State::is(const uint16_t status_code) const {
    return (status & status_code);
}

void State::resetState() {
    localization.setTime(0.0f);
    kinematicsAngle[0] = 0.0f;  // radians -- pitch/roll/yaw (x,y,z)
    kinematicsAngle[1] = 0.0f;
    kinematicsAngle[2] = 0.0f;
    kinematicsRate[0] = 0.0f;  // radians -- pitch/roll/yaw (x,y,z)
    kinematicsRate[1] = 0.0f;
    kinematicsRate[2] = 0.0f;
    kinematicsAltitude = 0.0f;  // meters
    p0 = pressure;              // reset filter to current value
    localization = Localization(0.0f, 1.0f, 0.0f, 0.0f, STATE_EXPECTED_TIME_STEP, FilterType::Madgwick, CONFIG.data.stateEstimationParameters, STATE_BARO_VARIANCE);
}

float State::mixRadians(float w1, float a1, float a2) {
    float correction = 0.0f;
    if ((a2 - a1) > PI)
        correction = TWO_PI;
    else if ((a2 - a1) < -PI)
        correction = -TWO_PI;
    return (1.0f - w1) * a2 + w1 * (a1 + correction);
}

void State::updateStateIMU(uint32_t currentTime) {
    // update IIRs (@500Hz)
    for (int i = 0; i < 3; i++) {
        gyro_filter[i] = 0.1 * gyro[i] + 0.9 * gyro_filter[i];
        accel_filter[i] = 0.1 * accel[i] + 0.9 * accel_filter[i];
        accel_filter_sq[i] = 0.1 * accel[i] * accel[i] + 0.9 * accel_filter_sq[i];
    }

    for (int i = 0; i < 3; i++) {
        kinematicsRate[i] = gyro[i] * DEG2RAD;
    }
    localization.ProcessMeasurementIMU(currentTime, kinematicsRate, accel);

    const float* q = localization.getAhrsQuaternion();
    float r11 = 2.0f * (q[2] * q[3] + q[1] * q[0]);
    float r12 = q[1] * q[1] + q[2] * q[2] - q[3] * q[3] - q[0] * q[0];
    float r21 = -2.0f * (q[2] * q[0] - q[1] * q[3]);
    float r31 = 2.0f * (q[3] * q[0] + q[1] * q[2]);
    float r32 = q[1] * q[1] - q[2] * q[2] - q[3] * q[3] + q[0] * q[0];
    kinematicsAngle[0] = -atan2(r11, r12);
    kinematicsAngle[1] = asin(r21);
    kinematicsAngle[2] = -atan2(r31, r32);
}

void State::updateStatePT(uint32_t currentTime) {
    localization.ProcessMeasurementPT(currentTime, STATE_P_SCALE * p0, STATE_P_SCALE * pressure, STATE_T_SCALE * temperature);
    kinematicsAltitude = localization.getElevation();
}

void State::updateStateMag() {
    localization.ProcessMeasurementMagnetometer(mag);
}
