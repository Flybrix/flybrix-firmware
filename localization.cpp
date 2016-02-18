#include "localization.h"

#include <algorithm>
#include <cmath>
#include "ahrs.h"
#include "kalman.h"

#define SE_ACC_VARIANCE 0.01f

#define SE_STATE_P_Z 0
#define SE_STATE_V_Z 1
#define SE_STATE_A_Z 2

namespace {
void se_compensate_imu_gyro_offsets(const float gyro_drift[3], float gyro[3]) {
    int i;
    for (i = 0; i < 3; ++i)
        gyro[i] -= gyro_drift[i];
}

void se_compensate_imu_acc_offsets(const float q[4], float gravity, float acc[3]) {
    acc[0] += gravity * 2.0f * (q[1] * q[3] - q[0] * q[2]);
    acc[1] += gravity * 2.0f * (q[2] * q[3] + q[0] * q[1]);
    acc[2] += gravity * (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}

void se_compensate_imu(float delta_time, FilterType type, const float parameters[], IMUState* state, float gyro[3], float acc[3], float mag[3], int use_mag) {
    se_compensate_imu_gyro_offsets(state->gyro_drift, gyro);

    switch (type) {
        case FilterType::Madgwick:
            if (use_mag)
                se_madgwick_ahrs_update_imu_with_mag(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], delta_time, parameters[0], state->q);
            else
                se_madgwick_ahrs_update_imu(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], delta_time, parameters[0], state->q);
            break;
        case FilterType::Mahony:
            if (use_mag)
                se_mahony_ahrs_update_imu_with_mag(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], delta_time, parameters[0], parameters[1], state->fb_i, state->q);
            else
                se_mahony_ahrs_update_imu(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], delta_time, parameters[0], parameters[1], state->fb_i, state->q);
            break;
    }

    se_compensate_imu_acc_offsets(state->q, state->gravity, acc);
}

float getVerticalAcceleration(const float* q, const float* v) {
    return 2.0f * (q[1] * q[3] - q[0] * q[2]) * v[0] + 2.0f * (q[2] * q[3] + q[0] * q[1]) * v[1] + (1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * v[2];
}

float powForPT(float x) {
    // gives satisfactory results for x = 0.5 .. 1.5
    return ((0.0464624f * x - 0.216406f) * x + 0.483648f) * x + 0.686296f;
}

float calculateElevation(float p_sl, float p, float t) {
    return (powForPT(p_sl / p) - 1.0f) * (t + 273.15f) / 0.0065f;
}
}

Localization::Localization(float deltaTime, FilterType ahrsType, const float* ahrsParameters, float elevationVariance)
    : Localization(1.0f, 0.0f, 0.0f, 0.0f, deltaTime, ahrsType, ahrsParameters, elevationVariance) {
}

Localization::Localization(float q0, float q1, float q2, float q3, float deltaTime, FilterType ahrsType, const float* ahrsParameters, float elevationVariance)
    : imuState{{0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, {q0, q1, q2, q3}, {0.0f, 0.0f, 0.0f}},
      z{0.0, 0.0, 0.0},
      zCovar{1e30f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f},
      magLastMeas{0.0f, 0.0f, 0.0f},
      hasMagMeas{false},
      deltaTime(deltaTime),
      ahrsParameters(ahrsParameters),
      elevationVariance(elevationVariance),
      ahrsType(ahrsType),
      timeNow(0.0f) {
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

void Localization::ProcessMeasurementIMU(unsigned int time, const float* gyroscope, const float* accelerometer) {
    float gyroCorrected[3];
    float accelCorrected[3];
    float deltaTime = (time - timeNow) / 1000000.0f;
    deltaTime = std::min(deltaTime, 4.0f * this->deltaTime);
    for (int i = 0; i < 3; ++i) {
        gyroCorrected[i] = gyroscope[i];
        accelCorrected[i] = accelerometer[i] * 9.81f;
    }
    se_compensate_imu(deltaTime, ahrsType, ahrsParameters, &imuState, gyroCorrected, accelCorrected, magLastMeas, hasMagMeas);
    hasMagMeas = false;
    se_kalman_predict(deltaTime, z, zCovar);
    timeNow = time;
    se_kalman_correct(z, zCovar, SE_STATE_A_Z, getVerticalAcceleration(imuState.q, accelCorrected), SE_ACC_VARIANCE);
}

void Localization::ProcessMeasurementMagnetometer(const float* magnetometer) {
    for (int i = 0; i < 3; ++i)
        magLastMeas[i] = magnetometer[i];
    hasMagMeas = true;
}

void Localization::setTime(unsigned int time) {
    timeNow = time;
}

void Localization::setGravityEstimate(float gravity) {
    imuState.gravity = gravity;
}

void Localization::setGyroDriftEstimate(float x, float y, float z) {
    imuState.gyro_drift[0] = x;
    imuState.gyro_drift[1] = y;
    imuState.gyro_drift[2] = z;
}

const float* Localization::getAhrsQuaternion() const {
    return imuState.q;
}

float Localization::getElevation() const {
    return z[0];
}
