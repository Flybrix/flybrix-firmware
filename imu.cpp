#include "imu.h"

#include "state.h"
#include "i2cManager.h"
#include "quickmath.h"

Imu::Imu(State& state, I2CManager& i2c) : magnetometer_{i2c}, accel_and_gyro_{i2c}, state_(state) {
}

bool Imu::hasCorrectIDs() {
    return accel_and_gyro_.getID() == 0x71 && magnetometer_.getID() == 0x48;
}

void Imu::initialize() {
    // Important so we set the ready flag!
    // Do not actually pass results to state
    while (!accel_and_gyro_.startMeasurement([](Vector3<float>, Vector3<float>) {})) {
        delay(1);
    }
    while (!magnetometer_.startMeasurement([](Vector3<float>) {})) {
        delay(1);
    }
}

void Imu::restart() {
    magnetometer_.restart();
    accel_and_gyro_.restart();
}

void Imu::correctBiasValues() {
    Vector3<float> a{state_.accel_filter};
    quick::normalize(a);

    if (a.z > 1.0f - 1e-6f) {
        /* If u and v are antiparallel, perform 180 degree rotation around X. */
        sensor_to_flyer_ = RotationMatrix<float>();
        sensor_to_flyer_(1, 1) = -1.0f;
        sensor_to_flyer_(2, 2) = -1.0f;
    } else {
        /* Otherwise, build quaternion the standard way. */
        // w = sqrt(a.lengthSq() * b.lengthSq()) + a . b = 1 + a . b
        // [x, y, z] = a x b
        // a . [0 0 -1] = -az
        // a x [0 0 -1] = [-ay ax 0]
        Quaternion<float> q(1 - a.z, -a.y, a.x, 0.0f);
        quick::normalize(q);
        sensor_to_flyer_ = q.toRotation();
    }

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            sensor_to_flyer_(i, j) = -sensor_to_flyer_(i, j);
        }
    }

    accel_and_gyro_.correctBiasValues(state_.accel_filter - a, state_.gyro_filter);
}

void Imu::forgetBiasValues() {
    sensor_to_flyer_ = RotationMatrix<float>();
    accel_and_gyro_.forgetBiasValues();
}

bool Imu::startInertialMeasurement() {
    if (!accel_and_gyro_.ready) {
        return false;
    }

    return accel_and_gyro_.startMeasurement(
        [this](Vector3<float> linear_acceleration, Vector3<float> angular_velocity) { state_.updateStateIMU(micros(), sensor_to_flyer_ * linear_acceleration, sensor_to_flyer_ * angular_velocity); });
}

bool Imu::startMagnetFieldMeasurement() {
    if (!magnetometer_.ready) {
        return false;
    }
    return magnetometer_.startMeasurement([this](Vector3<float> magnet_field) { state_.updateStateMag(sensor_to_flyer_ * magnet_field); });
}

AK8963::MagBias& Imu::magnetometer_bias() {
    return magnetometer_.mag_bias;
}
