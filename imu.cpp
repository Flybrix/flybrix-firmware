#include "imu.h"

#include "state.h"
#include "i2cManager.h"

Imu::Imu(State& state, I2CManager& i2c) : magnetometer_{i2c, pcb_to_world_}, accel_and_gyro_{i2c, pcb_to_world_}, state_(state) {
}

bool Imu::hasCorrectIDs() {
    return accel_and_gyro_.getID() == 0x71 && magnetometer_.getID() == 0x48;
}

void Imu::initialize() {
    // Important so we set the ready flag!
    // Do not actually pass results to state
    while (!accel_and_gyro_.startMeasurement([]() {})) {
        delay(1);
    }
    while (!magnetometer_.startMeasurement([]() {})) {
        delay(1);
    }
}

void Imu::restart() {
    magnetometer_.restart();
    accel_and_gyro_.restart();
}

void Imu::correctBiasValues() {
    accel_and_gyro_.correctBiasValues(state_.accel_filter, state_.gyro_filter);
}

void Imu::forgetBiasValues() {
    accel_and_gyro_.forgetBiasValues();
}

bool Imu::startInertialMeasurement() {
    if (!accel_and_gyro_.ready) {
        return false;
    }

    return accel_and_gyro_.startMeasurement([this]() { state_.updateStateIMU(micros(), accel_and_gyro_.linear_acceleration, accel_and_gyro_.angular_velocity); });
}

bool Imu::startMagnetFieldMeasurement() {
    if (!magnetometer_.ready) {
        return false;
    }
    return magnetometer_.startMeasurement([this]() { state_.updateStateMag(magnetometer_.last_read); });
}

Vector3<float> Imu::magnet_field() const {
    return magnetometer_.last_read;
}

Vector3<float> Imu::linear_acceleration() const {
    return accel_and_gyro_.linear_acceleration;
}

Vector3<float> Imu::angular_velocity() const {
    return accel_and_gyro_.angular_velocity;
}

AK8963::MagBias& Imu::magnetometer_bias() {
    return magnetometer_.mag_bias;
}
