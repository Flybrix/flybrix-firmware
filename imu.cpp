#include "imu.h"

#include "state.h"
#include "i2cManager.h"

Imu::Imu(State& state, I2CManager& i2c) : magnetometer_{&state, &i2c, pcb_to_world_}, accel_and_gyro_{&state, &i2c, pcb_to_world_}, state_(state) {
}

bool Imu::hasCorrectIDs() {
    return accel_and_gyro_.getID() == 0x71 && magnetometer_.getID() == 0x48;
}

void Imu::initialize() {
    // Important so we set the ready flag!
    while (!accel_and_gyro_.startMeasurement()) {
        delay(1);
    }
    while (!magnetometer_.startMeasurement()) {
        delay(1);
    }
}

void Imu::restart() {
    magnetometer_.restart();
    accel_and_gyro_.restart();
}

void Imu::correctBiasValues() {
    accel_and_gyro_.correctBiasValues();
}

void Imu::forgetBiasValues() {
    accel_and_gyro_.forgetBiasValues();
}

bool Imu::startInertialMeasurement() {
    if (!accel_and_gyro_.ready) {
        return false;
    }

    if (do_state_update_) {
        state_.updateStateIMU(micros());  // update state as often as we can
    }

    do_state_update_ = accel_and_gyro_.startMeasurement();

    return do_state_update_;
}

bool Imu::startMagnetFieldMeasurement() {
    if (!magnetometer_.ready) {
        return false;
    }
    return magnetometer_.startMeasurement();
}

AK8963::MagBias& Imu::magnetometer_bias() {
    return magnetometer_.mag_bias;
}
