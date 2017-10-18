#include "imu.h"

#include "state.h"
#include "quickmath.h"

Imu::Imu(State& state) : state_(state) {
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
    accel_and_gyro_.correctBiasValues(state_.accel_filter - a, state_.gyro_filter);
}

void Imu::forgetBiasValues() {
    accel_and_gyro_.forgetBiasValues();
}

void Imu::parseConfig() {
    sensor_to_flyer_ = RotationMatrix<float>(pcb_transform.orientation.x, pcb_transform.orientation.y, pcb_transform.orientation.z);
}

bool Imu::startInertialMeasurement() {
    if (!accel_and_gyro_.ready) {
        return false;
    }

    return accel_and_gyro_.startMeasurement([this](Vector3<float> linear_acceleration, Vector3<float> angular_velocity) {
        if (calibrate_rotation_) {
            rotation_estimator_.updateGravity(calibration_pose_, linear_acceleration);
            if (calibration_pose_ == RotationEstimator::Pose::Flat) {
                correctBiasValues();
            }
        }
        state_.updateStateIMU(micros(), sensor_to_flyer_ * linear_acceleration, sensor_to_flyer_ * angular_velocity);
    });
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
