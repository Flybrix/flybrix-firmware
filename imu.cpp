#include "imu.h"

#include "quickmath.h"
#include "state.h"

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
    Vector3<float> a{accel_filter};
    quick::normalize(a);
    bias.accel = accel_filter - a;
    bias.gyro = gyro_filter;
    readBiasValues();
}

void Imu::readBiasValues() {
    accel_and_gyro_.correctBiasValues(bias.accel, bias.gyro);
}

void Imu::forgetBiasValues() {
    accel_and_gyro_.forgetBiasValues();
}

void Imu::parseConfig() {
    sensor_to_flyer_ = RotationMatrix<float>(pcb_transform.orientation.x, pcb_transform.orientation.y, pcb_transform.orientation.z);
    readBiasValues();
}

void Imu::updateAccelGyro(uint32_t time, const Vector3<float>& accel, const Vector3<float>& gyro) {
    // update IIRs (@500Hz)
    accel_ = accel;
    gyro_ = gyro;
    gyro_filter = gyro * 0.1 + gyro_filter * 0.9;
    accel_filter = accel * 0.1 + accel_filter * 0.9;
    accel_filter_sq = accel.squared() * 0.1 + accel_filter_sq * 0.9;

    Vector3<float> rate_scaled = gyro * DEG2RAD;

    state_.updateLocalization(time, accel, rate_scaled);
}

void Imu::updateMag(const Vector3<float>& mag) {
    mag_ = mag;
    state_.updateStateMag(mag);
}

const Vector3<float>& Imu::accel() const {
    return accel_;
}

const Vector3<float>& Imu::gyro() const {
    return gyro_;
}

const Vector3<float>& Imu::mag() const {
    return mag_;
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
        updateAccelGyro(micros(), sensor_to_flyer_ * linear_acceleration, sensor_to_flyer_ * angular_velocity);
    });
}

bool Imu::startMagnetFieldMeasurement() {
    if (!magnetometer_.ready) {
        return false;
    }
    return magnetometer_.startMeasurement([this](Vector3<float> magnet_field) { updateMag(sensor_to_flyer_ * magnet_field); });
}

AK8963::MagBias& Imu::magnetometer_bias() {
    return magnetometer_.mag_bias;
}

#define DEGREES_TO_RADIANS 0.01745329252f

bool Imu::upright() const {
    // cos(angle) = (a dot g) / |a| / |g| = -a.z
    // cos(angle)^2 = a.z*a.z / (a dot a)
    float cos_test_angle = quick::cos(state_.parameters.enable[1] * DEGREES_TO_RADIANS);  // enable angle parameter is specified in degrees
    return accel_filter.z * accel_filter.z > accel_filter.lengthSq() * cos_test_angle * cos_test_angle;
}

bool Imu::stable() const {
    Vector3<float> variance = accel_filter_sq - accel_filter.squared();
    float max_variance = std::max(std::max(variance.x, variance.y), variance.z);
    return max_variance < state_.parameters.enable[0];
}
