/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "imu.h"

#include "quickmath.h"
#include "state.h"
#include "utility/clock.h"

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
    //order is important!
    accel_and_gyro_.restart(); //first
    magnetometer_.restart();   //second
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

namespace {
// The IIRs below feed bias calibration and the arming checks. The original 0.1 weight was used at a
// ~170Hz sample rate (time constant ~60ms); scale it with the sample rate to keep that time constant.
constexpr float IMU_SAMPLE_RATE_HZ = 1000.0f / (1 + MPU9250::SAMPLE_RATE_DIVIDER);
constexpr float FILTER_WEIGHT = 0.1f * 170.0f / IMU_SAMPLE_RATE_HZ;
}  // namespace

bool Imu::healthy(ClockTime now) const {
    return has_sample_ && (now - last_sample_time_) < MAX_SAMPLE_AGE_USEC;
}

void Imu::updateAccelGyro(ClockTime time, const Vector3<float>& accel, const Vector3<float>& gyro) {
    has_sample_ = true;
    ++sample_count_;
    last_sample_time_ = time;

    accel_ = accel;
    gyro_ = gyro;
    gyro_filter = gyro * FILTER_WEIGHT + gyro_filter * (1.0f - FILTER_WEIGHT);
    accel_filter = accel * FILTER_WEIGHT + accel_filter * (1.0f - FILTER_WEIGHT);
    accel_filter_sq = accel.squared() * FILTER_WEIGHT + accel_filter_sq * (1.0f - FILTER_WEIGHT);

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
        updateAccelGyro(ClockTime::now(), sensor_to_flyer_ * linear_acceleration, sensor_to_flyer_ * angular_velocity);
    });
}

bool Imu::startMagnetFieldMeasurement() {
    if (!magnetometer_.ready) {
        return false;
    }
    return magnetometer_.startMeasurement([this](Vector3<float> magnet_field) { updateMag(sensor_to_flyer_ * magnet_field); });
}

bool Imu::magnetReportsAllZeroes() {
    return magnetometer_.allZeroes;
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
