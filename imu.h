/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef IMU_H
#define IMU_H

#include "AK8963.h"
#include "MPU9250.h"
#include "utility/clock.h"
#include "utility/rotation.h"
#include "utility/vector3.h"
#include "rotationEstimator.h"

class State;

struct __attribute__((packed)) PcbTransform {
    bool verify() const {
        return true;
    }
    Vector3<float> orientation;  // x/y/z representing pitch/roll/yaw in standard flyer coordinate system
                                 // --> applied in that order!
    Vector3<float> translation;  // translation in standard flyer coordinate system
};

static_assert(sizeof(PcbTransform) == 3 * 2 * 4, "Data is not packed");

class Imu final {
   public:
    explicit Imu(State& state);

    bool hasCorrectIDs();

    void initialize();
    void restart();
    void readBiasValues();
    void forgetBiasValues();
    void parseConfig();

    void setMagnetometerCalibrating(bool calibrating) {
        magnetometer_.setCalibrating(calibrating);
    }

    void setAccelerometerCalibrating(bool calibrating, RotationEstimator::Pose pose) {
        if (calibrate_rotation_ && !calibrating) {
            sensor_to_flyer_ = rotation_estimator_.estimate();
            pcb_transform.orientation = sensor_to_flyer_.pry();
            rotation_estimator_.clear();
        }
        if (calibrating && pose == RotationEstimator::Pose::Flat) {
            forgetBiasValues();
        }
        calibrate_rotation_ = calibrating;
        calibration_pose_ = pose;
    }

    bool startInertialMeasurement();
    bool startMagnetFieldMeasurement();
    bool magnetReportsAllZeroes();

    bool upright() const;
    bool stable() const;

    // A read of the accelerometer/gyro has been queued and has not completed (or failed) yet.
    bool inertialMeasurementPending() const {
        return !accel_and_gyro_.ready;
    }

    // If no good accelerometer/gyro sample arrives for this long, the attitude data is stale and it is
    // unsafe to keep flying on it.
    static constexpr uint32_t MAX_SAMPLE_AGE_USEC = 50000;

    // True if a good accelerometer/gyro sample has arrived within MAX_SAMPLE_AGE_USEC.
    bool healthy(ClockTime now) const;

    // Number of good accelerometer/gyro samples processed so far.
    uint32_t sampleCount() const {
        return sample_count_;
    }

    const Vector3<float>& accel() const;
    const Vector3<float>& gyro() const;
    const Vector3<float>& mag() const;

    AK8963::MagBias& magnetometer_bias();

    PcbTransform pcb_transform;

    struct __attribute__((packed)) InertialBias {
        bool verify() const {
            return true;
        }
        Vector3<float> accel;
        Vector3<float> gyro;
    } bias;

    static_assert(sizeof(InertialBias) == 2 * 3 * 4, "Data is not packed");

   private:
    void correctBiasValues();
    void updateMag(const Vector3<float>& mag);
    void updateAccelGyro(ClockTime time, const Vector3<float>& accel, const Vector3<float>& gyro);

    Vector3<float> accel_filter{0.0, 0.0, 0.0}, accel_filter_sq{0.0, 0.0, 0.0};  // for stability variance calculation
    Vector3<float> gyro_filter{0.0, 0.0, 0.0};                                   // for gyro drift correction

    bool has_sample_{false};
    uint32_t sample_count_{0};
    ClockTime last_sample_time_{ClockTime::zero()};

    Vector3<float> accel_{0.0, 0.0, 0.0};
    Vector3<float> gyro_{0.0, 0.0, 0.0};
    Vector3<float> mag_{0.0, 0.0, 0.0};

    bool calibrate_rotation_{false};
    RotationEstimator::Pose calibration_pose_{RotationEstimator::Pose::Flat};
    RotationEstimator rotation_estimator_;
    RotationMatrix<float> sensor_to_flyer_;
    AK8963 magnetometer_;
    MPU9250 accel_and_gyro_;
    State& state_;
};

#endif  // IMU_H
