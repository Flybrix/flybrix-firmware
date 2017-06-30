#ifndef IMU_H
#define IMU_H

#include "AK8963.h"
#include "MPU9250.h"
#include "utility/rotation.h"
#include "utility/vector3.h"
#include "rotationEstimator.h"

class State;
class I2CManager;

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
    Imu(State& state, I2CManager& i2c);

    bool hasCorrectIDs();

    void initialize();
    void restart();
    void correctBiasValues();
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
        calibrate_rotation_ = calibrating;
        calibration_pose_ = pose;
    }

    bool startInertialMeasurement();
    bool startMagnetFieldMeasurement();

    AK8963::MagBias& magnetometer_bias();

    PcbTransform pcb_transform;

   private:
    bool calibrate_rotation_{false};
    RotationEstimator::Pose calibration_pose_{RotationEstimator::Pose::Flat};
    RotationEstimator rotation_estimator_;
    RotationMatrix<float> sensor_to_flyer_;
    AK8963 magnetometer_;
    MPU9250 accel_and_gyro_;
    State& state_;
};

#endif  // IMU_H
