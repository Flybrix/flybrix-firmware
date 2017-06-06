#ifndef IMU_H
#define IMU_H

#include "AK8963.h"
#include "MPU9250.h"
#include "utility/rotation.h"
#include "utility/vector3.h"

class State;
class I2CManager;

class Imu final {
   public:
    Imu(State& state, I2CManager& i2c);

    bool hasCorrectIDs();

    void initialize();
    void restart();
    void correctBiasValues();
    void forgetBiasValues();

    bool startInertialMeasurement();
    bool startMagnetFieldMeasurement();

    Vector3<float> magnet_field() const;
    Vector3<float> linear_acceleration() const;
    Vector3<float> angular_velocity() const;

    AK8963::MagBias& magnetometer_bias();

   private:
    RotationMatrix<float> pcb_to_world_;
    AK8963 magnetometer_;
    MPU9250 accel_and_gyro_;
    State& state_;
};

#endif  // IMU_H