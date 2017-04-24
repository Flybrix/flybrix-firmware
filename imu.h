#ifndef IMU_H
#define IMU_H

#include "AK8963.h"
#include "MPU9250.h"
#include "utility/rotation.h"

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

    AK8963::MagBias& magnetometer_bias();

   private:
    RotationMatrix<float> pcb_to_world_;
    AK8963 magnetometer_;
    MPU9250 accel_and_gyro_;
    State& state_;
    bool do_state_update_{true};
};

#endif  // IMU_H
