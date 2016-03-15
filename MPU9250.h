/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <MPU9250.h/cpp>

    Driver code for our inertial sensor.

*/

#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "i2cManager.h"

class State;

// we have three coordinate systems here:
// 1. REGISTER coordinates: native values as read
// 2. IC/PCB coordinates: matches FLYER system if the pcb is in standard orientation
// 3. FLYER coordinates: if the pcb is mounted in a non-standard way the FLYER system is a rotation of the IC/PCB system

class MPU9250 : public CallbackProcessor {
   public:  // all in FLYER system
    MPU9250(State *state, I2CManager *i2c);

    void restart();  // calculate bias and prepare for flight

    bool ready;

    void correctBiasValues();  // set bias values from state
    void forgetBiasValues();  // discard bias values

    bool startMeasurement();
    void processCallback(uint8_t count, uint8_t *rawData);  // handles return for getAccelGryo()

    float getTemp() {
        return (float)temperatureCount[0] / 333.87 + 21.0;
    }

    uint8_t getID();

    void setFilters(uint8_t gyrofilter, uint8_t accelfilter);

   private:
    State *state;
    I2CManager *i2c;

    bool dataReadyInterrupt();  // check interrupt
    uint8_t getStatusByte();

    void rotate(float R[3][3], float x[3]);

    void reset();
    void configure();  // set up filters and resolutions for flight

    float invSqrt(float x);
    const float aRes = 8.0 / 32768.0;     // +/- 8g
    const float gRes = 1000.0 / 32768.0;  // +/- 1000 deg/s

    // 16-bit raw values, bias correction, factory calibration
    int16_t temperatureCount[1] = {0};
    int16_t gyroCount[3] = {0, 0, 0}, accelCount[3] = {0, 0, 0};
    float gyroBias[3] = {0.0, 0.0, 0.0}, accelBias[3] = {0.0, 0.0, 0.0};

    // buffers for processCallback
    uint8_t data_to_read[14];
    uint8_t data_to_send[1];

};  // class MPU9250

#define DEG2RAD 0.01745329251f

//*************************************************************
//
// MPU Registers (See Table 1 Register Map on page 7)
//
//*************************************************************

#define MPU9250_ADDRESS 0x68
/* Page 32: "The slave address of the MPU-9250 is b110100X which is 7 bits long. The LSB bit of the
    7 bit address is determined by the logic level on pin AD0. This allows two MPU-9250s to be
    connected to the same I2C bus. When used in this configuration, the address of the one of
    the devices should be b1101000 (pin AD0 is logic low) and the address of the other should
be b1101001 (pin AD0 is logic high)." */

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define XG_OFFSET_H 0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define SMPLRT_DIV 0x19
#define MPU9250_CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define LP_ACCEL_ODR 0x1E
#define WOM_THR 0x1F

#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DI 0x35
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38

#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60

#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A   // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1 0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2 0x6C

#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75  // Should return 0x71

#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78

#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B

#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

#endif
