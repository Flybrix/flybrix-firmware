/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com

    Credit is due to Kris Winer ("https://github.com/kriswiner/MPU-9250")
*/

#include "MPU9250.h"
#include <i2c_t3.h>
#include <stdio.h>
#include <cmath>
#include "board.h"
#include "i2cManager.h"

// we have three coordinate systems here:
// 1. REGISTER coordinates: native values as read
// 2. IC/PCB coordinates: matches FLYER system if the pcb is in standard orientation
// 3. FLYER coordinates: if the pcb is mounted in a non-standard way the FLYER system is a rotation of the IC/PCB system

// map from the REGISTER coordinate system to the IC/PCB coordinate system
// this is done immediately whenever we read from the registers

#define ACCEL_XDIR 0
#define ACCEL_YDIR 1
#define ACCEL_ZDIR 2
#define ACCEL_XSIGN 1
#define ACCEL_YSIGN 1
#define ACCEL_ZSIGN 1  // verified by experiment (units definition issue?)

#define GYRO_XDIR 0
#define GYRO_YDIR 1
#define GYRO_ZDIR 2
#define GYRO_XSIGN 1
#define GYRO_YSIGN 1
#define GYRO_ZSIGN 1  // verified by experiment

MPU9250::MPU9250() : ready{false} {
    pinMode(board::MPU_INTERRUPT, INPUT);
}

uint8_t MPU9250::getID() {
    return i2c().readByte(MPU9250_ADDRESS, WHO_AM_I);  // Read WHO_AM_I register for MPU-9250 --> 0x71
}

void MPU9250::restart() {
    reset();
    configure();
    forgetBiasValues();
}

bool MPU9250::dataReadyInterrupt() {
    return digitalRead(board::MPU_INTERRUPT);  // use the interrupt pin -- be sure to set INT_PIN_CFG
}

uint8_t MPU9250::getStatusByte() {
    return i2c().readByte(MPU9250_ADDRESS, INT_STATUS);
}

void MPU9250::correctBiasValues(const Vector3<float>& accel_filter, const Vector3<float>& gyro_filter) {
    // the bias correction in the IC/PCB coordinates
    accelBias = accel_filter;

    // biases are additive corrections -- we were not rotating during calibration so we measured gyro drift
    // construct biases for later manual subtraction
    gyroBias = gyro_filter;
}

void MPU9250::forgetBiasValues() {
    gyroBias = Vector3<float>();
    accelBias = Vector3<float>();
}

// writes values to state in g's and in degrees per second
bool MPU9250::startMeasurement(std::function<void(Vector3<float>, Vector3<float>)> on_success) {
    if (dataReadyInterrupt()) {
        ready = false;
        data_to_send[0] = ACCEL_XOUT_H;
        i2c().addTransfer(MPU9250_ADDRESS, 1, data_to_send, 14, data_to_read, [this, on_success]() { triggerCallback(on_success); });
        return true;
    }
    return false;
}

void MPU9250::triggerCallback(std::function<void(Vector3<float>, Vector3<float>)> on_success) {
    // convert from REGISTER system to IC/PCB system
    int16_t registerValuesAccel[3];
    // be careful not to misinterpret 2's complement registers
    registerValuesAccel[0] = (int16_t)(((uint16_t)data_to_read[0]) << 8) | (uint16_t)data_to_read[1];  // high byte, low byte
    registerValuesAccel[1] = (int16_t)(((uint16_t)data_to_read[2]) << 8) | (uint16_t)data_to_read[3];
    registerValuesAccel[2] = (int16_t)(((uint16_t)data_to_read[4]) << 8) | (uint16_t)data_to_read[5];
    accelCount.x = ACCEL_XSIGN * registerValuesAccel[ACCEL_XDIR];
    accelCount.y = ACCEL_YSIGN * registerValuesAccel[ACCEL_YDIR];
    accelCount.z = ACCEL_ZSIGN * registerValuesAccel[ACCEL_ZDIR];

    // be careful not to misinterpret 2's complement registers
    temperatureCount[0] = (int16_t)(((uint16_t)data_to_read[6]) << 8) | (uint16_t)data_to_read[7];

    int16_t registerValuesGyro[3];
    // be careful not to misinterpret 2's complement registers
    registerValuesGyro[0] = (int16_t)(((uint16_t)data_to_read[8]) << 8) | (uint16_t)data_to_read[9];  // high byte, low byte
    registerValuesGyro[1] = (int16_t)(((uint16_t)data_to_read[10]) << 8) | (uint16_t)data_to_read[11];
    registerValuesGyro[2] = (int16_t)(((uint16_t)data_to_read[12]) << 8) | (uint16_t)data_to_read[13];
    gyroCount.x = GYRO_XSIGN * registerValuesGyro[GYRO_XDIR];
    gyroCount.y = GYRO_YSIGN * registerValuesGyro[GYRO_YDIR];
    gyroCount.z = GYRO_ZSIGN * registerValuesGyro[GYRO_ZDIR];

    ready = true;

    on_success(Vector3<float>(accelCount) * aRes - accelBias, Vector3<float>(gyroCount) * gRes - gyroBias);
}

void MPU9250::reset() {
    i2c().writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    delay(100);
}

void MPU9250::setFilters(uint8_t gyrofilter, uint8_t accelfilter) {
    // register MPU9250_CONFIG (0x1A) contains gyro and temp filters:
    switch (gyrofilter) {
        case 0:
            // 250     0.97    8       4000    0.04
            i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x00);
            break;
        case 1:
            // 184     2.9     1       188     1.9
            i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);
            break;
        case 2:
            // 92      3.9     1       98      2.8
            i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);
            break;
        case 3:
            // 41      5.9     1       42      4.8
            i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);
            break;
        case 4:
            // 20      9.9     1       20      8.3
            i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x04);
            break;
        case 5:
            // 10      17.85   1       10      13.4
            i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x05);
            break;
        case 6:
            // 5       33.48   1       5       18.6
            i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x06);
            break;
        case 7:
            // 3600    0.17    8       4000    0.04
            i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x07);
            break;
        default:
            // bad value
            break;
    }
    // register ACCEL_CONFIG2 (0x1D) contains accel filter:
    switch (accelfilter) {
        case 0:
            // 460     1.94        250
            i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x00);
            break;
        case 1:
            // 184     5.80        250
            i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x01);
            break;
        case 2:
            // 92      7.80        250
            i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);
            break;
        case 3:
            // 41      11.80       250
            i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03);
            break;
        case 4:
            // 20      19.80       250
            i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x04);
            break;
        case 5:
            // 10      35.70       250
            i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x05);
            break;
        case 6:
            // 5       66.96       250
            i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x06);
            break;
        case 7:
            // 460     1.94        250
            i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x07);
            break;
        default:
            // bad value
            break;
    }
}

void MPU9250::configure() {
    // wake up device
    i2c().writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors
    // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
    delay(200);
    // get stable time source
    i2c().writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    // there's no downside to updating the registers at the internal sample rate of 1kHz (even though we read more slowly)
    i2c().writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
    // set up the FIFO, ext. SYNC and set gyro filter to 184Hz lowpass / 1kHz output rate (before SMPLRT_DIV)
    i2c().writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);
    // Set gyroscope to +/-1000dps resolutions and enable the low pass filter for gyro and temp
    i2c().writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x10);
    // Set accelerometer to +/-8g resolution
    i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x10);  // 0x00=+/-2g; Ox08=+/-4g;0x10=+/-8g,0x18=+/-16g
    // Set accelerometer to 5Hz low pass / 1kHz output rate (before SMPLRT_DIV)
    i2c().writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x06);
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    //  -- 0x22 or 0x32 depending on dataReady() mode above --
    // i2c().writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22); // clear interrupt by reading INT_STATUS
    i2c().writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x32);  // clear interrupt by any read operation
    i2c().writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);   // Enable data ready (bit 0) interrupt
}
