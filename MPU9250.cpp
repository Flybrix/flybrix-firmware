/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
    *
    *
    Credit is due to Kris Winer ("https://github.com/kriswiner/MPU-9250")

*/

#include "MPU9250.h"
#include <i2c_t3.h>
#include <stdio.h>
#include <math.h>
#include "state.h"

// we have three coordinate systems here:
// 1. REGISTER coordinates: native values as read
// 2. IC/PCB coordinates: matches FLYER system if the pcb is in standard orientation
// 3. FLYER coordinates: if the pcb is mounted in a non-standard way the FLYER system is a rotation of the IC/PCB system

// map from the REGISTER coordinate system to the IC/PCB coordinate system
// this is done immediately whenever we read from the registers

#define ACCEL_XDIR 0
#define ACCEL_YDIR 1
#define ACCEL_ZDIR 2
#define ACCEL_XSIGN -1
#define ACCEL_YSIGN -1
#define ACCEL_ZSIGN -1  // verified by experiment (units definition issue?)

#define GYRO_XDIR 0
#define GYRO_YDIR 1
#define GYRO_ZDIR 2
#define GYRO_XSIGN 1
#define GYRO_YSIGN 1
#define GYRO_ZSIGN 1  // verified by experiment

MPU9250::MPU9250(State *__state, I2CManager *__i2c) {
    state = __state;
    i2c = __i2c;
    ready = false;
    pinMode(MPU_INTERRUPT, INPUT);
}

uint8_t MPU9250::getID() {
    return i2c->readByte(MPU9250_ADDRESS, WHO_AM_I);  // Read WHO_AM_I register for MPU-9250 --> 0x71
}

void MPU9250::restart() {
    reset();
    configure();
    forgetBiasValues();
}

bool MPU9250::dataReadyInterrupt() {
    return digitalRead(MPU_INTERRUPT);  // use the interrupt pin -- be sure to set INT_PIN_CFG
}

uint8_t MPU9250::getStatusByte() {
    return i2c->readByte(MPU9250_ADDRESS, INT_STATUS);
}

void MPU9250::correctBiasValues()  // all in FLYER system
{
    float ax, ay, az; /*normalized accel_filter values */
    float recipNorm = 0.0f;
    for (uint8_t i = 0; i < 3; i++) {
        recipNorm += state->accel_filter[i] * state->accel_filter[i];
    }
    recipNorm = invSqrt(recipNorm);
    ax = state->accel_filter[0] * recipNorm;
    ay = state->accel_filter[1] * recipNorm;
    az = state->accel_filter[2] * recipNorm;

    //
    // Generation of Rotation Matrix from quaternion between [ax,ay,az] and [0,0,-1]
    // http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    // u = <0,0-1> v = <ax, ay, az>
    // norm_u_norm_v = 1;

    //  float real_part = norm_u_norm_v + dot(u, v);
    float qw, qx, qy;  //, qz;
    float r = 1.0f - az;

    if (r < 1.e-6f) {
        /* If u and v are exactly opposite, rotate 180 degrees
         * around an arbitrary orthogonal axis. Axis normalisation
         * can happen later, when we normalise the quaternion. */
        /*
            r = 0.0f;
            w = abs(u.x) > abs(u.z) ? vec3(-u.y, u.x, 0.f)
                                    : vec3(0.f, -u.z, u.y);
            //0 > 1 is false so we use the false output
        */
        qw = 0;
        qx = 0;
        qy = 1;
        // qz = 0;
    } else {
        /* Otherwise, build quaternion the standard way. */
        // w = cross(u, v);
        // A x B = (a2b3 - a3b2, a3b1 - a1b3, a1b2 - a2b1);
        // a1 and a2 = 0, a3 = -1, b1 = ax, b2=ay, b3 = az
        //(ay, -ax, 0);
        qw = r;
        qx = ay;
        qy = -ax;
        // qz = 0;

        recipNorm = invSqrt(qw * qw + qx * qx + qy * qy);
        qw *= recipNorm;
        qx *= recipNorm;
        qy *= recipNorm;
    }

    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
    // note: qz = 0.
    // we also want this to be the inverse/transpose to take us from ax ay az back to 0 0 -1

    state->R[0][0] = 1 - 2 * qy * qy;
    state->R[1][0] = 2 * qx * qy;
    state->R[2][0] = 2 * qy * qw;

    state->R[0][1] = 2 * qx * qy;
    state->R[1][1] = 1 - 2 * qx * qx;
    state->R[2][1] = -2 * qx * qw;

    state->R[0][2] = -2 * qy * qw;
    state->R[1][2] = 2 * qx * qw;
    state->R[2][2] = 1 - 2 * qx * qx - 2 * qy * qy;

    // the bias correction in the IC/PCB coordinates
    accelBias[0] = state->accel_filter[0] - ax;
    accelBias[1] = state->accel_filter[1] - ay;
    accelBias[2] = state->accel_filter[2] - az;

    // biases are additive corrections -- we were not rotating during calibration so we measured gyro drift
    for (uint8_t i = 0; i < 3; i++) {  // construct biases for later manual subtraction
        gyroBias[i] = (float)state->gyro_filter[i];
    }
}

void MPU9250::forgetBiasValues() {
    for (uint8_t i = 0; i < 3; i++) {
        gyroBias[i] = 0.0f;
        accelBias[i] = 0.0f;
    }
    // set R to identity
    state->R[0][0] = 1.0f;
    state->R[0][1] = 0.0f;
    state->R[0][2] = 0.0f;

    state->R[1][0] = 0.0f;
    state->R[1][1] = 1.0f;
    state->R[1][2] = 0.0f;

    state->R[2][0] = 0.0f;
    state->R[2][1] = 0.0f;
    state->R[2][2] = 1.0f;
}

// writes values to state in g's and in degrees per second
bool MPU9250::startMeasurement() {
    if (dataReadyInterrupt()) {
        ready = false;
        data_to_send[0] = ACCEL_XOUT_H;
        i2c->addTransfer((uint8_t)MPU9250_ADDRESS, (uint8_t)1, &data_to_send[0], (uint8_t)14, &data_to_read[0], this);
        return true;
    }
    return false;
}

void MPU9250::processCallback(uint8_t count, uint8_t *rawData) {
    // count should always be 14 if we wanted to check...

    // convert from REGISTER system to IC/PCB system
    int16_t registerValuesAccel[3];
    // be careful not to misinterpret 2's complement registers
    registerValuesAccel[0] = (int16_t)(((uint16_t)rawData[0]) << 8) | (uint16_t)rawData[1];  // high byte, low byte
    registerValuesAccel[1] = (int16_t)(((uint16_t)rawData[2]) << 8) | (uint16_t)rawData[3];
    registerValuesAccel[2] = (int16_t)(((uint16_t)rawData[4]) << 8) | (uint16_t)rawData[5];
    accelCount[0] = ACCEL_XSIGN * registerValuesAccel[ACCEL_XDIR];
    accelCount[1] = ACCEL_YSIGN * registerValuesAccel[ACCEL_YDIR];
    accelCount[2] = ACCEL_ZSIGN * registerValuesAccel[ACCEL_ZDIR];

    // be careful not to misinterpret 2's complement registers
    temperatureCount[0] = (int16_t)(((uint16_t)rawData[6]) << 8) | (uint16_t)rawData[7];

    int16_t registerValuesGyro[3];
    // be careful not to misinterpret 2's complement registers
    registerValuesGyro[0] = (int16_t)(((uint16_t)rawData[8]) << 8) | (uint16_t)rawData[9];  // high byte, low byte
    registerValuesGyro[1] = (int16_t)(((uint16_t)rawData[10]) << 8) | (uint16_t)rawData[11];
    registerValuesGyro[2] = (int16_t)(((uint16_t)rawData[12]) << 8) | (uint16_t)rawData[13];
    gyroCount[0] = GYRO_XSIGN * registerValuesGyro[GYRO_XDIR];
    gyroCount[1] = GYRO_YSIGN * registerValuesGyro[GYRO_YDIR];
    gyroCount[2] = GYRO_ZSIGN * registerValuesGyro[GYRO_ZDIR];

    state->accel[0] = (float)accelCount[0] * aRes - accelBias[0];
    state->accel[1] = (float)accelCount[1] * aRes - accelBias[1];
    state->accel[2] = (float)accelCount[2] * aRes - accelBias[2];
    rotate(state->R, state->accel);  // rotate to FLYER coords

    state->gyro[0] = (float)gyroCount[0] * gRes - gyroBias[0];
    state->gyro[1] = (float)gyroCount[1] * gRes - gyroBias[1];
    state->gyro[2] = (float)gyroCount[2] * gRes - gyroBias[2];
    rotate(state->R, state->gyro);  // rotate to FLYER coords

    ready = true;
}

void MPU9250::reset() {
    i2c->writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    delay(100);
}

void MPU9250::setFilters(uint8_t gyrofilter, uint8_t accelfilter) {
    // register MPU9250_CONFIG (0x1A) contains gyro and temp filters:
    switch (gyrofilter) {
        case 0:
            // 250     0.97    8       4000    0.04
            i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x00);
            break;
        case 1:
            // 184     2.9     1       188     1.9
            i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);
            break;
        case 2:
            // 92      3.9     1       98      2.8
            i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);
            break;
        case 3:
            // 41      5.9     1       42      4.8
            i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);
            break;
        case 4:
            // 20      9.9     1       20      8.3
            i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x04);
            break;
        case 5:
            // 10      17.85   1       10      13.4
            i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x05);
            break;
        case 6:
            // 5       33.48   1       5       18.6
            i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x06);
            break;
        case 7:
            // 3600    0.17    8       4000    0.04
            i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x07);
            break;
        default:
            // bad value
            break;
    }
    // register ACCEL_CONFIG2 (0x1D) contains accel filter:
    switch (accelfilter) {
        case 0:
            // 460     1.94        250
            i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x00);
            break;
        case 1:
            // 184     5.80        250
            i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x01);
            break;
        case 2:
            // 92      7.80        250
            i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);
            break;
        case 3:
            // 41      11.80       250
            i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03);
            break;
        case 4:
            // 20      19.80       250
            i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x04);
            break;
        case 5:
            // 10      35.70       250
            i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x05);
            break;
        case 6:
            // 5       66.96       250
            i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x06);
            break;
        case 7:
            // 460     1.94        250
            i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x07);
            break;
        default:
            // bad value
            break;
    }
}

void MPU9250::configure() {
    // wake up device
    i2c->writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors
    // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
    delay(200);
    // get stable time source
    i2c->writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    // there's no downside to updating the registers at the internal sample rate of 1kHz (even though we read more slowly)
    i2c->writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
    // set up the FIFO, ext. SYNC and set gyro filter to 184Hz lowpass / 1kHz output rate (before SMPLRT_DIV)
    i2c->writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);
    // Set gyroscope to +/-1000dps resolutions and enable the low pass filter for gyro and temp
    i2c->writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x10);
    // Set accelerometer to +/-8g resolution
    i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x10);  // 0x00=+/-2g; Ox08=+/-4g;0x10=+/-8g,0x18=+/-16g
    // Set accelerometer to 5Hz low pass / 1kHz output rate (before SMPLRT_DIV)
    i2c->writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x06);
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    //  -- 0x22 or 0x32 depending on dataReady() mode above --
    // i2c->writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22); // clear interrupt by reading INT_STATUS
    i2c->writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x32);  // clear interrupt by any read operation
    i2c->writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);   // Enable data ready (bit 0) interrupt
}

void MPU9250::rotate(float R[3][3], float x[3]) {
    /* R is [3][3] - [row][col], x is [3] */
    float y[3] = {0.0, 0.0, 0.0};
    float sum = 0.0;
    for (uint8_t i = 0; i < 3; i++) {
        sum = 0.0;
        for (uint8_t j = 0; j < 3; j++) {
            sum += R[i][j] * x[j];
        }
        y[i] = sum;
    }
    for (uint8_t i = 0; i < 3; i++) {
        x[i] = y[i];
    }
}

float MPU9250::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
