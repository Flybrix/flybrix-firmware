/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <AK8963.h/cpp>

    Driver code for the AK8963 magnetometer (inside the MPU9250).

*/

#ifndef AK8963_h
#define AK8963_h

#include "Arduino.h"
#include "i2cManager.h"

class State;

// we have three coordinate systems here:
// 1. REGISTER coordinates: native values as read
// 2. IC/PCB coordinates: matches FLYER system if the pcb is in standard orientation
// 3. FLYER coordinates: if the pcb is mounted in a non-standard way the FLYER system is a rotation of the IC/PCB system

class AK8963 : public CallbackProcessor {
   public:  // all in FLYER system
    AK8963(State *state, I2CManager *i2c);

    void restart();  // calculate bias and prepare for flight

    bool ready;

    bool startMeasurement();  // writes values to state (when data is ready)
    void triggerCallback();   // handles return for getAccelGryo()

    uint8_t getID();

    struct __attribute__((packed)) MagBias {
        MagBias();
        bool verify() const {
            return true;
        }
        float x;  // Bx (milligauss)
        float y;  // By (milligauss)
        float z;  // Bz (milligauss)
    } mag_bias;

    static_assert(sizeof(MagBias) == 3 * 4, "Data is not packed");

   private:
    State *state;
    I2CManager *i2c;

    uint8_t getStatusByte();

    void rotate(float R[3][3], float x[3]);

    void reset();
    void configure();
    void disable();

    const float mRes = 10. * 4912. / 32760.0;  // +/- 0.15 uT (or 1.5mG) per LSB; range is -32760...32760

    // 16-bit raw values, bias correction, factory calibration
    int16_t magCount[3] = {0, 0, 0};
    // bias is stored in MagBias
    float magCalibration[3] = {0.0, 0.0, 0.0};

    // buffers for processCallback
    uint8_t data_to_read[7];
    uint8_t data_to_send[1];

};  // class AK8963

#define DEG2RAD 0.01745329251f

//*************************************************************
//
// Magnetometer Registers (See Table 3 Register Map on page 48)
//
//*************************************************************

#define AK8963_ADDRESS 0x0C
/*  Page 24: "Pass-Through mode is also used to access the AK8963 magnetometer
    directly from the host. In this configuration the slave address for the AK8963
is 0X0C or 12 decimal. */

#define WHO_AM_I_AK8963 0x00  // should return 0x48
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02     // data ready status bit 0
#define AK8963_XOUT_L 0x03  // data
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09    // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL1 0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2 0x0B
#define AK8963_ASTC 0x0C    // Self test control
#define AK8963_TS1 0x0D     // Self test control
#define AK8963_TS2 0x0E     // Self test control
#define AK8963_I2CDIS 0x0F  // I2C disable
#define AK8963_ASAX 0x10    // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY 0x11    // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ 0x12    // Fuse ROM z-axis sensitivity adjustment value

#endif
