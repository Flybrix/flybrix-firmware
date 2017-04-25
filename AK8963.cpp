/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
    *
    *
    Credit is due to Kris Winer ("https://github.com/kriswiner/MPU-9250")

*/

#include "AK8963.h"
#include <i2c_t3.h>
#include <stdio.h>
#include <math.h>
#include "state.h"

AK8963::MagBias::MagBias() : x{0.0}, y{0.0}, z{0.0} {
}

// we have three coordinate systems here:
// 1. REGISTER coordinates: native values as read
// 2. IC/PCB coordinates: matches FLYER system if the pcb is in standard orientation
// 3. FLYER coordinates: if the pcb is mounted in a non-standard way the FLYER system is a rotation of the IC/PCB system

// map from the REGISTER coordinate system to the IC/PCB coordinate system
// this is done immediately whenever we read from the registers

#define MAG_XDIR 1  // none of these have been verified yet!
#define MAG_YDIR 0  // based on Section 9.1 in datasheet
#define MAG_ZDIR 2
#define MAG_XSIGN 1
#define MAG_YSIGN 1
#define MAG_ZSIGN -1

AK8963::AK8963(State* state, I2CManager* i2c, const RotationMatrix<float>& R) : ready{false}, state{state}, i2c{i2c}, R(R) {
}

uint8_t AK8963::getID() {
    return i2c->readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for MPU-9250 --> 0x48
}

void AK8963::restart() {
    configure();
}

uint8_t AK8963::getStatusByte() {
    return i2c->readByte(AK8963_ADDRESS, AK8963_ST1);
    // bit 0 turns to “1” when data is ready in single measurement mode, continuous measurement mode1, 2, external trigger measurement mode or self-test mode. It returns to “0” when any one of ST2
    // register or measurement data register (HXL~HZH) is read.
    // bit 1 turns to “1” when data has been skipped in continuous measurement mode or external trigger measurement mode. It returns to “0” when any one of ST2 register or measurement data register
    // (HXL~HZH) is read.
}

// writes values to state in milligauss
bool AK8963::startMeasurement() {
    ready = false;
    data_to_send[0] = AK8963_XOUT_L;
    i2c->addTransfer(AK8963_ADDRESS, 1, data_to_send, 7, data_to_read, this);
    return true;
}

namespace {
constexpr float RAW_TO_uT = 10. * 4912. / 32760.0;  // +/- 0.15 uT (or 1.5mG) per LSB; range is -32760...32760
}

void AK8963::triggerCallback() {
    uint8_t c = data_to_read[6];  // ST2 register
    if (!(c & 0x08)) {            // Check if magnetic sensor overflow set, if not then report data
        // convert from REGISTER system to IC/PCB system
        // "Measurement data is stored in two’s complement and Little Endian format."
        // be careful not to misinterpret 2's complement registers
        int16_t registerValues[3];
        registerValues[0] = (int16_t)(((uint16_t)data_to_read[1]) << 8) | (uint16_t)data_to_read[0];  // low byte, high byte
        registerValues[1] = (int16_t)(((uint16_t)data_to_read[3]) << 8) | (uint16_t)data_to_read[2];
        registerValues[2] = (int16_t)(((uint16_t)data_to_read[5]) << 8) | (uint16_t)data_to_read[4];
        magCount.x = MAG_XSIGN * registerValues[MAG_XDIR];
        magCount.y = MAG_YSIGN * registerValues[MAG_YDIR];
        magCount.z = MAG_ZSIGN * registerValues[MAG_ZDIR];
        // scale by sensitivity before rotating
        magCount = Vector3<float>(magCount) * magCalibration;
        last_read = Vector3<float>(magCount) * RAW_TO_uT - Vector3<float>(mag_bias.x, mag_bias.y, mag_bias.z);
        last_read = R * last_read;  // rotate to FLYER coords
        state->updateStateMag(last_read);
    } else {
        // ERROR: ("ERROR: Magnetometer overflow!");
    }
    ready = true;
}

void AK8963::disable() {
    i2c->writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00);  // Power down magnetometer
    delay(100);
}

void AK8963::configure() {
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];                                  // x/y/z gyro calibration data stored here
    i2c->writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00);  // Power down magnetometer
    delay(10);
    i2c->writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x0F);  // Enter Fuse ROM access mode
    delay(10);
    i2c->readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis sensitivity calibration values

    // adjustment formula is taken from the datasheet
    magCalibration = Vector3<float>(rawData[MAG_XDIR] - 128, rawData[MAG_YDIR] - 128, rawData[MAG_ZDIR] - 128) / 256 + 1;
    i2c->writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00);  // Power down magnetometer
    delay(10);
    i2c->writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x16);  // Set magnetometer to 16bit, 100Hz continuous acquisition
    delay(10);
}
