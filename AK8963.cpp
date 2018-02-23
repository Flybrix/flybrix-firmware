/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com

    Credit is due to Kris Winer ("https://github.com/kriswiner/MPU-9250")
*/

#include "AK8963.h"
#include <i2c_t3.h>
#include "i2cManager.h"
#include "debug.h"

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

uint8_t AK8963::getID() {
    return i2c().readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for MPU-9250 --> 0x48
}

void AK8963::restart() {
    configure();
}

uint8_t AK8963::getStatusByte() {
    return i2c().readByte(AK8963_ADDRESS, AK8963_ST1);
    // bit 0 turns to “1” when data is ready in single measurement mode, continuous measurement mode1, 2, external trigger measurement mode or self-test mode. It returns to “0” when any one of ST2
    // register or measurement data register (HXL~HZH) is read.
    // bit 1 turns to “1” when data has been skipped in continuous measurement mode or external trigger measurement mode. It returns to “0” when any one of ST2 register or measurement data register
    // (HXL~HZH) is read.
}

// writes values to state in milligauss
bool AK8963::startMeasurement(std::function<void(Vector3<float>)> on_success) {
    ready = false;
    data_to_send[0] = AK8963_XOUT_L;
    i2c().addTransfer(AK8963_ADDRESS, 1, data_to_send, 7, data_to_read, [this, on_success]() { triggerCallback(on_success); });
    return true;
}

void AK8963::setCalibrating(bool calibrating) {
    if (calibrating && !calibrating_) {
        min_reading_ = {1e30, 1e30, 1e30};
        max_reading_ = {-1e30, -1e30, -1e30};
    }
    calibrating_ = calibrating;
}

namespace {
constexpr float RAW_TO_uT = 10. * 4912. / 32760.0;  // +/- 0.15 uT (or 1.5mG) per LSB; range is -32760...32760
}

void AK8963::calibrate(const Vector3<float>& measurement) {
    if (!calibrating_) {
        return;
    }

    if (measurement.x < min_reading_.x ) {
        min_reading_.x = measurement.x;
    }
    if (measurement.y < min_reading_.y) {
        min_reading_.y = measurement.y;
    }
    if (measurement.z < min_reading_.z) {
        min_reading_.z = measurement.z;
    }
    if (measurement.x > max_reading_.x) {
        max_reading_.x = measurement.x;
    }
    if (measurement.y > max_reading_.y) {
        max_reading_.y = measurement.y;
    }
    if (measurement.z > max_reading_.z) {
        max_reading_.z = measurement.z;
    }

    mag_bias.offset = (min_reading_ + max_reading_) / 2.0f;
}

bool hideAllZeroesWarning{false};

void AK8963::triggerCallback(std::function<void(Vector3<float>)> on_success) {
    uint8_t c = data_to_read[6];  // ST2 register
    if (!(c & 0x08)) {            // Check if magnetic sensor overflow set, if not then report data
        // convert from REGISTER system to IC/PCB system
        // "Measurement data is stored in two’s complement and Little Endian format."
        // be careful not to misinterpret 2's complement registers
        int16_t registerValues[3];
        registerValues[0] = (int16_t)(((uint16_t)data_to_read[1]) << 8) | (uint16_t)data_to_read[0];  // low byte, high byte
        registerValues[1] = (int16_t)(((uint16_t)data_to_read[3]) << 8) | (uint16_t)data_to_read[2];
        registerValues[2] = (int16_t)(((uint16_t)data_to_read[5]) << 8) | (uint16_t)data_to_read[4];

        // 16-bit raw values
        Vector3<int16_t> magCount{
            MAG_XSIGN * registerValues[MAG_XDIR],  // X
            MAG_YSIGN * registerValues[MAG_YDIR],  // Y
            MAG_ZSIGN * registerValues[MAG_ZDIR]   // Z
        };
        
        if ( registerValues[MAG_XDIR] == 0 && registerValues[MAG_YDIR] == 0 && registerValues[MAG_ZDIR] == 0){
            if (!hideAllZeroesWarning) {
                DebugPrintf("ERROR: Magnetometer reading all zeroes!"); // sometimes the magnetometer returns all zeroes and we don't know why yet...
                hideAllZeroesWarning = true;
            }
            allZeroes = true;
        }
        else {
            allZeroes = false;
        }
        
        // scale by sensitivity before rotating
        magCount = Vector3<float>(magCount) * magCalibration;
        Vector3<float> measurement = Vector3<float>(magCount) * RAW_TO_uT;
        calibrate(measurement);
        measurement -= mag_bias.offset;
        on_success(measurement);
    } else {
        DebugPrintf("ERROR: Magnetometer overflow!");
    }
    ready = true;
}

void AK8963::disable() {
    i2c().writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00);  // Power down magnetometer
    delay(100);
}

void AK8963::configure() {
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];                                   // x/y/z gyro calibration data stored here
    i2c().writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00);  // Power down magnetometer
    delay(10);
    i2c().writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x0F);  // Enter Fuse ROM access mode
    delay(10);
    i2c().readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis sensitivity calibration values

    // adjustment formula is taken from the datasheet
    magCalibration = Vector3<float>(rawData[MAG_XDIR] - 128, rawData[MAG_YDIR] - 128, rawData[MAG_ZDIR] - 128) / 256 + 1;
    i2c().writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00);  // Power down magnetometer
    delay(10);
    i2c().writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x16);  // Set magnetometer to 16bit, 100Hz continuous acquisition
    delay(10);
}
