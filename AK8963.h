/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef AK8963_h
#define AK8963_h

#include <functional>
#include <Arduino.h>
#include "utility/vector3.h"

// we have three coordinate systems here:
// 1. REGISTER coordinates: native values as read
// 2. IC/PCB coordinates: matches FLYER system if the pcb is in standard orientation
// 3. FLYER coordinates: if the pcb is mounted in a non-standard way the FLYER system is a rotation of the IC/PCB system

class AK8963 {
   public:
    void restart();  // calculate bias and prepare for flight

    bool ready{false};

    // Callback: magnet field strength in milligauss -- (x,y,z)
    bool startMeasurement(std::function<void(Vector3<float>)> on_success);

    void setCalibrating(bool calibrating);

    uint8_t getID();

    struct __attribute__((packed)) MagBias {
        bool verify() const {
            return true;
        }
        Vector3<float> offset;  // B (milligauss)
    } mag_bias;

    static_assert(sizeof(MagBias) == 3 * 4, "Data is not packed");

   private:
    void triggerCallback(std::function<void(Vector3<float>)> on_success);

    uint8_t getStatusByte();

    void calibrate(const Vector3<float>& measurement);

    void reset();
    void configure();
    void disable();

    // bias is stored in MagBias
    Vector3<float> magCalibration{0.0, 0.0, 0.0};
    bool calibrating_{false};
    Vector3<float> min_reading_;
    Vector3<float> max_reading_;

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
