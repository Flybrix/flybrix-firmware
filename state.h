/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <state.h/cpp>

    State tracks everything we know about the UAV, computes derived state variables, and handles data logging

*/

#ifndef state_h
#define state_h

#include <cstdint>
#include "localization.h"

struct Systems;

class State {
   public:
    explicit State(Systems* sys);

    // timing
    uint32_t loopCount = 0;

    // Status bitfield / LED
    uint16_t status = 0;
    void set(const uint16_t status_bit);
    void clear(const uint16_t status_bit);
    bool is(const uint16_t status_bit) const;

    // Power
    uint16_t V0_raw = 0, I0_raw = 0, I1_raw = 0;  // raw ADC levels

    // MPU9250 and AK8963
    float R[3][3];                                                                  // rotation matrix from pcb to flyer frame
    float accel[3] = {0.0, 0.0, 0.0};                                               // g's        -- (x,y,z)
    float gyro[3] = {0.0, 0.0, 0.0};                                                // deg/sec    -- (x,y,z)
    float mag[3] = {0.0, 0.0, 0.0};                                                 // milligauss -- (x,y,z)
    float accel_filter[3] = {0.0, 0.0, 0.0}, accel_filter_sq[3] = {0.0, 0.0, 0.0};  // for stability variance calculation
    float gyro_filter[3] = {0.0, 0.0, 0.0};                                         // for gyro drift correction

    // Command
    int16_t command_invalid_count = 0;
    uint8_t command_source_mask = 0;  // bitfield order is {x, R415X, BTLE, x, x, x, x, x} (LSB-->MSB)
    uint8_t command_AUX_mask = 0;     // bitfield order is {AUX1_low, AUX1_mid, AUX1_high, AUX2_low, AUX2_mid, AUX2_high, x, x} (LSB-->MSB)
    int16_t command_throttle = 0, command_pitch = 0, command_roll = 0, command_yaw = 0;

    // Control
    float Fz = 0, Tx = 0, Ty = 0, Tz = 0;  // control vectors (forces/torques)

    // Kinematics
    float kinematicsAngle[3] = {0.0f, 0.0f, 0.0f};  // radians -- pitch/roll/yaw (x,y,z)
    float kinematicsRate[3] = {0.0f, 0.0f, 0.0f};   // radians/sec -- pitch/roll/yaw (x,y,z) rates
    float kinematicsAltitude = 0.0f;                // meters

    // Motors
    void processMotorEnablingIteration();  // must be called ~80 times to enable motors.
    void disableMotors();
    uint16_t enableAttempts = 0;  // increment when we're in the STATUS_ENABLING state

    void resetState();
    void updateStateIMU(uint32_t currentTime);
    void updateStatePT(uint32_t currentTime);
    void updateStateMag();
    bool motorsEnabled();

    // const float* q; //quaternion storage for logging
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    struct __attribute__((packed)) Parameters {
        Parameters();
        bool verify() const {
            return true;
        }
        // state estimation parameters for tuning
        float state_estimation[2];  // Madwick 2Kp, 2Ki, Mahony Beta

        // limits for enabling motors
        float enable[2];  // variance and gravity angle
    } parameters;

    static_assert(sizeof(Parameters) == 2 * 2 * 4, "Data is not packed");

   private:
    bool stable();
    bool upright();
    float fast_cosine(float x_deg);

    float mixRadians(float w1, float a1, float a2);
    uint32_t lastUpdateMicros = 0;  // 1.2 hrs should be enough

    Localization localization;
    Systems* sys_;
};  // end of class State

#define DEG2RAD 0.01745329251f

// Controller ready codes
#define COMMAND_READY_R415X 0x01
#define COMMAND_READY_BTLE 0x02

// STATUS BITS
#define STATUS_BOOT 0x0001
#define STATUS_MPU_FAIL 0x0002
#define STATUS_BMP_FAIL 0x0004
#define STATUS_RX_FAIL 0x0008

#define STATUS_IDLE 0x0010

#define STATUS_ENABLING 0x0020
#define STATUS_CLEAR_MPU_BIAS 0x0040
#define STATUS_SET_MPU_BIAS 0x0080

#define STATUS_FAIL_STABILITY 0x0100
#define STATUS_FAIL_ANGLE 0x0200
#define STATUS_FAIL_OTHER 0x4000  // flag other arming failures

#define STATUS_ENABLED 0x0400

#define STATUS_BATTERY_LOW 0x0800
#define STATUS_TEMP_WARNING 0x1000
#define STATUS_LOG_FULL 0x2000

#define STATUS_OVERRIDE 0x8000

#endif
