/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "config.h"
#include "version.h"

void(*config_handler)(CONFIG_struct&) {nullptr};
bool(*config_verifier)(const CONFIG_struct&) {nullptr};

CONFIG_union CONFIG;  // global CONFIG variable

#define BYPASS_THRUST_MASTER 1<<0
#define BYPASS_PITCH_MASTER  1<<1
#define BYPASS_ROLL_MASTER   1<<2
#define BYPASS_YAW_MASTER    1<<3
#define BYPASS_THRUST_SLAVE  1<<4
#define BYPASS_PITCH_SLAVE   1<<5
#define BYPASS_ROLL_SLAVE    1<<6
#define BYPASS_YAW_SLAVE     1<<7

void initializeEEPROM(void) {  // Default Settings

    CONFIG.data.version[0] = FIRMWARE_VERSION_A;
    CONFIG.data.version[1] = FIRMWARE_VERSION_B;
    CONFIG.data.version[2] = FIRMWARE_VERSION_C;

    CONFIG.data.pcbOrientation[0] = 0.0f;  // pitch; applied first
    CONFIG.data.pcbOrientation[1] = 0.0f;  // roll;  applied second
    CONFIG.data.pcbOrientation[2] = 0.0f;  // yaw;   applied last

    CONFIG.data.pcbTranslation[0] = 0.0f;  // x (mm)
    CONFIG.data.pcbTranslation[1] = 0.0f;  // y (mm)
    CONFIG.data.pcbTranslation[2] = 0.0f;  // z (mm)

    // default "x quad" configuration:
    //  * CH0 ( CW: red +, blue -, type B prop) at front right
    //  * CH1 (CCW: wht +, blk  -, type A prop) at front left
    //  * CH2 (CCW: wht +, blk  -, type A prop) at rear right
    //  * CH3 ( CW: red +, blue -, type B prop) at rear left
    //
    // pitch positive (nose up) needs a Tx negative restoring torque --> (Tx<0) should drop the nose by increasing 2 & 3
    // roll positive (RHS down) needs a Ty negative restoring torque --> (Ty<0) should raise the RHS by increasing 0 & 2
    // yaw positive (CCW rotation from top down) needs a Tz negative restoring torque --> (Tz<0) should decrease [1,2] CCW b/w motors & increase [0,3] CW r/b motors
    //
    CONFIG.data.mixTableFz[0] =  1; CONFIG.data.mixTableTx[0] =  1; CONFIG.data.mixTableTy[0] = -1; CONFIG.data.mixTableTz[0] = -1;
    CONFIG.data.mixTableFz[1] =  1; CONFIG.data.mixTableTx[1] =  1; CONFIG.data.mixTableTy[1] =  1; CONFIG.data.mixTableTz[1] =  1;
    CONFIG.data.mixTableFz[2] =  1; CONFIG.data.mixTableTx[2] = -1; CONFIG.data.mixTableTy[2] = -1; CONFIG.data.mixTableTz[2] =  1;
    CONFIG.data.mixTableFz[3] =  1; CONFIG.data.mixTableTx[3] = -1; CONFIG.data.mixTableTy[3] =  1; CONFIG.data.mixTableTz[3] = -1;
    CONFIG.data.mixTableFz[4] =  0; CONFIG.data.mixTableTx[4] =  0; CONFIG.data.mixTableTy[4] =  0; CONFIG.data.mixTableTz[4] =  0;
    CONFIG.data.mixTableFz[5] =  0; CONFIG.data.mixTableTx[5] =  0; CONFIG.data.mixTableTy[5] =  0; CONFIG.data.mixTableTz[5] =  0;
    CONFIG.data.mixTableFz[6] =  0; CONFIG.data.mixTableTx[6] =  0; CONFIG.data.mixTableTy[6] =  0; CONFIG.data.mixTableTz[6] =  0;
    CONFIG.data.mixTableFz[7] =  0; CONFIG.data.mixTableTx[7] =  0; CONFIG.data.mixTableTy[7] =  0; CONFIG.data.mixTableTz[7] =  0;

    CONFIG.data.magBias[0] = 0.0f;  // Bx (milligauss)
    CONFIG.data.magBias[1] = 0.0f;  // By (milligauss)
    CONFIG.data.magBias[2] = 0.0f;  // Bz (milligauss)

    // RX -- PKZ3341 sends: RHS left/right, RHS up/down, LHS up/down, LHS left/right, RHS click (latch), LHS button(momentary)
    CONFIG.data.assignedChannel[0] = 2;  // map throttle to LHS up/down
    CONFIG.data.assignedChannel[1] = 1;  // map pitch to RHS up/down
    CONFIG.data.assignedChannel[2] = 0;  // map roll to RHS left/righ
    CONFIG.data.assignedChannel[3] = 3;  // map yaw to LHS up/down
    CONFIG.data.assignedChannel[4] = 4;  // map AUX1 to RHS click
    CONFIG.data.assignedChannel[5] = 5;  // map AUX2 to LHS click

    CONFIG.data.commandInversion = 3; //invert pitch and roll

    CONFIG.data.channelMidpoint[0] = 1515;
    CONFIG.data.channelMidpoint[1] = 1515;
    CONFIG.data.channelMidpoint[2] = 1500;
    CONFIG.data.channelMidpoint[3] = 1520;
    CONFIG.data.channelMidpoint[4] = 1500;
    CONFIG.data.channelMidpoint[5] = 1500;

    CONFIG.data.channelDeadzone[0] = 20;
    CONFIG.data.channelDeadzone[1] = 20;
    CONFIG.data.channelDeadzone[2] = 20;
    CONFIG.data.channelDeadzone[3] = 40;
    CONFIG.data.channelDeadzone[4] = 20;
    CONFIG.data.channelDeadzone[5] = 20;

    // PID Parameters

    CONFIG.data.thrustMasterPIDParameters[0] = 1.0f;  // P
    CONFIG.data.thrustMasterPIDParameters[1] = 0.0f;  // I
    CONFIG.data.thrustMasterPIDParameters[2] = 0.0f;  // D
    CONFIG.data.thrustMasterPIDParameters[3] = 0.0f;  // Windup guard
    CONFIG.data.thrustMasterPIDParameters[4] = 0.005f;  // D filter usec (15Hz)
    CONFIG.data.thrustMasterPIDParameters[5] = 0.005f;  // setpoint filter usec (30Hz)
    CONFIG.data.thrustMasterPIDParameters[6] = 1.0f;  // (meters / full stick action)

    CONFIG.data.pitchMasterPIDParameters[0] = 5.0f;  // P
    CONFIG.data.pitchMasterPIDParameters[1] = 1.0f;  // I
    CONFIG.data.pitchMasterPIDParameters[2] = 0.0f;  // D
    CONFIG.data.pitchMasterPIDParameters[3] = 10.0f;  // Windup guard
    CONFIG.data.pitchMasterPIDParameters[4] = 0.005f;  // D filter usec (15Hz)
    CONFIG.data.pitchMasterPIDParameters[5] = 0.005f;  // setpoint filter usec (30Hz)
    CONFIG.data.pitchMasterPIDParameters[6] = 10.0f;  // (degrees / full stick action)

    CONFIG.data.rollMasterPIDParameters[0] = 5.0f;  // P
    CONFIG.data.rollMasterPIDParameters[1] = 1.0f;  // I
    CONFIG.data.rollMasterPIDParameters[2] = 0.0f;  // D
    CONFIG.data.rollMasterPIDParameters[3] = 10.0f;  // Windup guard
    CONFIG.data.rollMasterPIDParameters[4] = 0.005f;  // D filter usec (15Hz)
    CONFIG.data.rollMasterPIDParameters[5] = 0.005f;  // setpoint filter usec (30Hz)
    CONFIG.data.rollMasterPIDParameters[6] = 10.0f;  // (degrees / full stick action)

    CONFIG.data.yawMasterPIDParameters[0] = 5.0f;  // P
    CONFIG.data.yawMasterPIDParameters[1] = 1.0f;  // I
    CONFIG.data.yawMasterPIDParameters[2] = 0.0f;  // D
    CONFIG.data.yawMasterPIDParameters[3] = 10.0f;  // Windup guard
    CONFIG.data.yawMasterPIDParameters[4] = 0.005f;  // D filter usec (15Hz)
    CONFIG.data.yawMasterPIDParameters[5] = 0.005f;  // setpoint filter usec (30Hz)
    CONFIG.data.yawMasterPIDParameters[6] = 180.0f;  // (degrees / full stick action)

    CONFIG.data.thrustSlavePIDParameters[0] = 1.0f;  // P
    CONFIG.data.thrustSlavePIDParameters[1] = 0.0f;  // I
    CONFIG.data.thrustSlavePIDParameters[2] = 0.0f;  // D
    CONFIG.data.thrustSlavePIDParameters[3] = 10.0f;  // Windup guard
    CONFIG.data.thrustSlavePIDParameters[4] = 0.001f;  // D filter usec (150Hz)
    CONFIG.data.thrustSlavePIDParameters[5] = 0.001f;  // setpoint filter usec (300Hz)
    CONFIG.data.thrustSlavePIDParameters[6] = 0.3f;  // (meters/sec / full stick action)

    CONFIG.data.pitchSlavePIDParameters[0] = 20.0f;  // P
    CONFIG.data.pitchSlavePIDParameters[1] = 8.0f;  // I
    CONFIG.data.pitchSlavePIDParameters[2] = 0.0f;  // D
    CONFIG.data.pitchSlavePIDParameters[3] = 30.0f;  // Windup guard
    CONFIG.data.pitchSlavePIDParameters[4] = 0.001f;  // D filter usec (150Hz)
    CONFIG.data.pitchSlavePIDParameters[5] = 0.001f;  // setpoint filter usec (300Hz)
    CONFIG.data.pitchSlavePIDParameters[6] = 30.0f;  // (deg/sec / full stick action)

    CONFIG.data.rollSlavePIDParameters[0] = 20.0f;  // P
    CONFIG.data.rollSlavePIDParameters[1] = 8.0f;  // I
    CONFIG.data.rollSlavePIDParameters[2] = 0.0f;  // D
    CONFIG.data.rollSlavePIDParameters[3] = 30.0f;  // Windup guard
    CONFIG.data.rollSlavePIDParameters[4] = 0.001f;  // D filter usec (150Hz)
    CONFIG.data.rollSlavePIDParameters[5] = 0.001f;  // setpoint filter usec (300Hz)
    CONFIG.data.rollSlavePIDParameters[6] = 30.0f;  // (deg/sec / full stick action)

    CONFIG.data.yawSlavePIDParameters[0] = 30.0f;  // P
    CONFIG.data.yawSlavePIDParameters[1] = 5.0f;  // I
    CONFIG.data.yawSlavePIDParameters[2] = 0.0f;  // D
    CONFIG.data.yawSlavePIDParameters[3] = 20.0f;  // Windup guard
    CONFIG.data.yawSlavePIDParameters[4] = 0.001f;  // D filter usec (150Hz)
    CONFIG.data.yawSlavePIDParameters[5] = 0.001f;  // setpoint filter usec (300Hz)
    CONFIG.data.yawSlavePIDParameters[6] = 240.0f;  // (deg/sec / full stick action)

    CONFIG.data.pidBypass = BYPASS_THRUST_MASTER | BYPASS_THRUST_SLAVE | BYPASS_YAW_MASTER; //AHRS/Horizon mode

    CONFIG.data.stateEstimationParameters[0] = 1.00f;  // 2*kp or BETA
    CONFIG.data.stateEstimationParameters[1] = 0.01f;  // 2*ki

    CONFIG.data.enableParameters[0] = 0.001f;  // max variance
    CONFIG.data.enableParameters[1] = 30.0f;  // max angle

    // This function will only initialize data variables
    // writeEEPROM() needs to be called manually to store this data in EEPROM
}

void writeEEPROM(void) {
    for (uint16_t i = 0; i < sizeof(struct CONFIG_struct); i++) {
        if (CONFIG.raw[i] != EEPROM.read(i)) {  // Only re-write new data
            EEPROM.write(i, CONFIG.raw[i]);
        }
    }
    if (config_handler)
        config_handler(CONFIG.data);
}

bool isEmptyEEPROM() {
    return EEPROM.read(0) == 255;
}

void readEEPROM(void) {
    if (isEmptyEEPROM()) {
        // No EEPROM values detected, re-initialize to default values
        initializeEEPROM();
        writeEEPROM();  // store the default values
    } else {
        // There "is" data in the EEPROM, read it all
        for (uint16_t i = 0; i < sizeof(struct CONFIG_struct); i++) {
            CONFIG.raw[i] = EEPROM.read(i);
        }
        // Verify version
        if ((CONFIG.data.version[0] != FIRMWARE_VERSION_A) ||
            (CONFIG.data.version[1] != FIRMWARE_VERSION_B) ||
            (CONFIG.data.version[2] != FIRMWARE_VERSION_C) ){
            // Version doesn't match, re-initialize to default values
            initializeEEPROM();
            writeEEPROM();  // store the default values
        } else {
        }
    }
    if (config_handler)
        config_handler(CONFIG.data);
}
