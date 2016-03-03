/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <config.h/cpp>

    EEPROM based configuration data storage structure

    Nonvolatile parameters are being stored inside an CONFIG structure
    that can be accesed as data union, for easier manipulation as a javascript
    ArrayBuffer object over serial.

*/

#ifndef config_h
#define config_h

#include <Arduino.h>
#include <EEPROM.h>

struct __attribute__((packed)) CONFIG_struct {
    uint8_t version[3];

    // for future use
    float pcbOrientation[3];  // pitch/roll/yaw in standard flyer coordinate system --> applied in that order!
    float pcbTranslation[3];  // translation in standard flyer coordinate system

    // control to motor map
    int8_t mixTableFz[8];
    int8_t mixTableTx[8];
    int8_t mixTableTy[8];
    int8_t mixTableTz[8];

    // magnetometer bias values
    float magBias[3];

    // RX channel mapping
    uint8_t assignedChannel[6];
    uint8_t commandInversion; //bitfield order is {pitch_command, roll_command, yaw_command, x, x, x, x, x} (LSB-->MSB)

    // tunable control parameters
    float thrustMasterPIDParameters[7]; //parameters are {P,I,D,integral windup guard, D filter delay sec, setpoint filter delay sec, command scaling factor}
    float pitchMasterPIDParameters[7];
    float rollMasterPIDParameters[7];
    float yawMasterPIDParameters[7];

    float thrustSlavePIDParameters[7];
    float pitchSlavePIDParameters[7];
    float rollSlavePIDParameters[7];
    float yawSlavePIDParameters[7];

    uint8_t pidBypass; //bitfield order for bypass: {thrustMaster, pitchMaster, rollMaster, yawMaster, thrustSlave, pitchSlave, rollSlave, yawSlave} (LSB-->MSB)

    // state estimation parameters for tuning
    float stateEstimationParameters[2];  // Madwick 2Kp, 2Ki

    // limits for enabling motors
    float enableParameters[2];  // variance and gravity angle
};

union CONFIG_union {
    struct CONFIG_struct data;
    uint8_t raw[sizeof(struct CONFIG_struct)];
};

#define EEPROM_LOG_START 500
#define EEPROM_LOG_END 2048

extern void(*config_handler)(CONFIG_struct&);

extern CONFIG_union CONFIG;
extern void initializeEEPROM(void);
extern void writeEEPROM(void);
extern void readEEPROM(void);

#endif
