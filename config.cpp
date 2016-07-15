/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "config.h"

#include "systems.h"

#define BYPASS_THRUST_MASTER 1 << 0
#define BYPASS_PITCH_MASTER 1 << 1
#define BYPASS_ROLL_MASTER 1 << 2
#define BYPASS_YAW_MASTER 1 << 3
#define BYPASS_THRUST_SLAVE 1 << 4
#define BYPASS_PITCH_SLAVE 1 << 5
#define BYPASS_ROLL_SLAVE 1 << 6
#define BYPASS_YAW_SLAVE 1 << 7

CONFIG_struct::CONFIG_struct() {  // Default Settings
    pcb.orientation[0] = 0.0f;    // pitch; applied first
    pcb.orientation[1] = 0.0f;    // roll;  applied second
    pcb.orientation[2] = 0.0f;    // yaw;   applied last

    pcb.translation[0] = 0.0f;  // x (mm)
    pcb.translation[1] = 0.0f;  // y (mm)
    pcb.translation[2] = 0.0f;  // z (mm)

    // default configuration is the flat8 octocopter:
    //  * CH0 ( CW: red +, blue -, type A prop) at full front right
    //  * CH2 (CCW: wht +, blk  -, type B prop) at mid front right
    //  * CH4 ( CW: red +, blue -, type A prop) at mid rear right
    //  * CH6 (CCW: wht +, blk  -, type B prop) at full rear right
    //  * CH1 (CCW: wht +, blk  -, type B prop) at full front left
    //  * CH3 ( CW: red +, blue -, type A prop) at mid front left
    //  * CH5 (CCW: wht +, blk  -, type B prop) at mid rear left
    //  * CH7 ( CW: red +, blue -, type A prop) at rull rear left
    // Note that the same mixtable can be used to build a quad on CH0, CH6, CH1, CH7
    //
    // pitch positive (nose up) needs a Tx negative restoring torque --> (Tx<0) should drop the nose by increasing rear channels and decreasing front channels
    // roll positive (right side down) needs a Ty negative restoring torque --> (Ty<0) should raise the right side by increasing right channels and decreasing left channels
    // yaw positive (CCW rotation from top down) needs a Tz negative restoring torque --> (Tz<0) should decrease CCW motors & increase CW motors
    //
    mix_table.fz[0] = 1;
    mix_table.tx[0] = 1;
    mix_table.ty[0] = -1;
    mix_table.tz[0] = 1;
    mix_table.fz[1] = 1;
    mix_table.tx[1] = 1;
    mix_table.ty[1] = 1;
    mix_table.tz[1] = -1;
    mix_table.fz[2] = 1;
    mix_table.tx[2] = 1;
    mix_table.ty[2] = -1;
    mix_table.tz[2] = -1;
    mix_table.fz[3] = 1;
    mix_table.tx[3] = 1;
    mix_table.ty[3] = 1;
    mix_table.tz[3] = 1;
    mix_table.fz[4] = 1;
    mix_table.tx[4] = -1;
    mix_table.ty[4] = -1;
    mix_table.tz[4] = 1;
    mix_table.fz[5] = 1;
    mix_table.tx[5] = -1;
    mix_table.ty[5] = 1;
    mix_table.tz[5] = -1;
    mix_table.fz[6] = 1;
    mix_table.tx[6] = -1;
    mix_table.ty[6] = -1;
    mix_table.tz[6] = -1;
    mix_table.fz[7] = 1;
    mix_table.tx[7] = -1;
    mix_table.ty[7] = 1;
    mix_table.tz[7] = 1;

    mag_bias.x = 0.0f;  // Bx (milligauss)
    mag_bias.y = 0.0f;  // By (milligauss)
    mag_bias.z = 0.0f;  // Bz (milligauss)

    // RX -- PKZ3341 sends: RHS left/right, RHS up/down, LHS up/down, LHS left/right, RHS click (latch), LHS button(momentary)
    channel.assignment[0] = 2;  // map throttle to LHS up/down
    channel.assignment[1] = 1;  // map pitch to RHS up/down
    channel.assignment[2] = 0;  // map roll to RHS left/righ
    channel.assignment[3] = 3;  // map yaw to LHS up/down
    channel.assignment[4] = 4;  // map AUX1 to RHS click
    channel.assignment[5] = 5;  // map AUX2 to LHS click

    channel.inversion = 6;  // invert both pitch and roll

    channel.midpoint[0] = 1515;
    channel.midpoint[1] = 1515;
    channel.midpoint[2] = 1500;
    channel.midpoint[3] = 1520;
    channel.midpoint[4] = 1500;
    channel.midpoint[5] = 1500;

    channel.deadzone[0] = 20;
    channel.deadzone[1] = 20;
    channel.deadzone[2] = 20;
    channel.deadzone[3] = 40;
    channel.deadzone[4] = 20;
    channel.deadzone[5] = 20;

    // PID Parameters

    pid_parameters.thrust_master[0] = 1.0f;    // P
    pid_parameters.thrust_master[1] = 0.0f;    // I
    pid_parameters.thrust_master[2] = 0.0f;    // D
    pid_parameters.thrust_master[3] = 0.0f;    // Windup guard
    pid_parameters.thrust_master[4] = 0.005f;  // D filter usec (15Hz)
    pid_parameters.thrust_master[5] = 0.005f;  // setpoint filter usec (30Hz)
    pid_parameters.thrust_master[6] = 1.0f;    // (meters / full stick action)

    pid_parameters.pitch_master[0] = 5.0f;    // P
    pid_parameters.pitch_master[1] = 1.0f;    // I
    pid_parameters.pitch_master[2] = 0.0f;    // D
    pid_parameters.pitch_master[3] = 10.0f;   // Windup guard
    pid_parameters.pitch_master[4] = 0.005f;  // D filter usec (15Hz)
    pid_parameters.pitch_master[5] = 0.005f;  // setpoint filter usec (30Hz)
    pid_parameters.pitch_master[6] = 10.0f;   // (degrees / full stick action)

    pid_parameters.roll_master[0] = 5.0f;    // P
    pid_parameters.roll_master[1] = 1.0f;    // I
    pid_parameters.roll_master[2] = 0.0f;    // D
    pid_parameters.roll_master[3] = 10.0f;   // Windup guard
    pid_parameters.roll_master[4] = 0.005f;  // D filter usec (15Hz)
    pid_parameters.roll_master[5] = 0.005f;  // setpoint filter usec (30Hz)
    pid_parameters.roll_master[6] = 10.0f;   // (degrees / full stick action)

    pid_parameters.yaw_master[0] = 5.0f;    // P
    pid_parameters.yaw_master[1] = 1.0f;    // I
    pid_parameters.yaw_master[2] = 0.0f;    // D
    pid_parameters.yaw_master[3] = 10.0f;   // Windup guard
    pid_parameters.yaw_master[4] = 0.005f;  // D filter usec (15Hz)
    pid_parameters.yaw_master[5] = 0.005f;  // setpoint filter usec (30Hz)
    pid_parameters.yaw_master[6] = 180.0f;  // (degrees / full stick action)

    pid_parameters.thrust_slave[0] = 1.0f;    // P
    pid_parameters.thrust_slave[1] = 0.0f;    // I
    pid_parameters.thrust_slave[2] = 0.0f;    // D
    pid_parameters.thrust_slave[3] = 10.0f;   // Windup guard
    pid_parameters.thrust_slave[4] = 0.001f;  // D filter usec (150Hz)
    pid_parameters.thrust_slave[5] = 0.001f;  // setpoint filter usec (300Hz)
    pid_parameters.thrust_slave[6] = 0.3f;    // (meters/sec / full stick action)

    pid_parameters.pitch_slave[0] = 20.0f;   // P
    pid_parameters.pitch_slave[1] = 8.0f;    // I
    pid_parameters.pitch_slave[2] = 0.0f;    // D
    pid_parameters.pitch_slave[3] = 30.0f;   // Windup guard
    pid_parameters.pitch_slave[4] = 0.001f;  // D filter usec (150Hz)
    pid_parameters.pitch_slave[5] = 0.001f;  // setpoint filter usec (300Hz)
    pid_parameters.pitch_slave[6] = 30.0f;   // (deg/sec / full stick action)

    pid_parameters.roll_slave[0] = 20.0f;   // P
    pid_parameters.roll_slave[1] = 8.0f;    // I
    pid_parameters.roll_slave[2] = 0.0f;    // D
    pid_parameters.roll_slave[3] = 30.0f;   // Windup guard
    pid_parameters.roll_slave[4] = 0.001f;  // D filter usec (150Hz)
    pid_parameters.roll_slave[5] = 0.001f;  // setpoint filter usec (300Hz)
    pid_parameters.roll_slave[6] = 30.0f;   // (deg/sec / full stick action)

    pid_parameters.yaw_slave[0] = 30.0f;   // P
    pid_parameters.yaw_slave[1] = 5.0f;    // I
    pid_parameters.yaw_slave[2] = 0.0f;    // D
    pid_parameters.yaw_slave[3] = 20.0f;   // Windup guard
    pid_parameters.yaw_slave[4] = 0.001f;  // D filter usec (150Hz)
    pid_parameters.yaw_slave[5] = 0.001f;  // setpoint filter usec (300Hz)
    pid_parameters.yaw_slave[6] = 240.0f;  // (deg/sec / full stick action)

    pid_parameters.pid_bypass = BYPASS_THRUST_MASTER | BYPASS_THRUST_SLAVE | BYPASS_YAW_MASTER;  // AHRS/Horizon mode

    state_parameters.state_estimation[0] = 1.00f;  // 2*kp or BETA
    state_parameters.state_estimation[1] = 0.01f;  // 2*ki

    state_parameters.enable[0] = 0.001f;  // max variance
    state_parameters.enable[1] = 30.0f;   // max angle

    // This function will only initialize data variables
    // writeEEPROM() needs to be called manually to store this data in EEPROM
}

CONFIG_struct::CONFIG_struct(Systems& sys)
    : mix_table(sys.airframe.mix_table), mag_bias(sys.mag.mag_bias), channel(sys.receiver.channel), pid_parameters(sys.control.pid_parameters), state_parameters(sys.state.parameters) {
}

void CONFIG_struct::applyTo(Systems& systems) const {
    systems.airframe.mix_table = mix_table;
    systems.mag.mag_bias = mag_bias;
    systems.receiver.channel = channel;
    systems.control.pid_parameters = pid_parameters;
    systems.state.parameters = state_parameters;

    systems.control.parseConfig(pid_parameters);
}

template <class T>
bool verifyArgs(T& var) {
    return var.verify();
}

template <class T, class... TArgs>
bool verifyArgs(T& var, TArgs&... varArgs) {
    bool ok{verifyArgs(var)};
    return verifyArgs(varArgs...) && ok;
}

bool CONFIG_struct::verify() const {
    return verifyArgs(version, pcb, mix_table, mag_bias, channel, pid_parameters, state_parameters);
}

void writeEEPROM(const CONFIG_union& CONFIG) {
    for (size_t i = 0; i < sizeof(CONFIG_struct); i++) {
        if (CONFIG.raw[i] != EEPROM.read(i)) {  // Only re-write new data
            EEPROM.write(i, CONFIG.raw[i]);
        }
    }
}

bool isEmptyEEPROM() {
    return EEPROM.read(0) == 255;
}

CONFIG_union readEEPROM() {
    if (isEmptyEEPROM()) {
        // No EEPROM values detected, re-initialize to default values
        writeEEPROM(CONFIG_union());  // store the default values
        return readEEPROM();
    } else {
        CONFIG_union CONFIG;
        // There "is" data in the EEPROM, read it all
        for (uint16_t i = 0; i < sizeof(struct CONFIG_struct); i++) {
            CONFIG.raw[i] = EEPROM.read(i);
        }
        // Verify version and general settings
        if (CONFIG.data.verify()) {
            // If the stored configuration isn't legal in any way, report it
            // via debug and reset it
            writeEEPROM(CONFIG_union());  // store the default values
            return readEEPROM();
        } else {
            return CONFIG;
        }
    }
}
