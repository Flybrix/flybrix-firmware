/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "config.h"

#include "systems.h"
#include "eepromcursor.h"

PcbTransform::PcbTransform()          // Default settings
    : orientation{0.0f, 0.0f, 0.0f},  // pitch, roll, yaw; applied in that order
      translation{0.0f, 0.0f, 0.0f}   // x, y, z in mm
{
}

// Default Config ID
ConfigID::ConfigID() : ConfigID{0} {
}

CONFIG_struct::CONFIG_struct() {  // Default Settings
    // This function will only initialize data variables
    // writeEEPROM() needs to be called manually to store this data in EEPROM
}

CONFIG_struct::CONFIG_struct(Systems& sys)
    : id(sys.id),
      mix_table(sys.airframe.mix_table),
      mag_bias(sys.mag.mag_bias),
      channel(sys.receiver.channel),
      pid_parameters(sys.control.pid_parameters),
      state_parameters(sys.state.parameters),
      led_states(sys.led.states) {
}

void CONFIG_struct::applyTo(Systems& systems) const {
    systems.airframe.mix_table = mix_table;
    systems.mag.mag_bias = mag_bias;
    systems.receiver.channel = channel;
    systems.state.parameters = state_parameters;

    systems.control.parseConfig(pid_parameters);
    systems.led.parseConfig(led_states);
    systems.id = id;
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
    return verifyArgs(version, pcb, mix_table, mag_bias, channel, pid_parameters, state_parameters, led_states, id);
}

void writeEEPROM(const CONFIG_struct& CONFIG) {
    EEPROMCursor cursor;
    cursor.Append(CONFIG);
}

bool isEmptyEEPROM() {
    return EEPROM.read(0) == 255;
}

CONFIG_struct readEEPROM() {
    CONFIG_struct CONFIG;
    if (isEmptyEEPROM()) {
        // No EEPROM values detected, re-initialize to default values
        writeEEPROM(CONFIG_struct());  // store the default values
        CONFIG = readEEPROM();
    } else {
        EEPROMCursor cursor;
        cursor.ParseInto(CONFIG);
        // Verify version and general settings
        if (!CONFIG.verify()) {
            // If the stored configuration isn't legal in any way, report it
            // via debug and reset it
            writeEEPROM(CONFIG_struct());  // store the default values
            CONFIG = readEEPROM();
        }
    }
    return CONFIG;
}
