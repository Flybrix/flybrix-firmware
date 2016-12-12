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
    // data needs to be written manually to the EEPROM
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

void CONFIG_struct::resetPartial(uint16_t submask, uint16_t led_mask) {
    if (submask & CONFIG_struct::VERSION) {
        version = Version();
    }
    if (submask & CONFIG_struct::ID) {
        id = ConfigID();
    }
    if (submask & CONFIG_struct::PCB) {
        pcb = PcbTransform();
    }
    if (submask & CONFIG_struct::MIX_TABLE) {
        mix_table = Airframe::MixTable();
    }
    if (submask & CONFIG_struct::MAG_BIAS) {
        mag_bias = AK8963::MagBias();
    }
    if (submask & CONFIG_struct::CHANNEL) {
        channel = R415X::ChannelProperties();
    }
    if (submask & CONFIG_struct::PID_PARAMETERS) {
        pid_parameters = Control::PIDParameters();
    }
    if (submask & CONFIG_struct::STATE_PARAMETERS) {
        state_parameters = State::Parameters();
    }
    if (submask & CONFIG_struct::LED_STATES) {
        LED::States default_states;
        for (size_t led_code = 0; led_code < 16; ++led_code) {
            if (led_mask & (1 << led_code)) {
                led_states.states[led_code] = default_states.states[led_code];
            }
        }
    }
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
    return verifyArgs(version, id, pcb, mix_table, mag_bias, channel, pid_parameters, state_parameters, led_states);
}

bool isEmptyEEPROM() {
    return EEPROM.read(0) == 255;
}

CONFIG_struct readEEPROM() {
    CONFIG_struct CONFIG;
    if (isEmptyEEPROM()) {
        // No EEPROM values detected, re-initialize to default values
        CONFIG_struct().writeTo(EEPROMCursor());  // store the default values
        CONFIG = readEEPROM();
    } else {
        EEPROMCursor cursor;
        cursor.ParseInto(CONFIG);
        // Verify version and general settings
        if (!CONFIG.verify()) {
            // If the stored configuration isn't legal in any way, report it
            // via debug and reset it
            CONFIG_struct().writeTo(EEPROMCursor());  // store the default values
            CONFIG = readEEPROM();
        }
    }
    return CONFIG;
}
