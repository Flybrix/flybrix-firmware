#ifndef SERIAL_SUBCOMMANDS_H
#define SERIAL_SUBCOMMANDS_H

#include "serial_impl.h"

#include "autopilot.h"
#include "command.h"

enum SerialComm::Commands : uint8_t {
    REQ_RESPONSE,
    SET_EEPROM_DATA,
    REINIT_EEPROM_DATA,
    REQ_EEPROM_DATA,
    REQ_ENABLE_ITERATION,
    MOTOR_OVERRIDE_SPEED_0,
    MOTOR_OVERRIDE_SPEED_1,
    MOTOR_OVERRIDE_SPEED_2,
    MOTOR_OVERRIDE_SPEED_3,
    MOTOR_OVERRIDE_SPEED_4,
    MOTOR_OVERRIDE_SPEED_5,
    MOTOR_OVERRIDE_SPEED_6,
    MOTOR_OVERRIDE_SPEED_7,
    SET_COMMAND_OVERRIDE,
    SET_STATE_MASK,
    SET_STATE_DELAY,
    SET_SD_WRITE_DELAY,
    SET_LED,
    SET_SERIAL_RC,
    SET_CARD_RECORDING,
    SET_PARTIAL_EEPROM_DATA,
    REINIT_PARTIAL_EEPROM_DATA,
    REQ_PARTIAL_EEPROM_DATA,
    REQ_CARD_RECORDING_STATE,
    SET_PARTIAL_TEMPORARY_CONFIG,
    SET_COMMAND_SOURCES,
    SET_CALIBRATION,
    SET_AUTOPILOT,
    END_OF_COMMANDS,
};

// Assigns a subcommand to a flag, whereby the input is named "input"
#define DO_SUBCOMMAND(name) \
    template <>             \
    inline bool SerialComm::doSubcommand<SerialComm::Commands::name>(CobsReaderBuffer & input)

DO_SUBCOMMAND(REQ_RESPONSE) {
    return false;
}

DO_SUBCOMMAND(SET_EEPROM_DATA) {
    Config tmp_config;
    if (!tmp_config.readFrom(input)) {
        return false;
    }
    if (!tmp_config.verify()) {
        return false;
    }
    tmp_config.applyTo(systems_);
    tmp_config.writeTo(EEPROMCursor());
    return true;
}

DO_SUBCOMMAND(REINIT_EEPROM_DATA) {
    const Config tmp_config;
    tmp_config.applyTo(systems_);
    tmp_config.writeTo(EEPROMCursor());
    return true;
}

DO_SUBCOMMAND(REQ_EEPROM_DATA) {
    SendConfiguration();
    return true;
}

DO_SUBCOMMAND(REQ_ENABLE_ITERATION) {
    uint8_t flag;
    if (!input.ParseInto(flag)) {
        return false;
    }
    if (flag == 1) {
        pilot_.processMotorEnablingIteration();
    } else {
        pilot_.disableMotors();
    }
    return true;
}

DO_SUBCOMMAND(MOTOR_OVERRIDE_SPEED_0) {
    return pilot_.readMotor(0, input);
}

DO_SUBCOMMAND(MOTOR_OVERRIDE_SPEED_1) {
    return pilot_.readMotor(1, input);
}

DO_SUBCOMMAND(MOTOR_OVERRIDE_SPEED_2) {
    return pilot_.readMotor(2, input);
}

DO_SUBCOMMAND(MOTOR_OVERRIDE_SPEED_3) {
    return pilot_.readMotor(3, input);
}

DO_SUBCOMMAND(MOTOR_OVERRIDE_SPEED_4) {
    return pilot_.readMotor(4, input);
}

DO_SUBCOMMAND(MOTOR_OVERRIDE_SPEED_5) {
    return pilot_.readMotor(5, input);
}

DO_SUBCOMMAND(MOTOR_OVERRIDE_SPEED_6) {
    return pilot_.readMotor(6, input);
}

DO_SUBCOMMAND(MOTOR_OVERRIDE_SPEED_7) {
    return pilot_.readMotor(7, input);
}

DO_SUBCOMMAND(SET_COMMAND_OVERRIDE) {
    uint8_t flag;
    if (!input.ParseInto(flag)) {
        return false;
    }
    bool override = flag != 0;
    pilot_.override(override);
    return true;
}

DO_SUBCOMMAND(SET_STATE_MASK) {
    uint32_t new_state_mask;
    if (!input.ParseInto(new_state_mask)) {
        return false;
    }
    SetStateMsg(new_state_mask);
    return true;
}

DO_SUBCOMMAND(SET_STATE_DELAY) {
    uint16_t new_state_delay;
    if (!input.ParseInto(new_state_delay)) {
        return false;
    }
    send_state_delay = new_state_delay;
    return true;
}

DO_SUBCOMMAND(SET_SD_WRITE_DELAY) {
    uint16_t new_state_delay;
    if (!input.ParseInto(new_state_delay)) {
        return false;
    }
    sd_card_state_delay = new_state_delay;
    return true;
}

DO_SUBCOMMAND(SET_LED) {
    uint8_t mode, r1, g1, b1, r2, g2, b2, ind_r, ind_g;
    if (!input.ParseInto(mode, r1, g1, b1, r2, g2, b2, ind_r, ind_g)) {
        return false;
    }
    led_.set(LED::Pattern(mode), r1, g1, b1, r2, g2, b2, ind_r, ind_g);
    return true;
}

DO_SUBCOMMAND(SET_SERIAL_RC) {
    uint8_t enabled;
    int16_t throttle, pitch, roll, yaw;
    uint8_t auxmask;
    if (!input.ParseInto(enabled, throttle, pitch, roll, yaw, auxmask)) {
        return false;
    }
    if (enabled) {
        serial_rc_.update(auxmask, throttle, pitch, roll, yaw);
    } else {
        serial_rc_.clear();
    }
    return true;
}

DO_SUBCOMMAND(SET_CARD_RECORDING) {
    uint8_t recording_flags;
    if (!input.ParseInto(recording_flags)) {
        return false;
    }
    bool shouldRecordToCard = recording_flags & 1;
    bool shouldLock = recording_flags & 2;

    sdcard::writing::setLock(false);
    bool success{false};
    if (shouldRecordToCard) {
        if (sdcard::getState() == sdcard::State::Closed) {
            sdcard::writing::open();
            success = true;
        }
    } else {
        if (sdcard::getState() == sdcard::State::WriteStates) {
            sdcard::writing::close();
            success = true;
        }
    }
    sdcard::writing::setLock(shouldLock);
    return success;
}

DO_SUBCOMMAND(SET_PARTIAL_EEPROM_DATA) {
    Config tmp_config(systems_);
    uint16_t submask, led_mask;
    if (!tmp_config.readPartialFrom(input, submask, led_mask)) {
        return false;
    }
    if (!tmp_config.verify()) {
        return false;
    }
    tmp_config.applyTo(systems_);
    tmp_config.writeSkippableTo(EEPROMCursor(), submask, led_mask);
    return true;
}

DO_SUBCOMMAND(REINIT_PARTIAL_EEPROM_DATA) {
    uint16_t submask, led_mask;
    if (!Config::readMasks(input, submask, led_mask)) {
        return false;
    }
    Config tmp_config(systems_);
    tmp_config.resetPartial(submask, led_mask);
    if (!tmp_config.verify()) {
        return false;
    }
    tmp_config.applyTo(systems_);
    tmp_config.writeSkippableTo(EEPROMCursor(), submask, led_mask);
    return true;
}

DO_SUBCOMMAND(REQ_PARTIAL_EEPROM_DATA) {
    uint16_t submask, led_mask;
    if (!Config::readMasks(input, submask, led_mask)) {
        return false;
    }
    SendPartialConfiguration(submask, led_mask);
    return true;
}

DO_SUBCOMMAND(REQ_CARD_RECORDING_STATE) {
    CobsPayload<20> payload;
    WriteProtocolHead(SerialComm::MessageType::Command, FLAG(SET_SD_WRITE_DELAY) | FLAG(SET_CARD_RECORDING), payload);
    payload.Append(sd_card_state_delay);
    uint8_t flags = 0;
    if (sdcard::getState() == sdcard::State::WriteStates) {
        flags |= 1;
    }
    if (sdcard::writing::isLocked()) {
        flags |= 2;
    }
    payload.Append(flags);
    WriteToOutput(payload);
    return true;
}

DO_SUBCOMMAND(SET_PARTIAL_TEMPORARY_CONFIG) {
    Config tmp_config(systems_);
    uint16_t submask, led_mask;
    if (!tmp_config.readPartialFrom(input, submask, led_mask)) {
        return false;
    }
    if (!tmp_config.verify()) {
        return false;
    }
    tmp_config.applyTo(systems_);
    return true;
}

DO_SUBCOMMAND(SET_COMMAND_SOURCES) {
    uint8_t sources;
    if (!input.ParseInto(sources)) {
        return false;
    }
    rc_mux_.setFilter(sources);
    return true;
}

DO_SUBCOMMAND(SET_CALIBRATION) {
    uint8_t enabled;
    uint8_t mode;
    if (!input.ParseInto(enabled, mode)) {
        return false;
    }
    if (mode > 5) {
        return false;
    }
    if (mode == 0) {
        imu_.setMagnetometerCalibrating(enabled);
        imu_.setAccelerometerCalibrating(false, RotationEstimator::Pose::Flat);
    } else {
        imu_.setMagnetometerCalibrating(false);
        imu_.setAccelerometerCalibrating(enabled, RotationEstimator::Pose(mode - 1));
    }

    if (!enabled) {
        // Store config in permanent storage when calibration stops
        Config tmp_config(systems_);
        if (!tmp_config.verify()) {
            return false;
        }
        tmp_config.writeTo(EEPROMCursor());
    }

    return true;
}

DO_SUBCOMMAND(SET_AUTOPILOT) {
    uint8_t enabled;
    if (!input.ParseInto(enabled)) {
        return false;
    }

    if (enabled) {
        autopilot_.start(micros());
    } else {
        autopilot_.stop();
    }

    return true;
}

#undef DO_SUBCOMMAND

#endif /* SERIAL_SUBCOMMANDS_H */
