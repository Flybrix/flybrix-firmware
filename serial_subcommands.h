#ifndef SERIAL_SUBCOMMANDS_H
#define SERIAL_SUBCOMMANDS_H

#include "serial_impl.h"

template <>
inline bool SerialComm::doSubcommand<SerialComm::REQ_RESPONSE>(CobsReaderBuffer& input) {
    return false;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_EEPROM_DATA>(CobsReaderBuffer& input) {
    Config tmp_config;
    if (!tmp_config.readFrom(input)) {
        return false;
    }
    if (!tmp_config.verify()) {
        return false;
    }
    tmp_config.applyTo(*systems);
    tmp_config.writeTo(EEPROMCursor());
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::REINIT_EEPROM_DATA>(CobsReaderBuffer& input) {
    const Config tmp_config;
    tmp_config.applyTo(*systems);
    tmp_config.writeTo(EEPROMCursor());
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::REQ_EEPROM_DATA>(CobsReaderBuffer& input) {
    SendConfiguration();
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::REQ_ENABLE_ITERATION>(CobsReaderBuffer& input) {
    uint8_t flag;
    if (!input.ParseInto(flag)) {
        return false;
    }
    if (flag == 1) {
        state->processMotorEnablingIteration();
    } else {
        state->disableMotors();
    }
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::MOTOR_OVERRIDE_SPEED_0>(CobsReaderBuffer& input) {
    return input.ParseInto(state->MotorOut[0]);
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::MOTOR_OVERRIDE_SPEED_1>(CobsReaderBuffer& input) {
    return input.ParseInto(state->MotorOut[1]);
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::MOTOR_OVERRIDE_SPEED_2>(CobsReaderBuffer& input) {
    return input.ParseInto(state->MotorOut[2]);
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::MOTOR_OVERRIDE_SPEED_3>(CobsReaderBuffer& input) {
    return input.ParseInto(state->MotorOut[3]);
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::MOTOR_OVERRIDE_SPEED_4>(CobsReaderBuffer& input) {
    return input.ParseInto(state->MotorOut[4]);
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::MOTOR_OVERRIDE_SPEED_5>(CobsReaderBuffer& input) {
    return input.ParseInto(state->MotorOut[5]);
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::MOTOR_OVERRIDE_SPEED_6>(CobsReaderBuffer& input) {
    return input.ParseInto(state->MotorOut[6]);
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::MOTOR_OVERRIDE_SPEED_7>(CobsReaderBuffer& input) {
    return input.ParseInto(state->MotorOut[7]);
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_COMMAND_OVERRIDE>(CobsReaderBuffer& input) {
    uint8_t flag;
    if (!input.ParseInto(flag)) {
        return false;
    }
    if (flag == 1) {
        state->set(STATUS_OVERRIDE);
    } else {
        state->clear(STATUS_OVERRIDE);
    }
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_STATE_MASK>(CobsReaderBuffer& input) {
    uint32_t new_state_mask;
    if (!input.ParseInto(new_state_mask)) {
        return false;
    }
    SetStateMsg(new_state_mask);
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_STATE_DELAY>(CobsReaderBuffer& input) {
    uint16_t new_state_delay;
    if (!input.ParseInto(new_state_delay)) {
        return false;
    }
    send_state_delay = new_state_delay;
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_SD_WRITE_DELAY>(CobsReaderBuffer& input) {
    uint16_t new_state_delay;
    if (!input.ParseInto(new_state_delay)) {
        return false;
    }
    sd_card_state_delay = new_state_delay;
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_LED>(CobsReaderBuffer& input) {
    uint8_t mode, r1, g1, b1, r2, g2, b2, ind_r, ind_g;
    if (!input.ParseInto(mode, r1, g1, b1, r2, g2, b2, ind_r, ind_g)) {
        return false;
    }
    led->set(LED::Pattern(mode), r1, g1, b1, r2, g2, b2, ind_r, ind_g);
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_SERIAL_RC>(CobsReaderBuffer& input) {
    uint8_t enabled;
    int16_t throttle, pitch, roll, yaw;
    uint8_t auxmask;
    if (!input.ParseInto(enabled, throttle, pitch, roll, yaw, auxmask)) {
        return false;
    }
    if (enabled) {
        state->command_source_mask |= COMMAND_READY_BTLE;
        state->command_AUX_mask = auxmask;
        state->command_throttle = throttle;
        state->command_pitch = pitch;
        state->command_roll = roll;
        state->command_yaw = yaw;
    } else {
        state->command_source_mask &= ~COMMAND_READY_BTLE;
    }
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_CARD_RECORDING>(CobsReaderBuffer& input) {
    uint8_t recording_flags;
    if (!input.ParseInto(recording_flags)) {
        return false;
    }
    bool shouldRecordToCard = recording_flags & 1;
    bool shouldLock = recording_flags & 2;
    sdcard::setLock(false);
    if (shouldRecordToCard) {
        sdcard::openFile();
    } else {
        sdcard::closeFile();
    }
    sdcard::setLock(shouldLock);
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::SET_PARTIAL_EEPROM_DATA>(CobsReaderBuffer& input) {
    Config tmp_config(*systems);
    if (!tmp_config.readPartialFrom(input)) {
        return false;
    }
    if (tmp_config.verify()) {
        return false;
    }
    tmp_config.applyTo(*systems);
    tmp_config.writeTo(EEPROMCursor());
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::REINIT_PARTIAL_EEPROM_DATA>(CobsReaderBuffer& input) {
    uint16_t submask, led_mask;
    if (!Config::readMasks(input, submask, led_mask)) {
        return false;
    }
    Config tmp_config(*systems);
    tmp_config.resetPartial(submask, led_mask);
    if (!tmp_config.verify()) {
        return false;
    }
    tmp_config.applyTo(*systems);
    tmp_config.writeTo(EEPROMCursor());
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::REQ_PARTIAL_EEPROM_DATA>(CobsReaderBuffer& input) {
    uint16_t submask, led_mask;
    if (!Config::readMasks(input, submask, led_mask)) {
        return false;
    }
    SendPartialConfiguration(submask, led_mask);
    return true;
}

template <>
inline bool SerialComm::doSubcommand<SerialComm::REQ_CARD_RECORDING_STATE>(CobsReaderBuffer& input) {
    CobsPayload<20> payload;
    WriteProtocolHead(SerialComm::MessageType::Command, FLAG(SET_SD_WRITE_DELAY) | FLAG(SET_CARD_RECORDING), payload);
    payload.Append(sd_card_state_delay);
    uint8_t flags = 0;
    if (sdcard::isOpen()) {
        flags |= 1;
    }
    if (sdcard::isLocked()) {
        flags |= 2;
    }
    payload.Append(flags);
    WriteToOutput(payload);
    return true;
}

#endif /* SERIAL_SUBCOMMANDS_H */
