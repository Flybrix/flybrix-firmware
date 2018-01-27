/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "serial.h"
#include "BMP280.h"
#include "command.h"
#include "control.h"
#include "led.h"
#include "state.h"
#include "systems.h"
#include "stateFlag.h"
#include "imu.h"

namespace {

static_assert(0x4000000 == FLAG(26), "Mask is not calculated at compile time");

template <uint8_t I>
inline void doSubcommandWrap(SerialComm& serial, CobsReaderBuffer& input, uint32_t mask, uint32_t& ack) {
    if ((mask & FLAG(I)) && serial.doSubcommand<I>(input)) {
        ack |= FLAG(I);
    }
}

template <uint8_t I = 0>
inline typename std::enable_if<I == SerialComm::Commands::END_OF_COMMANDS, void>::type doSubcommands(SerialComm& serial, CobsReaderBuffer& input, uint32_t mask, uint32_t& ack) {
}

template <uint8_t I = 0>
    inline typename std::enable_if < I<SerialComm::Commands::END_OF_COMMANDS, void>::type doSubcommands(SerialComm& serial, CobsReaderBuffer& input, uint32_t mask, uint32_t& ack) {
    doSubcommandWrap<I>(serial, input, mask, ack);
    doSubcommands<I + 1>(serial, input, mask, ack);
}

template <uint8_t I>
inline void readSubstateWrap(const SerialComm& serial, CobsPayloadGeneric& payload, uint32_t mask) {
    if (mask & FLAG(I)) {
        serial.readSubstate<I>(payload);
    }
}

template <uint8_t I = 0>
inline typename std::enable_if<I == SerialComm::States::END_OF_STATES, void>::type readSubstates(const SerialComm& serial, CobsPayloadGeneric& payload, uint32_t mask) {
}

template <uint8_t I = 0>
    inline typename std::enable_if < I<SerialComm::States::END_OF_STATES, void>::type readSubstates(const SerialComm& serial, CobsPayloadGeneric& payload, uint32_t mask) {
    readSubstateWrap<I>(serial, payload, mask);
    readSubstates<I + 1>(serial, payload, mask);
}
}

SerialComm::SerialComm(Systems& systems, const volatile uint16_t* ppm)
    : state_(systems.state),
      ppm{ppm},
      control_(systems.control),
      systems_(systems),
      led_(systems.led),
      bmp_(systems.bmp),
      imu_(systems.imu),
      flag_(systems.flag),
      pilot_(systems.pilot),
      pwr_(systems.pwr),
      command_vector_(systems.command_vector),
      rc_mux_(systems.rc_mux),
      serial_rc_(systems.serialRc()),
      autopilot_(systems.autopilot),
      control_vectors_(systems.control_vectors) {
}

void SerialComm::Read() {
    for (;;) {
        CobsReaderBuffer* buffer{readSerial()};
        if (buffer == nullptr)
            return;
        ProcessData(*buffer, true);
    }
}

void SerialComm::ProcessData(CobsReaderBuffer& data_input, bool allow_response) {
    MessageType code;
    uint32_t mask;

    if (!data_input.ParseInto(code, mask)) {
        return;
    }
    if (code != MessageType::Command) {
        return;
    }

    uint32_t ack_data{0};
    doSubcommands(*this, data_input, mask, ack_data);

    if (allow_response && (mask & FLAG(REQ_RESPONSE))) {
        SendResponse(mask, ack_data);
    }
}

void SerialComm::SendConfiguration() const {
    CobsPayloadGeneric payload;
    WriteProtocolHead(SerialComm::MessageType::Command, FLAG(SET_EEPROM_DATA), payload);
    Config(systems_).writeTo(payload);
    WriteToOutput(payload);
}

void SerialComm::SendPartialConfiguration(uint16_t submask, uint16_t led_mask) const {
    CobsPayloadGeneric payload;
    WriteProtocolHead(SerialComm::MessageType::Command, FLAG(SET_PARTIAL_EEPROM_DATA), payload);
    Config(systems_).writePartialTo(payload, submask, led_mask);
    WriteToOutput(payload);
}

void SerialComm::SendDebugString(const String& string, MessageType type) const {
    CobsPayload<2000> payload;
    WriteProtocolHead(type, 0xFFFFFFFF, payload);
    size_t str_len = string.length();
    for (size_t i = 0; i < str_len; ++i)
        payload.Append(string.charAt(i));
    payload.Append(uint8_t(0));
    WriteToOutput(payload);
}

void SerialComm::SendState(uint32_t mask, bool redirect_to_sd_card) const {
    // No need to build the message if we are not writing to the card
    if (redirect_to_sd_card && sdcard::getState() != sdcard::State::WriteStates)
        return;
    if (!mask)
        mask = state_mask;
    // No need to publish empty state messages
    if (!mask)
        return;

    CobsPayloadGeneric payload;

    WriteProtocolHead(SerialComm::MessageType::State, mask, payload);
    readSubstates(*this, payload, mask);
    WriteToOutput(payload, redirect_to_sd_card);
}

void SerialComm::SendResponse(uint32_t mask, uint32_t response) const {
    CobsPayload<12> payload;
    WriteProtocolHead(MessageType::Response, mask, payload);
    payload.Append(response);
    WriteToOutput(payload);
}

uint16_t SerialComm::GetSendStateDelay() const {
    return send_state_delay;
}

uint16_t SerialComm::GetSdCardStateDelay() const {
    return sd_card_state_delay;
}

void SerialComm::SetStateMsg(uint32_t values) {
    state_mask = values;
}

void SerialComm::AddToStateMsg(uint32_t values) {
    state_mask |= values;
}

void SerialComm::RemoveFromStateMsg(uint32_t values) {
    state_mask &= ~values;
}
