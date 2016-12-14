/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <serial.h/cpp>

    Serial data transfers are encoded as packets using COBS. Data is parsed assuming a known byte order according to an inclusion bitmask.
*/

#ifndef serial_h
#define serial_h

#include <Arduino.h>
#include "cobs.h"

class PilotCommand;
class Control;
class LED;
class State;
struct Systems;

using CobsPayloadGeneric = CobsPayload<1000>;  // impacts memory use only; packet size should be <= client packet size

static constexpr uint32_t FLAG(uint8_t field) {
    return uint32_t{1} << field;
}

class SerialComm {
   public:
    enum class MessageType : uint8_t {
        State = 0,
        Command = 1,
        Response = 255,
        Timelog = 2,
        DebugString = 3,
        HistoryData = 4,
    };

    enum Command : uint8_t {
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
        END_OF_COMMANDS,
    };

    enum States : uint8_t {
        MICROS,
        STATUS,
        V0,
        I0,
        I1,
        ACCEL,
        GYRO,
        MAG,
        TEMPERATURE,
        PRESSURE,
        RX_PPM,
        AUX_CHAN_MASK,
        COMMANDS,
        F_AND_T,
        PID_FZ_MASTER,
        PID_TX_MASTER,
        PID_TY_MASTER,
        PID_TZ_MASTER,
        PID_FZ_SLAVE,
        PID_TX_SLAVE,
        PID_TY_SLAVE,
        PID_TZ_SLAVE,
        MOTOR_OUT,
        KINE_ANGLE,
        KINE_RATE,
        KINE_ALTITUDE,
        LOOP_COUNT,
        END_OF_STATES,
    };

    explicit SerialComm(State* state, const volatile uint16_t* ppm, const Control* control, Systems* systems, LED* led, PilotCommand* command);

    void Read();

    void SendConfiguration() const;
    void SendPartialConfiguration(uint16_t submask, uint16_t led_mask) const;
    void SendDebugString(const String& string, MessageType type = MessageType::DebugString) const;
    void SendState(uint32_t mask = 0, bool redirect_to_sd_card = false) const;
    void SendResponse(uint32_t mask, uint32_t response) const;

    uint16_t GetSendStateDelay() const;
    uint16_t GetSdCardStateDelay() const;
    void SetStateMsg(uint32_t values);
    void AddToStateMsg(uint32_t values);
    void RemoveFromStateMsg(uint32_t values);

    template <uint8_t I>
    inline bool doSubcommand(CobsReaderBuffer& input);
    template <uint8_t I>
    inline void readSubstate(CobsPayloadGeneric& payload) const;

   private:
    void ProcessData(CobsReaderBuffer& data_input);

    uint16_t PacketSize(uint32_t mask) const;

    State* state;
    const volatile uint16_t* ppm;
    const Control* control;
    Systems* systems;
    LED* led;
    PilotCommand* command;
    uint16_t send_state_delay{1001};  // anything over 1000 turns off state messages
    uint16_t sd_card_state_delay{2};  // write to SD at the highest rate by default
    uint32_t state_mask{0x7fffff};
};

#include "serial_impl.h"

#endif
