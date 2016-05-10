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

union CONFIG_union;
class PilotCommand;
class Control;
class LED;
class State;

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

    enum CommandFields : uint32_t {
        COM_REQ_RESPONSE = 1 << 0,
        COM_SET_EEPROM_DATA = 1 << 1,
        COM_REINIT_EEPROM_DATA = 1 << 2,
        COM_REQ_EEPROM_DATA = 1 << 3,
        COM_REQ_ENABLE_ITERATION = 1 << 4,
        COM_MOTOR_OVERRIDE_SPEED_0 = 1 << 5,
        COM_MOTOR_OVERRIDE_SPEED_1 = 1 << 6,
        COM_MOTOR_OVERRIDE_SPEED_2 = 1 << 7,
        COM_MOTOR_OVERRIDE_SPEED_3 = 1 << 8,
        COM_MOTOR_OVERRIDE_SPEED_4 = 1 << 9,
        COM_MOTOR_OVERRIDE_SPEED_5 = 1 << 10,
        COM_MOTOR_OVERRIDE_SPEED_6 = 1 << 11,
        COM_MOTOR_OVERRIDE_SPEED_7 = 1 << 12,
        COM_MOTOR_OVERRIDE_SPEED_ALL = COM_MOTOR_OVERRIDE_SPEED_0 | COM_MOTOR_OVERRIDE_SPEED_1 | COM_MOTOR_OVERRIDE_SPEED_2 | COM_MOTOR_OVERRIDE_SPEED_3 | COM_MOTOR_OVERRIDE_SPEED_4 |
                                       COM_MOTOR_OVERRIDE_SPEED_5 | COM_MOTOR_OVERRIDE_SPEED_6 | COM_MOTOR_OVERRIDE_SPEED_7,
        COM_SET_COMMAND_OVERRIDE = 1 << 13,
        COM_SET_STATE_MASK = 1 << 14,
        COM_SET_STATE_DELAY = 1 << 15,
        COM_REQ_HISTORY = 1 << 16,
        COM_SET_LED = 1 << 17,
        COM_SET_SERIAL_RC = 1 << 18,
    };

    enum StateFields : uint32_t {
        STATE_ALL = 0xFFFFFFFF,
        STATE_NONE = 0,
        STATE_MICROS = 1 << 0,
        STATE_STATUS = 1 << 1,
        STATE_V0 = 1 << 2,
        STATE_I0 = 1 << 3,
        STATE_I1 = 1 << 4,
        STATE_ACCEL = 1 << 5,
        STATE_GYRO = 1 << 6,
        STATE_MAG = 1 << 7,
        STATE_TEMPERATURE = 1 << 8,
        STATE_PRESSURE = 1 << 9,
        STATE_RX_PPM = 1 << 10,
        STATE_AUX_CHAN_MASK = 1 << 11,
        STATE_COMMANDS = 1 << 12,
        STATE_F_AND_T = 1 << 13,
        STATE_PID_FZ_MASTER = 1 << 15,
        STATE_PID_TX_MASTER = 1 << 16,
        STATE_PID_TY_MASTER = 1 << 17,
        STATE_PID_TZ_MASTER = 1 << 18,
        STATE_PID_FZ_SLAVE = 1 << 19,
        STATE_PID_TX_SLAVE = 1 << 20,
        STATE_PID_TY_SLAVE = 1 << 21,
        STATE_PID_TZ_SLAVE = 1 << 22,
        STATE_MOTOR_OUT = 1 << 23,
        STATE_KINE_ANGLE = 1 << 24,
        STATE_KINE_RATE = 1 << 25,
        STATE_KINE_ALTITUDE = 1 << 26,
        STATE_LOOP_COUNT = 1 << 27,
    };

    explicit SerialComm(State* state, const volatile uint16_t* ppm, const Control* control, CONFIG_union* config, LED* led, PilotCommand* command);

    void Read();

    void SendConfiguration() const;
    void SendDebugString(const String& string, MessageType type = MessageType::DebugString) const;
    void SendState(uint32_t timestamp_us, uint32_t mask = 0) const;
    void SendResponse(uint32_t mask, uint32_t response) const;

    uint16_t GetSendStateDelay() const;
    void SetStateMsg(uint32_t values);
    void AddToStateMsg(uint32_t values);
    void RemoveFromStateMsg(uint32_t values);

   private:
    void ProcessData(CobsReaderBuffer& data_input);

    uint16_t PacketSize(uint32_t mask) const;

    State* state;
    const volatile uint16_t* ppm;
    const Control* control;
    CONFIG_union* config;
    LED* led;
    PilotCommand* command;
    uint16_t send_state_delay{1001};  // anything over 1000 turns off state messages
    uint32_t state_mask{0x7fffff};
};

#endif
