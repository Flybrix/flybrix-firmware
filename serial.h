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
#include "BMP280.h"
#include "cobs.h"

class PilotCommand;
class Control;
class LED;
class State;
class StateFlag;
struct Systems;
struct Kinematics;
struct PowerMonitor;
struct CommandVector;
struct ControlVectors;

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

    enum Commands : uint8_t;
    enum States : uint8_t;

    SerialComm(Systems& systems, const volatile uint16_t* ppm);

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

    const State& state_;
    const volatile uint16_t* ppm;
    const Control& control_;
    Systems& systems_;
    LED& led_;
    const BMP280& bmp_;
    StateFlag& flag_;
    Kinematics& kinematics_;
    PilotCommand& pilot_;
    PowerMonitor& pwr_;
    CommandVector& command_vector_;
    ControlVectors& control_vectors_;

    uint16_t send_state_delay{1001};  // anything over 1000 turns off state messages
    uint16_t sd_card_state_delay{2};  // write to SD at the highest rate by default
    uint32_t state_mask{0x7fffff};
};

#include "serial_impl.h"

#endif
