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

#include "AK8963.h"
#include "R415X.h"
#include "airframe.h"
#include "control.h"
#include "imu.h"
#include "led.h"
#include "state.h"
#include "version.h"
#include "devicename.h"

#include <tuple>

struct Systems;

struct __attribute__((packed)) ConfigID {
    ConfigID();
    explicit ConfigID(uint32_t id) : id{id} {
    }
    bool verify() const {
        return true;
    }
    uint32_t id;
};

static_assert(sizeof(ConfigID) == 4, "Data is not packed");

struct Config {
    enum Field : uint16_t {
        VERSION,
        ID,
        PCB,
        MIX_TABLE,
        MAG_BIAS,
        CHANNEL,
        PID_PARAMETERS,
        STATE_PARAMETERS,
        LED_STATES,
        DEVICE_NAME,
        VELOCITY_PID_PARAMETERS,
        INERTIAL_BIAS,
    };

    using Data = std::tuple<Version, ConfigID, PcbTransform, Airframe::MixTable, AK8963::MagBias, R415X::ChannelProperties, Control::PIDParameters, State::Parameters, LED::States, DeviceName,
                            Control::VelocityPIDParameters, Imu::InertialBias>;

    Config();
    explicit Config(Systems& sys);
    void applyTo(Systems& systems) const;
    bool verify() const;

    void resetPartial(uint16_t submask, uint16_t led_mask);

    template <class Cursor>
    void writeTo(Cursor&& cursor) const;

    template <class Cursor>
    void writePartialTo(Cursor&& cursor, uint16_t submask, uint16_t led_mask) const;

    template <class Cursor>
    void writeSkippableTo(Cursor&& cursor, uint16_t submask, uint16_t led_mask) const;

    template <class Cursor>
    bool readFrom(Cursor&& cursor);

    template <class Cursor>
    bool readPartialFrom(Cursor&& cursor, uint16_t& submask, uint16_t& led_mask);

    template <class Cursor>
    static bool readMasks(Cursor&& cursor, uint16_t& submask, uint16_t& led_mask);

    Data data;
};

static_assert(sizeof(Config) ==
                  sizeof(Version) + sizeof(ConfigID) + sizeof(PcbTransform) + sizeof(Airframe::MixTable) + sizeof(AK8963::MagBias) + sizeof(R415X::ChannelProperties) + sizeof(State::Parameters) +
                      sizeof(Control::PIDParameters) + sizeof(LED::States) + sizeof(DeviceName) + sizeof(Control::VelocityPIDParameters) + sizeof(Imu::InertialBias),
              "Data is not packed");

static_assert(sizeof(Config) == 837, "Data does not have expected size");

Config readEEPROM();
bool isEmptyEEPROM();

#include "config_impl.h"

#endif
