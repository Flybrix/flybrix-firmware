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

Config::Config() {  // Default Settings
    // This function will only initialize data variables
    // data needs to be written manually to the EEPROM
}

Version version;
PcbTransform pcb_transform;

template <std::size_t field>
inline decltype(std::get<field>(Config::Data())) & systemMapping(Systems& sys);

template <>
inline Version& systemMapping<Config::VERSION>(Systems& sys) {
    // TODO: add this to the systems
    version = Version();
    return version;
}

template <>
inline ConfigID& systemMapping<Config::ID>(Systems& sys) {
    return sys.id;
}

template <>
inline PcbTransform& systemMapping<Config::PCB>(Systems& sys) {
    // TODO: add this to the systems
    pcb_transform = PcbTransform();
    return pcb_transform;
}

template <>
inline Airframe::MixTable& systemMapping<Config::MIX_TABLE>(Systems& sys) {
    return sys.airframe.mix_table;
}

template <>
inline AK8963::MagBias& systemMapping<Config::MAG_BIAS>(Systems& sys) {
    return sys.mag.mag_bias;
}

template <>
inline R415X::ChannelProperties& systemMapping<Config::CHANNEL>(Systems& sys) {
    return sys.receiver.channel;
}

template <>
inline Control::PIDParameters& systemMapping<Config::PID_PARAMETERS>(Systems& sys) {
    return sys.control.pid_parameters;
}

template <>
inline State::Parameters& systemMapping<Config::STATE_PARAMETERS>(Systems& sys) {
    return sys.state.parameters;
}

template <>
inline LED::States& systemMapping<Config::LED_STATES>(Systems& sys) {
    return sys.led.states;
}

bool isEmptyEEPROM() {
    return EEPROM.read(0) == 255;
}

Config readEEPROM() {
    Config CONFIG;
    if (isEmptyEEPROM()) {
        // No EEPROM values detected, re-initialize to default values
        Config().writeTo(EEPROMCursor());  // store the default values
        CONFIG = readEEPROM();
    } else {
        CONFIG.readFrom(EEPROMCursor());
        // Verify version and general settings
        if (!CONFIG.verify()) {
            // If the stored configuration isn't legal in any way, report it
            // via debug and reset it
            Config().writeTo(EEPROMCursor());  // store the default values
            CONFIG = readEEPROM();
        }
    }
    return CONFIG;
}

// No need to modify anything below this line

template <std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type applyFieldsTo(const std::tuple<Tp...>& t, Systems& systems) {
}

template <std::size_t I = 0, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), void>::type applyFieldsTo(const std::tuple<Tp...>& t, Systems& systems) {
    systemMapping<I>(systems) = std::get<I>(t);
    applyFieldsTo<I + 1, Tp...>(t, systems);
    systems.parseConfig();
}

template <std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type setFieldsFrom(std::tuple<Tp...>& t, Systems& systems) {
}

template <std::size_t I = 0, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), void>::type setFieldsFrom(std::tuple<Tp...>& t, Systems& systems) {
    std::get<I>(t) = systemMapping<I>(systems);
    setFieldsFrom<I + 1, Tp...>(t, systems);
}

Config::Config(Systems& sys) {
    setFieldsFrom(data, sys);
}

void Config::resetPartial(uint16_t submask, uint16_t led_mask) {
    resetFields(data, submask, led_mask);
}

void Config::applyTo(Systems& systems) const {
    applyFieldsTo(data, systems);
}

template <class T>
inline bool verifyArg(T& var) {
    return var.verify();
}

template <std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), bool>::type verifyArgs(const std::tuple<Tp...>& t) {
    return true;
}

template <std::size_t I = 0, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), bool>::type verifyArgs(const std::tuple<Tp...>& t) {
    bool ok{verifyArg(std::get<I>(t))};
    return verifyArgs<I + 1, Tp...>(t) && ok;
}

bool Config::verify() const {
    return verifyArgs(data);
}
