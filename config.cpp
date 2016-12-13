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

Version version;
PcbTransform pcb_transform;

template <std::size_t field>
inline decltype(std::get<field>(CONFIG_struct::Data())) & systemMapping(Systems& sys);

template <>
inline decltype(std::get<CONFIG_struct::VERSION>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::VERSION>(Systems& sys) {
    // TODO: add this to the systems
    version = Version();
    return version;
}

template <>
inline decltype(std::get<CONFIG_struct::ID>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::ID>(Systems& sys) {
    return sys.id;
}

template <>
inline decltype(std::get<CONFIG_struct::PCB>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::PCB>(Systems& sys) {
    // TODO: add this to the systems
    pcb_transform = PcbTransform();
    return pcb_transform;
}

template <>
inline decltype(std::get<CONFIG_struct::MIX_TABLE>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::MIX_TABLE>(Systems& sys) {
    return sys.airframe.mix_table;
}

template <>
inline decltype(std::get<CONFIG_struct::MAG_BIAS>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::MAG_BIAS>(Systems& sys) {
    return sys.mag.mag_bias;
}

template <>
inline decltype(std::get<CONFIG_struct::CHANNEL>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::CHANNEL>(Systems& sys) {
    return sys.receiver.channel;
}

template <>
inline decltype(std::get<CONFIG_struct::PID_PARAMETERS>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::PID_PARAMETERS>(Systems& sys) {
    return sys.control.pid_parameters;
}

template <>
inline decltype(std::get<CONFIG_struct::STATE_PARAMETERS>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::STATE_PARAMETERS>(Systems& sys) {
    return sys.state.parameters;
}

template <>
inline decltype(std::get<CONFIG_struct::LED_STATES>(CONFIG_struct::Data())) & systemMapping<CONFIG_struct::LED_STATES>(Systems& sys) {
    return sys.led.states;
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
        CONFIG.readFrom(EEPROMCursor());
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

CONFIG_struct::CONFIG_struct(Systems& sys) {
    setFieldsFrom(data, sys);
}

void CONFIG_struct::resetPartial(uint16_t submask, uint16_t led_mask) {
    resetFields(data, submask, led_mask);
}

void CONFIG_struct::applyTo(Systems& systems) const {
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

bool CONFIG_struct::verify() const {
    return verifyArgs(data);
}
