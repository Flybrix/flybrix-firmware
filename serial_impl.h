#ifndef SERIAL_IMPL_H
#define SERIAL_IMPL_H

#include "serial.h"

#include "cardManagement.h"
#include "config.h"
#include "eepromcursor.h"
#include "serialFork.h"

template <std::size_t N>
inline void WriteProtocolHead(SerialComm::MessageType type, uint32_t mask, CobsPayload<N>& payload) {
    payload.Append(type);
    payload.Append(mask);
}

template <std::size_t N>
inline void WriteToOutput(CobsPayload<N>& payload, bool use_logger = false) {
    auto package = payload.Encode();
    if (use_logger) {
        sdcard::writing::write(package.data, package.length);
    } else {
        writeSerial(package.data, package.length);
    }
}

template <std::size_t N>
inline void WritePIDData(CobsPayload<N>& payload, const PID& pid) {
    payload.Append(pid.lastTime(), pid.input(), pid.setpoint(), pid.pTerm(), pid.iTerm(), pid.dTerm());
}

#include "serial_subcommands.h"
#include "serial_substates.h"

#endif /* SERIAL_IMPL_H */
