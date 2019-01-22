/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef SERIAL_IMPL_H
#define SERIAL_IMPL_H

#include "serial.h"

#include "cardManagement.h"
#include "config.h"
#include "eepromcursor.h"
#include "serialFork.h"
#include "stateFlag.h"

template <std::size_t N>
inline void WriteProtocolHead(SerialComm::MessageType type, uint32_t mask, CobsPayload<N>& payload) {
    payload.Append(type);
    payload.Append(mask);
}

template <std::size_t N>
void SerialComm::WriteToOutput(CobsPayload<N>& payload, bool use_logger) const {
    auto package = payload.Encode();
    if (use_logger) {
        sdcard::writing::write(package.data, package.length);
        flag_.assign(Status::LOG_FULL, sdcard::writing::fileIsFull());
    } else {
        writeSerial(package.data, package.length);
    }
}

template <std::size_t N>
inline void WriteDebugToOutput(CobsPayload<N>& payload) {
    auto package = payload.Encode();
    // decide where to route Debug info here
    writeSerial(package.data, package.length);
}

template <std::size_t N>
inline void WritePIDData(CobsPayload<N>& payload, const PID& pid) {
    payload.Append(pid.lastTime(), pid.input(), pid.setpoint(), pid.p(), pid.i(), pid.d());
}

#include "serial_subcommands.h"
#include "serial_substates.h"

#endif /* SERIAL_IMPL_H */
