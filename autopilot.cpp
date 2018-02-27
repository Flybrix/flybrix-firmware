/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "autopilot.h"

#include "cardManagement.h"
#include "serial.h"

Autopilot::Autopilot(SerialComm& serial) : serial_(serial) {
}

void Autopilot::start(ClockTime now) {
    sdcard::reading::close();
    sdcard::reading::open();
    running_ = true;
    start_time_ = now;
    wait_until_ = 0;
}

bool Autopilot::run(ClockTime now) {
    bool did_something{false};
    if (!running_) {
        return did_something;
    }
    while (sdcard::reading::hasMore()) {
        if (wait_until_ > now - start_time_) {
            return did_something;
        }
        readCobs();
        did_something = true;
    }
    running_ = false;
    return did_something;
}

void Autopilot::stop() {
    running_ = false;
    sdcard::reading::close();
}

void Autopilot::readCobs() {
    while (sdcard::reading::hasMore()) {
        data_input.AppendToBuffer(sdcard::reading::read());
        if (data_input.IsDone()) {
            handleCobs();
            break;
        }
    }
}

void Autopilot::handleCobs() {
    SerialComm::MessageType command;
    if (!data_input.PeekInto(command)) {
        return;
    }
    if (SerialComm::MessageType::AutopilotWait == command) {
        data_input.ParseInto(command, wait_until_);
    } else {
        serial_.ProcessData(data_input, false);
    }
}
