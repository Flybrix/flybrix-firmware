#include "autopilot.h"

#include "serial.h"
#include "cardManagement.h"

Autopilot::Autopilot(SerialComm& serial) : serial_(serial) {
}

void Autopilot::start(uint32_t now) {
    sdcard::reading::close();
    sdcard::reading::open();
    running_ = true;
    start_time_ = now;
    wait_until_ = 0;
}

void Autopilot::run(uint32_t now) {
    if (!running_) {
        return;
    }
    while (sdcard::reading::hasMore()) {
        if (wait_until_ > now - start_time_) {
            return;
        }
        readCobs();
    }
    running_ = false;
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
