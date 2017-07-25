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
        // 0 means timestamp, otherwise it is a command
        if (sdcard::reading::read()) {
        } else {
            uint8_t a = sdcard::reading::read();
            uint8_t b = sdcard::reading::read();
            uint8_t c = sdcard::reading::read();
            uint8_t d = sdcard::reading::read();
            wait_until_ = (a << 24) + (b << 16) + (c << 8) + d;
        }
    }
    running_ = false;
}

void Autopilot::readCobs() {
    while (sdcard::reading::hasMore()) {
        data_input.AppendToBuffer(sdcard::reading::read());
        if (data_input.IsDone()) {
            serial_.ProcessData(data_input, false);
            break;
        }
    }
}
