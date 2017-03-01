/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <testMode.h/cpp>

    Test mode for LED and motor functionality

*/

#include "Arduino.h"
#include "led.h"
#include "airframe.h"
#include "state.h"

void runTestRoutine(State& state, LED& led, Airframe& airframe, size_t led_id, size_t motor_id) {
    airframe.resetMotors();
    airframe.setMotor(motor_id, 4095);
    airframe.applyChanges(true);
    led.setWhite(board::led::POSITION[led_id], board::led::POSITION[led_id], led_id % 2 == 0, led_id % 2 == 1);
    led.update();
}

void runTestMode(State& state, LED& led, Airframe& airframe) {
    state.set(STATUS_ENABLED);
    size_t led_id{0};
    size_t motor_id{0};
    while (true) {
        runTestRoutine(state, led, airframe, led_id, motor_id);
        led_id = (led_id + 1) % 4;
        motor_id = (motor_id + 1) % 8;
        delay(500);
    }
}
