/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "Arduino.h"
#include "led.h"
#include "motors.h"
#include "state.h"

void runTestRoutine(State& state, Motors& motors, size_t motor_id) {
    for (auto& motorOut : state.MotorOut) {
        motorOut = 0;
    }
    state.MotorOut[motor_id] = 4095;
    motors.updateAllChannels();
}

void runTestMode(State& state, LED& led, Motors& motors) {
    state.set(STATUS_ENABLED);
    size_t motor_id{0};
    led.setWhite(board::led::Position::Min(), board::led::Position::Max(), true, true, 250);
    led.update();
    while (true) {
        runTestRoutine(state, motors, motor_id);
        motor_id = (motor_id + 1) % 8;
        delay(500);
    }
}
