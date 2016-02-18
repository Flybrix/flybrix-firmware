/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

//#define LED_SERIAL_DEBUG

#include "led.h"
#include "state.h"

LED::LED(State* __state) : state(__state) {
    // RGB leds A and B are inverted
    pinMode(LED_A_RED, OUTPUT);
    pinMode(LED_A_GRN, OUTPUT);
    pinMode(LED_A_BLU, OUTPUT);
    pinMode(LED_B_RED, OUTPUT);
    pinMode(LED_B_GRN, OUTPUT);
    pinMode(LED_B_BLU, OUTPUT);

    // indicator leds are not inverted
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
}

void LED::update() {
    cycleIndex++;  // 500Hz update
    cycleIndex &= 4095;  // ~8 second period

    if (oldStatus != state->status) {
        oldStatus = state->status;
        changeLights();
    }

    if (lightType == 1) {
        updateFlash();
    } else if (lightType == 2) {
        updateBeacon();
    }
}

void LED::beacon(uint8_t red, uint8_t green, uint8_t blue) {
    lightType = 2;
    this->red = red;
    this->green = green;
    this->blue = blue;
}

void LED::flash(uint8_t red, uint8_t green, uint8_t blue) {
    lightType = 1;
    this->red = red;
    this->green = green;
    this->blue = blue;
}

void LED::changeLights() {
    lightType = 0;

    if (state->is(STATUS_MPU_FAIL)) {
        allOff();  // no dithering during setup!
        digitalWriteFast(LED_A_RED, LOW);
        indicatorRedOn();
    } else if (state->is(STATUS_BMP_FAIL)) {
        allOff();  // no dithering during setup!
        digitalWriteFast(LED_B_RED, LOW);
        indicatorRedOn();
    } else if (state->is(STATUS_BOOT)) {
        allOff();  // no dithering during setup!
        digitalWriteFast(LED_A_GRN, LOW);
        digitalWriteFast(LED_B_GRN, LOW);
    } else if (state->is(STATUS_UNPAIRED)) {
        // flash(255,180,20); //orange
        flash(255, 255, 255);  // for pcba testing purposes -- a "good" board will end up in this state
        indicatorGreenOn();    // for pcba testing purposes -- a "good" board will end up in this state
    } else if (state->is(STATUS_RX_FAIL)) {
        flash(255, 0, 0);  // red
    } else if (state->is(STATUS_FAIL_STABILITY) || state->is(STATUS_FAIL_ANGLE)) {
        flash(100, 110, 250);  // yellow
    } else if (state->is(STATUS_OVERRIDE)) {
        beacon(255, 0, 0);  // red
    } else if (state->is(STATUS_ENABLING)) {
        flash(0, 0, 255);  // blue
    } else if (state->is(STATUS_ENABLED)) {
        beacon(0, 0, 255);  // blue for enable
    }

    else if (state->is(STATUS_TEMP_WARNING)) {
        flash(100, 150, 250);  // yellow
    } else if (state->is(STATUS_LOG_FULL)) {
        flash(0, 0, 250);
    } else if (state->is(STATUS_BATTERY_LOW)) {
        beacon(255, 180, 20);
    } else if (state->is(STATUS_IDLE)) {
        indicatorRedOff();  // clear boot test
        beacon(0, 255, 0);  // breathe instead?
    } else {
        // ERROR: ("NO STATUS BITS SET???");
    }
}

uint8_t LED::getLightThreshold() {
    uint8_t x = cycleIndex & 0xFF;
    x = (((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (x >> 4) | (x << 4);
}

void LED::rgb() {
    uint8_t threshold = getLightThreshold();

    if (red > threshold) {
        digitalWriteFast(LED_A_RED, LOW);
        digitalWriteFast(LED_B_RED, LOW);
    } else {
        digitalWriteFast(LED_A_RED, HIGH);
        digitalWriteFast(LED_B_RED, HIGH);
    }

    if (green > threshold) {
        digitalWriteFast(LED_A_GRN, LOW);
        digitalWriteFast(LED_B_GRN, LOW);
    } else {
        digitalWriteFast(LED_A_GRN, HIGH);
        digitalWriteFast(LED_B_GRN, HIGH);
    }

    if (blue > threshold) {
        digitalWriteFast(LED_A_BLU, LOW);
        digitalWriteFast(LED_B_BLU, LOW);
    } else {
        digitalWriteFast(LED_A_BLU, HIGH);
        digitalWriteFast(LED_B_BLU, HIGH);
    }
}

void LED::updateFlash() {
    uint16_t cycleState = (cycleIndex & 511) >> 4;  // flash at ~3 HZ
    if (cycleState < 5 || (cycleState < 15 && cycleState > 9) || (cycleState < 26 && cycleState > 20))
        rgb();
    else
        allOff();
}

void LED::updateBeacon() {
    switch ((cycleIndex & 1023) >> 6)  // two second period
    {
        case 1:
        case 4:
            rgb();
            break;

        case 2:
        case 5:
            allOff();
            break;

        default:
            break;
    }
}

void LED::allOff() {
    digitalWriteFast(LED_A_RED, HIGH);
    digitalWriteFast(LED_A_GRN, HIGH);
    digitalWriteFast(LED_A_BLU, HIGH);
    digitalWriteFast(LED_B_RED, HIGH);
    digitalWriteFast(LED_B_GRN, HIGH);
    digitalWriteFast(LED_B_BLU, HIGH);
}

void LED::indicatorRedOn() {
    digitalWriteFast(RED_LED, HIGH);
}

void LED::indicatorGreenOn() {
    digitalWriteFast(GREEN_LED, HIGH);
}

void LED::indicatorRedOff() {
    digitalWriteFast(RED_LED, LOW);
}

void LED::indicatorGreenOff() {
    digitalWriteFast(GREEN_LED, LOW);
}
