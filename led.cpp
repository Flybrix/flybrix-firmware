/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "led.h"
#include "board.h"
#include "state.h"

LED::LED(State* __state) : state(__state) {
    // RGB leds A and B are inverted
    pinMode(board::LED_A_RED, OUTPUT);
    pinMode(board::LED_A_GRN, OUTPUT);
    pinMode(board::LED_A_BLU, OUTPUT);
    pinMode(board::LED_B_RED, OUTPUT);
    pinMode(board::LED_B_GRN, OUTPUT);
    pinMode(board::LED_B_BLU, OUTPUT);

    // indicator leds are not inverted
    pinMode(board::GREEN_LED, OUTPUT);
    pinMode(board::RED_LED, OUTPUT);
}

void LED::set(Pattern pattern, uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b, bool red_indicator, bool green_indicator) {
    override = pattern != LED::NO_OVERRIDE;
    if (!override)
        return;
    oldStatus = 0;
    red_indicator ? indicatorRedOn() : indicatorRedOff();
    green_indicator ? indicatorGreenOn() : indicatorGreenOff();
    use(pattern, red_a, green_a, blue_a, red_b, green_b, blue_b);
}

void LED::update() {
    cycleIndex++;        // 500Hz update
    cycleIndex &= 4095;  // ~8 second period

    if (!override && oldStatus != state->status) {
        oldStatus = state->status;
        changeLights();
    }

    switch (lightType) {
        case LED::FLASH:
            updateFlash();
            break;
        case LED::BEACON:
            updateBeacon();
            break;
        case LED::BREATHE:
            updateBreathe();
            break;
        case LED::ALTERNATE:
            updateAlternate();
            break;
        case LED::SOLID:
            updateSolid();
            break;
    }
}

void LED::use(Pattern pattern, uint8_t red, uint8_t green, uint8_t blue) {
    use(pattern, red, green, blue, red, green, blue);
}

void LED::use(Pattern pattern, uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b) {
    lightType = pattern;
    this->red_a_ = red_a;
    this->green_a_ = green_a;
    this->blue_a_ = blue_a;
    this->red_b_ = red_b;
    this->green_b_ = green_b;
    this->blue_b_ = blue_b;
}

void LED::changeLights() {
    lightType = 0;

    if (state->is(STATUS_MPU_FAIL)) {
        allOff();  // no dithering during setup!
        digitalWriteFast(board::LED_A_RED, LOW);
        indicatorRedOn();
    } else if (state->is(STATUS_BMP_FAIL)) {
        allOff();  // no dithering during setup!
        digitalWriteFast(board::LED_B_RED, LOW);
        indicatorRedOn();
    } else if (state->is(STATUS_BOOT)) {
        allOff();  // no dithering during setup!
        digitalWriteFast(board::LED_A_GRN, LOW);
        digitalWriteFast(board::LED_B_GRN, LOW);
    } else if (state->is(STATUS_UNPAIRED)) {
        // use(LED::FLASH, 255,180,20); //orange
        use(LED::FLASH, 255, 255, 255);  // for pcba testing purposes -- a "good" board will end up in this state
        indicatorGreenOn();              // for pcba testing purposes -- a "good" board will end up in this state
    } else if (state->is(STATUS_RX_FAIL)) {
        use(LED::FLASH, 255, 0, 0);  // red
    } else if (state->is(STATUS_FAIL_STABILITY) || state->is(STATUS_FAIL_ANGLE)) {
        use(LED::FLASH, 100, 110, 250);  // yellow
    } else if (state->is(STATUS_OVERRIDE)) {
        use(LED::BEACON, 255, 0, 0);  // red
    } else if (state->is(STATUS_ENABLING)) {
        use(LED::FLASH, 0, 0, 255);  // blue
    } else if (state->is(STATUS_ENABLED)) {
        use(LED::BEACON, 0, 0, 255);  // blue for enable
    }

    else if (state->is(STATUS_TEMP_WARNING)) {
        use(LED::FLASH, 100, 150, 250);  // yellow
    } else if (state->is(STATUS_LOG_FULL)) {
        use(LED::FLASH, 0, 0, 250);
    } else if (state->is(STATUS_BATTERY_LOW)) {
        use(LED::BEACON, 255, 180, 20);
    } else if (state->is(STATUS_IDLE)) {
        indicatorRedOff();            // clear boot test
        use(LED::BEACON, 0, 255, 0);  // breathe instead?
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
    rgb(red_a_, green_a_, blue_a_, red_b_, green_b_, blue_b_);
}

void LED::rgb(uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b) {
    uint8_t threshold = getLightThreshold();
    digitalWriteFast(board::LED_A_RED, (red_a > threshold) ? LOW : HIGH);
    digitalWriteFast(board::LED_B_RED, (red_b > threshold) ? LOW : HIGH);
    digitalWriteFast(board::LED_A_GRN, (green_a > threshold) ? LOW : HIGH);
    digitalWriteFast(board::LED_B_GRN, (green_b > threshold) ? LOW : HIGH);
    digitalWriteFast(board::LED_A_BLU, (blue_a > threshold) ? LOW : HIGH);
    digitalWriteFast(board::LED_B_BLU, (blue_b > threshold) ? LOW : HIGH);
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

void LED::updateBreathe() {
    uint16_t multiplier = cycleIndex & 2047;
    if (cycleIndex > 511) {
        allOff();
        return;
    }
    if (multiplier > 255)
        multiplier = 512 - multiplier;

    rgb((multiplier * red_a_) >> 8, (multiplier * green_a_) >> 8, (multiplier * blue_a_) >> 8, (multiplier * red_b_) >> 8, (multiplier * green_b_) >> 8, (multiplier * blue_b_) >> 8);
}

void LED::updateAlternate() {
    if (cycleIndex & 128)
        rgb(red_a_, green_a_, blue_a_, 0, 0, 0);
    else
        rgb(0, 0, 0, red_b_, green_b_, blue_b_);
}

void LED::updateSolid() {
    rgb();
}

void LED::allOff() {
    digitalWriteFast(board::LED_A_RED, HIGH);
    digitalWriteFast(board::LED_A_GRN, HIGH);
    digitalWriteFast(board::LED_A_BLU, HIGH);
    digitalWriteFast(board::LED_B_RED, HIGH);
    digitalWriteFast(board::LED_B_GRN, HIGH);
    digitalWriteFast(board::LED_B_BLU, HIGH);
}

void LED::indicatorRedOn() {
    digitalWriteFast(board::RED_LED, HIGH);
}

void LED::indicatorGreenOn() {
    digitalWriteFast(board::GREEN_LED, HIGH);
}

void LED::indicatorRedOff() {
    digitalWriteFast(board::RED_LED, LOW);
}

void LED::indicatorGreenOff() {
    digitalWriteFast(board::GREEN_LED, LOW);
}
