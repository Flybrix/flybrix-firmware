/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "ledDriver.h"

LEDDriver LED_driver;

LEDDriver::LEDDriver() {
    setColor(CRGB::Black);
    setPattern(LEDPattern::SOLID);
    FastLED.addLeds<WS2812B, board::led::DATA_PIN>(leds, board::led::COUNT);
}

uint8_t LEDDriver::getCycleIndex() const {
    return cycleIndex;
}

uint8_t LEDDriver::getPattern() const {
    return pattern;
}

inline bool isInside(const board::led::Position& p, const board::led::Position& p_min, const board::led::Position& p_max) {
    return p.x >= p_min.x && p.y >= p_min.y && p.x <= p_max.x && p.y <= p_max.y;
}

void LEDDriver::setColor(CRGB color, board::led::Position lower_left, board::led::Position upper_right) {
    color = CRGB(color.green, color.red, color.blue);
    for (size_t idx = 0; idx < board::led::COUNT; ++idx) {
        if (!isInside(board::led::POSITION[idx], lower_left, upper_right))
            continue;
        if (colors[idx].red == color.red && colors[idx].green == color.green && colors[idx].blue == color.blue)
            continue;
        hasChanges = true;
        colors[idx] = color;
    }
}

void LEDDriver::setPattern(LEDPattern::Pattern pattern) {
    if (pattern == this->pattern)
        return;
    this->pattern = pattern;
    hasChanges = true;
    cycleIndex = 255;
    scale = 255;
    for (size_t idx = 0; idx < board::led::COUNT; ++idx) {
        shining[idx] = true;
    }
}

void LEDDriver::set(LEDPattern::Pattern pattern, CRGB color) {
    setColor(color);
    setPattern(pattern);
}

void LEDDriver::update() {
    ++cycleIndex;
    writeToDisplay();
    if (!hasChanges)
        return;
    for (size_t idx = 0; idx < board::led::COUNT; ++idx) {
        leds[idx] = shining[idx] ? colors[idx] : CRGB::Black;
    }
    FastLED.show(scale);
    hasChanges = false;
}

void LEDDriver::updateAlternate() {
    if (cycleIndex & 15) {
        return;
    }
    board::led::Position lower_left = board::led::Position::Min();
    board::led::Position upper_right = board::led::Position::Max();
    if (cycleIndex & 16) {
        lower_left.x = 0;
    } else {
        upper_right.x = 0;
    }

    for (size_t idx = 0; idx < board::led::COUNT; ++idx) {
        shining[idx] = isInside(board::led::POSITION[idx], lower_left, upper_right);
    }
    hasChanges = true;
}

void LEDDriver::updateFlash() {
    if (cycleIndex & 3)
        return;
    scale = (cycleIndex & 4) ? 0 : 255;
    hasChanges = true;
}

void LEDDriver::updateBeacon() {
    switch ((cycleIndex & 63) >> 2) {  // two second period
        case 1:
        case 4:
            scale = 255;
            hasChanges = true;
            break;
        case 2:
        case 5:
            scale = 0;
            hasChanges = true;
            break;
        default:
            break;
    }
}

void LEDDriver::updateBreathe() {
    uint16_t multiplier = cycleIndex & 127;
    if (multiplier > 31)
        return;
    scale = multiplier;
    if (scale > 15)
        scale = 31 - scale;
    scale <<= 4;
    hasChanges = true;
}

void LEDDriver::updateSolid() {
    if (scale == 255)
        return;
    scale = 255;
    hasChanges = true;
}

void LEDDriver::writeToDisplay() {
    switch (pattern) {
        case LEDPattern::FLASH:
            updateFlash();
            break;
        case LEDPattern::BEACON:
            updateBeacon();
            break;
        case LEDPattern::BREATHE:
            updateBreathe();
            break;
        case LEDPattern::ALTERNATE:
            updateAlternate();
            break;
        case LEDPattern::SOLID:
            updateSolid();
            break;
    }
}
