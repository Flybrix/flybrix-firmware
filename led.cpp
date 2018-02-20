/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "led.h"
#include "stateFlag.h"

inline CRGB fadeBy(CRGB color, uint8_t amount) {
    return color.fadeLightBy(amount);
}

CRGB LED::fade(CRGB color) {
    return fadeBy(color, 230);
}

const CRGB BATTERY_CRITICAL_LIGHT = LED::fade(CRGB::Red);
const CRGB BATTERY_LOW_LIGHT = LED::fade(CRGB::Orange);
const CRGB RECORDING_SD_LIGHT = LED::fade(CRGB::DarkMagenta);

// fading is in 256ths : https://github.com/FastLED/FastLED/wiki/Pixel-reference
LED::States::States()
    : states{
          LED::StateCase(Status::CRASH_DETECTED, LEDPattern::FLASH, fade(CRGB::Orange)),
          LED::StateCase(Status::LOOP_SLOW, LEDPattern::SOLID, fade(CRGB::Red)),
          LED::StateCase(Status::OVERRIDE, LEDPattern::SOLID, fade(CRGB::LightSeaGreen)),
          LED::StateCase(Status::LOG_FULL, LEDPattern::FLASH, fade(CRGB::Orange)),
          LED::StateCase(Status::NO_SIGNAL, LEDPattern::FLASH, fade(CRGB::Green)),
          LED::StateCase(Status::ARMING, LEDPattern::FLASH, fade(CRGB::Blue)),
          LED::StateCase(Status::ARMED, LEDPattern::BEACON, fade(CRGB::Blue)),
          LED::StateCase(Status::IDLE, LEDPattern::BREATHE, fade(CRGB::Green)),
      } {
}

LED::LED(StateFlag& flag) : flag_(flag) {
    // indicator leds are not inverted
    pinMode(board::GREEN_LED, OUTPUT);
    pinMode(board::RED_LED, OUTPUT);
}

void LED::set(LEDPattern::Pattern pattern, uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b, bool red_indicator, bool green_indicator) {
    set(pattern, {red_a, green_a, blue_a}, {red_b, green_b, blue_b}, red_indicator, green_indicator);
}

void LED::set(LEDPattern::Pattern pattern, CRGB color_right, CRGB color_left, bool red_indicator, bool green_indicator) {
    set(pattern, color_right, color_right, color_left, color_left, red_indicator, green_indicator);
}

void LED::set(LEDPattern::Pattern pattern, CRGB color_right_front, CRGB color_right_back, CRGB color_left_front, CRGB color_left_back, bool red_indicator, bool green_indicator) {
    override = pattern != LEDPattern::NO_OVERRIDE;
    oldStatus = 0;
    if (!override)
        return;
    use(pattern, color_right_front, color_right_back, color_left_front, color_left_back, red_indicator, green_indicator);
}

void LED::set(LEDPattern::Pattern pattern, CRGB color, bool red_indicator, bool green_indicator) {
    set(pattern, color, color, red_indicator, green_indicator);
}

void LED::errorStart(LEDPattern::Pattern pattern, CRGB color_back, CRGB color_front, uint8_t count) {
    error_raised = true;
    switch (count) {
        case 0: {
            use(pattern, color_back, color_back, color_back, color_back, false, false);
        } break;
        case 1: {
            use(pattern, color_front, color_back, color_back, color_back, false, false);
        } break;
        case 2: {
            use(pattern, color_front, color_front, color_back, color_back, false, false);
        } break;
        case 3: {
            use(pattern, color_front, color_front, color_back, color_front, false, false);
        } break;
        case 4: {
            use(pattern, color_front, color_front, color_front, color_front, false, false);
        } break;
        default: { use(pattern, color_front, color_front, color_front, color_front, false, false); } break;
    }
    update();
}

void LED::errorStop() {
    error_raised = false;
    oldStatus = 0;
}

void LED::update() {
    if (!error_raised && !override && oldStatus != flag_.value()) {
        oldStatus = flag_.value();
        changeLights();
    }
    LED_driver.update();
}

void LED::forceRedIndicator(bool value) {
    force_red = value;
    updateIndicators();
}

void LED::forceGreenIndicator(bool value) {
    force_green = value;
    updateIndicators();
}

void LED::updateIndicators() {
    if (force_red || enabled_red) {
        indicatorRedOn();
    } else {
        indicatorRedOff();
    }
    if (force_green || enabled_green) {
        indicatorGreenOn();
    } else {
        indicatorGreenOff();
    }
}

void LED::use(LEDPattern::Pattern pattern, CRGB color_right_front, CRGB color_right_back, CRGB color_left_front, CRGB color_left_back, bool red_indicator, bool green_indicator) {
    enabled_red = red_indicator;
    enabled_green = green_indicator;
    updateIndicators();
    LED_driver.setPattern(pattern);
    LED_driver.setColor(color_right_front, {0, 0}, {127, 127});
    LED_driver.setColor(color_right_back, {0, -128}, {127, 0});
    LED_driver.setColor(color_left_front, {-128, 0}, {0, 127});
    LED_driver.setColor(color_left_back, {-128, -128}, {0, 0});
}

void LED::setWhite(board::led::Position lower_left, board::led::Position upper_right, bool red_indicator, bool green_indicator, uint8_t fading) {
    enabled_red = red_indicator;
    enabled_green = green_indicator;
    override = true;
    oldStatus = 0;
    updateIndicators();
    LED_driver.setPattern(LEDPattern::SOLID);
    LED_driver.setColor(CRGB::Black);
    LED_driver.setColor(fadeBy(CRGB::White, fading), lower_left, upper_right);
}

void LED::changeLights() {
    for (const StateCase& s : states.states) {
        if (!s.status || flag_.is(s.status)) {
            bool any_overridable = flag_.is(Status::IDLE | Status::NO_SIGNAL | Status::ARMING | Status::ARMED);
            if (any_overridable && flag_.is(Status::BATTERY_CRITICAL)) {
                use(s.pattern, BATTERY_CRITICAL_LIGHT, BATTERY_CRITICAL_LIGHT, BATTERY_CRITICAL_LIGHT, BATTERY_CRITICAL_LIGHT, s.indicator_red, s.indicator_green);
            } else if (any_overridable && flag_.is(Status::BATTERY_LOW)) {
                use(s.pattern, BATTERY_LOW_LIGHT, BATTERY_LOW_LIGHT, BATTERY_LOW_LIGHT, BATTERY_LOW_LIGHT, s.indicator_red, s.indicator_green);
            } else if (flag_.is(Status::IDLE | Status::NO_SIGNAL) && flag_.is(Status::RECORDING_SD)) {
                use(s.pattern, RECORDING_SD_LIGHT, RECORDING_SD_LIGHT, RECORDING_SD_LIGHT, RECORDING_SD_LIGHT, s.indicator_red, s.indicator_green);
            } else {
                use(s.pattern, s.color_right_front.crgb(), s.color_right_back.crgb(), s.color_left_front.crgb(), s.color_left_back.crgb(), s.indicator_red, s.indicator_green);
            }
            return;
        }
    }
    use(LEDPattern::ALTERNATE, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red, true, false);  // No status bits set
}

void LED::parseConfig() {
    LED_driver.setPattern(LEDPattern::NO_OVERRIDE);
    oldStatus = ~flag_.value();
    update();
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
