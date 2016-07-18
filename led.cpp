/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "led.h"
#include "state.h"

void (*LEDFastUpdate)(){nullptr};

namespace {
class LEDDriver {
   public:
    LEDDriver();
    void setPattern(LED::Pattern pattern);
    void setColor(CRGB color, board::led::Position lower_left = {-128, -128}, board::led::Position upper_right = {127, 127});
    void set(LED::Pattern pattern, CRGB color);
    void update();  // fire this off at 30Hz

    uint8_t getCycleIndex() const;
    uint8_t getPattern() const;

   private:
    void writeToDisplay();
    void updateBeacon();   // 2sec periodic double pulse
    void updateFlash();    //~3Hz flasher
    void updateBreathe();  // 4sec periodic breathe
    void updateSolid();    // maintain constant light level

    uint8_t cycleIndex{0};
    uint8_t pattern;
    uint8_t scale{0};
    CRGB leds[board::led::COUNT];
    bool hasChanges{true};
} LED_driver;

uint8_t scrambleCycle(uint8_t x) {
    x = (((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    x = (((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    return (x >> 4) | (x << 4);
}

inline uint8_t scaleLight(uint8_t light, uint8_t scale) {
    return (static_cast<uint16_t>(light) * static_cast<uint16_t>(scale)) >> 8;
}

LEDDriver::LEDDriver() {
    setColor(CRGB::Black);
    setPattern(LED::SOLID);
#ifndef ALPHA
    FastLED.addLeds<WS2812B, board::led::DATA_PIN>(leds, board::led::COUNT);
#else
    pinMode(board::LED_A_RED, OUTPUT);
    pinMode(board::LED_A_GRN, OUTPUT);
    pinMode(board::LED_A_BLU, OUTPUT);
    pinMode(board::LED_B_RED, OUTPUT);
    pinMode(board::LED_B_GRN, OUTPUT);
    pinMode(board::LED_B_BLU, OUTPUT);
    LEDFastUpdate = []() {
        static uint8_t cycle;
        uint8_t threshold = scrambleCycle(++cycle);
        digitalWriteFast(board::LED_A_RED, (scaleLight(LED_driver.leds[0].red, LED_driver.scale) > threshold) ? LOW : HIGH);
        digitalWriteFast(board::LED_B_RED, (scaleLight(LED_driver.leds[1].red, LED_driver.scale) > threshold) ? LOW : HIGH);
        digitalWriteFast(board::LED_A_GRN, (scaleLight(LED_driver.leds[0].green, LED_driver.scale) > threshold) ? LOW : HIGH);
        digitalWriteFast(board::LED_B_GRN, (scaleLight(LED_driver.leds[1].green, LED_driver.scale) > threshold) ? LOW : HIGH);
        digitalWriteFast(board::LED_A_BLU, (scaleLight(LED_driver.leds[0].blue, LED_driver.scale) > threshold) ? LOW : HIGH);
        digitalWriteFast(board::LED_B_BLU, (scaleLight(LED_driver.leds[1].blue, LED_driver.scale) > threshold) ? LOW : HIGH);
    };
#endif
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
#ifndef ALPHA
    color = CRGB(color.green, color.red, color.blue);
#endif
    for (size_t idx = 0; idx < board::led::COUNT; ++idx) {
        if (!isInside(board::led::POSITION[idx], lower_left, upper_right))
            continue;
        if (leds[idx].red == color.red && leds[idx].green == color.green && leds[idx].blue == color.blue)
            continue;
        hasChanges = true;
        leds[idx] = color;
    }
}

void LEDDriver::setPattern(LED::Pattern pattern) {
    if (pattern == this->pattern)
        return;
    this->pattern = pattern;
    hasChanges = true;
    cycleIndex = 255;
}

void LEDDriver::set(LED::Pattern pattern, CRGB color) {
    setColor(color);
    setPattern(pattern);
}

void LEDDriver::update() {
    ++cycleIndex;
    writeToDisplay();
    if (!hasChanges)
        return;
#ifndef ALPHA
    FastLED.show(scale);
#else
    if (LEDFastUpdate)
        LEDFastUpdate();
#endif
    hasChanges = false;
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
        // Alternate is handled outside of the driver
        // and here it's just a solid light
        case LED::SOLID:
            updateSolid();
            break;
    }
}
}  // namespace

LED::LED(State* __state) : state(__state) {
    // indicator leds are not inverted
    pinMode(board::GREEN_LED, OUTPUT);
    pinMode(board::RED_LED, OUTPUT);
}

void LED::set(Pattern pattern, uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b, bool red_indicator, bool green_indicator) {
    set(pattern, {red_a, green_a, blue_a}, {red_b, green_b, blue_b}, red_indicator, green_indicator);
}

void LED::set(Pattern pattern, CRGB color_right, CRGB color_left, bool red_indicator, bool green_indicator) {
    set(pattern, color_right, color_right, color_left, color_left, red_indicator, green_indicator);
}

void LED::set(Pattern pattern, CRGB color_right_front, CRGB color_right_back, CRGB color_left_front, CRGB color_left_back, bool red_indicator, bool green_indicator) {
    override = pattern != LED::NO_OVERRIDE;
    oldStatus = 0;
    if (!override)
        return;
    use(pattern, color_right_front, color_right_back, color_left_front, color_left_back, red_indicator, green_indicator);
}

void LED::set(Pattern pattern, CRGB color, bool red_indicator, bool green_indicator) {
    set(pattern, color, color, red_indicator, green_indicator);
}

void LED::update() {
    if (!override && oldStatus != state->status) {
        oldStatus = state->status;
        changeLights();
    }
    if (LED_driver.getPattern() == LED::ALTERNATE && !(LED_driver.getCycleIndex() & 15)) {
        if (LED_driver.getCycleIndex() & 16) {
            LED_driver.setColor(color_right_front, {0, 0}, {127, 127});
            LED_driver.setColor(color_right_back, {0, -128}, {127, 0});
            LED_driver.setColor(CRGB::Black, {-128, -128}, {0, 127});
        } else {
            LED_driver.setColor(CRGB::Black, {0, -128}, {127, 127});
            LED_driver.setColor(color_left_front, {-128, 0}, {0, 127});
            LED_driver.setColor(color_left_back, {-128, -128}, {0, 0});
        }
    }
    LED_driver.update();
}

void LED::use(Pattern pattern, CRGB color_right_front, CRGB color_right_back, CRGB color_left_front, CRGB color_left_back, bool red_indicator, bool green_indicator) {
    this->color_right_front = color_right_front;
    this->color_right_back = color_right_back;
    this->color_left_front = color_left_front;
    this->color_left_back = color_left_back;
    red_indicator ? indicatorRedOn() : indicatorRedOff();
    green_indicator ? indicatorGreenOn() : indicatorGreenOff();
    LED_driver.setPattern(pattern);
    LED_driver.setColor(color_right_front, {0, 0}, {127, 127});
    LED_driver.setColor(color_right_back, {0, -128}, {127, 0});
    LED_driver.setColor(color_left_front, {-128, 0}, {0, 127});
    LED_driver.setColor(color_left_back, {-128, -128}, {0, 0});
}

void LED::setWhite(board::led::Position lower_left, board::led::Position upper_right, bool red_indicator, bool green_indicator) {
    color_right_front = color_right_back = color_left_front = color_left_back = CRGB::Black;
    override = true;
    oldStatus = 0;
    red_indicator ? indicatorRedOn() : indicatorRedOff();
    green_indicator ? indicatorGreenOn() : indicatorGreenOff();
    LED_driver.setPattern(LED::SOLID);
    LED_driver.setColor(CRGB::Black);
    LED_driver.setColor(CRGB::White, lower_left, upper_right);
}

void LED::changeLights() {
    for (const StateCase& s : states.states) {
        if (!s.status || state->is(s.status)) {
            use(s.pattern, s.color_right_front.crgb(), s.color_right_back.crgb(), s.color_left_front.crgb(), s.color_left_back.crgb(), s.indicator_red, s.indicator_green);
            return;
        }
    }
    use(LED::ALTERNATE, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red, true, false);  // No status bits set
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
