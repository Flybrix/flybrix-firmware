/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef led_driver_h
#define led_driver_h

#include "board.h"
#define FASTLED_INTERNAL
#include "FastLED.h"

namespace LEDPattern {
enum Pattern : uint8_t {
    NO_OVERRIDE = 0,
    FLASH = 1,
    BEACON = 2,
    BREATHE = 3,
    ALTERNATE = 4,
    SOLID = 5,
};
}

class LEDDriver {
   public:
    LEDDriver();

    void setPattern(LEDPattern::Pattern pattern);
    void setColor(CRGB color, board::led::Position lower_left = {-128, -128}, board::led::Position upper_right = {127, 127});
    void set(LEDPattern::Pattern pattern, CRGB color);
    void update();  // fire this off at 30Hz

    uint8_t getCycleIndex() const;
    uint8_t getPattern() const;

   private:
    void writeToDisplay();
    void updateAlternate();
    void updateBeacon();   // 2sec periodic double pulse
    void updateFlash();    //~3Hz flasher
    void updateBreathe();  // 4sec periodic breathe
    void updateSolid();    // maintain constant light level

    uint8_t cycleIndex{0};
    uint8_t pattern;
    uint8_t scale{0};
    CRGB colors[board::led::COUNT];
    CRGB leds[board::led::COUNT];
    bool shining[board::led::COUNT];
    bool hasChanges{true};
};

extern LEDDriver LED_driver;

#endif  // led_driver_h
