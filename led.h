/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <led.h/cpp>

    Manages all leds.

*/

#ifndef led_h
#define led_h

#include "Arduino.h"
#define FASTLED_INTERNAL
#include "FastLED.h"

class State;

class LED {
   public:
    enum Pattern : uint8_t {
        NO_OVERRIDE = 0,
        FLASH = 1,
        BEACON = 2,
        BREATHE = 3,
        ALTERNATE = 4,
        SOLID = 5,
    };

    explicit LED(State *state);

    void update();

    void set(Pattern pattern, uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b, bool red_indicator, bool green_indicator);

    void set(Pattern pattern, CRGB color_right, CRGB color_left, bool red_indicator, bool green_indicator);
    void set(Pattern pattern, CRGB color, bool red_indicator = false, bool green_indicator = false);

   private:
    void use(Pattern pattern, CRGB color_right, CRGB color_left, bool red_indicator, bool green_indicator);
    void changeLights();

    void indicatorRedOn();
    void indicatorGreenOn();
    void indicatorRedOff();
    void indicatorGreenOff();

    State *state;
    uint16_t oldStatus{0};
    bool override{false};

    CRGB colorRight{CRGB::Black}, colorLeft{CRGB::Black};
};

#endif
