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

class State;

class LED {
   public:
    LED(State *state);

    enum Pattern : uint8_t {
        NO_OVERRIDE = 0,
        FLASH = 1,
        BEACON = 2,
        BREATHE = 3,
        ALTERNATE = 4,
        SOLID = 5,
    };

    void set(Pattern pattern, uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b, bool red_indicator, bool green_indicator);

    void update();  // update using the state STATUS bitfield

   private:
    State *state;
    uint16_t oldStatus{0};
    uint8_t lightType{0};
    uint8_t red_a_{0}, green_a_{0}, blue_a_{0};
    uint8_t red_b_{0}, green_b_{0}, blue_b_{0};

    uint16_t cycleIndex{0};  // incremented at 500Hz to support dithering, resets every ~8 seconds

    bool override{false};

    uint8_t getLightThreshold();

    void changeLights();

    void updateBeacon();     // 2sec periodic double pulse
    void updateFlash();      //~3Hz flasher
    void updateBreathe();    // 2sec periodic breathe
    void updateAlternate();  //~4Hz left/right alternating
    void updateSolid();      // maintain constant light level
    void rgb();              // dithered color
    void rgb(uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b);
    void use(Pattern pattern, uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b);
    void use(Pattern pattern, uint8_t red, uint8_t green, uint8_t blue);

    void allOff();

    void indicatorRedOn();
    void indicatorGreenOn();
    void indicatorRedOff();
    void indicatorGreenOff();

};  // class LED

#endif
