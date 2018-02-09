/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef led_h
#define led_h

#include "Arduino.h"
#define FASTLED_INTERNAL
#include "FastLED.h"

#include "board.h"
#include "ledDriver.h"

class StateFlag;

class LED {
   public:
    explicit LED(StateFlag& flag);

    void update();

    void set(LEDPattern::Pattern pattern, uint8_t red_a, uint8_t green_a, uint8_t blue_a, uint8_t red_b, uint8_t green_b, uint8_t blue_b, bool red_indicator, bool green_indicator);

    void set(LEDPattern::Pattern pattern, CRGB color_right_front, CRGB color_right_back, CRGB color_left_front, CRGB color_left_back, bool red_indicator, bool green_indicator);
    void set(LEDPattern::Pattern pattern, CRGB color_right, CRGB color_left, bool red_indicator, bool green_indicator);
    void set(LEDPattern::Pattern pattern, CRGB color, bool red_indicator = false, bool green_indicator = false);

    void errorStart(LEDPattern::Pattern pattern, CRGB color_back, CRGB color_front, uint8_t count, bool fade = true);
    void errorStop();

    void setWhite(board::led::Position lower_left = board::led::Position::Min(), board::led::Position upper_right = board::led::Position::Max(), bool red_indicator = false,
                  bool green_indicator = false, uint8_t fading = 0);

    struct __attribute__((packed)) Color {
        Color() : Color(CRGB::Black) {
        }
        Color(CRGB color) : red(color.r), green(color.g), blue(color.b) {
        }
        CRGB crgb() const {
            return CRGB(red, green, blue);
        }
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };

    static_assert(sizeof(Color) == 3, "Data is not packed");

    struct __attribute__((packed)) StateCase {
        StateCase(uint16_t status, LEDPattern::Pattern pattern, CRGB color_right_front, CRGB color_right_back, CRGB color_left_front, CRGB color_left_back, bool indicator_red = false,
                  bool indicator_green = false)
            : status{status},
              pattern{pattern},
              color_right_front{color_right_front},
              color_right_back{color_right_back},
              color_left_front{color_left_front},
              color_left_back{color_left_back},
              indicator_red{indicator_red},
              indicator_green{indicator_green} {
        }
        StateCase(uint16_t status, LEDPattern::Pattern pattern, CRGB color_right, CRGB color_left, bool indicator_red = false, bool indicator_green = false)
            : StateCase{status, pattern, color_right, color_right, color_left, color_left, indicator_red, indicator_green} {
        }

        StateCase(uint16_t status, LEDPattern::Pattern pattern, CRGB color) : StateCase{status, pattern, color, color, false, false} {
        }

        StateCase() : StateCase{0, LEDPattern::Pattern::NO_OVERRIDE, CRGB::Black} {
        }

        uint16_t status;
        LEDPattern::Pattern pattern;
        Color color_right_front;
        Color color_right_back;
        Color color_left_front;
        Color color_left_back;
        bool indicator_red;
        bool indicator_green;
    };

    static_assert(sizeof(StateCase) == 2 + 1 + 4 * sizeof(Color) + 2, "Data is not packed");

    struct __attribute__((packed)) States {
        States();
        bool verify() const {
            return true;
        }
        StateCase states[16];
    } states;

    static_assert(sizeof(States) == 16 * sizeof(StateCase), "Data is not packed");

    void parseConfig();

   private:
    void use(LEDPattern::Pattern pattern, CRGB color_right_front, CRGB color_right_back, CRGB color_left_front, CRGB color_left_back, bool red_indicator, bool green_indicator);
    void changeLights();

    void indicatorRedOn();
    void indicatorGreenOn();
    void indicatorRedOff();
    void indicatorGreenOff();

    StateFlag& flag_;
    uint16_t oldStatus{0};
    bool error_raised{false};
    bool override{false};

    CRGB color_right_front{CRGB::Black};
    CRGB color_right_back{CRGB::Black};
    CRGB color_left_front{CRGB::Black};
    CRGB color_left_back{CRGB::Black};
};

#endif
