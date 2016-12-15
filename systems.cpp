/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <systems.h/cpp>

    Set of systems that form the controller.
*/

#include "systems.h"
#include "config.h"

Systems::Systems()
    : receiver{},
      i2c{},
      state{},
      led{&state},
      bmp{&state, &i2c},  // pressure sensor object
      mpu{&state, &i2c},  // inertial sensor object
      mag{&state, &i2c},  // magnetometer
      pwr{&state},        // onboard power monitoring object
      motors{&state},     // eight PWM channels
      airframe{&state},
      pilot{&state, &receiver},
      control{&state, Control::PIDParameters()},
      // listen for configuration inputs
      conf{&state, RX, &control, this, &led, &pilot},
      id{0} {
    Config().applyTo(*this);
}

void Systems::parseConfig() {
    led.parseConfig();
    control.parseConfig();
}
