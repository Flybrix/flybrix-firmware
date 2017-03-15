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
    : flag{},
      receiver{},
      i2c{},
      state{this},
      led{flag},
      bmp{&i2c},          // pressure sensor object
      mpu{&state, &i2c},  // inertial sensor object
      mag{&state, &i2c},  // magnetometer
      pwr{&state},        // onboard power monitoring object
      airframe{flag},
      pilot{*this},
      control{Control::PIDParameters()},
      // listen for configuration inputs
      conf{*this, RX},
      id{0} {
    Config().applyTo(*this);
}

void Systems::parseConfig() {
    led.parseConfig();
    control.parseConfig();
}
