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
    : state{this},
      led{flag},
      imu{state},
      pilot{*this},
      control{Control::PIDParameters(), Control::VelocityPIDParameters()},
      // listen for configuration inputs
      conf{*this, RX},
      autopilot{conf},
      id{0} {
    Config().applyTo(*this);
}

void Systems::parseConfig() {
    led.parseConfig();
    imu.parseConfig();
    control.parseConfig();
}
