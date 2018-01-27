/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "systems.h"
#include "config.h"

Systems::Systems()
    : led{flag},
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
