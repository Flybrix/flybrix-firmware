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
      rc_mux{command_sources, serial_rc, radio_receiver},
      imu{state},
      pilot{*this},
      control{Control::PIDParameters()},
      // listen for configuration inputs
      conf{*this, RX},
      id{0} {
    Config().applyTo(*this);
}

void Systems::parseConfig() {
    led.parseConfig();
    imu.parseConfig();
    control.parseConfig();
}
