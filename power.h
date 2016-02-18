/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <power.h/cpp>

    Power monitoring using ADC inputs.

*/

#ifndef power_h
#define power_h
#include "Arduino.h"
#include <ADC.h>

class State;

class PowerMonitor {
   public:
    PowerMonitor(State* state);

    void measureRawLevels();

    float getTotalPower();
    // battery power in mW

    float getElectronicsPower();
    // electronics power in mW

    float getV0();
    // measured at battery input terminal in V

    float getI0();
    // total current from battery in mA

    float getI1();
    // electronics load current in mA; assume DC/DC output voltage is 3.6V

   private:
    State* state;
    ADC adc;

    uint16_t getV0Raw();
    uint16_t getI0Raw();
    uint16_t getI1Raw();

};  // class PowerMonitor

// pin definitions
#define V0_DETECT A13  // ADC0_DM3
#define I0_DETECT A10  // ADC0_DP0
#define I1_DETECT A11  // ADC0_DM0

#endif
