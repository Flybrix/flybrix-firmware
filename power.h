/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef power_h
#define power_h
#include <Arduino.h>
#include <ADC.h>

class PowerMonitor {
   public:
    PowerMonitor();

    void updateLevels();

    float totalPower() const;
    // battery power in mW

    float electronicsPower() const;
    // electronics power in mW

    uint16_t rawV0() const {
        return V0_;
    }

    uint16_t rawI0() const {
        return I0_;
    }

    uint16_t rawI1() const {
        return I1_;
    }

    float V0() const;
    // measured at battery input terminal in V

    float I0() const;
    // total current from battery in mA

    float I1() const;
    // electronics load current in mA; assume DC/DC output voltage is 3.6V

   private:
    ADC adc;

    uint16_t getV0Raw();
    uint16_t getI0Raw();
    uint16_t getI1Raw();

    uint16_t V0_;
    uint16_t I0_;
    uint16_t I1_;

};  // class PowerMonitor

#endif
