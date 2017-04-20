/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "power.h"
#include "board.h"

PowerMonitor::PowerMonitor() {
    // REFERENCE: https://github.com/pedvide/ADC
    pinMode(board::V0_DETECT, INPUT);
    pinMode(board::I0_DETECT, INPUT);
    pinMode(board::I1_DETECT, INPUT);

    adc.setReference(ADC_REF_1V2, ADC_0);
    adc.setAveraging(1, ADC_0);                     // set number of averages
    adc.setResolution(16, ADC_0);                   // set bits of resolution
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_0);  // change the conversion speed
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_0);    // change the sampling speed

    adc.setReference(BOARD_ADC_REF, ADC_1);
    adc.setAveraging(1, ADC_1);                     // set number of averages
    adc.setResolution(16, ADC_1);                   // set bits of resolution
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_1);  // change the conversion speed
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_1);    // change the sampling speed
}

void PowerMonitor::updateLevels(void) {
    V0_ = getV0Raw();
    I0_ = getI0Raw();
    I1_ = getI1Raw();
}

float PowerMonitor::totalPower(void) const {
    return I0() * V0();
}

float PowerMonitor::electronicsPower(void) const {
    return I1() * 3.7;
}

uint16_t PowerMonitor::getV0Raw(void) {
    return (uint16_t)adc.analogRead(board::V0_DETECT, ADC_1);
}

uint16_t PowerMonitor::getI0Raw(void) {
    return (uint16_t)adc.analogRead(board::I0_DETECT, ADC_0);
}

uint16_t PowerMonitor::getI1Raw(void) {
    return (uint16_t)adc.analogRead(board::I1_DETECT, ADC_0);
}

namespace {
constexpr float calculateVoltageScale(float v_ref, float r_in, float r_out, float int_max) {
    return v_ref * r_in / r_out / int_max;
}

constexpr float calculateCurrentScale(float v_ref, float ic_scaling, float r, float int_max) {
    return v_ref / ic_scaling / r / int_max;
}

constexpr float V0_SCALE = calculateVoltageScale(3.3, 150 + 150, 150, 65536);
constexpr float I0_SCALE = calculateCurrentScale(3.3, 50, 0.005, 65536);
constexpr float I1_SCALE = calculateCurrentScale(3.3, 50, 0.033, 65536);
}

float PowerMonitor::V0(void) const {
    return V0_SCALE * V0_;
}

float PowerMonitor::I0(void) const {
    return I0_SCALE * I0_;
}

float PowerMonitor::I1(void) const {
    return I1_SCALE * I1_;
}
