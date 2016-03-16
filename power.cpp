/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "power.h"
#include "board.h"
#include "state.h"

PowerMonitor::PowerMonitor(State* __state) {
    state = __state;

    // REFERENCE: https://github.com/pedvide/ADC
    pinMode(board::V0_DETECT, INPUT);
    pinMode(board::I0_DETECT, INPUT);
    pinMode(board::I1_DETECT, INPUT);

    adc.setReference(ADC_REF_1V2, ADC_0);
    adc.setAveraging(1, ADC_0);                     // set number of averages
    adc.setResolution(16, ADC_0);                   // set bits of resolution
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_0);  // change the conversion speed
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_0);    // change the sampling speed

    adc.setReference(ADC_REF_1V2, ADC_1);
    adc.setAveraging(1, ADC_1);                     // set number of averages
    adc.setResolution(16, ADC_1);                   // set bits of resolution
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_1);  // change the conversion speed
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_1);    // change the sampling speed
}

void PowerMonitor::measureRawLevels(void) {
    state->V0_raw = getV0Raw();
    state->I0_raw = getI0Raw();
    state->I1_raw = getI1Raw();
}

float PowerMonitor::getTotalPower(void) {
    return getI0() * getV0();
}

float PowerMonitor::getElectronicsPower(void) {
    return getI1() * 3.7;
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

float PowerMonitor::getV0(void) {
    // Volts = (20.5 + 226) / 20.5 * 1.2 / 65536 * raw
    return 0.00022017316 * getV0Raw();
}

float PowerMonitor::getI0(void) {
    // Amps = (1/50) / 0.003 * 1.2 / 65536 * raw
    return 0.00012207031 * getI0Raw();
}

float PowerMonitor::getI1(void) {
    // Amps = (1/50) / 0.03 * 1.2 / 65536 * raw
    return 0.00001220703 * getI1Raw();
}
