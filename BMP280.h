/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <BMP280.h/cpp>

    Driver code for our barometer.

*/

#ifndef BMP280_h
#define BMP280_h

#include "Arduino.h"
#include "i2cManager.h"

struct __attribute__((packed)) BMP_calibration {
    uint16_t dig_T1;    // 0x88 / 0x89 dig_T1 unsigned short
    int16_t dig_T2;     // 0x8A / 0x8B dig_T2 signed short
    int16_t dig_T3;     // 0x8C / 0x8D dig_T3 signed short
    uint16_t dig_P1;    // 0x8E / 0x8F dig_P1 unsigned short
    int16_t dig_P2;     // 0x90 / 0x91 dig_P2 signed short
    int16_t dig_P3;     // 0x92 / 0x93 dig_P3 signed short
    int16_t dig_P4;     // 0x94 / 0x95 dig_P4 signed short
    int16_t dig_P5;     // 0x96 / 0x97 dig_P5 signed short
    int16_t dig_P6;     // 0x98 / 0x99 dig_P6 signed short
    int16_t dig_P7;     // 0x9A / 0x9B dig_P7 signed short
    int16_t dig_P8;     // 0x9C / 0x9D dig_P8 signed short
    int16_t dig_P9;     // 0x9E / 0x9F dig_P9 signed short
    uint16_t reserved;  // 0xA0 / 0xA1 reserved reserved
};

union BMP_calibration_union {
    struct BMP_calibration val;
    uint8_t raw[sizeof(struct BMP_calibration)];
};

class BMP280 : public CallbackProcessor {
   public:
    explicit BMP280(I2CManager *i2c);  // base type

    void restart();

    bool ready;

    uint8_t getID();

    bool startMeasurement();
    void triggerCallback();  // handles return for getPT()

    uint16_t temperature = 0;
    uint32_t pressure = 0;
    uint32_t p0 = 25600000;  // initial pressure, set to 1 bar by default

   private:
    I2CManager *i2c;

    uint8_t getStatusByte();

    BMP_calibration_union CALIBRATION;
    boolean validateCalibation();

    uint16_t compensate_T_int32(int32_t rawT);
    uint32_t compensate_P_int32(int32_t rawP);
    uint32_t compensate_P_int64(int32_t rawP);  // higher accuracy; 10x more computation; Q24.8 format

    int32_t t_fine;

    // buffers for processCallback
    uint8_t data_to_read[6];
    uint8_t data_to_send[1];
};

#define BMP280_ADDR 0x77  // 7-bit address

#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS 0xF7
#define BMP280_REG_TEMP 0xFA

#define BMP280_FACTORY_CALIBRATION 0x88

#define BMP280_REG_RESULT 0xF7  // 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
                                // 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.

// BITFIELD FLAGS
#define OSRS_T_SKIP 0x00
#define OSRS_T_X1 0x20
#define OSRS_T_X2 0x40
#define OSRS_T_X4 0x60
#define OSRS_T_X8 0x80
#define OSRS_T_X16 0xE0

#define OSRS_P_SKIP 0x00
#define OSRS_P_X1 0x04
#define OSRS_P_X2 0x08
#define OSRS_P_X4 0x0C
#define OSRS_P_X8 0x10
#define OSRS_P_X16 0x1C

#define MODE_SLEEP 0x00
#define MODE_FORCED 0x01
#define MODE_NORMAL 0x03

#define FILTER_OFF 0x00
#define FILTER_X2 0x04
#define FILTER_X4 0x08
#define FILTER_X8 0x0C
#define FILTER_X16 0x1C

#define T_SB_0p5ms 0x00
#define T_SB_62p5ms 0x20
#define T_SB_125ms 0x40
#define T_SB_500ms 0x80
#define T_SB_1000ms 0xA0
#define T_SB_2000ms 0xC0
#define T_SB_4000ms 0xE0

#endif
