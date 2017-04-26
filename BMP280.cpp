/*
        *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
        *
        *  License and other details available at: http://www.flybrix.com/firmware
*/

//#define BMP280_SERIAL_DEBUG

#include "BMP280.h"
#include <math.h>
#include <i2c_t3.h>
#include <stdint.h>

BMP280::BMP280(I2CManager *__i2c) {
    i2c = __i2c;
    ready = false;
}

void BMP280::restart() {
    uint8_t settings;

    // full reset
    i2c->writeByte(BMP280_ADDR, 0xE0, 0xB6);

    delay(10);  // wait at least 2 ms for "power-on-reset"
    // load factory calibration
    i2c->readBytes(BMP280_ADDR, BMP280_FACTORY_CALIBRATION, sizeof(struct BMP_calibration), CALIBRATION.raw);
    // check to see if calibration values are sensible
    if (!validateCalibation()) {
        // ERROR: ("...WARNING -- CALIBRATION MAY NOT BE RELIABLE!...");
    }

    // set controls to recommended values

    // osrs_t[7,6,5] bits in control register 0xF4 -- 010 (x2)
    // osrs_p[4,3,2] bits in control register 0xF4 -- 111 (x16)
    //    mode[1,0] bits in control register 0xF4 --  11 (normal)
    settings = 0x0;
    settings |= OSRS_T_X2;
    settings |= OSRS_P_X16;
    settings |= MODE_NORMAL;
    i2c->writeByte(BMP280_ADDR, BMP280_REG_CTRL_MEAS, settings);

    //  t_sb[7,6,5] bits in control register 0xF5 -- 000 (0.5ms sleep)
    // filter[4,3,2] bits in control register 0xF5 -- 111 (16)
    //    ignore[1] bits in control register 0xF5 --   0 (ignore)
    //  spi3w_en[0] bits in control register 0xF5 --   0 (disable)
    settings = 0x0;
    settings |= FILTER_X16;
    settings |= T_SB_0p5ms;
    i2c->writeByte(BMP280_ADDR, BMP280_REG_CONFIG, settings);

    // resulting measurement rate is 26.32 Hz
    delay(250);  // first few values are bad
}

uint8_t BMP280::getID() {
    return i2c->readByte(BMP280_ADDR, 0xD0);  // Read WHO_AM_I register for BMP280 --> 0x58
}

uint8_t BMP280::getStatusByte() {
    return i2c->readByte(BMP280_ADDR, 0xF3);
    // bit 3 set to ‘1’ whenever a conversion is running and back to ‘0’ when the results have been transferred to the data registers.
    // bit 0 set to ‘1’ when the NVM data are being copied to image registers and back to ‘0’ when the copying is done.
}

boolean BMP280::validateCalibation() {
    return (CALIBRATION.val.dig_T3 == -1000);
}

#define BMP280_REG_RESULT 0xF7  // 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
                                // 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.
bool BMP280::startMeasurement(void) {
    ready = false;
    data_to_send[0] = BMP280_REG_RESULT;
    i2c->addTransfer(BMP280_ADDR, 1, data_to_send, 6, data_to_read, [this]() { triggerCallback(); });
    return true;
}

void BMP280::triggerCallback() {
    int32_t rawP, rawT;
    rawP = (((int32_t)data_to_read[0]) << 12) + (((int32_t)data_to_read[1]) << 4) + (((int32_t)data_to_read[2]) >> 4);
    rawT = (((int32_t)data_to_read[3]) << 12) + (((int32_t)data_to_read[4]) << 4) + (((int32_t)data_to_read[5]) >> 4);
    temperature = compensate_T_int32(rawT);  // calculate temp first to update t_fine
    pressure = compensate_P_int64(rawP);
    ready = true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
uint16_t BMP280::compensate_T_int32(int32_t rawT) {
    int32_t var1, var2;
    uint16_t T;
    var1 = ((((rawT >> 3) - ((int32_t)CALIBRATION.val.dig_T1 << 1))) * ((int32_t)CALIBRATION.val.dig_T2)) >> 11;
    var2 = (((((rawT >> 4) - ((int32_t)CALIBRATION.val.dig_T1)) * ((rawT >> 4) - ((int32_t)CALIBRATION.val.dig_T1))) >> 12) * ((int32_t)CALIBRATION.val.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (uint16_t)((t_fine * 5 + 128) >> 8);
    return T;  // noise ~ 0.004C
}

// Returns pressure in Pa. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t BMP280::compensate_P_int32(int32_t rawP) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)CALIBRATION.val.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)CALIBRATION.val.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)CALIBRATION.val.dig_P4) << 16);
    var1 = (((CALIBRATION.val.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)CALIBRATION.val.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)CALIBRATION.val.dig_P1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = (((uint32_t)(((int32_t)1048576) - rawP) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)CALIBRATION.val.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)CALIBRATION.val.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + CALIBRATION.val.dig_P7) >> 4));
    return p;  // noise < 0.2pa
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BMP280::compensate_P_int64(int32_t rawP) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)CALIBRATION.val.dig_P6;
    var2 = var2 + ((var1 * (int64_t)CALIBRATION.val.dig_P5) << 17);
    var2 = var2 + (((int64_t)CALIBRATION.val.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)CALIBRATION.val.dig_P3) >> 8) + ((var1 * (int64_t)CALIBRATION.val.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)CALIBRATION.val.dig_P1) >> 33;
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = 1048576 - rawP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)CALIBRATION.val.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)CALIBRATION.val.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)CALIBRATION.val.dig_P7) << 4);
    return (uint32_t)p;
}
