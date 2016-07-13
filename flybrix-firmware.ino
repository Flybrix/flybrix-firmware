#include <SPI.h>
#include <SdFat.h>
#include <FastLED.h>
/*
    *  Flybrix Flight Controller -- Copyright 2015, 2016 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    Credit for inspiration and guidance is due to several other projects, including:
    - multiwii ("https://github.com/multiwii")
    - cleanflight ("https://github.com/cleanflight")
    - phoenix flight controller ("https://github.com/cTn-dev/Phoenix-FlightController")

*/

// #define DEBUG

// library imports
#include <Arduino.h>
#include <EEPROM.h>
#include <i2c_t3.h>
#include <ADC.h>

// octoteensy custom library imports
#include "power.h"
#include "BMP280.h"
#include "MPU9250.h"
#include "AK8963.h"
#include "led.h"
#include "R415X.h"
#include "command.h"
#include "control.h"
#include "airframe.h"
#include "motors.h"
#include "state.h"
#include "config.h"
#include "serial.h"
#include "i2cManager.h"
#include "localization.h"
#include "debug.h"
#include "version.h"
#include "board.h"
#include "cardManagement.h"
#include "serialFork.h"
#include "testMode.h"

struct Systems {
    // subsystem objects initialize pins when created
    // Storing inside a struct forces the right initialization order
    R415X receiver;

    I2CManager i2c;
    State state;
    LED led;

    BMP280 bmp;
    MPU9250 mpu;
    AK8963 mag;
    PowerMonitor pwr;
    Motors motors;
    Airframe airframe;
    PilotCommand pilot;
    Control control;
    SerialComm conf;
    Systems();
} sys;

Systems::Systems()
    : receiver{},
      i2c{},
      state{},
      led{&state},
      bmp{&state, &i2c},  // pressure sensor object
      mpu{&state, &i2c},  // inertial sensor object
      mag{&state, &i2c},  // magnetometer
      pwr{&state},        // onboard power monitoring object
      motors{&state},     // eight PWM channels
      airframe{&state},
      pilot{&state, &receiver},
      control{&state, CONFIG.data},
      conf{&state, RX, &control, &CONFIG, &led, &pilot}  // listen for configuration inputs
{
}

void setup() {

    config_handler = [&](CONFIG_struct& config){
      sys.control.parseConfig(config);
    };

    config_verifier = [&](const CONFIG_struct& config) {
        bool retval{true};
        retval &= sys.control.verifyConfig(config);
        return retval;
    };

    debug_serial_comm = &sys.conf;

    // MPU9250 is limited to 400kHz bus speed.
    Wire.begin(I2C_MASTER, 0x00, board::I2C_PINS, board::I2C_PULLUP, I2C_RATE_400);  // For I2C pins 18 and 19
    sys.state.set(STATUS_BOOT);
    sys.led.update();

    bool go_to_test_mode{isEmptyEEPROM()};

    // load stored settings (this will reinitialize if there is no data in the EEPROM!
    readEEPROM();
    sys.state.resetState();

    sys.state.set(STATUS_BMP_FAIL);
    sys.led.update();
    sys.bmp.restart();
    if (sys.bmp.getID() == 0x58) {
        sys.state.clear(STATUS_BMP_FAIL);
        // state is unhappy without an initial pressure
        sys.bmp.startMeasurement();  // important; otherwise we'll never set ready!
        sys.i2c.update();  // write data
        delay(2);  // wait for data to arrive
        sys.i2c.update();  // read data
        sys.state.p0 = sys.state.pressure;  // initialize reference pressure
    } else {
        sys.led.update();
        while (1)
            ;
    }

    sys.state.set(STATUS_MPU_FAIL);
    sys.led.update();
    sys.mpu.restart();
    sys.mag.restart();
    if ((sys.mpu.getID() == 0x71) && (sys.mag.getID() == 0x48)) {
        sys.state.clear(STATUS_MPU_FAIL);
        while (!sys.mpu.startMeasurement()) {
            delay(1);
        };                           // important; otherwise we'll never set ready!
        sys.mag.startMeasurement();  // important; otherwise we'll never set ready!
    } else {
        sys.led.update();
        while (1)
            ;
    }

    sys.led.update();

    // factory test pattern runs only once
    if (go_to_test_mode)
        runTestMode(sys.state, sys.led, sys.motors);

    // Perform intial check for an SD card
    sdcard::startup();

    sys.state.clear(STATUS_BOOT);
    sys.state.set(STATUS_IDLE);
    sys.led.update();
}

uint32_t low_battery_counter = 0;

template <uint32_t f>
uint32_t RunProcess(uint32_t start);

template <uint32_t... Freqs>
struct freqlist{};

void RunProcessesHelper(freqlist<>) {
}

template <uint32_t F, uint32_t... Fargs>
void RunProcessesHelper(freqlist<F, Fargs...>) {
    RunProcess<F>(micros());
    sys.i2c.update();
    RunProcessesHelper(freqlist<Fargs...>());
}

template <uint32_t... Freqs>
void RunProcesses() {
    RunProcessesHelper(freqlist<Freqs...>());
}

template <uint32_t f>
bool ProcessTask();

bool skip_state_update = false;

void loop() {

    sys.state.loopCount++;

    sys.i2c.update();  // manages a queue of requests for mpu, mag, bmp

    if (sys.mpu.ready) {
        if (!skip_state_update) {
            sys.state.updateStateIMU(micros());  // update state as often as we can
        } else {
        }
        if (sys.mpu.startMeasurement()) {
            skip_state_update = false;
        } else {
            skip_state_update = true;  // stop updating state until we queue another mpu measurement
        }
    }

    sys.i2c.update();

    if (sys.state.is(STATUS_OVERRIDE)) {  // user is changing motor levels using Configurator
        sys.motors.updateAllChannels();
    } else {
        sys.control.calculateControlVectors();

        sys.airframe.updateMotorsMix();
        sys.motors.updateAllChannels();
    }

    RunProcesses<1000, 100, 40, 35, 30, 10, 1>();
}

template <>
bool ProcessTask<1000>() {
    static uint16_t counterSerial{0};
    static uint16_t counterSdCard{0};
    if (++counterSerial > sys.conf.GetSendStateDelay() - 1) {
        counterSerial = 0;
        sys.conf.SendState(micros());
    }
    if (++counterSdCard > sys.conf.GetSdCardStateDelay() - 1) {
        counterSdCard = 0;
        sys.conf.SendState(micros(), 0xFFFFFFFF, true);
    }
    counterSerial %= 1000;
    counterSdCard %= 1000;
    if (LEDFastUpdate)
        LEDFastUpdate();
    return true;
}

template <>
bool ProcessTask<30>() {
    sys.led.update();  // update quickly to support color dithering
    return true;
}

template <>
bool ProcessTask<100>() {
    if (sys.bmp.ready) {
        sys.state.updateStatePT(micros());
        sys.bmp.startMeasurement();
    } else {
        return false;
    }

    if (sys.state.is(STATUS_CLEAR_MPU_BIAS)) {
        sys.mpu.forgetBiasValues();
        sys.state.clear(STATUS_CLEAR_MPU_BIAS);
    }
    if (sys.state.is(STATUS_SET_MPU_BIAS)) {
        sys.mpu.correctBiasValues();
        sys.state.clear(STATUS_SET_MPU_BIAS);
    }

    sys.conf.Read();  // Respond to commands from the Configurator chrome extension

    return true;
}

template <>
bool ProcessTask<35>() {
    flushSerial();
    return true;
}

template <>
bool ProcessTask<40>() {
    sys.pilot.processCommands();

    sys.pwr.measureRawLevels();  // read all ADCs

    // check for low voltage condition
    if ( ((1/50)/0.003*1.2/65536 * sys.state.I1_raw ) > 1.0f ){ //if total battery current > 1A
        if ( ((20.5+226)/20.5*1.2/65536 * sys.state.V0_raw) < 2.8f ) {
            low_battery_counter++;
            if ( low_battery_counter > 40 ){
                sys.state.set(STATUS_BATTERY_LOW);
            }
        }
        else {
            low_battery_counter = 0;
        }
    }
    else {
        if ( ((20.5+226)/20.5*1.2/65536 * sys.state.V0_raw) < 3.63f ) {
            low_battery_counter++;
            if ( low_battery_counter > 40 ){
                sys.state.set(STATUS_BATTERY_LOW);
            }
        }
        else {
            low_battery_counter = 0;
        }
    }

    return true;
}

template <>
bool ProcessTask<10>() {
    if (sys.mag.ready) {
        sys.mag.startMeasurement();
    } else {
        return false;
    }

    return true;
}

template <>
bool ProcessTask<1>() {
    return true;
}

template <uint32_t f>
uint32_t RunProcess(uint32_t start) {
    static uint32_t previous_time{start};
    static uint32_t iterations{0};

    while (start - previous_time > 1000000 / f) {
        if (ProcessTask<f>()) {
            previous_time += 1000000 / f;
            ++iterations;
        } else {
          break;
        }
    }

    return iterations;
}
