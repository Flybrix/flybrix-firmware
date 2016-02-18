/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    Credit is due to several other projects, including:
    - multiwii ("https://github.com/multiwii")
    - phoenix flight controller ("https://github.com/cTn-dev/Phoenix-FlightController")

*/

// #define DEBUG

// library imports
#include <Arduino.h>
#include <EEPROM.h>
#include <i2c_t3.h>
#include <spi4teensy3.h>
#include <ADC.h>
#include <SPI.h>

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
      pilot{&state},
      control{&state, CONFIG.data},
      conf{&state, RX, &control, &CONFIG}  // listen for configuration inputs
{
}

void setup() {
    config_handler = [&](CONFIG_struct& config){
      sys.control.parseConfig(config);
    };

    debug_serial_comm = &sys.conf;

    // setup USB debug serial
    Serial.begin(9600);  // USB is always 12 Mbit/sec

    // MPU9250 is limited to 400kHz bus speed.
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);  // For I2C pins 18 and 19
    sys.state.set(STATUS_BOOT);
    sys.led.update();

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

    sys.state.clear(STATUS_BOOT);
    sys.state.set(STATUS_IDLE);
    sys.led.update();
}

// Main loop variables

uint32_t start_time = 0;
uint32_t state_updates = 0;
uint32_t interrupt_waits = 0;
uint32_t control_updates = 0;
uint32_t mpu_reads = 0;
uint32_t mag_reads = 0;
uint32_t bmp_reads = 0;
uint32_t pwr_reads = 0;


#define DEF_PROCESS_VARIABLES(F) uint32_t iterations_at_##F##Hz = 0;

/* F is target frequency, D is a tuning factor */
#define RUN_PROCESS(F)                               \
    iterations_at_##F##Hz = RunProcess<F>(micros()); \
    sys.i2c.update();

DEF_PROCESS_VARIABLES(1000)
DEF_PROCESS_VARIABLES(500)
DEF_PROCESS_VARIABLES(100)
DEF_PROCESS_VARIABLES(40)
DEF_PROCESS_VARIABLES(10)
DEF_PROCESS_VARIABLES(1)

template <uint32_t f>
uint32_t RunProcess(uint32_t start);

template <uint32_t f>
bool ProcessTask();

bool skip_state_update = false;

void loop() {

    sys.state.loopCount++;

    if (start_time == 0) {
        start_time = micros();
    }

    sys.i2c.update();  // manages a queue of requests for mpu, mag, bmp

    if (sys.mpu.ready) {
        if (!skip_state_update) {
            sys.state.updateStateIMU(micros());  // update state as often as we can
            state_updates++;
        } else {
            interrupt_waits++;
        }
        if (sys.mpu.startMeasurement()) {
            mpu_reads++;
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
        control_updates++;

        sys.airframe.updateMotorsMix();
        sys.motors.updateAllChannels();
    }

    RUN_PROCESS(1000)
    RUN_PROCESS(500)
    RUN_PROCESS(500)
    RUN_PROCESS(100)
    RUN_PROCESS(40)
    RUN_PROCESS(10)
    RUN_PROCESS(1)
}

uint32_t eeprom_log_start = EEPROM_LOG_START;

template <>
bool ProcessTask<1000>() {
    static uint16_t counter{0};
    if (++counter > sys.conf.GetSendStateDelay() - 1) {
        counter = 0;
        sys.conf.SendState(micros(), [&](uint8_t* data, size_t length) {
            if (eeprom_log_start + length >= EEPROM_LOG_END) {
                sys.state.set(STATUS_LOG_FULL);
                return;
            }
            for (size_t i = 0; i < length; ++i)
                EEPROM.write(eeprom_log_start++, data[i]);
        });
    }
    counter %= 1000;
    return true;
}

template <>
bool ProcessTask<500>() {
    sys.led.update();  // update quickly to support color dithering
    return true;
}

template <>
bool ProcessTask<100>() {
    if (sys.bmp.ready) {
        sys.state.updateStatePT(micros());
        sys.bmp.startMeasurement();
        bmp_reads++;
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

    sys.conf.ReadData();  // Respond to commands from the Configurator chrome extension

    return true;
}

template <>
bool ProcessTask<40>() {
    sys.pilot.processCommands();

    sys.pwr.measureRawLevels();  // read all ADCs
    pwr_reads++;

    return true;
}

template <>
bool ProcessTask<10>() {
    if (sys.mag.ready) {
        sys.mag.startMeasurement();
        mag_reads++;
    } else {
        return false;
    }

    return true;
}

template <>
bool ProcessTask<1>() {
#ifdef DEBUG
    float elapsed_seconds = (micros() - start_time) / 1000000.0;
    Serial.print("DEBUG: elapsed time (seconds)   = ");
    Serial.println(elapsed_seconds);
    Serial.print("DEBUG: main loop rate (Hz)      = ");
    Serial.println(sys.state.loopCount / elapsed_seconds);
    Serial.print("DEBUG: state update rate (Hz)   = ");
    Serial.println(state_updates / elapsed_seconds);
    Serial.print("DEBUG: control update rate (Hz) = ");
    Serial.println(control_updates / elapsed_seconds);
    Serial.print("DEBUG: mpu read rate (Hz) = ");
    Serial.println(mpu_reads / elapsed_seconds);
    Serial.print("DEBUG: mag read rate (Hz) = ");
    Serial.println(mag_reads / elapsed_seconds);
    Serial.print("DEBUG: bmp read rate (Hz) = ");
    Serial.println(bmp_reads / elapsed_seconds);
    Serial.print("DEBUG: pwr read rate (Hz) = ");
    Serial.println(pwr_reads / elapsed_seconds);
    Serial.print("DEBUG: 500Hz rate (Hz) = ");
    Serial.println(iterations_at_500Hz / elapsed_seconds);
    Serial.print("DEBUG: 100Hz rate (Hz) = ");
    Serial.println(iterations_at_100Hz / elapsed_seconds);
    Serial.print("DEBUG:  40Hz rate (Hz) = ");
    Serial.println(iterations_at_40Hz / elapsed_seconds);
    Serial.print("DEBUG:  10Hz rate (Hz) = ");
    Serial.println(iterations_at_10Hz / elapsed_seconds);
    Serial.print("DEBUG:   1Hz rate (Hz) = ");
    Serial.println(iterations_at_1Hz / elapsed_seconds);
    Serial.print("DEBUG: interrupt wait rate (Hz) = ");
    Serial.println(interrupt_waits / elapsed_seconds);
    Serial.println("");
#endif

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
