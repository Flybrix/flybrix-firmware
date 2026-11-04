
/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com

    Credit for inspiration and guidance is due to several other projects, including:
      - multiwii ("https://github.com/multiwii")
      - cleanflight ("https://github.com/cleanflight")
      - phoenix flight controller ("https://github.com/cTn-dev/Phoenix-FlightController")
*/

// library imports
#include <ADC.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <SPI.h>
#include <SdFat.h>
#include <i2c_t3.h>

#include "AK8963.h"
#include "BMP280.h"
#include "MPU9250.h"
#include "receiver.h"
#include "airframe.h"
#include "board.h"
#include "cardManagement.h"
#include "command.h"
#include "config.h"
#include "control.h"
#include "debug.h"
#include "i2cManager.h"
#include "led.h"
#include "ledDriver.h"
#include "localization.h"
#include "loop_stopper.h"
#include "motors.h"
#include "power.h"
#include "serial.h"
#include "serialFork.h"
#include "state.h"
#include "systems.h"
#include "taskRunner.h"
#include "testMode.h"
#include "usbModeSelector.h"
#include "utility/clock.h"
#include "version.h"
#include "watchdog.h"

Systems sys;
int8_t low_battery_counter = 0;
int8_t critical_battery_counter = 0;

bool updateLoopCount() {
    sys.state.loopCount++;
    return true;
}

bool updateI2C() {
    return i2c().update();
}

bool updateIndicatorLights() {
    sys.led.update();  // update quickly to support color dithering
    return true;
}

bool processPressureSensor() {
    if (!sys.bmp.ready) {
        return false;
    }
    if (sys.bmp.fresh) {  // skip if the last read failed, so a stale sample is not fed in as new
        sys.bmp.fresh = false;
        sys.state.readStatePT(sys.bmp.p0, sys.bmp.pressure, sys.bmp.temperature);
    }
    sys.bmp.startMeasurement();
    return true;
}

bool predict_phase{true};

// Altitude/velocity filter (UKF). It only gains information when the barometer delivers a sample (~26Hz), so it
// runs one predict + update per sample, split over two calls so neither blocks the flight control chain for long.
// It used to run at 640Hz, taking about half the CPU and re-fusing each barometer sample ~12 times.
bool updateStateEstimate() {
    if (predict_phase) {
        if (!sys.state.hasNewAltitudeMeasurement()) {
            return false;
        }
        sys.state.predictFilter(ClockTime::now());
    } else {
        sys.state.updateFilter();
    }
    predict_phase = !predict_phase;
    return true;
}

bool runAutopilot() {
    return sys.autopilot.run(ClockTime::now());
}

bool updateControlVectors() {
    if (!sys.pilot.isOverridden()) {  // user isn't changing motor levels using Configurator
        sys.control_vectors = sys.control.calculateControlVectors(sys.state.getVelocity(), sys.state.getKinematics(), sys.command_vector);
    }
    sys.pilot.applyControl(sys.control_vectors);
    return true;
}

bool processPilotInput() {
    sys.command_vector = sys.pilot.processCommands(sys.rc_mux.query());
    return true;
}

bool checkBatteryUse() {
    sys.pwr.updateLevels();  // read all ADCs

    // check for low voltage condition
    if (sys.pwr.I1() > 1000.0f) {  // if total battery current > 1A
        if (sys.pwr.V0() < 2.8f) {
            low_battery_counter++;
        } else {
            low_battery_counter--;
        }

        if (sys.pwr.V0() < 2.6f) {
            critical_battery_counter++;
        } else {
            critical_battery_counter--;
        }
    } else {
        if (sys.pwr.V0() < 3.2f) {
            low_battery_counter++;
        } else {
            low_battery_counter--;
        }

        if (sys.pwr.V0() < 3.0f) {
            critical_battery_counter++;
        } else {
            critical_battery_counter--;
        }
    }
    if (low_battery_counter > 100) {
        sys.flag.set(Status::BATTERY_LOW);
        low_battery_counter = 100;
    }
    if (low_battery_counter < 2) {
        sys.flag.clear(Status::BATTERY_LOW);
        low_battery_counter = 2;
    }

    if (critical_battery_counter > 100) {
        sys.flag.set(Status::BATTERY_CRITICAL);
        critical_battery_counter = 100;
    }
    if (critical_battery_counter < 2) {
        sys.flag.clear(Status::BATTERY_CRITICAL);
        critical_battery_counter = 2;
    }

    return true;
}

bool updateMagnetometer() {
    return sys.imu.startMagnetFieldMeasurement();
}

// The latency-critical chain, checked before every scheduled task: as soon as the MPU9250 flags a new sample, read it,
// update the attitude estimate, run the PID cascade and write the motors, all in one go. Previously each step waited
// for its own scheduled task, which added several milliseconds between a gyro sample and the motors reacting.
bool runFlightControl() {
    uint32_t samples_before = sys.imu.sampleCount();
    if (!sys.imu.startInertialMeasurement()) {
        return false;  // no new sample yet, or a read is already in progress
    }
    // Perform queued transfers until ours has completed or failed (a magnetometer/barometer read queued ahead of it
    // runs first). Each transfer is bounded by the I2C timeout and the queue is finite, so this terminates.
    while (sys.imu.inertialMeasurementPending() && i2c().busy()) {
        i2c().update();
    }
    if (sys.imu.sampleCount() == samples_before) {
        return false;  // the read failed: don't rerun the controller on old data
    }
    return updateControlVectors();
}

// Flying on frozen attitude data ends in a flip or a flyaway, so cut the motors if samples stop arriving.
bool checkImuHealth() {
    if (sys.imu.healthy(ClockTime::now())) {
        return false;
    }
    sys.pilot.emergencyDisarm();
    return true;
}

bool printTasks();

// channel (sd/usb/bluetooth) operations include buffer (read,write) and physical (get,send) transfers
// "higher level" channel commands include "writeState" and "processCommand"

bool sd_writeState(){
    sys.conf.SendState(0xFFFFFFFF, true); //only fills sd buffer
    return (sdcard::writing::bytesWritten() > 0);
}

bool sd_send(){
    sdcard::writing::send();
    return true;
}

bool serial_writeState() {
    sys.conf.SendState(); //fills bluetooth and/or usb buffers
    return true;
}

bool usb_send() {
    return usb_sendData();
}

bool usb_get() {
    return usb_getData();
}

bool bluetooth_send() {
    return bluetooth_sendData();
}

uint32_t serial_rc_updates_seen{0};

// Runs for each Bluetooth message the moment it is complete. Previously the bytes were read at 300Hz but messages
// were only decoded at 40Hz from a single-message buffer, so stick commands arriving in between were overwritten
// (dropped) and the rest waited up to 25ms, then up to another 25ms for the pilot input task.
void onBluetoothMessage(CobsReaderBuffer& message) {
    sys.conf.ProcessData(message, true);
    uint32_t updates = sys.serialRc().updateCount();
    if (updates != serial_rc_updates_seen) {
        serial_rc_updates_seen = updates;
        // New stick values: apply them now if Bluetooth is the active RC source and the pilot state machine is
        // passing sticks through (armed and flying). Arming, failsafe and timeouts still run in "pilot input".
        if (sys.rc_mux.lastSource() == 0) {
            sys.command_vector = sys.pilot.refreshSticks(sys.serialRc().latest(), sys.command_vector);
        }
    }
}

bool bluetooth_get() {
    return bluetooth_getData(onBluetoothMessage);
}

#define AVERAGE_DELAY_TARGET_USEC 1400

TaskRunner tasks[] = {
    {"print tasks       ", printTasks, 20/*sec*/*1000*1000, false, true},  // ( pause ) debug printing interval, enabled? (t/f), force to always log stats
    {"flight control    ", runFlightControl, 0},                          // polled before every task; "work" count = gyro samples handled (500Hz)
    {"loop count        ", updateLoopCount, AVERAGE_DELAY_TARGET_USEC},   // (<   5us)
    {"pilot input       ", processPilotInput, hzToMicros(40)},            // (< 160us) cPPM signals come at ~40Hz
    {"send bluetooth    ", bluetooth_send, 33000 /*usec*/},               // (<  20us) Rigado BMDWare limited to 20B each 30msec
    {"bluetooth rx      ", bluetooth_get, hzToMicros(300)},               // (< 300us) read bytes and process each message as it completes
    {"send SD block     ", sd_send, hzToMicros(300)},                     // (< 220us) slower requires larger buffer
    {"update lights     ", updateIndicatorLights, hzToMicros(30)},        // (< 160us) keep at 30Hz for visual effects
    {"SD state out      ", sd_writeState, hzToMicros(1)},                 // (< 200us) set dynamically; set index in #define below [8]
    {"update i2c        ", updateI2C, hzToMicros(300)},                   // (< 465us) background magnetometer/barometer reads
    {"serial state out  ", serial_writeState, hzToMicros(1)},             // (<  55us) set dynamically; set index in #define below [10]
    {"imu health        ", checkImuHealth, hzToMicros(100)},              // disarm if attitude data goes stale
    {"pressure sensor   ", processPressureSensor, hzToMicros(26.3)},      // (<  30us) bmp280 datasheet output data rate
    {"check battery     ", checkBatteryUse, hzToMicros(10)},              // (<  30us)
    {"update magnet     ", updateMagnetometer, hzToMicros(10)},           // (<  30us)
    {"get usb           ", usb_get, hzToMicros(100)},                     // (?)
    {"send usb          ", usb_send, hzToMicros(100)},                    // (?)
    {"autopilot         ", runAutopilot, hzToMicros(1), false},           // (?) future feature; turn off for now
    {"state estimate    ", updateStateEstimate, hzToMicros(100)},         // (1340us + 700us) one predict+update per barometer sample
};
#define TASK_COUNT 19
#define FLIGHT_CONTROL tasks[1]
#define SD_STATE_OUT tasks[8]
#define SERIAL_STATE_OUT tasks[10]

float sd_max_rate_Hz = 200.0f; // actual rate is ~197Hz

void sdLogTest(){
    TaskRunner& t = SD_STATE_OUT;
    if (t.log_count) {
        float rate = (t.log_count * 1000000.0f) / ((float) t.delay_track.value_sum);
        Serial.printf("%12" PRIu32 ", %12" PRIu32 ",  %12" PRIu32 ", %7.2f, %7.2f, ", ClockTime::now().readClockTick(), t.work_count, t.log_count, sd_max_rate_Hz, rate);
        sdcard::writing::printReport();
        Serial.flush();
    }
    for (size_t i = 0; i < TASK_COUNT; ++i) {
        TaskRunner& task = tasks[i];
        task.resetStats();
    }
    sd_max_rate_Hz += 1.0;
}

void performanceReport(){
    Serial.printf("\n[%10" PRIu32 "] Performance Report (Hz / us): \n", ClockTime::now().readClockTick());
    // print tasks in order from fastest to slowest
    size_t s[TASK_COUNT];
    for (size_t i = 0; i < TASK_COUNT; ++i) {
        s[i] = i;
    }
    for (size_t i = 0; i < TASK_COUNT-1; ++i) {
        size_t min = i;
        for (size_t j = i+1; j < TASK_COUNT; ++j) {
            TaskRunner& task = tasks[s[j]];
            TaskRunner& mintask = tasks[s[min]];
            uint32_t task_d = (!task.log_count) ? 10000000+j : (float) task.delay_track.value_sum / task.log_count;
            uint32_t mintask_d = (!mintask.log_count) ?  20000000 : (float) mintask.delay_track.value_sum / mintask.log_count;
            if ( task_d <= mintask_d) {
                min = j;
            }
        }
        size_t temp=s[i]; s[i]=s[min]; s[min]=temp; //swap(s[i], s[min])
    }
    float processor_load_percent{0.0f};
    for (size_t i = 0; i < TASK_COUNT; ++i) {
        TaskRunner& task = tasks[s[i]];
        //Serial.printf("[%2d] ", s[i]);
        if (!task.log_count) {
            Serial.printf("[%s %11d]\n", task.name, 0);
            continue;
        }

        float average_duration_usec = (float) task.duration_track.value_sum / task.log_count;
        float average_delay_usec = (float) task.delay_track.value_sum / task.log_count;
        float rate = 1000000.0f / average_delay_usec;
        processor_load_percent += 0.1 * (average_duration_usec / 1000.0f) * rate; // 100%/1000 msec *  msec/cycle * cycles/second

        Serial.printf("[%s %11d] rate: %7.2f       delay: %12d %12.1f %12d        duration: %12d %12.1f %12d\n",
                      task.name, task.work_count, rate,
                      task.delay_track.value_min,    average_delay_usec ,   task.delay_track.value_max,
                      task.duration_track.value_min, average_duration_usec, task.duration_track.value_max);
    }
    sdcard::writing::printReport();
    printSerialReport();
    Serial.printf("Average loop time use is %4.2f%%.\n", processor_load_percent);
}

bool printTasks() {
    if (usb_mode::get() != usb_mode::PERFORMANCE_REPORT) {
        return false;
    }
    loops::Stopper _stopper("print report");
    performanceReport();
    //sdLogTest();

    return true;
}

void setup() {
    LED_driver.begin();  // first: register the LED strip (see LEDDriver::begin)
    loops::setLedIndicator(&sys.led);
    CRGB green = LED::fade(CRGB::Green);
    CRGB red = LED::fade(CRGB::Red);

    debug_serial_comm = &sys.conf;

    // MPU9250 is limited to 400kHz bus speed.
    Wire.begin(I2C_MASTER, 0x00, board::I2C_PINS, board::I2C_PULLUP, I2C_RATE_400);  // For I2C pins 18 and 19
    i2c().begin();  // bus timeouts

    //EEPROM.write(0, 255); //mark EEPROM empty for factory reset
    bool go_to_test_mode{isEmptyEEPROM()};

    const uint32_t boot_sequence_delay_usec = 300000;

    ClockTime start = ClockTime::now();
    sys.led.errorStart(LEDPattern::SOLID, red, green, 0); /*begin boot phase 0*/

    readEEPROM().applyTo(sys); // load stored settings (this will reinitialize if there is no data in the EEPROM!
    sys.state.resetState();

    while(ClockTime::now()-start < boot_sequence_delay_usec){;} // boot pattern timing
    start = ClockTime::now();
    sys.led.errorStart(LEDPattern::SOLID, red, green, 1); /* begin boot phase 1*/

    sys.bmp.restart();
    if (sys.bmp.getID() == 0x58) {
        // state is unhappy without an initial pressure
        sys.bmp.startMeasurement();     // important; otherwise we'll never set ready!
        i2c().update();                 // write data
        delay(50);                       // wait for data to arrive
        i2c().update();                 // read data
        sys.bmp.p0 = sys.bmp.pressure;  // initialize reference pressure
    } else {
        while (1)
            ;
    }

    while(ClockTime::now()-start < boot_sequence_delay_usec){;} // boot pattern timing
    start = ClockTime::now();
    sys.led.errorStart(LEDPattern::SOLID, red, green, 2); /* begin boot phase 2*/

    sys.imu.restart();
    if (sys.imu.hasCorrectIDs()) {
        sys.imu.initialize();
    } else {
        while (1)
            ;
    }

    while(ClockTime::now()-start < boot_sequence_delay_usec){;} // boot pattern timing
    start = ClockTime::now();
    sys.led.errorStart(LEDPattern::SOLID, red, green, 3); /* begin boot phase 3*/

    uint8_t j=0;
    while ( j<5 || sys.imu.magnetReportsAllZeroes() ) {
        sys.imu.startMagnetFieldMeasurement();
        i2c().update();
        delay(50);
        i2c().update();
        j++;
    }

    while(ClockTime::now()-start < boot_sequence_delay_usec){;} // boot pattern timing
    start = ClockTime::now();
    sys.led.errorStart(LEDPattern::SOLID, red, green, 4); /* begin boot phase 4*/

    /* future tests*/

    // Perform intial check for an SD card
    while(ClockTime::now()-start < boot_sequence_delay_usec){;} // boot pattern timing
    start = ClockTime::now();
    sys.led.errorStop();
    sys.version.display(sys.led);

    // when there is no card present, we will sit here for ~2sec
    // better to show version pattern instead of four red lights
    sdcard::startup();

    // Do Bluetooth last becauase we have to wait 2.5 seconds for AT mode
    setBluetoothUart(sys.name);

    // factory test pattern runs only once
    if (go_to_test_mode) {
        runTestMode(sys.state, sys.led, sys.pilot);
    }

    loops::start();

    // last, after the multi-second boot steps above (SD card probe, Bluetooth AT mode, test mode)
    watchdog::begin();
}

// After a deliberate pause (loops::Stopper), restart every task's timer so nothing runs a burst of catch-up work.
void handleLoopStop() {
    if (loops::used()) {
        for (TaskRunner& task : tasks) {
            task.reset(loops::lastStart());
        }
        loops::reset();
    }
}

void loop() {
    if (loops::stopped()) {
        sys.led.errorStart(LEDPattern::SOLID, CRGB::White, CRGB::Red, 2);
        sys.led.errorStop();
        return;
    }

    watchdog::feed();

    SERIAL_STATE_OUT.setDesiredInterval(sys.conf.GetSendStateDelay() * 1000, 1000*1000);
    SD_STATE_OUT.setDesiredInterval(max( hzToMicros(sd_max_rate_Hz), sys.conf.GetSdCardStateDelay() * 1000), 1000*1000);

    // Every pass visits every background task. The flight control chain is polled before each one, so a new gyro
    // sample waits at most one task's duration. (The old loop ran tasks in a fixed priority order and stopped when
    // the pass ran long, so the IMU read -- near the end of the list -- was the first thing starved.)
    for (size_t i = 0; i < TASK_COUNT; ++i) {
        TaskRunner& task = tasks[i];
        if (&task == &FLIGHT_CONTROL || !task.isEnabled()) {
            continue;
        }

        FLIGHT_CONTROL.process(0);
        handleLoopStop();

        // Background tasks may run up to 1ms early. That only shifts their timing slightly and keeps their rates as
        // before; the latency-critical work above is driven by the sensor's data-ready signal, not by this timer.
        task.process(1000);
        handleLoopStop();
    }
}
