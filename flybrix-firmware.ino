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
#include "R415X.h"
#include "airframe.h"
#include "board.h"
#include "cardManagement.h"
#include "command.h"
#include "config.h"
#include "control.h"
#include "debug.h"
#include "i2cManager.h"
#include "led.h"
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
#include "version.h"

Systems sys;
uint8_t low_battery_counter = 0;
uint8_t critical_battery_counter = 0;

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
    sys.state.readStatePT(sys.bmp.p0, sys.bmp.pressure, sys.bmp.temperature);
    sys.bmp.startMeasurement();
    return true;
}

bool predict_phase{true};

bool updateStateEstimate() {
    if (predict_phase) {
        sys.state.predictFilter(micros());
    } else {
        sys.state.updateFilter();
    }
    predict_phase = !predict_phase;
    return true;
}

bool runAutopilot() {
    return sys.autopilot.run(micros());
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
    if (low_battery_counter > 40) {
        sys.flag.set(Status::BATTERY_LOW);
        low_battery_counter = 40;
    }
    if (low_battery_counter < 2) {
        sys.flag.clear(Status::BATTERY_LOW);
        low_battery_counter = 2;
    }

    if (critical_battery_counter > 40) {
        sys.flag.set(Status::BATTERY_CRITICAL);
        critical_battery_counter = 40;
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

bool performInertialMeasurement() {
    return sys.imu.startInertialMeasurement();
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

bool bluetooth_get() {
    return bluetooth_getData();
}

bool bluetooth_processCommand() {
    return sys.conf.processBluetoothCommand();
}

#define AVERAGE_DELAY_TARGET_USEC 1400

TaskRunner tasks[] = {
    {"print tasks       ", printTasks, 20/*sec*/*1000*1000, false, true},  // ( pause ) debug printing interval, enabled? (t/f), force to always log stats
    {"loop count        ", updateLoopCount, AVERAGE_DELAY_TARGET_USEC},   // (<   5us)
    {"pilot input       ", processPilotInput, hzToMicros(40)},            // (< 160us) cPPM signals come at ~40Hz
    {"send bluetooth    ", bluetooth_send, 33000 /*usec*/},               // (<  20us) Rigado BMDWare limited to 20B each 30msec
    {"process bluetooth ", bluetooth_processCommand, hzToMicros(40)},     // (< 265us) we expect 20Hz rcData; oversample
    {"control vectors   ", updateControlVectors, hzToMicros(320)},        // (< 260us) match state updates
    {"send SD block     ", sd_send, hzToMicros(300)},                     // (< 220us) slower requires larger buffer
    {"update lights     ", updateIndicatorLights, hzToMicros(30)},        // (< 160us) keep at 30Hz for visual effects
    {"SD state out      ", sd_writeState, hzToMicros(1)},                 // (< 200us) set dynamically; set index in #define below [8]
    {"update i2c        ", updateI2C, hzToMicros(300)},                   // (< 465us) update often
    {"get bluetooth     ", bluetooth_get, hzToMicros(300)},               // (<  85us)
    {"serial state out  ", serial_writeState, hzToMicros(1)},             // (<  55us) set dynamically; set index in #define below [11]
    {"run imu           ", performInertialMeasurement, hzToMicros(160)},  // (<  30us) gyro rate is 184Hz
    {"pressure sensor   ", processPressureSensor, hzToMicros(26.3)},      // (<  30us) bmp280 datasheet output data rate
    {"check battery     ", checkBatteryUse, hzToMicros(10)},              // (<  30us)
    {"update magnet     ", updateMagnetometer, hzToMicros(10)},           // (<  30us)
    {"get usb           ", usb_get, hzToMicros(100)},                     // (?)
    {"send usb          ", usb_send, hzToMicros(100)},                    // (?)
    {"autopilot         ", runAutopilot, hzToMicros(1), false},           // (?) future feature; turn off for now
    {"state estimate    ", updateStateEstimate, hzToMicros(640)},         // (1340us + 700us) split in two calls; should be ~2x gyro rate
};
#define TASK_COUNT 20
#define SD_STATE_OUT tasks[8]
#define SERIAL_STATE_OUT tasks[11]
#define STATE_ESTIMATE tasks[19]

float sd_max_rate_Hz = 200.0f; // actual rate is ~197Hz

void sdLogTest(){
    TaskRunner& t = SD_STATE_OUT;
    if (t.log_count) {
        float rate = (t.log_count * 1000000.0f) / ((float) t.delay_track.value_sum);
        Serial.printf("%12" PRIu32 ", %12" PRIu32 ",  %12" PRIu32 ", %7.2f, %7.2f, ", micros(), t.work_count, t.log_count, sd_max_rate_Hz, rate);
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
    Serial.printf("\n[%10" PRIu32 "] Performance Report (Hz / us): \n", micros());
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
    loops::setLedIndicator(&sys.led);
    CRGB green = LED::fade(CRGB::Green);
    CRGB red = LED::fade(CRGB::Red);

    debug_serial_comm = &sys.conf;

    // MPU9250 is limited to 400kHz bus speed.
    Wire.begin(I2C_MASTER, 0x00, board::I2C_PINS, board::I2C_PULLUP, I2C_RATE_400);  // For I2C pins 18 and 19
    sys.led.errorStart(LEDPattern::SOLID, green, red, 0);

    //EEPROM.write(0, 255); //mark EEPROM empty for factory reset

    bool go_to_test_mode{isEmptyEEPROM()};

    // load stored settings (this will reinitialize if there is no data in the EEPROM!
    readEEPROM().applyTo(sys);

    sys.state.resetState();

    sys.led.errorStart(LEDPattern::SOLID, green, red, 1);
    sys.bmp.restart();
    if (sys.bmp.getID() == 0x58) {
        // state is unhappy without an initial pressure
        sys.bmp.startMeasurement();     // important; otherwise we'll never set ready!
        i2c().update();                 // write data
        delay(2);                       // wait for data to arrive
        i2c().update();                 // read data
        sys.bmp.p0 = sys.bmp.pressure;  // initialize reference pressure
    } else {
        while (1)
            ;
    }

    sys.led.errorStart(LEDPattern::SOLID, green, red, 2);
    sys.imu.restart();
    if (sys.imu.hasCorrectIDs()) {
        sys.imu.initialize();
    } else {
        while (1)
            ;
    }

    sys.led.update();

    // factory test pattern runs only once
    if (go_to_test_mode) {
        sys.led.errorStop();
        runTestMode(sys.state, sys.led, sys.pilot);
    }

    sys.led.errorStart(LEDPattern::SOLID, green, red, 3);

    // Perform intial check for an SD card
    sdcard::startup();

    sys.led.errorStop();
    sys.version.display(sys.led);

    // Do Bluetooth last becauase we have to wait 2.5 seconds for AT mode
    setBluetoothUart(sys.name);

    loops::start();
}

uint32_t average_delay_usec = 0;
bool state_update_only = true;

void loop() {

    uint32_t start = micros();

    if (loops::stopped()) {
        sys.led.errorStart(LEDPattern::SOLID, CRGB::White, CRGB::Red, 2);
        sys.led.errorStop();
        return;
    }

    // state updates take about as long as everything else combined; so we will force every odd iteration to run only the update
    if (state_update_only) {
        SERIAL_STATE_OUT.setDesiredInterval(sys.conf.GetSendStateDelay() * 1000, 1000*1000);
        SD_STATE_OUT.setDesiredInterval(max( hzToMicros(sd_max_rate_Hz), sys.conf.GetSdCardStateDelay() * 1000), 1000*1000);
        STATE_ESTIMATE.process(0); // don't run consecutively if we squeezed in an update last time
        state_update_only = false;
        return;
    }
    state_update_only = true;

    // on the even updates, we will run the state estimation last if there is time
    for (size_t i = 0; i < TASK_COUNT; ++i) {

        TaskRunner& task = tasks[i];

        if (task.isEnabled()){
            task.process( 1000 ); // state estimation expected runtime in usec

            if (loops::used()) { // process any stop immediately in case other tasks run during this iteration!
                for (TaskRunner& task : tasks) {
                    task.reset(loops::lastStart());
                }
                start = micros();
                loops::reset();
            }

            uint32_t delay_usec = micros() - start;

            if ( delay_usec > AVERAGE_DELAY_TARGET_USEC ) {
                average_delay_usec = ( average_delay_usec*15 + delay_usec ) >> 4;
                if (average_delay_usec > (1860)) {
                    DebugPrintf("Main loop is slower than expected (%d usec)!", average_delay_usec);
                }
                break;
            }
        }
    }
}
