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

bool updateStateEstimate() {
    sys.state.updateFilter(micros());
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

        if (sys.pwr.V0() < 2.4f) {
            critical_battery_counter++;
        } else {
            critical_battery_counter--;
        }
    } else {
        if (sys.pwr.V0() < 3.5f) {
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

#define AVERAGE_DELAY_TARGET_USEC hzToMicros(500)

TaskRunner tasks[] = {
    {"serial state out  ", serial_writeState, hzToMicros(1)},             // set dynamically; leave at index [0]
    {"SD state out      ", sd_writeState, hzToMicros(1)},                 // set dynamically; leave at index [1]
    {"loop count        ", updateLoopCount, AVERAGE_DELAY_TARGET_USEC},   // overall pacing
    {"update i2c        ", updateI2C, hzToMicros(400)},                   // update i2c often to reduce sensor latency
    {"get bluetooth     ", bluetooth_get, hzToMicros(400)},               // 
    {"get usb           ", usb_get, hzToMicros(400)},                     // 
    {"send SD block     ", sd_send, hzToMicros(400)},                     // make faster than fastest sd writes (200Hz goal)
    {"state estimate    ", updateStateEstimate, hzToMicros(350)},         // this takes about 2 msec; run at 2x gyro rate
    {"control vectors   ", updateControlVectors, hzToMicros(350)},        // match state updates
    {"run imu           ", performInertialMeasurement, hzToMicros(175)},  // gyro rate is 184Hz
    {"process bluetooth ", bluetooth_processCommand, hzToMicros(40)},     // we expect 20Hz rcData; oversample
    {"pilot input       ", processPilotInput, hzToMicros(40)},            // cPPM signals come at ~40Hz
    {"update lights     ", updateIndicatorLights, hzToMicros(30)},        // keep at 30Hz for visual effects
    {"pressure sensor   ", processPressureSensor, hzToMicros(26.3)},      // bmp280 datasheet output data rate
    {"autopilot         ", runAutopilot, hzToMicros(1), false},           // (future feature; turn off for now)
    {"send usb          ", usb_send, hzToMicros(100)},                    // 
    {"send bluetooth    ", bluetooth_send, 33000 /*usec*/},               // Rigado BMDWare limited to 20B each 30msec
    {"check battery     ", checkBatteryUse, hzToMicros(10)},              // 
    {"update magnet     ", updateMagnetometer, hzToMicros(10)},           // 
    {"print tasks       ", printTasks, 30/*sec*/*1000*1000, true, true},  // debug printing interval, run? (t/f), always log stats
};

constexpr size_t TASK_COUNT = 20;

float sd_max_rate_Hz = 185.0f; //occasional stalls but manageable, actual rate is ~180Hz

void sdLogTest(){
    TaskRunner& t = tasks[1];
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
    sd_max_rate_Hz += 10.0;
}

void performanceReport(){
    Serial.printf("\n[%10" PRIu32 "] Performance Report (Hz and ms): \n", micros());
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
        float rate = (task.log_count * 1000000.0f) / ((float) task.delay_track.value_sum);
        float average_duration_msec = task.duration_track.value_sum / (1000.0f * task.log_count);
        processor_load_percent += 0.1 * average_duration_msec * rate; // 100%/1000 msec *  msec/cycle * cycles/second

        Serial.printf("[%s %11d] rate: %7.2f       delay: %9.2f %9.2f %9.2f        duration: %9.2f %9.2f %9.2f\n", 
                      task.name, task.work_count, rate, 
                      task.delay_track.value_min / 1000.0f, task.delay_track.value_sum / (1000.0f * task.log_count), task.delay_track.value_max / 1000.0f, 
                      task.duration_track.value_min / 1000.0f, average_duration_msec, task.duration_track.value_max / 1000.0f);
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

void loop() {
    
    uint32_t start = micros();
    
    if (loops::stopped()) {
        Serial.println("ERROR: loops stopped?!?!");
        sys.led.errorStart(LEDPattern::SOLID, CRGB::White, CRGB::Red, 2);
        sys.led.errorStop();
        return;
    }
    
    tasks[0].setDesiredInterval(sys.conf.GetSendStateDelay() * 1000, 1000*1000);  
    tasks[1].setDesiredInterval(max( hzToMicros(sd_max_rate_Hz), sys.conf.GetSdCardStateDelay() * 1000), 1000*1000);
    
    for (size_t i = 0; i < TASK_COUNT; ++i) {
        if (loops::used()) { // process any stop immediately in case other tasks were scheduled for this iteration!
            for (TaskRunner& task : tasks) {
                task.reset(loops::lastStart());
            }
            loops::reset();
        }
        
        TaskRunner& task = tasks[i];

        if (task.isEnabled()){
            task.process();
        }

        if ( !loops::used() &&  (micros() - start) > AVERAGE_DELAY_TARGET_USEC ) {
            average_delay_usec = ( average_delay_usec*15 + (micros()-start) ) >> 4;
            if (average_delay_usec > 2*AVERAGE_DELAY_TARGET_USEC) {
                //indicate slow loops
                Serial.println(average_delay_usec, DEC);
            }
            //break;
        }
    }
}
