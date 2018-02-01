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

bool writeToSerial() {
    sys.conf.SendState();
    return true;
}

bool writeToSdCard() {
    sys.conf.SendState(0xFFFFFFFF, true);
    return true;
}

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

bool processSerialInput() {
    return sys.conf.Read();
}

bool updateStateEstimate() {
    sys.state.updateFilter(micros());
    return true;
}

bool runAutopilot() {
    return sys.autopilot.run(micros());
}

bool flushBluetoothSerial() {
    return (flushSerial(1) > 0);
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
    } else {
        if (sys.pwr.V0() < 3.5f) {
            low_battery_counter++;
        } else {
            low_battery_counter--;
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

    return true;
}

bool updateMagnetometer() {
    return sys.imu.startMagnetFieldMeasurement();
}

bool performInertialMeasurement() {
    return sys.imu.startInertialMeasurement();
}

bool printTasks();

TaskRunner tasks[] = {
    {writeToSerial, hzToMicros(1)},                 //
    {writeToSdCard, hzToMicros(1)},                 //
    {updateLoopCount, hzToMicros(1000)},            //
    {updateI2C, hzToMicros(800)},                   //
    {updateIndicatorLights, hzToMicros(30)},        //
    {processPressureSensor, hzToMicros(100)},       //
    {processSerialInput, hzToMicros(100)},          //
    {updateStateEstimate, hzToMicros(200)},         //
    {runAutopilot, hzToMicros(100)},                //
    {flushBluetoothSerial, hzToMicros(200)},        //
    {updateControlVectors, hzToMicros(400)},        //
    {processPilotInput, hzToMicros(40)},            //
    {checkBatteryUse, hzToMicros(10)},              //
    {updateMagnetometer, hzToMicros(10)},           //
    {performInertialMeasurement, hzToMicros(400)},  //
    {printTasks, hzToMicros(1)},                    //
};

const char* task_names[] = {
    "write to serial",  //
    "write to sd    ",  //
    "loop count     ",  //
    "update i2c     ",  //
    "update lights  ",  //
    "pressure sensor",  //
    "serial input   ",  //
    "state estimate ",  //
    "autopilot      ",  //
    "flush bluetooth",  //
    "control vectors",  //
    "pilot input    ",  //
    "check battery  ",  //
    "run magnet     ",  //
    "run imu        ",  //
    "print tasks    ",  //
};

constexpr size_t TASK_COUNT = 16;

bool printTasks() {
    if (usb_mode::get() != usb_mode::PERFORMANCE_REPORT) {
        return false;
    }
    
    loops::Stopper _stopper;

    Serial.printf("\nPerformance report (Hz and ms):\n");

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
            uint32_t task_d = (!task.call_count) ? 10000000+j : task.delay_track.value_max;
            uint32_t mintask_d = (!mintask.call_count) ?  20000000 : mintask.delay_track.value_max;
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
        if (!task.call_count) {
            Serial.printf("[%s %11d]\n", task_names[s[i]], 0);
            continue;
        }
        float rate = (task.call_count * 1000000.0f) / ((float) task.delay_track.value_sum);
        float average_duration_msec = task.duration_track.value_sum / (1000.0f * task.call_count);
        
        processor_load_percent += 0.1 * average_duration_msec * rate; // 100%/1000 msec *  msec/cycle * cycles/second
        
        
        Serial.printf("[%s %11d] rate: %7.2f    delay: %7.2f %7.2f %7.2f      duration: %7.2f %7.2f %7.2f\n", 
                      task_names[s[i]], task.work_count, rate, 
                      task.delay_track.value_min / 1000.0f, task.delay_track.value_sum / (1000.0f * task.call_count), task.delay_track.value_max / 1000.0f, 
                      task.duration_track.value_min / 1000.0f, average_duration_msec, task.duration_track.value_max / 1000.0f);
    }
    printSerialReport();
    
    Serial.printf("Average processor use is %4.2f%%.\n", processor_load_percent);

    Serial.flush();
    return true;
}

void setup() {
    debug_serial_comm = &sys.conf;

    // MPU9250 is limited to 400kHz bus speed.
    Wire.begin(I2C_MASTER, 0x00, board::I2C_PINS, board::I2C_PULLUP, I2C_RATE_400);  // For I2C pins 18 and 19
    sys.flag.set(Status::BOOT);
    sys.led.update();

    bool go_to_test_mode{isEmptyEEPROM()};

    // load stored settings (this will reinitialize if there is no data in the EEPROM!
    readEEPROM().applyTo(sys);
    sys.state.resetState();

    sys.version.display(sys.led);

    setBluetoothUart(sys.name);

    sys.flag.set(Status::BMP_FAIL);
    sys.led.update();
    sys.bmp.restart();
    if (sys.bmp.getID() == 0x58) {
        sys.flag.clear(Status::BMP_FAIL);
        // state is unhappy without an initial pressure
        sys.bmp.startMeasurement();     // important; otherwise we'll never set ready!
        i2c().update();                 // write data
        delay(2);                       // wait for data to arrive
        i2c().update();                 // read data
        sys.bmp.p0 = sys.bmp.pressure;  // initialize reference pressure
    } else {
        sys.led.update();
        while (1)
            ;
    }

    sys.flag.set(Status::MPU_FAIL);
    sys.led.update();
    sys.imu.restart();
    if (sys.imu.hasCorrectIDs()) {
        sys.flag.clear(Status::MPU_FAIL);
        sys.imu.initialize();
    } else {
        sys.led.update();
        while (1)
            ;
    }

    sys.led.update();

    // factory test pattern runs only once
    if (go_to_test_mode)
        runTestMode(sys.state, sys.led, sys.pilot);

    // Perform intial check for an SD card
    sdcard::startup();

    sys.flag.clear(Status::BOOT);
    sys.led.update();

    loops::start();
}

void loop() {
    if (loops::stopped()) {
        return;
    }

    if (loops::consumeStop()) {
        for (TaskRunner& task : tasks) {
            task.reset();
        }
    }

    tasks[0].setDesiredInterval(sys.conf.GetSendStateDelay() * 1000);
    tasks[1].setDesiredInterval(sys.conf.GetSdCardStateDelay() * 1000);

    for (TaskRunner& task : tasks) {
        task.process();
    }
}
