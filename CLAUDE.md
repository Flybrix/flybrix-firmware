# CLAUDE.md

Flight-controller firmware for the Flybrix drone: an Arduino sketch for a **Teensy 3.2** (Cortex-M4 at 96 MHz, **no
FPU**). It drives a small brushed-motor quad flown from a phone app over Bluetooth, or from a PPM receiver.

**Read `docs/DEVELOPMENT.md` before changing anything.** It covers the architecture, build setup, safety behaviour,
tuning, debugging methods, pitfalls and known issues.

## Build

```sh
arduino-cli --config-file <cfg>.yaml compile -b "teensy:avr:teensy31:usb=serial,speed=96" \
  --build-path <build dir> <repo>
```

- Use Teensy core `teensy:avr@1.59.0`, with user libraries **SdFat 1.0.5** and **FastLED 3.1.6**.
- Install arduino-cli on a short path on Windows.
- Full setup is in `docs/DEVELOPMENT.md` → Building.

There are no unit tests. "It compiles" is the only check available without hardware. Behaviour changes need a flight
test by the owner.

## Rules

- **This code flies.** A bug can mean a crash, a flyaway or spinning props. Prefer small, explained changes. Say
  plainly what has and hasn't been tested on hardware, and tell the owner to test with **props off** first.
- **Don't flash the board yourself.** Give the owner the path to the built `.hex`; they flash it with the Teensy
  Loader. Automated upload doesn't work on their machine.
- **Keep `cxa_guard.cpp`.** It fixes a Teensyduino 1.59 core bug that otherwise hangs the board before `loop()`
  (no LEDs).
- **Never call library objects from global constructors** (FastLED, `Serial1`, the ADC library). Do it from
  `setup()`; static initialisation order across files is undefined.
- **Some code assumes 40 Hz.** `PilotCommand::processCommands()` runs at 40 Hz, and arming, hold-off, failsafe and RC
  timeout counters count those ticks. Rescale all of them if that rate changes.
- **Task table indexes:** if you reorder `tasks[]` in `flybrix-firmware.ino`, fix the `#define`s after it
  (`FLIGHT_CONTROL`, `SD_STATE_OUT`, `SERIAL_STATE_OUT`) and `TASK_COUNT`.
- **EEPROM settings:** changing a default in code doesn't reach existing boards. The owner can't use the configurator,
  so ship such changes as a one-time migration in `readEEPROM()` (`config.cpp`): replace the value only if it exactly
  equals the old default. Don't change the packed config structs' layout; their sizes are `static_assert`ed and the
  configurator protocol depends on them.
- **Keep work out of the flight chain.** `runFlightControl()` runs at 500 Hz on a CPU without an FPU. After adding
  work there, check CPU load with the performance report ("print tasks" task).
- **Quaternion naming:** `Quaternion::pitch()` is the rotation about X and `roll()` about Y. The UKF uses the opposite
  names, and the code already accounts for that. Don't "fix" it.

## Where things are

| Topic | Files |
| --- | --- |
| Main loop, task table, flight chain | `flybrix-firmware.ino` |
| Sensors | `MPU9250.*`, `AK8963.*`, `BMP280.*`, `imu.*`, `i2cManager.*` |
| Attitude and altitude estimation | `ahrs.*`, `state.*`, `localization.*`, `ukf.*` |
| PID cascades and stick smoothing | `control.*`, `PID.h`, `PIDCascade.h` |
| Mixer and motors | `airframe.*`, `motors.*` |
| Arming, failsafe, RC sources | `command.*`, `receiver.*`, `utility/rcHelpers.h`, `serial.*`, `serialFork.*` |
| Configuration and EEPROM migrations | `config.*` |
| Watchdog | `watchdog.*`, `loop_stopper.*` |

## Open issues (as of the last flight test)

- Still shaky in a hover, and pushed around in ground effect. The type of shake hasn't been identified yet; ask the
  owner first. Likely next step: rate D and data-driven tuning from SD logs.
- No position hold: there's no position sensor.
- Altitude hold should use a downward time-of-flight sensor, not the barometer.

Details are in `docs/DEVELOPMENT.md` → Status and known issues.
