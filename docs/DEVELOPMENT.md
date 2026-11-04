# Flybrix firmware: developer guide

How the firmware is built, how it works, what has been changed on the `stability-latency-fixes` branch, and what
to watch out for. Written for anyone picking the code up, human or AI.

## Contents

1. [Hardware](#hardware)
2. [Building](#building)
3. [Flashing](#flashing)
4. [Architecture](#architecture)
5. [Safety behaviour](#safety-behaviour)
6. [Tuning parameters](#tuning-parameters)
7. [Debugging on the board](#debugging-on-the-board)
8. [Pitfalls](#pitfalls)
9. [Status and known issues](#status-and-known-issues)

## Hardware

| Part | Details |
| --- | --- |
| MCU | Teensy 3.2 (NXP MK20DX256, Cortex-M4 at 96 MHz). **No FPU**: all `float` maths is software-emulated, so floating point is expensive. 64 KB RAM, 256 KB flash. |
| IMU | InvenSense MPU9250 (accelerometer and gyro) on I2C at 400 kHz. Its data-ready line goes to `board::MPU_INTERRUPT`. |
| Magnetometer | AK8963 (inside the MPU9250), reached through the MPU's I2C bypass. |
| Barometer | Bosch BMP280 on the same I2C bus, in normal mode at about 26 Hz. |
| Motors | 8 brushed-motor PWM channels, 12-bit at 11.7 kHz (`motors.cpp`). The default mix table is a flat octocopter; a quad uses CH0, CH1, CH6 and CH7. |
| RC input | PPM receiver on pin 3, timed with FTM1 input capture (`receiver.cpp`). |
| Bluetooth | A Rigado BMDware module on `Serial1`, limited to about 20 bytes every 30 ms. The phone app sends stick commands this way. |
| LEDs | WS2812B strip on pin 11 (FastLED), plus separate red and green indicator LEDs. |
| Storage | microSD (SdFat) for flight logs; EEPROM for configuration. |

Board schematics and placement drawings are in `documentation/`.

## Building

The README describes the original toolchain: Arduino 1.8.5, Teensyduino 1.41 and SdFat 1.0.5. That still works if you
install it.

The branch was developed with a current toolchain on `arduino-cli`. It needed three fixes, all now in the code (see
[Pitfalls](#pitfalls)).

### With arduino-cli (Windows example)

1. Put arduino-cli in a **short path** such as `C:\Users\<you>\arduino-cli`. Deep paths break gcc's include search
   on Windows (`fatal error: bits/c++config.h: No such file or directory`).
2. Create `arduino-cli.yaml` next to it:

   ```yaml
   board_manager:
     additional_urls:
       - https://www.pjrc.com/teensy/package_teensy_index.json
   directories:
     data: C:\Users\<you>\arduino-cli\data
     downloads: C:\Users\<you>\arduino-cli\dl
     user: C:\Users\<you>\arduino-cli\user
   ```

3. Install the core and libraries:

   ```sh
   arduino-cli --config-file arduino-cli.yaml core update-index
   arduino-cli --config-file arduino-cli.yaml core install teensy:avr@1.59.0
   arduino-cli --config-file arduino-cli.yaml lib install SdFat@1.0.5
   arduino-cli --config-file arduino-cli.yaml lib install FastLED@3.1.6
   ```

   - **Use core 1.59.0.** Core 1.62.0 (gcc 15) fails inside its own C++ headers for Teensy 3.x.
   - **SdFat 1.0.5 is required.** The code uses the SdFat 1.x API, and the user library overrides the SdFat 2.x bundled
     with the core.
   - **FastLED 3.1.6** is the version the flight-tested builds used. It prints about 800 harmless warnings with gcc 11.
     The bundled FastLED 3.4 should also work now that `cxa_guard.cpp` exists, but it hasn't been flown.

4. Build:

   ```sh
   arduino-cli --config-file arduino-cli.yaml compile \
     -b "teensy:avr:teensy31:usb=serial,speed=96" \
     --build-path <build dir> <path to flybrix-firmware>
   ```

   The result is `<build dir>/flybrix-firmware.ino.hex`. A clean build reports about 155 KB of flash (59%) and 49 KB of
   RAM (74%).

## Flashing

Open the `.hex` in the **Teensy Loader** app (File → Open HEX File), then press the program button on the board.

`arduino-cli upload` and `teensy_post_compile` could not reach the Teensy Loader from an automated shell on the
development machine. Plan on flashing by hand.

**Take the props off** for the first boot of any new build.

## Architecture

### Boot (`setup()` in `flybrix-firmware.ino`)

1. `LED_driver.begin()` registers the LED strip with FastLED. This must happen first, and must stay out of global
   constructors.
2. I2C starts (`Wire.begin`, then `i2c().begin()` for timeouts).
3. The configuration is loaded from EEPROM (`readEEPROM().applyTo(sys)`), including the one-time migrations.
4. Sensors start in order: barometer, IMU, magnetometer.
   - If a sensor's ID check fails, setup stops in `while(1)`.
   - Each phase is shown as a solid LED pattern. The strip is only refreshed from the main loop, though, so these
     patterns may not actually appear.
5. SD card probe (about 2 s without a card), then Bluetooth AT setup (about 2.5 s). Factory test mode runs if the
   EEPROM is empty.
6. `loops::start()`, then `watchdog::begin()` comes last.

All subsystems live in one global `Systems sys` (`systems.h`). Its member order is the construction order.

### Main loop and scheduler

- `loop()` visits every entry in the `tasks[]` table once per pass. Each `TaskRunner` runs its function when its
  interval has elapsed. Background tasks may run up to 1 ms early (`process(1000)`).
- The flight-control task (`tasks[1]`) has interval 0 and is **polled before every other task**. A new gyro sample
  therefore waits at most one task's duration.
- `loops::Stopper` (`loop_stopper.h`) marks deliberate long blocking work: SD open/close, EEPROM writes, reports. It
  pauses the watchdog, and afterwards every task timer is reset so nothing runs a burst of catch-up work.
- `#define`s right after the table refer to entries by index (`FLIGHT_CONTROL`, `SD_STATE_OUT`,
  `SERIAL_STATE_OUT`). **Update them if you reorder the table**, and keep `TASK_COUNT` equal to the number of
  entries.

### Flight-control chain (`runFlightControl()`)

The steps run in order every time the MPU9250 raises data-ready (500 Hz):

1. **Read the sample.** `Imu::startInertialMeasurement()` queues a 14-byte read, and the I2C queue is serviced until it
   completes or fails.
2. **Update the attitude.** The read's callback runs the update straight away: `Imu::updateAccelGyro` →
   `State::updateLocalization` → `Ahrs::update` (Madgwick).
3. **Run the controller.** `updateControlVectors()` → `Control::calculateControlVectors` (the PID cascades) →
   `PilotCommand::applyControl` → `Airframe` mixer → PWM.

A failed read does not re-run the controller on old data.

- The sample rate is `1 kHz / (1 + MPU9250::SAMPLE_RATE_DIVIDER)`; the divider is 1, so 500 Hz. If the CPU is
  overloaded, use 2 (333 Hz).
- Every sample is read exactly once, so the chip's filters must suit the read rate. The gyro filter is 92 Hz and the
  accelerometer filter 41 Hz (`MPU9250::configure`).

### Sensors and I2C (`i2cManager.*`)

- There is a single I2C bus. `addTransfer()` queues a transfer in a fixed 8-slot ring buffer and never allocates
  memory.
- `update()` performs one transfer. It blocks, but `TIMEOUT_USEC` (2 ms) bounds it. It then calls the callback
  straight away with `ok = true` or `ok = false`.
- After 3 failures in a row it calls `Wire.resetBus()`.
- The magnetometer (10 Hz) and barometer (about 26 Hz) are read in the background by the "update i2c" task, and by the
  flight chain whenever their transfers are queued ahead of the IMU's.
- Device drivers set `ready = false` while a transfer is pending, and set it back on both success and failure.
- Keep callback captures small (`[this]`) so `std::function` stores them inline instead of on the heap.

### State estimation

- **Attitude (`ahrs.cpp`):** a Madgwick filter.
  - `beta` (`state.parameters.state_estimation[0]`, default 0.1) is the fastest rate, in rad/s, at which the
    accelerometer may pull the estimate.
  - Accelerometer samples more than 15% away from 1 g are ignored.
  - The magnetometer only corrects heading (`correctHeading`): a rotation about the vertical axis, so it can never
    tilt roll or pitch.
  - Angles are read from the quaternion in `State::updateLocalization`. Note: `Quaternion::pitch()` is the rotation
    about **X** and `roll()` about **Y**.
- **Altitude (`ukf.*`, `localization.*`):** a small UKF on barometer elevation.
  - It runs one predict and update per barometer sample, split over two calls of "state estimate".
  - It has unused measurement slots for optical flow (`vu`, `vv`) and a downward distance sensor (`d_tof`).
  - The UKF's `update(..., roll, pitch)` uses roll = rotation about X, so `localization.cpp` correctly passes
    `pose().pitch()` as its roll.

### Control (`control.cpp`, `PID.h`, `PIDCascade.h`)

- Each axis is a `PIDCascade` of up to 4 regulators: velocity master, velocity slave, angle master, rate slave.
- Bits in `pid_bypass` (stored in EEPROM) turn individual stages off.
- The default is angle (self-levelling) mode on pitch and roll, rate mode on yaw, and direct throttle. Altitude and
  velocity control exist but are bypassed.
- The `PID` class:
  - The derivative term is computed from the measurement, not the error, so setpoint steps don't kick it. Kd is 0 in
    the defaults.
  - The time step is clamped to `MAX_DELTA_TIME` (20 ms), and Ki = 0 is guarded against.
  - `setTimer()` clears the derivative history.
- Stick commands are smoothed by a 25 ms low-pass (`STICK_SMOOTHING_SEC`). Throttle below 10 is checked *unsmoothed*,
  so the motors cut immediately.
- The mixer (`airframe.cpp`) allocates in order: thrust, then pitch and roll together (scaled uniformly, so neither
  axis loses authority first), then yaw, then any residual thrust.

### Pilot command state machine (`command.cpp`)

- `processCommands()` runs at **40 Hz** ("pilot input"). **Several counters assume that rate**:
  - arming checks at 20 and 40 ticks
  - the 80-tick throttle hold-off
  - the RX tolerance and freshness tickers in `receiver.*` and `utility/rcHelpers.h`
  - the failsafe decay and tick limits

  If you change the rate, rescale all of them.
- Arming: AUX1 low → `Enabling`. The IMU must be healthy, upright and stable for about 1 s, then `ThrottleLocked`, then
  `Enabled` once the throttle is at zero.
- RC sources are combined by `RcMux<SerialRc, Receiver>`, with source 0 = Bluetooth/USB serial taking priority over
  source 1 = PPM. `lastSource()` reports which one supplied the last command.
- Bluetooth messages are processed **the moment their last byte arrives** (`onBluetoothMessage`). The receive buffer
  holds only one message, and the next byte overwrites it.
  - New stick values are applied immediately through `PilotCommand::refreshSticks()`.
  - This only happens while the state machine is passing sticks through unchanged (armed, flying, link up). Arming
    and failsafe logic still run at 40 Hz.

### Configuration and EEPROM (`config.*`)

- `Config::Data` is a `std::tuple` of packed structs, written to EEPROM as one 837-byte block. The struct sizes are
  pinned by `static_assert`s, and the configurator (serial protocol) depends on the layout, so **don't change the
  structs casually**.
- Changing a default in code only affects boards whose EEPROM is reset. To move existing boards to a new default
  without the configurator, add a one-time migration in `readEEPROM()`: if the stored value exactly equals the *old*
  default, replace it and write the config back. There are two today: Madgwick beta 1.0 → 0.1, and pitch/roll rate
  P 5.0 → 3.5.
- Packed structs: binding a `float&` to a packed member does not compile. Take a reference to the whole struct instead.

### Communication

- Messages are COBS-framed (`cobs.h`) over USB serial and Bluetooth. Commands are bitmask-selected subcommands
  (`serial_subcommands.h`); the stick command is `SET_SERIAL_RC`.
- State streaming to USB/Bluetooth is off by default (`send_state_delay` 1001 means off), so the board **sends
  nothing unprompted**.
- SD logging writes state at the highest rate. The file is opened on arming and closed on disarming.

## Safety behaviour

| Situation | What happens | Where |
| --- | --- | --- |
| RC link lost (about 1 s without frames) | Levels pitch and roll, zeroes yaw, and multiplies the throttle by 0.99 every 25 ms. It cuts the motors when the throttle reaches about zero, after 10 s, or if tilt exceeds 60°. | `PilotCommand::startFailsafe` / `failsafeCommand` |
| RC link back | If still armed with AUX1 low, control is handed back (`Enabled`); otherwise the motors are cut and re-arming is required. | `endFailsafe` |
| No IMU sample for 50 ms | Motors cut (`FailStability`); the pilot must cycle the arm switch. Arming is refused while samples are stale. | `checkImuHealth`, `Imu::healthy` |
| I2C error or NACK | The transfer fails and is dropped, with a callback reporting the failure. The bus is reset after 3 errors in a row. | `I2CManager` |
| Main loop hangs for 250 ms | The hardware watchdog resets the MCU, so the motors stop instead of holding their last PWM. It is paused inside `loops::Stopper`. | `watchdog.*`, `loop_stopper.cpp` |
| Short or garbled PPM frame | Rejected: all 6 channels must arrive. | `receiver.cpp` ISR |

The failsafe constants at the top of `command.cpp` are first guesses. Check them with a tethered test.

## Tuning parameters

| What | Where | Default |
| --- | --- | --- |
| Rate loop (pitch/roll slave P, I, D) | `Control::PIDParameters` in `control.cpp` (EEPROM) | P 3.5, I 2.0, D 0 |
| Angle loop (pitch/roll master) | same | P 3.5, I 0.5; 30° at full stick |
| Yaw rate | same | P 40, I 10; 240°/s at full stick |
| Madgwick beta | `State::Parameters` in `state.cpp` (EEPROM) | 0.1 |
| Accelerometer gate | `ACCEL_GATE_FRACTION` in `ahrs.cpp` | ±15% of 1 g |
| Heading correction | `MAG_HEADING_GAIN` in `ahrs.cpp` | 0.05 per sample (about a 2 s time constant) |
| Stick smoothing | `Control::STICK_SMOOTHING_SEC` | 25 ms |
| IMU rate | `MPU9250::SAMPLE_RATE_DIVIDER` | 1 (500 Hz) |
| Failsafe | constants at the top of `command.cpp` | see table above |

EEPROM values can be changed with the Flybrix configurator, or in code through a migration (see above).

## Debugging on the board

- The board appears as a USB serial port (COM7 on the development machine) and stays quiet until asked. A
  PowerShell `System.IO.Ports.SerialPort` with `DtrEnable = $true` can read it.
- **Performance report:** enable the "print tasks" entry (`tasks[0]`, currently disabled). Every 20 s it prints each
  task's rate and timing over USB, and the total CPU load.
- **If the board hangs before the main loop** (no LEDs), these techniques found the `cxa_guard` bug. Use them in a
  throwaway copy of the sketch:
  1. Define `extern "C" void startup_late_hook()` to wait up to 30 s for `Serial`. USB is already running, and global
     constructors haven't started yet.
  2. Print markers between subsystem constructors, and at each step of `setup()`.
  3. Override `hard_fault_isr` with a naked handler that prints the stacked PC, LR and fault registers, keeping USB
     alive by calling `usb_isr()` in its loop.
  4. Start a once-a-second `IntervalTimer` that prints the interrupted PC and the code addresses found on the stack.
     Bound that stack scan: the top of RAM is `0x20008000`.
  5. Map the addresses to source lines with
     `arm-none-eabi-addr2line -f -C -i -e <build>/flybrix-firmware.ino.elf <addr>` (the compiler is in the
     `teensy-compile` tool folder).
- The Teensy 3 core's `fault_isr` / `unused_isr` loop forever while keeping USB alive. A board that opens its COM
  port but never talks is often stuck there.

## Pitfalls

- **`cxa_guard.cpp` must stay.** Teensyduino 1.59's Teensy 3 core defines weak `__cxa_guard_acquire` and
  `__cxa_guard_release` that treat the 32-bit ARM guard as 64 bits.
  - The effect: a function-local `static` object can be silently skipped, and the 4 bytes after its guard get
    overwritten.
  - FastLED's controller was hit by this. Its virtual call went through a null vtable into the vector table, and the
    board hung before `loop()` with no LEDs.
- **Static initialisation order across files is undefined.** Don't call library objects (FastLED, `Serial1`, the ADC
  library) from global constructors; do it from `setup()`.
  - `Systems sys` is constructed before `LED_driver`, so `sys` currently writes to `LED_driver` before
    `LED_driver`'s constructor runs. It's harmless only because that constructor merely resets the colours.
- **No FPU.** Floating-point maths is expensive. The altitude filter's predict step takes about 1.3 ms, and the
  attitude update plus control at 500 Hz is a significant share of the CPU. Check the performance report after adding
  work to the flight chain.
- **`static constexpr` class members** may only be used by value (in comparisons, or passed as `float` /
  `uint32_t`). Binding one to a `const&` (`std::min`, `std::max`) needs an out-of-class definition, or the link fails.
- **Interrupt handlers:** the Teensy core aliases every unhandled interrupt to `unused_isr` → `fault_isr`, an
  infinite loop. Enabling an interrupt without defining its `*_isr` hangs the board.
- **Pure virtual calls** end in `__cxa_pure_virtual()`, also an infinite loop.
- **Line endings:** most files are CRLF. Git's autocrlf normalises them on commit.
- **Timing-coupled code:** see the 40 Hz note under the pilot state machine.

## Status and known issues

As of the last flight test:

- **It flies, and better than the original firmware.** The pilot still reports shaking in a hover, and the craft gets
  pushed around a lot in ground effect.
  - Lowering the rate P from 5.0 to 3.5 and adding stick smoothing did not cure the shake. Lower P also weakens
    resistance to disturbances.
  - The *kind* of shake (fast buzz, slow wobble, or random twitches) hasn't been established; find that out first.
  - Likely next steps: add rate D (the code supports it; Kd is 0), possibly restore P, and use SD-card flight logs to
    tune from data rather than guesswork.
- **No position hold.** There's no sensor for horizontal position or velocity. An optical-flow sensor would be needed
  (the UKF has `vu` / `vv` slots for one).
- **Altitude hold was deliberately not built on the barometer.** Prop wash and ground effect make it unreliable for a
  small quad flown low. A downward time-of-flight sensor (for example a VL53L1X on the I2C bus, using the UKF's
  `d_tof` slot) is the recommended route.
- **Bluetooth latency:** the firmware-side delays are removed. The BLE link (about 30 ms per 20-byte packet) and the
  phone app are the remaining floor. A conventional RC transmitter on the PPM input is lower latency.
- **Failsafe constants** (decay rate, 10 s limit, 60° crash angle) are untested estimates.
- **Unexplained:** the original firmware's undefined `Channel` virtuals linked under the original toolchain. They're
  now pure virtual, which is the correct form.
