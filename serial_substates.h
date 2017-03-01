#ifndef SERIAL_SUBSTATES_H
#define SERIAL_SUBSTATES_H

#include "serial_impl.h"

#include "kinematics.h"
#include "stateFlag.h"

enum SerialComm::States : uint8_t {
    MICROS,
    STATUS,
    V0,
    I0,
    I1,
    ACCEL,
    GYRO,
    MAG,
    TEMPERATURE,
    PRESSURE,
    RX_PPM,
    AUX_CHAN_MASK,
    COMMANDS,
    F_AND_T,
    PID_FZ_MASTER,
    PID_TX_MASTER,
    PID_TY_MASTER,
    PID_TZ_MASTER,
    PID_FZ_SLAVE,
    PID_TX_SLAVE,
    PID_TY_SLAVE,
    PID_TZ_SLAVE,
    MOTOR_OUT,
    KINE_ANGLE,
    KINE_RATE,
    KINE_ALTITUDE,
    LOOP_COUNT,
    END_OF_STATES,
};

static_assert(SerialComm::States::END_OF_STATES == 27, "Added/removed states not acknowledged");

// Reads part of the state into a COBS payload, named "payload"
#define READ_SUBSTATE(name) \
    template <>             \
    inline void SerialComm::readSubstate<SerialComm::States::name>(CobsPayloadGeneric & payload) const

// A shorthand for a common case where all we do is append some data to a payload
#define READ_SUBSTATE_PAYLOAD(name, ...) \
    READ_SUBSTATE(name) {                \
        payload.Append(__VA_ARGS__);     \
    }

READ_SUBSTATE(MICROS) {
    uint32_t timestamp_us{micros()};
    payload.Append(timestamp_us);
}

READ_SUBSTATE_PAYLOAD(STATUS, flag_.value())
READ_SUBSTATE_PAYLOAD(V0, state_.V0_raw)
READ_SUBSTATE_PAYLOAD(I0, state_.I0_raw)
READ_SUBSTATE_PAYLOAD(I1, state_.I1_raw)
READ_SUBSTATE_PAYLOAD(ACCEL, state_.accel)
READ_SUBSTATE_PAYLOAD(GYRO, state_.gyro)
READ_SUBSTATE_PAYLOAD(MAG, state_.mag)
READ_SUBSTATE_PAYLOAD(TEMPERATURE, bmp_.temperature)
READ_SUBSTATE_PAYLOAD(PRESSURE, bmp_.pressure)

READ_SUBSTATE(RX_PPM) {
    for (std::size_t i = 0; i < 6; ++i) {
        payload.Append(ppm[i]);
    }
}

READ_SUBSTATE_PAYLOAD(AUX_CHAN_MASK, state_.command_AUX_mask)
READ_SUBSTATE_PAYLOAD(COMMANDS, state_.command_throttle, state_.command_pitch, state_.command_roll, state_.command_yaw)
READ_SUBSTATE_PAYLOAD(F_AND_T, state_.Fz, state_.Tx, state_.Ty, state_.Tz)

READ_SUBSTATE(PID_FZ_MASTER) {
    WritePIDData(payload, control_.thrust_pid.master());
}

READ_SUBSTATE(PID_TX_MASTER) {
    WritePIDData(payload, control_.pitch_pid.master());
}

READ_SUBSTATE(PID_TY_MASTER) {
    WritePIDData(payload, control_.roll_pid.master());
}

READ_SUBSTATE(PID_TZ_MASTER) {
    WritePIDData(payload, control_.yaw_pid.master());
}

READ_SUBSTATE(PID_FZ_SLAVE) {
    WritePIDData(payload, control_.thrust_pid.slave());
}

READ_SUBSTATE(PID_TX_SLAVE) {
    WritePIDData(payload, control_.pitch_pid.slave());
}

READ_SUBSTATE(PID_TY_SLAVE) {
    WritePIDData(payload, control_.roll_pid.slave());
}

READ_SUBSTATE(PID_TZ_SLAVE) {
    WritePIDData(payload, control_.yaw_pid.slave());
}

READ_SUBSTATE(MOTOR_OUT) {
    airframe_.writeMotorsTo(payload);
}

READ_SUBSTATE_PAYLOAD(KINE_ANGLE, kinematics_.angle)
READ_SUBSTATE_PAYLOAD(KINE_RATE, kinematics_.rate)
READ_SUBSTATE_PAYLOAD(KINE_ALTITUDE, kinematics_.altitude)
READ_SUBSTATE_PAYLOAD(LOOP_COUNT, state_.loopCount)

#undef READ_SUBSTATE
#undef READ_SUBSTATE_PAYLOAD

#endif /* SERIAL_SUBSTATES_H */
