#ifndef SERIAL_SUBSTATES_H
#define SERIAL_SUBSTATES_H

template <>
inline void SerialComm::readSubstate<SerialComm::MICROS>(CobsPayloadGeneric& payload) const {
    uint32_t timestamp_us{micros()};
    payload.Append(timestamp_us);
}

template <>
inline void SerialComm::readSubstate<SerialComm::STATUS>(CobsPayloadGeneric& payload) const {
    payload.Append(state->status);
}

template <>
inline void SerialComm::readSubstate<SerialComm::V0>(CobsPayloadGeneric& payload) const {
    payload.Append(state->V0_raw);
}

template <>
inline void SerialComm::readSubstate<SerialComm::I0>(CobsPayloadGeneric& payload) const {
    payload.Append(state->I0_raw);
}

template <>
inline void SerialComm::readSubstate<SerialComm::I1>(CobsPayloadGeneric& payload) const {
    payload.Append(state->I1_raw);
}

template <>
inline void SerialComm::readSubstate<SerialComm::ACCEL>(CobsPayloadGeneric& payload) const {
    payload.Append(state->accel);
}

template <>
inline void SerialComm::readSubstate<SerialComm::GYRO>(CobsPayloadGeneric& payload) const {
    payload.Append(state->gyro);
}

template <>
inline void SerialComm::readSubstate<SerialComm::MAG>(CobsPayloadGeneric& payload) const {
    payload.Append(state->mag);
}

template <>
inline void SerialComm::readSubstate<SerialComm::TEMPERATURE>(CobsPayloadGeneric& payload) const {
    payload.Append(state->temperature);
}

template <>
inline void SerialComm::readSubstate<SerialComm::PRESSURE>(CobsPayloadGeneric& payload) const {
    payload.Append(state->pressure);
}

template <>
inline void SerialComm::readSubstate<SerialComm::RX_PPM>(CobsPayloadGeneric& payload) const {
    for (std::size_t i = 0; i < 6; ++i) {
        payload.Append(ppm[i]);
    }
}

template <>
inline void SerialComm::readSubstate<SerialComm::AUX_CHAN_MASK>(CobsPayloadGeneric& payload) const {
    payload.Append(state->command_AUX_mask);
}

template <>
inline void SerialComm::readSubstate<SerialComm::COMMANDS>(CobsPayloadGeneric& payload) const {
    payload.Append(state->command_throttle, state->command_pitch, state->command_roll, state->command_yaw);
}

template <>
inline void SerialComm::readSubstate<SerialComm::F_AND_T>(CobsPayloadGeneric& payload) const {
    payload.Append(state->Fz, state->Tx, state->Ty, state->Tz);
}

template <>
inline void SerialComm::readSubstate<SerialComm::PID_FZ_MASTER>(CobsPayloadGeneric& payload) const {
    WritePIDData(payload, control->thrust_pid.master());
}

template <>
inline void SerialComm::readSubstate<SerialComm::PID_TX_MASTER>(CobsPayloadGeneric& payload) const {
    WritePIDData(payload, control->pitch_pid.master());
}

template <>
inline void SerialComm::readSubstate<SerialComm::PID_TY_MASTER>(CobsPayloadGeneric& payload) const {
    WritePIDData(payload, control->roll_pid.master());
}

template <>
inline void SerialComm::readSubstate<SerialComm::PID_TZ_MASTER>(CobsPayloadGeneric& payload) const {
    WritePIDData(payload, control->yaw_pid.master());
}

template <>
inline void SerialComm::readSubstate<SerialComm::PID_FZ_SLAVE>(CobsPayloadGeneric& payload) const {
    WritePIDData(payload, control->thrust_pid.slave());
}

template <>
inline void SerialComm::readSubstate<SerialComm::PID_TX_SLAVE>(CobsPayloadGeneric& payload) const {
    WritePIDData(payload, control->pitch_pid.slave());
}

template <>
inline void SerialComm::readSubstate<SerialComm::PID_TY_SLAVE>(CobsPayloadGeneric& payload) const {
    WritePIDData(payload, control->roll_pid.slave());
}

template <>
inline void SerialComm::readSubstate<SerialComm::PID_TZ_SLAVE>(CobsPayloadGeneric& payload) const {
    WritePIDData(payload, control->yaw_pid.slave());
}

template <>
inline void SerialComm::readSubstate<SerialComm::MOTOR_OUT>(CobsPayloadGeneric& payload) const {
    payload.Append(state->MotorOut);
}

template <>
inline void SerialComm::readSubstate<SerialComm::KINE_ANGLE>(CobsPayloadGeneric& payload) const {
    payload.Append(state->kinematicsAngle);
}

template <>
inline void SerialComm::readSubstate<SerialComm::KINE_RATE>(CobsPayloadGeneric& payload) const {
    payload.Append(state->kinematicsRate);
}

template <>
inline void SerialComm::readSubstate<SerialComm::KINE_ALTITUDE>(CobsPayloadGeneric& payload) const {
    payload.Append(state->kinematicsAltitude);
}

template <>
inline void SerialComm::readSubstate<SerialComm::LOOP_COUNT>(CobsPayloadGeneric& payload) const {
    payload.Append(state->loopCount);
}

#endif /* SERIAL_SUBSTATES_H */
