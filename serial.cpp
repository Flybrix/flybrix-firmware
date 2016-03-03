/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware
*/

#include "serial.h"
#include "state.h"

#include "config.h"  //CONFIG variable
#include "control.h"
#include "led.h"

namespace {
using CobsPayloadGeneric = CobsPayload<500>;  // impacts memory use only; packet size should be <= client packet size

template <std::size_t N>
inline void WriteProtocolHead(SerialComm::MessageType type, uint32_t mask, CobsPayload<N>& payload) {
    payload.Append(type);
    payload.Append(mask);
}

template <std::size_t N>
inline void WriteToOutput(CobsPayload<N>& payload, void (*f)(uint8_t*, size_t) = nullptr) {
    auto package = payload.Encode();
    Serial.write(package.data, package.length);
    if (f)
        f(package.data, package.length);
}

template <std::size_t N>
inline void WritePIDData(CobsPayload<N>& payload, const PID& pid) {
    payload.Append(pid.lastTime(), pid.input(), pid.setpoint(), pid.pTerm(), pid.iTerm(), pid.dTerm());
}
}

SerialComm::SerialComm(State* state, const volatile uint16_t* ppm, const Control* control, const CONFIG_union* config, LED* led) : state{state}, ppm{ppm}, control{control}, config{config}, led{led} {
}

void SerialComm::ReadData() {
    while (Serial.available()) {
        data_input.AppendToBuffer(Serial.read());

        if (!data_input.IsDone())
            continue;

        ProcessData();
    }
}

void SerialComm::ProcessData() {
    MessageType code;
    uint32_t mask;

    if (!data_input.ParseInto(code, mask))
        return;
    if (code != MessageType::Command)
        return;

    uint32_t ack_data{0};

    if (mask & COM_SET_EEPROM_DATA && data_input.ParseInto(config->raw)) {
        writeEEPROM();  // TODO: deal with side effect code
        ack_data |= COM_SET_EEPROM_DATA;
    }
    if (mask & COM_REINIT_EEPROM_DATA) {
        initializeEEPROM();  // TODO: deal with side effect code
        writeEEPROM();       // TODO: deal with side effect code
    }
    if (mask & COM_REQ_EEPROM_DATA) {
        SendConfiguration();
        ack_data |= COM_REQ_EEPROM_DATA;
    }
    if (mask & COM_REQ_ENABLE_ITERATION) {
        uint8_t flag;
        if (data_input.ParseInto(flag)) {
            if (flag == 1)
                state->processMotorEnablingIteration();
            else
                state->disableMotors();
            ack_data |= COM_REQ_ENABLE_ITERATION;
        }
    }
    // This should pass if any motor speed is set
    if (mask & COM_MOTOR_OVERRIDE_SPEED_ALL) {
        if (mask & COM_MOTOR_OVERRIDE_SPEED_0 && data_input.ParseInto(state->MotorOut[0]))
            ack_data |= COM_MOTOR_OVERRIDE_SPEED_0;
        if (mask & COM_MOTOR_OVERRIDE_SPEED_1 && data_input.ParseInto(state->MotorOut[1]))
            ack_data |= COM_MOTOR_OVERRIDE_SPEED_1;
        if (mask & COM_MOTOR_OVERRIDE_SPEED_2 && data_input.ParseInto(state->MotorOut[2]))
            ack_data |= COM_MOTOR_OVERRIDE_SPEED_2;
        if (mask & COM_MOTOR_OVERRIDE_SPEED_3 && data_input.ParseInto(state->MotorOut[3]))
            ack_data |= COM_MOTOR_OVERRIDE_SPEED_3;
        if (mask & COM_MOTOR_OVERRIDE_SPEED_4 && data_input.ParseInto(state->MotorOut[4]))
            ack_data |= COM_MOTOR_OVERRIDE_SPEED_4;
        if (mask & COM_MOTOR_OVERRIDE_SPEED_5 && data_input.ParseInto(state->MotorOut[5]))
            ack_data |= COM_MOTOR_OVERRIDE_SPEED_5;
        if (mask & COM_MOTOR_OVERRIDE_SPEED_6 && data_input.ParseInto(state->MotorOut[6]))
            ack_data |= COM_MOTOR_OVERRIDE_SPEED_6;
        if (mask & COM_MOTOR_OVERRIDE_SPEED_7 && data_input.ParseInto(state->MotorOut[7]))
            ack_data |= COM_MOTOR_OVERRIDE_SPEED_7;
    }
    if (mask & COM_SET_COMMAND_OVERRIDE) {
        uint8_t flag;
        if (data_input.ParseInto(flag)) {
            if (flag == 1)
                state->set(STATUS_OVERRIDE);
            else
                state->clear(STATUS_OVERRIDE);
            ack_data |= COM_SET_COMMAND_OVERRIDE;
        }
    }
    if (mask & COM_SET_STATE_MASK) {
        uint32_t new_state_mask;
        if (data_input.ParseInto(new_state_mask)) {
            SetStateMsg(new_state_mask);
            ack_data |= COM_SET_STATE_MASK;
        }
    }
    if (mask & COM_SET_STATE_DELAY) {
        uint16_t new_state_delay;
        if (data_input.ParseInto(new_state_delay)) {
            send_state_delay = new_state_delay;
            ack_data |= COM_SET_STATE_DELAY;
        }
    }
    if (mask & COM_REQ_HISTORY) {
        String eeprom_data;
        for (size_t i = EEPROM_LOG_START; i < EEPROM_LOG_END; ++i)
            eeprom_data += char(EEPROM[i]);
        SendDebugString(eeprom_data, MessageType::HistoryData);
        ack_data |= COM_REQ_HISTORY;
    }
    if (mask & COM_SET_LED) {
        uint8_t mode, r1, g1, b1, r2, g2, b2, ind_r, ind_g;
        if (data_input.ParseInto(mode, r1, g1, b1, r2, g2, b2, ind_r, ind_g)) {
            led->set(LED::Pattern(mode), r1, g1, b1, r2, g2, b2, ind_r, ind_g);
            ack_data |= COM_SET_LED;
        }
    }

    if (mask & COM_REQ_RESPONSE) {
        SendResponse(mask, ack_data);
    }
}

void SerialComm::SendConfiguration() const {
    CobsPayloadGeneric payload;
    WriteProtocolHead(SerialComm::MessageType::Command, COM_SET_EEPROM_DATA, payload);
    payload.Append(config->raw);
    WriteToOutput(payload);
}

void SerialComm::SendDebugString(const String& string, MessageType type) const {
    CobsPayload<2000> payload;
    WriteProtocolHead(type, 0xFFFFFFFF, payload);
    size_t str_len = string.length();
    for (size_t i = 0; i < str_len; ++i)
        payload.Append(string.charAt(i));
    WriteToOutput(payload);
}

uint16_t SerialComm::PacketSize(uint32_t mask) const {
    uint16_t sum = 0;
    if (mask & SerialComm::STATE_MICROS)
        sum += 4;
    if (mask & SerialComm::STATE_STATUS)
        sum += 2;
    if (mask & SerialComm::STATE_V0)
        sum += 2;
    if (mask & SerialComm::STATE_I0)
        sum += 2;
    if (mask & SerialComm::STATE_I1)
        sum += 2;
    if (mask & SerialComm::STATE_ACCEL)
        sum += 3 * 4;
    if (mask & SerialComm::STATE_GYRO)
        sum += 3 * 4;
    if (mask & SerialComm::STATE_MAG)
        sum += 3 * 4;
    if (mask & SerialComm::STATE_TEMPERATURE)
        sum += 2;
    if (mask & SerialComm::STATE_PRESSURE)
        sum += 4;
    if (mask & SerialComm::STATE_RX_PPM)
        sum += 9 * 2;  // six channels and three midpoints
    if (mask & SerialComm::STATE_AUX_CHAN_MASK)
        sum += 1;
    if (mask & SerialComm::STATE_COMMANDS)
        sum += 4 * 2;
    if (mask & SerialComm::STATE_F_AND_T)
        sum += 4 * 4;
    if (mask & SerialComm::STATE_T_TRIM)
        sum += 3 * 4;
    if (mask & SerialComm::STATE_PID_FZ_MASTER)
        sum += 6 * 4;
    if (mask & SerialComm::STATE_PID_TX_MASTER)
        sum += 6 * 4;
    if (mask & SerialComm::STATE_PID_TY_MASTER)
        sum += 6 * 4;
    if (mask & SerialComm::STATE_PID_TZ_MASTER)
        sum += 6 * 4;
    if (mask & SerialComm::STATE_PID_FZ_SLAVE)
        sum += 6 * 4;
    if (mask & SerialComm::STATE_PID_TX_SLAVE)
        sum += 6 * 4;
    if (mask & SerialComm::STATE_PID_TY_SLAVE)
        sum += 6 * 4;
    if (mask & SerialComm::STATE_PID_TZ_SLAVE)
        sum += 6 * 4;
    if (mask & SerialComm::STATE_MOTOR_OUT)
        sum += 8 * 2;
    if (mask & SerialComm::STATE_KINE_ANGLE)
        sum += 3 * 4;
    if (mask & SerialComm::STATE_KINE_RATE)
        sum += 3 * 4;
    if (mask & SerialComm::STATE_KINE_ALTITUDE)
        sum += 4;
    if (mask & SerialComm::STATE_LOOP_COUNT)
        sum += 4;
    return sum;
}

void SerialComm::SendState(uint32_t timestamp_us, void (*extra_handler)(uint8_t*, size_t), uint32_t mask) const {
    if (!mask)
        mask = state_mask;
    // No need to publish empty state messages
    if (!mask)
        return;

    CobsPayloadGeneric payload;

    WriteProtocolHead(SerialComm::MessageType::State, mask, payload);

    if (mask & SerialComm::STATE_MICROS)
        payload.Append(timestamp_us);
    if (mask & SerialComm::STATE_STATUS)
        payload.Append(state->status);
    if (mask & SerialComm::STATE_V0)
        payload.Append(state->V0_raw);
    if (mask & SerialComm::STATE_I0)
        payload.Append(state->I0_raw);
    if (mask & SerialComm::STATE_I1)
        payload.Append(state->I1_raw);
    if (mask & SerialComm::STATE_ACCEL)
        payload.Append(state->accel);
    if (mask & SerialComm::STATE_GYRO)
        payload.Append(state->gyro);
    if (mask & SerialComm::STATE_MAG)
        payload.Append(state->mag);
    if (mask & SerialComm::STATE_TEMPERATURE)
        payload.Append(state->temperature);
    if (mask & SerialComm::STATE_PRESSURE)
        payload.Append(state->pressure);
    if (mask & SerialComm::STATE_RX_PPM) {
        for (int i = 0; i < 6; ++i)
            payload.Append(ppm[i]);
        payload.Append(state->PPMchannel_pitch_mid);
        payload.Append(state->PPMchannel_roll_mid);
        payload.Append(state->PPMchannel_yaw_mid);
    }
    if (mask & SerialComm::STATE_AUX_CHAN_MASK)
        payload.Append(state->AUX_chan_mask);
    if (mask & SerialComm::STATE_COMMANDS)
        payload.Append(state->command_throttle, state->command_pitch, state->command_roll, state->command_yaw);
    if (mask & SerialComm::STATE_F_AND_T)
        payload.Append(state->Fz, state->Tx, state->Ty, state->Tz);
    if (mask & SerialComm::STATE_T_TRIM)
        payload.Append(state->Tx_trim, state->Ty_trim, state->Tz_trim);
    if (mask & SerialComm::STATE_PID_FZ_MASTER)
        WritePIDData(payload, control->thrust_pid.master());
    if (mask & SerialComm::STATE_PID_TX_MASTER)
        WritePIDData(payload, control->pitch_pid.master());
    if (mask & SerialComm::STATE_PID_TY_MASTER)
        WritePIDData(payload, control->roll_pid.master());
    if (mask & SerialComm::STATE_PID_TZ_MASTER)
        WritePIDData(payload, control->yaw_pid.master());
    if (mask & SerialComm::STATE_PID_FZ_SLAVE)
        WritePIDData(payload, control->thrust_pid.slave());
    if (mask & SerialComm::STATE_PID_TX_SLAVE)
        WritePIDData(payload, control->pitch_pid.slave());
    if (mask & SerialComm::STATE_PID_TY_SLAVE)
        WritePIDData(payload, control->roll_pid.slave());
    if (mask & SerialComm::STATE_PID_TZ_SLAVE)
        WritePIDData(payload, control->yaw_pid.slave());
    if (mask & SerialComm::STATE_MOTOR_OUT)
        payload.Append(state->MotorOut);
    if (mask & SerialComm::STATE_KINE_ANGLE)
        payload.Append(state->kinematicsAngle);
    if (mask & SerialComm::STATE_KINE_RATE)
        payload.Append(state->kinematicsRate);
    if (mask & SerialComm::STATE_KINE_ALTITUDE)
        payload.Append(state->kinematicsAltitude);
    if (mask & SerialComm::STATE_LOOP_COUNT)
        payload.Append(state->loopCount);
    WriteToOutput(payload, extra_handler);
}

void SerialComm::SendResponse(uint32_t mask, uint32_t response) const {
    CobsPayload<12> payload;
    WriteProtocolHead(MessageType::Response, mask, payload);
    payload.Append(response);
    WriteToOutput(payload);
}

uint16_t SerialComm::GetSendStateDelay() const {
    return send_state_delay;
}

void SerialComm::SetStateMsg(uint32_t values) {
    state_mask = values;
}

void SerialComm::AddToStateMsg(uint32_t values) {
    state_mask |= values;
}

void SerialComm::RemoveFromStateMsg(uint32_t values) {
    state_mask &= ~values;
}
